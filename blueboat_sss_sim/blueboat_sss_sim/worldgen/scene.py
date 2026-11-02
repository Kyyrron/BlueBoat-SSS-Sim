"""SceneModel -- the single source of truth for a generated world.

One :class:`SceneModel` instance is produced per generated world and then
consumed by *three* independent clients:

  * :mod:`.sdf_writer`               -> Gazebo world (visual + physics),
  * :mod:`blueboat_sss_sim.sonar`        -> acoustic rendering,
  * :mod:`blueboat_sss_sim.dataset`      -> ground-truth labeling.

Because all three read the same object, the Gazebo world, the sonar image
and the training labels can never disagree.

Persistence: ``scene.npz`` (rasters) + ``scene_manifest.yaml``
(config, seed, object ground truth) side by side.
"""

from __future__ import annotations

import dataclasses
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from ..core.geo import GeoAnchor
from ..core.geometry import bilinear_sample
from ..core.types import GridSpec, PlacedObject, Wall
from .depth import DepthConfig, build_height
from .materials import MaterialLibrary
from .objects import (ObjectFieldConfig, facet_bump_raster,
                      needs_facet_raster, needs_roughness_raster,
                      place_objects, rasterize_objects, roughness_raster,
                      transmission_raster)
from .terrain import TerrainConfig, TerrainRasters, synthesize_terrain

_log = logging.getLogger(__name__)

MANIFEST_VERSION = 1

#: Stated in the manifest's ``geo:`` block so a consumer can verify it uses
#: the same conversion (the project-wide equirectangular convention).
GEO_CONVENTION = "equirectangular_wgs84_r6378137"

_WALL_KEYS = {f.name for f in dataclasses.fields(Wall)}
#: Wall kinds the renderer models (core.types.Wall docstring).
WALL_KINDS = ("quay", "pontoon")


def _walls_from_list(items: Any) -> list[Wall]:
    """Parse the world config's ``walls:`` list.

    Unknown keys raise here rather than being ignored: a misspelt
    ``reflectivty`` would otherwise silently leave the wall at its default and
    the ghost at the wrong strength, which is exactly the kind of quiet wrong
    answer the sonar config's own key check exists to prevent.
    """
    walls: list[Wall] = []
    for i, item in enumerate(items):
        bad = set(item) - _WALL_KEYS
        if bad:
            raise KeyError(f"Unknown wall keys in walls[{i}]: {sorted(bad)}")
        w = Wall(**{k: v for k, v in item.items()})
        w.name = str(w.name)
        if w.length < 1e-6:
            raise ValueError(f"wall '{w.name}' has zero length")
        if not 0.0 <= w.reflectivity <= 1.0:
            raise ValueError(f"wall '{w.name}' reflectivity {w.reflectivity} "
                             "is outside 0..1")
        if w.kind not in WALL_KINDS:
            raise ValueError(f"wall '{w.name}' kind {w.kind!r} is not one of "
                             f"{sorted(WALL_KINDS)}")
        if w.kind == "pontoon" and not w.draft_m > 0.0:
            raise ValueError(f"pontoon '{w.name}' needs a positive draft_m")
        if w.roughness_m < 0.0:
            raise ValueError(f"wall '{w.name}' roughness_m must be >= 0")
        walls.append(w)
    names = [w.name for w in walls]
    if len(set(names)) != len(names):
        raise ValueError(f"wall names must be unique, got {names}")
    return walls


@dataclass
class WorldConfig:
    """Top-level world generation configuration (mirrors default_world.yaml)."""

    seed: int = 42
    size: tuple[float, float] = (80.0, 60.0)          # [m] (x, y)
    origin: tuple[float, float] = (-40.0, -30.0)      # world coords of min corner
    resolution: float = 0.10                          # raster cell [m]
    terrain: TerrainConfig = dataclasses.field(default_factory=TerrainConfig)
    objects: ObjectFieldConfig = dataclasses.field(default_factory=ObjectFieldConfig)
    walls: list[Wall] = dataclasses.field(default_factory=list)
    material_overrides: dict[str, Any] = dataclasses.field(default_factory=dict)
    geo: GeoAnchor | None = None                      # GPS of world (0, 0)
    depth: DepthConfig | None = None                  # authored bathymetry
    limits_collision: bool = False                    # invisible border boxes

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "WorldConfig":
        w = d.get("world", d)
        return cls(
            seed=int(w.get("seed", 42)),
            limits_collision=bool(w.get("limits_collision", False)),
            size=tuple(w.get("size", (80.0, 60.0))),
            origin=tuple(w.get("origin", (-40.0, -30.0))),
            resolution=float(w.get("resolution", 0.10)),
            terrain=TerrainConfig.from_dict(d.get("terrain", {})),
            objects=ObjectFieldConfig.from_dict(d.get("objects", {})),
            walls=_walls_from_list(d.get("walls", []) or []),
            material_overrides=dict(d.get("materials", {})),
            geo=GeoAnchor.from_dict(d.get("geo")),
            depth=DepthConfig.from_dict(d.get("depth")),
        )

    @classmethod
    def from_yaml(cls, path: str | Path) -> "WorldConfig":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(yaml.safe_load(f) or {})

    def grid(self) -> GridSpec:
        nx = int(round(self.size[0] / self.resolution)) + 1
        ny = int(round(self.size[1] / self.resolution)) + 1
        return GridSpec(self.origin[0], self.origin[1], self.resolution, nx, ny)


@dataclass
class SceneModel:
    """Generated scene: co-registered rasters + object ground truth."""

    grid: GridSpec
    height: np.ndarray                  # float64[ny, nx], z of seabed (<0)
    reflectivity: np.ndarray            # float64[ny, nx], 0..1
    material_id: np.ndarray             # uint8[ny, nx]
    material_names: list[str]
    objects: list[PlacedObject]
    walls: list[Wall] = dataclasses.field(default_factory=list)
    seed: int = 0
    config_snapshot: dict[str, Any] = dataclasses.field(default_factory=dict)
    geo_anchor: GeoAnchor | None = None               # GPS of world (0, 0)
    #: Two-way transmission of whatever stands on a cell (0 everywhere but
    #: under a thin-shelled object). ``None`` when nothing in the world
    #: transmits, which is every world built before 2026-09-18 -- the
    #: array is then absent from ``scene.npz`` and the file is byte-identical.
    transmission: np.ndarray | None = None
    #: RMS surface height [m] of whatever stands on a cell, where that is
    #: smooth enough at 450 kHz for the coherent/incoherent split to matter
    #: (0 = "nothing smooth here", take the rough default). ``None`` when no
    #: object in the world is smooth on that scale, which is every world
    #: built before 2026-09-19 -- the array is then absent from
    #: ``scene.npz`` and the file is byte-identical.
    roughness: np.ndarray | None = None
    #: The part of ``height`` that belongs to **facet objects** (NC #17):
    #: smooth solid objects the renderer scatters analytically from their
    #: geometry (``sonar/facets.py``). There, ``height - facet_bump`` is the
    #: seabed the beam meets, and the object only occludes it -- by its own
    #: two-way transmission, 0 for an opaque one. Additive (overlapping
    #: objects sum, like the height). ``None`` when the scene has no facet
    #: object -- every scene built before 2026-09-28 -- and the array is then
    #: absent from ``scene.npz``.
    facet_bump: np.ndarray | None = None
    #: What the sonar sees beyond the grid. ``"fill"`` (the historical
    #: default, frozen by the render fixture): a flat phantom plain at the
    #: deepest cell of the map with reflectivity 0.2. ``"edge"``: the
    #: nearest edge cell repeated outward, so the seabed simply continues.
    #: The World Builder builds ``"edge"`` worlds; additive in the manifest
    #: (absent = ``"fill"``), so every world on disk renders as before.
    outside: str = "fill"

    # ---- queries used by the sonar renderer -------------------------------
    def _clamp_to_map(self, x: np.ndarray, y: np.ndarray
                      ) -> tuple[np.ndarray, np.ndarray]:
        """World (x, y) pulled onto the grid's covered area (the edge cell
        is repeated outward); a hair inside the extent so floating-point
        round-off cannot push an edge query back out of bounds."""
        xmin, ymin, xmax, ymax = self.grid.extent
        eps = 1e-6 * self.grid.resolution
        return (np.clip(x, xmin + eps, xmax - eps),
                np.clip(y, ymin + eps, ymax - eps))

    def sample_height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Seabed z at world (x, y).

        Outside the grid this depends on ``outside``: ``"fill"`` returns the
        DEEPEST cell of the map (a flat phantom plain, so a boat within the
        sonar range of the world edge sees a step down and a horizon-cull
        shadow band there); ``"edge"`` repeats the nearest edge cell. The
        default stays ``"fill"`` on purpose: the render fixture is frozen on
        it (sonar_model.md A13).
        """
        if self.outside == "edge":
            x, y = self._clamp_to_map(x, y)
        return bilinear_sample(self.height, self.grid, x, y,
                               fill=float(self.height.min()))

    def sample_reflectivity(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Reflectivity at world (x, y); beyond the grid 0.2 (``"fill"``)
        or the nearest edge cell (``"edge"``), like ``sample_height``."""
        if self.outside == "edge":
            x, y = self._clamp_to_map(x, y)
        return bilinear_sample(self.reflectivity, self.grid, x, y, fill=0.2)

    def sample_transmission(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """How much of the beam passes through what stands here (0 = opaque).

        A scene with no transmitting object answers exactly zero, so the
        renderer's shadow is bit-identical to the pre-2026-09-18 one.
        """
        if self.transmission is None:
            return np.zeros(np.shape(x), dtype=np.float64)
        return bilinear_sample(self.transmission, self.grid, x, y, fill=0.0)

    def sample_roughness(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """RMS surface height [m] of what stands here; 0 = the rough default.

        Every seabed material is far past ``COHERENT_ROUGHNESS_LIMIT_M`` at
        450 kHz (the smoothest, mud, has 2 k sigma = 15), so the seabed needs
        no raster: only a smooth manufactured object can answer non-zero, and
        a scene without one answers exactly zero everywhere.
        """
        if self.roughness is None:
            return np.zeros(np.shape(x), dtype=np.float64)
        return bilinear_sample(self.roughness, self.grid, x, y, fill=0.0)

    def sample_facet_bump(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Height of facet-object occluders over the seabed at (x, y); 0
        where there is none (NC #17)."""
        if self.facet_bump is None:
            return np.zeros(np.shape(x), dtype=np.float64)
        return bilinear_sample(self.facet_bump, self.grid, x, y, fill=0.0)

    def stale_rasters(self) -> list[str]:
        """Object rasters the current model derives from this scene's
        objects but the scene does not carry.

        Empty for a scene built by current code. Non-empty for a world built
        before the model that needs the raster existed -- or by the World
        Builder between 2026-09-19 and 2026-09-28, whose build path dropped
        the roughness raster, so every smooth object in it rendered with
        sand's specular lobe (the field box: +20-25 dB where the model gives
        +5). Worlds are immutable (NC #10): the cure is to rebuild the world
        (World Builder, Edit), never to patch the folder.
        """
        missing: list[str] = []
        if self.roughness is None and needs_roughness_raster(self.objects):
            missing.append("roughness")
        if self.facet_bump is None and needs_facet_raster(self.objects):
            missing.append("facet_bump")
        return missing

    # ---- persistence -------------------------------------------------------
    def save(self, directory: str | Path) -> tuple[Path, Path]:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        npz = directory / "scene.npz"
        manifest = directory / "scene_manifest.yaml"
        arrays = {"height": self.height, "reflectivity": self.reflectivity,
                  "material_id": self.material_id}
        if self.transmission is not None and float(self.transmission.max()) > 0.0:
            arrays["transmission"] = self.transmission
        if self.roughness is not None and float(self.roughness.max()) > 0.0:
            arrays["roughness"] = self.roughness
        if self.facet_bump is not None and float(self.facet_bump.max()) > 0.0:
            arrays["facet_bump"] = self.facet_bump
        np.savez_compressed(npz, **arrays)
        doc = {
            "version": MANIFEST_VERSION,
            "seed": self.seed,
            "grid": dataclasses.asdict(self.grid),
            "material_names": self.material_names,
            "config": self.config_snapshot,
            "objects": [dataclasses.asdict(o) for o in self.objects],
            "walls": [dataclasses.asdict(w) for w in self.walls],
        }
        if self.geo_anchor is not None:
            # Additive, like "walls": the version does not move, and a
            # consumer that ignores the block sees the same local-metres
            # world as before.
            doc["geo"] = {"anchor": self.geo_anchor.to_dict(),
                          "convention": GEO_CONVENTION}
        if self.outside != "fill":
            # Additive as well: written only when it departs from the
            # historical phantom plain, so every other manifest is
            # byte-identical to before.
            doc["outside"] = self.outside
        with open(manifest, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, sort_keys=False)
        return npz, manifest

    @classmethod
    def load(cls, directory: str | Path) -> "SceneModel":
        directory = Path(directory)
        with open(directory / "scene_manifest.yaml", "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        if doc.get("version") != MANIFEST_VERSION:
            raise ValueError(f"Unsupported scene manifest version {doc.get('version')}")
        data = np.load(directory / "scene.npz")
        scene = cls(
            grid=GridSpec(**doc["grid"]),
            height=data["height"],
            reflectivity=data["reflectivity"],
            material_id=data["material_id"],
            material_names=list(doc["material_names"]),
            objects=[PlacedObject(**o) for o in doc["objects"]],
            # Optional: a bundle generated before walls existed carries no
            # "walls" key and loads as a wall-free basin. The manifest
            # version deliberately does NOT move for an additive key --
            # SceneModel.load rejects any other version outright, and every
            # bundle already on disk would stop loading.
            walls=[Wall(**w) for w in (doc.get("walls") or [])],
            seed=int(doc.get("seed", 0)),
            config_snapshot=dict(doc.get("config", {})),
            geo_anchor=GeoAnchor.from_dict(
                (doc.get("geo") or {}).get("anchor")),
            transmission=(data["transmission"]
                          if "transmission" in data.files else None),
            roughness=(data["roughness"]
                       if "roughness" in data.files else None),
            facet_bump=(data["facet_bump"]
                        if "facet_bump" in data.files else None),
            outside=str(doc.get("outside", "fill")),
        )
        if scene.outside not in ("fill", "edge"):
            raise ValueError(f"Unknown scene 'outside' mode "
                             f"'{scene.outside}' (fill | edge)")
        stale = scene.stale_rasters()
        if stale:
            # Every loader goes through here (the sonar node, the analysis
            # CLIs, export_scene_maps), so the notice cannot be bypassed.
            _log.warning(
                "world %s predates the current object model: it lacks the %s "
                "raster(s), so its objects render as they did before. Rebuild "
                "it (World Builder -> Edit World -> save) to render it with "
                "the current model; the folder itself is never patched "
                "(NC #10).", directory, ", ".join(stale))
        return scene


@dataclass
class ObjectRasters:
    """The per-object rasters one stamping pass produces. Each is ``None``
    when no object in the scene needs it, which keeps the array out of
    ``scene.npz`` and every scene without such an object byte-identical."""

    transmission: np.ndarray | None
    roughness: np.ndarray | None
    facet_bump: np.ndarray | None

    def scene_kwargs(self) -> dict[str, np.ndarray | None]:
        """The rasters as ``SceneModel`` keyword arguments."""
        return {f.name: getattr(self, f.name)
                for f in dataclasses.fields(self)}


def stamp_objects(objects: list[PlacedObject], grid: GridSpec,
                  height: np.ndarray,
                  reflectivity: np.ndarray) -> ObjectRasters:
    """Stamp placed objects into the terrain rasters *in place* and return
    the object rasters.

    The one place objects are stamped. Two builders used to do it apart --
    ``generate_scene`` and the World Builder's
    ``world_folder._generate_with_base`` -- and the second never built the
    roughness raster, so from 2026-09-19 to 2026-09-28 every world saved
    from the GUI rendered its smooth objects with sand's specular lobe (the
    field box: +20-25 dB near-edge where the model gives +5).
    """
    trans = transmission_raster(objects, grid)
    rough = roughness_raster(objects, grid)
    facet = facet_bump_raster(objects, grid)
    rasterize_objects(objects, grid, height, reflectivity, trans, rough,
                      facet)
    return ObjectRasters(transmission=trans, roughness=rough,
                         facet_bump=facet)


def generate_scene(cfg: WorldConfig,
                   raw_config: Mapping[str, Any] | None = None,
                   terrain_out: list | None = None) -> SceneModel:
    """World config -> fully populated SceneModel (deterministic per seed).

    ``terrain_out`` (a list, optional) receives a copy of the pre-object-stamp
    height raster -- what the Gazebo visuals stand on (``write_world_sdf``'s
    ``terrain_height``); no draw changes.

    For a world with authored bathymetry (a ``depth:`` section), terrain
    and object placement draw from *separate* RNG streams derived from the
    seed, so editing the bathymetry (a preset parameter, a different depth
    file) never moves the objects -- which is what lets two worlds
    differing only in depth be compared object-by-object. A legacy config
    without ``depth:`` keeps the single historical stream, bit-identical
    to what every existing bundle was generated with.
    """
    if cfg.depth is not None:
        rng_terrain = np.random.default_rng([cfg.seed, 0])
        rng_objects = np.random.default_rng([cfg.seed, 1])
    else:
        rng_terrain = rng_objects = np.random.default_rng(cfg.seed)
    grid = cfg.grid()
    materials = MaterialLibrary(cfg.material_overrides or None)

    base = None
    if cfg.depth is not None:
        base = build_height(grid, cfg.depth, cfg.walls, cfg.geo, rng_terrain)
    terrain: TerrainRasters = synthesize_terrain(
        grid, cfg.terrain, materials, rng_terrain, base_height=base)
    objects = place_objects(grid, cfg.objects, materials, rng_objects,
                            height=terrain.height)
    if terrain_out is not None:
        terrain_out.append(terrain.height.copy())
    rasters = stamp_objects(objects, grid, terrain.height,
                            terrain.reflectivity)

    return SceneModel(
        grid=grid,
        height=terrain.height,
        reflectivity=terrain.reflectivity,
        **rasters.scene_kwargs(),
        material_id=terrain.material_id,
        material_names=terrain.material_names,
        objects=objects,
        walls=list(cfg.walls),
        seed=cfg.seed,
        config_snapshot=dict(raw_config or {}),
        geo_anchor=cfg.geo,
    )
