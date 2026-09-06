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
from .objects import ObjectFieldConfig, place_objects, rasterize_objects
from .terrain import TerrainConfig, TerrainRasters, synthesize_terrain

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

    # ---- queries used by the sonar renderer -------------------------------
    def sample_height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Seabed z at world (x, y); outside the map, deep flat fallback."""
        return bilinear_sample(self.height, self.grid, x, y,
                               fill=float(self.height.min()))

    def sample_reflectivity(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return bilinear_sample(self.reflectivity, self.grid, x, y, fill=0.2)

    # ---- persistence -------------------------------------------------------
    def save(self, directory: str | Path) -> tuple[Path, Path]:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        npz = directory / "scene.npz"
        manifest = directory / "scene_manifest.yaml"
        np.savez_compressed(
            npz, height=self.height,
            reflectivity=self.reflectivity, material_id=self.material_id)
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
        return cls(
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
        )


def generate_scene(cfg: WorldConfig,
                   raw_config: Mapping[str, Any] | None = None) -> SceneModel:
    """World config -> fully populated SceneModel (deterministic per seed).

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
    rasterize_objects(objects, grid, terrain.height, terrain.reflectivity)

    return SceneModel(
        grid=grid,
        height=terrain.height,
        reflectivity=terrain.reflectivity,
        material_id=terrain.material_id,
        material_names=terrain.material_names,
        objects=objects,
        walls=list(cfg.walls),
        seed=cfg.seed,
        config_snapshot=dict(raw_config or {}),
        geo_anchor=cfg.geo,
    )
