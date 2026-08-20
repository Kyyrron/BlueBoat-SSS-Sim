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

from ..core.geometry import bilinear_sample
from ..core.types import GridSpec, PlacedObject
from .materials import MaterialLibrary
from .objects import ObjectFieldConfig, place_objects, rasterize_objects
from .terrain import TerrainConfig, TerrainRasters, synthesize_terrain

MANIFEST_VERSION = 1


@dataclass
class WorldConfig:
    """Top-level world generation configuration (mirrors default_world.yaml)."""

    seed: int = 42
    size: tuple[float, float] = (80.0, 60.0)          # [m] (x, y)
    origin: tuple[float, float] = (-40.0, -30.0)      # world coords of min corner
    resolution: float = 0.10                          # raster cell [m]
    terrain: TerrainConfig = dataclasses.field(default_factory=TerrainConfig)
    objects: ObjectFieldConfig = dataclasses.field(default_factory=ObjectFieldConfig)
    material_overrides: dict[str, Any] = dataclasses.field(default_factory=dict)

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "WorldConfig":
        w = d.get("world", d)
        return cls(
            seed=int(w.get("seed", 42)),
            size=tuple(w.get("size", (80.0, 60.0))),
            origin=tuple(w.get("origin", (-40.0, -30.0))),
            resolution=float(w.get("resolution", 0.10)),
            terrain=TerrainConfig.from_dict(d.get("terrain", {})),
            objects=ObjectFieldConfig.from_dict(d.get("objects", {})),
            material_overrides=dict(d.get("materials", {})),
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
    seed: int = 0
    config_snapshot: dict[str, Any] = dataclasses.field(default_factory=dict)

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
        }
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
            seed=int(doc.get("seed", 0)),
            config_snapshot=dict(doc.get("config", {})),
        )


def generate_scene(cfg: WorldConfig,
                   raw_config: Mapping[str, Any] | None = None) -> SceneModel:
    """World config -> fully populated SceneModel (deterministic per seed)."""
    rng = np.random.default_rng(cfg.seed)
    grid = cfg.grid()
    materials = MaterialLibrary(cfg.material_overrides or None)

    terrain: TerrainRasters = synthesize_terrain(grid, cfg.terrain, materials, rng)
    objects = place_objects(grid, cfg.objects, materials, rng)
    rasterize_objects(objects, grid, terrain.height, terrain.reflectivity)

    return SceneModel(
        grid=grid,
        height=terrain.height,
        reflectivity=terrain.reflectivity,
        material_id=terrain.material_id,
        material_names=terrain.material_names,
        objects=objects,
        seed=cfg.seed,
        config_snapshot=dict(raw_config or {}),
    )
