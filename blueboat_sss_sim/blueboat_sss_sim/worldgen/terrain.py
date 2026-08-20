"""Procedural bathymetry and seabed material layout.

Builds three co-registered rasters from a YAML-driven config:
  * ``height``       -- seabed elevation z(x, y) [m], negative below surface
  * ``material_id``  -- per-cell seabed material index
  * ``reflectivity`` -- per-cell mean backscatter strength in [0, 1]

The composition pipeline is intentionally additive and order-independent
(base + slope + dunes + fBm roughness + per-material micro-relief), so
adding new terrain features later does not disturb existing ones.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from ..core.types import GridSpec
from .materials import MaterialLibrary
from .noise import fbm, value_noise


@dataclass
class TerrainConfig:
    base_depth: float = 4.0                       # [m] mean water depth
    slope_direction_deg: float = 0.0              # up-slope compass-ish direction (world frame deg)
    slope_grade: float = 0.0                      # [m/m]
    dunes_enabled: bool = True
    dunes_wavelength: float = 6.0                 # [m]
    dunes_amplitude: float = 0.12                 # [m]
    dunes_direction_deg: float = 30.0             # crest-normal direction
    dunes_irregularity: float = 0.5               # 0 = pure sine, 1 = strongly modulated
    roughness_amplitude: float = 0.05             # [m] broadband fBm relief
    roughness_octaves: int = 5
    roughness_cells: int = 12                     # base spatial frequency
    material_layout: str = "patches"              # "uniform" | "patches"
    material_uniform: str = "sand"
    material_patch_cells: int = 6                 # patch spatial frequency
    material_composition: dict[str, float] = field(
        default_factory=lambda: {"sand": 0.5, "mud": 0.2, "gravel": 0.15,
                                 "rocks": 0.1, "seagrass": 0.05})

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "TerrainConfig":
        slope = d.get("slope", {})
        dunes = d.get("dunes", {})
        rough = d.get("roughness", {})
        mats = d.get("materials", {})
        return cls(
            base_depth=float(d.get("base_depth", 4.0)),
            slope_direction_deg=float(slope.get("direction_deg", 0.0)),
            slope_grade=float(slope.get("grade", 0.0)),
            dunes_enabled=bool(dunes.get("enabled", True)),
            dunes_wavelength=float(dunes.get("wavelength", 6.0)),
            dunes_amplitude=float(dunes.get("amplitude", 0.12)),
            dunes_direction_deg=float(dunes.get("direction_deg", 30.0)),
            dunes_irregularity=float(dunes.get("irregularity", 0.5)),
            roughness_amplitude=float(rough.get("amplitude", 0.05)),
            roughness_octaves=int(rough.get("octaves", 5)),
            roughness_cells=int(rough.get("cells", 12)),
            material_layout=str(mats.get("layout", "patches")),
            material_uniform=str(mats.get("uniform", "sand")),
            material_patch_cells=int(mats.get("patch_cells", 6)),
            material_composition=dict(mats.get(
                "composition",
                {"sand": 0.5, "mud": 0.2, "gravel": 0.15, "rocks": 0.1, "seagrass": 0.05})),
        )


@dataclass
class TerrainRasters:
    height: np.ndarray            # float64[ny, nx]
    material_id: np.ndarray       # uint8[ny, nx]
    reflectivity: np.ndarray      # float64[ny, nx]
    material_names: list[str]     # index -> name


def _world_coords(grid: GridSpec) -> tuple[np.ndarray, np.ndarray]:
    xs = grid.origin_x + np.arange(grid.nx) * grid.resolution
    ys = grid.origin_y + np.arange(grid.ny) * grid.resolution
    return np.meshgrid(xs, ys)


def _material_map(grid: GridSpec, cfg: TerrainConfig,
                  rng: np.random.Generator) -> tuple[np.ndarray, list[str]]:
    names = [n for n, w in cfg.material_composition.items() if w > 0.0]
    if cfg.material_layout == "uniform" or len(names) <= 1:
        names = names or [cfg.material_uniform]
        return np.zeros((grid.ny, grid.nx), dtype=np.uint8), [names[0]]

    weights = np.array([cfg.material_composition[n] for n in names], dtype=float)
    weights /= weights.sum()
    # Low-frequency field thresholded by cumulative weights -> contiguous patches.
    f = fbm((grid.ny, grid.nx), cfg.material_patch_cells, 3, rng)
    # Rank-transform to uniform [0,1) so composition fractions are respected.
    order = f.reshape(-1).argsort().argsort().astype(np.float64)
    u = (order / order.size).reshape(f.shape)
    edges = np.cumsum(weights)
    mat_id = np.searchsorted(edges, u, side="right").astype(np.uint8)
    mat_id = np.minimum(mat_id, len(names) - 1)
    return mat_id, names


def synthesize_terrain(grid: GridSpec, cfg: TerrainConfig,
                       materials: MaterialLibrary,
                       rng: np.random.Generator) -> TerrainRasters:
    """Build the terrain rasters. See module docstring."""
    xx, yy = _world_coords(grid)
    shape = (grid.ny, grid.nx)

    height = np.full(shape, -cfg.base_depth, dtype=np.float64)

    # Constant slope.
    if abs(cfg.slope_grade) > 0:
        d = np.radians(cfg.slope_direction_deg)
        height += cfg.slope_grade * (np.cos(d) * xx + np.sin(d) * yy)

    # Sand dunes: directional sine, phase-modulated by low-freq noise.
    if cfg.dunes_enabled and cfg.dunes_amplitude > 0:
        d = np.radians(cfg.dunes_direction_deg)
        s = np.cos(d) * xx + np.sin(d) * yy
        phase = cfg.dunes_irregularity * 2.0 * np.pi * fbm(shape, 4, 2, rng)
        amp_mod = 1.0 + 0.5 * cfg.dunes_irregularity * value_noise(shape, 5, rng)
        height += cfg.dunes_amplitude * amp_mod * np.sin(
            2.0 * np.pi * s / max(cfg.dunes_wavelength, 0.5) + phase)

    # Broadband roughness.
    if cfg.roughness_amplitude > 0:
        height += cfg.roughness_amplitude * fbm(
            shape, cfg.roughness_cells, cfg.roughness_octaves, rng)

    # Materials + per-material micro-relief and reflectivity texture.
    mat_id, names = _material_map(grid, cfg, rng)
    reflectivity = np.zeros(shape, dtype=np.float64)
    for idx, name in enumerate(names):
        m = materials.get(name)
        mask = mat_id == idx
        if not mask.any():
            continue
        tex = fbm(shape, max(m.texture_cells, 8), 3, rng)
        reflectivity[mask] = np.clip(
            m.reflectivity * (1.0 + m.texture_amp * tex[mask]), 0.02, 1.0)
        if m.micro_roughness_m > 0:
            micro = fbm(shape, 200, 2, rng)
            height[mask] += m.micro_roughness_m * micro[mask]

    return TerrainRasters(height=height, material_id=mat_id,
                          reflectivity=reflectivity, material_names=names)
