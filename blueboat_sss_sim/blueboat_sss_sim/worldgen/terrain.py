"""Procedural bathymetry and seabed material layout.

Builds three co-registered rasters from a YAML-driven config:
  * ``height``       -- seabed elevation z(x, y) [m], negative below surface
  * ``material_id``  -- per-cell seabed material index
  * ``reflectivity`` -- per-cell mean backscatter strength in [0, 1]

The composition pipeline is intentionally additive and order-independent
(base + slope + dunes + fBm roughness + per-material micro-relief), so
adding new terrain features later does not disturb existing ones.

``TERRAIN_TEXTURE_SCHEMA`` is the builder-facing view of the texture
knobs (the same ``(key, label, kind, default, min, max)`` rows the depth
presets use, with dotted keys for the nested sections); ``flatten_texture``
/ ``unflatten_texture`` convert between that flat form and the nested
``terrain:`` YAML section. Qt-free: the GUI merely renders the rows.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from ..core.types import GridSpec
from .depth.paint import normalize_strokes, paint_material_ids
from .materials import REFLECTIVITY_MAX, MaterialLibrary
from .noise import fbm, value_noise

MATERIAL_LAYOUTS = ("uniform", "patches", "painted")

_TOP_KEYS = {"base_depth", "slope", "dunes", "roughness", "materials"}
_SUB_KEYS = {
    "slope": {"direction_deg", "grade"},
    "dunes": {"enabled", "wavelength", "amplitude", "direction_deg",
              "irregularity"},
    "roughness": {"amplitude", "octaves", "cells"},
    "materials": {"layout", "uniform", "patch_cells", "composition", "paint",
                  "map"},
}
_DEFAULT_COMPOSITION = {"sand": 0.5, "mud": 0.2, "gravel": 0.15,
                        "rocks": 0.1, "seagrass": 0.05}


def _section(d: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    sub = d.get(name) or {}
    bad = set(sub) - _SUB_KEYS[name]
    if bad:
        raise KeyError(f"Unknown terrain.{name} key(s): {sorted(bad)} "
                       f"(accepted: {sorted(_SUB_KEYS[name])})")
    return sub


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
    material_layout: str = "patches"              # "uniform" | "patches" | "painted"
    material_uniform: str = "sand"
    material_patch_cells: int = 6                 # patch spatial frequency
    material_composition: dict[str, float] = field(
        default_factory=lambda: dict(_DEFAULT_COMPOSITION))
    #: Material brush strokes (``worldgen.depth.paint``): stamped over the
    #: base layout in order; empty = no paint (bit-identical to before).
    material_paint: list = field(default_factory=list)
    #: Baked material overlay + reflectivity gain of a layered world
    #: (``material_map.npz``, see ``worldgen.material_map``); "" = none.
    material_map: str = ""

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "TerrainConfig":
        d = d or {}
        bad = set(d) - _TOP_KEYS
        if bad:
            raise KeyError(f"Unknown terrain config key(s): {sorted(bad)} "
                           f"(accepted: {sorted(_TOP_KEYS)})")
        slope = _section(d, "slope")
        dunes = _section(d, "dunes")
        rough = _section(d, "roughness")
        mats = _section(d, "materials")
        layout = str(mats.get("layout", "patches"))
        if layout not in MATERIAL_LAYOUTS:
            raise ValueError(f"terrain.materials.layout '{layout}' unknown "
                             f"{MATERIAL_LAYOUTS}")
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
            material_layout=layout,
            material_uniform=str(mats.get("uniform", "sand")),
            material_patch_cells=int(mats.get("patch_cells", 6)),
            material_composition=dict(mats.get("composition",
                                               _DEFAULT_COMPOSITION)),
            material_paint=normalize_strokes(mats.get("paint"), "material"),
            material_map=str(mats.get("map", "") or ""),
        )


# ------------------------------------------------- builder-facing schema
#: (key, label, kind, default, min, max, help) -- dotted keys address the
#: nested ``terrain:`` sections; ``help`` is the one-sentence hover text the
#: builder shows on the row. Defaults are the *builder's* (quiet seabed: no
#: dunes, light roughness, one material), not TerrainConfig's legacy ones.
_MATERIAL_HELP = {
    "sand": "Bright, even backscatter: the default harbour floor.",
    "mud": "Dark, smooth return; soft mud absorbs and swallows small targets.",
    "gravel": "Coarse, bright, speckled return that hides small objects in clutter.",
    "rocks": "Brightest and roughest floor, with strong shadows behind each rock.",
    "seagrass": "Patchy, mottled, dark-to-mid return with a soft edge.",
}
TERRAIN_TEXTURE_SCHEMA: tuple[tuple, ...] = (
    ("dunes.enabled", "Sand dunes", "bool", False, 0, 1,
     "Adds a field of sand waves; each crest reads as a bright stripe with a "
     "dark shadow behind it in the sonar image."),
    ("dunes.wavelength", "Dune wavelength [m]", "float", 6.0, 0.5, 50.0,
     "Crest-to-crest spacing of the dunes; short dunes (1-2 m) give fine "
     "banding, long ones (10 m+) gentle undulation."),
    ("dunes.amplitude", "Dune amplitude [m]", "float", 0.12, 0.0, 1.0,
     "Half the crest-to-trough height; taller dunes cast longer shadows and "
     "hide more of the seabed behind their crests."),
    ("dunes.direction_deg", "Dune crest-normal direction [deg]", "float",
     30.0, -180.0, 180.0,
     "Compass direction the dunes advance in (perpendicular to their crests); "
     "dunes crossed at right angles band the image, dunes along the track do not."),
    ("dunes.irregularity", "Dune irregularity (0 sine .. 1)", "float",
     0.5, 0.0, 1.0,
     "0 makes perfectly regular sine dunes, 1 makes them wander and break "
     "up like real sand waves."),
    ("roughness.amplitude", "Roughness amplitude [m]", "float",
     0.03, 0.0, 0.5,
     "Height of the small-scale bumps on the seabed; more roughness means "
     "a grainier, higher-contrast sonar texture with more clutter."),
    ("roughness.octaves", "Roughness octaves", "int", 5, 1, 8,
     "How many scales of detail the roughness contains; more octaves add "
     "finer grain on top of the larger bumps."),
    ("roughness.cells", "Roughness base cells", "int", 12, 2, 64,
     "Number of bumps across the area at the coarsest scale; fewer cells make "
     "broad undulations, more make small, dense bumps."),
    ("materials.layout", "Material layout",
     "choice:" + "|".join(MATERIAL_LAYOUTS), "uniform", None, None,
     "How seabed materials are laid out: one material everywhere, random "
     "patches weighted below, or only what you paint with the material brush."),
    ("materials.uniform", "Base material",
     "choice:" + "|".join(MaterialLibrary().seabed_names), "sand", None, None,
     "The material of the whole floor (uniform layout) or the background the "
     "patches and brush strokes sit on."),
    ("materials.patch_cells", "Patch cells (patches layout)", "int",
     6, 1, 64,
     "Number of patches across the area in the patches layout; fewer cells "
     "make large regions, more make a fine mosaic."),
) + tuple(
    (f"materials.composition.{name}", f"Patches: {name} weight", "float",
     _DEFAULT_COMPOSITION[name], 0.0, 1.0,
     f"Share of the area covered by {name} patches (patches layout only). "
     + _MATERIAL_HELP[name])
    for name in MaterialLibrary().seabed_names
)


def unflatten_texture(flat: Mapping[str, Any]) -> dict[str, Any]:
    """Flat dotted keys -> the nested ``terrain:`` texture dict.

    ``materials.composition`` is emitted only for the ``patches`` layout
    (it is the only consumer of the weights), so a uniform or painted
    seabed carries ``composition: {}`` exactly as the builder always
    wrote it.
    """
    out: dict[str, Any] = {}
    for key, value in flat.items():
        node = out
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    mats = out.setdefault("materials", {})
    if mats.get("layout", "uniform") != "patches":
        mats["composition"] = {}
    return out


def flatten_texture(nested: Mapping[str, Any] | None) -> dict[str, Any]:
    """Nested texture dict -> flat dotted keys, schema defaults filling
    every absent key (so a legacy ``{dunes: {enabled}, roughness:
    {amplitude}}`` state loads into the full form)."""
    nested = nested or {}
    out: dict[str, Any] = {}
    for key, _label, _kind, default, _lo, _hi, *_rest in TERRAIN_TEXTURE_SCHEMA:
        node: Any = nested
        for part in key.split("."):
            node = node.get(part) if isinstance(node, Mapping) else None
            if node is None:
                break
        out[key] = default if node is None else node
    return out


def texture_defaults() -> dict[str, Any]:
    """The builder's default texture: the schema defaults, nested."""
    return unflatten_texture({row[0]: row[3] for row in TERRAIN_TEXTURE_SCHEMA})


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


def apply_material_overlay(mat_id: np.ndarray, names: list[str],
                           ids: np.ndarray, onames: list[str],
                           known=None) -> tuple[np.ndarray, list[str]]:
    """Stamp a baked overlay (``-1`` = untouched) over the base layout,
    appending unseen material names exactly as material paint does."""
    names = list(names)
    out = mat_id.copy()
    for k, nm in enumerate(onames):
        sel = ids == k
        if not sel.any():
            continue
        if known is not None:
            known(nm)                      # unknown material raises by name
        if nm not in names:
            if len(names) >= 255:
                raise ValueError("too many materials in one scene")
            names.append(nm)
        out[sel] = names.index(nm)
    return out.astype(np.uint8), names


def _material_map(grid: GridSpec, cfg: TerrainConfig,
                  rng: np.random.Generator,
                  materials: MaterialLibrary | None = None,
                  overlay: tuple[np.ndarray, list[str]] | None = None
                  ) -> tuple[np.ndarray, list[str]]:
    names = [n for n, w in cfg.material_composition.items() if w > 0.0]
    if cfg.material_layout == "painted":
        # Uniform base material; the strokes below carry the rest.
        mat_id, names = (np.zeros((grid.ny, grid.nx), dtype=np.uint8),
                         [cfg.material_uniform])
    elif cfg.material_layout == "uniform" or len(names) <= 1:
        names = names or [cfg.material_uniform]
        mat_id, names = np.zeros((grid.ny, grid.nx), dtype=np.uint8), [names[0]]
    else:
        weights = np.array([cfg.material_composition[n] for n in names],
                           dtype=float)
        weights /= weights.sum()
        # Low-frequency field thresholded by cumulative weights -> contiguous patches.
        f = fbm((grid.ny, grid.nx), cfg.material_patch_cells, 3, rng)
        # Rank-transform to uniform [0,1) so composition fractions are respected.
        order = f.reshape(-1).argsort().argsort().astype(np.float64)
        u = (order / order.size).reshape(f.shape)
        edges = np.cumsum(weights)
        mat_id = np.searchsorted(edges, u, side="right").astype(np.uint8)
        mat_id = np.minimum(mat_id, len(names) - 1)
    if cfg.material_paint:
        mat_id, names = paint_material_ids(
            mat_id, names, grid, cfg.material_paint,
            known=materials.get if materials is not None else None)
    if overlay is not None:
        mat_id, names = apply_material_overlay(
            mat_id, names, overlay[0], list(overlay[1]),
            known=materials.get if materials is not None else None)
    return mat_id, names


def synthesize_terrain(grid: GridSpec, cfg: TerrainConfig,
                       materials: MaterialLibrary,
                       rng: np.random.Generator,
                       base_height: np.ndarray | None = None,
                       material_overlay: tuple[np.ndarray, list[str]] | None = None,
                       reflectivity_gain: np.ndarray | None = None
                       ) -> TerrainRasters:
    """Build the terrain rasters. See module docstring.

    When ``base_height`` is given (authored bathymetry: a depth preset or
    an ingested depth file), it replaces the constant ``base_depth`` plane
    and the global slope, and this function contributes only the texture
    layers on top (dunes, broadband roughness, per-material micro-relief).

    ``material_overlay`` (``(ids, names)``, -1 = untouched) is stamped
    over the material layout after the material paint, and
    ``reflectivity_gain`` multiplies the reflectivity before its clip;
    both default to the baked ``materials.map`` file when the config
    names one, and to nothing otherwise (not one extra operation, and the
    RNG draw order is unchanged).
    """
    xx, yy = _world_coords(grid)
    shape = (grid.ny, grid.nx)
    if material_overlay is None and reflectivity_gain is None and cfg.material_map:
        from .material_map import load_material_map
        ids, onames, gain = load_material_map(cfg.material_map, grid)
        material_overlay, reflectivity_gain = (ids, onames), gain

    if base_height is not None:
        if base_height.shape != shape:
            raise ValueError(f"base_height shape {base_height.shape} "
                             f"does not match the grid {shape}")
        height = base_height.astype(np.float64).copy()
    else:
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
    mat_id, names = _material_map(grid, cfg, rng, materials,
                                  overlay=material_overlay)
    reflectivity = np.zeros(shape, dtype=np.float64)
    for idx, name in enumerate(names):
        m = materials.get(name)
        mask = mat_id == idx
        if not mask.any():
            continue
        tex = fbm(shape, max(m.texture_cells, 8), 3, rng)
        val = m.reflectivity * (1.0 + m.texture_amp * tex[mask])
        # The ceiling opens only under a gain map (a measured survey can
        # put rock several dB above sand); a world without one clips at
        # 1.0 exactly as before, so it renders bit-identically.
        hi = 1.0
        if reflectivity_gain is not None:
            val = val * np.asarray(reflectivity_gain, dtype=np.float64)[mask]
            hi = REFLECTIVITY_MAX
        reflectivity[mask] = np.clip(val, 0.02, hi)
        if m.micro_roughness_m > 0:
            micro = fbm(shape, 200, 2, rng)
            height[mask] += m.micro_roughness_m * micro[mask]

    return TerrainRasters(height=height, material_id=mat_id,
                          reflectivity=reflectivity, material_names=names)
