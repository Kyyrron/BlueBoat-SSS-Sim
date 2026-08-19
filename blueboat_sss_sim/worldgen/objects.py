"""Underwater litter object library.

Each object type is described by an :class:`ObjectSpec` (size ranges, shape
primitive, material, burial behaviour). Placement draws instances from a
configurable composition, and every instance is:

  1. recorded as a :class:`~blueboat_sss_sim.core.types.PlacedObject`
     (ground truth for labeling),
  2. rasterised into the shared acoustic rasters (height bump +
     reflectivity patch) -- so highlights, shadows and burial emerge from
     the single physical rendering path,
  3. converted to a simple SDF geometry for visual/physical presence in
     Gazebo (see :mod:`.sdf_writer`).

The acoustic footprint model is deliberately simple (2D masks with a
vertical profile); replacing it with meshes only requires overriding
``ObjectSpec.rasterize``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

import numpy as np

from ..core.types import GridSpec, PlacedObject
from .materials import MaterialLibrary

# --------------------------------------------------------------------------
# Footprint mask primitives (local frame, object at origin, yaw applied)
# --------------------------------------------------------------------------
MaskFn = Callable[[np.ndarray, np.ndarray, PlacedObject], np.ndarray]
"""(x_local, y_local, obj) -> height-profile mask in [0, 1]."""


def _mask_box(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
    inside = (np.abs(xl) <= o.length / 2) & (np.abs(yl) <= o.width / 2)
    return inside.astype(np.float64)


def _mask_cylinder_lying(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
    """Cylinder lying on the seabed along local x: circular cross-section."""
    r = o.width / 2
    inside = (np.abs(xl) <= o.length / 2) & (np.abs(yl) <= r)
    prof = np.sqrt(np.clip(1.0 - (yl / max(r, 1e-6)) ** 2, 0.0, 1.0))
    return inside * prof


def _mask_annulus(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
    """Tire lying flat: annulus with rounded (elliptic) cross section."""
    r_out = o.length / 2
    r_in = 0.55 * r_out
    rr = np.hypot(xl, yl)
    mid = 0.5 * (r_out + r_in)
    half = 0.5 * (r_out - r_in)
    prof = np.sqrt(np.clip(1.0 - ((rr - mid) / max(half, 1e-6)) ** 2, 0.0, 1.0))
    return prof


def _mask_capsule(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
    """Bottle-like: cylinder with hemispherical ends, lying along x."""
    r = o.width / 2
    ax = np.clip(np.abs(xl) - (o.length / 2 - r), 0.0, None)
    d = np.hypot(ax, yl)
    prof = np.sqrt(np.clip(1.0 - (d / max(r, 1e-6)) ** 2, 0.0, 1.0))
    return prof


def _mask_line_segments(n_seg: int, gap_ratio: float) -> MaskFn:
    """Rope/chain: a slightly wavy dashed line of small bumps along x."""

    def fn(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
        rng = np.random.default_rng(o.object_id)  # deterministic wiggle per object
        r = o.width / 2
        # Sinusoidal lateral wiggle.
        amp = 1.5 * o.width
        wav = max(o.length / max(n_seg, 1), 0.05)
        y_c = amp * np.sin(2 * np.pi * xl / (4 * wav) + rng.uniform(0, 2 * np.pi))
        dy = np.abs(yl - y_c)
        inside_x = np.abs(xl) <= o.length / 2
        prof = np.sqrt(np.clip(1.0 - (dy / max(r, 1e-6)) ** 2, 0.0, 1.0))
        if gap_ratio > 0:  # dashed (chain links)
            phase = (xl / wav) % 1.0
            prof = prof * (phase > gap_ratio)
        return inside_x * prof

    return fn


def _mask_anchor(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
    """Cross/T shape: shank along x + flukes along y."""
    shank = (np.abs(yl) <= 0.12 * o.width) & (np.abs(xl) <= o.length / 2)
    flukes = (np.abs(xl - o.length * 0.3) <= 0.12 * o.length) & (np.abs(yl) <= o.width / 2)
    return (shank | flukes).astype(np.float64)


def _mask_blob(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
    """Irregular debris blob: perturbed ellipse."""
    rng = np.random.default_rng(o.object_id)
    th = np.arctan2(yl, xl)
    wobble = 1.0 + 0.35 * np.sin(3 * th + rng.uniform(0, 6.28)) \
                 + 0.2 * np.sin(7 * th + rng.uniform(0, 6.28))
    d = np.hypot(xl / (o.length / 2 * wobble), yl / (o.width / 2 * wobble))
    return np.clip(1.0 - d ** 2, 0.0, 1.0)


# --------------------------------------------------------------------------
# Object catalog
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class ObjectSpec:
    """Static description of one litter type."""

    type: str
    mask_fn: MaskFn
    length_range: tuple[float, float]     # [m]
    width_range: tuple[float, float]      # [m] (or same as length for round)
    height_range: tuple[float, float]     # proud height [m]
    material: str
    burial_range: tuple[float, float] = (0.0, 0.4)
    round_footprint: bool = False         # width := length
    sdf_shape: str = "box"                # box | cylinder | cylinder_upright


CATALOG: dict[str, ObjectSpec] = {
    "tire_car":        ObjectSpec("tire_car", _mask_annulus, (0.55, 0.75), (0.55, 0.75),
                                  (0.18, 0.25), "rubber", (0.0, 0.5), True, "cylinder_upright"),
    "tire_bicycle":    ObjectSpec("tire_bicycle", _mask_annulus, (0.55, 0.70), (0.55, 0.70),
                                  (0.03, 0.05), "rubber", (0.0, 0.6), True, "cylinder_upright"),
    "pipe_pvc":        ObjectSpec("pipe_pvc", _mask_cylinder_lying, (0.8, 3.0), (0.05, 0.20),
                                  (0.05, 0.20), "pvc", (0.0, 0.5), False, "cylinder"),
    "bottle_plastic":  ObjectSpec("bottle_plastic", _mask_capsule, (0.20, 0.35), (0.07, 0.12),
                                  (0.07, 0.12), "plastic", (0.0, 0.6), False, "cylinder"),
    "can":             ObjectSpec("can", _mask_cylinder_lying, (0.10, 0.17), (0.06, 0.10),
                                  (0.06, 0.10), "metal", (0.0, 0.6), False, "cylinder"),
    "tent_weight":     ObjectSpec("tent_weight", _mask_box, (0.15, 0.30), (0.10, 0.20),
                                  (0.08, 0.15), "concrete", (0.0, 0.4), False, "box"),
    "rope":            ObjectSpec("rope", _mask_line_segments(0, 0.0), (2.0, 8.0), (0.02, 0.05),
                                  (0.02, 0.05), "rope", (0.0, 0.6), False, "box"),
    "cylinder_metal":  ObjectSpec("cylinder_metal", _mask_cylinder_lying, (0.3, 1.0), (0.10, 0.30),
                                  (0.10, 0.30), "metal", (0.0, 0.4), False, "cylinder"),
    "concrete_block":  ObjectSpec("concrete_block", _mask_box, (0.3, 0.8), (0.2, 0.5),
                                  (0.15, 0.40), "concrete", (0.0, 0.3), False, "box"),
    "brick":           ObjectSpec("brick", _mask_box, (0.20, 0.25), (0.09, 0.12),
                                  (0.05, 0.08), "brickclay", (0.0, 0.5), False, "box"),
    "chain":           ObjectSpec("chain", _mask_line_segments(12, 0.35), (1.5, 6.0), (0.03, 0.08),
                                  (0.03, 0.08), "metal", (0.0, 0.6), False, "box"),
    "anchor":          ObjectSpec("anchor", _mask_anchor, (0.4, 1.0), (0.3, 0.8),
                                  (0.10, 0.30), "metal", (0.0, 0.4), False, "box"),
    "debris":          ObjectSpec("debris", _mask_blob, (0.2, 1.2), (0.15, 0.8),
                                  (0.05, 0.35), "generic", (0.0, 0.5), False, "box"),
}

DEFAULT_COMPOSITION: dict[str, float] = {
    "tire_car": 1.0, "tire_bicycle": 0.5, "pipe_pvc": 1.0, "bottle_plastic": 1.5,
    "can": 1.5, "tent_weight": 0.5, "rope": 0.7, "cylinder_metal": 0.8,
    "concrete_block": 0.8, "brick": 1.0, "chain": 0.6, "anchor": 0.4, "debris": 1.2,
}


# --------------------------------------------------------------------------
# Placement + rasterisation
# --------------------------------------------------------------------------
@dataclass
class ObjectFieldConfig:
    density_per_hectare: float = 60.0
    composition: dict[str, float] | None = None      # type -> weight; None = defaults
    margin_m: float = 3.0                            # keep-out from world border
    min_separation_m: float = 1.0
    size_scale: float = 1.0                          # global size multiplier
    burial_scale: float = 1.0                        # global burial multiplier
    reflectivity_jitter: float = 0.15                # +-fraction on material reflectivity
    overrides: dict[str, dict[str, Any]] | None = None  # per-type range overrides

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ObjectFieldConfig":
        return cls(
            density_per_hectare=float(d.get("density_per_hectare", 60.0)),
            composition=dict(d["composition"]) if "composition" in d else None,
            margin_m=float(d.get("margin_m", 3.0)),
            min_separation_m=float(d.get("min_separation_m", 1.0)),
            size_scale=float(d.get("size_scale", 1.0)),
            burial_scale=float(d.get("burial_scale", 1.0)),
            reflectivity_jitter=float(d.get("reflectivity_jitter", 0.15)),
            overrides=dict(d.get("overrides", {})) or None,
        )


def _sample_range(rng: np.random.Generator, rr: tuple[float, float]) -> float:
    return float(rng.uniform(rr[0], rr[1]))


def place_objects(grid: GridSpec, cfg: ObjectFieldConfig,
                  materials: MaterialLibrary,
                  rng: np.random.Generator) -> list[PlacedObject]:
    """Sample object instances with dart-throwing separation enforcement."""
    xmin, ymin, xmax, ymax = grid.extent
    area_ha = (xmax - xmin) * (ymax - ymin) / 1e4
    n_target = max(0, int(round(cfg.density_per_hectare * area_ha)))

    comp = dict(cfg.composition or DEFAULT_COMPOSITION)
    comp = {k: v for k, v in comp.items() if k in CATALOG and v > 0}
    if not comp:
        return []
    types = list(comp)
    weights = np.array([comp[t] for t in types], dtype=float)
    weights /= weights.sum()

    placed: list[PlacedObject] = []
    positions: list[tuple[float, float]] = []
    attempts = 0
    while len(placed) < n_target and attempts < n_target * 30 + 100:
        attempts += 1
        x = rng.uniform(xmin + cfg.margin_m, xmax - cfg.margin_m)
        y = rng.uniform(ymin + cfg.margin_m, ymax - cfg.margin_m)
        if positions:
            d2 = min((x - px) ** 2 + (y - py) ** 2 for px, py in positions)
            if d2 < cfg.min_separation_m ** 2:
                continue

        t = types[int(rng.choice(len(types), p=weights))]
        spec = CATALOG[t]
        ov = (cfg.overrides or {}).get(t, {})
        lr = tuple(ov.get("length_range", spec.length_range))
        wr = tuple(ov.get("width_range", spec.width_range))
        hr = tuple(ov.get("height_range", spec.height_range))
        br = tuple(ov.get("burial_range", spec.burial_range))

        length = _sample_range(rng, lr) * cfg.size_scale
        width = length if spec.round_footprint else _sample_range(rng, wr) * cfg.size_scale
        height = _sample_range(rng, hr) * cfg.size_scale
        burial = float(np.clip(_sample_range(rng, br) * cfg.burial_scale, 0.0, 0.95))

        mat = materials.get(str(ov.get("material", spec.material)))
        refl = float(np.clip(
            mat.reflectivity * (1.0 + rng.uniform(-1, 1) * cfg.reflectivity_jitter),
            0.05, 1.0))

        placed.append(PlacedObject(
            object_id=len(placed), type=t, x=x, y=y,
            yaw=float(rng.uniform(-np.pi, np.pi)),
            length=length, width=width, proud_height=height,
            burial=burial, reflectivity=refl, material=mat.name))
        positions.append((x, y))
    return placed


def rasterize_objects(objects: list[PlacedObject], grid: GridSpec,
                      height: np.ndarray, reflectivity: np.ndarray) -> None:
    """Stamp each object into the acoustic rasters *in place*.

    ``height`` gains a bump of ``effective_height * profile`` and
    ``reflectivity`` is blended toward the object reflectivity inside the
    footprint. Buried objects contribute a residual low bump plus a weak
    reflectivity contrast (sediment-covered target)."""
    res = grid.resolution
    for o in objects:
        spec = CATALOG[o.type]
        pad = o.footprint_radius + 2 * res
        cx0, cy0 = grid.world_to_grid(o.x - pad, o.y - pad)
        cx1, cy1 = grid.world_to_grid(o.x + pad, o.y + pad)
        i0, i1 = max(int(cx0), 0), min(int(np.ceil(cx1)) + 1, grid.nx)
        j0, j1 = max(int(cy0), 0), min(int(np.ceil(cy1)) + 1, grid.ny)
        if i0 >= i1 or j0 >= j1:
            continue
        xs = grid.origin_x + np.arange(i0, i1) * res
        ys = grid.origin_y + np.arange(j0, j1) * res
        xx, yy = np.meshgrid(xs, ys)
        c, s = np.cos(o.yaw), np.sin(o.yaw)
        xl = c * (xx - o.x) + s * (yy - o.y)
        yl = -s * (xx - o.x) + c * (yy - o.y)
        prof = spec.mask_fn(xl, yl, o)
        if prof.max() <= 0:
            continue
        h_eff = o.effective_height
        height[j0:j1, i0:i1] += h_eff * prof
        # Reflectivity blend, weakened with burial.
        w = np.clip(prof, 0.0, 1.0) * (1.0 - 0.8 * o.burial)
        reflectivity[j0:j1, i0:i1] = (
            (1.0 - w) * reflectivity[j0:j1, i0:i1] + w * o.reflectivity)
