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

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

import numpy as np

from ..core.types import GridSpec, PlacedObject
from .materials import MaterialLibrary
from .solids import (Solid, rotation_rpy, rpy_from_rotation, settle,
                     stable_poses)

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


def _mask_cylinder_upright(xl: np.ndarray, yl: np.ndarray,
                           o: PlacedObject) -> np.ndarray:
    """Drum/pail standing on the seabed: flat-topped disc footprint."""
    return (np.hypot(xl, yl) <= o.length / 2).astype(np.float64)


def _mask_cone(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
    """Traffic cone / bollard: linear radial taper -- no broadside face."""
    rr = np.hypot(xl, yl)
    return np.clip(1.0 - rr / max(o.length / 2, 1e-6), 0.0, 1.0)


def _mask_coil(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
    """Rope coil / net ball: low dome with toroidal ring texture -- a
    diffuse many-scatterer lump with no specular face (family F8)."""
    rng = np.random.default_rng(o.object_id)
    a, b = max(o.length / 2, 1e-6), max(o.width / 2, 1e-6)
    d = np.hypot(xl / a, yl / b)
    dome = np.sqrt(np.clip(1.0 - d ** 2, 0.0, 1.0))
    rings = 0.55 + 0.45 * np.cos(np.pi * d * rng.uniform(2.5, 4.5)) ** 2
    return dome * rings


def _mask_lattice(base_fn: MaskFn, cell_m: float = 0.10) -> MaskFn:
    """Open-frame wrapper (family F4): the outline of the base footprint
    stays solid, interior cells of a ``cell_m`` lattice are kept with
    probability ``fill_fraction`` -- deterministic per object via a cell
    hash, so the same object always renders the same skeleton.

    The acoustic dilution then *emerges* from the ordinary raster path:
    height bumps exist only on kept cells (sparse thin-member flashes,
    broken/partial shadow) and the reflectivity blend weight follows the
    same sparse profile (diluted highlight). Nothing in the renderer
    changes, so the first-bottom-return machinery (NC #4) is untouched.
    """

    def fn(xl: np.ndarray, yl: np.ndarray, o: PlacedObject) -> np.ndarray:
        base = base_fn(xl, yl, o)
        if base.max() <= 0:
            return base
        fill = float(np.clip(o.fill_fraction, 0.02, 1.0))
        # Outline frame: a thin solid rim (like a crate's or frame's edge
        # members), obtained by evaluating the base mask on per-axis
        # scaled-up local coords so each axis shrinks by the same absolute
        # rim width; frame = inside the footprint but outside the shrunk
        # one.
        rim = min(0.04, 0.25 * min(o.length, o.width))
        sx = o.length / max(o.length - 2.0 * rim, 1e-6)
        sy = o.width / max(o.width - 2.0 * rim, 1e-6)
        inner = base_fn(xl * sx, yl * sy, o)
        frame = (base > 0) & (inner <= 0)
        ci = np.floor(xl / cell_m).astype(np.int64)
        cj = np.floor(yl / cell_m).astype(np.int64)
        h = (ci * 73856093) ^ (cj * 19349663) ^ np.int64(o.object_id * 83492791)
        keep = ((h & 0xFFFF) / 65535.0) < fill
        return base * (frame | keep)

    return fn


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
    sdf_shape: str = "box"                # box | cylinder | cylinder_upright | cone
    label: str = ""                       # GUI display name ("" = from type)
    #: Open-frame classes only: fill-fraction sampling range (None = solid).
    fill_fraction_range: tuple[float, float] | None = None
    #: Class-specific GUI knobs, SchemaForm rows (key, label, kind, default,
    #: min, max); the builder renders them automatically.
    params_schema: tuple = ()

    def solid(self, o: PlacedObject) -> Solid | None:
        """The class's 3-D primitive in its canonical frame, or None for
        classes with no stable rest poses (lines, blobs, tangles)."""
        return _solid_for(self, o)

    @property
    def display_label(self) -> str:
        return self.label or self.type.replace("_", " ")


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

    # ---- target-family additions (target_families_and_surrogates.md) ----
    # F1 upright rigid box -- the mirror class (sharp aspect dependence).
    "crate_plastic":   ObjectSpec("crate_plastic", _mask_box, (0.45, 0.55),
                                  (0.30, 0.40), (0.18, 0.25), "plastic",
                                  (0.0, 0.3), False, "box",
                                  label="fish crate (PP)"),
    "tin_18l":         ObjectSpec("tin_18l", _mask_box, (0.22, 0.25),
                                  (0.22, 0.25), (0.32, 0.36), "tinplate",
                                  (0.0, 0.3), False, "box",
                                  label="18 L tin can (isshoka)"),
    "polytank":        ObjectSpec("polytank", _mask_box, (0.32, 0.36),
                                  (0.22, 0.26), (0.42, 0.48), "plastic",
                                  (0.0, 0.3), False, "box",
                                  label="20 L polytank"),
    # F2 large upright cylinder -- the aspect-independence control.
    "drum_200l":       ObjectSpec("drum_200l", _mask_cylinder_upright,
                                  (0.56, 0.60), (0.56, 0.60), (0.85, 0.92),
                                  "metal", (0.0, 0.2), True, "cylinder_upright",
                                  label="200 L steel drum"),
    "pail_20l":        ObjectSpec("pail_20l", _mask_cylinder_upright,
                                  (0.27, 0.31), (0.27, 0.31), (0.34, 0.40),
                                  "tinplate", (0.0, 0.3), True,
                                  "cylinder_upright", label="20 L pail can"),
    "fire_extinguisher": ObjectSpec("fire_extinguisher", _mask_cylinder_lying,
                                    (0.44, 0.49), (0.13, 0.15), (0.13, 0.15),
                                    "metal", (0.0, 0.4), False, "cylinder",
                                    label="fire extinguisher"),
    # F5 elongated linear rigid (wooden pallet: large footprint, open slats).
    "pallet_wood":     ObjectSpec("pallet_wood", _mask_lattice(_mask_box),
                                  (1.05, 1.15), (1.05, 1.15), (0.12, 0.16),
                                  "wood", (0.0, 0.4), False, "box",
                                  label="wooden pallet (JIS T11)",
                                  fill_fraction_range=(0.50, 0.70)),
    # F6 tapered/conical -- a cone has no broadside.
    "traffic_cone":    ObjectSpec("traffic_cone", _mask_cone, (0.36, 0.40),
                                  (0.36, 0.40), (0.60, 0.75), "plastic",
                                  (0.0, 0.2), True, "cone",
                                  label="traffic cone"),
    # F8 tangled compliant mass -- most frequent family in marina surveys.
    "rope_coil":       ObjectSpec("rope_coil", _mask_coil, (0.30, 0.80),
                                  (0.30, 0.80), (0.08, 0.18), "rope",
                                  (0.0, 0.5), True, "box",
                                  label="rope/mooring-line coil"),
    "net_ball":        ObjectSpec("net_ball", _mask_coil, (0.50, 1.50),
                                  (0.40, 1.20), (0.20, 0.50), "rope",
                                  (0.0, 0.4), False, "box",
                                  label="net ball / trawl offcut"),
    # F9 small hard threshold objects.
    "bottle_glass":    ObjectSpec("bottle_glass", _mask_capsule, (0.30, 0.40),
                                  (0.09, 0.11), (0.09, 0.11), "glass",
                                  (0.0, 0.6), False, "cylinder",
                                  label="glass bottle (isshobin)"),
    # F4 open-frame / skeletal -- the wireframe failure family.
    "bicycle":         ObjectSpec("bicycle", _mask_lattice(_mask_box),
                                  (1.50, 1.80), (0.50, 0.80), (0.35, 0.60),
                                  "metal", (0.0, 0.3), False, "box",
                                  label="bicycle (lying)",
                                  fill_fraction_range=(0.12, 0.25)),
    "mesh_crate":      ObjectSpec("mesh_crate", _mask_lattice(_mask_box),
                                  (0.50, 0.56), (0.34, 0.39), (0.28, 0.34),
                                  "plastic", (0.0, 0.3), False, "box",
                                  label="50 L mesh container",
                                  fill_fraction_range=(0.30, 0.50)),
    "shopping_trolley": ObjectSpec("shopping_trolley", _mask_lattice(_mask_box),
                                   (0.90, 1.10), (0.55, 0.70), (0.45, 0.70),
                                   "metal", (0.0, 0.3), False, "box",
                                   label="shopping trolley (lying)",
                                   fill_fraction_range=(0.18, 0.35)),
}

#: Cells whose terrain height (z, negative down) is at or above this are
#: dry or awash -- a beach above the waterline -- and never receive a
#: random object (the sonar cannot see them; explicit instances stay
#: verbatim wherever the author pins them).
DRY_HEIGHT_M = -0.05

#: Legacy random-composition weights. Deliberately unchanged by the
#: target-family additions: extending this dict would silently reshuffle
#: every legacy seed's object draw. New classes enter a world through the
#: explicit per-class ``objects.classes`` config (the world builder's path).
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
    #: Builder-style per-class placement (see :func:`place_objects_v2`).
    #: When not ``None`` (or ``explicit`` is non-empty), it replaces the
    #: legacy density/composition draw entirely -- an empty dict places
    #: nothing at all; ``None`` (key absent) is the legacy random field.
    classes: dict[str, dict[str, Any]] | None = None
    #: Hand-authored instances placed verbatim (matched pairs, RTK-fixed
    #: clusters): ``[{type, x, y[, yaw, length, width, proud_height,
    #: burial, fill_fraction, material]}]``.
    explicit: list = field(default_factory=list)
    #: The source survey path in **world** metres, ``[[x, y], ...]`` --
    #: what ``orientation.mode: path_relative`` measures headings against
    #: (the builder passes ``BuilderState.path_world_xy()``).
    path_xy: list | None = None
    #: Headless fallback for ``path_relative`` when no ``path_xy`` is
    #: given: one track heading in ENU degrees (0 = +x/east, CCW).
    path_heading_deg: float | None = None

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "ObjectFieldConfig":
        path_xy = d.get("path_xy")
        heading = d.get("path_heading_deg")
        return cls(
            density_per_hectare=float(d.get("density_per_hectare", 60.0)),
            composition=dict(d["composition"]) if "composition" in d else None,
            margin_m=float(d.get("margin_m", 3.0)),
            min_separation_m=float(d.get("min_separation_m", 1.0)),
            size_scale=float(d.get("size_scale", 1.0)),
            burial_scale=float(d.get("burial_scale", 1.0)),
            reflectivity_jitter=float(d.get("reflectivity_jitter", 0.15)),
            overrides=dict(d.get("overrides", {})) or None,
            # ``classes: {}`` (the builder with nothing ticked) means *no
            # objects*; only an absent key falls to the legacy composition.
            classes=dict(d["classes"]) if "classes" in d else None,
            explicit=list(d.get("explicit", []) or []),
            path_xy=([[float(x), float(y)] for x, y in path_xy]
                     if path_xy is not None and len(path_xy) else None),
            path_heading_deg=(float(heading) if heading is not None
                              else None),
        )


def _sample_range(rng: np.random.Generator, rr: tuple[float, float]) -> float:
    return float(rng.uniform(rr[0], rr[1]))


# ------------------------------------------------- class introspection
# Qt-free helpers behind the builder GUI's per-class "Info" dialog (and
# smoke-testable for every CATALOG entry).
_SHAPE_TEXT = {
    "_mask_box": "rectangular box",
    "_mask_cylinder_lying": "cylinder lying on the seabed",
    "_mask_cylinder_upright": "upright cylinder (flat circular footprint)",
    "_mask_annulus": "ring lying flat (tire), rounded cross-section",
    "_mask_capsule": "capsule lying on its side (bottle)",
    "_mask_anchor": "cross/T silhouette (shank + flukes)",
    "_mask_blob": "irregular rounded blob",
    "_mask_cone": "cone -- radius tapers linearly, no flat face",
    "_mask_coil": "low dome with ring texture (coiled line / tangle)",
}


def _solid_for(spec: ObjectSpec, o: PlacedObject) -> Solid | None:
    """Primitive behind a class's mask, sized from the instance."""
    name = getattr(spec.mask_fn, "__name__", "")
    if spec.fill_fraction_range is not None or name == "_mask_box":
        return Solid("box", o.length, o.width, o.proud_height)
    if name == "_mask_cylinder_lying":
        return Solid("cylinder", 0.5 * o.width, o.length, axis=0)
    if name == "_mask_cylinder_upright":
        return Solid("cylinder", 0.5 * o.length, o.proud_height, axis=2)
    if name == "_mask_capsule":
        return Solid("capsule", 0.5 * o.width, o.length, axis=0)
    if name == "_mask_annulus":
        return Solid("annulus", 0.5 * o.length, 0.275 * o.length, o.proud_height)
    if name == "_mask_cone":
        return Solid("cone", 0.5 * o.length, o.proud_height)
    return None                       # rope/chain/anchor/blob/coil: 2.5-D only


def rest_poses(spec: ObjectSpec) -> tuple:
    """Stable rest poses of the class's typical instance (empty when the
    class has no 3-D solid)."""
    sol = spec.solid(typical_instance(spec))
    return stable_poses(sol) if sol is not None else ()


def rest_poses_text(spec: ObjectSpec) -> str:
    """One line for the Info dialog: how the class can lie on the seabed."""
    poses = rest_poses(spec)
    if not poses:
        return "no 3-D rest poses (drawn flat on the seabed)"
    tot = sum(p.weight for p in poses) or 1.0
    return ", ".join(f"{p.name} {100 * p.weight / tot:.0f}%" for p in poses)


def _rest_rng(o: PlacedObject) -> np.random.Generator:
    """Per-object stream for the rest-pose draw: seeded from the object's
    id and position only, so enabling 3-D rest for one class moves no
    other object and never touches the shared placement stream."""
    return np.random.default_rng([2, int(o.object_id),
                                  int(round(o.x * 1000.0)) & 0xFFFFFFFF,
                                  int(round(o.y * 1000.0)) & 0xFFFFFFFF])


def draw_rest(spec: ObjectSpec, o: PlacedObject, randomness: float) -> None:
    """Give ``o`` a resting orientation in place.

    With probability ``1 - randomness`` the canonical pose (today's look),
    else a draw from the class's stable poses by weight; on top, a settle
    tilt ``N(0, 4 deg * randomness)`` about both horizontal axes. The pose
    is expressed as roll/pitch of the canonical class frame; ``rest`` names
    it (``standing``, ``lying``, ``face:+x`` ...). Classes without a solid
    are left untouched.
    """
    sol = spec.solid(o)
    if sol is None:
        return
    r = float(np.clip(randomness, 0.0, 1.0))
    rng = _rest_rng(o)
    poses = stable_poses(sol)
    u = float(rng.uniform())
    if u < r and len(poses) > 1:
        w = np.array([p.weight for p in poses], dtype=np.float64)
        pick = poses[int(rng.choice(len(poses), p=w / w.sum()))]
    else:
        rng.choice(len(poses))               # keep the stream aligned
        pick = poses[0]
    tilt = np.radians(4.0) * r
    o.roll = float(pick.roll + rng.normal(0.0, tilt))
    o.pitch = float(pick.pitch + rng.normal(0.0, tilt))
    o.rest = pick.name if pick is not poses[0] else "settled"


def shape_description(spec: ObjectSpec) -> str:
    """One-line 'typical shape' text for a catalog class."""
    if spec.fill_fraction_range is not None:
        lo, hi = spec.fill_fraction_range
        return (f"open frame / lattice: a thin solid rim with only "
                f"{lo:.0%}-{hi:.0%} of the interior filled by structure "
                "(sparse flashes, broken shadow)")
    name = getattr(spec.mask_fn, "__name__", "")
    if name == "fn":                       # _mask_line_segments closure
        return "thin wavy line on the seabed (rope / chain)"
    return _SHAPE_TEXT.get(name, "rectangular box")


def typical_instance(spec: ObjectSpec) -> PlacedObject:
    """A representative instance: every dimension at its range midpoint,
    unburied, mid fill fraction. Deterministic -- the 'regular size'."""
    def mid(rr):
        return 0.5 * (rr[0] + rr[1])

    length = mid(spec.length_range)
    return PlacedObject(
        object_id=1, type=spec.type, x=0.0, y=0.0, yaw=0.0,
        length=length,
        width=length if spec.round_footprint else mid(spec.width_range),
        proud_height=mid(spec.height_range), burial=0.0, reflectivity=0.8,
        material=spec.material,
        fill_fraction=(mid(spec.fill_fraction_range)
                       if spec.fill_fraction_range is not None else 1.0))


def footprint_profile(spec: ObjectSpec, n: int = 220
                      ) -> tuple[np.ndarray, PlacedObject]:
    """Top-down height-profile image [n, n] in [0, 1] of the typical
    instance -- the acoustic footprint the sonar will actually see."""
    o = typical_instance(spec)
    half = 0.58 * max(o.length, o.width)
    xs = np.linspace(-half, half, n)
    xl, yl = np.meshgrid(xs, xs)
    prof = np.clip(spec.mask_fn(xl, yl, o), 0.0, 1.0)
    peak = float(prof.max())
    return (prof / peak if peak > 0 else prof), o


def _draw_dist(dist: Mapping[str, Any] | None, rng: np.random.Generator,
               default: float = 1.0) -> float:
    """Draw from a ``{kind: uniform, lo, hi}`` / ``{kind: gauss, mean,
    sigma}`` distribution spec; None returns ``default``."""
    if dist is None:
        return default
    kind = str(dist.get("kind", "uniform"))
    if kind == "uniform":
        return float(rng.uniform(float(dist.get("lo", default)),
                                 float(dist.get("hi", default))))
    if kind == "gauss":
        return float(rng.normal(float(dist.get("mean", default)),
                                float(dist.get("sigma", 0.0))))
    raise ValueError(f"distribution kind '{kind}' unknown (uniform | gauss)")


def _mid(rr: tuple[float, float]) -> float:
    return 0.5 * (rr[0] + rr[1])


# ------------------------------------------------- orientation
ORIENTATION_MODES = ("random", "fixed", "path_relative")
_ORIENTATION_KEYS = {"mode", "heading_deg", "jitter_deg", "rest"}
_REST_KEYS = {"enabled", "randomness"}


def normalise_orientation(orient: Mapping[str, Any] | None) -> dict[str, Any]:
    """Validate a per-class ``orientation`` block and fill its defaults.

    Schema: ``{mode: random | fixed | path_relative, heading_deg: float
    (default 0), jitter_deg: float (default 0)}``. Under ``fixed``,
    ``heading_deg`` is the object's long-axis heading in ENU degrees
    (0 = +x/east, CCW positive); under ``path_relative`` it is the offset
    from the local track heading (0 = along-track, 90 = across-track).
    Unknown modes and unknown keys raise by name.
    """
    orient = dict(orient or {})
    unknown = set(orient) - _ORIENTATION_KEYS
    if unknown:
        raise ValueError(f"orientation: unknown key(s) {sorted(unknown)} "
                         f"(allowed: {sorted(_ORIENTATION_KEYS)})")
    mode = str(orient.get("mode", "random"))
    if mode not in ORIENTATION_MODES:
        raise ValueError(f"orientation mode '{mode}' unknown "
                         f"({' | '.join(ORIENTATION_MODES)})")
    jitter = float(orient.get("jitter_deg", 0.0))
    if jitter < 0.0:
        raise ValueError(f"orientation jitter_deg must be >= 0, got {jitter}")
    rest = dict(orient.get("rest") or {})
    bad = set(rest) - _REST_KEYS
    if bad:
        raise ValueError(f"orientation.rest: unknown key(s) {sorted(bad)} "
                         f"(allowed: {sorted(_REST_KEYS)})")
    randomness = float(rest.get("randomness", 0.5))
    if not 0.0 <= randomness <= 1.0:
        raise ValueError("orientation.rest.randomness must be in 0..1, "
                         f"got {randomness}")
    return {"mode": mode,
            "heading_deg": float(orient.get("heading_deg", 0.0)),
            "jitter_deg": jitter,
            "rest": {"enabled": bool(rest.get("enabled", False)),
                     "randomness": randomness}}


def nearest_segment_heading(path_xy: np.ndarray, x: float, y: float) -> float:
    """ENU heading [rad] of the polyline segment closest to ``(x, y)``.

    Zero-length segments are ignored; a path with fewer than two distinct
    points raises.
    """
    pts = np.asarray(path_xy, dtype=np.float64).reshape(-1, 2)
    if len(pts) < 2:
        raise ValueError("path_relative orientation needs a path with at "
                         "least two points")
    a = pts[:-1]
    d = pts[1:] - a
    seg_len2 = (d ** 2).sum(axis=1)
    ok = seg_len2 > 0.0
    if not ok.any():
        raise ValueError("path_relative orientation: every path segment "
                         "has zero length")
    a, d, seg_len2 = a[ok], d[ok], seg_len2[ok]
    q = np.array([x, y]) - a
    t = np.clip((q * d).sum(axis=1) / seg_len2, 0.0, 1.0)
    dist2 = ((q - t[:, None] * d) ** 2).sum(axis=1)
    k = int(np.argmin(dist2))
    return float(np.arctan2(d[k, 1], d[k, 0]))


def _wrap_pi(a: float) -> float:
    if -np.pi <= a <= np.pi:
        return a                      # untouched: exact for in-range values
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


def _draw_yaw(orient: Mapping[str, Any], rng: np.random.Generator,
              path_heading_rad: float | None) -> float:
    """One object's yaw [rad, ENU] under a normalised ``orientation``.

    RNG contract: **every mode draws the same ``rng.uniform(-pi, pi)``
    variate first**, so switching a class between modes never moves the
    other objects and ``random`` (the default) reproduces every existing
    seed bit-for-bit. ``fixed`` / ``path_relative`` then add one
    ``rng.normal(0, jitter)`` draw **only when ``jitter_deg > 0``** -- that
    extra draw is the one place the stream differs from ``random`` for the
    objects that follow within the same class.
    """
    u = float(rng.uniform(-np.pi, np.pi))
    mode = orient["mode"]
    if mode == "random":
        return u
    if mode == "fixed":
        base = float(np.radians(orient["heading_deg"]))
    elif mode == "path_relative":
        if path_heading_rad is None:
            raise ValueError("orientation mode path_relative needs a path: "
                             "give objects.path_xy ([[x, y], ...] in world "
                             "metres) or objects.path_heading_deg")
        base = float(path_heading_rad) + float(np.radians(orient["heading_deg"]))
    else:  # pragma: no cover - normalise_orientation already rejected it
        raise ValueError(f"orientation mode '{mode}' unknown")
    jitter = float(orient["jitter_deg"])
    if jitter > 0.0:
        base += float(rng.normal(0.0, np.radians(jitter)))
    return _wrap_pi(base)


def _make_instance(spec: ObjectSpec, object_id: int, x: float, y: float,
                   yaw: float, cfg: ObjectFieldConfig,
                   materials: MaterialLibrary, rng: np.random.Generator,
                   size_dist: Mapping[str, Any] | None = None,
                   burial_dist: Mapping[str, Any] | None = None,
                   material_override: str | None = None,
                   fill_override: float | None = None) -> PlacedObject:
    """One instance from a spec: sizes, burial, fill, reflectivity."""
    if size_dist is not None:
        s = max(_draw_dist(size_dist, rng), 0.05)
        length = _mid(spec.length_range) * s
        width = length if spec.round_footprint else _mid(spec.width_range) * s
        height = _mid(spec.height_range) * s
    else:
        length = _sample_range(rng, spec.length_range) * cfg.size_scale
        width = (length if spec.round_footprint
                 else _sample_range(rng, spec.width_range) * cfg.size_scale)
        height = _sample_range(rng, spec.height_range) * cfg.size_scale
    if burial_dist is not None:
        burial = float(np.clip(_draw_dist(burial_dist, rng, 0.0), 0.0, 0.95))
    else:
        burial = float(np.clip(
            _sample_range(rng, spec.burial_range) * cfg.burial_scale, 0.0, 0.95))
    if fill_override is not None:
        fill = float(np.clip(fill_override, 0.02, 1.0))
    elif spec.fill_fraction_range is not None:
        fill = _sample_range(rng, spec.fill_fraction_range)
    else:
        fill = 1.0
    mat = materials.get(material_override or spec.material)
    refl = float(np.clip(
        mat.reflectivity * (1.0 + rng.uniform(-1, 1) * cfg.reflectivity_jitter),
        0.05, 1.0))
    return PlacedObject(
        object_id=object_id, type=spec.type, x=float(x), y=float(y),
        yaw=float(yaw), length=length, width=width, proud_height=height,
        burial=burial, reflectivity=refl, material=mat.name,
        fill_fraction=fill)


def _is_dry(grid: GridSpec, height: np.ndarray, x: float, y: float) -> bool:
    """True when the terrain at (x, y) is at or above ``DRY_HEIGHT_M``."""
    ci, cj = grid.world_to_grid(x, y)
    ci = int(np.clip(np.round(ci), 0, grid.nx - 1))
    cj = int(np.clip(np.round(cj), 0, grid.ny - 1))
    return bool(height[cj, ci] >= DRY_HEIGHT_M)


def place_objects_v2(grid: GridSpec, cfg: ObjectFieldConfig,
                     materials: MaterialLibrary,
                     rng: np.random.Generator,
                     height: np.ndarray | None = None) -> list[PlacedObject]:
    """Per-class placement: count/density, optional Gaussian clustering,
    per-class size/burial distributions, per-class ``orientation``
    (random | fixed | path_relative, see :func:`normalise_orientation`),
    plus verbatim explicit instances.

    Classes are processed in sorted name order so the draw sequence -- and
    therefore the whole world -- is deterministic per seed regardless of
    dict insertion order.
    """
    xmin, ymin, xmax, ymax = grid.extent
    area_ha = (xmax - xmin) * (ymax - ymin) / 1e4
    m = cfg.margin_m
    lo = np.array([xmin + m, ymin + m])
    hi = np.array([xmax - m, ymax - m])
    if (hi <= lo).any():
        raise ValueError("world too small for the object margin")

    placed: list[PlacedObject] = []
    positions: list[tuple[float, float]] = []
    path_pts = (np.asarray(cfg.path_xy, dtype=np.float64).reshape(-1, 2)
                if cfg.path_xy else None)

    def far_enough(x: float, y: float) -> bool:
        if not positions:
            return True
        d2 = min((x - px) ** 2 + (y - py) ** 2 for px, py in positions)
        return d2 >= cfg.min_separation_m ** 2

    for type_name in sorted(cfg.classes or {}):
        c = cfg.classes[type_name]
        if not c.get("enabled", True):
            continue
        if type_name not in CATALOG:
            raise KeyError(f"Unknown object class '{type_name}' "
                           f"(available: {sorted(CATALOG)})")
        spec = CATALOG[type_name]
        amount = c.get("amount", {}) or {}
        a_kind = str(amount.get("kind", "count"))
        a_val = float(amount.get("value", 0))
        if a_kind == "count":
            n = max(0, int(round(a_val)))
        elif a_kind == "density":
            n = max(0, int(round(a_val * area_ha)))
        else:
            raise ValueError(f"amount kind '{a_kind}' unknown (count | density)")

        orient = normalise_orientation(c.get("orientation"))
        if orient["mode"] == "path_relative":
            if path_pts is None and cfg.path_heading_deg is None:
                raise ValueError(
                    f"class '{type_name}': orientation mode path_relative "
                    "needs a path -- give objects.path_xy ([[x, y], ...] in "
                    "world metres) or objects.path_heading_deg")
            if path_pts is not None:
                nearest_segment_heading(path_pts, 0.0, 0.0)   # validate once

        clusters = c.get("clusters")
        centres = None
        sigma = 0.0
        if clusters and int(clusters.get("n", 0)) > 0:
            k = int(clusters["n"])
            sigma = float(clusters.get("sigma_m", 3.0))
            centres = rng.uniform(lo, hi, size=(k, 2))

        n_before = len(placed)
        attempts = 0
        while len(placed) - n_before < n and attempts < n * 30 + 100:
            attempts += 1
            if centres is not None:
                c0 = centres[int(rng.integers(len(centres)))]
                x, y = np.clip(c0 + rng.normal(0.0, sigma, 2), lo, hi)
            else:
                x, y = rng.uniform(lo, hi)
            if not far_enough(float(x), float(y)):
                continue
            if height is not None and _is_dry(grid, height, float(x), float(y)):
                continue
            fill_p = (c.get("params", {}) or {}).get("fill_fraction")
            track = None
            if orient["mode"] == "path_relative":
                track = (nearest_segment_heading(path_pts, float(x), float(y))
                         if path_pts is not None
                         else float(np.radians(cfg.path_heading_deg)))
            # Drawn before _make_instance's own draws, exactly where the
            # historical uniform yaw sat in the stream.
            yaw = _draw_yaw(orient, rng, track)
            inst = _make_instance(
                spec, len(placed), float(x), float(y),
                yaw, cfg, materials, rng,
                size_dist=c.get("size"), burial_dist=c.get("burial"),
                material_override=c.get("material"),
                fill_override=float(fill_p) if fill_p is not None else None)
            if orient["rest"]["enabled"]:
                # Own per-object stream: the shared placement stream is
                # untouched, so the other classes stay where they are.
                draw_rest(spec, inst, orient["rest"]["randomness"])
            placed.append(inst)
            positions.append((float(x), float(y)))

    for item in cfg.explicit:
        t = str(item["type"])
        if t not in CATALOG:
            raise KeyError(f"Unknown object class '{t}' in objects.explicit "
                           f"(available: {sorted(CATALOG)})")
        spec = CATALOG[t]
        inst = _make_instance(spec, len(placed), float(item["x"]),
                              float(item["y"]),
                              float(item.get("yaw", 0.0)), cfg, materials, rng)
        # Verbatim overrides: whatever the author pins, stays pinned.
        for key in ("length", "width", "proud_height", "burial",
                    "fill_fraction", "reflectivity"):
            if key in item:
                setattr(inst, key, float(item[key]))
        if "material" in item:
            inst.material = str(item["material"])
        if spec.round_footprint and "length" in item and "width" not in item:
            inst.width = inst.length
        # Verbatim 3-D rest: roll/pitch in degrees of the canonical frame.
        if "roll_deg" in item or "pitch_deg" in item:
            inst.roll = float(np.radians(float(item.get("roll_deg", 0.0))))
            inst.pitch = float(np.radians(float(item.get("pitch_deg", 0.0))))
            inst.rest = str(item.get("rest", "explicit"))
        placed.append(inst)
        positions.append((inst.x, inst.y))
    return placed


def place_objects(grid: GridSpec, cfg: ObjectFieldConfig,
                  materials: MaterialLibrary,
                  rng: np.random.Generator,
                  height: np.ndarray | None = None) -> list[PlacedObject]:
    """Sample object instances with dart-throwing separation enforcement.

    A config carrying a ``classes`` mapping (even an empty one -- "nothing
    ticked" means nothing placed) or ``explicit`` entries routes to
    :func:`place_objects_v2`; the legacy density/composition body below is
    untouched, keeping every existing seed's world bit-identical.

    ``height`` (the terrain-only raster, z negative down) lets the v2 path
    keep objects off dry cells (height above ``DRY_HEIGHT_M``); a world
    without dry cells rejects no candidate, so its draw sequence -- and
    therefore its objects -- are bit-identical with and without it.
    """
    if cfg.classes is not None or cfg.explicit:
        return place_objects_v2(grid, cfg, materials, rng, height)
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


def _rasterize_solid(o: PlacedObject, spec: ObjectSpec, grid: GridSpec,
                     height: np.ndarray, reflectivity: np.ndarray) -> None:
    """Stamp a rested (rotated) solid: z-buffer from above, settled on
    the local seabed plane (``worldgen/solids.py``). Lattice classes keep
    their skeleton: the solid's height is multiplied by the yaw-frame
    lattice mask evaluated on the rotated footprint."""
    res = grid.resolution
    sol = spec.solid(o)
    assert sol is not None
    pad = 0.5 * float(np.sqrt(o.length ** 2 + o.width ** 2
                              + o.proud_height ** 2)) + 2 * res
    cx0, cy0 = grid.world_to_grid(o.x - pad, o.y - pad)
    cx1, cy1 = grid.world_to_grid(o.x + pad, o.y + pad)
    i0, i1 = max(int(cx0), 0), min(int(np.ceil(cx1)) + 1, grid.nx)
    j0, j1 = max(int(cy0), 0), min(int(np.ceil(cy1)) + 1, grid.ny)
    if i1 - i0 < 2 or j1 - j0 < 2:
        return
    xs = grid.origin_x + np.arange(i0, i1) * res
    ys = grid.origin_y + np.arange(j0, j1) * res
    xx, yy = np.meshgrid(xs, ys)
    R = rotation_rpy(o.roll, o.pitch, o.yaw)
    terrain = height[j0:j1, i0:i1]
    bump, prof, centre_z, R_world = settle(
        sol, R, xx - o.x, yy - o.y, terrain, burial=o.burial)
    if spec.fill_fraction_range is not None:
        # Skeleton: the box footprint of the *rotated* solid, lattice-
        # thinned in the yaw frame like the flat stamp.
        c, s = np.cos(o.yaw), np.sin(o.yaw)
        xl = c * (xx - o.x) + s * (yy - o.y)
        yl = -s * (xx - o.x) + c * (yy - o.y)
        ext = np.abs(R_world @ np.diag(sol.extent))   # projected half-sizes
        proxy = PlacedObject(
            object_id=o.object_id, type=o.type, x=0.0, y=0.0, yaw=0.0,
            length=2.0 * float(np.hypot(ext[0, 0], ext[0, 1]) + ext[0, 2]),
            width=2.0 * float(np.hypot(ext[1, 0], ext[1, 1]) + ext[1, 2]),
            proud_height=o.proud_height, burial=o.burial,
            reflectivity=o.reflectivity, fill_fraction=o.fill_fraction)
        keep = spec.mask_fn(xl, yl, proxy) > 0.0
        bump = bump * keep
        prof = prof * keep
    if bump.max() <= 0.0:
        return
    height[j0:j1, i0:i1] += bump
    w = np.clip(prof, 0.0, 1.0) * (1.0 - 0.8 * o.burial)
    reflectivity[j0:j1, i0:i1] = (
        (1.0 - w) * reflectivity[j0:j1, i0:i1] + w * o.reflectivity)
    o.rest_z = float(centre_z)
    o.roll, o.pitch, _ = rpy_from_rotation(R_world)   # includes the bank tilt


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
        if o.rested and spec.solid(o) is not None:
            _rasterize_solid(o, spec, grid, height, reflectivity)
            continue
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
