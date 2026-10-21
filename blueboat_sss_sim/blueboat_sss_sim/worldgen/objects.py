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
from .materials import (COHERENT_ROUGHNESS_LIMIT_M, MaterialLibrary,
                        surface_roughness_m)
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
    #: False for a class fixed in place (a driven pile): it has no 3-D rest
    #: poses and keeps its upright flat stamp, whatever the resting-pose
    #: control says.
    rest_allowed: bool = True

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
    # The 2026-09-07 beach target itself: a 68 L NAFCO storage box (NA
    # Container Clear #75), 716 x 425 x 323 mm outer, lid on, flooded. Its
    # three dimensions are class knobs rather than a size range, because
    # the point of the class is to reproduce a *known* object: set them and
    # every instance is that box. `wall_mm` and `flooded` describe the
    # shell the facet layer reads (NC #17): the wall thickness sets its
    # reflection coefficient (the fluid-layer formula, `pp_shell` in
    # worldgen/materials), and an air-filled box is a near-total reflector
    # that casts a full shadow. It sat on the sand, so it is drawn unburied
    # by default (2026-09-28; the draw still consumes its RNG value, so no
    # other object moves).
    "plastic_box":     ObjectSpec("plastic_box", _mask_box, (0.70, 0.73),
                                  (0.41, 0.44), (0.31, 0.33), "pp_shell",
                                  (0.0, 0.0), False, "box",
                                  label="storage box, PP shell (68 L)",
                                  params_schema=(
                                      ("length_m", "Length [m]", "float",
                                       0.716, 0.05, 3.0,
                                       "Outer length of the box. The "
                                       "default is the 68 L NAFCO box used "
                                       "on the beach on 2026-09-07."),
                                      ("width_m", "Width [m]", "float",
                                       0.425, 0.05, 3.0,
                                       "Outer width of the box."),
                                      ("height_m", "Height [m]", "float",
                                       0.323, 0.02, 2.0,
                                       "Outer height, i.e. how proud of "
                                       "the seabed it stands unburied."),
                                      ("wall_mm", "Wall thickness [mm]",
                                       "float", 2.0, 0.2, 20.0,
                                       "Thickness of the shell. At 450 kHz "
                                       "a polypropylene wall reflects most "
                                       "strongly near a quarter wavelength "
                                       "(1.5 mm) and is transparent at a "
                                       "half (3.0 mm) -- see the pp_shell "
                                       "material."),
                                      ("flooded", "Flooded (water inside)",
                                       "bool", True, 0, 0,
                                       "A flooded shell lets sound through "
                                       "and returns its far wall too; an "
                                       "air-filled one is an near-total "
                                       "reflector and casts a full "
                                       "shadow."))),
    "tin_18l":         ObjectSpec("tin_18l", _mask_box, (0.22, 0.25),
                                  (0.22, 0.25), (0.32, 0.36), "tinplate",
                                  (0.0, 0.3), False, "box",
                                  label="18 L tin can (isshoka)"),
    "polytank":        ObjectSpec("polytank", _mask_box, (0.32, 0.36),
                                  (0.22, 0.26), (0.42, 0.48), "plastic",
                                  (0.0, 0.3), False, "box",
                                  label="20 L polytank"),
    # Harbour furniture rather than litter: a mooring pile is what a
    # pontoon is actually held by, and what stands beside it in every
    # marina (2026-09-18 review P6-6). Modelled as what it is -- an
    # upright cylinder driven into the bed, standing proud of it -- so a
    # pontoon line can be furnished by hand on the placement screen
    # instead of the wall model inventing a signature it has no field
    # anchor for.
    "mooring_pile":    ObjectSpec("mooring_pile", _mask_cylinder_upright,
                                  (0.30, 0.60), (0.30, 0.60), (1.20, 3.00),
                                  "concrete", (0.0, 0.0), True,
                                  "cylinder_upright",
                                  label="mooring pile (driven)",
                                  rest_allowed=False,
                                  params_schema=(
                                      ("diameter_m", "Diameter [m]", "float",
                                       0.4, 0.1, 1.5,
                                       "Outer diameter of the pile."),
                                      ("height_m", "Height above the bed [m]",
                                       "float", 2.0, 0.2, 6.0,
                                       "How far it stands proud of the "
                                       "seabed. A pile that breaks the "
                                       "surface is dry above it and returns "
                                       "nothing there (NC #13)."))),
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
    class has no 3-D solid or is fixed in place)."""
    if not spec.rest_allowed:
        return ()
    sol = spec.solid(typical_instance(spec))
    return stable_poses(sol) if sol is not None else ()


def rest_poses_text(spec: ObjectSpec) -> str:
    """One line for the Info dialog: how the class can lie on the seabed."""
    poses = rest_poses(spec)
    if not spec.rest_allowed:
        return "fixed upright (driven into the bed): no 3-D rest poses"
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
    it (``standing``, ``lying``, ``face:+x`` ...). Classes without a solid,
    or fixed in place (``rest_allowed=False``), are left untouched.
    """
    if not spec.rest_allowed:
        return
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
                   fill_override: float | None = None,
                   params: Mapping[str, Any] | None = None) -> PlacedObject:
    """One instance from a spec: sizes, burial, fill, reflectivity.

    ``params`` are the class's own knobs (``ObjectSpec.params_schema``).
    A class that pins its dimensions (``length_m`` / ``width_m`` /
    ``height_m``, as ``plastic_box`` does for a known field object)
    overrides the drawn sizes *after* the draw, so the random stream is
    untouched and no other object moves.
    """
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
    extra = {k: v for k, v in dict(params or {}).items()
             if k != "fill_fraction"}
    if extra.get("diameter_m") is not None and float(extra["diameter_m"]) > 0.0:
        # A round footprint has one size, not two (mooring_pile).
        length = width = float(extra["diameter_m"])
    for key, val in (("length_m", "length"), ("width_m", "width"),
                     ("height_m", "height")):
        if extra.get(key) is not None and float(extra[key]) > 0.0:
            if val == "length":
                length = float(extra[key])
                if spec.round_footprint:
                    width = length
            elif val == "width":
                width = float(extra[key])
            else:
                height = float(extra[key])
    mat = materials.get(material_override or spec.material)
    refl = float(np.clip(
        mat.reflectivity * (1.0 + rng.uniform(-1, 1) * cfg.reflectivity_jitter),
        0.05, 1.0))
    return PlacedObject(
        object_id=object_id, type=spec.type, x=float(x), y=float(y),
        yaw=float(yaw), length=length, width=width, proud_height=height,
        burial=burial, reflectivity=refl, material=mat.name,
        fill_fraction=fill, params=extra,
        transmission=_instance_transmission(mat.transmission, extra),
        roughness_m=float(max(surface_roughness_m(mat), 0.0)))


def _instance_transmission(material_t: float,
                           params: Mapping[str, Any] | None) -> float:
    """The material's two-way transmission, unless the instance's own
    construction says otherwise: an air-filled shell (``flooded: false``,
    the ``plastic_box`` knob) is a near-total reflector and lets nothing
    through."""
    if (params or {}).get("flooded") is False:
        return 0.0
    return float(np.clip(material_t, 0.0, 1.0))


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
                fill_override=float(fill_p) if fill_p is not None else None,
                params=c.get("params"))
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
        if isinstance(item.get("params"), Mapping):
            # The class knobs describe the instance's construction (the
            # plastic_box's wall and whether it is flooded), which the facet
            # layer reads (NC #17): they travel with the pinned entry.
            inst.params = {k: v for k, v in item["params"].items()
                           if k != "fill_fraction"}
        if "material" in item:
            inst.material = str(item["material"])
        # The shell and the surface follow the material (a pinned entry that
        # names one gets that material's transmission and roughness, not the
        # class default's -- roughness decides facet vs raster, NC #17) and
        # the instance's construction. Reflectivity does not follow: it is
        # only drawn, or pinned verbatim (see the configuration guide).
        _mat = materials.get(inst.material)
        inst.transmission = _instance_transmission(_mat.transmission,
                                                   inst.params)
        inst.roughness_m = float(max(surface_roughness_m(_mat), 0.0))
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
                     height: np.ndarray, reflectivity: np.ndarray,
                     transmission: np.ndarray | None = None,
                     roughness: np.ndarray | None = None,
                     facet_bump: np.ndarray | None = None) -> None:
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
    _stamp_surface(o, (slice(j0, j1), slice(i0, i1)), bump, prof,
                   reflectivity, transmission, roughness, facet_bump)
    o.rest_z = float(centre_z)
    o.roll, o.pitch, _ = rpy_from_rotation(R_world)   # includes the bank tilt


def _grow1(mask: np.ndarray) -> np.ndarray:
    """``mask`` grown by one cell (8-connected), within its own window."""
    g = mask.copy()
    g[1:, :] |= mask[:-1, :]
    g[:-1, :] |= mask[1:, :]
    g2 = g.copy()
    g2[:, 1:] |= g[:, :-1]
    g2[:, :-1] |= g[:, 1:]
    return g2


def _stamp_surface(o: PlacedObject, win: tuple, bump: np.ndarray,
                   prof: np.ndarray, reflectivity: np.ndarray,
                   transmission: np.ndarray | None,
                   roughness: np.ndarray | None,
                   facet_bump: np.ndarray | None) -> None:
    """Everything but the height: what the object's footprint carries.

    A **facet object** (NC #17, :func:`is_facet_object`) scatters from its
    exact geometry in the renderer's facet layer, so the raster keeps only
    what occludes: its bump goes into ``facet_bump`` as well as the height,
    the seabed beneath keeps its own reflectivity (it is what a shell lets
    the beam see), and no roughness is stamped (there is no raster surface
    of the object left to be smooth). A shell's transmission is stamped over
    the footprint grown by one cell: the bilinear up-ramp in front of the
    wall sets the horizon too, and left at 0 it made a dark seam there.
    Every other object is stamped exactly as it always was.
    """
    facet = facet_bump is not None and is_facet_object(o)
    if facet:
        facet_bump[win] += bump
    else:
        w = np.clip(prof, 0.0, 1.0) * (1.0 - 0.8 * o.burial)
        reflectivity[win] = (1.0 - w) * reflectivity[win] + w * o.reflectivity
    if transmission is not None and o.transmission > 0.0:
        t = o.transmission * (1.0 - o.burial)
        foot = prof > 0.0
        if facet:
            foot = _grow1(foot)
        np.maximum(transmission[win], np.where(foot, t, 0.0),
                   out=transmission[win])
    # A buried shell is covered by sediment, which is rough: only what is
    # still proud keeps the smooth surface that makes the echo coherent.
    _rq = object_roughness_m(o)
    if (not facet and roughness is not None and o.burial < 1.0
            and 0.0 < _rq < COHERENT_ROUGHNESS_LIMIT_M):
        np.maximum(roughness[win], np.where(prof > 0.0, _rq, 0.0),
                   out=roughness[win])


def transmission_raster(objects: list[PlacedObject],
                        grid: GridSpec) -> np.ndarray | None:
    """A zero raster to stamp transmissions into, or ``None`` when nothing
    in this scene transmits -- which keeps the array out of ``scene.npz``
    and every existing world byte-identical."""
    if not any(getattr(o, "transmission", 0.0) > 0.0 for o in objects):
        return None
    return np.zeros((grid.ny, grid.nx), dtype=np.float64)


#: Default material library, for objects that carry a material name but no
#: resolved surface roughness -- a hand-built ``PlacedObject``, or one loaded
#: from a manifest written before ``roughness_m`` existed.
_DEFAULT_MATERIALS = MaterialLibrary()


def object_roughness_m(o: PlacedObject) -> float:
    """This object's RMS surface height [m].

    Its own resolved value when it has one, else its material's. Resolving
    it here rather than only at placement means a manifest written before
    ``roughness_m`` existed, and an instance built by hand in a test, both
    behave like the material they name instead of silently falling back to
    "rough".
    """
    r = float(getattr(o, "roughness_m", 0.0))
    if r > 0.0:
        return r
    try:
        return surface_roughness_m(_DEFAULT_MATERIALS.get(o.material))
    except Exception:
        return 0.0


#: The solid primitives the facet layer tessellates (``solids.tessellate``).
FACET_SOLID_KINDS = frozenset({"box", "cylinder", "capsule", "cone",
                               "annulus"})


def is_facet_object(o: PlacedObject) -> bool:
    """True when the renderer's facet layer scatters this object (NC #17).

    The rule, from material physics and never from the object's name: a
    **smooth** material (RMS surface height under
    ``COHERENT_ROUGHNESS_LIMIT_M`` -- a mirror at 450 kHz, where Lambert's
    law does not hold) on a **solid primitive** the layer can tessellate,
    proud of the seabed. Everything else -- rough materials (concrete, rope,
    tyre tread, generic), aggregates of many small members (the lattice
    classes, rope, chain, anchor, debris, coils) and fully buried objects --
    stays on the raster's Lambert path, which is the right physics for a
    wavelength-rough scatterer. Fired brick (1 mm) is on the smooth side of
    the limit: its coherent fraction is 6e-7 at normal incidence but 0.5 at
    77 deg off the normal, so it is a facet object (smoke [14]).
    """
    spec = CATALOG.get(o.type)
    if spec is None or spec.fill_fraction_range is not None:
        return False
    if not o.rested and o.effective_height <= 0.005:
        return False
    rq = object_roughness_m(o)
    if not 0.0 < rq < COHERENT_ROUGHNESS_LIMIT_M:
        return False
    sol = spec.solid(o)
    return sol is not None and sol.kind in FACET_SOLID_KINDS


def needs_facet_raster(objects: list[PlacedObject]) -> bool:
    """True when some object is rendered by the facet layer, i.e. the scene
    needs a ``facet_bump`` raster to know where its occluders stand."""
    return any(is_facet_object(o) for o in objects)


def facet_bump_raster(objects: list[PlacedObject],
                      grid: GridSpec) -> np.ndarray | None:
    """A zero raster to stamp facet objects' bumps into, or ``None`` when
    the scene has none (the array then stays out of ``scene.npz``)."""
    if not needs_facet_raster(objects):
        return None
    return np.zeros((grid.ny, grid.nx), dtype=np.float64)


def needs_roughness_raster(objects: list[PlacedObject]) -> bool:
    """True when some object is smooth at 450 kHz *and* stays on the raster
    path, i.e. the scene needs a roughness raster for the renderer's
    coherent split. Facet objects scatter from their own geometry."""
    return any(0.0 < object_roughness_m(o) < COHERENT_ROUGHNESS_LIMIT_M
               and not is_facet_object(o) for o in objects)


def roughness_raster(objects: list[PlacedObject],
                     grid: GridSpec) -> np.ndarray | None:
    """A zero raster to stamp object surface roughnesses into, or ``None``
    when every object in the scene is acoustically rough at 450 kHz.

    0 means "nothing smooth here", so the renderer falls back to the scene's
    rough default and the coherent split is exactly a no-op -- which keeps
    the array out of ``scene.npz`` and every pre-2026-09-19 world
    byte-identical."""
    if not needs_roughness_raster(objects):
        return None
    return np.zeros((grid.ny, grid.nx), dtype=np.float64)


def rasterize_objects(objects: list[PlacedObject], grid: GridSpec,
                      height: np.ndarray, reflectivity: np.ndarray,
                      transmission: np.ndarray | None = None,
                      roughness: np.ndarray | None = None,
                      facet_bump: np.ndarray | None = None) -> None:
    """Stamp each object into the acoustic rasters *in place*.

    ``height`` gains a bump of ``effective_height * profile`` and
    ``reflectivity`` is blended toward the object reflectivity inside the
    footprint. Buried objects contribute a residual low bump plus a weak
    reflectivity contrast (sediment-covered target).

    ``transmission``, when given, receives each object's own two-way
    transmission over its footprint -- what the renderer lets through into
    the shadow behind it. It is a separate raster because it is a property
    of the *occluder*, read where the horizon cull finds the crest, not of
    the ground it hides. Buried objects transmit nothing: what occludes is
    then the sediment mound, not the shell.
    """
    res = grid.resolution
    for o in objects:
        spec = CATALOG[o.type]
        if o.rested and spec.solid(o) is not None:
            _rasterize_solid(o, spec, grid, height, reflectivity,
                             transmission, roughness, facet_bump)
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
        bump = h_eff * prof
        height[j0:j1, i0:i1] += bump
        # Reflectivity blend (weakened with burial), transmission, roughness
        # -- or, for a facet object, only its occluder (``_stamp_surface``).
        _stamp_surface(o, (slice(j0, j1), slice(i0, i1)), bump, prof,
                       reflectivity, transmission, roughness, facet_bump)


# --------------------------------------------------------------------------
# Hand placement (the world builder's Screen 5)
# --------------------------------------------------------------------------
#: Every field ``_make_instance`` draws from the shared RNG stream. An
#: explicit entry that pins all of them rebuilds its instance exactly,
#: wherever it later sits in the list.
PINNED_FIELDS = ("length", "width", "proud_height", "burial",
                 "fill_fraction", "reflectivity")

#: Class knobs that describe how an instance is BUILT and that the renderer
#: reads (NC #17), so a pinned entry carries them. The size knobs
#: (``length_m`` ...) are already pinned as the sizes they produced.
_CONSTRUCTION_PARAMS = ("wall_mm", "flooded")


def placed_to_explicit(o: PlacedObject) -> dict[str, Any]:
    """A placed instance as an ``objects.explicit`` entry that rebuilds it.

    Used to **pin** a random field: every value the engine would otherwise
    re-draw is written out, so the entry no longer depends on its position
    in the RNG stream and a world rebuilt from the pinned list is
    object-for-object identical to the scattered one. ``object_id`` is
    positional and ``rest_z`` is recomputed by the rasteriser from the
    terrain, so neither is stored. ``object_id`` also seeds the masks of
    rope, chain, debris, the coils and the lattice classes: deleting or
    reordering a pinned object re-draws the wiggle, outline or skeleton of
    every later such object (never its size, burial or pose).

    ``roll_deg`` / ``pitch_deg`` are written **only** for an object that
    actually rested: writing them at all sends the instance down the 3-D
    solid path (``place_objects_v2`` sets ``rest`` away from
    ``canonical``), which would change the imagery of an object that was
    stamped flat.
    """
    item: dict[str, Any] = {"type": str(o.type), "x": float(o.x),
                            "y": float(o.y), "yaw": float(o.yaw)}
    for key in PINNED_FIELDS:
        item[key] = float(getattr(o, key))
    item["material"] = str(o.material)
    knobs = {k: v for k, v in dict(getattr(o, "params", None) or {}).items()
             if k in _CONSTRUCTION_PARAMS}
    if knobs:
        item["params"] = knobs
    if o.rested:
        item["roll_deg"] = float(np.degrees(o.roll))
        item["pitch_deg"] = float(np.degrees(o.pitch))
        item["rest"] = str(o.rest)
    return item


def make_explicit_instances(type_name: str, x: float, y: float,
                            rng: np.random.Generator, *,
                            count: int = 1, sigma_m: float = 0.0,
                            size: Mapping[str, Any] | None = None,
                            burial: Mapping[str, Any] | None = None,
                            orientation: Mapping[str, Any] | None = None,
                            material: str | None = None,
                            params: Mapping[str, Any] | None = None,
                            materials: MaterialLibrary | None = None,
                            path_xy=None) -> list[dict[str, Any]]:
    """``count`` hand-placed instances of ``type_name`` about ``(x, y)``.

    The knobs are the scatter screen's, with one substitution: the click
    *is* the cluster centre, so ``count`` instances are drawn from a
    Gaussian of width ``sigma_m`` about it (``sigma_m = 0`` stacks them on
    the point). ``size`` and ``burial`` are the same distribution specs,
    ``orientation`` the same block (``random`` | ``fixed`` |
    ``path_relative``, plus the 3-D ``rest`` sub-block), ``params`` carries
    the class extras (``fill_fraction`` for the open-frame classes).

    Every drawn value is pinned in the returned entries
    (:func:`placed_to_explicit`), so what the operator previewed at the
    click is exactly what the world will contain.
    """
    if type_name not in CATALOG:
        raise KeyError(f"Unknown object class '{type_name}' "
                       f"(available: {sorted(CATALOG)})")
    spec = CATALOG[type_name]
    materials = materials if materials is not None else MaterialLibrary()
    cfg = ObjectFieldConfig()
    orient = normalise_orientation(orientation)
    pts = (np.asarray(path_xy, dtype=np.float64).reshape(-1, 2)
           if path_xy is not None and len(path_xy) else None)
    if orient["mode"] == "path_relative" and pts is None:
        raise ValueError(f"class '{type_name}': orientation mode "
                         "path_relative needs the survey path")
    fill_p = (params or {}).get("fill_fraction")

    out: list[dict[str, Any]] = []
    for k in range(max(0, int(count))):
        if k == 0 or sigma_m <= 0.0:
            px, py = float(x), float(y)
        else:
            px, py = (float(x) + float(rng.normal(0.0, sigma_m)),
                      float(y) + float(rng.normal(0.0, sigma_m)))
        track = (nearest_segment_heading(pts, px, py)
                 if orient["mode"] == "path_relative" else None)
        yaw = _draw_yaw(orient, rng, track)
        inst = _make_instance(
            spec, k, px, py, yaw, cfg, materials, rng,
            size_dist=size, burial_dist=burial, material_override=material,
            fill_override=float(fill_p) if fill_p is not None else None,
            params=params)
        if orient["rest"]["enabled"]:
            draw_rest(spec, inst, orient["rest"]["randomness"])
        out.append(placed_to_explicit(inst))
    return out
