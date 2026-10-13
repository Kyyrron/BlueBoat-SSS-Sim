"""Material layers -- the seabed *texture* map: which sediment covers
which cell, and a multiplicative reflectivity gain.

* ``material_region``   one material under the selection, with a hard or
                        a dithered (patchy) edge.
* ``material_gradient`` two materials grading into each other along an
                        axis, a radial or a distance from a line -- the
                        inward fining of a basin, the cross-shore sorting
                        of a beach (dithered, so the boundary is patchy as
                        real sand-mud transitions are).
* ``material_patches``  a patch mosaic with a characteristic patch length
                        and a composition (the ``patches`` layout as a
                        layer).
* ``material_band``     a belt along a line: the rubble apron along a quay.
* ``reflectivity_gain`` a dB adjustment of the rendered reflectivity under
                        the selection (fluid mud, shell hash) -- no relief.

Backscatter orders mud < sand < gravel < rock over roughly 10-15 dB at
hundreds of kHz (Jackson & Richardson 2007; APL-UW 1994); note the
renderer's own material spread is only a few dB today (TODO.md).
"""

from __future__ import annotations

import numpy as np

from ...materials import MaterialLibrary
from ...noise import fbm
from .. import geometry as geo
from ..paint import smoothstep
from . import (MATERIAL_CHOICE, LayerContext, LayerOutput, LayerSpec,
               register)

_NAMES = MaterialLibrary().seabed_names

_REGION = (
    ("material", "Material", MATERIAL_CHOICE, "mud", 0, 0,
     "Seabed material laid under the selection."),
    ("edge", "Edge", "choice:dithered|hard", "dithered", 0, 0,
     "How the feathered edge of the selection is rendered: a patchy "
     "sand-mud style transition, or a clean line."),
)


def region_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    return LayerOutput(material=(np.zeros((grid.ny, grid.nx), dtype=np.int16),
                                 [p["material"]]),
                       material_edge=p["edge"])


register(LayerSpec(
    kind="material_region", fn=region_fn, label="Material region",
    family="material", schema=_REGION, geometry=(), channels=("material",),
    reference="[33] Jackson & Richardson 2007; [34] APL-UW 1994; "
              "[35] Lyons & Abraham 1999",
    consequence="A patch of different tone and speckle; mud dark and smooth, "
                "gravel bright and grainy.",
    blend="normal"))


_GRADIENT = (
    ("material_from", "Material at the start", MATERIAL_CHOICE, "sand", 0, 0,
     "Seabed material at the line, the axis start or the centre."),
    ("material_to", "Material at the end", MATERIAL_CHOICE, "mud", 0, 0,
     "Seabed material at the end of the gradient and beyond it."),
    ("length_m", "Transition length [m]", "float", 40.0, 0.5, 5000.0,
     "Distance over which the mix changes from one material to the other."),
    ("profile", "Profile", "choice:linear|smoothstep", "smoothstep", 0, 0,
     "How the mix changes with distance."),
    ("side", "Side of the line", "choice:both|left|right", "both", 0, 0,
     "Which side of a drawn line or axis the gradient applies to."),
)


def gradient_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    side = None
    if g["type"] == "axis":
        along, side, length = geo.axis_coords(grid, g["a"], g["b"])
        u = along / max(length, 1e-6)
    elif g["type"] == "radial":
        u = geo.radial_distance(grid, g["centre"]) / max(g["radius_m"], 1e-6)
    else:
        xmin, ymin, xmax, ymax = grid.extent
        reach = float(np.hypot(xmax - xmin, ymax - ymin)) + 1.0
        fr = geo.polyline_frame(g["points"], grid, reach)
        u, side = fr.dist / max(p["length_m"], 1e-6), fr.n
    u = np.clip(u, 0.0, 1.0)
    if p["profile"] == "smoothstep":
        u = smoothstep(u)
    pick = rng.uniform(size=u.shape) < u
    ids = np.where(pick, 1, 0).astype(np.int16)
    default = None
    if p["side"] != "both" and side is not None:
        default = ((side >= 0.0) if p["side"] == "left"
                   else (side <= 0.0)).astype(np.float64)
    return LayerOutput(material=(ids, [p["material_from"], p["material_to"]]),
                       material_edge="hard", default_mask=default)


register(LayerSpec(
    kind="material_gradient", fn=gradient_fn, label="Material gradient",
    family="material", schema=_GRADIENT,
    geometry=("axis", "radial", "polyline"), channels=("material",),
    reference="[30] Winterwerp 2005; [13] Leopold & Wolman 1957; "
              "[5] Wright & Short 1984; [35] Lyons & Abraham 1999",
    consequence="Tone and speckle change gradually and patchily along the "
                "gradient rather than at a line.",
    blend="normal"))


_PATCHES = (("patch_m", "Patch length [m]", "float", 8.0, 0.5, 200.0,
             "Characteristic size of one patch of the mosaic."),) + tuple(
    (f"share_{n}", f"Share of {n}", "float",
     {"sand": 0.5, "mud": 0.2, "gravel": 0.15, "rocks": 0.1, "seagrass": 0.05}[n],
     0.0, 1.0, f"Fraction of the area covered by {n} patches.")
    for n in _NAMES)


def patches_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    names = [n for n in _NAMES if p[f"share_{n}"] > 0.0]
    if not names:
        raise ValueError("material_patches: every share is zero")
    weights = np.array([p[f"share_{n}"] for n in names])
    weights /= weights.sum()
    xmin, ymin, xmax, ymax = grid.extent
    cells = int(np.clip(max(xmax - xmin, ymax - ymin) / max(p["patch_m"], 1e-6),
                        2, 512))
    f = fbm((grid.ny, grid.nx), cells, 3, rng)
    order = f.reshape(-1).argsort().argsort().astype(np.float64)
    u = (order / order.size).reshape(f.shape)
    ids = np.searchsorted(np.cumsum(weights), u, side="right")
    ids = np.minimum(ids, len(names) - 1).astype(np.int16)
    return LayerOutput(material=(ids, names), material_edge="hard")


register(LayerSpec(
    kind="material_patches", fn=patches_fn, label="Material patches",
    family="material", schema=_PATCHES, geometry=(), channels=("material",),
    reference="[12] Lefebvre et al. 2009; [35] Lyons & Abraham 1999",
    consequence="A mosaic of tones at the patch scale: seagrass and mud dark, "
                "sand and gravel bright.",
    blend="normal"))


_BAND = (
    ("material", "Material", MATERIAL_CHOICE, "rocks", 0, 0,
     "Seabed material of the belt (rubble along a quay, gravel along a bank)."),
    ("width_m", "Belt width [m]", "float", 4.0, 0.1, 100.0,
     "Full width of the belt centred on the line."),
    ("feather_m", "Edge feather [m]", "float", 1.0, 0.0, 20.0,
     "Distance over which the belt frays into the surrounding bed."),
    ("side", "Side of the line", "choice:both|left|right", "both", 0, 0,
     "Which side of the line the belt lies on."),
)


def band_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    half = 0.5 * p["width_m"]
    fr = geo.polyline_frame(g["points"], grid, half + p["feather_m"] + grid.resolution)
    w = geo.band_weight(fr.dist, half, p["feather_m"])
    w[~fr.valid] = 0.0
    if p["side"] != "both":
        keep = (fr.n >= 0.0) if p["side"] == "left" else (fr.n <= 0.0)
        w = np.where(keep, w, 0.0)
    return LayerOutput(material=(np.zeros((grid.ny, grid.nx), dtype=np.int16),
                                 [p["material"]]),
                       material_edge="dithered", default_mask=w)


register(LayerSpec(
    kind="material_band", fn=band_fn, label="Material belt along a line",
    family="material", schema=_BAND, geometry=("polyline",),
    channels=("material",),
    reference="[27] PIANC 2015; [23] Canal & River Trust",
    consequence="A bright grainy strip hugging the quay or the bank.",
    blend="normal"))


_GAIN = (("gain_db", "Reflectivity gain [dB]", "float", 3.0, -20.0, 20.0,
          "How much brighter (positive) or darker (negative) the seabed "
          "renders under the selection, without any relief."),)


def gain_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    return LayerOutput(gain_db=float(p["gain_db"]))


register(LayerSpec(
    kind="reflectivity_gain", fn=gain_fn, label="Reflectivity gain",
    family="material", schema=_GAIN, geometry=(), channels=("gain",),
    reference="[33] Jackson & Richardson 2007; [36] Lurton 2010",
    consequence="A brighter or darker patch with the same texture.",
    blend="normal"))
