"""Relief -- micro-topography the sonar sees as texture, shadow and
lineation, written into the height raster so the renderer casts real
shadows from it.

* ``ripples``  wave ripples (Wiberg & Harris 1994; Clifton & Dingler 1984).
* ``dunes``    current dunes with Ashley's height-wavelength law (Ashley
               1990; van Rijn 1984).
* ``furrows``  parallel dredge or trawl marks (Boyd et al. 2005).
* ``roughness`` broadband fBm relief.
* ``outcrops`` seeded rock heads.
* ``pits``     biogenic pits or mounds.

Each layer draws from its own RNG stream, so editing one never moves
another's texture.
"""

from __future__ import annotations

import numpy as np

from ...noise import fbm, value_noise
from .. import geometry as geo
from . import (MATERIAL_CHOICE, LayerContext, LayerOutput, LayerSpec,
               register)


def _bedform(grid, p, rng, height):
    xmin, ymin, xmax, ymax = grid.extent
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    xx, yy = geo.grid_coords(grid)
    th = np.radians(p["heading_deg"])
    across = -np.sin(th) * (xx - cx) + np.cos(th) * (yy - cy)
    lam = max(p["wavelength_m"], 1e-3)
    shape = (grid.ny, grid.nx)
    span = max(xmax - xmin, ymax - ymin)
    cells = int(np.clip(span / (5.0 * lam), 2, 64))
    phase = 2.0 * np.pi * across / lam
    if p["irregularity"] > 0.0:
        phase = phase + p["irregularity"] * 2.0 * np.pi * fbm(shape, cells, 2, rng)
    a = float(np.clip(p["asymmetry"], 0.0, 0.95))
    wave = np.sin(phase + a * np.sin(phase))
    mod = 1.0 + 0.5 * p["irregularity"] * value_noise(shape, max(cells, 2), rng)
    return 0.5 * height * mod * wave


_RIPPLES = (
    ("wavelength_m", "Ripple wavelength [m]", "float", 0.5, 0.2, 3.0,
     "Crest-to-crest distance: about 0.6 times the wave orbital diameter, or "
     "500 grain diameters for fine sand; under 0.2 m the 0.1 m world grid "
     "aliases it into moire instead of ripples."),
    ("height_m", "Ripple height [m]", "float", 0.06, 0.0, 0.5,
     "Crest-to-trough height, typically a tenth to a sixth of the wavelength."),
    ("heading_deg", "Crest heading [deg]", "float", 0.0, -180.0, 180.0,
     "Compass heading the crests run along (parallel to the shoreline for "
     "wave ripples)."),
    ("asymmetry", "Asymmetry", "float", 0.0, 0.0, 1.0,
     "0 makes symmetric ripples, 1 makes a sawtooth with a steep lee side."),
    ("irregularity", "Irregularity", "float", 0.4, 0.0, 1.0,
     "0 makes perfectly straight crests; 1 makes them wander and break up."),
)


def ripples_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    return LayerOutput(delta=_bedform(grid, p, rng, p["height_m"]))


register(LayerSpec(
    kind="ripples", fn=ripples_fn, label="Wave ripples", family="relief",
    schema=_RIPPLES, geometry=(), channels=("height",),
    reference="[9] Wiberg & Harris 1994; [10] Clifton & Dingler 1984",
    consequence="Fine bright-dark banding across the track where the crests "
                "are crossed at an angle; none along the track.",
    blend="add"))


_DUNES = (
    ("wavelength_m", "Dune wavelength [m]", "float", 20.0, 0.6, 500.0,
     "Crest-to-crest distance: about seven times the water depth for "
     "current dunes."),
    ("height_m", "Dune height [m] (0 = from wavelength)", "float", 0.0, 0.0,
     10.0, "Crest-to-trough height; 0 uses Ashley's law, 0.068 times the "
     "wavelength to the power 0.81."),
    ("heading_deg", "Crest heading [deg]", "float", 90.0, -180.0, 180.0,
     "Compass heading the crests run along (normal to the flow)."),
    ("asymmetry", "Asymmetry", "float", 0.5, 0.0, 1.0,
     "0 makes symmetric dunes, 1 makes a sawtooth with a steep lee side."),
    ("irregularity", "Irregularity", "float", 0.5, 0.0, 1.0,
     "0 makes perfectly straight crests; 1 makes them wander and break up."),
)


def dunes_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    h = p["height_m"] if p["height_m"] > 0.0 else 0.0677 * p["wavelength_m"] ** 0.8098
    return LayerOutput(delta=_bedform(grid, p, rng, h))


register(LayerSpec(
    kind="dunes", fn=dunes_fn, label="Current dunes", family="relief",
    schema=_DUNES, geometry=(), channels=("height",),
    reference="[19] Ashley 1990; [20] van Rijn 1984",
    consequence="Broad bright stoss faces and shadowed lee faces marching "
                "across the image.",
    blend="add"))


_FURROWS = (
    ("spacing_m", "Furrow spacing [m]", "float", 3.0, 0.3, 50.0,
     "Distance between neighbouring furrows."),
    ("depth_m", "Furrow depth [m]", "float", 0.2, 0.0, 2.0,
     "Depth of each furrow below the bed."),
    ("width_m", "Furrow width [m]", "float", 1.0, 0.1, 10.0,
     "Width of each furrow."),
    ("heading_deg", "Furrow heading [deg]", "float", 0.0, -180.0, 180.0,
     "Compass heading the furrows run along (the dredge or trawl run)."),
    ("jitter", "Wander", "float", 0.2, 0.0, 1.0,
     "0 keeps the furrows perfectly straight and evenly spaced; 1 lets them "
     "wander."),
)


def furrows_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    shape = (grid.ny, grid.nx)
    phase = 0.0
    d = geo.line_family_distance(grid, p["heading_deg"], p["spacing_m"], phase)
    if p["jitter"] > 0.0:
        shift = 0.25 * p["jitter"] * p["spacing_m"] * fbm(shape, 6, 2, rng)
        xmin, ymin, xmax, ymax = grid.extent
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        xx, yy = geo.grid_coords(grid)
        th = np.radians(p["heading_deg"])
        across = -np.sin(th) * (xx - cx) + np.cos(th) * (yy - cy) + shift
        sp = max(p["spacing_m"], 1e-6)
        d = np.abs((across + 0.5 * sp) % sp - 0.5 * sp)
    delta = -p["depth_m"] * np.exp(-(d / max(0.5 * p["width_m"], 1e-6)) ** 2)
    return LayerOutput(delta=delta)


register(LayerSpec(
    kind="furrows", fn=furrows_fn, label="Dredge / trawl furrows",
    family="relief", schema=_FURROWS, geometry=(), channels=("height",),
    reference="[31] Boyd et al. 2005; [32] Broad et al. 2020",
    consequence="Parallel dark lineations with thin bright edges, persisting "
                "for years after the dredger left.",
    blend="add"))


_ROUGH = (
    ("amplitude_m", "Relief amplitude [m]", "float", 0.05, 0.0, 2.0,
     "Height of the random relief."),
    ("cells", "Cells", "int", 12, 2, 64,
     "Number of relief blobs across the area: more means finer relief."),
    ("octaves", "Octaves", "int", 4, 1, 8,
     "How many finer scales are layered on the blobs."),
)


def roughness_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    return LayerOutput(delta=p["amplitude_m"]
                       * fbm((grid.ny, grid.nx), p["cells"], p["octaves"], rng))


register(LayerSpec(
    kind="roughness", fn=roughness_fn, label="Random roughness",
    family="relief", schema=_ROUGH, geometry=(), channels=("height",),
    reference="[35] Lyons & Abraham 1999; [33] Jackson & Richardson 2007",
    consequence="Mottled, unstructured texture with soft shadows.",
    blend="add"))


_OUTCROPS = (
    ("count", "Number of outcrops", "int", 12, 1, 500,
     "How many rock heads are scattered over the area (or the mask)."),
    ("height_m", "Typical height [m]", "float", 0.8, 0.05, 20.0,
     "Height of a typical outcrop above the bed."),
    ("radius_m", "Typical radius [m]", "float", 3.0, 0.2, 100.0,
     "Footprint radius of a typical outcrop."),
    ("jitter", "Size spread", "float", 0.4, 0.0, 0.9,
     "Relative spread of the individual sizes and heights."),
    ("material_enabled", "Paint as rock", "bool", True, 0, 1,
     "Gives the outcrops the rocks material."),
    ("material", "Outcrop material", MATERIAL_CHOICE, "rocks", 0, 0,
     "Seabed material of the outcrop tops."),
)


def outcrops_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    xmin, ymin, xmax, ymax = grid.extent
    if ctx.mask is not None and (ctx.mask > 0.5).any():
        jj, ii = np.nonzero(ctx.mask > 0.5)
        res = grid.resolution
        xmin, xmax = grid.origin_x + ii.min() * res, grid.origin_x + ii.max() * res
        ymin, ymax = grid.origin_y + jj.min() * res, grid.origin_y + jj.max() * res
    xx, yy = geo.grid_coords(grid)
    delta = np.zeros((grid.ny, grid.nx))
    for _ in range(int(p["count"])):
        cx = rng.uniform(xmin, xmax)
        cy = rng.uniform(ymin, ymax)
        r = p["radius_m"] * (1.0 + p["jitter"] * rng.uniform(-1.0, 1.0))
        h = p["height_m"] * (1.0 + p["jitter"] * rng.uniform(-1.0, 1.0))
        r2 = ((xx - cx) ** 2 + (yy - cy) ** 2) / max(r * r, 1e-9)
        delta = np.maximum(delta, h * np.exp(-(r2 ** 1.5)))
    out = LayerOutput(delta=delta)
    if p["material_enabled"]:
        out.material = (np.where(delta > 0.3 * p["height_m"], 0, -1).astype(np.int16),
                        [p["material"]])
    return out


register(LayerSpec(
    kind="outcrops", fn=outcrops_fn, label="Rock outcrops", family="relief",
    schema=_OUTCROPS, geometry=(), channels=("height", "material"),
    reference="[37] Blondel 2009; [33] Jackson & Richardson 2007",
    consequence="Bright rounded heads each trailing a hard shadow.",
    blend="add"))


_PITS = (
    ("density_per_100m2", "Pits per 100 m²", "float", 2.0, 0.01, 100.0,
     "How many pits (or mounds) per 100 square metres."),
    ("depth_m", "Pit depth [m] (negative = mound)", "float", 0.15, -2.0, 2.0,
     "Depth of each pit below the bed; a negative value makes mounds."),
    ("radius_m", "Pit radius [m]", "float", 0.5, 0.05, 10.0,
     "Radius of each pit or mound."),
)


def pits_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    xmin, ymin, xmax, ymax = grid.extent
    area = max((xmax - xmin) * (ymax - ymin), 1.0)
    n = int(round(p["density_per_100m2"] * area / 100.0))
    xx, yy = geo.grid_coords(grid)
    delta = np.zeros((grid.ny, grid.nx))
    r2n = max(p["radius_m"] ** 2, 1e-9)
    for _ in range(n):
        cx, cy = rng.uniform(xmin, xmax), rng.uniform(ymin, ymax)
        win = geo.cell_window(grid, cx - 3 * p["radius_m"], cy - 3 * p["radius_m"],
                              cx + 3 * p["radius_m"], cy + 3 * p["radius_m"])
        if win is None:
            continue
        r2 = ((xx[win] - cx) ** 2 + (yy[win] - cy) ** 2) / r2n
        delta[win] -= p["depth_m"] * np.exp(-r2)
    return LayerOutput(delta=delta)


register(LayerSpec(
    kind="pits", fn=pits_fn, label="Biogenic pits / mounds", family="relief",
    schema=_PITS, geometry=(), channels=("height",),
    reference="[33] Jackson & Richardson 2007; [37] Blondel 2009",
    consequence="A sprinkling of small dark dots with bright rims.",
    blend="add"))
