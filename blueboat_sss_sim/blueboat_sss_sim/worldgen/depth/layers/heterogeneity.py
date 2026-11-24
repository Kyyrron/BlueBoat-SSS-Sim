"""Seabed heterogeneity layers (completion round, session 02, 2026-11-15).

What a real harbour floor has between its landforms and its sonar wavelength, at decimetre-to-metre scale. A
uniform sediment gives a picture that is too clean; these layers add what the field recordings show (a
reflectivity texture, patches, bright points), every one as *sediment classes and geometry*, never as a grey
level (NC #17 for the seabed):

* ``sediment_field``  patches of a finer and of a coarser sediment in the reference one: a seeded Gaussian
                      random field with a stated correlation length, cut at two quantiles;
* ``microrelief``     zero-mean relief with a stated RMS height and correlation length (the slope it gives the
                      terrain mesh changes the local grazing angle, hence the level);
* ``bioturbation``    feeding pits and the large mounds of burrowing animals;
* ``drag_marks``      anchor, chain and keel furrows: a few long troughs;
* ``small_litter``    sub-threshold hard items (bottles, cans): sparse cells of a hard class with their relief;
  ``cobbles``         the same mechanism for stones.

The sediment classes are measured sites of Jackson & Richardson (2007), Table 13.1
(``sonar/raycast/seabed_ssa.SEDIMENTS``); sizes and densities of the relief are sourced in
``docs/seabed_model.md`` section 9. ROS-free, Qt-free.
"""

from __future__ import annotations

import numpy as np

from .. import geometry as geo
from . import MATERIAL_CHOICE, LayerContext, LayerOutput, LayerSpec, register


# ----------------------------------------------------------------------------- random fields
def gaussian_field(ny: int, nx: int, resolution: float, corr_len_m: float,
                   rng: np.random.Generator) -> np.ndarray:
    """A zero-mean, unit-variance Gaussian random field on an (ny, nx) grid whose correlation is
    ``exp(-r^2 / corr_len^2)`` (so ``corr_len_m`` is the distance at which it falls to 1/e).

    White noise filtered in the Fourier domain by ``exp(-k^2 L^2 / 8)`` (the square root of the spectrum of
    that correlation), on a grid padded by three correlation lengths so that the periodic wrap of the
    transform stays outside. Deterministic for a given generator state, grid and length."""
    L = max(float(corr_len_m), 1e-6)
    pad = int(np.ceil(3.0 * L / float(resolution)))
    n0, n1 = int(ny) + 2 * pad, int(nx) + 2 * pad
    w = rng.standard_normal((n0, n1))
    ky = 2.0 * np.pi * np.fft.fftfreq(n0, d=float(resolution))
    kx = 2.0 * np.pi * np.fft.rfftfreq(n1, d=float(resolution))
    filt = np.exp(-(ky[:, None] ** 2 + kx[None, :] ** 2) * (L * L) / 8.0)
    f = np.fft.irfft2(np.fft.rfft2(w) * filt, s=(n0, n1))
    f = f[pad:pad + int(ny), pad:pad + int(nx)]
    # the filter's own gain, not this realisation's spread: the variance asked for is the ensemble's
    gain = float(np.sqrt((2.0 * np.sum(filt[:, 1:] ** 2) + np.sum(filt[:, 0] ** 2)) / (n0 * n1)))
    return f / max(gain, 1e-300)


def quantile_classes(field: np.ndarray, share_low: float, share_high: float) -> np.ndarray:
    """int16 ids of a field cut at two quantiles: 0 for the lowest ``share_low`` of the cells, 1 for the
    highest ``share_high``, -1 elsewhere. The two sets never touch (the middle lies between them)."""
    lo, hi = float(share_low), float(share_high)
    if lo < 0.0 or hi < 0.0 or lo + hi > 1.0:
        raise ValueError(f"shares {lo}, {hi} must be >= 0 and sum to at most 1")
    ids = np.full(field.shape, -1, dtype=np.int16)
    if lo > 0.0:
        ids[field <= np.quantile(field, lo)] = 0
    if hi > 0.0:
        ids[field >= np.quantile(field, 1.0 - hi)] = 1
    return ids


# ----------------------------------------------------------------------------- sediment patches
_FIELD = (
    ("corr_len_m", "Patch correlation length [m]", "float", 2.0, 0.3, 50.0,
     "Distance over which the patch field stays alike; patches are a few times this across."),
    ("finer", "Finer sediment", MATERIAL_CHOICE, "soft_mud", 0, 0,
     "Sediment of the dark patches (a finer class than the reference)."),
    ("share_finer", "Share of the finer sediment", "float", 0.03, 0.0, 0.5,
     "Fraction of the area covered by patches of the finer sediment."),
    ("coarser", "Coarser sediment", MATERIAL_CHOICE, "gravel", 0, 0,
     "Sediment of the bright patches (a coarser class, shell hash or gravel)."),
    ("share_coarser", "Share of the coarser sediment", "float", 0.02, 0.0, 0.5,
     "Fraction of the area covered by patches of the coarser sediment."),
)


def sediment_field_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    f = gaussian_field(grid.ny, grid.nx, grid.resolution, p["corr_len_m"], rng)
    ids = quantile_classes(f, p["share_finer"], p["share_coarser"])
    return LayerOutput(material=(ids, [p["finer"], p["coarser"]]), material_edge="hard")


register(LayerSpec(
    kind="sediment_field", fn=sediment_field_fn, label="Sediment patch field",
    family="material", schema=_FIELD, geometry=(), channels=("material",),
    reference="[33] Jackson & Richardson 2007 (Table 13.1, Fig. 16.2); [35] Lyons & Abraham 1999; "
              "[51] Abraham & Lyons 2002",
    consequence="Dark patches of finer sediment and bright patches of coarser sediment, metres across, "
                "in the reference seabed.",
    blend="normal"))


# ----------------------------------------------------------------------------- micro-relief
_MICRO = (
    ("rms_m", "RMS height [m]", "float", 0.01, 0.0, 0.2,
     "Root-mean-square height of the relief about the bed."),
    ("corr_len_m", "Correlation length [m]", "float", 0.5, 0.2, 50.0,
     "Distance over which the relief stays alike; under four grid cells the terrain mesh flattens it."),
)


def microrelief_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    f = gaussian_field(grid.ny, grid.nx, grid.resolution, p["corr_len_m"], rng)
    return LayerOutput(delta=p["rms_m"] * f)


register(LayerSpec(
    kind="microrelief", fn=microrelief_fn, label="Micro-relief (random field)",
    family="relief", schema=_MICRO, geometry=(), channels=("height",),
    reference="[33] Jackson & Richardson 2007 (Table 6.1: RMS relief 0.2-2.3 cm over a photograph's field)",
    consequence="A fine mottling of the seabed level: sonar-facing slopes brighter, lee slopes darker.",
    blend="add"))


# ----------------------------------------------------------------------------- pits and mounds
def _bumps(grid, xy: np.ndarray, radius: np.ndarray, height: np.ndarray) -> np.ndarray:
    """Sum of Gaussian bumps ``height * exp(-r^2 / radius^2)`` (negative height = a pit)."""
    xx, yy = geo.grid_coords(grid)
    delta = np.zeros((grid.ny, grid.nx))
    for (cx, cy), r, h in zip(xy, radius, height):
        win = geo.cell_window(grid, cx - 3.0 * r, cy - 3.0 * r, cx + 3.0 * r, cy + 3.0 * r)
        if win is None:
            continue
        delta[win] += h * np.exp(-((xx[win] - cx) ** 2 + (yy[win] - cy) ** 2) / max(r * r, 1e-9))
    return delta


def _uniform_points(grid, n: int, rng: np.random.Generator) -> np.ndarray:
    xmin, ymin, xmax, ymax = grid.extent
    return np.column_stack([rng.uniform(xmin, xmax, n), rng.uniform(ymin, ymax, n)])


_BIO = (
    ("pit_cover", "Share of the floor in pits", "float", 0.005, 0.0, 0.05,
     "Fraction of the seabed area taken by feeding pits (measured: up to 0.013)."),
    ("pit_radius_m", "Pit radius [m]", "float", 0.25, 0.1, 1.0,
     "Typical radius of a pit; each pit is drawn between 0.6 and 1.6 times this."),
    ("pit_depth_m", "Pit depth [m]", "float", 0.06, 0.0, 0.3,
     "Typical depth of a pit; each pit is drawn between 0.5 and 2 times this."),
    ("mounds_per_m2", "Large mounds per m²", "float", 0.01, 0.0, 1.0,
     "Number of large burrow mounds per square metre (small mounds are below the grid)."),
    ("mound_radius_m", "Mound radius [m]", "float", 0.18, 0.1, 1.0,
     "Typical radius of a large mound."),
    ("mound_height_m", "Mound height [m]", "float", 0.10, 0.0, 0.5,
     "Typical height of a large mound; each is drawn between 0.5 and 2 times this."),
)


def bioturbation_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    xmin, ymin, xmax, ymax = grid.extent
    area = max((xmax - xmin) * (ymax - ymin), 1.0)
    # a pit of radius r covers about pi r^2
    n_pit = int(round(p["pit_cover"] * area / (np.pi * max(p["pit_radius_m"], 1e-3) ** 2)))
    n_mnd = int(round(p["mounds_per_m2"] * area))
    delta = np.zeros((grid.ny, grid.nx))
    if n_pit:
        delta += _bumps(grid, _uniform_points(grid, n_pit, rng),
                        p["pit_radius_m"] * rng.uniform(0.6, 1.6, n_pit),
                        -p["pit_depth_m"] * rng.uniform(0.5, 2.0, n_pit))
    if n_mnd:
        delta += _bumps(grid, _uniform_points(grid, n_mnd, rng),
                        p["mound_radius_m"] * rng.uniform(0.8, 1.2, n_mnd),
                        p["mound_height_m"] * rng.uniform(0.5, 2.0, n_mnd))
    return LayerOutput(delta=delta, info={"bioturbation": {"pits": n_pit, "mounds": n_mnd}})


register(LayerSpec(
    kind="bioturbation", fn=bioturbation_fn, label="Feeding pits and burrow mounds",
    family="relief", schema=_BIO, geometry=(), channels=("height",),
    reference="[52] Nickell & Atkinson 1995; [53] Townsend & Fonseca 1998; [54] Dworschak & Rodrigues 1997",
    consequence="Scattered dark dots with a bright far rim (pits) and bright dots with a short shadow (mounds).",
    blend="add"))


# ----------------------------------------------------------------------------- drag marks
_DRAG = (
    ("per_hectare", "Marks per hectare", "float", 4.0, 0.0, 200.0,
     "Number of anchor, chain or keel furrows per hectare."),
    ("width_m", "Width [m]", "float", 1.0, 0.3, 6.0,
     "Typical width of a furrow; each is drawn between 0.5 and 2 times this."),
    ("depth_m", "Depth [m]", "float", 0.12, 0.0, 1.0,
     "Typical depth of a furrow; each is drawn between 0.5 and 2 times this."),
    ("length_m", "Length [m]", "float", 30.0, 3.0, 400.0,
     "Typical length of a furrow; each is drawn between 0.4 and 2 times this."),
)


def drag_marks_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    xmin, ymin, xmax, ymax = grid.extent
    area_ha = max((xmax - xmin) * (ymax - ymin), 1.0) / 1e4
    n = int(round(p["per_hectare"] * area_ha))
    delta = np.zeros((grid.ny, grid.nx))
    marks = []
    for _ in range(n):
        cx, cy = rng.uniform(xmin, xmax), rng.uniform(ymin, ymax)
        th = rng.uniform(0.0, np.pi)
        L = p["length_m"] * rng.uniform(0.4, 2.0)
        w = p["width_m"] * rng.uniform(0.5, 2.0)
        d = p["depth_m"] * rng.uniform(0.5, 2.0)
        bend = rng.uniform(-0.15, 0.15) * L                    # a gentle arc: the mid point pushed sideways
        ux, uy = np.cos(th), np.sin(th)
        pts = [[cx - 0.5 * L * ux, cy - 0.5 * L * uy],
               [cx - bend * uy, cy + bend * ux],
               [cx + 0.5 * L * ux, cy + 0.5 * L * uy]]
        # the arc as a few chords
        t = np.linspace(0.0, 1.0, 9)[:, None]
        a, b, c = (np.array(q) for q in pts)
        curve = ((1 - t) ** 2) * a + 2 * (1 - t) * t * b + (t ** 2) * c
        dist = geo.polyline_distance(curve.tolist(), grid, 3.0 * w)
        prof = np.exp(-(np.where(np.isfinite(dist), dist, 1e9) / max(0.5 * w, 1e-6)) ** 2)
        # a trough with a low berm on each side: the sediment pushed out of it
        berm = 0.25 * np.exp(-((np.where(np.isfinite(dist), dist, 1e9) - 0.9 * w) / max(0.35 * w, 1e-6)) ** 2)
        delta += d * (berm - prof)
        marks.append({"x": round(float(cx), 2), "y": round(float(cy), 2), "heading_deg": round(float(np.degrees(th)), 1),
                      "length_m": round(float(L), 1), "width_m": round(float(w), 2), "depth_m": round(float(d), 3)})
    return LayerOutput(delta=delta, info={"drag_marks": marks})


register(LayerSpec(
    kind="drag_marks", fn=drag_marks_fn, label="Anchor, chain and keel drag marks",
    family="relief", schema=_DRAG, geometry=(), channels=("height",),
    reference="[55] Watson et al. 2022; [56] Watson et al. 2020; [57] Madricardo et al. 2019",
    consequence="A few long dark furrows with thin bright edges crossing the floor.",
    blend="add"))


# ----------------------------------------------------------------------------- small hard items
def _hard_items_schema(what: str, density: float, lmin: float, lmax: float, hmin: float, hmax: float,
                       material: str) -> tuple:
    return (
        ("per_m2", f"{what} per m²", "float", density, 0.0, 20.0,
         f"Number of {what.lower()} per square metre of seabed."),
        ("length_min_m", "Shortest [m]", "float", lmin, 0.05, 2.0,
         "Length of the smallest item."),
        ("length_max_m", "Longest [m]", "float", lmax, 0.05, 2.0,
         "Length of the largest item."),
        ("height_min_m", "Lowest [m]", "float", hmin, 0.0, 1.0,
         "Height of the lowest item above the bed."),
        ("height_max_m", "Highest [m]", "float", hmax, 0.0, 1.0,
         "Height of the highest item above the bed."),
        ("material", "Seabed class of the items", MATERIAL_CHOICE, material, 0, 0,
         "Hard seabed class the cells under an item take."),
    )


def _hard_items(key: str):
    def fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
        xmin, ymin, xmax, ymax = grid.extent
        area = max((xmax - xmin) * (ymax - ymin), 1.0)
        n = int(round(p["per_m2"] * area))
        xy = _uniform_points(grid, n, rng)
        lo, hi = sorted((p["length_min_m"], p["length_max_m"]))
        h0, h1 = sorted((p["height_min_m"], p["height_max_m"]))
        length = rng.uniform(lo, hi, n)
        height = rng.uniform(h0, h1, n)
        yaw = rng.uniform(0.0, np.pi, n)
        res = float(grid.resolution)
        delta = np.zeros((grid.ny, grid.nx))
        ids = np.full((grid.ny, grid.nx), -1, dtype=np.int16)
        for (cx, cy), L, h, th in zip(xy, length, height, yaw):
            # an item is drawn as a short ridge along its axis: its cells take the hard class and its height
            m = max(int(np.ceil(L / res)), 1)
            for s in (np.arange(m) - 0.5 * (m - 1)) * min(res, L):
                i = int(np.floor((cx + s * np.cos(th) - grid.origin_x) / res + 0.5))
                j = int(np.floor((cy + s * np.sin(th) - grid.origin_y) / res + 0.5))
                if 0 <= i < grid.nx and 0 <= j < grid.ny:
                    delta[j, i] = max(delta[j, i], h)
                    ids[j, i] = 0
        items = {"x": np.round(xy[:, 0], 3), "y": np.round(xy[:, 1], 3), "length_m": np.round(length, 3),
                 "height_m": np.round(height, 3), "yaw_rad": np.round(yaw, 3)}
        return LayerOutput(delta=delta, material=(ids, [p["material"]]), material_edge="hard",
                           info={f"{key}_arrays": items})
    return fn


register(LayerSpec(
    kind="small_litter", fn=_hard_items("small_litter"), label="Small litter (bottles, cans)",
    family="relief", schema=_hard_items_schema("Items", 0.06, 0.10, 0.40, 0.06, 0.12, "gravel"),
    geometry=(), channels=("height", "material"),
    reference="[58] Schernewski et al. 2023 (0.02-3.71 items per m2 in 14 harbours)",
    consequence="Isolated bright points, too small to be recognised: the seabed's own false alarms.",
    blend="add"))

register(LayerSpec(
    kind="cobbles", fn=_hard_items("cobbles"), label="Cobbles and stones",
    family="relief", schema=_hard_items_schema("Stones", 0.02, 0.10, 0.40, 0.05, 0.25, "gravel"),
    geometry=(), channels=("height", "material"),
    reference="[59] NILIM 2022 (foundation rubble 10-200 kg per piece)",
    consequence="Bright points with short shadows, dense at a quay toe.",
    blend="add"))
