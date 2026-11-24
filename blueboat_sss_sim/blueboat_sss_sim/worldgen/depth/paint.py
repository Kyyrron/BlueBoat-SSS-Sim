"""Brush strokes -> height edits, soundings and material stamps (Qt-free).

A *stroke* is the World Builder's paint primitive, YAML-native so a
painted world reproduces headlessly through ``generate_world`` and
reloads in Edit mode::

    {"points": [[x, y], ...], "radius_m": float,
     "value": float | str, "erase": bool[, "feather_m": float]}

Coordinates are metres in whichever frame the caller works in (the
builder paints in the design frame and re-bases at build time, exactly
like walls). A depth stroke's ``value`` is a depth [m, positive down --
zero or negative is dry land]; a material stroke's ``value`` is a
material name. Strokes apply in list order, later strokes winning; an
``erase`` stroke removes earlier paint under its disc. A depth stroke's
optional ``feather_m`` is the width of the smooth blend from its painted
depth back to the surrounding seabed, outside its radius (default half
the radius).

Nothing here draws from an RNG: painting is a pure function of the
strokes, so two builds of the same state are bit-identical, and a world
without strokes is untouched (``apply_depth_paint`` returns the input
unchanged and ``paint_material_ids`` is never called).
"""

from __future__ import annotations

from typing import Any, Callable, Mapping, Sequence

import numpy as np

from ...core.types import GridSpec, Wall
from .ingest import DepthSamples
from .interpolate import interpolate_to_grid, rasterize_wall_barriers

STROKE_KEYS = {"points", "radius_m", "value", "erase", "feather_m"}
#: Ring samples per densified stroke point in :func:`strokes_to_samples`.
RING_SAMPLES = 8
#: A depth stroke without an explicit ``feather_m`` blends into the base
#: over this fraction of its radius ("a small, smooth interpolation").
DEFAULT_FEATHER_RATIO = 0.5
#: Historical name kept for callers; the feather no longer reaches 2R.
FEATHER_SCALE = 1.0 + DEFAULT_FEATHER_RATIO


# ------------------------------------------------------------ validation
def normalize_strokes(strokes: Sequence[Mapping[str, Any]] | None,
                      kind: str = "depth") -> list[dict[str, Any]]:
    """Validate and canonicalise a stroke list.

    Unknown keys raise by name (the same quiet-wrong-answer guard as the
    depth and wall configs); a depth stroke's value is coerced to float, a
    material stroke's to str; an erase stroke needs no value.
    """
    out: list[dict[str, Any]] = []
    for i, s in enumerate(strokes or []):
        if not isinstance(s, Mapping):
            raise TypeError(f"{kind} paint[{i}] is not a mapping: {s!r}")
        bad = set(s) - STROKE_KEYS
        if bad:
            raise KeyError(f"Unknown stroke key(s) in {kind} paint[{i}]: "
                           f"{sorted(bad)} (accepted: {sorted(STROKE_KEYS)})")
        pts = [[float(p[0]), float(p[1])] for p in (s.get("points") or [])]
        if not pts:
            raise ValueError(f"{kind} paint[{i}] has no points")
        radius = float(s.get("radius_m", 1.0))
        if not radius > 0.0:
            raise ValueError(f"{kind} paint[{i}] radius_m must be > 0, "
                             f"got {radius}")
        erase = bool(s.get("erase", False))
        value: Any = s.get("value")
        if not erase:
            if value is None:
                raise ValueError(f"{kind} paint[{i}] has no value")
            value = float(value) if kind == "depth" else str(value)
        rec: dict[str, Any] = {"points": pts, "radius_m": radius,
                               "value": value, "erase": erase}
        if "feather_m" in s:
            if kind != "depth":
                raise KeyError(f"{kind} paint[{i}]: feather_m is a "
                               "depth-stroke key")
            feather = float(s["feather_m"])
            if not feather > 0.0:
                raise ValueError(f"{kind} paint[{i}] feather_m must be > 0, "
                                 f"got {feather}")
            rec["feather_m"] = feather
        elif kind == "depth":
            rec["feather_m"] = DEFAULT_FEATHER_RATIO * radius
        out.append(rec)
    return out


# ------------------------------------------------------------- geometry
def _segments(points: Sequence[Sequence[float]]
              ) -> list[tuple[np.ndarray, np.ndarray]]:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(pts) == 1:
        return [(pts[0], pts[0])]
    return [(pts[k], pts[k + 1]) for k in range(len(pts) - 1)]


def _dist2_to_segment(x: np.ndarray, y: np.ndarray,
                      a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Squared distance from points (x, y) to segment a-b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 <= 1e-18:
        return (x - a[0]) ** 2 + (y - a[1]) ** 2
    t = np.clip(((x - a[0]) * dx + (y - a[1]) * dy) / l2, 0.0, 1.0)
    return (x - (a[0] + t * dx)) ** 2 + (y - (a[1] + t * dy)) ** 2


def densify(points: Sequence[Sequence[float]], spacing_m: float
            ) -> np.ndarray:
    """(N, 2) points along the polyline every ``spacing_m`` (both ends
    included; a single point is returned as is)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(pts) == 1:
        return pts.copy()
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        seg = float(np.hypot(*(b - a)))
        n = max(int(np.ceil(seg / max(spacing_m, 1e-6))), 1)
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return np.asarray(out)


def stroke_mask(stroke: Mapping[str, Any], grid: GridSpec,
                radius_scale: float = 1.0) -> np.ndarray:
    """Boolean [ny, nx] mask of cells within ``radius_m * radius_scale``
    of the stroke's polyline (a capsule per segment, a disc for a dot).
    Work is confined to each segment's bounding box, so a stroke costs
    O(its own area), not O(grid)."""
    r = float(stroke["radius_m"]) * radius_scale
    res = grid.resolution
    mask = np.zeros((grid.ny, grid.nx), dtype=bool)
    xs = grid.origin_x + np.arange(grid.nx) * res
    ys = grid.origin_y + np.arange(grid.ny) * res
    for a, b in _segments(stroke["points"]):
        x0, x1 = min(a[0], b[0]) - r, max(a[0], b[0]) + r
        y0, y1 = min(a[1], b[1]) - r, max(a[1], b[1]) + r
        if x1 < xs[0] or x0 > xs[-1] or y1 < ys[0] or y0 > ys[-1]:
            continue
        i0 = int(np.clip(np.floor((x0 - grid.origin_x) / res), 0, grid.nx - 1))
        i1 = int(np.clip(np.ceil((x1 - grid.origin_x) / res), 0, grid.nx - 1))
        j0 = int(np.clip(np.floor((y0 - grid.origin_y) / res), 0, grid.ny - 1))
        j1 = int(np.clip(np.ceil((y1 - grid.origin_y) / res), 0, grid.ny - 1))
        xx, yy = np.meshgrid(xs[i0:i1 + 1], ys[j0:j1 + 1])
        mask[j0:j1 + 1, i0:i1 + 1] |= _dist2_to_segment(xx, yy, a, b) <= r * r
    return mask


def rasterize_strokes(strokes: Sequence[Mapping[str, Any]], grid: GridSpec,
                      value_of: Callable[[Mapping[str, Any]], Any],
                      empty: Any, dtype=np.float64) -> np.ndarray:
    """Stamp strokes in order into a [ny, nx] raster initialised to
    ``empty``; erase strokes reset their disc to ``empty``."""
    out = np.full((grid.ny, grid.nx), empty, dtype=dtype)
    for s in strokes:
        m = stroke_mask(s, grid)
        out[m] = empty if s["erase"] else value_of(s)
    return out


# -------------------------------------------------------------- samples
def strokes_to_samples(strokes: Sequence[Mapping[str, Any]],
                       spacing_m: float | None = None) -> DepthSamples:
    """Depth strokes as scattered soundings.

    Each non-erase stroke contributes, every ``spacing_m`` (default
    ``radius_m / 2``) along its polyline, the point itself plus
    ``RING_SAMPLES`` points on the circle of its radius, all at the
    stroke's depth. An erase stroke drops every earlier sample within its
    radius. Feeding these into the same wall-aware interpolator the depth
    files use is what makes the paint blend like soundings would.
    """
    strokes = normalize_strokes(strokes, "depth")
    ang = np.linspace(0.0, 2.0 * np.pi, RING_SAMPLES, endpoint=False)
    east: list[np.ndarray] = []
    north: list[np.ndarray] = []
    depth: list[np.ndarray] = []
    for s in strokes:
        r = s["radius_m"]
        pts = densify(s["points"], spacing_m if spacing_m else r / 2.0)
        if s["erase"]:
            if east:
                e = np.concatenate(east)
                n = np.concatenate(north)
                d = np.concatenate(depth)
                d2 = np.full(len(e), np.inf)
                for a, b in _segments(pts):
                    d2 = np.minimum(d2, _dist2_to_segment(e, n, a, b))
                keep = d2 > r * r
                east, north, depth = [e[keep]], [n[keep]], [d[keep]]
            continue
        cx = np.concatenate([pts[:, 0], (pts[:, 0:1] + r * np.cos(ang)).ravel()])
        cy = np.concatenate([pts[:, 1], (pts[:, 1:2] + r * np.sin(ang)).ravel()])
        east.append(cx)
        north.append(cy)
        depth.append(np.full(len(cx), s["value"]))
    if not east:
        return DepthSamples(np.empty(0), np.empty(0), np.empty(0))
    return DepthSamples(np.concatenate(east), np.concatenate(north),
                        np.concatenate(depth))


# --------------------------------------------------------- depth paint
def smoothstep(u: np.ndarray) -> np.ndarray:
    """``u^2 (3 - 2u)`` on ``clip(u, 0, 1)``: a C1 ramp from 0 to 1."""
    u = np.clip(np.asarray(u, dtype=np.float64), 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def stroke_distance(stroke: Mapping[str, Any], grid: GridSpec,
                    reach: float) -> np.ndarray:
    """[ny, nx] distance (m) from every cell to the stroke's polyline,
    evaluated only within ``reach`` of it (``inf`` elsewhere). Analytic --
    the distance to a segment, minimised over segments -- so it is smooth
    outside the stroke wherever the polyline is, unlike the distance to a
    rasterised union of discs, which kinks along its medial axis."""
    res = grid.resolution
    out = np.full((grid.ny, grid.nx), np.inf)
    xs = grid.origin_x + np.arange(grid.nx) * res
    ys = grid.origin_y + np.arange(grid.ny) * res
    for a, b in _segments(stroke["points"]):
        x0, x1 = min(a[0], b[0]) - reach, max(a[0], b[0]) + reach
        y0, y1 = min(a[1], b[1]) - reach, max(a[1], b[1]) + reach
        if x1 < xs[0] or x0 > xs[-1] or y1 < ys[0] or y0 > ys[-1]:
            continue
        i0 = int(np.clip(np.floor((x0 - grid.origin_x) / res), 0, grid.nx - 1))
        i1 = int(np.clip(np.ceil((x1 - grid.origin_x) / res), 0, grid.nx - 1))
        j0 = int(np.clip(np.floor((y0 - grid.origin_y) / res), 0, grid.ny - 1))
        j1 = int(np.clip(np.ceil((y1 - grid.origin_y) / res), 0, grid.ny - 1))
        xx, yy = np.meshgrid(xs[i0:i1 + 1], ys[j0:j1 + 1])
        win = (slice(j0, j1 + 1), slice(i0, i1 + 1))
        out[win] = np.minimum(out[win],
                              np.sqrt(_dist2_to_segment(xx, yy, a, b)))
    return out


def _stroke_weight(stroke: Mapping[str, Any], grid: GridSpec,
                   feather: float) -> np.ndarray:
    """1 inside the stroke's radius, a C1 smoothstep down to 0 at
    ``radius + feather``, 0 beyond (exactly)."""
    r = float(stroke["radius_m"])
    d = stroke_distance(stroke, grid, r + feather)
    return smoothstep(1.0 - np.maximum(d - r, 0.0) / feather)


def apply_depth_paint(height: np.ndarray, grid: GridSpec,
                      strokes: Sequence[Mapping[str, Any]] | None,
                      walls: list[Wall]) -> np.ndarray:
    """Blend depth strokes into a base height raster [ny, nx].

    Inside a stroke's disc the seabed is exactly the painted depth. From
    the disc edge outward the surface blends back to the base over the
    stroke's ``feather_m`` with a C1 smoothstep of the (analytic)
    distance to the stroke, so a dot feathers radially and a polyline as
    a smooth capsule. Strokes combine in list order with the union rule
    ``W = 1 - (1 - W)(1 - w_i)`` and an erase stroke cuts ``W`` by its own
    smooth weight, so overlapping strokes and erased holes never leave a
    ridge; the value blended toward is a Gaussian-weighted average of the
    nearby paint. Only the wall-regions that hold paint are touched: a
    stroke on one side of a quay never changes the other side. Cells
    beyond every feather are untouched (bit-identical to the base).

    Returns the input object itself when there is nothing to paint.
    """
    strokes = normalize_strokes(strokes, "depth")
    if not strokes:
        return height
    from scipy import ndimage

    shape = (grid.ny, grid.nx)
    res = float(grid.resolution)
    painted = np.full(shape, np.nan)
    weight = np.zeros(shape)
    feathers: list[float] = []
    for s in strokes:
        fe = max(float(s["feather_m"]), 2.0 * res)
        w = _stroke_weight(s, grid, fe)
        m = stroke_mask(s, grid)
        if s["erase"]:
            painted[m] = np.nan
            weight *= (1.0 - w)
        else:
            painted[m] = -s["value"]
            weight = 1.0 - (1.0 - weight) * (1.0 - w)
            feathers.append(fe)
    have = np.isfinite(painted)
    if not have.any():
        return height.copy()

    # Work on the influence bounding box only: paint cost scales with the
    # painted area, not the world.
    touched = weight > 0.0
    rows = np.flatnonzero(touched.any(axis=1))
    cols = np.flatnonzero(touched.any(axis=0))
    j0, j1, i0, i1 = int(rows[0]), int(rows[-1]), int(cols[0]), int(cols[-1])
    sub = GridSpec(origin_x=grid.origin_x + i0 * res,
                   origin_y=grid.origin_y + j0 * res,
                   resolution=res, nx=int(i1 - i0 + 1), ny=int(j1 - j0 + 1))
    win = (slice(j0, j1 + 1), slice(i0, i1 + 1))
    base_sub = np.asarray(height, dtype=np.float64)[win]
    have_sub, painted_sub, w_sub = have[win], painted[win], weight[win]

    blocked = rasterize_wall_barriers(walls, sub)
    labels, _ = ndimage.label(~blocked,
                              structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])
    regions = np.unique(labels[have_sub])
    regions = regions[regions > 0]

    out = np.array(height, dtype=np.float64, copy=True)
    out_sub = out[win]
    sigma = max(0.5 * float(np.median(feathers)) / res, 0.5)
    for r in regions:
        region = labels == r
        src = have_sub & region
        # The value blended toward: a normalised Gaussian average of the
        # region's paint (nearest paint where the average has no support).
        num = ndimage.gaussian_filter(np.where(src, painted_sub, 0.0), sigma)
        den = ndimage.gaussian_filter(src.astype(np.float64), sigma)
        _, (jy, jx) = ndimage.distance_transform_edt(~src, return_indices=True)
        ext = np.where(den > 1e-6, num / np.maximum(den, 1e-12),
                       painted_sub[jy, jx])
        sel = region & (w_sub > 0.0)
        out_sub[sel] = base_sub[sel] + (ext[sel] - base_sub[sel]) * w_sub[sel]
        exact = src & (w_sub >= 1.0 - 1e-9)
        out_sub[exact] = painted_sub[exact]
    return out


# ------------------------------------------------------ material paint
def paint_material_ids(mat_id: np.ndarray, names: list[str], grid: GridSpec,
                       strokes: Sequence[Mapping[str, Any]] | None,
                       known: Callable[[str], Any] | None = None
                       ) -> tuple[np.ndarray, list[str]]:
    """Stamp material strokes over a base material-index raster.

    Returns ``(mat_id, names)``; a stroke naming a material absent from
    ``names`` appends it. ``known(name)`` is called on every painted
    material before stamping (pass ``MaterialLibrary.get`` so an unknown
    name raises by name); an erase stroke restores the base layout.
    """
    strokes = normalize_strokes(strokes, "material")
    if not strokes:
        return mat_id, list(names)
    names = list(names)
    base = mat_id.copy()
    out = mat_id.copy()
    for s in strokes:
        m = stroke_mask(s, grid)
        if s["erase"]:
            out[m] = base[m]
            continue
        name = s["value"]
        if known is not None:
            known(name)
        if name not in names:
            if len(names) >= 255:
                raise ValueError("too many distinct materials (max 255)")
            names.append(name)
        out[m] = names.index(name)
    return out, names
