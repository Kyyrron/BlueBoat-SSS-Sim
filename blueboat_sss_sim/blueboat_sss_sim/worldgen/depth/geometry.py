"""Geometry fields for the layer editor (Qt-free: numpy, scipy, matplotlib.path).

Every function maps a drawn shape -- polygon, polyline, ellipse, rectangle,
axis, radial -- onto the world grid as a *field*: an inside mask, a signed
distance, a normalised radius, or the curvilinear (s, n) frame of a
polyline. Layers (``worldgen.depth.layers``) and masks (``.masks``) are
built from these fields; nothing here knows about depth values.

Conventions: :class:`GridSpec` origin is the *centre* of cell (0, 0),
rasters are ``[ny, nx]`` (row = y, column = x), the polyline normal ``n``
is positive to the **left** of the direction of travel, and curvature
``kappa`` is positive for a left turn (the centre of curvature lies on the
``n > 0`` side).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np
from scipy import ndimage

from ...core.types import GridSpec, Wall
from .interpolate import rasterize_wall_barriers
from .paint import densify, smoothstep, stroke_distance


def _dist2_to_segments(x: np.ndarray, y: np.ndarray, a: np.ndarray,
                       b: np.ndarray) -> np.ndarray:
    """Squared distance from points (x, y) to their own segment a-b
    (``a``, ``b`` are (N, 2) arrays, one segment per point)."""
    dx, dy = b[:, 0] - a[:, 0], b[:, 1] - a[:, 1]
    l2 = np.maximum(dx * dx + dy * dy, 1e-18)
    t = np.clip(((x - a[:, 0]) * dx + (y - a[:, 1]) * dy) / l2, 0.0, 1.0)
    return (x - (a[:, 0] + t * dx)) ** 2 + (y - (a[:, 1] + t * dy)) ** 2


# ---------------------------------------------------------------- grids
def grid_coords(grid: GridSpec) -> tuple[np.ndarray, np.ndarray]:
    """(xx, yy) world coordinates of every cell centre, ``[ny, nx]``."""
    xs = grid.origin_x + np.arange(grid.nx) * grid.resolution
    ys = grid.origin_y + np.arange(grid.ny) * grid.resolution
    return np.meshgrid(xs, ys)


def cell_window(grid: GridSpec, x0: float, y0: float, x1: float, y1: float
                ) -> tuple[slice, slice] | None:
    """Index window ``(rows, cols)`` of the cells whose centres may lie in
    the world bbox ``[x0, x1] x [y0, y1]`` (inclusive, clipped to the
    grid); ``None`` when the bbox misses the grid entirely."""
    xmin, ymin, xmax, ymax = grid.extent
    if x1 < xmin or x0 > xmax or y1 < ymin or y0 > ymax:
        return None
    res = grid.resolution
    i0 = int(np.clip(np.floor((x0 - grid.origin_x) / res), 0, grid.nx - 1))
    i1 = int(np.clip(np.ceil((x1 - grid.origin_x) / res), 0, grid.nx - 1))
    j0 = int(np.clip(np.floor((y0 - grid.origin_y) / res), 0, grid.ny - 1))
    j1 = int(np.clip(np.ceil((y1 - grid.origin_y) / res), 0, grid.ny - 1))
    return slice(j0, j1 + 1), slice(i0, i1 + 1)


def sub_grid(grid: GridSpec, win: tuple[slice, slice]) -> GridSpec:
    """The :class:`GridSpec` of an index window of ``grid``."""
    rows, cols = win
    res = grid.resolution
    return GridSpec(origin_x=grid.origin_x + cols.start * res,
                    origin_y=grid.origin_y + rows.start * res,
                    resolution=res, nx=cols.stop - cols.start,
                    ny=rows.stop - rows.start)


def _points(points: Sequence[Sequence[float]]) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(pts) == 0:
        raise ValueError("geometry has no points")
    return pts


# ------------------------------------------------------------- polygons
def polygon_inside(points: Sequence[Sequence[float]], grid: GridSpec
                   ) -> np.ndarray:
    """Boolean ``[ny, nx]``: cell centres inside the polygon (even-odd rule,
    ``matplotlib.path``). Work is confined to the polygon's bbox."""
    from matplotlib.path import Path

    pts = _points(points)
    if len(pts) < 3:
        raise ValueError("a polygon needs at least 3 points")
    inside = np.zeros((grid.ny, grid.nx), dtype=bool)
    win = cell_window(grid, pts[:, 0].min(), pts[:, 1].min(),
                      pts[:, 0].max(), pts[:, 1].max())
    if win is None:
        return inside
    xx, yy = grid_coords(sub_grid(grid, win))
    hit = Path(pts).contains_points(
        np.column_stack([xx.ravel(), yy.ravel()]))
    inside[win] = hit.reshape(xx.shape)
    return inside


def polygon_signed_distance(points: Sequence[Sequence[float]], grid: GridSpec,
                            reach: float) -> tuple[np.ndarray, np.ndarray]:
    """``(inside, sd)``: the inside mask and the signed distance to the
    polygon boundary -- negative inside, positive outside, evaluated within
    ``reach`` of the boundary (``+inf`` beyond it outside, ``-inf`` beyond it
    inside)."""
    pts = _points(points)
    closed = np.vstack([pts, pts[:1]])
    d = stroke_distance({"points": closed.tolist()}, grid, max(reach, 0.0))
    inside = polygon_inside(pts, grid)
    return inside, np.where(inside, -d, d)


def edt_outside(inside: np.ndarray, resolution: float) -> np.ndarray:
    """Euclidean distance [m] from every cell to the nearest ``inside``
    cell (0 on the inside; ``+inf`` when nothing is inside)."""
    if not inside.any():
        return np.full(inside.shape, np.inf)
    return ndimage.distance_transform_edt(~inside, sampling=resolution)


def feather_weight(distance: np.ndarray, feather: float) -> np.ndarray:
    """1 where ``distance <= 0``, a C1 smoothstep down to exactly 0 at
    ``feather``; a non-positive feather is a hard edge."""
    if feather <= 0.0:
        return (distance <= 0.0).astype(np.float64)
    return smoothstep(1.0 - np.maximum(distance, 0.0) / feather)


def band_weight(distance: np.ndarray, half_width: float, feather: float
                ) -> np.ndarray:
    """1 within ``half_width`` of a line, feathered to 0 beyond it."""
    return feather_weight(distance - half_width, feather)


# ----------------------------------------------------- simple primitives
def rect_inside(grid: GridSpec, x_min: float, y_min: float, x_max: float,
                y_max: float) -> np.ndarray:
    xx, yy = grid_coords(grid)
    return (xx >= x_min) & (xx <= x_max) & (yy >= y_min) & (yy <= y_max)


def rect_distance(grid: GridSpec, x_min: float, y_min: float, x_max: float,
                  y_max: float) -> np.ndarray:
    """Signed distance to an axis-aligned rectangle (negative inside)."""
    xx, yy = grid_coords(grid)
    dx = np.maximum(np.maximum(x_min - xx, xx - x_max), 0.0)
    dy = np.maximum(np.maximum(y_min - yy, yy - y_max), 0.0)
    outside = np.hypot(dx, dy)
    inner = np.minimum(np.minimum(xx - x_min, x_max - xx),
                       np.minimum(yy - y_min, y_max - yy))
    return np.where(outside > 0.0, outside, -np.maximum(inner, 0.0))


def ellipse_rho(grid: GridSpec, centre: Sequence[float], rx: float, ry: float,
                angle_deg: float = 0.0) -> np.ndarray:
    """Normalised elliptical radius: 0 at the centre, 1 on the boundary."""
    xx, yy = grid_coords(grid)
    th = np.radians(angle_deg)
    c, s = np.cos(th), np.sin(th)
    dx, dy = xx - float(centre[0]), yy - float(centre[1])
    u = c * dx + s * dy
    v = -s * dx + c * dy
    return np.sqrt((u / max(float(rx), 1e-9)) ** 2
                   + (v / max(float(ry), 1e-9)) ** 2)


def ellipse_distance(grid: GridSpec, centre: Sequence[float], rx: float,
                     ry: float, angle_deg: float = 0.0) -> np.ndarray:
    """Approximate signed distance to an ellipse boundary [m] (exact for a
    circle): ``(rho - 1) * min(rx, ry)``."""
    return (ellipse_rho(grid, centre, rx, ry, angle_deg) - 1.0) * min(
        float(rx), float(ry))


def axis_coords(grid: GridSpec, a: Sequence[float], b: Sequence[float]
                ) -> tuple[np.ndarray, np.ndarray, float]:
    """(along, across, length) about the axis from ``a`` to ``b``: ``along``
    is 0 at ``a`` and ``length`` at ``b``; ``across`` is positive to the
    left of a -> b."""
    xx, yy = grid_coords(grid)
    ax, ay = float(a[0]), float(a[1])
    dx, dy = float(b[0]) - ax, float(b[1]) - ay
    length = float(np.hypot(dx, dy))
    if length < 1e-9:
        raise ValueError("axis endpoints coincide")
    tx, ty = dx / length, dy / length
    along = tx * (xx - ax) + ty * (yy - ay)
    across = tx * (yy - ay) - ty * (xx - ax)
    return along, across, length


def radial_distance(grid: GridSpec, centre: Sequence[float]) -> np.ndarray:
    xx, yy = grid_coords(grid)
    return np.hypot(xx - float(centre[0]), yy - float(centre[1]))


def line_family_distance(grid: GridSpec, heading_deg: float, spacing: float,
                         phase_m: float = 0.0) -> np.ndarray:
    """Distance to the nearest of a family of parallel lines (crests along
    ``heading_deg``, ``spacing`` apart) -- the furrow/dune skeleton."""
    xmin, ymin, xmax, ymax = grid.extent
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    xx, yy = grid_coords(grid)
    th = np.radians(heading_deg)
    across = -np.sin(th) * (xx - cx) + np.cos(th) * (yy - cy) + phase_m
    sp = max(float(spacing), 1e-6)
    return np.abs((across + 0.5 * sp) % sp - 0.5 * sp)


# ------------------------------------------------------------ polylines
def polyline_distance(points: Sequence[Sequence[float]], grid: GridSpec,
                      reach: float) -> np.ndarray:
    """Distance [m] to a polyline within ``reach`` (``+inf`` beyond)."""
    return stroke_distance({"points": _points(points).tolist()}, grid,
                           max(reach, 0.0))


@dataclass
class PolylineFrame:
    """The curvilinear frame of a polyline over (part of) the grid.

    ``valid`` marks the cells within ``reach``; there ``s`` is the arc
    length of the nearest point on the line, ``n`` the signed offset (left
    positive), ``dist`` = ``|n|``, and ``kappa`` the (smoothed) curvature at
    that point. ``length`` is the total arc length.
    """

    valid: np.ndarray
    s: np.ndarray
    n: np.ndarray
    dist: np.ndarray
    kappa: np.ndarray
    length: float
    s_dense: np.ndarray
    kappa_dense: np.ndarray

    def kappa_lagged(self, lag_m: float) -> np.ndarray:
        """Curvature evaluated ``lag_m`` *upstream* of each cell's foot
        point (``kappa(s - lag)``): the thalweg and the pool settle a
        little after the bend that made them (Ahmed et al. 2024)."""
        if abs(lag_m) < 1e-9:
            return self.kappa
        s_q = np.clip(self.s - lag_m, self.s_dense[0], self.s_dense[-1])
        out = np.zeros_like(self.s)
        out[self.valid] = np.interp(s_q[self.valid], self.s_dense,
                                    self.kappa_dense)
        return out


def polyline_frame(points: Sequence[Sequence[float]], grid: GridSpec,
                   reach: float, smooth_m: float | None = None,
                   spacing_m: float | None = None) -> PolylineFrame:
    """Build the :class:`PolylineFrame` of a polyline within ``reach``.

    The line is densified every ``spacing_m`` (default: one cell, at least
    0.25 m), headings are differenced and Gaussian-smoothed over
    ``smooth_m`` (default 4 spacings) to give the curvature, and every
    cell in the band takes its nearest dense point (``cKDTree``).
    """
    from scipy.spatial import cKDTree

    pts = _points(points)
    if len(pts) < 2:
        raise ValueError("a polyline needs at least 2 points")
    sp = float(spacing_m) if spacing_m else max(grid.resolution, 0.25)
    dense = densify(pts, sp)
    # Drop consecutive duplicates (zero-length segments) before differencing.
    keep = np.r_[True, np.hypot(*(dense[1:] - dense[:-1]).T) > 1e-9]
    dense = dense[keep]
    seg = np.hypot(*(dense[1:] - dense[:-1]).T)
    s_dense = np.r_[0.0, np.cumsum(seg)]
    tang = np.gradient(dense, s_dense, axis=0) if len(dense) > 2 else (
        np.repeat((dense[-1:] - dense[:1]) / max(s_dense[-1], 1e-9), len(dense), 0))
    norm = np.maximum(np.hypot(tang[:, 0], tang[:, 1]), 1e-12)
    tang = tang / norm[:, None]
    theta = np.unwrap(np.arctan2(tang[:, 1], tang[:, 0]))
    kappa_dense = np.gradient(theta, s_dense) if len(dense) > 2 else np.zeros(len(dense))
    sm = (float(smooth_m) if smooth_m is not None else 4.0 * sp) / sp
    if sm > 0.0 and len(dense) > 2:
        kappa_dense = ndimage.gaussian_filter1d(kappa_dense, sm, mode="nearest")

    shape = (grid.ny, grid.nx)
    valid = np.zeros(shape, dtype=bool)
    s = np.zeros(shape)
    n = np.zeros(shape)
    dist = np.full(shape, np.inf)
    kappa = np.zeros(shape)
    win = cell_window(grid, dense[:, 0].min() - reach, dense[:, 1].min() - reach,
                      dense[:, 0].max() + reach, dense[:, 1].max() + reach)
    if win is not None:
        xx, yy = grid_coords(sub_grid(grid, win))
        q = np.column_stack([xx.ravel(), yy.ravel()])
        d, k = cKDTree(dense).query(q)
        # Exact distance to the segments around the nearest dense point
        # (the point spacing alone would quantise the distance field).
        px, py = dense[k, 0], dense[k, 1]
        tx, ty = tang[k, 0], tang[k, 1]
        dx, dy = q[:, 0] - px, q[:, 1] - py
        along = tx * dx + ty * dy
        n_loc = tx * dy - ty * dx
        d2 = np.full(len(q), np.inf)
        for off in (-1, 0):
            i0 = np.clip(k + off, 0, len(dense) - 2)
            d2 = np.minimum(d2, _dist2_to_segments(q[:, 0], q[:, 1],
                                                   dense[i0], dense[i0 + 1]))
        d_exact = np.sqrt(d2)
        ok = d_exact <= reach
        v = np.zeros(xx.shape, dtype=bool)
        v.ravel()[ok] = True
        valid[win] = v
        buf = np.zeros(len(q))
        buf[ok] = (s_dense[k] + along)[ok]
        s[win] = buf.reshape(xx.shape)
        buf = np.zeros(len(q))
        buf[ok] = (np.sign(n_loc) * d_exact)[ok]
        n[win] = buf.reshape(xx.shape)
        buf = np.full(len(q), np.inf)
        buf[ok] = d_exact[ok]
        dist[win] = buf.reshape(xx.shape)
        buf = np.zeros(len(q))
        buf[ok] = kappa_dense[k][ok]
        kappa[win] = buf.reshape(xx.shape)
    return PolylineFrame(valid=valid, s=s, n=n, dist=dist, kappa=kappa,
                         length=float(s_dense[-1]), s_dense=s_dense,
                         kappa_dense=kappa_dense)


# ---------------------------------------------------------------- walls
def wall_labels(grid: GridSpec, walls: Sequence[Wall]) -> np.ndarray:
    """Connected regions of the grid not crossed by a wall (0 = wall
    cell); the same 4-connected labelling the depth paint uses."""
    blocked = rasterize_wall_barriers(list(walls), grid)
    labels, _ = ndimage.label(~blocked,
                              structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])
    return labels


def confine_to_regions(weight: np.ndarray, labels: np.ndarray,
                       seed: np.ndarray) -> np.ndarray:
    """Zero ``weight`` in every wall-region that holds no ``seed`` cell, so
    a feather never reaches across a quay to the other side."""
    if labels.max() <= 1:
        return weight
    hit = np.unique(labels[seed & (labels > 0)])
    keep = np.isin(labels, hit)
    return np.where(keep, weight, 0.0)


def walls_distance(grid: GridSpec, walls: Sequence[Wall], reach: float,
                   names: Sequence[str] | None = None) -> np.ndarray:
    """Distance [m] to the nearest of the given walls within ``reach``."""
    d = np.full((grid.ny, grid.nx), np.inf)
    for w in walls:
        if names and w.name not in names:
            continue
        d = np.minimum(d, stroke_distance(
            {"points": [[w.x0, w.y0], [w.x1, w.y1]]}, grid, reach))
    return d


def coverage_distance(grid: GridSpec, xy: np.ndarray) -> np.ndarray:
    """Distance [m] from every cell to the nearest of the sample points
    ``xy`` (N, 2) -- the support of a survey-derived source."""
    from scipy.spatial import cKDTree

    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(xy) == 0:
        return np.full((grid.ny, grid.nx), np.inf)
    xx, yy = grid_coords(grid)
    d, _ = cKDTree(xy).query(np.column_stack([xx.ravel(), yy.ravel()]))
    return d.reshape(xx.shape)


def geometry_points(geometry: dict[str, Any] | None) -> np.ndarray:
    """Every control point a geometry dict carries (for bboxes)."""
    if not geometry:
        return np.empty((0, 2))
    t = geometry.get("type")
    if t in ("polygon", "polyline"):
        return _points(geometry["points"])
    if t == "ellipse":
        c = geometry["centre"]
        r = max(float(geometry["rx_m"]), float(geometry["ry_m"]))
        return np.array([[c[0] - r, c[1] - r], [c[0] + r, c[1] + r]])
    if t == "rect":
        return np.array([[geometry["x_min"], geometry["y_min"]],
                         [geometry["x_max"], geometry["y_max"]]])
    if t == "axis":
        return np.array([geometry["a"], geometry["b"]], dtype=np.float64)
    if t == "radial":
        c = geometry["centre"]
        r = float(geometry["radius_m"])
        return np.array([[c[0] - r, c[1] - r], [c[0] + r, c[1] + r]])
    if t == "strokes":
        pts = [p for s in geometry["strokes"] for p in s["points"]]
        return np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    return np.empty((0, 2))


def round_region(inside: np.ndarray, resolution: float, radius_m: float
                 ) -> np.ndarray:
    """``inside`` with every corner rounded to ``radius_m``.

    A morphological opening followed by a closing, both with a disc of
    that radius, computed on Euclidean distance transforms: the opening
    rounds the convex corners (and drops spurs thinner than the disc), the
    closing rounds the concave ones (and fills notches). The result is the
    set a disc of that radius can sweep, so no boundary of it turns
    sharper than the disc -- which is what makes a hand-drawn polygon read
    as a natural shoreline rather than as a polygon.

    An operator can only click vertices, so without this every drawn
    water's edge came out faceted (reported 2026-09-18). Radius 0 returns
    the input untouched."""
    if radius_m <= 0.0 or not inside.any():
        return inside
    r = float(radius_m)
    d_in = ndimage.distance_transform_edt(inside, sampling=resolution)
    eroded = d_in > r
    if not eroded.any():
        # The region is thinner than the disc everywhere: rounding it away
        # entirely would silently delete the operator's area, so keep it.
        return inside
    opened = ndimage.distance_transform_edt(~eroded, sampling=resolution) <= r
    dilated = ndimage.distance_transform_edt(~opened, sampling=resolution) <= r
    return ndimage.distance_transform_edt(dilated, sampling=resolution) > r


def signed_distance_of(inside: np.ndarray, resolution: float) -> np.ndarray:
    """Signed Euclidean distance [m] to the boundary of ``inside``:
    negative inside, positive outside, 0 on the edge."""
    if not inside.any():
        return np.full(inside.shape, np.inf)
    if inside.all():
        return np.full(inside.shape, -np.inf)
    out = ndimage.distance_transform_edt(~inside, sampling=resolution)
    ins = ndimage.distance_transform_edt(inside, sampling=resolution)
    return np.where(inside, -ins, out)
