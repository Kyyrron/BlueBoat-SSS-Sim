"""Scattered soundings -> a smooth, realistic depth raster.

The file/CSV path (:mod:`.interpolate`) triangulates linearly and holds
the nearest value outside the data -- exact where a survey gave a
number, flat plateaus where it did not. A sonar track is different:
thousands of noisy soundings along a few lines, and the seabed between
and beyond the lines is unknown but *continuous*. This module fits a
smoothing thin-plate spline to the decimated soundings, continues the
local slope beyond the surveyed strip with a damped trend, and adds a
seeded low-amplitude undulation scaled by how rough the seabed actually
measured -- a flat beach stays flat, a rippled harbour floor ripples.

Walls partition the world exactly as in :func:`interpolate_to_grid`:
each wall-separated region is fitted from its own soundings only, and
the trend extension never crosses a wall.
"""

from __future__ import annotations

import numpy as np

from ...core.types import GridSpec, Wall
from .interpolate import flood_fill_nearest, rasterize_wall_barriers

#: Default distance over which a continued slope saturates (m).
DEFAULT_TREND_LENGTH_M = 15.0
#: Wavelength band (m) of the seabed undulation added between soundings.
TEXTURE_WAVELENGTHS_M = (5.0, 25.0)
#: Soundings within this distance of a cell keep it exactly untextured.
TEXTURE_TAPER_M = 2.0
#: Coarse evaluation grid: at most this many cells along the longer axis.
MAX_COARSE_CELLS = 300
#: Hard cap on the soundings handed to one spline (decimate first).
MAX_FIT_SAMPLES = 20_000


def _coarse_grid(grid: GridSpec, cell_m: float | None) -> GridSpec:
    xmin, ymin, xmax, ymax = grid.extent
    w, h = xmax - xmin, ymax - ymin
    cell = float(cell_m) if cell_m else max(0.5, 4.0 * grid.resolution)
    cell = max(cell, max(w, h) / MAX_COARSE_CELLS, grid.resolution)
    nx = int(np.ceil(w / cell)) + 1
    ny = int(np.ceil(h / cell)) + 1
    return GridSpec(origin_x=xmin, origin_y=ymin, resolution=cell, nx=nx, ny=ny)


def _fit_tps(pts: np.ndarray, z: np.ndarray, smoothing: float):
    """Global smoothing thin-plate spline with 3-pass MAD rejection.
    Returns ``(interpolator, keep_mask, residual_rms)``; raises
    ``numpy.linalg.LinAlgError`` on degenerate (collinear) soundings."""
    from scipy.interpolate import RBFInterpolator
    keep = np.ones(len(z), dtype=bool)
    rbf = None
    for _ in range(3):
        rbf = RBFInterpolator(pts[keep], z[keep], kernel="thin_plate_spline",
                              smoothing=float(smoothing) * int(keep.sum()),
                              degree=1)
        r = z - rbf(pts)
        mad = 1.4826 * float(np.median(np.abs(r[keep] - np.median(r[keep]))))
        new = np.abs(r) < max(3.0 * mad, 0.15)
        if new.sum() < 3 or np.array_equal(new, keep):
            break
        keep = new
    resid = z[keep] - rbf(pts[keep])
    return rbf, keep, float(np.sqrt(np.mean(np.square(resid)))) if len(resid) else 0.0


#: Soundings whose across-track (minor principal) spread is below this
#: are one survey line: no across-track information for a spline.
MIN_ACROSS_TRACK_STD_M = 1.5


def _degenerate(pts: np.ndarray, cell: float) -> bool:
    """True when the soundings lie (nearly) on one line. A spline through
    a single survey line has no across-track information and would
    extrapolate wildly; such a region takes the nearest-value fill
    instead (``cell`` is accepted for symmetry with the callers)."""
    if len(pts) < 3:
        return True
    c = pts - pts.mean(axis=0)
    ev = np.linalg.eigvalsh(c.T @ c / max(len(pts) - 1, 1))
    return bool(np.sqrt(max(ev.min(), 0.0)) < MIN_ACROSS_TRACK_STD_M)


def hull_mask(pts: np.ndarray, xx: np.ndarray, yy: np.ndarray) -> np.ndarray:
    """Cells inside the soundings' convex hull (False everywhere for a
    degenerate hull)."""
    from scipy.spatial import Delaunay, QhullError
    try:
        tri = Delaunay(pts)
    except (QhullError, ValueError):
        return np.zeros(xx.shape, dtype=bool)
    return tri.find_simplex(np.column_stack([xx.ravel(), yy.ravel()])
                            ).reshape(xx.shape) >= 0


def _band_noise(shape: tuple[int, int], cell: float,
                rng: np.random.Generator) -> np.ndarray:
    """Unit-RMS band-limited field with wavelengths in
    ``TEXTURE_WAVELENGTHS_M`` (difference of two Gaussian blurs)."""
    from scipy.ndimage import gaussian_filter
    lo, hi = TEXTURE_WAVELENGTHS_M
    white = rng.standard_normal(shape)
    s_lo = max(lo / (2.0 * np.pi) / cell, 0.3)
    s_hi = max(hi / (2.0 * np.pi) / cell, s_lo * 1.5)
    band = gaussian_filter(white, s_lo) - gaussian_filter(white, s_hi)
    rms = float(np.sqrt(np.mean(np.square(band))))
    return band / rms if rms > 0 else band


def interpolate_smooth_to_grid(east: np.ndarray, north: np.ndarray,
                               depth: np.ndarray, grid: GridSpec,
                               walls: list[Wall], *, smoothing: float = 1.0,
                               trend_length_m: float = DEFAULT_TREND_LENGTH_M,
                               texture_gain: float = 0.5,
                               rng: np.random.Generator | None = None,
                               cell_m: float | None = None) -> np.ndarray:
    """Soundings (depth positive down) -> seabed height raster [ny, nx]
    (z, negative down), smooth, damped beyond the data, gently textured.

    * inside each wall-region's sounding hull: a global smoothing
      thin-plate spline (``smoothing`` scales scipy's per-sample penalty;
      0 interpolates exactly, 1 is a gentle fit);
    * outside the hull: the spline's slope at the BFS-nearest hull edge
      (never across a wall) continued with ``L (1 - exp(-d/L))``,
      ``L = trend_length_m`` -- the edge slope carries on and saturates at
      ``L * |slope|``; a plane stays a plane;
    * a seeded undulation of wavelength 5-25 m whose RMS is
      ``texture_gain`` times the soundings' RMS residual about the fit,
      tapered to zero within ``TEXTURE_TAPER_M`` of a sounding
      (``texture_gain`` 0, or a residual of 0, adds exactly nothing);
    * regions without a sounding and the wall cells take nearest values.

    Evaluated on a coarse grid (``cell_m``, default ``max(0.5 m, 4 *
    resolution)``, at most ``MAX_COARSE_CELLS`` per axis) and bilinearly
    upsampled: the surface is smooth by construction, so nothing is lost
    and a 1500 x 1500 raster never meets the spline.
    """
    from scipy import ndimage
    from scipy.interpolate import RegularGridInterpolator

    east = np.asarray(east, dtype=np.float64).ravel()
    north = np.asarray(north, dtype=np.float64).ravel()
    depth = np.asarray(depth, dtype=np.float64).ravel()
    if len(depth) == 0:
        raise ValueError("depth file yielded no samples")
    if len(depth) > MAX_FIT_SAMPLES:
        stride = int(np.ceil(len(depth) / MAX_FIT_SAMPLES))
        east, north, depth = east[::stride], north[::stride], depth[::stride]

    cg = _coarse_grid(grid, cell_m)
    cell = cg.resolution
    xs = cg.origin_x + np.arange(cg.nx) * cell
    ys = cg.origin_y + np.arange(cg.ny) * cell
    xx, yy = np.meshgrid(xs, ys)
    shape = (cg.ny, cg.nx)

    blocked = rasterize_wall_barriers(walls, cg)
    labels, n_regions = ndimage.label(
        ~blocked, structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])
    ci, cj = cg.world_to_grid(east, north)
    ci = np.clip(np.round(ci).astype(int), 0, cg.nx - 1)
    cj = np.clip(np.round(cj).astype(int), 0, cg.ny - 1)
    inside_grid = ((east >= xs[0] - 0.5 * cell) & (east <= xs[-1] + 0.5 * cell)
                   & (north >= ys[0] - 0.5 * cell)
                   & (north <= ys[-1] + 0.5 * cell))
    sample_region = np.where(inside_grid, labels[cj, ci], 0)

    z = np.full(shape, np.nan)
    stamped = np.zeros(shape, dtype=bool)
    resid_sq, resid_n = 0.0, 0
    any_inside = False
    L = max(float(trend_length_m), 1e-6)
    for r in range(1, n_regions + 1):
        sel = sample_region == r
        if not sel.any():
            continue
        any_inside = True
        region = labels == r
        pts = np.column_stack([east[sel], north[sel]])
        zs = depth[sel]
        stamped[cj[sel], ci[sel]] = True
        fitted = False
        if int(sel.sum()) >= 3 and not _degenerate(pts, cell):
            try:
                rbf, keep, rms = _fit_tps(pts, zs, smoothing)
                fitted = True
            except np.linalg.LinAlgError:
                fitted = False          # collinear soundings: no plane
        if not fitted:
            # A single line of soundings (or too few): hold the nearest
            # value, exactly as the file path does.
            z[cj[sel], ci[sel]] = zs
            filled = flood_fill_nearest(z, ~region)
            z[region] = filled[region]
            continue
        resid_sq += rms * rms * int(keep.sum())
        resid_n += int(keep.sum())
        hull = hull_mask(pts[keep], xx, yy) & region
        if not hull.any():
            hull = np.zeros(shape, dtype=bool)
            hull[cj[sel], ci[sel]] = True
            hull &= region
        z_r = np.full(shape, np.nan)
        z_r[hull] = rbf(np.column_stack([xx[hull], yy[hull]]))
        # Slope at the hull edge: the spline is evaluated a few cells past
        # the data too (that close its extrapolation is benign), so the
        # edge gradient is two-sided rather than diluted by a flat fill.
        near = ndimage.binary_dilation(hull, iterations=3) & region & ~hull
        z_near = z_r.copy()
        if near.any():
            z_near[near] = rbf(np.column_stack([xx[near], yy[near]]))
        z_fill = flood_fill_nearest(z_near, ~region)
        gy, gx = np.gradient(np.where(region, z_fill, 0.0), cell)
        gx = ndimage.gaussian_filter(gx, 1.0)
        gy = ndimage.gaussian_filter(gy, 1.0)
        edge = hull & ~ndimage.binary_erosion(hull, border_value=0)
        # BFS-nearest edge cell for every outside cell of the region
        # (never across a wall), by propagating the edge cells' indices.
        seed = np.full(shape, np.nan)
        flat = np.flatnonzero(edge)
        seed.flat[flat] = flat.astype(np.float64)
        src = flood_fill_nearest(seed, ~region | (hull & ~edge))
        outside = region & ~hull & np.isfinite(src)
        if outside.any():
            s_idx = src[outside].astype(np.int64)
            sj, si = np.divmod(s_idx, cg.nx)
            oj, oi = np.nonzero(outside)
            dx = (oi - si) * cell
            dy = (oj - sj) * cell
            d = np.hypot(dx, dy)
            slope_along = (gx[sj, si] * dx + gy[sj, si] * dy) / np.maximum(d, 1e-9)
            z_r[oj, oi] = (z_fill[sj, si]
                           + slope_along * L * (1.0 - np.exp(-d / L)))
        z[region] = z_r[region]
    if not any_inside:
        raise ValueError(
            "no depth sample lands inside the world limits -- the recording "
            "and the drawn rectangle do not overlap")
    if not np.isfinite(z).all():
        z = flood_fill_nearest(z, None)

    rms_res = float(np.sqrt(resid_sq / resid_n)) if resid_n else 0.0
    if texture_gain > 0.0 and rms_res > 0.0:
        gen = rng if rng is not None else np.random.default_rng(0)
        noise = _band_noise(shape, cell, gen)
        d_smp = ndimage.distance_transform_edt(~stamped) * cell
        taper = np.clip(d_smp / TEXTURE_TAPER_M, 0.0, 1.0)
        z = z + float(texture_gain) * rms_res * noise * taper

    # Coarse -> the target raster (bilinear).
    fx = grid.origin_x + np.arange(grid.nx) * grid.resolution
    fy = grid.origin_y + np.arange(grid.ny) * grid.resolution
    interp = RegularGridInterpolator((ys, xs), z, method="linear",
                                     bounds_error=False, fill_value=None)
    fxx, fyy = np.meshgrid(fx, fy)
    fine = interp(np.column_stack([fyy.ravel(), fxx.ravel()])).reshape(
        grid.ny, grid.nx)
    return -fine
