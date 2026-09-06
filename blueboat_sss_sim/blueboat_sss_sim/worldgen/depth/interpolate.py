"""Scattered soundings -> dense height raster, wall-aware.

Inside the data's convex hull the surface is linear interpolation
(``scipy.interpolate.griddata``). Outside it -- the part of the world the
depth file never covered -- each cell takes the value of its BFS-nearest
known cell, where the search never crosses a wall: "extend the furthest
known depth until a wall is found". The grid border needs no special
case; the fill simply stops there, which is exactly the world-limit
semantics. Wall cells themselves (and any region fully sealed off from
every sounding) get a second, unrestricted fill so the raster ends
finite everywhere.
"""

from __future__ import annotations

import numpy as np

from ...core.types import GridSpec, Wall
from .ingest import DepthSamples


def rasterize_wall_barriers(walls: list[Wall], grid: GridSpec) -> np.ndarray:
    """Boolean [ny, nx] mask of cells crossed by any wall segment.

    Supercover line rasterisation (dense sampling at sub-cell step), so a
    diagonal wall leaves no corner-leak for the 4-connected fill.
    """
    blocked = np.zeros((grid.ny, grid.nx), dtype=bool)
    for w in walls:
        n = max(2, int(np.ceil(w.length / (0.25 * grid.resolution))) + 1)
        xs = np.linspace(w.x0, w.x1, n)
        ys = np.linspace(w.y0, w.y1, n)
        ci, cj = grid.world_to_grid(xs, ys)
        ci = np.clip(np.round(ci).astype(int), 0, grid.nx - 1)
        cj = np.clip(np.round(cj).astype(int), 0, grid.ny - 1)
        blocked[cj, ci] = True
        # Thicken by one cell across the wall so the 4-connected BFS cannot
        # slip diagonally between two rasterised cells.
        nx_, ny_ = w.normal
        ci2 = np.clip(np.round(ci + np.sign(nx_)).astype(int), 0, grid.nx - 1)
        cj2 = np.clip(np.round(cj + np.sign(ny_)).astype(int), 0, grid.ny - 1)
        blocked[cj2, ci2] = True
    return blocked


def flood_fill_nearest(values: np.ndarray,
                       blocked: np.ndarray | None = None) -> np.ndarray:
    """Fill NaN cells with the value of the BFS-nearest finite cell.

    Multi-source breadth-first wave over the 4-connected grid; ``blocked``
    cells neither receive nor forward values. Two equally-near sources tie
    arbitrarily (both are correct at that distance). O(cells).
    """
    ny, nx = values.shape
    v = values.astype(np.float64).ravel().copy()
    blk = (np.zeros((ny, nx), dtype=bool) if blocked is None
           else blocked).ravel()
    known = np.isfinite(v) & ~blk
    frontier = np.flatnonzero(known)

    while frontier.size:
        nxt = []
        col = frontier % nx
        for off, sel in ((1, col < nx - 1), (-1, col > 0),
                         (nx, frontier < (ny - 1) * nx), (-nx, frontier >= nx)):
            src = frontier[sel]
            dst = src + off
            take = ~known[dst] & ~blk[dst]
            src, dst = src[take], dst[take]
            if dst.size:
                v[dst] = v[src]
                known[dst] = True
                nxt.append(dst)
        frontier = np.unique(np.concatenate(nxt)) if nxt else np.empty(0, int)
    return v.reshape(ny, nx)


def interpolate_to_grid(samples: DepthSamples, grid: GridSpec,
                        walls: list[Wall]) -> np.ndarray:
    """Soundings -> seabed height raster [ny, nx] (z, negative down).

    Walls partition the world into connected regions, and each region is
    interpolated from *its own* soundings only -- two sides of a quay wall
    never blend, in the linear surface or in the fill. A region with no
    sounding at all (and the wall cells themselves) takes nearest values
    in a final unrestricted pass, so the raster ends finite everywhere.
    """
    if len(samples) == 0:
        raise ValueError("depth file yielded no samples")
    from scipy import ndimage
    from scipy.interpolate import griddata

    xs = grid.origin_x + np.arange(grid.nx) * grid.resolution
    ys = grid.origin_y + np.arange(grid.ny) * grid.resolution
    xx, yy = np.meshgrid(xs, ys)
    shape = (grid.ny, grid.nx)

    blocked = rasterize_wall_barriers(walls, grid)
    labels, n_regions = ndimage.label(
        ~blocked, structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])

    # Cell (clipped into the grid) and region of every sample.
    ci, cj = grid.world_to_grid(samples.east, samples.north)
    ci = np.clip(np.round(ci).astype(int), 0, grid.nx - 1)
    cj = np.clip(np.round(cj).astype(int), 0, grid.ny - 1)
    sample_region = labels[cj, ci]

    depth = np.full(shape, np.nan)
    any_inside = False
    for r in range(1, n_regions + 1):
        sel = sample_region == r
        if not sel.any():
            continue
        any_inside = True
        region = labels == r
        if int(sel.sum()) >= 3:
            try:
                est = griddata(
                    np.column_stack([samples.east[sel], samples.north[sel]]),
                    samples.depth[sel], (xx, yy), method="linear")
                depth[region] = est[region]
            except Exception:
                # Degenerate geometry (collinear soundings): Qhull cannot
                # triangulate -- the region falls to the nearest fill.
                pass
        # Exact values at the samples' own cells anchor the fill even when
        # the linear surface could not be built.
        rci, rcj = ci[sel], cj[sel]
        depth[rcj, rci] = samples.depth[sel]
        # "Extend the furthest known depth until a wall is found": nearest
        # fill confined to this region (cells outside it are barriers, so
        # other regions' values neither leak in nor out).
        filled = flood_fill_nearest(depth, ~region)
        depth[region] = filled[region]

    if not any_inside:
        raise ValueError(
            "no depth sample lands inside the world limits -- the file and "
            "the drawn rectangle do not overlap")
    if not np.isfinite(depth).all():
        # Wall cells + regions sealed off from every sounding.
        depth = flood_fill_nearest(depth, None)
    return -depth
