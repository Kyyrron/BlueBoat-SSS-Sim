"""Spatial statistics of a point pattern on the water of an environment: what
the report of a plan and the many-seed validation measure.

* :func:`clark_evans`: mean nearest-neighbour distance over that of a
  uniform (Poisson) pattern of the same density. Under 1: clustered; 1:
  random; over 1: regular. (Clark and Evans 1954, Ecology 35, 445-453.)
* :func:`pair_correlation`: density of pairs at distance r over that of a
  uniform pattern, on the allowed water, estimated against a uniform
  reference pattern on the same cells (so the shape of the water needs no
  edge correction).
* :func:`distance_to_sources`: distance of each point to the nearest source
  feature.

Pure Python and numpy; scipy.spatial for neighbour searches.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np


def clark_evans(xy: np.ndarray, area_m2: float, perimeter_m: float | None = None) -> float | None:
    """The Clark-Evans ratio of ``xy`` over an area. With the area's
    perimeter, Donnelly's edge correction of the expected distance (Donnelly
    1978: E = 0.5 sqrt(A/n) + (0.0514 + 0.041/sqrt(n)) P/n); without it the
    ratio of a uniform pattern on a small or narrow area lies over 1, so a
    value under 1 is a conservative sign of clustering."""
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    n = len(xy)
    if n < 2 or area_m2 <= 0.0:
        return None
    from scipy.spatial import cKDTree
    d, _ = cKDTree(xy).query(xy, k=2)
    mean = float(d[:, 1].mean())
    expected = 0.5 * float(np.sqrt(area_m2 / n))
    if perimeter_m:
        expected += (0.0514 + 0.041 / float(np.sqrt(n))) * float(perimeter_m) / n
    return mean / expected


def perimeter_m(mask: np.ndarray, cell_m: float) -> float:
    """Length of the boundary of a raster region [m] (cell edges between the
    region and the rest, the grid's border included)."""
    m = np.pad(np.asarray(mask, dtype=bool), 1)
    edges = (m[1:, :] != m[:-1, :]).sum() + (m[:, 1:] != m[:, :-1]).sum()
    return float(edges) * float(cell_m)


def distance_to_sources(xy: np.ndarray, features: Sequence) -> np.ndarray:
    """Distance of each point to the nearest of ``features`` [m] (infinite
    with no feature)."""
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    best = np.full(len(xy), np.inf)
    for f in features:
        best = np.minimum(best, np.maximum(f.distance(xy[:, 0], xy[:, 1]) - float(f.clear_m), 0.0))
    return best


def pair_correlation(xy: np.ndarray, reference_xy: np.ndarray, edges: Sequence[float]) -> np.ndarray:
    """g(r) in the distance bins ``edges``: pairs of the pattern over pairs of
    a uniform reference pattern on the same water, each normalised by its
    number of ordered pairs. NaN where the reference has no pair."""
    from scipy.spatial import cKDTree
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    ref = np.asarray(reference_xy, dtype=np.float64).reshape(-1, 2)
    edges = np.asarray(edges, dtype=np.float64)
    out = np.full(len(edges) - 1, np.nan)
    if len(xy) < 2 or len(ref) < 2:
        return out

    def counts(pts):
        t = cKDTree(pts)
        c = t.count_neighbors(t, edges, cumulative=True).astype(np.float64) - len(pts)
        return np.diff(c)
    a, b = counts(xy), counts(ref)
    na, nb = len(xy) * (len(xy) - 1.0), len(ref) * (len(ref) - 1.0)
    ok = b > 0
    out[ok] = (a[ok] / na) / (b[ok] / nb)
    return out
