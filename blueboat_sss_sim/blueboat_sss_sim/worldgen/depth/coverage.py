"""How much of a world a depth source actually measured.

Both external depth sources -- a bathymetry file and a sonar recording --
place their samples by GPS, so they can land anywhere, including nowhere
near the drawn limits. This module answers two questions the fitters
themselves cannot: *do the samples reach the world at all*, and *how much
of the world did they cover*. The first is an error, the second a warning
carried in the survey summary and in the world's metadata.

ROS-free and Qt-free, like the rest of ``worldgen``.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ...core.geo import (GeoAnchor, format_distance, format_latlon,
                         range_bearing)
from ...core.types import GridSpec

#: Below this share of samples inside the limits, or of world area within
#: reach of a sample, the build warns: most of the seabed is extrapolated.
MIN_INSIDE_FRACTION = 0.5
MIN_AREA_FRACTION = 0.35
#: How far from a sample the fitted surface is still *supported* by data.
#: The damped trend saturates over this distance, so beyond it the seabed
#: is the edge slope continued, not a measurement. Defaults to the depth
#: source's own ``trend_length_m``.
DEFAULT_REACH_M = 15.0


def sample_coverage(east, north, grid: GridSpec,
                    reach_m: float = DEFAULT_REACH_M,
                    max_cells: int = 200) -> dict[str, Any]:
    """Where a set of samples sits relative to the world grid.

    Returns ``inside`` / ``total`` / ``inside_fraction`` (samples within
    the grid extent), ``area_fraction`` (share of grid cells within
    ``reach_m`` of a sample, on a coarse lattice) and both boxes in world
    metres. Distance to the nearest sample rather than the convex hull,
    because a single survey line has a hull of zero area yet supports a
    band either side of it -- and that band is exactly what the damped
    trend can carry.
    """
    e = np.asarray(east, dtype=float).ravel()
    n = np.asarray(north, dtype=float).ravel()
    ok = np.isfinite(e) & np.isfinite(n)
    e, n = e[ok], n[ok]
    xmin, ymin, xmax, ymax = grid.extent
    inside = ((e >= xmin) & (e <= xmax) & (n >= ymin) & (n <= ymax))
    out: dict[str, Any] = {
        "total": int(len(e)),
        "inside": int(inside.sum()),
        "inside_fraction": float(inside.mean()) if len(e) else 0.0,
        "world_box_m": [float(xmin), float(ymin), float(xmax), float(ymax)],
        "sample_box_m": ([float(e.min()), float(n.min()),
                          float(e.max()), float(n.max())]
                         if len(e) else None),
        "area_fraction": 0.0,
        "reach_m": float(reach_m),
    }
    if int(inside.sum()):
        from scipy.spatial import cKDTree
        nx = min(max_cells, max(2, grid.nx))
        ny = min(max_cells, max(2, grid.ny))
        xx, yy = np.meshgrid(np.linspace(xmin, xmax, nx),
                             np.linspace(ymin, ymax, ny))
        d, _ = cKDTree(np.column_stack([e, n])).query(
            np.column_stack([xx.ravel(), yy.ravel()]))
        out["area_fraction"] = float((d <= float(reach_m)).mean())
    return out


def coverage_warning(cov: Mapping[str, Any], kind: str = "recording"
                     ) -> str | None:
    """One sentence when most of the world is extrapolated rather than
    measured, else ``None``."""
    if cov["inside"] == 0:
        return None                       # that is an error, not a warning
    area = 100.0 * float(cov["area_fraction"])
    frac = float(cov["inside_fraction"])
    if (frac >= MIN_INSIDE_FRACTION and cov["area_fraction"] >= MIN_AREA_FRACTION):
        return None
    parts = [f"the {kind} reaches {area:.0f} % of the world area "
             f"(within {float(cov.get('reach_m', DEFAULT_REACH_M)):.0f} m of "
             "a sounding)"]
    if frac < MIN_INSIDE_FRACTION:
        parts.append(f"only {100.0 * frac:.0f} % of its samples land inside "
                     "the limits")
    parts.append("the rest is extrapolated from the edge slope")
    return "; ".join(parts)


def _box_text(box, anchor: GeoAnchor | None) -> str:
    x0, y0, x1, y1 = box
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    txt = f"{x1 - x0:.0f} x {y1 - y0:.0f} m"
    if anchor is not None:
        lat, lon = anchor.to_latlon(cx, cy)
        txt += f" around {format_latlon(float(lat), float(lon))}"
    return txt


def no_overlap_message(cov: Mapping[str, Any], anchor: GeoAnchor | None,
                       source_name: str, kind: str = "recording") -> str:
    """The actionable form of "nothing to interpolate": what the source
    covers, what the world covers, and how far apart the two are."""
    what = "the recording and the drawn rectangle" if kind == "recording" \
        else "the file and the drawn rectangle"
    msg = (f"no depth sample lands inside the world limits -- {what} do not "
           f"overlap. ")
    if not cov["total"] or cov["sample_box_m"] is None:
        return msg + (f"'{source_name}' carries no positioned depth sample at "
                      "all (no usable GPS fix in it).")
    wx0, wy0, wx1, wy1 = cov["world_box_m"]
    sx0, sy0, sx1, sy1 = cov["sample_box_m"]
    # "Near" = the gap between the two boxes is small against the world's
    # own size, so the recording is of this place and the rectangle is the
    # thing to move.
    gap = float(np.hypot(max(sx0 - wx1, wx0 - sx1, 0.0),
                         max(sy0 - wy1, wy0 - sy1, 0.0)))
    near = gap <= float(np.hypot(wx1 - wx0, wy1 - wy0))
    msg += f"'{source_name}' covers {_box_text(cov['sample_box_m'], anchor)}"
    if anchor is not None:
        slat, slon = anchor.to_latlon(0.5 * (sx0 + sx1), 0.5 * (sy0 + sy1))
        wlat, wlon = anchor.to_latlon(0.5 * (wx0 + wx1), 0.5 * (wy0 + wy1))
        dist, _, point = range_bearing(float(wlat), float(wlon),
                                       float(slat), float(slon))
        msg += (f", which is {format_distance(dist)} {point} of the world "
                f"area ({_box_text(cov['world_box_m'], anchor)})")
    else:
        msg += f", the world area is {_box_text(cov['world_box_m'], None)}"
    return msg + ". " + _advice(cov, kind, near)


def _advice(cov: Mapping[str, Any], kind: str, near: bool) -> str:
    """What to do about it -- a near miss is a rectangle problem, a far
    miss is a wrong-file (or wrong-path) problem."""
    if near:
        return (f"The {kind} passes close by but no sample lands inside: "
                "redraw the world limits (Screen 2) over the sailed track, "
                "or pick the path this area belongs to (Screen 1).")
    if kind == "recording":
        return ("Choose a recording of this area, or go back to Screen 1 and "
                "pick the path it was sailed on.")
    return ("Choose a depth file covering this area, or go back to Screen 1 "
            "and pick the path this file belongs to.")
