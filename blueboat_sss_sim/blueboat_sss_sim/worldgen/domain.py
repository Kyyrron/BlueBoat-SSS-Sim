"""The depth domain of the harbour sensor (acceptance criterion C1) and the
survey water of a world (2026-11-15, completion round session 02).

The production sensor (``raycast_harbour_fast``) is valid for a sonar
altitude of 4.5 to 11.3 m. A world is *in domain* when at least 95 % of its
survey water lies in that band:

* **survey water**: the wet cells that the boat can reach from the spawn,
  world (0, 0), without crossing a quay (a pontoon floats: the boat and the
  sound pass under it, so it is no barrier here);
* **altitude** = water depth - transducer depth -/+ a heave margin (the
  transducer depth is the sonar model's own, ``SonarModelConfig.
  sensor_depth_m``: one source).

ROS-free, Qt-free; numpy and scipy.ndimage.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

from .objects import DRY_HEIGHT_M

#: the production sensor's altitude band [m] (acceptance criterion C1)
ALTITUDE_MIN_M = 4.5
ALTITUDE_MAX_M = 11.3
#: share of the survey water that must lie in the band
IN_DOMAIN_SHARE = 0.95
#: heave margin [m] [DESIGN, bracket 0.05-0.20: about 1.5 x the heave
#: amplitude of a 0.07 m harbour chop]
HEAVE_MARGIN_M = 0.10


def sensor_depth_m() -> float:
    """The transducer's depth under the surface: the sonar model's default."""
    from ..sonar.config import SonarModelConfig
    return float(SonarModelConfig().sensor_depth_m)


def _segment_mask(grid, x0, y0, x1, y1, half_width: float) -> np.ndarray:
    """Cells within ``half_width`` of the segment (a window around it)."""
    res = float(grid.resolution)
    xmin, ymin, _, _ = grid.extent
    pad = half_width + res
    i0 = int(np.clip(np.floor((min(x0, x1) - pad - xmin) / res), 0, grid.nx - 1))
    i1 = int(np.clip(np.ceil((max(x0, x1) + pad - xmin) / res), 0, grid.nx - 1))
    j0 = int(np.clip(np.floor((min(y0, y1) - pad - ymin) / res), 0, grid.ny - 1))
    j1 = int(np.clip(np.ceil((max(y0, y1) + pad - ymin) / res), 0, grid.ny - 1))
    out = np.zeros((grid.ny, grid.nx), dtype=bool)
    if i1 < i0 or j1 < j0:
        return out
    x = xmin + (np.arange(i0, i1 + 1) + 0.5) * res
    y = ymin + (np.arange(j0, j1 + 1) + 0.5) * res
    X, Y = np.meshgrid(x, y)
    ax, ay = x1 - x0, y1 - y0
    t = np.clip(((X - x0) * ax + (Y - y0) * ay) / max(ax * ax + ay * ay, 1e-12), 0.0, 1.0)
    d = np.hypot(X - (x0 + t * ax), Y - (y0 + t * ay))
    out[j0:j1 + 1, i0:i1 + 1] = d <= half_width
    return out


def wall_mask(grid, walls: Sequence[Any], kinds=("quay",), margin_m: float = 0.0) -> np.ndarray:
    """Cells within half a wall's thickness (+ ``margin_m``) of a wall of
    one of ``kinds``."""
    out = np.zeros((grid.ny, grid.nx), dtype=bool)
    for w in walls:
        g = (lambda k, d=None: w.get(k, d)) if isinstance(w, dict) else (lambda k, d=None: getattr(w, k, d))
        if str(g("kind", "quay")) not in kinds:
            continue
        # at least one cell wide, so that the barrier is closed on the grid's diagonals
        hw = max(0.5 * float(g("thickness", 0.3)), 0.75 * float(grid.resolution)) + float(margin_m)
        out |= _segment_mask(grid, float(g("x0")), float(g("y0")), float(g("x1")), float(g("y1")), hw)
    return out


def survey_water(height: np.ndarray, grid, walls: Sequence[Any],
                 spawn: tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """Wet cells connected to the spawn without crossing a quay. When the
    spawn itself is not on a wet cell (a hand-made world), the largest wet
    region."""
    from scipy import ndimage
    wet = (np.asarray(height) < DRY_HEIGHT_M) & ~wall_mask(grid, walls, ("quay",))
    lab, n = ndimage.label(wet)
    if n == 0:
        return np.zeros_like(wet)
    xmin, ymin, _, _ = grid.extent
    i = int(np.clip((spawn[0] - xmin) / grid.resolution, 0, grid.nx - 1))
    j = int(np.clip((spawn[1] - ymin) / grid.resolution, 0, grid.ny - 1))
    k = int(lab[j, i])
    if k == 0:
        k = int(np.argmax(np.bincount(lab.ravel())[1:])) + 1
    return lab == k


#: half-width of the strip along the source path that the path's own survey
#: looks at [m]: the production sensor's range
PATH_SWATH_M = 20.0


def domain_report(height: np.ndarray, grid, walls: Sequence[Any], *,
                  spawn: tuple[float, float] = (0.0, 0.0),
                  heave_margin_m: float = HEAVE_MARGIN_M,
                  transducer_depth_m: float | None = None,
                  path_xy=None) -> tuple[dict, dict]:
    """``(report, rasters)`` of a terrain ``height`` (z, negative down, before
    any object is stamped). Rasters: ``survey_area`` and ``in_domain`` (bool,
    the world grid). The report is what ``metadata.yaml`` stores under
    ``domain``. ``path_xy`` (the source path, world metres): also the share
    in the band among the survey water within :data:`PATH_SWATH_M` of the
    path -- what that path's survey insonifies (a hand-made world of a real
    harbour holds much water the path never goes near)."""
    zt = sensor_depth_m() if transducer_depth_m is None else float(transducer_depth_m)
    water = survey_water(height, grid, walls, spawn)
    alt = -np.asarray(height, dtype=np.float64) - zt
    shallow = water & (alt - heave_margin_m < ALTITUDE_MIN_M)
    deep = water & (alt + heave_margin_m > ALTITUDE_MAX_M)
    inside = water & ~shallow & ~deep
    n = max(int(water.sum()), 1)
    share = float(inside.sum()) / n
    a = alt[water] if water.any() else np.zeros(1)
    report = {
        "altitude_band_m": [ALTITUDE_MIN_M, ALTITUDE_MAX_M],
        "transducer_depth_m": round(zt, 3), "heave_margin_m": round(float(heave_margin_m), 3),
        "survey_area_m2": round(float(water.sum()) * float(grid.resolution) ** 2, 1),
        "share_in_domain": round(share, 4),
        "share_too_shallow": round(float(shallow.sum()) / n, 4),
        "share_too_deep": round(float(deep.sum()) / n, 4),
        "altitude_min_m": round(float(a.min()), 2), "altitude_median_m": round(float(np.median(a)), 2),
        "altitude_max_m": round(float(a.max()), 2),
        "required_share": IN_DOMAIN_SHARE,
        "in_domain": bool(share >= IN_DOMAIN_SHARE),
    }
    if path_xy is not None and len(path_xy) >= 2:
        from scipy import ndimage
        pts = np.asarray(path_xy, dtype=np.float64).reshape(-1, 2)
        xmin, ymin, _, _ = grid.extent
        res = float(grid.resolution)
        # densify so that successive points are at most a cell apart
        seg = np.hypot(*np.diff(pts, axis=0).T)
        nn = np.maximum(np.ceil(seg / res).astype(int), 1)
        fine = np.concatenate([np.linspace(pts[i], pts[i + 1], nn[i], endpoint=False)
                               for i in range(len(seg))] + [pts[-1:]])
        on = np.ones((grid.ny, grid.nx), dtype=bool)
        i = np.clip(((fine[:, 0] - xmin) / res).astype(int), 0, grid.nx - 1)
        j = np.clip(((fine[:, 1] - ymin) / res).astype(int), 0, grid.ny - 1)
        on[j, i] = False
        near = water & (ndimage.distance_transform_edt(on) * res <= PATH_SWATH_M)
        m = max(int(near.sum()), 1)
        report["path_swath_m"] = PATH_SWATH_M
        report["share_in_domain_path_swath"] = round(float((inside & near).sum()) / m, 4)
        report["in_domain_path_swath"] = bool(report["share_in_domain_path_swath"] >= IN_DOMAIN_SHARE)
    return report, {"survey_area": water, "in_domain": inside}
