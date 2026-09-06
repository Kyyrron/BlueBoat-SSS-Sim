"""``depth.mode: svlog`` -- the World Builder's "depth from a sonar
recording" source.

Thin adapter between the world config and
:mod:`blueboat_sss_sim.analysis.bathymetry` (FBR soundings) plus
:mod:`.smooth` (the surface). The bathymetry import is **deferred into
the function**: ``analysis/__init__`` imports ``metrics``, which imports
``worldgen.scene``, which imports this package -- a module-level import
here would be a cycle.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np

from ...core.geo import GeoAnchor
from .ingest import DepthSamples

SVLOG_KEYS = {"datum_offset_m", "trend_length_m", "texture_gain",
              "smoothing", "cell_m", "backscatter_layer"}
SVLOG_DEFAULTS: dict[str, Any] = {
    "datum_offset_m": 0.0,      # tide / draft: + when water stood higher
    "trend_length_m": 15.0,     # damped-trend saturation distance
    "texture_gain": 0.5,        # undulation RMS / measured residual RMS
    "smoothing": 1.0,           # spline penalty (0 = exact)
    "cell_m": None,             # decimation cell; None = auto (<= 2500)
    "backscatter_layer": False, # also derive the reflectivity hint
}


def normalize_svlog_options(d: Mapping[str, Any] | None) -> dict[str, Any]:
    """Defaults filled, unknown keys raise by name, numbers coerced."""
    d = dict(d or {})
    bad = set(d) - SVLOG_KEYS
    if bad:
        raise KeyError(f"Unknown depth.svlog key(s): {sorted(bad)} "
                       f"(accepted: {sorted(SVLOG_KEYS)})")
    out = dict(SVLOG_DEFAULTS)
    out.update(d)
    for k in ("datum_offset_m", "trend_length_m", "texture_gain", "smoothing"):
        out[k] = float(out[k])
    out["cell_m"] = None if out["cell_m"] in (None, 0, "") else float(out["cell_m"])
    out["backscatter_layer"] = bool(out["backscatter_layer"])
    if out["trend_length_m"] <= 0.0:
        raise ValueError("depth.svlog.trend_length_m must be > 0")
    if out["texture_gain"] < 0.0 or out["smoothing"] < 0.0:
        raise ValueError("depth.svlog.texture_gain / smoothing must be >= 0")
    return out


def load_svlog(path: str | Path, anchor: GeoAnchor, opts: Mapping[str, Any]
               ) -> tuple[DepthSamples, dict[str, Any]]:
    """The decimated soundings of a recording about ``anchor`` plus a
    survey-quality dict (also carrying the raw soundings for the folder's
    ``depth_samples.npz``)."""
    from ...analysis.bathymetry import survey_track   # deferred: see module doc
    o = normalize_svlog_options(opts)
    res = survey_track(path, anchor, datum_offset_m=o["datum_offset_m"],
                       cell_m=o["cell_m"])
    s = res.soundings
    info = dict(res.summary)
    info["soundings_arrays"] = {
        "east": s.east, "north": s.north, "depth": s.depth, "t_ms": s.t_ms,
        "quality": s.quality, "lat": s.lat, "lon": s.lon,
        "cell_east": res.cells.east, "cell_north": res.cells.north,
        "cell_depth": res.cells.depth, "cell_count": res.cells.count,
    }
    return (DepthSamples(res.cells.east, res.cells.north, res.cells.depth),
            info)


def backscatter_hint(path: str | Path, anchor: GeoAnchor, grid,
                     cell_m: float = 2.0) -> dict[str, Any]:
    """A 0-1 reflectivity-hint raster (``values``, origin, cell) on a
    coarse lattice over ``grid`` from the recording's range-equalised
    mid-swath backscatter; NaN where nothing was insonified."""
    from scipy import ndimage

    from ...analysis.bathymetry import backscatter_samples
    e, n, lvl = backscatter_samples(path, anchor)
    xmin, ymin, xmax, ymax = grid.extent
    nx = int(np.ceil((xmax - xmin) / cell_m)) + 1
    ny = int(np.ceil((ymax - ymin) / cell_m)) + 1
    acc = np.zeros((ny, nx))
    cnt = np.zeros((ny, nx))
    ci = np.round((e - xmin) / cell_m).astype(int)
    cj = np.round((n - ymin) / cell_m).astype(int)
    ok = (ci >= 0) & (ci < nx) & (cj >= 0) & (cj < ny) & np.isfinite(lvl)
    np.add.at(acc, (cj[ok], ci[ok]), lvl[ok])
    np.add.at(cnt, (cj[ok], ci[ok]), 1.0)
    mean = np.where(cnt > 0, acc / np.maximum(cnt, 1), np.nan)
    vals = mean[np.isfinite(mean)]
    if len(vals) < 4:
        return {"values": np.full((ny, nx), np.nan), "origin": (xmin, ymin),
                "cell_m": cell_m, "n": int(ok.sum())}
    lo, hi = np.percentile(vals, 5), np.percentile(vals, 95)
    norm = np.clip((mean - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    filled = np.where(np.isfinite(norm), norm, 0.0)
    w = np.isfinite(norm).astype(float)
    sm = ndimage.gaussian_filter(filled, 1.0) / np.maximum(
        ndimage.gaussian_filter(w, 1.0), 1e-6)
    out = np.where(ndimage.gaussian_filter(w, 1.0) > 0.05, sm, np.nan)
    return {"values": out, "origin": (xmin, ymin), "cell_m": cell_m,
            "n": int(ok.sum())}
