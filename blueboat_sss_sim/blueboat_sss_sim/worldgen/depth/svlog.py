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
              "smoothing", "cell_m", "backscatter_layer", "relief",
              "relief_gain", "relief_max_m",
              "relief_smooth_m"}
RELIEF_MODES = ("synthetic", "shading", "none")
SVLOG_DEFAULTS: dict[str, Any] = {
    "datum_offset_m": 0.0,      # tide / draft: + when water stood higher
    "trend_length_m": 15.0,     # damped-trend saturation distance
    "texture_gain": 0.5,        # undulation RMS / measured residual RMS
    "smoothing": 1.0,           # spline penalty (0 = exact)
    "cell_m": None,             # decimation cell; None = auto (<= 2500)
    # Relief between the lines: "synthetic" = the seeded undulation above
    # (every older config, bit-identically), "shading" = the relief the
    # recording's own shading measures across the swath
    # (analysis/shading.py), fused over the spline, "none" = flat.
    "relief": "synthetic",
    "relief_gain": 1.0,         # shading amplitude; 0 = calibrate on the
                                # tie lines' FBR soundings (Ran et al. 2025)
    "relief_max_m": 1.5,        # clamp of the shading relief
    "relief_smooth_m": 0.75,    # coverage-weighted blend of the
                                # assembled relief: bridges each
                                # pass's nadir hole and the seams
                                # between passes (keeps 90 % of a
                                # 10 m ripple)
    "backscatter_layer": False, # the builder's reflectivity_survey layer
                                # (kept for older states; the layer is
                                # what applies the measurement)
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
    out["relief"] = str(out["relief"])
    out["relief_gain"] = float(out["relief_gain"])
    out["relief_max_m"] = float(out["relief_max_m"])
    out["relief_smooth_m"] = float(out["relief_smooth_m"])
    if out["trend_length_m"] <= 0.0:
        raise ValueError("depth.svlog.trend_length_m must be > 0")
    if out["texture_gain"] < 0.0 or out["smoothing"] < 0.0:
        raise ValueError("depth.svlog.texture_gain / smoothing must be >= 0")
    if out["relief"] not in RELIEF_MODES:
        raise ValueError(f"depth.svlog.relief '{out['relief']}' not in "
                         f"{RELIEF_MODES}")
    if out["relief_smooth_m"] < 0.0:
        raise ValueError("depth.svlog.relief_smooth_m must be >= 0")
    if out["relief_gain"] < 0.0 or out["relief_max_m"] <= 0.0:
        raise ValueError("depth.svlog.relief_gain must be >= 0 (0 = "
                         "calibrate) and relief_max_m > 0")
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


def load_relief(path: str | Path, anchor: GeoAnchor, opts: Mapping[str, Any],
                soundings: Mapping[str, Any] | None = None
                ) -> tuple[Any, dict[str, Any]]:
    """The shading relief of a recording (``analysis.shading``) with its
    gain resolved -- calibrated on the tie soundings when
    ``relief_gain`` is 0 -- and its quality block for the metadata."""
    from ...analysis.shading import (calibrate_gain, relief_quality,
                                     relief_samples)  # deferred: see module doc
    o = normalize_svlog_options(opts)
    smp = relief_samples(path, anchor, trend_length_m=o["trend_length_m"],
                         relief_max_m=o["relief_max_m"],
                         datum_offset_m=o["datum_offset_m"])
    gain, cal = o["relief_gain"], {"gain_mode": "set"}
    if gain <= 0.0:
        gain, cal = calibrate_gain(smp, soundings,
                                   trend_length_m=o["trend_length_m"])
    smp.gain = float(gain)
    q = relief_quality(smp, soundings, reach_m=o["trend_length_m"])
    q["gain"] = float(gain)
    q.update(cal)
    return smp, q

