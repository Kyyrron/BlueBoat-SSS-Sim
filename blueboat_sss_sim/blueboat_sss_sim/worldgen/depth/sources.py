"""The three legacy depth sources -- preset, depth file, recording -- as
one function, shared by ``build_height`` and the ``source`` layer kind.

Moved out of the package ``__init__`` so the layer editor can call it
without an import cycle; behaviour is byte-for-byte the historical one,
plus the sample positions a positioned source contributes (the support
of a survey, used for coverage masks).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ...core.geo import GeoAnchor
from ...core.types import GridSpec, Wall
from .coverage import (DEFAULT_REACH_M, coverage_warning, no_overlap_message,
                       sample_coverage)
from .ingest import DepthSamples, load_csv, load_geotiff
from .interpolate import interpolate_to_grid
from .presets import DEPTH_PRESETS
from .smooth import interpolate_smooth_to_grid
from .svlog import load_svlog, load_relief


def base_height(grid: GridSpec, cfg: "DepthConfig", walls: list[Wall],
                anchor: GeoAnchor | None, rng: np.random.Generator
                ) -> tuple[np.ndarray, dict[str, Any], np.ndarray | None]:
    """``(height, info, samples_xy)``: the source's height raster, its
    info dict (survey quality / coverage) and, for a positioned source,
    the (N, 2) sample positions in grid metres (``None`` for a preset)."""
    if cfg.mode == "preset":
        fn, _, _ = DEPTH_PRESETS[cfg.preset]
        return fn(grid, cfg.params, rng), {}, None
    if anchor is None:
        raise ValueError(
            f"depth.mode '{cfg.mode}' needs a geo anchor (the soundings' "
            "lat/lon must land somewhere): give the world a geo: section")
    kind = cfg.resolved_kind()
    if cfg.mode == "svlog" or kind == "svlog":
        if cfg.mode != "svlog":
            raise ValueError("a .svlog depth source needs depth.mode 'svlog'")
        samples, info = load_svlog(cfg.file, anchor, cfg.svlog)
        check_coverage(samples, grid, anchor, cfg.file, "recording", info,
                       reach_m=cfg.svlog["trend_length_m"])
        o = cfg.svlog
        relief_smp, relief_gain = None, 1.0
        if o["relief"] == "shading":
            relief_smp, rel_q = load_relief(cfg.file, anchor, o,
                                            soundings=info.get("soundings_arrays"))
            info["relief"] = rel_q
            relief_gain = float(relief_smp.gain)
        height = interpolate_smooth_to_grid(
            samples.east, samples.north, samples.depth, grid, walls,
            smoothing=o["smoothing"], trend_length_m=o["trend_length_m"],
            texture_gain=(0.0 if o["relief"] == "none" else o["texture_gain"]),
            rng=rng, relief=relief_smp, relief_gain=relief_gain,
            relief_max_m=o["relief_max_m"],
            relief_smooth_m=o["relief_smooth_m"])
        return height, info, np.column_stack([samples.east, samples.north])
    if kind == "geotiff":
        samples = load_geotiff(cfg.file, anchor, cfg.file_values, grid=grid)
    elif kind == "csv":
        samples = load_csv(cfg.file, anchor, cfg.file_values)
    else:
        raise ValueError(f"depth.file_kind '{kind}' unknown (csv | geotiff)")
    info: dict[str, Any] = {}
    check_coverage(samples, grid, anchor, cfg.file, "file", info)
    xy = (None if samples.aligned
          else np.column_stack([samples.east, samples.north]))
    return interpolate_to_grid(samples, grid, walls), info, xy


def check_coverage(samples: DepthSamples, grid: GridSpec,
                   anchor: GeoAnchor | None, file: str, kind: str,
                   info: dict[str, Any],
                   reach_m: float = DEFAULT_REACH_M) -> None:
    """Refuse a source that never reaches the world, and record how much
    of the world it did reach. The fitters raise on an empty overlap too;
    this runs first so the message can name the two places. A raster
    aligned with the grid (a baked layer stack) covers it by construction."""
    if getattr(samples, "aligned", False):
        xmin, ymin, xmax, ymax = grid.extent
        info["coverage"] = {"total": int(len(samples)),
                            "inside": int(len(samples)),
                            "inside_fraction": 1.0,
                            "world_box_m": [float(xmin), float(ymin),
                                            float(xmax), float(ymax)],
                            "sample_box_m": [float(xmin), float(ymin),
                                             float(xmax), float(ymax)],
                            "area_fraction": 1.0, "reach_m": float(reach_m)}
        return
    cov = sample_coverage(samples.east, samples.north, grid, reach_m=reach_m)
    if not cov["inside"]:
        raise ValueError(no_overlap_message(cov, anchor, Path(file).name,
                                            kind))
    info["coverage"] = cov
    warn = coverage_warning(cov, kind)
    if warn:
        info["warning"] = warn
