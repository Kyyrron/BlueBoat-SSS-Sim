"""Authored bathymetry: environment presets, depth files and sonar
recordings.

``DepthConfig`` is the world YAML's optional ``depth:`` section; when it
is present, ``build_height`` produces the *base* seabed height raster and
the legacy ``terrain:`` section contributes only texture (dunes,
roughness, materials) on top. When absent, terrain behaves exactly as it
always has (flat ``base_depth`` plus global slope). Three sources:
``preset`` (a procedural pattern), ``file`` (CSV / GeoTIFF soundings,
linear inside the data) and ``svlog`` (a recording's first-bottom-return
soundings, smoothed -- see :mod:`.svlog` and :mod:`.smooth`).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from ...core.geo import GeoAnchor
from ...core.types import GridSpec, Wall
from .coverage import (DEFAULT_REACH_M, coverage_warning,
                       no_overlap_message, sample_coverage)
from .ingest import DepthSamples, load_csv, load_geotiff
from .interpolate import (flood_fill_nearest, interpolate_to_grid,
                          rasterize_wall_barriers)
from .paint import (apply_depth_paint, normalize_strokes,
                    paint_material_ids, stroke_mask, strokes_to_samples)
from .presets import DEPTH_PRESETS
from .smooth import interpolate_smooth_to_grid
from .sources import base_height, check_coverage
from .svlog import SVLOG_KEYS, load_svlog, normalize_svlog_options

__all__ = ["DepthConfig", "build_height", "build_height_with_info",
           "DEPTH_PRESETS", "DepthSamples",
           "load_csv", "load_geotiff", "load_svlog", "interpolate_to_grid",
           "interpolate_smooth_to_grid", "normalize_svlog_options",
           "SVLOG_KEYS", "flood_fill_nearest", "rasterize_wall_barriers",
           "apply_depth_paint", "strokes_to_samples", "stroke_mask",
           "normalize_strokes", "paint_material_ids",
           "sample_coverage", "coverage_warning",
           "no_overlap_message", "base_height", "check_coverage"]

_MODES = ("preset", "file", "svlog")
_KEYS = {"mode", "preset", "params", "file", "file_kind", "file_values",
         "paint", "svlog"}


@dataclass
class DepthConfig:
    """The world config's ``depth:`` section."""

    mode: str = "preset"                  # "preset" | "file" | "svlog"
    preset: str = "harbour_basin"
    params: dict[str, Any] = field(default_factory=dict)
    file: str = ""                        # depth source (file / svlog modes)
    file_kind: str = ""                   # csv | geotiff; "" = from suffix
    file_values: str = "auto"             # auto | depth | elevation
    #: Depth brush strokes (``worldgen.depth.paint``), applied after the
    #: preset/file layer; empty = no paint (bit-identical to before).
    paint: list = field(default_factory=list)
    #: ``svlog`` mode knobs (:data:`.svlog.SVLOG_DEFAULTS`); validated
    #: and filled with defaults on construction.
    svlog: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.paint = normalize_strokes(self.paint, "depth")
        self.svlog = normalize_svlog_options(self.svlog)
        if self.mode not in _MODES:
            raise ValueError(f"depth.mode '{self.mode}' unknown {_MODES}")
        if self.mode == "preset" and self.preset not in DEPTH_PRESETS:
            raise KeyError(f"depth.preset '{self.preset}' unknown "
                           f"(available: {sorted(DEPTH_PRESETS)})")
        if self.mode in ("file", "svlog") and not self.file:
            raise ValueError(f"depth.mode '{self.mode}' needs depth.file")

    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "DepthConfig | None":
        if d is None:
            return None
        bad = set(d) - _KEYS
        if bad:
            raise KeyError(f"Unknown depth config key(s): {sorted(bad)} "
                           f"(accepted: {sorted(_KEYS)})")
        return cls(mode=str(d.get("mode", "preset")),
                   preset=str(d.get("preset", "harbour_basin")),
                   params=dict(d.get("params", {}) or {}),
                   file=str(d.get("file", "")),
                   file_kind=str(d.get("file_kind", "")),
                   file_values=str(d.get("file_values", "auto")),
                   paint=list(d.get("paint", []) or []),
                   svlog=dict(d.get("svlog", {}) or {}))

    def resolved_kind(self) -> str:
        if self.file_kind:
            return self.file_kind
        suffix = Path(self.file).suffix.lower()
        if suffix == ".svlog":
            return "svlog"
        return "geotiff" if suffix in (".tif", ".tiff") else "csv"


def build_height(grid: GridSpec, cfg: DepthConfig, walls: list[Wall],
                 anchor: GeoAnchor | None,
                 rng: np.random.Generator) -> np.ndarray:
    """Base seabed height [ny, nx] (z, negative down) from the config:
    the preset / file / recording layer, then the depth paint strokes."""
    return build_height_with_info(grid, cfg, walls, anchor, rng)[0]


def build_height_with_info(grid: GridSpec, cfg: DepthConfig,
                           walls: list[Wall], anchor: GeoAnchor | None,
                           rng: np.random.Generator
                           ) -> tuple[np.ndarray, dict[str, Any]]:
    """:func:`build_height` plus a source-dependent info dict -- the
    survey quality summary (and raw soundings) for ``svlog`` mode, empty
    for the others."""
    base, info, _ = base_height(grid, cfg, walls, anchor, rng)
    return apply_depth_paint(base, grid, cfg.paint, walls), info


def _base_height(grid: GridSpec, cfg: DepthConfig, walls: list[Wall],
                 anchor: GeoAnchor | None,
                 rng: np.random.Generator) -> tuple[np.ndarray, dict[str, Any]]:
    """Historical name: the source's height and info (see ``sources``)."""
    h, info, _ = base_height(grid, cfg, walls, anchor, rng)
    return h, info


_check_coverage = check_coverage
