"""``source`` -- the base of every stack: today's preset / depth file /
recording, unchanged. Its ``params`` is the legacy ``depth:`` dict
(``mode``, ``preset``, ``params``, ``file``, ``file_kind``,
``file_values``, ``svlog``); strokes are not a source key -- they live in
a ``paint`` layer. A positioned source placed *above* another layer gets a
coverage mask by default: full weight within the survey's reach, fading
to nothing at twice that distance, so what lies beneath shows through
where the survey never went.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .. import geometry as geo
from ..coverage import DEFAULT_REACH_M
from . import LayerContext, LayerOutput, LayerSpec, register

SOURCE_KEYS = ("mode", "preset", "params", "file", "file_kind",
               "file_values", "svlog")


def validate_source_params(params: Mapping[str, Any]) -> dict[str, Any]:
    from .. import DepthConfig

    p = dict(params or {})
    if "paint" in p:
        raise KeyError("a source layer takes no 'paint': depth strokes live "
                       "in a paint layer above it")
    cfg = DepthConfig.from_dict(p)
    return {"mode": cfg.mode, "preset": cfg.preset, "params": dict(cfg.params),
            "file": cfg.file, "file_kind": cfg.file_kind,
            "file_values": cfg.file_values, "svlog": dict(cfg.svlog)}


def source_fn(grid, geometry, params, rng: np.random.Generator,
              ctx: LayerContext) -> LayerOutput:
    from .. import DepthConfig
    from ..sources import base_height

    cfg = DepthConfig.from_dict(params)
    height, info, xy = base_height(grid, cfg, ctx.walls, ctx.anchor, rng)
    reach = (float(cfg.svlog["trend_length_m"]) if cfg.mode == "svlog"
             else DEFAULT_REACH_M)
    ctx.samples_xy, ctx.reach_m = xy, reach
    ctx.svlog_path = cfg.file if cfg.mode == "svlog" else None
    default = None
    if not ctx.is_base and xy is not None and len(xy):
        d = geo.coverage_distance(grid, xy)
        default = geo.band_weight(d, reach, reach)
    return LayerOutput(height=height, default_mask=default, info=info)


register(LayerSpec(
    kind="source", fn=source_fn, label="Depth source", family="source",
    schema=(), geometry=(), channels=("height",),
    reference="the preset, file and recording sources of "
              "docs/configuration_guide.md",
    consequence="The seabed everything else is laid over.",
    validate_params=validate_source_params))
