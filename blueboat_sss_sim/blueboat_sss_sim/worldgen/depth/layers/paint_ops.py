"""``paint`` -- brush strokes as a layer, with an operation.

``set`` is today's depth brush (exact depth inside the radius, C1 feather
outside, union rule between strokes, walls respected) and compiles
bit-identically to ``apply_depth_paint``. The other operations use the
same stroke weight: ``raise`` / ``lower`` shift the bed by ``amount_m``,
``smooth`` blurs it under the brush, ``flatten`` pulls it toward the mean
depth under the brush, ``noise`` adds seeded relief.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from ...noise import fbm
from ..paint import _stroke_weight, apply_depth_paint
from . import LayerContext, LayerOutput, LayerSpec, register

_SCHEMA = (
    ("op", "Operation", "choice:set|raise|lower|smooth|flatten|noise", "set",
     0, 0, "What the brush does: set the painted depth, raise or lower the "
     "bed, smooth it, flatten it toward its mean, or roughen it."),
    ("amount_m", "Amount [m]", "float", 0.5, 0.0, 20.0,
     "Metres to raise or lower the bed, or the noise height, under the brush."),
    ("sigma_m", "Smoothing radius [m]", "float", 2.0, 0.1, 50.0,
     "Blur radius of the smooth operation."),
    ("cells", "Noise cells", "int", 12, 2, 64,
     "Number of noise blobs across the area for the noise operation: more "
     "means finer relief."),
)


def strokes_weight(strokes, grid) -> np.ndarray:
    """The union weight of a stroke list (erase strokes cut it), the same
    rule ``apply_depth_paint`` uses."""
    w = np.zeros((grid.ny, grid.nx))
    for s in strokes:
        fe = max(float(s["feather_m"]), 2.0 * grid.resolution)
        ws = _stroke_weight(s, grid, fe)
        w = w * (1.0 - ws) if s["erase"] else 1.0 - (1.0 - w) * (1.0 - ws)
    return w


def paint_fn(grid, geometry, p, rng: np.random.Generator,
             ctx: LayerContext) -> LayerOutput:
    strokes = geometry["strokes"]
    H = ctx.height
    if H is None:
        raise ValueError("a paint layer needs a source below it")
    if p["op"] == "set":
        return LayerOutput(height=apply_depth_paint(H, grid, strokes, ctx.walls))
    W = strokes_weight(strokes, grid)
    if p["op"] in ("raise", "lower"):
        sign = 1.0 if p["op"] == "raise" else -1.0
        return LayerOutput(delta=sign * p["amount_m"] * W)
    if p["op"] == "smooth":
        X = ndimage.gaussian_filter(H, max(p["sigma_m"] / grid.resolution, 0.5))
        return LayerOutput(height=H + (X - H) * W)
    if p["op"] == "flatten":
        sel = W > 0.5
        mean = float(H[sel].mean()) if sel.any() else float(H.mean())
        return LayerOutput(height=H + (mean - H) * W)
    noise = fbm((grid.ny, grid.nx), p["cells"], 3, rng)
    return LayerOutput(delta=p["amount_m"] * noise * W)


register(LayerSpec(
    kind="paint", fn=paint_fn, label="Brush strokes", family="paint",
    schema=_SCHEMA, geometry=("strokes",), channels=("height",),
    reference="docs/configuration_guide.md 'Painting'",
    consequence="Local hand edits: an exact depth, a lift or a cut, a "
                "smoothed patch, or added roughness under the brush."))
