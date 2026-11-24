"""``adjust`` -- an adjustment layer: no shape of its own, it acts on the
composite beneath it within its mask (offset, scale about a datum, clamp,
smooth, curve, noise, or match the edge of a patch to its surroundings).
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from ...noise import fbm
from . import LayerContext, LayerOutput, LayerSpec, register

_SCHEMA = (
    ("op", "Operation",
     "choice:offset|scale|clamp|smooth|curve|noise|match_edge", "offset", 0, 0,
     "What to do to the bed under the mask: shift it, scale it about a datum, "
     "clamp its depth, smooth it, bend its depth curve, roughen it, or fit its "
     "edge to the surroundings."),
    ("offset_m", "Offset [m] (positive = deeper)", "float", 0.5, -20.0, 20.0,
     "Metres of depth added under the mask."),
    ("scale", "Scale factor", "float", 1.2, 0.05, 10.0,
     "Multiplies the depth measured from the datum."),
    ("datum_m", "Datum depth [m]", "float", 0.0, -5.0, 60.0,
     "Depth about which the scale operation stretches the bed."),
    ("min_depth_m", "Minimum depth [m]", "float", 0.5, -5.0, 60.0,
     "Clamp: the bed is never shallower than this under the mask."),
    ("max_depth_m", "Maximum depth [m]", "float", 10.0, -5.0, 60.0,
     "Clamp: the bed is never deeper than this under the mask."),
    ("sigma_m", "Smoothing radius [m]", "float", 3.0, 0.1, 100.0,
     "Blur radius of the smooth operation."),
    ("gamma", "Curve exponent", "float", 1.0, 0.2, 5.0,
     "Curve: depths between the area's shallowest and deepest points are "
     "remapped with this exponent (below 1 lifts the middle, above 1 sinks it)."),
    ("amplitude_m", "Noise amplitude [m]", "float", 0.1, 0.0, 5.0,
     "Height of the added random relief."),
    ("cells", "Noise cells", "int", 12, 2, 64,
     "Number of noise blobs across the area: more means finer relief."),
    ("ring_m", "Edge ring width [m]", "float", 3.0, 0.5, 50.0,
     "Match edge: width of the band along the mask edge that is compared "
     "with the bed just outside it."),
)


def match_edge(H: np.ndarray, mask: np.ndarray | None, grid, ring_m: float
               ) -> np.ndarray:
    """Subtract, inside the mask, the plane that best fits the height
    mismatch between the band just inside the mask edge and the bed just
    outside it -- closing the seam between an authored patch and its
    surroundings with an offset and a tilt."""
    if mask is None:
        return H
    inside = mask > 0.5
    if not inside.any() or inside.all():
        return H
    k = max(int(round(ring_m / grid.resolution)), 1)
    ring_in = inside & ~ndimage.binary_erosion(inside, iterations=k)
    outside = ~inside
    _, (jy, jx) = ndimage.distance_transform_edt(inside, return_indices=True)
    H_out = H[jy, jx]                    # nearest outside value for every cell
    diff = (H - H_out)[ring_in]
    jj, ii = np.nonzero(ring_in)
    x = grid.origin_x + ii * grid.resolution
    y = grid.origin_y + jj * grid.resolution
    A = np.column_stack([np.ones(len(x)), x - x.mean(), y - y.mean()])
    coef, *_ = np.linalg.lstsq(A, diff, rcond=None)
    xx = grid.origin_x + np.arange(grid.nx) * grid.resolution
    yy = grid.origin_y + np.arange(grid.ny) * grid.resolution
    X, Y = np.meshgrid(xx, yy)
    plane = coef[0] + coef[1] * (X - x.mean()) + coef[2] * (Y - y.mean())
    del outside
    return H - plane


def adjust_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    H = ctx.height
    if H is None:
        raise ValueError("an adjustment layer needs a source below it")
    op = p["op"]
    if op == "offset":
        return LayerOutput(delta=np.full(H.shape, -p["offset_m"]))
    if op == "scale":
        d = -p["datum_m"]
        return LayerOutput(height=d + (H - d) * p["scale"])
    if op == "clamp":
        return LayerOutput(height=np.clip(H, -p["max_depth_m"], -p["min_depth_m"]))
    if op == "smooth":
        return LayerOutput(height=ndimage.gaussian_filter(
            H, max(p["sigma_m"] / grid.resolution, 0.5)))
    if op == "curve":
        lo, hi = float(H.min()), float(H.max())
        if hi - lo < 1e-9:
            return LayerOutput(height=H.copy())
        u = (H - lo) / (hi - lo)
        return LayerOutput(height=lo + (hi - lo) * u ** p["gamma"])
    if op == "noise":
        return LayerOutput(delta=p["amplitude_m"]
                           * fbm(H.shape, p["cells"], 3, rng))
    return LayerOutput(height=match_edge(H, ctx.mask, grid, p["ring_m"]))


register(LayerSpec(
    kind="adjust", fn=adjust_fn, label="Adjustment", family="adjust",
    schema=_SCHEMA, geometry=(), channels=("height",),
    reference="docs/seabed_model.md (editing operations)",
    consequence="Changes the bed under the selection without adding a "
                "shape of its own.",
    blend="normal"))
