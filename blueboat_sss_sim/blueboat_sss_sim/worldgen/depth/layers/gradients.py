"""``gradient`` -- a depth trend with a real profile.

Drawn as an axis (from a to b), a radial (centre and radius) or a
distance from a polyline (a shoreline, a quay line, a bank), the depth
runs from ``depth_start`` to ``depth_end`` along ``linear``,
``smoothstep``, ``power``, damped ``exp`` or Dean's ``h = A x^(2/3)``
profiles. The Dean profile from a drawn shoreline is the beach
(Bruun 1954; Dean 1991), with ``depth_end`` as its closure-depth cap
(Hallermeier 1980); the exponential is the damped trend a recording's
edge continues with.
"""

from __future__ import annotations

import numpy as np

from .. import geometry as geo
from ..paint import smoothstep
from . import LayerContext, LayerOutput, LayerSpec, register

_SCHEMA = (
    ("profile", "Profile", "choice:linear|smoothstep|power|exp|dean", "linear",
     0, 0, "Shape of the depth change along the gradient: straight, eased, "
     "a power law, a damped exponential, or Dean's x^(2/3) beach curve."),
    ("depth_start", "Depth at the start [m]", "float", 0.5, -5.0, 60.0,
     "Water depth where the gradient begins (at the line, the axis start or "
     "the centre); zero or negative is dry land."),
    ("depth_end", "Depth at the end [m]", "float", 6.0, -5.0, 60.0,
     "Water depth reached at the end of the gradient and held beyond it; "
     "for the Dean profile it is the closure-depth cap."),
    ("length_m", "Gradient length [m]", "float", 50.0, 0.5, 5000.0,
     "Distance over which the depth changes (an axis gradient uses the axis "
     "length instead)."),
    ("dean_A", "Dean A [m^(1/3)]", "float", 0.10, 0.02, 0.40,
     "Steepness of the Dean profile: 0.10 is fine sand, 0.20 coarse sand or "
     "gravel."),
    ("exp_length_m", "Exponential length [m]", "float", 15.0, 0.5, 1000.0,
     "Distance over which an exponential gradient completes 63 % of its "
     "depth change."),
    ("power_exp", "Power exponent", "float", 1.0, 0.2, 5.0,
     "Exponent of the power profile: below 1 deepens fast at first, above 1 "
     "deepens late."),
    ("side", "Side of the line", "choice:both|left|right", "both", 0, 0,
     "Which side of a drawn line or axis the gradient applies to; the other "
     "side is left untouched."),
)


def profile_depth(p: dict, d: np.ndarray, length: float) -> np.ndarray:
    """Depth (positive down) at distance ``d`` along the profile."""
    d0, d1 = p["depth_start"], p["depth_end"]
    d = np.maximum(d, 0.0)
    if p["profile"] == "dean":
        depth = d0 + p["dean_A"] * d ** (2.0 / 3.0)
        return np.minimum(depth, d1) if d1 > d0 else depth
    if p["profile"] == "exp":
        return d1 + (d0 - d1) * np.exp(-d / max(p["exp_length_m"], 1e-6))
    u = np.clip(d / max(length, 1e-6), 0.0, 1.0)
    if p["profile"] == "smoothstep":
        u = smoothstep(u)
    elif p["profile"] == "power":
        u = u ** p["power_exp"]
    return d0 + (d1 - d0) * u


def gradient_fn(grid, g, p, rng: np.random.Generator,
                ctx: LayerContext) -> LayerOutput:
    side = None
    if g["type"] == "axis":
        along, across, length = geo.axis_coords(grid, g["a"], g["b"])
        d, side = along, across
    elif g["type"] == "radial":
        d, length = geo.radial_distance(grid, g["centre"]), g["radius_m"]
    else:
        xmin, ymin, xmax, ymax = grid.extent
        reach = float(np.hypot(xmax - xmin, ymax - ymin)) + 1.0
        fr = geo.polyline_frame(g["points"], grid, reach)
        d, side, length = fr.dist, fr.n, p["length_m"]
    depth = profile_depth(p, d, length)
    default = None
    if p["side"] != "both" and side is not None:
        default = ((side >= 0.0) if p["side"] == "left"
                   else (side <= 0.0)).astype(np.float64)
    return LayerOutput(height=-depth, default_mask=default)


register(LayerSpec(
    kind="gradient", fn=gradient_fn, label="Depth gradient", family="gradient",
    schema=_SCHEMA, geometry=("axis", "radial", "polyline"),
    channels=("height",),
    reference="[1] Bruun 1954; [2] Dean 1991; [3] Dean & Dalrymple 2001; "
              "[4] Hallermeier 1980; [11] Friedrichs 2011",
    consequence="A trend across the area: the far range darkens as the bed "
                "deepens, a beach profile shortens the swath shoreward.",
    blend="normal"))
