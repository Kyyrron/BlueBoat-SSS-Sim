"""The ambient seabed reference of a ray-cast ping, for the NC #15 shadow
floor (harbour patch, 2026-11-10; ``raycast.floor``).

The shared noise stage (``sonar/noise.py::apply_ping_noise``) fills every
bin up to ``reverberation_db`` under a reference level with the seabed's own
Exp(1) speckle. NC #15 defines that reference as the **ambient** seabed at
that range -- what the lit, flat seabed returns there, which never collapses
in a shadow, behind a back-facing flank or over land. The geometric sensor
computes it analytically (``GeometricRenderer._ambient_curve``); this sensor
estimates it from its own ping, so it follows the ping's level exactly
(gain, TVG, correction) with no cross-model rescaling:

1. ``10 log10`` of the ping's **physical** mean (before the empirical
   correction, whose near-nadir bump is no seabed law) in ~16 log-spaced
   range blocks over ``u = r / altitude`` in ``[1.2, max]``;
2. the median of each block that is at least half lit -- a shadow or an
   object covering less than half a block cannot move it;
3. a power law in range: its **slope** is the per-ping Theil-Sen slope
   (the median of the pairwise slopes) of the block medians, smoothed across pings (``state``, one per
   side; a slope is a property of the altitude and the seabed law, which
   change slowly), and held when too few blocks span the range; its
   **level** is the median of the blocks' residuals about that slope, also
   smoothed across pings. Both are robust: a large shadow can remove blocks
   but cannot tilt or step the line (v1, a free per-ping fit, jumped 6-8 dB
   between pings behind an anchor and drew horizontal lines across its
   shadow; measured on the owner's run, ``harbour_patch/line_diag/``);
4. times the correction's own factor (``power / physical``, interpolated
   across shadowed bins), so the reference is the corrected seabed's level;
   0 for ``r <= altitude`` (the water column).

No block at all (a ping looking at a wall, at land, at nothing): ``None``,
and the floor is off for that ping. O(bins) numpy, ~0.05 ms.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

import numpy as np

#: start of the fit window in u = r / altitude (past the crest)
U_START = 1.2
#: number of log-spaced blocks over the fit window
N_BLOCKS = 16
#: a block counts when at least this fraction of its bins is lit
MIN_LIT = 0.5
#: bins per block at least (fewer: the block is skipped)
MIN_BINS = 4
#: blocks needed to measure a slope, and the range factor they must span
SLOPE_BLOCKS = 4
SLOPE_SPAN = 1.5
#: smoothing of the slope across pings (fraction of the new value per ping)
SLOPE_ALPHA = 0.2
#: smoothing of the level (at ``R_REF_M``) across pings
LEVEL_ALPHA = 0.3
R_REF_M = 10.0
#: slope bounds [dB / decade] and the value before any measurement
SLOPE_BOUNDS = (-80.0, 0.0)
SLOPE_DEFAULT = -40.0


def ambient_reference(power: np.ndarray, r_m: np.ndarray, altitude_m: float,
                      physical: np.ndarray | None = None,
                      state: dict | None = None) -> np.ndarray | None:
    """The ambient lit-seabed level per bin (linear, the units of
    ``power``), or ``None`` when the ping shows no seabed. ``physical``: the
    mean before the empirical correction (None: ``power`` itself);
    ``state``: a dict this function keeps the smoothed slope in (one per
    side and run; None: the per-ping slope)."""
    p = np.asarray(power, dtype=np.float64)
    x = p if physical is None else np.asarray(physical, dtype=np.float64)
    r = np.asarray(r_m, dtype=np.float64)
    a = float(altitude_m)
    if not (a > 0.0 and np.isfinite(a)) or p.shape != r.shape or x.shape != r.shape \
            or r.size < MIN_BINS:
        return None
    r_lo, r_hi = U_START * a, float(r[-1])
    if r_hi <= r_lo * 1.05:
        return None
    edges = np.geomspace(r_lo, r_hi, N_BLOCKS + 1)
    xs, ys = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (r >= lo) & (r < hi)
        if int(m.sum()) < MIN_BINS:
            continue
        v = x[m]
        if (v > 0.0).mean() < MIN_LIT:
            continue
        med = float(np.median(v))
        if not med > 0.0:
            continue
        xs.append(float(np.log10(np.sqrt(lo * hi))))
        ys.append(10.0 * float(np.log10(med)))
    if not xs:
        return None
    xs_a, ys_a = np.asarray(xs), np.asarray(ys)
    held = None if state is None else state.get("slope")
    slope = held if held is not None else SLOPE_DEFAULT
    if len(xs) >= SLOPE_BLOCKS and xs_a[-1] - xs_a[0] >= np.log10(SLOPE_SPAN):
        # Theil-Sen: the median of the pairwise slopes, robust to the bright
        # blocks of an object's lit face (a least-squares slope tilts toward them)
        i, j = np.triu_indices(len(xs_a), 1)
        s_ping = float(np.clip(np.median((ys_a[j] - ys_a[i]) / (xs_a[j] - xs_a[i])),
                               *SLOPE_BOUNDS))
        slope = s_ping if held is None else held + SLOPE_ALPHA * (s_ping - held)
    # the level at R_REF_M: the median of the block residuals, smoothed
    # across pings (a reverberation background changes slowly; one more or
    # one fewer bright block beside a shadow must not step it)
    lvl = float(np.median(ys_a - slope * xs_a)) + slope * np.log10(R_REF_M)
    if state is not None:
        prev = state.get("level")
        if prev is not None:
            lvl = prev + LEVEL_ALPHA * (lvl - prev)
        state["slope"], state["level"] = slope, lvl
    icpt = lvl - slope * np.log10(R_REF_M)
    past = r > a
    ref = np.zeros_like(p)
    ref[past] = 10.0 ** ((icpt + slope * np.log10(r[past])) / 10.0)
    if physical is not None:
        # the correction's factor where the physical mean is lit, carried
        # across shadowed bins (the table is smooth in range)
        ok = past & (x > 0.0)
        if ok.any():
            f = np.interp(r, r[ok], p[ok] / x[ok])
            ref[past] *= f[past]
    return ref
