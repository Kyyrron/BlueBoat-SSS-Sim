"""Synthetic yaw disturbance: a wind and wave yaw-moment surrogate.

Not a physical model of wind or of second-order wave yaw. It is a
first-order Gauss-Markov yaw moment about the hull's vertical axis,
``N(t) = std_nm * bounded(xi(t))``, whose two numbers (``std_nm`` [N·m],
``tau_s`` [s]) are FITTED so that the simulated boat's yaw statistics match
field statistics the 3 Hz logs can observe (docs/sea_state_model.md
"Synthetic yaw disturbance"). It is off unless a state carries a
``yaw_moment`` block with ``std_nm > 0``; absent, nothing is evaluated and
the wrench is bit-identical to a model without it.

The unit process is the current's (:class:`~.current.UnitGaussMarkov`):
exact AR(1) knots on a fixed sim-time grid, C1-joined, soft-bounded at
~2.8 sigma and rescaled to unit std, so ``N`` is a pure function of
``(seed, t)`` -- never of the tick rate -- and a ramp (a change of
``std_nm``) never restarts it. The knot spacing is ``tau/12`` (clipped to
10-500 ms), finer than the current's because yaw-moment time constants of
a second or less are in scope; the spline's variance dip stays <= 2 %.
A change of ``tau_s`` crossfades two independent unit processes,
variance-preserving, exactly as the current does.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .current import UnitGaussMarkov, bounded

#: Stream id of the yaw-moment process in the seed's key (current: 11, 12;
#: eddies: 21).
STREAM = 13


@dataclass(frozen=True)
class YawMomentSpec:
    """``yaw_moment:`` block of a sea state: std [N·m] and time constant [s]."""

    std_nm: float = 0.0
    tau_s: float = 2.0

    @property
    def active(self) -> bool:
        return self.std_nm > 0.0


def knot_dt(tau_s: float) -> float:
    return float(np.clip(float(tau_s) / 12.0, 0.01, 0.5))


class YawMomentField:
    """The yaw moment of one run, deterministic per ``seed``."""

    def __init__(self, seed: int) -> None:
        self.seed = int(seed)
        self._procs: dict[float, UnitGaussMarkov] = {}

    def unit(self, tau_s: float) -> UnitGaussMarkov:
        key = round(float(tau_s), 6)
        proc = self._procs.get(key)
        if proc is None:
            tau = max(float(tau_s), 1e-3)
            proc = UnitGaussMarkov(tau, (self.seed, STREAM, int(round(tau * 1000.0))),
                                   knot_dt_s=knot_dt(tau))
            self._procs[key] = proc
        return proc

    def xi(self, t: float, tau_a: float, tau_b: float | None = None,
           w: float = 0.0) -> float:
        """Unit fluctuation at ``t``; crossfades the two time constants."""
        tau_b = tau_a if tau_b is None else tau_b
        if abs(tau_a - tau_b) < 1e-9 or w <= 0.0:
            return float(self.unit(tau_a)(t))
        if w >= 1.0:
            return float(self.unit(tau_b)(t))
        xa, xb = self.unit(tau_a)(t), self.unit(tau_b)(t)
        return float(((1.0 - w) * xa + w * xb) / np.hypot(1.0 - w, w))

    def moment(self, t: float, std_nm: float, tau_a: float,
               tau_b: float | None = None, w: float = 0.0) -> float:
        """Yaw moment [N·m] about the vertical axis; exactly 0 when std is 0."""
        if std_nm <= 0.0:
            return 0.0
        return float(std_nm) * float(bounded(self.xi(t, tau_a, tau_b, w)))
