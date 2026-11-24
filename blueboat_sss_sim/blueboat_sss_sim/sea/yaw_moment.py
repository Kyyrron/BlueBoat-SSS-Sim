"""Isotropic Gauss-Markov yaw moment: the 2026-10-21 stand-in, kept to reproduce its results.

Port (re-implementation, session 03, 2026-11-15) of ``sea/yaw_moment.py`` on branch ``experiment`` (commit
``a51a44c``), where it was fitted by BlueBoat-Experiments E3 on the BlueROV2 plant (sigma 0.5 N·m, tau 1.0 s,
condition ``arob_c2``). ``N(t) = std_nm * bounded(xi(t))`` about the hull's vertical axis, with ``xi`` the current's
unit Gauss-Markov process (:class:`~.current.UnitGaussMarkov`): exact AR(1) knots on a fixed sim-time grid spaced
``tau/12`` (clipped to 10-500 ms), C1-joined, soft-bounded at about 2.84 sigma. A pure function of ``(seed, t)`` --
never of the tick rate -- on the same random stream as on ``experiment`` (STREAM 13, key
``(seed, 13, round(1000 tau))``), so the same seed gives the same moment sample for sample (smoke ``[10]`` compares
it with values computed from the ``experiment`` code, ``test/fixtures/yaw_moment_experiment_a51a44c.npz``).

Physical meaning: none by itself -- it was a fitted surrogate. Since 2026-11-15 the documented forcing of the
field-matched sea is :mod:`.wind` (mean wind and gusts on the windage, cited coefficients); this term remains for
``config/sea_situations/legacy_standin.yaml`` and as the isotropic "wave-group yaw moment" alternative, used only if
the field yaw fluctuation shows no dependence on the heading relative to the wind (``docs/sea_state_model.md`` §5.9).
Off unless a state carries ``yaw_moment`` with ``std_nm > 0``: absent, nothing is evaluated and the wrench is
bit-identical to a model without it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .current import UnitGaussMarkov, bounded

#: Stream id of the yaw-moment process in the seed's key (current: 11, 12; eddies: 21; wind gusts: 14).
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

    def xi(self, t: float, tau_a: float, tau_b: float | None = None, w: float = 0.0) -> float:
        """Unit fluctuation at ``t``; crossfades the two time constants (variance-preserving)."""
        tau_b = tau_a if tau_b is None else tau_b
        if abs(tau_a - tau_b) < 1e-9 or w <= 0.0:
            return float(self.unit(tau_a)(t))
        if w >= 1.0:
            return float(self.unit(tau_b)(t))
        xa, xb = self.unit(tau_a)(t), self.unit(tau_b)(t)
        return float(((1.0 - w) * xa + w * xb) / np.hypot(1.0 - w, w))

    def moment(self, t: float, std_nm: float, tau_a: float, tau_b: float | None = None,
               w: float = 0.0) -> float:
        """Yaw moment [N·m] about the vertical axis; exactly 0 when std is 0."""
        if std_nm <= 0.0:
            return 0.0
        return float(std_nm) * float(bounded(self.xi(t, tau_a, tau_b, w)))
