"""Random long-period wave events: the "kinda random" part of a harbour sea.

A basin's background chop is a stationary JONSWAP process (``wavefield``),
but what a small boat actually *feels* most are the groups that arrive at
random: a passing vessel's wake (a few cycles of 4-8 s waves, minutes
apart), a swell train leaking through the entrance, a gust-raised group.
This module models them as a **marked Poisson process** (exponential
inter-arrival times, mean ``3600 / rate_per_hour`` s): each event is its
own short, narrow directional wave train -- a small :class:`WaveField`
with a peak period jittered ±15 % about ``tp_s``, a direction drawn
uniformly within ``±from_spread_deg`` of the sea's from-direction, seeded
phases -- multiplied by a Hann envelope of ``duration_s``, so it grows,
peaks at ``hs_m`` and dies without a step. At most ``max_overlap`` events
are live at once; the elevation, slope and orbital velocity are the sums
over the live events, and exact zeros when none is live.

The Poisson arrival assumption and the Hann envelope are modelling
choices, not measurements (no wake statistics exist for the study site);
the *shape* of a wake group -- a few long-period cycles with a growing and
decaying envelope -- follows the ship-wake literature (Soomere 2007, Appl. Mech. Rev. 60(3),
doi:10.1115/1.2730847; Kelvin wake theory). Deterministic per
seed: the arrival stream is ``[seed, 3]``, event ``k`` draws from
``[seed, 3, k]``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .wavefield import WaveField


@dataclass(frozen=True)
class EventSpec:
    """Per-preset event statistics (``events:`` block of a wave preset)."""

    rate_per_hour: float = 0.0     # mean arrivals per hour (0 = no events)
    hs_m: float = 0.0              # significant height at the envelope peak
    tp_s: float = 6.0              # peak period of the group (±15 % jitter)
    duration_s: float = 30.0       # Hann envelope length
    from_spread_deg: float = 45.0  # uniform direction scatter about the sea
    max_overlap: int = 3

    @property
    def active(self) -> bool:
        return self.rate_per_hour > 0.0 and self.hs_m > 0.0 and self.tp_s > 0.0


class WaveEventProcess:
    """Live wave groups at time ``t``; ``advance(t)`` before evaluating."""

    #: A time jump longer than this restarts the arrivals instead of
    #: replaying every group in between.
    MAX_JUMP_S = 6.0 * 3600.0

    def __init__(self, spec: EventSpec | None, from_deg: float, depth_m: float,
                 seed: int = 0, t_start: float = 0.0) -> None:
        self.spec = spec if spec is not None else EventSpec()
        self.from_deg = float(from_deg) % 360.0
        self.depth_m = float(depth_m)
        self.seed = int(seed)
        self._arrivals = np.random.default_rng([self.seed, 3])
        self._t = float(t_start)
        self._next = (self._t + self._draw_gap()) if self.spec.active else np.inf
        self._k = 0
        self._live: list[tuple[float, float, WaveField]] = []   # (t0, dur, field)
        self.count = 0                                            # events spawned

    # ------------------------------------------------------------ arrivals
    def _draw_gap(self) -> float:
        return float(self._arrivals.exponential(3600.0 / self.spec.rate_per_hour))

    def _spawn(self, t0: float) -> None:
        rng = np.random.default_rng([self.seed, 3, self._k])
        self._k += 1
        tp = self.spec.tp_s * float(rng.uniform(0.85, 1.15))
        frm = self.from_deg + float(rng.uniform(-self.spec.from_spread_deg,
                                                self.spec.from_spread_deg))
        # 8 frequencies x 3 bearings, gamma 1.5, s 8 (2026-09-05): a wake
        # group is a short *train*, not a monochromatic wave -- the earlier
        # 4 x 1 field was a metronome whenever a group was live.
        field = WaveField(self.spec.hs_m, tp, 1.5, frm, 8.0, self.depth_m,
                          n_freq=8, n_dir=3, rng=rng)
        self._live.append((t0, float(self.spec.duration_s), field))
        self.count += 1
        if len(self._live) > self.spec.max_overlap:
            self._live.pop(0)

    def advance(self, t: float) -> None:
        """Spawn the arrivals up to ``t`` and retire the finished groups."""
        t = float(t)
        if not self.spec.active:
            return
        if t - self._t > self.MAX_JUMP_S:
            # A sim-time jump of hours (replay seek, clock reset): the
            # arrivals in between are of no interest -- restart the
            # process at t instead of spawning them all.
            self._live.clear()
            self._next = t + self._draw_gap()
        while self._next <= t:
            self._spawn(self._next)
            self._next += self._draw_gap()
        self._t = t
        self._live = [e for e in self._live if t < e[0] + e[1]]

    @staticmethod
    def _envelope(t: float, t0: float, dur: float) -> float:
        if t <= t0 or t >= t0 + dur:
            return 0.0
        return 0.5 * (1.0 - np.cos(2.0 * np.pi * (t - t0) / dur))

    # ---------------------------------------------------------- evaluation
    @property
    def active(self) -> bool:
        return bool(self._live)

    def current_event(self, t: float) -> dict | None:
        """The strongest live group at ``t`` for the readback, or None."""
        best, best_env = None, 0.0
        for t0, dur, field in self._live:
            env = self._envelope(t, t0, dur)
            if env > best_env:
                best_env, best = env, (t0, dur, field)
        if best is None:
            return None
        t0, dur, field = best
        return {"hs_m": round(field.hs * best_env, 3), "tp_s": round(field.tp, 2),
                "from_deg": round(field.from_deg, 1),
                "t_remaining_s": round(t0 + dur - t, 1),
                "envelope": round(best_env, 3)}

    def elevation(self, x, y, t: float) -> np.ndarray:
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        out = np.zeros(x.shape)
        for t0, dur, field in self._live:
            env = self._envelope(t, t0, dur)
            if env > 0.0:
                out += env * field.elevation(x, y, t)
        return out

    def slope(self, x, y, t: float) -> tuple[np.ndarray, np.ndarray]:
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        sx, sy = np.zeros(x.shape), np.zeros(x.shape)
        for t0, dur, field in self._live:
            env = self._envelope(t, t0, dur)
            if env > 0.0:
                a, b = field.slope(x, y, t)
                sx += env * a
                sy += env * b
        return sx, sy

    def orbital_velocity(self, x, y, t: float) -> np.ndarray:
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        out = np.zeros((x.shape[0], 3))
        for t0, dur, field in self._live:
            env = self._envelope(t, t0, dur)
            if env > 0.0:
                out += env * field.orbital_velocity(x, y, t)
        return out
