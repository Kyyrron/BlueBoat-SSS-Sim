"""The sea model the ROS node drives: one ``update()`` per tick.

Composes :class:`CurrentProcess`, :class:`WaveField` and :class:`HullBox`
under a :class:`SeaSchedule`. State changes never step: a new target
(schedule keyframe crossing, or a runtime command) is reached by blending
the previous and the next wave fields' *elevations* and current velocities
over a ramp -- a blend of two linear wave fields is itself a valid linear
wave field, and both fields share the same seeded phases, so a ramp in
Hs is a pure amplitude ramp. Along a linearly interpolated schedule the
target is re-sampled every ``resample_s`` and ramped over the same
interval, which keeps the field rebuilds rare (a rebuild is ~100 µs).

Determinism: the wave phases derive from ``seed`` alone and the current's
Gauss-Markov draws from a second stream ``[seed, 1]``, so a re-run of the
same mission with the same seed sees the same sea.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from ..core.types import Pose3D
from .current import CurrentProcess
from .events import WaveEventProcess
from .hull import HullBoxes, WaveExcitation
from .motion import DampingSpec, damping_wrench, hull_damping_from_config
from .presets import (SeaPresets, SeaSchedule, SeaState, compass_name,
                      parse_command)
from .wavefield import WaveField


@dataclass
class SeaSample:
    t_sim: float
    water_velocity: np.ndarray        # (3,) world, current + orbital
    force: np.ndarray                 # (3,) world N, wave excitation
    torque: np.ndarray                # (3,) world N·m about the link origin
    eta_m: float                      # surface elevation at the boat
    state: SeaState                   # the effective (blended) state
    current_vec: np.ndarray           # (3,) the current alone
    waves_active: bool
    blend_w: float
    next_change: tuple[float, SeaState] | None
    event: dict | None = None         # the live wave group, if any


class SeaModel:
    def __init__(self, presets: SeaPresets, schedule: SeaSchedule,
                 excitation: WaveExcitation | None = None, depth_m: float = 5.0,
                 seed: int = 0, ramp_s: float = 20.0,
                 resample_s: float = 2.0, n_freq: int = 16, n_dir: int = 12,
                 damping: DampingSpec | None = None) -> None:
        self.presets = presets
        if excitation is None:
            gz_box, hull = HullBoxes.from_config(presets.hull)
            excitation = WaveExcitation(
                gz_box, hull, **WaveExcitation.clip_from_config(presets.hull))
        self.excitation = excitation
        #: Extra hull damping (heave / roll / pitch) added to the wrench
        #: whenever ``update`` is given the boat's twist (sea/motion.py).
        self.damping = (damping if damping is not None
                        else hull_damping_from_config(presets.hull))
        self.depth_m = float(depth_m)
        self.seed = int(seed)
        self.ramp_s = float(ramp_s)
        self.resample_s = float(resample_s)
        self.n_freq, self.n_dir = int(n_freq), int(n_dir)
        self._cur_rng = np.random.default_rng([self.seed, 1])
        self._schedule = schedule
        self._schedule_t0 = 0.0
        self._t_last: float | None = None
        self._t_resample = -np.inf
        # Blend state: from -> to over [blend_t0, blend_t0 + blend_dur].
        s0 = schedule.state_at(0.0)
        self._from = self._build(s0)
        self._to = self._from
        self._blend_t0 = 0.0
        self._blend_dur = 0.0
        self._target_state = s0
        self.changed_flag = True       # set whenever the target changes
        self.waves_available = True    # the ROS shell clears it if injection fails
        # Random wave groups follow the *target* state's events block and
        # live outside the blend (a ramp must not restart the arrivals).
        self._events = WaveEventProcess(s0.waves.events, s0.waves.from_deg,
                                        self.depth_m, self.seed)

    # ----------------------------------------------------------- builders
    def _build(self, state: SeaState) -> dict:
        w = state.waves
        field = WaveField(w.hs_m, w.tp_s, w.gamma, w.from_deg, w.spread_s,
                          self.depth_m, self.n_freq, self.n_dir,
                          rng=np.random.default_rng(self.seed))
        c = state.current
        cur = CurrentProcess(c.speed_mps, c.from_deg, c.sigma_speed_mps,
                             c.sigma_dir_deg, c.tau_speed_s, c.tau_dir_s,
                             rng=self._cur_rng)
        return {"state": state, "field": field, "current": cur}

    def _retarget(self, state: SeaState, t: float, ramp: float) -> None:
        if state == self._target_state and self._blend_dur == 0.0:
            return
        # Freeze the current blend as the new starting point.
        w = self.blend_weight(t)
        if 0.0 < w < 1.0:
            frozen = SeaState.interpolate(self._from["state"], self._to["state"], w)
            self._from = self._build(frozen)
        else:
            self._from = self._to
        self._to = self._build(state)
        self._target_state = state
        self._blend_t0 = t
        self._blend_dur = max(float(ramp), 0.0)
        self.changed_flag = True
        ev = state.waves.events
        if (ev != self._events.spec
                or abs(state.waves.from_deg - self._events.from_deg) > 1e-9):
            self._events = WaveEventProcess(ev, state.waves.from_deg,
                                            self.depth_m, self.seed, t_start=t)

    # ------------------------------------------------------------ control
    @property
    def schedule(self) -> SeaSchedule:
        return self._schedule

    def set_schedule(self, schedule: SeaSchedule, t_now: float,
                     t0_offset: float = 0.0) -> None:
        """Replace the timeline; its t = 0 is ``t_now + t0_offset``."""
        self._schedule = schedule
        self._schedule_t0 = t_now + t0_offset
        self._t_resample = -np.inf
        self._retarget(schedule.state_at(t_now - self._schedule_t0), t_now,
                       min(self.ramp_s, self.resample_s))

    def set_state(self, state: SeaState, t_now: float, ramp_s: float = -1.0) -> None:
        """Hold ``state`` from now on, reached over ``ramp_s`` (default the
        model's ramp)."""
        self._schedule = SeaSchedule.constant(state, self.seed)
        self._schedule_t0 = t_now
        self._retarget(state, t_now, self.ramp_s if ramp_s < 0.0 else ramp_s)

    def apply_command(self, payload, t_now: float) -> str:
        kind, obj, x = parse_command(self.presets, payload)
        if kind == "schedule":
            self.set_schedule(obj, t_now, x)
            return f"schedule '{obj.name or 'unnamed'}' ({len(obj.keyframes)} keyframes)"
        self.set_state(obj, t_now, x)
        return f"state {self.describe(obj)}"

    def blend_weight(self, t: float) -> float:
        if self._blend_dur <= 0.0:
            return 1.0
        return float(min(max((t - self._blend_t0) / self._blend_dur, 0.0), 1.0))

    # ------------------------------------------------------------- update
    def update(self, t_sim: float, boat: Pose3D | None,
               twist: tuple | None = None) -> SeaSample:
        """One tick. ``twist`` = ``(v_world, w_body)`` -- the boat's world
        linear velocity and body angular rates -- enables the hull
        damping term; without it the wrench is the excitation alone."""
        dt = 0.0 if self._t_last is None else max(t_sim - self._t_last, 0.0)
        self._t_last = t_sim
        # Follow the schedule (re-sample its interpolation sparsely).
        ts = t_sim - self._schedule_t0
        if t_sim - self._t_resample >= self.resample_s:
            self._t_resample = t_sim
            target = self._schedule.state_at(ts)
            if target != self._target_state:
                nxt = self._schedule.next_change(ts)
                ramp = (self.resample_s if self._schedule.interpolation == "linear"
                        and nxt is not None else self.ramp_s)
                self._retarget(target, t_sim, ramp)
        w = self.blend_weight(t_sim)
        if w >= 1.0 and self._from is not self._to:
            self._from = self._to
            self._blend_dur = 0.0
        # Current: both processes step with the same rng stream.
        self._from["current"].step(dt)
        if self._to is not self._from:
            self._to["current"].step(dt)
        cur = ((1.0 - w) * self._from["current"].velocity_world()
               + w * self._to["current"].velocity_world())
        fa, fb = self._from["field"], self._to["field"]
        state = (self._to["state"] if w >= 1.0 else
                 SeaState.interpolate(self._from["state"], self._to["state"], w))
        self._events.advance(t_sim)
        ev = self._events
        active = ((fa.active or fb.active or ev.active) and self.waves_available)
        zeros = np.zeros(3)
        if boat is None or not active:
            return SeaSample(t_sim, cur.copy(), zeros, zeros.copy(), 0.0, state,
                             cur, False, w, self._next_change(ts),
                             ev.current_event(t_sim))

        def surface(x, y):
            if w >= 1.0:
                base = fb.elevation(x, y, t_sim)
            elif w <= 0.0:
                base = fa.elevation(x, y, t_sim)
            else:
                base = ((1.0 - w) * fa.elevation(x, y, t_sim)
                        + w * fb.elevation(x, y, t_sim))
            return base + ev.elevation(x, y, t_sim) if ev.active else base

        def slope(x, y):
            if w >= 1.0:
                sx, sy = fb.slope(x, y, t_sim)
            elif w <= 0.0:
                sx, sy = fa.slope(x, y, t_sim)
            else:
                ax, ay = fa.slope(x, y, t_sim)
                bx, by = fb.slope(x, y, t_sim)
                sx, sy = (1.0 - w) * ax + w * bx, (1.0 - w) * ay + w * by
            if ev.active:
                ex_, ey_ = ev.slope(x, y, t_sim)
                sx, sy = sx + ex_, sy + ey_
            return sx, sy

        force, torque = self.excitation(boat, surface, slope)
        xb, yb = np.array([boat.x]), np.array([boat.y])
        eta = float(surface(xb, yb)[0])
        orb = ((1.0 - w) * fa.orbital_velocity(xb, yb, t_sim)[0]
               + w * fb.orbital_velocity(xb, yb, t_sim)[0])
        if ev.active:
            orb = orb + ev.orbital_velocity(xb, yb, t_sim)[0]
        if twist is not None and self.damping.active:
            fd, td = damping_wrench(twist[0], twist[1], cur + orb,
                                    self.damping, boat.yaw)
            force = force + fd
            torque = torque + td
        return SeaSample(t_sim, cur + orb, force, torque, eta, state, cur,
                         True, w, self._next_change(ts), ev.current_event(t_sim))

    def _next_change(self, ts: float):
        nxt = self._schedule.next_change(ts)
        if nxt is None:
            return None
        return (nxt[0] + self._schedule_t0, nxt[1])

    # ------------------------------------------------------------ readback
    @staticmethod
    def describe(state: SeaState) -> str:
        c, w = state.current, state.waves
        cur = ("no current" if c.speed_mps <= 0.0 else
               f"current {c.preset} {c.speed_mps:.2f} m/s from "
               f"{compass_name(c.from_deg)} ({c.from_deg:.0f}°)")
        wav = ("calm" if w.hs_m <= 0.0 else
               f"waves {w.preset} Hs {w.hs_m:.2f} m Tp {w.tp_s:.1f} s from "
               f"{compass_name(w.from_deg)} ({w.from_deg:.0f}°)")
        return f"{cur}; {wav}"

    def status_dict(self, sample: SeaSample, world: str = "", entity: str = "",
                    waves_reason: str = "") -> dict:
        c, w = sample.state.current, sample.state.waves
        cur_vec = sample.current_vec
        speed = float(np.hypot(cur_vec[0], cur_vec[1]))
        inst_from = ((270.0 - np.degrees(np.arctan2(cur_vec[1], cur_vec[0]))) % 360.0
                     if speed > 1e-6 else c.from_deg)
        nxt = sample.next_change
        return {
            "t_sim": round(float(sample.t_sim), 3),
            "wall_time": time.time(),
            "seed": self.seed,
            "world": world,
            "entity": entity,
            "current": {
                "preset": c.preset, "label": c.label,
                "mean_speed_mps": round(float(c.speed_mps), 3),
                "from_deg": round(float(c.from_deg), 1),
                "from": compass_name(c.from_deg),
                "toward": compass_name(c.from_deg + 180.0),
                "instant": {"speed_mps": round(speed, 3),
                            "from_deg": round(float(inst_from), 1),
                            "vx": round(float(cur_vec[0]), 4),
                            "vy": round(float(cur_vec[1]), 4)},
            },
            "waves": {
                "preset": w.preset, "label": w.label, "douglas": int(w.douglas),
                "hs_m": round(float(w.hs_m), 3), "tp_s": round(float(w.tp_s), 2),
                "from_deg": round(float(w.from_deg), 1),
                "from": compass_name(w.from_deg),
                "toward": compass_name(w.from_deg + 180.0),
                "eta_m": round(float(sample.eta_m), 3),
                "status": ("active" if sample.waves_active else
                           ("unavailable" if not self.waves_available else
                            ("calm" if w.hs_m <= 0.0 else "idle"))),
                "reason": waves_reason,
                "events_per_hour": round(float(w.events.rate_per_hour), 1),
                "event": sample.event,
            },
            "water_velocity": [round(float(v), 4) for v in sample.water_velocity],
            "wrench": {"fz": round(float(sample.force[2]), 2),
                       "mx": round(float(sample.torque[0]), 3),
                       "my": round(float(sample.torque[1]), 3),
                       # additive (2026-09-05): surge / sway / yaw
                       "fx": round(float(sample.force[0]), 2),
                       "fy": round(float(sample.force[1]), 2),
                       "mz": round(float(sample.torque[2]), 3)},
            "blend": round(float(sample.blend_w), 3),
            "schedule": {
                "name": self._schedule.name,
                "keyframes": len(self._schedule.keyframes),
                "interpolation": self._schedule.interpolation,
                "t_s": round(float(sample.t_sim - self._schedule_t0), 1),
                "next_t_sim": None if nxt is None else round(float(nxt[0]), 1),
                "next": None if nxt is None else self.describe(nxt[1]),
            },
            "summary": (self.describe(sample.state)
                        + (f"; wake group {sample.event['hs_m']:.2f} m from "
                           f"{compass_name(sample.event['from_deg'])}, "
                           f"{sample.event['t_remaining_s']:.0f} s left"
                           if sample.event else "")),
        }
