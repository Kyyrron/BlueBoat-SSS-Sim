"""The sea model the ROS node drives: one ``update()`` per tick.

Composes :class:`CurrentField`, :class:`WaveField` and :class:`HullBox`
under a :class:`SeaSchedule`. State changes never step: a new target
(schedule keyframe crossing, or a runtime command) is reached by blending
the previous and the next wave fields' *elevations* over a ramp -- a blend
of two linear wave fields is itself a valid linear wave field, and both
fields share the same seeded phases, so a ramp in Hs is a pure amplitude
ramp -- while the current's mean, direction and sigmas follow the blended
state and its variation keeps running (it is never restarted). Along a
linearly interpolated schedule the target is re-sampled every
``resample_s`` and ramped over the same interval, which keeps the field
rebuilds rare (a rebuild is ~100 µs).

Determinism: the wave phases derive from ``seed`` alone, and the current
is a pure function of ``(seed, t, x, y)`` and the schedule
(sea/current.py), so a re-run of the same mission with the same seed sees
the same sea -- however the ROS timer happens to tick.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace

import numpy as np

from ..core.types import Pose3D
from .current import CurrentField
from .events import WaveEventProcess
from .hull import HullBoxes, WaveExcitation
from .motion import (DampingSpec, damping_wrench, default_plant_for,
                     hull_damping_from_config)
from .plant import PlantVertical
from .presets import (SeaPresets, SeaSchedule, SeaState, compass_name,
                      parse_command)
from .wavefield import WaveField
from .wind import WindField, Windage
from .yaw_moment import YawMomentField

#: kappa of the identified plant (commanded -> applied newton, PARAMETER_REGISTER.md §2): the reference of the
#: ``kappa`` bracket below.
KAPPA_IDENTIFIED = 0.493


@dataclass
class SeaSample:
    t_sim: float
    water_velocity: np.ndarray        # (3,) world, current + orbital
    force: np.ndarray                 # (3,) world N, wave excitation
    torque: np.ndarray                # (3,) world N·m about the link origin
    eta_m: float                      # surface elevation at the boat
    state: SeaState                   # the effective (blended) state
    current_vec: np.ndarray           # (3,) the current alone (at the boat)
    waves_active: bool
    blend_w: float
    next_change: tuple[float, SeaState] | None
    event: dict | None = None         # the live wave group, if any
    #: The current's parts at the boat: mean / temporal / spatial (3,) each.
    current_parts: dict | None = None
    #: A wrench is to be published: the catamaran hydrostatics (in every
    #: run since 2026-10-09) and/or wave forcing. ``waves_active`` alone
    #: says whether there is wave forcing.
    wrench_active: bool = False


HYDROSTATICS = ("catamaran", "gazebo_box_when_calm")


def hydrostatics_from_config(hull_cfg) -> str:
    """``hull.hydrostatics`` of ``config/sea_states.yaml`` (default
    ``catamaran``); an unknown value raises by name."""
    v = str((hull_cfg or {}).get("hydrostatics", "catamaran"))
    if v not in HYDROSTATICS:
        raise ValueError(f"hull.hydrostatics must be one of {HYDROSTATICS}, got {v!r}")
    return v


#: ``hull.wave_response`` defaults: how the wave field is built for the hull
#: and whether its pressure is read at the draft. Absent from a presets
#: file, the pre-2026-11-10 behaviour holds bit for bit (equal-energy
#: bins, surface pressure).
WAVE_RESPONSE_DEFAULTS = {"discretisation": "equal_energy", "f_max_hz": 3.0,
                          "draft_decay": False}


def wave_response_from_config(hull_cfg) -> dict:
    """``hull.wave_response`` of ``config/sea_states.yaml``; unknown keys
    raise by name."""
    raw = dict((hull_cfg or {}).get("wave_response") or {})
    unknown = set(raw) - set(WAVE_RESPONSE_DEFAULTS)
    if unknown:
        raise KeyError(f"Unknown hull.wave_response keys: {sorted(unknown)}")
    out = {**WAVE_RESPONSE_DEFAULTS, **raw}
    if out["discretisation"] not in ("equal_energy", "hybrid"):
        raise ValueError("hull.wave_response.discretisation must be "
                         f"'equal_energy' or 'hybrid', got {out['discretisation']!r}")
    out["f_max_hz"] = float(out["f_max_hz"])
    out["draft_decay"] = bool(out["draft_decay"])
    return out


def _flat(x, y):
    return np.zeros_like(x)


class SeaModel:
    def __init__(self, presets: SeaPresets, schedule: SeaSchedule,
                 excitation: WaveExcitation | None = None, depth_m: float = 5.0,
                 seed: int = 0, ramp_s: float = 20.0,
                 resample_s: float = 2.0, n_freq: int = 16, n_dir: int = 12,
                 damping: DampingSpec | None = None,
                 plant: PlantVertical | None = None, kappa: float = 0.0) -> None:
        self.presets = presets
        #: The plant Gazebo integrates (sea/plant.py): sizes a ratio-based hull
        #: damping as the complement of the plugin's (R-8, 2026-11-15).
        self.plant = plant if plant is not None else default_plant_for(presets.hull)
        if excitation is None:
            gz_box, hull = HullBoxes.from_config(presets.hull)
            excitation = WaveExcitation(
                gz_box, hull, **WaveExcitation.clip_from_config(presets.hull))
        self.excitation = excitation
        #: ``catamaran`` (default since 2026-10-09): the catamaran
        #: hydrostatics replace Gazebo's URDF box in EVERY run, calm water
        #: included, so every scenario floats the same hull (S11, resolved:
        #: docs/sea_state_model.md §5). ``gazebo_box_when_calm`` restores
        #: the earlier swap-only-with-waves behaviour (old runs, A/B).
        self.hydrostatics = hydrostatics_from_config(presets.hull)
        #: Extra hull damping (heave / roll / pitch) added to the wrench
        #: whenever ``update`` is given the boat's twist (sea/motion.py).
        self.damping = (damping if damping is not None
                        else hull_damping_from_config(presets.hull, self.plant,
                                                      getattr(excitation, "hull", None)))
        #: The boat's windage (hull.windage; None = no wind on this hull) and
        #: the run's wind and yaw-moment processes, pure functions of
        #: (seed, t) like the current (sea/wind.py, sea/yaw_moment.py).
        self.windage = Windage.from_config(presets.hull)
        #: kappa bracket (R4, 2026-11-15): the identified plant is in commanded
        #: newtons, so a PHYSICAL external force enters it divided by kappa.
        #: Varying kappa with the drag (the response to commands unchanged) is
        #: therefore exactly a scale kappa_identified / kappa on the horizontal
        #: sea forces (wind, waves' surge / sway / yaw); 0 = off (scale 1).
        self.kappa = float(kappa)
        self.horizontal_scale = (KAPPA_IDENTIFIED / self.kappa if self.kappa > 0.0 else 1.0)
        #: How the field is built and read (``hull.wave_response``,
        #: 2026-11-10): the hybrid discretisation resolves the slope
        #: spectrum to ``f_max_hz``; with ``draft_decay`` the hull feels the
        #: pressure at its draft (each component x cosh(k(h-d))/cosh(kh)).
        self.wave_response = wave_response_from_config(presets.hull)
        _hull = getattr(excitation, "hull", None)
        self.decay_depth_m = ((_hull.pressure_depth_at_trim()
                               if hasattr(_hull, "pressure_depth_at_trim")
                               else _hull.draft_at_trim())
                              if self.wave_response["draft_decay"]
                              and hasattr(_hull, "draft_at_trim") else 0.0)
        self.depth_m = float(depth_m)
        self.seed = int(seed)
        self.ramp_s = float(ramp_s)
        self.resample_s = float(resample_s)
        self.n_freq, self.n_dir = int(n_freq), int(n_dir)
        #: The run's current: mean + variation in time and space, a pure
        #: function of (seed, t, x, y) and the state it is given.
        self.current = CurrentField(self.seed)
        self.wind = WindField(self.seed)
        self.yaw_field = YawMomentField(self.seed)
        self._last_wind: dict = {"active": False}
        self._last_yaw_nm = 0.0
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
        wr = self.wave_response
        n_freq = 24 if wr["discretisation"] == "hybrid" else self.n_freq
        field = WaveField(w.hs_m, w.tp_s, w.gamma, w.from_deg, w.spread_s,
                          self.depth_m, n_freq, self.n_dir,
                          rng=np.random.default_rng(self.seed),
                          discretisation=wr["discretisation"],
                          f_max_hz=wr["f_max_hz"])
        return {"state": state, "field": field}

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
        self._t_last = t_sim
        # Follow the schedule (re-sample its interpolation sparsely).
        ts = t_sim - self._schedule_t0
        if t_sim - self._t_resample >= self.resample_s:
            self._t_resample = t_sim
            target = self._schedule.state_at(ts)
            if target != self._target_state:
                nxt = self._schedule.next_change(ts)
                if self._schedule.interpolation == "linear" and nxt is not None:
                    ramp = self.resample_s
                else:
                    kf_ramp = self._schedule.ramp_for(ts)
                    ramp = self.ramp_s if kf_ramp is None else kf_ramp
                self._retarget(target, t_sim, ramp)
        w = self.blend_weight(t_sim)
        if w >= 1.0 and self._from is not self._to:
            self._from = self._to
            self._blend_dur = 0.0
        fa, fb = self._from["field"], self._to["field"]
        state = (self._to["state"] if w >= 1.0 else
                 SeaState.interpolate(self._from["state"], self._to["state"], w))
        # Current at the boat (the origin before the first pose): the
        # blended mean/sigmas, the from-state's time constants crossfaded
        # into the target's (the same process when they agree).
        ca, cb = self._from["state"].current, self._to["state"].current
        spec = replace(state.current, tau_speed_s=ca.tau_speed_s,
                       tau_dir_s=ca.tau_dir_s)
        bx, by = (boat.x, boat.y) if boat is not None else (0.0, 0.0)
        parts = self.current.components(t_sim, bx, by, spec, cb, w)
        cur = parts["total"]
        self._events.advance(t_sim)
        ev = self._events
        active = ((fa.active or fb.active or ev.active) and self.waves_available)
        hull_always = self.hydrostatics == "catamaran" and self.waves_available
        zeros = np.zeros(3)
        dist = self._disturbance(t_sim, boat, twist, w)
        if boat is None or not (active or hull_always or dist is not None):
            return SeaSample(t_sim, cur.copy(), zeros, zeros.copy(), 0.0, state,
                             cur, False, w, self._next_change(ts),
                             ev.current_event(t_sim), parts, False)
        if not active:
            # Calm water, catamaran hydrostatics: the hull swap on the flat
            # plane (no wave field evaluated) plus the hull damping -- the
            # same hull as every wave run, so calm-vs-waves isolates the sea.
            if hull_always:
                force, torque = self.excitation(boat, _flat)
            else:
                force, torque = zeros.copy(), zeros.copy()
            if twist is not None and self.damping.active and hull_always:
                fd, td = damping_wrench(twist[0], twist[1], cur, self.damping,
                                        boat.yaw)
                force = force + fd
                torque = torque + td
            force, torque = self._with_disturbance(force, torque, dist)
            return SeaSample(t_sim, cur.copy(), force, torque, 0.0, state, cur,
                             False, w, self._next_change(ts),
                             ev.current_event(t_sim), parts, True)

        d = self.decay_depth_m

        def both(x, y, depth):
            """(η, ∂η/∂x, ∂η/∂y) of the blended field + live wave groups,
            one phase evaluation per field."""
            if w >= 1.0:
                e, sx, sy = fb.elevation_and_slope(x, y, t_sim, depth)
            elif w <= 0.0:
                e, sx, sy = fa.elevation_and_slope(x, y, t_sim, depth)
            else:
                ea, ax, ay = fa.elevation_and_slope(x, y, t_sim, depth)
                eb, bx, by = fb.elevation_and_slope(x, y, t_sim, depth)
                e = (1.0 - w) * ea + w * eb
                sx, sy = (1.0 - w) * ax + w * bx, (1.0 - w) * ay + w * by
            if ev.active:
                e = e + ev.elevation(x, y, t_sim, depth)
                ex_, ey_ = ev.slope(x, y, t_sim, depth)
                sx, sy = sx + ex_, sy + ey_
            return e, sx, sy

        # The hull samples the surface and its slope at the same column
        # points within one excitation call: evaluate the field once for
        # both (the cache lives for this tick only).
        _cache: dict = {}

        def _at(x, y):
            key = (np.asarray(x).__array_interface__["data"][0],
                   np.asarray(y).__array_interface__["data"][0], np.shape(x))
            if _cache.get("key") != key:
                _cache["key"], _cache["val"] = key, both(x, y, d)
            return _cache["val"]

        def surface(x, y):
            return _at(x, y)[0]

        def slope(x, y):
            _e, sx, sy = _at(x, y)
            return sx, sy

        force, torque = self.excitation(boat, surface, slope)
        xb, yb = np.array([boat.x]), np.array([boat.y])
        # The readback is the TRUE surface at the boat (no draft decay).
        eta = float(both(xb, yb, 0.0)[0][0])
        orb = ((1.0 - w) * fa.orbital_velocity(xb, yb, t_sim)[0]
               + w * fb.orbital_velocity(xb, yb, t_sim)[0])
        if ev.active:
            orb = orb + ev.orbital_velocity(xb, yb, t_sim)[0]
        if self.horizontal_scale != 1.0:
            force = force.copy(); torque = torque.copy()
            force[:2] *= self.horizontal_scale
            torque[2] *= self.horizontal_scale
        if twist is not None and self.damping.active:
            fd, td = damping_wrench(twist[0], twist[1], cur + orb,
                                    self.damping, boat.yaw)
            force = force + fd
            torque = torque + td
        force, torque = self._with_disturbance(force, torque, dist)
        return SeaSample(t_sim, cur + orb, force, torque, eta, state, cur,
                         True, w, self._next_change(ts), ev.current_event(t_sim),
                         parts, True)

    # ------------------------------------------------------- disturbance
    def _disturbance(self, t_sim: float, boat: Pose3D | None, twist, w: float):
        """Wind on the windage plus the isotropic yaw moment of the blended
        state; None when neither is active (nothing evaluated: bit-identical
        to a model without them)."""
        a, b = self._from["state"], self._to["state"]
        st = b if w >= 1.0 else SeaState.interpolate(a, b, w)
        wind_on = st.wind.active and self.windage is not None
        yaw_on = st.yaw_moment.active
        if boat is None or not (wind_on or yaw_on):
            self._last_wind, self._last_yaw_nm = {"active": False}, 0.0
            return None
        force, torque = np.zeros(3), np.zeros(3)
        if wind_on:
            spec = replace(st.wind, gust_tau_s=a.wind.gust_tau_s if a.wind.active else b.wind.gust_tau_s)
            fw, tw, info = self.wind.wrench(t_sim, spec, self.windage, boat.yaw,
                                            None if twist is None else twist[0],
                                            tau_b=b.wind.gust_tau_s, w=w)
            force, torque = force + fw, torque + tw
            self._last_wind = info
        else:
            self._last_wind = {"active": False}
        if yaw_on:
            ta = a.yaw_moment.tau_s if a.yaw_moment.active else b.yaw_moment.tau_s
            nz = self.yaw_field.moment(t_sim, st.yaw_moment.std_nm, ta, b.yaw_moment.tau_s, w)
            torque = torque + np.array([0.0, 0.0, nz])
            self._last_yaw_nm = nz
        else:
            self._last_yaw_nm = 0.0
        s = self.horizontal_scale
        force[:2] *= s
        torque[2] *= s
        return force, torque

    @staticmethod
    def _with_disturbance(force, torque, dist):
        if dist is None:
            return force, torque
        return force + dist[0], torque + dist[1]

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
               f"{compass_name(c.from_deg)} ({c.from_deg:.0f}°)"
               + (f", {c.variation} variation" if c.variation else ""))
        wav = ("calm" if w.hs_m <= 0.0 else
               f"waves {w.preset} Hs {w.hs_m:.2f} m Tp {w.tp_s:.1f} s from "
               f"{compass_name(w.from_deg)} ({w.from_deg:.0f}°)")
        extra = ""
        if state.wind.active:
            extra += (f"; wind {state.wind.speed_mps:.1f} m/s from {compass_name(state.wind.from_deg)} "
                      f"({state.wind.from_deg:.0f}°), gusts {100 * state.wind.gust_intensity:.0f} % "
                      f"tau {state.wind.gust_tau_s:.1f} s")
        if state.yaw_moment.active:
            extra += (f"; yaw moment {state.yaw_moment.std_nm:.2f} N·m tau "
                      f"{state.yaw_moment.tau_s:.1f} s")
        return f"{cur}; {wav}{extra}"

    def status_dict(self, sample: SeaSample, world: str = "", entity: str = "",
                    waves_reason: str = "") -> dict:
        c, w = sample.state.current, sample.state.waves
        cur_vec = sample.current_vec
        speed = float(np.hypot(cur_vec[0], cur_vec[1]))
        inst_from = ((270.0 - np.degrees(np.arctan2(cur_vec[1], cur_vec[0]))) % 360.0
                     if speed > 1e-6 else c.from_deg)
        nxt = sample.next_change
        parts = sample.current_parts or {}
        spatial = parts.get("spatial")
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
                # additive (2026-10-08): the variation around the mean
                "variation": c.variation,
                "sigma_speed_mps": round(float(c.sigma_speed_mps), 4),
                "sigma_dir_deg": round(float(c.sigma_dir_deg), 2),
                "spatial_rms_mps": round(float(c.spatial_rms_mps), 4),
                "spatial_mps": (round(float(np.hypot(spatial[0], spatial[1])), 4)
                                if spatial is not None else 0.0),
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
                # additive (2026-10-08): plain-language choices
                "crest": w.crest, "wakes": w.wakes,
                "spread_s": round(float(w.spread_s), 2),
            },
            # additive (2026-10-09): which hydrostatics carry the hull
            "hull": {"model": ("catamaran" if (self.hydrostatics == "catamaran"
                                               or sample.waves_active)
                               and self.waves_available else "gazebo_box"),
                     "active": bool(sample.wrench_active)},
            "water_velocity": [round(float(v), 4) for v in sample.water_velocity],
            "wrench": {"fz": round(float(sample.force[2]), 2),
                       "mx": round(float(sample.torque[0]), 3),
                       "my": round(float(sample.torque[1]), 3),
                       # additive (2026-09-05): surge / sway / yaw
                       "fx": round(float(sample.force[0]), 2),
                       "fy": round(float(sample.force[1]), 2),
                       "mz": round(float(sample.torque[2]), 3)},
            # additive (2026-11-15): the disturbance and the kappa bracket
            "wind": {"speed_mps": round(float(sample.state.wind.speed_mps), 3),
                     "from_deg": round(float(sample.state.wind.from_deg), 1),
                     "gust_intensity": round(float(sample.state.wind.gust_intensity), 3),
                     "gust_tau_s": round(float(sample.state.wind.gust_tau_s), 2),
                     "gust_dir_deg": round(float(sample.state.wind.gust_dir_deg), 1),
                     "windage": self.windage is not None,
                     "instant": {k: (round(float(v), 3) if isinstance(v, float) else v)
                                 for k, v in self._last_wind.items()}},
            "yaw_moment": {"std_nm": round(float(sample.state.yaw_moment.std_nm), 3),
                           "tau_s": round(float(sample.state.yaw_moment.tau_s), 2),
                           "nm": round(float(self._last_yaw_nm), 4)},
            "plant": {"name": self.plant.name, "source": self.plant.source},
            "kappa": {"value": self.kappa, "horizontal_scale": round(self.horizontal_scale, 4)},
            "damping": {"heave_ns_per_m": round(float(self.damping.heave_ns_per_m), 2),
                        "roll_nms_per_rad": round(float(self.damping.roll_nms_per_rad), 3),
                        "pitch_nms_per_rad": round(float(self.damping.pitch_nms_per_rad), 3)},
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
