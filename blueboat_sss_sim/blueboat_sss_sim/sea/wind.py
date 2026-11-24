"""Wind on the BlueBoat's windage: mean wind plus gusts, as a surge / sway force and a yaw moment (session 03,
2026-11-15).

Physics (Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed. 2021, §10.1, and the TTK4190
lecture notes ch. 10, "Wind Forces and Moments": the area-based wind coefficients; [full] for §10.1 of the notes)::

    X = q A_F C_X(γ),   Y = q A_L C_Y(γ),   N = q A_L L_oa C_N(γ),   q = ½ ρ_a V_r²

with the symmetric-craft approximation ``C_X = -c_x cos γ``, ``C_Y = -c_y sin γ``, ``C_N = c_n sin 2γ``
(experiments: c_x 0.50-0.90, c_y 0.70-0.95, c_n 0.05-0.20, "to be used with care"). V_r is the speed of the air
relative to the boat (the boat's own velocity is subtracted), γ the direction the relative wind comes FROM, measured
from the bow, positive to port (FLU body axes: x forward, y left, z up). With this sign convention a positive ``c_n``
turns the bow INTO a wind from forward of the beam; the sign for the BlueBoat (windage centre fore or aft of the
centre of lateral resistance) is not documented, so ``c_n`` carries a sign and a bracket. The side force also heels
the boat about the link origin through the windage centroid height (small: ~0.05 deg per newton).

Gusts: the wind speed is ``U (1 + I_u bounded(ξ_u(t)))`` and its direction ``β + σ_β bounded(ξ_β(t))`` with ξ unit
Gauss-Markov processes of time constant ``gust_tau_s`` (corner frequency 1/(2π τ)), exactly the current's
(:class:`~.current.UnitGaussMarkov`): a pure function of ``(seed, t)``, never of the tick rate, on streams 14 (speed)
and 15 (direction); a change of the block blends (never steps) through :meth:`SeaState.interpolate`, and a change of
time constant crossfades two processes, variance-preserving. A first-order process is the low-frequency
approximation of a gust spectrum (Fossen 2021 §10.1 uses spectra such as Harris'; at the boat's yaw bandwidth,
0.05-0.3 Hz, a corner frequency and a variance are the two numbers the 3 Hz field logs can constrain).

Off unless the state's ``wind`` block has ``speed_mps > 0`` and the hull block carries ``windage``: absent, nothing
is evaluated and the wrench is bit-identical to a model without wind.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, Mapping

import numpy as np

from .current import UnitGaussMarkov, bounded

STREAM_SPEED = 14
STREAM_DIR = 15
AIR_DENSITY = 1.225          # kg/m³, 15 °C sea level (ISA)


@dataclass(frozen=True)
class WindSpec:
    """``wind:`` block of a sea state. Speeds are at the windage centroid (~0.15 m above the water), not at 10 m:
    the conversion from a weather station's 10 m wind is the caller's (``docs/sea_state_model.md`` §5.10)."""

    speed_mps: float = 0.0
    from_deg: float = 0.0            # compass bearing the wind comes FROM (0 = N, clockwise)
    gust_intensity: float = 0.0      # std of the speed fluctuation / mean speed (turbulence intensity)
    gust_tau_s: float = 2.0          # time constant of the gust processes [s]
    gust_dir_deg: float = 0.0        # std of the direction fluctuation [deg]
    preset: str = "none"
    label: str = "No wind"

    @property
    def active(self) -> bool:
        return self.speed_mps > 0.0


@dataclass(frozen=True)
class Windage:
    """The boat's above-water geometry and wind coefficients (``hull.windage`` of the presets file)."""

    frontal_area_m2: float
    lateral_area_m2: float
    length_m: float
    centroid_height_m: float = 0.15      # lateral-area centroid above the waterline
    cx: float = 0.7
    cy: float = 0.8
    cn: float = 0.1
    air_density: float = AIR_DENSITY

    @staticmethod
    def from_config(hull_cfg: Mapping[str, Any] | None) -> "Windage | None":
        raw = dict((hull_cfg or {}).get("windage") or {})
        if not raw:
            return None
        known = {f.name for f in fields(Windage)}
        unknown = set(raw) - known
        if unknown:
            raise KeyError(f"Unknown hull.windage keys: {sorted(unknown)}")
        return Windage(**{k: float(v) for k, v in raw.items()})


def _knot_dt(tau_s: float) -> float:
    return float(np.clip(float(tau_s) / 12.0, 0.01, 0.5))


class WindField:
    """The wind of one run, deterministic per ``seed``."""

    def __init__(self, seed: int) -> None:
        self.seed = int(seed)
        self._procs: dict[tuple[int, float], UnitGaussMarkov] = {}

    def _unit(self, stream: int, tau_s: float) -> UnitGaussMarkov:
        key = (stream, round(float(tau_s), 6))
        proc = self._procs.get(key)
        if proc is None:
            tau = max(float(tau_s), 1e-3)
            proc = UnitGaussMarkov(tau, (self.seed, stream, int(round(tau * 1000.0))),
                                   knot_dt_s=_knot_dt(tau))
            self._procs[key] = proc
        return proc

    def _xi(self, stream: int, t: float, tau_a: float, tau_b: float, w: float) -> float:
        if abs(tau_a - tau_b) < 1e-9 or w <= 0.0:
            return float(self._unit(stream, tau_a)(t))
        if w >= 1.0:
            return float(self._unit(stream, tau_b)(t))
        xa, xb = self._unit(stream, tau_a)(t), self._unit(stream, tau_b)(t)
        return float(((1.0 - w) * xa + w * xb) / np.hypot(1.0 - w, w))

    def speed_dir(self, t: float, spec: WindSpec, tau_b: float | None = None, w: float = 0.0
                  ) -> tuple[float, float]:
        """(speed [m/s], FROM bearing [deg]) at ``t``: the mean plus the bounded gust fluctuations."""
        if not spec.active:
            return 0.0, spec.from_deg
        tb = spec.gust_tau_s if tau_b is None else tau_b
        u = spec.speed_mps
        if spec.gust_intensity > 0.0:
            u = u * (1.0 + spec.gust_intensity * float(bounded(self._xi(STREAM_SPEED, t, spec.gust_tau_s, tb, w))))
        d = spec.from_deg
        if spec.gust_dir_deg > 0.0:
            d = d + spec.gust_dir_deg * float(bounded(self._xi(STREAM_DIR, t, spec.gust_tau_s, tb, w)))
        return max(u, 0.0), d % 360.0

    @staticmethod
    def air_velocity_world(speed: float, from_deg: float) -> np.ndarray:
        """ENU velocity of the air for a wind of ``speed`` coming FROM compass ``from_deg``."""
        toward = np.radians(90.0 - (from_deg + 180.0))
        return np.array([speed * np.cos(toward), speed * np.sin(toward), 0.0])

    def wrench(self, t: float, spec: WindSpec, windage: Windage | None, boat_yaw: float,
               v_world: np.ndarray | None = None, link_depth_m: float = 0.111,
               tau_b: float | None = None, w: float = 0.0) -> tuple[np.ndarray, np.ndarray, dict]:
        """World force [N] and torque [N·m] about the link origin, and a readback dict."""
        zero = np.zeros(3)
        if windage is None or not spec.active:
            return zero, zero.copy(), {"active": False}
        u, d = self.speed_dir(t, spec, tau_b, w)
        va = self.air_velocity_world(u, d)
        if v_world is not None:
            va = va - np.array([v_world[0], v_world[1], 0.0])
        c, s = np.cos(boat_yaw), np.sin(boat_yaw)
        ax, ay = c * va[0] + s * va[1], -s * va[0] + c * va[1]      # body FLU
        vr2 = ax * ax + ay * ay
        gamma = float(np.arctan2(-ay, -ax))                            # where the relative wind comes from
        q = 0.5 * windage.air_density * vr2
        X = q * windage.frontal_area_m2 * (-windage.cx * np.cos(gamma))
        Y = q * windage.lateral_area_m2 * (-windage.cy * np.sin(gamma))
        N = q * windage.lateral_area_m2 * windage.length_m * windage.cn * np.sin(2.0 * gamma)
        h = windage.centroid_height_m + link_depth_m                  # centroid above the link origin
        force = np.array([c * X - s * Y, s * X + c * Y, 0.0])
        # r x F with r = (0, 0, h) in body axes: roll -h Y, pitch +h X (body), rotated to world
        kb, mb = -h * Y, h * X
        torque = np.array([c * kb - s * mb, s * kb + c * mb, N])
        return force, torque, {"active": True, "speed_mps": u, "from_deg": d, "relative_speed_mps": float(np.sqrt(vr2)),
                               "gamma_deg": float(np.degrees(gamma)), "X": float(X), "Y": float(Y), "N": float(N)}
