"""Slowly varying ocean current: mean + first-order Gauss-Markov fluctuation.

Fossen (2011) §8.3 (doi:10.1002/9781119994138) models the current speed
and direction as saturated first-order Gauss-Markov (Ornstein-Uhlenbeck)
processes,  V̇_c + μ V_c = w,  |V_c| ≤ V_max. Here the stationary standard
deviations σ_speed / σ_dir and the time constants τ = 1/μ are the preset
knobs, the perturbations are saturated at ±2σ, and the current is
spatially uniform over the world (assumption S2 in the model doc).

Direction convention: ``from_deg`` is the compass direction the current
comes FROM (the user's "from North to South"), i.e. the water moves toward
``from_deg + 180``. Oceanographers report the *set* (toward) instead; the
readback carries both to avoid the ambiguity.
"""

from __future__ import annotations

import numpy as np

from ..core.geometry import compass_deg_to_enu_yaw


class CurrentProcess:
    def __init__(self, mean_speed: float, from_deg: float,
                 sigma_speed: float = 0.0, sigma_dir_deg: float = 0.0,
                 tau_speed_s: float = 60.0, tau_dir_s: float = 120.0,
                 rng: np.random.Generator | None = None) -> None:
        self.mean_speed = max(float(mean_speed), 0.0)
        self.from_deg = float(from_deg) % 360.0
        self.sigma_speed = max(float(sigma_speed), 0.0)
        self.sigma_dir_deg = max(float(sigma_dir_deg), 0.0)
        self.tau_speed_s = max(float(tau_speed_s), 1e-3)
        self.tau_dir_s = max(float(tau_dir_s), 1e-3)
        self.rng = rng or np.random.default_rng(0)
        self.dv = 0.0       # speed perturbation [m/s]
        self.ddir = 0.0     # direction perturbation [deg]
        self.active = self.mean_speed > 0.0 or self.sigma_speed > 0.0

    def step(self, dt: float) -> None:
        """Advance the OU perturbations by ``dt`` [s] (Euler-Maruyama)."""
        if not self.active or dt <= 0.0:
            return
        if self.sigma_speed > 0.0:
            self.dv += (-self.dv / self.tau_speed_s * dt
                        + self.sigma_speed * np.sqrt(2.0 * dt / self.tau_speed_s)
                        * float(self.rng.standard_normal()))
            self.dv = float(np.clip(self.dv, -2.0 * self.sigma_speed,
                                    2.0 * self.sigma_speed))
        if self.sigma_dir_deg > 0.0:
            self.ddir += (-self.ddir / self.tau_dir_s * dt
                          + self.sigma_dir_deg * np.sqrt(2.0 * dt / self.tau_dir_s)
                          * float(self.rng.standard_normal()))
            self.ddir = float(np.clip(self.ddir, -2.0 * self.sigma_dir_deg,
                                      2.0 * self.sigma_dir_deg))

    @property
    def speed(self) -> float:
        return max(self.mean_speed + self.dv, 0.0) if self.active else 0.0

    @property
    def instant_from_deg(self) -> float:
        return (self.from_deg + self.ddir) % 360.0

    def velocity_world(self) -> np.ndarray:
        """World-frame (ENU) water velocity, shape (3,); exact zeros when
        the preset is 'none'."""
        if not self.active:
            return np.zeros(3)
        ang = compass_deg_to_enu_yaw(self.instant_from_deg + 180.0)
        v = self.speed
        return np.array([v * np.cos(ang), v * np.sin(ang), 0.0])
