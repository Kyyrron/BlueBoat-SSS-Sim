"""Directional linear wave field synthesised from a JONSWAP spectrum.

η(x, y, t) = Σ_ij a_ij cos(k_i (x cos θ_j + y sin θ_j) − ω_i t + φ_ij)

with a_ij² = 2 · (m0 / N_f) · D_j, equal-energy frequency bins
(:func:`spectrum.equal_energy_frequencies`), a cos^{2s} directional
spreading D_j (Longuet-Higgins, Cartwright & Smith 1963) about the
propagation direction, finite-depth dispersion, and seeded uniform random
phases. Surface orbital velocities follow from the same components
(Airy theory at z = 0). Pure NumPy; every method is vectorised over query
points, so the hull integration evaluates hundreds of points per tick.

Direction convention: ``from_deg`` is the compass direction the waves come
FROM (meteorological, like wind); they travel toward ``from_deg + 180``.
"""

from __future__ import annotations

import numpy as np

from ..core.geometry import compass_deg_to_enu_yaw
from .spectrum import dispersion, equal_energy_frequencies


def _spreading_weights(n_dir: int, spread_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Relative directions (rad, about the propagation direction) and their
    normalised cos^{2s}(Δθ/2) weights (sum 1)."""
    if n_dir <= 1:
        return np.zeros(1), np.ones(1)
    half = np.pi / 2.0
    rel = np.linspace(-half, half, n_dir)
    w = np.cos(rel / 2.0) ** (2.0 * spread_s)
    return rel, w / w.sum()


class WaveField:
    """One sea state's wave field; ``hs == 0`` is an exact no-op."""

    def __init__(self, hs: float, tp: float, gamma: float, from_deg: float,
                 spread_s: float, depth_m: float,
                 n_freq: int = 16, n_dir: int = 12,
                 rng: np.random.Generator | None = None,
                 detune: float = 0.04) -> None:
        # n_dir 6 -> 12 (2026-09-05): six bearings 36° apart with s = 6 put
        # three quarters of the energy in two rays -- a long-crested sea
        # whose moment sat about one fixed world axis. Twelve bearings and
        # the presets' s = 3 make a short-crested harbour chop that rolls
        # *and* pitches the boat at every heading.
        self.hs = float(hs)
        self.tp = float(tp)
        self.gamma = float(gamma)
        self.from_deg = float(from_deg) % 360.0
        self.spread_s = float(spread_s)
        self.depth_m = float(depth_m)
        self.active = self.hs > 0.0 and self.tp > 0.0
        if not self.active:
            self.omega = np.zeros(0)
            return
        rng = rng or np.random.default_rng(0)
        omega, m0 = equal_energy_frequencies(self.hs, self.tp, self.gamma, n_freq)
        rel, dw = _spreading_weights(n_dir, self.spread_s)
        theta0 = compass_deg_to_enu_yaw(self.from_deg + 180.0)
        # Component arrays, flattened (N_f * N_dir,).
        om = np.repeat(omega, len(rel))
        # A small *random* frequency offset per component keeps the
        # components of one frequency bin from adding coherently at every
        # point (which would make the local variance a random draw) --
        # random rather than a fixed ladder, whose ±3 % steps beat with a
        # period of their own and read as a regular swell (2026-09-05).
        # Drawn before the phases, so a seed still fixes the whole field.
        om = om * (1.0 + rng.uniform(-float(detune), float(detune), size=om.shape))
        th = theta0 + np.tile(rel, len(omega))
        amp = np.sqrt(2.0 * (m0 / n_freq) * np.tile(dw, len(omega)))
        k = dispersion(om, self.depth_m)
        self.omega = om
        self.k = k
        self.amp = amp
        self.cos_t = np.cos(th)
        self.sin_t = np.sin(th)
        self.phase0 = rng.uniform(0.0, 2.0 * np.pi, size=om.shape)
        # Surface orbital amplitude factor: cosh(kh)/sinh(kh) at z = 0.
        kh = np.clip(k * self.depth_m, 1e-6, 50.0)
        self.horiz_factor = 1.0 / np.tanh(kh)
        self.m0 = float(m0)
        self.theta0 = float(theta0)

    # ----------------------------------------------------------- evaluation
    def _phase(self, x: np.ndarray, y: np.ndarray, t: float) -> np.ndarray:
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))[:, None]
        y = np.atleast_1d(np.asarray(y, dtype=np.float64))[:, None]
        return (self.k * (x * self.cos_t + y * self.sin_t)
                - self.omega * t + self.phase0)

    def elevation(self, x, y, t: float) -> np.ndarray:
        """Surface elevation η [m] at the points (x, y), time t [s]."""
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        if not self.active:
            return np.zeros(x.shape)
        return (self.amp * np.cos(self._phase(x, y, t))).sum(axis=1)

    def slope(self, x, y, t: float) -> tuple[np.ndarray, np.ndarray]:
        """(∂η/∂x, ∂η/∂y) at the points."""
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        if not self.active:
            z = np.zeros(x.shape)
            return z, z.copy()
        s = -self.amp * self.k * np.sin(self._phase(x, y, t))
        return (s * self.cos_t).sum(axis=1), (s * self.sin_t).sum(axis=1)

    def orbital_velocity(self, x, y, t: float) -> np.ndarray:
        """Water particle velocity at the surface, world frame, shape (P, 3):
        horizontal a ω coth(kh) cos(phase) along propagation, vertical
        a ω sin(phase)."""
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        if not self.active:
            return np.zeros((x.shape[0], 3))
        ph = self._phase(x, y, t)
        aw = self.amp * self.omega
        horiz = aw * self.horiz_factor * np.cos(ph)
        vert = aw * np.sin(ph)
        return np.stack([(horiz * self.cos_t).sum(axis=1),
                         (horiz * self.sin_t).sum(axis=1),
                         vert.sum(axis=1)], axis=1)

    def peak_wavelength_m(self) -> float:
        if not self.active:
            return 0.0
        return float(2.0 * np.pi / dispersion(2.0 * np.pi / self.tp, self.depth_m)[0])
