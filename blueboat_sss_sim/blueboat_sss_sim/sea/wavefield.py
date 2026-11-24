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
from .spectrum import dispersion, equal_energy_frequencies, hybrid_frequencies


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
                 detune: float = 0.04, *,
                 discretisation: str = "equal_energy",
                 f_max_hz: float = 3.0) -> None:
        """``discretisation`` ``equal_energy`` (the default, bit-identical
        to every field before 2026-11-10; the wake groups use it) or
        ``hybrid`` (:func:`spectrum.hybrid_frequencies`: the slope spectrum
        resolved up to ``f_max_hz``; ``n_freq`` is then the cap on the
        number of frequencies)."""
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
        if discretisation == "equal_energy":
            omega, m0 = equal_energy_frequencies(self.hs, self.tp, self.gamma, n_freq)
            var = np.full(len(omega), m0 / n_freq)
        elif discretisation == "hybrid":
            omega, var = hybrid_frequencies(self.hs, self.tp, self.gamma,
                                            self.depth_m, f_max_hz=f_max_hz,
                                            max_total=max(int(n_freq), 13))
            m0 = float(var.sum())
        else:
            raise ValueError(f"wave discretisation must be 'equal_energy' or "
                             f"'hybrid', got {discretisation!r}")
        self.discretisation = discretisation
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
        amp = np.sqrt(2.0 * np.repeat(var, len(rel)) * np.tile(dw, len(omega)))
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
        self._decay_cache: dict[float, np.ndarray] = {}

    # --------------------------------------------------------- draft decay
    def decay(self, depth: float) -> np.ndarray:
        """Per-component linear pressure attenuation at ``depth`` below the
        mean surface: cosh(k(h - d)) / cosh(k h) (Airy, finite depth;
        e^(-k d) in deep water), written overflow-free. The Froude-Krylov
        pressure on a hull bottom at the draft is this fraction of the
        surface elevation's ("Smith effect"; Journee & Massie 2001 eq.
        8.43): 0.83 at 1 Hz, 0.49 at 2 Hz for the BlueBoat's 4.5 cm."""
        d = float(depth)
        f = self._decay_cache.get(d)
        if f is None:
            h = max(self.depth_m, d + 1e-6)
            kd = self.k * d
            f = (np.exp(-kd) * (1.0 + np.exp(-2.0 * self.k * (h - d)))
                 / (1.0 + np.exp(-2.0 * self.k * h)))
            self._decay_cache[d] = f
        return f

    def _amp(self, depth: float) -> np.ndarray:
        return self.amp if depth <= 0.0 else self.amp * self.decay(depth)

    # ----------------------------------------------------------- evaluation
    def _phase(self, x: np.ndarray, y: np.ndarray, t: float) -> np.ndarray:
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))[:, None]
        y = np.atleast_1d(np.asarray(y, dtype=np.float64))[:, None]
        return (self.k * (x * self.cos_t + y * self.sin_t)
                - self.omega * t + self.phase0)

    def elevation(self, x, y, t: float, depth: float = 0.0) -> np.ndarray:
        """Surface elevation η [m] at the points (x, y), time t [s]; with
        ``depth`` > 0 the pressure-equivalent elevation at that depth below
        the mean surface (each component attenuated by :meth:`decay`;
        ``depth`` 0 is bit-identical to the plain surface)."""
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        if not self.active:
            return np.zeros(x.shape)
        return (self._amp(depth) * np.cos(self._phase(x, y, t))).sum(axis=1)

    def slope(self, x, y, t: float, depth: float = 0.0
              ) -> tuple[np.ndarray, np.ndarray]:
        """(∂η/∂x, ∂η/∂y) at the points (attenuated like :meth:`elevation`
        at ``depth`` > 0)."""
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        if not self.active:
            z = np.zeros(x.shape)
            return z, z.copy()
        s = -self._amp(depth) * self.k * np.sin(self._phase(x, y, t))
        return (s * self.cos_t).sum(axis=1), (s * self.sin_t).sum(axis=1)

    def elevation_and_slope(self, x, y, t: float, depth: float = 0.0
                            ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(η, ∂η/∂x, ∂η/∂y) from ONE phase evaluation -- the hull samples
        both at the same points every tick; identical to calling
        :meth:`elevation` and :meth:`slope`."""
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))
        if not self.active:
            z = np.zeros(x.shape)
            return z, z.copy(), z.copy()
        ph = self._phase(x, y, t)
        a = self._amp(depth)
        eta = (a * np.cos(ph)).sum(axis=1)
        s = -a * self.k * np.sin(ph)
        return eta, (s * self.cos_t).sum(axis=1), (s * self.sin_t).sum(axis=1)

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
