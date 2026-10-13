"""Ocean current: a mean, plus smooth, bounded, reproducible variation.

The configured speed and direction are the **mean** current. Around them
the simulator adds two small variations, both deterministic functions of
the run seed (same seed + same scenario = same current, whatever the ROS
timer does):

* **in time** -- Fossen (2011) §8.3 (doi:10.1002/9781119994138) models the
  speed and direction of a current as first-order Gauss-Markov processes
  with saturation, ``V̇ + μ V = w, V_min ≤ V ≤ V_max``; the UUV Simulator
  and DAVE Gazebo plugins implement exactly that, stepped per physics
  update. Here the process is **sampled exactly** on a fixed sim-time grid
  (the AR(1) form of Gillespie 1996, doi:10.1103/PhysRevE.54.2084, exact for
  any step) and **joined smoothly** (Catmull-Rom, C1), so the water never
  jerks from tick to tick, and it is saturated with a smooth ``tanh`` bound
  instead of a hard clip. Speed and direction each get one unit-variance
  process, scaled by the state's ``sigma``: a ramp or a schedule changes
  the means and sigmas, never the realisation, so a change of situation
  does not restart the variation;
* **in space** -- a weak, divergence-free eddy field (random Fourier modes
  of a stream function, Kraichnan 1970, doi:10.1063/1.1692799) with
  wavelengths of 25-100 m that evolves over 5-15 minutes, so two survey
  lines a few metres apart, or the same line sailed back, feel a slightly
  different, repeatable current. Its scale and strength are judgement
  values for a harbour basin (no site measurement exists) and are labelled
  as such in ``docs/sea_state_model.md``.

Direction convention: ``from_deg`` is the compass direction the current
comes FROM (the user's "from North to South"); the water moves toward
``from_deg + 180``. Oceanographers report the *set* (toward); the readback
carries both to avoid the ambiguity.
"""

from __future__ import annotations

import numpy as np

from ..core.geometry import compass_deg_to_enu_yaw

#: Soft bound of a unit fluctuation, in raw-process standard deviations.
BOUND_SIGMAS = 2.5
#: std of BOUND·tanh(x/BOUND) for x ~ N(0, 1) (quadrature; B = 2.5), so the
#: bounded fluctuation is rescaled back to the configured sigma.
_TANH_STD = 0.8798787526975604
#: Hard ceiling of the rescaled fluctuation, in configured sigmas (≈ 2.84).
FLUCTUATION_LIMIT = BOUND_SIGMAS / _TANH_STD


def bounded(xi):
    """Unit-std, smooth (C∞), odd and bounded image of a unit Gaussian
    fluctuation: ``|bounded(xi)| < FLUCTUATION_LIMIT``."""
    return (BOUND_SIGMAS / _TANH_STD) * np.tanh(np.asarray(xi) / BOUND_SIGMAS)


class UnitGaussMarkov:
    """Unit-variance first-order Gauss-Markov process ξ(t), t ≥ 0.

    Knots on the fixed grid ``t_j = (j - 1)·Δ`` follow the exact AR(1)
    recursion ``ξ_{j+1} = a ξ_j + sqrt(1 - a²) n_j``, ``a = exp(-Δ/τ)``,
    from a stationary start ``ξ_0 ~ N(0, 1)``. The normals of knot block
    ``b`` come from ``default_rng([*key, b])``, so the realisation is a
    function of ``(key, t)`` only -- never of how often or when it is
    evaluated. Between knots a Catmull-Rom spline (C1) joins them; its
    variance dips at most 2 % below 1 mid-interval at Δ = τ/12.
    """

    BLOCK = 256

    def __init__(self, tau_s: float, key: tuple[int, ...],
                 knot_dt_s: float | None = None) -> None:
        self.tau_s = max(float(tau_s), 1e-3)
        self.dt = (float(np.clip(self.tau_s / 12.0, 0.5, 5.0))
                   if knot_dt_s is None else float(knot_dt_s))
        self.a = float(np.exp(-self.dt / self.tau_s))
        self.b = float(np.sqrt(1.0 - self.a * self.a))
        self.key = tuple(int(k) for k in key)
        self._knots = np.zeros(0)

    def _ensure(self, j_max: int) -> None:
        while self._knots.size <= j_max:
            blk = self._knots.size // self.BLOCK
            z = np.random.default_rng([*self.key, blk]).standard_normal(self.BLOCK)
            out = np.empty(self.BLOCK)
            prev = self._knots[-1] if self._knots.size else None
            for i in range(self.BLOCK):
                prev = z[i] if prev is None else self.a * prev + self.b * z[i]
                out[i] = prev
            self._knots = np.concatenate([self._knots, out])

    def __call__(self, t):
        """ξ at time(s) ``t`` [s]; negative times read as 0."""
        t = np.maximum(np.asarray(t, dtype=np.float64), 0.0)
        s = t / self.dt
        k = np.floor(s).astype(np.int64)
        u = s - k
        j = k + 1                                  # t sits between knots j, j+1
        self._ensure(int(np.max(j)) + 2)
        p0, p1 = self._knots[j - 1], self._knots[j]
        p2, p3 = self._knots[j + 1], self._knots[j + 2]
        u2, u3 = u * u, u * u * u
        out = 0.5 * (2.0 * p1 + (p2 - p0) * u
                     + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2
                     + (3.0 * p1 - p0 - 3.0 * p2 + p3) * u3)
        return float(out) if out.ndim == 0 else out


class EddyField:
    """Weak divergence-free horizontal velocity field with unit RMS.

    ``ψ(x, y, t) = Σ_n (c/|k_n|) sin(k_n·r + ω_n t + φ_n)``,
    ``u = ∂ψ/∂y, v = -∂ψ/∂x`` -- each mode is a shear wave whose velocity
    lies along its crests, so ``∇·u = 0`` exactly (Kraichnan's random
    Fourier modes). Wavelengths log-uniform in ``wavelength_m``, directions
    uniform, periods uniform in ``period_s``, phases uniform; ``c`` makes
    the spatial RMS of the vector field 1 m/s, and ``|u| ≤ N c`` always.
    Multiply by the state's ``spatial_rms_mps``.
    """

    def __init__(self, seed: int, n_modes: int = 8,
                 wavelength_m: tuple[float, float] = (25.0, 100.0),
                 period_s: tuple[float, float] = (300.0, 900.0)) -> None:
        rng = np.random.default_rng([int(seed), 21])
        lam = np.exp(rng.uniform(np.log(wavelength_m[0]), np.log(wavelength_m[1]),
                                 n_modes))
        th = rng.uniform(0.0, 2.0 * np.pi, n_modes)
        kmag = 2.0 * np.pi / lam
        self.kx, self.ky = kmag * np.cos(th), kmag * np.sin(th)
        self.khat = np.stack([np.cos(th), np.sin(th)], 1)
        self.omega = 2.0 * np.pi / rng.uniform(period_s[0], period_s[1], n_modes)
        self.phase = rng.uniform(0.0, 2.0 * np.pi, n_modes)
        self.c = float(np.sqrt(2.0 / n_modes))
        self.n_modes = int(n_modes)

    @property
    def bound(self) -> float:
        return self.n_modes * self.c

    def velocity(self, x, y, t: float) -> np.ndarray:
        """Unit-RMS (u, v) at the points, shape (P, 2)."""
        x = np.atleast_1d(np.asarray(x, dtype=np.float64))[:, None]
        y = np.atleast_1d(np.asarray(y, dtype=np.float64))[:, None]
        amp = self.c * np.cos(self.kx * x + self.ky * y + self.omega * t + self.phase)
        # velocity of mode n = amp_n * (khat_y, -khat_x)
        return np.stack([(amp * self.khat[:, 1]).sum(axis=1),
                         (-amp * self.khat[:, 0]).sum(axis=1)], axis=1)


class CurrentField:
    """The current of one run: mean + Gauss-Markov variation in time + eddy
    variation in space, deterministic per ``seed``.

    ``velocity(t, x, y, spec)`` takes any object with the
    :class:`~.presets.CurrentSpec` fields (``speed_mps``, ``from_deg``,
    ``sigma_speed_mps``, ``sigma_dir_deg``, ``tau_speed_s``, ``tau_dir_s``,
    ``spatial_rms_mps``) and returns the world-frame (ENU) water velocity,
    shape (3,). A spec with no speed, no sigma and no spatial part returns
    exact zeros (the 'none' no-op). ``spec_b``/``w`` crossfade two specs
    whose time constants differ (a variance-preserving blend of their two
    independent processes); with equal time constants one process serves
    both, so a ramp never restarts the variation.
    """

    def __init__(self, seed: int) -> None:
        self.seed = int(seed)
        self._procs: dict[tuple[str, float], UnitGaussMarkov] = {}
        self.eddies = EddyField(self.seed)

    def unit(self, which: str, tau_s: float) -> UnitGaussMarkov:
        key = (which, round(float(tau_s), 6))
        proc = self._procs.get(key)
        if proc is None:
            stream = 11 if which == "speed" else 12
            proc = UnitGaussMarkov(tau_s, (self.seed, stream,
                                           int(round(float(tau_s) * 1000.0))))
            self._procs[key] = proc
        return proc

    def _xi(self, which: str, t: float, tau_a: float, tau_b: float, w: float) -> float:
        if abs(tau_a - tau_b) < 1e-9 or w <= 0.0:
            return float(self.unit(which, tau_a)(t))
        if w >= 1.0:
            return float(self.unit(which, tau_b)(t))
        xa, xb = self.unit(which, tau_a)(t), self.unit(which, tau_b)(t)
        return float(((1.0 - w) * xa + w * xb) / np.hypot(1.0 - w, w))

    @staticmethod
    def is_active(spec) -> bool:
        return (spec.speed_mps > 0.0 or spec.sigma_speed_mps > 0.0
                or getattr(spec, "spatial_rms_mps", 0.0) > 0.0)

    def components(self, t: float, x: float, y: float, spec, spec_b=None,
                   w: float = 0.0) -> dict:
        """The parts of the current at (t, x, y): mean vector, temporal
        fluctuation and spatial (eddy) part, each (3,) world ENU."""
        zeros = np.zeros(3)
        if not self.is_active(spec):
            return {"mean": zeros, "temporal": zeros.copy(), "spatial": zeros.copy(),
                    "total": zeros.copy()}
        sb = spec_b if spec_b is not None else spec
        xs = self._xi("speed", t, spec.tau_speed_s, sb.tau_speed_s, w)
        xd = self._xi("dir", t, spec.tau_dir_s, sb.tau_dir_s, w)
        u_mean = max(float(spec.speed_mps), 0.0)
        speed = max(u_mean + float(spec.sigma_speed_mps) * float(bounded(xs)), 0.0)
        frm = float(spec.from_deg) + float(spec.sigma_dir_deg) * float(bounded(xd))
        ang_mean = compass_deg_to_enu_yaw(float(spec.from_deg) + 180.0)
        ang = compass_deg_to_enu_yaw(frm + 180.0)
        mean = np.array([u_mean * np.cos(ang_mean), u_mean * np.sin(ang_mean), 0.0])
        inst = np.array([speed * np.cos(ang), speed * np.sin(ang), 0.0])
        rms = float(getattr(spec, "spatial_rms_mps", 0.0))
        spatial = np.zeros(3)
        if rms > 0.0:
            uv = self.eddies.velocity(x, y, t)[0]
            spatial[:2] = rms * uv
        return {"mean": mean, "temporal": inst - mean, "spatial": spatial,
                "total": inst + spatial}

    def velocity(self, t: float, x: float, y: float, spec, spec_b=None,
                 w: float = 0.0) -> np.ndarray:
        return self.components(t, x, y, spec, spec_b, w)["total"]


class CurrentProcess:
    """One fixed current specification evaluated over time (and space) --
    a convenience wrapper around :class:`CurrentField` for tools and tests.

    ``velocity_at(t, x, y)`` is a pure function of the arguments and the
    seed. The ``speed`` / ``instant_from_deg`` readouts refer to the last
    evaluated time.
    """

    def __init__(self, mean_speed: float, from_deg: float,
                 sigma_speed: float = 0.0, sigma_dir_deg: float = 0.0,
                 tau_speed_s: float = 60.0, tau_dir_s: float = 120.0,
                 spatial_rms: float = 0.0, seed: int = 0) -> None:
        from types import SimpleNamespace
        self.spec = SimpleNamespace(
            speed_mps=max(float(mean_speed), 0.0), from_deg=float(from_deg) % 360.0,
            sigma_speed_mps=max(float(sigma_speed), 0.0),
            sigma_dir_deg=max(float(sigma_dir_deg), 0.0),
            tau_speed_s=max(float(tau_speed_s), 1e-3),
            tau_dir_s=max(float(tau_dir_s), 1e-3),
            spatial_rms_mps=max(float(spatial_rms), 0.0))
        self.field = CurrentField(seed)
        self.active = CurrentField.is_active(self.spec)

    def velocity_at(self, t: float, x: float = 0.0, y: float = 0.0) -> np.ndarray:
        return self.field.velocity(t, x, y, self.spec)

    def series(self, t, x: float = 0.0, y: float = 0.0) -> np.ndarray:
        """Velocities at many times, shape (T, 3)."""
        return np.array([self.velocity_at(float(ti), x, y) for ti in np.atleast_1d(t)])
