"""The transducer's directivity: a rectangular aperture (note 09).

Along track, a line aperture of length ``L`` evaluated in the **near field**:
the Omniscan 450 SS's two-way 3 dB width of 0.5 deg (datasheet) implies a
uniform aperture of L = 24.4 cm, whose far field begins at L^2/lambda = 17.8 m
(datasheet: "Far Field: 15 m") -- so the whole 20 m swath is near field and
the footprint there is not R*theta. The amplitude pattern is the aperture
integral over exact distances,

    b_a(x) = (1/L) * integral_{-L/2}^{L/2} w(u) exp(-i k (|x - u e_a| - |x|)) du,

by Gauss-Legendre quadrature with a node count that resolves the phase across
the aperture; it reduces to sinc(k L s_a / 2) in the far field (gate B2).

**Sign of the phase (2026-11-13, a bug fix).** Every contributor is deposited
with the carrier ``exp(-i omega tau)`` (``receiver.BinGrid.add``): a path that
is longer by ``dr`` carries ``exp(-i k dr)``. An aperture point at ``u`` is
``|x - u e_a| - |x|`` farther from x than the centre, so its term is
``exp(-i k (...))``. Until 2026-11-13 the integral was summed with ``+i``: the
complex conjugate. ``|b|`` is the same, so a point target, the beam widths and
the diffuse seabed did not see it (the seabed mean moves by < 1e-8 dB). A
coherent sum over an extended smooth surface in the aperture's near field
did: against an independent Kirchhoff integral a 0.72 m plate at 6 m was
5.2 dB too weak at normal incidence and 9-11 dB too strong 1-2 deg off it;
with the sign fixed it agrees within 0.1-0.6 dB (session 01 of the simulator
completion round, V3; ``~/ros2_ws/sim_runs/sim_acceptance/01_sonar_physics``).
``legacy_phase`` keeps the old sum for the dated profile copies only.

Across track (in the fan plane), a vertical aperture ``H`` (3.5 mm for a
50 deg one-way height) is in its far field beyond ~4 mm: ``sinc(k H s_v / 2)``
about the axis depressed by the mounting tilt. Nothing radiates behind the
face (baffled piston). Bell & Linnett 1997 eq. 13-14 (product of two line
arrays); J&R 2007 App. F (far-field condition F.1).

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _sinc(x: np.ndarray) -> np.ndarray:
    """sin(x)/x (numpy's sinc is sin(pi x)/(pi x))."""
    return np.sinc(np.asarray(x) / np.pi)


def uniform_half_power_u(two_way: bool) -> float:
    """``u = k L s / 2`` at the half-power point of a uniform aperture:
    sinc^2(u) = 1/2 one-way, sinc^4(u) = 1/2 two-way (bisection)."""
    target = 0.5 ** (0.25 if two_way else 0.5)
    lo, hi = 1e-6, np.pi
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if np.sinc(mid / np.pi) > target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def aperture_from_width(width_deg: float, k: float, two_way: bool) -> float:
    """Uniform aperture length [m] whose -3 dB full width is ``width_deg``."""
    u = uniform_half_power_u(two_way)
    return 2.0 * u / (k * np.sin(np.radians(0.5 * width_deg)))


def taper(x: np.ndarray, pedestal: float) -> np.ndarray:
    """Amplitude weighting of the line aperture at ``x = 2 u / L`` in
    [-1, 1]: a cosine on a pedestal, ``p + (1 - p) cos(pi x / 2)``; ``p = 1``
    is the uniform aperture."""
    return pedestal + (1.0 - pedestal) * np.cos(0.5 * np.pi * np.asarray(x, float))


def aperture_from_width_tapered(width_deg: float, k: float, pedestal: float) -> float:
    """Aperture length [m] whose TWO-WAY -3 dB full width is ``width_deg``
    with the cosine-on-pedestal weighting (far field; bisection on the
    pattern, 4001-point quadrature). ``pedestal = 1`` reproduces
    :func:`aperture_from_width` to 1e-6."""
    x = (np.arange(4001) + 0.5) / 4001 * 2.0 - 1.0
    w = taper(x, pedestal)
    s = np.sin(np.radians(0.5 * width_deg))
    lo, hi = 0.02, 2.0
    for _ in range(80):
        L = 0.5 * (lo + hi)
        b = abs(np.sum(w * np.exp(1j * k * 0.5 * L * x * s)) / w.sum())
        if b ** 4 > 0.5:
            lo = L
        else:
            hi = L
    return 0.5 * (lo + hi)


@dataclass(frozen=True)
class Aperture:
    """Rectangular aperture, in the transducer frame.

    ``e_a`` array axis (along track), ``e_n`` acoustic axis (look direction
    depressed by the tilt), ``e_v`` = ``e_n x e_a`` (in the fan plane,
    pointing up-ish), all unit, world frame."""

    length_m: float              # along-track aperture L
    height_m: float              # vertical aperture H
    k: float                     # wavenumber [rad/m]
    #: One-way amplitude of the large-angle vertical response that a pure
    #: strip's sinc lacks (diffraction at the baffle / housing edges, the
    #: housing being 20-29 mm across a ~3.5 mm element): added in POWER,
    #: ``b_v = sqrt(sinc^2 + floor^2)``. Unpublished for the Omniscan;
    #: bounded [0, 0.3], set by the NC #4 step / Phase 7 (raycast_design §8).
    vertical_floor: float = 0.0
    #: Evaluate the near-field integral on the GPU (gpu_warp.near_pattern;
    #: set by the renderer for ``backend: warp``; G1 holds it to the CPU).
    gpu: bool = False
    #: The pre-2026-11-13 sign of the near-field phase (``exp(+i k dr)``, the
    #: conjugate of what the carrier's convention requires; module docstring).
    #: Only ``raycast.beam.legacy_phase_sign`` sets it, for the dated profile
    #: copies that must reproduce the renders of 2026-11-12.
    legacy_phase: bool = False
    #: Amplitude weighting along the aperture (:func:`taper`): 1 = uniform (the
    #: default, every result before 2026-11-14); < 1 = a cosine on that
    #: pedestal (``raycast.beam.taper_pedestal``), lower side lobes for a
    #: longer aperture at the same -3 dB width.
    pedestal: float = 1.0

    def pattern(self, d: np.ndarray, e_a: np.ndarray, e_n: np.ndarray,
                e_v: np.ndarray, near_field: bool = True,
                chunk: int = 20000) -> np.ndarray:
        """Complex one-way amplitude pattern at offsets ``d`` (n, 3) from the
        aperture centre. 1 on axis in the far field."""
        d = np.atleast_2d(np.asarray(d, dtype=np.float64))
        R = np.linalg.norm(d, axis=1)
        Rs = np.maximum(R, 1e-9)
        s_a = d @ e_a / Rs
        s_v = d @ e_v / Rs
        s_n = d @ e_n / Rs
        vert = _sinc(0.5 * self.k * self.height_m * s_v)
        if self.vertical_floor > 0.0:
            vert = np.sqrt(vert * vert + self.vertical_floor ** 2)
        front = s_n > 0.0
        if not near_field:
            if self.pedestal != 1.0:
                raise NotImplementedError("a tapered aperture is evaluated in the near field only")
            along = _sinc(0.5 * self.k * self.length_m * s_a).astype(complex)
        else:
            along = np.empty(d.shape[0], dtype=complex)
            for a in range(0, d.shape[0], chunk):
                b = min(a + chunk, d.shape[0])
                along[a:b] = self._near(d[a:b], R[a:b], s_a[a:b], e_a)
        return np.where(front, along * vert, 0.0)

    def parts(self, d: np.ndarray, e_a: np.ndarray, e_n: np.ndarray,
              e_v: np.ndarray, near_field: bool = True, chunk: int = 20000):
        """The pattern's factors at offsets ``d``: (along-track complex
        amplitude, the vertical strip's sinc, front mask). ``pattern`` is
        ``front * along * sqrt(sinc^2 + vertical_floor^2)`` (or ``* sinc``
        when the floor is 0); the fit basis (``fitbasis.py``) re-evaluates the
        floor from these."""
        d = np.atleast_2d(np.asarray(d, dtype=np.float64))
        R = np.linalg.norm(d, axis=1)
        Rs = np.maximum(R, 1e-9)
        s_a = d @ e_a / Rs
        s_v = d @ e_v / Rs
        front = (d @ e_n / Rs) > 0.0
        sinc_v = _sinc(0.5 * self.k * self.height_m * s_v)
        if not near_field:
            along = _sinc(0.5 * self.k * self.length_m * s_a).astype(complex)
        else:
            along = np.empty(d.shape[0], dtype=complex)
            for a in range(0, d.shape[0], chunk):
                b = min(a + chunk, d.shape[0])
                along[a:b] = self._near(d[a:b], R[a:b], s_a[a:b], e_a)
        return along, sinc_v, front

    def _near(self, d: np.ndarray, R: np.ndarray, s_a: np.ndarray,
              e_a: np.ndarray) -> np.ndarray:
        """Line-aperture integral over exact distances (Gauss-Legendre)."""
        L, k = self.length_m, self.k
        # Phase gradient across the aperture: |d/du (k |d - u e_a|)| <=
        # k (|s_a| + L / (2 R)) -- nodes to resolve it (>= 6 per 2 pi).
        grad = k * (np.max(np.abs(s_a)) + L / (2.0 * max(np.min(R), 1e-3)))
        n = int(np.clip(np.ceil(grad * L / (2.0 * np.pi) * 6.0) + 16, 32, 4096))
        x, w = _gauss_legendre(n)
        u = 0.5 * L * x                                   # (n,)
        if self.pedestal != 1.0:        # weighted aperture, normalised like the uniform one (sum w = 2)
            w = w * taper(x, self.pedestal)
            w = w * (2.0 / w.sum())
        if self.gpu:
            from .gpu_warp import near_pattern
            return near_pattern(d, e_a, u, w, k, legacy_phase=self.legacy_phase)
        # |d - u e_a| = sqrt(R^2 - 2 u (d.e_a) + u^2), stable for R >> u
        da = (d @ e_a)[:, None]
        r = np.sqrt(np.maximum(R[:, None] ** 2 - 2.0 * u[None, :] * da
                               + u[None, :] ** 2, 0.0))
        # exp(-i k dr): the carrier's convention (module docstring)
        ph = (np.exp(1j * k * (r - R[:, None])) if self.legacy_phase
              else np.exp(-1j * k * (r - R[:, None])))
        # Spherical spreading of each element relative to the centre.
        amp = R[:, None] / np.maximum(r, 1e-9)
        return 0.5 * (ph * amp) @ w


_GL_CACHE: dict = {}


def _gauss_legendre(n: int):
    """Memoised Gauss-Legendre nodes and weights (numpy recomputes them by an
    eigenvalue solve on every call)."""
    g = _GL_CACHE.get(n)
    if g is None:
        g = _GL_CACHE[n] = np.polynomial.legendre.leggauss(n)
    return g


def transducer_frame(yaw: float, pitch: float, roll: float, side_sign: float,
                     tilt_rad: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(e_a, e_n, e_v) in the world frame for a transducer on side
    ``side_sign`` (+1 port, -1 starboard), its axis depressed by ``tilt``
    below the body's horizontal athwartship direction, rotated by the full
    attitude (ZYX, the renderer's ``rotation_zyx`` convention)."""
    from ...core.geometry import rotation_zyx
    R = rotation_zyx(roll, pitch, yaw)
    e_a = R @ np.array([1.0, 0.0, 0.0])
    e_n = R @ np.array([0.0, side_sign * np.cos(tilt_rad), -np.sin(tilt_rad)])
    e_v = np.cross(e_n, e_a)        # the vertical pattern is even in s_v
    return e_a, e_n, e_v
