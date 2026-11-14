"""A thin elastic wall in water: plane-wave reflection and transmission
(``raycast.objects.layer_model: elastic``; session 01 of the simulator
completion round, 2026-11-14).

The ray-cast lets sound through the wall of a flooded shell (a plastic box, a
polytank). Until now that wall was a FLUID layer (``scene_mesh.
layer_transmission``): no shear wave. A fluid layer stops transmitting beyond
its longitudinal critical angle (33 deg from the normal for polypropylene),
where a real solid wall still passes sound through its shear wave. This
module solves the solid wall exactly.

**The problem.** A homogeneous isotropic elastic plate of thickness ``d``
(density ``rho``, longitudinal and shear speeds ``c_L``, ``c_S``, each with an
absorption) between two half-spaces of the same water. A plane pressure wave
of unit amplitude arrives at angle ``theta`` from the normal. Inside the
plate: an up- and a down-going longitudinal wave and an up- and a down-going
vertically polarised shear wave (four amplitudes); in the water: the
reflected (``R``) and the transmitted (``T``) pressure. Six unknowns, six
boundary conditions -- at each face the normal displacement is continuous,
the normal stress equals minus the water pressure, the shear stress is zero
(an inviscid fluid carries none). Brekhovskikh & Godin, *Acoustics of Layered
Media I*, ch. 4; Folds & Loggins 1977 (the n-layer form). The 6 x 6 linear
system is solved numerically for every angle: no approximation in ``k d``.

**Checks.** Smoke ``[19]``: at normal incidence the shear wave decouples and
``T`` equals the fluid layer's closed form; without absorption ``|R|^2 +
|T|^2 = 1`` at every angle. In the session's evidence folder
(``layers/v5_plate.py``): also an independent propagator-matrix solution (the
same physics written as a first-order system and integrated through the
plate), and the rendered transmission and reflection against this module
(``layers/v5_rendered.py``, V5 of the acceptance criteria).

**Conventions.** Solved with the textbook time factor ``exp(-i omega t)``;
:func:`plate_transmission` returns ``T`` as the engine uses it -- referred to
the straight water path through the same thickness and conjugated to the
engine's ``exp(-i omega tau)`` carrier, exactly as
``scene_mesh.layer_transmission`` does, so the two are interchangeable.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: ln(10) / 20: dB -> nepers
_NP_PER_DB = np.log(10.0) / 20.0


@dataclass(frozen=True)
class Wall:
    """A wall material and thickness, as physical constants."""

    density_kgm3: float
    c_long_mps: float
    c_shear_mps: float
    thickness_m: float
    #: absorption of the longitudinal / shear wave [dB/m] at the frequency
    #: the table is built for (0 = lossless)
    alpha_long_db_m: float = 0.0
    alpha_shear_db_m: float = 0.0


def _vertical(k2: np.ndarray) -> np.ndarray:
    """sqrt on the branch of an up-going / decaying wave (Im >= 0; Re >= 0
    when it is real)."""
    s = np.sqrt(np.asarray(k2, dtype=complex))
    return np.where((s.imag < 0) | ((s.imag == 0) & (s.real < 0)), -s, s)


def plate_coefficients(cos1: np.ndarray, frequency_hz: float, wall: Wall,
                       rho_w: float, c_w: float) -> tuple[np.ndarray, np.ndarray]:
    """Pressure reflection ``R`` and transmission ``T`` of the plate for a
    unit plane wave arriving at incidence cosine ``cos1`` (textbook
    ``exp(-i omega t)``; ``R`` at the front face, ``T`` at the back face,
    both relative to the incident pressure at the front face)."""
    c1 = np.clip(np.abs(np.atleast_1d(np.asarray(cos1, dtype=np.float64))), 1e-9, 1.0)
    om = 2.0 * np.pi * float(frequency_hz)
    k1 = om / float(c_w)
    kx = k1 * np.sqrt(1.0 - c1 * c1)
    q = k1 * c1
    rho, d = float(wall.density_kgm3), float(wall.thickness_m)
    kL = om / wall.c_long_mps + 1j * wall.alpha_long_db_m * _NP_PER_DB
    kS = om / wall.c_shear_mps + 1j * wall.alpha_shear_db_m * _NP_PER_DB
    mu = rho * om * om / (kS * kS)                 # complex shear modulus
    p = _vertical(kL * kL - kx * kx)
    s = _vertical(kS * kS - kx * kx)
    m2 = mu * (kS * kS - 2.0 * kx * kx)            # rho omega^2 - 2 mu kx^2
    Ep, Es = np.exp(1j * p * d), np.exp(1j * s * d)
    yw = 1j * q / (rho_w * om * om)                # fluid: u_z = yw * (+-pressure amplitude)
    n = c1.size
    M = np.zeros((n, 6, 6), dtype=complex)
    b = np.zeros((n, 6), dtype=complex)
    # unknowns: A, B (longitudinal up/down), C, D (shear up/down), R, T
    # z = 0: u_z continuous
    M[:, 0, 0], M[:, 0, 1], M[:, 0, 2], M[:, 0, 3], M[:, 0, 4] = 1j * p, -1j * p, 1j * kx, 1j * kx, yw
    b[:, 0] = yw
    # z = 0: sigma_zz = -pressure
    M[:, 1, 0], M[:, 1, 1], M[:, 1, 2], M[:, 1, 3], M[:, 1, 4] = -m2, -m2, -2 * mu * kx * s, 2 * mu * kx * s, 1.0
    b[:, 1] = -1.0
    # z = 0: sigma_xz = 0
    M[:, 2, 0], M[:, 2, 1], M[:, 2, 2], M[:, 2, 3] = -2 * kx * p, 2 * kx * p, s * s - kx * kx, s * s - kx * kx
    # z = d
    M[:, 3, 0], M[:, 3, 1], M[:, 3, 2], M[:, 3, 3], M[:, 3, 5] = (1j * p * Ep, -1j * p / Ep, 1j * kx * Es,
                                                                 1j * kx / Es, -yw)
    M[:, 4, 0], M[:, 4, 1], M[:, 4, 2], M[:, 4, 3], M[:, 4, 5] = (-m2 * Ep, -m2 / Ep, -2 * mu * kx * s * Es,
                                                                 2 * mu * kx * s / Es, 1.0)
    M[:, 5, 0], M[:, 5, 1], M[:, 5, 2], M[:, 5, 3] = (-2 * kx * p * Ep, 2 * kx * p / Ep, (s * s - kx * kx) * Es,
                                                    (s * s - kx * kx) / Es)
    # row and column scaling: the potentials and the stresses differ by ~rho omega^2
    rs = 1.0 / np.maximum(np.abs(M).max(axis=2), 1e-300)
    Ms = M * rs[:, :, None]
    cs = 1.0 / np.maximum(np.abs(Ms).max(axis=1), 1e-300)
    x = np.linalg.solve(Ms * cs[:, None, :], (b * rs)[:, :, None])[:, :, 0] * cs
    return x[:, 4], x[:, 5]


def plate_transmission(cos1: np.ndarray, frequency_hz: float, wall: Wall,
                       rho_w: float, c_w: float) -> np.ndarray:
    """``T`` as the engine applies it to a crossing ray: referred to the
    straight water path through the wall and in the ``exp(-i omega tau)``
    convention (``scene_mesh.layer_transmission``'s own form)."""
    c1 = np.clip(np.abs(np.atleast_1d(np.asarray(cos1, dtype=np.float64))), 1e-9, 1.0)
    _, T = plate_coefficients(c1, frequency_hz, wall, rho_w, c_w)
    k1 = 2.0 * np.pi * float(frequency_hz) / float(c_w)
    return np.conj(T * np.exp(-1j * k1 * c1 * wall.thickness_m))


class WallTable:
    """``T(theta)`` (engine form) and ``|R(theta)|`` of one wall, tabulated
    every ``step_deg`` from normal to grazing incidence; linear
    interpolation in the angle."""

    def __init__(self, wall: Wall, frequency_hz: float, rho_w: float, c_w: float,
                 step_deg: float = 0.1) -> None:
        self.wall = wall
        self.step_deg = float(step_deg)
        self.theta_deg = np.arange(0.0, 90.0 + 0.5 * step_deg, step_deg)
        c1 = np.cos(np.radians(np.minimum(self.theta_deg, 89.999)))
        R, _ = plate_coefficients(c1, frequency_hz, wall, rho_w, c_w)
        self.T = plate_transmission(c1, frequency_hz, wall, rho_w, c_w)
        self.R_abs = np.abs(R)

    def _frac(self, cos1: np.ndarray):
        th = np.degrees(np.arccos(np.clip(np.abs(np.asarray(cos1, float)), 0.0, 1.0)))
        x = np.clip(th / self.step_deg, 0.0, self.theta_deg.size - 1.000001)
        i = x.astype(np.int64)
        return i, x - i

    def transmission(self, cos1: np.ndarray) -> np.ndarray:
        i, f = self._frac(cos1)
        return (1.0 - f) * self.T[i] + f * self.T[i + 1]

    def reflection(self, cos1: np.ndarray) -> np.ndarray:
        i, f = self._frac(cos1)
        return (1.0 - f) * self.R_abs[i] + f * self.R_abs[i + 1]
