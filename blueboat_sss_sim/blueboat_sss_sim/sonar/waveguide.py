"""Shallow-water multipath of a wall's (or smooth object's) echo: the image comb.

Pass 4 (2026-11-02). In water of depth ``D`` with a free surface (z = 0) and a
floor (z = -D), every path from the transducer to a scatterer and back may
bounce off the surface and the floor any number of times. Unfolding the
waveguide across both boundaries turns each such path into a straight line
from an *image* of the transducer (image method: Jensen, Kuperman, Porter &
Schmidt, *Computational Ocean Acoustics*, 2nd ed., Springer 2011, §2.4,
doi:10.1007/978-1-4419-8678-8; Allen & Berkley, J. Acoust. Soc. Am. 65(4),
943-950, 1979, doi:10.1121/1.382599). With depths counted positive down,
transducer depth ``z_t``:

    image depths   d = 2 m D + z_t   and   d = 2 m D - z_t ,   m in Z

and the number of boundary crossings of the unfolded line between ``z_t``
and ``d`` gives the bounce counts: crossing ``z = k D`` with ``k`` odd is a
floor bounce, with ``k`` even a surface bounce. A wall (or object) point at
depth ``z_w`` and horizontal distance ``f`` is reached from image ``i`` along
``l_i = hypot(f, z_w - d_i)``; the out/back pair ``(i, j)`` arrives at slant
``(l_i + l_j) / 2``. The direct echo is the pair (0, 0).

Each bounce keeps the coherent part of a plane-wave reflection (the image
path is a specular ray):

* floor: ``|R(psi)|^2 * C(psi)`` -- the fluid-fluid Rayleigh coefficient
  (Brekhovskikh & Lysanov, *Fundamentals of Ocean Acoustics*, 3rd ed.,
  Springer 2003, ch. 2, doi:10.1007/b97388; J&R 2007 eq. 2.20,
  doi:10.1007/978-0-387-36945-7), total reflection below the critical
  grazing angle of a fast bottom, times Eckart's coherent fraction
  ``C = exp(-(2 k sigma sin psi)^2)`` (Eckart, J. Acoust. Soc. Am. 25(3),
  566-570, 1953, doi:10.1121/1.1907123);
* surface: ``|R| = 1`` (pressure release) times the same Eckart fraction of
  the water surface's RMS height.

Pure functions, no renderer state: the renderer (walls) and the facet layer
(smooth objects) call them, and smoke ``[17]`` proves them against the
closed form of the incoherent ideal-waveguide image sum.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Seabed boundary constants for the image comb's floor bounce: density ratio
# rho_floor / rho_water, sound-speed ratio c_floor / c_water, RMS height.
#: Medium sand, J&R 2007 Table 5.1 (SAX99: Vp ratio 1.154, rho 2.066 g/cm3
#: against 1.024 sea water): ratio 2.0 / 1.15 -> critical grazing 29.6 deg.
SAND_DENSITY_RATIO = 2.0
SAND_SPEED_RATIO = 1.15
#: Sand's RMS height at the cm scale (ripples and bioturbation), J&R 2007
#: ch. 4 roughness spectra; JUDGED (no sand survey of ours measures it).
SAND_RMS_HEIGHT_M = 0.01
#: Painted / trowelled concrete (the pool): density 2400 kg/m3, 3700 m/s
#: (Kinsler et al., Fundamentals of Acoustics, 4th ed., App. A10; the
#: materials table `concrete`) against fresh water 1000 kg/m3 / 1480 m/s
#: -> critical grazing 66.4 deg. A thin paint film (<< lambda) is ignored.
PAINTED_CONCRETE_DENSITY_RATIO = 2.4
PAINTED_CONCRETE_SPEED_RATIO = 3700.0 / 1480.0
#: RMS height relevant at lambda = 3.3 mm: micro-texture of finished concrete
#: is microns (Ra 1-5 um, stylus profilometry; Courard, Garbacz & Gorka,
#: ICPIC 2004) and trowel waviness sub-mm; JUDGED 0.1 mm, bounds 1e-5..5e-4.
PAINTED_CONCRETE_RMS_HEIGHT_M = 1.0e-4

SEABED_BOUNDARY: dict[str, tuple[float, float, float]] = {
    "sand": (SAND_DENSITY_RATIO, SAND_SPEED_RATIO, SAND_RMS_HEIGHT_M),
    "painted_concrete": (PAINTED_CONCRETE_DENSITY_RATIO,
                         PAINTED_CONCRETE_SPEED_RATIO,
                         PAINTED_CONCRETE_RMS_HEIGHT_M),
}


def seabed_boundary(name: str) -> tuple[float, float, float]:
    """(density ratio, sound-speed ratio, RMS height) of a seabed material;
    unknown names fall back to sand (the calibrated stand-in)."""
    return SEABED_BOUNDARY.get(str(name), SEABED_BOUNDARY["sand"])


def floor_boundary(cfg) -> tuple[float, float, float]:
    """The comb's floor for a sonar model config: ``seabed_material``'s
    constants, its RMS height replaced by ``floor_rms_height_m`` when that
    is >= 0 (an environment overlay; -1 = the material's own)."""
    rho, cr, sig = seabed_boundary(getattr(cfg, "seabed_material", "sand"))
    o = float(getattr(cfg, "floor_rms_height_m", -1.0))
    return (rho, cr, o if o >= 0.0 else sig)


def rayleigh_r(psi: np.ndarray, density_ratio: float,
               speed_ratio: float) -> np.ndarray:
    """|R| of a lossless fluid half-space at grazing angle ``psi`` [rad]
    (Brekhovskikh & Lysanov eq. 2.1.x / J&R eq. 2.20)::

        R = (m sin psi - sqrt(n^2 - cos^2 psi)) / (m sin psi + sqrt(...))

    with ``m = rho2/rho1``, ``n = c1/c2``; below the critical angle of a
    fast bottom the root is imaginary and ``|R| = 1``."""
    psi = np.asarray(psi, dtype=np.float64)
    m = float(density_ratio)
    n = 1.0 / float(speed_ratio)
    s = np.sin(psi)
    root = np.sqrt((n * n - np.cos(psi) ** 2).astype(np.complex128))
    r = (m * s - root) / (m * s + root)
    return np.abs(r)


def eckart_c(psi: np.ndarray, rms_m: float, k: float) -> np.ndarray:
    """Coherent (specular) energy fraction of a rough boundary at grazing
    angle ``psi``: ``exp(-(2 k sigma sin psi)^2)`` (Eckart 1953)."""
    g = 2.0 * k * float(rms_m) * np.sin(np.asarray(psi, dtype=np.float64))
    return np.exp(-np.minimum(g * g, 700.0))


@dataclass(frozen=True)
class Images:
    """The transducer's images in a surface/floor waveguide."""

    depth: np.ndarray     # image depth, m (positive down)
    n_floor: np.ndarray   # floor bounces of the unfolded line
    n_surface: np.ndarray  # surface bounces
    order: np.ndarray     # |m|
    direct: int           # index of the real transducer (d = z_t)

    @property
    def n_bounce(self) -> np.ndarray:
        return self.n_floor + self.n_surface


def image_set(z_t: float, depth: float, max_order: int) -> Images:
    """Images ``2 m D +- z_t`` for ``|m| <= max_order`` with exact bounce
    counts (crossings of ``z = k D`` strictly between ``z_t`` and the
    image; odd ``k`` = floor, even ``k`` = surface)."""
    D = float(depth)
    zt = float(z_t)
    ms = np.arange(-int(max_order), int(max_order) + 1)
    d = np.concatenate([2.0 * ms * D + zt, 2.0 * ms * D - zt])
    order = np.concatenate([np.abs(ms), np.abs(ms)])
    lo = np.minimum(d, zt)
    hi = np.maximum(d, zt)
    nf = np.zeros(d.size, dtype=np.int64)
    ns = np.zeros(d.size, dtype=np.int64)
    kmax = 2 * int(max_order) + 2
    for kk in range(-kmax, kmax + 1):
        c = (lo < kk * D - 1e-12) & (kk * D + 1e-12 < hi)
        if kk % 2:
            nf += c
        else:
            ns += c
    direct = int(np.flatnonzero(np.isclose(d, zt) & (nf + ns == 0))[0])
    return Images(depth=d, n_floor=nf, n_surface=ns, order=order,
                  direct=direct)


def ideal_image_sum(r: float, depth: float, z_t: float,
                    max_order: int) -> float:
    """``sum_images 1 / (r^2 + (d - z_t)^2)`` over ``|m| <= max_order`` --
    the incoherent ideal waveguide (|R| = 1, omnidirectional), the quantity
    :func:`ideal_image_sum_closed` gives in closed form."""
    im = image_set(z_t, depth, max_order)
    return float(np.sum(1.0 / (r * r + (im.depth - z_t) ** 2)))


def ideal_image_sum_closed(r: float, depth: float, z_t: float) -> float:
    """Closed form of the infinite incoherent image sum (r = horizontal
    distance of the unfolded path; for a wall at f, r = 2 f)::

        pi/(2 D r) * [ coth(pi r / 2D)
                       + sinh(pi r / D) / (cosh(pi r / D) - cos(2 pi z_t / D)) ]

    from sum_m 1/((m - b)^2 + a^2) = (pi/a) sinh(2 pi a)/(cosh(2 pi a) -
    cos(2 pi b)) with a = r/2D and b = 0 (images 2 m D) or b = z_t/D
    (images 2 m D - 2 z_t, relative to the transducer)."""
    D = float(depth)
    a = np.pi * r / (2.0 * D)
    fam_a = 1.0 / np.tanh(a)
    fam_b = np.sinh(2.0 * a) / (np.cosh(2.0 * a) - np.cos(2.0 * np.pi * z_t / D))
    return float(np.pi / (2.0 * D * r) * (fam_a + fam_b))


def leg_factors(images: Images, horiz: np.ndarray | float,
                depth_pt: np.ndarray, *, floor: tuple[float, float, float],
                surface_rms_m: float, k: float
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """For every image (rows) and scatterer point (columns, depth
    ``depth_pt`` at horizontal distance ``horiz``): the leg length ``l``,
    the unfolded elevation ``e`` (+ = going down at the point) and the
    boundary factor ``prod |R|^2 C`` of that leg's bounces (intensity)."""
    d = images.depth[:, None]
    zp = np.asarray(depth_pt, dtype=np.float64)[None, :]
    h = np.asarray(horiz, dtype=np.float64)
    dz = zp - d
    l = np.hypot(h, dz)
    e = np.arctan2(dz, h)
    psi = np.abs(e)
    rho, cr, sig_f = floor
    f_b = rayleigh_r(psi, rho, cr) ** 2 * eckart_c(psi, sig_f, k)
    s_b = eckart_c(psi, surface_rms_m, k)
    fac = (f_b ** images.n_floor[:, None]) * (s_b ** images.n_surface[:, None])
    return l, e, fac


def departure(images: Images, e: np.ndarray) -> np.ndarray:
    """Depression angle at the transducer of each leg: the unfolded line's
    elevation, its vertical component flipped once per bounce."""
    sign = np.where(images.n_bounce % 2 == 0, 1.0, -1.0)[:, None]
    return e * sign
