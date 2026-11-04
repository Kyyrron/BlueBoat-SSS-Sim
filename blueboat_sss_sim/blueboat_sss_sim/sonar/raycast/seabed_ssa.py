"""The physical seabed law: small-slope approximation + sediment volume
(Jackson & Richardson 2007, doi:10.1007/978-0-387-36945-7; note 03).

Equations, all read in J&R (book page = PDF page - 8):

* flat fluid-fluid reflection (8.46)-(8.52): ``a_p = nu_p / (1 + i delta_p)``
  (8.25), ``sin(theta_p) = sqrt(1 - a_p^2 cos^2 theta)`` (8.46),
  ``V = (z - 1)/(z + 1)``, ``z = a_rho a_p sin(theta) / sin(theta_p)``;
* first-order perturbation factor (13.21)-(13.25):
  ``A = 1/2 [1 + V(th_i)][1 + V(th_s)] G``,
  ``G = (1 - 1/a_rho)[cos th_i cos th_s cos phi_s - B] - 1 + 1/(a_p^2 a_rho)``,
  ``B = sin th_pi sin th_ps / (a_p^2 a_rho)``;
* lowest-order small-slope cross section (13.11), (13.15)-(13.17), (D.11)-(D.13):
  ``sigma = k^4 |A|^2 / (2 pi dK^2 dkz^2) I_K``,
  ``I_K = int_0^inf J0(u) exp(-q u^(2 alpha)) u du``,
  ``q = 1/2 C_h^2 dkz^2 dK^(-2 alpha)``, ``alpha = gamma_2/2 - 1``,
  ``C_h^2 = 2 pi w_2 Gamma(2 - alpha) 2^(-2 alpha) / (alpha (1 - alpha) Gamma(1 + alpha))``;
* sediment volume, equivalent interface cross section (14.6)-(14.9):
  ``sigma_v = |1 + V_i|^2 |1 + V_s|^2 s_v / (2 k |a_rho|^2 Im[P_i + P_s])``,
  ``P = sqrt(a_p^-2 - cos^2 theta)``, ``s_v = sigma_2 alpha_p``,
  ``alpha_p = 40 pi f delta_p / (nu_p c ln 10)`` (8.26).

Geometry (13.1)-(13.7): incident ``k_i = k (cos th_i, 0, -sin th_i)``, scattered
``k_s = k (cos th_s cos phi, cos th_s sin phi, sin th_s)``; backscatter is
``th_s = th_i``, ``phi = pi``. Angles are grazing angles in the local frame
of the scattering facet.

The mesh-Nyquist cutoff of the composite picture is immaterial here: at 450
kHz the I_K integral is dominated by lags < 0.2 mm (q-scale of SAX99 sand),
three orders below the 0.1 m mesh, so the full power-law structure function
is used (test S-cut checks it). ROS-free, Qt-free, numpy only (math.gamma).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# ------------------------------------------------------------ the I_K integral
def _j0(x: np.ndarray) -> np.ndarray:
    """Bessel J0 for x >= 0 (numpy only): power series below 8, the
    Hankel asymptotic with 3 terms above (|error| < 2e-9)."""
    x = np.asarray(x, dtype=np.float64)
    out = np.empty_like(x)
    s = x < 8.0
    if s.any():
        xs = x[s]
        t = -(0.25 * xs * xs)
        term = np.ones_like(xs)
        acc = np.ones_like(xs)
        for m in range(1, 40):
            term = term * t / (m * m)
            acc = acc + term
        out[s] = acc
    b = ~s
    if b.any():
        xb = x[b]
        z = 8.0 / xb
        y = z * z
        p0 = 1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4 + y * (
            -0.2073370639e-5 + y * 0.2093887211e-6)))
        q0 = -0.1562499995e-1 + y * (0.1430488765e-3 + y * (
            -0.6911147651e-5 + y * (0.7621095161e-6 - y * 0.934935152e-7)))
        xx = xb - 0.785398164
        out[b] = np.sqrt(0.636619772 / xb) * (np.cos(xx) * p0 - z * np.sin(xx) * q0)
    return out


def ik_series(q: float, beta: float, n_max: int = 60) -> float:
    """Small-q series of I_K from the Mellin transform of J0
    (``int_0^inf J0(u) u^mu du = 2^mu Gamma((1+mu)/2) / Gamma((1-mu)/2)``):
    ``sum_n (-q)^n / n! 2^(1+n beta) Gamma(1 + n beta/2) / Gamma(-n beta/2)``,
    truncated at its smallest term (convergent for beta < 1, asymptotic
    above). beta = 2 alpha = gamma_2 - 2."""
    acc, best = 0.0, math.inf
    for n in range(1, n_max):
        x = 0.5 * n * beta
        if abs(x - round(x)) < 1e-12:          # 1/Gamma(-integer) = 0
            continue
        # 1/Gamma(-x) via reflection: -x sin(pi x) Gamma(x) / pi
        inv_g = -x * math.sin(math.pi * x) * math.gamma(x) / math.pi
        try:
            term = ((-q) ** n / math.factorial(n) * 2.0 ** (1 + n * beta)
                    * math.gamma(1.0 + x) * inv_g)
        except OverflowError:
            break
        if abs(term) > best and n > 2:
            break
        best = min(best, abs(term))
        acc += term
        if abs(term) < 1e-16 * max(abs(acc), 1e-300):
            break
    return acc


def ik_quadrature(q: float, beta: float, nodes: int = 32,
                  tail: float = 40.0) -> float:
    """I_K by Gauss-Legendre on [0, U], ``exp(-q U^beta) = exp(-tail)``,
    over intervals no longer than the J0 half-period (pi) nor U / 64, so
    both the oscillation and the decay scale are resolved."""
    U = (tail / q) ** (1.0 / beta)
    n_int = max(int(np.ceil(U / np.pi)), 64)
    edges = np.linspace(0.0, U, n_int + 1)
    x, w = np.polynomial.legendre.leggauss(nodes)
    a, b = edges[:-1], edges[1:]
    mid, half = 0.5 * (a + b), 0.5 * (b - a)
    u = mid[:, None] + half[:, None] * x[None, :]
    f = _j0(u) * np.exp(-q * u ** beta) * u
    return float(np.sum((f * w[None, :]).sum(axis=1) * half))


def ik_large(q: float, beta: float, m_max: int = 60) -> float:
    """Large-q series: J0's Taylor series integrated term by term,
    ``sum_m (-1/4)^m / (m!)^2 Gamma((2m+2)/beta) / (beta q^((2m+2)/beta))``
    (the Kirchhoff / near-specular regime)."""
    acc = 0.0
    lq = math.log(q)
    for m in range(m_max):
        e = (2 * m + 2) / beta
        lt = (m * math.log(0.25) - 2 * math.lgamma(m + 1) + math.lgamma(e)
              - math.log(beta) - e * lq)
        term = (-1) ** m * math.exp(lt)
        acc += term
        if abs(term) < 1e-17 * abs(acc):
            return acc
    return acc


def ik(q: float, beta: float) -> float:
    """I_K(q, beta): the small-q series (beta <= 1.7, where it converges
    to 1e-10), the large-q series above q = 30 when it has converged, the
    quadrature otherwise (each closed-form checked, test S2)."""
    if q < 0.02 and beta <= 1.7:
        return ik_series(q, beta)
    if q > 30.0:
        v = ik_large(q, beta)
        if v > 0.0:
            return v
    return ik_quadrature(q, beta)


class IkTable:
    """log-log interpolation of I_K(q) for one spectral exponent."""

    def __init__(self, beta: float, q_lo: float = 1e-7, q_hi: float = 1e7,
                 n: int = 561) -> None:
        self.beta = beta
        self.lq = np.linspace(np.log(q_lo), np.log(q_hi), n)
        vals = np.array([ik(float(np.exp(l)), beta) for l in self.lq])
        self.lv = np.log(np.maximum(vals, 1e-300))
        self._lo_slope = (self.lv[1] - self.lv[0]) / (self.lq[1] - self.lq[0])

    def __call__(self, q: np.ndarray) -> np.ndarray:
        lq = np.log(np.maximum(np.asarray(q, dtype=np.float64), 1e-300))
        v = np.interp(lq, self.lq, self.lv)
        lo = lq < self.lq[0]                      # SPM regime: I_K ~ q
        v = np.where(lo, self.lv[0] + self._lo_slope * (lq - self.lq[0]), v)
        hi = lq > self.lq[-1]                     # Kirchhoff: ~ q^(-2/beta)
        v = np.where(hi, self.lv[-1] - (2.0 / self.beta) * (lq - self.lq[-1]), v)
        return np.exp(v)


_IK_TABLES: dict = {}


def ik_table(beta: float) -> "IkTable":
    """A memoised :class:`IkTable` (a fit rebuilds the law per evaluation;
    the table depends on the spectral exponent only)."""
    key = round(float(beta), 9)
    t = _IK_TABLES.get(key)
    if t is None:
        t = _IK_TABLES[key] = IkTable(key)
    return t


# ------------------------------------------------------------------- the law
@dataclass(frozen=True)
class SedimentParams:
    """J&R geoacoustic + roughness parameters of one sediment."""

    name: str
    a_rho: float          # density ratio
    nu_p: float           # sound speed ratio
    delta_p: float        # loss parameter
    gamma2: float         # spectral exponent (2 < gamma2 < 4)
    w2: float             # spectral strength [m^(4 - gamma2)]
    sigma2: float         # volume parameter sigma_v / alpha_p [dB^-1]
    source: str = ""

    @property
    def alpha(self) -> float:
        return 0.5 * self.gamma2 - 1.0

    @property
    def ch2(self) -> float:
        a = self.alpha
        return (2.0 * math.pi * self.w2 * math.gamma(2.0 - a) * 2.0 ** (-2.0 * a)
                / (a * (1.0 - a) * math.gamma(1.0 + a)))

    @property
    def a_p(self) -> complex:
        return self.nu_p / (1.0 + 1j * self.delta_p)


#: Literature defaults (JR07 Table 13.1 at 35-40 kHz; extrapolated to 450 kHz,
#: the caveat of JR07 Sect. 6.5). sigma2: mid-range of JR07 eq. 14.9's
#: 1e-4..4e-3 -- a fitted parameter inside those bounds (raycast_design §8).
SEDIMENTS = {
    "sand": SedimentParams("sand", 2.00, 1.16, 0.010, 3.04, 4.33e-5, 2e-3,
                           "JR07 Table 13.1, SAX99 (phi 1.28)"),
    "mud": SedimentParams("mud", 1.499, 1.003, 0.0153, 3.29, 4.64e-4, 2e-3,
                          "JR07 Table 13.1, Key West (phi 7.0)"),
}


#: Bounds of the fitted overrides (raycast_design §8). w2 and gamma2: the
#: range of J&R Table 13.1 (p. 349; read 2026-11-03) -- sand sites (phi
#: 0.51-2.63): w2 3.56e-7 (Panama City 84) .. 1.475e-4 (Panama City 93);
#: mud (Key West, the only fine site besides Quinault): the whole table,
#: 3.56e-7 .. 4.1e-3 (Quinault). gamma2: the table's 3.92 at most, and
#: > 2 for the small-slope law's alpha = gamma2/2 - 1 in (0, 1) (and a finite
#: coherent-sum RMS; Panama City 84's 1.89 is outside the law's range).
#: sigma2: J&R eq. 14.9, "typical values 0.0001 to 0.004".
OVERRIDE_BOUNDS = {
    "sand": {"w2": (3.56e-7, 1.475e-4), "gamma2": (2.0 + 1e-6, 3.92),
             "sigma2": (1e-4, 4e-3)},
    "mud": {"w2": (3.56e-7, 4.1e-3), "gamma2": (2.0 + 1e-6, 3.92),
            "sigma2": (1e-4, 4e-3)},
}


def sediments_for(overrides=None) -> dict:
    """``SEDIMENTS`` with the bounded overrides applied (unknown sediment
    or key, or a value outside its bound, raises)."""
    import dataclasses
    out = dict(SEDIMENTS)
    for name, d in dict(overrides or {}).items():
        if name not in OVERRIDE_BOUNDS:
            raise KeyError(f"raycast.seabed.sediments: unknown sediment {name!r}")
        bad = set(d) - set(OVERRIDE_BOUNDS[name])
        if bad:
            raise KeyError(f"raycast.seabed.sediments.{name}: unknown keys {sorted(bad)}")
        for key, val in d.items():
            lo, hi = OVERRIDE_BOUNDS[name][key]
            if not lo <= float(val) <= hi:
                raise ValueError(f"raycast.seabed.sediments.{name}.{key} = {val} "
                                 f"outside [{lo}, {hi}]")
        out[name] = dataclasses.replace(
            SEDIMENTS[name], **{k: float(v) for k, v in d.items()},
            source=SEDIMENTS[name].source + " (overridden: "
            + ", ".join(sorted(d)) + ")")
    return out


def sin_theta_p(a_p: complex, cos_th: np.ndarray) -> np.ndarray:
    """``sin(theta_p) = sqrt(1 - a_p^2 cos^2 theta)`` on the branch whose
    transmitted wave decays into the sediment (Im(k_p sin theta_p) >= 0)."""
    s = np.sqrt(1.0 - (a_p ** 2) * np.asarray(cos_th, dtype=complex) ** 2)
    flip = np.imag(s / a_p) < 0.0
    return np.where(flip, -s, s)


def reflection(p: SedimentParams, sin_th: np.ndarray) -> np.ndarray:
    """Flat-interface pressure reflection coefficient V(theta) (8.49)."""
    sin_th = np.asarray(sin_th, dtype=np.float64)
    cos_th = np.sqrt(np.maximum(1.0 - sin_th ** 2, 0.0))
    stp = sin_theta_p(p.a_p, cos_th)
    # at a lossless fast medium's critical angle sin(theta_p) = 0: z -> inf,
    # V -> 1 (total reflection begins); guard the 0/0 there
    crit = np.abs(stp) < 1e-15
    with np.errstate(divide="ignore", invalid="ignore"):
        z = p.a_rho * p.a_p * sin_th / np.where(crit, 1.0, stp)
        V = (z - 1.0) / (z + 1.0)
    return np.where(crit, 1.0 + 0j, V)


class SSALaw:
    """Bistatic sigma(theta_i, theta_s, phi) of one sediment (roughness by the
    small-slope approximation + volume)."""

    def __init__(self, p: SedimentParams, k: float, frequency_hz: float,
                 c_water: float, table: IkTable | None = None) -> None:
        self.p = p
        self.k = float(k)
        self.table = table or IkTable(2.0 * p.alpha)
        self.alpha_p = (40.0 * math.pi * frequency_hz * p.delta_p
                        / (p.nu_p * c_water * math.log(10.0)))
        self.s_v = p.sigma2 * self.alpha_p

    def roughness(self, sin_i, sin_s, cos_phi) -> np.ndarray:
        p, k = self.p, self.k
        si = np.clip(np.asarray(sin_i, dtype=np.float64), 1e-6, 1.0)
        ss = np.clip(np.asarray(sin_s, dtype=np.float64), 1e-6, 1.0)
        ci = np.sqrt(1.0 - si * si)
        cs = np.sqrt(1.0 - ss * ss)
        cphi = np.asarray(cos_phi, dtype=np.float64)
        Vi, Vs = reflection(p, si), reflection(p, ss)
        ap2 = p.a_p ** 2
        B = sin_theta_p(p.a_p, ci) * sin_theta_p(p.a_p, cs) / (ap2 * p.a_rho)
        G = (1.0 - 1.0 / p.a_rho) * (ci * cs * cphi - B) - 1.0 + 1.0 / (ap2 * p.a_rho)
        A = 0.5 * (1.0 + Vi) * (1.0 + Vs) * G
        dK2 = k * k * (ci * ci + cs * cs - 2.0 * ci * cs * cphi)
        dK2 = np.maximum(dK2, (1e-9 * k) ** 2)
        dkz = k * (si + ss)
        q = 0.5 * p.ch2 * dkz ** 2 * dK2 ** (-p.alpha)
        return (k ** 4 * np.abs(A) ** 2 / (2.0 * math.pi * dK2 * dkz ** 2)
                * self.table(q))

    def volume(self, sin_i, sin_s) -> np.ndarray:
        p, k = self.p, self.k
        si = np.clip(np.asarray(sin_i, dtype=np.float64), 1e-6, 1.0)
        ss = np.clip(np.asarray(sin_s, dtype=np.float64), 1e-6, 1.0)
        def P(s):
            c2 = 1.0 - s * s
            v = np.sqrt(p.a_p ** -2 - c2 + 0j)
            return np.where(np.imag(v) < 0, -v, v)
        Vi, Vs = reflection(p, si), reflection(p, ss)
        den = 2.0 * k * abs(p.a_rho) ** 2 * np.imag(P(si) + P(ss))
        return (np.abs(1.0 + Vi) ** 2 * np.abs(1.0 + Vs) ** 2 * self.s_v
                / np.maximum(den, 1e-300))

    def sigma(self, sin_i, sin_s, cos_phi) -> np.ndarray:
        return self.roughness(sin_i, sin_s, cos_phi) + self.volume(sin_i, sin_s)

    def backscatter(self, sin_g) -> np.ndarray:
        return self.sigma(sin_g, sin_g, -np.ones_like(np.asarray(sin_g, float)))
