"""Wind-wave spectra and linear-wave dispersion. Pure NumPy.

References (full list in ``docs/sea_state_model.md``):

* Hasselmann et al. (1973), the JONSWAP spectrum -- fetch-limited wind
  sea; Deutsche Hydrographische Zeitschrift, Ergänzungsheft A8(12).
* Pierson & Moskowitz (1964), fully developed sea (JONSWAP with γ = 1);
  J. Geophys. Res. 69(24), doi:10.1029/JZ069i024p05181.
* DNV-RP-C205 (2017) §3.5.5 for the (Hs, Tp)-parameterised JONSWAP form
  used here, and §3.5.10 for the Tp/√Hs steepness band.
* Fossen (2011) §8.2, doi:10.1002/9781119994138, for the same forms in a
  marine-control setting.
"""

from __future__ import annotations

import numpy as np

#: NumPy 2.0 removed `np.trapz` in favour of `np.trapezoid`. This workspace
#: runs the package under both (the system interpreter carries numpy 1.26,
#: the venv 2.4), so bind whichever the interpreter has rather than pinning
#: the package to one of them. Same function, same result.
_trapezoid = getattr(np, "trapezoid", None) or np.trapz

G = 9.80665


def jonswap(omega: np.ndarray, hs: float, tp: float,
            gamma: float = 3.3) -> np.ndarray:
    """JONSWAP spectral density S(ω) [m² s] for significant wave height
    ``hs`` [m], peak period ``tp`` [s] and peak-enhancement ``gamma``.

    DNV-RP-C205 (3.5.5):  S = A_γ · (5/16) Hs² ωp⁴ ω⁻⁵ exp(−1.25 (ωp/ω)⁴) γ^r,
    r = exp(−(ω − ωp)² / (2 σ² ωp²)), σ = 0.07 below ωp and 0.09 above,
    A_γ = 1 − 0.287 ln γ. ``gamma = 1`` is Pierson-Moskowitz exactly.
    """
    omega = np.asarray(omega, dtype=np.float64)
    if hs <= 0.0 or tp <= 0.0:
        return np.zeros_like(omega)
    wp = 2.0 * np.pi / tp
    sigma = np.where(omega <= wp, 0.07, 0.09)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = np.exp(-((omega - wp) ** 2) / (2.0 * sigma ** 2 * wp ** 2))
        pm = (5.0 / 16.0) * hs ** 2 * wp ** 4 * omega ** -5.0 * np.exp(
            -1.25 * (wp / omega) ** 4)
    a_gamma = 1.0 - 0.287 * np.log(gamma)
    s = a_gamma * pm * gamma ** r
    return np.where(omega > 0.0, s, 0.0)


def pierson_moskowitz(omega: np.ndarray, hs: float, tp: float) -> np.ndarray:
    """Pierson-Moskowitz (fully developed) spectrum -- JONSWAP with γ = 1."""
    return jonswap(omega, hs, tp, gamma=1.0)


def spectral_moment(omega: np.ndarray, s: np.ndarray, n: int = 0) -> float:
    """m_n = ∫ ωⁿ S(ω) dω by the trapezoid rule."""
    return float(_trapezoid(np.asarray(omega) ** n * np.asarray(s), omega))


def dispersion(omega: np.ndarray | float, depth_m: float) -> np.ndarray:
    """Wavenumber k [rad/m] from ω² = g k tanh(k h) (linear Airy theory,
    finite depth), Newton iterations from the deep-water guess k = ω²/g."""
    w = np.atleast_1d(np.asarray(omega, dtype=np.float64))
    h = max(float(depth_m), 0.05)
    k = w ** 2 / G
    for _ in range(12):
        th = np.tanh(k * h)
        f = G * k * th - w ** 2
        df = G * th + G * k * h * (1.0 - th ** 2)
        k = np.maximum(k - f / np.maximum(df, 1e-12), 1e-9)
    return k


#: Fetch-limited young-sea steepness (2026-11-10): Tp = 3.6 sqrt(Hs) (Hs in m,
#: Tp in s), the lower bound of the DNV-RP-C205 §3.5.10 band. The
#: fetch-limited growth laws give Tp/sqrt(Hs) = 3.0-4.0 for a harbour (U10
#: 4-10 m/s over 0.3-2 km): JONSWAP, Hasselmann et al. 1973 (eps = 1.6e-7 X,
#: f_m U/g = 3.5 X^-0.33) and Kahma & Calkoen 1992 eqs (2)-(3), (6)-(7)
#: (stable eps = 9.25e-7 X^0.766, w_p U/g = 11.99 X^-0.242; unstable
#: 5.38e-7 X^0.940, 14.19 X^-0.283). 3.6 is the value that reproduces the
#: real BlueBoat's attitude spectrum at Shiraishi-jima (centroid ~1.1 Hz,
#: config/corpus_attitude_targets.yaml); every wave preset follows it.
TP_PER_SQRT_HS = 3.6


def tp_fetch_limited(hs: float, factor: float = TP_PER_SQRT_HS) -> float:
    """Peak period [s] of a fetch-limited young sea of height ``hs`` [m]:
    ``factor * sqrt(hs)`` (:data:`TP_PER_SQRT_HS`)."""
    return float(factor * np.sqrt(max(hs, 0.0)))


def tp_from_hs_dnv(hs: float, factor: float = 4.3) -> float:
    """Peak period from Hs by the DNV-RP-C205 §3.5.10 steepness relation
    3.6 √Hs ≤ Tp ≤ 5 √Hs (Hs in m, Tp in s); ``factor`` 4.3 is the middle
    of the band, used for the harbour presets."""
    return float(factor * np.sqrt(max(hs, 0.0)))


def equal_energy_frequencies(hs: float, tp: float, gamma: float,
                             n_freq: int) -> tuple[np.ndarray, float]:
    """``n_freq`` component frequencies carrying equal spectral energy, and
    the total variance m0 of the spectrum.

    Each component sits at the energy-weighted centre of its equal-m0
    band, so the discrete sum reproduces m0 = Hs²/16 exactly and the
    peak is resolved finely while the tails get no wasted components.
    """
    wp = 2.0 * np.pi / tp
    w = np.linspace(0.4 * wp, 4.0 * wp, 4000)
    s = jonswap(w, hs, tp, gamma)
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (s[1:] + s[:-1]) * np.diff(w))])
    m0 = float(cum[-1])
    if m0 <= 0.0:
        return np.full(n_freq, wp), 0.0
    edges = np.interp(np.linspace(0.0, m0, n_freq + 1), cum, w)
    centres = np.empty(n_freq)
    for i in range(n_freq):
        sel = (w >= edges[i]) & (w <= edges[i + 1])
        if sel.sum() < 2:
            centres[i] = 0.5 * (edges[i] + edges[i + 1])
        else:
            centres[i] = float(_trapezoid(w[sel] * s[sel], w[sel])
                               / max(_trapezoid(s[sel], w[sel]), 1e-30))
    return centres, m0


def hybrid_frequencies(hs: float, tp: float, gamma: float, depth_m: float,
                       f_max_hz: float = 3.0, n_bulk: int = 12,
                       split: float = 1.5, tail_ratio: float = 1.25,
                       max_total: int = 24) -> tuple[np.ndarray, np.ndarray]:
    """Component frequencies and their variances, covering the SLOPE
    spectrum up to ``f_max_hz`` (2026-11-10).

    :func:`equal_energy_frequencies` resolves the elevation spectrum and
    lumps everything above its last quantile into one component: for a
    2.8 s sea the whole tail sat at 0.89 Hz, carrying ~40-50 % of the
    slope forcing that rolls a 1.2 m catamaran, and nothing existed above
    it (OrcaFlex and ITTC 7.5-02-07-01.1 warn that wide tail bins fail to
    excite a response). Here ``n_bulk`` equal-energy bands span
    0.4-``split`` x the peak, then log-spaced bands (ratio ``tail_ratio``,
    widened if needed to stay within ``max_total``) run to ``f_max_hz``.
    Each band's single component sits at the frequency whose wavenumber
    satisfies ``k^2 = K/E`` (K = the band's slope variance, E its
    variance), so every band carries its exact elevation AND slope
    variance; the variances are rescaled so their sum is the spectrum's
    m0 (the sliver beyond ``f_max_hz`` is redistributed). Returns
    ``(omega [rad/s], variance [m^2])`` per component.
    """
    wp = 2.0 * np.pi / tp
    w_split = split * wp
    w_max = max(2.0 * np.pi * float(f_max_hz), 1.05 * w_split)
    w_hi = max(w_max, 4.0 * wp)
    w = np.linspace(0.4 * wp, w_hi, 20000)
    s = jonswap(w, hs, tp, gamma)
    m0_total = float(_trapezoid(s, w))
    if m0_total <= 0.0:
        return np.full(1, wp), np.zeros(1)
    k = dispersion(w, depth_m)
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (s[1:] + s[:-1]) * np.diff(w))])
    e_split = float(np.interp(w_split, w, cum))
    bulk_edges = np.interp(np.linspace(0.0, e_split, n_bulk + 1), cum, w)
    n_tail = int(np.ceil(np.log(w_max / w_split) / np.log(tail_ratio)))
    if n_bulk + n_tail > max_total:
        n_tail = max(max_total - n_bulk, 1)
    tail_edges = np.geomspace(w_split, w_max, n_tail + 1)
    edges = np.concatenate([bulk_edges[:-1], tail_edges])
    omega, var = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        sel = (w >= a) & (w <= b)
        if sel.sum() < 2:
            continue
        e = float(_trapezoid(s[sel], w[sel]))
        if e <= 0.0:
            continue
        kk = float(_trapezoid(k[sel] ** 2 * s[sel], w[sel])) / e
        kc = np.sqrt(kk)
        h = max(float(depth_m), 0.05)
        omega.append(float(np.sqrt(G * kc * np.tanh(kc * h))))
        var.append(e)
    var = np.asarray(var)
    var *= m0_total / var.sum()
    return np.asarray(omega), var
