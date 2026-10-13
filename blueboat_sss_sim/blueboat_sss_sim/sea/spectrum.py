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
