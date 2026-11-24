"""Fetch-limited wind sea (2026-11-15, session 03; the owner's ruling on condition D).

The site level ``harbour_chop`` is a **physical** estimate, not an inverse fit to the boat's motion: the wind sea a
measured wind raises over the harbour basin's fetch, by a cited growth law. No field attitude enters it; the July
attitude band (``config/corpus_attitude_targets.yaml``) is report-only everywhere since this date.

Growth law -- K. K. Kahma and C. J. Calkoen, "Reconciling discrepancies in the observed growth of wind-generated
waves", J. Phys. Oceanogr. 22(12), 1389-1405, 1992, doi:10.1175/1520-0485(1992)022<1389:RDITOG>2.0.CO;2 [full,
read first-hand 2026-11-15, p. 1400-1401]: with X = g F / U10^2, eps = g^2 E / U10^4 (E the total variance) and
w_p = omega_p U10 / g,
    stable    eps = 9.25e-7 X^0.766,  w_p = 11.99 X^-0.242    (eqs 2, 3)
    unstable  eps = 5.38e-7 X^0.940,  w_p = 14.19 X^-0.283    (eqs 6, 7)
and Hs = Hm0 = 4 sqrt(E), Tp = 2 pi / omega_p. Their data reach down to X of order 10^2 (Lake Ontario's short
fetches); the harbour sits at X ~ 94.

The inputs of ``harbour_chop`` (:data:`HARBOUR_CHOP`) and how they were measured:
``~/ros2_ws/sim_runs/sim_acceptance/03_sea_state/harbour_chop/`` (``derive_harbour_chop.py``, ``harbour_chop.json``,
the JMA pages).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

G = 9.81

#: Kahma & Calkoen 1992: (a, b, c, d) of eps = a X^b, w_p = c X^d.
KC92 = {"stable": (9.25e-7, 0.766, 11.99, -0.242),
        "unstable": (5.38e-7, 0.940, 14.19, -0.283)}


def fetch_limited(u10_mps: float, fetch_m: float, stratification: str = "stable") -> tuple[float, float]:
    """(Hs [m], Tp [s]) of the fetch-limited wind sea, Kahma & Calkoen 1992."""
    a, b, c, d = KC92[stratification]
    x = G * fetch_m / u10_mps ** 2
    energy = a * x ** b * u10_mps ** 4 / G ** 2
    omega_p = c * x ** d * G / u10_mps
    return float(4.0 * np.sqrt(energy)), float(2.0 * np.pi / omega_p)


@dataclass(frozen=True)
class FetchSea:
    """A site's wind sea: its inputs, their ranges and sources."""
    name: str
    u10_mps: float
    u10_range_mps: tuple[float, float]
    fetch_m: float
    fetch_range_m: tuple[float, float]
    from_deg: float
    stratification: str
    wind_source: str
    fetch_source: str

    def sea(self) -> tuple[float, float]:
        return fetch_limited(self.u10_mps, self.fetch_m, self.stratification)

    def bracket_wind(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """(Hs, Tp) at the two ends of the wind range, nominal fetch."""
        return (fetch_limited(self.u10_range_mps[0], self.fetch_m, self.stratification),
                fetch_limited(self.u10_range_mps[1], self.fetch_m, self.stratification))


#: The July 2026 Shiraishi-jima harbour (34.408 N, 133.524 E).
HARBOUR_CHOP = FetchSea(
    name="harbour_chop",
    u10_mps=3.683, u10_range_mps=(3.3, 4.0),
    fetch_m=130.4, fetch_range_m=(89.2, 165.4),
    from_deg=195.0,
    stratification="stable",
    wind_source=("JMA AMeDAS Kasaoka (66481, 10.7 km N of the harbour, coastal, anemometer 6.4 m -- used as U10 "
                 "without a height correction, which would raise it a few per cent): hourly 10-min means at "
                 "2026-07-21 16-17 h, 07-22 11-12 h, 07-23 11-12 h JST, the hours around the survey recordings; "
                 "3.3-4.0 m/s from S-SSW. Air 33-34 deg C over a cooler July sea: stable [sea temperature not "
                 "looked up]"),
    fetch_source=("harbour world Shiraishijima_real_toOptimize/harbour_wl1p50 (walls drawn on the satellite map): "
                  "upwind distance from every surveyed path point to the south breakwater or the west quay, "
                  "pontoons excluded; mean 130 m (p10-p90 over S and SSW: 89-165 m)"),
)
