"""Propagation in the water: absorption, sound speed, spreading (note 07).

* Absorption: Francois & Garrison (1982a,b; doi:10.1121/1.388170,
  10.1121/1.388673) as printed in Jackson & Richardson 2007, App. B.3,
  eqs. B.6-B.12 (doi:10.1007/978-0-387-36945-7) -- read there, implemented
  verbatim. At 450 kHz: 0.163-0.177 dB/m in the corpus's seawater at the
  per-site temperatures (note 07; every site is seawater, owner 2026-11-03).
* Sound speed: the simplified relation used inside Francois-Garrison,
  ``c = 1412 + 3.21 T + 1.19 S + 0.0167 z`` (J&R eq. B.10). Within a few m/s
  of the full equations (Mackenzie 1981 for seawater, Bilaniuk & Wong 1993
  for fresh water -- records only, their coefficients not yet read), which
  is ~0.3 % in range; the device's own assumed speed (``sos_dmps``) is what
  maps time to bins, so a true-vs-assumed difference shows as the device's.
* Spreading: spherical per leg, amplitude 1/r. Cylindrical-like range laws
  in shallow water come from the image paths, never from a spreading law.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

LN10_OVER_20 = np.log(10.0) / 20.0


def sound_speed_fg(T: float, S: float, z: float = 0.0) -> float:
    """Sound speed [m/s] of J&R eq. B.10 (T [degC], S [ppt], z [m] depth)."""
    return 1412.0 + 3.21 * T + 1.19 * S + 0.0167 * z


def absorption_fg(f_hz: float, T: float, S: float, z: float = 1.0,
                  pH: float = 8.0) -> float:
    """Francois-Garrison absorption [dB/m], J&R eqs. B.6-B.12.

    ``f_hz`` frequency [Hz], ``T`` temperature [degC], ``S`` salinity [ppt],
    ``z`` depth [m], ``pH``. S = 0 leaves the pure-water viscous term only,
    the right limit for fresh water (J&R Fig. B.2)."""
    f = f_hz / 1000.0                                   # kHz
    c = sound_speed_fg(T, S, z)
    A1 = 8.86 / c * 10.0 ** (0.78 * pH - 5.0)
    P1 = 1.0
    f1 = 2.8 * np.sqrt(S / 35.0) * 10.0 ** (4.0 - 1245.0 / (273.0 + T))
    A2 = 21.44 * S / c * (1.0 + 0.025 * T)
    P2 = 1.0 - 1.37e-4 * z + 6.2e-9 * z * z
    f2 = 8.17 * 10.0 ** (8.0 - 1990.0 / (273.0 + T)) / (1.0 + 0.0018 * (S - 35.0))
    if T <= 20.0:
        A3 = 4.937e-4 - 2.59e-5 * T + 9.11e-7 * T ** 2 - 1.50e-8 * T ** 3
    else:
        A3 = 3.964e-4 - 1.146e-5 * T + 1.45e-7 * T ** 2 - 6.5e-10 * T ** 3
    P3 = 1.0 - 3.83e-5 * z + 4.9e-10 * z * z
    boric = A1 * P1 * f1 * f * f / (f * f + f1 * f1) if S > 0 else 0.0
    mgso4 = A2 * P2 * f2 * f * f / (f * f + f2 * f2)
    water = A3 * P3 * f * f
    return 1e-3 * (boric + mgso4 + water)


@dataclass(frozen=True)
class Water:
    """The site's water: what absorption and sound speed are computed from."""

    temperature_c: float = 20.0
    salinity_ppt: float = 35.0
    ph: float = 8.0
    mean_depth_m: float = 1.0
    frequency_hz: float = 450000.0

    @property
    def alpha_db_per_m(self) -> float:
        return absorption_fg(self.frequency_hz, self.temperature_c,
                             self.salinity_ppt, self.mean_depth_m, self.ph)

    @property
    def sound_speed(self) -> float:
        return sound_speed_fg(self.temperature_c, self.salinity_ppt,
                              self.mean_depth_m)

    @property
    def wavenumber(self) -> float:
        return 2.0 * np.pi * self.frequency_hz / self.sound_speed


def leg_amplitude(r: np.ndarray, alpha_db_per_m: float) -> np.ndarray:
    """One-way pressure amplitude factor of a straight leg of length ``r``
    [m] from a unit source at 1 m: ``10^(-alpha r / 20) / r``."""
    r = np.asarray(r, dtype=np.float64)
    return np.exp(-alpha_db_per_m * LN10_OVER_20 * r) / np.maximum(r, 1e-6)


def transmission_loss_db(r: np.ndarray, alpha_db_per_m: float) -> np.ndarray:
    """One-way TL [dB re 1 m] = 20 log10 r + alpha r (gate A1)."""
    r = np.asarray(r, dtype=np.float64)
    return 20.0 * np.log10(np.maximum(r, 1e-6)) + alpha_db_per_m * r
