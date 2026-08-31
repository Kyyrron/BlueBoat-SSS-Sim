"""Sim-to-real calibration: one reduction, applied to both sides of the pair.

The comparison statistic is the **per-range mean dB relative to the ping's own
first-bottom-return peak**, grouped by ``(range setting, side, altitude band)``.
Three properties make it the right one:

* The device normalises every ping onto ``[min_pwr_db, max_pwr_db]``, so the
  raw array carries no absolute level. Referencing to the FBR peak therefore
  costs nothing and removes the one thing the array cannot tell us.
* It is **gain-independent**. The field corpus was recorded under the device's
  auto-gain, which walks indices 4..7 within every file; a statistic that moved
  with the gain would be measuring the AGC, not the seabed.
* Grouping by altitude band is what separates ``tvg_compensation`` from
  ``lambert_exponent``. Both enter the far-field curve as a pure ``log10(r)``
  slope, so at one altitude they are interchangeable to 0.083 dB p-p (smoke
  ``[2e]``); grazing angle at a given slant range moves the Lambert term and
  not the TVG term, and altitude is what moves grazing angle.

Scope: this module reads a byte format and re-expresses two formulas that
``BlueBoat-SSS`` owns (``scale_to_db`` and the FBR detector, both quoted below
against their source). It **imports nothing from that repository and modifies
nothing in it** (CM-3 / NC #8), exactly as :mod:`.svlog_reader` does for
framing. Recordings are opened read-only and never written to (CM-7).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from ..core.types import Side
from .svlog_reader import ProfileRecord, read_profiles

# --- the downstream detector's constants, so the reduction sees what it sees --
# Mirrors sss_processor_node.py's NOISE_FLOOR_WINDOW / FBR_THRESHOLD_DELTA_DB /
# WITHIN_PING_PERSISTENCE / RINGING_*. Re-expressed rather than imported (CM-3).
NOISE_FLOOR_WINDOW = 20
FBR_THRESHOLD_DELTA_DB = 8.0
WITHIN_PING_PERSISTENCE = 3
RINGING_SEARCH_MAX = 60
RINGING_DROP_DB = 10.0
RINGING_PERSISTENCE = 5

#: Repeatability floor of this reduction, measured in smoke ``[2e]``: two
#: 80-ping renders of the same scene under different speckle seeds differ by
#: this much RMS over 6.0-14.5 m. A sim-vs-real residual below it is not a
#: result, it is the statistic's own noise.
REDUCTION_FLOOR_DB = 0.67


def scale_to_db(pwr: np.ndarray, min_pwr_db: float,
                max_pwr_db: float) -> np.ndarray:
    """Invert the device's per-ping normalisation.

    The Cerulean Omniscan 450 template, as ``ping-python`` defines it and as
    ``sss_processor_node`` and the GCS both apply it::

        db = min_pwr_db + (raw / 65535) * (max_pwr_db - min_pwr_db)
    """
    return min_pwr_db + (np.asarray(pwr, dtype=np.float64) / 65535.0) \
        * (max_pwr_db - min_pwr_db)


def noise_window_start(db: np.ndarray,
                       search_max: int = RINGING_SEARCH_MAX,
                       drop_db: float = RINGING_DROP_DB,
                       persistence: int = RINGING_PERSISTENCE,
                       fallback: int = 30) -> int:
    """First sample past the transmit-pulse ringing.

    The near bins carry the transmit pulse and its ringing tail, well above
    the water-column noise behind it. Estimating the noise floor *inside* that
    tail sets the FBR threshold ~20 dB too high and the detector then either
    misses the bottom entirely or fires on the ringing at bin 0 -- both of
    which showed up on the real corpus before this step was included.

    The rule is relative to the ping's own ringing peak, so it carries no
    tuned absolute level across gain settings or environments.
    """
    n = db.size
    if n < search_max + persistence:
        return fallback
    target = float(db[:search_max].max()) - drop_db
    below = db[:search_max] < target
    for i in range(search_max - persistence + 1):
        if below[i:i + persistence].all():
            return i
    return fallback


def fbr_bin(db: np.ndarray,
            noise_floor_window: int = NOISE_FLOOR_WINDOW,
            threshold_delta_db: float = FBR_THRESHOLD_DELTA_DB,
            persistence: int = WITHIN_PING_PERSISTENCE) -> int | None:
    """Index of the first bottom return, by the downstream detector's rule.

    Noise floor from ``noise_floor_window`` samples taken *after* the ringing
    settles, then the first index starting a run of ``persistence`` samples
    more than ``threshold_delta_db`` above it. ``None`` when no such run
    exists -- a miss, not a zero: the caller drops the ping rather than
    inventing an altitude for it.
    """
    nw = noise_window_start(db)
    nw_end = nw + noise_floor_window
    if db.size < nw_end + persistence:
        return None
    threshold = float(db[nw:nw_end].mean()) + threshold_delta_db
    above = db > threshold
    for i in range(nw_end, db.size - persistence + 1):
        if above[i] and above[i:i + persistence].all():
            return i
    return None


@dataclass(frozen=True)
class RangeCurve:
    """Mean dB against slant range, referenced to the FBR peak.

    ``curve`` is ``nan`` in any bin no contributing ping reached -- distinct
    from a bin that was measured and read low.
    """

    length_mm: int
    num_results: int
    side: Side
    altitude_band_m: float
    n_pings: int
    slant_r_m: np.ndarray
    curve: np.ndarray

    @property
    def bin_size_m(self) -> float:
        return self.length_mm / 1000.0 / max(self.num_results, 1)


def _band(alt: float, width: float) -> float:
    return float(np.round(alt / width) * width)


def reduce_profiles(profiles: Iterable[ProfileRecord],
                    altitude_band_m: float = 1.0,
                    min_pings: int = 30) -> list[RangeCurve]:
    """Group pings by ``(length_mm, side, altitude band)`` and average in dB.

    Pings whose FBR the detector misses are dropped: without an altitude they
    cannot be banded, and banding them wrongly would smear the very lever the
    reduction exists to expose. The drop is reported through ``n_pings``.
    """
    groups: dict[tuple[int, int, Side, float], list[np.ndarray]] = {}
    for p in profiles:
        if p.pwr_results is None or p.num_results <= 0:
            continue
        db = scale_to_db(p.pwr_results, p.min_pwr_db, p.max_pwr_db)
        i = fbr_bin(db)
        if i is None:
            continue
        bin_m = p.length_mm / 1000.0 / p.num_results
        alt = i * bin_m
        # Peak within the FBR onset, not the onset sample itself: the detector
        # fires on the leading edge, and the return crests a bin or two later.
        peak = float(db[max(i - 2, 0):i + 3].max())
        key = (p.length_mm, p.num_results, p.side, _band(alt, altitude_band_m))
        groups.setdefault(key, []).append(db - peak)

    out: list[RangeCurve] = []
    for (length_mm, nres, side, band), rows in sorted(
            groups.items(), key=lambda kv: (kv[0][0], kv[0][1],
                                            kv[0][2].value, kv[0][3])):
        if len(rows) < min_pings:
            continue
        bin_m = length_mm / 1000.0 / nres
        out.append(RangeCurve(
            length_mm=length_mm, num_results=nres, side=side,
            altitude_band_m=band, n_pings=len(rows),
            slant_r_m=(np.arange(nres) + 0.5) * bin_m,
            curve=np.mean(rows, axis=0),
        ))
    return out


def reduce_svlog(path: str | Path, **kw) -> list[RangeCurve]:
    """Reduce a recording. The file is opened read-only and never written."""
    return reduce_profiles(read_profiles(path, with_power=True), **kw)


@dataclass(frozen=True)
class Residual:
    """Sim-vs-real disagreement on one range band, with what it took to get it."""

    band_lo_m: float
    band_hi_m: float
    altitude_band_m: float
    n_real: int
    n_sim: int
    rms_db: float
    p2p_db: float

    @property
    def above_floor(self) -> bool:
        """Whether the disagreement is larger than the reduction's own noise."""
        return self.rms_db > REDUCTION_FLOOR_DB


def compare(real: RangeCurve, sim: RangeCurve,
            band_lo_m: float, band_hi_m: float) -> Residual:
    """Compare two curves over one slant-range band, on the finer bin grid.

    A constant offset is **not** removed: under the corrected encoding the
    curves are both referenced to their own FBR peak, so a constant difference
    is a real disagreement about how far the far field sits below the bottom
    return -- which is exactly what the specular and noise-floor stages fit.
    """
    lo, hi = band_lo_m, band_hi_m
    grid = real.slant_r_m[(real.slant_r_m >= lo) & (real.slant_r_m <= hi)]
    if grid.size == 0:
        raise ValueError(f"band {lo}-{hi} m is outside the real curve's range")
    a = np.interp(grid, real.slant_r_m, real.curve)
    b = np.interp(grid, sim.slant_r_m, sim.curve)
    d = a - b
    return Residual(
        band_lo_m=lo, band_hi_m=hi,
        altitude_band_m=real.altitude_band_m,
        n_real=real.n_pings, n_sim=sim.n_pings,
        rms_db=float(np.sqrt(np.mean(d ** 2))),
        p2p_db=float(d.max() - d.min()),
    )


def normalisation_invariants(profiles: Sequence[ProfileRecord]) -> dict:
    """The three device invariants, as fractions of the pings inspected.

    Measured over the whole field corpus these are 1.0, 1.0 and 1.0 on
    68 948 pings; the simulator's encoder must satisfy the same three, and
    a mapping that emits absolute counts satisfies none of them.
    """
    n = one_peak = zero_min = in_span = 0
    for p in profiles:
        if p.pwr_results is None:
            continue
        n += 1
        c = np.asarray(p.pwr_results)
        one_peak += int((c == 65535).sum() == 1)
        zero_min += int(c.min() == 0)
        in_span += int((p.max_pwr_db - p.min_pwr_db) <= 90.0 + 1e-6)
    if n == 0:
        return {"n": 0}
    return {"n": n,
            "single_full_scale_bin": one_peak / n,
            "zero_minimum": zero_min / n,
            "span_within_90db": in_span / n}
