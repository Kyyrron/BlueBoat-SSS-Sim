"""Quay contrast in a recording: the numbers the wall model is anchored to.

A vertical harbour wall shows in every field log as a persistent bright
peak late in the ping, followed by a dark zone. This reduces a ``.svlog``
(real or simulated) to three contrasts around that peak, measured on the
device's own reported dB (``scale_to_db``) so a simulated log and a field
log are reduced identically:

* ``peak - before``   the face line over the seabed 1.5-4 m in front of it,
* ``beyond - before`` the strip 1.5-4 m past the face (single-bounce band
                      + mirror ghosts of the near seabed),
* ``far - before``    6-12 m past the face (mirror ghosts of the far seabed),

plus the width of the bright band (> before + 3 dB) and the face range.
The quay is found per ping as the strongest smoothed peak beyond 25 % of
the range, kept only where it persists (moves < 2 m) over at least ten
consecutive pings and stands >= 6 dB over the seabed before it.

Measured 2026-09-03 on ``fullHarbourCleanExample.svlog`` (95.6 m range,
the anchor): port peak +22.8, beyond -5.6, far -20.8 dB; starboard peak
+18.9, beyond -17.4, far -24.8 dB; band 0.7-0.9 m at 30-35 m -- the
defaults of ``wall_echo_strength``, ``wall_corner_gain`` and
``wall_diffuse_fraction`` reproduce peak +21.8, beyond -15.5 and far
-19 dB on a flat basin at that geometry (docs/sonar_model.md section 11).

    python3 -m blueboat_sss_sim.analysis.wall_contrast <file.svlog> [...]
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from . import svlog_reader
from .calibration import scale_to_db


@dataclass
class WallContrast:
    side: str
    pings: int                 # pings with a persistent quay line
    peak_minus_before_db: float
    beyond_minus_before_db: float
    far_minus_before_db: float
    band_width_m: float
    face_range_m: float

    @property
    def found(self) -> bool:
        return self.pings > 0


def _runs(stable: np.ndarray, min_len: int) -> list[tuple[int, int]]:
    runs, start = [], None
    for i, s in enumerate(stable):
        if s and start is None:
            start = i
        if not s and start is not None:
            if i - start >= min_len:
                runs.append((start, i + 1))
            start = None
    if start is not None and len(stable) - start >= min_len:
        runs.append((start, len(stable) + 1))
    return runs


def measure_side(db: np.ndarray, bin_m: float, *, smooth_m: float = 0.3,
                 min_run: int = 10, min_peak_db: float = 6.0) -> WallContrast | None:
    """One side's contrasts from its ``[pings, bins]`` dB array."""
    n = db.shape[1]
    k = max(int(smooth_m / bin_m), 1)
    sm = np.apply_along_axis(lambda v: np.convolve(v, np.ones(k) / k, "same"),
                             1, db)
    i0, i1 = int(0.25 * n), int(0.95 * n)
    pk = i0 + np.argmax(sm[:, i0:i1], axis=1)
    stable = np.abs(np.diff(pk)) * bin_m < 2.0
    sel = [j for a, b in _runs(stable, min_run) for j in range(a, min(b, len(pk)))]
    w15, w4, w6, w12 = (int(x / bin_m) for x in (1.5, 4.0, 6.0, 12.0))
    rows = []
    for j in sel:
        p = int(pk[j])
        d = db[j]
        if p - w4 < i0 or p + w12 >= n:
            continue
        before = float(d[p - w4:p - w15].mean())
        peak = float(sm[j, p])
        if peak - before < min_peak_db:
            continue
        beyond = float(d[p + w15:p + w4].mean())
        far = float(d[p + w6:p + w12].mean())
        thr = before + 3.0
        a = b = p
        while a > 0 and sm[j, a - 1] > thr:
            a -= 1
        while b < n - 1 and sm[j, b + 1] > thr:
            b += 1
        rows.append((peak - before, beyond - before, far - before,
                     (b - a) * bin_m, p * bin_m))
    if not rows:
        return None
    med = np.median(np.array(rows), axis=0)
    return WallContrast("", len(rows), *(float(v) for v in med))


def measure(path: str | Path) -> dict[str, WallContrast | None]:
    """Both sides of one ``.svlog``; ``None`` for a side without a quay."""
    recs = svlog_reader.read_profiles(path, with_power=True)
    out: dict[str, WallContrast | None] = {}
    for side in ("PORT", "STARBOARD"):
        rs = [r for r in recs if r.side.name == side and r.pwr_results is not None]
        if not rs:
            out[side.lower()] = None
            continue
        # The dominant acquisition (auto-gain changes range on some logs).
        key = max(set((r.length_mm, r.num_results) for r in rs),
                  key=lambda kk: sum(1 for r in rs if (r.length_mm, r.num_results) == kk))
        rs = [r for r in rs if (r.length_mm, r.num_results) == key]
        bin_m = key[0] / 1000.0 / key[1]
        db = np.array([scale_to_db(r.pwr_results, r.min_pwr_db, r.max_pwr_db)
                       for r in rs])
        res = measure_side(db, bin_m)
        if res is not None:
            res.side = side.lower()
        out[side.lower()] = res
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("svlog", nargs="+")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args(argv)
    report = {}
    for f in args.svlog:
        res = measure(f)
        report[f] = {k: (asdict(v) if v else None) for k, v in res.items()}
        if not args.json:
            print(f)
            for side, v in res.items():
                if v is None:
                    print(f"  {side:9s} no persistent quay line")
                    continue
                print(f"  {side:9s} {v.pings} pings | peak {v.peak_minus_before_db:+.1f} dB, "
                      f"1.5-4 m beyond {v.beyond_minus_before_db:+.1f} dB, "
                      f"6-12 m beyond {v.far_minus_before_db:+.1f} dB, band "
                      f"{v.band_width_m:.2f} m, face at {v.face_range_m:.1f} m")
    if args.json:
        print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
