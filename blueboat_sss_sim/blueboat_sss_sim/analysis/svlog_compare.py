"""Reduce ``.svlog`` recordings -- simulated or real -- to one comparable table.

The 2026-09-02 sim-vs-real investigation settled three questions by reading
the same statistics off both kinds of log with the same code: the ping-rate
law, the reported-dB level structure, and the speckle texture. This module is
that measuring instrument, kept so every later change to the acoustic model
is judged against the field corpus by the numbers rather than by eye.

Per file, per ``(side, length_mm, num_results, gain_index)`` subset:

* **timing** -- median / p5 / p95 inter-ping ``timestamp_ms`` step, against
  the device law ``max(50 ms, 2R/c + processing)`` (Omniscan 450: 20 pings/s
  up to 30 m, range-limited beyond; measured on all seven corpus files), the
  reported ``pulse_duration_sec`` against the sample-period law, and the two
  simulator tells (counter resets on re-enable, identical port/starboard
  clocks);
* **level** -- mean reported dB at fixed slant ranges, the far-decile floor,
  the near-quarter level, ``max_pwr_db`` / ``min_pwr_db`` statistics and the
  per-ping 2-98 % span;
* **texture** -- after subtracting the subset's mean range profile (what the
  GCS ``range_equalize`` display does): std, p5-p95, lag-1/3 correlation
  along range, lag-1/3/10 correlation along track, and the residual std
  after a 9x9 box high-pass (the speckle contrast, 5.6 dB for single-look
  ``Exp(1)``).

Read-only (CM-7); imports nothing from ``BlueBoat-SSS`` (CM-3).
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from ..core.types import Side
from ..sonar.config import DEVICE_PROCESSING_MS, SPEED_OF_SOUND_MPS
from .calibration import scale_to_db
from .svlog_reader import ProfileRecord, read_profiles

#: Spec-sheet cap: 20 pings per second up to 30 m range.
MIN_PERIOD_MS = 50.0
#: Shortest transmit pulse the device reports (18-27 m / 1200 bins).
PULSE_FLOOR_S = 44.3e-6
#: Pulse length in sample periods at the two settings the corpus was recorded
#: with: the device law is max(44.3 us, pulse_len_percent x 2R/c)
#: (``AcquisitionParams.pulse_duration_s``), and the ROS driver's former
#: 0.002 at 600 bins is 1.2 periods, SonarView's 1.5 / num_results 1.5. The
#: frame does not carry pulse_len_percent, so this reference assumes those; a
#: 600-bin SonarView file (0.0025) reads 1.5 against it.
PULSE_SAMPLES = {600: 1.2, 1200: 1.5}
#: Slant ranges [m] at which the mean reported level is probed (+-10 %).
RANGE_PROBES_M = (0.5, 1.0, 2.0, 3.0, 5.0, 7.5, 10.0, 12.5, 15.0, 20.0, 25.0,
                  30.0, 35.0, 40.0, 60.0, 90.0)
#: Texture band, as fractions of the range window.
TEXTURE_BAND = (0.30, 0.70)


def device_period_ms(length_mm: int, start_mm: int = 0) -> float:
    """The device's free-run ping period for a range setting [ms]."""
    two_way = 2.0 * (start_mm + length_mm) / 1000.0 / SPEED_OF_SOUND_MPS
    return max(MIN_PERIOD_MS, two_way * 1000.0 + DEVICE_PROCESSING_MS)


def device_pulse_s(length_mm: int, num_results: int, start_mm: int = 0) -> float:
    """The device's transmit-pulse length for a range/bin setting [s]."""
    two_way = 2.0 * (start_mm + length_mm) / 1000.0 / SPEED_OF_SOUND_MPS
    k = PULSE_SAMPLES.get(int(num_results), 1.5)
    return max(PULSE_FLOOR_S, k * two_way / max(int(num_results), 1))


@dataclass
class SubsetStats:
    side: str
    length_mm: int
    num_results: int
    gain_index: int
    n_pings: int
    # timing
    period_median_ms: float
    period_p5_ms: float
    period_p95_ms: float
    period_law_ms: float
    period_residual_ms: float
    pulse_duration_us: float
    pulse_law_us: float
    # level
    max_pwr_db_mean: float
    max_pwr_db_sd: float
    min_pwr_db_mean: float
    span_mean_db: float
    span_2_98_db: float
    far_decile_db: float
    near_quarter_db: float
    db_at_range: dict[str, float] = field(default_factory=dict)
    # texture
    tex_std_db: float = float("nan")
    tex_p5_p95_db: float = float("nan")
    range_corr_lag1: float = float("nan")
    range_corr_lag3: float = float("nan")
    track_corr_lag1: float = float("nan")
    track_corr_lag3: float = float("nan")
    track_corr_lag10: float = float("nan")
    hp_std_db: float = float("nan")


@dataclass
class FileStats:
    path: str
    n_profiles: int
    counter_resets: int
    sides_share_timestamps: float     # fraction of port stamps also on stbd
    subsets: list[SubsetStats] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


# ------------------------------------------------------------------ helpers
def _corr(a: np.ndarray, b: np.ndarray) -> float:
    a = a.ravel() - a.mean()
    b = b.ravel() - b.mean()
    d = np.sqrt((a * a).mean() * (b * b).mean())
    return float((a * b).mean() / d) if d > 0 else float("nan")


def _box_highpass(db: np.ndarray, k: int = 9) -> np.ndarray:
    """``db`` minus its k x k box mean (separable cumulative sums; no scipy)."""
    def _box1(x: np.ndarray, axis: int) -> np.ndarray:
        n = x.shape[axis]
        if n < k:
            return x - x.mean(axis=axis, keepdims=True) + x.mean(axis=axis, keepdims=True)
        pad = k // 2
        xp = np.concatenate([np.repeat(np.take(x, [0], axis=axis), pad, axis=axis),
                             x,
                             np.repeat(np.take(x, [n - 1], axis=axis), pad, axis=axis)],
                            axis=axis)
        cs = np.cumsum(xp, axis=axis)
        cs = np.concatenate([np.zeros_like(np.take(cs, [0], axis=axis)), cs], axis=axis)
        hi = np.take(cs, np.arange(k, k + n), axis=axis)
        lo = np.take(cs, np.arange(0, n), axis=axis)
        return (hi - lo) / k
    return db - _box1(_box1(db, 1), 0)


def texture_stats(db: np.ndarray) -> dict[str, float]:
    """Texture statistics of a (pings x bins) dB block, mid-range band."""
    n = db.shape[1]
    lo, hi = int(TEXTURE_BAND[0] * n), int(TEXTURE_BAND[1] * n)
    eq = db - db.mean(axis=0, keepdims=True)
    tex = eq[:, lo:hi]
    out = dict(tex_std_db=float(tex.std()),
               tex_p5_p95_db=float(np.percentile(tex, 95) - np.percentile(tex, 5)))
    if tex.shape[1] > 4:
        out["range_corr_lag1"] = _corr(tex[:, :-1], tex[:, 1:])
        out["range_corr_lag3"] = _corr(tex[:, :-3], tex[:, 3:])
    if tex.shape[0] > 11:
        out["track_corr_lag1"] = _corr(tex[:-1], tex[1:])
        out["track_corr_lag3"] = _corr(tex[:-3], tex[3:])
        out["track_corr_lag10"] = _corr(tex[:-10], tex[10:])
    hp = _box_highpass(db)[:, lo:hi]
    out["hp_std_db"] = float(hp.std())
    return out


def level_at_ranges(db: np.ndarray, length_mm: int, num_results: int,
                    start_mm: int = 0) -> dict[str, float]:
    """Mean reported dB in a +-10 % window around each probe range."""
    r = start_mm / 1000.0 + (np.arange(num_results) + 0.5) * (length_mm / 1000.0
                                                              / num_results)
    r_max = (start_mm + length_mm) / 1000.0
    prof = db.mean(axis=0)
    out: dict[str, float] = {}
    for x in RANGE_PROBES_M:
        if x > r_max:
            continue
        m = (r >= 0.9 * x) & (r <= 1.1 * x)
        if m.any():
            out[f"{x:g}"] = float(prof[m].mean())
    return out


def _subset(side: Side, recs: Sequence[ProfileRecord], stride: int
            ) -> SubsetStats:
    first = recs[0]
    ts = np.array([p.timestamp_ms for p in recs], dtype=np.float64)
    dt = np.diff(ts)
    dt = dt[(dt > 0.0) & (dt < 2000.0)]
    law = device_period_ms(first.length_mm, first.start_mm)
    med = float(np.median(dt)) if dt.size else float("nan")

    sel = recs[::max(1, stride)]
    db = np.stack([scale_to_db(p.pwr_results, p.min_pwr_db, p.max_pwr_db)
                   for p in sel])
    n = db.shape[1]
    mx = np.array([p.max_pwr_db for p in recs])
    mn = np.array([p.min_pwr_db for p in recs])
    p2, p98 = np.percentile(db, [2, 98], axis=1)
    pulse_us = float(np.median([p.pulse_duration_sec for p in recs])) * 1e6 \
        if hasattr(first, "pulse_duration_sec") else float("nan")

    st = SubsetStats(
        side=side.value, length_mm=int(first.length_mm),
        num_results=int(first.num_results), gain_index=int(first.gain_index),
        n_pings=len(recs),
        period_median_ms=med,
        period_p5_ms=float(np.percentile(dt, 5)) if dt.size else float("nan"),
        period_p95_ms=float(np.percentile(dt, 95)) if dt.size else float("nan"),
        period_law_ms=law, period_residual_ms=med - law,
        pulse_duration_us=pulse_us,
        pulse_law_us=device_pulse_s(first.length_mm, first.num_results,
                                    first.start_mm) * 1e6,
        max_pwr_db_mean=float(mx.mean()), max_pwr_db_sd=float(mx.std()),
        min_pwr_db_mean=float(mn.mean()),
        span_mean_db=float((mx - mn).mean()),
        span_2_98_db=float((p98 - p2).mean()),
        far_decile_db=float(db[:, int(0.9 * n):].mean()),
        near_quarter_db=float(db[:, int(0.2 * n):int(0.3 * n)].mean()),
        db_at_range=level_at_ranges(db, first.length_mm, first.num_results,
                                    first.start_mm),
    )
    for k, v in texture_stats(db).items():
        setattr(st, k, v)
    return st


# ------------------------------------------------------------------ public
def reduce_profiles(profiles: Sequence[ProfileRecord], *, stride: int = 3,
                    min_pings: int = 50) -> tuple[list[SubsetStats], int, float]:
    """Reduce decoded profiles: ``(subsets, counter_resets, shared_stamps)``.

    ``profiles`` must carry power (read with ``with_power=True``); a
    ``pulse_duration_sec`` attribute is used when present.
    """
    by: dict[tuple, list[ProfileRecord]] = {}
    per_side: dict[Side, list[ProfileRecord]] = {Side.PORT: [], Side.STARBOARD: []}
    for p in profiles:
        by.setdefault((p.side, p.length_mm, p.num_results, p.gain_index),
                      []).append(p)
        per_side[p.side].append(p)

    resets = 0
    for recs in per_side.values():
        pn = np.array([p.ping_number for p in recs])
        if pn.size > 1:
            # A re-enable restarts the simulator's counter at 1; a real log
            # only ever shows small out-of-order steps from the two devices'
            # interleaved packets, never a drop back to the start.
            resets += int(np.count_nonzero(np.diff(pn) < -100))
    port_ts = {p.timestamp_ms for p in per_side[Side.PORT]}
    stbd_ts = {p.timestamp_ms for p in per_side[Side.STARBOARD]}
    shared = (len(port_ts & stbd_ts) / len(port_ts)) if port_ts else 0.0

    subsets = [_subset(k[0], recs, stride) for k, recs in by.items()
               if len(recs) >= min_pings]
    subsets.sort(key=lambda s: (-s.n_pings, s.side))
    return subsets, resets, shared


def reduce_file(path: str | Path, *, stride: int = 3,
                min_pings: int = 50) -> FileStats:
    """Reduce one ``.svlog`` (opened read-only)."""
    profiles = _read_with_pulse(path)
    subsets, resets, shared = reduce_profiles(profiles, stride=stride,
                                              min_pings=min_pings)
    return FileStats(path=str(path), n_profiles=len(profiles),
                     counter_resets=resets, sides_share_timestamps=shared,
                     subsets=subsets)


def _read_with_pulse(path: str | Path) -> list[ProfileRecord]:
    """``read_profiles(with_power=True)`` plus the pulse length per record.

    ``ProfileRecord`` deliberately omits ``pulse_duration_sec``; it is attached
    here as a plain attribute on a mutable copy so the reader's contract stays
    untouched.
    """
    from dataclasses import replace  # noqa: PLC0415 -- local, tiny
    from ..sonar.encoder import OS_MONO_PROFILE_ID, parse_frame
    from .svlog_reader import iter_packets
    out: list[ProfileRecord] = []
    for pkt in iter_packets(path):
        if pkt.packet_id != OS_MONO_PROFILE_ID:
            continue
        d = parse_frame(pkt.raw)
        rec = _record_from_decoded(d)
        object.__setattr__(rec, "pulse_duration_sec", float(d["pulse_duration_sec"]))
        out.append(rec)
    del replace
    return out


def _record_from_decoded(d: dict) -> ProfileRecord:
    from .svlog_reader import _side_from_packet
    return ProfileRecord(
        side=_side_from_packet(d), ping_number=int(d["ping_number"]),
        timestamp_ms=int(d["timestamp_ms"]),
        vehicle_heading_deg=float(d["vehicle_heading_deg"]),
        start_mm=int(d["start_mm"]), length_mm=int(d["length_mm"]),
        num_results=int(d["num_results"]), gain_index=int(d["gain_index"]),
        analog_gain=float(d["analog_gain"]), min_pwr_db=float(d["min_pwr_db"]),
        max_pwr_db=float(d["max_pwr_db"]), pwr_results=d["pwr_results"])


_ROWS: list[tuple[str, str, str]] = [
    # (label, attribute, format)
    ("pings", "n_pings", "d"),
    ("period median [ms]", "period_median_ms", ".1f"),
    ("period p5-p95 [ms]", None, ""),
    ("device law [ms]", "period_law_ms", ".1f"),
    ("law residual [ms]", "period_residual_ms", "+.1f"),
    ("pulse reported / law [us]", None, ""),
    ("max_pwr_db mean +- sd", None, ""),
    ("min_pwr_db mean", "min_pwr_db_mean", ".1f"),
    ("span mean [dB]", "span_mean_db", ".1f"),
    ("2-98 % span [dB]", "span_2_98_db", ".1f"),
    ("near-quarter level [dB]", "near_quarter_db", ".1f"),
    ("far-decile floor [dB]", "far_decile_db", ".1f"),
    ("texture std [dB]", "tex_std_db", ".1f"),
    ("texture p5-p95 [dB]", "tex_p5_p95_db", ".1f"),
    ("range corr lag1 / lag3", None, ""),
    ("track corr lag1 / 3 / 10", None, ""),
    ("high-pass speckle std [dB]", "hp_std_db", ".1f"),
]


def _cell(s: SubsetStats, label: str, attr: str | None, fmt: str) -> str:
    if attr:
        v = getattr(s, attr)
        return f"{v:{fmt}}" if np.isfinite(v) else "-"
    if label.startswith("period p5"):
        return f"{s.period_p5_ms:.0f}-{s.period_p95_ms:.0f}"
    if label.startswith("pulse"):
        return f"{s.pulse_duration_us:.1f} / {s.pulse_law_us:.1f}"
    if label.startswith("max_pwr_db"):
        return f"{s.max_pwr_db_mean:.1f} +- {s.max_pwr_db_sd:.1f}"
    if label.startswith("range corr"):
        return f"{s.range_corr_lag1:.2f} / {s.range_corr_lag3:.2f}"
    if label.startswith("track corr"):
        return (f"{s.track_corr_lag1:.2f} / {s.track_corr_lag3:.2f} / "
                f"{s.track_corr_lag10:.2f}")
    return "-"


def compare(stats: Iterable[FileStats], *, max_subsets: int = 3) -> str:
    """Markdown table: one column per (file, largest subsets)."""
    cols: list[tuple[str, SubsetStats]] = []
    for fs in stats:
        name = Path(fs.path).stem[:28]
        for s in fs.subsets[:max_subsets]:
            cols.append((f"{name} {s.side[:4]} {s.length_mm/1000:.1f}m/"
                         f"{s.num_results} g{s.gain_index}", s))
    lines = ["# svlog comparison", ""]
    for fs in stats:
        lines.append(f"- `{fs.path}`: {fs.n_profiles} profiles, "
                     f"{fs.counter_resets} counter reset(s), "
                     f"{100*fs.sides_share_timestamps:.0f} % of port stamps "
                     f"also on starboard")
    lines += ["", "| statistic | " + " | ".join(c for c, _ in cols) + " |",
              "|---|" + "---|" * len(cols)]
    for label, attr, fmt in _ROWS:
        lines.append(f"| {label} | "
                     + " | ".join(_cell(s, label, attr, fmt) for _, s in cols)
                     + " |")
    probes = [f"{x:g}" for x in RANGE_PROBES_M]
    lines += ["", "Mean reported dB at slant range [m]:", "",
              "| range | " + " | ".join(c for c, _ in cols) + " |",
              "|---|" + "---|" * len(cols)]
    for pr in probes:
        vals = [s.db_at_range.get(pr) for _, s in cols]
        if all(v is None for v in vals):
            continue
        lines.append(f"| {pr} | " + " | ".join(
            "-" if v is None else f"{v:.1f}" for v in vals) + " |")
    lines.append("")
    return "\n".join(lines)


def write_compare(stats: list[FileStats], out_dir: str | Path,
                  *, max_subsets: int = 3) -> tuple[Path, Path]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    js = out / "compare.json"
    md = out / "compare.md"
    js.write_text(json.dumps([fs.to_dict() for fs in stats], indent=2),
                  encoding="utf-8")
    md.write_text(compare(stats, max_subsets=max_subsets), encoding="utf-8")
    return js, md


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="sss_svlog_compare",
        description="Reduce .svlog recordings (sim or real) to one comparable "
                    "table of timing, level and texture statistics.")
    ap.add_argument("svlog", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True,
                    help="directory for compare.json + compare.md")
    ap.add_argument("--stride", type=int, default=3,
                    help="use every n-th ping for the power statistics")
    ap.add_argument("--min-pings", type=int, default=50)
    ap.add_argument("--max-subsets", type=int, default=3,
                    help="columns per file in the markdown table")
    a = ap.parse_args(argv)
    stats = [reduce_file(p, stride=a.stride, min_pings=a.min_pings)
             for p in a.svlog]
    js, md = write_compare(stats, a.out, max_subsets=a.max_subsets)
    print(md.read_text(encoding="utf-8"))
    print(f"wrote {js} and {md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
