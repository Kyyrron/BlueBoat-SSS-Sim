"""Data-quality report for a ``.svlog`` before it may enter a fit.

Built on the readers the rest of the package already trusts
(``analysis.svlog_digest.digest``, ``analysis.svlog_reader.read_svlog``,
``analysis.calibration.normalisation_invariants``): it adds nothing new to
the format, only turns what they measure into **flags** a calibration run
must look at before using the recording:

* ``error``  -- the file is not what a calibration can use as is (the
  device's per-ping normalisation invariants fail; no pings);
* ``warn``   -- usable only with care (gain walk, mixed acquisitions,
  counter restarts, a frozen autopilot clock, a ping period off the
  device law for one of its settings, missing ping numbers (> 1 %) or
  duplicated ones, a weak bottom return, one side only, no navigation);
* ``info``   -- worth knowing (``channel_number`` 255 present, the first
  pings' counter offset between the two devices).

Read-only, and privacy-preserving: the report carries the file's name,
size and optional SHA-256, never its path tree, and never a position fix
(the digest's ``first_fix`` is dropped). ROS-free.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

QUALITY_FORMAT = "blueboat_svlog_quality/1"
#: Ping period off the device law by more than this is flagged [ms].
PERIOD_TOLERANCE_MS = 5.0
#: Bottom return found on fewer sampled pings than this is flagged.
MIN_FBR_FOUND = 0.9
#: A normalisation invariant holding on fewer pings than this is an error;
#: below 1.0 a warning (a real log was seen with 1 of 4236 pings carrying
#: two bins tied at 65535, 2026-10-21).
INVARIANT_ERROR_BELOW = 0.99


#: Minimum consecutive same-setting intervals before a setting's period is judged.
MIN_PERIOD_INTERVALS = 20
#: Missing ping numbers above this share of a side's pings is a warning
#: (any is reported as info).
GAP_WARN_FRACTION = 0.01


def _flag(flags: list, level: str, code: str, msg: str) -> None:
    flags.append({"level": level, "code": code, "message": msg})


def acquisition_periods(profiles) -> dict[str, list[dict]]:
    """Per side and per acquisition setting, the median interval between
    consecutive pings *of that setting* against the device law.

    The digest's single ``period_ms`` is the median over every interval of
    the side, judged against the law of the most common setting. On a file
    that switches range often, the two describe different settings:
    ``toOptimize-harbourScan`` (8 settings, the most common at 39 m) read
    50.0 ms against a 55.0 ms law, which is no defect of the device
    (2026-10-23).
    """
    from ..analysis.svlog_compare import device_period_ms
    out: dict[str, list[dict]] = {}
    for side in ("port", "starboard"):
        # File order, which is each device's time order; a timestamp sort
        # would interleave concatenated files whose device clocks restart.
        recs = [p for p in profiles if p.side.value == side]
        by: dict[tuple, list[float]] = {}
        for a, b in zip(recs, recs[1:]):
            key = (int(b.length_mm), int(b.start_mm))
            if (int(a.length_mm), int(a.start_mm)) != key:
                continue
            dt = float(b.timestamp_ms - a.timestamp_ms)
            if 0.0 < dt < 2000.0:
                by.setdefault(key, []).append(dt)
        rows = []
        for (length, start), dts in sorted(by.items()):
            rows.append({"range_m": round((start + length) / 1000.0, 3),
                         "intervals": len(dts),
                         "period_ms": round(float(np.median(dts)), 2),
                         "law_ms": round(device_period_ms(length, start), 2)})
        if rows:
            out[side] = rows
    return out


def counter_census(profiles) -> dict[str, dict]:
    """Per side, in file order: ping-counter gaps (missing numbers between
    consecutive pings), exact duplicates, small reorders and restarts.

    A jump back of more than 100 is a restart (the digest's rule) and a
    jump forward of more than 100 is not counted as missing pings; both
    sides keep their own counter (CM-6), so nothing here pairs them."""
    out: dict[str, dict] = {}
    for side in ("port", "starboard"):
        pn = np.array([p.ping_number for p in profiles if p.side.value == side],
                      dtype=np.int64)
        if pn.size < 2:
            continue
        d = np.diff(pn)
        gaps = d[(d > 1) & (d <= 100)] - 1
        out[side] = {"pings": int(pn.size),
                     "missing": int(gaps.sum()),
                     "gap_events": int(gaps.size),
                     "max_gap": int(gaps.max()) if gaps.size else 0,
                     "duplicates": int(np.count_nonzero(d == 0)),
                     "reordered": int(np.count_nonzero((d < 0) & (d >= -100))),
                     "restarts": int(np.count_nonzero(d < -100))}
    return out


def svlog_quality(path: str | Path, *, with_hash: bool = False) -> dict[str, Any]:
    """Quality report of one ``.svlog`` (see module doc)."""
    from ..analysis.calibration import normalisation_invariants
    from ..analysis.svlog_digest import digest
    from ..analysis.svlog_reader import read_svlog
    from .manifest import sha256_file

    p = Path(path)
    g = digest(p)
    g.pop("first_fix", None)          # never carry a position fix
    g.pop("path", None)
    st = read_svlog(p, with_power=True)
    inv = normalisation_invariants(st.profiles)
    periods = acquisition_periods(st.profiles)
    census = counter_census(st.profiles)
    flags: list[dict] = []
    sides = g.get("sides") or {}
    if not sides:
        _flag(flags, "error", "no_pings", "no OS_MONO_PROFILE (2198) packets")
    if inv.get("n", 0):
        for k in ("single_full_scale_bin", "zero_minimum", "span_within_90db"):
            if inv[k] < INVARIANT_ERROR_BELOW:
                _flag(flags, "error", f"invariant_{k}",
                      f"{k} holds on {inv[k]:.2%} of pings: not "
                      "device-normalised data, or damaged"
                      + (" (simulator recordings encoded before 2026-10-21 "
                         "exceed 90 dB by float32 rounding, ~4e-6 dB)"
                         if k == "span_within_90db" else ""))
            elif inv[k] < 1.0:
                bad = int(round((1.0 - inv[k]) * inv["n"]))
                _flag(flags, "warn", f"invariant_{k}",
                      f"{k} fails on {bad} of {inv['n']} pings (e.g. two "
                      "bins tied at full scale); inspect before use")
    if len(sides) == 1:
        _flag(flags, "warn", "one_side",
              f"only {next(iter(sides))} present")
    for name, s in sides.items():
        gains = s.get("gain_index") or {}
        if len(gains) > 1:
            _flag(flags, "warn", f"{name}_gain_walk",
                  f"{name}: gain indices {sorted(gains)} -- level fits must "
                  "use the per-ping gain (20 log10 steps), or one index")
        if len(s.get("acquisition") or {}) > 1:
            _flag(flags, "warn", f"{name}_mixed_acquisition",
                  f"{name}: {len(s['acquisition'])} range/bin settings -- "
                  "reduce per setting")
        if s.get("counter_restarts", 0):
            _flag(flags, "warn", f"{name}_counter_restart",
                  f"{name}: {s['counter_restarts']} ping-counter restart(s)")
        off = [r for r in periods.get(name, ())
               if r["intervals"] >= MIN_PERIOD_INTERVALS
               and abs(r["period_ms"] - r["law_ms"]) > PERIOD_TOLERANCE_MS]
        if off:
            _flag(flags, "warn", f"{name}_period",
                  f"{name}: ping period off the device law for "
                  + ", ".join(f"{r['range_m']:g} m ({r['period_ms']:.1f} vs "
                              f"{r['law_ms']:.1f} ms)" for r in off)
                  + " (dropped pings, a forced interval or a simulated clock)")
        cc = census.get(name)
        if cc and cc["missing"]:
            share = cc["missing"] / max(cc["pings"] + cc["missing"], 1)
            _flag(flags, "warn" if share > GAP_WARN_FRACTION else "info",
                  f"{name}_ping_gaps",
                  f"{name}: {cc['missing']} ping number(s) missing in "
                  f"{cc['gap_events']} gap(s), largest {cc['max_gap']} "
                  f"({100 * share:.2f} % of the side's pings)")
        if cc and cc["duplicates"]:
            _flag(flags, "warn", f"{name}_duplicate_pings",
                  f"{name}: {cc['duplicates']} ping(s) repeat the previous "
                  "ping number (duplicated packets?)")
        if s.get("fbr_found", 1.0) < MIN_FBR_FOUND:
            _flag(flags, "warn", f"{name}_weak_fbr",
                  f"{name}: bottom return found on "
                  f"{s.get('fbr_found', 0):.0%} of sampled pings")
        if s.get("channel_255", 0):
            _flag(flags, "info", f"{name}_channel_255",
                  f"{name}: channel_number 255 on {s['channel_255']} pings "
                  "(side from the heading fallback, CM-5)")
    if g.get("clock_frozen"):
        _flag(flags, "warn", "clock_frozen",
              "autopilot time_boot_ms frozen: navigation pairs by file "
              "order, not by time")
    if not g.get("gps_fixes"):
        _flag(flags, "warn", "no_gps", "no GPS fixes: cannot georeference")
    if "port" in sides and "starboard" in sides:
        off = (sides["starboard"]["ping_number"][0]
               - sides["port"]["ping_number"][0])
        _flag(flags, "info", "counter_offset_first",
              f"starboard - port first ping number = {off} (per-device "
              "counters; align before pairing, CM-6)")
    out = {"format": QUALITY_FORMAT, "file": p.name,
           "size_bytes": p.stat().st_size, "digest": g,
           "normalisation_invariants": inv, "flags": flags,
           "acquisition_periods": periods, "counter_census": census,
           "usable": not any(f["level"] == "error" for f in flags)}
    if with_hash:
        out["sha256"] = sha256_file(p)
    return out
