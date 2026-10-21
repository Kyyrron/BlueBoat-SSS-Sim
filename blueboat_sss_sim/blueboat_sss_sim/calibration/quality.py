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
  device law, a weak bottom return, one side only, no navigation);
* ``info``   -- worth knowing (``channel_number`` 255 present, the first
  pings' counter offset between the two devices).

Read-only, and privacy-preserving: the report carries the file's name,
size and optional SHA-256, never its path tree, and never a position fix
(the digest's ``first_fix`` is dropped). ROS-free.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

QUALITY_FORMAT = "blueboat_svlog_quality/1"
#: Ping period off the device law by more than this is flagged [ms].
PERIOD_TOLERANCE_MS = 5.0
#: Bottom return found on fewer sampled pings than this is flagged.
MIN_FBR_FOUND = 0.9
#: A normalisation invariant holding on fewer pings than this is an error;
#: below 1.0 a warning (a real log was seen with 1 of 4236 pings carrying
#: two bins tied at 65535, 2026-10-21).
INVARIANT_ERROR_BELOW = 0.99


def _flag(flags: list, level: str, code: str, msg: str) -> None:
    flags.append({"level": level, "code": code, "message": msg})


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
        per, law = s.get("period_ms"), s.get("period_law_ms")
        if per == per and law == law and abs(per - law) > PERIOD_TOLERANCE_MS:
            _flag(flags, "warn", f"{name}_period",
                  f"{name}: ping period {per:.1f} ms vs device law "
                  f"{law:.1f} ms (mode setting; dropped pings or a "
                  "simulated clock)")
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
           "usable": not any(f["level"] == "error" for f in flags)}
    if with_hash:
        out["sha256"] = sha256_file(p)
    return out
