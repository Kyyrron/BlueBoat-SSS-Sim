"""One-screen, read-only digest of ``.svlog`` recordings -- "what is in this file".

    python3 -m blueboat_sss_sim.analysis.svlog_digest FILE.svlog [...] [--json]

Run from the package source root (``BlueBoat-SSS-Sim/blueboat_sss_sim/``). It
writes nothing (CM-7). Packet census, sessions, per-side acquisition (range /
bins / gain walk / pulse), ping period against the device law, counter range and
restarts, ``channel_number`` 255 use, the bottom return's median slant range,
the GPS / attitude / position streams and the autopilot clock regime -- about
4 s for 175 MB. Counter offsets, ping gaps, ``src`` mis-tags and waterfall
pictures are the GCS forensics tool's (``blueboat_gcs.analysis.svlog_forensics``
in ``BlueBoat-SSS``). The superproject skill ``.claude/skills/svlog/SKILL.md``
says which tool answers which question.

A separate module rather than a ``__main__`` in :mod:`.svlog_reader`: the
package ``__init__`` imports the reader, so ``python3 -m`` on it would warn.
"""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

import numpy as np

from ..sonar.encoder import OS_MONO_PROFILE_ID, parse_frame
from .calibration import fbr_bin, scale_to_db
from .svlog_compare import device_period_ms
from .svlog_reader import _CHANNEL_UNKNOWN, _side_from_packet, iter_packets, read_svlog

#: Every n-th profile per side feeds the bottom-return (FBR) statistic.
_DIGEST_FBR_STRIDE = 10

#: Ping Protocol ids seen in the corpus (``BlueBoat-SSS/.claude/CLAUDE.md``,
#: the ``.svlog`` paragraph).
_PACKET_NAMES = {10: "session JSON", 12: "view config (SonarView)",
                 150: "mavlink JSON", 2194: "Omniscan status",
                 OS_MONO_PROFILE_ID: "OS_MONO_PROFILE"}


def digest(path: str | Path) -> dict:
    """What is in a ``.svlog``, in one read-only pass plus :func:`read_svlog`.

    Packet census, sessions, per-side acquisition (range / bins / gain /
    pulse), ping period against the device law, counter range and restarts,
    ``channel_number`` 255 use, the bottom return's median slant range, the
    GPS / attitude / position streams and the clock regime. Everything a
    first look needs; counter offsets, gaps and ``src`` mis-tags are the GCS
    forensics tool's (``blueboat_gcs.analysis.svlog_forensics``).
    """
    p = Path(path)
    census: Counter = Counter()
    sides: dict[str, dict] = {}
    for pkt in iter_packets(p):
        census[pkt.packet_id] += 1
        if pkt.packet_id != OS_MONO_PROFILE_ID:
            continue
        d = parse_frame(pkt.raw)
        s = sides.setdefault(_side_from_packet(d).value, dict(
            acq=Counter(), gain=Counter(), pulse=[], ts=[], pn=[], ch255=0,
            fbr=[], n=0))
        s["n"] += 1
        s["acq"][(int(d["length_mm"]), int(d["num_results"]))] += 1
        s["gain"][int(d["gain_index"])] += 1
        s["pulse"].append(float(d["pulse_duration_sec"]))
        s["ts"].append(int(d["timestamp_ms"]))
        s["pn"].append(int(d["ping_number"]))
        s["ch255"] += int(d["channel_number"]) == _CHANNEL_UNKNOWN
        if s["n"] % _DIGEST_FBR_STRIDE == 1 and int(d["num_results"]) > 0:
            i = fbr_bin(scale_to_db(d["pwr_results"].astype(np.float64),
                                    d["min_pwr_db"], d["max_pwr_db"]))
            bin_m = int(d["length_mm"]) / 1000.0 / int(d["num_results"])
            s["fbr"].append(np.nan if i is None
                            else int(d["start_mm"]) / 1000.0 + i * bin_m)
    out_sides = {}
    for name, s in sorted(sides.items()):
        ts = np.sort(np.asarray(s["ts"], dtype=np.float64))
        dt = np.diff(ts)
        dt = dt[(dt > 0.0) & (dt < 2000.0)]
        (length_mm, _n), _ = s["acq"].most_common(1)[0]
        fbr = np.asarray(s["fbr"], dtype=np.float64)
        pn = np.asarray(s["pn"])
        out_sides[name] = dict(
            pings=s["n"],
            acquisition={f"{l / 1000:g} m / {n} bins": c
                         for (l, n), c in s["acq"].most_common()},
            gain_index=dict(sorted(s["gain"].items())),
            pulse_us=float(np.median(s["pulse"])) * 1e6,
            period_ms=float(np.median(dt)) if dt.size else float("nan"),
            period_law_ms=device_period_ms(length_mm),
            device_span_s=float((ts[-1] - ts[0]) / 1000.0) if ts.size else 0.0,
            ping_number=(int(pn.min()), int(pn.max())),
            counter_restarts=int(np.count_nonzero(np.diff(pn) < -100)),
            channel_255=int(s["ch255"]),
            fbr_median_m=(float(np.nanmedian(fbr))
                          if np.isfinite(fbr).any() else float("nan")),
            fbr_found=float(np.isfinite(fbr).mean()) if fbr.size else 0.0)
    st = read_svlog(p)
    return dict(
        path=str(p), size_mb=p.stat().st_size / 1e6,
        packets={f"{k} {_PACKET_NAMES.get(k, '?')}": v
                 for k, v in sorted(census.items())},
        sessions=int(census.get(10, 0)),
        sonarview_view_state=bool(census.get(12, 0)),
        sides=out_sides,
        gps_fixes=len(st.gps), attitude=len(st.attitude),
        positions=len(st.positions), clock_frozen=st.clock_frozen,
        first_fix=((st.gps[0].lat, st.gps[0].lon) if st.gps else None))


def format_digest(g: dict) -> str:
    """The :func:`digest` as one screen of text."""
    lines = [f"{g['path']}  ({g['size_mb']:.1f} MB)",
             "  packets: " + ", ".join(f"{k}: {v}" for k, v in g["packets"].items()),
             f"  sessions (id-10 headers): {g['sessions']}; SonarView view state "
             f"(id 12): {'yes' if g['sonarview_view_state'] else 'no'}"]
    for name, s in g["sides"].items():
        lines += [
            f"  {name}: {s['pings']} pings over {s['device_span_s']:.1f} s "
            f"(device clock); counter {s['ping_number'][0]}..{s['ping_number'][1]}, "
            f"{s['counter_restarts']} restart(s); channel 255 on {s['channel_255']}",
            "    acquisition: " + ", ".join(f"{k} x{v}" for k, v in s["acquisition"].items())
            + "; gain_index " + ", ".join(f"{k}: {v}" for k, v in s["gain_index"].items()),
            f"    pulse {s['pulse_us']:.1f} us; ping period {s['period_ms']:.1f} ms "
            f"(device law {s['period_law_ms']:.1f} ms); bottom return median "
            f"{s['fbr_median_m']:.2f} m slant, found on {100 * s['fbr_found']:.0f} % "
            f"of sampled pings"]
    fix = (f"first fix {g['first_fix'][0]:.6f}, {g['first_fix'][1]:.6f}"
           if g["first_fix"] else "no fix")
    lines.append(f"  mavlink: {g['gps_fixes']} GPS fixes ({fix}), "
                 f"{g['attitude']} attitude, {g['positions']} local positions; "
                 f"autopilot clock "
                 f"{'FROZEN (processor-written: pair by file order)' if g['clock_frozen'] else 'live'}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        prog="python3 -m blueboat_sss_sim.analysis.svlog_reader",
        description="One-screen, read-only digest of .svlog recordings "
                    "(writes nothing).")
    ap.add_argument("svlog", nargs="+", type=Path)
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    a = ap.parse_args(argv)
    ds = [digest(f) for f in a.svlog]
    print(json.dumps(ds, indent=2) if a.json
          else "\n\n".join(format_digest(g) for g in ds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
