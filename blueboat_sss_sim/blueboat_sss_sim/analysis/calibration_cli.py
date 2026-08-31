"""``sss_calibration_report`` CLI: a recording and a bundle -> the fit's evidence.

Reduces a real ``.svlog`` and a simulated bundle to the same statistic -- the
per-range mean dB referenced to each ping's own first-bottom-return peak,
grouped by range setting and altitude band -- and reports the residual between
them against the reduction's own repeatability floor::

    python3 -m blueboat_sss_sim.analysis.calibration_cli \\
        --svlog ~/ros2_ws/data/SSS_data/diffDepthCompensation.svlog \\
        --bundle ~/runs/r3 --out ~/calib/r3

Reads the recording and the bundle; writes only into ``--out``. Recordings are
primary field data and bundles are immutable snapshots -- neither is ever
written to (CM-7 / NC #10).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from ..core.types import Pose3D, Side
from ..mission.patterns import WaypointTrajectory
from ..sonar.config import SonarConfig
from ..sonar.encoder import PingEncoder
from ..sonar.noise import apply_ping_noise
from ..sonar.renderer import GeometricRenderer
from ..worldgen.scene import SceneModel
from .calibration import (REDUCTION_FLOOR_DB, RangeCurve, compare,
                          normalisation_invariants, reduce_profiles)
from .svlog_reader import ProfileRecord, read_profiles

#: Altitude bands further apart than this are not compared: grazing angle at a
#: given slant range moves with altitude, so the curves are different curves.
MAX_ALT_GAP_M = 1.5

#: Two passes of the same real harbour, at the same range setting and altitude
#: band, disagree by this much RMS (measured: 1.8-3.1 dB over four bands). It is
#: the floor a flat-seabed model faces against survey data, well above the
#: reduction's own 0.67 dB repeatability.
SITE_FLOOR_DB = 3.1


def simulate_profiles(bundle: Path, n_pings: int, seed: int,
                      t0: float = 0.0, dt: float = 0.5
                      ) -> list[ProfileRecord]:
    """Render the bundle and encode it into the same record type as a log.

    Going through :class:`PingEncoder` rather than reading the renderer's
    float power directly is the point: the comparison must see exactly the
    bytes a downstream consumer would, normalisation and all.
    """
    scene = SceneModel.load(bundle)
    cfg = SonarConfig.from_yaml(bundle / "sonar.yaml")
    acq, model = cfg.acquisition, cfg.model
    traj = WaypointTrajectory.load_yaml(bundle / "trajectory.yaml")
    rr = GeometricRenderer(scene, acq, model)
    rng = np.random.default_rng(seed)
    bin_r = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m

    out: list[ProfileRecord] = []
    for side in (Side.PORT, Side.STARBOARD):
        enc = PingEncoder(side, acq, model)
        for k in range(n_pings):
            t = t0 + k * dt
            x, y, yaw = traj.pose_at(t)
            ping = rr.render(side, Pose3D(x, y, 0.0, 0.0, 0.0, yaw), t).ping
            ping.power = apply_ping_noise(ping.power, bin_r < ping.altitude_m,
                                          1.0, model, rng,
                                          specular=ping.specular)
            e = enc.encode(ping)
            out.append(ProfileRecord(
                side=side, ping_number=e.ping_number,
                timestamp_ms=e.timestamp_ms,
                vehicle_heading_deg=e.vehicle_heading_deg,
                start_mm=e.start_mm, length_mm=e.length_mm,
                num_results=e.num_results, gain_index=e.gain_index,
                analog_gain=e.analog_gain, min_pwr_db=e.min_pwr_db,
                max_pwr_db=e.max_pwr_db, pwr_results=e.pwr_results,
            ))
    return out


def _curve_key(c: RangeCurve) -> tuple:
    return (c.side.value, c.altitude_band_m)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--svlog", required=True, help="real recording (read-only)")
    ap.add_argument("--bundle", required=True, help="mission bundle (read-only)")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--sim-pings", type=int, default=200,
                    help="ping cycles to render per side (default: 200)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--altitude-band-m", type=float, default=1.0)
    ap.add_argument("--band", type=float, nargs=2, default=(6.0, 14.5),
                    metavar=("LO", "HI"),
                    help="slant-range band to score over, metres")
    args = ap.parse_args()

    real_profiles = read_profiles(args.svlog, with_power=True)
    sim_profiles = simulate_profiles(Path(args.bundle), args.sim_pings,
                                     args.seed)

    real = reduce_profiles(real_profiles,
                           altitude_band_m=args.altitude_band_m)
    sim = reduce_profiles(sim_profiles,
                          altitude_band_m=args.altitude_band_m, min_pings=10)

    lo, hi = args.band
    # Score every simulated curve against the real curve on the same side whose
    # altitude band is closest, and say plainly whether the two are comparable
    # at all. Altitude and range set
    # the whole shape of the curve, so a residual taken across mismatched
    # geometry measures the mismatch and nothing else. It is reported as not
    # comparable rather than as a number, because a large figure there reads as
    # a bad fit when it is not one.
    rows = []
    for s in sim:
        cands = [r for r in real if r.side is s.side
                 and r.slant_r_m[-1] >= hi]
        if not cands:
            continue
        r = min(cands, key=lambda c: abs(c.altitude_band_m - s.altitude_band_m))
        matched = (abs(r.altitude_band_m - s.altitude_band_m) <= MAX_ALT_GAP_M
                   and abs(r.length_mm - s.length_mm)
                   <= 0.10 * max(r.length_mm, s.length_mm))
        res = compare(r, s, lo, hi)
        rows.append({
            "side": s.side.value,
            "sim_altitude_band_m": s.altitude_band_m,
            "real_altitude_band_m": r.altitude_band_m,
            "real_range_length_mm": r.length_mm,
            "sim_range_length_mm": s.length_mm,
            "n_real_pings": res.n_real, "n_sim_pings": res.n_sim,
            "geometry_comparable": matched,
            "rms_db": round(res.rms_db, 3) if matched else None,
            "p2p_db": round(res.p2p_db, 3) if matched else None,
            "above_reduction_floor": res.above_floor if matched else None,
        })
    scored = [r for r in rows if r["geometry_comparable"]]

    doc = {
        "svlog": str(Path(args.svlog).resolve()),
        "bundle": str(Path(args.bundle).resolve()),
        "band_m": [lo, hi],
        "reduction_floor_db": REDUCTION_FLOOR_DB,
        "site_floor_db": SITE_FLOOR_DB,
        "n_comparable_rows": len(scored),
        "real_normalisation_invariants": normalisation_invariants(real_profiles),
        "sim_normalisation_invariants": normalisation_invariants(sim_profiles),
        "real_curves": [
            {"side": c.side.value, "altitude_band_m": c.altitude_band_m,
             "range_length_mm": c.length_mm, "n_pings": c.n_pings}
            for c in sorted(real, key=_curve_key)],
        "residuals": rows,
    }

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "calibration.json").write_text(json.dumps(doc, indent=2) + "\n")

    md = [f"# Sim-to-real calibration — {Path(args.svlog).name}", "",
          f"Band {lo:.1f}–{hi:.1f} m · reduction floor "
          f"{REDUCTION_FLOOR_DB} dB RMS (smoke `[2e]`) · site floor "
          f"{SITE_FLOOR_DB} dB RMS (two real passes of the same water).", "",
          "Residuals are stated with the ping counts behind them; one below "
          "the reduction floor is the statistic's own noise, not agreement, "
          "and one below the site floor is as close as a flat-seabed model "
          "gets to survey data. Rows whose geometry does not overlap are not "
          "scored — the number would measure the geometry gap.", "",
          "| side | sim alt | real alt | n real | n sim | RMS dB | p-p dB | above floor |",
          "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if not r["geometry_comparable"]:
            md.append(f"| {r['side']} | {r['sim_altitude_band_m']:.0f} m | "
                      f"{r['real_altitude_band_m']:.0f} m | "
                      f"{r['n_real_pings']} | {r['n_sim_pings']} | "
                      "— | — | geometry not comparable |")
            continue
        md.append(f"| {r['side']} | {r['sim_altitude_band_m']:.0f} m | "
                  f"{r['real_altitude_band_m']:.0f} m | {r['n_real_pings']} | "
                  f"{r['n_sim_pings']} | {r['rms_db']:.2f} | "
                  f"{r['p2p_db']:.2f} | {'yes' if r['above_reduction_floor'] else 'no'} |")
    md += ["", "## Device normalisation invariants", "",
           "| stream | pings | one full-scale bin | zero minimum | span ≤ 90 dB |",
           "|---|---|---|---|---|"]
    for name, inv in (("real", doc["real_normalisation_invariants"]),
                      ("simulated", doc["sim_normalisation_invariants"])):
        if inv.get("n"):
            md.append(f"| {name} | {inv['n']} | "
                      f"{inv['single_full_scale_bin']:.4f} | "
                      f"{inv['zero_minimum']:.4f} | "
                      f"{inv['span_within_90db']:.4f} |")
    (out / "calibration.md").write_text("\n".join(md) + "\n")

    print(f"real: {len(real_profiles)} pings -> {len(real)} curves")
    print(f"sim:  {len(sim_profiles)} pings -> {len(sim)} curves")
    for r in rows:
        head = (f"  {r['side']:9s} sim alt {r['sim_altitude_band_m']:.0f} m vs "
                f"real {r['real_altitude_band_m']:.0f} m: ")
        if not r["geometry_comparable"]:
            print(head + "geometry not comparable, not scored (range "
                  f"{r['sim_range_length_mm']} vs "
                  f"{r['real_range_length_mm']} mm)")
            continue
        print(head + f"{r['rms_db']:.2f} dB RMS over {lo}-{hi} m "
              f"({r['n_real_pings']} real / {r['n_sim_pings']} sim pings)")
    if not scored:
        print("  no comparable geometry in this pairing: the bundle's altitude "
              "and range do not overlap the recording's. Compare a bundle "
              "generated at the recording's depth and range setting.")
    print(f"  wrote {out / 'calibration.json'}")
    print(f"         {out / 'calibration.md'}")


if __name__ == "__main__":
    main()
