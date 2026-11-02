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

from ..core.trajectory import load_trajectory
from ..core.types import Pose3D, Side
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
                      t0: float = 0.0, dt: float = 0.5,
                      trajectory: str | Path | None = None,
                      sonar_yaml: str | Path | None = None
                      ) -> list[ProfileRecord]:
    """Render the world and encode it into the same record type as a log.

    Going through :class:`PingEncoder` rather than reading the renderer's
    float power directly is the point: the comparison must see exactly the
    bytes a downstream consumer would, normalisation and all.
    ``trajectory``/``sonar_yaml`` name the flown path and the acquisition;
    a legacy mission bundle's own frozen copies are the fallback.
    """
    from .contacts import resolve_sonar_yaml

    cfg = SonarConfig.from_yaml(resolve_sonar_yaml(Path(bundle), sonar_yaml))
    return _simulate_with(Path(bundle), cfg, n_pings, seed, trajectory,
                          t0=t0, dt=dt)


def _curve_key(c: RangeCurve) -> tuple:
    return (c.side.value, c.altitude_band_m)


def nadir_profile(profiles: list[ProfileRecord], out_to_m: float = 2.0
                  ) -> dict | None:
    """Mean absolute dB over the first ``out_to_m`` of slant range.

    The near field is where the transmit-pulse ringing lives (measured on
    the corpus: ~55 dB at the transducer decaying to ~33 dB by 1 m), and
    it is the region the sim's ``ringing_level_db``/``ringing_decay_db_per_m``
    model — so real and simulated streams are reduced to the same
    summary: dB at bin 0, at 0.5 m and 1.0 m, and the distance at which
    the profile has decayed 10 dB below its own near-field peak.
    """
    curves, r_axis = [], None
    for p in profiles:
        if p.pwr_results is None or p.num_results == 0 or p.length_mm == 0:
            continue
        bin_m = p.length_mm / 1000.0 / max(p.num_results - 1, 1)
        n = int(out_to_m / bin_m) + 1
        if n < 4 or n > p.pwr_results.size:
            continue
        db = (p.min_pwr_db + p.pwr_results[:n].astype(np.float64) / 65535.0
              * (p.max_pwr_db - p.min_pwr_db))
        if r_axis is None:
            r_axis = np.arange(n) * bin_m
        m = min(n, db.size, r_axis.size)
        curves.append(db[:m])
    if not curves or r_axis is None:
        return None
    m = min(c.size for c in curves)
    mean = np.mean([c[:m] for c in curves], axis=0)
    r = r_axis[:m]

    def at(dist: float) -> float:
        return float(mean[min(int(dist / (r[1] - r[0])), m - 1)])

    peak = float(mean.max())
    below = np.nonzero(mean < peak - 10.0)[0]
    settle_m = float(r[below[0]]) if below.size else float(r[-1])
    return {"n_pings": len(curves),
            "db_at_transducer": round(float(mean[0]), 2),
            "db_at_0_5m": round(at(0.5), 2),
            "db_at_1m": round(at(1.0), 2),
            "settle_10db_m": round(settle_m, 2)}


def fit_beam_tilt(bundle: Path, real_curves: list[RangeCurve],
                  band: tuple[float, float], seed: int,
                  trajectory: str | Path | None,
                  sonar_yaml: str | Path | None,
                  tilts_deg=tuple(np.arange(5.0, 42.5, 2.5)),
                  n_pings: int = 60) -> list[dict]:
    """Constrain the (unpublished) bracket tilt from the corpus.

    The BlueBoat integration kit's mounting tilt is not documented, so
    the effective ``beam_tilt_deg`` is fit by re-rendering the bundle at
    a ladder of tilts and scoring each against the closest real curve —
    the tilt shapes the whole per-range response, so the residual over
    the scoring band is the natural objective.
    """
    from .contacts import resolve_sonar_yaml

    lo, hi = band
    rows = []
    cfg_path = resolve_sonar_yaml(Path(bundle), sonar_yaml)
    for tilt in tilts_deg:
        cfg = SonarConfig.from_yaml(cfg_path)
        cfg.model.beam_tilt_deg = float(tilt)
        profs = _simulate_with(bundle, cfg, n_pings, seed, trajectory)
        sim = reduce_profiles(profs, min_pings=10)
        best = None
        for s in sim:
            cands = [c for c in real_curves if c.side is s.side
                     and c.slant_r_m[-1] >= hi
                     and abs(c.altitude_band_m - s.altitude_band_m)
                     <= MAX_ALT_GAP_M]
            for c in cands:
                res = compare(c, s, lo, hi)
                if best is None or res.rms_db < best:
                    best = res.rms_db
        rows.append({"beam_tilt_deg": float(tilt),
                     "rms_db": round(best, 3) if best is not None else None})
    return rows


def _simulate_with(bundle: Path, cfg: SonarConfig, n_pings: int, seed: int,
                   trajectory: str | Path | None,
                   t0: float = 0.0, dt: float = 0.5) -> list[ProfileRecord]:
    """Render ``n_pings`` per side under an explicit SonarConfig."""
    from .contacts import resolve_trajectory

    scene = SceneModel.load(bundle)
    acq, model = cfg.acquisition, cfg.model
    traj = load_trajectory(resolve_trajectory(Path(bundle), trajectory))
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
            # The node's own noise call, `ambient` included: without it a
            # shadow's reverberation floor fell back to the unoccluded
            # reference and this report's shadows differed from a live run's.
            ping.power = apply_ping_noise(ping.power, bin_r < ping.altitude_m,
                                          1.0, model, rng,
                                          specular=ping.specular,
                                          texture=ping.texture,
                                          unoccluded=ping.unoccluded,
                                          ambient=ping.ambient,
                                          speckle_fill=ping.speckle_fill)
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


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--svlog", required=True, help="real recording (read-only)")
    ap.add_argument("--bundle", "--world", dest="bundle", required=True,
                    help="world folder or legacy mission bundle (read-only)")
    ap.add_argument("--trajectory", default=None,
                    help="trajectory YAML to fly (required for a world "
                         "folder; legacy bundles carry their own)")
    ap.add_argument("--sonar", default=None,
                    help="sonar profile YAML (default: the bundle's frozen "
                         "copy, else the package default)")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--sim-pings", type=int, default=200,
                    help="ping cycles to render per side (default: 200)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--altitude-band-m", type=float, default=1.0)
    ap.add_argument("--band", type=float, nargs=2, default=(6.0, 14.5),
                    metavar=("LO", "HI"),
                    help="slant-range band to score over, metres")
    ap.add_argument("--fit-tilt", action="store_true",
                    help="scan beam_tilt_deg over 5..40 deg and report the "
                         "residual per tilt (constrains the unpublished "
                         "BlueBoat bracket tilt from the corpus; slower)")
    args = ap.parse_args()
    from ..calibration.io_guard import refuse_inside
    # CM-7 / NC #10: neither the recording's tree nor the bundle is written to.
    refuse_inside(args.out, [args.svlog], roots=[args.bundle])

    real_profiles = read_profiles(args.svlog, with_power=True)
    sim_profiles = simulate_profiles(Path(args.bundle), args.sim_pings,
                                     args.seed, trajectory=args.trajectory,
                                     sonar_yaml=args.sonar)

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

    # Near-field / nadir column: the transmit-ringing region the sim's
    # ringing model reproduces (fix for the nadir-gap realism gap).
    nadir_real = nadir_profile(real_profiles)
    nadir_sim = nadir_profile(sim_profiles)

    tilt_rows = []
    if args.fit_tilt:
        tilt_rows = fit_beam_tilt(Path(args.bundle), real, (lo, hi),
                                  args.seed, args.trajectory, args.sonar)

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
        "nadir_profile": {"real": nadir_real, "sim": nadir_sim},
        "beam_tilt_fit": tilt_rows,
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
    md += ["", "## Near-field / nadir column (first 2 m of slant range)", "",
           "The transmit-ringing region: the corpus measures ~55 dB at the "
           "transducer decaying to ~33 dB by 1 m, which the sim reproduces "
           "with `ringing_level_db`/`ringing_decay_db_per_m`.", "",
           "| stream | pings | dB @ 0 | dB @ 0.5 m | dB @ 1 m | 10 dB settle |",
           "|---|---|---|---|---|---|"]
    for name, prof in (("real", nadir_real), ("simulated", nadir_sim)):
        if prof:
            md.append(f"| {name} | {prof['n_pings']} | "
                      f"{prof['db_at_transducer']:.1f} | "
                      f"{prof['db_at_0_5m']:.1f} | {prof['db_at_1m']:.1f} | "
                      f"{prof['settle_10db_m']:.2f} m |")
    if tilt_rows:
        md += ["", "## Beam-tilt fit (unpublished bracket tilt, "
               "constrained from the corpus)", "",
               "| beam_tilt_deg | best RMS dB |", "|---|---|"]
        for r in tilt_rows:
            md.append(f"| {r['beam_tilt_deg']:.1f} | "
                      + (f"{r['rms_db']:.2f} |" if r["rms_db"] is not None
                         else "— |"))
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
    for name, prof in (("real", nadir_real), ("sim", nadir_sim)):
        if prof:
            print(f"  nadir {name}: {prof['db_at_transducer']:.1f} dB at the "
                  f"transducer, {prof['db_at_1m']:.1f} dB at 1 m, 10 dB "
                  f"settle {prof['settle_10db_m']:.2f} m")
    fitted = [r for r in tilt_rows if r["rms_db"] is not None]
    if fitted:
        best = min(fitted, key=lambda r: r["rms_db"])
        print(f"  beam-tilt fit: best {best['beam_tilt_deg']:.1f} deg at "
              f"{best['rms_db']:.2f} dB RMS "
              f"({len(fitted)} tilts scored)")
    print(f"  wrote {out / 'calibration.json'}")
    print(f"         {out / 'calibration.md'}")


if __name__ == "__main__":
    main()
