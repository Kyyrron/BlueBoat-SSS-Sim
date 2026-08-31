"""Offline end-to-end smoke test (no ROS required).

Exercises the full ROS-free pipeline exactly as the ROS nodes drive it:

    mission YAML -> world bundle -> lawnmower trajectory -> per-ping
    render -> noise -> byte-exact encode -> decode round-trip ->
    waterfall tiles -> auto labels -> YOLO dataset export

Run with:  python3 -m test.smoke_test   (from the package root)
Artifacts land in /tmp/blueboat_sss_smoke for visual inspection.
"""

from __future__ import annotations

import os
import shutil
import sys
from collections import namedtuple
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from blueboat_sss_sim.analysis import (compute_metrics,  # noqa: E402
                                       from_jsonl, from_replay,
                                       from_svlog, write_report)
from blueboat_sss_sim.analysis import svlog_reader  # noqa: E402
from blueboat_sss_sim.analysis.calibration import (fbr_bin,  # noqa: E402
                                                   normalisation_invariants,
                                                   scale_to_db)
from blueboat_sss_sim.analysis.report import (content_digest,  # noqa: E402
                                              to_dict)
from blueboat_sss_sim.core.types import (GridSpec, GroundTruthContact,  # noqa: E402
                                         PlacedObject, Pose3D, Side, Wall)
from blueboat_sss_sim.dataset.exporter import ExportConfig, YoloDatasetWriter  # noqa: E402
from blueboat_sss_sim.dataset.labeler import LabelConfig, TileLabeler  # noqa: E402
from blueboat_sss_sim.dataset.waterfall import (WaterfallBuilder,  # noqa: E402
                                            WaterfallTileConfig)
from blueboat_sss_sim.mission.generate import generate_mission  # noqa: E402
from blueboat_sss_sim.mission.patterns import WaypointTrajectory  # noqa: E402
from blueboat_sss_sim.sonar.config import SonarConfig  # noqa: E402
from blueboat_sss_sim.sonar.acoustics import net_range_response  # noqa: E402
from blueboat_sss_sim.sonar.config import MAX_GAIN_INDEX  # noqa: E402
from blueboat_sss_sim.sonar.encoder import (ANALOG_GAIN_TABLE,  # noqa: E402
                                            PingEncoder, gain_step_db,
                                            parse_frame)
from blueboat_sss_sim.sonar.noise import GainDrift, apply_ping_noise  # noqa: E402
from blueboat_sss_sim.sonar.multipath import (crossing_mask,  # noqa: E402
                                              mirror_sources)
from blueboat_sss_sim.sonar.renderer import GeometricRenderer  # noqa: E402
from blueboat_sss_sim.worldgen.scene import SceneModel  # noqa: E402

OUT = Path("/tmp/blueboat_sss_smoke")

#: Minimal stand-in for analysis.svlog_reader.ProfileRecord, so the corpus's
#: normalisation invariants can be asserted on freshly encoded pings without
#: routing them through a file.
_PowerRec = namedtuple("_PowerRec", "pwr_results min_pwr_db max_pwr_db")


def check(name: str, cond: bool, detail: str = "") -> None:
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {name}" + (f" -- {detail}" if detail else ""))
    if not cond:
        raise SystemExit(f"smoke test failed at: {name}")


def main() -> None:
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)

    # ---- 1. Mission bundle -------------------------------------------------
    print("[1] mission bundle generation")
    bundle = generate_mission(PKG_ROOT / "config" / "default_mission.yaml",
                              OUT / "mission", seed=7)
    for f in ("world.sdf", "seabed.stl", "scene.npz", "scene_manifest.yaml",
              "trajectory.yaml", "sonar.yaml", "mission_snapshot.yaml"):
        check(f"bundle contains {f}", (bundle / f).exists())

    scene = SceneModel.load(bundle)
    check("scene has objects", len(scene.objects) > 0,
          f"{len(scene.objects)} objects")
    traj_check = WaypointTrajectory.load_yaml(bundle / "trajectory.yaml")
    x0, y0, _ = traj_check.pose_at(0.0)
    check("path starts at robot spawn (0,0)", abs(x0) < 0.01 and abs(y0) < 0.01,
          f"first pose ({x0:.2f}, {y0:.2f})")
    sdf = (bundle / "world.sdf").read_text()
    check("world.sdf has physics plugin", "physics" in sdf)
    check("world.sdf has buoyancy plugin", "buoyancy" in sdf)
    check("world.sdf references seabed mesh", "seabed.stl" in sdf)

    # ---- 2. Sonar simulation along the trajectory ----------------------------
    print("[2] sonar rendering along lawnmower pass")
    cfg = SonarConfig.from_yaml(bundle / "sonar.yaml")
    acq, model = cfg.acquisition, cfg.model
    traj = WaypointTrajectory.load_yaml(bundle / "trajectory.yaml")
    renderer = GeometricRenderer(scene, acq, model)
    rng = np.random.default_rng(1)

    builders = {s: WaterfallBuilder(acq.num_results,
                                    WaterfallTileConfig(tile_pings=400,
                                                        overlap_pings=50))
                for s in Side}
    labelers = {s: TileLabeler(acq.num_results, acq.bin_size_m,
                               LabelConfig()) for s in Side}
    encoders = {s: PingEncoder(s, acq, model) for s in Side}
    drifts = {s: GainDrift(model, rng) for s in Side}

    period = acq.ping_period_s(model.max_ping_rate_hz)
    n_pings = min(int(traj.duration / period), 2600)
    contact_pings = 0
    roundtrip_checked = False
    channels_seen: dict = {s: set() for s in Side}   # side -> byte-34 values
    altitudes = []
    # Probes carry what a downstream consumer actually sees: the dB curve the
    # device formula recovers from the normalised counts. `pwr_results` itself
    # is not linear power any more, so a check that treats it as power measures
    # nothing. The linear field is kept alongside for the speckle statistics,
    # which live there (NC #5).
    fbr_probe: list[np.ndarray] = []   # per-ping dB curve (port)
    raw_probe: list[tuple] = []        # (counts, min_pwr_db, max_pwr_db)
    fbr_power: list[np.ndarray] = []   # per-ping noisy linear power (port)
    fbr_maxdb: list[float] = []        # per-ping reported max_pwr_db
    fbr_alts: list[float] = []

    for k in range(n_pings):
        t = k * period
        x, y, yaw = traj.pose_at(t)
        # small synthetic surface motion so attitude coupling is exercised
        pose = Pose3D(x, y, 0.0,
                      roll=np.radians(2.0) * np.sin(2 * np.pi * t / 3.1),
                      pitch=np.radians(1.0) * np.sin(2 * np.pi * t / 4.7),
                      yaw=yaw)
        for side in Side:
            r = renderer.render(side, pose, t)
            altitudes.append(r.ping.altitude_m)
            wc_mask = (acq.range_start_mm / 1000.0
                       + (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
                       ) < r.ping.altitude_m
            r.ping.power = apply_ping_noise(r.ping.power, wc_mask,
                                            drifts[side].value(t), model, rng,
                                            specular=r.ping.specular)
            enc = encoders[side].encode(r.ping)
            channels_seen[side].add(enc.raw_frame[34])

            if not roundtrip_checked:
                # parse_frame raises on bad magic/id/checksum, so reaching
                # the next line proves the frame is byte-valid Ping Protocol.
                dec = parse_frame(enc.raw_frame)
                check("raw frame magic/id/checksum valid", dec is not None)
                corrupted = bytearray(enc.raw_frame)
                corrupted[10] ^= 0xFF
                try:
                    parse_frame(bytes(corrupted))
                    check("checksum detects corruption", False)
                except ValueError:
                    check("checksum detects corruption", True)
                check("num_results round-trip",
                      dec["num_results"] == acq.num_results)
                check("pwr_results round-trip",
                      np.array_equal(dec["pwr_results"], enc.pwr_results))
                check("frame length matches real capture layout",
                      len(enc.raw_frame) == 8 + 52 + 2 * acq.num_results + 2,
                      f"{len(enc.raw_frame)} bytes")
                roundtrip_checked = True

            if side is Side.PORT and len(fbr_probe) < 200:
                fbr_probe.append(scale_to_db(enc.pwr_results,
                                             enc.min_pwr_db, enc.max_pwr_db))
                raw_probe.append((np.asarray(enc.pwr_results),
                                  enc.min_pwr_db, enc.max_pwr_db))
                fbr_power.append(np.asarray(r.ping.power, dtype=np.float64))
                fbr_maxdb.append(enc.max_pwr_db)
                fbr_alts.append(r.ping.altitude_m)

            if r.contacts:
                contact_pings += 1
            builders[side].add_ping(enc.pwr_results, r.contacts,
                                    min_pwr_db=enc.min_pwr_db,
                                    max_pwr_db=enc.max_pwr_db)

    alt = np.array(altitudes)
    check("altitude in shallow regime", bool((alt > 1.0).all() and
                                             (alt < 8.0).all()),
          f"min {alt.min():.2f} m, max {alt.max():.2f} m")
    check("contacts observed on the pass", contact_pings > 0,
          f"{contact_pings} pings with ground-truth contacts")
    # Side identity lives in the packet (byte 34), never in the topic or the
    # `src` device tag: the .svlog writer derives the device id from it and
    # the reader groups port/starboard on it, so both sides carrying the same
    # value silently loses one side of every ping on replay.
    check("channel_number is 0 on port, 1 on starboard (every ping)",
          channels_seen[Side.PORT] == {0} and channels_seen[Side.STARBOARD] == {1},
          f"port {sorted(channels_seen[Side.PORT])}, "
          f"starboard {sorted(channels_seen[Side.STARBOARD])}")

    # ---- 2b. FBR structure (what downstream bottom-tracking locks onto) ----
    print("[2b] first-bottom-return structure")
    mean_p = np.mean(fbr_probe, axis=0)          # mean dB curve
    mean_pw = np.mean(fbr_power, axis=0)         # mean linear power
    mean_alt = float(np.mean(fbr_alts))
    fbr_idx = int(mean_alt / acq.bin_size_m)
    wc = mean_p[:max(fbr_idx - 8, 1)]
    peak = float(mean_p[:fbr_idx + 40].max())
    # Already dB, so the contrast is a subtraction. The field corpus puts the
    # water column 22-31 dB below the bottom return across three range
    # settings and 14 altitude bands; the fitted floor targets 26.
    contrast_db = peak - float(wc.mean())
    check("water column is quiet vs FBR peak (corpus: 22-31 dB)",
          16.0 < contrast_db < 36.0,
          f"{contrast_db:.1f} dB gap->peak contrast")
    lock = int(np.argmax(mean_p > float(np.median(wc)) + 8.0))
    check("naive FBR bootstrap locks at altitude",
          abs(lock * acq.bin_size_m - mean_alt) < 0.25,
          f"lock bin {lock} = {lock*acq.bin_size_m:.2f} m, "
          f"altitude {mean_alt:.2f} m")

    # ---- 2c. Omniscan device fidelity ------------------------------------
    print("[2c] device fidelity (rate cap, power scale, speckle PDF)")
    check("ping period respects max_ping_rate_hz",
          period >= 0.999 / max(model.max_ping_rate_hz, 1e-9)
          if model.max_ping_rate_hz > 0 else True,
          f"{period*1000:.0f} ms (cap {model.max_ping_rate_hz} Hz)")
    check("uncapped free-run matches captured 22 ms @ 15 m",
          abs(acq.ping_period_s(0.0) - 0.022) < 0.002,
          f"{acq.ping_period_s(0.0)*1000:.0f} ms")
    # The device normalisation invariants, asserted on simulated pings: the
    # field corpus satisfies all three on 68948/68948 pings, so a mapping that
    # emits absolute counts fails here rather than silently reaching the
    # processor as a linear-power vector wearing a dB label.
    inv = normalisation_invariants([_PowerRec(c, lo, hi)
                                    for c, lo, hi in raw_probe])
    check("pwr_results carries the device's per-ping normalisation",
          inv["single_full_scale_bin"] == 1.0 and inv["zero_minimum"] == 1.0
          and inv["span_within_90db"] == 1.0,
          f"{inv['n']} pings: one full-scale bin, zero minimum, span <= "
          f"{model.max_span_db:g} dB -- all 1.0 on the field corpus too")
    # The anchor is max_pwr_db itself, now that it is a physical level and not
    # a function of the u16 clip -- under the old encoding it was pinned at
    # 10log10(65535)+offset whenever any bin clipped, so the gate mostly
    # asserted that counts reached the clip. Tolerance sized from the
    # quantity's own noise as before: the mean varies by 0.064 dB sd
    # (0.22 p-p) across speckle seeds, so +-0.5 dB leaves ~8x margin while
    # still catching a moved anchor (NC #6). Never widen it to make a fit
    # pass. `calibration_db_offset` itself is checked against the corpus at
    # matched geometry in [2f].
    check("max_pwr_db sits on the measured level anchor (67.85 +- 0.5 dB)",
          abs(float(np.mean(fbr_maxdb)) - 67.85) <= 0.5,
          f"mean {np.mean(fbr_maxdb):.2f} dB at {mean_alt:.1f} m altitude "
          f"(calibration_db_offset {model.calibration_db_offset:g} dB)")
    # Speckle lives in the LINEAR power field. The encoded array is a dB axis,
    # where Exp(1) does not have CV 1 -- checking it there would silently
    # relax the statistic NC #5 makes load-bearing.
    one = fbr_power[0][fbr_idx + 30:fbr_idx + 120]
    cv = float(one.std() / max(one.mean(), 1e-9))
    check("speckle PDF on flat seabed (CV ~ 1 for Exp(1), linear power)",
          0.6 < cv < 1.4, f"CV {cv:.2f}")
    # Emptiness is a property of the power field too: a mid-scale count is not
    # evidence of a populated bin once the axis is dB.
    check("no empty far-range bins (sampling tracks bin size)",
          float((mean_pw[400:590] <= 0.0).mean()) < 0.02,
          f"{(mean_pw[400:590] <= 0.0).mean():.1%} empty")

    # Downstream bottom-tracking lockability: emulate the fleet's FBR
    # detector (noise floor from first 20 samples, +8 dB threshold,
    # persistence 3) on consecutive moving pings; the tracker bootstrap
    # needs 10 consecutive estimates within a 0.30 m band.
    def _fbr_est(db: np.ndarray, bin_m: float | None = None) -> float | None:
        """Downstream FBR detector, on the dB curve the processor reconstructs.

        `analysis.calibration.fbr_bin` re-expresses sss_processor_node's rule
        including its ringing-settle step, so this is the fleet detector rather
        than an approximation of it. `bin_m` defaults to the default profile's
        bin; pass it explicitly for a different range setting."""
        step = acq.bin_size_m if bin_m is None else bin_m
        i = fbr_bin(np.asarray(db, dtype=np.float64))
        return None if i is None else i * step

    ests = [_fbr_est(p) for p in fbr_probe]
    lockable = 0
    windows = max(len(ests) - 10, 1)
    for i in range(len(ests) - 10):
        w = ests[i:i + 10]
        if all(x is not None for x in w) and (max(w) - min(w)) <= 0.30:
            lockable += 1
    check("downstream FBR tracker can lock while moving",
          lockable / windows > 0.8,
          f"{lockable/windows:.0%} of 10-ping windows lockable "
          f"(spread <= 0.30 m, no misses)")

    # ---- 2d. azimuth beam + high-res bins ----------------------------------
    print("[2d] along-track 0.5 deg beam + 1/1200 bins")
    import dataclasses

    class _Bump:
        class G:
            nx = ny = 100
            resolution = 0.10
        grid = G()
        objects = []

        def sample_height(self, x, y):
            return (np.full_like(np.asarray(x, float), -2.0)
                    + 0.25 * (np.hypot(np.asarray(x),
                                       np.asarray(y) - 13.0) < 0.15))

        def sample_reflectivity(self, x, y):
            return (np.full_like(np.asarray(x, float), 0.55)
                    + 0.35 * (np.hypot(np.asarray(x),
                                       np.asarray(y) - 13.0) < 0.15))

    def _extent(n_lines: int) -> float:
        c = dataclasses.replace(model, alongtrack_beam_lines=n_lines)
        rr = GeometricRenderer(_Bump(), acq, c)
        b = int(np.hypot(12.8, 1.60) / acq.bin_size_m)
        xs = np.arange(-0.5, 0.5, 0.02)

        def _tot(x: float) -> float:
            pg = rr.render(Side.PORT, Pose3D(x, 0, 0, 0, 0, 0), 0.0).ping
            comb = pg.power + (pg.specular if pg.specular is not None else 0.0)
            return float(comb[b - 5:b + 5].max())

        resp = np.array([_tot(float(x)) for x in xs])
        # Background from the scan edges only -- the widened K=5 response
        # can cover more than half the scan, which would contaminate a
        # global median.
        bg = float(np.median(np.concatenate([resp[:6], resp[-6:]])))
        return float((resp > 3.0 * bg).sum() * 0.02)

    e1, e5 = _extent(1), _extent(5)
    check("along-track response widens with the azimuth beam",
          e5 > e1 + 0.05,
          f"extent {e1*100:.0f} cm (K=1) -> {e5*100:.0f} cm (K=5) at 13 m")

    acq12 = dataclasses.replace(acq, num_results=1200)
    r12 = GeometricRenderer(scene, acq12, model)
    enc12 = PingEncoder(Side.PORT, acq12, model)
    rp12 = r12.render(Side.PORT, Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0)
    e12 = enc12.encode(rp12.ping)
    check("1/1200-range bins render + frame correctly",
          len(e12.raw_frame) == 8 + 52 + 2 * 1200 + 2
          and e12.raw_frame[34] == Side.PORT.channel
          and float((rp12.ping.power[900:1180] == 0).mean()) < 0.05,
          f"frame {len(e12.raw_frame)} B, byte34={e12.raw_frame[34]}, "
          f"far empty {(rp12.ping.power[900:1180]==0).mean():.1%}")

    # ---- 2e. calibration identifiability ---------------------------------------
    # What a sim-to-real calibration fit can and cannot determine, and from
    # which statistic. These are properties of the model's parameterisation
    # rather than of a particular tuning, and the capture-day fit plan
    # (.claude/specs/sim-to-real-calibration-WITH_SVLOG.md) rests on them: if
    # sonar/acoustics.py or the encoder ever changes the range law or the
    # count mapping, these fail and the capture plan is revisited, instead of
    # the fit silently returning a confident meaningless answer.
    print("[2e] calibration identifiability (what one capture can fit)")

    bin_r = (acq.range_start_mm / 1000.0
             + (np.arange(acq.num_results) + 0.5) * acq.bin_size_m)
    far = (bin_r > 6.0) & (bin_r < 14.5)   # past the FBR, inside the swath

    def _pings(m, n, seed=None):
        """Render `n` pings along the mission trajectory under model config `m`.

        ``seed=None`` leaves the noiseless field, whose ensemble mean is the
        per-range statistic a fit compares; an int applies the shipped speckle
        so the reduction's own repeatability can be measured.
        """
        rr = GeometricRenderer(scene, acq, m)
        rg = np.random.default_rng(seed) if seed is not None else None
        out = []
        for k in range(n):
            t = k * period
            px, py, pyaw = traj.pose_at(t)
            pg = rr.render(Side.PORT,
                           Pose3D(px, py, 0.0, 0.0, 0.0, pyaw), t).ping
            if rg is not None:
                pg.power = apply_ping_noise(pg.power, bin_r < pg.altitude_m,
                                            1.0, m, rg, specular=pg.specular)
            out.append(pg)
        return out

    def _mean_db(pings):
        return 10 * np.log10(np.maximum(
            np.mean([p.power for p in pings], axis=0), 1e-12))

    # (a) The level lives in min/max_pwr_db, never in the counts. The device
    #     rescales every ping to full scale on a dB axis, so the array spans
    #     0..65535 whatever the level is -- measured on 68948/68948 field
    #     pings. Consequence for the fit, and the reason the runbook's
    #     original count-span plan does not work: a count histogram carries no
    #     level information at all, and `calibration_db_offset` is the sole
    #     level constant, determined from the reported max_pwr_db.
    shared = _pings(model, 60, seed=11)   # same field, encoded two ways
    m_louder = dataclasses.replace(model,
                                   calibration_db_offset=
                                   model.calibration_db_offset + 20.0)

    def _encoded(m):
        enc = PingEncoder(Side.PORT, acq, m)
        out = [enc.encode(p) for p in shared]
        return (np.array([np.asarray(e.pwr_results, dtype=np.float64)
                          for e in out]),
                float(np.mean([e.max_pwr_db for e in out])))

    c_a, db_a = _encoded(model)
    c_b, db_b = _encoded(m_louder)
    check("level rides in max_pwr_db, not in the counts",
          np.array_equal(c_a, c_b) and abs((db_b - db_a) - 20.0) < 1e-6,
          f"a 20 dB level change leaves every count identical and moves "
          f"max_pwr_db by {db_b - db_a:+.2f} dB -- so a count histogram "
          f"cannot fit the level, and a dB one can")

    # (b) tvg_compensation and lambert_exponent both enter the far-field curve
    #     as a pure log10(r) slope, so on a flat seabed at one altitude they
    #     trade off against each other almost exactly. A fit over such a
    #     capture returns one point on a ridge, not a determination. The
    #     capture must span >= 2 altitudes (grazing angle at a given slant
    #     range moves the Lambert term but not the TVG term), or one of the
    #     two must be fixed independently.
    # The washing pair is a property of the range law, so it moved when the
    # law was fitted: at spreading 4.0 / tvg 0.0 it is tvg 0.20 vs lambert
    # 0.10, not the tvg 0.80 vs lambert 1.945 the old law washed at.
    c_tvg = _mean_db(_pings(dataclasses.replace(model,
                                                tvg_compensation=0.20), 150))
    c_lam = _mean_db(_pings(dataclasses.replace(model,
                                                lambert_exponent=0.10), 150))
    c_off = _mean_db(_pings(dataclasses.replace(model,
                                                lambert_exponent=2.6), 150))

    def _p2p(a, b):
        """Peak-to-peak disagreement of two per-range curves, ignoring a
        constant offset -- that offset is absorbed by base_scale and so
        carries no information about the range law."""
        d = (a - b)[far]
        return float((d - d.mean()).max() - (d - d.mean()).min())

    p_match, p_off = _p2p(c_tvg, c_lam), _p2p(c_tvg, c_off)
    # 0.4 dB is below the reduction's own 0.67 dB repeatability floor at
    # 80 pings, so a capture at one altitude cannot tell the pair apart even
    # in principle.
    check("tvg_compensation/lambert_exponent confounded at one altitude",
          p_match <= 0.4 and p_off > 1.0,
          f"tvg 0.20 vs lambert 0.10: {p_match:.3f} dB p-p over 6.0-14.5 m "
          f"(under the 0.67 dB reduction floor); vs a mismatched lambert 2.6: "
          f"{p_off:.2f} dB p-p (clearly separable)")

    # (c) The per-range mean-dB curve is the statistic the fit will compare
    #     against real data, so its own repeatability is the floor below which
    #     a sim-vs-real difference is not measurable. Stated here so capture
    #     day can size the ping count instead of guessing a tolerance.
    rep = (_mean_db(_pings(model, 80, seed=21))
           - _mean_db(_pings(model, 80, seed=22)))[far]
    rep_rms = float(np.sqrt((rep ** 2).mean()))
    # Two-sided on purpose: an exploding floor means the reduction is unusable
    # as a fit statistic, and a vanishing one means the speckle that NC #5
    # makes load-bearing has stopped being applied.
    check("per-range reduction repeatable across speckle seeds",
          0.2 < rep_rms < 1.0,
          f"{rep_rms:.3f} dB RMS over 6.0-14.5 m at 80 pings -- the floor "
          f"below which a sim-vs-real difference is not measurable")

    # ---- 2f. against the real field corpus --------------------------------------
    # The constants above are fitted, not assumed, so the corpus they were
    # fitted against is a gate. Skipped when the recordings are not on this
    # machine, so a fresh clone still reaches ALL CHECKS PASSED -- point
    # $BLUEBOAT_SSS_CORPUS at a directory of .svlog files to enable it.
    # Recordings are primary field data: opened read-only, never rewritten
    # (CM-7).
    print("[2f] against the real field corpus")
    corpus = Path(os.environ.get("BLUEBOAT_SSS_CORPUS",
                                 Path.home() / "ros2_ws/data/SSS_data"))
    corpus_files = sorted(corpus.glob("*.svlog")) if corpus.is_dir() else []
    if not corpus_files:
        print(f"  SKIP: no .svlog corpus at {corpus} "
              "(set $BLUEBOAT_SSS_CORPUS to enable)")
    else:
        # Bounded read: enough pings to make the fractions meaningful, few
        # enough to keep this section inside the suite's per-turn budget.
        # Spread across recordings rather than draining the first: the device's
        # auto-gain settles differently per run, so a sample from one file
        # exercises one gain index and the ladder check would pass vacuously.
        real: list = []
        per_file = max(2000 // len(corpus_files), 1)
        for f in corpus_files:
            taken = 0
            for pkt in svlog_reader.iter_packets(f):
                if pkt.packet_id != 2198:
                    continue
                real.append(parse_frame(pkt.raw))
                taken += 1
                if taken >= per_file:
                    break
        inv_r = normalisation_invariants(
            [_PowerRec(d["pwr_results"], d["min_pwr_db"], d["max_pwr_db"])
             for d in real])
        check("the device normalises every ping onto its own dB axis",
              inv_r["single_full_scale_bin"] == 1.0
              and inv_r["zero_minimum"] == 1.0
              and inv_r["span_within_90db"] == 1.0,
              f"{inv_r['n']} field pings from {len(corpus_files)} recordings: "
              "exactly one bin at 65535, minimum 0, span within "
              f"{model.max_span_db:g} dB -- the encoder's contract, verified "
              "against the hardware")
        # The ladder is the device's own reported analog_gain. A table that
        # disagreed would misreport the gain on every simulated ping.
        seen = {}
        for d in real:
            seen.setdefault(int(d["gain_index"]), set()).add(
                round(float(d["analog_gain"]), 3))
        ladder_match = all(len(v) == 1 and g in ANALOG_GAIN_TABLE
                           and abs(ANALOG_GAIN_TABLE[g] - next(iter(v))) < 1e-3
                           for g, v in seen.items())
        check("ANALOG_GAIN_TABLE matches what the device reports",
              ladder_match and set(seen) <= set(ANALOG_GAIN_TABLE),
              ", ".join(f"{g}:{next(iter(v)):g}" for g, v in sorted(seen.items())))
        # The quantity the noise floor was fitted to.
        wc_gaps = []
        for d in real:
            db = scale_to_db(d["pwr_results"], d["min_pwr_db"], d["max_pwr_db"])
            i = fbr_bin(db)
            if i is None or i < 30:
                continue
            wc_gaps.append(float(db[:i + 40].max())
                           - float(db[10:max(i - 8, 11)].mean()))
        wc_med = float(np.median(wc_gaps))
        check("water column sits where the fitted noise floor puts it",
              16.0 < wc_med < 36.0,
              f"{wc_med:.1f} dB below the bottom return over {len(wc_gaps)} "
              "field pings; the simulator's fitted floor targets 26")
        # The range law. The shipped tvg_compensation of 0.90 left a ~2 dB per
        # decade residual; the corpus falls an order of magnitude faster, which
        # is what moved spreading_exponent to the two-way value with no TVG.
        r_probe = np.array([5.0, 50.0])
        net = net_range_response(r_probe, model)
        model_slope = float(10 * np.log10(net[0] / net[1]))
        check("the model's range law falls as steeply as the corpus does",
              30.0 < model_slope < 55.0,
              f"{model_slope:.1f} dB per decade of slant range "
              f"(spreading {model.spreading_exponent:g}, tvg "
              f"{model.tvg_compensation:g}); the corpus falls 40-52 dB/decade "
              "and the old 0.90 TVG gave 2")

    # ---- 3. Waterfall + labels + export ----------------------------------------
    print("[3] waterfall tiles, labels, YOLO export")
    writer = YoloDatasetWriter(OUT / "dataset", ExportConfig(),
                               WaterfallTileConfig())
    n_boxes = 0
    tile_shapes = []
    for side, b in builders.items():
        for i, (img, rows) in enumerate(b.ready_tiles(flush=True)):
            boxes = labelers[side].label_tile(rows)
            n_boxes += len(boxes)
            tile_shapes.append(img.shape)
            writer.add_tile(img, boxes, f"smoke_{side.value}_{i:04d}")
    check("tiles produced", writer.tile_count > 0,
          f"{writer.tile_count} tiles {tile_shapes[0]}")
    check("YOLO boxes produced", n_boxes > 0, f"{n_boxes} boxes")
    classes = []
    for lab in labelers.values():
        for n in lab.class_names:
            if n not in classes:
                classes.append(n)
    yaml_path = writer.finalize(classes)
    check("dataset.yaml written", yaml_path.exists(),
          f"classes: {classes}")

    # normalized box sanity
    from blueboat_sss_sim.dataset.labeler import YoloBox  # noqa: F401
    bad = 0
    for lbl in (OUT / "dataset" / "labels").rglob("*.txt"):
        for line in lbl.read_text().splitlines():
            vals = [float(v) for v in line.split()[1:]]
            if not all(0.0 <= v <= 1.0 for v in vals):
                bad += 1
    check("all YOLO coords normalized", bad == 0)

    # ---- 4. Shallow-regime world ------------------------------------------------
    # `config/shallow_water_world.yaml` ships alongside the 4 m default at
    # 2.5 m mean depth. It differs from the default in `base_depth` alone, and
    # base_depth consumes no RNG draws, so the same seed places the *same*
    # objects at the *same* poses -- the two bundles are directly comparable
    # and shadow extents can be paired by object_id.
    print("[4] shallow-regime world config")
    shallow = generate_mission(PKG_ROOT / "config" / "shallow_water_mission.yaml",
                               OUT / "mission_shallow", seed=7)
    scene_sh = SceneModel.load(shallow)
    depth_sh = -scene_sh.height
    check("shallow bundle sits in the 1-3 m thesis regime",
          1.0 < float(depth_sh.min()) and float(depth_sh.max()) < 3.5,
          f"{depth_sh.min():.2f}-{depth_sh.max():.2f} m, "
          f"mean {depth_sh.mean():.2f} m")

    def _sweep(sc, t0: float = 60.0, n: int = 800):
        """Fly the shared trajectory over `sc`; collect altitudes, port probe
        pings and per-(object, side) ground-truth shadow extents.

        `t0` skips the transit leg so the window lands on the survey legs,
        where enough objects are ensonified on both bundles to pair."""
        rend = GeometricRenderer(sc, acq, model)
        enc_p = PingEncoder(Side.PORT, acq, model)
        rg = np.random.default_rng(1)
        alts: list[float] = []
        probe: list[np.ndarray] = []
        probe_alt: list[float] = []
        shad: dict = {}
        for i in range(n):
            ts = t0 + i * period
            px, py, pyaw = traj.pose_at(ts)
            p = Pose3D(px, py, 0.0, yaw=pyaw)
            for sd in Side:
                rr = rend.render(sd, p, ts)
                alts.append(rr.ping.altitude_m)
                for c in rr.contacts:
                    if c.visible:
                        shad.setdefault((c.object_id, sd), []).append(c.shadow_bins)
                if sd is Side.PORT and len(probe) < 200:
                    # The dB curve a downstream tracker reconstructs, as [2b]
                    # uses -- pwr_results itself is a normalised dB axis, so
                    # _fbr_est works on the decode, not on the raw array.
                    m = (acq.range_start_mm / 1000.0
                         + (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
                         ) < rr.ping.altitude_m
                    rr.ping.power = apply_ping_noise(rr.ping.power, m, 1.0,
                                                     model, rg,
                                                     specular=rr.ping.specular)
                    e = enc_p.encode(rr.ping)
                    probe.append(scale_to_db(e.pwr_results, e.min_pwr_db,
                                             e.max_pwr_db))
                    probe_alt.append(rr.ping.altitude_m)
        return np.array(alts), np.array(probe), np.array(probe_alt), shad

    alt_sh, probe_sh, palt_sh, shad_sh = _sweep(scene_sh)
    alt_4m, _, _, shad_4m = _sweep(scene)

    # NC #4: the sidelobe floor, near-nadir specular lobe, separately-speckled
    # specular component and pulse smearing were all tuned at ~4 m altitude.
    # Bottom tracking must still bootstrap 1.5 m shallower. Run the same
    # downstream detector emulation as [2b] per ping, against that ping's own
    # altitude -- averaging pings taken across a sloped leg smears the peak
    # and would measure the slope, not the return.
    ests = [_fbr_est(p) for p in probe_sh]
    misses = sum(e is None for e in ests)
    err = np.array([abs(e - a) for e, a in zip(ests, palt_sh) if e is not None])
    cons = []
    for p, a in zip(probe_sh, palt_sh):
        fb = int(a / acq.bin_size_m)
        # Already dB: contrast is a subtraction, not a log ratio.
        cons.append(float(p[:fb + 40].max())
                    - float(p[:max(fb - 8, 1)].mean()))
    contrast_sh = float(np.median(cons))
    check("FBR still locks at the shallow altitude (NC #4)",
          misses == 0 and float(np.median(err)) < 0.25 and contrast_sh > 8.0,
          f"{len(ests)} pings, {misses} misses, median error "
          f"{float(np.median(err))*100:.0f} cm at {palt_sh.mean():.2f} m "
          f"altitude, {contrast_sh:.1f} dB contrast")

    # The point of the shallow config: shadow length goes as h*R/altitude, so
    # the same object casts a longer shadow from lower down.
    ratios = []
    for key in set(shad_sh) & set(shad_4m):
        d4 = float(np.median(shad_4m[key]))
        if d4 > 0:
            ratios.append(float(np.median(shad_sh[key])) / d4)
    check("shadows lengthen at the shallower altitude",
          len(ratios) >= 5 and float(np.median(ratios)) > 1.3
          and min(ratios) > 1.0,
          f"{len(ratios)} objects paired, median x{np.median(ratios):.2f}, "
          f"range x{min(ratios):.2f}-x{max(ratios):.2f} "
          f"({alt_4m.mean():.2f} m -> {alt_sh.mean():.2f} m altitude)")

    # ---- 4b. Device-facing acquisition constants -------------------------------
    # These are the numbers this module and the real stack must not silently
    # disagree on (root CLAUDE.md CM-2). The simulator keeps its own calibrated
    # defaults, so comparability rests on the acquisition actually in force
    # being explicit and traceable to the bundle -- that is what is gated here.
    print("[4b] acquisition constants: auto-gain, ladder, bundle binding")

    # gain_index -1 carries the real node's meaning (device auto-gain). It must
    # produce a defined, reportable gain: the profile field is uint16, so a
    # literal -1 cannot be framed at all, and the -15 dB it would scale to
    # would move max_pwr_db off its real-capture anchor (NC #6).
    acq_auto = dataclasses.replace(acq, gain_index=-1)
    acq_cal = dataclasses.replace(acq, gain_index=4)
    probe_pose = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rp = GeometricRenderer(scene, acq_cal, model).render(Side.PORT, probe_pose, 0.0)
    e_auto = PingEncoder(Side.PORT, acq_auto, model).encode(rp.ping)
    e_cal = PingEncoder(Side.PORT, acq_cal, model).encode(rp.ping)
    check("gain_index -1 (device auto) resolves to the calibrated index",
          acq_auto.effective_gain_index == 4 and e_auto.gain_index == 4,
          f"reported gain_index {e_auto.gain_index}, "
          f"analog_gain {e_auto.analog_gain}")
    check("auto-gain frames byte-identically to the calibrated gain",
          e_auto.raw_frame == e_cal.raw_frame
          and parse_frame(e_auto.raw_frame)["gain_index"] == 4,
          f"{len(e_auto.raw_frame)} B, decoded gain "
          f"{parse_frame(e_auto.raw_frame)['gain_index']}")
    # The ladder runs 0-7. Indices 4-7 are the values the device itself
    # reports in the field corpus (analog_gain 74.55 / 142.8 / 242.025 /
    # 464.625); its auto-gain walks all four inside every recording, so a
    # ladder that stopped at 5 could not express most real acquisitions.
    ladder_ok = []
    for g in range(MAX_GAIN_INDEX + 1):
        eg = PingEncoder(Side.PORT, dataclasses.replace(acq, gain_index=g),
                         model).encode(rp.ping)
        ladder_ok.append(eg.gain_index == g
                         and eg.analog_gain == ANALOG_GAIN_TABLE[g]
                         and len(eg.raw_frame) == len(e_cal.raw_frame))
    check("gain ladder 0-7 encodes and reports its own index and analog gain",
          all(ladder_ok) and MAX_GAIN_INDEX == 7,
          f"{sum(ladder_ok)}/{MAX_GAIN_INDEX + 1} indices; measured 4-7 = "
          + ", ".join(f"{g}:{ANALOG_GAIN_TABLE[g]:g}" for g in range(4, 8)))
    # Gain is a level shift under the device's normalisation, so it must move
    # the reported dB and leave the counts alone -- the opposite of what an
    # absolute-count encoding does.
    e_g4 = PingEncoder(Side.PORT, dataclasses.replace(acq, gain_index=4),
                       model).encode(rp.ping)
    e_g6 = PingEncoder(Side.PORT, dataclasses.replace(acq, gain_index=6),
                       model).encode(rp.ping)
    check("a gain step moves the reported dB, not the normalised counts",
          np.array_equal(e_g4.pwr_results, e_g6.pwr_results)
          and abs((e_g6.max_pwr_db - e_g4.max_pwr_db)
                  - gain_step_db(6)) < 1e-6,
          f"counts identical, max_pwr_db {e_g6.max_pwr_db - e_g4.max_pwr_db:+.2f} dB "
          f"for index 4 -> 6")
    rejected = 0
    for bad in (-2, 8, 99):
        try:
            dataclasses.replace(acq, gain_index=bad)
        except ValueError:
            rejected += 1
    check("gain indices outside the ladder are rejected, not silently defaulted",
          rejected == 3, f"{rejected}/3 rejected (-2, 8, 99)")

    # The bundle freezes its own sonar.yaml (NC #10) and sss_sim_launch.py
    # passes those six values to the node explicitly, so a run images what the
    # bundle records. Exercise the launch's own resolver, not a copy of it.
    sys.path.insert(0, str(PKG_ROOT / "launch"))
    from sss_sim_launch import ACQUISITION_ARGS, resolve_acquisition  # noqa: E402

    bundle_acq = resolve_acquisition(bundle / "sonar.yaml")
    check("launch passes every acquisition parameter the real node declares",
          tuple(bundle_acq) == ACQUISITION_ARGS
          and len(ACQUISITION_ARGS) == 6,
          ", ".join(ACQUISITION_ARGS))
    check("launch binds the bundle's frozen acquisition, not node defaults",
          bundle_acq == dataclasses.asdict(acq),
          f"range_length_mm {bundle_acq['range_length_mm']}, "
          f"gain_index {bundle_acq['gain_index']}")
    over = resolve_acquisition(bundle / "sonar.yaml",
                               {"range_length_mm": "30000", "gain_index": "-1",
                                "num_results": "", "pulse_len_percent": "0.004"})
    check("per-run overrides apply, empty overrides fall back to the bundle",
          over["range_length_mm"] == 30000 and over["gain_index"] == -1
          and over["num_results"] == bundle_acq["num_results"]
          and abs(over["pulse_len_percent"] - 0.004) < 1e-12,
          f"{over['range_length_mm']} mm, gain {over['gain_index']}, "
          f"n={over['num_results']}")
    bad_override = False
    try:
        resolve_acquisition(bundle / "sonar.yaml", {"gain_index": "11"})
    except ValueError:
        bad_override = True
    check("an unusable gain override fails at launch, not at the first ping",
          bad_override)

    # project_synthesis.md 8.5's coverage pass ships as its own profile rather
    # than moving the calibrated default. It must differ in range alone, and
    # must render as well as the default does.
    cov = SonarConfig.from_yaml(PKG_ROOT / "config" / "coverage_pass_sonar.yaml")
    check("coverage-pass profile differs from the default in range alone",
          cov.model == model
          and cov.acquisition.range_length_mm == 30_000
          and dataclasses.asdict(dataclasses.replace(
              cov.acquisition, range_length_mm=acq.range_length_mm))
          == dataclasses.asdict(acq),
          f"{cov.acquisition.range_length_mm} mm, "
          f"{cov.acquisition.bin_size_m*1000:.0f} mm bins, "
          f"free-run {cov.acquisition.ping_period_s(0.0)*1000:.0f} ms")
    rend_cov = GeometricRenderer(scene, cov.acquisition, model)
    enc_cov = PingEncoder(Side.PORT, cov.acquisition, model)
    rng_cov = np.random.default_rng(1)
    cov_probe, cov_power, cov_alt = [], [], []
    for i in range(120):
        ts = 60.0 + i * cov.acquisition.ping_period_s(model.max_ping_rate_hz)
        px, py, pyaw = traj.pose_at(ts)
        rr = rend_cov.render(Side.PORT, Pose3D(px, py, 0.0, yaw=pyaw), ts)
        m = (cov.acquisition.range_start_mm / 1000.0
             + (np.arange(cov.acquisition.num_results) + 0.5)
             * cov.acquisition.bin_size_m) < rr.ping.altitude_m
        rr.ping.power = apply_ping_noise(rr.ping.power, m, 1.0, model, rng_cov,
                                         specular=rr.ping.specular)
        e_cov = enc_cov.encode(rr.ping)
        cov_probe.append(scale_to_db(e_cov.pwr_results, e_cov.min_pwr_db,
                                     e_cov.max_pwr_db))
        cov_power.append(np.asarray(rr.ping.power, dtype=np.float64))
        cov_alt.append(rr.ping.altitude_m)
    cov_probe = np.array(cov_probe)
    check("coverage-pass frames are byte-valid at 30 m",
          len(enc_cov.encode(rr.ping).raw_frame)
          == 8 + 52 + 2 * cov.acquisition.num_results + 2)
    cov_est = [_fbr_est(p, cov.acquisition.bin_size_m) for p in cov_probe]
    cov_err = [abs(e - a) for e, a in zip(cov_est, cov_alt) if e is not None]
    check("FBR still locks at the 30 m coverage setting (NC #4)",
          len(cov_err) == len(cov_probe) and float(np.median(cov_err)) < 0.25,
          f"{len(cov_err)}/{len(cov_probe)} pings, median error "
          f"{np.median(cov_err)*100:.0f} cm at {np.mean(cov_alt):.2f} m")
    far = np.mean(cov_power, axis=0)[400:590]
    check("no empty far-range bins at 30 m (sampling tracks bin size, NC #9)",
          float((far <= 0.0).mean()) < 0.02, f"{(far <= 0.0).mean():.1%} empty")

    # ---- 4c. Detector metrics from ground truth ----------------------------
    print("[4c] detector metrics from ground truth")
    import hashlib
    import json as _json

    met_scene = SceneModel.load(bundle)
    met_obs = from_replay(bundle, max_pings=700)
    check("replay yields ground-truth observations",
          len(met_obs.observations) > 0 and met_obs.has_aspect,
          f"{len(met_obs.observations)} observations over "
          f"{met_obs.ping_cycles} ping cycles, aspect available")

    # The manifest reconciliation: every placed object is accounted for.
    # compute_metrics asserts the partition internally, so reaching this line
    # already proves it; the check restates it as a visible number.
    met = compute_metrics(met_obs, met_scene, criterion="resolved")
    partition = (len(met.detected) + len(met.observed_not_detected)
                 + len(met.never_ensonified))
    check("every manifest object is accounted for",
          partition == met.n_objects == len(met_scene.objects),
          f"{len(met.detected)} detected + "
          f"{len(met.observed_not_detected)} below criterion + "
          f"{len(met.never_ensonified)} never ensonified = {partition}")

    # An unvisited bin is unmeasured, not a failure: it must read None, never
    # 0.0, or a range the survey never presented an object at would look like
    # a detection failure that never happened.
    bad_bins = [(name, i)
                for name, cur in list(met.range_curves.items())
                + list(met.aspect_curves.items())
                for i, pr in enumerate(cur.probability)
                if (pr is None) != (cur.opportunities[i] == 0)
                or (pr is not None and not 0.0 <= pr <= 1.0)]
    check("empty bins read as unmeasured, probabilities stay in [0, 1]",
          not bad_bins, f"{len(met.range_curves)} range + "
          f"{len(met.aspect_curves)} aspect curves clean")

    # The criterion ladder nests: resolved/shadowed are geometric plus a
    # further condition, so neither can detect what geometric did not.
    geo = set(compute_metrics(met_obs, met_scene, criterion="geometric").detected)
    res = set(compute_metrics(met_obs, met_scene, criterion="resolved").detected)
    sha = set(compute_metrics(met_obs, met_scene, criterion="shadowed").detected)
    check("detection criteria nest inside the geometric criterion",
          res <= geo and sha <= geo,
          f"geometric {len(geo)}, resolved {len(res)}, shadowed {len(sha)}")

    # Reproducibility (the point of the exercise): identical twice on one
    # bundle, and identical again on a bundle regenerated from the same seed.
    d_a = content_digest(to_dict(met))
    d_b = content_digest(to_dict(compute_metrics(
        from_replay(bundle, max_pings=700), met_scene, criterion="resolved")))
    regen = generate_mission(PKG_ROOT / "config" / "default_mission.yaml",
                             OUT / "mission_regen", seed=7)
    d_c = content_digest(to_dict(compute_metrics(
        from_replay(regen, max_pings=700), SceneModel.load(regen),
        criterion="resolved")))
    check("metrics reproduce on a bundle regenerated from its seed",
          d_a == d_b == d_c, f"digest {d_a[:16]} three times")

    js, md = write_report(met, OUT / "metrics")
    check("report writes deterministically and outside the bundle",
          js.exists() and md.exists()
          and not (bundle / "metrics.json").exists(),
          f"{js.name} + {md.name} in {js.parent.name}/")

    # A .svlog carries no ground truth, so the svlog path recovers the pose
    # track and re-derives contacts against the same scene. Synthesised here
    # from the frames already encoded above, with the two clocks deliberately
    # skewed as a real log's are.
    def _frame(pid: int, payload: bytes, src: int = 0) -> bytes:
        buf = bytearray(b"BR")
        buf += len(payload).to_bytes(2, "little")
        buf += pid.to_bytes(2, "little") + bytes([src, 0]) + payload
        return bytes(buf + (sum(buf) & 0xFFFF).to_bytes(2, "little"))

    met_cfg = SonarConfig.from_yaml(bundle / "sonar.yaml")
    met_rend = GeometricRenderer(met_scene, met_cfg.acquisition, met_cfg.model)
    met_enc = {s: PingEncoder(s, met_cfg.acquisition, met_cfg.model)
               for s in Side}
    met_period = met_cfg.acquisition.ping_period_s(
        met_cfg.model.max_ping_rate_hz)
    SKEW_MS = 2_644_886          # a real log's arbitrary per-file clock offset
    blob, poses = bytearray(), {}
    for k in range(200):
        t = k * met_period
        px, py, pyaw = traj.pose_at(t)
        ppose = Pose3D(px, py, 0.0, yaw=pyaw)
        poses[k] = ppose
        if k % 4 == 0:
            blob += _frame(150, _json.dumps({
                "header": {"sequence": k % 256},
                "message": {"type": "LOCAL_POSITION_NED",
                            "time_boot_ms": int(t * 1000) - SKEW_MS,
                            "x": py, "y": px, "z": -0.0,
                            "vx": 0.0, "vy": 0.0, "vz": 0.0}}).encode(), src=3)
        for side in Side:
            enc = met_enc[side].encode(met_rend.render(side, ppose, t).ping)
            blob += _frame(2198, enc.raw_frame[8:-2],
                           src=1 if side is Side.PORT else 2)
    svlog_path = OUT / "run.svlog"
    svlog_path.write_bytes(bytes(blob))

    profs, positions, skew = svlog_reader.read_streams(svlog_path)
    check("svlog decodes both channels and recovers the clock skew",
          len(profs) == 400 and len(positions) == 50
          and abs(skew - SKEW_MS) <= 100
          and {p.side for p in profs} == set(Side),
          f"{len(profs)} profiles, {len(positions)} positions, "
          f"skew {skew} vs {SKEW_MS} injected")

    _, paired = svlog_reader.pose_track(svlog_path)
    perr = max(max(abs(pp.x - poses[i // 2].x), abs(pp.y - poses[i // 2].y))
               for i, pp in paired)
    yerr = max(abs((pp.yaw - poses[i // 2].yaw + np.pi) % (2 * np.pi) - np.pi)
               for i, pp in paired)
    check("svlog pose track matches the poses that produced it",
          perr < 0.10 and yerr < 1e-5,
          f"{len(paired)}/{len(profs)} paired, max position error "
          f"{perr*100:.1f} cm, heading {np.degrees(yerr):.1e} deg")

    sv_obs = from_svlog(svlog_path, bundle)
    check("svlog source re-derives contacts against the same scene",
          len(sv_obs.observations) > 0 and sv_obs.has_aspect,
          f"{len(sv_obs.observations)} observations from "
          f"{sv_obs.ping_cycles} located pings")

    # A corrupted byte must cost that one packet, not be read as valid.
    corrupt = bytearray(blob)
    corrupt[20] ^= 0xFF
    (OUT / "run_corrupt.svlog").write_bytes(bytes(corrupt))
    check("a corrupted svlog packet is rejected, not silently accepted",
          len(list(svlog_reader.iter_packets(OUT / "run_corrupt.svlog")))
          == len(list(svlog_reader.iter_packets(svlog_path))) - 1,
          "one packet lost, the rest resynchronised")

    # The published-stream path: no pose rides that topic, so aspect must
    # report as unavailable rather than being invented.
    jl = OUT / "contacts.jsonl"
    with open(jl, "w", encoding="utf-8") as fh:
        for k in range(0, 200, 3):
            t = k * met_period
            cs = []
            for side in Side:
                for c in met_rend.render(side, poses[k], t).contacts:
                    cs.append({"side": c.side.value, "object_id": c.object_id,
                               "type": c.object_type,
                               "slant_range_m": round(c.slant_range_m, 3),
                               "extent_bins": round(c.extent_bins, 1),
                               "shadow_bins": round(c.shadow_bins, 1),
                               "visible": c.visible, "ping_number": k + 1})
            if cs:
                fh.write(_json.dumps({"t_sim": round(t, 3),
                                      "contacts": cs}) + "\n")
    jl_obs = from_jsonl(jl)
    jl_met = compute_metrics(jl_obs, met_scene, criterion="resolved")
    check("published-stream source parses, and reports aspect as unavailable",
          len(jl_obs.observations) > 0 and not jl_obs.has_aspect
          and not jl_met.aspect_curves and jl_met.aspect_available is False,
          f"{len(jl_obs.observations)} observations, no aspect claimed")

    check("blocked metrics are named, not silently omitted",
          len(met.blocked_metrics) == 5
          and "false positives per hectare" in met.blocked_metrics,
          f"{len(met.blocked_metrics)} rows deferred to the detector (D7)")

    # ---- 5. Visual artifact ------------------------------------------------------
    # ---- 4d. Wall multipath (mirror-source ghosts) -----------------------------
    # The enclosed-basin regime's signature artifact. Everything here is
    # geometry the physics fixes in advance, so every check is against a
    # hand-computed number rather than against whatever the renderer happens
    # to produce.
    print("[4d] wall multipath: mirror sources, ghosts, per-ghost ground truth")

    W_Y, W_DEPTH = 10.0, -4.0
    basin_wall = Wall(name="quay", x0=-60.0, y0=W_Y, x1=60.0, y1=W_Y,
                      top_z=1.0, reflectivity=0.65)

    def _flat_basin(walls: list) -> SceneModel:
        g = GridSpec(-50.0, -50.0, 0.25, 401, 401)
        return SceneModel(grid=g, height=np.full((g.ny, g.nx), W_DEPTH),
                          reflectivity=np.full((g.ny, g.nx), 0.5),
                          material_id=np.zeros((g.ny, g.nx), np.uint8),
                          material_names=["sand"], objects=[], walls=walls,
                          seed=1)

    veh = Pose3D(x=0.0, y=4.0, z=0.0, yaw=0.0)      # heading +x, port faces
    mp_cfg = SonarConfig.from_yaml(PKG_ROOT / "config" / "default_sonar.yaml")

    def _mp_render(walls: list, on: bool, bins: int = 600,
                   surface: bool = False):
        c = SonarConfig.from_yaml(PKG_ROOT / "config" / "default_sonar.yaml")
        c.model.wall_multipath_enabled = on
        c.model.surface_mirror_enabled = surface
        c.acquisition.num_results = bins
        r = GeometricRenderer(_flat_basin(walls), c.acquisition, c.model)
        return r.render(Side.PORT, veh, 0.0), c, r

    ping_off, mp_c, mp_r = _mp_render([basin_wall], False)
    ping_nowall, _, _ = _mp_render([], True)
    ping_on, _, _ = _mp_render([basin_wall], True)
    mp_bin = mp_c.acquisition.bin_size_m
    sensor_mp = mp_r._sensor_pose(Side.PORT, veh)   # noqa: SLF001 -- the
    # mount offset is exactly what the hand computation must not forget

    # Mirror-source construction, against the hand-computed case: a wall along
    # x at y = W_Y reflects the transducer to 2*W_Y - y, keeps its z and its
    # along-wall heading, and reverses the athwartship look. The water surface
    # reflects z instead and flips the launch depression.
    srcs = {m.name: m for m in mirror_sources(
        [basin_wall], sensor_mp, Side.PORT, mp_c.acquisition.range_max_m,
        wall_gain=1.0, surface_enabled=True, surface_reflectivity=0.9)}
    mw, ms = srcs["wall:quay"], srcs["surface"]
    check("mirror sources match the hand-computed reflection",
          abs(mw.origin.y - (2 * W_Y - sensor_mp.y)) < 1e-12
          and abs(mw.origin.z - sensor_mp.z) < 1e-12
          and abs((np.degrees(mw.look) % 360) - 270.0) < 1e-9
          and abs(np.degrees(mw.fwd) % 360) < 1e-9
          and mw.depression_sign == 1.0
          and abs(ms.origin.z + sensor_mp.z) < 1e-12
          and ms.depression_sign == -1.0,
          f"wall image at y={mw.origin.y:.2f} looking "
          f"{np.degrees(mw.look) % 360:.0f} deg, surface image at "
          f"z={ms.origin.z:+.2f} with flipped depression")

    # A wall further than the receive window cannot put anything into it: the
    # shortest folded path is the perpendicular distance to the plane.
    far_wall = Wall(name="far", x0=-60.0, y0=40.0, x1=60.0, y1=40.0,
                    reflectivity=0.65)
    culled = mirror_sources([far_wall], sensor_mp, Side.PORT,
                            mp_c.acquisition.range_max_m, wall_gain=1.0,
                            surface_enabled=False, surface_reflectivity=0.9)
    # Finite walls: a sample off the end, and a ray passing over a low wall,
    # are not reflected.
    short = Wall(name="short", x0=-1.0, y0=W_Y, x1=1.0, y1=W_Y,
                 reflectivity=0.65)
    short_img = mirror_sources([short], sensor_mp, Side.PORT, 15.0,
                               wall_gain=1.0, surface_enabled=False,
                               surface_reflectivity=0.9)[0]
    off_end = crossing_mask(short, short_img.origin, np.array([40.0]),
                            np.array([6.0]), np.array([W_DEPTH]))
    low = Wall(name="low", x0=-60.0, y0=W_Y, x1=60.0, y1=W_Y, top_z=-3.5,
               reflectivity=0.65)
    over_top = crossing_mask(low, mw.origin, np.array([0.0]),
                             np.array([6.0]), np.array([W_DEPTH]))
    on_face = crossing_mask(basin_wall, mw.origin, np.array([0.0]),
                            np.array([6.0]), np.array([W_DEPTH]))
    check("a wall reflects only what it geometrically can",
          not culled and not bool(off_end[0]) and not bool(over_top[0])
          and bool(on_face[0]),
          "out-of-range wall culled; off-the-end and over-the-top rays "
          "rejected; the face itself reflects")

    # The ghost lands where its folded path length puts it. Pulse smearing
    # (boxcar of w bins, mode="same") spreads an edge (w-1)/2 bins earlier,
    # so the unsmeared onset is that much later than the first nonzero bin.
    mirror_y = 2 * W_Y - sensor_mp.y
    pred_m = float(np.hypot(mirror_y - W_Y, sensor_mp.z - W_DEPTH))
    ghost_diff = ping_on.ping.power - ping_off.ping.power
    first_bin = int(np.flatnonzero(ghost_diff > 1e-9)[0])
    tau_mp = mp_c.acquisition.pulse_duration_s(mp_c.model.max_ping_rate_hz)
    w_mp = int(round((1500.0 * tau_mp / 2.0) / mp_bin))
    onset_bin = first_bin + (w_mp - 1) // 2
    check("the ghost arrives at its hand-computed slant range",
          onset_bin == int(pred_m / mp_bin),
          f"onset bin {onset_bin} ({onset_bin*mp_bin:.3f} m) vs predicted "
          f"{pred_m:.3f} m from the mirror at y={mirror_y:.2f}")

    check("disabled, the feature leaves no trace whatsoever",
          np.array_equal(ping_off.ping.power, ping_nowall.ping.power)
          and not ping_off.contacts,
          "a walled world with the model off renders bit-identically to a "
          "wall-free one")

    # NC #4/#5: ghost energy joins the diffuse channel and never the coherent
    # specular one, so the first-bottom-return channel downstream bottom
    # tracking locks onto cannot be perturbed by this feature at all.
    check("ghosts never enter the coherent specular channel (NC #4)",
          np.array_equal(ping_on.ping.specular, ping_off.ping.specular)
          and np.array_equal(ping_on.ping.power[:first_bin],
                             ping_off.ping.power[:first_bin]),
          "specular bit-identical; diffuse bins below the ghost onset "
          "bit-identical")

    # The whole survey, ghosts on: the tracker must still bootstrap, the
    # speckle statistic must still be Exp(1), and no far bin may go empty.
    basin = generate_mission(PKG_ROOT / "config" / "enclosed_basin_mission.yaml",
                             OUT / "mission_basin", seed=7)
    scene_b = SceneModel.load(basin)
    cfg_b = SonarConfig.from_yaml(basin / "sonar.yaml")
    check("the shipped basin bundle carries its walls and turns the model on",
          len(scene_b.walls) == 3 and cfg_b.model.wall_multipath_enabled
          and {w.name for w in scene_b.walls} == {"quay_north", "quay_south",
                                                  "pontoon_east"}
          and "wall_quay_north" in (basin / "world.sdf").read_text(),
          f"{len(scene_b.walls)} walls in the manifest, the same three in "
          f"world.sdf, wall_multipath_enabled=True in the frozen sonar.yaml")

    # A bundle written before walls existed must still load: the manifest
    # version does not move for an additive key.
    legacy = SceneModel.load(bundle)
    check("a wall-free bundle still loads at the same manifest version",
          legacy.walls == [] and len(legacy.objects) > 0,
          "scene_manifest.yaml without a walls: key reads as open water")

    acq_b, model_b = cfg_b.acquisition, cfg_b.model
    rend_b = GeometricRenderer(scene_b, acq_b, model_b)
    enc_b = PingEncoder(Side.PORT, acq_b, model_b)
    traj_b = WaypointTrajectory.load_yaml(basin / "trajectory.yaml")
    per_b = acq_b.ping_period_s(model_b.max_ping_rate_hz)
    rg_b = np.random.default_rng(5)
    probe_b, power_b, palt_b, ghost_seen, direct_seen = [], [], [], [], []
    for i in range(220):
        ts = 60.0 + i * per_b
        bx, by, byaw = traj_b.pose_at(ts)
        rr = rend_b.render(Side.PORT, Pose3D(bx, by, 0.0, yaw=byaw), ts)
        for c in rr.contacts:
            (ghost_seen if c.ghost else direct_seen).append(c)
        m = (acq_b.range_start_mm / 1000.0
             + (np.arange(acq_b.num_results) + 0.5) * acq_b.bin_size_m
             ) < rr.ping.altitude_m
        rr.ping.power = apply_ping_noise(rr.ping.power, m, 1.0, model_b, rg_b,
                                         specular=rr.ping.specular)
        e_b = enc_b.encode(rr.ping)
        probe_b.append(scale_to_db(e_b.pwr_results, e_b.min_pwr_db,
                                   e_b.max_pwr_db))
        power_b.append(np.asarray(rr.ping.power, dtype=np.float64))
        palt_b.append(rr.ping.altitude_m)
    probe_b = np.array(probe_b)
    power_b = np.array(power_b)

    ests_b = [_fbr_est(p, acq_b.bin_size_m) for p in probe_b]
    miss_b = sum(e is None for e in ests_b)
    err_b = np.array([abs(e - a) for e, a in zip(ests_b, palt_b)
                      if e is not None])
    check("FBR still locks with wall ghosts in the image (NC #4)",
          miss_b == 0 and float(np.median(err_b)) < 0.25,
          f"{len(ests_b)} pings, {miss_b} misses, median error "
          f"{float(np.median(err_b))*100:.0f} cm at "
          f"{float(np.mean(palt_b)):.2f} m altitude")

    # NC #5 + NC #9, with ghosts summed in: the diffuse field is still
    # fully-developed speckle, and the ground step still fills every bin.
    # Speckle is measured on the linear power, where Exp(1) has CV 1; the
    # encoded array is a dB axis and would report a much smaller number.
    far_b = power_b[:, 400:590]
    cv_b = float(far_b.std() / max(far_b.mean(), 1e-9))
    empty_b = float((power_b.mean(axis=0)[400:590] <= 0.0).mean())
    hi_ping, hi_c, _ = _mp_render([basin_wall], True, bins=1200)
    hi_empty = float((hi_ping.ping.power[800:1180] <= 0.0).mean())
    check("speckle and bin filling survive the extra energy (NC #5, NC #9)",
          0.7 < cv_b < 1.4 and empty_b < 0.02 and hi_empty < 0.02,
          f"far-range CV {cv_b:.2f}, {empty_b:.1%} empty at 600 bins, "
          f"{hi_empty:.1%} at 1200 with ghosts on")

    # Per-ghost ground truth, on a purpose-built basin: a flat seabed, one
    # quay, and two objects planted where the mirror source images them. A
    # real survey does produce ghosts (the shipped basin bundle yields 200
    # over its full 8k ping cycles, off all three walls), but the 0.5 deg
    # along-track beam makes that a matter of where the objects happen to
    # fall -- a planted scene checks the same machinery against a range that
    # can be computed by hand, in a fraction of the time.
    lab_dir = OUT / "ghost_lab"
    lab_scene = _flat_basin([basin_wall])
    lab_scene.objects = [
        PlacedObject(object_id=1, type="tire_car", x=0.0, y=6.0, yaw=0.0,
                     length=0.8, width=0.8, proud_height=0.35, burial=0.0,
                     reflectivity=0.8),
        PlacedObject(object_id=2, type="block_concrete", x=4.0, y=8.0,
                     yaw=0.0, length=0.6, width=0.6, proud_height=0.4,
                     burial=0.0, reflectivity=0.85),
    ]
    lab_scene.save(lab_dir)
    shutil.copyfile(PKG_ROOT / "config" / "enclosed_basin_sonar.yaml",
                    lab_dir / "sonar.yaml")
    WaypointTrajectory(np.array([[-6.0, 4.0], [6.0, 4.0]]), speed=1.0,
                       name="ghost_lab").save_yaml(lab_dir / "trajectory.yaml")

    obs_lab = from_replay(lab_dir)
    g_obs = [o for o in obs_lab.observations if o.ghost]
    d_obs = [o for o in obs_lab.observations if not o.ghost]
    # Hand-computed: object 1 sits at y = 6, the transducer track at y = 4
    # (+ the 0.20 m mount), the quay at y = 10. The mirror images the object
    # at (mirror_y - 6) across, so the ghost is that much further out than
    # the direct return of the same object.
    g1 = [o for o in g_obs if o.object_id == 1]
    d1 = [o for o in d_obs if o.object_id == 1]
    beam_x = min(abs(o.slant_range_m) for o in g1) if g1 else 0.0
    pred_ghost = float(np.hypot((2 * W_Y - sensor_mp.y) - 6.0,
                                sensor_mp.z - W_DEPTH))
    check("every ghost is labelled with its object and its reflector",
          len(g_obs) > 0
          and all(o.ghost and o.via == "wall:quay" for o in g_obs)
          and all((not o.ghost) and o.via == "" for o in d_obs)
          and {o.object_id for o in g_obs} == {1, 2}
          and abs(beam_x - pred_ghost) < 0.05,
          f"{len(g_obs)} ghosts of objects "
          f"{sorted({o.object_id for o in g_obs})} via wall:quay; object 1 "
          f"ghosts at {beam_x:.2f} m (hand-computed {pred_ghost:.2f} m) "
          f"against its direct {min(o.slant_range_m for o in d1):.2f} m")

    # The published JSON schema stays additive: a consumer written before
    # ghosts existed reads the stream unchanged, and one written after
    # recovers both new fields.
    legacy_payload = {"side": "port", "object_id": 3, "type": "can",
                      "slant_range_m": 5.0, "extent_bins": 4.0,
                      "shadow_bins": 9.0, "visible": True, "ping_number": 12}
    new_payload = dict(legacy_payload, ghost=True, via="wall:quay_north")
    parsed = [GroundTruthContact(
        object_id=int(c["object_id"]), object_type=str(c["type"]),
        side=Side(c["side"]), slant_range_m=float(c["slant_range_m"]),
        extent_bins=float(c["extent_bins"]),
        shadow_bins=float(c["shadow_bins"]), visible=bool(c["visible"]),
        ghost=bool(c.get("ghost", False)), via=str(c.get("via", "")))
        for c in (legacy_payload, new_payload)]
    check("the ground-truth JSON schema extends additively",
          parsed[0].ghost is False and parsed[0].via == ""
          and parsed[1].ghost is True
          and parsed[1].via == "wall:quay_north"
          and parsed[0].group_key != parsed[1].group_key,
          "a pre-ghost message parses as direct; the two group separately")

    # The labeller must not merge an object with its own ghost into one box
    # spanning the water between them.
    lab = TileLabeler(600, 0.025, LabelConfig(min_rows=2))
    rows_mp = []
    for r_i in range(6):
        rows_mp.append(type(  # a PingRow-alike: the labeller reads .contacts
            "R", (), {"contacts": [
                GroundTruthContact(object_id=7, object_type="tire_car",
                                   side=Side.PORT, slant_range_m=4.0,
                                   extent_bins=6.0, shadow_bins=8.0,
                                   visible=True),
                GroundTruthContact(object_id=7, object_type="tire_car",
                                   side=Side.PORT, slant_range_m=11.0,
                                   extent_bins=6.0, shadow_bins=8.0,
                                   visible=True, ghost=True,
                                   via="wall:quay_north")]})())
    boxes_mp = lab.label_tile(rows_mp)
    kinds = sorted(b.object_type for b in boxes_mp)
    widest = max(b.width for b in boxes_mp) if boxes_mp else 1.0
    check("a ghost is boxed as its own class, not merged with the object",
          len(boxes_mp) == 2 and kinds == ["ghost", "tire_car"]
          and widest < 0.25 and "ghost" in lab.class_names,
          f"{len(boxes_mp)} boxes {kinds}, widest {widest:.2f} of the swath "
          "(a merged box would span the 7 m between them)")

    # Metrics: a ghost carries a real object's id, so it must not be able to
    # detect that object -- the manifest partition stays exact and the ghosts
    # are reported on their own.
    met_b = compute_metrics(obs_lab, lab_scene)
    doc_b = to_dict(met_b)
    n_ghost_obs = len(g_obs)
    check("ghosts are counted, reported, and kept out of every rate",
          met_b.n_ghost_observations == n_ghost_obs
          and met_b.n_observations == len(obs_lab.observations) - n_ghost_obs
          and len(met_b.detected) + len(met_b.observed_not_detected)
          + len(met_b.never_ensonified) == met_b.n_objects
          and doc_b["reconciliation"]["n_ghost_observations"] == n_ghost_obs
          and sum(met_b.ghosts_by_reflector.values()) == n_ghost_obs
          and all(o.n_looks == sum(1 for x in d_obs
                                   if x.object_id == o.object_id)
                  for o in met_b.objects),
          f"{n_ghost_obs} ghosts via {sorted(met_b.ghosts_by_reflector)} "
          f"excluded; every object's looks count direct returns only; "
          f"partition {len(met_b.detected)}+"
          f"{len(met_b.observed_not_detected)}+"
          f"{len(met_b.never_ensonified)} = {met_b.n_objects}")

    print("[5] visual artifact")
    from PIL import Image
    imgs = sorted((OUT / "dataset" / "images").rglob("*.png"))
    montage_src = [np.array(Image.open(p)) for p in imgs[:2]]
    if len(montage_src) == 2 and montage_src[0].shape == montage_src[1].shape:
        # port mirrored | starboard, classic waterfall presentation
        m = np.hstack([np.fliplr(montage_src[0]), montage_src[1]])
        Image.fromarray(m).save(OUT / "waterfall_preview.png")
        print(f"  preview: {OUT/'waterfall_preview.png'} {m.shape}")

    stats = montage_src[0].astype(float)
    check("waterfall has dynamic range", stats.std() > 10.0,
          f"std {stats.std():.1f}, mean {stats.mean():.1f}")

    print("\nALL CHECKS PASSED")


if __name__ == "__main__":
    main()
