"""Offline end-to-end smoke test (no ROS required).

Exercises the full ROS-free pipeline exactly as the ROS nodes drive it:

    world config / builder state -> GPS-anchored world folder -> survey
    trajectory -> per-ping render -> noise -> byte-exact encode -> decode
    round-trip -> ground-truth metrics -> calibration invariants

Run with:  python3 -m test.smoke_test   (from the package root)
Artifacts land in /tmp/blueboat_sss_smoke for visual inspection.
``test/fixtures/survey_lawnmower.yaml`` is the retired default mission's
lawnmower, frozen verbatim so the calibrated sensor anchors ([2c]) keep
their measured context.
"""

from __future__ import annotations

import dataclasses
import json
import os
import re
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
from blueboat_sss_sim.core.geo import (EARTH_RADIUS_M, GeoAnchor,  # noqa: E402
                                       latlon_to_local_en,
                                       local_en_to_latlon)
from blueboat_sss_sim.core.trajectory import (TimedTrajectory,  # noqa: E402
                                              WaypointTrajectory,
                                              load_trajectory)
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
from blueboat_sss_sim.worldgen.scene import (SceneModel,  # noqa: E402
                                             WorldConfig, generate_scene)
from blueboat_sss_sim.worldgen.sdf_writer import write_world_sdf  # noqa: E402
from blueboat_sss_sim.worldgen.objects import (CATALOG as _CATALOG_V3,  # noqa: E402
                                               typical_instance)

OUT = Path("/tmp/blueboat_sss_smoke")
FIXTURES = PKG_ROOT / "test" / "fixtures"

#: Minimal stand-in for analysis.svlog_reader.ProfileRecord, so the corpus's
#: normalisation invariants can be asserted on freshly encoded pings without
#: routing them through a file.
_PowerRec = namedtuple("_PowerRec", "pwr_results min_pwr_db max_pwr_db")


def _ringing_settle_bin(mean_db, search_max: int = 60, drop_db: float = 10.0,
                        persistence: int = 5, fallback: int = 30) -> int:
    """First bin past the transmit-ringing tail — the processor's
    ``sss_helper.find_noise_window_start`` logic, reimplemented (this
    module deliberately imports nothing from BlueBoat-SSS)."""
    peak = float(mean_db[:search_max].max())
    for i in range(search_max - persistence + 1):
        if bool((mean_db[i:i + persistence] < peak - drop_db).all()):
            return i
    return fallback


def _processor_lock_bin(mean_db, noise_window: int = 20,
                        threshold_delta_db: float = 8.0,
                        persistence: int = 3) -> int:
    """Downstream FBR bootstrap on a mean dB curve, the way
    ``sss_processor_node`` actually does it: skip the ringing relative
    to its own peak, estimate the noise floor just past it, then take
    the first persistent crossing of floor + delta. A detector that
    scans from bin 0 locks on the (real, corpus-measured) transmit
    ringing instead — which is exactly why the processor carries the
    settle logic."""
    settle = _ringing_settle_bin(mean_db)
    noise = float(mean_db[settle:settle + noise_window].mean())
    for i in range(settle + noise_window, len(mean_db) - persistence + 1):
        if bool((mean_db[i:i + persistence] > noise + threshold_delta_db
                 ).all()):
            return i
    return 0


def check(name: str, cond: bool, detail: str = "") -> None:
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {name}" + (f" -- {detail}" if detail else ""))
    if not cond:
        raise SystemExit(f"smoke test failed at: {name}")


_LOCK_HANDLE = None


def main() -> None:
    # One run at a time: the Stop hook and a manual run share OUT, and two
    # concurrent runs deleted each other's artefacts mid-way (mkdir races
    # in [9]). The second run simply waits for the first to finish.
    global _LOCK_HANDLE
    import fcntl
    _LOCK_HANDLE = open(str(OUT) + ".lock", "w")
    fcntl.flock(_LOCK_HANDLE, fcntl.LOCK_EX)
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)

    # ---- 1. World generation -----------------------------------------------
    print("[1] world generation (legacy default scene)")
    import yaml as _y1

    def legacy_world(world_yaml: Path, out_dir: Path,
                     seed: int = 7, density: float = 60.0) -> SceneModel:
        """The retired default mission bundle's world half, reproduced
        bit-identically: same world config, the mission's density override,
        seed 7. Verified equal to the last generate_mission output when the
        fixture was frozen."""
        with open(world_yaml, "r", encoding="utf-8") as f1:
            raw1 = _y1.safe_load(f1) or {}
        raw1.setdefault("world", {})["seed"] = seed
        raw1.setdefault("objects", {})["density_per_hectare"] = density
        sc1 = generate_scene(WorldConfig.from_dict(raw1), raw_config=raw1)
        sc1.save(out_dir)
        write_world_sdf(sc1, out_dir)
        return sc1

    world_dir = OUT / "legacy_world"
    scene = legacy_world(PKG_ROOT / "config" / "default_world.yaml", world_dir)
    for f in ("world.sdf", "seabed.stl", "scene.npz", "scene_manifest.yaml"):
        check(f"world dir contains {f}", (world_dir / f).exists())

    check("scene has objects", len(scene.objects) > 0,
          f"{len(scene.objects)} objects")
    traj_check = WaypointTrajectory.load_yaml(FIXTURES / "survey_lawnmower.yaml")
    x0, y0, _ = traj_check.pose_at(0.0)
    check("survey path starts at robot spawn (0,0)",
          abs(x0) < 0.01 and abs(y0) < 0.01,
          f"first pose ({x0:.2f}, {y0:.2f})")
    sdf = (world_dir / "world.sdf").read_text()
    check("world.sdf has physics plugin", "physics" in sdf)
    check("world.sdf has buoyancy plugin", "buoyancy" in sdf)
    check("world.sdf references seabed mesh", "seabed.stl" in sdf)

    # ---- 2. Sonar simulation along the trajectory ----------------------------
    print("[2] sonar rendering along lawnmower pass")
    cfg = SonarConfig.from_yaml(PKG_ROOT / "config" / "default_sonar.yaml")
    acq, model = cfg.acquisition, cfg.model
    traj = WaypointTrajectory.load_yaml(FIXTURES / "survey_lawnmower.yaml")
    renderer = GeometricRenderer(scene, acq, model)
    rng = np.random.default_rng(1)

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
                                            specular=r.ping.specular,
                                            texture=r.ping.texture)
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
    # Already dB, so the contrast is a subtraction. `wc` here is the whole
    # gap from bin 0, so it averages the transmit ringing (55 -> 33 dB over
    # the first metre) with the measured 25 dB water column; at this
    # fixture's ~3.5 m altitude that mean sits near 37 dB against a ~68 dB
    # peak. (The corpus at gain 4, 6-9 m altitude, excluding the ringing
    # bins, gives 30-36 dB -- the [2f] statistic.)
    contrast_db = peak - float(wc.mean())
    check("water column is quiet vs FBR peak (~31 dB incl. ringing bins)",
          24.0 < contrast_db < 44.0,
          f"{contrast_db:.1f} dB gap->peak contrast")
    # Transmit ringing (corpus-measured 55 dB at the transducer decaying
    # to ~33 dB by 1 m): present, at level, and settled well before the
    # first bottom return.
    check("transmit ringing at the corpus level",
          abs(float(mean_p[0]) - model.ringing_level_db) < 3.0,
          f"first bin {mean_p[0]:.1f} dB vs measured "
          f"{model.ringing_level_db:g} dB")
    # The processor's actual bootstrap (sss_helper.find_noise_window_start
    # + detect_fbr_slant_m): skip the ringing tail relative to its own
    # peak, estimate the noise floor past it, then find the persistent
    # threshold crossing. A detector that scans from bin 0 instead locks
    # on the ringing — exactly why the real one carries the settle logic.
    settle = _ringing_settle_bin(mean_p)
    check("ringing settles inside the corpus envelope (< 1.2 m)",
          settle * acq.bin_size_m < 1.2,
          f"settled at bin {settle} = {settle*acq.bin_size_m:.2f} m")
    lock = _processor_lock_bin(mean_p)
    check("processor-style FBR bootstrap locks at altitude",
          abs(lock * acq.bin_size_m - mean_alt) < 0.25,
          f"lock bin {lock} = {lock*acq.bin_size_m:.2f} m, "
          f"altitude {mean_alt:.2f} m")

    # ---- 2c. Omniscan device fidelity ------------------------------------
    print("[2c] device fidelity (rate cap, power scale, speckle PDF)")
    check("ping period respects max_ping_rate_hz",
          period >= 0.999 / max(model.max_ping_rate_hz, 1e-9)
          if model.max_ping_rate_hz > 0 else True,
          f"{period*1000:.0f} ms (cap {model.max_ping_rate_hz} Hz)")
    # The device law, measured on all seven corpus recordings: 20 pings/s up
    # to 30 m (18.4 m and 20 m files at exactly 50 ms), then 2R/c + ~3 ms
    # (131 ms at 95.6 m, 170 ms at 125.6 m).
    _p = lambda mm: dataclasses.replace(acq, range_length_mm=mm).ping_period_s(  # noqa: E731
        model.max_ping_rate_hz)
    check("ping period follows the corpus law: 50 ms to 30 m, 2R/c+3 ms beyond",
          abs(_p(15_000) - 0.050) < 1e-9 and abs(_p(30_000) - 0.050) < 1e-9
          and abs(_p(95_646) - 0.131) < 0.0015
          and abs(_p(125_639) - 0.170) < 0.0015,
          f"15 m {_p(15_000)*1000:.1f} ms, 30 m {_p(30_000)*1000:.1f} ms, "
          f"95.6 m {_p(95_646)*1000:.1f} ms, 125.6 m {_p(125_639)*1000:.1f} ms")
    # The transmit pulse follows the device's sample-period law (1.2 sample
    # periods at 600 bins, 1.5 at 1200, 44.3 us floor), reproducing every
    # pulse_duration_sec the corpus reports.
    _pl = lambda mm, n: dataclasses.replace(  # noqa: E731
        acq, range_length_mm=mm, num_results=n).pulse_duration_s()
    check("pulse duration follows the device law (44.3 / 106.4 / 159.6 us)",
          abs(_pl(18_400, 1200) - 44.3e-6) < 1.5e-6
          and abs(_pl(40_000, 600) - 106.4e-6) < 2e-6
          and abs(_pl(95_646, 1200) - 159.6e-6) < 2e-6
          and abs(_pl(80_000, 600) - 212.8e-6) < 2e-6,
          f"{_pl(18_400, 1200)*1e6:.1f} / {_pl(40_000, 600)*1e6:.1f} / "
          f"{_pl(95_646, 1200)*1e6:.1f} / {_pl(80_000, 600)*1e6:.1f} us")
    _enc_cnt = PingEncoder(Side.PORT, acq, model)
    _n1 = _enc_cnt.encode(r.ping).ping_number
    _enc_cnt.reconfigure(dataclasses.replace(acq, range_length_mm=30_000), model)
    _n2 = _enc_cnt.encode(r.ping).ping_number
    check("the ping counter survives a reconfigure (device counts from power-up)",
          (_n1, _n2) == (1, 2), f"ping numbers {_n1}, {_n2} across a range change")
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
    # ---- 2g. Speckle texture: single-look contrast AND correlation ----------
    # The corpus (mid-range band, after range equalisation) shows single-look
    # contrast (9x9 high-pass std 5.5-6.7 dB), lag-1 correlation 0.30-0.68
    # along range and 0.36-0.66 along track. The coherent scatterer field
    # reproduces both from geometry: the pulse/receive kernel on the complex
    # field along range, footprint overlap between consecutive pings along
    # track. Measured here on `Ping.texture` (unit mean by construction)
    # over 60 consecutive port pings of the straight first leg.
    print("[2g] speckle texture: contrast and correlation")
    tex_rows = []
    for k in range(60):
        t = k * period
        x, y, yaw = traj.pose_at(t)
        tex_rows.append(renderer.render(Side.PORT, Pose3D(x, y, 0.0, yaw=yaw),
                                        t).ping.texture)
    tex = np.stack(tex_rows)
    mid = slice(int(0.3 * acq.num_results), int(0.7 * acq.num_results))
    tex_mid = tex[:, mid]
    tex_db = 10 * np.log10(tex_mid + 1e-9)

    def _corr(a, b):
        a = a.ravel() - a.mean()
        b = b.ravel() - b.mean()
        return float((a * b).mean() / np.sqrt((a * a).mean() * (b * b).mean()))

    check("texture has unit mean and single-look contrast (Exp(1): CV 1, 5.6 dB)",
          0.9 < float(tex_mid.mean()) < 1.1
          and 0.85 < float(tex_mid.std() / tex_mid.mean()) < 1.15
          and 4.8 < float(tex_db.std()) < 6.4,
          f"mean {tex_mid.mean():.3f}, CV {tex_mid.std()/tex_mid.mean():.2f}, "
          f"dB std {tex_db.std():.1f}")
    r1 = _corr(tex_db[:, :-1], tex_db[:, 1:])
    t1 = _corr(tex_db[:-1], tex_db[1:])
    check("texture is correlated along range (corpus lag-1 0.30-0.68)",
          0.2 < r1 < 0.7, f"lag-1 range correlation {r1:.2f}")
    check("texture is correlated along track (corpus lag-1 0.36-0.66)",
          0.12 < t1 < 0.7, f"lag-1 along-track correlation {t1:.2f} at "
          f"{traj.speed if hasattr(traj, 'speed') else float('nan'):.2f} m/s")
    tex_b = GeometricRenderer(
        scene, acq, dataclasses.replace(model, scatterer_seed=99)).render(
        Side.PORT, Pose3D(*traj.pose_at(0.0)[:2], 0.0,
                          yaw=traj.pose_at(0.0)[2]), 0.0).ping.texture
    tex_a2 = renderer.render(Side.PORT, Pose3D(*traj.pose_at(0.0)[:2], 0.0,
                                               yaw=traj.pose_at(0.0)[2]),
                             0.0).ping.texture
    check("texture is deterministic per seed and decorrelates across seeds",
          np.array_equal(tex_a2, tex[0])
          and abs(_corr(10 * np.log10(tex_b[mid] + 1e-9), tex_db[0])) < 0.15,
          f"same seed identical; other seed correlation "
          f"{_corr(10*np.log10(tex_b[mid]+1e-9), tex_db[0]):.2f}")
    nospk = GeometricRenderer(scene, acq, dataclasses.replace(model, speckle=False))
    p_ns = nospk.render(Side.PORT, Pose3D(*traj.pose_at(0.0)[:2], 0.0,
                                          yaw=traj.pose_at(0.0)[2]), 0.0).ping
    check("speckle: false keeps the noiseless mean path (no texture)",
          p_ns.texture is None and np.all(np.isfinite(p_ns.power)),
          "renderer returns the mean only")

    print("[2d] along-track 0.5 deg beam + 1/1200 bins")

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
        # Two clock regimes (2026-09-05): a SonarView-native recording
        # decodes GPS fixes on a live autopilot clock; a processor-written
        # one carries a single frozen time_boot_ms and must say so.
        _live_f = corpus / "diffDepthCompensation.svlog"
        _frz_f = corpus / "MainReflectionEvidence+misspingsWaterfall.svlog"
        if _live_f.exists() and _frz_f.exists():
            _s_live = svlog_reader.read_svlog(_live_f)
            _s_frz = svlog_reader.read_svlog(_frz_f)
            check("corpus: GPS fixes decode on a live clock; the processor-"
                  "written file reports a frozen clock",
                  len(_s_live.gps) > 100
                  and abs(_s_live.gps[0].lat - 34.408) < 0.01
                  and not _s_live.clock_frozen and _s_frz.clock_frozen
                  and len(_s_frz.gps) > 100,
                  f"{len(_s_live.gps)} fixes, skew {_s_live.skew_ms} ms; "
                  f"frozen file: {len(_s_frz.gps)} fixes")
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

    # ---- 4. Shallow-regime world ------------------------------------------------
    # `config/shallow_water_world.yaml` ships alongside the 4 m default at
    # 2.5 m mean depth. It differs from the default in `base_depth` alone, and
    # base_depth consumes no RNG draws, so the same seed places the *same*
    # objects at the *same* poses -- the two bundles are directly comparable
    # and shadow extents can be paired by object_id.
        # Quay contrast (analysis/wall_contrast): the numbers the 2026-09-03
        # wall model is anchored to. On the anchor file the quay face reads
        # ~+20 dB over the seabed in front, the zone beyond it darker.
        from blueboat_sss_sim.analysis.wall_contrast import measure as _wc
        anchor_log = corpus / "fullHarbourCleanExample.svlog"
        if anchor_log.is_file():
            wcr = _wc(anchor_log)
            got = [v for v in wcr.values() if v is not None]
            check("the corpus quay: bright face, darker beyond (wall_contrast)",
                  len(got) == 2
                  and all(15.0 < v.peak_minus_before_db < 30.0 for v in got)
                  and all(v.beyond_minus_before_db < 0.0 for v in got)
                  and all(v.far_minus_before_db < -15.0 for v in got)
                  and all(0.4 < v.band_width_m < 1.5 for v in got),
                  "; ".join(f"{v.side}: peak {v.peak_minus_before_db:+.1f} "
                            f"beyond {v.beyond_minus_before_db:+.1f} far "
                            f"{v.far_minus_before_db:+.1f} dB band "
                            f"{v.band_width_m:.2f} m" for v in got))

    print("[4] shallow-regime world config")
    scene_sh = legacy_world(PKG_ROOT / "config" / "shallow_water_world.yaml",
                            OUT / "shallow_world")
    depth_sh = -scene_sh.height
    check("shallow world sits in the 1-3 m thesis regime",
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
    # being explicit and traceable to a named profile -- that is what is
    # gated here.
    print("[4b] acquisition constants: auto-gain, ladder, profile binding")

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

    # World folders carry no frozen sonar.yaml: the acquisition in force is
    # the package profile plus explicit per-run overrides, and
    # sss_sim_launch.py passes those six values to the node explicitly.
    # Exercise the launch's own resolver, not a copy of it.
    sys.path.insert(0, str(PKG_ROOT / "launch"))
    from sss_sim_launch import ACQUISITION_ARGS, resolve_acquisition  # noqa: E402

    profile_yaml = PKG_ROOT / "config" / "default_sonar.yaml"
    bundle_acq = resolve_acquisition(profile_yaml)
    check("launch passes every acquisition parameter the real node declares",
          tuple(bundle_acq) == ACQUISITION_ARGS
          and len(ACQUISITION_ARGS) == 6,
          ", ".join(ACQUISITION_ARGS))
    check("launch binds the named sonar profile, not node defaults",
          bundle_acq == dataclasses.asdict(acq),
          f"range_length_mm {bundle_acq['range_length_mm']}, "
          f"gain_index {bundle_acq['gain_index']}")
    over = resolve_acquisition(profile_yaml,
                               {"range_length_mm": "30000", "gain_index": "-1",
                                "num_results": "", "pulse_len_percent": "0.004"})
    check("per-run overrides apply, empty overrides fall back to the profile",
          over["range_length_mm"] == 30000 and over["gain_index"] == -1
          and over["num_results"] == bundle_acq["num_results"]
          and abs(over["pulse_len_percent"] - 0.004) < 1e-12,
          f"{over['range_length_mm']} mm, gain {over['gain_index']}, "
          f"n={over['num_results']}")
    bad_override = False
    try:
        resolve_acquisition(profile_yaml, {"gain_index": "11"})
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

    met_traj = FIXTURES / "survey_lawnmower.yaml"
    met_scene = SceneModel.load(world_dir)
    met_obs = from_replay(world_dir, max_pings=700, trajectory=met_traj)
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
    # world, and identical again on a world regenerated from the same seed.
    d_a = content_digest(to_dict(met))
    d_b = content_digest(to_dict(compute_metrics(
        from_replay(world_dir, max_pings=700, trajectory=met_traj),
        met_scene, criterion="resolved")))
    legacy_world(PKG_ROOT / "config" / "default_world.yaml",
                 OUT / "legacy_regen")
    d_c = content_digest(to_dict(compute_metrics(
        from_replay(OUT / "legacy_regen", max_pings=700, trajectory=met_traj),
        SceneModel.load(OUT / "legacy_regen"), criterion="resolved")))
    check("metrics reproduce on a world regenerated from its seed",
          d_a == d_b == d_c, f"digest {d_a[:16]} three times")

    js, md = write_report(met, OUT / "metrics")
    check("report writes deterministically and outside the world dir",
          js.exists() and md.exists()
          and not (world_dir / "metrics.json").exists(),
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

    met_cfg = SonarConfig.from_yaml(PKG_ROOT / "config" / "default_sonar.yaml")
    met_rend = GeometricRenderer(met_scene, met_cfg.acquisition, met_cfg.model)
    met_enc = {s: PingEncoder(s, met_cfg.acquisition, met_cfg.model)
               for s in Side}
    met_period = met_cfg.acquisition.ping_period_s(
        met_cfg.model.max_ping_rate_hz)
    SKEW_MS = 2_644_886          # a real log's arbitrary per-file clock offset
    FROZEN_MS = 4_274_768_955    # what a processor-written file carries
    # The GPS fixes are written about the same anchor section [7] uses
    # (wakamatsu_beach), so the soundings round-trip exactly there.
    anchor = GeoAnchor(lat0=33.930824, lon0=130.7311812)
    blob, blob_f, poses = bytearray(), bytearray(), {}

    def _nav(k, t, px, py, pyaw, boot_ms):
        lat_k, lon_k = anchor.to_latlon(px, py)
        out = bytearray()
        for msg in (
                {"type": "LOCAL_POSITION_NED", "time_boot_ms": boot_ms,
                 "x": py, "y": px, "z": -0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0},
                {"type": "GLOBAL_POSITION_INT", "time_boot_ms": boot_ms,
                 "lat": int(round(lat_k * 1e7)), "lon": int(round(lon_k * 1e7)),
                 "alt": 5000, "relative_alt": 0, "vx": 0, "vy": 0, "vz": 0,
                 "hdg": int(round((90.0 - np.degrees(pyaw)) % 360.0 * 100))},
                {"type": "ATTITUDE", "time_boot_ms": boot_ms, "roll": 0.01,
                 "pitch": 0.02, "yaw": float(pyaw), "rollspeed": 0.0,
                 "pitchspeed": 0.0, "yawspeed": 0.0}):
            out += _frame(150, _json.dumps({"header": {"sequence": k % 256},
                                            "message": msg}).encode(), src=3)
        return bytes(out)

    for k in range(200):
        t = k * met_period
        px, py, pyaw = traj.pose_at(t)
        ppose = Pose3D(px, py, 0.0, yaw=pyaw)
        poses[k] = ppose
        if k % 4 == 0:
            blob += _nav(k, t, px, py, pyaw, int(t * 1000) - SKEW_MS)
            blob_f += _nav(k, t, px, py, pyaw, FROZEN_MS)
        for side in Side:
            enc = met_enc[side].encode(met_rend.render(side, ppose, t).ping)
            frame = _frame(2198, enc.raw_frame[8:-2],
                           src=1 if side is Side.PORT else 2)
            blob += frame
            blob_f += frame
    svlog_path = OUT / "run.svlog"
    svlog_path.write_bytes(bytes(blob))
    frozen_path = OUT / "run_frozen.svlog"
    frozen_path.write_bytes(bytes(blob_f))

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

    sv_obs = from_svlog(svlog_path, world_dir)
    check("svlog source re-derives contacts against the same scene",
          len(sv_obs.observations) > 0 and sv_obs.has_aspect,
          f"{len(sv_obs.observations)} observations from "
          f"{sv_obs.ping_cycles} located pings")

    # GPS + attitude envelopes, and the two clock regimes of the corpus.
    s_live = svlog_reader.read_svlog(svlog_path)
    s_frz = svlog_reader.read_svlog(frozen_path)
    _, gi, glat, glon = svlog_reader.gps_track(svlog_path, s_live)
    ge, gn = anchor.to_en(glat, glon)
    gerr = max(max(abs(ge[m] - poses[i // 2].x), abs(gn[m] - poses[i // 2].y))
               for m, i in enumerate(gi))
    check("svlog GPS and attitude envelopes decode on a live clock; fixes "
          "interpolate onto the ping clock",
          len(s_live.gps) == 50 and len(s_live.attitude) == 50
          and not s_live.clock_frozen and abs(s_live.skew_ms - SKEW_MS) <= 100
          and s_live.profiles[0].transducer_heading_deg != 0.0
          and s_live.profiles[0].order >= 0 and gerr < 0.10,
          f"{len(gi)} placed, max position error {gerr * 100:.1f} cm")
    _, fi, flat_, flon_ = svlog_reader.gps_track(frozen_path, s_frz)
    fe, fn = anchor.to_en(flat_, flon_)
    ferr = max(max(abs(fe[m] - poses[i // 2].x), abs(fn[m] - poses[i // 2].y))
               for m, i in enumerate(fi))
    _, paired_f = svlog_reader.pose_track(frozen_path)
    check("a frozen autopilot clock is detected and pairs by file order "
          "(within 4 pings of travel)",
          s_frz.clock_frozen and s_frz.skew_ms == 0 and len(fi) == 400
          and ferr < 4.5 * met_period * 1.0 + 0.05 and len(paired_f) == 400,
          f"max position error {ferr * 100:.1f} cm")

    # Bathymetry: FBR soundings under the boat reproduce the scene depth.
    from blueboat_sss_sim.analysis.bathymetry import survey_track
    surv = survey_track(svlog_path, anchor)
    true_z = met_scene.sample_height(surv.soundings.east, surv.soundings.north)
    dep_err = np.abs(surv.soundings.depth + true_z)
    check("FBR soundings placed at the GPS fixes match the scene depth "
          "under the boat",
          surv.summary["altitude_fraction"] >= 0.9
          and float(np.median(dep_err)) <= 0.15
          and surv.summary["clock_mode"] == "live" and len(surv.cells) > 8,
          f"{100 * surv.summary['altitude_fraction']:.0f} % pings, median "
          f"depth error {np.median(dep_err):.3f} m, {len(surv.cells)} cells")
    surv_f = survey_track(frozen_path, anchor)
    check("the frozen-clock recording still yields soundings, by file order",
          surv_f.summary["clock_mode"] == "file_order"
          and len(surv_f.soundings) >= 0.9 * len(surv.soundings))

    # Where a recording was made, read from the header stream alone.
    from blueboat_sss_sim.analysis.bathymetry import recording_extent
    ext = recording_extent(svlog_path)
    ext_lat, ext_lon = anchor.to_latlon(surv.soundings.east.mean(),
                                        surv.soundings.north.mean())
    check("recording_extent reports the fixes' box and centre without "
          "decoding a power array",
          ext["n_fixes"] > 0 and ext["clock_mode"] == "live"
          and ext["lat_min"] <= ext["lat_c"] <= ext["lat_max"]
          and abs(ext["lat_c"] - float(ext_lat)) < 1e-3
          and abs(ext["lon_c"] - float(ext_lon)) < 1e-3
          and ext["span_e_m"] > 1.0 and ext["span_n_m"] > 1.0,
          f"{ext['n_fixes']} fixes over {ext['span_e_m']:.0f} x "
          f"{ext['span_n_m']:.0f} m")
    # The reporting geodesy: a known pair, against the conversion pair.
    from blueboat_sss_sim.core.geo import (format_distance, latlon_to_local_en,
                                           range_bearing)
    _d, _b, _pt = range_bearing(33.930824, 130.7311812, 34.408454, 133.523628)
    _e0, _n0 = latlon_to_local_en(34.408454, 133.523628, 33.930824, 130.7311812)
    check("range_bearing agrees with the conversion pair on a far site and "
          "names the compass point",
          abs(_d - float(np.hypot(_e0, _n0))) / _d < 0.02
          and _pt == "ENE" and 70.0 < _b < 85.0
          and format_distance(_d) == "263 km",
          f"{format_distance(_d)} {_pt} at {_b:.0f} deg")

    # A corrupted byte must cost that one packet, not be read as valid.
    corrupt = bytearray(blob)
    corrupt[20] ^= 0xFF
    (OUT / "run_corrupt.svlog").write_bytes(bytes(corrupt))
    # ---- 4e. The sim/real comparison instrument on its own output ---------
    print("[4e] svlog compare: timing law, level and texture reductions")
    from blueboat_sss_sim.analysis import svlog_compare as _svc
    fs = _svc.reduce_file(svlog_path, stride=1, min_pings=20)
    sub = {s_.side: s_ for s_ in fs.subsets}
    check("compare reduces both channels of the in-test log",
          set(sub) == {"port", "starboard"} and fs.counter_resets == 0
          and fs.sides_share_timestamps == 1.0,
          f"{fs.n_profiles} profiles, {len(fs.subsets)} subsets, "
          f"{fs.counter_resets} resets, shared stamps "
          f"{fs.sides_share_timestamps:.2f} (one tick drives both sides)")
    check("the simulated ping period sits on the device law",
          abs(sub["port"].period_residual_ms) < 1.0
          and abs(sub["port"].period_median_ms - met_period * 1000) < 1.0,
          f"median {sub['port'].period_median_ms:.1f} ms vs law "
          f"{sub['port'].period_law_ms:.1f} ms")
    check("compare carries level and texture statistics and round-trips JSON",
          np.isfinite(sub["port"].far_decile_db)
          and np.isfinite(sub["port"].hp_std_db)
          and "10" in sub["port"].db_at_range
          and _json.loads(_json.dumps(fs.to_dict()))["subsets"][0]["n_pings"]
          == fs.subsets[0].n_pings,
          f"far decile {sub['port'].far_decile_db:.1f} dB, hp std "
          f"{sub['port'].hp_std_db:.1f} dB, range lag-1 "
          f"{sub['port'].range_corr_lag1:.2f}")
    _js, _md = _svc.write_compare([fs], OUT / "compare")
    check("compare writes compare.json + compare.md outside the world dir",
          _js.exists() and _md.exists() and "svlog comparison" in _md.read_text(),
          f"{_js.name}, {_md.name}")

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
                   surface: bool = False, echo: bool = False,
                   pose: Pose3D | None = None):
        c = SonarConfig.from_yaml(PKG_ROOT / "config" / "default_sonar.yaml")
        c.model.wall_multipath_enabled = on
        c.model.surface_mirror_enabled = surface
        c.model.wall_echo_enabled = echo    # the two wall features are
        c.acquisition.num_results = bins    # tested separately
        r = GeometricRenderer(_flat_basin(walls), c.acquisition, c.model)
        return r.render(Side.PORT, pose or veh, 0.0), c, r

    ping_off, mp_c, mp_r = _mp_render([basin_wall], False)
    ping_nowall, _, _ = _mp_render([], True)
    ping_on, _, _ = _mp_render([basin_wall], True)
    mp_bin = mp_c.acquisition.bin_size_m
    sensor_mp = mp_r._sensor_pose(Side.PORT, veh)   # noqa: SLF001 -- the
    # mount offset is exactly what the hand computation must not forget

    # ---- direct wall echo + occlusion (closes the former assumption A9) ----
    # A quay's wet face returns a bright coherent line at its own horizontal
    # distance and hides every seabed sample behind it -- the bright band
    # followed by a dark region that every real harbour mosaic shows.
    d_face = W_Y - sensor_mp.y                       # 5.8 m with the mount
    r_foot = float(np.hypot(d_face, sensor_mp.z - W_DEPTH))
    b_face = int(d_face / mp_bin)
    echo_nowall, _, _ = _mp_render([], False, echo=True)
    echo_on, _, _ = _mp_render([basin_wall], False, echo=True)
    check("wall echo is a strict no-op without walls",
          np.array_equal(echo_nowall.ping.power, ping_nowall.ping.power)
          and np.array_equal(echo_nowall.ping.specular,
                             ping_nowall.ping.specular),
          "power and specular bit-identical with the feature on and off")
    fbr_end = int((mp_r._sensor_pose(Side.PORT, veh).z - W_DEPTH + 0.6)  # noqa: SLF001
                  / mp_bin)
    spec_on, spec_off = echo_on.ping.specular, ping_off.ping.specular
    peak_bin = fbr_end + int(np.argmax(spec_on[fbr_end:]))
    check("the wall face echoes on the coherent channel at its own range",
          abs(peak_bin - b_face) <= 2
          and spec_on[b_face] > 5.0 * max(spec_off[b_face], 1e-30),
          f"peak bin {peak_bin} ({peak_bin*mp_bin:.2f} m) vs face at "
          f"{d_face:.2f} m; {10*np.log10(spec_on[b_face]/max(spec_off[b_face], 1e-30)):.1f} dB "
          "over the wall-free ping")
    b_shadow = int((r_foot + 0.4) / mp_bin)
    check("the seabed behind the wall is occluded",
          float(echo_on.ping.power[b_shadow:].max()) == 0.0
          and float(ping_nowall.ping.power[b_shadow:].min()) > 0.0,
          f"no direct return past {b_shadow*mp_bin:.2f} m with the wall, "
          "seabed everywhere without it")
    echo_obl, obl_c, obl_r = _mp_render(
        [basin_wall], False, echo=True,
        pose=Pose3D(x=0.0, y=4.0, z=0.0, yaw=np.radians(50.0)))
    s_obl = obl_r._sensor_pose(Side.PORT, echo_obl.ping.pose)  # noqa: SLF001
    d_obl = (W_Y - s_obl.y) / np.cos(np.radians(50.0))
    b_obl = int(d_obl / mp_bin)
    lvl_b = spec_on[b_face - 2:b_face + 3].max() / float(
        net_range_response(np.array([d_face]), mp_c.model)[0])
    lvl_o = echo_obl.ping.specular[b_obl - 2:b_obl + 3].max() / float(
        net_range_response(np.array([d_obl]), obl_c.model)[0])
    check("the wall echo falls with obliquity (cos^2 law)",
          10 * np.log10(lvl_b / max(lvl_o, 1e-30)) > 2.0,
          f"{10*np.log10(lvl_b/max(lvl_o, 1e-30)):.1f} dB dimmer at 50 deg "
          "off broadside, range law removed")
    wall_ct = [c for c in echo_on.contacts if c.object_type == "wall"]
    check("the wall face carries its own ground-truth contact",
          len(wall_ct) == 1 and wall_ct[0].wall_name == "quay"
          and abs(wall_ct[0].slant_range_m - d_face) < 1e-6
          and wall_ct[0].object_id < 0,
          f"{len(wall_ct)} wall contact(s): "
          + ", ".join(f"{c.wall_name} at {c.slant_range_m:.2f} m" for c in wall_ct))
    rg_w = np.random.default_rng(3)
    bins_w = (np.arange(mp_c.acquisition.num_results) + 0.5) * mp_bin
    curves = []
    for k in range(20):
        pw = obl_r.__class__(_flat_basin([basin_wall]), mp_c.acquisition,
                             mp_c.model).render(
            Side.PORT, Pose3D(0.05 * k, 4.0, 0.0, yaw=0.0), 0.0).ping
        pw.power = apply_ping_noise(pw.power, bins_w < pw.altitude_m, 1.0,
                                    mp_c.model, rg_w, specular=pw.specular,
                                    texture=pw.texture)
        curves.append(10 * np.log10(pw.power) + mp_c.model.calibration_db_offset)
    lock_w = _processor_lock_bin(np.mean(curves, axis=0))
    check("bottom tracking still locks at the altitude with a quay in view",
          abs(lock_w * mp_bin - (sensor_mp.z - W_DEPTH)) < 0.25,
          f"lock {lock_w*mp_bin:.2f} m vs altitude {sensor_mp.z - W_DEPTH:.2f} m")

    # ---- the SHIPPED combination: face echo + occlusion + ghosts together ----
    # Until 2026-09-03 the two features were only ever tested apart, and the
    # wall's own mirror ghost (gain 0.65 on power, applied once) repainted the
    # occluded band with a -1.9 dB copy of the seabed: walls "did not block".
    # Anchored on the fullHarbourCleanExample quay (analysis/wall_contrast):
    # face ~+21 dB over the seabed just before it, 1.5-4 m beyond ~-12 dB,
    # 6-12 m beyond ~-23 dB. Levels are compared TVG-flattened against the
    # wall-free ping at the SAME slant range.
    both_on, both_c, _ = _mp_render([basin_wall], True, echo=True)
    _flat = lambda rp: ((rp.ping.power + rp.ping.specular)  # noqa: E731
                        / net_range_response(bins_w, both_c.model))
    _bdb = lambda arr, a, b: float(10 * np.log10(  # noqa: E731
        arr[(bins_w >= a) & (bins_w < b)].mean() + 1e-30))
    ref_f, tot_f = _flat(ping_nowall), _flat(both_on)
    face_sel = (bins_w > d_face - 0.2) & (bins_w < r_foot + 0.3)
    face_db = 10 * np.log10(tot_f[face_sel].max() / ref_f[face_sel].max())
    beyond_db = _bdb(tot_f, r_foot + 1.5, r_foot + 4.0) - _bdb(ref_f, r_foot + 1.5, r_foot + 4.0)
    far_db = _bdb(tot_f, 12.0, 15.0) - _bdb(ref_f, 12.0, 15.0)
    check("shipped wall model: bright face, dark zone behind, faint far ghosts",
          12.0 < face_db < 30.0 and beyond_db < -8.0 and far_db < -15.0
          and beyond_db > far_db,
          f"face {face_db:+.1f} dB, 1.5-4 m beyond {beyond_db:+.1f} dB, "
          f"far {far_db:+.1f} dB vs the wall-free seabed at equal slant")
    # Two bounces: a wall of reflectivity 1 and no roughness mirrors the
    # seabed at full strength; the shipped 0.65 mm-rough concrete does not.
    smooth = Wall(name="mirror", x0=-60.0, y0=W_Y, x1=60.0, y1=W_Y, top_z=1.0,
                  reflectivity=1.0, roughness_m=0.0)
    smooth_on, _, _ = _mp_render([smooth], True, echo=False)
    smooth_far = (_bdb(_flat(smooth_on), 12.0, 15.0)
                  - _bdb(ref_f, 12.0, 15.0))
    check("ghost gain follows reflectivity^2 x the Rayleigh roughness law",
          smooth_far > far_db + 6.0 and
          abs(mirror_sources([smooth], sensor_mp, Side.PORT, 15.0, wall_gain=1.0,
                             surface_enabled=False, surface_reflectivity=0.9
                             )[0].gain - 1.0) < 1e-12 and
          abs(mirror_sources([basin_wall], sensor_mp, Side.PORT, 15.0,
                             wall_gain=1.0, surface_enabled=False,
                             surface_reflectivity=0.9)[0].gain - 0.65 ** 2) < 1e-12,
          f"perfect mirror far ghost {smooth_far:+.1f} dB vs concrete {far_db:+.1f} dB")

    # ---- pontoon: floating, looked under -- the structure a boat can cross ----
    pont = Wall(name="pont", x0=-60.0, y0=W_Y, x1=60.0, y1=W_Y, top_z=0.5,
                reflectivity=0.4, kind="pontoon", draft_m=0.4)
    pont_on, _, _ = _mp_render([pont], True, echo=True)
    pont_f = _flat(pont_on)
    pont_beyond = _bdb(pont_f, r_foot + 1.5, r_foot + 4.0) - _bdb(ref_f, r_foot + 1.5, r_foot + 4.0)
    pont_face = _bdb(pont_f, d_face - 0.1, d_face + 0.4) - _bdb(ref_f, d_face - 0.1, d_face + 0.4)
    check("a pontoon casts no shadow and shows only a faint edge",
          abs(pont_beyond) < 1.0 and 0.5 < pont_face < 12.0
          and float(np.min(pont_on.ping.power[b_shadow:])) > 0.0,
          f"seabed beyond {pont_beyond:+.1f} dB (unchanged), edge {pont_face:+.1f} dB")
    # Crossing the line: the transducer 0.3 m from a pontoon sees no slab in
    # its water column (the range law is floored at the altitude) and the
    # first bottom return stays at the altitude; a quay this close would
    # light the whole column with its face.
    cross_pose = Pose3D(x=0.0, y=W_Y - 0.5, z=0.0, yaw=0.0)
    cross_p, _, _ = _mp_render([pont], True, echo=True, pose=cross_pose)
    cross_q, _, _ = _mp_render([basin_wall], True, echo=True, pose=cross_pose)
    cross_0, _, _ = _mp_render([], True, echo=True, pose=cross_pose)
    wc = (bins_w > 1.0) & (bins_w < 3.5)          # past the transmit ringing
    _tot = lambda rp: (rp.ping.power + rp.ping.specular)  # noqa: E731
    col_p = float(np.abs(_tot(cross_p)[wc] - _tot(cross_0)[wc]).max())
    col_q = float((_tot(cross_q)[wc] / np.maximum(_tot(cross_0)[wc], 1e-30)).min())
    check("crossing a pontoon line leaves the water column dark; a quay would not",
          col_p < 1e-12 and col_q > 100.0
          and abs(cross_p.ping.altitude_m - 3.85) < 0.01,
          f"pontoon column identical to open water (max diff {col_p:.1e}); "
          f"a quay 0.3 m away lights it x{col_q:.0f}")
    check("pontoon geometry is declared, validated and written for Gazebo",
          pont.z_bottom == -0.4 and basin_wall.z_bottom == -np.inf
          and "wall_pont" in write_world_sdf.__module__ + "wall_pont"
          and Wall(name="x", x0=0, y0=0, x1=1, y1=0).kind == "quay",
          "z_bottom per kind; quay default")

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
    # Finite-segment cull: a short chord far along the wall's plane is
    # skipped even though its infinite plane is within range (the cost
    # lever for curved quays flattened into many chords).
    far_chord = Wall(name="chord", x0=200.0, y0=W_Y, x1=203.0, y1=W_Y,
                     reflectivity=0.65)
    near_chord = Wall(name="near", x0=-2.0, y0=W_Y, x1=2.0, y1=W_Y,
                      reflectivity=0.65)
    culled_seg = mirror_sources([far_chord, near_chord], sensor_mp, Side.PORT,
                                15.0, wall_gain=1.0, surface_enabled=False,
                                surface_reflectivity=0.9)
    check("mirror sources: a far chord on an in-range plane is culled, a near one kept",
          [m.wall.name for m in culled_seg] == ["near"])
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
    w_mp = mp_r.smear_bins()      # pulse c*tau/2, floored at range_filter_bins
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
    basin = OUT / "basin_world"
    scene_b = legacy_world(PKG_ROOT / "config" / "enclosed_basin_world.yaml",
                           basin)
    cfg_b = SonarConfig.from_yaml(PKG_ROOT / "config"
                                  / "enclosed_basin_sonar.yaml")
    check("the shipped basin config carries its walls and turns the model on",
          len(scene_b.walls) == 3 and cfg_b.model.wall_multipath_enabled
          and {w.name for w in scene_b.walls} == {"quay_north", "quay_south",
                                                  "pontoon_east"}
          and "wall_quay_north" in (basin / "world.sdf").read_text(),
          f"{len(scene_b.walls)} walls in the manifest, the same three in "
          f"world.sdf, wall_multipath_enabled=True in the basin profile")

    # A world written before walls existed must still load: the manifest
    # version does not move for an additive key.
    legacy = SceneModel.load(world_dir)
    check("a wall-free world still loads at the same manifest version",
          legacy.walls == [] and len(legacy.objects) > 0,
          "scene_manifest.yaml without a walls: key reads as open water")

    acq_b, model_b = cfg_b.acquisition, cfg_b.model
    rend_b = GeometricRenderer(scene_b, acq_b, model_b)
    enc_b = PingEncoder(Side.PORT, acq_b, model_b)
    traj_b = WaypointTrajectory.load_yaml(FIXTURES / "survey_lawnmower.yaml")
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
    # Measured on the lit bins only: with the direct wall echo on, a ping
    # looking at the quay has no seabed return behind it (the occlusion is
    # the point), and pooling those floor-level bins with lit ones would
    # report a mixture, not the speckle statistic. Per ping, bins within
    # 15 dB of that ping's band median; median CV across pings.
    far_b = power_b[:, 400:590]
    _cvs = []
    for _row in far_b:
        _med = float(np.median(_row))
        _lit = _row[_row > _med * 10 ** (-1.5)]
        if _lit.size > 50:
            _cvs.append(float(_lit.std() / max(_lit.mean(), 1e-30)))
    cv_b = float(np.median(_cvs))
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

    # ---- 6. GPS anchoring --------------------------------------------------
    print("[6] geo: anchor conversions, manifest block, spherical coords")
    import math

    anchor = GeoAnchor(lat0=33.930824, lon0=130.7311812)  # wakamatsu_beach
    pts = [(0.0, 0.0), (25.0, -40.0), (-160.5, 92.25), (1000.0, 1000.0)]
    max_rt = 0.0
    for e, n in pts:
        lat, lon = local_en_to_latlon(e, n, anchor.lat0, anchor.lon0)
        e2, n2 = latlon_to_local_en(lat, lon, anchor.lat0, anchor.lon0)
        max_rt = max(max_rt, abs(float(e2) - e), abs(float(n2) - n))
    check("equirect EN->latlon->EN is an exact inverse", max_rt < 1e-9,
          f"max round-trip error {max_rt:.2e} m")

    # Same math as mavros_shim_node._maybe_publish_fix, character for
    # character -- the shim, MCS and GCS all share this convention.
    e, n = 137.25, -83.5
    lat_shim = anchor.lat0 + math.degrees(n / EARTH_RADIUS_M)
    lon_shim = anchor.lon0 + math.degrees(
        e / (EARTH_RADIUS_M * math.cos(math.radians(anchor.lat0))))
    lat_g, lon_g = anchor.to_latlon(e, n)
    check("core.geo matches the shim/MCS formula exactly",
          float(lat_g) == lat_shim and float(lon_g) == lon_shim)

    try:
        GeoAnchor(lat0=1.0, lon0=2.0, theta_deg=15.0)
        rotated_ok = False
    except ValueError:
        rotated_ok = True
    check("rotated anchor (theta_deg != 0) is rejected", rotated_ok)

    tiny_grid = GridSpec(origin_x=-5.0, origin_y=-5.0, resolution=0.5,
                         nx=21, ny=21)
    tiny = SceneModel(
        grid=tiny_grid,
        height=np.full((21, 21), -3.0),
        reflectivity=np.full((21, 21), 0.5),
        material_id=np.zeros((21, 21), dtype=np.uint8),
        material_names=["sand"], objects=[], walls=[],
        seed=1, config_snapshot={}, geo_anchor=anchor)
    tiny.save(OUT / "geo_world")
    tiny_back = SceneModel.load(OUT / "geo_world")
    check("manifest geo block round-trips (version still 1)",
          tiny_back.geo_anchor == anchor)
    write_world_sdf(tiny, OUT / "geo_world")
    geo_sdf = (OUT / "geo_world" / "world.sdf").read_text()
    check("anchored world.sdf carries <spherical_coordinates>",
          "<spherical_coordinates>" in geo_sdf
          and "33.930824" in geo_sdf and "130.731181" in geo_sdf
          and "EARTH_WGS84" in geo_sdf)
    check("unanchored scene emits no spherical block",
          "<spherical_coordinates>" not in sdf)

    # Trajectory schemas: MCS blueboat_trajectory/1 next to the sim's own.
    mcs_doc = {"format": "blueboat_trajectory/1", "name": "t", "loop": False,
               "points": [[0.0, 0.0, 0.0, 3.1], [10.0, 20.0, 0.0, -3.1],
                          [20.0, 20.0, 10.0, -3.1]]}
    import yaml as _yaml
    (OUT / "mcs_traj.yaml").write_text(_yaml.safe_dump(mcs_doc))
    tt = load_trajectory(OUT / "mcs_traj.yaml")
    check("load_trajectory dispatches points -> TimedTrajectory",
          isinstance(tt, TimedTrajectory))
    x_m, y_m, yaw_m = tt.pose_at(5.0)
    check("TimedTrajectory interpolates position on t",
          abs(x_m - 10.0) < 1e-9 and abs(y_m) < 1e-9,
          f"pose_at(5) = ({x_m:.2f}, {y_m:.2f})")
    check("TimedTrajectory yaw interpolation crosses +/-pi the short way",
          abs(yaw_m) > 3.0, f"yaw {yaw_m:.3f}")
    xe, ye, _ = tt.pose_at(999.0)
    check("TimedTrajectory clamps at the last sample",
          abs(xe - 20.0) < 1e-9 and abs(ye - 10.0) < 1e-9)
    check("TimedTrajectory duration/length from samples",
          abs(tt.duration - 20.0) < 1e-9 and abs(tt.total_length - 30.0) < 1e-9)
    wt = load_trajectory(FIXTURES / "survey_lawnmower.yaml")
    check("load_trajectory dispatches waypoints -> WaypointTrajectory",
          isinstance(wt, WaypointTrajectory))

    # ---- 7. Authored bathymetry -------------------------------------------
    print("[7] depth: file ingestion, wall-aware interpolation, presets")
    from blueboat_sss_sim.worldgen.depth import (DepthConfig, build_height,
                                                 build_height_with_info,
                                                 DEPTH_PRESETS,
                                                 interpolate_to_grid)
    from blueboat_sss_sim.worldgen.depth.ingest import load_csv, load_geotiff
    from blueboat_sss_sim.worldgen.materials import MaterialLibrary as _MatLib

    for m in ("glass", "wood", "eps", "tinplate"):
        check(f"material '{m}' registered", _MatLib().get(m).reflectivity > 0)

    d_grid = GridSpec(origin_x=-10.0, origin_y=-10.0, resolution=0.5,
                      nx=41, ny=41)
    rng7 = np.random.default_rng(7)

    def plane_depth(e, n):
        return 3.0 + 0.05 * np.asarray(e) - 0.02 * np.asarray(n)

    # CSV round trip against an analytic plane (headered, shuffled columns).
    en = rng7.uniform(-12.0, 12.0, size=(220, 2))
    lat_s, lon_s = anchor.to_latlon(en[:, 0], en[:, 1])
    csv_path = OUT / "depth_plane.csv"
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("depth,latitude,longitude\n")
        for d, la, lo in zip(plane_depth(en[:, 0], en[:, 1]), lat_s, lon_s):
            f.write(f"{d:.6f},{la:.10f},{lo:.10f}\n")
    h_csv = interpolate_to_grid(load_csv(csv_path, anchor), d_grid, [])
    xs7 = d_grid.origin_x + np.arange(d_grid.nx) * d_grid.resolution
    ys7 = d_grid.origin_y + np.arange(d_grid.ny) * d_grid.resolution
    xx7, yy7 = np.meshgrid(xs7, ys7)
    interior = (np.abs(xx7) <= 8.0) & (np.abs(yy7) <= 8.0)
    err = np.abs(-h_csv - plane_depth(xx7, yy7))[interior]
    check("CSV soundings reproduce an analytic plane (linear interp)",
          float(err.max()) < 0.05, f"max interior error {err.max():.4f} m")

    # Smoothing spline (the recording path): plane inside the soundings,
    # the local slope continued -- damped -- beyond them, texture scaled by
    # the measured residual and absent at the soundings themselves.
    from blueboat_sss_sim.worldgen.depth import interpolate_smooth_to_grid
    h_sm = interpolate_smooth_to_grid(en[:, 0], en[:, 1],
                                      plane_depth(en[:, 0], en[:, 1]),
                                      d_grid, [], texture_gain=0.0)
    err_sm = np.abs(-h_sm - plane_depth(xx7, yy7))
    check("smoothing spline reproduces the plane through the soundings",
          float(err_sm[interior].max()) < 0.05 and float(err_sm.max()) < 0.15,
          f"max interior error {err_sm[interior].max():.4f} m")
    g_ext = GridSpec(origin_x=-10.0, origin_y=-10.0, resolution=1.0,
                     nx=71, ny=21)
    rng_s = np.random.default_rng(11)
    e_h = rng_s.uniform(-10.0, 0.0, 300)
    n_h = rng_s.uniform(-10.0, 10.0, 300)
    z_h = 3.0 + 0.1 * e_h
    d_ext = -interpolate_smooth_to_grid(e_h, n_h, z_h, g_ext, [],
                                        texture_gain=0.0, trend_length_m=15.0)
    x_ext = g_ext.origin_x + np.arange(g_ext.nx)
    row_ext = d_ext[10, :]
    dev_far = float(row_ext[x_ext == 55.0][0] - row_ext[x_ext == 0.0][0])
    check("beyond the soundings the edge slope continues and saturates at "
          "L * slope (damped trend)",
          float(np.abs(row_ext[x_ext <= -1.0] - (3.0 + 0.1 * x_ext[x_ext <= -1.0])
                       ).max()) < 0.08
          and 0.6 * 1.5 <= dev_far <= 1.05 * 1.5
          and bool((np.diff(row_ext[x_ext >= 0.0]) >= -1e-6).all()),
          f"55 m past the edge: +{dev_far:.2f} m (saturation 1.50 m)")
    z_j = z_h + rng_s.normal(0.0, 0.2, 300)
    h_t0 = interpolate_smooth_to_grid(e_h, n_h, z_j, g_ext, [],
                                      texture_gain=0.0,
                                      rng=np.random.default_rng(1))
    h_t1 = interpolate_smooth_to_grid(e_h, n_h, z_j, g_ext, [],
                                      texture_gain=1.0,
                                      rng=np.random.default_rng(1))
    dz_t = h_t1 - h_t0
    far = np.zeros(dz_t.shape, dtype=bool)
    far[:, x_ext >= 8.0] = True
    tex_rms = float(np.sqrt(np.mean(np.square(dz_t[far]))))
    # Texture is exactly zero at the coarse cells holding soundings.
    cell_c = 4.0                                    # max(0.5, 4 * res)
    ci_c = np.round((e_h - g_ext.origin_x) / cell_c).astype(int)
    cj_c = np.round((n_h - g_ext.origin_y) / cell_c).astype(int)
    at_nodes = [abs(float(dz_t[int(j * cell_c), int(i * cell_c)]))
                for i, j in set(zip(ci_c, cj_c))
                if 0 <= i * cell_c < g_ext.nx and 0 <= j * cell_c < g_ext.ny]
    check("texture scales with the measured residual and vanishes at the "
          "soundings",
          0.5 * 0.2 < tex_rms < 1.6 * 0.2 and max(at_nodes) < 1e-9
          and float(np.abs(h_t0 - h_t0).max()) == 0.0,
          f"texture RMS {tex_rms:.3f} m for a 0.20 m residual")
    try:
        DepthConfig.from_dict({"mode": "svlog", "file": "x.svlog",
                               "svlog": {"bogus": 1}})
        svk_ok = False
    except KeyError as exc:
        svk_ok = "bogus" in str(exc)
    try:
        DepthConfig.from_dict({"mode": "svlog"})
        svf_ok = False
    except ValueError:
        svf_ok = True
    check("svlog depth config rejects unknown keys by name and needs a file",
          svk_ok and svf_ok)

    # ---- coverage: does the recording reach the world at all, and how far --
    # The [4c] recording (svlog_path) was sailed about `anchor`; the grid
    # below is the same frame, so a shifted grid is a shifted world.
    from blueboat_sss_sim.worldgen.depth.coverage import sample_coverage
    sv_cfg = DepthConfig.from_dict({"mode": "svlog", "file": str(svlog_path),
                                    "svlog": {"texture_gain": 0.0}})
    sb = surv.summary["extent_m"]                       # sounding box, metres
    sw = max(sb[2] - sb[0], 1.0), max(sb[3] - sb[1], 1.0)
    g_full = GridSpec(sb[0] - sw[0], sb[1] - sw[1], 0.5,
                      int(3 * sw[0] / 0.5) + 1, int(3 * sw[1] / 0.5) + 1)
    _, cov_info = build_height_with_info(g_full, sv_cfg, [], anchor,
                                         np.random.default_rng(0))
    cov_full = cov_info["coverage"]
    check("a recording that covers its world reports full coverage and no "
          "warning",
          cov_full["inside"] == cov_full["total"] and cov_full["total"] > 8
          and cov_full["inside_fraction"] == 1.0
          and cov_full["area_fraction"] > 0.9
          and "warning" not in cov_info,
          f"{cov_full['inside']}/{cov_full['total']} samples inside, "
          f"{100 * cov_full['area_fraction']:.0f} % of the area within "
          f"{cov_full['reach_m']:.0f} m of a sounding")

    # A world far larger than the survey: builds, but says so.
    g_wide = GridSpec(sb[0] - 100.0, sb[1] - 100.0, 1.0, 201, 201)
    _, wide_info = build_height_with_info(g_wide, sv_cfg, [], anchor,
                                          np.random.default_rng(0))
    cov_wide = wide_info["coverage"]
    check("a world far larger than the survey still builds and warns that "
          "the rest is extrapolated",
          cov_wide["inside_fraction"] == 1.0
          and cov_wide["area_fraction"] < 0.35
          and "extrapolated" in wide_info.get("warning", ""),
          f"{100 * cov_wide['area_fraction']:.0f} % of the area reached; "
          f"{wide_info.get('warning', '')[:70]}")

    # Half the track outside the limits: the sample share drops below one.
    g_half = GridSpec(sb[0] - sw[0], 0.5 * (sb[1] + sb[3]), 0.5,
                      int(3 * sw[0] / 0.5) + 1, int(1.5 * sw[1] / 0.5) + 1)
    cov_half = sample_coverage(surv.cells.east, surv.cells.north, g_half)
    check("a rectangle over part of the track counts only the samples it "
          "contains",
          0.0 < cov_half["inside_fraction"] < 1.0
          and cov_half["inside"] < cov_half["total"],
          f"{cov_half['inside']}/{cov_half['total']} samples inside")

    # 500 m away: refused, with both places, a distance and a compass point.
    g_away = GridSpec(sb[0] + 500.0, sb[1] + 500.0, 0.5, 81, 81)
    try:
        build_height_with_info(g_away, sv_cfg, [], anchor,
                               np.random.default_rng(0))
        away_msg = ""
    except ValueError as exc:
        away_msg = str(exc)
    cov_away = sample_coverage(surv.cells.east, surv.cells.north, g_away)
    check("a recording that never reaches the world is refused with the "
          "distance, the direction and both centres",
          cov_away["inside"] == 0
          and away_msg.startswith("no depth sample lands inside the world "
                                  "limits")
          and svlog_path.name in away_msg
          and re.search(r"\d+ m (N|NE|E|SE|S|SW|W|NW|NNE|ENE|ESE|SSE|SSW|"
                        r"WSW|WNW|NNW) of the world area", away_msg)
          and away_msg.count(" N, ") == 2
          and "Choose a recording of this area" in away_msg,
          away_msg[-120:])

    # A near miss is a rectangle problem, and the advice says so.
    g_near = GridSpec(sb[2] + 20.0, sb[1], 0.5, 81, 81)
    try:
        build_height_with_info(g_near, sv_cfg, [], anchor,
                               np.random.default_rng(0))
        near_msg = ""
    except ValueError as exc:
        near_msg = str(exc)
    check("a recording that passes just outside the limits is told to move "
          "the rectangle, not the file",
          "redraw the world limits" in near_msg
          and " m " in near_msg and "km" not in near_msg,
          near_msg[-110:])

    # A depth *file* far from the world gets the same treatment.
    try:
        build_height_with_info(
            GridSpec(5000.0, 5000.0, 1.0, 41, 41),
            DepthConfig.from_dict({"mode": "file", "file": str(csv_path)}),
            [], anchor, np.random.default_rng(0))
        file_msg = ""
    except ValueError as exc:
        file_msg = str(exc)
    check("a depth file outside the limits is refused with the same "
          "actionable message",
          "the file and the drawn rectangle do not overlap" in file_msg
          and "km" in file_msg and "depth file" in file_msg,
          file_msg[-100:])

    # GeoTIFF written in-test through PIL, elevation convention, auto-detect.
    from PIL import Image as _Image, TiffImagePlugin as _Tiff
    deg_e = 0.5 / (EARTH_RADIUS_M * math.cos(math.radians(anchor.lat0)))
    deg_n = 0.5 / EARTH_RADIUS_M
    sx_deg, sy_deg = math.degrees(deg_e), math.degrees(deg_n)
    tif_ny, tif_nx = 60, 60
    lon_nw = anchor.lon0 - (tif_nx / 2) * sx_deg
    lat_nw = anchor.lat0 + (tif_ny / 2) * sy_deg
    cols7 = np.arange(tif_nx)
    rows7 = np.arange(tif_ny)
    lon_g7 = lon_nw + cols7 * sx_deg
    lat_g7 = lat_nw - rows7 * sy_deg
    lon_m, lat_m = np.meshgrid(lon_g7, lat_g7)
    e_g7, n_g7 = anchor.to_en(lat_m, lon_m)
    elev = -plane_depth(e_g7, n_g7)          # elevation: negative below surface
    tif_path = OUT / "depth_plane.tif"

    def _write_tif(path, geokeys):
        info = _Tiff.ImageFileDirectory_v2()
        info[33550] = (sx_deg, sy_deg, 0.0)
        info.tagtype[33550] = 12                 # DOUBLE
        info[33922] = (0.0, 0.0, 0.0, lon_nw, lat_nw, 0.0)
        info.tagtype[33922] = 12
        info[34735] = geokeys
        info.tagtype[34735] = 3                  # SHORT
        _Image.fromarray(elev.astype(np.float32), mode="F").save(
            path, tiffinfo=info)

    _write_tif(tif_path, (1, 1, 0, 1, 1024, 0, 1, 2))   # geographic CRS
    h_tif = interpolate_to_grid(load_geotiff(tif_path, anchor), d_grid, [])
    err_t = np.abs(-h_tif - plane_depth(xx7, yy7))[interior]
    check("GeoTIFF raster round-trips (Pillow tags, elevation auto-negated)",
          float(err_t.max()) < 0.05, f"max interior error {err_t.max():.4f} m")

    _write_tif(OUT / "depth_utm.tif", (1, 1, 0, 1, 1024, 0, 1, 1))
    try:
        load_geotiff(OUT / "depth_utm.tif", anchor)
        utm_rejected = False
    except ValueError as exc:
        utm_rejected = "projected" in str(exc)
    check("projected-CRS GeoTIFF rejected with a clear message", utm_rejected)

    # Wall-aware fill: soundings on the two sides of a wall never blend.
    from blueboat_sss_sim.worldgen.depth.ingest import DepthSamples
    wall7 = Wall(name="divider", x0=0.0, y0=-11.0, x1=0.0, y1=11.0)
    left = rng7.uniform([-9.0, -9.0], [-3.0, 9.0], size=(25, 2))
    right = rng7.uniform([3.0, -9.0], [9.0, 9.0], size=(25, 2))
    two_sided = DepthSamples(
        np.concatenate([left[:, 0], right[:, 0]]),
        np.concatenate([left[:, 1], right[:, 1]]),
        np.concatenate([np.full(25, 2.0), np.full(25, 5.0)]))
    h_wall = interpolate_to_grid(two_sided, d_grid, [wall7])
    d_wall = -h_wall
    col_l = np.argmin(np.abs(xs7 - (-1.0)))
    col_r = np.argmin(np.abs(xs7 - 1.0))
    check("nearest-fill extends to the wall and never crosses it",
          np.allclose(d_wall[:, col_l], 2.0) and np.allclose(d_wall[:, col_r], 5.0),
          f"left of wall {d_wall[:, col_l].min():.2f}..{d_wall[:, col_l].max():.2f}, "
          f"right {d_wall[:, col_r].min():.2f}..{d_wall[:, col_r].max():.2f}")
    check("wall cells themselves still end finite", np.isfinite(h_wall).all())

    # Preset shapes.
    rng_p = np.random.default_rng(0)
    h_h = preset = DEPTH_PRESETS["harbour_basin"][0](
        d_grid, {"basin_depth": 2.5, "edge_shoal_m": 0.8, "shoal_width_m": 4.0},
        rng_p)
    centre = -h_h[20, 20]
    edge = -h_h[20, 0]
    check("harbour preset: flat dredged centre, shoaled edge",
          abs(centre - 2.5) < 1e-6 and abs(edge - 1.7) < 1e-6,
          f"centre {centre:.2f} m, edge {edge:.2f} m")
    h_r = DEPTH_PRESETS["river_channel"][0](
        d_grid, {"max_depth": 3.0, "bank_depth": 0.5, "thalweg_offset_frac": 0.3,
                 "pool_riffle_amplitude_m": 0.0}, rng_p)
    # Flow axis is +x (heading 0), so the cross-section varies with y.
    thal_row = int(np.argmax(-h_r[:, 20]))
    check("river preset: thalweg offset toward one bank",
          abs(ys7[thal_row] - 3.0) < 0.6
          and abs(-h_r[thal_row, 20] - 3.0) < 1e-6
          and -h_r[0, 20] < 1.2,
          f"thalweg at y={ys7[thal_row]:.1f} (expect ~3.0), depth "
          f"{-h_r[thal_row, 20]:.2f}, bank {-h_r[0, 20]:.2f}")
    h_b = DEPTH_PRESETS["beach_profile"][0](
        d_grid, {"dean_A": 0.10, "min_depth": 0.4}, rng_p)
    prof = -h_b[20, :]
    off7 = xs7 - xs7[0]
    dean_err = np.abs(prof - (0.4 + 0.10 * off7 ** (2.0 / 3.0)))
    check("beach preset follows the Dean x^(2/3) profile",
          float(dean_err.max()) < 1e-9 and bool(np.all(np.diff(prof) >= 0)),
          f"max deviation {dean_err.max():.2e}")
    h_c = DEPTH_PRESETS["canal"][0](
        d_grid, {"channel_depth": 2.0, "channel_width_m": 8.0,
                 "side_slope": 0.5, "bank_depth": 0.4}, rng_p)
    # Canal axis is +x (heading 0): flat at y=0, sloping toward the banks.
    check("canal preset: flat channel, sloped sides, floored banks",
          abs(-h_c[20, 20] - 2.0) < 1e-6 and abs(-h_c[0, 20] - 0.4) < 1e-6
          and 0.4 + 1e-6 < -h_c[8, 20] < 2.0 - 1e-6,
          f"centre {-h_c[20, 20]:.2f}, slope {-h_c[8, 20]:.2f}, "
          f"bank {-h_c[0, 20]:.2f}")
    try:
        DEPTH_PRESETS["canal"][0](d_grid, {"chanel_depth": 2.0}, rng_p)
        bad_key_ok = False
    except KeyError as exc:
        bad_key_ok = "chanel_depth" in str(exc)
    check("preset rejects unknown parameter keys by name", bad_key_ok)

    # New presets: shape assertions on the same 41x41 grid (centre (0,0)).
    rng_p2 = np.random.default_rng(0)
    h_dry = DEPTH_PRESETS["flat"][0](d_grid, {"depth": -1.0}, rng_p2)
    h_bd = DEPTH_PRESETS["beach_profile"][0](
        d_grid, {"dean_A": 0.10, "min_depth": -1.0}, rng_p)
    h_bb = DEPTH_PRESETS["beach_profile"][0](
        d_grid, {"dean_A": 0.10, "min_depth": -1.0, "bar_enabled": True,
                 "bar_height_m": 2.0, "bar_distance_m": 5.0,
                 "bar_width_m": 3.0}, rng_p)
    h_sd = DEPTH_PRESETS["sloped_plane"][0](
        d_grid, {"depth_centre": 0.5, "grade": 0.5, "min_depth": -2.0}, rng_p)
    check("presets accept depth <= 0: flat land, a dry beach edge that "
          "deepens offshore, a bar capped at the shoreline, a dry plateau",
          bool(np.allclose(h_dry, 1.0))
          and abs(float(h_bd[20, 0]) - 1.0) < 1e-9
          and bool((np.diff(h_bd[20, :]) <= 1e-9).all())
          and float(h_bb.max()) <= 1.0 + 1e-9
          and abs(float(h_sd[20, 0]) - 2.0) < 1e-9
          and float(h_sd[20, -1]) < -3.0,
          f"beach edge {h_bd[20, 0]:+.2f} m, bar max {h_bb.max():+.2f} m, "
          f"plateau {h_sd[20, 0]:+.2f} m")
    from blueboat_sss_sim.worldgen.sdf_writer import _limits_sdf
    sc_land = SceneModel(grid=d_grid, height=h_sd, reflectivity=np.full(
        h_sd.shape, 0.3), material_id=np.zeros(h_sd.shape, dtype=np.uint8),
        material_names=["sand"], objects=[], walls=[], seed=0,
        config_snapshot={})
    box_h = [float(m.group(3)) for m in re.finditer(
        r"<size>([-\d.]+) ([-\d.]+) ([-\d.]+)</size>", _limits_sdf(sc_land))]
    check("limit borders reach over dry land (box height spans min-0.5 "
          "to max+0.5)",
          len(box_h) == 4 and all(abs(h - (h_sd.max() + 0.5
                                            - (h_sd.min() - 0.5))) < 1e-3
                                  for h in box_h),
          f"box heights {box_h}")
    h_f = DEPTH_PRESETS["flat"][0](d_grid, {"depth": 3.5}, rng_p2)
    check("flat preset: constant depth", bool(np.allclose(-h_f, 3.5)))
    h_s = DEPTH_PRESETS["sloped_plane"][0](
        d_grid, {"depth_centre": 3.0, "grade": 0.05, "direction_deg": 0.0},
        rng_p2)
    check("sloped plane: linear grade along the direction, flat across",
          bool(np.allclose(-h_s[20, :], 3.0 + 0.05 * xs7))
          and bool(np.allclose(-h_s[:, 20], 3.0)))
    h_d = DEPTH_PRESETS["dredged_channel"][0](
        d_grid, {"basin_depth": 2.0, "channel_depth": 4.0,
                 "channel_width_m": 8.0, "channel_heading_deg": 0.0,
                 "scarp_smooth_m": 0.5, "shoal_height_m": 0.5,
                 "shoal_width_m": 2.0}, rng_p2)
    d_cut = -h_d[20, 20]
    d_shoal = -h_d[int(np.argmin(np.abs(ys7 - 6.0))), 20]
    d_far = -h_d[0, 20]
    check("dredged channel: deep cut, spoil shoal beside it, basin beyond",
          abs(d_cut - 4.0) < 1e-3 and 1.4 < d_shoal < 1.7
          and abs(d_far - 2.0) < 0.05,
          f"cut {d_cut:.2f}, shoal {d_shoal:.2f}, basin {d_far:.2f} m")
    h_m = DEPTH_PRESETS["marina"][0](
        d_grid, {"basin_depth": 2.5, "pocket_depth": 3.5,
                 "pontoon_pitch_m": 10.0, "pontoon_heading_deg": 0.0,
                 "pocket_width_frac": 0.6, "edge_smooth_m": 0.3,
                 "pile_pitch_m": 5.0, "pit_depth_m": 0.4,
                 "pit_radius_m": 1.0}, rng_p2)
    m_pocket, m_pile, m_line = -h_m[20, 20], -h_m[30, 20], -h_m[30, 25]
    check("marina: berth pocket between pontoons, scour pit at a pile",
          abs(m_pocket - 3.5) < 1e-3 and abs(m_pile - 2.9) < 0.02
          and abs(m_line - 2.5) < 0.02,
          f"pocket {m_pocket:.2f}, pile {m_pile:.2f}, pontoon line "
          f"{m_line:.2f} m")
    rock_params = {"base_depth": 4.0, "count": 5, "height_m": 1.0,
                   "radius_m": 2.0}
    h_r1 = DEPTH_PRESETS["rocky_outcrop"][0](
        d_grid, rock_params, np.random.default_rng([3, 0]))
    h_r2 = DEPTH_PRESETS["rocky_outcrop"][0](
        d_grid, rock_params, np.random.default_rng([3, 0]))
    check("rocky outcrops: seeded bumps over the base, deterministic",
          abs(float((-h_r1).max()) - 4.0) < 1e-3
          and float((-h_r1).min()) < 3.6
          and np.array_equal(h_r1, h_r2),
          f"shallowest rock top {(-h_r1).min():.2f} m")
    h_sb = DEPTH_PRESETS["sandbar_ripples"][0](
        d_grid, {"depth": 3.0, "bar_spacing_m": 10.0, "bar_height_m": 1.0,
                 "heading_deg": 0.0, "asymmetry": 0.0}, rng_p2)
    crest = -h_sb[int(np.argmin(np.abs(ys7 - 2.5))), 20]
    trough = -h_sb[int(np.argmin(np.abs(ys7 + 2.5))), 20]
    check("sandbar field: periodic crests/troughs about the mean depth",
          abs(crest - 2.5) < 1e-6 and abs(trough - 3.5) < 1e-6
          and abs(-h_sb[20, 20] - 3.0) < 1e-6)
    h_b2 = DEPTH_PRESETS["beach_profile"][0](
        d_grid, {"dean_A": 0.10, "min_depth": 0.4, "berm_enabled": True,
                 "berm_distance_m": 4.0, "berm_height_m": 0.2,
                 "berm_width_m": 1.0}, rng_p2)
    i_berm = int(np.argmin(np.abs(off7 - 4.0)))
    check("beach berm: shallower at the berm, Dean profile untouched elsewhere",
          (-h_b[20, i_berm]) - (-h_b2[20, i_berm]) > 0.15
          and abs(h_b2[20, -1] - h_b[20, -1]) < 1e-9)

    # Terrain schema: unknown-key guard + the builder's flat/nested view.
    from blueboat_sss_sim.worldgen.terrain import (
        TerrainConfig, flatten_texture, synthesize_terrain,
        texture_defaults, unflatten_texture)
    try:
        TerrainConfig.from_dict({"dunes": {"enabeld": True}})
        tguard = False
    except KeyError as exc:
        tguard = "enabeld" in str(exc)
    for cfg_name in ("default_world.yaml", "enclosed_basin_world.yaml",
                     "shallow_water_world.yaml"):
        WorldConfig.from_yaml(PKG_ROOT / "config" / cfg_name)
    check("terrain config rejects unknown keys by name; shipped configs pass",
          tguard)
    legacy_tex = {"dunes": {"enabled": False}, "roughness": {"amplitude": 0.03}}
    flat_tex = flatten_texture(legacy_tex)
    check("texture schema: legacy state fills from defaults, round-trips",
          flat_tex["dunes.enabled"] is False
          and flat_tex["roughness.amplitude"] == 0.03
          and flat_tex["dunes.wavelength"] == 6.0
          and unflatten_texture(flatten_texture(texture_defaults()))
          == texture_defaults()
          and TerrainConfig.from_dict(texture_defaults()).material_layout
          == "uniform")

    # Depth painting (pure, RNG-free).
    from blueboat_sss_sim.worldgen.depth import (apply_depth_paint,
                                                 strokes_to_samples)
    stroke_line = {"points": [[0.0, 0.0], [4.0, 0.0]], "radius_m": 2.0,
                   "value": 5.0}
    smp = strokes_to_samples([stroke_line])
    check("strokes_to_samples: centre + 8 ring samples every radius/2",
          len(smp) == 45 and bool(np.allclose(smp.depth, 5.0))
          and abs(float(np.hypot(smp.east[5:13], smp.north[5:13]).max())
                  - 2.0) < 1e-9)
    smp_e = strokes_to_samples([stroke_line, {"points": [[4.0, 0.0]],
                                              "radius_m": 2.5,
                                              "erase": True}])
    check("eraser stroke drops the samples under it",
          0 < len(smp_e) < 45
          and bool((np.hypot(smp_e.east - 4.0, smp_e.north) > 2.5).all()))
    flat_h = np.full((41, 41), -3.0)
    stroke6 = {"points": [[0.0, 0.0]], "radius_m": 3.0, "value": 6.0}
    dp = -apply_depth_paint(flat_h, d_grid, [stroke6], [])
    rr7 = np.hypot(xx7, yy7)
    feather = dp[(rr7 > 3.0) & (rr7 < 4.5)]
    # Default feather = R/2 = 1.5 m: at r = 3.5 (0.5 m into it) the C1
    # smoothstep sits at 0.74 -> 3 + 3 * 0.74 = 5.22 m.
    d_mid = dp[20, int(np.argmin(np.abs(xs7 - 3.5)))]
    ray = dp[20, xs7 >= 0.0]
    check("depth paint: painted depth in the disc, C1 feather over R/2, "
          "base beyond, monotone",
          bool(np.allclose(dp[rr7 <= 3.0], 6.0)) and 4.9 < d_mid < 5.5
          and bool(np.allclose(dp[rr7 >= 4.6], 3.0))
          and bool((feather <= 6.0 + 1e-9).all())
          and bool((feather >= 3.0 - 1e-9).all())
          and bool((np.diff(ray) <= 1e-9).all()),
          f"at R + 0.5 m {d_mid:.2f} m (expect ~5.2)")
    # (The feather is floored at two cells -- 1.0 m on this 0.5 m fixture.)
    dp_f = -apply_depth_paint(flat_h, d_grid,
                              [dict(stroke6, feather_m=1.0)], [])
    check("a stroke's own feather_m narrows the blend",
          bool(np.allclose(dp_f[rr7 <= 3.0], 6.0))
          and bool(np.allclose(dp_f[rr7 >= 4.1], 3.0))
          and int(((dp > 3.0 + 1e-9) & (rr7 >= 4.1)).sum()) > 0
          and int(((dp_f > 3.0) & (rr7 > 3.0)).sum()) > 0)
    # Two overlapping strokes of different depth: no ridge in the shared
    # feather zone (a nearest-stroke blend would leave a Voronoi crease).
    fine = GridSpec(origin_x=-5.0, origin_y=-5.0, resolution=0.1,
                    nx=101, ny=101)
    dp2 = -apply_depth_paint(
        np.full((101, 101), -3.0), fine,
        [{"points": [[0.0, 0.0]], "radius_m": 2.0, "value": 6.0,
          "feather_m": 1.5},
         {"points": [[2.0, 0.0]], "radius_m": 2.0, "value": 5.0,
          "feather_m": 1.5}], [])
    row_f = dp2[25, :]                       # y = -2.5: feather zone only
    d2 = np.abs(np.diff(row_f, 2))
    check("overlapping strokes blend without a ridge (second difference "
          "bounded on a 0.1 m grid)",
          float(d2.max()) < 0.15 and float(dp2.min()) >= 3.0 - 1e-9
          and float(dp2.max()) <= 6.0 + 1e-9
          and abs(float(dp2[50, 50]) - 5.0) < 1e-9,
          f"max |d2| {d2.max():.3f} m")
    wall_x4 = Wall(name="x4", x0=4.0, y0=-11.0, x1=4.0, y1=11.0)
    dp_w = -apply_depth_paint(flat_h, d_grid, [stroke6], [wall_x4])
    check("depth paint never feathers across a wall",
          bool(np.allclose(dp_w[:, xs7 > 4.3], 3.0))
          and int((dp_w[:, xs7 < 3.7] != 3.0).sum()) > 0)
    dp_e = -apply_depth_paint(
        flat_h, d_grid, [stroke6, {"points": [[0.0, 0.0]], "radius_m": 4.0,
                                   "erase": True}], [])
    check("an eraser stroke over the paint restores the base exactly",
          np.array_equal(dp_e, -flat_h))
    try:
        apply_depth_paint(flat_h, d_grid,
                          [{"points": [[0.0, 0.0]], "radius": 1.0,
                            "value": 2.0}], [])
        skey_ok = False
    except KeyError as exc:
        skey_ok = "radius" in str(exc)
    check("strokes reject unknown keys by name", skey_ok)
    try:
        synthesize_terrain(d_grid, TerrainConfig.from_dict({
            "materials": {"layout": "painted", "paint": [
                {"points": [[0.0, 0.0]], "radius_m": 1.0,
                 "value": "kryptonite"}]}}), _MatLib(),
            np.random.default_rng(0))
        mat_ok = False
    except KeyError as exc:
        mat_ok = "kryptonite" in str(exc)
    check("material paint rejects unknown material names by name", mat_ok)

    # Painted world through the headless CLI path: reproducible, and the
    # strokes land where the YAML says.
    from blueboat_sss_sim.worldgen.generate import generate_world
    paint_yaml = OUT / "painted_world.yaml"
    paint_yaml.write_text(_y1.safe_dump({
        "world": {"seed": 5, "size": [30.0, 20.0], "origin": [-15.0, -10.0],
                  "resolution": 0.5},
        "geo": {"lat0": anchor.lat0, "lon0": anchor.lon0},
        "depth": {"mode": "preset", "preset": "flat",
                  "params": {"depth": 3.0},
                  "paint": [{"points": [[-5.0, 0.0], [5.0, 0.0]],
                             "radius_m": 2.0, "value": 5.0,
                             "erase": False}]},
        "terrain": {"dunes": {"enabled": False},
                    "roughness": {"amplitude": 0.0},
                    "materials": {"layout": "painted", "uniform": "sand",
                                  "paint": [{"points": [[8.0, 5.0]],
                                             "radius_m": 3.0,
                                             "value": "mud",
                                             "erase": False}]}},
        "objects": {"density_per_hectare": 0.0}}))
    sc_p1 = generate_world(paint_yaml, OUT / "painted_w1")
    sc_p2 = generate_world(paint_yaml, OUT / "painted_w2")

    def _cell(sc, x, y):
        ci_, cj_ = sc.grid.world_to_grid(x, y)
        return int(round(float(cj_))), int(round(float(ci_)))

    c_tr, c_mud, c_far = (_cell(sc_p1, 0.0, 0.0), _cell(sc_p1, 8.0, 5.0),
                          _cell(sc_p1, -12.0, -8.0))
    check("painted world reproduces headlessly, bit-identical twice",
          np.array_equal(sc_p1.height, sc_p2.height)
          and np.array_equal(sc_p1.material_id, sc_p2.material_id)
          and abs(-sc_p1.height[c_tr] - 5.0) < 0.05
          and abs(-sc_p1.height[c_far] - 3.0) < 0.05,
          f"trench {-sc_p1.height[c_tr]:.2f} m, far {-sc_p1.height[c_far]:.2f} m")
    check("material paint: stamped cells carry the index, base untouched",
          sc_p1.material_names == ["sand", "mud"]
          and int(sc_p1.material_id[c_mud]) == 1
          and int(sc_p1.material_id[c_far]) == 0
          and float(sc_p1.reflectivity[c_mud]) < float(sc_p1.reflectivity[c_far]))

    # generate_scene: depth section drives the base; objects immune to it.
    wc_a = WorldConfig.from_dict({
        "world": {"seed": 11, "size": [40.0, 30.0], "origin": [-20.0, -15.0]},
        "geo": {"lat0": anchor.lat0, "lon0": anchor.lon0},
        "depth": {"mode": "preset", "preset": "harbour_basin",
                  "params": {"basin_depth": 2.0}},
        "terrain": {"dunes": {"enabled": False}, "roughness": {"amplitude": 0.0}},
        "objects": {"density_per_hectare": 40.0}})
    wc_b = WorldConfig.from_dict({
        "world": {"seed": 11, "size": [40.0, 30.0], "origin": [-20.0, -15.0]},
        "geo": {"lat0": anchor.lat0, "lon0": anchor.lon0},
        "depth": {"mode": "preset", "preset": "canal", "params": {}},
        "terrain": {"dunes": {"enabled": False}, "roughness": {"amplitude": 0.0}},
        "objects": {"density_per_hectare": 40.0}})
    sc_a, sc_b = generate_scene(wc_a), generate_scene(wc_b)
    same_obj = (len(sc_a.objects) == len(sc_b.objects) and all(
        oa.type == ob.type and oa.x == ob.x and oa.y == ob.y
        and oa.length == ob.length and oa.burial == ob.burial
        for oa, ob in zip(sc_a.objects, sc_b.objects)))
    check("changing the depth pattern never moves the objects (split RNG)",
          same_obj and len(sc_a.objects) > 0,
          f"{len(sc_a.objects)} objects identical across two depth modes")
    check("depth-authored scene carries its anchor into the manifest",
          sc_a.geo_anchor == anchor
          or (sc_a.geo_anchor.lat0 == anchor.lat0
              and sc_a.geo_anchor.lon0 == anchor.lon0))
    check("preset base survives into the scene rasters",
          abs(float(np.median(sc_a.height)) - (-2.0)) < 0.15,
          f"median height {np.median(sc_a.height):.2f} (expect ~-2.0)")

    # ---- 8. Object catalog v2 ---------------------------------------------
    print("[8] objects v2: new families, clustering, fill-fraction, limits")
    from blueboat_sss_sim.worldgen.objects import (CATALOG,  # noqa: E402
                                                   DEFAULT_COMPOSITION,
                                                   ObjectFieldConfig,
                                                   place_objects,
                                                   rasterize_objects)

    new_types = ["crate_plastic", "tin_18l", "polytank", "drum_200l",
                 "pail_20l", "fire_extinguisher", "pallet_wood",
                 "traffic_cone", "rope_coil", "net_ball", "bottle_glass",
                 "bicycle", "mesh_crate", "shopping_trolley"]
    check("all target-family classes registered (27 total)",
          all(t in CATALOG for t in new_types) and len(CATALOG) == 27,
          f"{len(CATALOG)} classes")
    check("legacy composition untouched by the additions",
          set(DEFAULT_COMPOSITION) == {
              "tire_car", "tire_bicycle", "pipe_pvc", "bottle_plastic", "can",
              "tent_weight", "rope", "cylinder_metal", "concrete_block",
              "brick", "chain", "anchor", "debris"})

    lib8 = _MatLib()
    grid8 = GridSpec(origin_x=-30.0, origin_y=-20.0, resolution=0.1,
                     nx=601, ny=401)
    cfg8 = ObjectFieldConfig.from_dict({
        "min_separation_m": 1.0,
        "classes": {
            "drum_200l": {"amount": {"kind": "count", "value": 6}},
            "mesh_crate": {"amount": {"kind": "count", "value": 5},
                           "size": {"kind": "gauss", "mean": 1.0,
                                    "sigma": 0.05}},
            "traffic_cone": {"amount": {"kind": "count", "value": 4}},
            "rope_coil": {"amount": {"kind": "count", "value": 4},
                          "burial": {"kind": "uniform", "lo": 0.0, "hi": 0.2}},
            "bottle_glass": {"amount": {"kind": "density", "value": 12.5}},
        },
        "explicit": [
            {"type": "crate_plastic", "x": 5.0, "y": 5.0, "yaw": 0.3},
            {"type": "mesh_crate", "x": -5.0, "y": 5.0, "length": 0.5,
             "width": 0.36, "proud_height": 0.3, "fill_fraction": 0.35,
             "burial": 0.0},
        ]})
    objs1 = place_objects(grid8, cfg8, lib8, np.random.default_rng(3))
    objs2 = place_objects(grid8, cfg8, lib8, np.random.default_rng(3))
    check("an empty class selection places nothing; a config without the "
          "key keeps the legacy random field",
          place_objects(grid8, ObjectFieldConfig.from_dict({"classes": {}}),
                        lib8, np.random.default_rng(3)) == []
          and len(place_objects(grid8, ObjectFieldConfig.from_dict(
              {"density_per_hectare": 40.0}), lib8,
              np.random.default_rng(3))) > 0)
    h_wet = np.full((401, 601), -3.0)
    h_half = h_wet.copy()
    h_half[:, 301:] = 0.5                      # x > 0 is dry land
    objs_wet = place_objects(grid8, cfg8, lib8, np.random.default_rng(3),
                             height=h_wet)
    objs_half = place_objects(grid8, cfg8, lib8, np.random.default_rng(3),
                              height=h_half)
    explicit_xy = {(5.0, 5.0), (-5.0, 5.0)}
    rand_half = [o for o in objs_half if (o.x, o.y) not in explicit_xy]
    check("random objects stay off dry cells; a fully wet raster places "
          "bit-identically to no raster",
          len(rand_half) > 0 and all(o.x < 0.0 for o in rand_half)
          and len(objs_wet) == len(objs1)
          and all(a.x == b.x and a.y == b.y and a.yaw == b.yaw
                  for a, b in zip(objs_wet, objs1)),
          f"{len(rand_half)} random objects, all on the wet half")
    check("per-class placement is deterministic per seed",
          len(objs1) == len(objs2) and all(
              a.type == b.type and a.x == b.x and a.y == b.y
              and a.length == b.length and a.fill_fraction == b.fill_fraction
              for a, b in zip(objs1, objs2)),
          f"{len(objs1)} objects twice")
    by_type = {}
    for o in objs1:
        by_type[o.type] = by_type.get(o.type, 0) + 1
    # density 12.5/ha on the 60x40 m grid (0.24 ha) -> 3 bottles.
    check("counts and densities honoured per class",
          by_type.get("drum_200l") == 6 and by_type.get("mesh_crate") == 6
          and by_type.get("traffic_cone") == 4
          and by_type.get("rope_coil") == 4
          and by_type.get("bottle_glass") == 3
          and by_type.get("crate_plastic") == 1, f"{by_type}")
    exp = objs1[-1]
    check("explicit instance placed verbatim (matched-pair capability)",
          exp.type == "mesh_crate" and exp.x == -5.0 and exp.y == 5.0
          and exp.length == 0.5 and exp.fill_fraction == 0.35
          and exp.burial == 0.0)
    check("burial distribution honoured",
          all(o.burial <= 0.2 for o in objs1 if o.type == "rope_coil"))
    check("open-frame classes draw a fill fraction, solid stay 1.0",
          all(0.30 <= o.fill_fraction <= 0.50 for o in objs1
              if o.type == "mesh_crate" and o is not exp)
          and all(o.fill_fraction == 1.0 for o in objs1
                  if o.type == "drum_200l"))

    def _mean_nn(objs):
        pts = np.array([[o.x, o.y] for o in objs])
        d = np.hypot(pts[:, None, 0] - pts[None, :, 0],
                     pts[:, None, 1] - pts[None, :, 1])
        np.fill_diagonal(d, np.inf)
        return float(d.min(axis=1).mean())

    cl_cfg = {"amount": {"kind": "count", "value": 24}}
    o_uni = place_objects(grid8, ObjectFieldConfig.from_dict(
        {"classes": {"concrete_block": dict(cl_cfg)}}), lib8,
        np.random.default_rng(9))
    o_clu = place_objects(grid8, ObjectFieldConfig.from_dict(
        {"classes": {"concrete_block": dict(
            cl_cfg, clusters={"n": 3, "sigma_m": 1.5})}}), lib8,
        np.random.default_rng(9))
    check("clustering pulls objects together (known-spacing clusters)",
          len(o_clu) == 24 and _mean_nn(o_clu) < 0.6 * _mean_nn(o_uni),
          f"mean NN clustered {_mean_nn(o_clu):.2f} m vs uniform "
          f"{_mean_nn(o_uni):.2f} m")

    # Fill-fraction dilution at the raster level: mesh twin vs solid twin.
    g_tiny = GridSpec(origin_x=-1.5, origin_y=-1.5, resolution=0.02,
                      nx=151, ny=151)

    def _stamp(obj):
        h = np.zeros((151, 151))
        r = np.full((151, 151), 0.3)
        rasterize_objects([obj], g_tiny, h, r)
        return h

    o_solid = PlacedObject(0, "crate_plastic", 0.0, 0.0, 0.0, 0.53, 0.37,
                           0.31, 0.0, 0.7, "plastic")
    o_mesh = PlacedObject(0, "mesh_crate", 0.0, 0.0, 0.0, 0.53, 0.37,
                          0.31, 0.0, 0.7, "plastic", fill_fraction=0.35)
    h_solid, h_mesh = _stamp(o_solid), _stamp(o_mesh)
    ratio = float(h_mesh.sum() / h_solid.sum())
    check("lattice mask dilutes the stamped structure toward fill_fraction",
          0.25 < ratio < 0.70,
          f"mesh/solid height-energy ratio {ratio:.2f} at fill 0.35")
    # Outline frame stays solid: every cell of the footprint's long edge
    # (|x| ~ length/2, inside the width) carries the full height.
    xs8 = np.arange(151) * 0.02 - 1.5
    edge_col = int(np.argmin(np.abs(xs8 - 0.25)))       # just inside x=0.265
    row_in = np.abs(xs8) < 0.17                          # inside the width
    check("lattice keeps a solid outline frame",
          bool((h_mesh[row_in, edge_col] > 0.9 * 0.31).all())
          and h_mesh.max() == h_solid.max())

    # Rendered echo dilution + FBR lock with the new families in scene
    # (NC #4): masks only ever add local height bumps, verified live.
    def _flat_scene(objects):
        g = GridSpec(origin_x=0.0, origin_y=-6.0, resolution=0.05,
                     nx=401, ny=241)
        h = np.full((241, 401), -3.0)
        r = np.full((241, 401), 0.3)
        rasterize_objects(objects, g, h, r)
        return SceneModel(grid=g, height=h, reflectivity=r,
                          material_id=np.zeros((241, 401), dtype=np.uint8),
                          material_names=["sand"], objects=objects,
                          walls=[], seed=0, config_snapshot={})

    # No return from land: a dry ridge across the swath masks its own
    # cells and (through the horizon cull) everything beyond it.
    def _ridge_scene(dry: bool):
        g = GridSpec(origin_x=0.0, origin_y=-6.0, resolution=0.05,
                     nx=401, ny=241)
        h = np.full((241, 401), -3.0)
        if dry:
            ys_r = g.origin_y + np.arange(g.ny) * g.resolution
            h[(ys_r > 5.9) & (ys_r < 7.9), :] = 0.5
        return SceneModel(grid=g, height=h, reflectivity=np.full(h.shape, 0.3),
                          material_id=np.zeros(h.shape, dtype=np.uint8),
                          material_names=["sand"], objects=[], walls=[],
                          seed=0, config_snapshot={})

    p_wet = GeometricRenderer(_ridge_scene(False), acq, model).render(
        Side.PORT, Pose3D(10.0, 0.0, 0.0), 0.0).ping.power
    p_dry = GeometricRenderer(_ridge_scene(True), acq, model).render(
        Side.PORT, Pose3D(10.0, 0.0, 0.0), 0.0).ping.power
    b_beyond = int(math.hypot(6.6, 3.0 - model.sensor_depth_m)
                   / acq.bin_size_m) + 2
    check("no return from dry land, and nothing beyond it",
          float(p_dry[b_beyond:].sum()) == 0.0
          and float(p_wet[b_beyond:].sum()) > 0.0
          and bool(np.allclose(p_dry[:int(5.0 / acq.bin_size_m)],
                               p_wet[:int(5.0 / acq.bin_size_m)])),
          f"beyond-ridge power {p_dry[b_beyond:].sum():.3g} vs wet "
          f"{p_wet[b_beyond:].sum():.3g}")

    def _echo_window(scene8):
        r8 = GeometricRenderer(scene8, acq, model)
        ping8 = r8.render(Side.PORT, Pose3D(10.0, 0.0, 0.0), 0.0).ping
        slant = math.hypot(6.0, 3.0 - model.sensor_depth_m)
        b0 = int((slant - 0.5) / acq.bin_size_m)
        b1 = int((slant + 0.8) / acq.bin_size_m)
        return float(np.sum(ping8.power[b0:b1]))

    e_solid = _echo_window(_flat_scene(
        [PlacedObject(0, "crate_plastic", 10.0, 6.0, 0.0, 0.53, 0.37, 0.31,
                      0.0, 0.7, "plastic")]))
    e_mesh = _echo_window(_flat_scene(
        [PlacedObject(0, "mesh_crate", 10.0, 6.0, 0.0, 0.53, 0.37, 0.31,
                      0.0, 0.7, "plastic", fill_fraction=0.35)]))
    check("open-frame echo is diluted vs its solid twin (rendered)",
          e_mesh < 0.9 * e_solid and e_mesh > 0.05 * e_solid,
          f"mesh {e_mesh:.3g} vs solid {e_solid:.3g}")

    mixed = ObjectFieldConfig.from_dict({"classes": {
        t: {"amount": {"kind": "count", "value": 2}} for t in new_types}})
    sc_mix = _flat_scene(place_objects(
        GridSpec(0.0, -6.0, 0.05, 401, 241), mixed, lib8,
        np.random.default_rng(4)))
    r_mix = GeometricRenderer(sc_mix, acq, model)
    fbr_curves = []
    for k in range(30):
        p8 = r_mix.render(Side.PORT, Pose3D(2.0 + 0.5 * k, 0.0, 0.0), 0.0).ping
        n8 = apply_ping_noise(
            p8.power, np.arange(len(p8.power)) * acq.bin_size_m
            < p8.altitude_m, 1.0, model, np.random.default_rng(50 + k),
            specular=p8.specular)
        fbr_curves.append(10.0 * np.log10(np.maximum(n8, 1e-12)))
    mean8 = np.mean(fbr_curves, axis=0)
    alt8 = 3.0 - model.sensor_depth_m
    # Processor-style bootstrap (ringing settle + noise window + persistent
    # crossing) — the transmit-ringing core is real data that a naive
    # scan-from-bin-0 detector would lock on, as on the real device.
    lock8 = _processor_lock_bin(mean8)
    check("FBR still locks with every new family in the scene (NC #4)",
          abs(lock8 * acq.bin_size_m - alt8) < 0.25,
          f"lock at {lock8 * acq.bin_size_m:.2f} m vs altitude {alt8:.2f} m")

    # Limits (S6): invisible collision-only borders, never acoustic.
    tiny_l = SceneModel(
        grid=tiny_grid, height=np.full((21, 21), -3.0),
        reflectivity=np.full((21, 21), 0.5),
        material_id=np.zeros((21, 21), dtype=np.uint8),
        material_names=["sand"],
        objects=[], walls=[Wall(name="q", x0=-5.0, y0=5.0, x1=5.0, y1=5.0)],
        seed=1, config_snapshot={}, geo_anchor=anchor)
    tiny_l.save(OUT / "limits_world")
    write_world_sdf(tiny_l, OUT / "limits_world", limits_collision=True)
    lsdf = (OUT / "limits_world" / "world.sdf").read_text()
    for side8 in ("north", "south", "east", "west"):
        blk = lsdf.split(f'<model name="limit_{side8}">')[1].split("</model>")[0]
        check(f"limit_{side8}: collision-only, invisible",
              "<collision" in blk and "<visual" not in blk)
    wall_blk = lsdf.split('<model name="wall_q">')[1].split("</model>")[0]
    check("walls carry a collision mirroring their visual",
          wall_blk.count("<box>") == 2 and "<collision" in wall_blk
          and "<visual" in wall_blk)
    back_l = SceneModel.load(OUT / "limits_world")
    check("limits never enter the acoustic scene (walls list unchanged)",
          [w.name for w in back_l.walls] == ["q"])
    check("legacy worlds emit no limit models", "limit_" not in sdf)

    # Every class must introspect for the GUI's Info dialog: a non-empty
    # typical footprint and a shape description. Catches a future class
    # whose mask breaks before anyone opens the wizard.
    from blueboat_sss_sim.worldgen.objects import (footprint_profile,
                                                   shape_description,
                                                   typical_instance)
    bad_info = []
    for t8, spec8 in CATALOG.items():
        prof8, o8 = footprint_profile(spec8, n=80)
        # Coverage is an absolute pixel count, not an area fraction: a
        # rope is centimetres wide over metres of length and legitimately
        # fills well under 1% of its own bounding image.
        if (prof8.max() <= 0 or int((prof8 > 0).sum()) < 10
                or not shape_description(spec8)
                or typical_instance(spec8).length <= 0):
            bad_info.append(t8)
    check("every catalog class renders an Info footprint + description",
          not bad_info, f"{len(CATALOG)} classes introspect; bad: {bad_info}")

    # ---- orientation: random | fixed | path_relative ------------------
    from blueboat_sss_sim.worldgen.objects import (  # noqa: E402
        nearest_segment_heading, normalise_orientation)

    def _same_objs(a, b):
        return len(a) == len(b) and all(
            p.type == q.type and p.x == q.x and p.y == q.y and p.yaw == q.yaw
            and p.length == q.length and p.width == q.width
            and p.burial == q.burial and p.reflectivity == q.reflectivity
            and p.fill_fraction == q.fill_fraction for p, q in zip(a, b))

    # (c) default == explicit random: the yaw draw stays the same uniform
    # variate in the same stream position, so every existing seed holds.
    cfg8_rand = ObjectFieldConfig.from_dict({
        "min_separation_m": 1.0,
        "classes": {t: dict(c, orientation={"mode": "random"})
                    for t, c in cfg8.classes.items()},
        "explicit": list(cfg8.explicit)})
    objs_rand = place_objects(grid8, cfg8_rand, lib8, np.random.default_rng(3))
    check("orientation default is bit-identical to explicit mode random",
          _same_objs(objs1, objs_rand), f"{len(objs_rand)} objects")
    check("random-mode yaws are still the historical uniform draw",
          all(-np.pi <= o.yaw <= np.pi for o in objs_rand)
          and len({round(o.yaw, 6) for o in objs_rand}) == len(objs_rand))

    # (a) fixed, no jitter: every yaw is exactly the requested heading.
    o_fix = place_objects(grid8, ObjectFieldConfig.from_dict({"classes": {
        "drum_200l": {"amount": {"kind": "count", "value": 8},
                      "orientation": {"mode": "fixed",
                                      "heading_deg": 30.0}}}}),
        lib8, np.random.default_rng(11))
    check("fixed orientation, jitter 0: all yaws equal the heading",
          len(o_fix) == 8 and all(
              abs(o.yaw - math.radians(30.0)) < 1e-12 for o in o_fix),
          f"yaws {sorted({round(math.degrees(o.yaw), 3) for o in o_fix})} deg")

    # A fixed class placed BEFORE a random class (sorted order) draws the
    # same uniform variate, so the later class does not move.
    base_b = {"drum_200l": {"amount": {"kind": "count", "value": 6}}}
    o_ab_rand = place_objects(grid8, ObjectFieldConfig.from_dict({"classes": {
        "bottle_glass": {"amount": {"kind": "count", "value": 5}},
        **base_b}}), lib8, np.random.default_rng(12))
    o_ab_fix = place_objects(grid8, ObjectFieldConfig.from_dict({"classes": {
        "bottle_glass": {"amount": {"kind": "count", "value": 5},
                         "orientation": {"mode": "fixed",
                                         "heading_deg": -45.0}},
        **base_b}}), lib8, np.random.default_rng(12))
    drums_r = [o for o in o_ab_rand if o.type == "drum_200l"]
    drums_f = [o for o in o_ab_fix if o.type == "drum_200l"]
    check("a fixed class does not move the classes drawn after it",
          _same_objs(drums_r, drums_f)
          and [(o.x, o.y) for o in o_ab_rand] == [(o.x, o.y) for o in o_ab_fix]
          and all(abs(o.yaw + math.radians(45.0)) < 1e-12
                  for o in o_ab_fix if o.type == "bottle_glass"),
          f"{len(drums_f)} drums identical; positions identical")

    # (b) path_relative on a straight +x path, offset 90 -> yaw = pi/2.
    o_rel = place_objects(grid8, ObjectFieldConfig.from_dict({
        "path_xy": [[-20.0, 0.0], [20.0, 0.0]],
        "classes": {"concrete_block": {
            "amount": {"kind": "count", "value": 7},
            "orientation": {"mode": "path_relative",
                            "heading_deg": 90.0}}}}),
        lib8, np.random.default_rng(13))
    check("path_relative on a +x track, offset 90: yaw = pi/2 (across-track)",
          len(o_rel) == 7 and all(abs(o.yaw - math.pi / 2) < 1e-9
                                  for o in o_rel))
    # Nearest-segment heading on an L-shaped path: east leg vs north leg.
    l_path = np.array([[0.0, 0.0], [30.0, 0.0], [30.0, 20.0]])
    check("nearest-segment heading follows the local track leg",
          abs(nearest_segment_heading(l_path, 10.0, 3.0)) < 1e-12
          and abs(nearest_segment_heading(l_path, 28.0, 15.0)
                  - math.pi / 2) < 1e-12)
    o_hdg = place_objects(grid8, ObjectFieldConfig.from_dict({
        "path_heading_deg": 45.0,
        "classes": {"brick": {
            "amount": {"kind": "count", "value": 3},
            "orientation": {"mode": "path_relative"}}}}),
        lib8, np.random.default_rng(14))
    check("headless path_heading_deg fallback: yaw = track heading + 0",
          all(abs(o.yaw - math.radians(45.0)) < 1e-12 for o in o_hdg))

    # (d) jitter > 0: a gaussian spread of the right magnitude.
    o_jit = place_objects(grid8, ObjectFieldConfig.from_dict({"classes": {
        "can": {"amount": {"kind": "count", "value": 200},
                "orientation": {"mode": "fixed", "heading_deg": 0.0,
                                "jitter_deg": 20.0}}}}),
        lib8, np.random.default_rng(15))
    sd_jit = float(np.degrees(np.std([o.yaw for o in o_jit])))
    check("jitter_deg gives a gaussian yaw spread of that sigma",
          len(o_jit) == 200 and 14.0 < sd_jit < 26.0,
          f"sd {sd_jit:.1f} deg for jitter 20 over {len(o_jit)} objects")

    # Validation: unknown modes / keys and a path-less path_relative raise.
    bad = []
    for label8, orient8 in (("mode", {"mode": "sideways"}),
                            ("key", {"mode": "fixed", "heading": 1.0}),
                            ("jitter", {"mode": "fixed", "jitter_deg": -1})):
        try:
            normalise_orientation(orient8)
            bad.append(label8)
        except ValueError:
            pass
    try:
        place_objects(grid8, ObjectFieldConfig.from_dict({"classes": {
            "brick": {"amount": {"kind": "count", "value": 1},
                      "orientation": {"mode": "path_relative"}}}}),
            lib8, np.random.default_rng(1))
        bad.append("no-path")
    except ValueError as e:
        if "path_xy" not in str(e):
            bad.append("no-path-message")
    check("orientation validation raises by name (mode, key, jitter, no path)",
          not bad, f"unraised: {bad}")
    check("normalise_orientation fills defaults (random, 0, 0, rest off)",
          normalise_orientation(None)
          == {"mode": "random", "heading_deg": 0.0, "jitter_deg": 0.0,
              "rest": {"enabled": False, "randomness": 0.5}})

    # Builder plumbing: the world config carries path_xy only when a class
    # asks for path_relative (other worlds' snapshots stay byte-identical).
    from blueboat_sss_sim.builder.state import (BuilderState,  # noqa: E402
                                                ObjectsState, PathState)
    from blueboat_sss_sim.worldgen.world_folder import (  # noqa: E402
        state_to_world_config)
    ps8 = PathState(name="p8", file="", geo_anchor={"lat0": 43.7,
                                                    "lon0": 7.3},
                    points=[[0.0, 5.0, 5.0, 0.0], [10.0, 25.0, 5.0, 0.0]])
    st8 = BuilderState(path=ps8, limits_rect={"x_min": 0.0, "y_min": 0.0,
                                              "x_max": 40.0, "y_max": 20.0},
                       objects=ObjectsState(seed=2, classes={
                           "brick": {"amount": {"kind": "count",
                                                "value": 2}}}))
    raw_plain = state_to_world_config(st8)
    st8.objects.classes["brick"]["orientation"] = {"mode": "path_relative",
                                                   "heading_deg": 90.0}
    raw_rel = state_to_world_config(st8)
    check("state_to_world_config adds path_xy (world frame) only for "
          "path_relative",
          "path_xy" not in raw_plain["objects"]
          and raw_rel["objects"]["path_xy"] == [[0.0, 0.0], [20.0, 0.0]])
    sc8 = generate_scene(WorldConfig.from_dict(raw_rel), raw_config=raw_rel)
    check("a built world honours path_relative through the config snapshot",
          len(sc8.objects) == 2
          and all(abs(o.yaw - math.pi / 2) < 1e-9 for o in sc8.objects)
          and sc8.config_snapshot["objects"]["path_xy"] == [[0.0, 0.0],
                                                            [20.0, 0.0]])

    # ---- 9. World folders --------------------------------------------------
    # ---- objects v3: 3-D resting orientation (worldgen/solids.py) ----
    from blueboat_sss_sim.worldgen.objects import (draw_rest as _draw_rest,
                                                    rest_poses as _rest_poses,
                                                    rasterize_objects as _rast)
    from blueboat_sss_sim.worldgen.solids import (Solid as _Solid, settle as _settle,
                                                   rotation_rpy as _rpy,
                                                   zbuffer as _zbuf,
                                                   stable_poses as _stable)
    _g = GridSpec(-4.0, -4.0, 0.05, 161, 161)
    _xs = _g.origin_x + np.arange(_g.nx) * _g.resolution
    _XX, _YY = np.meshgrid(_xs, _xs)
    _drum = lambda oid, x, y, **kw: PlacedObject(  # noqa: E731
        object_id=oid, type="drum_200l", x=x, y=y, yaw=0.3, length=0.58,
        width=0.58, proud_height=0.9, burial=0.0, reflectivity=0.8,
        material="metal", **kw)
    # A standing drum through the solid path equals the legacy flat disc.
    h_leg, h_sol = np.full((161, 161), -3.0), np.full((161, 161), -3.0)
    r_leg, r_sol = np.full((161, 161), 0.5), np.full((161, 161), 0.5)
    _rast([_drum(1, 0.0, 0.0)], _g, h_leg, r_leg)
    _rast([_drum(1, 0.0, 0.0, rest="standing")], _g, h_sol, r_sol)
    _cells = (h_leg > -2.99).sum()
    check("a standing drum stamps the same disc through the solid path",
          abs(float(h_sol.max() - h_leg.max())) < 1e-9
          and abs(int((h_sol > -2.99).sum()) - _cells) <= 0.05 * _cells,
          f"peak {h_sol.max() + 3:.3f} vs {h_leg.max() + 3:.3f} m, "
          f"{int((h_sol > -2.99).sum())} vs {_cells} cells")
    # A lying drum: semicircular cross-section, wider footprint, lower top.
    h_lie = np.full((161, 161), -3.0)
    _rast([_drum(2, 0.0, 0.0, roll=np.pi / 2, rest="lying")], _g, h_lie,
          np.full((161, 161), 0.5))
    check("a lying drum is diameter-high with the cylinder's footprint",
          abs(float(h_lie.max()) + 3.0 - 0.58) < 0.02
          and 0.8 * _cells * 0.9 * 0.58 / (np.pi * 0.29 ** 2) * 1.0
          < (h_lie > -2.99).sum() < 1.3 * 0.9 * 0.58 / _g.resolution ** 2,
          f"top {h_lie.max() + 3:.3f} m, {(h_lie > -2.99).sum()} cells")
    # Contact invariant on a slope: lowest point exactly on the resting
    # plane, never below the terrain, for every stable pose of every solid.
    _terr = 0.25 * _XX - 3.0
    _worst = 0.0
    _n = 0
    for _spec_name in ("drum_200l", "concrete_block", "traffic_cone",
                       "tire_car", "pipe_pvc", "bottle_glass"):
        _o = typical_instance(_CATALOG_V3[_spec_name])
        _sol = _CATALOG_V3[_spec_name].solid(_o)
        for _p in _stable(_sol):
            _b, _pr, _cz, _Rw = _settle(_sol, _rpy(_p.roll, _p.pitch, 0.7),
                                        _XX, _YY, _terr)
            _top, _bot = _zbuf(_sol, _Rw, _XX, _YY)
            _gap = float(np.nanmin(_bot + _cz - (0.25 * _XX - 3.0)))
            _worst = max(_worst, abs(_gap))
            _n += 1
            assert _b.max() > 0.0
    check("every stable pose settles with its lowest point on the seabed plane",
          _worst < 1e-6 and _n >= 15, f"{_n} poses, worst gap {_worst:.1e} m")
    # Determinism + independence: the rest draw is a per-object stream.
    _a, _b2 = _drum(7, 1.2, -0.4), _drum(7, 1.2, -0.4)
    _draw_rest(_CATALOG_V3["drum_200l"], _a, 1.0)
    _draw_rest(_CATALOG_V3["drum_200l"], _b2, 1.0)
    _c = _drum(7, 1.2, -0.4)
    _draw_rest(_CATALOG_V3["drum_200l"], _c, 0.0)
    check("rest draws are deterministic per object and r=0 keeps the pose",
          (_a.roll, _a.pitch, _a.rest) == (_b2.roll, _b2.pitch, _b2.rest)
          and _c.rest == "settled" and abs(_c.roll) < 1e-12
          and abs(_c.pitch) < 1e-12 and _a.rest in ("lying", "settled",
                                                    "inverted"),
          f"{_a.rest} ({np.degrees(_a.roll):.0f}, {np.degrees(_a.pitch):.0f}) deg")
    # The whole placement: a class with rest on moves no other class.
    _cfg_off = {"seed": 5, "size": [60, 60], "origin": [-30, -30],
                "resolution": 0.1}
    _wc_a = WorldConfig.from_dict({
        "world": _cfg_off, "objects": {"classes": {
            "drum_200l": {"amount": {"kind": "count", "value": 12}},
            "tire_car": {"amount": {"kind": "count", "value": 12}}}}})
    _wc_b = WorldConfig.from_dict({
        "world": _cfg_off, "objects": {"classes": {
            "drum_200l": {"amount": {"kind": "count", "value": 12},
                          "orientation": {"rest": {"enabled": True,
                                                   "randomness": 1.0}}},
            "tire_car": {"amount": {"kind": "count", "value": 12}}}}})
    _sa = generate_scene(_wc_a)
    _sb = generate_scene(_wc_b)
    _tires_a = [(o.x, o.y, o.yaw) for o in _sa.objects if o.type == "tire_car"]
    _tires_b = [(o.x, o.y, o.yaw) for o in _sb.objects if o.type == "tire_car"]
    _drums_b = [o for o in _sb.objects if o.type == "drum_200l"]
    check("enabling 3-D rest for one class moves no other object",
          _tires_a == _tires_b
          and [(o.x, o.y) for o in _sa.objects] == [(o.x, o.y) for o in _sb.objects]
          and any(o.rest == "lying" for o in _drums_b)
          and all(o.rested for o in _drums_b),
          f"{sum(o.rest == 'lying' for o in _drums_b)}/{len(_drums_b)} drums lying")
    _sdf_b = write_world_sdf.__doc__ is not None
    check("rested objects carry roll/pitch/rest in the manifest and the SDF pose",
          _sdf_b and all(hasattr(o, "rest_z") for o in _drums_b)
          and "rest" in dataclasses.asdict(_drums_b[0]),
          "PlacedObject fields present")

    print("[9] world folder: build, immutability, edit round-trip")
    from blueboat_sss_sim.builder.paths import (list_anchored_paths,  # noqa: E402
                                                read_geo_anchor)
    from blueboat_sss_sim.builder.state import (BuilderState,  # noqa: E402
                                                DepthState, ObjectsState,
                                                PathState)
    from blueboat_sss_sim.worldgen.scene import generate_scene as _gen  # noqa: E402
    from blueboat_sss_sim.worldgen.world_folder import (  # noqa: E402
        _generate_with_base, build_world_folder, list_worlds,
        load_builder_state, state_to_world_config)

    tdir = OUT / "trajectories"
    tdir.mkdir()
    pts9 = [[0.0, 0.0, 0.0, 0.0], [30.0, 30.0, 0.0, 0.0],
            [60.0, 30.0, 20.0, 1.57], [90.0, 0.0, 20.0, 3.14]]
    (tdir / "wakamatsu_like.yaml").write_text(_yaml.safe_dump({
        "format": "blueboat_trajectory/1", "name": "wakamatsu_like",
        "speed": 0.8, "loop": False,
        "geo_anchor": {"lat0": anchor.lat0, "lon0": anchor.lon0,
                       "theta_deg": 0.0},
        "points": pts9}))
    (tdir / "plain_local.yaml").write_text(_yaml.safe_dump({
        "format": "blueboat_trajectory/1", "name": "plain_local",
        "speed": 0.8, "points": pts9}))
    (tdir / "wakamatsu_like.meta.yaml").write_text("format: meta\n")
    found = list_anchored_paths(tdir)
    check("anchored-path discovery matches the MCS predicate",
          [p.name for p in found] == ["wakamatsu_like"]
          and read_geo_anchor(tdir / "plain_local.yaml") is None)

    ps9 = PathState.from_mcs_file(tdir / "wakamatsu_like.yaml")
    try:
        PathState.from_mcs_file(tdir / "plain_local.yaml")
        unanchored_ok = False
    except ValueError:
        unanchored_ok = True
    check("a plain local path cannot anchor a world", unanchored_ok)

    st9 = BuilderState(
        path=ps9,
        limits_rect={"x_min": -10.0, "y_min": -10.0,
                     "x_max": 45.0, "y_max": 35.0},
        walls=[{"name": "quay", "x0": -10.0, "y0": 34.0, "x1": 45.0,
                "y1": 34.0, "top_z": 1.0, "reflectivity": 0.65}],
        depth=DepthState(mode="preset", preset="harbour_basin",
                         params={"basin_depth": 3.0}),
        objects=ObjectsState(seed=7, classes={
            "drum_200l": {"amount": {"kind": "count", "value": 3}},
            "mesh_crate": {"amount": {"kind": "count", "value": 2}}},
            explicit=[{"type": "tire_car", "x": 5.0, "y": 5.0}]),
        world_name="smoke_world")
    wroot = OUT / "worlds"
    wdir = build_world_folder(st9, worlds_root=wroot)
    check("folder lands at worlds/<path_name>/<world_name>",
          wdir == wroot / "wakamatsu_like" / "smoke_world")
    for f9 in ("world.sdf", "seabed.stl", "scene.npz", "scene_manifest.yaml",
               "depth.npz", "preview_objects.png", "metadata.yaml",
               "builder_state.yaml", "source_path_world.yaml"):
        check(f"world folder contains {f9}", (wdir / f9).exists())

    sc9 = SceneModel.load(wdir)
    check("world folder loads as a scene (anchor, walls, objects intact)",
          sc9.geo_anchor.lat0 == anchor.lat0 and len(sc9.walls) == 1
          and len(sc9.objects) == 6)
    sdf9 = (wdir / "world.sdf").read_text()
    check("builder world.sdf: anchored, walled, bounded",
          "<spherical_coordinates>" in sdf9 and "wall_quay" in sdf9
          and all(f"limit_{s}" in sdf9 for s in
                  ("north", "south", "east", "west")))
    # The water plane must cover the (generally origin-offset) grid extent,
    # not sit at the Gazebo origin: rect -10..45 x -10..35 -> centre
    # (17.5, 12.5), size 55 x 45.
    wp9 = sdf9.split('<model name="water_plane">')[1].split("</model>")[0]
    check("water plane is centred on the world extent, not the origin",
          "<pose>17.500 12.500 0 0 0 0</pose>" in wp9
          and "<size>55.0 45.0</size>" in wp9,
          "pose (17.5, 12.5), size 55 x 45 for the offset rectangle")
    dnpz = np.load(wdir / "depth.npz")
    bump9 = sc9.height - dnpz["height"]
    check("depth.npz is the pre-object-stamp design surface",
          dnpz["height"].shape == sc9.height.shape
          and float(bump9.max()) > 0.3 and float(bump9.min()) >= 0.0
          and abs(float(np.median(dnpz["height"])) + 3.0) < 0.2,
          f"median {np.median(dnpz['height']):.2f}, tallest object bump "
          f"{bump9.max():.2f} m")
    raw9 = state_to_world_config(st9)
    sc9b, _pre = _generate_with_base(WorldConfig.from_dict(raw9), raw9)
    sc9c = _gen(WorldConfig.from_dict(raw9), raw_config=raw9)
    check("world_folder generation matches generate_scene exactly",
          np.array_equal(sc9b.height, sc9c.height)
          and np.array_equal(sc9.height, sc9c.height)
          and [o.x for o in sc9b.objects] == [o.x for o in sc9c.objects])

    meta9 = _yaml.safe_load((wdir / "metadata.yaml").read_text())
    ga9 = GeoAnchor.from_dict(meta9["geo_anchor"])
    ok_gps = True
    for om in meta9["objects"]:
        e9, n9 = ga9.to_en(om["lat"], om["lon"])
        ok_gps &= (abs(float(e9) - om["x"]) < 2e-2
                   and abs(float(n9) - om["y"]) < 2e-2)
    sw = meta9["limits"]["corners_gps"][0]
    lat_sw, lon_sw = ga9.to_latlon(-10.0, -10.0)
    check("metadata GPS ground truth inverts to scene positions",
          ok_gps and abs(sw[0] - float(lat_sw)) < 1e-7
          and abs(sw[1] - float(lon_sw)) < 1e-7
          and meta9["depth"]["mode"] == "preset"
          and abs(meta9["depth"]["mean_m"] - 3.0) < 0.3
          and meta9["walls"][0]["gps"][0] is not None)

    # Immutability: a rebuild -- Edit-mode re-save included -- always takes
    # a fresh _i folder and never touches the original.
    manifest_before = (wdir / "scene_manifest.yaml").read_bytes()
    wdir2 = build_world_folder(st9, worlds_root=wroot)
    check("name collision silently takes _1 (worlds immutable)",
          wdir2.name == "smoke_world_1"
          and (wdir / "scene_manifest.yaml").read_bytes() == manifest_before)

    st9b = load_builder_state(wdir)
    check("builder_state round-trips for Edit mode",
          st9b.limits_rect == st9.limits_rect
          and st9b.objects.classes == st9.objects.classes
          and st9b.walls == st9.walls
          and st9b.world_name == "smoke_world"
          and st9b.path.points == st9.path.points)
    wdir3 = build_world_folder(st9b, worlds_root=wroot)
    sc9d = SceneModel.load(wdir3)
    check("an edited-then-rebuilt world reproduces the original scene",
          wdir3.name == "smoke_world_2"
          and np.array_equal(sc9d.height, sc9.height)
          and len(sc9d.objects) == len(sc9.objects))

    # Painted state: strokes survive builder_state.yaml, re-base into the
    # world frame like walls, and rebuild bit-identically.
    st_paint = BuilderState(
        path=PathState.from_mcs_file(tdir / "wakamatsu_like.yaml"),
        limits_rect={"x_min": -10.0, "y_min": -10.0,
                     "x_max": 45.0, "y_max": 35.0},
        walls=[{"name": "quay", "x0": -10.0, "y0": 34.0, "x1": 45.0,
                "y1": 34.0, "top_z": 1.0, "reflectivity": 0.65}],
        depth=DepthState(mode="preset", preset="flat",
                         params={"depth": 3.0},
                         paint=[{"points": [[10.0, 10.0], [20.0, 10.0]],
                                 "radius_m": 3.0, "value": 5.5,
                                 "erase": False}],
                         material_paint=[{"points": [[0.0, -5.0]],
                                          "radius_m": 4.0, "value": "mud",
                                          "erase": False}]),
        objects=ObjectsState(seed=3),
        world_name="painted_world")
    wdir_p = build_world_folder(st_paint, worlds_root=wroot)
    sc_p = SceneModel.load(wdir_p)

    def _cell9(sc, x, y):
        ci_, cj_ = sc.grid.world_to_grid(x, y)
        return int(round(float(cj_))), int(round(float(ci_)))

    check("painted world: trench and material patch land where painted",
          abs(-sc_p.height[_cell9(sc_p, 15.0, 10.0)] - 5.5) < 0.05
          and sc_p.material_names[
              int(sc_p.material_id[_cell9(sc_p, 0.0, -5.0)])] == "mud"
          and sc_p.material_names[
              int(sc_p.material_id[_cell9(sc_p, 40.0, 30.0)])] == "sand"
          and sc_p.config_snapshot["depth"]["paint"][0]["points"][0]
          == [10.0, 10.0])
    st_paint_back = load_builder_state(wdir_p)
    wdir_p2 = build_world_folder(st_paint_back, worlds_root=wroot)
    sc_p2 = SceneModel.load(wdir_p2)
    check("a painted builder_state reloads and rebuilds bit-identically",
          wdir_p2.name == "painted_world_1"
          and st_paint_back.depth.paint == st_paint.depth.paint
          and st_paint_back.depth.material_paint
          == st_paint.depth.material_paint
          and st_paint_back.depth.texture == st_paint.depth.texture
          and np.array_equal(sc_p2.height, sc_p.height)
          and np.array_equal(sc_p2.material_id, sc_p.material_id))

    st_file = BuilderState(
        path=ps9,
        limits_rect={"x_min": -8.0, "y_min": -8.0,
                     "x_max": 8.0, "y_max": 8.0},
        depth=DepthState(mode="file", file=str(csv_path)),
        objects=ObjectsState(seed=3, classes={
            "traffic_cone": {"amount": {"kind": "count", "value": 2}}}),
        world_name="csv_world")
    wdir4 = build_world_folder(st_file, worlds_root=wroot)
    st_file_back = load_builder_state(wdir4)
    check("depth-file worlds are self-contained (source copied in)",
          (wdir4 / "depth_source.csv").exists()
          and st_file_back.depth.file == str(wdir4 / "depth_source.csv"))
    d4 = np.load(wdir4 / "depth.npz")["height"]
    check("depth-file build uses the interpolated surface",
          abs(float(-d4[len(d4) // 2, len(d4[0]) // 2]) - 3.0) < 0.3,
          f"centre depth {-d4[len(d4) // 2, len(d4[0]) // 2]:.2f} m")

    st_svlog = BuilderState(
        path=ps9,
        limits_rect={"x_min": -36.0, "y_min": -20.0,
                     "x_max": 36.0, "y_max": 20.0},
        depth=DepthState(mode="svlog", file=str(svlog_path),
                         svlog={"texture_gain": 0.3, "backscatter_layer": True}),
        objects=ObjectsState(seed=3),
        world_name="svlog_world")
    wdir_s = build_world_folder(st_svlog, worlds_root=wroot)
    st_svlog_back = load_builder_state(wdir_s)
    with open(wdir_s / "metadata.yaml", "r", encoding="utf-8") as f:
        meta_s = _yaml.safe_load(f)
    smp_s = np.load(wdir_s / "depth_samples.npz")
    check("recording worlds are self-contained: source copy, soundings, "
          "survey quality in the metadata, no objects unless asked",
          (wdir_s / "depth_source.svlog").exists()
          and st_svlog_back.depth.file == str(wdir_s / "depth_source.svlog")
          and st_svlog_back.depth.svlog["texture_gain"] == 0.3
          and meta_s["depth"]["mode"] == "svlog"
          and meta_s["depth"]["svlog_quality"]["pings"] == 200
          and meta_s["depth"]["svlog_quality"]["clock_mode"] == "live"
          and len(smp_s["east"]) > 100 and meta_s["objects"] == []
          and (wdir_s / "reflectivity_hint.npz").exists()
          and np.isfinite(np.load(wdir_s / "reflectivity_hint.npz")["values"]).any()
          and meta_s["depth"]["svlog_quality"]["backscatter_samples"] > 100,
          f"{len(smp_s['east'])} soundings, "
          f"{meta_s['depth']['svlog_quality']['cells']} cells, "
          f"{meta_s['depth']['svlog_quality']['backscatter_samples']} "
          "backscatter samples")
    cov_meta = meta_s["depth"]["svlog_quality"].get("coverage") or {}
    check("the world records how much of it was measured rather than "
          "inferred",
          cov_meta.get("inside", 0) > 0
          and cov_meta["inside"] <= cov_meta["total"]
          and 0.0 < cov_meta["inside_fraction"] <= 1.0
          and 0.0 < cov_meta["area_fraction"] <= 1.0
          and cov_meta["reach_m"] > 0.0
          and all(isinstance(v, (int, float)) for v in cov_meta["world_box_m"]),
          f"{cov_meta.get('inside')}/{cov_meta.get('total')} samples inside, "
          f"{100 * cov_meta.get('area_fraction', 0):.0f} % of the area "
          f"within {cov_meta.get('reach_m', 0):.0f} m of a sounding")
    wdir_s2 = build_world_folder(st_svlog_back, worlds_root=wroot)
    d_s1 = np.load(wdir_s / "depth.npz")["height"]
    d_s2 = np.load(wdir_s2 / "depth.npz")["height"]
    z_true9 = met_scene.sample_height(smp_s["cell_east"], smp_s["cell_north"])
    check("a recording world rebuilds bit-identically and follows the "
          "recorded seabed",
          wdir_s2.name == "svlog_world_1" and np.array_equal(d_s1, d_s2)
          and float(np.median(np.abs(smp_s["cell_depth"] + z_true9))) < 0.2,
          f"median cell error {np.median(np.abs(smp_s['cell_depth'] + z_true9)):.3f} m")

    listing = list_worlds(wroot)
    check("library scan finds every saved world",
          {w["world_name"] for w in listing} ==
          {"smoke_world", "smoke_world_1", "smoke_world_2", "csv_world",
           "painted_world", "painted_world_1", "svlog_world",
           "svlog_world_1"}
          and all(w["path_name"] == "wakamatsu_like" for w in listing))

    # --- world-frame re-basing: a real MCS path whose waypoints sit far
    # from its design origin (the field case that stranded the boat
    # outside the area). World (0, 0) must be the spawn -- the path's
    # first waypoint -- inside the drawn limits, with GPS ground truth
    # unchanged by the shift.
    (tdir / "wakamatsu_off.yaml").write_text(_yaml.safe_dump({
        "format": "blueboat_trajectory/1", "name": "wakamatsu_off",
        "speed": 0.8, "loop": False,
        "geo_anchor": {"lat0": anchor.lat0, "lon0": anchor.lon0,
                       "theta_deg": 0.0},
        "points": [[0.0, 200.0, 100.0, 0.0], [30.0, 230.0, 100.0, 0.0],
                   [60.0, 230.0, 120.0, 1.57]]}))
    ps_off = PathState.from_mcs_file(tdir / "wakamatsu_off.yaml")
    st_off = BuilderState(
        path=ps_off,
        limits_rect={"x_min": 185.0, "y_min": 85.0,
                     "x_max": 245.0, "y_max": 135.0},
        walls=[{"name": "quay", "x0": 185.0, "y0": 134.0, "x1": 245.0,
                "y1": 134.0, "top_z": 1.0, "reflectivity": 0.65}],
        depth=DepthState(mode="preset", preset="canal", params={}),
        objects=ObjectsState(seed=3, explicit=[
            {"type": "drum_200l", "x": 210.0, "y": 110.0, "yaw": 0.0}]),
        world_name="offset_world")
    check("spawn origin is the path start when inside the limits",
          st_off.spawn_origin() == (200.0, 100.0))
    st_ctr = BuilderState(path=ps_off,
                          limits_rect={"x_min": 300.0, "y_min": 300.0,
                                       "x_max": 360.0, "y_max": 340.0})
    check("spawn origin falls back to the area centre otherwise",
          st_ctr.spawn_origin() == (330.0, 320.0))

    wdir5 = build_world_folder(st_off, worlds_root=OUT / "worlds_off")
    sc5 = SceneModel.load(wdir5)
    x5min, y5min, x5max, y5max = sc5.grid.extent
    check("world (0,0) -- the spawn -- lies inside the re-based limits",
          x5min < 0.0 < x5max and y5min < 0.0 < y5max
          and (x5min, y5min) == (-15.0, -15.0),
          f"extent x {x5min}..{x5max}, y {y5min}..{y5max}")
    lat5, lon5 = anchor.to_latlon(200.0, 100.0)
    check("world anchor is the spawn origin's GPS",
          abs(sc5.geo_anchor.lat0 - float(lat5)) < 1e-9
          and abs(sc5.geo_anchor.lon0 - float(lon5)) < 1e-9)
    meta5 = _yaml.safe_load((wdir5 / "metadata.yaml").read_text())
    o5 = meta5["objects"][0]
    lat_d, lon_d = anchor.to_latlon(210.0, 110.0)   # design-frame GPS
    check("GPS ground truth is unchanged by the frame shift",
          o5["x"] == 10.0 and o5["y"] == 10.0
          and abs(o5["lat"] - float(lat_d)) < 2e-7
          and abs(o5["lon"] - float(lon_d)) < 2e-7
          and meta5["source_path"]["design_offset"] == [200.0, 100.0]
          # metadata rounds GPS to 8 decimals (~1 mm)
          and abs(meta5["spawn"]["lat"] - sc5.geo_anchor.lat0) < 1e-7
          and abs(meta5["spawn"]["lon"] - sc5.geo_anchor.lon0) < 1e-7)
    check("walls shift into the world frame with the rectangle",
          sc5.walls[0].y0 == 34.0 and sc5.walls[0].x0 == -15.0)

    # ---- endpoint merging + curved walls (core.geometry, 2026-09-03) ----
    from blueboat_sss_sim.core.geometry import (WALL_SNAP_M, curve_base_name,
                                                 expand_walls, merge_endpoints,
                                                 quad_bezier_polyline,
                                                 wall_segments)
    _wm = [{"name": "a", "x0": 0.0, "y0": 0.0, "x1": 20.0, "y1": 0.2},
           {"name": "b", "x0": 20.3, "y0": -0.1, "x1": 20.0, "y1": 30.0},
           {"name": "c", "x0": 19.9, "y0": 30.4, "x1": 0.0, "y1": 30.0},
           {"name": "far", "x0": 40.0, "y0": 0.0, "x1": 60.0, "y1": 0.0}]
    _m1 = merge_endpoints(_wm, WALL_SNAP_M)
    _m2 = merge_endpoints(_m1, WALL_SNAP_M)
    check("close wall endpoints merge to one shared point, idempotently",
          (_m1[0]["x1"], _m1[0]["y1"]) == (_m1[1]["x0"], _m1[1]["y0"])
          and (_m1[1]["x1"], _m1[1]["y1"]) == (_m1[2]["x0"], _m1[2]["y0"])
          and _m1 == _m2 and _m1[3] == _wm[3]
          and abs(_m1[0]["x1"] - 20.15) < 1e-9 and abs(_m1[0]["y1"] - 0.05) < 1e-9,
          f"corner ({_m1[0]['x1']:.3f}, {_m1[0]['y1']:.3f})")
    _tiny = merge_endpoints([{"name": "t", "x0": 0.0, "y0": 0.0, "x1": 0.4, "y1": 0.0},
                             {"name": "u", "x0": 0.4, "y0": 0.0, "x1": 9.0, "y1": 0.0}])
    check("a wall that collapses when merged is dropped", [w["name"] for w in _tiny] == ["u"])
    _straight = wall_segments({"name": "q", "x0": 0.0, "y0": 0.0, "x1": 30.0,
                               "y1": 0.0, "top_z": 1.0})
    _curve = {"name": "q", "x0": 0.0, "y0": 0.0, "x1": 30.0, "y1": 0.0,
              "top_z": 1.0, "ctrl": [15.0, 4.0]}
    _segs = wall_segments(_curve)
    _pts = quad_bezier_polyline((0.0, 0.0), (15.0, 4.0), (30.0, 0.0))
    _chained = all(abs(_segs[k]["x1"] - _segs[k + 1]["x0"]) < 1e-12
                   and abs(_segs[k]["y1"] - _segs[k + 1]["y0"]) < 1e-12
                   for k in range(len(_segs) - 1))
    # Sagitta check: the true curve at the chord midpoints is within tol.
    _t = np.linspace(0.0, 1.0, 2 * (len(_pts) - 1) + 1)[1::2][:, None]
    _p1 = 2.0 * np.array([15.0, 4.0]) - 0.5 * (np.array([0.0, 0.0]) + np.array([30.0, 0.0]))
    _true = (1 - _t) ** 2 * np.array([0.0, 0.0]) + 2 * (1 - _t) * _t * _p1 + _t ** 2 * np.array([30.0, 0.0])
    _mid = 0.5 * (_pts[:-1] + _pts[1:])
    _sag = float(np.max(np.linalg.norm(_true - _mid, axis=1)))
    check("a straight wall expands 1:1 unnamed; a curved one to chained chords",
          _straight == [{"name": "q", "x0": 0.0, "y0": 0.0, "x1": 30.0, "y1": 0.0,
                         "top_z": 1.0}]
          and len(_segs) >= 4 and _chained and "ctrl" not in _segs[0]
          and _segs[0]["name"] == "q_s01" and curve_base_name(_segs[-1]["name"]) == "q"
          and abs(_segs[len(_segs) // 2]["y0"] - 4.0) < 0.3 and _sag < 0.11
          and all(s["top_z"] == 1.0 for s in _segs),
          f"{len(_segs)} chords, max sagitta {_sag:.3f} m")
    st_curve = dataclasses.replace(
        st_off, walls=[{"name": "quay", "x0": 185.0, "y0": 134.0, "x1": 245.0,
                        "y1": 134.0, "top_z": 1.0, "reflectivity": 0.65,
                        "ctrl": [215.0, 128.0]},
                       {"name": "pier", "x0": 245.4, "y0": 133.7, "x1": 245.0,
                        "y1": 100.0, "top_z": 1.0}],
        world_name="curved_world")
    st_curve.validate_for_build()
    wdir_c = build_world_folder(st_curve, worlds_root=OUT / "worlds_curve")
    sc_c = SceneModel.load(wdir_c)
    meta_c = _yaml.safe_load((wdir_c / "metadata.yaml").read_text())
    st_c2 = BuilderState.from_dict(st_curve.to_dict())
    check("a curved wall reaches the world folder as chords, everywhere alike",
          len(sc_c.walls) >= 5
          and all(curve_base_name(w.name) == "quay" for w in sc_c.walls[:-1])
          and sc_c.walls[-1].name == "pier"
          and abs(sc_c.walls[-2].x1 - sc_c.walls[-1].x0) < 1e-9
          and abs(sc_c.walls[-2].y1 - sc_c.walls[-1].y0) < 1e-9
          and (wdir_c / "world.sdf").read_text().count("<model name=\"wall_quay_s") == len(sc_c.walls) - 1
          and [m["curve"] for m in meta_c["walls"]][:-1] == ["quay"] * (len(sc_c.walls) - 1)
          and meta_c["walls"][-1]["curve"] is None
          and st_c2.walls[0]["ctrl"] == [215.0, 128.0]
          and st_c2.walls[0]["ctrl"] is not st_curve.walls[0]["ctrl"],
          f"{len(sc_c.walls)} engine walls from 2 drawn; pier corner merged")
    wdir_c2 = build_world_folder(st_curve, worlds_root=OUT / "worlds_curve")
    check("the merged + flattened world builds byte-identically twice",
          (wdir_c / "scene_manifest.yaml").read_bytes()
          == (wdir_c2 / "scene_manifest.yaml").read_bytes()
          and (wdir_c / "world.sdf").read_bytes() == (wdir_c2 / "world.sdf").read_bytes())
    st_off_p = dataclasses.replace(st_off, depth=DepthState(
        mode="preset", preset="flat", params={"depth": 3.0},
        paint=[{"points": [[210.0, 110.0]], "radius_m": 2.0, "value": 4.0,
                "erase": False}],
        material_paint=[{"points": [[230.0, 120.0]], "radius_m": 2.0,
                         "value": "gravel", "erase": False}]),
        world_name="offset_painted")
    raw_off_p = state_to_world_config(st_off_p)
    check("paint strokes re-base into the world frame like walls",
          raw_off_p["depth"]["paint"][0]["points"] == [[10.0, 10.0]]
          and raw_off_p["terrain"]["materials"]["paint"][0]["points"]
          == [[30.0, 20.0]])
    wp5 = (wdir5 / "world.sdf").read_text().split(
        '<model name="water_plane">')[1].split("</model>")[0]
    check("offset world's water plane covers its extent",
          "<pose>15.000 10.000 0 0 0 0</pose>" in wp5)
    tt5 = load_trajectory(wdir5 / "source_path_world.yaml")
    x5, y5, _ = tt5.pose_at(0.0)
    check("source_path_world.yaml starts at world (0,0) -- the spawn",
          isinstance(tt5, TimedTrajectory)
          and abs(x5) < 1e-9 and abs(y5) < 1e-9
          and abs(tt5.duration - 60.0) < 1e-9)

    # Launch-side resolution (module-scope, launch-free helpers, like
    # resolve_acquisition in [4b]).
    from full_mission_launch import trajectory_total_time  # noqa: E402
    from sss_sim_launch import (resolve_sonar_config,  # noqa: E402
                                resolve_world_anchor)
    check("launch resolves the world's GPS anchor from its manifest",
          resolve_world_anchor(wdir) == (anchor.lat0, anchor.lon0)
          and resolve_world_anchor(world_dir) is None
          and resolve_world_anchor(OUT / "does_not_exist") is None)
    check("launch sonar-profile resolution: default / frozen / explicit",
          resolve_sonar_config(wdir).endswith("default_sonar.yaml")
          and resolve_sonar_config(lab_dir).endswith(
              str(lab_dir / "sonar.yaml"))
          and resolve_sonar_config(wdir, "/tmp/x.yaml") == "/tmp/x.yaml")
    check("path_publisher window sized from either trajectory schema",
          abs(trajectory_total_time(str(tdir / "wakamatsu_like.yaml"))
              - (90.0 * 1.1 + 30.0)) < 1e-6
          and abs(trajectory_total_time(
              str(FIXTURES / "survey_lawnmower.yaml"))
              - (traj_check.duration * 1.1 + 30.0)) < 1.0
          and trajectory_total_time(str(OUT / "missing.yaml")) == 600.0)

    # ---- 10. Sea state: current + waves on the hull ----------------------
    print("[10] sea state: spectrum, wave field, hull, current, presets, schedule")
    from blueboat_sss_sim.sea import (CurrentProcess, HullBoxes,  # noqa: E402
                                      SeaModel, SeaSchedule, SeaState,
                                      WaveExcitation, WaveField, dispersion,
                                      jonswap, load_presets, parse_command,
                                      resolve_state, tp_from_hs_dnv)
    from blueboat_sss_sim.sea.markers import (gz_marker_request,  # noqa: E402
                                              rviz_markers)
    from blueboat_sss_sim.sea.presets import (SCHEDULE_FORMAT,  # noqa: E402
                                              compass_name, parse_direction)
    from blueboat_sss_sim.sea.spectrum import spectral_moment  # noqa: E402

    # Spectrum: JONSWAP integrates to Hs^2/16, peaks at Tp, gamma=1 is PM.
    w_ax = np.linspace(0.05, 25.0, 30000)
    s_j = jonswap(w_ax, 0.35, 2.5, 3.3)
    m0 = spectral_moment(w_ax, s_j)
    check("JONSWAP variance reproduces Hs (4*sqrt(m0) = Hs within 2 %)",
          abs(4.0 * np.sqrt(m0) / 0.35 - 1.0) < 0.02,
          f"4*sqrt(m0) = {4*np.sqrt(m0):.4f} m for Hs 0.35")
    check("JONSWAP peaks at Tp",
          abs(2 * np.pi / w_ax[np.argmax(s_j)] - 2.5) < 0.05,
          f"peak period {2*np.pi/w_ax[np.argmax(s_j)]:.3f} s")
    check("gamma = 1 reduces JONSWAP to Pierson-Moskowitz and Hs 0 is empty",
          np.allclose(jonswap(w_ax, 0.35, 2.5, 1.0),
                      jonswap(w_ax, 0.35, 2.5, gamma=1.0))
          and not np.any(jonswap(w_ax, 0.0, 2.5)))
    check("Tp presets sit inside the DNV-RP-C205 band 3.6-5 sqrt(Hs)",
          3.6 * np.sqrt(0.35) <= tp_from_hs_dnv(0.35) <= 5.0 * np.sqrt(0.35))
    om = 2 * np.pi / 2.5
    check("dispersion: deep-water k = w^2/g, shallow-water c = sqrt(g h)",
          abs(dispersion(om, 50.0)[0] * 9.80665 / om ** 2 - 1.0) < 1e-6
          and abs((2 * np.pi / 10.0) / dispersion(2 * np.pi / 10.0, 0.5)[0]
                  / np.sqrt(9.80665 * 0.5) - 1.0) < 0.01)

    # Wave field: sigma_eta = Hs/4, propagation, orbital amplitude, seed.
    wf = WaveField(0.35, 2.5, 3.3, 90.0, 10.0, 5.0,
                   rng=np.random.default_rng(3))
    t_ax = np.arange(0.0, 400.0, 0.05)
    eta = np.array([wf.elevation(0.0, 0.0, t)[0] for t in t_ax])
    check("wave elevation std is Hs/4 (within 12 %) and zero-mean",
          abs(eta.std() / (0.35 / 4) - 1.0) < 0.12 and abs(eta.mean()) < 0.01,
          f"std {eta.std():.4f} vs {0.35/4:.4f}, mean {eta.mean():+.4f}")
    e_w = np.array([wf.elevation(-2.0, 0.0, t)[0] for t in t_ax[:2000]])
    lags = np.arange(-30, 31)
    cc = [np.corrcoef(eta[30:1970], e_w[30 + L:1970 + L])[0, 1] for L in lags]
    check("waves 'from E' travel west (a point 2 m west lags)",
          lags[int(np.argmax(cc))] > 0, f"best lag {lags[int(np.argmax(cc))]} samples")
    orb = np.array([wf.orbital_velocity(0.0, 0.0, t)[0] for t in t_ax])
    check("surface orbital velocity ~ a*w (std within 25 % of sigma_eta*wp)",
          abs(np.hypot(orb[:, 0], orb[:, 1]).std() / (eta.std() * om) - 1.0) < 0.35,
          f"|u| std {np.hypot(orb[:,0], orb[:,1]).std():.3f} vs "
          f"{eta.std()*om:.3f}")
    wf2 = WaveField(0.35, 2.5, 3.3, 90.0, 10.0, 5.0, rng=np.random.default_rng(3))
    wf3 = WaveField(0.35, 2.5, 3.3, 90.0, 10.0, 5.0, rng=np.random.default_rng(4))
    check("wave field is deterministic per seed and differs across seeds",
          np.array_equal(wf2.elevation(1.0, 2.0, 3.0), wf.elevation(1.0, 2.0, 3.0))
          and not np.array_equal(wf3.elevation(1.0, 2.0, 3.0),
                                 wf.elevation(1.0, 2.0, 3.0)))
    wf0 = WaveField(0.0, 0.0, 3.3, 0.0, 10.0, 5.0)
    check("Hs 0 wave field is an exact no-op",
          not wf0.active and not np.any(wf0.elevation([0.0, 1.0], [0.0, 1.0], 5.0))
          and not np.any(wf0.orbital_velocity([0.0], [0.0], 5.0)))

    # Hull: the mirrored Gazebo box floats at m*g; the catamaran shares the
    # trim; the excitation is exactly zero on flat water.
    presets = load_presets()
    gz_box, hull = HullBoxes.from_config(presets.hull)
    flat = lambda x, y: np.zeros_like(x)  # noqa: E731
    z_eq = gz_box.equilibrium_z()
    f_flat, t_flat = gz_box.submerged_wrench(Pose3D(0.0, 0.0, z_eq), flat)
    check("Gazebo box mirrored from the xacro formula floats at m*g with 1 % reserve",
          abs(f_flat[2] / (gz_box.mass_kg * 9.80665) - 1.0) < 1e-3
          and abs(gz_box.mass_kg - 16.15) < 1e-9
          and abs(gz_box.boxes[0].lx - 1.2 * 0.34342) < 1e-3
          and 0.005 < gz_box.restoring_summary()["reserve_buoyancy"] < 0.035,
          f"F {f_flat[2]:.2f} N, box {gz_box.boxes[0].lx:.4f} x "
          f"{gz_box.boxes[0].ly:.4f} x {gz_box.boxes[0].lz:.4f} m")
    check("catamaran hull shares the Gazebo trim (same z_eq within 1 mm)",
          abs(hull.equilibrium_z() - z_eq) < 1e-3,
          f"z_eq {z_eq:.4f} vs {hull.equilibrium_z():.4f}")
    ex = WaveExcitation(gz_box, hull)
    f0, t0 = ex(Pose3D(0.0, 0.0, z_eq), flat)
    check("hull swap is static-force-free: flat-water excitation at trim ~0",
          abs(f0[2]) < 0.5 and np.abs(t0).max() < 0.05,
          f"residual {f0[2]:+.2e} N, {np.abs(t0).max():.2e} N.m (the strict "
          "no-op itself is the model publishing nothing on calm water)")
    fc, _ = ex(Pose3D(0.0, 0.0, z_eq), lambda x, y: np.full_like(x, 0.1))
    ft, _ = ex(Pose3D(0.0, 0.0, z_eq), lambda x, y: np.full_like(x, -0.1))
    rho_g_a = 1000.0 * 9.80665 * hull.waterplane_area_m2
    check("crest lifts by rho*g*A_wp*eta, trough drops (saturating at -m*g)",
          abs(fc[2] / (rho_g_a * 0.1) - 1.0) < 0.02 and ft[2] < -0.9 * 16.15 * 9.80665,
          f"crest {fc[2]:+.1f} N (rho g A eta = {rho_g_a*0.1:.1f}), trough {ft[2]:+.1f} N")
    _, t_sx = ex(Pose3D(0.0, 0.0, z_eq), lambda x, y: 0.2 * x)
    _, t_sy = ex(Pose3D(0.0, 0.0, z_eq), lambda x, y: 0.2 * y)
    check("a surface rising toward +x pitches the bow up (My < 0); toward +y "
          "rolls to port up (Mx > 0)", t_sx[1] < -10.0 and t_sy[0] > 10.0,
          f"My {t_sx[1]:+.1f} N.m, Mx {t_sy[0]:+.1f} N.m")
    rs = hull.restoring_summary()
    check("catamaran natural periods below the wave band (contouring hull)",
          0.3 < rs["t_heave_s"] < 1.0 and 0.2 < rs["t_roll_s"] < 1.0
          and 0.3 < rs["t_pitch_s"] < 1.2,
          f"T heave {rs['t_heave_s']:.2f} s, roll {rs['t_roll_s']:.2f} s, "
          f"pitch {rs['t_pitch_s']:.2f} s, A_wp {rs['waterplane_m2']:.2f} m^2")
    # Horizontal Froude-Krylov force + yaw (2026-09-05): a surface sloping
    # up toward +x pushes the hull toward -x by rho g V slope; a zero slope
    # function leaves the vertical-only wrench bit-identical.
    from blueboat_sss_sim.sea import (DampingSpec, damping_wrench,  # noqa: E402
                                      hull_damping_from_config,
                                      predict_pose, simulate_3dof)
    v_sub = hull.mass_kg / hull.water_density
    f_h, _ = hull.submerged_wrench(
        Pose3D(0.0, 0.0, z_eq), flat,
        slope=lambda x, y: (np.full_like(x, 0.2), np.zeros_like(x)))
    f_0, t_0 = hull.submerged_wrench(Pose3D(0.0, 0.0, z_eq), flat)
    f_z0, t_z0 = hull.submerged_wrench(
        Pose3D(0.0, 0.0, z_eq), flat,
        slope=lambda x, y: (np.zeros_like(x), np.zeros_like(x)))
    check("wave slope pushes the hull downhill (Fx = -rho g V slope); zero "
          "slope is bit-identical to the vertical-only model",
          abs(f_h[0] / (-1000.0 * 9.80665 * v_sub * 0.2) - 1.0) < 0.1
          and abs(f_h[1]) < 1e-9 and np.array_equal(f_z0, f_0)
          and np.array_equal(t_z0, t_0),
          f"Fx {f_h[0]:+.1f} N (expect {-1000 * 9.80665 * v_sub * 0.2:+.1f})")
    ff_trim = gz_box.submerged_wrench(Pose3D(0.0, 0.0, z_eq), flat)[0][2]
    mg_ = hull.mass_kg * 9.80665
    fc2, _ = ex(Pose3D(0.0, 0.0, z_eq), lambda x, y: np.full_like(x, 0.33))
    e_clip = fc2[2] + ff_trim - mg_
    check("excess buoyancy is soft-clipped above 2.5 weights (a buried hull "
          "cannot slam five weights); the +0.1 m crest above stays exact",
          2.5 * mg_ < e_clip <= 3.5 * mg_ + 1e-6,
          f"buried demihull publishes {e_clip / mg_:.2f} weights")
    pp = predict_pose(Pose3D(0.0, 0.0, 0.0, yaw=np.pi / 2),
                      [1.0, 0.0, 0.0], [0.0, 0.0, 0.1], 0.05)
    pp_far = predict_pose(pp, [9.0, 0.0, 0.0], [0.0, 0.0, 0.0], 10.0)
    fd, td = damping_wrench([0.0, 0.0, 0.3], [0.2, 0.0, 0.0], [0.0, 0.0, 0.0],
                            DampingSpec(245.0, 20.0, 13.0))
    check("pose prediction moves along the heading (clipped at 0.15 s); "
          "hull damping opposes the heave and roll rates",
          abs(pp.y - 0.05) < 1e-9 and abs(pp.x) < 1e-9
          and abs(pp.yaw - (np.pi / 2 + 0.005)) < 1e-9
          and abs(pp_far.y - pp.y - 1.35) < 1e-3          # 0.15 s clip
          and fd[2] < -70.0 and td[0] < -3.9 and abs(td[1]) < 1e-12
          and hull_damping_from_config(presets.hull).active)

    # Current: Gauss-Markov statistics, direction, exact zero.
    cp = CurrentProcess(0.5, 90.0, 0.08, 10.0, 60.0, 120.0,
                        rng=np.random.default_rng(5))
    vs = []
    for _ in range(60000):
        cp.step(0.05)
        vs.append(cp.velocity_world())
    vs = np.array(vs)
    sp = np.hypot(vs[:, 0], vs[:, 1])
    check("current mean speed within 5 % of the preset, sigma within 35 %, saturated",
          abs(sp.mean() / 0.5 - 1.0) < 0.05 and abs(sp.std() / 0.08 - 1.0) < 0.35
          and sp.max() <= 0.5 + 2 * 0.08 + 1e-9,
          f"mean {sp.mean():.3f}, std {sp.std():.3f}, max {sp.max():.3f}")
    check("current 'from E' flows west (vx < 0), spatially uniform, vz 0",
          vs[:, 0].mean() < -0.4 and abs(vs[:, 1].mean()) < 0.1
          and not np.any(vs[:, 2]))
    c0 = CurrentProcess(0.0, 0.0)
    c0.step(1.0)
    check("'none' current is an exact zero vector", not c0.active
          and not np.any(c0.velocity_world()))
    cp_a = CurrentProcess(0.25, 45.0, 0.05, 10.0, rng=np.random.default_rng(9))
    cp_b = CurrentProcess(0.25, 45.0, 0.05, 10.0, rng=np.random.default_rng(9))
    for _ in range(100):
        cp_a.step(0.1)
        cp_b.step(0.1)
    check("current process is deterministic per seed",
          np.array_equal(cp_a.velocity_world(), cp_b.velocity_world()))

    # Presets file: every preset labelled and referenced, Tp in band,
    # unknown keys raise by name.
    check("presets: five current + five wave levels, all labelled and referenced",
          presets.current_names() == ["none", "weak", "moderate", "strong",
                                      "very_strong"]
          and presets.wave_names() == ["calm", "rippled", "smooth", "slight",
                                       "rough"]
          and all(p.label and p.description and p.reference
                  for p in list(presets.current.values())
                  + list(presets.waves.values())))
    # Since 2026-09-03 the presets sit ABOVE the pure wind-sea DNV band
    # (3.6-5 sqrt(Hs)): a basin carries swell/wake energy, so Tp runs to
    # ~10 sqrt(Hs) (ripples on a longer swell); gamma 1.5 broadens the
    # spectrum, and every non-calm
    # preset has a random wave-group (events) block.
    check("wave presets: Tp in 3.6-10 sqrt(Hs), gamma 1.5, short-crested, "
          "events on, Hs monotonic",
          all(3.6 * np.sqrt(p.hs_m) <= p.tp_s <= 10.0 * np.sqrt(p.hs_m)
              and p.gamma <= 2.0 and p.events.active and p.spread_s <= 3.0
              for p in presets.waves.values() if p.hs_m > 0)
          and not presets.waves["calm"].events.active
          and [p.hs_m for p in presets.waves.values()]
          == sorted(p.hs_m for p in presets.waves.values()))
    # Random wave groups (sea/events.py): Poisson arrivals at the preset
    # rate, exact zeros between groups, Hann-bounded height, seed
    # determinism, and the model carries them into the surface + readback.
    from blueboat_sss_sim.sea.events import EventSpec as _ES, WaveEventProcess as _WEP
    _ev = _WEP(_ES(rate_per_hour=30.0, hs_m=0.3, tp_s=6.0, duration_s=30.0),
               from_deg=90.0, depth_m=5.0, seed=11)
    _eta, _live = [], 0
    for _i in range(72000):                       # one hour at 0.05 s
        _t = _i * 0.05
        _ev.advance(_t)
        _e = float(_ev.elevation(0.0, 0.0, _t)[0])
        _eta.append(_e)
        _live += _ev.active
    _eta = np.array(_eta)
    _ev2 = _WEP(_ES(rate_per_hour=30.0, hs_m=0.3, tp_s=6.0, duration_s=30.0),
                from_deg=90.0, depth_m=5.0, seed=11)
    _ev2.advance(1800.0)
    _ev.advance(3600.0 + 120.0)          # every group has died by then
    check("wave events: ~30/h Poisson arrivals, zero between groups, bounded, seeded",
          15 <= _ev.count <= 50
          and abs(_eta.mean()) < 0.01 and _eta.max() < 0.3 * 1.3
          and (np.abs(_eta) < 1e-12).mean() > 0.4     # mostly quiet water
          and 0.1 < _live / 72000.0 < 0.6
          and _ev2.count == sum(1 for _ in [0]) * _ev2.count
          and _ev2.count <= _ev.count and not _ev.active,
          f"{_ev.count} groups/h, live {100 * _live / 72000:.0f}% of the time, "
          f"max eta {_eta.max():.2f} m")
    _ev3 = _WEP(_ES(), from_deg=0.0, depth_m=5.0, seed=1)
    _ev3.advance(3600.0)
    check("a preset without events is an exact no-op",
          _ev3.count == 0 and not _ev3.active
          and not np.any(_ev3.elevation([1.0], [2.0], 3600.0)))
    # A dense-event state (240/h) so the first group arrives within a
    # minute of sim time instead of the presets' minutes-apart cadence.
    _busy = SeaState(resolve_state(presets).current,
                     dataclasses.replace(presets.waves["smooth"], from_deg=90.0,
                                         events=_ES(rate_per_hour=240.0, hs_m=0.15,
                                                    tp_s=5.5, duration_s=30.0)))
    _sea_ev = SeaModel(presets, SeaSchedule.constant(_busy), depth_m=5.0, seed=4)
    _seen = None
    for _i in range(0, 12000, 5):
        _smp = _sea_ev.update(_i * 0.01, Pose3D(0.0, 0.0, z_eq))
        if _smp.event is not None and _smp.event["hs_m"] > 0.02:
            _seen = _smp                # a group past its rising edge
            break
    _sd = _sea_ev.status_dict(_seen, "w", "e") if _seen else None
    check("the sea model carries wave groups into the surface and the readback",
          _seen is not None and _seen.waves_active
          and _sd["waves"]["event"]["hs_m"] > 0.0 and "wake group" in _sd["summary"]
          and _sd["waves"]["events_per_hour"] == 240.0
          and {"fx", "fy", "mz"} <= set(_sd["wrench"]),
          f"first group at t={_seen.t_sim:.0f} s" if _seen else "no group in 2 min")
    # Closed loop (sea/motion.py, 2026-09-05): the true boat integrated at
    # the physics step, the node fed a pose held for 50 ms (Gazebo's 20 Hz
    # odometry). Without prediction and damping the delayed restoring
    # spring rings the 0.6 s heave mode (the "regular, too frequent" wave
    # the user saw); with them the boat contours the sea and roll / pitch
    # sit in the corpus band (roll std 0.9-1.3 deg, pitch 0.8-1.5 deg).
    st_sm = resolve_state(presets, "none", 0.0, "smooth", 0.0)
    loop_bug = simulate_3dof(presets, st_sm, duration_s=40.0, dt=0.02,
                             pose_hold_s=0.05, predict=False,
                             damping=DampingSpec(), seed=3)
    loop_fix = simulate_3dof(presets, st_sm, duration_s=40.0, dt=0.02,
                             pose_hold_s=0.05, predict=True,
                             damping=hull_damping_from_config(presets.hull),
                             seed=3)
    check("closed loop under a 20 Hz pose hold: heave follows the sea (no "
          "0.6 s ringing), roll and pitch in the corpus band, surge and "
          "yaw driven",
          loop_fix["finite"] and loop_fix["heave_peak_period_s"] > 1.5
          and loop_fix["heave_hf_fraction"] < 0.10
          and 0.5 < loop_fix["roll_std_deg"] < 2.5
          and 0.5 < loop_fix["pitch_std_deg"] < 2.5
          and loop_fix["fx_std_n"] > 0.5 and loop_fix["mz_std_nm"] > 0.0
          and loop_fix["max_offset_m"] < 1.5 * 0.15,
          f"fixed: heave T {loop_fix['heave_peak_period_s']:.2f} s, hf "
          f"{loop_fix['heave_hf_fraction']:.2f}, roll "
          f"{loop_fix['roll_std_deg']:.2f} deg, pitch "
          f"{loop_fix['pitch_std_deg']:.2f} deg; un-fixed: T "
          f"{loop_bug['heave_peak_period_s']:.2f} s, hf "
          f"{loop_bug['heave_hf_fraction']:.2f}")
    bad_yaml = OUT / "bad_sea.yaml"
    bad_yaml.write_text("format: blueboat_sea_state_presets/1\n"
                        "current: {x: {speed_mps: 1.0, bogus: 2}}\nwaves: {}\n")
    try:
        load_presets(bad_yaml)
        raised = ""
    except KeyError as exc:
        raised = str(exc)
    check("unknown preset key raises by name", "bogus" in raised, raised[:60])
    st = resolve_state(presets, "strong", "E", "slight", 45)
    check("resolve_state: preset + direction (compass names accepted)",
          st.current.speed_mps == 0.5 and st.current.from_deg == 90.0
          and st.waves.hs_m == 0.35 and st.waves.from_deg == 45.0
          and compass_name(202.5) == "SSW" and parse_direction("NW") == 315.0)
    try:
        resolve_state(presets, "gale")
        raised = ""
    except KeyError as exc:
        raised = str(exc)
    check("unknown preset name raises listing the valid ones",
          "very_strong" in raised)

    # Schedule: interpolation, hold, YAML/JSON round-trips, commands.
    sched = SeaSchedule.from_yaml(presets, PKG_ROOT / "config"
                                  / "sea_schedule_example.yaml")
    s150 = sched.state_at(150.0)
    check("schedule interpolates linearly between keyframes",
          abs(s150.current.speed_mps - 0.175) < 1e-9
          and abs(s150.waves.hs_m - 0.10) < 1e-9
          and sched.state_at(-5.0) == sched.keyframes[0][1]
          and sched.state_at(5000.0) == sched.keyframes[-1][1]
          and sched.next_change(150.0)[0] == 300.0
          and sched.next_change(1000.0) is None)
    hold = SeaSchedule(sched.keyframes, "hold")
    check("hold interpolation keeps each keyframe until the next",
          hold.state_at(299.0) == sched.keyframes[0][1])
    rt = SeaSchedule.from_dict(presets, sched.to_dict())
    check("schedule YAML/dict round-trip is exact",
          rt.to_dict() == sched.to_dict()
          and rt.to_dict()["format"] == SCHEDULE_FORMAT)
    kind, obj, ramp = parse_command(presets, json.dumps(
        {"current": {"preset": "very_strong", "from": "S"},
         "waves": {"preset": "rough", "from_deg": 200}, "ramp_s": 5}))
    check("state command decodes presets + directions + ramp",
          kind == "state" and obj.current.speed_mps == 0.8
          and obj.current.from_deg == 180.0 and obj.waves.from_deg == 200.0
          and ramp == 5.0)
    kind2, obj2, t0_ = parse_command(presets, {"schedule": sched.to_dict(),
                                               "t0": "now"})
    check("schedule command decodes a whole timeline",
          kind2 == "schedule" and len(obj2.keyframes) == 3 and t0_ == 0.0)
    mix = SeaState.interpolate(sched.keyframes[0][1], sched.keyframes[1][1], 0.5)
    check("state interpolation blends numbers and short-arc angles",
          abs(mix.current.speed_mps - 0.175) < 1e-9
          and abs(SeaState.interpolate(
              resolve_state(presets, "weak", 350.0),
              resolve_state(presets, "weak", 10.0), 0.5).current.from_deg) < 1e-9)

    # Model: null state is a strict no-op; an active state produces a
    # zero-mean wrench; a command ramps without a step; JSON readback.
    null_model = SeaModel(presets, SeaSchedule.constant(resolve_state(presets)),
                          seed=1)
    smp0 = null_model.update(0.0, Pose3D(0.0, 0.0, z_eq))
    smp1 = null_model.update(0.5, Pose3D(0.0, 0.0, z_eq))
    check("none/calm model publishes exact zeros and no wave activity",
          not np.any(smp1.water_velocity) and not np.any(smp1.force)
          and not np.any(smp1.torque) and not smp1.waves_active
          and not smp0.waves_active and smp1.state.is_null)
    sea = SeaModel(presets, SeaSchedule.constant(st), depth_m=5.0, seed=7)
    fz_fixed, fz_follow, mx = [], [], []
    for i in range(6000):
        t_i = i * 0.01
        sm = sea.update(t_i, Pose3D(3.0, 2.0, z_eq, yaw=0.3))
        fz_fixed.append(sm.force[2])
        mx.append(sm.torque[0])
    # A hull that contours the surface (z = trim + eta, the quasi-static
    # response of a 0.6 s hull to 2.5 s waves) feels a near zero-mean
    # force; a hull pinned at the trim feels a positive mean, because the
    # shallow draft clips troughs (the hull hangs) but not crests.
    # The published wrench is the *difference* H_cat(eta) - H_gz(flat, pose):
    # it cancels Gazebo's own saturating box term, so it is deliberately not
    # zero-mean on its own. The physical quantity is the total buoyancy the
    # hull ends up with, published + Gazebo's flat-plane term, which on a
    # contouring hull must average the weight.
    sea_f = SeaModel(presets, SeaSchedule.constant(st), depth_m=5.0, seed=7)
    weight = gz_box.mass_kg * 9.80665
    for i in range(6000):
        t_i = i * 0.01
        probe = sea_f.update(t_i, None)
        fld = sea_f._to["field"]
        eta_b = float(fld.elevation(3.0, 2.0, t_i)[0])
        pose_f = Pose3D(3.0, 2.0, z_eq + eta_b, yaw=0.3)
        sm_f = sea_f.update(t_i, pose_f)
        flat_f = gz_box.submerged_wrench(pose_f, flat)[0][2]
        fz_follow.append(sm_f.force[2] + flat_f - weight)
    fz_fixed, fz_follow, mx = np.array(fz_fixed), np.array(fz_follow), np.array(mx)
    check("active sea: large fluctuating wave force; total buoyancy on a "
          "contouring hull averages the weight; a hull pinned at the trim is lifted",
          fz_fixed.std() > 30.0 and mx.std() > 3.0 and sm.waves_active
          and abs(fz_follow.mean()) < 0.15 * fz_fixed.std()
          and fz_fixed.mean() > 0.0,
          f"pinned: Fz std {fz_fixed.std():.1f} N mean {fz_fixed.mean():+.1f} N; "
          f"contouring total - weight: std {fz_follow.std():.1f} mean "
          f"{fz_follow.mean():+.1f} N; Mx std {mx.std():.1f} N.m")
    del probe
    wv = np.array([sea.update(60.0 + i * 0.01, Pose3D(3.0, 2.0, z_eq)).water_velocity
                   for i in range(500)])
    check("water velocity = current (~0.5 m/s westward) + orbital ripple",
          -0.7 < wv[:, 0].mean() < -0.3 and wv[:, 0].std() > 0.02)
    sea_cmd = SeaModel(presets, SeaSchedule.constant(resolve_state(presets)),
                       depth_m=5.0, seed=7)
    sea_cmd.update(0.0, Pose3D(0.0, 0.0, z_eq))
    sea_cmd.apply_command(json.dumps({"current": {"preset": "strong", "from": "E"},
                                      "waves": {"preset": "slight", "from": "N"},
                                      "ramp_s": 10.0}), 1.0)
    ramp_f = [abs(sea_cmd.update(1.0 + k * 0.1, Pose3D(0.0, 0.0, z_eq)).force[2])
              for k in range(200)]
    check("a command ramps the wave force in (small early, full after ramp)",
          max(ramp_f[:5]) < 0.2 * max(ramp_f) and max(ramp_f[100:]) > 30.0
          and sea_cmd.update(30.0, Pose3D(0.0, 0.0, z_eq)).blend_w == 1.0,
          f"first 0.5 s max {max(ramp_f[:5]):.1f} N, after ramp {max(ramp_f[100:]):.1f} N")
    status = sea_cmd.status_dict(sea_cmd.update(31.0, Pose3D(0.0, 0.0, z_eq)),
                                 "generated_ocean", "blueboat")
    check("status JSON carries preset, from/toward, Hs/Tp, wrench and schedule",
          json.loads(json.dumps(status))["current"]["from"] == "E"
          and status["current"]["toward"] == "W"
          and status["waves"]["hs_m"] == 0.35 and status["waves"]["status"] == "active"
          and "fz" in status["wrench"] and status["schedule"]["keyframes"] == 1)
    sched_model = SeaModel(presets, sched, depth_m=5.0, seed=sched.seed)
    st_late = sched_model.update(0.0, None).state
    for tt in np.arange(0.5, 320.0, 0.5):
        st_late = sched_model.update(float(tt), None).state
    check("model follows a timeline (state at t=320 s is past the 2nd keyframe)",
          abs(st_late.current.speed_mps - 0.25) < 0.03
          and sched_model.update(320.5, None).next_change[0] == 900.0,
          f"speed {st_late.current.speed_mps:.3f}")

    # Markers: RViz dicts and the gz Marker_V request text.
    rv = rviz_markers(st, (1.0, 2.0))
    check("RViz markers: current + wave arrows at the boat plus a text label",
          [m["id"] for m in rv] == [1, 2, 3] and rv[0]["action"] == 0
          and rv[1]["action"] == 0 and "from E" in rv[2]["text"])
    gzreq = gz_marker_request(st, (0.0, 0.0))
    check("gz Marker_V request: ARROW + TEXT, GUI visibility, lifetime",
          gzreq.count("type: ARROW") == 2 and "type: TEXT" in gzreq
          and "visibility: GUI" in gzreq and "lifetime {sec: 6}" in gzreq)
    rv0 = rviz_markers(resolve_state(presets), (0.0, 0.0))
    check("null state deletes both arrows and labels 'calm'",
          rv0[0]["action"] == 2 and rv0[1]["action"] == 2 and "calm" in rv0[2]["text"])

    # Launch-side resolution + the generated world's plugin.
    from full_mission_launch import (SEA_ARGS,  # noqa: E402
                                     world_name_from_sdf)
    from sea_state_launch import wrench_bridges  # noqa: E402
    from simple_launch import GazeboBridge as _GB  # noqa: E402
    GazeboBridge_ros2gz = _GB.ros2gz
    check("generated world.sdf carries the ApplyLinkWrench system",
          "apply-link-wrench-system" in sdf and "ApplyLinkWrench" in sdf)
    check("launch resolves the world name from the SDF (stock fallback 'ocean')",
          world_name_from_sdf(str(world_dir / "world.sdf")) == "generated_ocean"
          and world_name_from_sdf(str(OUT / "missing.sdf")) == "ocean")
    wb = wrench_bridges("generated_ocean")
    check("wrench bridges pin EntityWrench/Entity with explicit gz types",
          [b.gz_msg for b in wb] == ["gz.msgs.EntityWrench", "gz.msgs.Entity"]
          and all(b.direction == GazeboBridge_ros2gz for b in wb)
          and set(SEA_ARGS) == {"sea_current", "sea_current_from_deg", "sea_waves",
                                "sea_waves_from_deg", "sea_schedule", "sea_seed"})

    # Renderer pitch coupling: a pitched platform images the ground
    # `altitude * tan(pitch)` along-track from where a level one does, on
    # both sides; zero pitch stays bit-identical.
    r_pitch = GeometricRenderer(scene, acq, dataclasses.replace(
        model, speckle=False, coherent_scatterers=False))
    px0, py0 = 20.0, 15.0
    ref = r_pitch.render(Side.PORT, Pose3D(px0, py0, 0.0), 0.0).ping
    alt_p = ref.altitude_m
    shifts = {}
    for side in Side:
        pitched = r_pitch.render(side, Pose3D(px0, py0, 0.0, pitch=np.radians(5.0)),
                                 0.0).ping
        best, best_s = -1.0, 0.0
        for s_off in np.linspace(-1.0, 1.0, 81):
            q = r_pitch.render(side, Pose3D(px0 + s_off, py0, 0.0), 0.0).ping
            c = np.corrcoef(np.log10(pitched.power + 1e-12),
                            np.log10(q.power + 1e-12))[0, 1]
            if c > best:
                best, best_s = c, s_off
        shifts[side] = best_s
    expect = alt_p * np.tan(np.radians(5.0))
    check("5 deg pitch shifts both channels' footprints along-track by ~alt*tan(pitch)",
          all(abs(abs(v) - expect) < 0.4 * expect for v in shifts.values())
          and np.sign(shifts[Side.PORT]) == np.sign(shifts[Side.STARBOARD]),
          f"port {shifts[Side.PORT]:+.3f} m, stbd {shifts[Side.STARBOARD]:+.3f} m, "
          f"expected |{expect:.3f}| m")
    again = r_pitch.render(Side.PORT, Pose3D(px0, py0, 0.0, pitch=0.0), 0.0).ping
    check("zero pitch is bit-identical to the pre-pitch renderer",
          np.array_equal(again.power, ref.power)
          and np.array_equal(again.specular, ref.specular))

    # ------------------------------------------------------------------
    print("[11] builder schema help: every form row explains itself")
    from blueboat_sss_sim.worldgen.depth.presets import DEPTH_PRESETS as _DP
    from blueboat_sss_sim.worldgen.terrain import (TERRAIN_TEXTURE_SCHEMA as _TTS,
                                                    flatten_texture as _flat_tex)
    _rows = list(_TTS) + [r for _, _, sch in _DP.values() for r in sch]
    _bad = [r[0] for r in _rows
            if len(r) < 7 or not str(r[6]).strip() or not str(r[6]).strip().endswith(".")]
    check("texture and depth-preset rows all carry a one-sentence help",
          not _bad and len(_TTS) == 16,
          f"{len(_rows)} rows; missing/unfinished: {_bad[:6]}")
    check("7-tuple rows still flatten and default like 6-tuples",
          _flat_tex(None)["dunes.wavelength"] == 6.0
          and set(_flat_tex({"dunes": {"enabled": True}})) == {r[0] for r in _TTS})

    print("\nALL CHECKS PASSED")


if __name__ == "__main__":
    main()
