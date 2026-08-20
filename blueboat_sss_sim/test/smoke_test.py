"""Offline end-to-end smoke test (no ROS required).

Exercises the full ROS-free pipeline exactly as the ROS nodes drive it:

    mission YAML -> world bundle -> lawnmower trajectory -> per-ping
    render -> noise -> byte-exact encode -> decode round-trip ->
    waterfall tiles -> auto labels -> YOLO dataset export

Run with:  python3 -m test.smoke_test   (from the package root)
Artifacts land in /tmp/blueboat_sss_smoke for visual inspection.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from blueboat_sss_sim.core.types import Pose3D, Side  # noqa: E402
from blueboat_sss_sim.dataset.exporter import ExportConfig, YoloDatasetWriter  # noqa: E402
from blueboat_sss_sim.dataset.labeler import LabelConfig, TileLabeler  # noqa: E402
from blueboat_sss_sim.dataset.waterfall import (WaterfallBuilder,  # noqa: E402
                                            WaterfallTileConfig)
from blueboat_sss_sim.mission.generate import generate_mission  # noqa: E402
from blueboat_sss_sim.mission.patterns import WaypointTrajectory  # noqa: E402
from blueboat_sss_sim.sonar.config import SonarConfig  # noqa: E402
from blueboat_sss_sim.sonar.encoder import PingEncoder, parse_frame  # noqa: E402
from blueboat_sss_sim.sonar.noise import GainDrift, apply_ping_noise  # noqa: E402
from blueboat_sss_sim.sonar.renderer import GeometricRenderer  # noqa: E402
from blueboat_sss_sim.worldgen.scene import SceneModel  # noqa: E402

OUT = Path("/tmp/blueboat_sss_smoke")


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
    altitudes = []
    fbr_probe: list[np.ndarray] = []   # encoded port pings for FBR analysis
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
                fbr_probe.append(np.asarray(enc.pwr_results, dtype=np.float64))
                fbr_alts.append(r.ping.altitude_m)

            if r.contacts:
                contact_pings += 1
            builders[side].add_ping(enc.pwr_results, r.contacts)

    alt = np.array(altitudes)
    check("altitude in shallow regime", bool((alt > 1.0).all() and
                                             (alt < 8.0).all()),
          f"min {alt.min():.2f} m, max {alt.max():.2f} m")
    check("contacts observed on the pass", contact_pings > 0,
          f"{contact_pings} pings with ground-truth contacts")

    # ---- 2b. FBR structure (what downstream bottom-tracking locks onto) ----
    print("[2b] first-bottom-return structure")
    mean_p = np.mean(fbr_probe, axis=0)
    mean_alt = float(np.mean(fbr_alts))
    fbr_bin = int(mean_alt / acq.bin_size_m)
    wc = mean_p[:max(fbr_bin - 8, 1)]
    peak = float(mean_p[:fbr_bin + 40].max())
    contrast_db = 10 * np.log10(peak / max(float(wc.mean()), 1e-9))
    check("water column is quiet vs FBR peak", contrast_db > 8.0,
          f"{contrast_db:.1f} dB gap->peak contrast")
    lock = int(np.argmax(mean_p > 3.0 * np.median(wc)))
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
    peak_dbs = [10*np.log10(max(p.max(), 1)) + model.calibration_db_offset
                for p in fbr_probe]
    check("max_pwr_db in real-capture range (~60-64.2 dB)",
          58.0 < float(np.mean(peak_dbs)) <= 64.2,
          f"mean {np.mean(peak_dbs):.1f} dB (real capture: 63.9)")
    flat = mean_p[fbr_bin + 30:fbr_bin + 120]
    one = fbr_probe[0][fbr_bin + 30:fbr_bin + 120]
    cv = float(one.std() / max(one.mean(), 1e-9))
    check("speckle PDF on flat seabed (CV ~ 1 for Exp(1))",
          0.6 < cv < 1.4, f"CV {cv:.2f}")
    check("no empty far-range bins (sampling tracks bin size)",
          float((mean_p[400:590] < 1.0).mean()) < 0.02,
          f"{(mean_p[400:590] < 1.0).mean():.1%} empty")

    # Downstream bottom-tracking lockability: emulate the fleet's FBR
    # detector (noise floor from first 20 samples, +8 dB threshold,
    # persistence 3) on consecutive moving pings; the tracker bootstrap
    # needs 10 consecutive estimates within a 0.30 m band.
    def _fbr_est(counts: np.ndarray) -> float | None:
        db = 10 * np.log10(np.maximum(counts, 1.0))
        floor = float(db[:20].mean())
        above = db > floor + 8.0
        run = 0
        for i, a in enumerate(above):
            run = run + 1 if a else 0
            if run >= 3:
                return (i - 2) * (acq.range_length_mm / 1000.0) / acq.num_results
        return None

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
          and float((rp12.ping.power[900:1180] == 0).mean()) < 0.05,
          f"frame {len(e12.raw_frame)} B, "
          f"far empty {(rp12.ping.power[900:1180]==0).mean():.1%}")

    # ---- 3. Waterfall + labels + export ----------------------------------------
    print("[3] waterfall tiles, labels, YOLO export")
    writer = YoloDatasetWriter(OUT / "dataset", ExportConfig())
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
    from blueboat_sss.dataset.labeler import YoloBox  # noqa: F401
    bad = 0
    for lbl in (OUT / "dataset" / "labels").rglob("*.txt"):
        for line in lbl.read_text().splitlines():
            vals = [float(v) for v in line.split()[1:]]
            if not all(0.0 <= v <= 1.0 for v in vals):
                bad += 1
    check("all YOLO coords normalized", bad == 0)

    # ---- 4. Visual artifact ------------------------------------------------------
    print("[4] visual artifact")
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
