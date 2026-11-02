"""Slow checks of the field-calibration engine (``calibration.seabed``).

Manual, minutes -- not part of the Stop-hook gate (``test.smoke_test``):

    python3 -m test.calibration_fit_test                 # from the package root
    python3 -m test.calibration_fit_test --equivalence NEW/scores.json \\
        --pass1 ~/calib/validation_scores/scores.json

1. Round trip: a recording simulated with the shipped profile (flat sand,
   noisy, both sides, written as the device's frames) scored against the
   same profile reads zero bias -- level (noisy and expected), crest and FBR
   detection.
2. Recovery: recordings simulated with a known truth (offset +1.5 dB,
   tvg 0.40, pulse-level exponent 1) at two altitudes and two pulses; the
   least-squares fit started from the shipped values recovers them.
3. Refusal: a fit on a validation session without a reason is refused.
4. (optional) Equivalence with the pass-1 reduction: per stratum, the
   level-boat noisy LEVEL bias of a ``score --replay none`` run equals the
   pass-1 scorer's within 0.05 dB.
5. The field-calibration candidate (``config/field_candidate_v1_sonar.yaml``)
   keeps the first bottom return (NC #4): the downstream detector finds it
   on >= 95 % of noisy pings at 0.6-12 m altitude over flat sand, its
   10-ping tracker windows lock at least as often as the real device's do
   (>= 0.55: MEASURED 0.56-0.83 at 0.6 m and 0.65-0.75 at 1.4 m on the beach
   calibration files, gain 4, 2026-10-30), and over a field holding every
   World Builder object class it locks like the shipped profile.

Everything is written to a temporary directory; nothing is read from or
written to the field corpus except the optional pass-1 comparison's JSON.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from blueboat_sss_sim.calibration import seabed
from blueboat_sss_sim.core.types import Pose3D, Side
from blueboat_sss_sim.sonar.config import SonarConfig, find_default_sonar_yaml
from blueboat_sss_sim.sonar.encoder import PingEncoder
from blueboat_sss_sim.sonar.noise import apply_ping_noise
from blueboat_sss_sim.sonar.renderer import GeometricRenderer

RESULTS: list[tuple[str, bool, str]] = []
#: The real device's lowest 10-ping lockable-window fraction on the beach
#: calibration files (bath1-3, gain 4, windows at 0.55-0.75 m and 1.3-1.5 m
#: median altitude): 0.56. A simulated profile that locks less often than
#: this would make bottom tracking harder than the device does.
REAL_LOCK_FLOOR = 0.55


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail
                                                      else ""), flush=True)


def write_recording(path: Path, cfg, *, alt: float, range_mm: int,
                    num_results: int, pulse_us: float, n: int, seed: int,
                    step: float = 0.05) -> Path:
    """A level-boat straight line over flat sand, every ping through the
    node's noise call and the wire encoder, both sides, as raw frames."""
    acq = seabed.acquisition_for(cfg, range_mm, num_results, pulse_us, 4)
    model = cfg.model
    scene = seabed.flat_scene(alt + model.sensor_depth_m,
                              range_mm / 1000.0 + n * step + 5.0)
    rr = GeometricRenderer(scene, acq, model)
    encs = {s: PingEncoder(s, acq, model) for s in Side}
    rng = np.random.default_rng(seed)
    bins = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
    blob = bytearray()
    for k in range(n):
        pose = Pose3D(-0.5 * n * step + k * step, 0.0, 0.0)
        for side in Side:
            pg = rr.render(side, pose, k * 0.05).ping
            pg.power = apply_ping_noise(pg.power, bins < pg.altitude_m, 1.0,
                                        model, rng, specular=pg.specular,
                                        texture=pg.texture,
                                        unoccluded=pg.unoccluded,
                                        ambient=pg.ambient,
                                        speckle_fill=pg.speckle_fill)
            blob += encs[side].encode(pg).raw_frame
    path.write_bytes(bytes(blob))
    return path


def _strata(paths: dict[str, Path], role: str = "calibration"):
    out = []
    for rel, p in paths.items():
        st, _ = seabed.reduce_recording(p, rel=rel, session=f"S-{rel}",
                                        role=role)
        out.extend(st)
    return out


def round_trip(tmp: Path, ex) -> None:
    print("[1] round trip: the shipped profile scored on its own recording")
    cfg = SonarConfig.from_yaml(find_default_sonar_yaml())
    f = write_recording(tmp / "rt.svlog", cfg, alt=1.45, range_mm=20000,
                        num_results=600, pulse_us=66.5, n=400, seed=5)
    strata = _strata({"rt.svlog": f})
    res = seabed.score(strata, cfg, mode="both", replay="none", executor=ex)
    lv = [r["level_bias_db"] for r in res if "level_bias_db" in r]
    le = [r["level_expected_bias_db"] for r in res
          if "level_expected_bias_db" in r]
    cr = [r["crest_bias_db"] for r in res if "crest_bias_db" in r]
    det = [r["sim_fbr_detection"] for r in res]
    check("a simulated recording scored against its own profile reads no "
          "level bias (noisy and expected modes), no crest bias, and the "
          "simulated FBR is always found",
          bool(lv) and max(abs(v) for v in lv) < 0.35
          and max(abs(v) for v in le) < 0.35
          and max(abs(v) for v in cr) < 1.0 and min(det) >= 0.95,
          f"{len(res)} strata; level {np.round(lv, 2).tolist()}, expected "
          f"{np.round(le, 2).tolist()}, crest {np.round(cr, 2).tolist()}, "
          f"detection min {min(det):.3f}")


def recovery(tmp: Path, ex) -> None:
    print("[2] recovery of a known truth by the level fit")
    base = SonarConfig.from_yaml(find_default_sonar_yaml())
    truth = dataclasses.replace(base, model=dataclasses.replace(
        base.model, calibration_db_offset=base.model.calibration_db_offset
        + 1.5, tvg_compensation=0.40, pulse_level_exponent=1.0))
    files = {
        "low.svlog": write_recording(tmp / "low.svlog", truth, alt=1.45,
                                     range_mm=20000, num_results=600,
                                     pulse_us=66.5, n=300, seed=11),
        "high.svlog": write_recording(tmp / "high.svlog", truth, alt=6.5,
                                      range_mm=40000, num_results=600,
                                      pulse_us=106.4, n=300, seed=12),
    }
    strata = _strata(files)
    start = dataclasses.replace(base, model=dataclasses.replace(
        base.model, pulse_level_exponent=1.0))
    t0 = time.time()
    res = seabed.fit(strata, start, ["model.calibration_db_offset",
                                     "model.tvg_compensation"],
                     metric="level", replay="none", executor=ex)
    off, tvg = res.x
    check("the level fit recovers a known offset and TVG from two "
          "altitudes at two pulses (pulse-level exponent fixed at its truth)",
          abs(off - truth.model.calibration_db_offset) < 0.3
          and abs(tvg - 0.40) < 0.05 and res.rms_db < res.rms0_db,
          f"offset {off:.2f} (truth {truth.model.calibration_db_offset:.2f}), "
          f"tvg {tvg:.3f} (truth 0.400), per-stratum rms {res.rms0_db:.2f} -> "
          f"{res.rms_db:.2f} dB, corr {res.corr[0][1]:+.2f}, "
          f"{res.nfev} evaluations, {time.time() - t0:.0f} s")
    print("[3] a fit on a validation session needs a reason")
    for s in strata:
        s.role = "validation"
    try:
        seabed.check_fit_roles(strata, None)
        refused = False
    except PermissionError:
        refused = True
    reused = seabed.check_fit_roles(strata, "test: deliberate reuse")
    check("validation sessions are refused without a reason and recorded "
          "with one", refused and len(reused) == 2, f"reused {reused}")


def equivalence(new: Path, pass1: Path) -> None:
    print("[4] equivalence with the pass-1 reduction")
    a = json.loads(new.read_text())["strata"]
    b = json.loads(pass1.read_text())["strata"]
    key = lambda s: (s["file"], s["side"], s["L"], s["N"], s["pulse_us"],
                     s["band"])
    old = {key(s): s for s in b}
    diffs, missing = [], 0
    for s in a:
        o = old.get(key(s))
        if o is None or ("level_bias_db" in s) != ("level_bias_db" in o):
            missing += 1            # a stratum one side scored, the other not
            continue
        if "level_bias_db" in s:    # < 10 tail bins: no LEVEL on either side
            diffs.append(abs(s["level_bias_db"] - o["level_bias_db"]))
    check("score --replay none reproduces pass 1's LEVEL bias per stratum "
          "within 0.05 dB",
          bool(diffs) and max(diffs) <= 0.05 and missing == 0,
          f"{len(diffs)} strata, max |diff| {max(diffs) if diffs else 0:.4f}"
          f" dB, {missing} unmatched")


def _lock_stats(cfg, scene, alt: float, n: int = 80, seed: int = 9,
                y0: float = 0.0) -> tuple[float, float]:
    """(detection rate, lockable 10-ping window fraction) of the downstream
    detector on ``n`` noisy, encoded port pings along x at ``y0``."""
    from blueboat_sss_sim.analysis.calibration import fbr_bin, scale_to_db
    acq, model = cfg.acquisition, cfg.model
    rr = GeometricRenderer(scene, acq, model)
    enc = PingEncoder(Side.PORT, acq, model)
    rng = np.random.default_rng(seed)
    bins = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
    est = []
    for k in range(n):
        pg = rr.render(Side.PORT, Pose3D(-8.0 + 0.1 * k, y0, 0.0), 0.0).ping
        pg.power = apply_ping_noise(pg.power, bins < pg.altitude_m, 1.0, model,
                                    rng, specular=pg.specular,
                                    texture=pg.texture,
                                    unoccluded=pg.unoccluded,
                                    ambient=pg.ambient,
                                    speckle_fill=pg.speckle_fill)
        e = enc.encode(pg)
        i = fbr_bin(scale_to_db(e.pwr_results, e.min_pwr_db, e.max_pwr_db))
        est.append(None if i is None else i * acq.bin_size_m)
    det = sum(x is not None for x in est) / n
    win = [est[i:i + 10] for i in range(n - 10)]
    lock = sum(all(x is not None for x in w) and max(w) - min(w) <= 0.30
               for w in win) / max(len(win), 1)
    return det, lock


def candidate_lock() -> None:
    print("[5] the field-calibration candidate keeps the first bottom return")
    from blueboat_sss_sim.core.types import GridSpec
    from blueboat_sss_sim.worldgen.materials import MaterialLibrary
    from blueboat_sss_sim.worldgen.objects import (CATALOG, ObjectFieldConfig,
                                                   place_objects)
    from blueboat_sss_sim.worldgen.scene import SceneModel, stamp_objects
    pkg = Path(__file__).resolve().parents[1]
    cand = SonarConfig.from_yaml(pkg / "config" / "field_candidate_v1_sonar.yaml")
    ship = SonarConfig.from_yaml(find_default_sonar_yaml())
    rows = []
    for alt in (0.6, 1.4, 3.0, 7.0, 12.0):
        sc = seabed.flat_scene(alt + cand.model.sensor_depth_m, 30.0)
        rows.append((alt, _lock_stats(cand, sc, alt), _lock_stats(ship, sc, alt)))
    check("flat sand, 0.6-12 m: the candidate's FBR is found on >= 95 % of "
          "noisy pings and its tracker locks on 10-ping windows at least as "
          "often as the real device's (>= 0.55; device 0.56-0.83 at 0.6 m)",
          all(c[0] >= 0.95 and c[1] >= REAL_LOCK_FLOOR for _, c, _ in rows),
          "; ".join(f"{a:g} m: cand {c[0]:.2f}/{c[1]:.2f}, shipped "
                    f"{s[0]:.2f}/{s[1]:.2f}" for a, c, s in rows))
    g = GridSpec(origin_x=-10.0, origin_y=-10.0, resolution=0.05, nx=401,
                 ny=401)
    h = np.full((g.ny, g.nx), -3.0)
    r = np.full((g.ny, g.nx), 0.55)
    field = ObjectFieldConfig.from_dict({"min_separation_m": 0.8, "classes": {
        name: {"amount": {"kind": "count", "value": 2}} for name in CATALOG}})
    objs = place_objects(g, field, MaterialLibrary(), np.random.default_rng(4),
                         height=h)
    ras = stamp_objects(objs, g, h, r)
    sc = SceneModel(grid=g, height=h, reflectivity=r, **ras.scene_kwargs(),
                    material_id=np.zeros((g.ny, g.nx), dtype=np.uint8),
                    material_names=["sand"], objects=objs, walls=[], seed=0,
                    config_snapshot={})
    lines = [_lock_stats(c, sc, 2.85, n=150, y0=y)
             for c in (cand, ship) for y in (-4.0, 0.0, 4.0)]
    check("a field of every object class (2 each) under the swath: the "
          "candidate finds the FBR on >= 95 % of pings and its tracker locks "
          "like the shipped profile's (within 0.05 per line)",
          all(d >= 0.95 and lk >= ls - 0.05
              for (d, lk), (_, ls) in zip(lines[:3], lines[3:])),
          f"{len(objs)} objects ({len({o.type for o in objs})} classes); "
          f"candidate det/lock {[(round(d, 2), round(lk, 2)) for d, lk in lines[:3]]}, "
          f"shipped {[(round(d, 2), round(lk, 2)) for d, lk in lines[3:]]}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--equivalence", type=Path)
    ap.add_argument("--pass1", type=Path)
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args(argv)
    from concurrent.futures import ProcessPoolExecutor
    with tempfile.TemporaryDirectory(prefix="bb_calfit_") as td, \
            ProcessPoolExecutor(a.workers) as ex:
        tmp = Path(td)
        round_trip(tmp, ex)
        recovery(tmp, ex)
    candidate_lock()
    if a.equivalence and a.pass1:
        equivalence(a.equivalence, a.pass1)
    bad = [n for n, ok, _ in RESULTS if not ok]
    print("ALL CHECKS PASSED" if not bad else f"FAILED: {bad}")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
