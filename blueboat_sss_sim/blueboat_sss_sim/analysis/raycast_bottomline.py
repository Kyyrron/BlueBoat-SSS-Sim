"""Does the real processor track the bottom on ray-cast pings? (offline)

The processor (``BlueBoat-SSS`` ``sss_processor_node``) finds the first
bottom return of every ping and tracks it; its altitude georeferences every
seabed image downstream. On a ray-cast run it read about a metre late
(``raycast_assessment.md`` §13.6, §14.1; the 2026-11-11 session). This tool
answers the question for the CURRENT SOURCE on a RECORDED TRACK, without
Gazebo:

1. the poses of a recorded GCS session (``seabed_images/metadata/*.json``
   ``rows_data``: boat x, y, yaw, roll, pitch per ping; heave is not
   recorded, the boat is taken on the still waterline);
2. each ping rendered by the ray-cast sensor through the profile's own
   noise stage and ``PingEncoder`` -- what the node puts on the wire;
3. the processor's OWN code on those samples: ``scale_to_db`` ->
   ``detect_fbr_slant_m`` -> ``FBRTracker`` from the ``BlueBoat-SSS``
   checkout's ``sss_helper.py``, with the constants read from
   ``sss_processor_node.py`` (a missing checkout is an error said plainly);
4. the truth: the world's terrain under each transducer (``depth.npz``),
   as the image shows it (true range x 1500 / c).

Printed and returned: per side the detection rate, the ringing end and the
noise estimate the processor's rule finds, raw first return minus truth
(percentiles, share within 0.30 m, ping-to-ping jumps); the tracker's lock
share and error; the correction's status counts; and the same error for the
altitude the session itself recorded ("before"), when its rows carry one.

A measurement, not a gate: thresholds live with the caller
(``docs/engineering/independent_review_2026-11-11.md`` §10). Nothing is
written into the session or the world; ``--out`` names a JSON elsewhere.

ROS-free, Qt-free. The ray-cast runs on the GPU when Warp imports
(``--backend``); a few thousand pings take minutes.

    python3 -m blueboat_sss_sim.analysis.raycast_bottomline \\
        --session ~/ros2_ws/data/SSS_data/sessions/<name> \\
        --world ~/worlds/<path>/<world> \\
        --profile config/raycast_harbour_fast_sonar.yaml --out /tmp/x.json
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

#: the processor constants this tool needs, by their names in sss_processor_node.py
_CONSTANTS = ("NOISE_FLOOR_WINDOW", "FBR_THRESHOLD_DELTA_DB", "WITHIN_PING_PERSISTENCE",
              "RINGING_SEARCH_MAX", "RINGING_DROP_DB", "RINGING_PERSISTENCE",
              "BOOTSTRAP_PINGS", "ALTITUDE_AGREEMENT_TOL_M", "ALTITUDE_OUTLIER_TOL_M",
              "ALTITUDE_RELOCK_AFTER")


def default_processor_src() -> Path:
    """``BlueBoat-SSS/blueboat_sss/src`` beside this module's submodule."""
    here = Path(__file__).resolve()
    for up in here.parents:
        cand = up / "BlueBoat-SSS" / "blueboat_sss" / "src"
        if cand.is_dir():
            return cand
    return Path("BlueBoat-SSS/blueboat_sss/src")


def processor_rule(src: str | Path):
    """``(sss_helper module, {constant: value})`` of the processor in the
    ``BlueBoat-SSS`` checkout at ``src`` (its ``blueboat_sss/src``)."""
    src = Path(src).expanduser()
    node = src / "sss_processor_node.py"
    lib = src / "_custom_libraries"
    if not node.is_file() or not (lib / "sss_helper.py").is_file():
        raise FileNotFoundError(
            f"no processor at {src} (sss_processor_node.py + _custom_libraries/sss_helper.py); "
            "pass --processor <BlueBoat-SSS/blueboat_sss/src>")
    text = node.read_text()
    consts: dict[str, float] = {}
    for name in _CONSTANTS:
        m = re.search(rf"^{name}\s*:\s*\w+\s*=\s*([-+0-9.eE]+)", text, re.M)
        if m is None:
            raise ValueError(f"{node}: constant {name} not found")
        consts[name] = float(m.group(1))
    if str(lib) not in sys.path:
        sys.path.insert(0, str(lib))
    import sss_helper  # noqa: PLC0415  (the processor's own, read as code)
    return sss_helper, consts


def session_pings(session: str | Path) -> list[dict]:
    """The session's pings, oldest first: one ``rows_data`` entry per device
    ping (the images overlap by half and a row can repeat a ping)."""
    meta = Path(session).expanduser() / "seabed_images" / "metadata"
    seen: dict[tuple, dict] = {}
    for js in sorted(meta.glob("seabed_*.json")):
        for q in json.loads(js.read_text()).get("rows_data", []):
            key = (int(q.get("port_ping_number", 0)), int(q.get("stbd_ping_number", 0)))
            if key == (0, 0):
                continue
            seen.setdefault(key, q)
    rows = sorted(seen.values(), key=lambda q: (float(q.get("t_s", 0.0)),
                                                int(q.get("port_ping_number", 0))))
    if not rows:
        raise FileNotFoundError(f"no rows_data with ping numbers under {meta}")
    return rows


def _pct(a: np.ndarray, qs) -> list:
    return [round(float(v), 3) for v in np.percentile(a, qs)] if a.size else []


def run(session: str | Path, world: str | Path, profile: str | Path,
        processor_src: str | Path | None = None, backend: str = "auto",
        max_pings: int = 0, seed: int = 7, progress: bool = True) -> dict:
    """Replay ``session``'s track in ``world`` with ``profile`` and put the
    processor's rule on the result (module docstring)."""
    from ..core.types import Pose3D, Side
    from ..sonar.config import SPEED_OF_SOUND_MPS
    from ..sonar.encoder import PingEncoder
    from ..sonar.noise import apply_ping_noise
    from ..sonar.raycast import config as rcc
    from ..sonar.raycast.correction import bin_ranges
    from ..sonar.raycast.renderer import RaycastRenderer
    from ..worldgen.scene import SceneModel

    helper, K = processor_rule(processor_src or default_processor_src())
    world = Path(world).expanduser()
    cfg, rc = rcc.load_profile(profile)
    if backend == "auto":
        try:
            import warp  # noqa: F401, PLC0415
            from ..core import gauss_legendre_cache
            gauss_legendre_cache.install()
            backend = "warp"
        except Exception:  # noqa: BLE001
            backend = "cpu"
    rc = dataclasses.replace(rc, backend=backend)
    acq = cfg.acquisition
    scene = SceneModel.load(world)
    rr = RaycastRenderer(scene, acq, rcc.shared_stage(cfg.model, rc), rc, world_dir=world)
    rr.stage_timing = False
    rr.labels_on = False
    noise_m = rcc.noise_stage_model(cfg.model, rc, acq, float(cfg.model.max_ping_rate_hz))
    bins = bin_ranges(acq)
    scale = float(SPEED_OF_SOUND_MPS) / float(rr.c)

    d = np.load(world / "depth.npz") if (world / "depth.npz").is_file() else None
    if d is not None:
        H, ox, oy, res = d["height"], float(d["origin_x"]), float(d["origin_y"]), \
            float(d["resolution"])
    else:
        g = scene.grid
        H, ox, oy, res = scene.height, g.origin_x, g.origin_y, g.resolution

    def terrain(x: float, y: float) -> float:
        i = int(np.clip(round((y - oy) / res), 0, H.shape[0] - 1))
        j = int(np.clip(round((x - ox) / res), 0, H.shape[1] - 1))
        return float(H[i, j])

    rows = session_pings(session)
    if max_pings and len(rows) > max_pings:
        rows = rows[:max_pings]
    rng = np.random.default_rng(seed)
    sides = ((Side.PORT, "port"), (Side.STARBOARD, "starboard"))
    enc = {nm: PingEncoder(sd, acq, noise_m) for sd, nm in sides}
    tracker = helper.FBRTracker(int(K["BOOTSTRAP_PINGS"]), K["ALTITUDE_AGREEMENT_TOL_M"],
                                K["ALTITUDE_OUTLIER_TOL_M"], int(K["ALTITUDE_RELOCK_AFTER"]))
    per = {nm: dict(fbr=[], true=[], ring=[], noise=[]) for _, nm in sides}
    status: dict[str, int] = {}
    trk, trk_true, locked, rec, rec_true = [], [], [], [], []
    t0 = time.perf_counter()
    for n, q in enumerate(rows):
        pose = Pose3D(float(q["x_m"]), float(q["y_m"]), 0.0, float(q.get("roll_rad", 0.0)),
                      float(q.get("pitch_rad", 0.0)), float(q["yaw_rad"]))
        fb, tr = {}, {}
        for sd, nm in sides:
            pg = rr.render(sd, pose, 0.0).ping
            st = getattr(pg, "correction_status", None)
            if st is not None:
                key = str(st).split(":")[0] + (":" + str(st).split(":")[1]
                                               if ":" in str(st) else "")
                status[key] = status.get(key, 0) + 1
            pg.power = apply_ping_noise(
                pg.power, bins < pg.altitude_m, 1.0, noise_m, rng, specular=pg.specular,
                texture=pg.texture, unoccluded=pg.unoccluded, ambient=pg.ambient,
                speckle_fill=pg.speckle_fill, floor_addition=str(rc.floor.addition))
            e = enc[nm].encode(pg)
            db = helper.scale_to_db(e.pwr_results, e.min_pwr_db, e.max_pwr_db)
            ns = helper.find_noise_window_start(
                db, search_max=int(K["RINGING_SEARCH_MAX"]), drop_db=K["RINGING_DROP_DB"],
                persistence=int(K["RINGING_PERSISTENCE"]))
            f = helper.detect_fbr_slant_m(
                db, e.start_mm, e.length_mm, e.num_results,
                noise_floor_window=int(K["NOISE_FLOOR_WINDOW"]),
                threshold_delta_db=K["FBR_THRESHOLD_DELTA_DB"],
                persistence=int(K["WITHIN_PING_PERSISTENCE"]),
                ringing_search_max=int(K["RINGING_SEARCH_MAX"]),
                ringing_drop_db=K["RINGING_DROP_DB"],
                ringing_persistence=int(K["RINGING_PERSISTENCE"]))
            sp = pg.pose
            true = (float(sp.z) - terrain(float(sp.x), float(sp.y))) * scale
            R = per[nm]
            R["fbr"].append(np.nan if f is None else float(f))
            R["true"].append(true)
            R["ring"].append(ns * float(acq.bin_size_m))
            w = int(K["NOISE_FLOOR_WINDOW"])
            R["noise"].append(float(np.mean(db[ns:ns + w])))
            fb[nm], tr[nm] = f, true
        alt = tracker.update(fb["port"], fb["starboard"])
        trk.append(np.nan if alt is None else float(alt))
        locked.append(bool(tracker.locked))
        trk_true.append(max(tr.values()))
        b = q.get("bottom_m")
        if b is not None and float(b) > 0.0:
            rec.append(float(b))
            rec_true.append(max(tr.values()))
        if progress and (n + 1) % 250 == 0:
            el = time.perf_counter() - t0
            print(f"  {n + 1}/{len(rows)} pings, {el:.0f} s "
                  f"({el / (n + 1) * 1e3:.0f} ms per cycle)", flush=True)

    out: dict = {"session": str(Path(session).expanduser()), "world": str(world),
                 "profile": str(profile), "backend": backend, "pings": len(rows),
                 "range_scale": round(scale, 6), "constants": K,
                 "correction_status": status,
                 "wall_s": round(time.perf_counter() - t0, 1), "sides": {}}
    for _, nm in sides:
        R = per[nm]
        f, t = np.asarray(R["fbr"]), np.asarray(R["true"])
        ok = np.isfinite(f)
        err = f[ok] - t[ok]
        out["sides"][nm] = dict(
            detected=round(float(ok.mean()), 4),
            ring_end_m=round(float(np.median(R["ring"])), 3),
            noise_est_db=round(float(np.median(R["noise"])), 2),
            err_p10_p50_p90_m=_pct(err, [10, 50, 90]),
            within_0p3m=round(float(np.mean(np.abs(err) <= 0.30)), 4) if err.size else None,
            jump_p50_p90_m=_pct(np.abs(np.diff(f[ok])), [50, 90]))
    a, tt, lk = np.asarray(trk), np.asarray(trk_true), np.asarray(locked)
    ok = np.isfinite(a)
    e = a[ok] - tt[ok]
    out["tracker"] = dict(
        locked=round(float(lk.mean()), 4),
        err_p10_p50_p90_m=_pct(e, [10, 50, 90]),
        within_0p3m=round(float(np.mean(np.abs(e) <= 0.30)), 4) if e.size else None,
        jump_p50_p90_m=_pct(np.abs(np.diff(a[ok])), [50, 90]))
    if rec:
        e = np.asarray(rec) - np.asarray(rec_true)
        out["recorded_in_session"] = dict(
            note="the altitude the session's own processor tracked, against the same truth",
            err_p10_p50_p90_m=_pct(e, [10, 50, 90]),
            within_0p3m=round(float(np.mean(np.abs(e) <= 0.30)), 4))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="raycast_bottomline",
                                 description=__doc__.split("\n\n")[0])
    ap.add_argument("--session", required=True, help="a GCS session folder")
    ap.add_argument("--world", required=True, help="the world folder the session ran in")
    ap.add_argument("--profile", required=True, help="a ray-cast sonar profile (YAML)")
    ap.add_argument("--processor", default="",
                    help="BlueBoat-SSS/blueboat_sss/src (default: beside this checkout)")
    ap.add_argument("--backend", default="auto", choices=("auto", "warp", "cpu"))
    ap.add_argument("--max-pings", type=int, default=0, help="0 = the whole track")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="", help="write the result as JSON here")
    args = ap.parse_args(argv)
    res = run(args.session, args.world, args.profile, args.processor or None,
              args.backend, args.max_pings, args.seed)
    for nm, s in res["sides"].items():
        print(f"{nm:9s} detected {100 * s['detected']:.1f} %  ringing end {s['ring_end_m']} m  "
              f"noise {s['noise_est_db']} dB  first return - truth p10/50/90 "
              f"{s['err_p10_p50_p90_m']} m  within 0.30 m {100 * (s['within_0p3m'] or 0):.1f} %  "
              f"jumps p50/p90 {s['jump_p50_p90_m']} m")
    t = res["tracker"]
    print(f"tracker   locked {100 * t['locked']:.1f} %  altitude - truth p10/50/90 "
          f"{t['err_p10_p50_p90_m']} m  within 0.30 m {100 * (t['within_0p3m'] or 0):.1f} %  "
          f"jumps p50/p90 {t['jump_p50_p90_m']} m")
    if "recorded_in_session" in res:
        r = res["recorded_in_session"]
        print(f"recorded  (the session's own run) altitude - truth p10/50/90 "
              f"{r['err_p10_p50_p90_m']} m  within 0.30 m {100 * r['within_0p3m']:.1f} %")
    print(f"correction status counts: {res['correction_status']}; {res['pings']} pings in "
          f"{res['wall_s']} s ({res['backend']})")
    if args.out:
        p = Path(args.out).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(res, indent=1))
        print("wrote", p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
