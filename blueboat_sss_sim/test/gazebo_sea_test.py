#!/usr/bin/env python3
"""Waves and current in Gazebo: the same survey under four sea states.

The end-to-end gate of the sea-state model (docs/sea_state_validation.md).
It is **not** part of the offline smoke test or its Stop hook: it needs a
sourced workspace (``source ~/ros2_ws/env.sh``), a built
``blueboat_sss_sim`` and real time (about 5-6 min per scenario, launch
included, as measured on 2026-10-09; 1.5 with ``--quick``). For every scenario it

1. writes a one-keyframe sea scenario (``blueboat_sea_schedule/1``),
2. launches ``full_mission_launch.py`` headless on the validation world
   with the unmodified control stack following the validation trajectory,
3. records a ``ros2 bag`` of the run (sim-time stamps),
4. stops when the boat has sailed the whole path (or line 1 with
   ``--quick``), tearing the launch down SIGINT -> SIGTERM -> SIGKILL,

then analyses every bag (``analysis/sea_validation.py``), writes the
comparison figures, ``report.md`` and ``results.json`` into ``--out``, and
checks each scenario against the expectations below. Exit code 0 = every
check passed.

Scenarios (the run's seed fixes the sea; the current comes from the north,
across the east-west survey lines; waves from the north-east, oblique):

* ``calm``    -- null test: no wave forcing (the catamaran hydrostatics
  active, as in every scenario since 2026-10-09), the boat level, every
  ping lands one footprint-step ahead of the last;
* ``current`` -- the realised current matches its configured mean within
  the statistics of a run this short and never leaves its bound; the boat
  crabs and its track deviates more than in calm water;
* ``waves``   -- the motion test (TODO, done 2026-10-08): roll/pitch std in the
  corpus band, heave bounded, the hull's mean height unchanged (impulse
  conservation), no 1.6 Hz ringing, a fresh pose under every ping -- and
  the SSS consequence: footprints that step back over seabed already
  imaged or skip ahead, uneven pings per target;
* ``both``    -- both sets.

Usage (from the package source root)::

    source ~/ros2_ws/env.sh
    python3 -m test.gazebo_sea_test --world-dir ~/worlds/sea_validation/flat_5p0m \\
        --trajectory ~/.config/blueboat_mcs/trajectories/sea_validation.yaml \\
        --out ~/ros2_ws/sim_runs/sea_validation/$(date +%F)_5m
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from blueboat_sss_sim.analysis import sea_validation as sv  # noqa: E402
from blueboat_sss_sim.sea.presets import SCHEDULE_FORMAT  # noqa: E402

SCENARIOS = ("calm", "current", "waves", "both")


# ------------------------------------------------------------- scenario
def scenario_doc(name: str, a: argparse.Namespace) -> dict:
    cur = ({"speed_mps": a.current_speed, "from_deg": a.current_from,
            "variation": a.variation} if name in ("current", "both")
           else {"preset": "none"})
    wav = ({"preset": a.waves, "from_deg": a.waves_from, "crest": a.crest,
            "wakes": a.wakes} if name in ("waves", "both") else {"preset": "calm"})
    return {"format": SCHEDULE_FORMAT, "name": f"seaval_{name}", "seed": a.seed,
            "interpolation": "hold",
            "keyframes": [{"t_s": 0.0, "current": cur, "waves": wav}]}


# ------------------------------------------------------------ processes
def _killpg(proc: subprocess.Popen, sig: int) -> None:
    try:
        os.killpg(proc.pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def stop(proc: subprocess.Popen, name: str, grace=(20.0, 10.0)) -> str:
    """SIGINT the process group, then SIGTERM, then SIGKILL."""
    for sig, wait in ((signal.SIGINT, grace[0]), (signal.SIGTERM, grace[1]),
                      (signal.SIGKILL, 5.0)):
        if proc.poll() is not None:
            break
        _killpg(proc, sig)
        try:
            proc.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue
    # A child that outlived its parent still holds the group id.
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid, 0)
        except (ProcessLookupError, PermissionError):
            break
        _killpg(proc, sig)
        time.sleep(2.0)
    return f"{name} exit {proc.poll()}"


class Watcher:
    """Minimal rclpy node: the boat's odometry and sim time."""

    def __init__(self) -> None:
        import rclpy  # noqa: PLC0415
        from nav_msgs.msg import Odometry  # noqa: PLC0415
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("gazebo_sea_test_watch")
        self.reset()
        self.node.create_subscription(Odometry, "/blueboat/odom", self._odom, 10)

    def reset(self) -> None:
        self.n, self.t, self.xy, self.dist = 0, 0.0, None, 0.0
        self.start, self.max_disp = None, 0.0

    def _odom(self, msg) -> None:
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.n == 0 and t > 60.0:
            return            # a late sample of the previous run's Gazebo
        self.t = t
        if self.xy is not None:
            step = float(np.hypot(p.x - self.xy[0], p.y - self.xy[1]))
            if step < 1.0:                    # ignore a respawn jump
                self.dist += step
        self.xy = (p.x, p.y)
        if self.start is None:
            self.start = (p.x, p.y)
        self.max_disp = max(self.max_disp, float(np.hypot(p.x - self.start[0],
                                                          p.y - self.start[1])))
        self.n += 1

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)


def first_line_progress(traj_points: np.ndarray, xy) -> float:
    """How far along survey line 1 the boat has come [m]: drift across the
    line (an idle boat carried by a current) does not count."""
    lines = sv.survey_lines(traj_points)
    if not lines or xy is None:
        return 0.0
    a, _ = lines[0].project(np.array([xy[0]]), np.array([xy[1]]))
    return float(a[0])


def mission_goal(traj_points: np.ndarray, quick: bool) -> tuple[np.ndarray, float]:
    """(end point to reach, displacement the boat must have made first):
    the end of survey line 1 with ``--quick``, else the path's last point.
    Progress is judged by WHERE the boat is, never by summed path length,
    which wave jitter inflates (a pilot "sailed" 72 m without moving)."""
    lines = sv.survey_lines(traj_points)
    end = lines[0].p1 if (quick and lines) else traj_points[-1, 1:3]
    span = float(np.max(np.hypot(traj_points[:, 1] - traj_points[0, 1],
                                 traj_points[:, 2] - traj_points[0, 2])))
    need = 0.8 * (float(np.hypot(*(end - traj_points[0, 1:3]))) if quick else span)
    return np.asarray(end, dtype=float), need


def _attempt(name: str, a, d: Path, cmd: list[str], watch: "Watcher",
             goal: np.ndarray, need: float, duration: float,
             traj_points: np.ndarray) -> tuple[str, list]:
    log = open(d / "launch.log", "w")
    launch = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    bag = subprocess.Popen(["ros2", "bag", "record", "--use-sim-time",
                            "-o", str(d / "bag"), *sv.BAG_TOPICS],
                           stdout=open(d / "bag.log", "w"), stderr=subprocess.STDOUT,
                           start_new_session=True)
    watch.reset()
    t_wall0 = time.monotonic()
    status = "ok"
    try:
        while watch.n == 0:
            watch.spin(1.0)
            if time.monotonic() - t_wall0 > a.startup_timeout:
                status = "no odometry (Gazebo did not start?)"
                break
            if launch.poll() is not None:
                status = f"launch exited early ({launch.returncode})"
                break
        t_sim0 = watch.t
        progress = 0.0
        while status == "ok":
            watch.spin(1.0)
            el = watch.t - t_sim0
            to_go = float(np.hypot(watch.xy[0] - goal[0], watch.xy[1] - goal[1]))
            if watch.max_disp >= need and to_go < 2.5:
                watch.spin(3.0)               # a few pings past the end
                break
            progress = max(progress, first_line_progress(traj_points, watch.xy))
            if el > a.stall_s and progress < 3.0:
                # The controller never started (the boat may still drift
                # with a current -- hence progress ALONG line 1, not
                # displacement). Until 2026-10-09 BlueBoat-Control's sim
                # path announced readiness only once (latched) and
                # master_control, subscribing volatile, could miss it
                # (CM-4); simulation_interface now re-publishes it every
                # second, so this is a guard. Relaunch rather than inject
                # the signal.
                status = "stalled"
                break
            if el > 2.0 * duration + 60.0:
                status = f"timeout: {to_go:.0f} m from the end"
                break
            if launch.poll() is not None:
                status = f"launch exited early ({launch.returncode})"
                break
            print(f"\r  {name}: sim {el:6.1f} s, {to_go:5.1f} m to the end, "
                  f"moved {watch.max_disp:5.1f} m", end="", flush=True)
    finally:
        print()
        r_bag = stop(bag, "bag", grace=(10.0, 5.0))
        r_launch = stop(launch, "launch")
        log.close()
    time.sleep(3.0)                            # DDS settles between runs
    return status, [r_bag, r_launch]


def run_scenario(name: str, a: argparse.Namespace, out: Path,
                 watch: Watcher, traj_points: np.ndarray, duration: float) -> dict:
    d = out / name
    d.mkdir(parents=True, exist_ok=True)
    doc = scenario_doc(name, a)
    (d / "scenario.yaml").write_text(yaml.safe_dump(doc, sort_keys=False))
    cmd = ["ros2", "launch", "blueboat_sss_sim", "full_mission_launch.py",
           f"world_dir:={a.world_dir}", "with_control:=true",
           f"trajectory_file:={a.trajectory}", "headless:=true",
           f"sea_schedule:={d / 'scenario.yaml'}", f"sea_seed:={a.seed}",
           f"note:=seaval_{name}", f"data_dir:={d / 'data'}"]
    goal, need = mission_goal(traj_points, a.quick)
    attempts = []
    for attempt in range(1 + max(int(a.retries), 0)):
        if (d / "bag").exists():
            import shutil  # noqa: PLC0415
            shutil.rmtree(d / "bag")           # a stalled attempt's recording
        status, teardown = _attempt(name, a, d, cmd, watch, goal, need, duration,
                                    traj_points)
        attempts.append(status)
        if status != "stalled":
            break
        print(f"  {name}: controller never started (attempt {attempt + 1}); relaunching")
    (d / "run.yaml").write_text(yaml.safe_dump({
        "name": name, "world_dir": str(a.world_dir), "trajectory": str(a.trajectory),
        "scenario": doc, "seed": a.seed, "command": " ".join(cmd),
        "attempts": attempts,
        "started": datetime.datetime.now().isoformat(timespec="seconds")},
        sort_keys=False))
    return {"name": name, "status": status, "attempts": attempts,
            "moved_m": round(watch.max_disp, 1), "teardown": teardown}


# -------------------------------------------------------------- checks
class Checks:
    def __init__(self) -> None:
        self.rows: list[tuple[str, str, bool, str]] = []

    def __call__(self, scen: str, what: str, ok: bool | None, detail: str = "") -> None:
        """``ok=None`` records an INFO row: an effect not expected under
        these conditions, with the reason -- never a pass, never a fail."""
        self.rows.append((scen, what, ok if ok is None else bool(ok), detail))
        tag = "INFO" if ok is None else ("PASS" if ok else "FAIL")
        print(f"  [{tag}] {scen}: {what}" + (f" -- {detail}" if detail else ""))

    @property
    def failed(self) -> int:
        return sum(1 for r in self.rows if r[2] is False)

    def markdown(self) -> str:
        out = ["| Scenario | Check | Result | Detail |", "|---|---|---|---|"]
        out += [f"| {s} | {w} | {'INFO' if ok is None else ('PASS' if ok else '**FAIL**')} "
                f"| {d} |" for s, w, ok, d in self.rows]
        return "\n".join(out)


def _g(r: dict, *path, default=float("nan")):
    v = sv._get(r, path)
    return default if v is None else v


def evaluate(results: dict[str, dict], a: argparse.Namespace) -> Checks:
    ck = Checks()
    calm = results.get("calm")
    for name, r in results.items():
        waves = name in ("waves", "both")
        current = name in ("current", "both")
        att, fp = r.get("attitude", {}), r.get("footprint", {})
        if name == "calm":
            ck(name, "calm water: no wave forcing, and the catamaran "
               "hydrostatics carry the hull as in every scenario",
               _g(r, "sea", "waves_status", default="") == "calm"
               and _g(r, "sea", "hull", "model", default="") == "catamaran",
               f"waves {_g(r, 'sea', 'waves_status', default='?')}, hull "
               f"{_g(r, 'sea', 'hull', 'model', default='?')}, "
               f"{_g(r, 'sea', 'wrench_messages', default=-1)} hull-wrench messages")
            ck(name, "the boat stays level (roll/pitch std < 0.1 deg, heave std < 5 mm)",
               att.get("roll_std_deg", 9) < 0.1 and att.get("pitch_std_deg", 9) < 0.1
               and att.get("heave_std_m", 9) < 0.005,
               f"roll {att.get('roll_std_deg')}, pitch {att.get('pitch_std_deg')} deg, "
               f"heave {att.get('heave_std_m')} m")
            bs = [_g(r, "footprint", f"r{int(x)}", s, "backscan_fraction", default=1.0)
                  for x in sv.FOOTPRINT_RANGES_M for s in sv.SIDES]
            ck(name, "every ping lands ahead of the last (no back-scan)",
               max(bs) == 0.0, f"max back-scan fraction {max(bs)}")
        if current:
            c = r.get("current", {})
            n_eff = max(r.get("duration_s", 1.0) / (2.0 * 60.0), 1.0)
            tol_v = 3.0 * c.get("speed_std_mps", 0.0) / np.sqrt(n_eff) + 0.01
            tol_d = 3.0 * c.get("from_std_deg", 0.0) / np.sqrt(n_eff / 2.0) + 1.0
            ck(name, "realised current: mean speed and direction match the setting "
               "(within the statistics of a run this short)",
               abs(c.get("speed_mean_mps", 0) - c.get("configured_mps", 1)) <= tol_v
               and abs(c.get("from_mean_dev_deg", 99)) <= tol_d,
               f"speed {c.get('speed_mean_mps')} vs {c.get('configured_mps')} "
               f"(+-{tol_v:.3f}), direction off by {c.get('from_mean_dev_deg')} deg "
               f"(+-{tol_d:.1f})")
            ck(name, "realised current never leaves its bound",
               c.get("speed_dev_max_mps", 9) <= c.get("speed_bound_mps", 0) + 1e-3,
               f"max speed deviation {c.get('speed_dev_max_mps')} <= "
               f"{c.get('speed_bound_mps')} m/s; direction within "
               f"{c.get('from_dev_max_deg')} deg of the mean")
            ck(name, "the boat crabs into the cross-current (|crab| > 3 deg) and its "
               "track deviates more than in calm water",
               _g(r, "track", "crab_abs_mean_deg", default=0) > 3.0
               and (calm is None or _g(r, "track", "xte_rms_m", default=0)
                    > _g(calm, "track", "xte_rms_m", default=0)),
               f"crab {_g(r, 'track', 'crab_abs_mean_deg')} deg, xte RMS "
               f"{_g(r, 'track', 'xte_rms_m')} m (calm "
               f"{_g(calm, 'track', 'xte_rms_m') if calm else '-'})")
        if waves:
            ck(name, f"roll and pitch std in the corpus band (0.5-2.5 deg) at "
               f"'{a.waves}'",
               0.5 <= att.get("roll_std_deg", 0) <= 2.5
               and 0.5 <= att.get("pitch_std_deg", 0) <= 2.5,
               f"roll {att.get('roll_std_deg')}, pitch {att.get('pitch_std_deg')} deg")
            # A contouring hull heaves with the surface (docs/sea_state_model.md
            # §5): heave std = sigma_eta = Hs/4 and its spectrum peaks at Tp.
            # (TODO [P1]'s "p2p < 0.15 m at 'smooth'" contradicted its own Hs
            # of 0.15 m: a Gaussian sea's largest crest-to-trough over ~50
            # waves is ~2 sigma sqrt(2 ln 50) = 5.6 sigma = 1.4 Hs.)
            hs = float(_g(r, "sea", "hs_m", default=0.0))
            tp = float(_g(r, "sea", "tp_s", default=0.0))
            hstd = att.get("heave_std_m", 0.0)
            hper = _g(r, "attitude", "heave_spectrum", "peak_period_s", default=0.0)
            ck(name, "the hull rides the waves: heave std = Hs/4 (0.7-1.3x), heave "
               "period near Tp (0.6-1.6x), no 1.6 Hz ringing",
               hs > 0 and 0.7 <= hstd / (hs / 4.0) <= 1.3
               and (tp <= 0 or 0.6 <= hper / tp <= 1.6)
               and _g(r, "attitude", "heave_spectrum", "ring_fraction", default=1) < 0.05,
               f"heave std {hstd} m vs Hs/4 {hs / 4:.4f} m; period {hper} s vs Tp "
               f"{tp} s; 1.4-1.8 Hz power "
               f"{_g(r, 'attitude', 'heave_spectrum', 'ring_fraction')}; p2p "
               f"{att.get('heave_p2p_m')} m")
            if calm is not None:
                dz = abs(att.get("z_mean_m", 0) - _g(calm, "attitude", "z_mean_m", default=0))
                ck(name, "the wave impulse is conserved: mean hull height = calm "
                   "water's within 3 mm", dz < 0.003, f"{dz * 1000:.1f} mm")
            if calm is not None:
                dp = abs(att.get("pitch_mean_deg", 0.0)
                         - _g(calm, "attitude", "pitch_mean_deg", default=0.0))
                ck(name, "the same hull in every scenario: mean pitch trim "
                   "within 0.5 deg of calm water's (no hydrostatics switch)",
                   dp < 0.5, f"{att.get('pitch_mean_deg')} vs calm "
                   f"{_g(calm, 'attitude', 'pitch_mean_deg')} deg")
            sea_age = _g(r, "sea", "pose_age_ms_max", default=999)
            son_frac = _g(r, "footprint", "pose_age_frac_over_60ms", default=1.0)
            ck(name, "a fresh pose under every wrench and every ping while "
               "surveying (wrench <= 60 ms; >= 99.9 % of pings <= 60 ms)",
               sea_age <= 60.0 and son_frac <= 0.001,
               f"sea node {sea_age} ms; sonar: {100 * son_frac:.2f} % of pings over "
               f"60 ms, worst {_g(r, 'footprint', 'pose_age_ms_max')} ms (start-up, "
               f"first {sv.STARTUP_S:.0f} s: "
               f"{_g(r, 'sea', 'pose_age_ms_startup_max', default='-')} ms)")
            bs15 = max(_g(r, "footprint", "r15", s, "backscan_fraction", default=0)
                       for s in sv.SIDES)
            gp15 = max(_g(r, "footprint", "r15", s, "gap_fraction", default=0)
                       for s in sv.SIDES)
            cm5 = _g(r, "footprint", "r5", "common_mode_std_m", default=0)
            cm5_calm = _g(calm, "footprint", "r5", "common_mode_std_m", default=0) if calm else 0
            ck(name, "the SSS sees it: footprints step back over imaged seabed or "
               "skip ahead at 15 m, and the pitch-like footprint wobble at 5 m "
               "is several times calm water's",
               (bs15 > 0 or gp15 > 0) and cm5 > 3.0 * max(cm5_calm, 1e-4),
               f"back-scan {bs15}, gaps {gp15}; common-mode {cm5} m "
               f"(calm {cm5_calm} m)")
            mv = r.get("mosaic_vs_calm") or {}
            if mv:
                ck(name, "targets appear displaced or distorted against the same "
                   "survey in calm water (median shift > 0.1 m or image "
                   "correlation < 0.9)",
                   mv.get("shift_median_m", 0) > 0.1 or mv.get("corr_median", 1) < 0.9,
                   f"median shift {mv.get('shift_median_m')} m (max "
                   f"{mv.get('shift_max_m')}), correlation {mv.get('corr_median')} "
                   f"(min {mv.get('corr_min')})")
            if calm is not None and r.get("targets") and calm.get("targets"):
                # A target collects many pings or none only when the beam
                # sweeps back over it (back-scan) at its range -- 5-10 m here.
                # With ~3 pings per crossing a back-scan fraction b re-sweeps
                # a crossing with probability ~1-(1-b)^3: at 2 % that is one
                # crossing in ~16, invisible in 20 passes next to the counts'
                # own discreteness; at 15 % it is four in ten. Pitch moves the
                # footprint by altitude * tan(pitch), so in very shallow water
                # the sweep stays monotonic at near range -- the current's
                # crab still makes coverage uneven in 'both'.
                bs_t = max(_g(r, "footprint", k, sd, "backscan_fraction", default=0)
                           for k in ("r5", "r10") for sd in sv.SIDES)
                detail = (f"CV {_g(r, 'targets', 'hits_cv')} vs calm "
                          f"{_g(calm, 'targets', 'hits_cv')}; missed "
                          f"{_g(r, 'targets', 'missed_fraction')}; back-scan at "
                          f"5-10 m {bs_t:.3f}")
                if current or bs_t >= 0.05:
                    ck(name, "uneven coverage: pings per target pass vary more "
                       "than in calm water",
                       _g(r, "targets", "hits_cv", default=0)
                       > _g(calm, "targets", "hits_cv", default=0), detail)
                else:
                    ck(name, "uneven coverage not expected here: the beam "
                       "hardly ever sweeps back at the targets' range (< 5 %) "
                       "-- shallow water, pitch barely moves the footprint",
                       None, detail)
    return ck


# ------------------------------------------------------------------ main
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    home = Path.home()
    ap.add_argument("--world-dir", type=Path,
                    default=home / "worlds/sea_validation/flat_5p0m")
    ap.add_argument("--trajectory", type=Path,
                    default=home / ".config/blueboat_mcs/trajectories/sea_validation.yaml")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--quick", action="store_true", help="first line only")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--current-speed", type=float, default=0.3)
    ap.add_argument("--current-from", type=float, default=0.0)
    ap.add_argument("--variation", default="natural")
    ap.add_argument("--waves", default="smooth")
    ap.add_argument("--waves-from", type=float, default=45.0)
    ap.add_argument("--crest", default="short")
    ap.add_argument("--wakes", default="none")
    ap.add_argument("--startup-timeout", type=float, default=180.0)
    ap.add_argument("--stall-s", type=float, default=45.0,
                    help="sim seconds without 3 m of progress along survey line 1 "
                         "= stalled start (a current's drift does not count)")
    ap.add_argument("--retries", type=int, default=2,
                    help="relaunches of a stalled start")
    ap.add_argument("--analyse-only", action="store_true",
                    help="re-analyse the runs already in --out")
    a = ap.parse_args(argv)
    a.world_dir = a.world_dir.expanduser().resolve()
    a.trajectory = a.trajectory.expanduser().resolve()
    out = (a.out or home / "ros2_ws/sim_runs/sea_validation"
           / datetime.datetime.now().strftime("%Y-%m-%d_%H%M")).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    names = [s.strip() for s in a.scenarios.split(",") if s.strip()]
    traj = yaml.safe_load(a.trajectory.read_text())
    duration = float(traj.get("duration_s", 200.0))
    traj_points = np.asarray(traj.get("points") or [], dtype=float).reshape(-1, 4)

    runs_meta = []
    if not a.analyse_only:
        watch = Watcher()
        for name in names:
            print(f"[run] {name}")
            runs_meta.append(run_scenario(name, a, out, watch, traj_points, duration))
            print(f"  -> {runs_meta[-1]}")
    print("[analyse]")
    runs, results = [], {}
    for name in names:
        try:
            run = sv.load_run_dir(out / name)
        except Exception as exc:  # noqa: BLE001 -- report and go on
            print(f"  {name}: cannot read the run ({exc})")
            continue
        res = sv.analyse(run)
        sv.write_metrics(res, out / name)
        sv.plot_run(run, out / name)
        runs.append(run)
        results[name] = res
    sv.compare_to_reference(runs, [results[r.name] for r in runs])
    for r in runs:
        sv.write_metrics(results[r.name], out / r.name)
    figs = sv.plot_compare(runs, list(results.values()), out) if runs else []
    print("[check]")
    ck = evaluate(results, a)
    (out / "results.json").write_text(json.dumps(
        {"runs": runs_meta, "results": results, "checks": ck.rows}, indent=2,
        default=sv._json_default))
    report = [f"# Sea-state validation in Gazebo — {out.name}", "",
              f"World `{a.world_dir}`, trajectory `{a.trajectory}`, seed {a.seed}; "
              f"current {a.current_speed} m/s from {a.current_from:.0f}° "
              f"({a.variation}); waves '{a.waves}' from {a.waves_from:.0f}° "
              f"({a.crest} crests, wakes {a.wakes}).", "",
              "## Measurements", "", sv.summary_table(list(results.values())), "",
              "## Checks", "", ck.markdown(), "",
              "## Figures", ""] + [f"![{f}]({f})" for f in figs]
    (out / "report.md").write_text("\n".join(report) + "\n")
    print(f"report: {out / 'report.md'}  ({ck.failed} failed check(s))")
    return 1 if ck.failed or not results else 0


if __name__ == "__main__":
    raise SystemExit(main())
