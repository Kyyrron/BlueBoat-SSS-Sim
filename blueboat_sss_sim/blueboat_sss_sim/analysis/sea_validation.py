"""What waves and current do to a side-scan survey -- measured on a run.

Reads one recorded Gazebo run (a ``ros2 bag`` of the topics listed in
:data:`BAG_TOPICS`, recorded by ``test/gazebo_sea_test.py``) or a field
``.svlog`` (attitude + nav only), and measures the effects the literature
describes for a sonar on a moving platform (``docs/sea_state_validation.md``):

* **track** -- cross-track error and crab angle on the survey lines
  (current-induced track deviation; Paravisi et al. 2019);
* **attitude** -- roll, pitch, heave and yaw about the course, their
  dominant period and the heave power above 1 Hz (wave-induced motion);
* **footprint** -- where each ping's beam actually lands on the seabed,
  from the exact transducer pose the simulator rendered it at
  (``/side_scan_sonar/ground_truth/sensor``): the along-track spacing of
  successive footprints, the fraction that step *backwards* (the same
  seabed imaged again -- Anstee's back-scanning), the fraction that skip
  more than a footprint (coverage gaps), and the port/starboard
  common-mode (pitch) and differential-mode (yaw) displacement (Cobra et
  al. 1992; Lei et al. 2026);
* **targets** -- how many pings insonify each known object on each pass:
  even in calm water, uneven or zero under waves (repeated or missed
  targets);
* **imagery** -- dB waterfalls per line and a "naive" mosaic placed with
  position and heading only (what a display without attitude correction
  shows), with each target's apparent displacement from the truth;
* **current** -- the realised current at the boat against the configured
  mean, sigma and bound.

The core is ROS-free (numpy + yaml; matplotlib for figures, imported in
the plotting functions); only :func:`read_bag` imports ``rosbag2_py`` and
the message types, inside the function.

CLI::

    python3 -m blueboat_sss_sim.analysis.sea_validation run --bag B \\
        --world-dir W --trajectory T --name waves --out OUT
    python3 -m blueboat_sss_sim.analysis.sea_validation compare OUT/calm \\
        OUT/current OUT/waves OUT/both --out OUT
    python3 -m blueboat_sss_sim.analysis.sea_validation svlog FILE \\
        --altitude 1.4 --out OUT
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ..core.geometry import compass_deg_to_enu_yaw, wrap_pi
from .calibration import scale_to_db

#: Topics a validation run records (test/gazebo_sea_test.py).
BAG_TOPICS = (
    "/blueboat/odom", "/current", "/sim/sea_state",
    "/side_scan_sonar/ground_truth/sensor",
    "/side_scan_sonar/ground_truth/contacts",
    "/side_scan_sonar/port/profile", "/side_scan_sonar/starboard/profile",
    "/monitoring_data", "/thruster_input",
)
SIDES = ("port", "starboard")
SIDE_SIGN = {"port": +1.0, "starboard": -1.0}
#: Omniscan 450 SS azimuth beamwidth (sonar/config.py horizontal_aperture_deg).
BEAMWIDTH_DEG = 0.5
#: Seconds after the first readback that count as start-up (spawn, first
#: odometry) for the pose-age statistic.
STARTUP_S = 10.0
#: Ground ranges at which the footprint is followed [m].
FOOTPRINT_RANGES_M = (5.0, 10.0, 15.0)


# ===================================================================== data
@dataclass
class Track:
    """Vehicle pose over time (odometry or field nav), world ENU."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    roll: np.ndarray
    pitch: np.ndarray
    yaw: np.ndarray

    def velocity(self, smooth_s: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
        """World-frame (vx, vy) by central differences over ``smooth_s``."""
        if self.t.size < 3:
            return np.zeros_like(self.t), np.zeros_like(self.t)
        dt = float(np.median(np.diff(self.t)))
        k = max(int(round(smooth_s / max(dt, 1e-3) / 2.0)), 1)
        vx, vy = np.zeros_like(self.x), np.zeros_like(self.y)
        i0, i1 = np.arange(self.t.size) - k, np.arange(self.t.size) + k
        i0, i1 = np.clip(i0, 0, self.t.size - 1), np.clip(i1, 0, self.t.size - 1)
        span = np.maximum(self.t[i1] - self.t[i0], 1e-6)
        vx = (self.x[i1] - self.x[i0]) / span
        vy = (self.y[i1] - self.y[i0]) / span
        return vx, vy


@dataclass
class PingPoses:
    """The transducer pose every ping was rendered at (sim ground truth)."""

    t: np.ndarray                                  # sim time [s]
    boat: np.ndarray                               # (N, 6)
    sensor: dict[str, np.ndarray]                  # side -> (N, 6)
    altitude: dict[str, np.ndarray]                # side -> (N,)
    ping_number: dict[str, np.ndarray]             # side -> (N,), -1 dropped
    pose_age_ms: np.ndarray


@dataclass
class SideProfiles:
    t: np.ndarray
    ping_number: np.ndarray
    db: np.ndarray                                 # (M, bins)
    start_m: float
    length_m: float

    @property
    def slant_m(self) -> np.ndarray:
        n = self.db.shape[1]
        return self.start_m + (np.arange(n) + 0.5) * self.length_m / n


@dataclass
class RunData:
    name: str
    track: Track
    pings: PingPoses | None = None
    profiles: dict[str, SideProfiles] = field(default_factory=dict)
    current_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    current_v: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    sea_status: list[dict] = field(default_factory=list)
    trajectory: np.ndarray | None = None           # (P, 4) t, x, y, yaw
    world_dir: str = ""
    targets: list[dict] = field(default_factory=list)
    scenario: dict = field(default_factory=dict)


# ================================================================== readers
def read_bag(bag_dir: str | Path, name: str = "") -> RunData:
    """A ``ros2 bag`` of :data:`BAG_TOPICS` -> :class:`RunData` (needs a
    sourced ROS 2 workspace: rosbag2_py and the message packages)."""
    import rosbag2_py  # noqa: PLC0415 -- ROS only here
    from rclpy.serialization import deserialize_message  # noqa: PLC0415
    from rosidl_runtime_py.utilities import get_message  # noqa: PLC0415

    from ..core.geometry import quat_to_rpy  # noqa: PLC0415

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = {n: get_message(t) for n, t in types.items()}
    odom: list = []
    cur_t, cur_v = [], []
    status, sensor = [], []
    prof: dict[str, list] = {s: [] for s in SIDES}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_cls:
            continue
        msg = deserialize_message(data, msg_cls[topic])
        if topic == "/blueboat/odom":
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            r, pi, ya = quat_to_rpy(q.x, q.y, q.z, q.w)
            odom.append((st if st > 0 else t_ns * 1e-9, p.x, p.y, p.z, r, pi, ya))
        elif topic == "/current":
            cur_t.append(t_ns * 1e-9)
            cur_v.append((msg.x, msg.y, msg.z))
        elif topic == "/sim/sea_state":
            try:
                status.append(json.loads(msg.data))
            except ValueError:
                pass
        elif topic == "/side_scan_sonar/ground_truth/sensor":
            try:
                sensor.append(json.loads(msg.data))
            except ValueError:
                pass
        elif topic.endswith("/profile"):
            side = "port" if "/port/" in topic else "starboard"
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            db = scale_to_db(np.asarray(msg.pwr_results, dtype=np.float64),
                             msg.min_pwr_db, msg.max_pwr_db)
            prof[side].append((st, int(msg.ping_number), db,
                               msg.start_mm / 1000.0, msg.length_mm / 1000.0))
    run = RunData(name=name or Path(bag_dir).name, track=_track_from_rows(odom))
    if cur_t:
        run.current_t = np.asarray(cur_t)
        run.current_v = np.asarray(cur_v)
    run.sea_status = status
    run.pings = _pings_from_json(sensor) if sensor else None
    for side, rows in prof.items():
        if not rows:
            continue
        n = min(len(r[2]) for r in rows)
        rows.sort(key=lambda r: r[0])
        run.profiles[side] = SideProfiles(
            t=np.array([r[0] for r in rows]),
            ping_number=np.array([r[1] for r in rows]),
            db=np.stack([r[2][:n] for r in rows]),
            start_m=float(rows[0][3]), length_m=float(rows[0][4]))
    return run


def _track_from_rows(rows: list) -> Track:
    a = np.array(sorted(rows), dtype=np.float64).reshape(-1, 7)
    return Track(*(a[:, i] for i in range(7)))


def _pings_from_json(rows: list[dict]) -> PingPoses:
    rows = sorted(rows, key=lambda d: d.get("stamp", d.get("t_sim", 0.0)))
    t = np.array([d.get("stamp", d.get("t_sim", 0.0)) for d in rows])
    boat = np.array([d["boat"] for d in rows], dtype=np.float64)
    sensor, alt, num = {}, {}, {}
    for s in SIDES:
        sensor[s] = np.array([d["sensors"][s]["pose"] for d in rows], dtype=np.float64)
        alt[s] = np.array([d["sensors"][s]["altitude_m"] for d in rows])
        num[s] = np.array([-1 if d["sensors"][s].get("ping_number") is None
                           else d["sensors"][s]["ping_number"] for d in rows])
    return PingPoses(t, boat, sensor, alt, num,
                     np.array([d.get("pose_age_ms", 0.0) for d in rows]))


def read_trajectory(path: str | Path) -> np.ndarray:
    """``blueboat_trajectory/1`` runtime YAML -> (P, 4) [t, x, y, yaw]."""
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    return np.asarray(doc.get("points") or [], dtype=np.float64).reshape(-1, 4)


def read_targets(world_dir: str | Path) -> list[dict]:
    """The world's objects from ``metadata.yaml`` (the cross-module
    contract, never ``builder_state.yaml``)."""
    p = Path(world_dir) / "metadata.yaml"
    if not p.exists():
        return []
    with open(p, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    return [dict(o) for o in (doc.get("objects") or [])]


# ============================================================ survey lines
@dataclass
class Line:
    p0: np.ndarray
    p1: np.ndarray

    @property
    def length(self) -> float:
        return float(np.hypot(*(self.p1 - self.p0)))

    @property
    def u(self) -> np.ndarray:
        d = self.p1 - self.p0
        return d / max(np.hypot(*d), 1e-9)

    @property
    def n(self) -> np.ndarray:              # left normal
        return np.array([-self.u[1], self.u[0]])

    @property
    def heading(self) -> float:
        return float(np.arctan2(self.u[1], self.u[0]))

    def project(self, x, y) -> tuple[np.ndarray, np.ndarray]:
        """(along, cross) of points; cross > 0 left of the direction."""
        d = np.stack([np.asarray(x) - self.p0[0], np.asarray(y) - self.p0[1]], -1)
        return d @ self.u, d @ self.n


def survey_lines(traj: np.ndarray, min_len_m: float = 20.0,
                 tol_deg: float = 2.0) -> list[Line]:
    """Straight runs of a dense trajectory: consecutive samples whose
    heading stays within ``tol_deg`` of the run's, at least ``min_len_m``."""
    if traj is None or traj.shape[0] < 3:
        return []
    xy = traj[:, 1:3]
    seg = np.diff(xy, axis=0)
    keep = np.hypot(seg[:, 0], seg[:, 1]) > 1e-6
    xy_k = np.vstack([xy[:1], xy[1:][keep]])
    seg = np.diff(xy_k, axis=0)
    hd = np.arctan2(seg[:, 1], seg[:, 0])
    lines, i = [], 0
    while i < hd.size:
        j = i
        while j + 1 < hd.size and abs(wrap_pi(hd[j + 1] - hd[i])) < np.radians(tol_deg):
            j += 1
        p0, p1 = xy_k[i], xy_k[j + 1]
        if np.hypot(*(p1 - p0)) >= min_len_m:
            lines.append(Line(p0.copy(), p1.copy()))
        i = j + 1
    return lines


def _on_line(line: Line, x, y, margin_m: float = 5.0,
             max_cross_m: float = 8.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    a, c = line.project(x, y)
    m = (a > margin_m) & (a < line.length - margin_m) & (np.abs(c) < max_cross_m)
    return m, a, c


# ================================================================== metrics
def _std_deg(a) -> float:
    return float(np.degrees(np.std(a))) if np.size(a) else float("nan")


def spectrum_summary(t: np.ndarray, v: np.ndarray) -> dict:
    """Dominant period and the power fractions above 1 Hz and in the
    1.4-1.8 Hz band (the old pose-hold ringing) of a detrended series."""
    if t.size < 64:
        return {"peak_period_s": float("nan"), "hf_fraction": float("nan"),
                "ring_fraction": float("nan")}
    dt = float(np.median(np.diff(t)))
    tu = np.arange(t[0], t[-1], dt)
    vu = np.interp(tu, t, v)
    vu = vu - np.polyval(np.polyfit(tu - tu[0], vu, 1), tu - tu[0])
    spec = np.abs(np.fft.rfft(vu * np.hanning(vu.size))) ** 2
    f = np.fft.rfftfreq(vu.size, d=dt)
    spec[f < 0.03] = 0.0
    tot = max(float(spec.sum()), 1e-30)
    fp = float(f[int(np.argmax(spec))]) if spec.any() else 0.0
    return {"peak_period_s": (1.0 / fp) if fp > 0 else float("inf"),
            "hf_fraction": float(spec[f > 1.0].sum() / tot),
            "ring_fraction": float(spec[(f > 1.4) & (f < 1.8)].sum() / tot)}


def track_metrics(run: RunData, lines: list[Line]) -> dict:
    tr = run.track
    vx, vy = tr.velocity()
    sp = np.hypot(vx, vy)
    out: dict[str, Any] = {"lines": []}
    xte_all, crab_all, sog_all = [], [], []
    for k, ln in enumerate(lines):
        m, _a, c = _on_line(ln, tr.x, tr.y)
        moving = m & (sp > 0.3)
        if moving.sum() < 10:
            continue
        cog = np.arctan2(vy[moving], vx[moving])
        crab = np.degrees(wrap_pi(tr.yaw[moving] - cog))
        out["lines"].append({
            "line": k, "length_m": round(ln.length, 1),
            "heading_deg": round(float(np.degrees(ln.heading)), 1),
            "xte_mean_m": round(float(c[m].mean()), 3),
            "xte_rms_m": round(float(np.sqrt((c[m] ** 2).mean())), 3),
            "xte_max_m": round(float(np.abs(c[m]).max()), 3),
            "crab_mean_deg": round(float(crab.mean()), 2),
            "crab_std_deg": round(float(crab.std()), 2),
            "sog_mean_mps": round(float(sp[moving].mean()), 3),
            "sog_cv": round(float(sp[moving].std() / max(sp[moving].mean(), 1e-6)), 3)})
        xte_all.append(c[m])
        crab_all.append(crab)
        sog_all.append(sp[moving])
    if xte_all:
        xa, ca, sa = (np.concatenate(v) for v in (xte_all, crab_all, sog_all))
        out.update({"xte_rms_m": round(float(np.sqrt((xa ** 2).mean())), 3),
                    "xte_max_m": round(float(np.abs(xa).max()), 3),
                    "crab_abs_mean_deg": round(float(np.mean([abs(L["crab_mean_deg"])
                                                              for L in out["lines"]])), 2),
                    "crab_std_deg": round(float(ca.std()), 2),
                    "sog_mean_mps": round(float(sa.mean()), 3),
                    "sog_cv": round(float(sa.std() / max(sa.mean(), 1e-6)), 3)})
    return out


def attitude_metrics(run: RunData, lines: list[Line]) -> dict:
    tr = run.track
    sel = np.zeros(tr.t.size, bool)
    yaw_dev = np.zeros(tr.t.size)
    for ln in lines:
        m, _, _ = _on_line(ln, tr.x, tr.y)
        sel |= m
        yaw_dev[m] = wrap_pi(tr.yaw[m] - ln.heading)
    if sel.sum() < 32:
        sel = np.ones(tr.t.size, bool)
    t = tr.t[sel]
    dt = np.diff(tr.t)
    rate = (lambda a: np.diff(a)[sel[1:]] / np.maximum(dt[sel[1:]], 1e-3)) if tr.t.size > 1 else None
    out = {
        "samples": int(sel.sum()),
        "z_mean_m": round(float(tr.z[sel].mean()), 4),
        "heave_std_m": round(float(np.std(tr.z[sel])), 4),
        "heave_p2p_m": round(float(np.ptp(tr.z[sel])), 4),
        "roll_std_deg": round(_std_deg(tr.roll[sel]), 3),
        "pitch_std_deg": round(_std_deg(tr.pitch[sel]), 3),
        "roll_mean_deg": round(float(np.degrees(tr.roll[sel].mean())), 3),
        "pitch_mean_deg": round(float(np.degrees(tr.pitch[sel].mean())), 3),
        "yaw_dev_std_deg": round(_std_deg(yaw_dev[sel]), 3),
    }
    if rate is not None:
        out["roll_rate_std_dps"] = round(_std_deg(rate(tr.roll)), 3)
        out["pitch_rate_std_dps"] = round(_std_deg(rate(tr.pitch)), 3)
        out["yaw_rate_std_dps"] = round(_std_deg(rate(np.unwrap(tr.yaw))), 3)
    out["heave_spectrum"] = {k: round(v, 3) for k, v in
                             spectrum_summary(t, tr.z[sel]).items()}
    out["roll_spectrum"] = {k: round(v, 3) for k, v in
                            spectrum_summary(t, tr.roll[sel]).items()}
    return out


def _seabed(world_dir: str):
    """Height function of the world's scene (z, negative below the
    surface); a flat -5 m seabed without one."""
    try:
        from ..worldgen.scene import SceneModel  # noqa: PLC0415
        scene = SceneModel.load(world_dir)
        return lambda x, y: scene.sample_height(np.asarray(x, float), np.asarray(y, float))
    except Exception:  # noqa: BLE001 -- analysis must run on any input
        return lambda x, y: np.full(np.shape(x), -5.0)


def footprints(sensor: np.ndarray, side: str, ground_range_m: float,
               seabed) -> np.ndarray:
    """(N, 2) seabed point each ping's beam reaches at ``ground_range_m``,
    by the renderer's own geometry: a horizontal ground line along the
    look direction (yaw ± 90°), each sample shifted ``dz·tan(pitch)``
    along-track (``renderer._ping_geometry``)."""
    x, y, z, _r, p, yaw = (sensor[:, i] for i in range(6))
    look = yaw + SIDE_SIGN[side] * np.pi / 2.0
    gx = x + ground_range_m * np.cos(look)
    gy = y + ground_range_m * np.sin(look)
    zk = seabed(gx, gy)
    shift = (z - zk) * np.tan(p)
    return np.stack([gx + shift * np.cos(yaw), gy + shift * np.sin(yaw)], -1)


def footprint_metrics(run: RunData, lines: list[Line]) -> dict:
    """Along-track footprint spacing, back-scan and gap fractions per side
    and range, and the common/differential split of the footprint
    displacement (pitch-like vs yaw-like)."""
    if run.pings is None or not lines:
        return {}
    seabed = _seabed(run.world_dir)
    pg = run.pings
    out: dict[str, Any] = {}
    beam = np.radians(BEAMWIDTH_DEG)
    for r in FOOTPRINT_RANGES_M:
        key = f"r{int(r)}"
        res: dict[str, Any] = {}
        disp = {}
        for side in SIDES:
            fp = footprints(pg.sensor[side], side, r, seabed)
            steps, back, gap = [], 0, 0
            d_side = np.full(fp.shape[0], np.nan)
            alt = np.nanmedian(pg.altitude[side]) if pg.altitude[side].size else 2.0
            width = 2.0 * np.hypot(r, alt) * np.tan(beam / 2.0)
            for ln in lines:
                m, a_boat, _ = _on_line(ln, pg.boat[:, 0], pg.boat[:, 1])
                if m.sum() < 10:
                    continue
                a_fp, _ = ln.project(fp[m, 0], fp[m, 1])
                ds = np.diff(a_fp)
                steps.append(ds)
                back += int((ds < 0).sum())
                gap += int((ds > width).sum())
                d_side[m] = a_fp - a_boat[m]
            disp[side] = d_side
            if steps:
                s = np.concatenate(steps)
                res[side] = {"steps": int(s.size),
                             "spacing_mean_m": round(float(s.mean()), 4),
                             "spacing_cv": round(float(s.std() / max(abs(s.mean()), 1e-9)), 3),
                             "backscan_fraction": round(back / max(s.size, 1), 4),
                             "gap_fraction": round(gap / max(s.size, 1), 4),
                             "footprint_width_m": round(float(width), 4)}
        if disp:
            dp, dsb = disp["port"], disp["starboard"]
            ok = np.isfinite(dp) & np.isfinite(dsb)
            if ok.sum() > 10:
                dp0, ds0 = dp[ok] - np.median(dp[ok]), dsb[ok] - np.median(dsb[ok])
                res["common_mode_std_m"] = round(float(np.std((dp0 + ds0) / 2)), 4)
                res["differential_mode_std_m"] = round(float(np.std((dp0 - ds0) / 2)), 4)
        out[key] = res
    out["pose_age_ms_max"] = round(float(pg.pose_age_ms.max()), 1) if pg.pose_age_ms.size else None
    out["pose_age_frac_over_60ms"] = (round(float((pg.pose_age_ms > 60.0).mean()), 5)
                                      if pg.pose_age_ms.size else None)
    return out


def target_hits(run: RunData, lines: list[Line]) -> dict:
    """Pings insonifying each known object on each pass: the target must lie
    within half a footprint width of the beam's (pitch-shifted) fan."""
    if run.pings is None or not run.targets or not lines:
        return {}
    pg = run.pings
    seabed = _seabed(run.world_dir)
    beam = np.radians(BEAMWIDTH_DEG)
    passes = []
    for k, ln in enumerate(lines):
        m, _, _ = _on_line(ln, pg.boat[:, 0], pg.boat[:, 1], margin_m=0.0)
        if m.sum() < 10:
            continue
        for o in run.targets:
            tx, ty = float(o["x"]), float(o["y"])
            tz = float(seabed(np.array([tx]), np.array([ty]))[0])
            for side in SIDES:
                s = pg.sensor[side][m]
                yaw, pitch = s[:, 5], s[:, 4]
                fwd = np.stack([np.cos(yaw), np.sin(yaw)], -1)
                look = yaw + SIDE_SIGN[side] * np.pi / 2.0
                lk = np.stack([np.cos(look), np.sin(look)], -1)
                d = np.stack([tx - s[:, 0], ty - s[:, 1]], -1)
                a = (d * lk).sum(1)
                f = (d * fwd).sum(1) - (s[:, 2] - tz) * np.tan(pitch)
                if np.median(a) < 1.0 or np.median(a) > 19.0:
                    continue                     # not on this side's swath
                width = 2.0 * np.hypot(a, s[:, 2] - tz) * np.tan(beam / 2.0)
                hit = (a > 0.5) & (np.abs(f) < width / 2.0)
                passes.append({"line": k, "object_id": o.get("object_id"),
                               "type": o.get("type"), "side": side,
                               "ground_range_m": round(float(np.median(a)), 2),
                               "hits": int(hit.sum())})
    if not passes:
        return {}
    h = np.array([p["hits"] for p in passes], dtype=float)
    return {"passes": passes, "n_passes": int(h.size),
            "hits_mean": round(float(h.mean()), 2),
            "hits_cv": round(float(h.std() / max(h.mean(), 1e-9)), 3),
            "hits_min": int(h.min()), "hits_max": int(h.max()),
            "missed_fraction": round(float((h == 0).mean()), 3)}


def current_metrics(run: RunData) -> dict:
    """Realised current at the boat (readback ``current.instant``, which
    excludes the wave orbital velocity) against the configured mean."""
    rows = [(d["t_sim"], d["current"]) for d in run.sea_status
            if isinstance(d.get("current"), dict) and "instant" in d["current"]]
    if not rows:
        return {}
    c = rows[-1][1]
    mean_cfg = float(c.get("mean_speed_mps", 0.0))
    if mean_cfg <= 0.0:
        sp = np.array([r[1]["instant"]["speed_mps"] for r in rows])
        return {"configured_mps": 0.0, "speed_max_mps": round(float(sp.max()), 4)}
    sp = np.array([r[1]["instant"]["speed_mps"] for r in rows])
    frm = np.array([r[1]["instant"]["from_deg"] for r in rows])
    dfrm = (frm - float(c["from_deg"]) + 180.0) % 360.0 - 180.0
    sig = float(c.get("sigma_speed_mps", 0.0))
    sig_d = float(c.get("sigma_dir_deg", 0.0))
    from ..sea.current import FLUCTUATION_LIMIT, EddyField  # noqa: PLC0415
    eddy_bound = EddyField(0).bound * float(c.get("spatial_rms_mps", 0.0))
    return {"configured_mps": mean_cfg, "configured_from_deg": float(c["from_deg"]),
            "variation": c.get("variation", ""),
            "speed_mean_mps": round(float(sp.mean()), 4),
            "speed_std_mps": round(float(sp.std()), 4),
            "speed_dev_max_mps": round(float(np.abs(sp - mean_cfg).max()), 4),
            "speed_bound_mps": round(FLUCTUATION_LIMIT * sig + eddy_bound, 4),
            "from_mean_dev_deg": round(float(dfrm.mean()), 2),
            "from_dev_max_deg": round(float(np.abs(dfrm).max()), 2),
            "from_std_deg": round(float(dfrm.std()), 2),
            "from_bound_deg": round(FLUCTUATION_LIMIT * sig_d, 2),
            "samples": len(rows)}


def sea_metrics(run: RunData) -> dict:
    if not run.sea_status:
        return {}
    last = run.sea_status[-1]
    # Pose age while surveying: the first seconds after spawn (the node's
    # first odometry is still arriving) are reported apart, not hidden.
    t0 = float(run.sea_status[0].get("t_sim", 0.0))
    ages = [d.get("pose_age_ms", 0.0) for d in run.sea_status
            if float(d.get("t_sim", 0.0)) - t0 >= STARTUP_S]
    ages_start = [d.get("pose_age_ms", 0.0) for d in run.sea_status
                  if float(d.get("t_sim", 0.0)) - t0 < STARTUP_S]
    return {"wrench_messages": int(last.get("wrench_messages", 0)),
            "waves_status": last.get("waves", {}).get("status", ""),
            "hs_m": last.get("waves", {}).get("hs_m", 0.0),
            "tp_s": last.get("waves", {}).get("tp_s", 0.0),
            "summary": last.get("summary", ""),
            "pose_age_ms_max": round(float(max(ages)), 1) if ages else None,
            "pose_age_ms_startup_max": (round(float(max(ages_start)), 1)
                                        if ages_start else None),
            "boat_readback": last.get("boat", {}),
            "hull": last.get("hull", {})}


# ================================================================= imagery
def waterfall(run: RunData, line: Line, side: str) -> tuple[np.ndarray, np.ndarray] | None:
    """(rows, slant axis) of a side's dB profiles for the pings taken on
    ``line`` (time order)."""
    sp = run.profiles.get(side)
    if sp is None or sp.t.size == 0:
        return None
    tr = run.track
    bx = np.interp(sp.t, tr.t, tr.x)
    by = np.interp(sp.t, tr.t, tr.y)
    m, _, _ = _on_line(line, bx, by, margin_m=0.0)
    if m.sum() < 5:
        return None
    return sp.db[m], sp.slant_m


def naive_mosaic(run: RunData, extent: tuple[float, float, float, float],
                 res_m: float = 0.1) -> np.ndarray:
    """Mean-dB grid of every ping placed with the vehicle position and
    heading only (no attitude, no lever arm, flat-seabed slant correction
    with the ping's altitude): what a display without motion correction
    shows. ``extent`` = (x0, x1, y0, y1); NaN where nothing landed."""
    x0, x1, y0, y1 = extent
    nx, ny = int(np.ceil((x1 - x0) / res_m)), int(np.ceil((y1 - y0) / res_m))
    acc = np.zeros((ny, nx))
    cnt = np.zeros((ny, nx))
    tr = run.track
    for side, sp in run.profiles.items():
        if sp.t.size == 0:
            continue
        bx = np.interp(sp.t, tr.t, tr.x)
        by = np.interp(sp.t, tr.t, tr.y)
        yaw = np.interp(sp.t, tr.t, np.unwrap(tr.yaw))
        if run.pings is not None and run.pings.t.size:
            alt = np.interp(sp.t, run.pings.t, run.pings.altitude[side])
        else:
            alt = np.full(sp.t.size, 2.0)
        rs = sp.slant_m
        look = yaw + SIDE_SIGN[side] * np.pi / 2.0
        for i in range(sp.t.size):
            ok = rs > alt[i]
            rg = np.sqrt(rs[ok] ** 2 - alt[i] ** 2)
            px = bx[i] + rg * np.cos(look[i])
            py = by[i] + rg * np.sin(look[i])
            ix = ((px - x0) / res_m).astype(int)
            iy = ((py - y0) / res_m).astype(int)
            inb = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            np.add.at(acc, (iy[inb], ix[inb]), sp.db[i][ok][inb])
            np.add.at(cnt, (iy[inb], ix[inb]), 1.0)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(cnt > 0, acc / cnt, np.nan)


MOSAIC_RES_M = 0.1


def run_mosaic(run: RunData) -> tuple[np.ndarray, tuple]:
    """The run's naive mosaic over the target area (computed once)."""
    cached = getattr(run, "_mosaic", None)
    if cached is None:
        ext = mosaic_extent(run)
        cached = (naive_mosaic(run, ext, MOSAIC_RES_M), ext)
        run._mosaic = cached                 # noqa: SLF001 -- per-run cache
    return cached


def target_chip(mosaic: np.ndarray, extent, target: dict,
                half_m: float = 3.0, res_m: float = MOSAIC_RES_M) -> np.ndarray | None:
    """The square of mosaic around a target (NaN where nothing landed)."""
    x0, _, y0, _ = extent
    cx, cy = (float(target["x"]) - x0) / res_m, (float(target["y"]) - y0) / res_m
    h = int(round(half_m / res_m))
    i0, j0 = int(round(cy)) - h, int(round(cx)) - h
    if i0 < 0 or j0 < 0 or i0 + 2 * h >= mosaic.shape[0] or j0 + 2 * h >= mosaic.shape[1]:
        return None
    return mosaic[i0:i0 + 2 * h + 1, j0:j0 + 2 * h + 1]


def _norm(chip: np.ndarray) -> np.ndarray:
    c = np.where(np.isfinite(chip), chip, np.nanmean(chip))
    c = c - c.mean()
    return c / max(float(np.sqrt((c * c).sum())), 1e-12)


def mosaic_vs_reference(mos: np.ndarray, ref: np.ndarray, extent,
                        targets: list[dict], max_shift_m: float = 1.5,
                        half_m: float = 3.0) -> dict:
    """How each target's image moved and changed against the reference run
    (calm water): the 2-D shift maximising the normalised correlation of
    the two mosaic chips, and that correlation (1 = the same picture).
    Both mosaics are placed the same naive way, so any shift or loss of
    similarity is the platform motion, not the placement."""
    rows = []
    s = int(round(max_shift_m / MOSAIC_RES_M))
    for o in targets:
        a = target_chip(ref, extent, o, half_m + max_shift_m)
        b = target_chip(mos, extent, o, half_m)
        if a is None or b is None or not np.isfinite(b).any():
            continue
        bn = _norm(b)
        n = b.shape[0]
        best, bij = -2.0, (0, 0)
        for di in range(-s, s + 1):
            for dj in range(-s, s + 1):
                w = a[s + di:s + di + n, s + dj:s + dj + n]
                c = float((bn * _norm(w)).sum())
                if c > best:
                    best, bij = c, (di, dj)
        rows.append({"object_id": o.get("object_id"), "type": o.get("type"),
                     "shift_m": round(float(np.hypot(*bij)) * MOSAIC_RES_M, 3),
                     "corr": round(best, 3)})
    if not rows:
        return {}
    sh = np.array([r["shift_m"] for r in rows])
    co = np.array([r["corr"] for r in rows])
    return {"targets": rows, "n": len(rows),
            "shift_median_m": round(float(np.median(sh)), 3),
            "shift_max_m": round(float(sh.max()), 3),
            "corr_median": round(float(np.median(co)), 3),
            "corr_min": round(float(co.min()), 3)}


def compare_to_reference(runs: list[RunData], results: list[dict],
                         ref_name: str = "calm") -> None:
    """Add ``mosaic_vs_calm`` to every result (the reference's own is the
    self-comparison: shift 0, correlation 1)."""
    ref = next((r for r in runs if r.name == ref_name), None)
    if ref is None or not ref.targets or not ref.profiles:
        return
    ref_mos, ext = run_mosaic(ref)
    for run, res in zip(runs, results):
        if not run.targets or not run.profiles:
            continue
        mos, ext_r = run_mosaic(run)
        if mos.shape != ref_mos.shape:
            continue
        res["mosaic_vs_calm"] = mosaic_vs_reference(mos, ref_mos, ext, run.targets)


# ================================================================ analysis
def analyse(run: RunData) -> dict:
    lines = survey_lines(run.trajectory) if run.trajectory is not None else []
    res = {"name": run.name, "scenario": run.scenario,
           "lines": [{"p0": ln.p0.round(2).tolist(), "p1": ln.p1.round(2).tolist()}
                     for ln in lines],
           "duration_s": round(float(run.track.t[-1] - run.track.t[0]), 1)
           if run.track.t.size else 0.0,
           "track": track_metrics(run, lines),
           "attitude": attitude_metrics(run, lines),
           "footprint": footprint_metrics(run, lines),
           "targets": target_hits(run, lines),
           "current": current_metrics(run),
           "sea": sea_metrics(run)}
    return res


def mosaic_extent(run: RunData, pad_m: float = 4.0):
    xs = [float(o["x"]) for o in run.targets] or [0.0]
    ys = [float(o["y"]) for o in run.targets] or [0.0]
    return (min(xs) - pad_m, max(xs) + pad_m, min(ys) - pad_m, max(ys) + pad_m)


# ================================================================== figures
def _plt():
    import matplotlib  # noqa: PLC0415
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415
    return plt


SCENARIO_COLORS = {"calm": "#2a6fdb", "current": "#e08a00", "waves": "#1a9e5a",
                   "both": "#c0392b"}


def plot_run(run: RunData, out: Path) -> list[str]:
    """Per-run figures: waterfalls of every line and the naive mosaic."""
    plt = _plt()
    out.mkdir(parents=True, exist_ok=True)
    files = []
    lines = survey_lines(run.trajectory) if run.trajectory is not None else []
    for k, ln in enumerate(lines):
        wp, ws = waterfall(run, ln, "port"), waterfall(run, ln, "starboard")
        if wp is None or ws is None:
            continue
        n = min(wp[0].shape[0], ws[0].shape[0])
        img = np.hstack([wp[0][:n, ::-1], ws[0][:n]])
        lo, hi = np.nanpercentile(img, [5, 99.5])
        fig, ax = plt.subplots(figsize=(7, 8))
        rmax = float(wp[1][-1])
        ax.imshow(img, aspect="auto", cmap="copper", vmin=lo, vmax=hi,
                  extent=(-rmax, rmax, n, 0), interpolation="nearest")
        ax.set_xlabel("slant range [m]  (port ← | → starboard)")
        ax.set_ylabel("ping (time ↓)")
        ax.set_title(f"{run.name}: waterfall, line {k + 1}")
        f = out / f"waterfall_line{k + 1}.png"
        fig.tight_layout()
        fig.savefig(f, dpi=110)
        plt.close(fig)
        files.append(f.name)
    if run.targets and run.profiles:
        mos, ext = run_mosaic(run)
        fig, ax = plt.subplots(figsize=(9, 6))
        lo, hi = (np.nanpercentile(mos, [5, 99.5]) if np.isfinite(mos).any()
                  else (0, 1))
        ax.imshow(mos, origin="lower", extent=ext, cmap="copper", vmin=lo, vmax=hi)
        ax.scatter([o["x"] for o in run.targets], [o["y"] for o in run.targets],
                   s=60, facecolors="none", edgecolors="cyan", linewidths=1.2,
                   label="true target")
        ax.set_aspect("equal")
        ax.set_xlabel("x east [m]")
        ax.set_ylabel("y north [m]")
        ax.set_title(f"{run.name}: mosaic placed with position + heading only")
        ax.legend(loc="upper right", fontsize=8)
        f = out / "mosaic_naive.png"
        fig.tight_layout()
        fig.savefig(f, dpi=110)
        plt.close(fig)
        files.append(f.name)
    return files


def plot_compare(runs: list[RunData], results: list[dict], out: Path) -> list[str]:
    """The side-by-side story: tracks, attitude, footprint staircase,
    waterfalls of the same line, naive mosaics, realised current."""
    plt = _plt()
    out.mkdir(parents=True, exist_ok=True)
    files = []
    col = lambda r: SCENARIO_COLORS.get(r.name, None)  # noqa: E731

    # 1. tracks
    fig, ax = plt.subplots(figsize=(9, 6))
    if runs and runs[0].trajectory is not None and runs[0].trajectory.size:
        tj = runs[0].trajectory
        ax.plot(tj[:, 1], tj[:, 2], "k--", lw=1, label="planned path")
    for r in runs:
        ax.plot(r.track.x, r.track.y, color=col(r), lw=1.3, label=r.name)
    if runs and runs[0].targets:
        ax.scatter([o["x"] for o in runs[0].targets], [o["y"] for o in runs[0].targets],
                   marker="s", s=18, c="0.3", label="targets")
    ax.set_aspect("equal")
    ax.set_xlabel("x east [m]")
    ax.set_ylabel("y north [m]")
    ax.legend(fontsize=8)
    ax.set_title("Boat track: same mission, four sea states")
    fig.tight_layout()
    fig.savefig(out / "compare_tracks.png", dpi=110)
    plt.close(fig)
    files.append("compare_tracks.png")

    # 2. attitude time series
    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for r in runs:
        t = r.track.t - (r.track.t[0] if r.track.t.size else 0.0)
        axs[0].plot(t, np.degrees(r.track.roll), color=col(r), lw=0.8, label=r.name)
        axs[1].plot(t, np.degrees(r.track.pitch), color=col(r), lw=0.8)
        axs[2].plot(t, r.track.z - np.median(r.track.z), color=col(r), lw=0.8)
        axs[3].plot(t, np.degrees(np.unwrap(r.track.yaw)), color=col(r), lw=0.8)
    for a, lab in zip(axs, ("roll [deg]", "pitch [deg]", "heave [m]", "heading [deg]")):
        a.set_ylabel(lab)
        a.grid(alpha=0.3)
    axs[0].legend(fontsize=8, ncol=4)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle("Boat attitude (Gazebo odometry)")
    fig.tight_layout()
    fig.savefig(out / "compare_attitude.png", dpi=110)
    plt.close(fig)
    files.append("compare_attitude.png")

    # 3. footprint staircase on the first line, starboard 15 m
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
    for ax, side in zip(axs, SIDES):
        for r in runs:
            if r.pings is None or r.trajectory is None:
                continue
            lines = survey_lines(r.trajectory)
            if not lines:
                continue
            ln = lines[0]
            m, ab, _ = _on_line(ln, r.pings.boat[:, 0], r.pings.boat[:, 1])
            idx = np.flatnonzero(m)
            if idx.size < 20:
                continue
            idx = idx[idx.size // 2: idx.size // 2 + 200]
            fp = footprints(r.pings.sensor[side][idx], side, 15.0, _seabed(r.world_dir))
            a_fp, _ = ln.project(fp[:, 0], fp[:, 1])
            ax.plot(np.arange(idx.size), a_fp - a_fp[0], color=col(r), lw=1.0,
                    drawstyle="steps-post", label=r.name)
        ax.set_title(f"{side}: where each ping lands, 15 m out")
        ax.set_xlabel("ping")
        ax.grid(alpha=0.3)
    axs[0].set_ylabel("along-track position on the seabed [m]")
    axs[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "compare_footprint.png", dpi=110)
    plt.close(fig)
    files.append("compare_footprint.png")

    # 4. waterfalls of line 1 side by side
    wfs = []
    for r in runs:
        lines = survey_lines(r.trajectory) if r.trajectory is not None else []
        if not lines:
            continue
        wp, ws = waterfall(r, lines[0], "port"), waterfall(r, lines[0], "starboard")
        if wp is not None and ws is not None:
            n = min(wp[0].shape[0], ws[0].shape[0])
            wfs.append((r.name, np.hstack([wp[0][:n, ::-1], ws[0][:n]]), float(wp[1][-1])))
    if wfs:
        fig, axs = plt.subplots(1, len(wfs), figsize=(4.2 * len(wfs), 8), squeeze=False)
        lo, hi = np.nanpercentile(wfs[0][1], [5, 99.5])
        for ax, (nm, img, rmax) in zip(axs[0], wfs):
            ax.imshow(img, aspect="auto", cmap="copper", vmin=lo, vmax=hi,
                      extent=(-rmax, rmax, img.shape[0], 0), interpolation="nearest")
            ax.set_title(nm)
            ax.set_xlabel("slant range [m]")
        axs[0][0].set_ylabel("ping (time ↓)")
        fig.suptitle("Same survey line, same seabed: waterfall per sea state")
        fig.tight_layout()
        fig.savefig(out / "compare_waterfalls.png", dpi=100)
        plt.close(fig)
        files.append("compare_waterfalls.png")

    # 5. naive mosaics
    mos = [(r.name, *run_mosaic(r), r.targets) for r in runs if r.targets and r.profiles]
    if mos:
        n = len(mos)
        cols = 2 if n > 1 else 1
        rows = int(np.ceil(n / cols))
        fig, axs = plt.subplots(rows, cols, figsize=(8 * cols, 5 * rows), squeeze=False)
        ref = mos[0][1]
        lo, hi = (np.nanpercentile(ref, [5, 99.5]) if np.isfinite(ref).any() else (0, 1))
        for ax, (nm, m, ext, tg) in zip(axs.ravel(), mos):
            ax.imshow(m, origin="lower", extent=ext, cmap="copper", vmin=lo, vmax=hi)
            ax.scatter([o["x"] for o in tg], [o["y"] for o in tg], s=50,
                       facecolors="none", edgecolors="cyan", linewidths=1.0)
            ax.set_title(f"{nm}: mosaic from position + heading only")
            ax.set_aspect("equal")
        for ax in axs.ravel()[n:]:
            ax.axis("off")
        fig.tight_layout()
        fig.savefig(out / "compare_mosaics.png", dpi=100)
        plt.close(fig)
        files.append("compare_mosaics.png")

    # 5b. the same targets up close: waterfall window and mosaic chips
    wz = []
    for r in runs:
        lines = survey_lines(r.trajectory) if r.trajectory is not None else []
        if not lines or "port" not in r.profiles or "starboard" not in r.profiles:
            continue
        ln = lines[0]
        rows = {}
        for side in SIDES:
            sp = r.profiles[side]
            a_b, _ = ln.project(np.interp(sp.t, r.track.t, r.track.x),
                                np.interp(sp.t, r.track.t, r.track.y))
            m = (a_b > 24.0) & (a_b < 40.0)
            rows[side] = (sp.db[m], sp.slant_m)
        n = min(rows["port"][0].shape[0], rows["starboard"][0].shape[0])
        if n < 10:
            continue
        keep = rows["port"][1] <= 12.0
        img = np.hstack([rows["port"][0][:n, keep][:, ::-1], rows["starboard"][0][:n, keep]])
        wz.append((r.name, img, float(rows["port"][1][keep][-1])))
    if wz:
        fig, axs = plt.subplots(1, len(wz), figsize=(3.6 * len(wz), 7.5), squeeze=False)
        lo, hi = np.nanpercentile(wz[0][1], [5, 99.7])
        for ax, (nm, img, rmax) in zip(axs[0], wz):
            ax.imshow(img, aspect="auto", cmap="copper", vmin=lo, vmax=hi,
                      extent=(-rmax, rmax, img.shape[0], 0), interpolation="nearest")
            ax.set_title(nm)
            ax.set_xlabel("slant range [m]")
        axs[0][0].set_ylabel("ping (time ↓)")
        fig.suptitle("Line 1, x = 24-40 m: a tyre (port), boxes and pipes (starboard)")
        fig.tight_layout()
        fig.savefig(out / "compare_waterfall_zoom.png", dpi=110)
        plt.close(fig)
        files.append("compare_waterfall_zoom.png")
    chip_runs = [r for r in runs if r.targets and r.profiles]
    if chip_runs:
        pick = []
        for typ in ("tire_car", "plastic_box", "pipe_pvc"):
            pick += [o for o in chip_runs[0].targets if o.get("type") == typ][:2]
        pick = pick[:6]
        fig, axs = plt.subplots(len(chip_runs), len(pick),
                                figsize=(1.9 * len(pick), 2.0 * len(chip_runs)),
                                squeeze=False)
        for i, r in enumerate(chip_runs):
            mos, ext = run_mosaic(r)
            for j, o in enumerate(pick):
                ax = axs[i][j]
                ch = target_chip(mos, ext, o, 3.0)
                if ch is not None and np.isfinite(ch).any():
                    lo, hi = np.nanpercentile(ch, [5, 99.5])
                    ax.imshow(ch, origin="lower", cmap="copper", vmin=lo, vmax=hi,
                              extent=(-3, 3, -3, 3))
                ax.plot([0], [0], "+", color="cyan", ms=10, mew=1.2)
                ax.set_xticks([])
                ax.set_yticks([])
                if i == 0:
                    ax.set_title(f"{o.get('type')}\n({o['x']:.0f}, {o['y']:.0f})", fontsize=8)
                if j == 0:
                    ax.set_ylabel(r.name)
        fig.suptitle("Each target, 6 x 6 m of mosaic (+ = true position)", fontsize=10)
        fig.tight_layout()
        fig.savefig(out / "compare_target_chips.png", dpi=110)
        plt.close(fig)
        files.append("compare_target_chips.png")

    # 6. realised current
    if any(r.sea_status for r in runs):
        fig, axs = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)
        for r in runs:
            rows = [(d["t_sim"], d["current"]) for d in r.sea_status
                    if isinstance(d.get("current"), dict) and "instant" in d["current"]]
            if not rows or rows[-1][1].get("mean_speed_mps", 0.0) <= 0.0:
                continue
            t = np.array([x[0] for x in rows])
            axs[0].plot(t - t[0], [x[1]["instant"]["speed_mps"] for x in rows],
                        color=col(r), lw=1.0, label=r.name)
            axs[0].axhline(rows[-1][1]["mean_speed_mps"], color=col(r), ls="--", lw=0.8)
            mean_from = float(rows[-1][1]["from_deg"])
            dev = [(x[1]["instant"]["from_deg"] - mean_from + 180.0) % 360.0 - 180.0
                   for x in rows]                     # no 0/360 wrap jumps
            axs[1].plot(t - t[0], dev, color=col(r), lw=1.0)
            axs[1].axhline(0.0, color=col(r), ls="--", lw=0.8)
        axs[0].set_ylabel("speed at the boat [m/s]")
        axs[1].set_ylabel("direction - mean [deg]")
        axs[1].set_xlabel("time [s]")
        if axs[0].get_legend_handles_labels()[0]:
            axs[0].legend(fontsize=8)
        axs[0].set_title("Current felt by the boat: configured mean (dashed) and its variation")
        for a in axs:
            a.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "compare_current.png", dpi=110)
        plt.close(fig)
        files.append("compare_current.png")
    return files


# ====================================================================== I/O
def load_run_dir(run_dir: Path) -> RunData:
    """A runner output folder: ``bag/`` + ``run.yaml`` (world, trajectory,
    scenario)."""
    meta = yaml.safe_load((run_dir / "run.yaml").read_text()) or {}
    run = read_bag(run_dir / "bag", name=str(meta.get("name", run_dir.name)))
    run.world_dir = str(meta.get("world_dir", ""))
    if meta.get("trajectory"):
        run.trajectory = read_trajectory(meta["trajectory"])
    run.targets = read_targets(run.world_dir) if run.world_dir else []
    run.scenario = dict(meta.get("scenario") or {})
    return run


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def write_metrics(res: dict, out: Path) -> Path:
    out.mkdir(parents=True, exist_ok=True)
    p = out / "metrics.json"
    p.write_text(json.dumps(res, indent=2, default=_json_default))
    return p


SUMMARY_ROWS = (
    ("Cross-track error RMS [m]", ("track", "xte_rms_m")),
    ("Crab angle |mean| [deg]", ("track", "crab_abs_mean_deg")),
    ("Speed over ground CV", ("track", "sog_cv")),
    ("Roll std [deg]", ("attitude", "roll_std_deg")),
    ("Pitch std [deg]", ("attitude", "pitch_std_deg")),
    ("Heave std [m]", ("attitude", "heave_std_m")),
    ("Heading wobble std [deg]", ("attitude", "yaw_dev_std_deg")),
    ("Footprint back-scan, stbd 15 m", ("footprint", "r15", "starboard", "backscan_fraction")),
    ("Footprint gaps, stbd 15 m", ("footprint", "r15", "starboard", "gap_fraction")),
    ("Footprint spacing CV, port 5 m", ("footprint", "r5", "port", "spacing_cv")),
    ("Common-mode (pitch-like) std 5 m [m]", ("footprint", "r5", "common_mode_std_m")),
    ("Differential (yaw-like) std 15 m [m]", ("footprint", "r15", "differential_mode_std_m")),
    ("Pings per target pass, mean", ("targets", "hits_mean")),
    ("Pings per target pass, CV", ("targets", "hits_cv")),
    ("Target passes missed", ("targets", "missed_fraction")),
    ("Target shift vs calm, median [m]", ("mosaic_vs_calm", "shift_median_m")),
    ("Target image similarity to calm (corr.)", ("mosaic_vs_calm", "corr_median")),
    ("Current at boat, mean [m/s]", ("current", "speed_mean_mps")),
)


def _get(d: dict, path: tuple):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def summary_table(results: list[dict]) -> str:
    head = "| Quantity | " + " | ".join(r["name"] for r in results) + " |"
    sep = "|---|" + "---|" * len(results)
    rows = [head, sep]
    for label, path in SUMMARY_ROWS:
        vals = []
        for r in results:
            v = _get(r, path)
            vals.append("—" if v is None else (f"{v:.3g}" if isinstance(v, float) else str(v)))
        rows.append(f"| {label} | " + " | ".join(vals) + " |")
    return "\n".join(rows)


# ===================================================================== svlog
def analyse_svlog(path: str | Path, altitude_m: float) -> dict:
    """Field corpus: attitude statistics from id-150 ATTITUDE and the
    footprint along-track displacement a pitch/yaw history implies at
    ``altitude_m`` (flat seabed), for side-by-side with a simulated run."""
    from .svlog_reader import read_svlog  # noqa: PLC0415
    s = read_svlog(path)
    if not s.attitude:
        return {"file": str(path), "error": "no ATTITUDE records"}
    att = s.attitude
    t = np.array([a.time_boot_ms for a in att], dtype=np.float64) / 1000.0
    if s.clock_frozen or np.ptp(t) <= 0:
        t = np.arange(len(att)) / 25.0          # the corpus's 25 Hz
    roll = np.array([a.roll for a in att])
    pitch = np.array([a.pitch for a in att])
    yaw = np.unwrap(np.array([a.yaw for a in att]))
    dt = np.maximum(np.diff(t), 1e-3)
    yaw_hp = yaw - np.convolve(yaw, np.ones(51) / 51, mode="same")   # ~2 s high-pass
    res = {"file": str(path), "samples": len(att),
           "duration_s": round(float(t[-1] - t[0]), 1),
           "roll_std_deg": round(_std_deg(roll - roll.mean()), 3),
           "pitch_std_deg": round(_std_deg(pitch - pitch.mean()), 3),
           "pitch_mean_deg": round(float(np.degrees(pitch.mean())), 3),
           "yaw_wobble_std_deg": round(_std_deg(yaw_hp[25:-25]), 3),
           "roll_rate_std_dps": round(_std_deg(np.diff(roll) / dt), 3),
           "pitch_rate_std_dps": round(_std_deg(np.diff(pitch) / dt), 3),
           "yaw_rate_std_dps": round(_std_deg(np.diff(yaw) / dt), 3),
           "roll_spectrum": {k: round(v, 3) for k, v in spectrum_summary(t, roll).items()},
           "altitude_m": altitude_m}
    # Footprint along-track displacement implied by the attitude alone
    # (the platform-motion part; the forward motion is common to both).
    for r in FOOTPRINT_RANGES_M:
        common = altitude_m * np.tan(pitch - pitch.mean())
        diff = r * np.sin(yaw_hp)
        res[f"r{int(r)}"] = {"common_mode_std_m": round(float(common.std()), 4),
                             "differential_mode_std_m": round(float(diff[25:-25].std()), 4)}
    pos = s.positions
    if len(pos) > 10:
        pt = np.array([p.time_boot_ms for p in pos], dtype=np.float64) / 1000.0
        px = np.array([p.x for p in pos])
        py = np.array([p.y for p in pos])
        if np.ptp(pt) > 0:
            v = np.hypot(np.gradient(px, pt), np.gradient(py, pt))
            res["speed_mean_mps"] = round(float(np.median(v)), 3)
    return res


# ======================================================================= CLI
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="sea_validation",
        description="Measure what waves and current do to a side-scan survey "
                    "(docs/sea_state_validation.md).")
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("run", help="analyse one recorded Gazebo run (needs ROS)")
    a.add_argument("run_dir", nargs="?", help="runner output folder (bag/ + run.yaml)")
    a.add_argument("--bag")
    a.add_argument("--world-dir", default="")
    a.add_argument("--trajectory", default="")
    a.add_argument("--name", default="")
    a.add_argument("--out", required=True)
    c = sub.add_parser("compare", help="side-by-side figures and table of several runs")
    c.add_argument("run_dirs", nargs="+")
    c.add_argument("--out", required=True)
    s = sub.add_parser("svlog", help="attitude/footprint statistics of a field .svlog")
    s.add_argument("files", nargs="+")
    s.add_argument("--altitude", type=float, default=2.0,
                   help="seabed altitude for the footprint displacement [m]")
    s.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    out = Path(args.out).expanduser()

    if args.cmd == "run":
        if args.run_dir:
            run = load_run_dir(Path(args.run_dir))
        else:
            run = read_bag(args.bag, args.name)
            run.world_dir = args.world_dir
            run.trajectory = read_trajectory(args.trajectory) if args.trajectory else None
            run.targets = read_targets(args.world_dir) if args.world_dir else []
        res = analyse(run)
        write_metrics(res, out)
        plot_run(run, out)
        print(json.dumps({k: res[k] for k in ("track", "attitude", "sea")},
                         indent=1, default=_json_default)[:3000])
        return 0
    if args.cmd == "compare":
        runs = [load_run_dir(Path(d)) for d in args.run_dirs]
        results = [analyse(r) for r in runs]
        compare_to_reference(runs, results)
        out.mkdir(parents=True, exist_ok=True)
        files = plot_compare(runs, results, out)
        (out / "summary.md").write_text(summary_table(results) + "\n")
        (out / "results.json").write_text(json.dumps(results, indent=2,
                                                     default=_json_default))
        print(summary_table(results))
        print("figures:", ", ".join(files))
        return 0
    res = [analyse_svlog(f, args.altitude) for f in args.files]
    out.mkdir(parents=True, exist_ok=True)
    (out / "svlog_attitude.json").write_text(json.dumps(res, indent=2,
                                                        default=_json_default))
    for r in res:
        print(json.dumps(r, default=_json_default))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
