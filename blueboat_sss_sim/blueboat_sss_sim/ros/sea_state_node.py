#!/usr/bin/env python3
"""Sea-state node: makes the simulated BlueBoat feel current and waves.

A thin ROS shell around :mod:`blueboat_sss_sim.sea` (ROS-free, smoke-
tested). Per tick it publishes the two things Gazebo cannot produce on
its own -- the model is documented in ``docs/sea_state_model.md``:

* the world-frame **water velocity** (Gauss-Markov current + wave orbital
  velocity) on ROS ``/current``, which the existing ros_gz bridge forwards
  to gz ``/ocean_current`` where the boat's ``Hydrodynamics`` plugin turns
  it into drag (relative-velocity damping);
* the **hull wrench** -- the catamaran's buoyancy under the actual
  surface (flat on calm water, the wave surface otherwise) minus the
  Gazebo box's flat-plane buoyancy, plus the hull damping -- as one-shot
  ``ros_gz_interfaces/EntityWrench`` messages on ``/world/<w>/wrench``.
  Since 2026-10-09 it is published in **every** run, calm water included
  (``hull.hydrostatics: catamaran``), so every scenario floats the same
  hull and a change of sea never switches the hydrostatics; it is
  bridged to gz-sim's ``ApplyLinkWrench`` system. One-shot (never
  persistent): each message is applied for exactly one physics step, so
  the impulse is conserved under transport jitter and a dead node leaves
  **no** force behind. The force is scaled by ``dt_sim / max_step_size``
  so a lower publish rate or an RTF dip is corrected rather than silently
  halving the force. Torques are about the link origin (the bridge drops
  the gz ``force_offset`` field).

If the running world carries no ``ApplyLinkWrench`` system (BlueBoat-
Control's stock ``ocean``, or a world folder built before 2026-09), the
node injects it at runtime through gz-sim's
``/world/<w>/entity/system/add`` service -- no neighbour package and no
saved world is modified. The system is detected by its own
``/world/<w>/wrench/persistent`` topic (:func:`wrench_system_loaded`),
never by ``/wrench``, which the launch's bridge advertises itself. If
injection fails, waves are reported ``unavailable`` and the current
still applies.

Topics
------
Pub  /current                    geometry_msgs/Vector3     rate_hz -- the
                                 water velocity AT THE BOAT (mean current +
                                 its smooth variation in time and space +
                                 wave orbital velocity)
Pub  /world/<w>/wrench           ros_gz_interfaces/EntityWrench (one-shot)
Pub  /world/<w>/wrench/clear     ros_gz_interfaces/Entity  startup/shutdown
Pub  /sim/sea_state              std_msgs/String (JSON)    2 Hz, TRANSIENT_LOCAL
Pub  /sim/sea_state/markers      visualization_msgs/MarkerArray (RViz)
Sub  <pose_topic>                geometry_msgs/TransformStamped (default
                                 /sim/sea_state/boat_pose: the 100 Hz Gazebo
                                 model pose, stamped, bridged by
                                 sea_state_launch.py): the pose the wrench uses
Sub  <true_odom_topic>           nav_msgs/Odometry (default /blueboat/odom_gz,
                                 2026-11-15, R-7): Gazebo's own odometry, the
                                 preferred fallback pose while it is fresh
Sub  <odom_topic>                nav_msgs/Odometry (default /blueboat/odom):
                                 the last fallback (field-rate and delayed in a
                                 field-equivalent run; the only one in a run
                                 without odom_gz) + the boat-motion readback
Sub  <robot_description_topic>   std_msgs/String, latched (default
                                 /blueboat/robot_description): the plant Gazebo
                                 integrates -- its vertical added mass, damping
                                 and inertia size the hull damping (R-8), and
                                 the legacy plant floats sea_states_legacy.yaml
Sub  /sim/sea_state/command      std_msgs/String (JSON): a state
                                 ``{"current": {"preset": "strong",
                                 "from": "E"}, "waves": {...},
                                 "ramp_s": 20}`` or a whole timeline
                                 ``{"schedule": {...}, "t0": "now"}``

The Gazebo GUI indicator (arrow + text markers over the world origin) is
sent through ``gz service -s /marker_array`` from a daemon thread (the
gz CLI; the start hold uses the gz-transport bindings when importable);
it is disabled silently when the GUI (and hence the marker service) is
absent.

Parameters
----------
current, current_from_deg, waves, waves_from_deg   preset names (from
    ``config/sea_states.yaml``) and compass FROM directions
schedule_file    timeline YAML (overrides the four above)
seed             wave phases + current fluctuations (0 = from the schedule)
rate_hz          publish/physics rate (100 = 1 / max_step_size)
max_step_size    Gazebo physics step [s] (read from world.sdf if world_dir)
world_name       gz world name ("" = discover; then the node runs its own
                 bridge for the wrench topics)
world_dir        world folder: mean depth for the dispersion relation and
                 max_step_size from its world.sdf (optional)
depth_m          fallback mean water depth
target_entity / target_entity_type   gz entity of the hull (model
                 "blueboat" -> its canonical link, base_link)
presets_file     alternative presets YAML ("" = by plant: sea_states.yaml for the
                 identified plant, sea_states_legacy.yaml for the legacy one)
plant            "" = from the robot description; or identified | bluerov2_legacy
description_timeout_s   wait for the description before arming [s] (default 15)
kappa            R4 bracket: commanded->applied thrust factor to emulate; the
                 horizontal sea forces are scaled 0.493 / kappa (0 = off)
run_context      JSON string the launch fills (sim_config, plant, tuning file
                 and schedule hashes...); republished in the status as "run"
release_world_on_arm   (2026-11-16) the launch started Gazebo PAUSED: unpause it once
                 the hull is armed (core/sim_hold.StartHold), or on any bootstrap
                 failure, or after release_deadline_s [s] of wall clock (default 60)
enable_waves, enable_gz_markers, marker_arrow_axis, ramp_s, odom_topic,
true_odom_topic, pose_topic (100 Hz stamped pose; "" = odometry only)
"""

from __future__ import annotations

import collections
import json
import queue
import re
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Entity, EntityWrench
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from ..core import run_context
from ..core.sim_hold import GzWorldControl, StartHold
from ..core.geometry import quat_to_rpy
from ..core.types import Pose3D
from ..sea import (HighRatePose, SeaModel, SeaSchedule, load_presets,
                   predict_pose,
                   resolve_state)
from ..sea.hull import rotation_zyx
from ..sea.plant import PLANTS, parse_urdf, presets_file_for
from ..sea.presets import find_default_presets_yaml
from ..sea.markers import (gz_delete_all_request, gz_marker_request,
                           rviz_markers)

GZ = shutil.which("gz") or "gz"
STATUS_TOPIC = "/sim/sea_state"
COMMAND_TOPIC = "/sim/sea_state/command"
MARKER_TOPIC = "/sim/sea_state/markers"
WRENCH_PLUGIN = ("gz-sim-apply-link-wrench-system",
                 "gz::sim::systems::ApplyLinkWrench")
#: Odometry window behind the readback's boat-motion statistics [s].
MOTION_WINDOW_S = 30.0


# --------------------------------------------------------------- helpers
def read_max_step_size(world_sdf: Path, default: float = 0.01) -> float:
    try:
        m = re.search(r"<max_step_size>\s*([0-9.eE+-]+)\s*</max_step_size>",
                      world_sdf.read_text(encoding="utf-8"))
        return float(m.group(1)) if m else default
    except OSError:
        return default


def read_mean_depth(world_dir: Path, default: float = 5.0) -> float:
    """Mean water depth of a world folder from its terrain-only
    ``depth.npz`` (heights are negative below the surface)."""
    try:
        d = np.load(world_dir / "depth.npz")
        h = d["height"] if "height" in d.files else d[d.files[0]]
        return float(max(-np.nanmean(h), 0.3))
    except Exception:
        return default


class _GzCliWorldControl:
    """World pause / resume through the gz CLI (when the gz-transport python bindings are absent)."""

    def __init__(self, world: str) -> None:
        self.world = world

    def set_paused(self, paused: bool) -> bool:
        code, out = _run_gz(["service", "-s", f"/world/{self.world}/control", "--reqtype", "gz.msgs.WorldControl",
                             "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                             "--req", f"pause: {'true' if paused else 'false'}"], timeout=6.0)
        return code == 0 and "data: true" in out


def _run_gz(args: list[str], timeout: float = 8.0) -> tuple[int, str]:
    try:
        r = subprocess.run([GZ, *args], capture_output=True, text=True,
                           timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except (OSError, subprocess.TimeoutExpired) as exc:
        return -1, str(exc)


def gz_topics() -> list[str]:
    code, out = _run_gz(["topic", "-l"])
    return out.split() if code == 0 else []


def discover_world_names(topics: list[str]) -> list[str]:
    return sorted({m.group(1) for m in
                   (re.match(r"^/world/([^/]+)/stats$", t) for t in topics) if m})


def wrench_system_loaded(topics: list[str], world: str) -> bool:
    """Whether the world runs gz-sim's ``ApplyLinkWrench`` system.

    Judged by ``/world/<w>/wrench/persistent``, which only that system
    creates. ``/world/<w>/wrench`` itself proves nothing: our own ROS->gz
    bridge advertises it (sea_state_launch starts the bridge before this
    node looks), so testing it skipped the injection into the stock
    ``ocean`` world and every wrench went to a topic nothing applied --
    the hull rode only the wave orbital drag (found live 2026-10-09)."""
    return f"/world/{world}/wrench/persistent" in topics


class SeaStateNode(Node):
    def __init__(self) -> None:
        super().__init__("sea_state")
        dp = self.declare_parameter
        dp("current", "none")
        dp("current_from_deg", 0.0)
        dp("waves", "calm")
        dp("waves_from_deg", 0.0)
        dp("schedule_file", "")
        dp("seed", 0)
        dp("rate_hz", 100.0)
        dp("max_step_size", 0.01)
        dp("world_name", "")
        dp("world_dir", "")
        dp("depth_m", 5.0)
        dp("target_entity", "")
        dp("target_entity_type", int(Entity.MODEL))
        dp("presets_file", "")
        dp("enable_waves", True)
        dp("enable_gz_markers", True)
        dp("marker_arrow_axis", "z")
        dp("ramp_s", 20.0)
        dp("odom_topic", "/blueboat/odom")
        dp("true_odom_topic", "/blueboat/odom_gz")
        dp("pose_topic", "/sim/sea_state/boat_pose")
        dp("robot_description_topic", "/blueboat/robot_description")
        dp("description_timeout_s", 15.0)
        dp("plant", "")
        dp("kappa", 0.0)
        dp("run_context", "")
        dp("release_world_on_arm", False)
        dp("release_deadline_s", 60.0)
        p = lambda k: self.get_parameter(k).value  # noqa: E731

        # ---- Model --------------------------------------------------------
        world_dir = Path(str(p("world_dir"))) if p("world_dir") else None
        depth = float(p("depth_m"))
        self._max_step = float(p("max_step_size"))
        if world_dir is not None and world_dir.exists():
            depth = read_mean_depth(world_dir, depth)
            self._max_step = read_max_step_size(world_dir / "world.sdf",
                                                self._max_step)
        self._depth = depth
        self._run_context = run_context.decode(str(p("run_context")))
        # The plant: an explicit parameter wins; else the robot description
        # decides (rebuilt in _on_description); until then the identified one.
        self._plant_param = str(p("plant"))
        if self._plant_param and self._plant_param not in PLANTS:
            raise ValueError(f"plant:={self._plant_param}: expected one of {sorted(PLANTS)}")
        self._plant = PLANTS[self._plant_param or "identified"]
        self._plant_ready = threading.Event()
        self._build_model(self._plant)
        schedule = self._model.schedule
        seed = self._model.seed
        self._entity_name = (str(p("target_entity"))
                             or str(self._presets.hull.get("target_entity", "blueboat")))
        self._entity_type = int(p("target_entity_type"))
        self._enable_waves = bool(p("enable_waves"))

        # ---- ROS I/O ------------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._current_pub = self.create_publisher(Vector3, "/current", 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, latched)
        self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, latched)
        self._wrench_pub = None
        self._clear_pub = None
        self._pose: Optional[Pose3D] = None
        self._pose_t = 0.0
        # Body-frame twist of the last odometry (Gazebo's OdometryPublisher
        # reports it in the child frame) and the age of the pose the last
        # tick actually used -- see sea/motion.py for why prediction matters.
        self._twist_body = (np.zeros(3), np.zeros(3))
        self._pose_age_ms = 0.0
        # What the sea does to the boat, for the readback: the last
        # MOTION_WINDOW_S of odometry (t, z, roll, pitch).
        self._motion = collections.deque()
        # R-7 (2026-11-15): Gazebo's own odometry (odom_gz) is preferred while
        # fresh; /blueboat/odom -- field-rate and delayed in a field-equivalent
        # run -- is the last fallback (the only pose of a bag replay).
        self._true_pose = None
        self._true_pose_t = -1e9
        self._true_twist = (np.zeros(3), np.zeros(3))
        self.create_subscription(Odometry, str(p("odom_topic")), self._on_odom, 10)
        if str(p("true_odom_topic")):
            self.create_subscription(Odometry, str(p("true_odom_topic")),
                                     self._on_true_odom, 10)
        if self._plant_param:
            self._plant_ready.set()
        elif str(p("robot_description_topic")):
            self.create_subscription(String, str(p("robot_description_topic")),
                                     self._on_description, latched)
        else:
            self._plant_ready.set()
        # The 100 Hz Gazebo pose (2026-10-09): the catamaran's stiff
        # restoring must not act on the 20 Hz odometry (it rang the roll
        # mode at 3.2 Hz) -- sea/motion.py::HighRatePose.
        self._hr = HighRatePose()
        self._pose_source = "odom"
        if str(p("pose_topic")):
            self.create_subscription(TransformStamped, str(p("pose_topic")),
                                     self._on_boat_pose, 50)
        self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)

        # ---- Gazebo-side state (set by the bootstrap thread) ---------------
        self._lock = threading.Lock()
        self._world = str(p("world_name"))
        self._wrench_armed = threading.Event()
        # The launch started Gazebo paused (2026-11-16): keep it so until the hull is armed (StartHold).
        self._start_hold: Optional[StartHold] = (
            StartHold(self._world_control, deadline_s=float(p("release_deadline_s")),
                      log=lambda m: self.get_logger().info(f"sea state: start hold: {m}"))
            if bool(p("release_world_on_arm")) else None)
        self._waves_reason = "starting: waiting for Gazebo"
        self._own_bridge: Optional[subprocess.Popen] = None
        self._gz_markers_ok = False
        self._marker_q: "queue.Queue[str]" = queue.Queue(maxsize=1)
        self._last_status: Optional[dict] = None

        # ---- Timers (sim time) ----------------------------------------------
        self._t_prev: Optional[float] = None
        self._wrench_count = 0
        self.create_timer(1.0 / max(float(p("rate_hz")), 1.0), self._tick)
        self.create_timer(0.5, self._publish_status)
        self._marker_period = 2.0
        self._marker_last = -1e9

        threading.Thread(target=self._bootstrap, name="sea-bootstrap",
                         daemon=True).start()
        if bool(p("enable_gz_markers")):
            threading.Thread(target=self._marker_worker, name="sea-gz-markers",
                             daemon=True).start()
        self.get_logger().info(
            f"sea state: {SeaModel.describe(schedule.state_at(0.0))}"
            + (f"; schedule '{schedule.name}' with {len(schedule.keyframes)} "
               f"keyframes" if len(schedule.keyframes) > 1 else "")
            + f"; seed {seed}; depth {depth:.1f} m; physics step "
            f"{self._max_step * 1e3:.0f} ms")

    # ------------------------------------------------------------- model
    def _build_model(self, plant) -> None:
        """(Re)build the sea model for ``plant``: its presets file (unless one is
        given), the schedule, and the hull damping sized on it."""
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        pf = str(p("presets_file"))
        if not pf:
            pf = str(presets_file_for(plant.name, find_default_presets_yaml().parent))
        self._presets = load_presets(pf)
        sched_file = str(p("schedule_file"))
        if sched_file:
            schedule = SeaSchedule.from_yaml(self._presets, sched_file)
        else:
            schedule = SeaSchedule.constant(resolve_state(
                self._presets, str(p("current")), float(p("current_from_deg")),
                str(p("waves")), float(p("waves_from_deg"))))
        seed = int(p("seed")) or int(schedule.seed) or 1
        old = getattr(self, "_model", None)
        self._model = SeaModel(self._presets, schedule, depth_m=self._depth,
                               seed=seed, ramp_s=float(p("ramp_s")), plant=plant,
                               kappa=float(p("kappa")))
        if old is not None:
            self._model.waves_available = old.waves_available
        self._plant = plant

    def _on_description(self, msg: String) -> None:
        if self._plant_ready.is_set():
            return
        try:
            plant = parse_urdf(msg.data)
        except Exception as exc:  # noqa: BLE001 -- keep the identified default
            self.get_logger().warning(f"sea state: robot description not usable ({exc}); "
                                      f"keeping plant {self._plant.name}")
            self._plant_ready.set()
            return
        self._build_model(plant)
        d = self._model.damping
        self.get_logger().info(
            f"sea state: plant {plant.name} from the robot description (link inertia "
            f"{tuple(round(v, 3) for v in plant.inertia_link)}, plugin added mass "
            f"{plant.added}, linear {plant.linear}); presets {Path(self._presets.path).name}; "
            f"hull damping {d.heave_ns_per_m:.1f} N·s/m, {d.roll_nms_per_rad:.2f} / "
            f"{d.pitch_nms_per_rad:.2f} N·m·s/rad")
        self._plant_ready.set()

    # ------------------------------------------------------------ inputs
    def _on_true_odom(self, msg: Odometry) -> None:
        pp, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._true_pose = Pose3D(pp.x, pp.y, pp.z, roll, pitch, yaw)
        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        self._true_twist = (np.array([lin.x, lin.y, lin.z]), np.array([ang.x, ang.y, ang.z]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._true_pose_t = stamp if stamp > 0.0 else self._now()
        self._motion.append((self._true_pose_t, pp.z, roll, pitch))
        while self._motion and self._motion[0][0] < self._true_pose_t - MOTION_WINDOW_S:
            self._motion.popleft()

    def _on_odom(self, msg: Odometry) -> None:
        pp, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._pose = Pose3D(pp.x, pp.y, pp.z, roll, pitch, yaw)
        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        self._twist_body = (np.array([lin.x, lin.y, lin.z]),
                            np.array([ang.x, ang.y, ang.z]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # The message's own (sim-time) stamp is what the prediction
        # measures against; an unstamped message counts from arrival.
        self._pose_t = stamp if stamp > 0.0 else self._now()
        if self._now() - self._true_pose_t < 1.0:
            return                          # the motion readback follows odom_gz
        self._motion.append((self._pose_t, pp.z, roll, pitch))
        while self._motion and self._motion[0][0] < self._pose_t - MOTION_WINDOW_S:
            self._motion.popleft()

    def _boat_motion(self) -> dict:
        """Roll / pitch / heave standard deviations of the boat over the
        last MOTION_WINDOW_S of odometry (additive readback, 2026-10-08):
        what the operator sees the waves actually doing to the hull."""
        if len(self._motion) < 20:
            return {"samples": len(self._motion), "window_s": 0.0}
        a = np.array(self._motion)
        return {"samples": int(a.shape[0]),
                "window_s": round(float(a[-1, 0] - a[0, 0]), 1),
                "heave_std_m": round(float(a[:, 1].std()), 4),
                "roll_std_deg": round(float(np.degrees(a[:, 2].std())), 2),
                "pitch_std_deg": round(float(np.degrees(a[:, 3].std())), 2),
                "roll_max_deg": round(float(np.degrees(np.abs(a[:, 2]).max())), 2),
                "pitch_max_deg": round(float(np.degrees(np.abs(a[:, 3]).max())), 2)}

    def _on_command(self, msg: String) -> None:
        try:
            what = self._model.apply_command(msg.data, self._now())
        except Exception as exc:  # noqa: BLE001 -- operator input
            self.get_logger().error(f"sea_state command rejected: {exc}")
            return
        self.get_logger().info(f"sea state command applied: {what}")
        self._publish_status(force=True)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------------- tick
    def _on_boat_pose(self, msg: TransformStamped) -> None:
        """The model pose of every physics step, with Gazebo's own sim-time
        stamp (exact differencing); only the hull's (child frame = the
        target entity), never a nested model's."""
        child = msg.child_frame_id.split("/")[-1]
        if child and child != self._entity_name:
            return
        tr, q = msg.transform.translation, msg.transform.rotation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._hr.update(st if st > 0.0 else self._now(),
                        Pose3D(tr.x, tr.y, tr.z, roll, pitch, yaw))

    def _tick(self) -> None:
        if not rclpy.ok():
            return                      # shutting down: publish nothing more
        t = self._now()
        dt = 0.0 if self._t_prev is None else t - self._t_prev
        self._t_prev = t
        hr = self._hr.state()
        if hr is not None and t - hr[3] < 0.1:
            # 100 Hz pose, at most a step old, its own differenced rates.
            pose, v_body, w_body, t_hr = hr
            age = max(t - t_hr, 0.0)
            self._pose_age_ms = 1000.0 * age
            self._pose_source = "pose_100hz"
            pose = predict_pose(pose, v_body, w_body, age)
            v_world = rotation_zyx(pose.roll, pose.pitch, pose.yaw) @ v_body
            sample = self._model.update(t, pose, (v_world, w_body))
            self._after_update(sample, dt)
            return
        if self._true_pose is not None and (t - self._true_pose_t) < 1.0:
            self._pose_source = "odom_gz"
            pose, pose_t, (v_body, w_body) = self._true_pose, self._true_pose_t, self._true_twist
        else:
            self._pose_source = "odom"
            pose = self._pose if (t - self._pose_t) < 1.0 else None
            pose_t = self._pose_t
            v_body, w_body = self._twist_body
        twist = None
        if pose is not None:
            # Propagate the 20 Hz odometry to this tick with its own twist:
            # the restoring part of the wrench must see the boat where it
            # is now, not up to 50 ms ago (a delayed spring rings).
            age = t - pose_t
            self._pose_age_ms = 1000.0 * age
            pose = predict_pose(pose, v_body, w_body, age)
            v_world = rotation_zyx(pose.roll, pose.pitch, pose.yaw) @ v_body
            twist = (v_world, w_body)
        sample = self._model.update(t, pose, twist)
        self._after_update(sample, dt)

    def _after_update(self, sample, dt: float) -> None:
        v = Vector3()
        v.x, v.y, v.z = (float(x) for x in sample.water_velocity)
        self._current_pub.publish(v)

        if (sample.wrench_active and self._enable_waves
                and self._wrench_armed.is_set() and dt > 0.0
                and (np.any(sample.force) or np.any(sample.torque))):
            k = float(np.clip(dt / self._max_step, 0.0, 5.0))
            msg = EntityWrench()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.entity.name = self._entity_name
            msg.entity.type = self._entity_type
            f, tq = sample.force * k, sample.torque * k
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (
                float(f[0]), float(f[1]), float(f[2]))
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = (
                float(tq[0]), float(tq[1]), float(tq[2]))
            self._wrench_pub.publish(msg)
            self._wrench_count += 1
        self._last_sample = sample

    # ------------------------------------------------------------ status
    def _waves_status(self, sample) -> str:
        if sample.waves_active:
            return "active"
        if not self._model.waves_available or not self._enable_waves:
            return "unavailable"
        return "calm" if sample.state.waves.hs_m <= 0.0 else "idle"

    def _publish_status(self, force: bool = False) -> None:
        sample = getattr(self, "_last_sample", None)
        if sample is None or not rclpy.ok():
            return
        reason = ("waves disabled by parameter" if not self._enable_waves
                  else self._waves_reason)
        d = self._model.status_dict(sample, self._world, self._entity_name, reason)
        d["wrench_messages"] = self._wrench_count
        d["pose_age_ms"] = round(float(self._pose_age_ms), 1)   # additive
        d["pose_source"] = self._pose_source                    # additive
        d["boat"] = self._boat_motion()                         # additive
        d["run"] = self._run_context                            # additive (2026-11-15, C3)
        d["presets_file"] = Path(self._presets.path).name       # additive (2026-11-15)
        if self._start_hold is not None:                         # additive (2026-11-16)
            d["start_hold"] = {"released_by": self._start_hold.released_by, "ok": self._start_hold.ok}
        d["waves"]["status"] = self._waves_status(sample)
        self._status_pub.publish(String(data=json.dumps(d)))
        # RViz markers ride at the boat.
        boat = (self._true_pose if self._now() - self._true_pose_t < 1.0 else self._pose)
        xy = (boat.x, boat.y) if boat is not None else (0.0, 0.0)
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for m in rviz_markers(sample.state, xy, d["waves"]["status"]):
            mk = Marker()
            mk.header.frame_id = m["frame_id"]
            mk.header.stamp = stamp
            mk.ns, mk.id, mk.type, mk.action = m["ns"], m["id"], m["type"], m["action"]
            x, y, z, qx, qy, qz, qw = m["pose"]
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = x, y, z
            (mk.pose.orientation.x, mk.pose.orientation.y,
             mk.pose.orientation.z, mk.pose.orientation.w) = qx, qy, qz, qw
            mk.scale.x, mk.scale.y, mk.scale.z = (float(s) for s in m["scale"])
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = (float(c) for c in m["rgba"])
            mk.text = m["text"]
            arr.markers.append(mk)
        self._marker_pub.publish(arr)
        # Gazebo GUI markers: on change + heartbeat, never on this thread.
        changed = self._model.changed_flag or force
        wall = time.monotonic()
        if self._gz_markers_ok and (changed or wall - self._marker_last > self._marker_period):
            self._model.changed_flag = False
            self._marker_last = wall
            req = gz_marker_request(sample.state, (0.0, 0.0), d["waves"]["status"],
                                    arrow_axis=str(self.get_parameter(
                                        "marker_arrow_axis").value))
            try:
                self._marker_q.get_nowait()
            except queue.Empty:
                pass
            self._marker_q.put_nowait(req)
        self._last_status = d

    # ---------------------------------------------------------- bootstrap
    def _bootstrap(self) -> None:
        """World discovery, ApplyLinkWrench detection/injection, bridge and
        wrench arming -- all subprocess work, off the ROS thread."""
        log = self.get_logger()
        topics: list[str] = []
        deadline = time.monotonic() + 300.0
        while rclpy.ok() and time.monotonic() < deadline:
            topics = gz_topics()
            names = discover_world_names(topics)
            if self._world and (f"/world/{self._world}/stats" in topics):
                break
            if not self._world and len(names) == 1:
                self._world = names[0]
                break
            if not self._world and len(names) > 1:
                log.error(f"several Gazebo worlds running {names}; set the "
                          "world_name parameter")
                self._waves_reason = "ambiguous world name"
                self._release_start("bootstrap failed: ambiguous world name")
                return
            time.sleep(1.0)
        if not self._world or f"/world/{self._world}/stats" not in topics:
            self._waves_reason = "Gazebo not found (no /world/<name>/stats topic)"
            log.warning(f"sea state: {self._waves_reason}; current only")
            self._release_start("bootstrap failed: Gazebo not found")
            return
        w = self._world
        self._gz_markers_ok = self._probe_marker_service()
        # Wrench topics need the ApplyLinkWrench system on the world.
        if not wrench_system_loaded(topics, w):
            log.info(f"world '{w}' has no ApplyLinkWrench system -- injecting it "
                     f"at runtime (/world/{w}/entity/system/add)")
            if not self._inject_wrench_plugin(w):
                self._release_start("bootstrap failed: ApplyLinkWrench injection")
                return
        if self.get_parameter("world_name").value == "":
            self._start_own_bridge(w)
        # The plant must be known before the hull wrench acts (its damping is
        # sized on it): wait for the robot description, bounded.
        tmo = float(self.get_parameter("description_timeout_s").value)
        if not self._plant_ready.wait(timeout=max(tmo, 0.0)):
            log.warning(f"sea state: no robot description within {tmo:.0f} s; "
                        f"arming on plant {self._plant.name} (mirror values)")
            self._plant_ready.set()
        self._wrench_pub = self.create_publisher(EntityWrench, f"/world/{w}/wrench", 10)
        self._clear_pub = self.create_publisher(Entity, f"/world/{w}/wrench/clear", 10)
        time.sleep(0.5)                      # let the bridge match
        self._clear_pub.publish(self._entity_msg())
        self._waves_reason = ""
        self._wrench_armed.set()
        log.info(f"sea state armed on world '{w}' (entity '{self._entity_name}', "
                 f"markers {'on' if self._gz_markers_ok else 'off'})")
        self._model.changed_flag = True
        self._release_start("armed")

    def _release_start(self, reason: str) -> None:
        """Unpause the world the launch started paused (no-op without the hold)."""
        if self._start_hold is not None:
            self._start_hold.release(reason)

    def _world_control(self):
        """Pause / resume control of the running world: gz-transport bindings, or the gz CLI."""
        w = self._world or str(self.get_parameter("world_name").value)
        if not w:
            raise RuntimeError("world name unknown")
        try:
            return GzWorldControl(w, timeout_ms=2000)
        except ImportError:
            return _GzCliWorldControl(w)

    def _entity_msg(self) -> Entity:
        e = Entity()
        e.name = self._entity_name
        e.type = self._entity_type
        return e

    def _inject_wrench_plugin(self, w: str) -> bool:
        req = (f'entity {{id: 1, name: "{w}", type: WORLD}} '
               f'plugins {{name: "{WRENCH_PLUGIN[1]}", filename: "{WRENCH_PLUGIN[0]}"}}')
        code, out = _run_gz(["service", "-s", f"/world/{w}/entity/system/add",
                             "--reqtype", "gz.msgs.EntityPlugin_V",
                             "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                             "--req", req], timeout=12.0)
        for _ in range(12):
            if wrench_system_loaded(gz_topics(), w):
                self.get_logger().info("ApplyLinkWrench injected")
                return True
            time.sleep(0.25)
        self._waves_reason = (f"ApplyLinkWrench injection failed (rc {code}: "
                              f"{out.strip()[:120]}); add the plugin to the "
                              "world SDF")
        self._model.waves_available = False
        self.get_logger().warning(f"sea state: {self._waves_reason}; current only")
        return False

    def _start_own_bridge(self, w: str) -> None:
        """No launch-provided bridge (world_name was discovered): run one."""
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
               f"/world/{w}/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
               f"/world/{w}/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity"]
        try:
            self._own_bridge = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info("started an own ros_gz bridge for the wrench topics")
        except OSError as exc:
            self.get_logger().error(f"could not start ros_gz_bridge: {exc}")

    def _probe_marker_service(self) -> bool:
        code, out = _run_gz(["service", "-l"])
        ok = code == 0 and "/marker_array" in out.split()
        if not ok:
            self.get_logger().info("no /marker_array service (headless Gazebo?) "
                                   "-- GUI indicator off, RViz markers still published")
        return ok

    # ------------------------------------------------------------ markers
    def _marker_worker(self) -> None:
        while rclpy.ok():
            try:
                req = self._marker_q.get(timeout=1.0)
            except queue.Empty:
                continue
            _run_gz(["service", "-s", "/marker_array", "--reqtype",
                     "gz.msgs.Marker_V", "--reptype", "gz.msgs.Boolean",
                     "--timeout", "3000", "--req", req], timeout=6.0)

    # ----------------------------------------------------------- shutdown
    def shutdown(self) -> None:
        try:
            if self._clear_pub is not None:
                self._clear_pub.publish(self._entity_msg())
            self._current_pub.publish(Vector3())
            if self._gz_markers_ok:
                _run_gz(["service", "-s", "/marker_array", "--reqtype",
                         "gz.msgs.Marker_V", "--reptype", "gz.msgs.Boolean",
                         "--timeout", "2000", "--req", gz_delete_all_request()],
                        timeout=4.0)
            if self._own_bridge is not None:
                self._own_bridge.terminate()
        except Exception:  # noqa: BLE001 -- best effort on the way out
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeaStateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl-C, or a launch teardown (rclpy's own SIGINT handler)
    except Exception:
        # A tick already past its rclpy.ok() test when the SIGINT handler
        # invalidated the context fails its publish (RCLError) -- a
        # teardown, not a fault. Anything raised while the context is still
        # valid is real.
        if rclpy.ok():
            raise
    finally:
        try:
            node.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            # A process-group SIGINT (MCS and the Gazebo gate stop launches
            # with killpg) reaches this node twice -- directly and forwarded
            # by ros2 launch -- and the second copy lands here. The wrench is
            # one-shot, so nothing is left applied either way.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
