#!/usr/bin/env python3
"""Simulated Side Scan Sonar node -- drop-in replacement for ``sss_node.py``.

Publishes exactly the topics, message types, QoS and control semantics of
the real node (see that file's docstring), but renders pings against the
generated :class:`SceneModel` instead of driving Omniscan 450 hardware:

Topics
------
Pub  ~/port/profile         blueboat_interfaces/OmniscanProfile
Pub  ~/port/raw             std_msgs/UInt8MultiArray   (byte-valid Ping Protocol)
Pub  ~/starboard/profile    blueboat_interfaces/OmniscanProfile
Pub  ~/starboard/raw        std_msgs/UInt8MultiArray
Pub  ~/ground_truth/contacts  std_msgs/String (JSON)   [simulation extra]
Pub  ~/ground_truth/sensor    std_msgs/String (JSON)   [simulation extra]:
                            per ping cycle, the vehicle pose and both
                            transducer poses the ping was rendered from
Sub  ~/ping/enable          std_msgs/Bool   true=start, false=stop
Sub  <odom_topic>           nav_msgs/Odometry (default /blueboat/odom)

Run-dependent parameters (identical names/semantics to the real node)
----------------------------------------------------------------------
range_start_mm, range_length_mm, msec_per_ping, gain_index, num_results,
pulse_len_percent -- re-read each time pinging is enabled.

These ROS parameters are the node's acquisition interface, exactly as on the
real node -- names, semantics and (since 2026-09-28) defaults: 20 m, device
auto-gain, 1200 bins, pulse 0.00125, the device's best range resolution.
``sss_sim_launch.py`` still passes all six from the resolved sonar profile, so
a run's acquisition is traceable to a named file; a profile whose
``acquisition:`` block disagrees with the parameters actually in force is
reported as a warning at every ping enable.

``gain_index: -1`` is accepted with the real device's meaning (auto-gain) and
resolves to the calibrated index 4, which is what the profile then reports --
the device likewise resolves auto-gain internally and reports a concrete index,
since the ``gain_index`` field is ``uint16`` and cannot carry ``-1``.

Simulation-only parameters
--------------------------
scene_dir (required), sonar_config (YAML with the ``model:`` section),
odom_topic, publish_ground_truth, seed, predict_pose, pose_topic.

Platform motion (2026-10-08; 100 Hz pose 2026-10-09). Each ping is
rendered at the vehicle pose *at the ping time*: Gazebo's per-step model
pose (``pose_topic``, default ``/sim/sea_state/boat_pose``, bridged by
``sea_state_launch.py``; at most a physics step old), else the 20 Hz
odometry, propagated by its own body-frame rates with
``sea.motion.predict_pose`` -- the function the sea-state node uses for
the hull wrench -- so a rolling or pitching boat is not imaged with an
attitude up to 50 ms old (``predict_pose:=false`` restores the newest
sample as-is). The renderer then places each transducer by the full
attitude (``GeometricRenderer._sensor_pose``).

The extra ``~/ground_truth/*`` topics are additive: downstream consumers
of the real interface never see them unless they subscribe.
"""

from __future__ import annotations

import dataclasses
import json
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8MultiArray

from blueboat_interfaces.msg import OmniscanProfile

from ..core.geometry import quat_to_rpy
from ..sea.motion import HighRatePose, predict_pose
from ..core.types import Pose3D, Side
from ..sonar.config import (AUTO_GAIN_INDEX, AcquisitionParams, SonarConfig,
                            SonarModelConfig)
from ..sonar.encoder import PingEncoder
from ..sonar.noise import GainDrift, apply_ping_noise, ping_dropped
from ..sonar.renderer import GeometricRenderer, SonarRenderer
from ..worldgen.scene import SceneModel


class _SideChannel:
    """One transducer channel: renderer side + encoder + drift + publishers."""

    def __init__(self, node: "SideScanSonarSimNode", side: Side,
                 qos: QoSProfile, rng: np.random.Generator) -> None:
        self.side = side
        self.profile_pub = node.create_publisher(
            OmniscanProfile, f"~/{side.value}/profile", qos)
        self.raw_pub = node.create_publisher(
            UInt8MultiArray, f"~/{side.value}/raw", qos)
        self.encoder: Optional[PingEncoder] = None
        self.drift: Optional[GainDrift] = None
        self.rng = rng


class SideScanSonarSimNode(Node):
    """See module docstring."""

    def __init__(self) -> None:
        super().__init__("side_scan_sonar")

        # ---- Run-dependent parameters (identical to the real node, names
        # and defaults; the launches pass the resolved profile's values) ----
        self.declare_parameter("range_start_mm", 0)
        self.declare_parameter("range_length_mm", 20000)
        self.declare_parameter("msec_per_ping", 0)
        self.declare_parameter("gain_index", -1)
        self.declare_parameter("num_results", 1200)
        self.declare_parameter("pulse_len_percent", 0.00125)

        # ---- Simulation-only parameters ------------------------------------
        self.declare_parameter("scene_dir", "")
        self.declare_parameter("sonar_config", "")
        self.declare_parameter("odom_topic", "/blueboat/odom")
        self.declare_parameter("publish_ground_truth", True)
        self.declare_parameter("seed", 0)
        self.declare_parameter("predict_pose", True)
        self.declare_parameter("pose_topic", "/sim/sea_state/boat_pose")

        scene_dir = self.get_parameter("scene_dir").value
        if not scene_dir:
            raise RuntimeError("parameter 'scene_dir' is required "
                               "(directory containing scene.npz + manifest)")
        self._scene = SceneModel.load(scene_dir)
        self.get_logger().info(
            f"scene loaded: {self._scene.grid.nx}x{self._scene.grid.ny} @ "
            f"{self._scene.grid.resolution} m, {len(self._scene.objects)} objects")
        stale = self._scene.stale_rasters()
        if stale:
            # SceneModel.load says the same through Python logging, which
            # the ROS console does not show: repeat it where the operator
            # looks.
            self.get_logger().warning(
                f"world {scene_dir} predates the current object model (no "
                f"{', '.join(stale)} raster): its objects render as they did "
                "when it was built. Rebuild it (World Builder -> Edit World "
                "-> save) for the current model.")

        sonar_cfg_path = self.get_parameter("sonar_config").value
        self._sonar_cfg_path: str = sonar_cfg_path or ""
        self._file_acq: Optional[AcquisitionParams] = None
        if sonar_cfg_path:
            file_cfg = SonarConfig.from_yaml(sonar_cfg_path)
            self._model_cfg: SonarModelConfig = file_cfg.model
            self._file_acq = file_cfg.acquisition
        else:
            self._model_cfg = SonarModelConfig()

        seed = int(self.get_parameter("seed").value)
        self._rng = np.random.default_rng(seed if seed else None)

        # ---- Publishers (best-effort: matches real node) --------------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._channels = {
            Side.PORT: _SideChannel(self, Side.PORT, qos, self._rng),
            Side.STARBOARD: _SideChannel(self, Side.STARBOARD, qos, self._rng),
        }
        gt_on = bool(self.get_parameter("publish_ground_truth").value)
        self._gt_pub = (self.create_publisher(String, "~/ground_truth/contacts", 10)
                        if gt_on else None)
        self._sensor_pub = (self.create_publisher(String, "~/ground_truth/sensor", 10)
                            if gt_on else None)

        # ---- Inputs ----------------------------------------------------------
        self._pose: Optional[Pose3D] = None
        self._pose_t = 0.0                         # odometry stamp, sim time
        self._twist_body = (np.zeros(3), np.zeros(3))
        self._predict = bool(self.get_parameter("predict_pose").value)
        self._hr = HighRatePose()
        self._pose_source = "odom"
        if str(self.get_parameter("pose_topic").value):
            self.create_subscription(TransformStamped,
                                     str(self.get_parameter("pose_topic").value),
                                     self._on_boat_pose, 50)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._on_odom, 10)
        self.create_subscription(Bool, "~/ping/enable", self._on_ping_enable, 10)

        # ---- Ping engine state ------------------------------------------------
        self._renderer: Optional[SonarRenderer] = None
        self._acq: Optional[AcquisitionParams] = None
        self._timer = None
        self._t0: Optional[float] = None

        self.get_logger().info(
            "side_scan_sonar (SIMULATED) ready, ping OFF. Toggle with:\n"
            "  ros2 topic pub --once /side_scan_sonar/ping/enable "
            "std_msgs/msg/Bool 'data: true'")

    # ---------------------------------------------------------------- inputs
    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._pose = Pose3D(p.x, p.y, p.z, roll, pitch, yaw)
        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        # Gazebo's OdometryPublisher reports the twist in the child (body)
        # frame -- the convention predict_pose expects.
        self._twist_body = (np.array([lin.x, lin.y, lin.z]),
                            np.array([ang.x, ang.y, ang.z]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._pose_t = stamp if stamp > 0.0 else self._now()

    def _on_ping_enable(self, msg: Bool) -> None:
        if msg.data:
            self._start_pinging()
        else:
            self._stop_pinging()

    # ------------------------------------------------------------- lifecycle
    def _start_pinging(self) -> None:
        acq = AcquisitionParams(
            range_start_mm=self._int("range_start_mm"),
            range_length_mm=self._int("range_length_mm"),
            msec_per_ping=self._int("msec_per_ping"),
            gain_index=self._int("gain_index"),
            num_results=self._int("num_results"),
            pulse_len_percent=float(self.get_parameter("pulse_len_percent").value),
        )
        self._acq = acq
        self._warn_if_acquisition_diverges(acq)
        self._renderer = GeometricRenderer(self._scene, acq, self._model_cfg)
        for ch in self._channels.values():
            # One encoder per side for the node's whole life: the real
            # device numbers pings from power-up and a re-enable never
            # restarts its counter (a restarted counter puts duplicate ping
            # numbers into one .svlog, which the GCS keys rows on).
            if ch.encoder is None:
                ch.encoder = PingEncoder(ch.side, acq, self._model_cfg)
            else:
                ch.encoder.reconfigure(acq, self._model_cfg)
            ch.drift = GainDrift(self._model_cfg, self._rng)
        if self._t0 is None:
            self._t0 = self._now()

        period = acq.ping_period_s(self._model_cfg.max_ping_rate_hz)
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(period, self._tick)
        gain = (f"{acq.gain_index} (auto -> {acq.effective_gain_index})"
                if acq.gain_index == AUTO_GAIN_INDEX else str(acq.gain_index))
        self.get_logger().info(
            f"pinging started (range {acq.range_start_mm}-"
            f"{acq.range_start_mm + acq.range_length_mm} mm, gain "
            f"{gain}, n={acq.num_results}, period {period*1000:.0f} ms)")

    def _warn_if_acquisition_diverges(self, acq: AcquisitionParams) -> None:
        """Report a bundle that records different acquisition than is in force.

        The bundle is an immutable record of what a run acquired at (NC #10),
        so a mismatch means the ground truth and the pings describe different
        surveys. Warn rather than raise: overriding acquisition per run is
        legitimate device behaviour, and the operator is told which knobs moved.
        """
        if self._file_acq is None:
            return
        differing = [
            f"{f.name}: bundle {getattr(self._file_acq, f.name)} "
            f"!= in force {getattr(acq, f.name)}"
            for f in dataclasses.fields(AcquisitionParams)
            if getattr(self._file_acq, f.name) != getattr(acq, f.name)
        ]
        if differing:
            self.get_logger().warning(
                f"acquisition differs from the bundle's frozen sonar.yaml "
                f"({self._sonar_cfg_path}): " + "; ".join(differing) +
                ". The bundle records the survey it was generated for; "
                "regenerate it if this override is the intended acquisition.")

    def _stop_pinging(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info("pinging stopped")

    # ------------------------------------------------------------------ tick
    def _on_boat_pose(self, msg: TransformStamped) -> None:
        child = msg.child_frame_id.split("/")[-1]
        if child and child != "blueboat":
            return
        tr, q = msg.transform.translation, msg.transform.rotation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._hr.update(st if st > 0.0 else self._now(),
                        Pose3D(tr.x, tr.y, tr.z, roll, pitch, yaw))

    def _ping_pose(self, t_now: float) -> tuple[Pose3D, float]:
        """The vehicle pose at the ping time and the source's age [s]:
        the 100 Hz model pose when it is fresh, else the odometry."""
        hr = self._hr.state()
        if hr is not None and t_now - hr[3] < 0.1:
            pose, v_body, w_body, t_hr = hr
            age = t_now - t_hr
            self._pose_source = "pose_100hz"
            if not self._predict:
                return pose, age
            return predict_pose(pose, v_body, w_body, age), age
        self._pose_source = "odom"
        assert self._pose is not None
        age = t_now - self._pose_t
        if not self._predict:
            return self._pose, age
        v_body, w_body = self._twist_body
        return predict_pose(self._pose, v_body, w_body, age), age

    def _tick(self) -> None:
        if self._pose is None or self._renderer is None or self._acq is None:
            return
        t_now = self._now()
        t_sim = t_now - (self._t0 or 0.0)
        pose, age = self._ping_pose(t_now)
        gt_payload = []
        sensors: dict = {}
        for ch in self._channels.values():
            rendered = self._renderer.render(ch.side, pose, t_sim)
            sp = rendered.ping.pose
            sensors[ch.side.value] = {
                "pose": [round(float(v), 5) for v in
                         (sp.x, sp.y, sp.z, sp.roll, sp.pitch, sp.yaw)],
                "altitude_m": round(float(rendered.ping.altitude_m), 4),
                "ping_number": None, "dropped": False}
            if ping_dropped(self._model_cfg, ch.rng):
                # Device counter still advances on the real hardware.
                if ch.encoder is not None:
                    ch.encoder._ping_number += 1  # noqa: SLF001 deliberate
                sensors[ch.side.value]["dropped"] = True
                continue
            wc = self._watercolumn_mask(rendered.ping.altitude_m)
            noisy = apply_ping_noise(
                rendered.ping.power, wc,
                ch.drift.value(t_sim) if ch.drift else 1.0,
                self._model_cfg, ch.rng,
                specular=rendered.ping.specular,
                texture=rendered.ping.texture,
                unoccluded=rendered.ping.unoccluded,
                ambient=rendered.ping.ambient)
            rendered.ping.power = noisy
            enc = ch.encoder.encode(rendered.ping)  # type: ignore[union-attr]
            self._publish(ch, enc)
            sensors[ch.side.value]["ping_number"] = int(enc.ping_number)
            if self._gt_pub is not None and rendered.contacts:
                gt_payload += [{
                    "side": c.side.value,
                    # A wall face's own echo is labelled by the wall's name;
                    # scene objects keep their integer id.
                    "object_id": (f"wall:{c.wall_name}"
                                  if c.object_type == "wall" else c.object_id),
                    "type": c.object_type,
                    "slant_range_m": round(c.slant_range_m, 3),
                    "extent_bins": round(c.extent_bins, 1),
                    "shadow_bins": round(c.shadow_bins, 1),
                    "visible": c.visible,
                    "ping_number": enc.ping_number,
                    # Additive: a multipath image of the object named by
                    # object_id, and the reflector that produced it. Absent
                    # keys mean "direct path" for a consumer reading an
                    # older stream, so nothing downstream has to change.
                    "ghost": c.ghost,
                    "via": c.via,
                } for c in rendered.contacts]
        if self._gt_pub is not None and gt_payload:
            m = String()
            m.data = json.dumps({"t_sim": round(t_sim, 3),
                                 "contacts": gt_payload})
            self._gt_pub.publish(m)
        if self._sensor_pub is not None:
            # Every cycle, contacts or not: what the validation tooling
            # (analysis/sea_validation.py) reads to know exactly where each
            # ping was taken from. Pose = [x, y, z, roll, pitch, yaw], world
            # ENU, metres / radians.
            m = String()
            m.data = json.dumps({
                "t_sim": round(t_sim, 4), "stamp": round(t_now, 4),
                "pose_age_ms": round(1000.0 * age, 1), "predicted": self._predict,
                "pose_source": self._pose_source,
                "boat": [round(float(v), 5) for v in
                         (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)],
                "sensors": sensors})
            self._sensor_pub.publish(m)

    def _publish(self, ch: _SideChannel, enc) -> None:
        msg = OmniscanProfile()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = enc.frame_id
        msg.side = ch.side.value
        msg.ping_number = enc.ping_number
        msg.start_mm = enc.start_mm
        msg.length_mm = enc.length_mm
        msg.timestamp_ms = enc.timestamp_ms
        msg.ping_hz = enc.ping_hz
        msg.gain_index = enc.gain_index
        msg.num_results = enc.num_results
        msg.sos_dmps = enc.sos_dmps
        msg.channel_number = enc.channel_number
        msg.pulse_duration_sec = enc.pulse_duration_sec
        msg.analog_gain = enc.analog_gain
        msg.max_pwr_db = enc.max_pwr_db
        msg.min_pwr_db = enc.min_pwr_db
        msg.transducer_heading_deg = enc.transducer_heading_deg
        msg.vehicle_heading_deg = enc.vehicle_heading_deg
        msg.pwr_results = [int(v) for v in enc.pwr_results]
        ch.profile_pub.publish(msg)

        raw = UInt8MultiArray()
        raw.data = list(enc.raw_frame)
        ch.raw_pub.publish(raw)

    # --------------------------------------------------------------- helpers
    def _watercolumn_mask(self, altitude_m: float) -> np.ndarray:
        assert self._acq is not None
        r = (self._acq.range_start_mm / 1000.0
             + (np.arange(self._acq.num_results) + 0.5) * self._acq.bin_size_m)
        return r < altitude_m

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SideScanSonarSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl-C, or a launch teardown (rclpy's own SIGINT handler)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
