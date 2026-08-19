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
Sub  ~/ping/enable          std_msgs/Bool   true=start, false=stop
Sub  <odom_topic>           nav_msgs/Odometry (default /blueboat/odom)

Run-dependent parameters (identical names/semantics to the real node)
----------------------------------------------------------------------
range_start_mm, range_length_mm, msec_per_ping, gain_index, num_results,
pulse_len_percent -- re-read each time pinging is enabled.

Simulation-only parameters
--------------------------
scene_dir (required), sonar_config (YAML with the ``model:`` section),
odom_topic, publish_ground_truth, seed.

The extra ``~/ground_truth/contacts`` topic is additive: downstream
consumers of the real interface never see it unless they subscribe.
"""

from __future__ import annotations

import json
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8MultiArray

from blueboat_interfaces.msg import OmniscanProfile

from ..core.geometry import quat_to_rpy
from ..core.types import Pose3D, Side
from ..sonar.config import AcquisitionParams, SonarConfig, SonarModelConfig
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

        # ---- Run-dependent parameters (identical to the real node) --------
        self.declare_parameter("range_start_mm", 0)
        self.declare_parameter("range_length_mm", 15000)
        self.declare_parameter("msec_per_ping", 0)
        self.declare_parameter("gain_index", 4)
        self.declare_parameter("num_results", 600)
        self.declare_parameter("pulse_len_percent", 0.002)

        # ---- Simulation-only parameters ------------------------------------
        self.declare_parameter("scene_dir", "")
        self.declare_parameter("sonar_config", "")
        self.declare_parameter("odom_topic", "/blueboat/odom")
        self.declare_parameter("publish_ground_truth", True)
        self.declare_parameter("seed", 0)

        scene_dir = self.get_parameter("scene_dir").value
        if not scene_dir:
            raise RuntimeError("parameter 'scene_dir' is required "
                               "(directory containing scene.npz + manifest)")
        self._scene = SceneModel.load(scene_dir)
        self.get_logger().info(
            f"scene loaded: {self._scene.grid.nx}x{self._scene.grid.ny} @ "
            f"{self._scene.grid.resolution} m, {len(self._scene.objects)} objects")

        sonar_cfg_path = self.get_parameter("sonar_config").value
        self._model_cfg: SonarModelConfig = (
            SonarConfig.from_yaml(sonar_cfg_path).model
            if sonar_cfg_path else SonarModelConfig())

        seed = int(self.get_parameter("seed").value)
        self._rng = np.random.default_rng(seed if seed else None)

        # ---- Publishers (best-effort: matches real node) --------------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._channels = {
            Side.PORT: _SideChannel(self, Side.PORT, qos, self._rng),
            Side.STARBOARD: _SideChannel(self, Side.STARBOARD, qos, self._rng),
        }
        self._gt_pub = (self.create_publisher(String, "~/ground_truth/contacts", 10)
                        if self.get_parameter("publish_ground_truth").value else None)

        # ---- Inputs ----------------------------------------------------------
        self._pose: Optional[Pose3D] = None
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
        self._renderer = GeometricRenderer(self._scene, acq, self._model_cfg)
        for ch in self._channels.values():
            ch.encoder = PingEncoder(ch.side, acq, self._model_cfg)
            ch.drift = GainDrift(self._model_cfg, self._rng)
        if self._t0 is None:
            self._t0 = self._now()

        period = acq.ping_period_s(self._model_cfg.max_ping_rate_hz)
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"pinging started (range {acq.range_start_mm}-"
            f"{acq.range_start_mm + acq.range_length_mm} mm, gain "
            f"{acq.gain_index}, n={acq.num_results}, period {period*1000:.0f} ms)")

    def _stop_pinging(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info("pinging stopped")

    # ------------------------------------------------------------------ tick
    def _tick(self) -> None:
        if self._pose is None or self._renderer is None or self._acq is None:
            return
        t_sim = self._now() - (self._t0 or 0.0)
        gt_payload = []
        for ch in self._channels.values():
            rendered = self._renderer.render(ch.side, self._pose, t_sim)
            if ping_dropped(self._model_cfg, ch.rng):
                # Device counter still advances on the real hardware.
                if ch.encoder is not None:
                    ch.encoder._ping_number += 1  # noqa: SLF001 deliberate
                continue
            wc = self._watercolumn_mask(rendered.ping.altitude_m)
            noisy = apply_ping_noise(
                rendered.ping.power, wc,
                ch.drift.value(t_sim) if ch.drift else 1.0,
                self._model_cfg, ch.rng,
                specular=rendered.ping.specular)
            rendered.ping.power = noisy
            enc = ch.encoder.encode(rendered.ping)  # type: ignore[union-attr]
            self._publish(ch, enc)
            if self._gt_pub is not None and rendered.contacts:
                gt_payload += [{
                    "side": c.side.value, "object_id": c.object_id,
                    "type": c.object_type,
                    "slant_range_m": round(c.slant_range_m, 3),
                    "extent_bins": round(c.extent_bins, 1),
                    "shadow_bins": round(c.shadow_bins, 1),
                    "visible": c.visible,
                    "ping_number": enc.ping_number,
                } for c in rendered.contacts]
        if self._gt_pub is not None and gt_payload:
            m = String()
            m.data = json.dumps({"t_sim": round(t_sim, 3),
                                 "contacts": gt_payload})
            self._gt_pub.publish(m)

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
