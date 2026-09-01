#!/usr/bin/env python3
"""Optional MAVROS shim for simulation.

Some downstream tooling written against the real boat reads navigation
context from MAVROS topics (visible in the field topic list:
``/mavros/global_position/compass_hdg``, ``/mavros/imu/data``,
``/mavros/local_position/pose``). In simulation MAVROS is absent; this
shim derives the same information from ``/blueboat/odom`` and republishes
it under the MAVROS names, so such tooling also runs unmodified.

It is strictly optional -- the sonar interface itself does not need it
(vehicle heading is embedded in every ``OmniscanProfile``).

Published topics
----------------
/mavros/global_position/compass_hdg   std_msgs/Float64 (deg, 0 = North, CW)
/mavros/global_position/global        sensor_msgs/NavSatFix (~5 Hz, synthetic)
/mavros/imu/data                      sensor_msgs/Imu  (orientation + ang vel)
/mavros/local_position/pose           geometry_msgs/PoseStamped

The NavSatFix is synthesized from the ENU odom position about the
``sim_origin_lat`` / ``sim_origin_lon`` parameters (equirectangular
inverse) and throttled to ~5 Hz, matching the field GPS rate. It exists
so the BlueBoat GCS's GPS-anchored map can anchor against the simulator
exactly as it does against the real boat.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from ..core.geometry import enu_yaw_to_compass_deg, quat_to_rpy

EARTH_RADIUS_M = 6378137.0
GPS_RATE_HZ = 5.0


class MavrosShimNode(Node):
    def __init__(self) -> None:
        super().__init__("mavros_shim")
        self.declare_parameter("odom_topic", "/blueboat/odom")
        # Geographic position of the ENU world origin (defaults match
        # the GCS's built-in simulator origin).
        self.declare_parameter("sim_origin_lat", 43.6961)
        self.declare_parameter("sim_origin_lon", 7.3080)

        self._hdg_pub = self.create_publisher(
            Float64, "/mavros/global_position/compass_hdg", 10)
        self._imu_pub = self.create_publisher(Imu, "/mavros/imu/data", 10)
        self._pose_pub = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", 10)
        # Real mavros publishes NavSatFix as sensor data (BEST_EFFORT).
        self._fix_pub = self.create_publisher(
            NavSatFix, "/mavros/global_position/global",
            QoSPresetProfiles.SENSOR_DATA.value)
        self._last_fix_stamp = -1e18

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)

        hdg = Float64()
        hdg.data = enu_yaw_to_compass_deg(yaw)
        self._hdg_pub.publish(hdg)

        imu = Imu()
        imu.header = msg.header
        imu.orientation = q
        imu.angular_velocity = msg.twist.twist.angular
        self._imu_pub.publish(imu)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._pose_pub.publish(pose)

        self._maybe_publish_fix(msg)

    def _maybe_publish_fix(self, msg: Odometry) -> None:
        """~5 Hz synthetic NavSatFix from the ENU position (see docstring)."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp - self._last_fix_stamp < 1.0 / GPS_RATE_HZ:
            return
        self._last_fix_stamp = stamp
        lat0 = float(self.get_parameter("sim_origin_lat").value)
        lon0 = float(self.get_parameter("sim_origin_lon").value)
        east = float(msg.pose.pose.position.x)
        north = float(msg.pose.pose.position.y)
        fix = NavSatFix()
        fix.header = msg.header
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat0 + math.degrees(north / EARTH_RADIUS_M)
        fix.longitude = lon0 + math.degrees(
            east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
        fix.altitude = 0.0
        self._fix_pub.publish(fix)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MavrosShimNode()
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
