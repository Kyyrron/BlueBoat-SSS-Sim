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
/mavros/imu/data                      sensor_msgs/Imu  (orientation + ang vel)
/mavros/local_position/pose           geometry_msgs/PoseStamped
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from ..core.geometry import enu_yaw_to_compass_deg, quat_to_rpy


class MavrosShimNode(Node):
    def __init__(self) -> None:
        super().__init__("mavros_shim")
        self.declare_parameter("odom_topic", "/blueboat/odom")

        self._hdg_pub = self.create_publisher(
            Float64, "/mavros/global_position/compass_hdg", 10)
        self._imu_pub = self.create_publisher(Imu, "/mavros/imu/data", 10)
        self._pose_pub = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", 10)

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)

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
