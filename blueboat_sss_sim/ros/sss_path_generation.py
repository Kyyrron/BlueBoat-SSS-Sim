#!/usr/bin/env python3
"""Mission path service -- interface-identical sibling of ``path_generation.py``.

Serves the same ``blueboat_interfaces/srv/RequestPath`` service on
``/path_request`` (time array in, ``nav_msgs/Path`` out), but the
time-parameterised trajectory comes from a mission bundle's
``trajectory.yaml`` (see :mod:`blueboat_sss.mission.generate`) instead of
hard-coded analytic shapes.

Because the service contract is byte-identical, the unmodified
``master_control.py`` / ``path_publisher.py`` stack tracks generated survey
missions with **zero changes** -- at launch time you simply start this node
instead of ``path_generation.py``.

Parameters
----------
trajectory_file       path to trajectory.yaml (required)
display_log           bool, mirrors the original node
display_resolution_m  sample spacing of the full-path display topic (0.5)

RViz display of the whole mission (task 4)
------------------------------------------
``path_publisher`` requests only a sliding time window around "now" from
``/path_request``, so the ``nav_msgs/Path`` it republishes -- the one
usually visualised -- covers just that window; RViz showing a partial
lawnmower is that windowing, not a sampling bug (the service answers any
requested time correctly, clamping only beyond mission end). For
inspection, this node additionally publishes the *complete* mission once,
latched (TRANSIENT_LOCAL), on ``/mission/full_path`` -- add a Path display
on that topic in RViz (set its Durability Policy to Transient Local) to
see the entire pattern for the whole run.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from scipy.spatial.transform import Rotation as R

from blueboat_interfaces.srv import RequestPath

from ..mission.patterns import WaypointTrajectory


class MissionPathGeneration(Node):
    def __init__(self) -> None:
        super().__init__("path_generation")

        self.declare_parameter("display_log", False)
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("display_resolution_m", 0.5)
        self._display_log = bool(self.get_parameter("display_log").value)

        traj_file = self.get_parameter("trajectory_file").value
        if not traj_file:
            raise RuntimeError("parameter 'trajectory_file' is required")
        self._traj = WaypointTrajectory.load_yaml(traj_file)
        self.get_logger().info(
            f"mission trajectory '{self._traj.name}': "
            f"{len(self._traj.waypoints)} waypoints, "
            f"{self._traj.total_length:.0f} m, "
            f"{self._traj.duration:.0f} s at {self._traj.speed} m/s")

        self.create_service(RequestPath, "/path_request", self._generate_path)
        self._publish_full_path()

    # ------------------------------------------------- full-path display
    def _publish_full_path(self) -> None:
        """Latched complete-mission Path for RViz (see module docstring)."""
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._full_path_pub = self.create_publisher(Path, "/mission/full_path",
                                                    qos)
        step = float(self.get_parameter("display_resolution_m").value)
        n = max(int(self._traj.total_length / max(step, 0.05)) + 1, 2)
        times = np.linspace(0.0, self._traj.duration, n)
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        for t in times:
            pose = self._single_pose(float(t))
            pose.header.stamp = msg.header.stamp
            msg.poses.append(pose)
        self._full_path_pub.publish(msg)
        self.get_logger().info(
            f"full mission path latched on /mission/full_path "
            f"({n} poses @ {step} m)")

    def _single_pose(self, t: float) -> PoseStamped:
        x, y, yaw = self._traj.pose_at(t)
        quat = R.from_euler("zyx", [yaw, 0.0, 0.0]).as_quat()
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def _generate_path(self, request, response):
        if self._display_log:
            self.get_logger().info(
                f"path_request with {len(request.path_request.data)} samples")
        path_msg = Path()
        path_msg.header.frame_id = "world"
        now = self.get_clock().now().to_msg()
        for t in request.path_request.data:
            pose = self._single_pose(float(t))
            pose.header.stamp = now
            path_msg.poses.append(pose)
        response.path = path_msg
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionPathGeneration()
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
