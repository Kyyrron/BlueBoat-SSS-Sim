"""Time-parameterised trajectories.

Two on-disk schemas exist in this project and both are readable here:

* the simulator's own ``trajectory.yaml`` -- ``waypoints: [[x, y], ...]``
  plus a constant ``speed`` (:class:`WaypointTrajectory`, historically in
  ``mission/patterns.py``);
* the MCS designer's ``blueboat_trajectory/1`` -- dense
  ``points: [[t, x, y, yaw], ...]`` samples (:class:`TimedTrajectory`),
  the format every anchored path in ``~/.config/blueboat_mcs/trajectories``
  uses, before and after deployment.

:func:`load_trajectory` dispatches on the keys present, so analysis and
launch tooling accept either file without caring which stack wrote it.
Both classes expose the same surface: ``pose_at(t) -> (x, y, yaw)``,
``duration``, ``total_length``, ``speed``, ``name``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass
class WaypointTrajectory:
    waypoints: np.ndarray            # (N, 2) world xy
    speed: float                     # surge speed [m/s]
    name: str = "mission"

    def __post_init__(self) -> None:
        self.waypoints = np.asarray(self.waypoints, dtype=np.float64)
        seg = np.diff(self.waypoints, axis=0)
        self._seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self._cum = np.concatenate([[0.0], np.cumsum(self._seg_len)])
        self._seg_yaw = np.arctan2(seg[:, 1], seg[:, 0])

    # ------------------------------------------------------------------
    @property
    def total_length(self) -> float:
        return float(self._cum[-1])

    @property
    def duration(self) -> float:
        return self.total_length / max(self.speed, 1e-6)

    def pose_at(self, t: float) -> tuple[float, float, float]:
        """(x, y, yaw) at time t; clamps at the last waypoint."""
        s = np.clip(t * self.speed, 0.0, self.total_length)
        i = int(np.clip(np.searchsorted(self._cum, s, side="right") - 1,
                        0, len(self._seg_len) - 1))
        f = (s - self._cum[i]) / max(self._seg_len[i], 1e-9)
        p = self.waypoints[i] + f * (self.waypoints[i + 1] - self.waypoints[i])
        return float(p[0]), float(p[1]), float(self._seg_yaw[i])

    def save_yaml(self, path: str | Path) -> None:
        doc = {"name": self.name, "speed": self.speed,
               # Derived metadata (informational; recomputed on load):
               # consumed e.g. by full_mission_launch to size the
               # path_publisher `total_time` window to the whole mission.
               "length_m": round(self.total_length, 2),
               "duration_s": round(self.duration, 1),
               "waypoints": self.waypoints.tolist()}
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, sort_keys=False)

    @classmethod
    def load_yaml(cls, path: str | Path) -> "WaypointTrajectory":
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        return cls(waypoints=np.array(doc["waypoints"]),
                   speed=float(doc["speed"]), name=str(doc.get("name", "mission")))


@dataclass
class TimedTrajectory:
    """Dense ``[t, x, y, yaw]`` samples (MCS ``blueboat_trajectory/1``)."""

    points: np.ndarray               # (N, 4) [t, x, y, yaw]
    name: str = "mission"
    loop: bool = False

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=np.float64)
        if self.points.ndim != 2 or self.points.shape[1] != 4:
            raise ValueError(
                f"points must be (N, 4) [t, x, y, yaw], got {self.points.shape}")
        if len(self.points) < 2:
            raise ValueError("trajectory needs at least 2 points")
        self._t = self.points[:, 0]
        if np.any(np.diff(self._t) < 0):
            raise ValueError("point times must be non-decreasing")
        # Unwrap yaw over the sample sequence so interpolation never takes
        # the long way round a +/-pi crossing.
        self._yaw = np.unwrap(self.points[:, 3])
        seg = np.diff(self.points[:, 1:3], axis=0)
        self._length = float(np.hypot(seg[:, 0], seg[:, 1]).sum())

    # ------------------------------------------------------------------
    @property
    def total_length(self) -> float:
        return self._length

    @property
    def duration(self) -> float:
        return float(self._t[-1] - self._t[0])

    @property
    def speed(self) -> float:
        """Mean ground speed (the schema has no constant-speed guarantee)."""
        return self.total_length / max(self.duration, 1e-6)

    def pose_at(self, t: float) -> tuple[float, float, float]:
        """(x, y, yaw) at time t; clamps at both ends (loops if ``loop``)."""
        t = float(t)
        if self.loop and self.duration > 0:
            t = self._t[0] + (t - self._t[0]) % self.duration
        t = float(np.clip(t, self._t[0], self._t[-1]))
        x = float(np.interp(t, self._t, self.points[:, 1]))
        y = float(np.interp(t, self._t, self.points[:, 2]))
        yaw = float(np.interp(t, self._t, self._yaw))
        return x, y, float(np.arctan2(np.sin(yaw), np.cos(yaw)))

    @classmethod
    def load_yaml(cls, path: str | Path) -> "TimedTrajectory":
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        return cls(points=np.array(doc["points"]),
                   name=str(doc.get("name", Path(path).stem)),
                   loop=bool(doc.get("loop", False)))


def load_trajectory(path: str | Path) -> WaypointTrajectory | TimedTrajectory:
    """Load either trajectory schema, dispatching on the keys present."""
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    if "points" in doc:
        return TimedTrajectory(points=np.array(doc["points"]),
                               name=str(doc.get("name", Path(path).stem)),
                               loop=bool(doc.get("loop", False)))
    if "waypoints" in doc:
        return WaypointTrajectory(waypoints=np.array(doc["waypoints"]),
                                  speed=float(doc["speed"]),
                                  name=str(doc.get("name", "mission")))
    raise KeyError(
        f"{path}: neither 'points' (blueboat_trajectory/1) nor 'waypoints' "
        "(simulator trajectory.yaml) present")
