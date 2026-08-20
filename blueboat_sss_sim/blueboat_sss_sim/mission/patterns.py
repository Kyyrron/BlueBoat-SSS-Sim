"""Survey path patterns.

Every pattern produces a :class:`WaypointTrajectory`: an ordered waypoint
list plus a constant surge speed, which can be

* sampled as a time-parameterised pose ``pose_at(t)`` (linear transit +
  in-place turn blending), matching the contract of the existing
  ``path_generation.py`` trajectories so the unmodified ``master_control``
  stack can track it via the same ``RequestPath`` service; or
* exported to YAML for inspection / replay.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

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


# ----------------------------------------------------------------- patterns
def lawnmower(bbox: tuple[float, float, float, float], spacing: float,
              speed: float = 1.0, heading_deg: float = 0.0,
              lead_in: float = 3.0) -> WaypointTrajectory:
    """Boustrophedon survey of ``bbox = (xmin, ymin, xmax, ymax)`` with
    track ``spacing``; lines run along ``heading_deg`` (world frame)."""
    xmin, ymin, xmax, ymax = bbox
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    th = np.radians(heading_deg)
    c, s = np.cos(th), np.sin(th)
    # Work in the rotated frame where lines are along +x.
    corners = np.array([[xmin, ymin], [xmin, ymax], [xmax, ymin], [xmax, ymax]])
    rel = corners - [cx, cy]
    rot = rel @ np.array([[c, s], [-s, c]]).T
    hx = rot[:, 0].max()
    hy = rot[:, 1].max()
    ys = np.arange(-hy + spacing / 2, hy, spacing)
    pts = []
    for i, y in enumerate(ys):
        xa, xb = (-hx - lead_in, hx + lead_in)
        if i % 2:
            xa, xb = xb, xa
        pts += [[xa, y], [xb, y]]
    pts = np.array(pts)
    world = pts @ np.array([[c, -s], [s, c]]).T + [cx, cy]
    return WaypointTrajectory(world, speed, "lawnmower")


def spiral(center: tuple[float, float], r_max: float, spacing: float,
           speed: float = 1.0, points_per_turn: int = 48) -> WaypointTrajectory:
    """Archimedean outward spiral (constant track spacing)."""
    b = spacing / (2 * np.pi)
    th_max = r_max / b
    th = np.linspace(0.5, th_max, max(int(points_per_turn * th_max / (2 * np.pi)), 8))
    x = center[0] + b * th * np.cos(th)
    y = center[1] + b * th * np.sin(th)
    return WaypointTrajectory(np.stack([x, y], 1), speed, "spiral")


def random_survey(bbox: tuple[float, float, float, float], n_legs: int,
                  speed: float = 1.0, margin: float = 3.0,
                  seed: int = 0) -> WaypointTrajectory:
    """Random straight legs inside the box -- diverse aspect angles for
    dataset variety."""
    rng = np.random.default_rng(seed)
    xmin, ymin, xmax, ymax = bbox
    pts = rng.uniform([xmin + margin, ymin + margin],
                      [xmax - margin, ymax - margin], size=(max(n_legs, 2), 2))
    return WaypointTrajectory(pts, speed, "random_survey")


def _best_entry_variant(pts: np.ndarray, start: tuple[float, float] | None,
                        start_heading_deg: float) -> np.ndarray:
    """Pick the boustrophedon traversal whose entry point lies most *in
    front of* the robot's spawn heading.

    A lawnmower over a box admits four equivalent covers (enter at either
    end of the first line x start from either the first or last line).
    Scoring each candidate's first waypoint by cos(angle from the spawn
    heading), with a mild distance tie-break, avoids missions that begin
    with a 180 deg turn and a long transit behind the robot."""
    if start is None:
        return pts
    p0 = np.asarray(start, dtype=np.float64)
    h = np.radians(start_heading_deg)
    hv = np.array([np.cos(h), np.sin(h)])
    n = len(pts)
    # line-flip variant: swap the two endpoints of every line segment pair
    flip = pts.reshape(n // 2, 2, 2)[:, ::-1, :].reshape(n, 2)
    best, best_score = pts, -np.inf
    for cand in (pts, pts[::-1], flip, flip[::-1]):
        d = cand[0] - p0
        dist = float(np.hypot(*d))
        score = float(d @ hv) / max(dist, 1e-6) - 0.002 * dist
        if score > best_score:
            best, best_score = cand, score
    return np.ascontiguousarray(best)


def _with_start(traj: WaypointTrajectory,
                start: tuple[float, float] | None,
                min_dist: float = 0.5) -> WaypointTrajectory:
    """Prepend the vehicle spawn/start point as the first waypoint (transit
    leg into the survey), so the path begins where the robot actually is.

    Skipped when the pattern already starts within ``min_dist`` of it.
    ``pose_at(0)`` then returns the start point heading toward the first
    survey waypoint, which the tracking stack follows smoothly."""
    if start is None:
        return traj
    p0 = np.asarray(start, dtype=np.float64)
    if float(np.hypot(*(traj.waypoints[0] - p0))) < min_dist:
        return traj
    return WaypointTrajectory(np.vstack([p0[None, :], traj.waypoints]),
                              traj.speed, traj.name)


def build_pattern(cfg: Mapping[str, Any], seed: int = 0) -> WaypointTrajectory:
    """Mission-YAML dispatch.

    ``cfg['start']`` (default ``[0, 0]``, the robot spawn) is prepended to
    every pattern as a transit waypoint; set ``start: null`` to disable."""
    start_raw = cfg.get("start", (0.0, 0.0))
    start = tuple(start_raw) if start_raw is not None else None
    start_heading = float(cfg.get("start_heading_deg", 0.0))
    kind = str(cfg.get("pattern", "lawnmower"))
    if kind == "lawnmower":
        p = cfg.get("lawnmower", {})
        traj = lawnmower(tuple(p.get("bbox", (-30, -20, 30, 20))),
                         float(p.get("spacing", 8.0)),
                         float(p.get("speed", 1.0)),
                         float(p.get("heading_deg", 0.0)))
        traj = WaypointTrajectory(
            _best_entry_variant(traj.waypoints, start, start_heading),
            traj.speed, traj.name)
    elif kind == "spiral":
        p = cfg.get("spiral", {})
        traj = spiral(tuple(p.get("center", (0.0, 0.0))),
                      float(p.get("r_max", 25.0)),
                      float(p.get("spacing", 8.0)),
                      float(p.get("speed", 1.0)))
    elif kind == "random":
        p = cfg.get("random", {})
        traj = random_survey(tuple(p.get("bbox", (-30, -20, 30, 20))),
                             int(p.get("n_legs", 12)),
                             float(p.get("speed", 1.0)),
                             seed=seed)
    elif kind == "waypoints":
        p = cfg.get("waypoints", {})
        traj = WaypointTrajectory(np.array(p["points"]),
                                  float(p.get("speed", 1.0)), "waypoints")
    else:
        raise ValueError(f"Unknown mission pattern '{kind}'")
    return _with_start(traj, start)
