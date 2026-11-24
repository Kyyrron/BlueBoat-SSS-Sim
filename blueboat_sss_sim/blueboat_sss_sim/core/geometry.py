"""Frame / angle / sampling helpers. Pure NumPy, no ROS."""

from __future__ import annotations

import numpy as np

from .types import GridSpec


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Quaternion (x, y, z, w) -> roll, pitch, yaw [rad], ZYX convention.

    Implemented locally to avoid a scipy dependency inside the hot ROS
    callback path.
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return float(roll), float(pitch), float(yaw)


def rotation_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body -> world rotation matrix, ZYX (yaw, pitch, roll) convention --
    the inverse of :func:`quat_to_rpy`."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def enu_yaw_to_compass_deg(yaw_rad: float) -> float:
    """ENU yaw (0 = +x, CCW) -> compass heading degrees (0 = +y/North, CW)."""
    return float((90.0 - np.degrees(yaw_rad)) % 360.0)


def wrap_pi(a: np.ndarray | float) -> np.ndarray | float:
    """Wrap angle(s) into (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def compass_deg_to_enu_yaw(heading_deg: float) -> float:
    """Compass heading degrees (0 = +y/North, CW) -> ENU yaw (0 = +x, CCW).

    Exact inverse of :func:`enu_yaw_to_compass_deg`, needed when a recorded
    stream carries only the hardware-facing compass heading (as every
    ``OmniscanProfile`` and every raw Ping-Protocol frame does) and the
    ENU yaw has to be recovered for geometry.
    """
    return float(wrap_pi(np.radians(90.0 - heading_deg)))


def reflect_point_across_line(px: float, py: float,
                              ax: float, ay: float,
                              nx: float, ny: float) -> tuple[float, float]:
    """Mirror the point (px, py) in the line through (ax, ay) with unit
    normal (nx, ny).

    In 3D this is the reflection in the *vertical plane* standing on that
    line, which leaves z untouched -- the mirror-source construction for a
    quay wall (see :mod:`blueboat_sss_sim.sonar.multipath`).
    """
    d = (px - ax) * nx + (py - ay) * ny        # signed distance to the plane
    return (px - 2.0 * d * nx, py - 2.0 * d * ny)


def reflect_direction_across_line(dx: float, dy: float,
                                  nx: float, ny: float) -> tuple[float, float]:
    """Mirror a direction vector in a plane with unit normal (nx, ny)."""
    d = dx * nx + dy * ny
    return (dx - 2.0 * d * nx, dy - 2.0 * d * ny)


def bilinear_sample(raster: np.ndarray, grid: GridSpec,
                    x: np.ndarray, y: np.ndarray,
                    fill: float = 0.0) -> np.ndarray:
    """Bilinearly sample ``raster[ny, nx]`` at world coordinates (x, y).

    Out-of-bounds queries return ``fill``. Vectorised over x/y arrays.
    """
    cx, cy = grid.world_to_grid(x, y)
    valid = (cx >= 0) & (cy >= 0) & (cx <= grid.nx - 1) & (cy <= grid.ny - 1)

    cx = np.clip(cx, 0, grid.nx - 1.000001)
    cy = np.clip(cy, 0, grid.ny - 1.000001)
    x0 = cx.astype(np.int64)
    y0 = cy.astype(np.int64)
    x1 = np.minimum(x0 + 1, grid.nx - 1)
    y1 = np.minimum(y0 + 1, grid.ny - 1)
    fx = cx - x0
    fy = cy - y0

    v = (raster[y0, x0] * (1 - fx) * (1 - fy)
         + raster[y0, x1] * fx * (1 - fy)
         + raster[y1, x0] * (1 - fx) * fy
         + raster[y1, x1] * fx * fy)
    return np.where(valid, v, fill)


# --------------------------------------------------------------- walls
#: Endpoints closer than this (design metres) become one shared point in
#: the built world (and the canvas snaps to them while drawing).
WALL_SNAP_M = 0.75
#: Chord-height tolerance a curved wall is flattened to: the raster cell.
WALL_SAGITTA_M = 0.10


def merge_endpoints(walls: list[dict], tol_m: float = WALL_SNAP_M,
                    min_length_m: float = 0.05) -> list[dict]:
    """Snap every cluster of wall endpoints within ``tol_m`` to its centroid.

    Union-find over all 2N endpoints (transitive: three quays meeting at a
    corner collapse to one point), deterministic and idempotent -- a second
    pass moves nothing. Walls shorter than ``min_length_m`` after merging
    are dropped. Returns new dicts; ``ctrl`` (a curved wall's through
    point) is carried along untouched. This is what keeps the depth
    interpolation barriers and the sonar faces water-tight at corners
    drawn by hand.
    """
    pts = []
    for w in walls:
        pts.append((float(w["x0"]), float(w["y0"])))
        pts.append((float(w["x1"]), float(w["y1"])))
    n = len(pts)
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        for j in range(i + 1, n):
            if np.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]) <= tol_m:
                ri, rj = find(i), find(j)
                if ri != rj:
                    parent[max(ri, rj)] = min(ri, rj)
    groups: dict[int, list[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    snapped = list(pts)
    for members in groups.values():
        if len(members) > 1:
            cx = float(np.mean([pts[m][0] for m in members]))
            cy = float(np.mean([pts[m][1] for m in members]))
            for m in members:
                snapped[m] = (cx, cy)
    out = []
    for k, w in enumerate(walls):
        w = dict(w)
        if "ctrl" in w and w["ctrl"] is not None:
            w["ctrl"] = [float(w["ctrl"][0]), float(w["ctrl"][1])]
        (w["x0"], w["y0"]), (w["x1"], w["y1"]) = snapped[2 * k], snapped[2 * k + 1]
        if np.hypot(w["x1"] - w["x0"], w["y1"] - w["y0"]) >= min_length_m:
            out.append(w)
    return out


def quad_bezier_polyline(p0, mid, p2, max_sagitta_m: float = WALL_SAGITTA_M,
                         max_segments: int = 64) -> np.ndarray:
    """Chords of the quadratic Bézier from ``p0`` to ``p2`` that passes
    through ``mid`` at t = 1/2 (control point ``2·mid − (p0 + p2)/2``),
    subdivided adaptively so no chord deviates from the curve by more than
    ``max_sagitta_m``: with n uniform-parameter chords the deviation is
    exactly ``|P0 − 2P1 + P2| / (4 n²)`` (the second difference of a
    quadratic is constant). Returns an (n + 1, 2) array of vertices."""
    p0 = np.asarray(p0, dtype=np.float64)
    p2 = np.asarray(p2, dtype=np.float64)
    m = np.asarray(mid, dtype=np.float64)
    p1 = 2.0 * m - 0.5 * (p0 + p2)
    d = float(np.linalg.norm(p0 - 2.0 * p1 + p2))
    n = int(np.ceil(np.sqrt(d / (4.0 * max(max_sagitta_m, 1e-6)))))
    n = int(min(max(n, 1), max_segments))
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2


def wall_segments(wall: dict, max_sagitta_m: float = WALL_SAGITTA_M) -> list[dict]:
    """A wall dict → the straight ``Wall``-shaped dicts the engine gets.

    A straight wall (no ``ctrl``) comes back as itself, name untouched --
    every existing world stays byte-identical. A curved one becomes chained
    chords ``<name>_s01 … <name>_sNN`` sharing every property, ``ctrl``
    stripped (it is a builder-only key the engine would reject).
    """
    w = dict(wall)
    ctrl = w.pop("ctrl", None)
    if ctrl is None:
        return [w]
    pts = quad_bezier_polyline((w["x0"], w["y0"]), ctrl, (w["x1"], w["y1"]),
                               max_sagitta_m)
    if pts.shape[0] <= 2:
        return [w]
    out = []
    for k in range(pts.shape[0] - 1):
        seg = dict(w)
        seg["name"] = f"{w['name']}_s{k + 1:02d}"
        seg["x0"], seg["y0"] = float(pts[k, 0]), float(pts[k, 1])
        seg["x1"], seg["y1"] = float(pts[k + 1, 0]), float(pts[k + 1, 1])
        out.append(seg)
    return out


def curve_base_name(name: str) -> str | None:
    """``quay_s03`` → ``quay`` for a chord of an expanded curve, else None."""
    import re
    m = re.match(r"^(.*)_s\d{2}$", str(name))
    return m.group(1) if m else None


def expand_walls(walls: list[dict], tol_m: float = WALL_SNAP_M,
                 max_sagitta_m: float = WALL_SAGITTA_M) -> list[dict]:
    """Merge close endpoints, then flatten curves: the single conversion
    from builder walls to engine walls (``world_folder`` and the Screen-3
    preview both call it, so they cannot disagree)."""
    out = []
    for w in merge_endpoints(walls, tol_m):
        out.extend(wall_segments(w, max_sagitta_m))
    return out
