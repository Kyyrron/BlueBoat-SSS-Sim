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


def enu_yaw_to_compass_deg(yaw_rad: float) -> float:
    """ENU yaw (0 = +x, CCW) -> compass heading degrees (0 = +y/North, CW)."""
    return float((90.0 - np.degrees(yaw_rad)) % 360.0)


def wrap_pi(a: np.ndarray | float) -> np.ndarray | float:
    """Wrap angle(s) into (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


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
