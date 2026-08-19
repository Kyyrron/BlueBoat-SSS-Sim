"""Shared value types used across the blueboat_sss_sim platform.

Pure Python / NumPy only -- this module must never import ROS so that the
world generator, sonar renderer and dataset tooling remain testable and
reusable outside a ROS environment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class Side(str, Enum):
    """Sonar transducer side, matching the `side` string field of
    ``blueboat_interfaces/OmniscanProfile``."""

    PORT = "port"
    STARBOARD = "starboard"

    @property
    def sign(self) -> float:
        """Athwartship direction sign in the vehicle frame (ENU, x forward):
        port looks to +y (left), starboard to -y (right)."""
        return +1.0 if self is Side.PORT else -1.0


@dataclass(frozen=True)
class Pose3D:
    """Vehicle (or sensor) pose in the local world frame.

    Convention: ENU-like local frame identical to the Gazebo world frame of
    the existing BlueBoat simulator. ``z = 0`` is the water surface, the
    seabed lies at negative ``z``. Angles in radians.
    """

    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def heading_deg(self) -> float:
        """Compass heading in degrees (0 = North = +y, clockwise positive),
        derived from the ENU yaw (0 = +x = East, counter-clockwise)."""
        return float((90.0 - np.degrees(self.yaw)) % 360.0)


@dataclass
class Ping:
    """One rendered side-scan ping, renderer output -> encoder input.

    ``power`` is the linear per-bin echo power *before* device scaling,
    in arbitrary units normalised so that a flat, mid-reflectivity seabed
    produces values of order 1.
    """

    side: Side
    power: np.ndarray            # float64[num_results], linear power
                                 # (diffuse/Lambert component)
    pose: Pose3D                 # sensor pose at ping time
    altitude_m: float            # sensor height above seabed at nadir
    t_sim: float                 # simulation time [s] of the ping
    start_mm: int
    length_mm: int
    dropped: bool = False        # True -> the "device" lost this ping
    specular: np.ndarray | None = None
    # Coherent quasi-specular component (near-nadir first-return lobe),
    # kept separate from `power` because its fluctuation statistics differ:
    # the coherent echo is low-CV (Rician, high K-factor) while the diffuse
    # field is fully-developed speckle. The noise stage combines them.


@dataclass
class GroundTruthContact:
    """Per-ping ground-truth observation of one scene object, used by the
    dataset labeler. Produced by the renderer as a by-product (free lunch:
    the renderer already knows the geometry)."""

    object_id: int
    object_type: str
    side: Side
    slant_range_m: float          # range to object centre
    extent_bins: float            # approx. object extent in range bins
    shadow_bins: float            # approx. shadow length in range bins
    visible: bool                 # False if fully occluded / out of swath


@dataclass
class RenderedPing:
    """Bundle returned by a renderer: the ping plus its ground truth."""

    ping: Ping
    contacts: list[GroundTruthContact] = field(default_factory=list)


@dataclass(frozen=True)
class GridSpec:
    """Regular 2D grid layout shared by all scene rasters."""

    origin_x: float               # world x of cell (0,0) centre
    origin_y: float               # world y of cell (0,0) centre
    resolution: float             # cell size [m]
    nx: int                       # number of columns (x)
    ny: int                       # number of rows (y)

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """(xmin, ymin, xmax, ymax) of the covered area."""
        return (
            self.origin_x,
            self.origin_y,
            self.origin_x + (self.nx - 1) * self.resolution,
            self.origin_y + (self.ny - 1) * self.resolution,
        )

    def world_to_grid(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """World coordinates -> fractional grid indices (col, row)."""
        return (
            (np.asarray(x) - self.origin_x) / self.resolution,
            (np.asarray(y) - self.origin_y) / self.resolution,
        )


@dataclass
class PlacedObject:
    """One object instance placed in the scene (ground truth record)."""

    object_id: int
    type: str
    x: float
    y: float
    yaw: float                    # radians
    length: float                 # along local x [m]
    width: float                  # along local y [m]
    proud_height: float           # height above seabed when unburied [m]
    burial: float                 # 0 = proud, 1 = fully buried
    reflectivity: float           # 0..1 acoustic reflectivity
    material: str = "generic"

    @property
    def effective_height(self) -> float:
        return max(0.0, self.proud_height * (1.0 - self.burial))

    @property
    def footprint_radius(self) -> float:
        return 0.5 * float(np.hypot(self.length, self.width))
