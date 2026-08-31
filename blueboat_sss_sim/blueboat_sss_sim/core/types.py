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

    @property
    def channel(self) -> int:
        """Device channel number: 0 = port, 1 = starboard.

        The project-wide side identity carried in the packet itself, at
        byte 34 of the framed Ping-Protocol profile and in
        ``OmniscanProfile.channel_number``. Downstream ``.svlog`` writers
        and readers route on this value rather than on the topic or the
        ``src`` device tag, so it must be correct per side.
        """
        return 0 if self is Side.PORT else 1


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
    ghost: bool = False           # True -> a multipath image, not the object
    via: str = ""                 # reflector that produced it ("" = direct
                                  # path; else the mirror-source name, e.g.
                                  # "wall:quay_north" or "surface")

    @property
    def group_key(self) -> tuple[int, str]:
        """Identity for aggregation: the same object seen down two different
        paths is two contacts, never one. Grouping on ``object_id`` alone
        would merge an object with its own ghost into a box spanning the gap
        between them."""
        return (self.object_id, self.via)


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
class Wall:
    """One vertical reflecting boundary of an enclosed basin (quay, pontoon).

    A finite segment in plan view, extending from the seabed up to ``top_z``.
    Walls are *acoustic* scene furniture: they are not stamped into the height
    raster (a 2.5-D heightfield cannot carry a vertical face), so they produce
    multipath ghosts but no direct echo of their own -- see docs/sonar_model.md
    assumption A9.

    ``reflectivity`` is the fraction of incident energy the face returns, per
    bounce; a concrete quay is high (~0.6), a fendered pontoon much lower.
    """

    name: str
    x0: float
    y0: float
    x1: float
    y1: float
    top_z: float = 0.0            # world z of the wall top (0 = waterline)
    reflectivity: float = 0.5     # 0..1 energy fraction returned per bounce
    thickness: float = 0.30       # visual thickness in Gazebo [m]

    @property
    def length(self) -> float:
        return float(np.hypot(self.x1 - self.x0, self.y1 - self.y0))

    @property
    def unit(self) -> tuple[float, float]:
        """Unit vector along the wall, from end 0 to end 1."""
        L = max(self.length, 1e-9)
        return ((self.x1 - self.x0) / L, (self.y1 - self.y0) / L)

    @property
    def normal(self) -> tuple[float, float]:
        """Unit normal of the wall plane (either face; sign is irrelevant --
        the mirror reflection is symmetric in it)."""
        ux, uy = self.unit
        return (-uy, ux)


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
