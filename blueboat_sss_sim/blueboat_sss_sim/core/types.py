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
    texture: np.ndarray | None = None
    # Unit-mean multiplicative speckle texture of the diffuse field, from the
    # renderer's coherent ground-scatterer sum (correlated along range by the
    # pulse and along track by footprint overlap). `power` stays the noiseless
    # mean; the noise stage multiplies by this instead of drawing Exp(1) per
    # bin. None -> the noise stage draws its own (legacy, uncorrelated).


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
    wall_name: str = ""           # set (with object_type "wall" and a
                                  # negative object_id) when the contact is
                                  # a wall face's own direct echo

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

    A finite segment in plan view. Its vertical extent depends on ``kind``:

    * ``quay`` -- a face from the **seabed** up to ``top_z`` (a ``top_z``
      below the waterline is a submerged sill, looked over by rays above
      it);
    * ``pontoon`` -- a **floating** structure: the wet face spans only
      ``[-draft_m, 0]``; sound passes underneath, so a pontoon occludes
      nothing below its draft, casts no quay-like shadow and reflects only
      the rays that meet its shallow band. A wall line the boat can drive
      over is a pontoon, never a quay.

    Walls are *acoustic* scene furniture: they are not stamped into the height
    raster (a 2.5-D heightfield cannot carry a vertical face). The renderer
    handles them analytically instead: a wall is a mirror source for
    multipath ghosts, its wet face returns a bright line at its own slant
    range, the seabed in front of it folds a single-bounce band just beyond
    the face, and a quay occludes every seabed sample behind it -- see
    docs/sonar_model.md section 11.

    ``reflectivity`` is the fraction of incident energy the face returns
    **per bounce** (a mirror ghost bounces twice and carries its square); a
    concrete quay is high (~0.65), a fendered pontoon much lower.
    ``roughness_m`` is the rms height roughness of the face, which at
    450 kHz (3.3 mm wavelength) decides how much of a bounce stays coherent
    (Rayleigh parameter, docs/sonar_model.md section 11).
    """

    name: str
    x0: float
    y0: float
    x1: float
    y1: float
    top_z: float = 0.0            # world z of the wall top (0 = waterline)
    reflectivity: float = 0.5     # 0..1 energy fraction returned per bounce
    thickness: float = 0.30       # visual thickness in Gazebo [m]
    kind: str = "quay"            # quay | pontoon
    draft_m: float = 0.4          # pontoon only: wet depth of the floating face
    roughness_m: float = 0.003    # rms face roughness [m] (concrete ~3 mm)

    @property
    def z_bottom(self) -> float:
        """World z where the reflecting face ends: the seabed (``-inf`` --
        the renderer clamps to the local seabed) for a quay, the draft
        for a pontoon."""
        return -float(self.draft_m) if self.kind == "pontoon" else -np.inf

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
    # Open-frame (skeletal) objects: fraction of the footprint filled by
    # actual structure -- 1.0 for a solid object, ~0.15 for a bicycle. The
    # lattice masks read it; solid masks ignore it. Additive with a default
    # so pre-existing manifests load unchanged.
    fill_fraction: float = 1.0
    # Class-specific extras (future per-class knobs); additive likewise.
    params: dict = field(default_factory=dict)
    # 3-D resting orientation (objects v3, 2026-09-03): roll/pitch of the
    # canonical class frame [rad] and the rest-pose name (``canonical``
    # = the legacy 2.5-D mask, bit-identical to every existing world;
    # anything else = a rotated solid settled on the local seabed plane,
    # ``worldgen/solids.py``). ``rest_z`` is the world z of the solid's
    # bounding-box centre above the pre-stamp terrain at (x, y), for the
    # Gazebo pose. Additive defaults: old manifests load unchanged.
    roll: float = 0.0
    pitch: float = 0.0
    rest: str = "canonical"
    rest_z: float = 0.0

    @property
    def effective_height(self) -> float:
        return max(0.0, self.proud_height * (1.0 - self.burial))

    @property
    def rested(self) -> bool:
        """True when the solid path (not the legacy mask) stamps it."""
        return self.rest != "canonical" or self.roll != 0.0 or self.pitch != 0.0

    @property
    def footprint_radius(self) -> float:
        return 0.5 * float(np.hypot(self.length, self.width))
