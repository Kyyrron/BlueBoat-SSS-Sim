"""Mirror-source model for wall and surface multipath.

In an enclosed basin some of the transmitted energy reaches the seabed only
after bouncing off a quay wall or the water surface. The device measures
elapsed time and nothing else, so it paints that late echo at a slant range
where no seabed lies: a **ghost** -- a dimmer, displaced copy of the seabed and
of every object on it. Ghosts are the signature artifact of the regime, and
they look exactly like targets, which is why they are worth modelling: a
dataset with no ghost in it trains a detector that has never had to reject one.

The standard construction is the *image source*. Reflecting the transducer in
the reflecting plane turns the folded path into a straight line from the mirror
source, so the existing geometry pass renders a ghost with no new machinery and
the arrival lands in the bin its true path length earns -- not at an
approximated fixed offset, as the second-bottom-echo ghost
(``multipath_enabled``) does.

First order only: the mirror set is ``{each wall}`` plus, optionally, ``{z=0}``.
Composed bounces (wall then surface) and any path beyond them are out of scope
and stay on the assumption list in ``docs/sonar_model.md``.

Three details separate this from a plausible-looking approximation:

* **Handedness.** Reflection in a vertical plane flips the frame, so the look
  direction must be reflected directly; mirroring the yaw and re-deriving the
  look from ``yaw + side.sign * pi/2`` picks the wrong side. The roll term,
  which tilts the vertical fan toward the look direction, flips with it.
* **The surface image radiates upward.** A ``z = 0`` reflection negates the
  launch depression, so its beam weight is evaluated at ``-depression``. The
  main lobe sits *below* horizontal, so this path passes only through the
  sidelobe floor -- physically right, and the reason a near-perfectly
  reflecting interface does not swamp the calibrated direct field.
* **Walls are finite.** A ray only reflects if it actually crosses the wall
  segment, below its top. Without that test, ghosts appear on the wrong side
  of the basin.

No ROS, no I/O, no randomness -- the whole model is testable offline.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..core.geometry import (reflect_direction_across_line,
                             reflect_point_across_line)
from ..core.types import Pose3D, Side, Wall

#: Name carried by the water-surface image on ground-truth contacts.
SURFACE_NAME = "surface"


def wall_via(wall_name: str) -> str:
    """The ``via`` tag a wall's ghosts carry."""
    return f"wall:{wall_name}"


@dataclass(frozen=True)
class MirrorSource:
    """One virtual transducer: where it sits, where it looks, what it costs.

    ``gain`` is the energy fraction surviving the bounce. ``depression_sign``
    is ``-1`` for an odd number of ``z = 0`` reflections (the ray leaves the
    real transducer upward). ``wall`` is ``None`` for the surface image, whose
    reflector is unbounded.
    """

    name: str
    origin: Pose3D
    look: float                   # world-frame athwartship look angle [rad]
    fwd: float                    # world-frame along-track angle [rad]
    depression_sign: float
    roll_toward_side: float
    gain: float
    wall: Optional[Wall] = None


def mirror_sources(walls: list[Wall], sensor: Pose3D, side: Side,
                   range_max_m: float, *, wall_gain: float,
                   surface_enabled: bool, surface_reflectivity: float
                   ) -> list[MirrorSource]:
    """Build the first-order mirror set for one ping, already range-culled.

    A wall whose plane lies further than the receive window can produce no
    arrival inside it: the shortest folded path is the perpendicular distance
    to the plane (reached only by a sample at the wall foot), so ``d >
    range_max`` culls the wall outright. That is what keeps a survey's cost at
    baseline everywhere except near a wall.
    """
    out: list[MirrorSource] = []
    look = sensor.yaw + side.sign * np.pi / 2.0
    roll_toward = -side.sign * sensor.roll

    for w in walls:
        if w.reflectivity <= 0.0:
            continue
        nx, ny = w.normal
        signed = (sensor.x - w.x0) * nx + (sensor.y - w.y0) * ny
        if abs(signed) > range_max_m:
            continue
        # A fan pointing away from the wall never bounces off it. The mirror
        # image of such a fan looks away from the basin, so every sample of it
        # would fail the crossing test -- cull it here instead of paying for a
        # full render that contributes nothing. Half the sources on a
        # wall-parallel survey leg are this case: the outboard side looks away.
        if (np.cos(look) * nx + np.sin(look) * ny) * signed > 0.0:
            continue
        mx, my = reflect_point_across_line(sensor.x, sensor.y,
                                           w.x0, w.y0, nx, ny)
        lx, ly = reflect_direction_across_line(np.cos(look), np.sin(look),
                                               nx, ny)
        fx, fy = reflect_direction_across_line(np.cos(sensor.yaw),
                                               np.sin(sensor.yaw), nx, ny)
        out.append(MirrorSource(
            name=wall_via(w.name),
            origin=Pose3D(x=mx, y=my, z=sensor.z, roll=sensor.roll,
                          pitch=sensor.pitch,
                          yaw=float(np.arctan2(fy, fx))),
            look=float(np.arctan2(ly, lx)),
            fwd=float(np.arctan2(fy, fx)),
            depression_sign=1.0,          # a vertical plane keeps the tilt
            roll_toward_side=-roll_toward,  # handedness flips with the frame
            gain=float(w.reflectivity * wall_gain),
            wall=w))

    if surface_enabled and surface_reflectivity > 0.0:
        out.append(MirrorSource(
            name=SURFACE_NAME,
            origin=Pose3D(x=sensor.x, y=sensor.y, z=-sensor.z,
                          roll=sensor.roll, pitch=sensor.pitch,
                          yaw=sensor.yaw),
            look=float(look),
            fwd=float(sensor.yaw),
            depression_sign=-1.0,         # the path leaves upward
            roll_toward_side=roll_toward,
            gain=float(surface_reflectivity * wall_gain),
            wall=None))
    return out


def crossing_mask(wall: Wall, origin: Pose3D, px: np.ndarray, py: np.ndarray,
                  pz: np.ndarray) -> np.ndarray:
    """Which samples are actually reached through this finite wall.

    ``origin`` is the *mirror* source. The straight segment from it to a sample
    crosses the wall plane exactly at the reflection point of the true folded
    path, so the test is: does that segment cross the plane at all, within the
    wall's plan extent, and below its top.
    """
    nx, ny = wall.normal
    ux, uy = wall.unit
    d_m = (origin.x - wall.x0) * nx + (origin.y - wall.y0) * ny
    d_p = (px - wall.x0) * nx + (py - wall.y0) * ny

    # The sample must lie on the far side of the plane from the mirror -- i.e.
    # on the same side as the real transducer.
    opposite = (d_m * d_p) < 0.0
    denom = d_m - d_p
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(np.abs(denom) > 1e-12, d_m / denom, -1.0)
    inside_segment = (t > 0.0) & (t < 1.0)

    cx = origin.x + t * (px - origin.x)
    cy = origin.y + t * (py - origin.y)
    cz = origin.z + t * (pz - origin.z)
    s = (cx - wall.x0) * ux + (cy - wall.y0) * uy
    within = (s >= 0.0) & (s <= wall.length)
    below_top = cz <= wall.top_z
    return opposite & inside_segment & within & below_top


def point_crosses(wall: Wall, origin: Pose3D, x: float, y: float,
                  z: float) -> bool:
    """Scalar :func:`crossing_mask`, for the per-object ground-truth test."""
    return bool(crossing_mask(wall, origin, np.array([x]), np.array([y]),
                              np.array([z]))[0])
