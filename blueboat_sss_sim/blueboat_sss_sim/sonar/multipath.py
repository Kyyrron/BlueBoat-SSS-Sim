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
  main lobe sits *below* horizontal (tilt 20 deg, FWHM 50 deg), so this path
  is attenuated by 8 dB at 20 deg up, 17 dB at 40 deg, and is held at the
  sidelobe floor (-24 dB) beyond 50.6 deg up -- physically right, and the
  reason a near-perfectly reflecting interface does not swamp the
  calibrated direct field.
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
    gain: float                   # two bounces: reflectivity^2 * wall_gain
    wall: Optional[Wall] = None
    gain_single: float = 0.0      # one bounce: reflectivity * wall_gain


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
        # Finite segment: a wall whose nearest *point* (not plane) lies
        # beyond the window reflects nothing into it either. A curved quay
        # flattened into a dozen short chords would otherwise keep every
        # chord's infinite plane alive and cost a mirror render each.
        ux, uy = w.unit
        s = (sensor.x - w.x0) * ux + (sensor.y - w.y0) * uy
        s = min(max(s, 0.0), w.length)
        if np.hypot(sensor.x - (w.x0 + s * ux),
                    sensor.y - (w.y0 + s * uy)) > range_max_m:
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
            # The folded path meets the face twice (out and back), so the
            # image carries the per-bounce reflectivity squared; the
            # single-bounce band (seabed -> face -> transducer) once.
            gain=float(w.reflectivity ** 2 * wall_gain),
            gain_single=float(w.reflectivity * wall_gain),
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
    # Between the face's bottom and its top: a quay reaches the seabed, a
    # pontoon stops at its draft and lets the ray under it.
    on_face = (cz <= wall.top_z) & (cz >= wall.z_bottom)
    return opposite & inside_segment & within & on_face


def reflection_point(wall: Wall, origin: Pose3D, px: np.ndarray,
                     py: np.ndarray, pz: np.ndarray
                     ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Where the folded path meets the face, per sample.

    The unfolded segment from the mirror ``origin`` to a sample crosses the
    wall plane exactly at the reflection point of the true path, so the two
    *real* legs are ``transducer -> here`` and ``here -> sample``. Samples
    that do not cross (``crossing_mask`` false) get the sample itself, which
    makes both legs degenerate and blocks nothing.
    """
    nx, ny = wall.normal
    d_m = (origin.x - wall.x0) * nx + (origin.y - wall.y0) * ny
    d_p = (px - wall.x0) * nx + (py - wall.y0) * ny
    denom = d_m - d_p
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(np.abs(denom) > 1e-12, d_m / denom, 0.0)
    t = np.clip(t, 0.0, 1.0)
    return (origin.x + t * (px - origin.x),
            origin.y + t * (py - origin.y),
            origin.z + t * (pz - origin.z))


def segment_blocked(walls: list[Wall], ax, ay, az, bx, by, bz,
                    skip: Wall | None = None) -> np.ndarray:
    """Which segments ``a -> b`` a wall stands in the way of.

    The same rule the direct path uses, written for an arbitrary segment so
    a folded path can be occluded too: the segment must cross the wall's
    plane between its ends, inside the chord's plan extent, at a height
    within the face's own band (a quay's band reaches the seabed, a
    pontoon's stops at its draft and the ray passes under it).

    Any argument may be a scalar or an array; the result broadcasts. Without
    this a ghost and the single-bounce corner band are painted straight
    through whatever stands between -- the 2026-09-18 review's wall census
    found the band on the far side of a nearer quay in 97 % of the pings
    that saw two walls.
    """
    ax, ay, az, bx, by, bz = np.broadcast_arrays(
        *(np.asarray(v, dtype=np.float64) for v in (ax, ay, az, bx, by, bz)))
    blocked = np.zeros(np.shape(ax), dtype=bool)
    for w in walls:
        if skip is not None and w is skip:
            continue
        nx, ny = w.normal
        ux, uy = w.unit
        d_a = (ax - w.x0) * nx + (ay - w.y0) * ny
        d_b = (bx - w.x0) * nx + (by - w.y0) * ny
        denom = d_a - d_b
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(np.abs(denom) > 1e-12, d_a / denom, -1.0)
        crosses = (d_a * d_b < 0.0) & (t > 0.0) & (t < 1.0)
        if not crosses.any():
            continue
        cx = ax + t * (bx - ax)
        cy = ay + t * (by - ay)
        cz = az + t * (bz - az)
        s = (cx - w.x0) * ux + (cy - w.y0) * uy
        blocked |= (crosses & (s >= 0.0) & (s <= w.length)
                    & (cz <= w.top_z) & (cz >= w.z_bottom))
    return blocked


def bounce_weight(wall: Wall, origin: Pose3D, px: np.ndarray, py: np.ndarray,
                  pz: np.ndarray, diffuse_fraction: float,
                  wavenumber: float) -> np.ndarray:
    """Energy fraction of **one** bounce surviving the face's roughness.

    The coherent (mirror) part of a reflection off a rough face falls with
    the Rayleigh roughness parameter ``Gamma = 2 k sigma_h cos(theta)``,
    ``theta`` the angle from the face normal, as ``exp(-Gamma^2)`` in power
    (Eckart 1953, Urick 1983 ch. 5). At 450 kHz (k = 1885 rad/m) a
    millimetre-rough quay keeps its mirror only near grazing -- which is
    exactly the band just beyond the face, where the mirrored seabed sits.
    The rest is scattered incoherently; ``diffuse_fraction`` is the share of
    it that still reaches the mirror geometry (lumped, set against the
    corpus: docs/sonar_model.md section 11). ``wavenumber`` is the model's
    one k, ``acoustics.wavenumber(cfg)`` (``sonar_frequency_hz``, the same k
    as the coherent split and the object facets); the reported ``ping_hz``
    of the profile is a separate wire field and never enters the physics.
    """
    nx, ny = wall.normal
    dx, dy, dz = px - origin.x, py - origin.y, pz - origin.z
    norm = np.sqrt(dx * dx + dy * dy + dz * dz)
    cos_face = np.abs(dx * nx + dy * ny) / np.maximum(norm, 1e-9)
    gamma = 2.0 * float(wavenumber) * float(wall.roughness_m) * cos_face
    coherent = np.exp(-gamma * gamma)
    return coherent + float(diffuse_fraction) * (1.0 - coherent)


def point_crosses(wall: Wall, origin: Pose3D, x: float, y: float,
                  z: float) -> bool:
    """Scalar :func:`crossing_mask`, for the per-object ground-truth test."""
    return bool(crossing_mask(wall, origin, np.array([x]), np.array([y]),
                              np.array([z]))[0])
