"""Ground truth of a world, for ``metadata.yaml`` (additive fields, schema
version 2) and ``ground_truth_rasters.npz`` (2026-11-15, completion round
session 02; acceptance criterion C5).

Per object: what it is (family, class, role, ``nominal_target``), its state
and construction, how it lies (``height_above_seabed_m`` after settling, the
direction of every flat face's normal and of its axis) and whether it is
``effectively_buried``. Per world: the depth domain, the seabed classes with
their shares, the structures. Read off the built scene, never from the
request that produced it, so that it cannot disagree with what is rendered.

Angles: azimuth in the world's ENU frame, degrees, 0 = east,
counter-clockwise (as ``yaw_deg``); elevation above the horizontal.

ROS-free, Qt-free; numpy.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..core.types import PlacedObject
from .compounds import envelope, parts
from .objects import V2_CLUTTER, V2_STRUCTURES, V2_TARGETS
from .population import EFFECTIVELY_BURIED
from .solids import rot_z, rotation_rpy

SCHEMA_VERSION = 2
RASTERS_NAME = "ground_truth_rasters.npz"
#: class -> family of the objects document (T targets, S structures, C clutter)
FAMILY = {"plastic_box": "T1", "drum_200l": "T2", "concrete_block": "T3", "trap_pot": "T4",
          "pipe_steel": "T5", "tire_car": "T6", "net_bundle": "T7",
          "mooring_block": "S1", "chain": "S2", "mooring_pile": "S3", "debris": "C1"}
#: an axis this close to the vertical is reported as "vertical" [deg]
VERTICAL_DEG = 80.0
_PARAM_SKIP = ("role", "nominal_target", "state", "variant", "hotspot", "hotspot_distance_m",
               "face_state", "pose", "sediment")


def role_of(o: PlacedObject) -> str:
    r = dict(o.params or {}).get("role")
    if r:
        return str(r)
    if o.type in V2_TARGETS:
        return "target"
    if o.type in V2_STRUCTURES:
        return "structure"
    return "clutter" if o.type in V2_CLUTTER else "legacy"


def _rotation(o: PlacedObject) -> np.ndarray:
    return rotation_rpy(o.roll, o.pitch, o.yaw) if o.rested else rot_z(o.yaw)


def _direction(v: np.ndarray) -> tuple[float | None, float]:
    """(azimuth, elevation) in degrees of the unit vector ``v``; azimuth None
    within ``90 - VERTICAL_DEG`` of the vertical."""
    el = float(np.degrees(np.arcsin(np.clip(v[2], -1.0, 1.0))))
    az = None if abs(el) > VERTICAL_DEG else float(np.degrees(np.arctan2(v[1], v[0])) % 360.0)
    return az, el


def faces_and_axis(o: PlacedObject) -> tuple[list, dict | None]:
    """The flat faces of the object's envelope (outward normal, area) and
    its axis of revolution, in the world."""
    sol = envelope(o)
    R = _rotation(o)
    faces: list = []
    axis = None
    if sol.kind == "box":
        dims = (sol.a, sol.b, sol.c)
        for ax in range(3):
            area = float(np.prod([dims[k] for k in range(3) if k != ax]))
            for sg, nm in ((1.0, "+"), (-1.0, "-")):
                az, el = _direction(sg * R[:, ax])
                faces.append({"face": f"{nm}{'xyz'[ax]}",
                              "azimuth_deg": None if az is None else round(az, 2),
                              "elevation_deg": round(el, 2), "area_m2": round(area, 4)})
    elif sol.kind in ("cylinder", "capsule", "annulus", "cone"):
        ax = sol.axis if sol.kind in ("cylinder", "capsule") else 2
        d = R[:, ax]
        az, el = _direction(d if d[2] >= 0.0 else -d)
        axis = ({"azimuth_deg": "vertical", "elevation_deg": round(el, 2)} if az is None
                else {"azimuth_deg": round(az % 180.0, 2), "elevation_deg": round(el, 2)})
        if sol.kind in ("cylinder", "annulus"):
            r_out = sol.a
            r_in = sol.b if sol.kind == "annulus" else 0.0
            area = float(np.pi * (r_out ** 2 - r_in ** 2))
            for sg, nm in ((1.0, "+"), (-1.0, "-")):
                fa, fe = _direction(sg * d)
                faces.append({"face": f"end{nm}", "azimuth_deg": None if fa is None else round(fa, 2),
                              "elevation_deg": round(fe, 2), "area_m2": round(area, 4)})
    return faces, axis


def height_above_seabed(o: PlacedObject, seabed) -> float:
    """Top of the object's parts above the terrain at its position [m],
    with the placement rule the sonar's mesh uses (``scene_mesh.
    facet_transform``): the stored rest pose, else the top of the envelope
    at terrain + effective height."""
    from ..sonar.raycast.scene_mesh import facet_transform
    Re, te = facet_transform(o, envelope(o), seabed)
    top = -np.inf
    for p in parts(o):
        c = Re @ (p.t + p.R @ p.solid.centre_offset) + te
        top = max(top, float(c[2] + (np.abs(Re @ p.R) @ p.solid.extent)[2]))
    z0 = float(seabed(np.array([o.x]), np.array([o.y]))[0])
    return max(top - z0, 0.0)


def object_truth(o: PlacedObject, seabed, sediment: str | None = None) -> dict[str, Any]:
    """The additive ground-truth fields of one object."""
    p = dict(o.params or {})
    role = role_of(o)
    faces, axis = faces_and_axis(o)
    out: dict[str, Any] = {
        "role": role, "family": FAMILY.get(o.type, "legacy"), "class": o.type,
        "nominal_target": bool(p.get("nominal_target", False)) and role == "target",
        "effectively_buried": bool(o.burial > EFFECTIVELY_BURIED),
        "state": str(p.get("state", "")), "variant": str(p.get("variant", "")),
        "pose": str(p.get("pose", o.rest)),
        "height_above_seabed_m": round(height_above_seabed(o, seabed), 3),
        "face_normals": faces, "axis": axis,
        "hotspot": str(p.get("hotspot", "")),
        "face_state": str(p.get("face_state", "profile")),
        "seabed_class": sediment or "",
        "params": {k: (v.item() if isinstance(v, np.generic) else v)
                   for k, v in p.items() if k not in _PARAM_SKIP},
    }
    return out


def seabed_classes(scene) -> tuple[np.ndarray, list, dict]:
    """(class index raster int8, class names, share per class) of a scene."""
    mid = np.asarray(scene.material_id)
    names = [str(n) for n in scene.material_names]
    cnt = np.bincount(mid.ravel().astype(np.int64), minlength=len(names))
    shares = {n: round(float(c) / max(mid.size, 1), 5) for n, c in zip(names, cnt) if c}
    return mid.astype(np.int8), names, shares


def world_truth(scene, terrain: np.ndarray, extras: Mapping[str, Any] | None = None,
                path_xy=None) -> tuple[dict, list, dict]:
    """``(world fields, per-object fields, rasters)`` of a built scene.
    ``terrain``: the height raster before objects were stamped.
    ``extras``: what the generator knows and the scene does not (campaign,
    conditions, population report, structures, run recommendations); its
    keys are written as they are."""
    from ..core.geometry import bilinear_sample
    from . import domain as dom
    g = scene.grid

    def seabed(x, y):
        return bilinear_sample(terrain, g, x, y, fill=float(terrain.min()))
    cls, names, shares = seabed_classes(scene)
    xmin, ymin, _, _ = g.extent

    def sediment_at(x: float, y: float) -> str:
        i = int(np.clip((x - xmin) / g.resolution, 0, g.nx - 1))
        j = int(np.clip((y - ymin) / g.resolution, 0, g.ny - 1))
        return names[int(cls[j, i])]
    objs = [object_truth(o, seabed, sediment_at(o.x, o.y)) for o in scene.objects]
    report, rasters = dom.domain_report(terrain, g, scene.walls, path_xy=path_xy)
    rasters = {"seabed_class": cls, "in_domain": rasters["in_domain"],
               "survey_area": rasters["survey_area"],
               "class_names": np.array(names), "origin_x": float(g.origin_x),
               "origin_y": float(g.origin_y), "resolution": float(g.resolution)}
    snap_seabed = dict(((scene.config_snapshot or {}).get("world") or {}).get("seabed") or {})
    counts: dict[str, dict[str, int]] = {}
    for o, t in zip(scene.objects, objs):
        counts.setdefault(t["role"], {}).setdefault(o.type, 0)
        counts[t["role"]][o.type] += 1
    world: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "domain": report,
        "seabed": {"reference": snap_seabed.get("reference", scene.material_names[0]
                                                if len(scene.material_names) == 1 else ""),
                   "classes": shares, "raster": RASTERS_NAME,
                   **{k: v for k, v in snap_seabed.items() if k not in ("reference",)}},
        "object_counts": counts,
        "detectable_targets": int(sum(1 for t in objs if t["nominal_target"]
                                      and not t["effectively_buried"])),
    }
    for k, v in dict(extras or {}).items():
        world[k] = v
    return world, objs, rasters
