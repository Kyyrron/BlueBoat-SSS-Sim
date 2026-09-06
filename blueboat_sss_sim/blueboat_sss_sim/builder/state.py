"""BuilderState -- the wizard's document, Qt-free.

One :class:`BuilderState` instance travels through the wizard pages and is
what :func:`blueboat_sss_sim.worldgen.world_folder.build_world_folder`
consumes. It round-trips to ``builder_state.yaml`` inside every world
folder, which is what makes Edit mode (and duplication) possible: loading
that file restores the wizard exactly as it was configured.

The path's points and anchor are *cached* here at selection time, so an
existing world stays editable even if the MCS trajectory file it came
from is later renamed or deleted.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from ..core.geo import GeoAnchor
from ..worldgen.terrain import texture_defaults

STATE_FORMAT = "blueboat_world_builder_state/1"

#: Concrete quay-wall default (amplitude reflection of concrete in water
#: is ~0.7; the shipped enclosed-basin worlds use 0.65).
DEFAULT_WALL_REFLECTIVITY = 0.65
DEFAULT_WALL_TOP_Z = 1.0
DEFAULT_WALL_THICKNESS = 0.30


@dataclass
class PathState:
    """The MCS anchored path the world is built around (cached copy)."""

    name: str = ""
    file: str = ""
    geo_anchor: dict[str, float] = field(default_factory=dict)
    points: list = field(default_factory=list)      # [[t, x, y, yaw], ...]

    @property
    def anchor(self) -> GeoAnchor | None:
        return GeoAnchor.from_dict(self.geo_anchor or None)

    def xy(self) -> np.ndarray:
        """(N, 2) waypoint positions in world/design metres."""
        if not self.points:
            return np.empty((0, 2))
        return np.asarray(self.points, dtype=np.float64)[:, 1:3]

    @classmethod
    def from_mcs_file(cls, file: str | Path) -> "PathState":
        """Read a ``blueboat_trajectory/1`` runtime YAML; requires a
        ``geo_anchor`` (only GPS-anchored paths can anchor a world)."""
        file = Path(file)
        with open(file, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        ga = doc.get("geo_anchor")
        if not (isinstance(ga, dict) and "lat0" in ga and "lon0" in ga):
            raise ValueError(
                f"{file}: no geo_anchor -- only GPS-anchored paths can "
                "anchor a world (create one in the MCS Pattern Designer "
                "with a GPS origin)")
        pts = doc.get("points")
        if not pts:
            raise ValueError(f"{file}: no points")
        state = cls(name=str(doc.get("name", file.stem)), file=str(file),
                    geo_anchor={k: float(v) for k, v in ga.items()},
                    points=[[float(c) for c in p] for p in pts])
        state.anchor  # validates theta_deg == 0 early
        return state


@dataclass
class DepthState:
    """Screen 3: bathymetry source + seabed material + texture."""

    mode: str = "preset"                     # "preset" | "file" | "svlog"
    preset: str = "harbour_basin"
    params: dict[str, Any] = field(default_factory=dict)
    file: str = ""                           # CSV / GeoTIFF / .svlog source
    file_kind: str = ""                      # "" = from suffix
    file_values: str = "auto"                # auto | depth | elevation
    #: ``svlog`` mode knobs (``worldgen.depth.svlog.SVLOG_DEFAULTS``):
    #: datum_offset_m, trend_length_m, texture_gain, smoothing, cell_m,
    #: backscatter_layer. Absent keys take the defaults.
    svlog: dict[str, Any] = field(default_factory=dict)
    #: Terrain texture layered on top: the nested ``terrain:`` sections
    #: ``dunes`` / ``roughness`` / ``materials`` (see
    #: ``worldgen.terrain.TERRAIN_TEXTURE_SCHEMA``). Older states carry
    #: only ``{dunes: {enabled}, roughness: {amplitude}}``; every absent
    #: key takes the schema default, so they load unchanged.
    texture: dict[str, Any] = field(default_factory=texture_defaults)
    #: The uniform seabed material (kept in step with
    #: ``texture["materials"]["uniform"]``; the metadata's
    #: ``seabed_material``).
    material: str = "sand"
    #: Depth brush strokes in the *design* frame (``worldgen.depth.paint``
    #: stroke dicts); re-based to the world frame at build time like walls.
    paint: list = field(default_factory=list)
    #: Material brush strokes, same frame and shape, value = material name.
    material_paint: list = field(default_factory=list)


@dataclass
class ObjectsState:
    """Screen 4: per-class placement configuration."""

    seed: int = 1
    classes: dict[str, dict[str, Any]] = field(default_factory=dict)
    explicit: list = field(default_factory=list)


def _copy_wall(w: Mapping[str, Any]) -> dict:
    """Wall dict copy that also copies the nested ``ctrl`` point (a
    curved wall's through point), as plain floats for YAML."""
    out = dict(w)
    if out.get("ctrl") is not None:
        out["ctrl"] = [float(out["ctrl"][0]), float(out["ctrl"][1])]
    return out


@dataclass
class BuilderState:
    """Everything the wizard configures; the document Edit mode reloads."""

    path: PathState = field(default_factory=PathState)
    #: World limits rectangle {x_min, y_min, x_max, y_max} in world metres
    #: (the drawn rectangle; becomes the grid extent + invisible borders).
    limits_rect: dict[str, float] | None = None
    #: Reflective walls, Wall-dataclass-shaped mappings.
    walls: list = field(default_factory=list)
    depth: DepthState = field(default_factory=DepthState)
    objects: ObjectsState = field(default_factory=ObjectsState)
    world_name: str = ""

    # ------------------------------------------------------------- I/O
    def to_dict(self) -> dict[str, Any]:
        return {
            "format": STATE_FORMAT,
            "path": dataclasses.asdict(self.path),
            "limits_rect": dict(self.limits_rect) if self.limits_rect else None,
            "walls": [_copy_wall(w) for w in self.walls],
            "depth": dataclasses.asdict(self.depth),
            "objects": dataclasses.asdict(self.objects),
            "world_name": self.world_name,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "BuilderState":
        fmt = d.get("format")
        if fmt != STATE_FORMAT:
            raise ValueError(f"unsupported builder state format {fmt!r} "
                             f"(expected {STATE_FORMAT})")
        return cls(
            path=PathState(**(d.get("path") or {})),
            limits_rect=(dict(d["limits_rect"]) if d.get("limits_rect")
                         else None),
            walls=[_copy_wall(w) for w in (d.get("walls") or [])],
            depth=DepthState(**(d.get("depth") or {})),
            objects=ObjectsState(**(d.get("objects") or {})),
            world_name=str(d.get("world_name", "")),
        )

    def save(self, path: str | Path) -> None:
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False,
                           default_flow_style=None)

    @classmethod
    def load(cls, path: str | Path) -> "BuilderState":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(yaml.safe_load(f) or {})

    # ----------------------------------------------------- world framing
    def spawn_origin(self) -> tuple[float, float]:
        """The design-frame point that becomes world (0, 0).

        The boat always spawns at world (0, 0) (`upload_rov_launch.py` and
        the MCS sim launch both hard-code it), so the world frame is
        re-based at build time to guarantee that point lies inside the
        drawn limits: the path's **first waypoint** (spawn at the survey
        start) when it is inside the rectangle, else the **rectangle
        centre**. Deterministic from the state, so Edit-mode rebuilds
        reproduce the same frame.
        """
        xy = self.path.xy()
        r = self.limits_rect
        if len(xy):
            x0, y0 = float(xy[0, 0]), float(xy[0, 1])
            if r is None or (r["x_min"] < x0 < r["x_max"]
                             and r["y_min"] < y0 < r["y_max"]):
                return (x0, y0)
        if r:
            return (0.5 * (r["x_min"] + r["x_max"]),
                    0.5 * (r["y_min"] + r["y_max"]))
        return (0.0, 0.0)

    def world_anchor(self) -> GeoAnchor | None:
        """The generated world's GPS anchor: the spawn origin's GPS.

        Identical to the path's own anchor when the spawn origin is the
        design origin; otherwise the anchor is translated along with the
        frame, so world (0, 0) keeps meaning "the anchor's location" and
        every GPS <-> world conversion stays one equirectangular call.
        """
        a = self.path.anchor
        if a is None:
            return None
        ox, oy = self.spawn_origin()
        if ox == 0.0 and oy == 0.0:
            return a
        lat, lon = a.to_latlon(ox, oy)
        return GeoAnchor(lat0=float(lat), lon0=float(lon))

    def path_world_xy(self) -> np.ndarray:
        """(N, 2) path waypoints in *world* metres (design minus the
        spawn origin)."""
        xy = self.path.xy()
        if not len(xy):
            return xy
        ox, oy = self.spawn_origin()
        return xy - np.array([ox, oy])

    # ------------------------------------------------------- validation
    def validate_for_build(self) -> None:
        """Raise with an actionable message on anything a build needs."""
        if self.path.anchor is None:
            raise ValueError("no GPS-anchored path selected (Screen 1)")
        if not self.limits_rect:
            raise ValueError("no world-limits rectangle drawn (Screen 2)")
        r = self.limits_rect
        if r["x_max"] - r["x_min"] < 5.0 or r["y_max"] - r["y_min"] < 5.0:
            raise ValueError("world limits rectangle is smaller than 5 m")
        if self.depth.mode in ("file", "svlog") and not self.depth.file:
            raise ValueError(f"depth mode is '{self.depth.mode}' but no "
                             "file was chosen (Screen 3)")
        from ..core.geometry import merge_endpoints
        kept = merge_endpoints(self.walls)
        if len(kept) != len(self.walls):
            gone = {w["name"] for w in self.walls} - {w["name"] for w in kept}
            raise ValueError(f"wall(s) {sorted(gone)} collapse to a point once "
                             "their ends are merged with a neighbour (Screen 2): "
                             "lengthen or delete them")
