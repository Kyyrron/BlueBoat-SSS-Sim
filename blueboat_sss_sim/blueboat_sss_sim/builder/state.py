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

import os

import dataclasses
import hashlib
import json
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
    #: The authored layer stack *above* the source (``worldgen.depth.layers``
    #: dicts, design frame). Empty = the legacy state, serialised exactly as
    #: before; non-empty = the build bakes ``source + paint + layers`` into
    #: a GeoTIFF and a material map (``worldgen.depth.bake``).
    layers: list = field(default_factory=list)


def scatter_signature(seed: int, classes: Mapping[str, Any]) -> str:
    """Stable fingerprint of what Screen 4 configured.

    Screen 5 records it next to a pinned field so a later change to the
    scatter (a count, a distribution, the seed) is detectable: the pinned
    objects no longer correspond to the classes that produced them.
    """
    blob = json.dumps({"seed": int(seed), "classes": classes or {}},
                      sort_keys=True, default=float)
    return hashlib.sha1(blob.encode("utf-8")).hexdigest()[:16]


@dataclass
class ObjectsState:
    """Screens 4 and 5: the scatter configuration and the placed objects."""

    seed: int = 1
    classes: dict[str, dict[str, Any]] = field(default_factory=dict)
    #: Hand-placed instances (Screen 5), design frame, engine-verbatim
    #: (``worldgen.objects`` ``objects.explicit`` entries).
    explicit: list = field(default_factory=list)
    #: The ``classes`` random field **pinned** into verbatim instances, so
    #: every scattered object can be moved and edited on Screen 5. Empty =
    #: the scatter is still live and ``classes`` generates it at build
    #: time, exactly as before this screen existed.
    placed: list = field(default_factory=list)
    #: :func:`scatter_signature` of the seed/classes that produced
    #: ``placed``; a mismatch means Screen 4 has changed since.
    placed_signature: str = ""
    #: The random field was pinned on Screen 5. Kept separately from
    #: ``placed`` so that deleting every pinned object leaves the classes
    #: silent instead of resurrecting the scatter (2026-09-16). False on
    #: every older state, which then behaves exactly as before.
    pinned: bool = False


def _copy_wall(w: Mapping[str, Any]) -> dict:
    """Wall dict copy that also copies the nested ``ctrl`` point (a
    curved wall's through point), as plain floats for YAML."""
    out = dict(w)
    if out.get("ctrl") is not None:
        out["ctrl"] = [float(out["ctrl"][0]), float(out["ctrl"][1])]
    return out


#: Where :meth:`BuilderState.autosave` writes. Under the cache directory,
#: not under ``~/worlds``: it is a crash net, never a world.
AUTOSAVE_NAME = "builder_autosave.yaml"


def autosave_path() -> Path:
    root = os.environ.get("XDG_CACHE_HOME") or (Path.home() / ".cache")
    return Path(root) / "blueboat_sss_sim" / AUTOSAVE_NAME


def load_autosave() -> "BuilderState | None":
    """The autosaved state, or ``None`` when there is none or it is
    unreadable (a stale file must never block the builder from starting)."""
    try:
        p = autosave_path()
        if not p.exists():
            return None
        return BuilderState.load(p)
    except Exception:
        return None


def clear_autosave() -> None:
    try:
        autosave_path().unlink()
    except OSError:
        pass


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

    # ------------------------------------------------------- autosave
    def autosave(self, path: str | Path | None = None) -> Path | None:
        """Write the wizard state to the autosave file, atomically.

        A world's ``builder_state.yaml`` is only written when the build
        succeeds, so until then everything the operator has drawn -- the
        limits, the walls, a whole seabed layer stack, hand-placed objects
        -- lives in memory alone. A process death loses all of it, and the
        World Builder *has* been killed outright by the OOM killer while
        previewing a large recording (2026-09-18). This costs a few
        milliseconds per page change and makes that survivable.

        Never raises: an autosave that fails must not take the session
        down with it."""
        try:
            p = Path(path) if path else autosave_path()
            p.parent.mkdir(parents=True, exist_ok=True)
            tmp = p.with_suffix(p.suffix + ".tmp")
            self.save(tmp)
            tmp.replace(p)
            return p
        except Exception:
            return None

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
        if self.depth.layers:
            from ..worldgen.depth.compose import stack_from_depth
            from ..worldgen.depth.layers import normalize_layers
            normalize_layers(stack_from_depth(self.depth))
        from ..core.geometry import merge_endpoints
        kept = merge_endpoints(self.walls)
        if len(kept) != len(self.walls):
            gone = {w["name"] for w in self.walls} - {w["name"] for w in kept}
            raise ValueError(f"wall(s) {sorted(gone)} collapse to a point once "
                             "their ends are merged with a neighbour (Screen 2): "
                             "lengthen or delete them")
