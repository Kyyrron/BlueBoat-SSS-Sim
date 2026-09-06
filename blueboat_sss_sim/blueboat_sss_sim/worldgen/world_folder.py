"""World folders: ``~/worlds/<path_name>/<world_name>/``.

The world-builder's output unit. One folder carries everything needed to
open the world in Gazebo and feed the simulated sonar (``world.sdf``,
``seabed.stl``, ``scene.npz``, ``scene_manifest.yaml``), the consumer
contract (``metadata.yaml`` -- GPS ground truth for every object, wall
and limit corner), the design surface (``depth.npz`` -- the terrain-only
height raster actually used, before object stamps), the Screen-5 preview
(``preview_objects.png``), a copy of the depth source file when one was
used, and the complete wizard state (``builder_state.yaml``) that makes
Edit mode possible.

Folders are **immutable**: a name collision -- including re-saving an
edited world under its old name -- silently takes the next free ``_1``,
``_2``, ... suffix and never touches an existing folder (CM-7).
"""

from __future__ import annotations

import copy
import dataclasses
import datetime
import re
import shutil
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Mapping

import numpy as np
import yaml

from ..core.geometry import curve_base_name, expand_walls
from ..core.geo import GeoAnchor
from .depth import build_height, build_height_with_info  # noqa: F401
from .export_maps import render_objects_preview
from .materials import MaterialLibrary
from .objects import normalise_orientation, place_objects, rasterize_objects
from .scene import SceneModel, WorldConfig, generate_scene
from .sdf_writer import write_world_sdf
from .terrain import synthesize_terrain

if TYPE_CHECKING:  # no runtime dependency on the GUI package
    from ..builder.state import BuilderState

DEFAULT_WORLDS_ROOT = Path.home() / "worlds"
METADATA_FORMAT = "blueboat_world_meta/1"
#: Raster resolution [m]; auto-coarsened so no axis exceeds this many cells.
BASE_RESOLUTION_M = 0.10
MAX_CELLS_AXIS = 1500


def _safe_name(name: str) -> str:
    """Filesystem-safe folder name (spaces -> _, path separators stripped)."""
    name = re.sub(r"[^\w.-]+", "_", name.strip()).strip("._")
    return name or "world"


def _dedupe_dir(parent: Path, name: str) -> Path:
    """First free ``name``, ``name_1``, ``name_2``, ... under ``parent``."""
    cand = parent / name
    i = 0
    while cand.exists():
        i += 1
        cand = parent / f"{name}_{i}"
    return cand


def _resolution_for(size_x: float, size_y: float) -> float:
    """BASE_RESOLUTION_M, coarsened in 0.05 m steps for very large worlds."""
    span = max(size_x, size_y)
    res = BASE_RESOLUTION_M
    if span / res > MAX_CELLS_AXIS:
        res = np.ceil(span / MAX_CELLS_AXIS / 0.05) * 0.05
    return float(res)


def state_to_world_config(state: "BuilderState",
                          depth_file: str | None = None) -> dict[str, Any]:
    """BuilderState -> the raw world-YAML dict ``WorldConfig`` consumes.

    The wizard draws in the source path's *design* frame; the generated
    world is **re-based** so that world (0, 0) — where the boat always
    spawns — is the state's ``spawn_origin()`` (the path's first waypoint,
    else the rectangle centre), guaranteed inside the drawn limits. The
    rectangle, walls and explicit objects shift by that offset, and the
    world's GPS anchor becomes the spawn origin's GPS, so every stored
    GPS value stays globally identical.

    ``depth_file`` overrides the depth source path (the folder-local copy
    at build time); the returned dict is also stored verbatim as the
    scene's config snapshot, so provenance survives inside the manifest.
    """
    r = state.limits_rect
    ox, oy = state.spawn_origin()
    size = [float(r["x_max"] - r["x_min"]), float(r["y_max"] - r["y_min"])]
    origin = [float(r["x_min"] - ox), float(r["y_min"] - oy)]
    walls = []
    # Close endpoints merged, curves flattened to chords (core.geometry):
    # the one conversion from builder walls to engine walls, so scene,
    # SDF, depth barriers, metadata and the sonar all see the same segments.
    for w in expand_walls(state.walls):
        for k, d in (("x0", ox), ("y0", oy), ("x1", ox), ("y1", oy)):
            w[k] = float(w[k]) - d
        walls.append(w)
    explicit = []
    for item in state.objects.explicit:
        item = dict(item)
        item["x"] = float(item["x"]) - ox
        item["y"] = float(item["y"]) - oy
        explicit.append(item)
    raw: dict[str, Any] = {
        "world": {
            "seed": int(state.objects.seed),
            "size": size,
            "origin": origin,
            "resolution": _resolution_for(*size),
            "limits_collision": True,
        },
        "geo": state.world_anchor().to_dict(),
        "terrain": terrain_section(state, ox, oy),
        "objects": {"classes": dict(state.objects.classes),
                    "explicit": explicit},
        "walls": walls,
    }
    # The survey path in world metres, so ``orientation.mode:
    # path_relative`` classes can align to the local track. Only written
    # when a class asks for it: the config snapshot of every other world
    # stays byte-identical.
    if any(str((c or {}).get("orientation", {}).get("mode", "random"))
           == "path_relative" for c in state.objects.classes.values()):
        raw["objects"]["path_xy"] = [
            [round(float(x), 3), round(float(y), 3)]
            for x, y in state.path_world_xy()]
    raw["depth"] = depth_section(state, depth_file, ox, oy)
    return raw


def terrain_section(state: "BuilderState", ox: float = 0.0,
                    oy: float = 0.0) -> dict[str, Any]:
    """The world YAML's ``terrain:`` section from the wizard state, with
    material strokes re-based by ``(ox, oy)``. Shared by the build and
    the Screen-3 live preview (which works in the design frame, offset
    zero)."""
    terrain = copy.deepcopy(dict(state.depth.texture or {}))
    mats = dict(terrain.get("materials") or {})
    if not mats:
        # Legacy state (texture without a materials block): one uniform
        # seabed material -- byte-for-byte what the builder always wrote.
        mats = {"layout": "uniform", "uniform": state.depth.material,
                "composition": {}}
    else:
        mats.setdefault("uniform", state.depth.material)
    if state.depth.material_paint:
        mats["paint"] = [_rebase_stroke(s, ox, oy)
                         for s in state.depth.material_paint]
    terrain["materials"] = mats
    return terrain


def depth_section(state: "BuilderState", depth_file: str | None = None,
                  ox: float = 0.0, oy: float = 0.0) -> dict[str, Any]:
    """The world YAML's ``depth:`` section from the wizard state, depth
    strokes re-based by ``(ox, oy)`` (present only when non-empty, so an
    unpainted state's snapshot is unchanged)."""
    if state.depth.mode == "svlog":
        raw = {"mode": "svlog",
               "file": depth_file or state.depth.file,
               "svlog": dict(state.depth.svlog or {})}
    elif state.depth.mode == "file":
        raw = {"mode": "file",
               "file": depth_file or state.depth.file,
               "file_kind": state.depth.file_kind,
               "file_values": state.depth.file_values}
    else:
        raw = {"mode": "preset", "preset": state.depth.preset,
               "params": dict(state.depth.params)}
    if state.depth.paint:
        raw["paint"] = [_rebase_stroke(s, ox, oy) for s in state.depth.paint]
    return raw


def _rebase_stroke(stroke: dict, ox: float, oy: float) -> dict:
    """A paint stroke shifted from the design frame into the world frame
    (world = design - spawn origin), exactly like the walls above."""
    s = dict(stroke)
    s["points"] = [[float(x) - ox, float(y) - oy] for x, y in s["points"]]
    return s


def _generate_with_base(cfg: WorldConfig, raw: dict[str, Any],
                        info_out: dict[str, Any] | None = None
                        ) -> tuple[SceneModel, np.ndarray]:
    """Exactly ``generate_scene``, but also returning the terrain-only
    height raster (pre-object-stamp) -- the "real depth map actually
    used" the folder stores. Kept in lockstep with ``generate_scene``'s
    RNG streams; the smoke test asserts the two produce identical scenes.
    ``info_out`` (a dict) receives the depth source's info -- the survey
    summary of a recording -- when given.
    """
    rng_terrain = np.random.default_rng([cfg.seed, 0])
    rng_objects = np.random.default_rng([cfg.seed, 1])
    grid = cfg.grid()
    materials = MaterialLibrary(cfg.material_overrides or None)
    base = None
    if cfg.depth is not None:
        base, info = build_height_with_info(grid, cfg.depth, cfg.walls,
                                            cfg.geo, rng_terrain)
        if info_out is not None:
            info_out.update(info)
    terrain = synthesize_terrain(grid, cfg.terrain, materials, rng_terrain,
                                 base_height=base)
    pre_stamp = terrain.height.copy()
    objects = place_objects(grid, cfg.objects, materials, rng_objects,
                            height=pre_stamp)
    rasterize_objects(objects, grid, terrain.height, terrain.reflectivity)
    scene = SceneModel(
        grid=grid, height=terrain.height, reflectivity=terrain.reflectivity,
        material_id=terrain.material_id,
        material_names=terrain.material_names, objects=objects,
        walls=list(cfg.walls), seed=cfg.seed, config_snapshot=dict(raw),
        geo_anchor=cfg.geo)
    return scene, pre_stamp


def _plain(d: Mapping[str, Any] | None) -> dict[str, Any]:
    """A YAML-safe copy of a survey summary (arrays dropped, numpy
    scalars to Python)."""
    out: dict[str, Any] = {}
    for k, v in (d or {}).items():
        if k == "soundings_arrays":
            continue
        if isinstance(v, np.generic):
            v = v.item()
        elif isinstance(v, dict):
            v = _plain(v)
        elif isinstance(v, (list, tuple)):
            v = [x.item() if isinstance(x, np.generic) else x for x in v]
        out[k] = v
    return out


def _write_metadata(out: Path, state: "BuilderState", scene: SceneModel,
                    pre_stamp: np.ndarray, world_name: str,
                    survey_info: Mapping[str, Any] | None = None) -> None:
    anchor: GeoAnchor = scene.geo_anchor
    g = scene.grid
    xmin, ymin, xmax, ymax = g.extent

    def gps(x: float, y: float) -> list[float]:
        lat, lon = anchor.to_latlon(x, y)
        return [round(float(lat), 8), round(float(lon), 8)]

    depth_field = -pre_stamp
    ox, oy = state.spawn_origin()
    # Per-object orientation provenance: the class's normalised
    # ``orientation`` block for placed instances; explicit instances are
    # pinned verbatim, which is a fixed heading with no jitter.
    n_explicit = len(state.objects.explicit)
    n_placed = len(scene.objects) - n_explicit

    def orientation_of(o) -> dict[str, Any]:
        if o.object_id >= n_placed:
            return {"mode": "fixed",
                    "heading_deg": round(float(np.degrees(o.yaw)), 2),
                    "jitter_deg": 0.0}
        return normalise_orientation(
            (state.objects.classes.get(o.type) or {}).get("orientation"))

    doc = {
        "format": METADATA_FORMAT,
        "world_name": world_name,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "source_path": {"name": state.path.name, "file": state.path.file,
                        # world = design - design_offset (the spawn origin)
                        "design_offset": [round(float(ox), 3),
                                          round(float(oy), 3)]},
        "geo_anchor": anchor.to_dict(),
        # Where the boat spawns: world (0, 0) by construction, always
        # inside the limits (the frame is re-based to guarantee it).
        "spawn": {"x": 0.0, "y": 0.0,
                  "lat": gps(0.0, 0.0)[0], "lon": gps(0.0, 0.0)[1]},
        "limits": {
            "local": {"x_min": float(xmin), "y_min": float(ymin),
                      "x_max": float(xmax), "y_max": float(ymax)},
            # SW, SE, NE, NW
            "corners_gps": [gps(xmin, ymin), gps(xmax, ymin),
                            gps(xmax, ymax), gps(xmin, ymax)],
        },
        "walls": [{
            "name": w.name,
            "local": [[float(w.x0), float(w.y0)], [float(w.x1), float(w.y1)]],
            "gps": [gps(w.x0, w.y0), gps(w.x1, w.y1)],
            "top_z": float(w.top_z), "reflectivity": float(w.reflectivity),
            "thickness": float(w.thickness), "kind": str(w.kind),
            "draft_m": float(w.draft_m), "roughness_m": float(w.roughness_m),
            # Chord of a curved wall: the builder wall it came from.
            "curve": curve_base_name(w.name),
        } for w in scene.walls],
        "objects": [{
            "object_id": o.object_id, "type": o.type,
            "x": round(float(o.x), 3), "y": round(float(o.y), 3),
            "lat": gps(o.x, o.y)[0], "lon": gps(o.x, o.y)[1],
            "yaw": round(float(o.yaw), 4),
            "yaw_deg": round(float(np.degrees(o.yaw)), 2),
            "roll_deg": round(float(np.degrees(o.roll)), 2),
            "pitch_deg": round(float(np.degrees(o.pitch)), 2),
            "rest": o.rest,
            "orientation": orientation_of(o),
            "length": round(float(o.length), 3),
            "width": round(float(o.width), 3),
            "proud_height": round(float(o.proud_height), 3),
            "burial": round(float(o.burial), 3),
            "fill_fraction": round(float(o.fill_fraction), 3),
            "material": o.material,
        } for o in scene.objects],
        "depth": {
            "mode": state.depth.mode,
            **({"preset": state.depth.preset}
               if state.depth.mode == "preset" else
               {"file": Path(state.depth.file).name}),
            "min_m": round(float(depth_field.min()), 2),
            "max_m": round(float(depth_field.max()), 2),
            "mean_m": round(float(depth_field.mean()), 2),
            **({"svlog": dict(state.depth.svlog or {}),
                "svlog_quality": _plain(survey_info)}
               if state.depth.mode == "svlog" else {}),
        },
        "seabed_material": state.depth.material,
        "seed": int(state.objects.seed),
    }
    with open(out / "metadata.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)


def build_world_folder(state: "BuilderState",
                       worlds_root: str | Path | None = None,
                       progress: Callable[[str], None] | None = None) -> Path:
    """Generate one immutable world folder from the wizard state.

    Returns the created directory (its basename is the final, possibly
    ``_i``-deduped world name).
    """
    def step(msg: str) -> None:
        if progress:
            progress(msg)

    state.validate_for_build()
    root = Path(worlds_root) if worlds_root else DEFAULT_WORLDS_ROOT
    parent = root / _safe_name(state.path.name)
    parent.mkdir(parents=True, exist_ok=True)
    out = _dedupe_dir(parent, _safe_name(state.world_name or "world"))
    out.mkdir()
    world_name = out.name

    depth_copy: str | None = None
    if state.depth.mode in ("file", "svlog"):
        src = Path(state.depth.file)
        # The copy makes the folder self-contained: Edit mode re-reads it
        # even if the original file is gone (a recording can be tens of
        # MB; that is the price of a self-contained, immutable folder).
        depth_copy = str(out / f"depth_source{src.suffix.lower()}")
        step(f"copying depth source ({src.stat().st_size / 1e6:.1f} MB)")
        shutil.copyfile(src, depth_copy)

    step("generating scene")
    raw = state_to_world_config(state, depth_file=depth_copy)
    cfg = WorldConfig.from_dict(raw)
    survey_info: dict[str, Any] = {}
    scene, pre_stamp = _generate_with_base(cfg, raw, survey_info)
    arrays = survey_info.pop("soundings_arrays", None)
    if arrays:
        np.savez_compressed(out / "depth_samples.npz", **arrays)
    if (state.depth.mode == "svlog" and depth_copy
            and bool((state.depth.svlog or {}).get("backscatter_layer"))):
        # The optional reflectivity hint (uncalibrated, relative): kept
        # beside the soundings so Edit mode and analysis can show it
        # without re-reading the recording.
        from .depth.svlog import backscatter_hint
        step("deriving the backscatter hint")
        hint = backscatter_hint(depth_copy, cfg.geo, scene.grid)
        np.savez_compressed(out / "reflectivity_hint.npz",
                            values=hint["values"],
                            origin_x=float(hint["origin"][0]),
                            origin_y=float(hint["origin"][1]),
                            cell_m=float(hint["cell_m"]),
                            n_samples=int(hint["n"]))
        survey_info["backscatter_samples"] = int(hint["n"])

    step("writing world files")
    scene.save(out)
    write_world_sdf(scene, out, material_overrides=None,
                    limits_collision=True)
    g = scene.grid
    np.savez_compressed(out / "depth.npz", height=pre_stamp,
                        origin_x=g.origin_x, origin_y=g.origin_y,
                        resolution=g.resolution)

    step("rendering preview")
    render_objects_preview(scene, out / "preview_objects.png",
                           path_xy=state.path_world_xy())

    # The source path re-expressed in this world's frame: its first
    # waypoint is world (0, 0), the spawn. Directly usable as the dev
    # launch's trajectory (blueboat_trajectory/1 points, no geo_anchor).
    ox, oy = state.spawn_origin()
    world_pts = [[float(t), float(x) - ox, float(y) - oy, float(yaw)]
                 for t, x, y, yaw in state.path.points]
    with open(out / "source_path_world.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump({"format": "blueboat_trajectory/1",
                        "name": state.path.name, "loop": False,
                        "duration_s": (world_pts[-1][0] - world_pts[0][0]
                                       if world_pts else 0.0),
                        "points": world_pts}, f, sort_keys=False,
                       default_flow_style=None)

    step("writing metadata")
    _write_metadata(out, state, scene, pre_stamp, world_name, survey_info)
    saved = dataclasses.replace(state, world_name=world_name)
    if depth_copy:
        saved.depth = dataclasses.replace(
            state.depth, file=Path(depth_copy).name)
    saved.save(out / "builder_state.yaml")
    step("done")
    return out


def load_builder_state(world_dir: str | Path):
    """The Edit-mode entry: the wizard state stored in a world folder,
    with a folder-relative depth source resolved back to an absolute
    path."""
    from ..builder.state import BuilderState

    world_dir = Path(world_dir)
    state = BuilderState.load(world_dir / "builder_state.yaml")
    if state.depth.file and not Path(state.depth.file).is_absolute():
        state.depth.file = str(world_dir / state.depth.file)
    return state


def list_worlds(worlds_root: str | Path | None = None) -> list[dict]:
    """Saved worlds, newest first: [{path_name, world_name, dir, meta}]."""
    root = Path(worlds_root) if worlds_root else DEFAULT_WORLDS_ROOT
    if not root.exists():
        return []
    out = []
    for meta_file in root.glob("*/*/metadata.yaml"):
        try:
            with open(meta_file, "r", encoding="utf-8") as f:
                meta = yaml.safe_load(f) or {}
        except Exception:
            continue
        out.append({"path_name": meta_file.parent.parent.name,
                    "world_name": meta_file.parent.name,
                    "dir": meta_file.parent,
                    "meta": meta})
    out.sort(key=lambda w: str(w["meta"].get("created", "")), reverse=True)
    return out
