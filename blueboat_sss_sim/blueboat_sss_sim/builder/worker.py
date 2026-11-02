"""Background workers for the builder (Qt threads, pattern from the
Aug-Studio's generation dialog: heavy work never blocks the GUI)."""

from __future__ import annotations

import shutil
import tempfile
import traceback
from pathlib import Path

from PySide6.QtCore import QThread, Signal

from .state import BuilderState


class GenerateWorker(QThread):
    """Runs ``build_world_folder`` off the GUI thread."""

    progressed = Signal(str)
    finished_path = Signal(str)
    failed = Signal(str)

    def __init__(self, state: BuilderState, worlds_root=None,
                 parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._root = worlds_root

    def run(self) -> None:
        try:
            from ..worldgen.world_folder import build_world_folder
            out = build_world_folder(self._state, worlds_root=self._root,
                                     progress=self.progressed.emit)
            self.finished_path.emit(str(out))
        except Exception as exc:
            traceback.print_exc()
            self.failed.emit(str(exc))


class PreviewWorker(QThread):
    """Renders the Screen-5 preview images (objects map + depth map) from
    the current state, into a temp directory, without saving a world."""

    ready = Signal(str, str, str)           # objects_png, depth_png, summary
    failed = Signal(str)

    def __init__(self, state: BuilderState, parent=None) -> None:
        super().__init__(parent)
        self._state = state

    def run(self) -> None:
        try:
            import numpy as np

            from ..worldgen.export_maps import (render_depth_image,
                                                render_objects_preview)
            from ..worldgen.scene import WorldConfig
            from ..worldgen.world_folder import (_generate_with_base,
                                                 build_config)
            self._state.validate_for_build()
            tmp = Path(tempfile.mkdtemp(prefix="world_builder_preview_"))
            raw, _baked = build_config(self._state, tmp)
            scene, pre_stamp = _generate_with_base(
                WorldConfig.from_dict(raw), raw)
            obj_png = render_objects_preview(
                scene, tmp / "objects.png",
                path_xy=self._state.path_world_xy())
            dep_png = render_depth_image(pre_stamp, tmp / "depth.png")
            depth = -pre_stamp
            summary = (f"{len(scene.objects)} objects · "
                       f"{len(scene.walls)} walls · depth "
                       f"{float(depth.min()):.1f}–{float(depth.max()):.1f} m "
                       f"(mean {float(np.mean(depth)):.1f}) · grid "
                       f"{scene.grid.nx}×{scene.grid.ny} @ "
                       f"{scene.grid.resolution:g} m")
            self.ready.emit(str(obj_png), str(dep_png), summary)
        except Exception as exc:
            traceback.print_exc()
            self.failed.emit(str(exc))


# ------------------------------------------------------ Screen-3 preview
#: Coarse preview grid: at most this many cells along the longer axis.
PREVIEW_MAX_CELLS = 400

#: Layer families whose content lives below the coarse preview grid, so a
#: stack carrying one is previewed finer (see ``compute_depth_preview``).
_DETAIL_KINDS = frozenset(("ripples", "dunes", "furrows", "roughness",
                           "pits", "outcrops"))


def _stack_needs_detail(state) -> bool:
    """True when the seabed stack carries relief finer than the coarse
    preview grid -- bedforms, roughness, pits."""
    for layer in (getattr(state.depth, "layers", None) or []):
        kind = layer.get("kind") if isinstance(layer, dict) else getattr(
            layer, "kind", None)
        if kind in _DETAIL_KINDS:
            return True
    tex = getattr(state, "terrain", None)
    dunes = (tex or {}).get("dunes", {}) if isinstance(tex, dict) else {}
    return bool(dunes.get("enabled"))
def compute_depth_preview(state: BuilderState,
                          max_cells: int = PREVIEW_MAX_CELLS) -> dict:
    """The Screen-3 terrain (depth layer + paint + texture) on a coarse
    grid in the *design* frame. Qt-free; the same ``terrain_section`` /
    ``depth_section`` the build uses, so the preview is the world minus
    resolution and objects."""
    import numpy as np

    from ..core.types import GridSpec, Wall
    from ..worldgen.depth import DepthConfig, build_height_with_info
    from ..worldgen.depth.bake import bake_composite
    from ..worldgen.materials import MaterialLibrary
    from ..worldgen.terrain import TerrainConfig, synthesize_terrain
    from ..worldgen.world_folder import depth_section, terrain_section

    r = state.limits_rect
    if not r:
        raise ValueError("no world limits yet (Screen 2)")
    w_m = float(r["x_max"] - r["x_min"])
    h_m = float(r["y_max"] - r["y_min"])
    # A relief layer's whole point is detail the base grid can hold: a
    # 0.5 m ripple is 2 cells at the 0.25 m preview floor, i.e. invisible,
    # so the field could only be judged after a full build. When the stack
    # carries one, preview finer and allow more cells (still well under
    # the build grid's 1500-per-axis cap).
    _fine = _stack_needs_detail(state)
    res = max(0.08 if _fine else 0.25,
              max(w_m, h_m) / (900 if _fine else max_cells))
    grid = GridSpec(float(r["x_min"]), float(r["y_min"]), res,
                    int(w_m / res) + 1, int(h_m / res) + 1)
    from ..core.geometry import expand_walls
    walls = [Wall(**w) for w in expand_walls(state.walls)]   # same as the build
    rng = np.random.default_rng(0)
    overlay, gain = None, None
    if state.depth.layers:
        # The layer stack composed in memory on the preview grid (design
        # frame, offset zero): the same code the build bakes to a GeoTIFF.
        comp = bake_composite(state.depth, grid, walls, state.path.anchor,
                              int(state.objects.seed))
        base, info = comp.height, comp.info
        overlay, gain = comp.material_overlay, comp.gain
    else:
        base, info = build_height_with_info(
            grid, DepthConfig.from_dict(depth_section(state)), walls,
            state.path.anchor, rng)
    materials = MaterialLibrary()
    terrain = synthesize_terrain(
        grid, TerrainConfig.from_dict(terrain_section(state)), materials,
        rng, base_height=base, material_overlay=overlay,
        reflectivity_gain=gain)
    fractions = {name: float((terrain.material_id == i).mean())
                 for i, name in enumerate(terrain.material_names)}
    return {"grid": grid, "height": terrain.height,
            "reflectivity": terrain.reflectivity,
            "material_id": terrain.material_id,
            "material_names": terrain.material_names,
            "material_fractions": fractions,
            # svlog mode: the survey summary + the soundings (design frame,
            # since the preview grid is the design frame) for the map.
            "info": info}


def coverage_line(info: dict | None) -> str:
    """How much of the world the depth source measured, and the warning
    when most of it is extrapolated."""
    cov = (info or {}).get("coverage")
    if not cov:
        return ""
    line = (f"coverage: {100 * cov['area_fraction']:.0f} % of the world area "
            f"within {cov.get('reach_m', 15.0):.0f} m of a sounding, "
            f"{100 * cov['inside_fraction']:.0f} % of {cov['total']} samples "
            "inside the limits")
    warn = (info or {}).get("warning")
    return f"⚠ {warn}\n{line}" if warn else line


def survey_stats_line(info: dict | None) -> str:
    """One line of survey quality for the Screen-3 stats box, plus the
    coverage line (and its warning) when the source is positioned."""
    cov = coverage_line(info)
    if not info or "pings" not in info:
        return cov
    xo = info.get("crossover") or {}
    xo_s = (f"crossover |Δ| {xo['median_abs_m']:.2f} m (n {xo['n']})"
            if xo else "no crossovers")
    line = (f"recording: {info['pings']} pings, "
            f"{100 * info['altitude_fraction']:.0f} % with a bottom return, "
            f"altitude {info['altitude_p5_m']:.1f}–{info['altitude_p95_m']:.1f} m, "
            f"{info['cells']} cells @ {info['cell_m']:.1f} m, {xo_s}, "
            f"clock {info['clock_mode'].replace('_', ' ')}")
    rel = info.get("relief")
    if rel:
        ov = (f"{rel['overlap_rms_m']:.2f} m over {rel['overlap_cells']} cells"
              if rel.get("overlap_rms_m") is not None else "no overlap")
        mode = rel.get("gain_mode", "set")
        # A clipped gain is a bound, not a fit: say so (2026-10-23 -- on the
        # beach corpus the tie soundings give a slope <= 0, and the old line
        # read "calibrated" at the 0.1 floor).
        clip = rel.get("clipped")
        if clip:
            mode += (f"; fit {rel['gain_raw']:+.3f} clipped {clip} -- the tie "
                     "soundings do not support this amplitude")
        line += (f"\nrelief from shading: {100 * rel['coverage_fraction']:.0f} % "
                 f"of the survey hull, rms {rel['rms_m']:.2f} m, passes "
                 f"disagree {ov}, gain ×{rel['gain']:.2f} ({mode}"
                 + (f", {rel['gain_pairs']} tie soundings"
                    if rel.get("gain_pairs") else "") + ")")
    return f"{line}\n{cov}" if cov else line


class DepthPreviewWorker(QThread):
    """Runs ``compute_depth_preview`` off the GUI thread and emits two
    images (depth, reflectivity) plus a stats line."""

    ready = Signal(object, object, str, object)   # QImage x2, stats, info
    failed = Signal(str)

    def __init__(self, state: BuilderState, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self.result: dict | None = None

    def run(self) -> None:
        try:
            import numpy as np

            from .imaging import raster_to_qimage
            res = compute_depth_preview(self._state)
            self.result = res                 # the rasters, for the map overlay
            info = res.get("info") or {}
            depth = -res["height"]
            lo, hi = float(depth.min()), float(depth.max())
            img_depth = raster_to_qimage(depth, lo, hi, invert=True)
            refl = res["reflectivity"]
            img_refl = raster_to_qimage(refl, 0.0,
                                        max(1.0, float(np.nanmax(refl))))
            fr = " · ".join(f"{n} {100 * f:.0f}%"
                            for n, f in res["material_fractions"].items()
                            if f > 0.0005)
            stats = (f"depth {lo:.2f}–{hi:.2f} m (mean "
                     f"{float(depth.mean()):.2f}) · reflectivity "
                     f"{float(refl.mean()):.2f} ± {float(refl.std()):.2f} · "
                     f"{fr} · grid {res['grid'].nx}×{res['grid'].ny} @ "
                     f"{res['grid'].resolution:g} m")
            line = survey_stats_line(info)
            if line:
                stats = stats + "\n" + line
            self.ready.emit(img_depth, img_refl, stats, info)
        except Exception as exc:
            traceback.print_exc()
            self.failed.emit(str(exc))


# ------------------------------------------------------ Screen-5 placement
class PlacementWorker(QThread):
    """Screen 5: runs the real build pipeline once and reports every object
    the current state places, in the **design** frame.

    The same ``build_config`` + ``_generate_with_base`` path the folder
    build uses, so what the screen draws is what the world will contain.
    Objects come back *pre-settle* (``placed_out``) -- rasterising banks a
    rested object onto the seabed slope in place, and pinning that banked
    pose would bank it a second time on the next build.

    Each row is ``{"kind", "item"}`` where ``item`` is a ready
    ``objects.explicit`` entry (design frame) and ``kind`` is:

    * ``scatter`` -- still generated by Screen 4's classes. Context only:
      editing one pins the whole field first.
    * ``pinned``  -- that field already frozen into verbatim instances.
    * ``manual``  -- hand-placed here.
    """

    ready = Signal(object)              # dict
    failed = Signal(str)

    def __init__(self, state: BuilderState, parent=None) -> None:
        super().__init__(parent)
        self._state = state

    def run(self) -> None:
        tmp: Path | None = None
        try:
            from ..worldgen.objects import placed_to_explicit
            from ..worldgen.scene import WorldConfig
            from ..worldgen.world_folder import (_generate_with_base,
                                                 build_config)
            state = self._state
            state.validate_for_build()
            tmp = Path(tempfile.mkdtemp(prefix="world_builder_objects_"))
            raw, _baked = build_config(state, tmp)
            placed: list = []
            scene, pre_stamp = _generate_with_base(
                WorldConfig.from_dict(raw), raw, placed_out=placed)
            ox, oy = state.spawn_origin()

            n_pinned = len(state.objects.placed)
            n_manual = len(state.objects.explicit)
            n_scatter = (0 if (n_pinned or state.objects.pinned)
                         else max(0, len(placed) - n_manual))
            rows = []
            for i, o in enumerate(placed):
                item = placed_to_explicit(o)
                item["x"] += ox                 # world frame -> design frame
                item["y"] += oy
                if i < n_scatter:
                    kind = "scatter"
                elif i < n_pinned:
                    kind = "pinned"
                else:
                    kind = "manual"
                rows.append({"kind": kind, "item": item})

            g = scene.grid
            depth = -pre_stamp
            self.ready.emit({
                "objects": rows,
                "depth": depth,
                # The raster in the design frame, like everything the
                # canvas draws (the build re-bases by the spawn origin).
                "grid": {"origin_x": float(g.origin_x) + ox,
                         "origin_y": float(g.origin_y) + oy,
                         "resolution": float(g.resolution),
                         "nx": int(g.nx), "ny": int(g.ny)},
                "summary": (f"{len(rows)} objects "
                            f"({n_scatter} scattered, {n_pinned} pinned, "
                            f"{len(rows) - n_scatter - n_pinned} hand-placed)"
                            f" · depth {float(depth.min()):.1f}–"
                            f"{float(depth.max()):.1f} m · grid "
                            f"{g.nx}×{g.ny} @ {g.resolution:g} m"),
            })
        except Exception as exc:
            traceback.print_exc()
            self.failed.emit(str(exc))
        finally:
            if tmp is not None:     # the baked depth, one per screen entry
                shutil.rmtree(tmp, ignore_errors=True)


#: A cell at or above this height is dry or awash and scatters nothing
#: back (``worldgen.objects.DRY_HEIGHT_M``, as a positive depth).
DRY_DEPTH_M = 0.05


def _grid_index(grid, x: float, y: float) -> tuple[int, int] | None:
    """(col, row) of a design-frame point on ``grid`` -- a ``GridSpec`` or
    the plain dict a worker result carries -- or ``None`` outside it."""
    if isinstance(grid, dict):
        ox, oy, res = grid["origin_x"], grid["origin_y"], grid["resolution"]
        nx, ny = grid["nx"], grid["ny"]
    else:
        ox, oy, res, nx, ny = (grid.origin_x, grid.origin_y, grid.resolution,
                               grid.nx, grid.ny)
    i = int(round((x - ox) / res))
    j = int(round((y - oy) / res))
    if not (0 <= i < nx and 0 <= j < ny):
        return None
    return i, j


def depth_at(preview: dict | None, x: float, y: float) -> float | None:
    """Water depth [m, positive down] at a design-frame point of a
    :class:`PlacementWorker` result (``depth``) or a
    :func:`compute_depth_preview` result (``height``, negative down);
    ``None`` outside the raster."""
    if not preview:
        return None
    ij = _grid_index(preview["grid"], x, y)
    if ij is None:
        return None
    i, j = ij
    if "depth" in preview:
        return float(preview["depth"][j, i])
    return float(-preview["height"][j, i])


def sample_at(preview: dict | None, x: float, y: float) -> dict | None:
    """Everything a hover readout wants at a design-frame point: the
    depth, the material name when the result carries one, and the cell;
    ``None`` outside the raster."""
    if not preview:
        return None
    ij = _grid_index(preview["grid"], x, y)
    if ij is None:
        return None
    i, j = ij
    out = {"col": i, "row": j, "depth": depth_at(preview, x, y)}
    ids = preview.get("material_id")
    names = preview.get("material_names")
    if ids is not None and names:
        k = int(ids[j, i])
        out["material"] = names[k] if 0 <= k < len(names) else ""
    return out


def hover_text(preview: dict | None, x: float, y: float, anchor=None) -> str:
    """The one-line readout under the map / preview for a design-frame
    point: local metres, the GPS position when anchored, the depth (or
    "dry" at or above the waterline) and the material."""
    s = sample_at(preview, x, y)
    if s is None:
        return ""
    parts = [f"E {x:+.1f} N {y:+.1f} m"]
    if anchor is not None:
        lat, lon = anchor.to_latlon(float(x), float(y))
        parts.append(f"{float(lat):.6f} N {float(lon):.6f} E")
    d = s["depth"]
    if d is None:
        pass
    elif d <= DRY_DEPTH_M:
        parts.append(f"dry (height {-d:+.2f} m)")
    else:
        parts.append(f"depth {d:.2f} m")
    if s.get("material"):
        parts.append(s["material"])
    return " · ".join(parts)


def is_dry(preview: dict | None, x: float, y: float) -> bool:
    """Whether a hand-placed object at this point would sit on land. The
    engine places an authored instance wherever it is pinned (NC #13
    exempts them), so Screen 5 warns rather than refuses."""
    d = depth_at(preview, x, y)
    return d is not None and d <= DRY_DEPTH_M
