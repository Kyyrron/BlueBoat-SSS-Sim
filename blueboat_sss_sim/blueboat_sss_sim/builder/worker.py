"""Background workers for the builder (Qt threads, pattern from the
Aug-Studio's generation dialog: heavy work never blocks the GUI)."""

from __future__ import annotations

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
                                                 state_to_world_config)
            self._state.validate_for_build()
            raw = state_to_world_config(self._state)
            scene, pre_stamp = _generate_with_base(
                WorldConfig.from_dict(raw), raw)
            tmp = Path(tempfile.mkdtemp(prefix="world_builder_preview_"))
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
def compute_depth_preview(state: BuilderState,
                          max_cells: int = PREVIEW_MAX_CELLS) -> dict:
    """The Screen-3 terrain (depth layer + paint + texture) on a coarse
    grid in the *design* frame. Qt-free; the same ``terrain_section`` /
    ``depth_section`` the build uses, so the preview is the world minus
    resolution and objects."""
    import numpy as np

    from ..core.types import GridSpec, Wall
    from ..worldgen.depth import DepthConfig, build_height_with_info
    from ..worldgen.materials import MaterialLibrary
    from ..worldgen.terrain import TerrainConfig, synthesize_terrain
    from ..worldgen.world_folder import depth_section, terrain_section

    r = state.limits_rect
    if not r:
        raise ValueError("no world limits yet (Screen 2)")
    w_m = float(r["x_max"] - r["x_min"])
    h_m = float(r["y_max"] - r["y_min"])
    res = max(0.25, max(w_m, h_m) / max_cells)
    grid = GridSpec(float(r["x_min"]), float(r["y_min"]), res,
                    int(w_m / res) + 1, int(h_m / res) + 1)
    from ..core.geometry import expand_walls
    walls = [Wall(**w) for w in expand_walls(state.walls)]   # same as the build
    rng = np.random.default_rng(0)
    base, info = build_height_with_info(
        grid, DepthConfig.from_dict(depth_section(state)), walls,
        state.path.anchor, rng)
    materials = MaterialLibrary()
    terrain = synthesize_terrain(
        grid, TerrainConfig.from_dict(terrain_section(state)), materials,
        rng, base_height=base)
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
    return f"{line}\n{cov}" if cov else line


class DepthPreviewWorker(QThread):
    """Runs ``compute_depth_preview`` off the GUI thread and emits two
    images (depth, reflectivity) plus a stats line."""

    ready = Signal(object, object, str, object)   # QImage x2, stats, info
    failed = Signal(str)

    def __init__(self, state: BuilderState, parent=None) -> None:
        super().__init__(parent)
        self._state = state

    def run(self) -> None:
        try:
            import numpy as np

            from .imaging import raster_to_qimage
            res = compute_depth_preview(self._state)
            info = res.get("info") or {}
            depth = -res["height"]
            lo, hi = float(depth.min()), float(depth.max())
            img_depth = raster_to_qimage(depth, lo, hi, invert=True)
            img_refl = raster_to_qimage(res["reflectivity"], 0.0, 1.0)
            refl = res["reflectivity"]
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
