"""Offscreen GUI smoke test for the world builder.

Not part of the gated offline suite (`test/smoke_test.py`): it needs
PySide6 and a display stub. Run with:

    QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke

Covers widget construction, the page enter/commit protocol against a
scripted BuilderState, the map canvas data API, and a synchronous run of
the preview worker (the same code path Screen 5 uses).
"""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

import numpy as np  # noqa: E402
import yaml  # noqa: E402


def check(name: str, cond: bool, detail: str = "") -> None:
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {name}" + (f" -- {detail}" if detail else ""))
    if not cond:
        raise SystemExit(f"gui smoke failed at: {name}")


def main() -> None:
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print("SKIP: PySide6 not installed")
        return

    from blueboat_sss_sim.builder import theme
    from blueboat_sss_sim.builder.forms import SchemaForm, parse_latlon
    from blueboat_sss_sim.builder.map.canvas import BuilderMapView, Mode
    from blueboat_sss_sim.builder.pages.depth_page import DepthPage
    from blueboat_sss_sim.builder.pages.layout_page import LayoutPage
    from blueboat_sss_sim.builder.pages.objects_page import ObjectsPage
    from blueboat_sss_sim.builder.state import (BuilderState, DepthState,
                                                ObjectsState, PathState)
    from blueboat_sss_sim.builder.wizard import WorldBuilderWindow
    from blueboat_sss_sim.builder.worker import (DepthPreviewWorker,
                                                 PreviewWorker)
    from blueboat_sss_sim.worldgen.terrain import texture_defaults
    from blueboat_sss_sim.core.geo import GeoAnchor

    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)

    print("[g1] forms")
    form = SchemaForm([("a", "A", "float", 1.5, 0.0, 10.0),
                       ("b", "B", "int", 3, 0, 9),
                       ("c", "C", "bool", True, 0, 1),
                       ("d", "D", "choice:x|y|z", "y", None, None)])
    check("schema form round-trips every kind",
          form.values() == {"a": 1.5, "b": 3, "c": True, "d": "y"})
    check("parse_latlon accepts Google-Maps style",
          parse_latlon("33.930824, 130.7311812") == (33.930824, 130.7311812)
          and parse_latlon("garbage") is None)

    print("[g2] map canvas data API")
    view = BuilderMapView()
    view.resize(640, 480)
    anchor = GeoAnchor(lat0=33.930824, lon0=130.7311812)
    view.set_anchor(anchor)
    view.set_path(np.array([[0.0, 0.0], [30.0, 30.0], [60.0, 30.0]]))
    view.set_limits({"x_min": -10.0, "y_min": -10.0,
                     "x_max": 45.0, "y_max": 35.0})
    view.set_walls([{"name": "quay", "x0": -10.0, "y0": 34.0,
                     "x1": 45.0, "y1": 34.0, "top_z": 1.0,
                     "reflectivity": 0.65, "thickness": 0.3}])
    check("canvas round-trips limits and walls",
          view.limits()["x_max"] == 45.0
          and view.walls()[0]["name"] == "quay")
    # Curved walls + endpoint snapping (2026-09-03).
    from PySide6.QtCore import QPointF
    view.set_walls([{"name": "quay", "x0": -10.0, "y0": 34.0,
                     "x1": 45.0, "y1": 34.0, "top_z": 1.0,
                     "reflectivity": 0.65, "thickness": 0.3,
                     "ctrl": [17.5, 30.0]}])
    got = view.walls()
    got[0]["ctrl"][0] = 99.0
    check("a curved wall's ctrl round-trips as a copy and draws as a curve",
          view.walls()[0]["ctrl"] == [17.5, 30.0]
          and not view._wall_items[0].path().isEmpty()
          and view._wall_items[0].path().elementCount() >= 3
          and 0 in view._ctrl_handles and not view._ctrl_handles[0].isVisible())
    view._wall_items[0].setSelected(True)
    check("selecting a wall shows its bend handle at the through point",
          view._ctrl_handles[0].isVisible()
          and abs(view._ctrl_handles[0].pos().x() - 17.5) < 1e-9)
    view.straighten_selected_walls()
    check("Straighten removes the bend", "ctrl" not in view.walls()[0])
    view.set_mode(Mode.WALL)
    view._on_wall_click(QPointF(44.6, 34.5))       # within 0.75 m of quay's end
    view._on_wall_click(QPointF(45.0, 10.0))
    w2 = view.walls()[1]
    check("a new wall snaps its end onto the existing endpoint",
          (w2["x0"], w2["y0"]) == (45.0, 34.0) and len(view.walls()) == 2)
    snapped, hit = view.snap_to_endpoint(QPointF(45.3, 9.8), exclude=None)
    check("snap helper finds the nearest endpoint within the radius",
          hit and (snapped.x(), snapped.y()) == (45.0, 10.0)
          and not view.snap_to_endpoint(QPointF(30.0, 20.0))[1])
    view.set_mode(Mode.SELECT)
    strokes_d = [{"points": [[0.0, 0.0], [10.0, 5.0]], "radius_m": 2.0,
                  "value": 4.5, "erase": False}]
    strokes_m = [{"points": [[20.0, 20.0]], "radius_m": 3.0, "value": "mud",
                  "erase": False}]
    view.set_strokes(strokes_d, "depth")
    view.set_strokes(strokes_m, "material")
    check("canvas round-trips depth and material strokes",
          view.strokes("depth") == strokes_d
          and view.strokes("material") == strokes_m)
    view.set_mode(Mode.DEPTH_BRUSH)
    layer_d = view.brush_layer
    view.set_mode(Mode.MATERIAL_BRUSH)
    view.set_mode(Mode.ERASER)
    check("brush modes select the layer the eraser targets",
          layer_d == "depth" and view.brush_layer == "material"
          and view.mode is Mode.ERASER)
    view.undo_stroke("material")
    view.set_brush(radius_m=999.0)
    check("undo pops one layer only; brush radius clamps",
          view.strokes("material") == [] and view.strokes("depth") == strokes_d
          and view.brush_radius == 50.0)
    view.set_geometry_editable(False)
    view.set_mode(Mode.WALL)
    check("mode switch works", view.mode is Mode.WALL)
    view.set_mode(Mode.SELECT)
    view.center_on_bounds([-10.0, 45.0], [-10.0, 35.0])

    print("[g3] page protocol against a scripted state")
    tdir = Path(tempfile.mkdtemp(prefix="gui_smoke_"))
    mcs_file = tdir / "beach.yaml"
    mcs_file.write_text(yaml.safe_dump({
        "format": "blueboat_trajectory/1", "name": "beach", "speed": 0.8,
        "loop": False,
        "geo_anchor": {"lat0": anchor.lat0, "lon0": anchor.lon0,
                       "theta_deg": 0.0},
        "points": [[0.0, 0.0, 0.0, 0.0], [30.0, 30.0, 0.0, 0.0],
                   [60.0, 30.0, 20.0, 1.57]]}))
    tex_in = texture_defaults()
    tex_in["dunes"]["enabled"] = True
    tex_in["dunes"]["wavelength"] = 8.0
    tex_in["materials"]["uniform"] = "gravel"
    paint_in = [{"points": [[5.0, 5.0], [15.0, 8.0]], "radius_m": 2.5,
                 "value": 4.0, "erase": False}]
    mpaint_in = [{"points": [[30.0, 20.0]], "radius_m": 3.0, "value": "mud",
                  "erase": False}]
    state = BuilderState(
        path=PathState.from_mcs_file(mcs_file),
        limits_rect={"x_min": -10.0, "y_min": -10.0,
                     "x_max": 45.0, "y_max": 35.0},
        walls=[{"name": "quay", "x0": -10.0, "y0": 34.0, "x1": 45.0,
                "y1": 34.0, "top_z": 1.0, "reflectivity": 0.65,
                "thickness": 0.3}],
        depth=DepthState(mode="preset", preset="canal",
                         params={"channel_depth": 2.5},
                         texture=tex_in, paint=paint_in,
                         material_paint=mpaint_in),
        objects=ObjectsState(seed=5, classes={
            "drum_200l": {"amount": {"kind": "count", "value": 2}}}),
        world_name="gui_smoke")

    layout = LayoutPage()
    layout.enter(state)
    check("layout page loads and commits the drawn geometry",
          layout.commit(state) and state.limits_rect["x_max"] == 45.0
          and len(state.walls) == 1)
    check("spawn origin resolves to the path start (world re-basing)",
          state.spawn_origin() == (0.0, 0.0)
          and state.world_anchor() == state.path.anchor)

    depth = DepthPage()
    depth.enter(state)
    check("depth page restores the stored preset and params",
          depth.commit(state) and state.depth.preset == "canal"
          and state.depth.params["channel_depth"] == 2.5)
    check("depth page preserves paint strokes and the texture dict",
          state.depth.paint == paint_in
          and state.depth.material_paint == mpaint_in
          and state.depth.texture == tex_in
          and state.depth.material == "gravel")
    # Brush toolbar: negative depth (dry land) and the per-stroke feather.
    depth._brush_depth.setValue(-0.5)
    depth._brush_feather.setValue(0.75)
    depth.map.set_mode(Mode.DEPTH_BRUSH)
    depth.map._begin_stroke(QPointF(1.0, 1.0))
    depth.map._end_stroke()
    last = depth.map.strokes("depth")[-1]
    check("depth brush accepts a dry-land value and stamps its feather",
          depth._brush_depth.value() == -0.5 and last["value"] == -0.5
          and last["feather_m"] == 0.75 and last["radius_m"] == 2.0)
    depth.map.undo_stroke("depth")
    # The recording source round-trips through enter()/commit().
    svlog_state = BuilderState(
        path=state.path, limits_rect=dict(state.limits_rect),
        depth=DepthState(mode="svlog", file=str(tdir / "run.svlog"),
                         svlog={"datum_offset_m": 0.3, "trend_length_m": 20.0,
                                "texture_gain": 0.0, "smoothing": 2.0,
                                "backscatter_layer": True}),
        world_name="svlog_gui")
    depth_s = DepthPage()
    depth_s.enter(svlog_state)
    check("depth page selects the recording source and restores its knobs",
          depth_s._r_svlog.isChecked()
          and depth_s._svlog_edit.text() == str(tdir / "run.svlog")
          and depth_s._svlog_spins["datum_offset_m"].value() == 0.3
          and depth_s._svlog_spins["trend_length_m"].value() == 20.0
          and depth_s._svlog_backscatter.isChecked())
    (tdir / "run.svlog").write_bytes(b"")
    d_back = depth_s._collect(svlog_state)
    check("depth page emits the recording mode with its knobs",
          depth_s.commit(svlog_state) and d_back.mode == "svlog"
          and d_back.svlog["smoothing"] == 2.0
          and d_back.svlog["backscatter_layer"] is True
          and svlog_state.depth.mode == "svlog")
    # A stale quality line under a failure is the one thing that misleads:
    # the previous recording's stats next to "does not overlap".
    depth_s._svlog_quality.setText("recording: 200 pings, 100 % locked")
    depth_s.map.set_track(np.array([1.0]), np.array([1.0]), np.array([2.0]))
    depth_s._fail("no depth sample lands inside the world limits -- the "
                  "recording and the drawn rectangle do not overlap.")
    check("a failed preview replaces the recording's quality line with the "
          "reason and clears the track",
          "do not overlap" in depth_s._svlog_quality.text()
          and "200 pings" not in depth_s._svlog_quality.text()
          and not depth_s.map._track_items)
    # The coverage line reaches the label on a good preview.
    depth_s._show_survey({"coverage": {"total": 12, "inside": 9,
                                       "inside_fraction": 0.75,
                                       "area_fraction": 0.2,
                                       "reach_m": 15.0},
                          "warning": "the recording reaches 20 % of the "
                                     "world area; the rest is extrapolated"})
    check("the coverage line and its warning reach the quality label",
          "coverage:" in depth_s._svlog_quality.text()
          and "extrapolated" in depth_s._svlog_quality.text())
    depth_s.map.set_track(np.array([0.0, 5.0]), np.array([0.0, 5.0]),
                          np.array([2.0, 3.0]))
    depth_s.map.set_overlay({"values": np.array([[0.2, 0.8], [0.5, np.nan]]),
                             "origin": (0.0, 0.0), "cell_m": 2.0})
    n_items = len(depth_s.map._track_items)
    depth_s.map.set_track(None)
    depth_s.map.set_overlay(None)
    check("map shows and clears the sounding track and the hint overlay",
          n_items == 2 and depth_s.map._track_items == []
          and depth_s.map._overlay_item is None)

    objects = ObjectsPage()
    objects.enter(state)
    check("objects page restores and re-emits the class config",
          objects.commit(state)
          and state.objects.classes["drum_200l"]["amount"]["value"] == 2.0
          and state.objects.seed == 5)
    check("objects page: no orientation key emitted for the default (random)",
          "orientation" not in state.objects.classes["drum_200l"])
    row_on = objects._rows["drum_200l"]
    row_off = objects._rows["brick"]
    check("an unchecked row is collapsed: no parameter widgets exposed",
          row_off._body.isHidden() and not row_on._body.isHidden()
          and not row_off.isChecked() and row_on.isChecked())
    objects._selected_only.setChecked(True)
    check("'Selected only' hides the unchecked rows",
          row_off.isHidden() and not row_on.isHidden())
    objects._selected_only.setChecked(False)
    objects._filter.setText("brick")
    check("text filter keeps only the matching classes",
          not row_off.isHidden() and row_on.isHidden())
    objects._filter.setText("")
    check("clearing the filter shows every row again",
          not row_off.isHidden() and not row_on.isHidden())
    check("orientation spins are disabled under random",
          not row_on.orient.heading.isEnabled()
          and not row_on.orient.jitter.isEnabled()
          and row_on._orient_label.text() == "Heading [deg]")
    row_on.orient.mode.setCurrentText("path relative")
    row_on.orient.heading.setValue(90.0)
    row_on.orient.jitter.setValue(0.0)
    check("path-relative mode relabels the heading as a track offset",
          row_on.orient.heading.isEnabled()
          and row_on._orient_label.text() == "Offset from track [deg]")
    check("objects page commits the orientation block",
          objects.commit(state)
          and state.objects.classes["drum_200l"]["orientation"]
          == {"mode": "path_relative", "heading_deg": 90.0,
              "jitter_deg": 0.0})
    objects2 = ObjectsPage()
    objects2.enter(state)
    r2 = objects2._rows["drum_200l"]
    check("orientation round-trips through enter()",
          r2.orient.mode_key() == "path_relative"
          and r2.orient.heading.value() == 90.0
          and r2.orient.jitter.value() == 0.0
          and objects2.commit(state)
          and state.objects.classes["drum_200l"]["orientation"]["mode"]
          == "path_relative")

    r2.rest.enabled.setChecked(True)
    r2.rest.randomness.setValue(0.8)
    objects2.commit(state)
    objects3 = ObjectsPage()
    objects3.enter(state)
    r3 = objects3._rows["drum_200l"]
    check("3-D rest toggle round-trips inside the orientation block",
          state.objects.classes["drum_200l"]["orientation"]["rest"]
          == {"enabled": True, "randomness": 0.8}
          and r3.rest.enabled.isChecked() and r3.rest.randomness.value() == 0.8
          and not objects3._rows["rope"].rest.enabled.isEnabled())

    from blueboat_sss_sim.builder.pages.objects_page import ClassInfoDialog
    ok_info = True
    for cls in ("drum_200l", "mesh_crate", "rope", "traffic_cone"):
        dlg = ClassInfoDialog(cls)
        ok_info &= dlg.windowTitle().startswith("About:")
        dlg.deleteLater()
    check("class Info dialog builds (size, shape, footprint preview)",
          ok_info, "solid, lattice, line and cone classes")

    print("[g4] preview worker (synchronous run)")
    results: dict = {}
    worker = PreviewWorker(state)
    worker.ready.connect(lambda o, d, s: results.update(
        obj=o, dep=d, summary=s))
    worker.failed.connect(lambda m: results.update(fail=m))
    worker.run()                      # synchronous: no thread, deterministic
    check("preview renders both maps",
          "fail" not in results and Path(results["obj"]).exists()
          and Path(results["dep"]).exists(),
          results.get("summary", results.get("fail", "")))
    dres: dict = {}
    dworker = DepthPreviewWorker(state)
    dworker.ready.connect(lambda a, b, s, info: dres.update(
        depth=a, refl=b, stats=s, info=info))
    dworker.failed.connect(lambda m: dres.update(fail=m))
    dworker.run()                     # synchronous, like PreviewWorker above
    from PySide6.QtWidgets import QWidget
    from blueboat_sss_sim.builder.forms import SchemaForm as _SF
    from blueboat_sss_sim.worldgen.terrain import TERRAIN_TEXTURE_SCHEMA as _TTS
    _form = _SF(list(_TTS))
    _tips = [w.toolTip() for w in _form.findChildren(QWidget)
             if w.toolTip()]
    check("every seabed-texture row shows a hover help",
          len(_tips) >= 2 * len(_TTS) and all(t.endswith(".") for t in _tips),
          f"{len(_tips)} tooltips for {len(_TTS)} rows")
    check("depth preview worker renders depth and reflectivity (no sonar strip)",
          "fail" not in dres
          and all(not dres[k].isNull() for k in ("depth", "refl"))
          and "sonar" not in dres and "mud" in dres["stats"]
          and dres.get("info") == {},
          dres.get("stats", dres.get("fail", "")))

    print("[g5] wizard shells")
    win_create = WorldBuilderWindow(state=BuilderState(), edit_mode=False)
    check("create wizard has 5 screens", len(win_create.pages) == 5)
    win_edit = WorldBuilderWindow(state=state, edit_mode=True)
    check("edit wizard skips Screen 1 and starts on the map",
          len(win_edit.pages) == 4
          and win_edit.pages[0].TITLE.startswith("World limits"))

    print("\nGUI SMOKE PASSED")


if __name__ == "__main__":
    main()
