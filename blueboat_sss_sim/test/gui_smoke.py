"""Offscreen GUI smoke test for the world builder.

Not part of the gated offline suite (`test/smoke_test.py`): it needs
PySide6 and a display stub. Run with:

    QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke

Covers widget construction, the page enter/commit protocol against a
scripted BuilderState, the map canvas data API, and a synchronous run of
the preview worker (the same code path Screen 5 uses).
"""

from __future__ import annotations

import copy
import dataclasses
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
    from blueboat_sss_sim.worldgen.depth.layers.survey import SURVEY_LAYER_ID
    check("the recording's reflectivity flag becomes the tagged survey layer "
          "on load (an older state gets the layer that does the job now)",
          depth_s._doc.ids() == [SURVEY_LAYER_ID]
          and depth_s._doc.layers[0]["kind"] == "reflectivity_survey")
    depth_s._svlog_backscatter.setChecked(False)
    gone = depth_s._doc.ids()
    depth_s._svlog_backscatter.setChecked(True)
    back = depth_s._doc.ids()
    check("unticking removes the survey layer, ticking adds exactly one back "
          "and shows the reflectivity overlay",
          gone == [] and back == [SURVEY_LAYER_ID]
          and depth_s._overlay.currentText() == "reflectivity")
    depth_s._r_preset.setChecked(True)
    after_switch = (depth_s._doc.ids(), depth_s._svlog_backscatter.isChecked())
    depth_s._r_svlog.setChecked(True)
    check("switching the depth source away removes the survey layer and "
          "unticks the box (the document is the one source of truth)",
          after_switch == ([], False) and depth_s._doc.ids() == [])
    depth_s._undo.undo()
    check("undo restores the survey layer and re-ticks the box",
          depth_s._doc.ids() == [SURVEY_LAYER_ID]
          and depth_s._svlog_backscatter.isChecked())
    depth_s._panel.select(SURVEY_LAYER_ID)
    depth_s._remove_layer(SURVEY_LAYER_ID)
    check("deleting the row from the panel unticks the box too",
          depth_s._doc.ids() == [] and not depth_s._svlog_backscatter.isChecked())
    depth_s._svlog_backscatter.setChecked(True)
    (tdir / "run.svlog").write_bytes(b"")
    d_back = depth_s._collect(svlog_state)
    check("depth page emits the recording mode with its knobs",
          depth_s.commit(svlog_state) and d_back.mode == "svlog"
          and d_back.svlog["smoothing"] == 2.0
          and d_back.svlog["backscatter_layer"] is True
          and [L["kind"] for L in d_back.layers] == ["reflectivity_survey"]
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
    # Orientation pin: a raster overlay's row 0 is the min-y (south) row
    # and must be drawn at the origin, rows growing northward -- the same
    # convention as every producer (GridSpec) and the brush strokes.
    ox3, oy3, c3 = 10.0, 100.0, 2.0
    depth_s.map.set_overlay({"values": np.array([[0.1, 0.2, 0.3, 0.4],
                                                 [0.5, 0.6, 0.7, 0.8],
                                                 [0.9, 1.0, 0.0, 0.5]]),
                             "origin": (ox3, oy3), "cell_m": c3})
    it3 = depth_s.map._overlay_items["hint"]
    p00 = it3.mapToScene(QPointF(0.5, 0.5))          # pixel (col 0, row 0)
    p32 = it3.mapToScene(QPointF(3.5, 2.5))          # pixel (col 3, row 2)
    check("a raster overlay draws row 0 at the origin and rows northward "
          "(no N-S mirror against the brush)",
          abs(p00.x() - ox3) < 1e-6 and abs(p00.y() - oy3) < 1e-6
          and abs(p32.x() - (ox3 + 3 * c3)) < 1e-6
          and abs(p32.y() - (oy3 + 2 * c3)) < 1e-6,
          f"row0 -> ({p00.x():.2f}, {p00.y():.2f}), "
          f"(3,2) -> ({p32.x():.2f}, {p32.y():.2f})")
    depth_s.map.set_overlay(None)

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

    print("[g6] layer editor: panel, properties, geometry tools, undo, overlay")
    from PySide6.QtCore import Qt
    from blueboat_sss_sim.builder.layers.document import LayerDocument
    from blueboat_sss_sim.worldgen.depth.layers import LAYER_KINDS
    lay_state = BuilderState(
        path=state.path, limits_rect=dict(state.limits_rect),
        walls=[dict(w) for w in state.walls],
        depth=DepthState(mode="preset", preset="flat", params={"depth": 3.0},
                         paint=[dict(p) for p in paint_in]),
        world_name="layers_gui")
    dl = DepthPage()
    dl.enter(lay_state)
    keys = [dl._panel.list.item(i).data(Qt.ItemDataRole.UserRole)
            for i in range(dl._panel.list.count())]
    check("a legacy state loads as the three fixed rows and no layer",
          keys == ["texture", "brush", "source"] and dl._doc.layers == []
          and dl._collect(lay_state).layers == []
          and dl._collect(lay_state).paint == paint_in)
    dl._add_layer("basin")
    check("adding a landform selects it, opens its property page and starts "
          "the polygon tool",
          dl._doc.layers[0]["kind"] == "basin" and dl._selected_key == "L1"
          and dl._prop_stack.currentIndex() == 3 and dl.map.mode is Mode.POLYGON
          and dl._geom_target == ("geometry", "L1"))
    dl.map.commit_geometry({"type": "polygon",
                            "points": [[5.0, 5.0], [25.0, 5.0], [25.0, 20.0],
                                       [5.0, 20.0]]})
    d_col = dl._collect(lay_state)
    check("a committed polygon lands in the layer, the view returns to "
          "SELECT and the state carries the layer with its base paint",
          dl._doc.layers[0]["geometry"]["type"] == "polygon"
          and len(dl._doc.layers[0]["geometry"]["points"]) == 4
          and dl.map.mode is Mode.SELECT and len(d_col.layers) == 1
          and d_col.paint == paint_in and d_col.mode == "preset")
    geoms = dl.map._layer_geoms
    check("the selected layer's geometry is drawn with vertex handles",
          [g[0] for g in geoms] == ["L1"] and geoms[0][2] is True
          and sum(1 for h in dl.map._handles if h.role[0] == "layer") == 4)
    idx0 = dl._undo.index()
    dl._props.opacity.setValue(0.5)
    dl._props.blend.setCurrentText("normal")
    check("property edits reach the layer and the panel row",
          dl._doc.layers[0]["opacity"] == 0.5
          and dl._doc.layers[0]["blend"] == "normal"
          and "50 %" in dl._panel.list.item(0).text())
    n_cmds = dl._undo.count()
    while dl._undo.index() > idx0:
        dl._undo.undo()
    op_after_undo = dl._doc.layers[0]["opacity"]
    blend_after_undo = dl._doc.layers[0]["blend"]
    while dl._undo.index() < n_cmds:
        dl._undo.redo()
    check("undo and redo walk the document snapshots",
          n_cmds > idx0 >= 2 and op_after_undo == 1.0
          and blend_after_undo == "deepen" and dl._doc.layers[0]["opacity"] == 0.5
          and dl._doc.layers[0]["blend"] == "normal")
    dl._props.mask_kind.setCurrentIndex(dl._props.mask_kind.findData("rect"))
    check("choosing a shaped mask kind starts its drawing tool",
          dl.map.mode is Mode.RECT_GEOM and dl._geom_target == ("mask", "L1", "rect"))
    dl.map.commit_geometry({"type": "rect", "x_min": 0.0, "y_min": 0.0,
                            "x_max": 30.0, "y_max": 30.0})
    m = dl._doc.layers[0]["mask"]
    check("the drawn rectangle becomes the layer's mask and is displayed",
          m["kind"] == "rect" and m["x_max"] == 30.0
          and any(g[0] == "L1:mask" for g in dl.map._layer_geoms))
    dl._add_layer("paint")
    dl.map.set_mode(Mode.DEPTH_BRUSH)
    dl.map._begin_stroke(QPointF(2.0, 2.0))
    dl.map._end_stroke()
    d_col = dl._collect(lay_state)
    pl = [L for L in d_col.layers if L["kind"] == "paint"][0]
    check("a brush-strokes layer takes the depth brush while selected; the "
          "base strokes stay put",
          len(pl["geometry"]["strokes"]) == 1 and d_col.paint == paint_in)
    dl._panel.select("brush")
    check("selecting the base brush row shows the base strokes on the map",
          dl._selected_key == "brush" and dl.map.strokes("depth") == paint_in
          and dl._prop_stack.currentIndex() == 1)
    dl._panel.select("L1")
    dl._start_mask_brush()
    dl.map._begin_stroke(QPointF(10.0, 10.0))
    dl.map._end_stroke()
    check("the mask brush paints into the selected layer's brush mask",
          dl.map.mode is Mode.MASK_BRUSH
          and dl._doc.layers[0]["mask"]["kind"] == "brush"
          and len(dl._doc.layers[0]["mask"]["strokes"]) == 1)
    dl.map.set_mode(Mode.SELECT)
    if dl._recipes:
        before = len(dl._doc.layers)
        dl._add_recipe(next(iter(dl._recipes)))
        check("a recipe appends its layers over the source",
              len(dl._doc.layers) > before and dl.commit(lay_state) is True
              and len(lay_state.depth.layers) == len(dl._doc.layers))
    dres2: dict = {}
    w2 = DepthPreviewWorker(dataclasses.replace(lay_state, depth=dl._collect(lay_state)))
    w2.ready.connect(lambda a, b, s_, info: dres2.update(depth=a, stats=s_))
    w2.failed.connect(lambda msg: dres2.update(fail=msg))
    w2.run()
    check("the layered state previews (composed in memory) and keeps its result",
          "fail" not in dres2 and w2.result is not None
          and "height" in w2.result, dres2.get("fail", ""))
    dl._worker = w2
    dl._show(dres2["depth"], dres2["depth"], dres2["stats"], {})
    check("the composite overlay is drawn on the map from the preview result",
          "composite" in dl.map._overlay_items)
    # The paint map and the preview picture must agree with the raster at
    # the same design point: sample three cells of distinct depth through
    # the overlay item (scene position -> pixel) and through the preview
    # QImage (north-up pixel), and require the same depth ordering.
    g6 = w2.result["grid"]
    dep6 = -w2.result["height"]
    it6 = dl.map._overlay_items["composite"]
    img6 = it6.pixmap().toImage()
    prev6 = dres2["depth"]
    check("the layered preview raster is finite everywhere (a landform's "
          "NaN beyond its reach stays out of the composite)",
          bool(np.isfinite(dep6).all()),
          f"{int(np.isnan(dep6).sum())} NaN cells")
    order6 = np.argsort(dep6.ravel())
    cells6 = [np.unravel_index(order6[k], dep6.shape)
              for k in (0, dep6.size // 2, dep6.size - 1)]
    ov6, pv6, dp6 = [], [], []
    for (j6, i6) in cells6:
        sp6 = QPointF(g6.origin_x + i6 * g6.resolution,
                      g6.origin_y + j6 * g6.resolution)
        px6 = it6.mapFromScene(sp6)
        ov6.append(img6.pixelColor(int(px6.x()), int(px6.y())).red())
        pv6.append(prev6.pixelColor(int(i6), int(g6.ny - 1 - j6)).red())
        dp6.append(float(dep6[j6, i6]))
    check("the map overlay and the preview picture agree with the raster at "
          "the same design points (shallow bright, deep dark, both)",
          dp6[0] < dp6[1] < dp6[2] and ov6[0] > ov6[1] > ov6[2]
          and pv6[0] > pv6[1] > pv6[2],
          f"depth {dp6} overlay {ov6} preview {pv6}")
    # Hover readout: the map and a preview picture name the same depth at
    # the same design point, and the GPS of that point.
    j6, i6 = cells6[1]
    hx6 = g6.origin_x + i6 * g6.resolution
    hy6 = g6.origin_y + j6 * g6.resolution
    dl._on_map_hover(hx6, hy6)
    map_txt6 = dl._hover.text()
    dl._on_preview_hover((i6 + 0.5) / g6.nx, (j6 + 0.5) / g6.ny)
    prev_txt6 = dl._hover.text()
    lat6, lon6 = lay_state.path.anchor.to_latlon(hx6, hy6)
    check("hovering the map or a preview picture reads the depth, material "
          "and GPS of the same design point",
          f"depth {dp6[1]:.2f} m" in map_txt6 and map_txt6 == prev_txt6
          and f"{float(lat6):.6f} N" in map_txt6
          and any(m in map_txt6 for m in w2.result["material_names"]),
          map_txt6)
    from PySide6.QtCore import QEvent as _QEv6, QPoint as _QP6
    from PySide6.QtGui import QMouseEvent as _QME6
    dl.map.resize(400, 300)
    dl.map.center_on_bounds([hx6 - 5, hx6 + 5], [hy6 - 5, hy6 + 5], margin_m=1.0)
    vp6 = dl.map.mapFromScene(QPointF(hx6, hy6))
    ev6 = _QME6(_QEv6.Type.MouseMove, QPointF(vp6), QPointF(vp6),
                Qt.MouseButton.NoButton, Qt.MouseButton.NoButton,
                Qt.KeyboardModifier.NoModifier)
    dl._hover.setText("")
    dl.map.mouseMoveEvent(ev6)
    check("a real mouse move over the map drives the readout",
          f"depth {dp6[1]:.2f} m" in dl._hover.text(), dl._hover.text())
    dl.map.leaveEvent(_QEv6(_QEv6.Type.Leave))
    check("leaving the map clears it", dl._hover.text() == "")
    dl._overlay.setCurrentText("material")
    dl._overlay.setCurrentText("none")
    check("overlay channels switch and clear independently of the hint",
          "composite" not in dl.map._overlay_items and dl.map._overlay_item is None)
    dl._on_section(((0.0, 0.0), (30.0, 30.0)))
    check("the section tool plots a depth profile along a line",
          dl._section._profile is not None and dl._section.pixmap() is not None)
    from PySide6.QtWidgets import QWidget as _QW
    dl._panel.select("L1")
    tips = [w.toolTip() for w in dl._props.findChildren(_QW) if w.toolTip()]
    n_rows = len(LAYER_KINDS["basin"].schema)
    check("every layer-kind row shows a hover help in the property editor",
          len(tips) >= 2 * n_rows and all(t.strip().endswith(".") for t in tips
                                          if t and "\n" not in t),
          f"{len(tips)} tooltips")

    print("[g7] hand placement: arm, click, select, move, pin, edit")
    from PySide6.QtCore import QPointF as _QPF
    from PySide6.QtWidgets import QMessageBox as _QMB

    from blueboat_sss_sim.builder.map.canvas import Mode as _Mode7
    from blueboat_sss_sim.builder.pages.place_page import (ObjectEditDialog,
                                                           PlacePage)
    from blueboat_sss_sim.worldgen.objects import CATALOG as _CAT7

    pp = PlacePage()
    check("the catalog screen offers every class, not only the ticked ones",
          pp._list.count() == len(_CAT7), f"{pp._list.count()} classes")

    def _arm(type_name: str) -> None:
        rows = [pp._list.item(i).data(Qt.ItemDataRole.UserRole)
                for i in range(pp._list.count())]
        pp._list.setCurrentRow(rows.index(type_name))

    pp.enter(state)
    pp._worker.wait(120000)
    app.processEvents()
    n_scatter7 = sum(1 for r in pp._rows if r["kind"] == "scatter")
    check("the screen opens on the objects the scatter screen produced",
          n_scatter7 > 0 and len(pp._rows) == n_scatter7,
          f"{n_scatter7} scattered")
    check("the depth composite is drawn under the objects",
          "composite" in pp.map._overlay_items)

    _arm("traffic_cone")
    check("arming a class arms the map's place mode",
          pp._armed == "traffic_cone" and pp.map.mode is _Mode7.OBJECT)
    pp._form.prefill(None)
    pp._on_click(_QPF(210.0, 110.0))
    check("a click drops one object exactly where it landed",
          sum(1 for r in pp._rows if r["kind"] == "manual") == 1
          and pp._rows[-1]["item"]["x"] == 210.0
          and pp._rows[-1]["item"]["y"] == 110.0)
    pp._form.count.setValue(5)
    pp._form.sigma.setValue(2.0)
    pp._on_click(_QPF(220.0, 120.0))
    check("a cluster click drops the whole cluster and stays armed",
          sum(1 for r in pp._rows if r["kind"] == "manual") == 6
          and pp.map.mode is _Mode7.OBJECT)
    check("the map draws every object to scale, turned to its heading",
          len(pp.map.objects()) == len(pp._rows)
          and pp.map.objects()[-1]["length"] > 0.0)

    pp._undo.undo()
    check("undo takes back a whole cluster click",
          sum(1 for r in pp._rows if r["kind"] == "manual") == 1)
    pp._undo.redo()
    check("redo puts it back",
          sum(1 for r in pp._rows if r["kind"] == "manual") == 6)

    # Selecting a hand-placed object exposes Edit/Delete; a scattered one
    # does not, until the field is pinned.
    pp._on_select(len(pp._rows) - 1)
    check("selecting a hand-placed object enables edit and delete",
          pp._b_edit.isEnabled() and pp._b_del.isEnabled())
    pp._on_select(0)
    check("a scattered object is context only until the field is pinned",
          not pp._b_edit.isEnabled() and not pp._b_del.isEnabled())

    before7 = [(r["item"]["x"], r["item"]["y"]) for r in pp._rows]
    _answer7 = {"v": _QMB.StandardButton.No}
    _real_question = _QMB.question
    _QMB.question = staticmethod(lambda *a, **k: _answer7["v"])
    try:
        # A drag is live moves + one release; the pin question is asked
        # after the release, on the event loop, never inside the drag.
        pose7 = {"x": before7[0][0] + 5.0, "y": before7[0][1], "yaw": 0.0}
        pp._on_moved(0, pose7)
        pp._on_drag_finished(0, pose7)
        app.processEvents()
        check("declining the pin leaves the scattered field untouched",
              all(r["kind"] == "scatter" for r in pp._rows[:n_scatter7])
              and [(r["item"]["x"], r["item"]["y"])
                   for r in pp._rows] == before7)
        _answer7["v"] = _QMB.StandardButton.Yes
        pp._on_moved(0, pose7)
        pp._on_drag_finished(0, pose7)
        app.processEvents()
    finally:
        _QMB.question = _real_question
    after7 = [(r["item"]["x"], r["item"]["y"]) for r in pp._rows]
    check("moving one scattered object pins the field and moves only it",
          all(r["kind"] == "pinned" for r in pp._rows[:n_scatter7])
          and [i for i, (a, b) in enumerate(zip(before7, after7))
               if a != b] == [0],
          f"{n_scatter7} objects pinned")
    pp._on_select(0)
    check("a pinned object is editable like a hand-placed one",
          pp._b_edit.isEnabled() and pp._b_del.isEnabled())
    pp._undo.undo()
    check("undo un-pins the field",
          all(r["kind"] == "scatter" for r in pp._rows[:n_scatter7]))
    pp._undo.redo()

    n_before7 = len(pp._rows)
    pp._on_select(0)
    pp._delete_selected()
    check("delete removes exactly one object", len(pp._rows) == n_before7 - 1)
    pp._undo.undo()

    # The per-object editor: every value, verbatim.
    item7 = dict(pp._rows[-1]["item"])
    dlg7 = ObjectEditDialog(item7)
    dlg7._spins["length"].setValue(1.25)
    dlg7._spins["yaw_deg"].setValue(45.0)
    edited7 = dlg7.item()
    check("the object editor writes back length and heading verbatim",
          edited7["length"] == 1.25
          and abs(np.degrees(edited7["yaw"]) - 45.0) < 1e-9
          and edited7["width"] == edited7["length"],
          "a round class keeps width = length")
    check("a flat object keeps no tilt keys (they force the 3-D path)",
          "roll_deg" not in edited7 and "rest" not in edited7)
    dlg7._spins["roll_deg"].setValue(12.0)
    check("a tilt written by hand carries roll, pitch and the rest name",
          {"roll_deg", "pitch_deg", "rest"} <= set(dlg7.item()))

    n_manual7 = sum(1 for r in pp._rows if r["kind"] == "manual")
    n_pinned7 = sum(1 for r in pp._rows if r["kind"] == "pinned")
    pp.commit(state)
    check("commit splits the screen back into the pinned and hand-placed "
          "lists",
          len(state.objects.placed) == n_pinned7
          and len(state.objects.explicit) == n_manual7
          and state.objects.placed_signature != "")
    pp._clear_manual()
    check("clear removes the hand-placed objects and keeps the pinned ones",
          not any(r["kind"] == "manual" for r in pp._rows)
          and sum(1 for r in pp._rows if r["kind"] == "pinned") == n_pinned7)

    # A pinned field whose Screen-4 settings moved on: the operator picks.
    from blueboat_sss_sim.builder.state import scatter_signature as _sig7

    def _stale_state():
        st7 = dataclasses.replace(state, objects=dataclasses.replace(
            state.objects,
            placed=[{"type": "drum_200l", "x": 5.0, "y": 5.0, "yaw": 0.0}],
            placed_signature=_sig7(state.objects.seed,
                                   state.objects.classes)))
        st7.objects.classes = copy.deepcopy(st7.objects.classes)
        first7 = sorted(st7.objects.classes)[0]
        st7.objects.classes[first7]["amount"] = {"kind": "count", "value": 99}
        return st7

    _asked7 = {"n": 0}
    _real_exec = _QMB.exec

    def _click7(text):
        def _exec(self, *a):
            _asked7["n"] += 1
            btn = next(b for b in self.buttons()
                       if b.text().replace("&", "") == text)
            self.clickedButton = lambda: btn
        _QMB.exec = _exec

    try:
        fresh7 = dataclasses.replace(state, objects=dataclasses.replace(
            state.objects,
            placed=[{"type": "drum_200l", "x": 5.0, "y": 5.0, "yaw": 0.0}],
            placed_signature=_sig7(state.objects.seed,
                                   state.objects.classes)))
        _click7("Re-scatter")
        pp._check_stale(fresh7)
        check("a pin that still matches Screen 4 asks nothing",
              _asked7["n"] == 0 and len(fresh7.objects.placed) == 1)

        stale7 = _stale_state()
        _click7("Re-scatter")
        pp._check_stale(stale7)
        check("a stale pin offers re-scatter, which drops it and keeps the "
              "new settings",
              _asked7["n"] == 1 and stale7.objects.placed == []
              and stale7.objects.placed_signature == "")

        stale7 = _stale_state()
        _click7("Keep the pinned objects")
        pp._check_stale(stale7)
        check("keeping the pinned objects adopts the new signature, so it "
              "asks once",
              len(stale7.objects.placed) == 1
              and stale7.objects.placed_signature
              == _sig7(stale7.objects.seed, stale7.objects.classes))
    finally:
        _QMB.exec = _real_exec

    print("[g5] wizard shells")
    win_create = WorldBuilderWindow(state=BuilderState(), edit_mode=False)
    check("create wizard has 6 screens", len(win_create.pages) == 6)
    win_edit = WorldBuilderWindow(state=state, edit_mode=True)
    check("edit wizard skips Screen 1 and starts on the map",
          len(win_edit.pages) == 5
          and win_edit.pages[0].TITLE.startswith("World limits"))
    check("hand placement sits between the scatter screen and the preview",
          [p.TITLE for p in win_create.pages[-3:]]
          == ["Objects", "Place objects by hand", "Preview & save"])

    # --- real Qt events on a fresh page: the crash class of 2026-09-16 ---
    # (selection change and handle drags used to tear the scene down from
    # inside Qt's own dispatch -- a use-after-free segfault).
    import faulthandler as _fh
    import math as _math7
    _fh.enable()
    from PySide6.QtTest import QTest as _QT
    # A state whose scatter is still live (the commit above pinned it).
    state2 = dataclasses.replace(
        state, objects=ObjectsState(seed=state.objects.seed,
                                    classes=dict(state.objects.classes)))
    pp2 = PlacePage()
    pp2.show()
    pp2.map.resize(640, 440)
    pp2.enter(state2)
    w_first = pp2._worker
    pp2.enter(state2)                     # again, while the worker runs
    check("re-entering the screen while its worker runs waits for it "
          "instead of replacing a running QThread",
          pp2._worker is w_first and pp2._pending)
    pp2._worker.wait(120000)
    app.processEvents()
    if pp2._worker is not None:
        pp2._worker.wait(120000)
    app.processEvents()
    check("the pending preview ran afterwards",
          not pp2._pending and pp2._preview is not None)

    def _vp2(x, y):
        return pp2.map.mapFromScene(_QPF(x, y))

    rows2 = [pp2._list.item(i).data(Qt.ItemDataRole.UserRole)
             for i in range(pp2._list.count())]
    pp2._list.setCurrentRow(rows2.index("drum_200l"))
    pp2._form.prefill(None)
    pp2._form.count.setValue(1)
    pp2._on_click(_QPF(20.0, 20.0))
    app.processEvents()
    idx2 = len(pp2._rows) - 1
    pp2._set_mode(_Mode7.SELECT)
    pp2.map.center_on_bounds([14.0, 26.0], [14.0, 26.0], margin_m=1.0)
    app.processEvents()
    _QT.mouseClick(pp2.map.viewport(), Qt.MouseButton.LeftButton,
                   pos=_vp2(25.0, 15.0))            # empty water: deselect
    app.processEvents()
    item2 = pp2.map._object_items.get(idx2)
    deselected2 = pp2._selected == -1 and item2 is not None and not item2.isSelected()
    _QT.mouseClick(pp2.map.viewport(), Qt.MouseButton.LeftButton,
                   pos=_vp2(20.0, 20.0))            # the footprint
    app.processEvents()
    check("clicking an unselected object selects it and re-pens the same "
          "item (no scene teardown inside Qt's dispatch)",
          deselected2 and pp2._selected == idx2
          and pp2.map._object_items.get(idx2) is item2 and item2.isSelected(),
          f"deselected {deselected2}, selected {pp2._selected}")
    yaw_h = [h for h in pp2.map._object_handles if h.role[2] == "yaw"]
    check("the selected object carries its move and yaw handles",
          len(pp2.map._object_handles) == 2 and len(yaw_h) == 1)
    hp2 = yaw_h[0].pos()
    r2 = _math7.hypot(hp2.x() - 20.0, hp2.y() - 20.0)
    yaw_before2 = float(pp2._rows[idx2]["item"]["yaw"])   # a random draw
    n_undo2 = pp2._undo.count()
    _QT.mousePress(pp2.map.viewport(), Qt.MouseButton.LeftButton,
                   pos=_vp2(hp2.x(), hp2.y()))
    _QT.mouseMove(pp2.map.viewport(), _vp2(20.0 + 0.3 * r2, 20.0 + 0.9 * r2))
    _QT.mouseMove(pp2.map.viewport(), _vp2(20.0, 20.0 + r2))
    _QT.mouseRelease(pp2.map.viewport(), Qt.MouseButton.LeftButton,
                     pos=_vp2(20.0, 20.0 + r2))
    app.processEvents()
    yaw2 = float(pp2._rows[idx2]["item"]["yaw"])
    check("dragging the nose handle turns the object and pushes exactly one "
          "undo step on release",
          abs(_math7.degrees(yaw2) - 90.0) < 12.0
          and pp2._undo.count() == n_undo2 + 1,
          f"heading {_math7.degrees(yaw2):.1f} deg, undo {n_undo2} -> "
          f"{pp2._undo.count()}")
    pp2._undo.undo()
    check("undoing the turn restores the heading it had before the drag",
          abs(float(pp2._rows[idx2]["item"]["yaw"]) - yaw_before2) < 1e-9)

    sc2 = next((i for i, r in enumerate(pp2._rows) if r["kind"] == "scatter"), -1)
    check("the fresh page still has a scattered field to test the drag on",
          sc2 >= 0)
    sx2, sy2 = (float(pp2._rows[sc2]["item"]["x"]),
                float(pp2._rows[sc2]["item"]["y"]))
    pp2.map.center_on_bounds([sx2 - 6.0, sx2 + 6.0], [sy2 - 6.0, sy2 + 6.0],
                             margin_m=1.0)
    app.processEvents()
    _QT.mouseClick(pp2.map.viewport(), Qt.MouseButton.LeftButton,
                   pos=_vp2(sx2, sy2))
    app.processEvents()
    check("a scattered footprint is selectable from the map (the documented "
          "'drag it to pin the field' route is reachable)",
          pp2._selected == sc2 and len(pp2.map._object_handles) == 2)
    mv2 = [h for h in pp2.map._object_handles if h.role[2] == "move"][0]
    _answer7["v"] = _QMB.StandardButton.No
    _QMB.question = staticmethod(lambda *a, **k: _answer7["v"])
    try:
        _QT.mousePress(pp2.map.viewport(), Qt.MouseButton.LeftButton,
                       pos=_vp2(mv2.pos().x(), mv2.pos().y()))
        _QT.mouseMove(pp2.map.viewport(), _vp2(sx2 + 1.5, sy2))
        _QT.mouseMove(pp2.map.viewport(), _vp2(sx2 + 3.0, sy2))
        _QT.mouseRelease(pp2.map.viewport(), Qt.MouseButton.LeftButton,
                         pos=_vp2(sx2 + 3.0, sy2))
        app.processEvents()               # the deferred pin question fires
        app.processEvents()
    finally:
        _QMB.question = _real_question
    check("a scatter drag asks the pin question after the release, never "
          "inside the drag; declining snaps the object back",
          pp2._rows[sc2]["kind"] == "scatter"
          and abs(float(pp2._rows[sc2]["item"]["x"]) - sx2) < 1e-9
          and abs(pp2.map.objects()[sc2]["x"] - sx2) < 1e-9)
    check("a right double-click opens nothing and the buttons follow the "
          "map's mode",
          pp2._b_select.isChecked() and not pp2._b_place.isChecked())
    pp2.close()

    # Edit mode (2026-09-28): a world that already holds pinned and hand-
    # placed objects shows them the moment the screen opens -- the worker
    # re-runs the whole build, tens of seconds on a layered world -- and a
    # Back/Next in that wait commits them, never the empty list it used to.
    from PySide6.QtWidgets import QGraphicsItem as _QGI
    state3 = dataclasses.replace(state, objects=copy.deepcopy(state.objects))
    n3 = len(state3.objects.placed) + len(state3.objects.explicit)
    pp3 = PlacePage()
    pp3.show()
    pp3.map.resize(640, 440)
    pp3.enter(state3)
    check("an edited world's objects show the moment the screen opens, "
          "before the worker returns",
          n3 > 0 and len(pp3._rows) == n3 and len(pp3.map.objects()) == n3
          and pp3._worker is not None and pp3._worker.isRunning(),
          f"{n3} objects shown while the worker runs")
    check("the worker computes on a copy of the state, not the live one",
          pp3._worker._state is not state3
          and pp3._worker._state.objects is not state3.objects)
    m3 = next(i for i, r in enumerate(pp3._rows) if r["kind"] == "manual")
    moved_x3 = float(pp3._rows[m3]["item"]["x"]) + 1.0
    pp3._rows[m3]["item"]["x"] = moved_x3        # a move during the wait
    pp3.commit(state3)                           # Back / Next meanwhile
    check("a Back/Next while the worker runs commits the objects, not an "
          "empty list",
          len(state3.objects.placed) + len(state3.objects.explicit) == n3
          and any(abs(float(i["x"]) - moved_x3) < 1e-9
                  for i in state3.objects.explicit))
    pp3._worker.wait(120000)
    app.processEvents()
    own3 = [r for r in pp3._rows if r["kind"] != "scatter"]
    check("the worker's result keeps the page's own rows: the move made "
          "while it ran survives",
          len(own3) == n3
          and any(abs(float(r["item"]["x"]) - moved_x3) < 1e-9 for r in own3))
    rings3 = [it.childItems() for it in pp3.map._object_items.values()]
    check("every footprint carries a fixed-size ring, passive to the mouse",
          len(rings3) == len(pp3._rows) and all(
              len(k) == 1 and bool(k[0].flags()
                                   & _QGI.GraphicsItemFlag.ItemIgnoresTransformations)
              and k[0].acceptedMouseButtons() == Qt.MouseButton.NoButton
              for k in rings3))
    pp3.close()

    print("[g8] screen adaptation")
    from PySide6.QtCore import QSize, Qt
    from PySide6.QtGui import QGuiApplication, QPixmap
    from PySide6.QtWidgets import QPushButton
    from blueboat_sss_sim.builder.layout import (DEFAULT_WINDOW, PageScroll,
                                                 ScaledImage, ToolBar,
                                                 fit_to_screen)

    # A 1024x600 netbook is the floor the builder must open on.
    SMALL = QSize(1000, 600)
    worst = max((p for w in (win_create, win_edit) for p in w.pages),
                key=lambda p: p.minimumSizeHint().width())
    check("every screen's layout minimum fits a small display",
          all(p.minimumSizeHint().width() <= SMALL.width()
              and p.minimumSizeHint().height() <= SMALL.height()
              for w in (win_create, win_edit) for p in w.pages),
          f"widest: {type(worst).__name__} "
          f"{worst.minimumSizeHint().width()}x"
          f"{worst.minimumSizeHint().height()}")
    check("the window itself can be shrunk to it",
          win_create.minimumSize().width() <= SMALL.width()
          and win_create.minimumSize().height() <= SMALL.height()
          and win_create.minimumSizeHint().width() <= SMALL.width())

    bar8 = ToolBar()
    for _ in range(12):
        bar8.add(QPushButton("Material brush"))
    one_row = bar8.minimumSizeHint().height()
    check("a tool row wraps instead of widening the window",
          bar8.minimumSizeHint().width() < 200
          and bar8._flow.heightForWidth(300) > bar8._flow.heightForWidth(1600),
          f"min width {bar8.minimumSizeHint().width()} px")
    check("and it never eats the page: at most three rows, then it scrolls",
          bar8._wanted_height(200) <= one_row * ToolBar.MAX_ROWS + 12
          and bar8._wanted_height(4000) == one_row)

    img8 = ScaledImage("(none)", min_w=80, min_h=60)
    img8.show()
    img8.resize(400, 300)
    src8 = QPixmap(800, 600)
    src8.fill(Qt.GlobalColor.blue)
    img8.set_image(src8)
    big8 = img8.pixmap().width()
    img8.resize(200, 150)
    small8 = img8.pixmap().width()
    img8.resize(400, 300)
    check("a preview rescales with the window instead of freezing",
          big8 <= 400 and small8 <= 200 and img8.pixmap().width() > small8,
          f"{big8} -> {small8} -> {img8.pixmap().width()} px")

    holder8 = PageScroll(LayoutPage())
    holder8.show()
    holder8.resize(900, 560)
    page8 = holder8.widget()
    check("a page compresses to the window before any scroll bar appears",
          page8.width() <= 900 and page8.height() <= 560
          and not holder8.verticalScrollBar().isVisible())
    holder8.resize(1500, 950)
    check("and takes the extra room when the window grows",
          page8.width() > 900 and page8.height() > 560,
          f"{page8.width()}x{page8.height()}")
    tiny8 = QSize(300, 240)
    holder8.resize(tiny8)
    check("below its minimum the page scrolls rather than being clipped",
          page8.size().width() >= holder8.widget().minimumSizeHint().width())

    from blueboat_sss_sim.builder.layout import WrapScroll
    dp8 = DepthPage()
    dp8.show()
    dp8.resize(1000, 600)
    dp8._set_quality("recording: 1234 pings, 98 % with a bottom return, "
                     "altitude 0.7–1.6 m, 2500 cells @ 1.0 m, crossover |Δ| "
                     "0.04 m (n 12), clock live\n⚠ the recording reaches 20 % "
                     "of the world area; the rest is extrapolated\ncoverage: "
                     "20 % of the world area within 15 m of a sounding, 75 % "
                     "of 12 samples inside the limits")
    dp8._prop_stack.setCurrentIndex(0)
    app.processEvents()
    q8 = dp8._svlog_quality
    need8 = q8.heightForWidth(q8.width())
    check("the source column gives the wrapped recording-quality line its "
          "full height (no clipped text under 'From sonar recording')",
          isinstance(dp8._prop_scroll, WrapScroll) and q8.width() > 100
          and q8.height() >= need8 and need8 > q8.fontMetrics().height() * 2
          and q8.toolTip() == q8.text(),
          f"label {q8.width()}x{q8.height()} needs {need8}")
    src8 = dp8._prop_stack.widget(0)
    check("and the column stays narrow enough for a small display",
          src8.minimumSizeHint().width() <= 300
          and dp8._svlog_backscatter.text() == "Reflectivity from backscatter",
          f"source page min width {src8.minimumSizeHint().width()} px")
    dp8.close()

    fit_to_screen(win_create, *DEFAULT_WINDOW)
    avail8 = (win_create.screen() or QGuiApplication.primaryScreen())
    if avail8 is not None:
        geo8 = avail8.availableGeometry()
        check("the window opens inside the available screen area",
              win_create.width() <= geo8.width()
              and win_create.height() <= geo8.height(),
              f"{win_create.width()}x{win_create.height()} in "
              f"{geo8.width()}x{geo8.height()}")

    print("\nGUI SMOKE PASSED")


if __name__ == "__main__":
    main()
