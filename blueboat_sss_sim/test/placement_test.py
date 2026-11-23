"""Mission-like placement in the World Builder (simulator v1.1, 2026-11-21).

Not part of the gated offline suite (``test/smoke_test.py``), whose recorded
check count stays as it was: a separate script in the same style. Run from
the package source root:

    QT_QPA_PLATFORM=offscreen python3 -m test.placement_test

It needs PySide6 for the page sequences and the planner package
``blueboat_harbour_population`` (``../blueboat_harbour_population`` beside
this package, or installed); without either it says so and skips that part.

[p1] the builder state: an empty placement record is never written (a
     Manual world's ``builder_state.yaml`` is what it was), a record
     round-trips, the build of a Manual state is the build it always was;
[p2] Screen 5 offscreen: Manual -> Mission-like -> plan -> Next; the world
     built from it; Edit re-open (the stored plan, not re-planned); what
     Screen 3 drops is restored; Manual over a stored plan clears it;
     Convert to manual and the ``edited`` flag; a wall's use set from the
     map; the planner missing; the screen still fits a small display.
"""

from __future__ import annotations

import copy
import dataclasses
import os
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

PKG_ROOT = Path(__file__).resolve().parents[1]
PLANNER_ROOT = PKG_ROOT.parent / "blueboat_harbour_population"
sys.path.insert(0, str(PKG_ROOT))
if PLANNER_ROOT.is_dir():
    sys.path.insert(1, str(PLANNER_ROOT))

import yaml  # noqa: E402

N_CHECKS = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global N_CHECKS
    N_CHECKS += 1
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {name}" + (f" -- {detail}" if detail else ""))
    if not cond:
        raise SystemExit(f"placement test failed at: {name}")


def make_state(tdir: Path):
    from blueboat_sss_sim.builder.state import BuilderState, DepthState, ObjectsState, PathState
    from blueboat_sss_sim.worldgen.terrain import texture_defaults
    f = tdir / "ml_test_path.yaml"
    f.write_text(yaml.safe_dump({
        "format": "blueboat_trajectory/1", "name": "ml_test_path", "loop": False,
        "geo_anchor": {"lat0": 34.4082, "lon0": 133.5248, "theta_deg": 0.0},
        "points": [[0.0, 0.0, 0.0, 0.0], [40.0, 50.0, 0.0, 0.0], [50.0, 50.0, 12.0, 1.57],
                   [90.0, 0.0, 12.0, 3.14]]}))
    tex = texture_defaults()
    return BuilderState(
        path=PathState.from_mcs_file(f),
        limits_rect={"x_min": -25.0, "y_min": -25.0, "x_max": 75.0, "y_max": 40.0},
        walls=[{"name": "quay_n", "x0": -25.0, "y0": 37.0, "x1": 75.0, "y1": 37.0, "top_z": 1.0,
                "reflectivity": 0.65, "thickness": 0.3, "kind": "quay"},
               {"name": "float_1", "x0": 30.0, "y0": 36.7, "x1": 30.0, "y1": 24.0, "top_z": 0.6,
                "reflectivity": 0.65, "thickness": 0.3, "kind": "pontoon", "draft_m": 0.5}],
        depth=DepthState(mode="preset", preset="sloped_plane",
                         params={"depth_centre": 5.9, "grade": 0.002, "direction_deg": 90.0, "min_depth": 0.3},
                         texture=tex),
        objects=ObjectsState(seed=7, classes={"drum_200l": {"amount": {"kind": "count", "value": 2}}},
                             explicit=[{"type": "mooring_block", "x": -5.0, "y": 20.0, "yaw": 0.0, "length": 2.5,
                                        "width": 2.5, "proud_height": 1.2, "burial": 0.0, "fill_fraction": 1.0,
                                        "reflectivity": 0.5, "material": "concrete",
                                        "params": {"spec": 2, "role": "structure", "name": "block_a"}}]),
        world_name="ml_test")


def section_state(tdir: Path) -> None:
    print("[p1] builder state and the Manual build")
    from blueboat_sss_sim.builder import placement
    from blueboat_sss_sim.builder.state import BuilderState
    from blueboat_sss_sim.core import provenance
    from blueboat_sss_sim.worldgen.world_folder import build_world_folder
    st = make_state(tdir)
    d = st.to_dict()
    check("an empty placement record is not written", "placement" not in d and list(d)[-1] == "world_name")
    check("a Manual state is Manual and gives no extras",
          placement.mode_of(st) == placement.MANUAL and placement.extras(st) is None)
    rt = BuilderState.from_dict(d)
    check("a state without a record loads with an empty one", rt.placement == {} and rt.to_dict() == d)
    st2 = dataclasses.replace(st, placement={"mode": "mission_like", "metadata": {"mode": "mission_like"},
                                             "seabed": {"reference": "sand", "patch_classes": ["gravel"]}})
    d2 = st2.to_dict()
    check("a record round-trips through builder_state.yaml",
          BuilderState.from_dict(yaml.safe_load(yaml.safe_dump(d2))).placement == st2.placement
          and list(d2)[-1] == "placement")
    check("replace() shares no record between two states",
          dataclasses.replace(st).placement == {} and st.placement == {})
    # the Manual build: the GUI's worker against the library call
    from blueboat_sss_sim.builder.worker import GenerateWorker
    a = Path(build_world_folder(copy.deepcopy(st), tdir / "a"))
    w = GenerateWorker(copy.deepcopy(st), worlds_root=str(tdir / "b"))
    out: list = []
    w.finished_path.connect(out.append)
    w.failed.connect(lambda m: out.append("FAILED " + m))
    w.run()
    b = Path(out[0])
    check("the builder's worker builds a Manual state exactly as the library does",
          provenance.content_digest(a, also=(str(a),))["sha256"]
          == provenance.content_digest(b, also=(str(b),))["sha256"])
    sa = yaml.safe_load((a / "builder_state.yaml").read_text())
    check("a Manual world's builder_state.yaml holds no placement key", "placement" not in sa)
    meta = yaml.safe_load((a / "metadata.yaml").read_text())
    check("a Manual world's metadata.yaml holds no placement block", "placement" not in meta)
    # the planner's reserved prefix is the builder's
    try:
        from blueboat_harbour_population import layers as hl
        check("the builder and the planner reserve the same layer prefix",
              hl.RESERVED_PREFIX == placement.RESERVED_PREFIX)
    except ImportError:
        print("  SKIP: blueboat_harbour_population not importable")


def section_gui(tdir: Path) -> None:
    print("[p2] Screen 5 offscreen")
    try:
        from PySide6.QtCore import QThread
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print("  SKIP: PySide6 not installed")
        return
    from blueboat_sss_sim.builder import placement, theme
    hp, sa, why = placement.import_planner()
    if hp is None:
        print(f"  SKIP: the planner cannot be imported ({why})")
        return
    from blueboat_sss_sim.builder.map.canvas import Mode
    from blueboat_sss_sim.builder.pages import place_page as ppm
    from blueboat_sss_sim.builder.pages.place_page import PlacePage
    from blueboat_sss_sim.builder.worker import GenerateWorker
    from blueboat_harbour_population.schema import check_world_block
    import faulthandler
    faulthandler.enable()
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)

    def wait(pred, timeout: float = 180.0) -> bool:
        t0 = time.time()
        while not pred():
            app.processEvents()
            QThread.msleep(20)
            if time.time() - t0 > timeout:
                return False
        app.processEvents()
        return True

    def idle(pp) -> bool:
        busy = [w for w in (pp._worker, pp._env_worker, pp._plan_worker) if w is not None and w.isRunning()]
        return (not busy and not pp._pending and not pp._env_pending and not pp._plan_pending
                and not pp._plan_timer.isActive())

    st = make_state(tdir)
    pp = PlacePage()
    pp.show()
    pp.map.resize(640, 440)
    pp.enter(st)
    check("the screen opens in Manual mode on a Manual state",
          pp._mode == placement.MANUAL and pp._b_manual.isChecked() and pp._b_mission.isEnabled()
          and pp._left.currentIndex() == 0)
    check("the title the wizard tests is kept", PlacePage.TITLE == "Place objects by hand")
    wait(lambda: idle(pp))
    n_scatter = sum(1 for r in pp._rows if r["kind"] == "scatter")
    check("Manual mode shows Screen 4's scatter as always", n_scatter == 2, f"{n_scatter} scattered")
    rows_manual = copy.deepcopy(pp._rows)

    pp._switch_mode(placement.MISSION)
    check("Mission-like shows the panel, hides the hand tools and keeps the map in select mode",
          pp._left.currentIndex() == 1 and pp._b_place.isHidden() and pp.map.mode is Mode.SELECT)
    check("the hand-placed block stays; the scatter is put aside",
          [r["item"]["type"] for r in pp._rows] == ["mooring_block"])
    ok = wait(lambda: pp._env is not None and pp._plan is not None and idle(pp))
    check("the environment is read and a plan made with no click at all", ok)
    env = pp._env
    ids = {f.id for f in env.features}
    check("the environment's features are the walls and the hand-placed block",
          {"quay_n", "float_1", "block_a"} <= ids, f"{sorted(ids)}")
    plan5 = pp._plan
    blk = pp._panel.block()
    check("the defaults: Auto, all seven classes, level 5, a seed of the development split",
          blk["profile"] == "auto" and len(blk["classes"]) == 7 and blk["density_level"] == 5
          and 1 <= blk["seed"] <= 99999 and blk["split"] == "development")
    items = pp.map.objects()
    n_plan = sum(1 for o in items if o.get("kind") == "plan")
    check("the planned objects are drawn, in their class colours, not as page rows",
          n_plan == len([e for e in plan5.instances + plan5.structures if e["type"] != "chain"]) and n_plan > 0
          and all(o.get("color") for o in items if o.get("kind") == "plan")
          and len(pp._rows) == 1, f"{n_plan} planned")
    check("the panel shows the expected shares and counts",
          "expected" in pp._panel.level_text.text() and "%" in pp._panel.classes["tire_car"].text())
    # the same block gives the same plan (the panel's block is all the plan depends on)
    again = hp.populate(env, blk)
    check("the page's plan is the planner's plan for the panel's block", again.digest() == plan5.digest())
    # a control: the level
    pp._panel.level.setValue(9)
    ok = wait(lambda: idle(pp) and pp._plan_block == pp._panel.block())
    check("moving the slider plans again", ok and pp._plan.digest() != plan5.digest()
          and pp._plan.report["expected"]["total"] > plan5.report["expected"]["total"])
    t0 = time.time()
    hp.populate(env, pp._panel.block())
    check("a plan of this world takes under a second", time.time() - t0 < 1.0, f"{time.time() - t0:.2f} s")
    # a click on the map places nothing in this mode
    n_rows = len(pp._rows)
    from PySide6.QtCore import QPointF
    pp._on_click(QPointF(10.0, 10.0))
    check("a click on the map places nothing in Mission-like mode", len(pp._rows) == n_rows)
    # a wall's use from a double-click (the dialog is deferred to the event loop)
    real_get = ppm.QInputDialog.getItem
    ppm.QInputDialog.getItem = staticmethod(lambda *a, **k: ("working quay", True))
    try:
        idx = [w["name"] for w in pp.map.walls()].index("quay_n")
        pp._on_wall_double_clicked(idx)
        check("the use dialog does not open inside the scene's dispatch",
              pp._panel.block().get("feature_tags", {}).get("quay_n") is None)
        ok = wait(lambda: pp._panel.block().get("feature_tags", {}).get("quay_n") == "working_quay" and idle(pp)
                  and pp._plan_block == pp._panel.block())
    finally:
        ppm.QInputDialog.getItem = real_get
    uses = {f["id"]: f["use"] for f in pp._plan.report["features"]}
    check("a double-click on a wall sets its use, and the plan follows", ok and uses.get("quay_n") == "working_quay")
    # Next
    check("Next writes the plan", pp.commit(st))
    plan = pp._plan
    check("the state holds the plan: litter pinned, structures beside the hand-placed block, the seabed layers",
          st.placement.get("mode") == "mission_like" and st.objects.placed == plan.instances
          and [e for e in st.objects.explicit if not placement.is_plan_structure(e)][0]["params"]["name"] == "block_a"
          and [L["id"] for L in st.depth.layers if placement.is_reserved(L)] == [L["id"] for L in plan.layers]
          and st.objects.pinned)
    check("the seabed declaration lists the small litter's hard class",
          plan.litter_class is None or plan.litter_class in st.depth.seabed.get("patch_classes", []))
    ex = placement.extras(st)
    check("the build gets the placement block for metadata.yaml",
          ex is not None and check_world_block(ex["placement"]) == [] and "sonar_fouling" in ex["run"])
    w = GenerateWorker(copy.deepcopy(st), worlds_root=str(tdir / "ml"))
    out: list = []
    w.finished_path.connect(out.append)
    w.failed.connect(lambda m: out.append("FAILED " + m))
    w.run()
    check("the world builds", out and not str(out[0]).startswith("FAILED"), str(out[:1]))
    wdir = Path(out[0])
    meta = yaml.safe_load((wdir / "metadata.yaml").read_text())
    bs = yaml.safe_load((wdir / "builder_state.yaml").read_text())
    check("metadata.yaml carries the placement block and every planned object",
          meta["placement"]["plan_digest"] == plan.digest()
          and sum(1 for o in meta["objects"] if o["role"] in ("target", "clutter")) == len(plan.instances)
          and "gravel" in meta["seabed"]["patch_classes"])
    check("builder_state.yaml keeps the record (model digests included) for an Edit re-save",
          bs["placement"]["mode"] == "mission_like" and len(bs["placement"]["model"]["code_digest"]) == 16)

    # ---- Edit re-open: the stored plan, not re-planned
    st_edit = copy.deepcopy(st)
    before = yaml.safe_dump(st_edit.to_dict(), sort_keys=True)
    pe = PlacePage()
    pe.show()
    pe.map.resize(640, 440)
    pe.enter(st_edit)
    check("a state with a plan opens in Mission-like mode with the stored plan",
          pe._mode == placement.MISSION and pe._stored is not None and pe._plan is None
          and pe._panel.block()["density_level"] == 9)
    wait(lambda: pe._env is not None and idle(pe))
    check("the stored plan is not planned again", pe._plan is None and not pe._panel.b_replan.isVisible())
    check("Next keeps it, byte for byte", pe.commit(st_edit)
          and yaml.safe_dump(st_edit.to_dict(), sort_keys=True) == before)
    # what Screen 3 drops is restored
    st_drop = copy.deepcopy(st)
    st_drop.depth.seabed = {}
    st_drop.depth.layers = [L for L in st_drop.depth.layers if not placement.is_reserved(L)]
    pd = PlacePage()
    pd.enter(st_drop)
    check("the seabed declaration and the plan's layers Screen 3 dropped are back",
          st_drop.depth.seabed == st.depth.seabed
          and [L["id"] for L in st_drop.depth.layers] == [L["id"] for L in st.depth.layers])
    wait(lambda: idle(pd))
    # a changed world: stale, kept unless asked
    st_moved = copy.deepcopy(st)
    st_moved.walls[1]["x0"] = st_moved.walls[1]["x1"] = 10.0
    pm = PlacePage()
    pm.enter(st_moved)
    wait(lambda: pm._env is not None and idle(pm))
    check("a world changed since its plan says so and keeps the plan until Re-plan",
          pm._plan is None and pm._panel.b_replan.isVisibleTo(pm._panel) and pm.commit(st_moved))
    pm._replan()
    wait(lambda: pm._plan is not None and idle(pm))
    check("Re-plan follows the world as it is", pm._plan is not None
          and pm._plan.environment_digest != st.placement["environment_digest"])

    # ---- Manual over a stored plan clears it
    st_back = copy.deepcopy(st)
    pb = PlacePage()
    pb.enter(st_back)
    wait(lambda: idle(pb))
    pb._switch_mode(placement.MANUAL)
    wait(lambda: idle(pb))
    check("Manual over a stored plan", pb._mode == placement.MANUAL and pb.commit(st_back))
    check("... clears the record, the plan's layers and structures, and restores the seabed declaration",
          st_back.placement == {} and not any(placement.is_reserved(L) for L in st_back.depth.layers)
          and not any(placement.is_plan_structure(e) for e in st_back.objects.explicit)
          and st_back.depth.seabed == {} and placement.extras(st_back) is None)
    # Manual -> Mission-like -> Manual on a page: the rows as they were
    pp2 = PlacePage()
    st2 = make_state(tdir)
    pp2.enter(st2)
    wait(lambda: idle(pp2))
    rows0 = copy.deepcopy(pp2._rows)
    pp2._switch_mode(placement.MISSION)
    wait(lambda: pp2._plan is not None and idle(pp2))
    pp2._switch_mode(placement.MANUAL)
    wait(lambda: idle(pp2))
    check("switching to Mission-like and back gives the Manual rows back", pp2._rows == rows0)
    st2b = make_state(tdir)
    pp2.commit(st2)
    p3 = PlacePage()
    p3.enter(st2b)
    wait(lambda: idle(p3))
    p3.commit(st2b)
    check("... and the same state as a session that never switched",
          yaml.safe_dump(st2.to_dict()) == yaml.safe_dump(st2b.to_dict()))
    check("Screen 4's scatter came back in Manual", rows_manual == rows0)

    # ---- Convert to manual
    st_c = make_state(tdir)
    pc = PlacePage()
    pc.enter(st_c)
    wait(lambda: idle(pc))
    pc._switch_mode(placement.MISSION)
    wait(lambda: pc._env is not None and idle(pc))
    pc._panel.seed.setValue(14006)                  # a fixed seed: the panel rolls a random one at first
    pc._panel.level.setValue(8)
    wait(lambda: pc._plan is not None and idle(pc) and pc._plan_block == pc._panel.block())
    plan_c = pc._plan
    check("the plan to convert holds objects", len(plan_c.instances) > 0, f"{len(plan_c.instances)} objects")
    pc._convert_to_manual()
    check("Convert to manual: the planned objects become editable pinned objects",
          pc._mode == placement.MANUAL and sum(1 for r in pc._rows if r["kind"] == "pinned") == len(plan_c.instances)
          and not pc._b_place.isHidden())
    check("Next", pc.commit(st_c))
    rec = st_c.placement
    check("the world records where it came from, not edited",
          rec["mode"] == "manual" and rec["origin"]["plan_digest"] == plan_c.digest() and rec["edited"] is False
          and placement.extras(st_c)["placement"]["edited"] is False)
    check("its small-litter layers stay, as ordinary layers with their seeds",
          [L["seed"] for L in st_c.depth.layers if L["id"].startswith("converted_")]
          == [L["seed"] for L in plan_c.layers])
    pc2 = PlacePage()
    pc2.enter(st_c)
    wait(lambda: idle(pc2))
    k = next(i for i, r in enumerate(pc2._rows) if r["kind"] == "pinned")
    pc2._rows[k]["item"]["x"] += 1.5
    pc2.commit(st_c)
    check("moving one object marks the world edited", st_c.placement["edited"] is True
          and placement.extras(st_c)["placement"]["edited"] is True)

    # ---- re-entering the screen while the environment is still being read
    st_r = copy.deepcopy(st)
    pr = PlacePage()
    pr.enter(st_r)
    w_first = pr._env_worker
    pr.enter(st_r)                                  # Back and Next at once, while the worker runs
    check("re-entering keeps a running environment worker instead of dropping it",
          w_first is None or pr._env_worker is w_first)
    wait(lambda: pr._env is not None and idle(pr))
    check("... and the screen settles on the stored plan", pr._env is not None and pr._stored is not None
          and pr.commit(st_r) and st_r.placement["plan_digest"] == st.placement["plan_digest"])

    # ---- the planner missing
    real = placement.import_planner
    placement.import_planner = lambda: (None, None, "ImportError: test")
    try:
        pn = PlacePage()
        check("without the planner the Mission-like switch is off and Manual works",
              not pn._b_mission.isEnabled() and pn._panel is None and "test" in pn._b_mission.toolTip())
    finally:
        placement.import_planner = real

    # ---- size
    hint = pp.minimumSizeHint()
    check("the screen still fits a 1000 x 600 display", hint.width() <= 1000 and hint.height() <= 600,
          f"{hint.width()} x {hint.height()}")
    for page in (pp, pe, pd, pm, pb, pp2, p3, pc, pc2, pr):
        for wk in (page._worker, page._env_worker, page._plan_worker):
            if wk is not None:
                wk.wait(60000)
    app.processEvents()


def main() -> None:
    tdir = Path(tempfile.mkdtemp(prefix="placement_test_"))
    section_state(tdir)
    section_gui(tdir)
    print(f"\nALL {N_CHECKS} CHECKS PASSED")


if __name__ == "__main__":
    main()
