"""A real-harbour world from a specification (``environment:`` with walls of
several points, land, structures, use areas) and the same world opened again:
the built world must give the plan the specification gives (World Builder
Screen 5 "Mission-like" reads the state of ``builder_state.yaml`` and nothing
else: ``environment_from_state(state, features=[])``).

Version 1.1.0 (session 16, 2026-11-26). Synthetic grids only."""

import copy
import dataclasses

import numpy as np
import pytest
import yaml

from conftest import needs_simulator
from test_depth_file import LAT0, LON0, TRAJECTORY, chart_depth

from blueboat_harbour_population import populate
from blueboat_harbour_population.schema import check_world_block

pytestmark = needs_simulator

QUAY = [[-20.0, -20.0], [30.0, -22.0], [75.0, -20.0]]            # three points: two engine walls
LAND = [[-30.0, -30.0], [85.0, -30.0], [85.0, -20.0], [75.0, -20.0], [30.0, -22.0], [-20.0, -20.0], [-30.0, -20.0]]
DECLARATION = {"reference": "sand", "patch_classes": ["coarse_sand", "rocks"], "heterogeneity": "low"}


@pytest.fixture(scope="module")
def files(tmp_path_factory):
    from blueboat_sss_sim.core.geo import GeoAnchor
    from blueboat_sss_sim.core.types import GridSpec
    from blueboat_sss_sim.worldgen.depth.bake import write_depth_geotiff
    d = tmp_path_factory.mktemp("harbour")
    (d / "depth_test_path.yaml").write_text(TRAJECTORY)
    xs, ys = np.arange(-60.0, 215.0, 1.0), np.arange(-45.0, 95.0, 1.0)
    X, Y = np.meshgrid(xs, ys)
    write_depth_geotiff(-chart_depth(X, Y), GridSpec(-60.0, -45.0, 1.0, len(xs), len(ys)),
                        GeoAnchor(lat0=LAT0, lon0=LON0), d / "chart.tif")
    return d


def spec(**env):
    block = {
        "depth_file": "chart.tif", "water_level_m": 1.2, "smooth_sigma_m": 2.5, "shift_m": 3.6,
        "limits": {"x_min": -30.0, "y_min": -30.0, "x_max": 85.0, "y_max": 70.0},
        "walls": [{"name": "quay_w", "kind": "quay", "use": "working_quay", "points": QUAY,
                   "source": "a test quay"},
                  {"name": "float_a", "kind": "pontoon", "points": [[10.0, -19.0], [10.0, -6.0]]}],
        "land": [{"name": "apron", "points": LAND, "height_m": 0.4, "source": "a test apron"}],
        "features": [{"id": "ground", "kind": "area", "use": "fishing_ground",
                      "points": [[20.0, 20.0], [70.0, 20.0], [70.0, 60.0], [20.0, 60.0]]},
                     {"id": "lane", "kind": "lane", "use": "traffic_lane", "points": [[-20.0, 40.0], [80.0, 45.0]]}],
        "structures": [{"type": "mooring_block", "x": 45.0, "y": 5.0, "yaw_deg": 20.0, "length": 1.0,
                        "width": 1.0, "proud_height": 0.6, "material": "concrete",
                        "params": {"name": "block_a", "evidence": "a test block"}}],
        "sediment": {"reference": "sand", "finer": "coarse_sand", "coarser": "rocks", "heterogeneity": "low"},
    }
    block.update(env)
    return {"format": "blueboat_campaign/1", "name": "harbour", "path": "depth_test_path.yaml",
            "world": {"margin_m": 25, "resolution_m": 0.10},
            "defaults": {"uncertain_materials": "low"},
            "environment": block,
            "placement": {"profile": "working_port", "density_level": 8, "structures": "keep"}}


def write(files, doc, name="harbour.yaml"):
    f = files / name
    f.write_text(yaml.safe_dump(doc, sort_keys=False))
    return f


def state_of(files, doc, name, seed=3):
    """``state_from_environment`` of a specification written beside the test's files."""
    from blueboat_harbour_population import sim_adapter as sa
    loaded = sa.load_spec(write(files, doc, name))
    return sa.state_from_environment(sa._campaign_spec(loaded, True), loaded["environment"], "nominal", seed)


@pytest.fixture(scope="module")
def built(files, tmp_path_factory):
    """The specification's plan, the environment-only world and the populated world."""
    from blueboat_harbour_population import sim_adapter as sa
    from blueboat_sss_sim.worldgen import world_folder as wf
    root = tmp_path_factory.mktemp("worlds")
    f = write(files, spec())
    doc = sa.load_spec(f)
    _state, _extras, plan, env = sa.plan_world(doc, "nominal", 7)
    state, extras = sa.environment_world(doc, "nominal", 7)
    env_dir = wf.build_world_folder(state, root / "env", extras=extras)
    man = sa.run(f, [7], root / "pop")
    return {"doc": doc, "plan": plan, "env": env, "env_dir": env_dir, "root": root,
            "pop_dir": root / "pop" / man["worlds"][0]["world"], "manifest": man}


def sources(plan):
    return {f["id"]: (f["kind"], f["use"], f["expected"]) for f in plan.report["features"]}


def test_the_built_environment_world_plans_as_the_specification(built):
    from blueboat_harbour_population import sim_adapter as sa
    from blueboat_sss_sim.worldgen import world_folder as wf
    state = wf.load_builder_state(built["env_dir"])
    assert state.objects.placed == [] and [e["type"] for e in state.objects.explicit] == ["mooring_block"]
    env5 = sa.environment_from_state(state, features=[])          # Screen 5's call, word for word
    assert env5.digest() == built["env"].digest()
    plan5 = populate(env5, dict(built["plan"].params), built["plan"].seed)
    assert plan5.digest() == built["plan"].digest()
    assert sources(plan5) == sources(built["plan"])
    assert plan5.report["expected"] == built["plan"].report["expected"]
    assert len(plan5.instances) == len(built["plan"].instances) > 5
    assert sa.environment_from_world(built["env_dir"]).digest() == built["env"].digest()


def test_the_populated_world_keeps_its_environment(built):
    from blueboat_harbour_population import sim_adapter as sa
    from blueboat_sss_sim.worldgen import world_folder as wf
    w = built["manifest"]["worlds"][0]
    assert w["environment_digest"] == built["env"].digest() and w["plan_digest"] == built["plan"].digest()
    state = wf.load_builder_state(built["pop_dir"])
    assert state.placement["mode"] == "mission_like" and state.placement["environment"]["features"]
    env5 = sa.environment_from_state(state, features=[])
    assert env5.digest() == w["environment_digest"]
    assert sa.environment_from_world(built["pop_dir"]).digest() == w["environment_digest"]
    again = populate(env5, state.placement["params"], state.placement["seed"])
    assert again.digest() == w["plan_digest"]


def test_a_wall_of_several_points_is_one_source(built):
    feats = {f.id: f for f in built["env"].features}
    assert "quay_w" in feats and not [i for i in feats if i.startswith("quay_w_")]
    q = feats["quay_w"]
    assert q.kind == "quay" and q.use == "working_quay" and len(q.points) == 3
    assert q.clear_m == pytest.approx(0.15)                        # half the wall's 0.30 m
    assert feats["float_a"].kind == "pontoon" and feats["ground"].use == "fishing_ground"
    assert feats["lane"].kind == "lane" and feats["block_a"].kind == "mooring"
    src = sources(built["plan"])
    assert src["quay_w"][1] == "working_quay" and src["lane"][1] == "traffic_lane"
    meta = yaml.safe_load((built["env_dir"] / "metadata.yaml").read_text())
    assert [w["name"] for w in meta["walls"]] == ["quay_w_s01", "quay_w_s02", "float_a"]


def test_metadata_records_the_environment(built):
    meta = yaml.safe_load((built["env_dir"] / "metadata.yaml").read_text())
    env = meta["environment"]
    by = {f["id"]: f for f in env["features"]}
    assert set(by) == {"quay_w", "ground", "lane"}
    assert by["quay_w"]["use"] == "working_quay" and len(by["quay_w"]["points"]) == 3
    assert by["ground"]["kind"] == "area" and len(by["ground"]["points"]) == 4
    off = meta["source_path"]["design_offset"]
    assert by["ground"]["points"][0] == [round(20.0 - off[0], 3), round(20.0 - off[1], 3)]   # the world frame
    walls = {w["name"]: w for w in env["walls"]}
    assert walls["quay_w"]["use"] == "working_quay" and walls["quay_w"]["source"] == "a test quay"
    assert walls["quay_w"]["segments"] == 2 and walls["float_a"]["kind"] == "pontoon"
    assert env["land"][0]["name"] == "apron" and env["land"][0]["source"] == "a test apron"
    assert meta["structures"] == [{"kind": "quay", "name": "quay_w"}, {"kind": "area", "name": "ground"},
                                  {"kind": "lane", "name": "lane"}]
    blocks = [o for o in meta["objects"] if o["type"] == "mooring_block"]
    assert len(blocks) == 1 and blocks[0]["role"] == "structure" and blocks[0]["params"]["name"] == "block_a"
    assert meta["conditions"]["clutter"]["small_litter_layer"].startswith("left out")
    assert "placement" not in meta and meta["object_counts"].get("target", 0) == 0
    assert meta["object_counts"].get("structure") == {"mooring_block": 1}


def test_land_is_dry_and_there_is_no_water_behind_the_quay(built):
    meta = yaml.safe_load((built["env_dir"] / "metadata.yaml").read_text())
    off = meta["source_path"]["design_offset"]
    with np.load(built["env_dir"] / "depth.npz") as z:
        h, ox, oy, res = z["height"], float(z["origin_x"]), float(z["origin_y"]), float(z["resolution"])

    def at(e, n):
        return float(h[int(round((n - off[1] - oy) / res)), int(round((e - off[0] - ox) / res))])
    for e, n in ((0.0, -25.0), (60.0, -27.0), (-28.0, -21.0), (30.0, -23.0)):
        assert at(e, n) == pytest.approx(0.4, abs=0.03), (e, n)      # the apron, at its height (plus the texture)
    for e, n in ((0.0, -15.0), (30.0, -20.0), (60.0, 0.0)):
        assert at(e, n) < -4.5, (e, n)                               # water in front of the quay
    state = yaml.safe_load((built["env_dir"] / "builder_state.yaml").read_text())
    ids = [L["id"] for L in state["depth"]["layers"]]
    assert ids[:3] == ["env_smooth", "env_water_level", "env_shift"] and ids[-1] == "land_apron"
    assert "small_litter" not in ids


def test_the_seabed_declaration_survives_screen_3(built):
    """Screen 3 rebuilds the depth state without ``seabed`` (``depth_page._collect``); the record brings it
    back on entry to Screen 5 (``builder.placement.reassert``), and the planner's class does not depend on it."""
    from blueboat_harbour_population import sim_adapter as sa
    from blueboat_sss_sim.builder import placement as bp
    from blueboat_sss_sim.builder.state import DepthState
    from blueboat_sss_sim.worldgen import world_folder as wf
    state = wf.load_builder_state(built["env_dir"])
    assert state.depth.seabed == DECLARATION and bp.mode_of(state) == bp.MANUAL and bp.extras(state) is None
    d = state.depth
    state.depth = DepthState(mode="file", file=d.file, file_values=d.file_values, texture=copy.deepcopy(d.texture),
                             material=d.material, paint=copy.deepcopy(d.paint),
                             material_paint=copy.deepcopy(d.material_paint), layers=copy.deepcopy(d.layers))
    assert state.depth.seabed == {}
    assert sa.environment_from_state(state, features=[]).digest() == built["env"].digest()
    bp.reassert(state)
    assert state.depth.seabed == DECLARATION


def test_a_screen_5_plan_and_its_re_save(built, tmp_path):
    """What Screen 5 does with a plan (``apply_plan`` on the state, ``builder.placement.extras`` to the build),
    without Qt: the re-saved world is the headless world, writes the envelope, and still knows its features."""
    from blueboat_harbour_population import sim_adapter as sa
    from blueboat_sss_sim.builder import placement as bp
    from blueboat_sss_sim.core import provenance
    from blueboat_sss_sim.worldgen import world_folder as wf
    state = wf.load_builder_state(built["env_dir"])
    env5 = sa.environment_from_state(copy.deepcopy(state), features=[])
    plan5 = populate(env5, dict(built["plan"].params), built["plan"].seed)
    sa.apply_plan(state, plan5, run={"sonar_fouling": sa.fouling_of(plan5.seed)})
    assert state.placement["environment"]["features"] and state.placement["mode"] == "mission_like"
    state.world_name = "resaved"
    out = wf.build_world_folder(state, tmp_path, extras=bp.extras(state))
    meta = yaml.safe_load((out / "metadata.yaml").read_text())
    pl = meta["placement"]
    assert check_world_block(pl) == [] and 0.5 < pl["envelope"]["share_in_band"] <= 1.0
    assert pl["plan_digest"] == built["plan"].digest() and pl["environment_digest"] == built["env"].digest()
    a, b = provenance.content_digest(out)["parts"], provenance.content_digest(built["pop_dir"])["parts"]
    for part in ("scene.npz", "depth.npz", "ground_truth_rasters.npz"):
        assert a[part] == b[part], part                              # the same terrain, objects and ground truth
    again = wf.load_builder_state(out)
    assert sa.environment_from_state(again, features=[]).digest() == built["env"].digest()
    assert again.depth.seabed["patch_classes"] == ["coarse_sand", "rocks"]


def test_wall_rules(files, tmp_path):
    from blueboat_harbour_population import sim_adapter as sa
    # vertices half a metre apart are thinned: the generator merges wall ends closer than 0.75 m
    dense = [[-20.0 + 0.5 * k, -20.0] for k in range(41)] + [[30.0, -22.0]]
    doc = spec(walls=[{"name": "dense", "kind": "quay", "points": dense}], land=[], structures=[])
    state, _extras, records = state_of(files, doc, "dense.yaml")
    segs = [w for w in state.walls if w["name"].startswith("dense")]
    assert 2 <= len(segs) <= 26 and all(np.hypot(w["x1"] - w["x0"], w["y1"] - w["y0"]) >= 0.8 for w in segs)
    assert (segs[0]["x0"], segs[0]["y0"]) == (-20.0, -20.0) and (segs[-1]["x1"], segs[-1]["y1"]) == (30.0, -22.0)
    assert [r["id"] for r in records if r["id"] == "dense"] == ["dense"]
    long = [[-29.0 + 1.0 * k, -20.0 + (0.9 if k % 2 else 0.0)] for k in range(102)]
    with pytest.raises(ValueError, match="long_one.*99"):
        state_of(files, spec(walls=[{"name": "long_one", "kind": "quay", "points": long}], land=[], structures=[]),
                 "long.yaml")
    with pytest.raises(ValueError, match="quay_s01"):
        state_of(files, spec(walls=[{"name": "quay_s01", "kind": "quay", "points": QUAY}], land=[], structures=[]),
                 "name.yaml")


def test_structure_and_land_rules(files):
    def state_of_(doc, name):
        return state_of(files, doc, name)
    with pytest.raises(KeyError, match="proud_height"):              # a pinned instance states everything
        state_of_(spec(structures=[{"type": "mooring_block", "x": 45.0, "y": 5.0, "length": 1.0, "width": 1.0,
                                   "material": "concrete", "params": {"name": "b"}}]), "s1.yaml")
    with pytest.raises(ValueError, match="tire_car"):                # structures only
        state_of_(spec(structures=[{"type": "tire_car", "x": 45.0, "y": 5.0, "length": 0.6, "width": 0.6,
                                   "proud_height": 0.2, "material": "rubber", "params": {"name": "t"}}]), "s2.yaml")
    with pytest.raises(ValueError, match="ml_x"):                    # the plan's own prefix
        state_of_(spec(structures=[{"type": "mooring_block", "x": 45.0, "y": 5.0, "length": 1.0, "width": 1.0,
                                   "proud_height": 0.6, "material": "concrete", "params": {"name": "ml_x"}}]), "s3.yaml")
    with pytest.raises(ValueError, match="three"):
        state_of_(spec(land=[{"name": "thin", "points": [[0.0, 0.0], [1.0, 1.0]]}]), "l1.yaml")
    with pytest.raises(KeyError, match="heigth_m"):
        state_of_(spec(land=[{"name": "typo", "points": LAND, "heigth_m": 0.4}]), "l2.yaml")
    # GPS points give the same state as metres
    from blueboat_sss_sim.core.geo import GeoAnchor
    a = GeoAnchor(lat0=LAT0, lon0=LON0)
    gps = [[float(a.to_latlon(x, y)[0]), float(a.to_latlon(x, y)[1])] for x, y in LAND]
    la, lo = (float(v) for v in a.to_latlon(45.0, 5.0))
    s1, _, _ = state_of_(spec(), "m.yaml")
    s2, _, _ = state_of_(spec(land=[{"name": "apron", "points_gps": gps, "height_m": 0.4, "source": "a test apron"}],
                             structures=[{"type": "mooring_block", "lat": la, "lon": lo, "yaw_deg": 20.0,
                                          "length": 1.0, "width": 1.0, "proud_height": 0.6, "material": "concrete",
                                          "params": {"name": "block_a", "evidence": "a test block"}}]), "g.yaml")
    p1 = np.asarray(s1.depth.layers[-1]["geometry"]["points"])
    p2 = np.asarray(s2.depth.layers[-1]["geometry"]["points"])
    assert np.abs(p1 - p2).max() < 0.01
    assert abs(s1.objects.explicit[0]["x"] - s2.objects.explicit[0]["x"]) < 0.01


def test_raster_resolution(files, tmp_path):
    """``environment.raster_resolution_m`` asks the generator for a finer raster than its automatic one (a
    world side over 150 m is coarsened): simulator v1.1 carries it in the builder state."""
    from blueboat_harbour_population import sim_adapter as sa
    from blueboat_sss_sim.builder.state import BuilderState
    from blueboat_sss_sim.worldgen import world_folder as wf
    doc = spec(limits={"x_min": -50.0, "y_min": -30.0, "x_max": 110.0, "y_max": 70.0}, raster_resolution_m=0.10,
               land=[], structures=[], walls=[])
    if "raster_resolution_m" not in {f.name for f in dataclasses.fields(BuilderState)}:
        with pytest.raises(RuntimeError, match="v1.1"):
            state_of(files, doc, "fine.yaml")
        pytest.skip("this simulator has no BuilderState.raster_resolution_m (before v1.1)")
    state, extras, _ = state_of(files, doc, "fine.yaml")
    assert wf.world_block(state)[2] == pytest.approx(0.10) and wf._resolution_for(160.0, 100.0) > 0.14
    assert extras["environment"]["raster_resolution_m"] == 0.10
    out = wf.build_world_folder(state, tmp_path, extras=extras)
    with np.load(out / "depth.npz") as z:
        assert float(z["resolution"]) == pytest.approx(0.10) and z["height"].shape == (1001, 1601)
    again = wf.load_builder_state(out)
    assert again.raster_resolution_m == pytest.approx(0.10)
    assert sa.environment_from_state(again, features=[]).origin["build_resolution_m"] == pytest.approx(0.10)



def test_a_closed_float_outline_and_automatic_structures(files):
    """A float drawn as an area is a pontoon wall round its outline (first point = last point). With
    ``structures: auto`` the planner sets guide piles along a pontoon's centre line: an outline has none, and
    must be passed over without a division by its zero length."""
    import warnings
    ring = [[30.0, 10.0], [42.0, 10.0], [42.0, 14.0], [30.0, 14.0], [30.0, 10.0]]
    doc = spec(walls=[{"name": "ring_float", "kind": "pontoon", "points": ring},
                      {"name": "quay_w", "kind": "quay", "points": QUAY}], structures=[])
    state, _extras, records = state_of(files, doc, "ring.yaml")
    from blueboat_harbour_population import sim_adapter as sa
    env = sa.environment_from_state(state, features=records)
    f = {q.id: q for q in env.features}["ring_float"]
    assert f.kind == "pontoon" and np.allclose(f.points[0], f.points[-1]) and len(f.points) == 5
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        plan = populate(env, {"profile": "marina", "density_level": 5, "seed": 11, "structures": "auto"})
    for e in plan.structures:
        assert np.isfinite(e["x"]) and np.isfinite(e["y"])
