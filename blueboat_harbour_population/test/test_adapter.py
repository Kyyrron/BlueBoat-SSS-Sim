"""The simulator adapter: mission-like worlds built with the FROZEN world
generator, without changing it. Skipped, visibly, when ``blueboat_sss_sim``
cannot be imported."""
import copy

import numpy as np
import pytest
import yaml

from conftest import needs_simulator

from blueboat_harbour_population import TARGETS, populate
from blueboat_harbour_population.schema import check_object_params, check_world_block

pytestmark = needs_simulator

TRAJECTORY = """\
format: blueboat_trajectory/1
name: pop_test_lawn
loop: false
geo_anchor: {lat0: 34.4082, lon0: 133.5248, theta_deg: 0.0}
points:
- [0.0, 0.0, 0.0, 0.0]
- [60.0, 40.0, 0.0, 0.0]
- [80.0, 40.0, 12.0, 1.5708]
- [140.0, 0.0, 12.0, 3.1416]
- [160.0, 0.0, 24.0, 1.5708]
- [220.0, 40.0, 24.0, 0.0]
"""

SPEC = {
    "format": "blueboat_campaign/1", "name": "pop", "path": "pop_test_lawn.yaml",
    "world": {"margin_m": 22, "resolution_m": 0.10},
    "defaults": {"depth_band_m": [5.4, 6.2], "grade_max": 0.003,
                 "basin": {"quays": 2, "pontoon": True, "mooring_blocks": 1},
                 "sediment": {"reference": "sand", "finer": "soft_mud", "coarser": "gravel",
                              "heterogeneity": "low"},
                 "clutter": "low", "uncertain_materials": "low"},
    "placement": {"profile": "fishing_harbour", "density_level": 6},
    "cases": [{"name": "l6"}, {"name": "l9", "placement": {"density_level": 9}}],
}


@pytest.fixture(scope="module")
def spec_file(tmp_path_factory):
    d = tmp_path_factory.mktemp("spec")
    (d / "pop_test_lawn.yaml").write_text(TRAJECTORY)
    f = d / "pop.yaml"
    f.write_text(yaml.safe_dump(SPEC, sort_keys=False))
    return f


@pytest.fixture(scope="module")
def built(spec_file, tmp_path_factory):
    from blueboat_harbour_population import sim_adapter as sa
    root = tmp_path_factory.mktemp("worlds")
    man = sa.run(spec_file, [3], root, ["l6"])
    return root, man


def test_the_simulator_is_the_frozen_one():
    from blueboat_harbour_population import sim_adapter as sa
    ident = sa.simulator_identity()
    assert len(ident["code_digest"]) == 16 and len(ident["generator_digest"]) == 16


def test_spec_keys():
    from blueboat_harbour_population import sim_adapter as sa
    spec, placement, env = sa.split_spec(SPEC)
    assert "placement" not in spec and placement["density_level"] == 6 and env is None
    assert sa.case_placement(SPEC, "l9")["density_level"] == 9 and sa.case_placement(SPEC, "l9")["profile"] == "fishing_harbour"
    assert sa.is_mission(placement) and not sa.is_mission(None) and not sa.is_mission({"mode": "legacy"})
    bad = copy.deepcopy(SPEC)
    bad["placement"]["profil"] = "x"
    with pytest.raises(KeyError, match="profil"):
        from blueboat_harbour_population import PlacementParams
        PlacementParams.from_dict(sa.case_placement(bad, "l6"))


def test_world_is_built_with_its_records(built):
    root, man = built
    w = man["worlds"][0]
    assert w["skipped"] is False and len(w["content_digest"]) == 64
    wdir = root / w["world"]
    meta = yaml.safe_load((wdir / "metadata.yaml").read_text())
    assert meta["format"] == "blueboat_world_meta/1" and meta["schema_version"] == 2
    pl = meta["placement"]
    assert check_world_block(pl) == []
    assert pl["mode"] == "mission_like" and pl["profile"] == "fishing_harbour" and pl["density_level"] == 6
    assert pl["seed"] == 3 and pl["split"] == "development" and pl["plan_digest"] == w["plan_digest"]
    assert "population" not in meta                              # the legacy model's record is gone
    assert meta["conditions"]["objects"]["density_per_ha"] == 0
    assert 0.2 <= meta["run"]["sonar_fouling"] <= 1.0
    # the world's seabed declaration lists the litter's hard class
    assert "gravel" in meta["seabed"]["patch_classes"] and meta["seabed"]["reference"] == "sand"
    litter = [o for o in meta["objects"] if o["role"] in ("target", "clutter")]
    assert len(litter) == pl["realised"]["targets"] + pl["realised"]["debris"] and litter
    for o in litter:
        assert check_object_params(o["params"], o["type"]) == []
        assert o["params"]["spec"] == 2
        assert o["hotspot"] == ("" if o["params"]["source"]["feature"] == "far_field"
                                else o["params"]["source"]["feature"])
        assert o["seabed_class"] == o["params"]["sediment_under"] or o["burial"] >= 0.0
        if o["role"] == "target":
            assert o["type"] in TARGETS and o["family"].startswith("T")
    # the ground truth the simulator computes agrees with the plan's own figures
    assert meta["detectable_targets"] == pl["realised"]["detectable_targets"]
    assert sum(meta["object_counts"].get("target", {}).values()) == pl["realised"]["targets"]
    state = yaml.safe_load((wdir / "builder_state.yaml").read_text())
    ids = [L["id"] for L in state["depth"]["layers"]]
    assert "small_litter" not in ids and "ml_litter_base" in ids      # the level's litter replaced the preset's
    assert state["objects"]["pinned"] is True and len(state["objects"]["placed"]) == len(litter)
    # the card lies beside the worlds, never in the world folder
    card = root / w["card"]
    assert card.is_file() and (card.parent / "world_card.png").is_file() and ".population" in str(card)
    assert not (wdir / "world_card.md").exists()


def test_depth_recorded_per_object_is_the_built_floor(built):
    """``depth_m`` of an object is the floor of the built world at its place."""
    root, man = built
    wdir = root / man["worlds"][0]["world"]
    meta = yaml.safe_load((wdir / "metadata.yaml").read_text())
    with np.load(wdir / "depth.npz") as z:
        h, ox, oy, res = z["height"], float(z["origin_x"]), float(z["origin_y"]), float(z["resolution"])
    worst = 0.0
    for o in meta["objects"]:
        if o["role"] not in ("target", "clutter"):
            continue
        i, j = int(round((o["x"] - ox) / res)), int(round((o["y"] - oy) / res))
        win = h[max(j - 5, 0):j + 6, max(i - 5, 0):i + 6]
        worst = max(worst, abs(-float(np.median(win)) - o["params"]["depth_m"]))
        assert o["params"]["in_envelope"] and 4.5 <= o["params"]["altitude_m"] <= 7.0
    assert worst < 0.10, worst


def test_the_content_digest_repeats(spec_file, built, tmp_path):
    from blueboat_harbour_population import sim_adapter as sa
    root, man = built
    again = sa.run(spec_file, [3], tmp_path / "again", ["l6"])
    assert again["worlds"][0]["content_digest"] == man["worlds"][0]["content_digest"]
    assert again["worlds"][0]["plan_digest"] == man["worlds"][0]["plan_digest"]
    # an existing world is never touched
    third = sa.run(spec_file, [3], root, ["l6"])
    assert third["worlds"][0].get("skipped_last_run") is True


def test_same_basin_and_fouling_as_the_legacy_world_of_that_seed(spec_file, built, tmp_path):
    """A mission-like world shares its basin, seabed, structures and fouling
    with the frozen generator's own world of the same seed."""
    from blueboat_sss_sim.worldgen import campaign
    from blueboat_harbour_population import sim_adapter as sa
    root, man = built
    doc = sa.load_spec(spec_file)
    legacy = campaign.check_spec(sa.split_spec(doc)[0] | {"cases": [{"name": "l6"}]})
    state, extras = campaign.plan_world(legacy, "l6", 3)
    meta = yaml.safe_load((root / man["worlds"][0]["world"] / "metadata.yaml").read_text())
    assert meta["run"]["sonar_fouling"] == extras["run"]["sonar_fouling"]
    assert meta["conditions"]["drawn"] == extras["conditions"]["drawn"]
    assert [s for s in meta["structures"]] == [yaml.safe_load(yaml.safe_dump(s)) for s in extras["structures"]]


def test_a_spec_without_placement_goes_to_the_frozen_generator(spec_file, tmp_path):
    from blueboat_sss_sim.worldgen import campaign
    from blueboat_harbour_population import sim_adapter as sa
    doc = sa.load_spec(spec_file)
    plain_spec = {k: v for k, v in doc.items() if k != "placement"}
    plain_spec["cases"] = [{"name": "nominal"}]
    f = tmp_path / "plain.yaml"
    f.write_text(yaml.safe_dump(plain_spec, sort_keys=False))
    a = sa.run(f, [2], tmp_path / "a")
    b = campaign.run_campaign(f, [2], tmp_path / "b")
    assert a["worlds"][0]["content_digest"] == b["worlds"][0]["content_digest"]
    meta = yaml.safe_load((tmp_path / "a" / a["worlds"][0]["world"] / "metadata.yaml").read_text())
    assert "placement" not in meta and "population" in meta
    legacy = dict(doc, placement={"mode": "legacy"}, cases=[{"name": "nominal"}])
    f2 = tmp_path / "legacy.yaml"
    f2.write_text(yaml.safe_dump(legacy, sort_keys=False))
    c = sa.run(f2, [2], tmp_path / "c")
    assert c["worlds"][0]["content_digest"] == b["worlds"][0]["content_digest"]


def test_environment_of_a_built_world_and_replanning(built):
    """The environment read back from a world folder is the one the plan was
    made on: planning again gives the same plan."""
    from blueboat_harbour_population import sim_adapter as sa
    root, man = built
    wdir = root / man["worlds"][0]["world"]
    env = sa.environment_from_world(wdir)
    meta = yaml.safe_load((wdir / "metadata.yaml").read_text())
    assert env.digest() == meta["placement"]["environment_digest"]
    ids = {f.id for f in env.features}
    assert {"berth", "pontoon", "block_1"} <= ids and any(i.startswith("quay_") for i in ids)
    assert env.transducer_depth_m == 0.22 and abs(env.cell_m - 1.0) < 1e-9
    blk = {"profile": "fishing_harbour", "density_level": 6, "seed": 3,
           "advanced": {"uncertain_materials": "low"}}
    assert populate(env, blk).digest() == meta["placement"]["plan_digest"]


def test_apply_is_idempotent(spec_file):
    from blueboat_harbour_population import sim_adapter as sa
    doc = sa.load_spec(spec_file)
    state, extras, plan, env = sa.plan_world(doc, "l6", 4)
    before = yaml.safe_dump(state.to_dict(), sort_keys=True)
    sa.apply_plan(state, plan)
    sa.apply_plan(state, plan)
    assert yaml.safe_dump(state.to_dict(), sort_keys=True) == before
    assert [L["id"] for L in state.depth.layers].count("ml_litter_base") == 1
    assert sa.strip_reserved(state.depth.layers) == [L for L in state.depth.layers if not L["id"].startswith("ml_")]
