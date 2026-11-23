"""The planner on synthetic basins: determinism, counts, classes, spacing,
envelope, groups, the split guard, nested levels."""
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from blueboat_harbour_population import (DEBRIS, TARGETS, PlacementParams, SplitError, allowed_mask,
                                         default_parameters, populate, synthetic_basin)
from blueboat_harbour_population.plan import canonical
from blueboat_harbour_population.rng import dexp, poisson_from_uniform
from blueboat_harbour_population.schema import check_object_params, check_world_block, is_plain

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ("fishing_harbour", "trap_harbour", "city_harbour", "marina", "working_port", "generic", "auto")


def key(e):
    s = e["params"]["source"]
    return (s["feature"], e["type"], s["process"], str(s["index"]))


def test_deterministic_helpers():
    x = np.linspace(-40.0, 0.0, 4001)
    assert np.max(np.abs(dexp(x) - np.exp(x)) / np.exp(x)) < 1e-12
    assert dexp(-800.0) == 0.0
    rng = np.random.default_rng(1)
    for mean in (0.3, 2.0, 17.5, 240.0, 3000.0):
        u = rng.random(4000)
        k = np.array([poisson_from_uniform(v, mean) for v in u])
        assert abs(k.mean() - mean) < 4.0 * math.sqrt(mean / len(u)) + 0.02
        assert abs(k.var() - mean) < 0.15 * mean + 0.05


def test_same_inputs_same_plan(basin):
    blk = {"profile": "fishing_harbour", "density_level": 7, "seed": 11}
    a, b = populate(basin, blk), populate(basin, blk)
    assert canonical(a.to_dict()) == canonical(b.to_dict())
    assert a.digest() == b.digest()
    assert populate(basin, {**blk, "seed": 12}).digest() != a.digest()


def test_same_plan_in_another_process(basin):
    code = ("import sys; sys.path.insert(0, %r)\n"
            "from blueboat_harbour_population import populate, synthetic_basin\n"
            "from blueboat_harbour_population.plan import canonical\n"
            "import hashlib\n"
            "env = synthetic_basin(140.0, 130.0, depth_m=6.0, grade=0.003, quays=('south', 'west'), pontoon=True,"
            " mooring=1, mud_patch=True)\n"
            "for prof, lvl, seed in (('fishing_harbour', 7, 11), ('trap_harbour', 5, 3), ('city_harbour', 9, 5)):\n"
            "    p = populate(env, {'profile': prof, 'density_level': lvl, 'seed': seed})\n"
            "    print(p.digest(), hashlib.sha256(canonical(p.to_dict()).encode()).hexdigest())\n") % str(ROOT)
    import hashlib
    mine = []
    for prof, lvl, seed in (("fishing_harbour", 7, 11), ("trap_harbour", 5, 3), ("city_harbour", 9, 5)):
        p = populate(basin, {"profile": prof, "density_level": lvl, "seed": seed})
        mine.append(f"{p.digest()} {hashlib.sha256(canonical(p.to_dict()).encode()).hexdigest()}")
    for _ in range(2):
        out = subprocess.run([sys.executable, "-B", "-c", code], capture_output=True, text=True, check=True)
        assert out.stdout.split("\n")[:3] == mine


def test_same_plan_under_the_other_numpy(basin):
    """The venv's numpy (2.x) gives the plans of the system's (1.26)."""
    other = Path.home() / "ros2_ws" / ".venv" / "bin" / "python3"
    if not other.exists() or Path(sys.executable).resolve() == other.resolve() and "venv" in sys.prefix:
        pytest.skip("no second interpreter")
    code = ("import sys; sys.path.insert(0, %r)\n"
            "import numpy\n"
            "from blueboat_harbour_population import populate, synthetic_basin\n"
            "env = synthetic_basin(140.0, 130.0, depth_m=6.0, grade=0.003, quays=('south', 'west'), pontoon=True,"
            " mooring=1, mud_patch=True)\n"
            "print(numpy.__version__)\n"
            "for prof in %r:\n"
            "    for lvl in (2, 5, 8, 10):\n"
            "        print(populate(env, {'profile': prof, 'density_level': lvl, 'seed': 21}).digest())\n"
            ) % (str(ROOT), PROFILES)
    out = subprocess.run([str(other), "-B", "-c", code], capture_output=True, text=True)
    if out.returncode != 0:
        pytest.skip(f"the second interpreter cannot run the package: {out.stderr[-200:]}")
    lines = out.stdout.split()
    if lines[0] == np.__version__:
        pytest.skip("the second interpreter has the same numpy")
    mine = [populate(basin, {"profile": prof, "density_level": lvl, "seed": 21}).digest()
            for prof in PROFILES for lvl in (2, 5, 8, 10)]
    assert lines[1:] == mine


def test_counts_within_poisson_intervals(basin):
    """Over many seeds the number of objects follows the expected count."""
    for prof, lvl in (("fishing_harbour", 5), ("trap_harbour", 3), ("generic", 5)):
        n, exp, removed = [], None, 0
        for seed in range(1, 121):
            plan = populate(basin, {"profile": prof, "density_level": lvl, "seed": seed, "advanced": {"groups": False}})
            z = plan.report["realised"]
            n.append(z["targets"] + z["debris"] + z["removed_by_spacing"] + z["no_room"])
            exp = plan.report["expected"]["total"] / plan.report["retention"]
            removed += z["removed_by_spacing"]
        n = np.array(n, dtype=float)
        # the mean of 120 Poisson counts lies within 4 standard errors of the expectation
        assert abs(n.mean() - exp) < 4.0 * math.sqrt(exp / len(n)), (prof, n.mean(), exp)
        # and the counts are Poisson-like: variance over mean near 1
        assert 0.6 < n.var(ddof=1) / n.mean() < 1.6, (prof, n.var(ddof=1) / n.mean())


def test_density_follows_the_level(basin):
    p = default_parameters()
    prev = 0.0
    for lvl in (1, 3, 5, 7, 10):
        plan = populate(basin, {"profile": "generic", "density_level": lvl, "seed": 4})
        r = plan.report
        assert r["density_per_ha"] == p("levels.density_per_ha")[lvl]
        # expected objects = the level's density times the active band (no activity here), plus the far field
        far = r["far_field_ha"] * r["density_per_ha"] * p("far_field_share")
        assert abs(r["expected"]["total"] - (r["density_per_ha"] * r["active_band_ha"] + far)) < 0.02 * r["expected"]["total"] + 0.02
        assert r["expected"]["total"] > prev
        prev = r["expected"]["total"]


def test_fixed_count(basin):
    plan = populate(basin, {"profile": "generic", "density_level": 6, "seed": 2,
                            "advanced": {"count": "fixed", "groups": False}})
    z = plan.report["realised"]
    placed = z["targets"] + z["debris"] + z["removed_by_spacing"] + z["no_room"]
    assert abs(placed - plan.report["expected"]["total"]) <= 0.5 * 8 + 1      # each stream rounds its own count


def test_class_selection(basin):
    base = {"profile": "fishing_harbour", "density_level": 8, "seed": 6}
    full = populate(basin, base)
    sel = populate(basin, {**base, "classes": ["tire_car", "trap_pot"]})
    # a deselected class is absent
    assert set(sel.counts()) <= {"tire_car", "trap_pot", DEBRIS}
    # the level fixes the total: the selected classes share the whole target part
    assert abs(sel.report["expected"]["targets"] - full.report["expected"]["targets"]) < 1e-6 * 100
    assert abs(sel.report["expected"]["debris"] - full.report["expected"]["debris"]) < 1e-6 * 100
    assert abs(sum(sel.report["expected_shares"].values()) - 1.0) < 1e-3
    # balanced: equal expected counts
    bal = populate(basin, {**base, "mix": "balanced"})
    sh = list(bal.report["expected_shares"].values())
    assert len(sh) == 7 and max(sh) - min(sh) < 1e-3
    # a mapping: its weights
    cus = populate(basin, {**base, "mix": {"tire_car": 3.0, "pipe_steel": 1.0}})
    assert abs(cus.report["expected_shares"]["tire_car"] - 0.75) < 1e-3
    assert set(cus.counts()) <= {"tire_car", "pipe_steel", DEBRIS}
    # no debris: the debris goes, the targets stay
    nod = populate(basin, {**base, "clutter": {"debris": False}})
    assert DEBRIS not in nod.counts()
    assert abs(nod.report["expected"]["targets"] - full.report["expected"]["targets"]) < 1e-6 * 100
    # ... and the targets placed are the very same objects
    assert [e for e in nod.instances] == [e for e in full.instances if e["type"] != DEBRIS] or \
        {key(e) for e in nod.instances} >= {key(e) for e in full.instances if e["type"] != DEBRIS}


def test_spacing(basin):
    p = default_parameters()
    for prof in ("fishing_harbour", "city_harbour"):
        plan = populate(basin, {"profile": prof, "density_level": 10, "seed": 9})
        E = plan.instances
        xy = plan.xy()
        clear = p("spacing.clearance_m")
        for i in range(len(E)):
            for j in range(i + 1, len(E)):
                gi, gj = E[i]["params"]["group_id"], E[j]["params"]["group_id"]
                if gi is not None and gi == gj:
                    continue                                  # inside a group there is no spacing
                d = math.hypot(xy[i, 0] - xy[j, 0], xy[i, 1] - xy[j, 1])
                ri = 0.5 * math.hypot(E[i]["length"], E[i]["width"])
                rj = 0.5 * math.hypot(E[j]["length"], E[j]["width"])
                both = E[i]["type"] != DEBRIS and E[j]["type"] != DEBRIS
                if gi is None and gj is None:
                    need = max(3.0 if both else 1.0, ri + rj + clear)
                else:
                    need = ri + rj + clear                    # a group keeps its disc clear: at least no overlap
                assert d >= need - 2e-3, (E[i]["type"], E[j]["type"], d, need)


def test_every_object_is_on_water_in_the_band_and_clear(basin):
    p = default_parameters()
    lo, hi = p("envelope.altitude_m")
    m = p("envelope.heave_margin_m")
    for prof in PROFILES:
        plan = populate(basin, {"profile": prof, "density_level": 9, "seed": 13})
        allowed, _ = allowed_mask(basin, PlacementParams.from_dict({"profile": prof}), p)
        for e in plan.instances + [s for s in plan.structures if s["type"] != "chain"]:
            x, y = e["x"], e["y"]
            assert bool(basin.inside(x, y))
            j, i = basin.cell_of(x, y)
            assert basin.water[j, i]
            if e in plan.instances:
                assert allowed[j, i]
                alt = e["params"]["altitude_m"]
                assert lo + m - 0.006 <= alt <= hi - m + 0.006 and e["params"]["in_envelope"]
                rad = 0.5 * math.hypot(e["length"], e["width"])
                for f in basin.features:
                    if f.clear_m > 0.0:
                        assert float(f.distance(x, y)) >= f.clear_m + rad + 0.3 - 2e-3
                for o in basin.occupied:
                    assert math.hypot(x - o["x"], y - o["y"]) >= o["radius"] + rad + 0.3 - 2e-3


def test_outside_band_is_flagged():
    shallow = synthetic_basin(100.0, 80.0, depth_m=4.2, quays=("south",))
    plan = populate(shallow, {"profile": "generic", "density_level": 8, "seed": 1})
    assert not plan.instances and plan.report["envelope"]["share_in_band"] == 0.0
    show = populate(shallow, {"profile": "generic", "density_level": 8, "seed": 1,
                              "advanced": {"outside_band": True}})
    assert show.instances and show.report["showcase_world"]
    assert all(not e["params"]["in_envelope"] for e in show.instances)
    assert show.report["realised"]["outside_band"] == len(show.instances)


def test_groups_are_intact(basin):
    found = set()
    for seed in range(1, 40):
        plan = populate(basin, {"profile": "fishing_harbour", "density_level": 10, "seed": seed,
                                "feature_tags": {"block_1": "dumping_spot"}})
        ids = {}
        for k, e in enumerate(plan.instances):
            g = e["params"]["group_id"]
            if g is not None:
                ids.setdefault(g, []).append(k)
        assert set(ids) == {g["id"] for g in plan.groups}
        for g in plan.groups:
            found.add(g["type"])
            assert sorted(g["members"]) == sorted(ids[g["id"]]) and len(g["members"]) >= 2
            assert sum(g["classes"].values()) == len(g["members"])
            for k in g["members"]:
                e = plan.instances[k]
                assert e["params"]["source"]["process"] == g["type"]
                assert math.hypot(e["x"] - g["x"], e["y"] - g["y"]) <= g["radius_m"] + 1e-3
            if g["type"] == "tyre_line":
                assert set(g["classes"]) == {"tire_car"} and 2 <= len(g["members"]) <= 6
    assert {"tyre_line", "pile"} <= found
    # no groups when they are switched off
    plan = populate(basin, {"profile": "fishing_harbour", "density_level": 10, "seed": 3,
                            "advanced": {"groups": False}})
    assert not plan.groups and all(e["params"]["group_id"] is None for e in plan.instances)


def test_levels_are_nested(basin):
    """A lower level holds a subset of the candidates of a higher one: every
    single of the lower level that is not in the higher one lies within the
    spacing distance of an object of the higher level (the spacing rule is
    the only thing that can take it away)."""
    blk = {"profile": "working_port", "seed": 17, "advanced": {"groups": False}}
    lo, hi = populate(basin, {**blk, "density_level": 4}), populate(basin, {**blk, "density_level": 8})
    hi_by = {key(e): e for e in hi.instances}
    hi_xy = hi.xy()
    same = 0
    for e in lo.instances:
        if key(e) in hi_by:
            assert e == hi_by[key(e)]                          # the very same object
            same += 1
        else:
            d = np.hypot(hi_xy[:, 0] - e["x"], hi_xy[:, 1] - e["y"])
            assert d.min() < 3.0 + 2.5                         # displaced by a neighbour of the denser world
    assert same >= 0.8 * len(lo.instances) and len(hi.instances) > len(lo.instances)


def test_split_guard(basin):
    blk = {"profile": "generic", "density_level": 3}
    with pytest.raises(SplitError):
        populate(basin, {**blk, "seed": 7, "split": "dataset"})            # a development seed
    with pytest.raises(SplitError):
        populate(basin, {**blk, "seed": 300001, "split": "dataset"})       # an evaluation seed
    with pytest.raises(SplitError):
        populate(basin, {**blk, "seed": 100001, "split": "dataset", "harbour": "plymouth"})   # an evaluation harbour
    ok = populate(basin, {**blk, "seed": 100001, "split": "dataset", "harbour": "provincetown"})
    assert ok.report["split"]["overridden"] is False
    over = populate(basin, {**blk, "seed": 300001, "split": "dataset", "split_override": "regression test"})
    assert over.report["split"]["overridden"] and "regression test" in " ".join(over.report["warnings"])
    assert over.metadata_block()["split_override"] == "regression test"


def test_unknown_keys_raise_by_name(basin):
    for blk, word in (({"profil": "generic"}, "profil"), ({"advanced": {"burrial": "low"}}, "burrial"),
                      ({"clutter": {"litter": True}}, "litter")):
        with pytest.raises(KeyError, match=word):
            PlacementParams.from_dict(blk)
    with pytest.raises(ValueError):
        PlacementParams.from_dict({"density_level": 11})
    with pytest.raises(ValueError):
        PlacementParams.from_dict({"profile": "harbour"})
    with pytest.raises(KeyError, match="nowhere"):
        populate(basin, {"profile": "generic", "feature_tags": {"nowhere": "fender_line"}})


def test_metadata_is_complete_and_plain(basin):
    for prof in PROFILES:
        plan = populate(basin, {"profile": prof, "density_level": 8, "seed": 5})
        blk = plan.metadata_block()
        assert check_world_block(blk) == [], prof
        assert is_plain(blk) and is_plain(plan.state_block()) and is_plain(plan.to_dict())
        # the plan itself, before any conversion: what goes into a world file
        for part in (plan.instances, plan.structures, plan.layers, plan.groups, plan.report, plan.params):
            assert is_plain(part)
        import yaml
        yaml.safe_dump(plan.to_dict())                           # nothing a world file cannot hold
        for e in plan.instances:
            assert check_object_params(e["params"], e["type"]) == []
            assert e["params"]["spec"] == 2 and e["params"]["role"] in ("target", "clutter")
            assert isinstance(e["params"]["nominal_target"], bool)
            for k in ("type", "x", "y", "yaw", "length", "width", "proud_height", "burial", "fill_fraction",
                      "reflectivity", "material"):
                assert k in e
        assert blk["realised"]["targets"] == sum(1 for e in plan.instances if e["type"] in TARGETS)
        assert "code_digest" not in str(blk["model"])            # metadata.yaml is in the content digest


def test_stand_ins_are_recorded(basin):
    plan = populate(basin, {"profile": "city_harbour", "density_level": 9, "seed": 2})
    st = [e for e in plan.instances if e["params"]["stand_in_for"]]
    assert st and all(e["type"] == "trap_pot" and e["params"]["source"]["use"] == "urban_quay" for e in st)
    assert set(e["params"]["stand_in_for"] for e in st) <= {"bicycle", "e_scooter"}
    assert plan.report["realised"]["stand_ins"] and any("stand-ins" in w for w in plan.report["warnings"])


def test_burial_follows_the_sediment():
    p = default_parameters()
    basin = synthetic_basin(140.0, 130.0, quays=("north", "east"), mud_patch=True)     # mud by the quays
    on, off = [], []
    for seed in range(1, 30):
        plan = populate(basin, {"profile": "generic", "density_level": 10, "seed": seed})
        for e in plan.instances:
            if e["type"] == "pipe_steel":
                (on if e["params"]["on_mud"] else off).append(e["burial"])
                assert e["params"]["sediment_under"] in basin.sediment_names
    assert off and max(off) <= 0.30 + 1e-9
    assert on and max(on) <= 0.60 + 1e-9 and max(on) > 0.30
    hi = populate(basin, {"profile": "generic", "density_level": 10, "seed": 3, "advanced": {"burial": "high"}})
    assert max(e["burial"] for e in hi.instances) <= p("burial.cap") + 1e-9
    assert any("on mud" in w for w in hi.report["warnings"]) and hi.report["realised"]["on_mud"] > 0


def test_small_litter_and_structures(basin):
    p = default_parameters()
    plan = populate(basin, {"profile": "marina", "density_level": 5, "seed": 1})
    assert [L["id"] for L in plan.layers] == ["ml_litter_base", "ml_litter_band"]
    sl = plan.report["small_litter"]
    mean = sl["base_per_m2"] * (1 - sl["band_share"]) + sl["band_per_m2"] * sl["band_share"]
    assert abs(mean - p("levels.small_litter_per_m2")[5]) < 2e-3
    assert sl["band_per_m2"] <= p("small_litter.band")["cap_per_m2"] + 1e-9
    assert plan.litter_class == "gravel"
    # a marina's pontoon gets its piles; the basin has a block already, so no mooring field is added
    assert plan.report["structures"]["piles"] >= 1 and plan.report["structures"]["mooring_blocks"] == 0
    assert all(s["params"]["role"] == "structure" for s in plan.structures)
    none = populate(basin, {"profile": "marina", "density_level": 5, "seed": 1, "structures": "none",
                            "clutter": {"small_litter": False}})
    assert not none.structures and none.litter_class is None
    assert [L["params"]["per_m2"] for L in none.layers] == [0.0]          # one empty layer: the same build path
    open_water = synthetic_basin(140.0, 130.0, quays=("south",))
    field = populate(open_water, {"profile": "marina", "density_level": 5, "seed": 1})
    assert field.report["structures"]["mooring_blocks"] >= 1 and is_plain(field.structures)
    assert {s["type"] for s in field.structures} == {"mooring_block", "chain"}


def test_preview_is_fast():
    import time
    one = synthetic_basin(100.0, 100.0, quays=("south", "west"), pontoon=True)
    populate(one, {"profile": "fishing_harbour", "density_level": 5, "seed": 1})
    t = time.time()
    populate(one, {"profile": "fishing_harbour", "density_level": 5, "seed": 2})
    assert time.time() - t < 1.0
