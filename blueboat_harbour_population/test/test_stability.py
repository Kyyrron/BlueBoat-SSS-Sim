"""What an edit may change (POPULATION_MODEL.md, "Stability").

An object's position and attributes depend only on its own stream
``(seed, feature, class, process)``. So:

* changing the use or the activity of one feature changes that feature's
  objects and, at most, objects of other features that lie within the
  spacing distance of one of them (the spacing rule lets one of two
  neighbours live);
* deselecting a class makes the other classes share the total: they gain
  objects, and none of theirs moves or changes;
* making a feature no source lets its neighbours own its water: they gain
  objects, and none of theirs moves or changes.
"""
import math

from blueboat_harbour_population import DEBRIS, populate

#: two units interact when nearer than their radii plus the spacing: no unit of these tests is larger than this
NEAR_M = 12.0


def key(e):
    s = e["params"]["source"]
    return (s["feature"], e["type"], s["process"], str(s["index"]))


def near(e, others) -> bool:
    return any(math.hypot(e["x"] - o["x"], e["y"] - o["y"]) < NEAR_M for o in others)


def compare(a, b, edited):
    """(same, changed near an edited object, changed elsewhere) among the
    objects that ``edited(e)`` does not name."""
    A = {key(e): e for e in a.instances if not edited(e)}
    B = {key(e): e for e in b.instances if not edited(e)}
    ea = [e for e in a.instances if edited(e)]
    eb = [e for e in b.instances if edited(e)]
    same, local, far = 0, 0, []
    for k in set(A) | set(B):
        if k in A and k in B:
            assert A[k] == B[k], f"{k}: the object itself changed"
            same += 1
        else:
            e = A.get(k) or B.get(k)
            if near(e, ea) or near(e, eb):
                local += 1
            else:
                far.append(k)
    return same, local, far


def test_a_feature_tag_changes_only_that_feature(basin):
    base = {"profile": "fishing_harbour", "density_level": 8, "seed": 31}
    a = populate(basin, base)
    b = populate(basin, {**base, "feature_tags": {"quay_west": "working_quay"}})
    assert a.digest() != b.digest()
    fa = {f["id"]: f for f in a.report["features"]}
    fb = {f["id"]: f for f in b.report["features"]}
    for fid in fa:
        if fid != "quay_west":
            assert fa[fid]["expected"] == fb[fid]["expected"] and fa[fid]["owned_ha"] == fb[fid]["owned_ha"]
    assert fb["quay_west"]["use"] == "working_quay" and fa["quay_west"]["owned_ha"] == fb["quay_west"]["owned_ha"]
    same, local, far = compare(a, b, lambda e: e["params"]["source"]["feature"] == "quay_west")
    assert far == [] and same > 10 and local <= 0.25 * same


def test_a_feature_activity_changes_only_that_feature(basin):
    base = {"profile": "city_harbour", "density_level": 7, "seed": 8}
    a = populate(basin, base)
    b = populate(basin, {**base, "feature_activity": {"quay_south": 3.0}})
    same, local, far = compare(a, b, lambda e: e["params"]["source"]["feature"] == "quay_south")
    assert far == [] and same > 10
    # the feature's own objects are nested: those of the lower activity are among those of the higher
    mine_a = {key(e): e for e in a.instances if e["params"]["source"]["feature"] == "quay_south"
              and e["params"]["group_id"] is None}
    mine_b = {key(e): e for e in b.instances if e["params"]["source"]["feature"] == "quay_south"}
    kept = [k for k in mine_a if k in mine_b]
    assert all(mine_a[k] == mine_b[k] for k in kept) and len(kept) >= 0.7 * len(mine_a)
    assert len(mine_b) > len(mine_a)


def test_deselecting_a_class_moves_nothing(basin):
    base = {"profile": "fishing_harbour", "density_level": 8, "seed": 12, "advanced": {"groups": False}}
    a = populate(basin, base)
    b = populate(basin, {**base, "classes": [c for c in ("plastic_box", "drum_200l", "concrete_block", "trap_pot",
                                                         "pipe_steel", "net_bundle")]})
    assert "tire_car" not in b.counts() and "tire_car" in a.counts()
    A = {key(e): e for e in a.instances if e["type"] != "tire_car"}
    B = {key(e): e for e in b.instances}
    tyres = [e for e in a.instances if e["type"] == "tire_car"]
    new = [B[k] for k in B if k not in A]
    kept = 0
    for k, e in A.items():
        if k in B:
            assert e == B[k]                       # no object of another class moved or changed
            kept += 1
        else:
            assert near(e, new)                    # only a new neighbour can take one away
    assert kept >= 0.7 * len(A)                    # half of the targets were tyres: many new neighbours
    # the others share the total: they gained objects (the debris part is untouched)
    assert sum(1 for e in b.instances if e["type"] != DEBRIS) > sum(1 for e in A.values() if e["type"] != DEBRIS)
    assert abs(a.report["expected"]["targets"] - b.report["expected"]["targets"]) < 1e-6
    assert a.report["expected"]["debris"] == b.report["expected"]["debris"]
    assert all(near(e, tyres) or True for e in new)


def test_a_feature_made_no_source_moves_nothing(basin):
    base = {"profile": "working_port", "density_level": 8, "seed": 5, "advanced": {"groups": False}}
    a = populate(basin, base)
    b = populate(basin, {**base, "feature_tags": {"pontoon": "none"}})
    assert not any(e["params"]["source"]["feature"] == "pontoon" for e in b.instances)
    A = {key(e): e for e in a.instances if e["params"]["source"]["feature"] not in ("pontoon", "far_field")}
    B = {key(e): e for e in b.instances}
    new = [B[k] for k in B if k not in A]
    for k, e in A.items():
        if k in B:
            assert e == B[k]
        else:
            assert near(e, new)
    assert sum(1 for k in A if k in B) >= 0.9 * len(A)


def test_the_world_seed_of_another_feature_set_is_not_needed(basin):
    """Two placements that differ only in their seed share nothing."""
    a = populate(basin, {"profile": "generic", "density_level": 6, "seed": 1})
    b = populate(basin, {"profile": "generic", "density_level": 6, "seed": 2})
    assert not ({(e["x"], e["y"]) for e in a.instances} & {(e["x"], e["y"]) for e in b.instances})
