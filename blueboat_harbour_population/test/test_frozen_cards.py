"""The class cards of this package restate the frozen simulator's
(``worldgen/population.py``, ``worldgen/objects.py``): every FROZEN number is
compared with the simulator's own, and every instance the planner writes is
built by the simulator's own builders and published by its ground truth."""
import numpy as np

from conftest import needs_simulator

from blueboat_harbour_population import TARGETS, default_parameters, populate, synthetic_basin
from blueboat_harbour_population.instances import tyre_codes

pytestmark = needs_simulator


def test_cards_equal_the_frozen_ones():
    from blueboat_sss_sim.worldgen import objects as ob
    from blueboat_sss_sim.worldgen import population as pop
    from blueboat_sss_sim.worldgen import campaign
    p = default_parameters()
    assert tuple(TARGETS) == tuple(ob.V2_TARGETS)
    burial = {"plastic_box": "default", "concrete_block": "default", "trap_pot": "default",
              "pipe_steel": "default", "tire_car": "default", "net_bundle": "default", "debris": "default"}
    for cls, key in burial.items():
        assert tuple(p(f"classes.{cls}.burial")[key]) == pop.BURIAL_MAX[cls], cls
    assert tuple(p("classes.drum_200l.burial")["lying"]) == pop.BURIAL_MAX["drum_200l:lying"]
    assert tuple(p("classes.drum_200l.burial")["standing"]) == pop.BURIAL_MAX["drum_200l:standing"]
    assert p("burial.scale") == pop.BURIAL_SCALE and p("burial.cap") == pop.BURIAL_CAP
    assert p("burial.effectively_buried") == pop.EFFECTIVELY_BURIED
    assert p("nominal_target_min_m") == pop.NOMINAL_TARGET_MIN_M
    assert p("classes.net_bundle.volume_fraction") == pop.NET_VOLUME_FRACTION
    s = p("classes.tire_car.size")
    assert tuple(s["widths_mm"]) == pop.TYRE_WIDTHS_MM and tuple(s["aspects"]) == pop.TYRE_ASPECTS
    assert tuple(s["rims_in"]) == pop.TYRE_RIMS_IN
    assert p("profiles.generic.weights") == pop.REALISTIC
    for k in ("generic_quay", "generic_pontoon", "generic_mooring", "generic_berth", "generic_outlet",
              "generic_breakwater"):
        kind = k.split("_", 1)[1]
        em = p(f"tags.{k}.emits")
        assert all(em[c] == pop.AFFINITY[c][kind] for c in TARGETS), k
        assert p(f"tags.{k}.debris") == pop.AFFINITY["debris"][kind], k
    sl = p("levels.small_litter_per_m2")
    assert (sl[1], sl[5], sl[10]) == (campaign.CLUTTER["low"]["litter"], campaign.CLUTTER["nominal"]["litter"],
                                      campaign.CLUTTER["high"]["litter"])
    for cls in TARGETS + ("debris",):
        assert cls in ob.CATALOG
    from blueboat_sss_sim.worldgen import domain
    assert tuple(p("envelope.altitude_m")) == (4.5, 7.0) and p("envelope.heave_margin_m") == domain.HEAVE_MARGIN_M
    assert domain.sensor_depth_m() == 0.22


def test_shapes_follow_the_cards_over_many_draws():
    """The shapes the planner draws have the frozen cards' frequencies."""
    from blueboat_harbour_population import instances as inst
    p = default_parameters()
    rng = np.random.default_rng(5)
    n = 4000
    U = rng.random((n, inst.N_ATTR))
    box = [inst.shape("plastic_box", u, p) for u in U]
    assert abs(np.mean([b["pose"][0] == "upright" for b in box]) - 0.5) < 0.03
    assert abs(np.mean([b["params"]["state"] == "part_filled" for b in box]) - 0.3) < 0.03
    drum = [inst.shape("drum_200l", u, p) for u in U]
    assert abs(np.mean([d["pose"][0] == "lying" for d in drum]) - 0.6) < 0.03
    tyre = [inst.shape("tire_car", u, p) for u in U]
    assert abs(np.mean([t["pose"][0] == "flat" for t in tyre]) - 0.85) < 0.03
    assert all(0.55 <= t["length"] <= 0.80 for t in tyre)
    assert {t["params"]["tyre_code"] for t in tyre} == {f"{w}/{a}R{r}" for w, a, r, _ in tyre_codes(p)}
    trap = [inst.shape("trap_pot", u, p) for u in U]
    assert abs(np.mean([t["pose"][0] == "upright" for t in trap]) - 0.8) < 0.03
    assert abs(np.mean([t["params"]["variant"] == "us_wire" for t in trap]) - 0.5) < 0.03


def test_every_instance_builds_in_the_frozen_engine():
    from blueboat_sss_sim.core.types import PlacedObject
    from blueboat_sss_sim.worldgen.compounds import envelope, parts
    from blueboat_sss_sim.worldgen.groundtruth import object_truth
    basin = synthetic_basin(140.0, 130.0, quays=("south", "west"), pontoon=True, mooring=1, mud_patch=True)
    seen = set()
    for prof in ("fishing_harbour", "city_harbour", "working_port", "trap_harbour", "marina"):
        plan = populate(basin, {"profile": prof, "density_level": 10, "seed": 4})
        for k, e in enumerate(plan.instances + plan.structures):
            o = PlacedObject(object_id=k, type=e["type"], x=e["x"], y=e["y"], yaw=e["yaw"], length=e["length"],
                             width=e["width"], proud_height=e["proud_height"], burial=e["burial"],
                             reflectivity=e["reflectivity"], material=e["material"],
                             fill_fraction=e["fill_fraction"], params=dict(e["params"]),
                             roll=float(np.radians(e.get("roll_deg", 0.0))),
                             pitch=float(np.radians(e.get("pitch_deg", 0.0))),
                             rest=e.get("rest", "canonical"))
            assert envelope(o) is not None and len(parts(o)) >= 1, e["type"]
            t = object_truth(o, lambda x, y: np.full(np.shape(x), -6.0), "sand")
            assert t["role"] == e["params"]["role"] and t["nominal_target"] == (
                e["params"].get("nominal_target", False) and e["params"]["role"] == "target")
            seen.add(e["type"])
    assert set(TARGETS) <= seen
