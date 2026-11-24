"""Depth from a file in headless generation (the ``environment:`` block): a
chart-datum CSV or GeoTIFF read by the frozen generator's own reader, the
water level, the smoothing and the shift or compression as its own
``adjust`` layers, all recorded in the metadata. Synthetic grids only."""

import numpy as np
import pytest
import yaml

from conftest import needs_simulator

from blueboat_harbour_population.schema import check_world_block

pytestmark = needs_simulator

LAT0, LON0 = 42.0500, -70.1800
TRAJECTORY = f"""\
format: blueboat_trajectory/1
name: depth_test_path
loop: false
geo_anchor: {{lat0: {LAT0}, lon0: {LON0}, theta_deg: 0.0}}
points:
- [0.0, 0.0, 0.0, 0.0]
- [50.0, 50.0, 0.0, 0.0]
- [60.0, 50.0, 15.0, 1.5708]
- [110.0, 0.0, 15.0, 3.1416]
- [120.0, 0.0, 30.0, 1.5708]
- [170.0, 50.0, 30.0, 0.0]
"""


def chart_depth(e, n):
    """A synthetic chart: 8 m at the south, deepening north by 1 m per 100 m,
    with a 0.8 m bump that a real grid would hold (a wreck, a rock)."""
    d = 8.0 + 0.010 * (n + 30.0)
    return d - 0.8 * np.exp(-((e - 25.0) ** 2 + (n - 15.0) ** 2) / (2 * 1.5 ** 2))


@pytest.fixture(scope="module")
def files(tmp_path_factory):
    from blueboat_sss_sim.core.geo import GeoAnchor
    d = tmp_path_factory.mktemp("depth")
    (d / "depth_test_path.yaml").write_text(TRAJECTORY)
    a = GeoAnchor(lat0=LAT0, lon0=LON0)
    xs, ys = np.arange(-40.0, 95.0, 1.0), np.arange(-45.0, 80.0, 1.0)
    X, Y = np.meshgrid(xs, ys)
    lat, lon = a.to_latlon(X.ravel(), Y.ravel())
    dep = chart_depth(X.ravel(), Y.ravel())
    with open(d / "chart.csv", "w") as f:
        f.write("lat,lon,depth\n")
        for la, lo, de in zip(np.asarray(lat), np.asarray(lon), dep):
            f.write(f"{la:.8f},{lo:.8f},{de:.3f}\n")
    with open(d / "chart_elevation.csv", "w") as f:
        f.write("latitude,longitude,elevation\n")
        for la, lo, de in zip(np.asarray(lat), np.asarray(lon), dep):
            f.write(f"{la:.8f},{lo:.8f},{-de:.3f}\n")
    # a geographic GeoTIFF with the generator's own writer
    from blueboat_sss_sim.core.types import GridSpec
    from blueboat_sss_sim.worldgen.depth.bake import write_depth_geotiff
    g = GridSpec(-40.0, -45.0, 1.0, len(xs), len(ys))
    write_depth_geotiff(-chart_depth(X, Y), g, a, d / "chart.tif")
    return d


def spec(files, **env):
    block = {"depth_file": "chart.csv", "water_level_m": 1.2, "smooth_sigma_m": 2.5, "shift_m": 3.6,
             "walls": [{"name": "quay_s", "kind": "quay", "points": [[-30.0, -22.0], [80.0, -22.0]]},
                       {"name": "float_a", "kind": "pontoon", "points": [[10.0, -21.5], [10.0, -6.0]],
                        "use": "marina_pontoon"}],
             "features": [{"id": "ground", "kind": "area", "use": "fishing_ground",
                           "points": [[20.0, 20.0], [70.0, 20.0], [70.0, 60.0], [20.0, 60.0]]}],
             "sediment": {"heterogeneity": "low"}}
    block.update(env)
    return {"format": "blueboat_campaign/1", "name": "chart", "path": "depth_test_path.yaml",
            "world": {"margin_m": 25, "resolution_m": 0.10},
            "defaults": {"uncertain_materials": "low"},
            "environment": block,
            "placement": {"profile": "trap_harbour", "density_level": 8}}


def build(files, tmp_path, doc, name="chart.yaml", seed=5):
    from blueboat_harbour_population import sim_adapter as sa
    f = files / name
    f.write_text(yaml.safe_dump(doc, sort_keys=False))
    man = sa.run(f, [seed], tmp_path)
    w = man["worlds"][0]
    meta = yaml.safe_load((tmp_path / w["world"] / "metadata.yaml").read_text())
    return w, meta, tmp_path / w["world"]


def test_csv_world_with_level_smoothing_and_shift(files, tmp_path):
    w, meta, wdir = build(files, tmp_path, spec(files))
    env = meta["environment"]
    assert env["depth_file"] == "chart.csv" and len(env["depth_file_sha256"]) == 16
    assert env["water_level_m"] == 1.2 and env["smooth_sigma_m"] == 2.5 and env["shift_m"] == 3.6
    assert meta["depth"]["mode"] == "file"
    # depth' = chart + water level - shift: 8.0..9.3 m becomes 5.6..6.9 m
    with np.load(wdir / "depth.npz") as z:
        h, ox, oy, res = z["height"], float(z["origin_x"]), float(z["origin_y"]), float(z["resolution"])
    off = meta["source_path"]["design_offset"]
    for (e, n) in ((0.0, 0.0), (40.0, 30.0), (10.0, 50.0)):
        i, j = int(round((e - off[0] - ox) / res)), int(round((n - off[1] - oy) / res))
        got = -float(np.median(h[j - 3:j + 4, i - 3:i + 4]))
        assert abs(got - (8.0 + 0.010 * (n + 30.0) + 1.2 - 3.6)) < 0.08, (e, n, got)
    # the bump is smoothed (a 0.8 m object in a real grid would become an unlabelled target)
    i, j = int(round((25.0 - off[0] - ox) / res)), int(round((15.0 - off[1] - oy) / res))
    bump = (8.0 + 0.010 * 45.0 + 1.2 - 3.6) - (-float(h[j, i]))
    assert 0.05 < bump < 0.45
    assert meta["domain"]["altitude_min_m"] > 4.5 and meta["domain"]["altitude_max_m"] < 7.0
    pl = meta["placement"]
    assert check_world_block(pl) == [] and pl["envelope"]["share_in_band"] > 0.95
    uses = {f["id"]: f["use"] for f in pl["features"]}
    assert uses["quay_s"] == "fishing_berth" and uses["float_a"] == "marina_pontoon" and uses["ground"] == "fishing_ground"
    assert {wl["name"] for wl in meta["walls"]} == {"quay_s", "float_a"}
    litter = [o for o in meta["objects"] if o["role"] in ("target", "clutter")]
    assert litter and all(o["params"]["in_envelope"] for o in litter)
    # the layers that did it are in the state, in their order, before the seabed's own
    state = yaml.safe_load((wdir / "builder_state.yaml").read_text())
    ids = [L["id"] for L in state["depth"]["layers"]]
    assert ids[:3] == ["env_smooth", "env_water_level", "env_shift"]
    assert (wdir / "depth_source.csv").is_file()                 # the folder keeps its source


def test_elevation_csv_and_geotiff_give_the_same_floor(files, tmp_path):
    _, m1, d1 = build(files, tmp_path / "a", spec(files))
    _, m2, d2 = build(files, tmp_path / "b", spec(files, depth_file="chart_elevation.csv"))
    _, m3, d3 = build(files, tmp_path / "c", spec(files, depth_file="chart.tif"))
    h = []
    for d in (d1, d2, d3):
        with np.load(d / "depth.npz") as z:
            h.append(z["height"])
    assert np.max(np.abs(h[0] - h[1])) < 1e-6                    # depth and elevation: one convention
    assert np.percentile(np.abs(h[0] - h[2]), 99) < 0.05         # a raster and its soundings: one floor
    assert m3["environment"]["depth_file"] == "chart.tif"


def test_compression_and_its_rules(files, tmp_path):
    doc = spec(files, shift_m=None)
    doc["environment"].pop("shift_m")
    doc["environment"]["compress"] = {"k": 0.6, "d_lo": 9.2, "to": 5.0}
    _, meta, wdir = build(files, tmp_path, doc)
    assert meta["environment"]["compress"] == {"k": 0.6, "d_lo": 9.2, "to": 5.0}
    with np.load(wdir / "depth.npz") as z:
        h, ox, oy, res = z["height"], float(z["origin_x"]), float(z["origin_y"]), float(z["resolution"])
    off = meta["source_path"]["design_offset"]
    i, j = int(round((40.0 - off[0] - ox) / res)), int(round((30.0 - off[1] - oy) / res))
    chart = 8.0 + 0.010 * 60.0 + 1.2                              # with the water level
    assert abs(-float(np.median(h[j - 3:j + 4, i - 3:i + 4])) - (5.0 + 0.6 * (chart - 9.2))) < 0.08
    from blueboat_harbour_population import sim_adapter as sa
    with pytest.raises(ValueError, match="0.5"):
        sa.environment_layers({"compress": {"k": 0.4, "d_lo": 5.0}})
    with pytest.raises(ValueError, match="not both"):
        sa.environment_layers({"shift_m": 1.0, "compress": {"k": 0.6, "d_lo": 5.0}})
    with pytest.raises(KeyError, match="depht"):
        bad = spec(files)
        bad["environment"]["depht"] = 1
        build(files, tmp_path / "bad", bad, name="bad.yaml")


def test_mud_is_allowed_and_flagged(files, tmp_path):
    doc = spec(files)
    doc["environment"]["sediment"] = {"reference": "sand", "finer": "soft_mud", "coarser": "gravel",
                                      "heterogeneity": "high"}
    doc["placement"]["density_level"] = 10
    _, meta, _ = build(files, tmp_path, doc, seed=9)
    assert "soft_mud" in meta["seabed"]["classes"]
    litter = [o for o in meta["objects"] if o["role"] in ("target", "clutter")]
    flags = [o["params"]["on_mud"] for o in litter]
    assert all(isinstance(v, bool) for v in flags)
    for o in litter:
        assert o["params"]["on_mud"] == (o["params"]["sediment_under"] in ("mud", "soft_mud"))
