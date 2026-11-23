"""The batch writer: specifications and the world list of a split, from a
list of environments, with the split guard on seeds and harbours."""
import pytest
import yaml

from blueboat_harbour_population import SplitError
from blueboat_harbour_population.batch import write_batch
from blueboat_harbour_population.params import PlacementParams

BASE = {"format": "blueboat_campaign/1", "name": "x", "path": "traj.yaml",
        "world": {"margin_m": 25, "resolution_m": 0.10},
        "defaults": {"depth_band_m": [5.2, 6.6]}}


@pytest.fixture()
def envs(tmp_path):
    (tmp_path / "specs").mkdir()
    (tmp_path / "specs" / "traj.yaml").write_text("format: blueboat_trajectory/1\n")
    (tmp_path / "specs" / "basin.yaml").write_text(yaml.safe_dump(BASE))
    (tmp_path / "specs" / "chart.yaml").write_text(yaml.safe_dump(
        {**BASE, "environment": {"depth_file": "chart.csv", "shift_m": 2.0}}))
    f = tmp_path / "envs.yaml"
    f.write_text(yaml.safe_dump({"format": "blueboat_population_batch/1", "name": "ds_a",
                                 "placement": {"mix": "realistic"},
                                 "environments": [{"name": "basin", "spec": "specs/basin.yaml",
                                                   "profile": "fishing_harbour"},
                                                  {"name": "provincetown_t1", "spec": "specs/chart.yaml",
                                                   "harbour": "provincetown"}]}))
    return f


def test_batch_writes_specs_and_worlds(envs, tmp_path):
    out = write_batch(envs, tmp_path / "out", split="dataset", levels=[2, 5, 8], worlds=12)
    man = yaml.safe_load(open(out["manifest"]))
    assert man["split"] == "dataset" and man["seed_range"] == [100000, 199999]
    seeds = [w["seed"] for w in man["worlds"]]
    assert seeds == list(range(100000, 100012))
    # every environment sees every level
    seen = {(w["environment"], w["density_level"]) for w in man["worlds"]}
    assert seen == {(e, lv) for e in ("basin", "provincetown_t1") for lv in (2, 5, 8)}
    spec = yaml.safe_load(open(out["specs"]["provincetown_t1"]))
    assert spec["placement"]["split"] == "dataset" and spec["placement"]["harbour"] == "provincetown"
    assert spec["placement"]["profile"] == "trap_harbour"                 # the harbour's profile
    assert [c["name"] for c in spec["cases"]] == ["l2", "l5", "l8"]
    assert spec["environment"]["depth_file"].endswith("specs/chart.csv")   # resolved beside its source
    assert spec["path"].endswith("specs/traj.yaml")
    PlacementParams.from_dict({**spec["placement"], "density_level": 5, "seed": 100000})
    assert all("--seeds" in c for c in out["commands"])


def test_batch_refuses_a_foreign_harbour_or_seed(envs, tmp_path):
    with pytest.raises(SplitError):
        write_batch(envs, tmp_path / "o1", split="evaluation", levels=[5], worlds=4)   # provincetown is dataset
    with pytest.raises(SplitError):
        write_batch(envs, tmp_path / "o2", split="dataset", levels=[5], worlds=4, first_seed=199999)
