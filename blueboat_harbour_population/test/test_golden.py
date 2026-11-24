"""The model version guard: fixed inputs give the recorded plans. A failure
means a plan changes for the same inputs, which is a new model version: bump
``__version__`` (and setup.py, package.xml), then regenerate
``golden_plans.yaml``::

    from blueboat_harbour_population import populate, synthetic_basin
    env = synthetic_basin(140.0, 130.0, depth_m=6.0, grade=0.003, quays=("south", "west"), pontoon=True,
                          mooring=1, mud_patch=True)
    populate(env, {"profile": <profile>, "density_level": <level>, "seed": 1234}).digest()
"""
from pathlib import Path

import yaml

from blueboat_harbour_population import __version__, populate


def test_plans_of_this_version(basin):
    gold = yaml.safe_load((Path(__file__).parent / "golden_plans.yaml").read_text())
    assert gold["version"] == __version__
    assert basin.digest() == gold["environment_digest"]
    got = {}
    for key in gold["plans"]:
        prof, lvl = key.split("/L")
        got[key] = populate(basin, {"profile": prof, "density_level": int(lvl), "seed": 1234}).digest()
    assert got == gold["plans"]


def test_versions_agree():
    root = Path(__file__).resolve().parents[1]
    assert f'VERSION = "{__version__}"' in (root / "setup.py").read_text()
    assert f"<version>{__version__}</version>" in (root / "package.xml").read_text()
