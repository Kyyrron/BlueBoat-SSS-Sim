"""Shared fixtures. Run from the package root with the system python:

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -B -m pytest

The tests that drive the frozen simulator put its package source root
(``../blueboat_sss_sim``) on the path themselves and skip, visibly, when it
cannot be imported.
"""
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_ROOT = ROOT.parent / "blueboat_sss_sim"
for p in (str(SIM_ROOT), str(ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from blueboat_harbour_population import synthetic_basin  # noqa: E402


@pytest.fixture(scope="session")
def basin():
    """A 140 x 130 m basin: two quays, a pontoon, a mooring block, a mud patch."""
    return synthetic_basin(140.0, 130.0, depth_m=6.0, grade=0.003, quays=("south", "west"), pontoon=True,
                           mooring=1, mud_patch=True)


@pytest.fixture(scope="session")
def small_basin():
    return synthetic_basin(100.0, 80.0, depth_m=6.0, quays=("south",), pontoon=False, mooring=0)


def have_simulator() -> bool:
    try:
        import blueboat_sss_sim.worldgen.world_folder  # noqa: F401
        return True
    except Exception:
        return False


needs_simulator = pytest.mark.skipif(not have_simulator(), reason="blueboat_sss_sim cannot be imported")
