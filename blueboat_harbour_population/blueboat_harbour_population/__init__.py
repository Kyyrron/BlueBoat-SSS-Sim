"""blueboat_harbour_population: evidence-based population of a harbour's seabed
with litter, debris, small litter and structures (the "mission-like" object
placement of the BlueBoat world builder).

    from blueboat_harbour_population import populate, PlacementParams
    plan = populate(environment, {"profile": "fishing_harbour", "density_level": 5, "seed": 7})

* ``environment``: where the water is, how deep, on what sediment, and the
  harbour's features (:mod:`.environment`);
* ``params``: the ``placement:`` block (:mod:`.params`);
* the plan: instances in the frozen generator's explicit-instance format,
  groups, structures, seabed layers and a report (:mod:`.plan`).

The numbers are in ``data/evidence_parameters.yaml`` (every one with its tag
and sources); the model is described in ``docs/population/POPULATION_MODEL.md``
of the BlueBoat-SSS-Sim repository, the evidence in ``EVIDENCE.md`` beside it.

The core is pure Python (numpy, scipy, PyYAML): no ROS, no Qt, no simulator.
:mod:`.sim_adapter` is the one module that knows the simulator.
"""

#: The model version (the same as setup.py and package.xml). Bump it whenever
#: a plan can change for the same inputs; test/test_golden.py guards it.
#: 1.1.0 (2026-11-26, session 16): the model itself is unchanged (the golden
#: plans are those of 1.0.0); a depth-file specification with a wall of
#: several points plans differently (the wall is one source, it was counted
#: per segment as well), and the ``environment:`` block gained ``land``,
#: ``structures`` and ``raster_resolution_m``.
__version__ = "1.1.0"

from .data import CLASSES, DEBRIS, TARGETS, code_digest, default_parameters, evidence_digest  # noqa: E402,F401
from .environment import Environment, Feature, synthetic_basin  # noqa: E402,F401
from .params import PlacementParams  # noqa: E402,F401
from .plan import PlacementPlan  # noqa: E402,F401
from .splits import SplitError, check_split  # noqa: E402,F401
from .model import allowed_mask, populate  # noqa: E402,F401
