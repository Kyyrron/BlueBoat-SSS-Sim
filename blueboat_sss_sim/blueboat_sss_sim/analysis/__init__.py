"""Detection metrics from simulated ground truth.

The renderer already knows, for every ping, which scene object it insonified
and at what slant range, extent and shadow length. ``sss_sim_node`` puts that
on ``/side_scan_sonar/ground_truth/contacts``; this package turns it into the
numbers ``project_synthesis.md`` §8.4 asks for -- per-class P(detect) against
range *and* relative aspect, plus the continuous per-target metrics §8.4's
warning prefers over a binary rate.

The point is timing: every one of these can be computed **before** a field
session, which is the project's scarcest resource (root ``CLAUDE.md`` CM-14).

Layering: like ``core/``, ``worldgen/``, ``sonar/``, ``dataset/`` and
``mission/``, this package imports **no ROS**. The ``.svlog`` reader parses a
documented byte format rather than importing anything from ``BlueBoat-SSS``
(CM-3 / NC #8), and every entry point is read-only: bundles and recordings are
write-once (CM-7 / NC #10).

Results derived here are simulation-derived and are reported as such
(CM-13); they support the policy comparison, never an unqualified empirical
detection claim.
"""

from .calibration import (REDUCTION_FLOOR_DB, RangeCurve, Residual, compare,
                          fbr_bin, normalisation_invariants, reduce_profiles,
                          reduce_svlog, scale_to_db)
from .contacts import (ContactObservation, from_jsonl, from_replay,
                       from_svlog)
from .metrics import (CRITERIA, DetectionCriterion, MissionMetrics,
                      compute_metrics)
from .report import write_report

__all__ = [
    "ContactObservation", "from_replay", "from_svlog", "from_jsonl",
    "scale_to_db", "fbr_bin", "RangeCurve", "Residual", "compare",
    "reduce_profiles", "reduce_svlog", "normalisation_invariants",
    "REDUCTION_FLOOR_DB",
    "DetectionCriterion", "CRITERIA", "MissionMetrics", "compute_metrics",
    "write_report",
]
