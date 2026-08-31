"""Detection metrics over ground-truth observations.

Pure functions: observations plus a :class:`SceneModel` in, numbers out. No
I/O, no ROS, no randomness -- so the same inputs always give the same figures,
which is the whole point of the exercise (a metric that drifts between runs
cannot support a claim).

What "detection" means
----------------------
The definition *is* the metric, so it is declared rather than assumed, and
reported as a ladder rather than one arbitrary threshold:

===========  =============================================
``geometric``  ``visible``
``resolved``   ``visible`` and ``extent_bins >= 2``
``shadowed``   ``visible`` and ``shadow_bins >= 2``
===========  =============================================

``visible`` is the renderer's own horizon-culling result, so it already
excludes objects occluded by terrain or lying outside the swath, and objects
whose effective height is below 5 mm. On top of the per-observation predicate,
an *object* counts as detected only once it has accumulated ``min_pings``
qualifying observations -- 2 by default, the same threshold
``dataset.labeler.LabelConfig.min_rows`` uses, so this metric and the YOLO
labeller agree on what a contact is.

The ladder nests by construction: ``resolved`` and ``shadowed`` are both
``geometric`` plus a further condition.

Multipath ghosts
----------------
A ghost observation images a real object down a folded path, so it carries
that object's id. It is **excluded from every rate here** -- opportunities,
detections, looks, aspects -- because a detection rate measured against the
manifest counts objects, not their reflections. The ghosts are counted and
reported separately (``n_ghost_observations``, ``ghosts_by_reflector``):
they are the run's false-positive stimulus, and a report that dropped them
would understate exactly what the enclosed-basin regime makes hard.

What this delivers, and what it cannot
--------------------------------------
``project_synthesis.md`` §8.4 asks for continuous metrics in preference to a
binary rate, and for ``P(detect | class, R, aspect)``. Computable from ground
truth alone, and delivered here: detection probability against range and
against relative aspect, looks per target, distinct aspects per target, and
aspect-unique detections.

Blocked on the detector and belief layer, which no module implements (root
``CLAUDE.md`` §6 D7): time-to-confirmation, false positives per hectare,
detection confidence against number of looks, localisation error, and energy
cost per validated object. The report names these rather than quietly omitting
them.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from ..core.types import PlacedObject
from ..worldgen.scene import SceneModel
from .contacts import ContactObservation, ObservationSet, relative_aspect_deg


@dataclass(frozen=True)
class DetectionCriterion:
    """A named, explicit definition of what counts as a detection."""

    name: str
    min_extent_bins: float = 0.0
    min_shadow_bins: float = 0.0
    min_pings: int = 2

    def holds(self, o: ContactObservation) -> bool:
        """Does this single observation qualify?"""
        return (o.visible
                and o.extent_bins >= self.min_extent_bins
                and o.shadow_bins >= self.min_shadow_bins)

    def describe(self) -> str:
        parts = ["visible"]
        if self.min_extent_bins > 0:
            parts.append(f"extent_bins >= {self.min_extent_bins:g}")
        if self.min_shadow_bins > 0:
            parts.append(f"shadow_bins >= {self.min_shadow_bins:g}")
        return (f"{' and '.join(parts)}, on at least "
                f"{self.min_pings} ping(s)")


#: The shipped ladder. ``geometric`` is the weakest and contains the others.
CRITERIA: dict[str, DetectionCriterion] = {
    "geometric": DetectionCriterion("geometric"),
    "resolved": DetectionCriterion("resolved", min_extent_bins=2.0),
    "shadowed": DetectionCriterion("shadowed", min_shadow_bins=2.0),
}

DEFAULT_CRITERION = "resolved"


# ------------------------------------------------------------------ results
@dataclass
class Curve:
    """A probability against a binned quantity.

    ``probability[i]`` is ``None`` where the bin had no opportunities at all.
    That distinction is load-bearing: a range the vehicle never presented an
    object at is *unmeasured*, and reporting it as 0.0 would read as a
    detection failure that never happened.
    """

    bin_edges: list[float]
    opportunities: list[int]
    detections: list[int]
    probability: list[Optional[float]]


@dataclass
class ObjectSummary:
    """Per-object outcome, the unit the manifest reconciles against."""

    object_id: int
    object_type: str
    n_looks: int                      # observations, whether or not qualifying
    n_qualifying: int
    n_distinct_aspects: int           # aspect bins in which it qualified
    detected: bool
    min_detection_range_m: Optional[float] = None
    max_detection_range_m: Optional[float] = None
    aspect_unique: bool = False       # qualified in exactly one aspect bin


@dataclass
class MissionMetrics:
    """Everything one run of the tool produces."""

    criterion: str
    criterion_description: str
    source: str
    bundle: Optional[str]
    seed: Optional[int]
    detail: str
    range_bin_m: float
    aspect_bin_deg: float
    aspect_available: bool
    ping_cycles: int
    n_objects: int
    n_observations: int              # direct-path observations only
    n_ghost_observations: int = 0    # multipath images, excluded from every
                                     # rate below (see compute_metrics)
    ghosts_by_reflector: dict[str, int] = field(default_factory=dict)
    detected: list[int] = field(default_factory=list)
    observed_not_detected: list[int] = field(default_factory=list)
    never_ensonified: list[int] = field(default_factory=list)
    range_curves: dict[str, Curve] = field(default_factory=dict)
    aspect_curves: dict[str, Curve] = field(default_factory=dict)
    per_class: dict[str, dict] = field(default_factory=dict)
    objects: list[ObjectSummary] = field(default_factory=list)
    blocked_metrics: list[str] = field(default_factory=list)


#: §8.4 rows that need a detector or the belief layer, neither of which exists
#: in any module (root ``CLAUDE.md`` §6 D7).
BLOCKED_METRICS = [
    "mean time-to-confirmation per contact",
    "false positives per hectare",
    "detection confidence as a function of number of looks",
    "localisation error per detection",
    "energy cost per validated object",
]


# ------------------------------------------------------------------ binning
def _curve(values: Sequence[float], qualifies: Sequence[bool],
           edges: np.ndarray) -> Curve:
    """Bin ``values`` by ``edges`` and count opportunities vs detections."""
    v = np.asarray(values, dtype=np.float64)
    q = np.asarray(qualifies, dtype=bool)
    idx = np.clip(np.digitize(v, edges) - 1, 0, len(edges) - 2) if v.size else v
    n = len(edges) - 1
    opp = np.zeros(n, dtype=np.int64)
    det = np.zeros(n, dtype=np.int64)
    if v.size:
        np.add.at(opp, idx.astype(int), 1)
        np.add.at(det, idx.astype(int)[q], 1)
    prob: list[Optional[float]] = [
        (float(det[i]) / float(opp[i])) if opp[i] else None for i in range(n)]
    return Curve([float(e) for e in edges], opp.tolist(), det.tolist(), prob)


def _by_class(observations: Sequence[ContactObservation]
              ) -> dict[str, list[ContactObservation]]:
    grouped: dict[str, list[ContactObservation]] = defaultdict(list)
    for o in observations:
        grouped[o.object_type].append(o)
    return grouped


# ------------------------------------------------------------------- driver
def compute_metrics(obs_set: ObservationSet, scene: SceneModel,
                    criterion: str = DEFAULT_CRITERION,
                    range_bin_m: float = 1.0,
                    aspect_bin_deg: float = 15.0,
                    range_max_m: Optional[float] = None) -> MissionMetrics:
    """Observations + scene -> every number the report prints.

    ``scene`` is the denominator: every object the manifest places is
    accounted for, whether or not the survey ever saw it.
    """
    crit = CRITERIA[criterion]
    # Multipath images are separated out before anything is counted. A ghost
    # carries the object_id of the object it images, so leaving it in would
    # inflate that object's looks and could mark it "detected" off an echo
    # that arrived from a wall -- a detection rate measured against the
    # manifest has to count the object, not its reflection. They are reported
    # rather than discarded: their number is what makes a run's false-positive
    # stimulus visible.
    obs = [o for o in obs_set.observations if not o.ghost]
    ghosts = [o for o in obs_set.observations if o.ghost]
    ghosts_by_reflector: dict[str, int] = defaultdict(int)
    for o in ghosts:
        ghosts_by_reflector[o.via or "unknown"] += 1
    objects: dict[int, PlacedObject] = {o.object_id: o for o in scene.objects}

    if range_max_m is None:
        range_max_m = (max((o.slant_range_m for o in obs), default=1.0))
    r_edges = np.arange(0.0, float(range_max_m) + range_bin_m, range_bin_m)
    if r_edges.size < 2:
        r_edges = np.array([0.0, max(float(range_max_m), range_bin_m)])
    a_edges = np.arange(0.0, 180.0 + aspect_bin_deg, aspect_bin_deg)

    aspect_ok = obs_set.has_aspect

    # ---- per-object accumulation ----------------------------------------
    looks: dict[int, int] = defaultdict(int)
    qual: dict[int, list[ContactObservation]] = defaultdict(list)
    aspects: dict[int, set[int]] = defaultdict(set)
    for o in obs:
        looks[o.object_id] += 1
        if crit.holds(o):
            qual[o.object_id].append(o)
            if aspect_ok and o.look_heading_deg is not None:
                obj = objects.get(o.object_id)
                if obj is not None:
                    a = relative_aspect_deg(o.look_heading_deg, obj.yaw)
                    aspects[o.object_id].add(
                        int(min(a // aspect_bin_deg, len(a_edges) - 2)))

    summaries: list[ObjectSummary] = []
    detected, observed_not_detected, never = [], [], []
    for oid in sorted(objects):
        obj = objects[oid]
        q = qual.get(oid, [])
        is_det = len(q) >= crit.min_pings
        n_asp = len(aspects.get(oid, ()))
        summaries.append(ObjectSummary(
            object_id=oid, object_type=obj.type,
            n_looks=looks.get(oid, 0), n_qualifying=len(q),
            n_distinct_aspects=n_asp, detected=is_det,
            min_detection_range_m=(min(c.slant_range_m for c in q)
                                   if is_det else None),
            max_detection_range_m=(max(c.slant_range_m for c in q)
                                   if is_det else None),
            aspect_unique=bool(is_det and aspect_ok and n_asp == 1)))
        if is_det:
            detected.append(oid)
        elif looks.get(oid, 0) > 0:
            observed_not_detected.append(oid)
        else:
            never.append(oid)

    # The partition must be exact -- this is the manifest reconciliation, and
    # an object lost between the three buckets is a bug in the metric, not a
    # survey result.
    assert len(detected) + len(observed_not_detected) + len(never) == \
        len(objects), "object partition does not cover the manifest"

    # ---- curves ----------------------------------------------------------
    grouped = _by_class(obs)
    range_curves = {
        cls: _curve([o.slant_range_m for o in items],
                    [crit.holds(o) for o in items], r_edges)
        for cls, items in sorted(grouped.items())}
    range_curves["__all__"] = _curve(
        [o.slant_range_m for o in obs], [crit.holds(o) for o in obs], r_edges)

    aspect_curves: dict[str, Curve] = {}
    if aspect_ok:
        def _asp(items):
            vals, qs = [], []
            for o in items:
                obj = objects.get(o.object_id)
                if obj is None or o.look_heading_deg is None:
                    continue
                vals.append(relative_aspect_deg(o.look_heading_deg, obj.yaw))
                qs.append(crit.holds(o))
            return _curve(vals, qs, a_edges)
        aspect_curves = {cls: _asp(items)
                         for cls, items in sorted(grouped.items())}
        aspect_curves["__all__"] = _asp(obs)

    # ---- per-class rollup -------------------------------------------------
    per_class: dict[str, dict] = {}
    by_type: dict[str, list[ObjectSummary]] = defaultdict(list)
    for s in summaries:
        by_type[s.object_type].append(s)
    for cls in sorted(by_type):
        rows = by_type[cls]
        det = [r for r in rows if r.detected]
        ens = [r for r in rows if r.n_looks > 0]
        per_class[cls] = {
            "n_objects": len(rows),
            "n_ensonified": len(ens),
            "n_detected": len(det),
            "detection_rate": (len(det) / len(rows)) if rows else None,
            "median_looks": (float(np.median([r.n_looks for r in rows]))
                             if rows else None),
            "median_distinct_aspects": (
                float(np.median([r.n_distinct_aspects for r in det]))
                if (det and aspect_ok) else None),
            "n_aspect_unique": sum(1 for r in det if r.aspect_unique),
            "max_detection_range_m": (
                max(r.max_detection_range_m for r in det) if det else None),
        }

    return MissionMetrics(
        criterion=crit.name, criterion_description=crit.describe(),
        source=obs_set.source, bundle=obs_set.bundle, seed=obs_set.seed,
        detail=obs_set.detail, range_bin_m=range_bin_m,
        aspect_bin_deg=aspect_bin_deg, aspect_available=aspect_ok,
        ping_cycles=obs_set.ping_cycles, n_objects=len(objects),
        n_observations=len(obs),
        n_ghost_observations=len(ghosts),
        ghosts_by_reflector=dict(sorted(ghosts_by_reflector.items())),
        detected=detected,
        observed_not_detected=observed_not_detected, never_ensonified=never,
        range_curves=range_curves, aspect_curves=aspect_curves,
        per_class=per_class, objects=summaries,
        blocked_metrics=list(BLOCKED_METRICS))
