"""What a mission-like world records: the ``placement`` block of
``metadata.yaml`` (per world) and the placement keys of each object's
``params``. The checks here are what the tests, the batch tools and a reader
of a world can rely on (additive to the simulator's metadata schema
version 2; ``format`` stays ``blueboat_world_meta/1``).

Per world, ``placement``:

    mode, model {package, version}, plan_digest, environment_digest, profile, density_level,
    lambda_active_per_ha, classes, mix, weights, seed, split, clutter, structures, advanced,
    features [{id, kind, use, length_m, activity, owned_ha, expected, placed}],
    groups [{id, type, source, use, x, y, radius_m, members, classes}],
    small_litter, envelope, expected, realised, warnings, stress_world, showcase_world, gaps
    (+ harbour, split_override, feature_tags, feature_activity, origin when used)

Per object, in ``objects[i].params`` (beside the simulator's own keys):

    source {feature, use, process, index}, group_id, stand_in_for, depth_m, altitude_m,
    sediment_under, on_mud, in_envelope

and, lifted by the simulator to the object itself: ``role``, ``nominal_target``,
``effectively_buried``, ``pose``, ``state``, ``variant``, ``hotspot`` (the source feature's id).

Pure Python.
"""

from __future__ import annotations

from typing import Any, Mapping

WORLD_KEYS = {
    "mode": str, "model": dict, "plan_digest": str, "environment_digest": str, "profile": str,
    "density_level": int, "lambda_active_per_ha": (int, float), "classes": list, "mix": (str, dict),
    "weights": dict, "seed": int, "split": str, "clutter": dict, "structures": str, "advanced": dict,
    "features": list, "groups": list, "small_litter": dict, "envelope": dict, "expected": dict,
    "realised": dict, "warnings": list, "stress_world": bool, "showcase_world": bool, "gaps": list,
}
WORLD_OPTIONAL = {"harbour": str, "split_override": str, "feature_tags": dict, "feature_activity": dict,
                  "origin": dict}
REALISED_KEYS = ("targets", "debris", "groups", "grouped_objects", "structures", "per_class",
                 "density_active_per_ha", "density_world_per_ha", "share_within_30m", "clark_evans",
                 "effectively_buried", "detectable_targets", "outside_band", "on_mud", "stand_ins",
                 "removed_by_spacing", "no_room")
FEATURE_KEYS = ("id", "kind", "use", "length_m", "activity", "owned_ha", "expected", "placed")
GROUP_KEYS = ("id", "type", "source", "use", "x", "y", "radius_m", "members", "classes")
OBJECT_KEYS = {"source": dict, "group_id": (str, type(None)), "stand_in_for": (str, type(None)),
               "depth_m": (int, float), "altitude_m": (int, float), "sediment_under": str, "on_mud": bool,
               "in_envelope": bool}
SOURCE_KEYS = ("feature", "use", "process", "index")


def check_world_block(block: Mapping[str, Any]) -> list[str]:
    """Problems of a ``placement`` block (an empty list: valid)."""
    bad: list[str] = []
    for k, t in WORLD_KEYS.items():
        if k not in block:
            bad.append(f"placement.{k}: missing")
        elif block[k] is not None and not isinstance(block[k], t) or (t is int and isinstance(block[k], bool)):
            bad.append(f"placement.{k}: {type(block[k]).__name__}")
    for k, t in WORLD_OPTIONAL.items():
        if k in block and not isinstance(block[k], t):
            bad.append(f"placement.{k}: {type(block[k]).__name__}")
    extra = set(block) - set(WORLD_KEYS) - set(WORLD_OPTIONAL)
    if extra:
        bad.append(f"placement: unknown key(s) {sorted(extra)}")
    model = block.get("model") or {}
    if set(model) != {"package", "version"}:
        bad.append(f"placement.model: package and version only, got {sorted(model)}")
    for k in REALISED_KEYS:
        if k not in (block.get("realised") or {}):
            bad.append(f"placement.realised.{k}: missing")
    for f in block.get("features") or []:
        for k in FEATURE_KEYS:
            if k not in f:
                bad.append(f"placement.features[{f.get('id')}].{k}: missing")
    for g in block.get("groups") or []:
        for k in GROUP_KEYS:
            if k not in g:
                bad.append(f"placement.groups[{g.get('id')}].{k}: missing")
    return bad


def check_object_params(params: Mapping[str, Any], where: str = "object") -> list[str]:
    """Problems of the placement keys of one object's ``params``."""
    bad: list[str] = []
    for k, t in OBJECT_KEYS.items():
        if k not in params:
            bad.append(f"{where}.params.{k}: missing")
        elif not isinstance(params[k], t):
            bad.append(f"{where}.params.{k}: {type(params[k]).__name__}")
    src = params.get("source") or {}
    for k in SOURCE_KEYS:
        if k not in src:
            bad.append(f"{where}.params.source.{k}: missing")
    return bad


def is_plain(obj: Any) -> bool:
    """Whether ``obj`` is made of plain YAML values only."""
    if obj is None or isinstance(obj, (str, bool, int, float)):
        return type(obj) in (type(None), str, bool, int, float)
    if isinstance(obj, (list, tuple)):
        return type(obj) is list and all(is_plain(v) for v in obj)
    if isinstance(obj, dict):
        return type(obj) is dict and all(type(k) in (str, int, float, bool) and is_plain(v) for k, v in obj.items())
    return False
