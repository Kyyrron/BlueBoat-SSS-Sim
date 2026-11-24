"""Mission-like placement: what the wizard keeps of it in its state
(simulator v1.1, 2026-11-21, owner order: world generation only).

Screen 5 has two modes. **Manual** is the page as it always was. In
**Mission-like** the objects are planned by the package
``blueboat_harbour_population`` (outside this package and outside its code
digest: its parameters are recalibrated without a new simulator version)
from the environment of Screens 1-4, a harbour type, a density level and a
seed. This module is the bookkeeping only -- no model, no Qt, no import of
the planner at module level:

* ``BuilderState.placement`` holds the record of a planned world (what was
  asked, the model's version and digests, the block for ``metadata.yaml``,
  the seabed layers and the seabed declaration the plan added). It is empty
  for every Manual world, and an empty record is not written: a Manual
  world's ``builder_state.yaml`` is byte for byte what it was.
* the planner's adapter writes a plan into the state
  (``sim_adapter.apply_plan``, idempotent); :func:`clear` takes it out
  again, :func:`reassert` restores what Screen 3 drops (it rebuilds the
  depth state without its ``seabed`` declaration).
* :func:`extras` is what the build passes to ``build_world_folder`` so that
  ``metadata.yaml`` carries the ``placement`` block.

The model's own description: ``BlueBoat-SSS-Sim/docs/population/``.
"""

from __future__ import annotations

import copy
from typing import Any, Mapping

#: ids of the seabed layers a plan adds start with this (the planner's
#: ``layers.RESERVED_PREFIX``; a test holds the two equal)
RESERVED_PREFIX = "ml_"
#: names of the structures a plan adds start with it too
MISSION = "mission_like"
MANUAL = "manual"


def plain(obj: Any) -> Any:
    """A deep copy made of plain Python values (what ``yaml.safe_dump``
    takes): a numpy scalar in the state would fail the build at its last
    step and make the autosave fail silently."""
    if isinstance(obj, Mapping):
        return {(k if isinstance(k, (str, int, float, bool)) else str(k)): plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [plain(v) for v in obj]
    if hasattr(obj, "tolist") and not isinstance(obj, (str, bytes)):
        return plain(obj.tolist())
    if hasattr(obj, "item") and not isinstance(obj, (str, bytes, int, float, bool)):
        return obj.item()
    return obj


def import_planner():
    """``(package, adapter, "")`` of ``blueboat_harbour_population``, or
    ``(None, None, why)`` when it cannot be imported: the Mission-like mode
    is then switched off and the builder works as before."""
    try:
        import blueboat_harbour_population as hp
        from blueboat_harbour_population import sim_adapter
        return hp, sim_adapter, ""
    except Exception as exc:                                    # noqa: BLE001
        return None, None, f"{type(exc).__name__}: {exc}"


def mode_of(state) -> str:
    """``mission_like`` when the state holds a plan, else ``manual`` (a plan
    converted to manual is manual)."""
    rec = getattr(state, "placement", None) or {}
    return MISSION if rec.get("mode") == MISSION else MANUAL


def is_reserved(layer: Mapping[str, Any]) -> bool:
    return str(dict(layer).get("id", "")).startswith(RESERVED_PREFIX)


def strip_reserved(layers) -> list:
    """The layers without those a plan added."""
    return [copy.deepcopy(dict(L)) for L in (layers or []) if not is_reserved(L)]


def is_plan_structure(entry: Mapping[str, Any]) -> bool:
    prm = dict(entry.get("params") or {})
    src = prm.get("source") if isinstance(prm.get("source"), Mapping) else {}
    return src.get("use") == "structure" and str(prm.get("name", "")).startswith(RESERVED_PREFIX)


def list_digest(items) -> str:
    """16 hex digits of a list of instances (to tell whether a converted
    plan was edited)."""
    import hashlib
    import json
    blob = json.dumps(plain(list(items or [])), sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(blob.encode()).hexdigest()[:16]


def restore_layers(state) -> None:
    """Put a stored plan's seabed layers back when they are gone (Screen 3
    lists them as ordinary rows: the operator may have deleted one)."""
    rec = dict(getattr(state, "placement", None) or {})
    if rec.get("mode") != MISSION:
        return
    want = [plain(L) for L in rec.get("layers") or []]
    have = [L for L in state.depth.layers if is_reserved(L)]
    if [dict(L).get("id") for L in have] != [L.get("id") for L in want]:
        state.depth.layers = strip_reserved(state.depth.layers) + want


def clear(state) -> None:
    """Take a plan's traces out of ``state`` (the mode was switched back to
    Manual): its record, its seabed layers, its structures and its seabed
    declaration. The pinned field is the page's own business."""
    rec = dict(getattr(state, "placement", None) or {})
    if not rec:
        return
    state.depth.layers = strip_reserved(state.depth.layers)
    state.objects.explicit = [e for e in state.objects.explicit if not is_plan_structure(e)]
    if "seabed_before" in rec:
        state.depth.seabed = plain(dict(rec["seabed_before"] or {}))
    state.placement = {}


def reassert(state) -> None:
    """Restore what a plan needs and Screen 3 drops: the depth page rebuilds
    its ``DepthState`` without the ``seabed`` declaration, so a state with a
    placement record gets it back from the record (a Manual state has no
    record: nothing happens)."""
    rec = dict(getattr(state, "placement", None) or {})
    if rec.get("seabed"):
        state.depth.seabed = plain(dict(rec["seabed"]))


def extras(state) -> dict | None:
    """The ``extras`` of ``build_world_folder`` for this state: the
    ``placement`` block of ``metadata.yaml`` (and the world's fouling under
    ``run``), or None for a state without a placement record -- the build
    call is then exactly what it always was."""
    rec = dict(getattr(state, "placement", None) or {})
    meta = rec.get("metadata")
    if not meta:
        return None
    out: dict[str, Any] = {"placement": plain(meta)}
    if rec.get("run"):
        out["run"] = plain(rec["run"])
    return out


def convert_record(record: Mapping[str, Any], edited: bool) -> dict:
    """The record of a plan converted to an editable manual list: the mode
    becomes ``manual``, the plan's own record is kept as ``origin`` and
    ``edited`` says whether an object was changed since."""
    rec = plain(dict(record or {}))
    origin = rec.get("origin") or {k: rec.get(k) for k in ("params", "model", "seed", "plan_digest",
                                                           "environment_digest")}
    meta = dict(rec.get("metadata") or {})
    meta_origin = meta.get("origin") or {k: meta.get(k) for k in meta if k not in ("origin", "mode", "edited")}
    out = {"mode": MANUAL, "origin": origin, "edited": bool(edited),
           "metadata": {"mode": MANUAL, "origin": meta_origin, "edited": bool(edited),
                        "note": "a mission-like plan converted to a manual list on World Builder Screen 5"}}
    for k in ("seabed", "seabed_before", "run"):
        if k in rec:
            out[k] = rec[k]
    return out


def unreserve(layers) -> list:
    """A plan's seabed layers as ordinary layers (their ids lose the
    reserved prefix, their seeds stay): what a converted plan keeps."""
    out = []
    for L in layers or []:
        L = plain(dict(L))
        if is_reserved(L):
            L["id"] = "converted_" + str(L["id"])[len(RESERVED_PREFIX):]
        out.append(L)
    return out
