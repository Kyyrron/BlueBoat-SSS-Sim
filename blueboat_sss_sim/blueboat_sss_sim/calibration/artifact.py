"""Calibration artifacts: versioned, validated, never silent edits.

A field calibration does not edit ``config/default_sonar.yaml``. It writes
one human-readable artifact (``blueboat_calibration_artifact/1``, YAML or
JSON) that records the fitted values WITH their units, uncertainty, the
sessions they were fitted on, the sessions held out to validate them, the
fit method and objective, the assumptions, the excluded data and the
validation results -- and nothing raw: no pings, no tracks, no precise GPS.
:func:`effective_config` then composes the model a run uses, in a fixed
precedence order, and says where every value came from.

Precedence (lowest first) -- docs/engineering/field_calibration_architecture.md:

1. dataclass defaults (``SonarModelConfig`` / ``AcquisitionParams``);
2. the sonar profile YAML (``config/*_sonar.yaml``);
3. the artifact's global ``parameters`` (installation / device /
   environment classes, ``tunable: true`` in the registry only);
4. the artifact's per-session values (``sessions:``; only the registry's
   ``session_classes`` -- a re-mounted transducer, never a seabed law);
5. per-run acquisition overrides (``acquisition.*`` only).

Augmentation and model-structure parameters can never come from a
calibration artifact. ROS-free; YAML + the registry only.
"""

from __future__ import annotations

import dataclasses
import datetime as _dt
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml

from .registry import Registry, load_registry

ARTIFACT_FORMAT = "blueboat_calibration_artifact/1"
_TOP_REQUIRED = ("format", "id", "created", "parameters", "data", "fit",
                 "assumptions", "validation")
_TOP_OPTIONAL = ("created_by", "description", "software", "scope",
                 "base_profile", "sessions", "excluded", "notes")
_FITTED_STATUSES = ("measured", "joint", "estimated")
_UNCERTAINTY_KEYS = ("sd", "interval", "unidentified", "equivalence_range")
#: Keys that would put a position fix into an artifact (sensitive, and not
#: calibration content).
_GPS_KEY = re.compile(r"^(lat|lon|latitude|longitude|gps|lat0|lon0|"
                      r"first_fix|track)$", re.IGNORECASE)
#: A numeric list longer than this is raw data, not a calibration value.
_MAX_LIST = 64


class ArtifactError(ValueError):
    """An artifact that must not be applied; ``errors`` lists every
    problem found."""

    def __init__(self, errors: list[str]) -> None:
        self.errors = list(errors)
        super().__init__("calibration artifact invalid:\n  - "
                         + "\n  - ".join(self.errors))


@dataclass
class ArtifactReport:
    """Validation outcome: ``errors`` block use, ``warnings`` do not."""

    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def load_artifact(path: str | Path) -> dict:
    """Read a YAML (or JSON) artifact as a plain mapping (unvalidated)."""
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    doc = (json.loads(text) if str(path).endswith(".json")
           else yaml.safe_load(text))
    if not isinstance(doc, Mapping):
        raise ArtifactError([f"{path}: not a mapping"])
    return dict(doc)


def save_artifact(doc: Mapping[str, Any], path: str | Path) -> Path:
    """Write ``doc`` after validating it (YAML, or JSON for ``.json``).
    Refuses to overwrite: an artifact is a record, like a world folder."""
    path = Path(path)
    rep = validate_artifact(doc)
    if not rep.ok:
        raise ArtifactError(rep.errors)
    if path.exists():
        raise FileExistsError(f"{path} exists; artifacts are write-once "
                              "(give the new calibration a new id/file)")
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "x", encoding="utf-8") as f:
        if path.suffix == ".json":
            json.dump(doc, f, indent=2, sort_keys=False)
        else:
            yaml.safe_dump(dict(doc), f, sort_keys=False)
    return path


def _walk(node: Any, where: str, errors: list[str]) -> None:
    """Content rules that apply anywhere in the document."""
    if isinstance(node, Mapping):
        for k, v in node.items():
            if _GPS_KEY.match(str(k)):
                errors.append(f"{where}.{k}: position data does not belong in "
                              "a calibration artifact (keep it in the "
                              "session's own records)")
            _walk(v, f"{where}.{k}", errors)
    elif isinstance(node, (list, tuple)):
        if len(node) > _MAX_LIST and all(isinstance(x, (int, float))
                                         for x in node):
            errors.append(f"{where}: a {len(node)}-number list is raw data; "
                          "store a summary and reference the recording")
        for i, v in enumerate(node):
            _walk(v, f"{where}[{i}]", errors)


def _check_uncertainty(u: Any, where: str, errors: list[str]) -> None:
    if not isinstance(u, Mapping) or not (set(u) & set(_UNCERTAINTY_KEYS)):
        errors.append(f"{where}: uncertainty must state one of "
                      f"{list(_UNCERTAINTY_KEYS)} (use unidentified: <why> "
                      "rather than an unjustified precise value)")
        return
    if "sd" in u and not (isinstance(u["sd"], (int, float))
                          and u["sd"] >= 0):
        errors.append(f"{where}: sd must be a non-negative number")
    for k in ("interval", "equivalence_range"):
        if k in u:
            iv = u[k]
            if (not isinstance(iv, (list, tuple)) or len(iv) != 2
                    or not all(isinstance(x, (int, float)) for x in iv)
                    or iv[0] > iv[1]):
                errors.append(f"{where}: {k} must be [lo, hi]")


def _check_value(reg: Registry, name: str, entry: Any, where: str,
                 errors: list[str], *, session: bool) -> None:
    try:
        p = reg.get(name)
    except KeyError as exc:
        errors.append(f"{where}: {exc.args[0]}")
        return
    if not p.tunable:
        errors.append(f"{where}: {name} is not tunable (class {p.cls}, "
                      f"status {p.status}); a calibration cannot set it")
    if session and p.cls not in reg.session_classes:
        errors.append(f"{where}: {name} (class {p.cls}) cannot vary per "
                      f"session; only {list(reg.session_classes)} can -- a "
                      "per-session value would hide a global model error")
    if not isinstance(entry, Mapping) or "value" not in entry:
        errors.append(f"{where}: needs {{value, unit, uncertainty, status}}")
        return
    if str(entry.get("unit")) != p.unit:
        errors.append(f"{where}: unit {entry.get('unit')!r} != registry "
                      f"unit {p.unit!r}")
    try:
        p.check_value(entry["value"])
    except ValueError as exc:
        errors.append(f"{where}: {exc}")
    if entry.get("status") not in _FITTED_STATUSES:
        errors.append(f"{where}: status must be one of {_FITTED_STATUSES} "
                      "(a value from data); defaults stay in the profile")
    _check_uncertainty(entry.get("uncertainty"), f"{where}.uncertainty",
                       errors)


def validate_artifact(doc: Mapping[str, Any],
                      registry: Registry | None = None) -> ArtifactReport:
    """Every rule an artifact must satisfy before it may be applied."""
    reg = registry or load_registry()
    rep = ArtifactReport()
    e = rep.errors
    if doc.get("format") != ARTIFACT_FORMAT:
        e.append(f"format must be {ARTIFACT_FORMAT!r}")
    missing = [k for k in _TOP_REQUIRED if k not in doc]
    if missing:
        e.append(f"missing top-level keys {missing}")
    unknown = sorted(set(doc) - set(_TOP_REQUIRED) - set(_TOP_OPTIONAL))
    if unknown:
        e.append(f"unknown top-level keys {unknown}")
    if not str(doc.get("id", "")).strip():
        e.append("id must be a non-empty string")
    try:
        _dt.date.fromisoformat(str(doc.get("created", ""))[:10])
    except ValueError:
        e.append("created must be an ISO date (YYYY-MM-DD...)")
    _walk(doc, "artifact", e)

    data = doc.get("data") or {}
    cal = [str(s) for s in (data.get("calibration_sessions") or [])]
    val = [str(s) for s in (data.get("validation_sessions") or [])]
    if not cal:
        e.append("data.calibration_sessions must name the sessions fitted on")
    leak = sorted(set(cal) & set(val))
    if leak:
        e.append(f"data: sessions {leak} are both calibration and "
                 "validation -- split by whole session")
    if not val:
        rep.warnings.append("no validation_sessions: the values are "
                            "unvalidated (state it wherever they are used)")

    params = doc.get("parameters") or {}
    if not isinstance(params, Mapping):
        e.append("parameters must be a mapping name -> {value, ...}")
        params = {}
    for name, entry in params.items():
        _check_value(reg, str(name), entry, f"parameters.{name}", e,
                     session=False)
    sessions = doc.get("sessions") or {}
    for sid, block in (sessions.items() if isinstance(sessions, Mapping)
                       else []):
        if str(sid) not in cal:
            e.append(f"sessions.{sid}: per-session values only for "
                     "calibration sessions")
        for name, entry in (block or {}).items():
            _check_value(reg, str(name), entry, f"sessions.{sid}.{name}", e,
                         session=True)
    # A per-session term that leans the same way in every session is a
    # global error dressed as nuisance: say so.
    by_param: dict[str, list[float]] = {}
    for block in (sessions.values() if isinstance(sessions, Mapping) else []):
        for name, entry in (block or {}).items():
            if isinstance(entry, Mapping) and isinstance(
                    entry.get("value"), (int, float)):
                by_param.setdefault(str(name), []).append(float(entry["value"]))
    for name, vals in by_param.items():
        if len(vals) < 3:
            continue
        base = params.get(name, {}).get("value") if isinstance(
            params.get(name), Mapping) else None
        if base is None:
            try:
                base = reg.get(name).default
            except KeyError:
                continue
        d = [v - float(base) for v in vals]
        if all(x > 0 for x in d) or all(x < 0 for x in d):
            rep.warnings.append(
                f"{name}: every session deviates the same way from "
                f"{base} ({min(d):+.3g}..{max(d):+.3g}) -- a systematic "
                "offset belongs in the global value, or in the model")

    fit = doc.get("fit") or {}
    if "placeholder" in str(doc.get("id", "")).lower() \
            or str(fit.get("method", "")).lower() == "placeholder":
        rep.warnings.append("placeholder artifact: its values are not a "
                            "calibration; never apply it to a run")
    for k in ("method", "objective"):
        if not str(fit.get(k, "")).strip():
            e.append(f"fit.{k} must say how the values were obtained")
    for name, b in (fit.get("bounds") or {}).items():
        if (not isinstance(b, (list, tuple)) or len(b) != 2
                or b[0] > b[1]):
            e.append(f"fit.bounds.{name} must be [lo, hi]")
    if not isinstance(doc.get("assumptions"), list) \
            or not doc.get("assumptions"):
        e.append("assumptions must list what the fit took as given")
    for i, v in enumerate(doc.get("validation") or []):
        where = f"validation[{i}]"
        if not isinstance(v, Mapping):
            e.append(f"{where}: must be a mapping")
            continue
        for k in ("metric", "session", "value", "passed"):
            if k not in v:
                e.append(f"{where}: missing {k}")
        if "session" in v and str(v["session"]) not in val:
            e.append(f"{where}: session {v['session']!r} is not a "
                     "validation session -- a fit scored on its own data "
                     "is not a validation")
    return rep


# ----------------------------------------------------------- composition
@dataclass
class EffectiveConfig:
    """A composed sonar configuration and where each value came from."""

    config: Any                         # sonar.config.SonarConfig
    provenance: dict[str, str]          # "model.x" -> layer label
    not_applied: dict[str, str]         # artifact names outside the profile


def effective_config(profile_path: str | Path | None = None,
                     artifact: Mapping[str, Any] | None = None,
                     session_id: str | None = None,
                     run_overrides: Mapping[str, Any] | None = None,
                     registry: Registry | None = None) -> EffectiveConfig:
    """Compose the model in the documented precedence order (module doc).

    Raises :class:`ArtifactError` for an invalid artifact and ``ValueError``
    for a run override that is not an ``acquisition.*`` parameter. Registry
    parameters that live outside the sonar profile (materials, hull, ...)
    are returned in ``not_applied`` with their owner: this function does
    not pretend to apply them.
    """
    from ..sonar.config import (AcquisitionParams, SonarConfig,
                                SonarModelConfig, find_default_sonar_yaml)
    reg = registry or load_registry()
    profile_path = Path(profile_path or find_default_sonar_yaml())
    cfg = SonarConfig.from_yaml(profile_path)
    with open(profile_path, "r", encoding="utf-8") as f:
        prof = yaml.safe_load(f) or {}
    prov: dict[str, str] = {}
    for sect, cls in (("acquisition", AcquisitionParams),
                      ("model", SonarModelConfig)):
        for fl in dataclasses.fields(cls):
            prov[f"{sect}.{fl.name}"] = (
                f"profile:{profile_path.name}"
                if fl.name in (prof.get(sect) or {}) else "dataclass_default")
    not_applied: dict[str, str] = {}

    def _set(name: str, value: Any, label: str) -> None:
        p = reg.get(name)
        if not p.in_sonar_profile:
            not_applied[name] = p.owner
            return
        p.check_value(value)
        setattr(getattr(cfg, p.section), p.key, value)
        prov[name] = label

    if artifact is not None:
        rep = validate_artifact(artifact, reg)
        if not rep.ok:
            raise ArtifactError(rep.errors)
        aid = str(artifact["id"])
        for name, entry in (artifact.get("parameters") or {}).items():
            _set(str(name), entry["value"], f"artifact:{aid}")
        if session_id is not None:
            sessions = artifact.get("sessions") or {}
            cal = [str(s) for s in
                   (artifact.get("data") or {}).get("calibration_sessions")
                   or []]
            if session_id not in sessions and session_id not in cal:
                raise ValueError(f"session {session_id!r} is not in artifact "
                                 f"{aid}")
            for name, entry in (sessions.get(session_id) or {}).items():
                _set(str(name), entry["value"],
                     f"artifact:{aid}/session:{session_id}")
    elif session_id is not None:
        raise ValueError("a session_id needs an artifact")
    for name, value in (run_overrides or {}).items():
        if not str(name).startswith("acquisition."):
            raise ValueError(f"run override {name!r}: only acquisition.* "
                             "parameters are set per run")
        p = reg.parameters.get(str(name))
        key = str(name).split(".", 1)[1]
        if p is not None:
            p.check_value(value)
        if not hasattr(cfg.acquisition, key):
            raise KeyError(f"{name}: no such acquisition parameter")
        setattr(cfg.acquisition, key, value)
        prov[str(name)] = "run"
    # Re-run the dataclass checks (gain ladder) on the composed values.
    cfg.acquisition = AcquisitionParams(**dataclasses.asdict(cfg.acquisition))
    return EffectiveConfig(config=cfg, provenance=prov,
                           not_applied=not_applied)


def write_effective_profile(eff: EffectiveConfig, path: str | Path,
                            sources: Mapping[str, Any] | None = None) -> Path:
    """Write a SELF-CONTAINED sonar profile: every acquisition and model
    field stated (a frozen copy then freezes everything), plus a
    ``provenance:`` block (ignored by ``SonarConfig.from_yaml``, which
    reads only ``acquisition:`` and ``model:``). Refuses to overwrite."""
    path = Path(path)
    if path.exists():
        raise FileExistsError(f"{path} exists; write the profile to a new "
                              "file")
    doc = {
        "acquisition": dataclasses.asdict(eff.config.acquisition),
        "model": dataclasses.asdict(eff.config.model),
        "provenance": {
            "format": "blueboat_effective_profile/1",
            "sources": dict(sources or {}),
            "from_layer": {k: v for k, v in sorted(eff.provenance.items())
                           if v != "dataclass_default"},
            "not_applied": dict(eff.not_applied),
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "x", encoding="utf-8") as f:
        f.write("# Effective sonar profile composed by `python3 -m "
                "blueboat_sss_sim.calibration effective-profile`.\n"
                "# Self-contained: every field is stated. `provenance` "
                "records where each non-default value came from.\n")
        yaml.safe_dump(doc, f, sort_keys=False)
    return path


def example_artifact() -> dict:
    """A syntactically complete artifact for documentation and tests. Its
    numbers are PLACEHOLDERS, not a calibration: they restate the shipped
    defaults with wide uncertainty and must never be applied to a run."""
    return {
        "format": ARTIFACT_FORMAT,
        "id": "example-placeholder-not-a-calibration",
        "created": "2026-10-21",
        "created_by": "documentation example",
        "description": "Shape of a calibration artifact; values are the "
                       "shipped defaults, NOT fitted.",
        "software": {"package": "blueboat_sss_sim", "version": "1.0.0"},
        "base_profile": {"name": "default_sonar.yaml"},
        "parameters": {
            "model.calibration_db_offset": {
                "value": 81.1, "unit": "dB", "status": "joint",
                "uncertainty": {"sd": 0.7},
                "note": "placeholder = shipped default"},
            "model.tvg_compensation": {
                "value": 0.30, "unit": "fraction", "status": "joint",
                "uncertainty": {"unidentified": "single-altitude data would "
                                                "not separate it"}},
        },
        "sessions": {
            "S1": {"model.sensor_depth_m": {
                "value": 0.15, "unit": "m", "status": "measured",
                "uncertainty": {"sd": 0.01}}},
        },
        "data": {"manifest": "example_manifest.yaml",
                 "calibration_sessions": ["S1"],
                 "validation_sessions": ["S2"]},
        "excluded": [{"recording": "S1/pass03.svlog",
                      "reason": "turn, roll > 5 deg"}],
        "fit": {"method": "placeholder", "objective": "placeholder",
                "bounds": {"model.calibration_db_offset": [60.0, 110.0]}},
        "assumptions": ["flat seabed under the fitted window",
                        "gain index 4 throughout"],
        "validation": [{"metric": "seabed_db_tail_bias", "session": "S2",
                        "value": None, "acceptance": "TBD from repeatability",
                        "passed": None}],
    }
