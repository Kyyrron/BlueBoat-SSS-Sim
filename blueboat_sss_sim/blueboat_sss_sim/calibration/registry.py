"""The parameter registry (``config/parameter_registry.yaml``).

One entry per important simulator constant: unit, class, provenance status,
bounds, whether a field calibration may move it, how it would be identified
and what it is confounded with. The registry is documentation the code can
check -- :func:`drift_report` compares every ``default`` with the value the
model actually uses -- and the gatekeeper a calibration artifact is checked
against (:mod:`.artifact`). Nothing reads a model value from it at run time.

ROS-free; numpy-free; reads only YAML and the package's own modules.
"""

from __future__ import annotations

import dataclasses
import importlib
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

REGISTRY_FORMAT = "blueboat_parameter_registry/1"
_PKG_ROOT = Path(__file__).resolve().parents[2]
_REQUIRED = ("name", "unit", "class", "status", "default", "tunable",
             "meaning", "identified_by", "metric", "uncertainty",
             "confounders", "experiment", "owner")
_OPTIONAL = ("bounds", "choices", "doc_label", "source", "tolerance")
_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*(\.[a-z0-9_]+)+$")
#: A ``key: value`` line whose plain (unquoted) value contains " #".
_UNQUOTED_HASH = re.compile(r"^\s*-?\s*([A-Za-z_][A-Za-z0-9_]*):[ \t]+"
                            r"(?![\"'|>\[{])[^#\n]*\S[ \t]+#")


def find_registry_yaml() -> Path:
    """The package's registry: source tree first, then the installed share
    directory (the same resolution as ``find_default_sonar_yaml``)."""
    src = _PKG_ROOT / "config" / "parameter_registry.yaml"
    if src.exists():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = (Path(get_package_share_directory("blueboat_sss_sim"))
                / "config" / "parameter_registry.yaml")
        if cand.exists():
            return cand
    except Exception:
        pass
    raise FileNotFoundError("config/parameter_registry.yaml not found")


@dataclass(frozen=True)
class Parameter:
    """One registry entry (see the YAML header for every field)."""

    name: str
    unit: str
    cls: str
    status: str
    default: Any
    tunable: bool
    meaning: str
    identified_by: str
    metric: str
    uncertainty: str
    confounders: tuple[str, ...]
    experiment: str
    owner: str
    bounds: tuple[float, float] | None = None
    choices: tuple[Any, ...] | None = None
    doc_label: str = ""
    source: dict = field(default_factory=dict)
    tolerance: float = 0.0

    @property
    def section(self) -> str:
        """``model`` / ``acquisition`` for sonar-profile parameters, else
        the registry's own namespace (``encoder``, ``material``, ``sea``…)."""
        return self.name.split(".", 1)[0]

    @property
    def key(self) -> str:
        return self.name.split(".", 1)[1]

    @property
    def in_sonar_profile(self) -> bool:
        """True for the parameters a sonar profile YAML carries."""
        return self.section in ("model", "acquisition")

    def check_value(self, value: Any) -> None:
        """Raise ``ValueError`` if ``value`` is outside what this parameter
        admits (choices, numeric bounds, type)."""
        if self.choices is not None:
            if value not in self.choices:
                raise ValueError(f"{self.name}: {value!r} not one of "
                                 f"{list(self.choices)}")
            return
        if isinstance(self.default, bool):
            if not isinstance(value, bool):
                raise ValueError(f"{self.name}: expects a boolean, got "
                                 f"{value!r}")
            return
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{self.name}: expects a number, got {value!r}")
        if not math.isfinite(float(value)):
            raise ValueError(f"{self.name}: {value!r} is not finite")
        if isinstance(self.default, int) and not isinstance(self.default, bool) \
                and float(value) != int(value):
            raise ValueError(f"{self.name}: expects an integer, got {value!r}")
        if self.bounds is not None:
            lo, hi = self.bounds
            if not (lo <= float(value) <= hi):
                raise ValueError(f"{self.name}: {value!r} outside bounds "
                                 f"[{lo}, {hi}] {self.unit}")


@dataclass
class Registry:
    """The parsed registry."""

    path: Path
    statuses: dict[str, str]
    classes: dict[str, str]
    session_classes: tuple[str, ...]
    parameters: dict[str, Parameter]

    def get(self, name: str) -> Parameter:
        try:
            return self.parameters[name]
        except KeyError as exc:
            raise KeyError(f"'{name}' is not in the parameter registry "
                           f"({self.path.name}); register it before a "
                           "calibration may set it") from exc

    def __iter__(self):
        return iter(self.parameters.values())

    def __len__(self) -> int:
        return len(self.parameters)


def load_registry(path: str | Path | None = None) -> Registry:
    """Parse and structurally validate the registry; raise ``ValueError``
    naming every problem found (not only the first)."""
    path = Path(path) if path is not None else find_registry_yaml()
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    doc = yaml.safe_load(text) or {}
    errors: list[str] = []
    # An unquoted " #" inside a plain value starts a YAML comment: the rest of
    # the line silently vanishes ("held at the NC #4-gated value" was read as
    # "held at the NC" for five entries until 2026-10-23). Quote such values.
    for n, line in enumerate(text.splitlines(), 1):
        m = _UNQUOTED_HASH.match(line)
        if m:
            errors.append(f"line {n}: {m.group(1)!r} has an unquoted ' #' -- "
                          "YAML drops the rest of the line as a comment; "
                          "quote the value")
    if doc.get("format") != REGISTRY_FORMAT:
        errors.append(f"format must be {REGISTRY_FORMAT!r}, got "
                      f"{doc.get('format')!r}")
    statuses = dict(doc.get("statuses") or {})
    classes = dict(doc.get("classes") or {})
    session_classes = tuple(doc.get("session_classes") or ())
    for c in session_classes:
        if c not in classes:
            errors.append(f"session_classes: unknown class {c!r}")
    params: dict[str, Parameter] = {}
    for i, e in enumerate(doc.get("parameters") or []):
        where = f"parameters[{i}] ({e.get('name', '?')})"
        missing = [k for k in _REQUIRED if k not in e]
        unknown = sorted(set(e) - set(_REQUIRED) - set(_OPTIONAL))
        if missing:
            errors.append(f"{where}: missing {missing}")
            continue
        if unknown:
            errors.append(f"{where}: unknown keys {unknown}")
        name = str(e["name"])
        if not _NAME_RE.match(name):
            errors.append(f"{where}: name must be dotted lower_snake")
        if name in params:
            errors.append(f"{where}: duplicate name")
        if e["status"] not in statuses:
            errors.append(f"{where}: unknown status {e['status']!r}")
        if e["class"] not in classes:
            errors.append(f"{where}: unknown class {e['class']!r}")
        bounds = e.get("bounds")
        if bounds is not None:
            if (not isinstance(bounds, list) or len(bounds) != 2
                    or not all(isinstance(b, (int, float)) for b in bounds)
                    or bounds[0] > bounds[1]):
                errors.append(f"{where}: bounds must be [lo, hi], lo <= hi")
                bounds = None
        p = Parameter(
            name=name, unit=str(e["unit"]), cls=str(e["class"]),
            status=str(e["status"]), default=e["default"],
            tunable=bool(e["tunable"]), meaning=str(e["meaning"]),
            identified_by=str(e["identified_by"]), metric=str(e["metric"]),
            uncertainty=str(e["uncertainty"]),
            confounders=tuple(str(c) for c in (e["confounders"] or ())),
            experiment=str(e["experiment"]), owner=str(e["owner"]),
            bounds=tuple(float(b) for b in bounds) if bounds else None,
            choices=tuple(e["choices"]) if e.get("choices") else None,
            doc_label=str(e.get("doc_label", "")),
            source=dict(e.get("source") or {}),
            tolerance=float(e.get("tolerance", 0.0)))
        try:
            p.check_value(p.default)
        except ValueError as exc:
            errors.append(f"{where}: default fails its own bounds: {exc}")
        if p.tunable and p.status in ("physical", "structural", "run_setting"):
            errors.append(f"{where}: a {p.status} parameter cannot be tunable")
        if p.tunable and p.cls in ("acquisition", "augmentation",
                                   "model_structure", "test_injection"):
            errors.append(f"{where}: class {p.cls} is never calibrated")
        params[name] = p
    for p in params.values():
        for c in p.confounders:
            # Only registry-shaped references are checked; free text
            # ("vehicle speed") documents a confounder outside the model.
            if (c.startswith("model.") or c.startswith("acquisition.")) \
                    and c not in params:
                errors.append(f"{p.name}: confounder {c!r} not registered")
    if errors:
        raise ValueError("parameter registry invalid:\n  - "
                         + "\n  - ".join(errors))
    return Registry(path=Path(path), statuses=statuses, classes=classes,
                    session_classes=session_classes, parameters=params)


# ------------------------------------------------------------- code values
def _python_value(spec: str) -> Any:
    """``"pkg.module:ATTR"``, ``"pkg.module:ATTR[key]"`` or
    ``"pkg.module:Dataclass.field"`` (the field's default)."""
    mod_name, _, expr = spec.partition(":")
    mod = importlib.import_module(mod_name)
    m = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)(?:\.([A-Za-z_][A-Za-z0-9_]*))?"
                 r"(?:\[(-?\d+|'[^']*'|\"[^\"]*\")\])?$", expr)
    if not m:
        raise ValueError(f"cannot parse python source {spec!r}")
    obj = getattr(mod, m.group(1))
    if m.group(2):
        if dataclasses.is_dataclass(obj):
            f = {x.name: x for x in dataclasses.fields(obj)}[m.group(2)]
            obj = f.default
        else:
            obj = getattr(obj, m.group(2))
    if m.group(3) is not None:
        k = m.group(3)
        obj = obj[int(k)] if k.lstrip("-").isdigit() else obj[k.strip("'\"")]
    return obj


def _yaml_value(file: str, key: str) -> Any:
    with open(_PKG_ROOT / file, "r", encoding="utf-8") as f:
        node = yaml.safe_load(f)
    for part in key.split("."):
        node = node[part]
    return node


def code_value(p: Parameter, profile_path: str | Path | None = None) -> Any:
    """The value the model uses for ``p`` today.

    ``model.*`` / ``acquisition.*``: the value the shipped default profile
    loads to (dataclass default filled in where the YAML is silent).
    Otherwise ``source`` decides: a module constant / dataclass default
    (``python``), a material attribute (``material``), or a YAML key
    (``yaml``).
    """
    if p.in_sonar_profile:
        from ..sonar.config import SonarConfig, find_default_sonar_yaml
        cfg = SonarConfig.from_yaml(profile_path or find_default_sonar_yaml())
        return getattr(getattr(cfg, p.section), p.key)
    src = p.source
    if "python" in src:
        return _python_value(str(src["python"]))
    if "material" in src:
        from ..worldgen.materials import MaterialLibrary
        mat, attr = str(src["material"]).split(".", 1)
        m = MaterialLibrary().get(mat)
        if attr == "roughness_m":
            from ..worldgen.materials import surface_roughness_m
            return surface_roughness_m(m)
        return getattr(m, attr)
    if "yaml" in src:
        y = src["yaml"]
        return _yaml_value(str(y["file"]), str(y["key"]))
    raise ValueError(f"{p.name}: no way to read its code value (add a "
                     "'source')")


def dataclass_default(p: Parameter) -> Any:
    """The dataclass default of a sonar-profile parameter (``None`` for
    other sections)."""
    if not p.in_sonar_profile:
        return None
    from ..sonar.config import AcquisitionParams, SonarModelConfig
    cls = SonarModelConfig if p.section == "model" else AcquisitionParams
    fields = {f.name: f for f in dataclasses.fields(cls)}
    if p.key not in fields:
        raise KeyError(f"{p.name}: no such field on {cls.__name__}")
    return fields[p.key].default


def _same(a: Any, b: Any, tol: float) -> bool:
    if isinstance(a, bool) or isinstance(b, bool) or isinstance(a, str) \
            or isinstance(b, str):
        return a == b
    if tol > 0.0:
        return abs(float(a) - float(b)) <= tol
    return float(a) == float(b)


def drift_report(reg: Registry,
                 profile_path: str | Path | None = None) -> dict:
    """Compare every registry default with the code.

    Returns ``{"ok": bool, "checked": n, "drift": [...], "implicit": [...]}``
    -- ``drift`` lists mismatches (an error: the registry or the code moved
    without the other), ``implicit`` the profile parameters the shipped
    YAML leaves to the dataclass default (information: a frozen copy of
    that profile would not freeze them).
    """
    drift: list[dict] = []
    implicit: list[str] = []
    prof_doc = None
    if profile_path is None:
        from ..sonar.config import find_default_sonar_yaml
        profile_path = find_default_sonar_yaml()
    with open(profile_path, "r", encoding="utf-8") as f:
        prof_doc = yaml.safe_load(f) or {}
    for p in reg:
        try:
            got = code_value(p, profile_path)
        except Exception as exc:  # a broken source is drift too
            drift.append({"name": p.name, "registry": p.default,
                          "code": f"<unreadable: {exc}>"})
            continue
        if not _same(p.default, got, p.tolerance):
            drift.append({"name": p.name, "registry": p.default, "code": got})
        if p.in_sonar_profile:
            dd = dataclass_default(p)
            if not _same(p.default, dd, p.tolerance):
                drift.append({"name": p.name, "registry": p.default,
                              "code": dd, "where": "dataclass default"})
            if p.key not in (prof_doc.get(p.section) or {}):
                implicit.append(p.name)
    return {"ok": not drift, "checked": len(reg), "drift": drift,
            "implicit": implicit, "profile": str(profile_path)}


def registry_markdown(reg: Registry) -> str:
    """A compact markdown table of the registry (the human view in
    ``docs/engineering/parameter_registry.md`` is generated from this)."""
    rows = ["| Name | Value | Unit | Class | Status | Tunable | Identified by "
            "| Confounders |", "|---|---|---|---|---|---|---|---|"]
    for p in sorted(reg, key=lambda q: (q.cls, q.name)):
        conf = ", ".join(f"`{c}`" if "." in c and " " not in c else c
                         for c in p.confounders) or "--"
        rows.append(f"| `{p.name}` | {p.default} | {p.unit} | {p.cls} | "
                    f"{p.status} | {'yes' if p.tunable else 'no'} | "
                    f"{p.identified_by} | {conf} |")
    return "\n".join(rows) + "\n"
