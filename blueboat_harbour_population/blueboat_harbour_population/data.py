"""The model's numbers: ``data/evidence_parameters.yaml`` and ``data/splits.yaml``.

Every number the model uses is a *parameter record* of the evidence file: a
mapping with ``value`` and ``tag`` (``REF``, ``DERIVED``, ``DESIGN``,
``CHECK`` or ``FROZEN``), a bracket, source keys and the function that reads
it. :class:`Parameters` gives the values by dotted path and checks the file's
form; ``docs/population/EVIDENCE.md`` holds the sources themselves.

Also the package's two digests (what a world's ``builder_state.yaml``, the
manifest and the world card record): the **code digest** over this package's
``*.py`` and ``data/*.yaml``, and the **evidence digest** over the evidence
file alone.

Pure Python (PyYAML).
"""

from __future__ import annotations

import copy
import hashlib
from functools import lru_cache
from pathlib import Path
from typing import Any, Iterator, Mapping

import yaml

PACKAGE = "blueboat_harbour_population"
PARAMETERS_FORMAT = "blueboat_population_parameters/1"
SPLITS_FORMAT = "blueboat_population_splits/1"
TAGS = ("REF", "DERIVED", "DESIGN", "CHECK", "FROZEN")
#: the target classes, in the order of the object set (T1..T7), and the clutter class
TARGETS = ("plastic_box", "drum_200l", "concrete_block", "trap_pot", "pipe_steel", "tire_car", "net_bundle")
DEBRIS = "debris"
CLASSES = TARGETS + (DEBRIS,)
LEVELS = tuple(range(1, 11))


def package_dir() -> Path:
    return Path(__file__).resolve().parent


def data_dir() -> Path:
    return package_dir() / "data"


def _sha(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def _digest_of(files: Mapping[str, str]) -> str:
    h = hashlib.sha256()
    for name in sorted(files):
        h.update(name.encode())
        h.update(b"\0")
        h.update(files[name].encode())
        h.update(b"\n")
    return h.hexdigest()


def code_digest() -> str:
    """16 hex digits over every ``*.py`` and ``data/*.yaml`` of the package
    (the same scheme as the simulator's ``core/provenance.py``)."""
    pkg = package_dir()
    files = {}
    for p in sorted(pkg.rglob("*.py")):
        if "__pycache__" not in p.parts:
            files[f"{PACKAGE}/{p.relative_to(pkg).as_posix()}"] = _sha(p)
    for p in sorted((pkg / "data").glob("*.yaml")):
        files[f"{PACKAGE}/data/{p.name}"] = _sha(p)
    return _digest_of(files)[:16]


def evidence_digest(path: str | Path | None = None) -> str:
    """16 hex digits of the evidence file as it is on disk."""
    return _sha(Path(path) if path else data_dir() / "evidence_parameters.yaml")[:16]


def is_record(node: Any) -> bool:
    return isinstance(node, Mapping) and "value" in node and "tag" in node


class Parameters:
    """The evidence file, read once. ``p("levels.density_per_ha")`` is the
    value of that record (a deep copy: callers cannot change the model)."""

    def __init__(self, doc: Mapping[str, Any], path: str = "") -> None:
        if doc.get("format") != PARAMETERS_FORMAT:
            raise ValueError(f"{path or 'parameters'}: format {doc.get('format')!r} (expected {PARAMETERS_FORMAT})")
        self.doc = doc
        self.path = path
        self._cache: dict[str, Any] = {}

    @classmethod
    def load(cls, path: str | Path | None = None) -> "Parameters":
        p = Path(path) if path else data_dir() / "evidence_parameters.yaml"
        return cls(yaml.safe_load(p.read_text(encoding="utf-8")) or {}, str(p))

    def node(self, dotted: str) -> Any:
        cur: Any = self.doc
        for key in dotted.split("."):
            if not isinstance(cur, Mapping) or key not in cur:
                raise KeyError(f"parameters: no {dotted!r} (missing {key!r})")
            cur = cur[key]
        return cur

    def record(self, dotted: str) -> Mapping[str, Any]:
        n = self.node(dotted)
        if not is_record(n):
            raise KeyError(f"parameters: {dotted!r} is not a parameter record (no value / tag)")
        return n

    def __call__(self, dotted: str) -> Any:
        if dotted not in self._cache:
            self._cache[dotted] = self.record(dotted)["value"]
        return copy.deepcopy(self._cache[dotted])

    def has(self, dotted: str) -> bool:
        try:
            self.record(dotted)
            return True
        except KeyError:
            return False

    def records(self) -> Iterator[tuple[str, Mapping[str, Any]]]:
        """Every parameter record with its dotted path."""
        def walk(node, pre):
            if is_record(node):
                yield pre, node
                return
            if isinstance(node, Mapping):
                for k, v in node.items():
                    yield from walk(v, f"{pre}.{k}" if pre else str(k))
        yield from walk({k: v for k, v in self.doc.items() if k != "format"}, "")

    # ----------------------------------------------------------- vocabulary
    @property
    def tags(self) -> list[str]:
        return list(self.doc["tags"])

    @property
    def profiles(self) -> list[str]:
        return list(self.doc["profiles"])

    def check(self) -> list[str]:
        """Problems of form (an empty list: none): a leaf outside a record, a
        record without a valid tag or without ``used_by``, a class or a tag
        that does not exist."""
        bad: list[str] = []

        def walk(node, pre):
            if is_record(node):
                if node["tag"] not in TAGS:
                    bad.append(f"{pre}: tag {node['tag']!r}")
                if not node.get("used_by"):
                    bad.append(f"{pre}: no used_by")
                if node["tag"] in ("REF", "DERIVED") and not node.get("sources"):
                    bad.append(f"{pre}: a {node['tag']} parameter names its sources")
                extra = set(node) - {"value", "tag", "bracket", "sources", "used_by", "note"}
                if extra:
                    bad.append(f"{pre}: unknown key(s) {sorted(extra)}")
                return
            if isinstance(node, Mapping):
                for k, v in node.items():
                    walk(v, f"{pre}.{k}" if pre else str(k))
            else:
                bad.append(f"{pre}: a bare value (every number sits in a record with a tag)")
        walk({k: v for k, v in self.doc.items() if k != "format"}, "")
        tags = set(self.tags) | {"none"}
        for t in self.tags:
            em = self(f"tags.{t}.emits")
            if set(em) != set(TARGETS):
                bad.append(f"tags.{t}.emits: the seven target classes, got {sorted(em)}")
            k = self(f"tags.{t}.kernel")
            if k.get("near") not in ("half_normal", "exponential", "uniform", "bump"):
                bad.append(f"tags.{t}.kernel.near: {k.get('near')!r}")
            if not 0.0 <= float(k.get("broad_share", 0.0)) <= 1.0:
                bad.append(f"tags.{t}.kernel.broad_share out of [0, 1]")
            if self.has(f"tags.{t}.patch"):
                pt = self(f"tags.{t}.patch")
                if not 0.0 <= float(pt["share"]) <= 1.0 or float(pt["sigma_m"]) <= 0.0 \
                        or not ("spots_per_100m" in pt or "spots_per_ha" in pt):
                    bad.append(f"tags.{t}.patch: share in [0, 1], sigma_m > 0, spots_per_100m or spots_per_ha")
            for g, spec in self(f"tags.{t}.groups").items():
                if g not in self.doc["groups"]:
                    bad.append(f"tags.{t}.groups: no group process {g!r}")
                if spec.get("class") not in TARGETS + ("*",):
                    bad.append(f"tags.{t}.groups.{g}: class {spec.get('class')!r}")
        for pr in self.profiles:
            w = self(f"profiles.{pr}.weights")
            if set(w) != set(TARGETS):
                bad.append(f"profiles.{pr}.weights: the seven target classes, got {sorted(w)}")
            for kind, use in self(f"profiles.{pr}.kind_use").items():
                if use not in tags:
                    bad.append(f"profiles.{pr}.kind_use.{kind}: no tag {use!r}")
            for use in self(f"profiles.{pr}.reference"):
                if use not in tags:
                    bad.append(f"profiles.{pr}.reference: no tag {use!r}")
        for name in ("density_per_ha", "group_chance", "small_litter_per_m2", "retention"):
            if set(self(f"levels.{name}")) != set(LEVELS):
                bad.append(f"levels.{name}: the levels 1..10")
        return bad


@lru_cache(maxsize=1)
def default_parameters() -> Parameters:
    p = Parameters.load()
    problems = p.check()
    if problems:
        raise ValueError("evidence_parameters.yaml: " + "; ".join(problems[:8])
                         + (f" (+{len(problems) - 8} more)" if len(problems) > 8 else ""))
    return p


@lru_cache(maxsize=1)
def default_splits() -> dict:
    doc = yaml.safe_load((data_dir() / "splits.yaml").read_text(encoding="utf-8")) or {}
    if doc.get("format") != SPLITS_FORMAT:
        raise ValueError(f"splits.yaml: format {doc.get('format')!r} (expected {SPLITS_FORMAT})")
    return doc
