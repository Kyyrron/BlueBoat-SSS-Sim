"""The result of :func:`populate`: what to place, and a report of it.

``instances`` are entries of the frozen generator's explicit-instance format
(``worldgen/objects.py``), in the environment's frame, in the order the world
will hold them. ``structures`` are the same kind of entry for the mooring
blocks, chains and piles the plan adds. ``layers`` are seabed layers (the
small litter). ``groups`` lists the groups; each member also carries its
``group_id``. ``report`` holds the expected and realised counts, the density
and the spatial statistics, and the warnings.

Pure Python and numpy.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

PLAN_FORMAT = "blueboat_placement_plan/1"


def plain(obj: Any) -> Any:
    """A copy made of plain Python values only (what ``yaml.safe_dump``
    accepts): numpy scalars and arrays converted, tuples to lists."""
    if isinstance(obj, dict):
        return {str(k) if not isinstance(k, (str, int, float, bool)) else k: plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [plain(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return [plain(v) for v in obj.tolist()]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def canonical(obj: Any) -> str:
    return json.dumps(plain(obj), sort_keys=True, separators=(",", ":"), default=str)


@dataclass
class PlacementPlan:
    instances: list = field(default_factory=list)
    groups: list = field(default_factory=list)
    structures: list = field(default_factory=list)
    layers: list = field(default_factory=list)
    report: dict = field(default_factory=dict)
    params: dict = field(default_factory=dict)          # the placement block as asked
    model: dict = field(default_factory=dict)           # {package, version, code_digest, evidence_digest}
    environment_digest: str = ""
    seed: int = 0
    #: what the seabed declaration of the world must list (the hard class of
    #: the small-litter layers), or None when the plan adds no such layer
    litter_class: str | None = None

    # --------------------------------------------------------------- digest
    def digest(self) -> str:
        """16 hex digits of what will be placed (instances, structures,
        layers, groups): two plans with the same digest build the same
        world on the same environment."""
        blob = canonical({"instances": self.instances, "structures": self.structures,
                          "layers": self.layers, "groups": self.groups})
        return hashlib.sha256(blob.encode()).hexdigest()[:16]

    # ------------------------------------------------------------- metadata
    def metadata_block(self) -> dict:
        """The per-world ``placement`` block of ``metadata.yaml``. It holds
        no code digest and no path: ``metadata.yaml`` is part of a world's
        content digest, which must not change with a comment in this
        package (the digests are in ``builder_state.yaml``, the manifest and
        the world card)."""
        r = self.report
        blk = {
            "mode": self.params.get("mode", "mission_like"),
            "model": {"package": self.model.get("package", ""), "version": self.model.get("version", "")},
            "plan_digest": self.digest(),
            "environment_digest": self.environment_digest,
            "profile": self.params.get("profile"),
            "density_level": self.params.get("density_level"),
            "lambda_active_per_ha": r.get("density_per_ha"),
            "classes": self.params.get("classes"),
            "mix": self.params.get("mix"),
            "weights": r.get("expected_shares"),
            "seed": int(self.seed),
            "split": self.params.get("split"),
            "clutter": self.params.get("clutter"),
            "structures": self.params.get("structures"),
            "advanced": self.params.get("advanced"),
            "features": r.get("features", []),
            "groups": self.groups,
            "small_litter": r.get("small_litter"),
            "envelope": r.get("envelope"),
            "expected": r.get("expected"),
            "realised": r.get("realised"),
            "warnings": r.get("warnings", []),
            "stress_world": bool(r.get("stress_world", False)),
            "showcase_world": bool(r.get("showcase_world", False)),
            "gaps": r.get("gaps", []),
        }
        for k in ("harbour", "split_override", "feature_tags", "feature_activity"):
            if self.params.get(k):
                blk[k] = self.params[k]
        return plain(blk)

    def state_block(self) -> dict:
        """What ``builder_state.yaml`` keeps (``BuilderState.placement``):
        the block as asked, the model with its digests, and the metadata
        block, so that an Edit re-save writes the same record."""
        return plain({"params": self.params, "model": self.model, "seed": int(self.seed),
                      "plan_digest": self.digest(), "environment_digest": self.environment_digest,
                      "metadata": self.metadata_block(), "litter_class": self.litter_class})

    # ------------------------------------------------------------------ I/O
    def to_dict(self) -> dict:
        return plain({"format": PLAN_FORMAT, "seed": int(self.seed), "model": self.model,
                      "environment_digest": self.environment_digest, "plan_digest": self.digest(),
                      "params": self.params, "report": self.report, "groups": self.groups,
                      "litter_class": self.litter_class, "layers": self.layers,
                      "structures": self.structures, "instances": self.instances})

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.write_text(yaml.safe_dump(self.to_dict(), sort_keys=False, default_flow_style=None),
                        encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: str | Path) -> "PlacementPlan":
        d = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        if d.get("format") != PLAN_FORMAT:
            raise ValueError(f"{path}: not a placement plan ({d.get('format')!r})")
        return cls(instances=d.get("instances") or [], groups=d.get("groups") or [],
                   structures=d.get("structures") or [], layers=d.get("layers") or [],
                   report=d.get("report") or {}, params=d.get("params") or {}, model=d.get("model") or {},
                   environment_digest=str(d.get("environment_digest", "")), seed=int(d.get("seed", 0)),
                   litter_class=d.get("litter_class"))

    # ------------------------------------------------------------ shortcuts
    def xy(self) -> np.ndarray:
        return np.array([(e["x"], e["y"]) for e in self.instances], dtype=np.float64).reshape(-1, 2)

    def counts(self) -> dict:
        out: dict[str, int] = {}
        for e in self.instances:
            out[e["type"]] = out.get(e["type"], 0) + 1
        return out
