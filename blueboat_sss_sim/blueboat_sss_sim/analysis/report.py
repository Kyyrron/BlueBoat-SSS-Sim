"""Serialise :class:`MissionMetrics` to ``metrics.json`` + ``metrics.md``.

Determinism is a requirement, not a nicety: a number that drifts between runs
cannot support a claim. Keys are sorted, floats are rounded to a fixed number
of decimals before serialisation, and nothing that varies run to run is
recorded -- no timestamp, no host, no wall-clock duration.

Provenance is deliberately *outside* that guarantee. Which bundle directory and
which recording produced a figure has to be recorded, but it is a label, not a
measurement: regenerating a bundle from its seed into a different directory
must not be reported as a changed result. So ``content_digest`` hashes
everything **except** the provenance block, and it is that digest -- not the
whole file -- that the reproducibility claim is made over. How much of the
mission was walked is *not* a label, so it sits in ``coverage``, inside the
digest: two runs that covered different amounts of a survey must not claim the
same result.

Every report leads with what CM-13 requires: these are simulation-derived
figures under a named detection criterion, not an empirical detection result.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Optional

from .metrics import Curve, MissionMetrics

#: Decimals kept for every float written. Well inside the repeatability of the
#: quantities themselves, and fixed so serialisation cannot introduce drift.
FLOAT_DECIMALS = 6

_PREAMBLE = (
    "Simulation-derived figures from renderer ground truth, under the "
    "detection criterion named below. They characterise the simulated "
    "environment and support the policy comparison; they are not an "
    "empirical detection result and are stated as model-conditional."
)


def _r(x):
    """Round floats (recursively) so serialisation is bit-stable."""
    if isinstance(x, float):
        return round(x, FLOAT_DECIMALS)
    if isinstance(x, dict):
        return {k: _r(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_r(v) for v in x]
    return x


def content_digest(doc: dict) -> str:
    """SHA-256 over everything in ``doc`` except provenance and the digest.

    This is the quantity the reproducibility claim is about: run the tool twice
    on one bundle, or regenerate the bundle from its seed into a fresh
    directory, and this must not move.
    """
    body = {k: v for k, v in doc.items()
            if k not in ("provenance", "content_digest")}
    canonical = json.dumps(body, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def to_dict(m: MissionMetrics) -> dict:
    """The full result as plain JSON-able data, deterministically ordered."""
    def curves(d: dict[str, Curve]) -> dict:
        return {k: _r(asdict(v)) for k, v in sorted(d.items())}

    doc = {
        "schema": 1,
        "preamble": _PREAMBLE,
        "provenance": {
            "bundle": Path(m.bundle).name if m.bundle else None,
            "seed": m.seed,
            "source": m.source,
            "detail": m.detail,
        },
        "coverage": {
            # Inside the digest: how much of the mission was walked is part of
            # what was measured, unlike the directory it was read from.
            "ping_cycles": m.ping_cycles,
        },
        "criterion": {
            "name": m.criterion,
            "definition": m.criterion_description,
        },
        "binning": {
            "range_bin_m": _r(m.range_bin_m),
            "aspect_bin_deg": _r(m.aspect_bin_deg),
            "aspect_available": m.aspect_available,
        },
        "reconciliation": {
            "n_objects": m.n_objects,
            "n_observations": m.n_observations,
            "n_ghost_observations": m.n_ghost_observations,
            "ghosts_by_reflector": dict(sorted(m.ghosts_by_reflector.items())),
            "detected": sorted(m.detected),
            "observed_not_detected": sorted(m.observed_not_detected),
            "never_ensonified": sorted(m.never_ensonified),
        },
        "per_class": _r({k: m.per_class[k] for k in sorted(m.per_class)}),
        "range_curves": curves(m.range_curves),
        "aspect_curves": curves(m.aspect_curves),
        "objects": [_r(asdict(o)) for o in
                    sorted(m.objects, key=lambda o: o.object_id)],
        "blocked_metrics": list(m.blocked_metrics),
    }
    doc["content_digest"] = content_digest(doc)
    return doc


def _fmt(p: Optional[float]) -> str:
    return "  --  " if p is None else f"{p:.3f}"


def to_markdown(m: MissionMetrics) -> str:
    """A human-readable companion to the JSON, equally deterministic."""
    L: list[str] = []
    L.append("# Detection metrics from simulated ground truth")
    L.append("")
    L.append(_PREAMBLE)
    L.append("")
    L.append(f"- **Bundle**: `{Path(m.bundle).name if m.bundle else '--'}`  "
             f"(seed {m.seed})")
    L.append(f"- **Source**: {m.source} -- {m.detail}")
    L.append(f"- **Criterion**: `{m.criterion}` = {m.criterion_description}")
    L.append(f"- **Observations**: {m.n_observations} over "
             f"{m.ping_cycles} ping cycles")
    if m.n_ghost_observations:
        by = ", ".join(f"{k} {v}"
                       for k, v in sorted(m.ghosts_by_reflector.items()))
        L.append(f"- **Multipath ghosts**: {m.n_ghost_observations} "
                 f"({by}) — excluded from every rate below; they image an "
                 f"object down a folded path and are the run's "
                 f"false-positive stimulus, not detections of it")
    L.append(f"- **Content digest**: `{content_digest(to_dict(m))[:16]}`  "
             f"(provenance excluded; stable across a regenerated bundle)")
    L.append("")

    L.append("## Manifest reconciliation")
    L.append("")
    L.append("| Outcome | Objects |")
    L.append("|---|---|")
    L.append(f"| Detected | {len(m.detected)} |")
    L.append(f"| Ensonified but below criterion | "
             f"{len(m.observed_not_detected)} |")
    L.append(f"| Never ensonified | {len(m.never_ensonified)} |")
    L.append(f"| **Total in manifest** | **{m.n_objects}** |")
    L.append("")

    L.append("## Per class")
    L.append("")
    L.append("| Class | Objects | Ensonified | Detected | Rate | "
             "Median looks | Median aspects | Aspect-unique | Max range (m) |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for cls in sorted(m.per_class):
        c = m.per_class[cls]
        L.append(
            f"| {cls} | {c['n_objects']} | {c['n_ensonified']} | "
            f"{c['n_detected']} | {_fmt(c['detection_rate'])} | "
            f"{_fmt(c['median_looks'])} | "
            f"{_fmt(c['median_distinct_aspects'])} | "
            f"{c['n_aspect_unique']} | "
            f"{_fmt(c['max_detection_range_m'])} |")
    L.append("")

    def curve_block(title: str, curves: dict[str, Curve], unit: str) -> None:
        L.append(f"## {title}")
        L.append("")
        cur = curves.get("__all__")
        if cur is None:
            L.append("_Unavailable for this source._")
            L.append("")
            return
        L.append(f"All classes pooled. `--` marks a bin with no "
                 f"opportunities (unmeasured, not a failure).")
        L.append("")
        L.append(f"| {unit} | Opportunities | Detections | P(detect) |")
        L.append("|---|---|---|---|")
        for i, p in enumerate(cur.probability):
            if cur.opportunities[i] == 0:
                continue
            lo, hi = cur.bin_edges[i], cur.bin_edges[i + 1]
            L.append(f"| {lo:g}-{hi:g} | {cur.opportunities[i]} | "
                     f"{cur.detections[i]} | {_fmt(p)} |")
        L.append("")

    curve_block("P(detect) vs slant range", m.range_curves, "Range (m)")
    curve_block("P(detect) vs relative aspect", m.aspect_curves,
                "Aspect (deg)")

    L.append("## Not computable here")
    L.append("")
    L.append("These `project_synthesis.md` §8.4 rows need a detector and the "
             "belief layer, which no module implements (root `CLAUDE.md` §6 "
             "D7):")
    L.append("")
    for b in m.blocked_metrics:
        L.append(f"- {b}")
    L.append("")
    return "\n".join(L)


def write_report(m: MissionMetrics, out_dir: str | Path) -> tuple[Path, Path]:
    """Write ``metrics.json`` and ``metrics.md`` into ``out_dir``.

    ``out_dir`` is created if absent. Nothing is ever written into a bundle or
    a recording -- those are write-once (CM-7 / NC #10) -- so the caller passes
    a destination of its own.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    js, md = out / "metrics.json", out / "metrics.md"
    js.write_text(json.dumps(to_dict(m), indent=2, sort_keys=True) + "\n",
                  encoding="utf-8")
    md.write_text(to_markdown(m) + "\n", encoding="utf-8")
    return js, md
