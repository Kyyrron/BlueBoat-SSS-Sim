"""The report of a plan: what was asked, what was expected, what came out, and
the warnings. It becomes the ``placement`` block of the world's metadata and
the body of its world card.

Pure Python and numpy.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .data import CLASSES, DEBRIS, TARGETS, Parameters
from .environment import Environment
from .params import PlacementParams
from .sources import FAR_FIELD
from .stats import clark_evans, distance_to_sources, perimeter_m

#: classes the evidence wants and the object set lacks, per source use: what
#: a world card must state (docs/population/POPULATION_MODEL.md, "Gaps")
GAPS_BY_USE = {
    "urban_quay": ["bicycles, e-scooters and trolleys (stood in for by trap_pot)"],
    "outfall": ["wood (logs, poles, planks)"],
    "fishing_ground": ["lobster traps and round crab pots (the class is a square wire pot)"],
    "traffic_lane": ["lobster traps and round crab pots (the class is a square wire pot)"],
    "fishing_berth": ["long nets and lines over 2 m"],
    "working_quay": ["long cables and chains over 3 m"],
    "fender_line": ["truck and tractor tyres over 0.8 m"],
    "dumping_spot": ["truck and tractor tyres over 0.8 m", "boats and dinghies"],
}
GAPS_ALWAYS = ["damaged or collapsed traps (pose only)", "fouling per object (one value per world)"]


def _round(d: dict, n: int = 3) -> dict:
    return {k: (round(float(v), n) if isinstance(v, (float, np.floating)) else v) for k, v in d.items()}


def build_report(env: Environment, allowed: np.ndarray, envelope: dict, srcs: list, src_report: dict,
                 expected_raw: dict, plan, p: Parameters, params: PlacementParams, notes: dict) -> dict:
    ret = float(src_report["retention"])
    lam = float(src_report["density_per_ha"])
    area = env.cell_area_ha
    expected = {c: round(expected_raw.get(c, 0.0) * ret, 3) for c in CLASSES if expected_raw.get(c, 0.0) > 0.0}
    exp_t = 0.0
    for c in TARGETS:
        exp_t += expected.get(c, 0.0)
    exp_all = exp_t + expected.get(DEBRIS, 0.0)
    counts = plan.counts()
    n_t = sum(counts.get(c, 0) for c in TARGETS)
    n_d = counts.get(DEBRIS, 0)
    n = n_t + n_d
    xy = plan.xy()
    # the active band: the cells some source reaches
    band = np.zeros(allowed.size, dtype=bool)
    for s in srcs:
        if s.id != FAR_FIELD:
            band[s.cells] = True
    band_ha = float(np.count_nonzero(band)) * area
    in_band = 0
    if n:
        j, i = env.cell_of(xy[:, 0], xy[:, 1])
        in_band = int(np.count_nonzero(band.reshape(allowed.shape)[j, i]))
    feats = [s.feature for s in srcs if s.feature is not None]
    dist = distance_to_sources(xy, feats) if n else np.zeros(0)
    allowed_m2 = float(np.count_nonzero(allowed)) * area * 1e4
    eb_limit = float(p("burial.effectively_buried"))
    buried = sum(1 for e in plan.instances if e["type"] in TARGETS and e["burial"] > eb_limit)
    detectable = sum(1 for e in plan.instances if e["params"].get("nominal_target") and e["burial"] <= eb_limit)
    outside = sum(1 for e in plan.instances if not e["params"].get("in_envelope", True))
    on_mud = sum(1 for e in plan.instances if e["params"].get("on_mud"))
    stand = {}
    for e in plan.instances:
        s = e["params"].get("stand_in_for")
        if s:
            stand[s] = stand.get(s, 0) + 1
    per_source = {}
    for e in plan.instances:
        fid = e["params"]["source"]["feature"]
        per_source[fid] = per_source.get(fid, 0) + 1
    shares = {}
    if exp_t > 0.0:
        shares = {c: round(expected.get(c, 0.0) / exp_t, 4) for c in TARGETS if expected.get(c, 0.0) > 0.0}
    warnings: list[str] = []
    # ---- realism guard
    guard = float(p("guard.factor"))
    doc = p("guard.documented_per_ha")
    stress = False
    if allowed_m2 > 0.0:
        ha = allowed_m2 / 1e4
        for c, top in sorted(doc.items()):
            dens = (exp_all if c == "all" else expected.get(c, 0.0)) / ha
            if dens > guard * float(top):
                stress = True
                what = "all sonar-size litter" if c == "all" else c
                warnings.append(f"{what}: {dens:.1f} per ha expected over the placement water, more than "
                                f"{guard:g} times the highest documented area mean ({float(top):g} per ha): "
                                "a stress world, not a documented harbour")
    if envelope["share_in_band"] < 0.95 and not params.outside_band:
        warnings.append(f"{100.0 * envelope['share_in_band']:.0f} % of the water lies in the altitude band "
                        f"{envelope['altitude_band_m'][0]:g}-{envelope['altitude_band_m'][1]:g} m "
                        "(the frozen sensor's envelope); objects are placed in that part only")
    if params.outside_band:
        warnings.append("objects may lie outside the altitude band of the frozen sensor (showcase world): "
                        f"{outside} do; results on them are not claimed")
    if notes["clipped"]:
        warnings.append("budget above the dominating density, clipped: " + ", ".join(sorted(notes["clipped"])))
    if not feats:
        warnings.append("the environment has no source feature: every object is far-field")
    if on_mud:
        warnings.append(f"{on_mud} object(s) lie on mud: outside what the validity conditions let claim")
    if stand:
        warnings.append("stand-ins (the object set lacks the real object): "
                        + ", ".join(f"{v} trap_pot for {k}" for k, v in sorted(stand.items())))
    sp = notes.get("split") or {}
    if sp.get("overridden"):
        warnings.append("split guard overridden: " + "; ".join(sp.get("violations", []))
                        + f" (reason: {sp.get('reason')})")
    litter = notes.get("small_litter") or {}
    if litter.get("enabled") and "sand" not in env.sediment_names[:1] and env.sediment_names:
        pass
    gaps = list(GAPS_ALWAYS)
    for s in srcs:
        for g in GAPS_BY_USE.get(s.use, []):
            if g not in gaps:
                gaps.append(g)
    features = []
    for s in srcs:
        rec = {"id": s.id, "kind": s.feature.kind if s.feature else FAR_FIELD, "use": s.use,
               "length_m": round(float(s.feature.length_m), 1) if s.feature else 0.0,
               "activity": round(float(s.activity), 3), "owned_ha": round(float(s.owned_ha), 4),
               "expected": round(float(s.budget) * ret * (1.0 if params.debris else s.target_share), 2),
               "placed": int(per_source.get(s.id, 0))}
        features.append(rec)
    realised: dict[str, Any] = {
        "targets": int(n_t), "debris": int(n_d), "groups": len(plan.groups),
        "grouped_objects": int(sum(len(g["members"]) for g in plan.groups)),
        "structures": len(plan.structures),
        "per_class": {c: int(counts[c]) for c in CLASSES if counts.get(c)},
        "density_active_per_ha": round(in_band / band_ha, 2) if band_ha > 0 else None,
        "density_world_per_ha": round(n / (allowed_m2 / 1e4), 2) if allowed_m2 > 0 else None,
        "share_within_30m": round(float(np.mean(dist <= 30.0)), 3) if n else None,
        "median_distance_to_source_m": round(float(np.median(dist[np.isfinite(dist)])), 2)
        if n and np.isfinite(dist).any() else None,
        "clark_evans": (lambda v: None if v is None else round(v, 3))(
            clark_evans(xy, allowed_m2, perimeter_m(allowed, env.cell_m))),
        "effectively_buried": int(buried), "detectable_targets": int(detectable),
        "outside_band": int(outside), "on_mud": int(on_mud), "stand_ins": stand,
        "removed_by_spacing": int(notes["before_spacing"] - n), "no_room": int(notes["no_room"]),
        "moved_by_redistribution": int(notes.get("moved", 0)),
    }
    return {
        "level": int(params.density_level), "density_per_ha": lam, "retention": ret,
        "active_band_ha": round(band_ha, 4), "allowed_ha": round(allowed_m2 / 1e4, 4),
        "far_field_ha": round(float(src_report.get("far_field_ha", 0.0)), 4),
        "envelope": envelope,
        "expected": {"total": round(exp_all, 2), "targets": round(exp_t, 2),
                     "debris": round(expected.get(DEBRIS, 0.0), 2), "per_class": expected},
        "expected_shares": shares,
        "centred_affinity": src_report.get("centred_affinity"),
        "realised": realised,
        "features": features,
        "small_litter": litter,
        "structures": notes.get("structures"),
        "split": sp,
        "warnings": warnings,
        "stress_world": bool(stress),
        "showcase_world": bool(params.outside_band),
        "gaps": gaps,
    }
