"""Campaign generator: harbour worlds from a specification and seeds, headless
(2026-11-15, completion round session 02).

    worldgen_campaign --spec campaign.yaml --seeds 1,2,3 --out <worlds root>

One world per (case, seed), built with the World Builder's own
``build_world_folder`` (so it is an ordinary world folder: MCS lists it, the
wizard's Edit mode opens it), at ``<root>/<path name>/<campaign>-<case>-
s<seed>/``. Nothing is ever overwritten: an existing (case, seed) is skipped
and reported. The manifest ``<root>/.campaigns/<campaign>/manifest.yaml``
records the specification as read, the generator's code digest and, per
world, every condition value as drawn, the content digest, the depth-domain
share and the object counts.

What a world is made of (every number's source: ``docs/seabed_model.md``
par. 9-11; values tagged [DESIGN] are stated assumptions):

* a basin: a sloping floor in the depth band, quays on 0-3 sides, a pontoon
  with its piles, mooring blocks with their scour hollows and tethers, a
  berth with its propeller scour, a rubble apron at the breakwater's toe;
* a seabed: the reference sediment with a patch field of a finer and a
  coarser class, micro-relief at two scales, pits and mounds, drag marks
  (the R2 heterogeneity presets, fitted on the FIT recording), small litter
  and cobbles (the clutter presets);
* litter: the object set version 2 from ``worldgen.population``.

Random streams: terrain ``[seed, 0]``, objects ``[seed, 1]``, layers
``[seed, 2, layer]`` (the engine's own), population ``[seed, 3]``, basin
``[seed, 4]``. The same specification and seed give the same content digest.

ROS-free, Qt-free.
"""

from __future__ import annotations

import argparse
import copy
import platform
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from . import domain as dom
from . import population as pop

CAMPAIGN_FORMAT = "blueboat_campaign/1"
GENERATOR_VERSION = 1
#: largest side of a campaign world [m]
MAX_SIDE_M = 150.0
#: a quay stands this far inside the world limits [m]
QUAY_INSET_M = 3.0

#: R2: the seabed heterogeneity presets. ``low`` / ``high`` reproduce the
#: smallest / largest statistics of the FIT recording's object-free
#: stretches, ``nominal`` their median (session 02's fit; docs/seabed_model.md
#: par. 9). ``none``: a uniform seabed (regression worlds). Keys: micro-relief
#: RMS height and correlation length at a fine and a broad scale [m]; shares
#: and patch length of the finer and the coarser class; pits (cover) and large
#: mounds (per m2); drag marks per hectare.
HETEROGENEITY: dict[str, dict[str, float]] = {
    "none":    dict(fine_rms=0.0, fine_len=0.4, broad_rms=0.0, broad_len=3.0, share_finer=0.0,
                    share_coarser=0.0, patch_len=2.0, pit_cover=0.0, mounds=0.0, drag=0.0),
    "low":     dict(fine_rms=0.012, fine_len=0.4, broad_rms=0.04, broad_len=3.0, share_finer=0.0,
                    share_coarser=0.0, patch_len=2.0, pit_cover=0.0, mounds=0.0, drag=0.0),
    "nominal": dict(fine_rms=0.012, fine_len=0.4, broad_rms=0.075, broad_len=3.0, share_finer=0.01,
                    share_coarser=0.005, patch_len=2.0, pit_cover=0.005, mounds=0.01, drag=4.0),
    # high: fine relief 12 mm since 2026-11-15 (was 20 mm; owner: bring the lag-3 correlation towards the real
    # maximum by the fine relief alone). 12 mm is the lowest it may take (the presets do not fall from nominal
    # to high); lag-3 0.196 there against the real 0.146 (batchB/r2_high_check.jsonl): the rest of the excess
    # is the broad relief's and the patches'
    "high":    dict(fine_rms=0.012, fine_len=0.4, broad_rms=0.20, broad_len=3.0, share_finer=0.08,
                    share_coarser=0.03, patch_len=3.0, pit_cover=0.013, mounds=0.05, drag=12.0),
}
#: clutter presets: hard small litter and cobbles per m2 (Schernewski et al.
#: Table 1: 0.02-3.71 items per m2 in 14 harbours, one half taken as hard and
#: at least 0.1 m long [DESIGN]; cobbles [DESIGN])
CLUTTER: dict[str, dict[str, float]] = {
    "none": dict(litter=0.0, cobbles=0.0),
    "low": dict(litter=0.01, cobbles=0.005),
    "nominal": dict(litter=0.06, cobbles=0.02),
    "high": dict(litter=0.5, cobbles=0.1),
}
#: the nominal presets by name (what ``config/parameter_registry.yaml`` cites)
HETEROGENEITY_NOMINAL = HETEROGENEITY["nominal"]
CLUTTER_NOMINAL = CLUTTER["nominal"]
DEFAULTS: dict[str, Any] = {
    "depth_band_m": [6.0, 8.0],
    "grade_max": 0.008,
    "basin": {"quays": 2, "pontoon": True, "mooring_blocks": 1},
    "sediment": {"reference": "sand", "finer": "soft_mud", "coarser": "gravel", "heterogeneity": "nominal"},
    "clutter": "nominal",
    "objects": {"set": "v2", "composition": "realistic", "density_per_ha": 10, "target_share": 0.5,
                "burial": "nominal", "fouling": [0.2, 1.0],
                "hotspots": {"decay_m": 15, "share_within_30m": 0.7}, "min_target_spacing_m": 3.0},
    "face_geometry": "new",
    "uncertain_materials": "nominal",
}
_TOP_KEYS = {"format", "name", "path", "world", "defaults", "run", "cases", "validation_instances"}
_SIDES = ("south", "east", "north", "west")


# ------------------------------------------------------------ specification
def _merge(base: Mapping[str, Any], over: Mapping[str, Any], where: str) -> dict:
    out = copy.deepcopy(dict(base))
    for k, v in over.items():
        if k not in base:
            raise KeyError(f"{where}: unknown key {k!r} (known: {sorted(base)})")
        if isinstance(base[k], Mapping) and k not in ("objects",):
            if not isinstance(v, Mapping):
                raise ValueError(f"{where}.{k}: a mapping is expected")
            out[k] = _merge(base[k], v, f"{where}.{k}")
        elif k == "objects":
            out[k] = {**base[k], **dict(v)}
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_spec(path: str | Path) -> dict:
    """The campaign specification, validated: unknown keys raise by name."""
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    # a trajectory file named relative to the specification lies beside it
    rel = Path(str(doc.get("path", "")))
    if rel.suffix == ".yaml" and not rel.is_absolute() and (Path(path).resolve().parent / rel).is_file():
        doc["path"] = str((Path(path).resolve().parent / rel).resolve())
    return check_spec(doc)


def check_spec(doc: Mapping[str, Any]) -> dict:
    doc = copy.deepcopy(dict(doc))
    if doc.get("format") != CAMPAIGN_FORMAT:
        raise ValueError(f"campaign format {doc.get('format')!r} (expected {CAMPAIGN_FORMAT})")
    bad = set(doc) - _TOP_KEYS
    if bad:
        raise KeyError(f"campaign: unknown key(s) {sorted(bad)} (known: {sorted(_TOP_KEYS)})")
    if not doc.get("name") or not doc.get("path"):
        raise ValueError("campaign: 'name' and 'path' are required")
    w = dict(doc.get("world") or {})
    bad = set(w) - {"margin_m", "resolution_m"}
    if bad:
        raise KeyError(f"campaign.world: unknown key(s) {sorted(bad)}")
    doc["world"] = {"margin_m": float(w.get("margin_m", 25.0)),
                    "resolution_m": float(w.get("resolution_m", 0.10))}
    doc["defaults"] = _merge(DEFAULTS, doc.get("defaults") or {}, "campaign.defaults")
    cases = list(doc.get("cases") or [{"name": "nominal"}])
    names = [str(c.get("name", "")) for c in cases]
    if not all(names) or len(set(names)) != len(names):
        raise ValueError(f"campaign.cases: every case needs its own name, got {names}")
    doc["cases"] = cases
    for c in cases:
        conditions(doc, str(c["name"]))                 # raises on a bad case
    doc["validation_instances"] = list(doc.get("validation_instances") or [])
    doc["run"] = dict(doc.get("run") or {})
    return doc


def conditions(spec: Mapping[str, Any], case: str) -> dict:
    """The condition variables of one case: the defaults with its overrides."""
    c = next((c for c in spec["cases"] if str(c.get("name")) == case), None)
    if c is None:
        raise KeyError(f"campaign: no case {case!r} (cases: {[x.get('name') for x in spec['cases']]})")
    cond = _merge(spec["defaults"], {k: v for k, v in c.items() if k != "name"}, f"campaign.cases[{case}]")
    lo, hi = (float(v) for v in cond["depth_band_m"])
    if not 0.5 <= lo <= hi:
        raise ValueError(f"case {case}: depth_band_m {cond['depth_band_m']}")
    b = cond["basin"]
    if not 0 <= int(b["quays"]) <= 3 or int(b["mooring_blocks"]) < 0:
        raise ValueError(f"case {case}: basin.quays 0..3, mooring_blocks >= 0")
    s = cond["sediment"]
    if s["reference"] != "sand":
        raise ValueError(f"case {case}: sediment.reference {s['reference']!r}: the harbour sensor is "
                         "calibrated on 'sand' only")
    if s["heterogeneity"] not in HETEROGENEITY:
        raise ValueError(f"case {case}: sediment.heterogeneity {s['heterogeneity']!r}: {sorted(HETEROGENEITY)}")
    if cond["clutter"] not in CLUTTER:
        raise ValueError(f"case {case}: clutter {cond['clutter']!r}: {sorted(CLUTTER)}")
    from .materials import MaterialLibrary
    seabed = MaterialLibrary().seabed_names
    for k in ("finer", "coarser"):
        if s[k] not in seabed:
            raise ValueError(f"case {case}: sediment.{k} {s[k]!r}: one of {seabed}")
    population_spec(cond)
    return cond


def population_spec(cond: Mapping[str, Any]) -> pop.PopulationSpec:
    o = dict(cond["objects"])
    o.setdefault("uncertain_materials", cond["uncertain_materials"])
    o.setdefault("face_geometry", cond["face_geometry"])
    return pop.PopulationSpec.from_dict(o)


def resolve_path(name: str) -> Path:
    """An MCS trajectory: a file, or a name in the Pattern Designer's folder."""
    p = Path(str(name)).expanduser()
    if p.suffix == ".yaml" and p.is_file():
        return p
    from ..builder.paths import DEFAULT_TRAJECTORIES_DIR
    q = DEFAULT_TRAJECTORIES_DIR / f"{name}.yaml"
    if not q.is_file():
        raise FileNotFoundError(f"campaign.path {name!r}: neither a file nor {q}")
    return q


# -------------------------------------------------------------------- basin
def _wall(name: str, a, b, kind: str = "quay", **kw) -> dict:
    w = {"name": name, "x0": float(a[0]), "y0": float(a[1]), "x1": float(b[0]), "y1": float(b[1]),
         "top_z": 1.0, "reflectivity": 0.65, "thickness": 0.30, "kind": kind, "draft_m": 0.4,
         "roughness_m": 0.003, "material": "concrete"}
    w.update(kw)
    return w


def _side_line(r: Mapping[str, float], side: str, inset: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(a, b, inward unit normal) of a wall along ``side`` of the rectangle,
    ``inset`` inside it and reaching the limits at both ends."""
    x0, y0, x1, y1 = r["x_min"], r["y_min"], r["x_max"], r["y_max"]
    if side == "south":
        return np.array([x0, y0 + inset]), np.array([x1, y0 + inset]), np.array([0.0, 1.0])
    if side == "north":
        return np.array([x0, y1 - inset]), np.array([x1, y1 - inset]), np.array([0.0, -1.0])
    if side == "west":
        return np.array([x0 + inset, y0]), np.array([x0 + inset, y1]), np.array([1.0, 0.0])
    return np.array([x1 - inset, y0]), np.array([x1 - inset, y1]), np.array([-1.0, 0.0])


def plan_basin(rect: Mapping[str, float], path_xy: np.ndarray, cond: Mapping[str, Any],
               margin: float, rng: np.random.Generator) -> dict:
    """The basin of one world in the design frame: walls, the layers its
    structures cut into the floor, mooring blocks, hotspot features, and the
    values drawn. Deterministic in ``rng``."""
    b = cond["basin"]
    d0 = float(rng.uniform(*cond["depth_band_m"]))
    grade = float(rng.uniform(0.0, float(cond["grade_max"])))
    direction = float(rng.uniform(-180.0, 180.0))
    sides = [_SIDES[i] for i in rng.permutation(4)[:int(b["quays"])]]
    walls, layers, feats, drawn = [], [], [], {}
    lines = {}
    breakwater = sides[-1] if len(sides) >= 2 else None
    for s in sides:
        a, e, n = _side_line(rect, s, QUAY_INSET_M)
        lines[s] = (a, e, n)
        walls.append(_wall(f"quay_{s}", a, e))
        feats.append({"kind": "breakwater" if s == breakwater else "quay",
                      "points": [a.tolist(), e.tolist()], "name": f"quay_{s}"})
    px0, py0 = path_xy.min(axis=0)
    px1, py1 = path_xy.max(axis=0)
    if sides:
        # the fishing berth: a stretch of the first quay, with its propeller scour
        a, e, n = lines[sides[0]]
        L = float(np.linalg.norm(e - a))
        lb = float(min(rng.uniform(20.0, 40.0), max(L - 10.0, 5.0)))
        t0 = float(rng.uniform(5.0, max(L - lb - 5.0, 5.0)))
        u = (e - a) / L
        b0, b1 = a + t0 * u, a + (t0 + lb) * u
        feats.append({"kind": "berth", "points": [b0.tolist(), b1.tolist()], "name": "berth"})
        c = 0.5 * (b0 + b1) + 4.0 * n
        scour = dict(depth_m=round(float(rng.uniform(0.3, 0.8)), 2), rx_m=round(float(rng.uniform(2.0, 5.0)), 2),
                     ry_m=round(float(rng.uniform(1.5, 3.0)), 2))
        layers.append({"id": "berth_scour", "kind": "hole", "name": "Propeller scour at the berth",
                       "geometry": {"type": "ellipse", "centre": c.tolist(), "rx_m": scour["rx_m"],
                                    "ry_m": scour["ry_m"],
                                    "angle_deg": float(np.degrees(np.arctan2(u[1], u[0])))},
                       "params": {"depth_m": scour["depth_m"], "profile": "gaussian"}})
        drawn["berth"] = {"quay": sides[0], "from_m": round(t0, 1), "length_m": round(lb, 1), "scour": scour}
        if rng.random() < 0.5:
            a2, e2, _ = lines[sides[int(rng.integers(len(sides)))]]
            o = a2 + float(rng.uniform(0.15, 0.85)) * (e2 - a2)
            feats.append({"kind": "outlet", "points": [o.tolist()], "name": "outlet"})
            drawn["outlet"] = [round(float(o[0]), 1), round(float(o[1]), 1)]
    if breakwater is not None:
        # rubble apron at the breakwater's toe
        a, e, n = lines[breakwater]
        wd = float(rng.uniform(2.0, 4.0))
        toe = [(a + (0.15 + 0.5 * wd) * n).tolist(), (e + (0.15 + 0.5 * wd) * n).tolist()]
        layers.append({"id": "riprap_class", "kind": "material_band", "name": "Rubble apron (class)",
                       "geometry": {"type": "polyline", "points": toe},
                       "params": {"material": cond["sediment"]["coarser"], "width_m": round(wd, 2),
                                  "feather_m": 0.5, "side": "both"}})
        layers.append({"id": "riprap_blocks", "kind": "cobbles", "name": "Rubble apron (blocks)",
                       "mask": [{"kind": "polyline_band", "points": toe, "width_m": round(wd, 2)}],
                       "params": {"per_m2": 1.5, "length_min_m": 0.2, "length_max_m": 0.5,
                                  "height_min_m": 0.15, "height_max_m": 0.4,
                                  "material": cond["sediment"]["coarser"]}})
        drawn["riprap"] = {"quay": breakwater, "width_m": round(wd, 2)}
    pontoon = None
    if b["pontoon"]:
        s = sides[0] if sides else _SIDES[int(rng.integers(4))]
        a, e, n = lines.get(s) or _side_line(rect, s, 0.0)
        root = a + float(rng.uniform(0.2, 0.8)) * (e - a)
        lp = float(rng.uniform(10.0, max(margin - QUAY_INSET_M - 5.0, 12.0)))
        wp = 2.5
        u = (e - a) / np.linalg.norm(e - a)
        c = [root - 0.5 * wp * u + 0.3 * n, root + 0.5 * wp * u + 0.3 * n,
             root + 0.5 * wp * u + lp * n, root - 0.5 * wp * u + lp * n]
        draft = round(float(rng.uniform(0.4, 0.6)), 2)
        for i, nm in enumerate(("a", "b", "c", "d")):
            walls.append(_wall(f"pontoon_{nm}", c[i], c[(i + 1) % 4], kind="pontoon", draft_m=draft, top_z=0.6))
        feats.append({"kind": "pontoon", "points": [(root + 0.3 * n).tolist(), (root + lp * n).tolist()],
                      "name": "pontoon"})
        mat = str(rng.choice(["concrete", "steel_rough", "wood_rough"]))
        dia = round(float(rng.uniform(0.3, 0.5)), 2)
        piles = [(c[2] + 0.5 * dia * u).tolist(), (c[3] - 0.5 * dia * u).tolist(),
                 (root + 0.5 * (wp + dia) * u + 0.5 * lp * n).tolist(),
                 (root - 0.5 * (wp + dia) * u + 0.5 * lp * n).tolist()]
        pontoon = {"side": s, "length_m": round(lp, 1), "width_m": wp, "draft_m": draft,
                   "corners": [p.tolist() for p in c], "piles": piles, "pile_material": mat,
                   "pile_diameter_m": dia, "end": (root + lp * n).tolist()}
        drawn["pontoon"] = {k: pontoon[k] for k in ("side", "length_m", "width_m", "draft_m", "pile_material",
                                                   "pile_diameter_m")}
    # mooring blocks: in the ring between the survey lines and the quays
    blocks = []
    lo = np.array([rect["x_min"] + QUAY_INSET_M + 8.0, rect["y_min"] + QUAY_INSET_M + 8.0])
    hi = np.array([rect["x_max"] - QUAY_INSET_M - 8.0, rect["y_max"] - QUAY_INSET_M - 8.0])
    for k in range(int(b["mooring_blocks"])):
        for _try in range(300):
            x, y = rng.uniform(lo, hi)
            if px0 - 6.0 < x < px1 + 6.0 and py0 - 6.0 < y < py1 + 6.0:
                continue
            if any(np.hypot(x - q["x"], y - q["y"]) < 12.0 for q in blocks):
                continue
            if pontoon is not None and min(np.hypot(x - p[0], y - p[1]) for p in pontoon["corners"]) < 6.0:
                continue
            break
        else:
            continue
        size = round(float(rng.uniform(2.0, 4.0)), 2)
        h = round(float(min(rng.uniform(1.0, 2.8), d0 - 3.0)), 2)
        blk = {"name": f"block_{k + 1}", "x": float(x), "y": float(y), "size_m": size, "height_m": h,
               "yaw_deg": round(float(rng.uniform(0.0, 90.0)), 1),
               "hollow_radius_m": round(float(rng.uniform(3.0, 10.0)), 2),
               "hollow_depth_m": round(float(rng.uniform(0.1, 0.4)), 2),
               "riser_length_m": round(float(rng.uniform(0.5, 2.0)), 2),
               "riser_yaw_deg": round(float(rng.uniform(0.0, 360.0)), 1),
               "bar_mm": round(float(rng.uniform(14.0, 26.0)), 1),
               "line_sag_m": round(float(rng.uniform(4.0, 6.0)), 2)}
        blocks.append(blk)
        layers.append({"id": f"hollow_{blk['name']}", "kind": "hole", "name": f"Chain scar round {blk['name']}",
                       "geometry": {"type": "ellipse", "centre": [blk["x"], blk["y"]],
                                    "rx_m": blk["hollow_radius_m"], "ry_m": blk["hollow_radius_m"],
                                    "angle_deg": 0.0},
                       "params": {"depth_m": blk["hollow_depth_m"], "profile": "gaussian"}})
        feats.append({"kind": "mooring", "points": [[blk["x"], blk["y"]]], "name": blk["name"]})
    drawn.update(depth_m=round(d0, 3), grade=round(grade, 5), grade_direction_deg=round(direction, 1),
                 quays=sides, breakwater=breakwater,
                 mooring_blocks=[{k: (round(v, 2) if isinstance(v, float) else v) for k, v in q.items()}
                                 for q in blocks])
    return {"walls": walls, "layers": layers, "features": feats, "blocks": blocks, "pontoon": pontoon,
            "depth": {"depth_centre": round(d0, 3), "grade": round(grade, 5),
                      "direction_deg": round(direction, 1), "min_depth": 0.3},
            "drawn": drawn}


def seabed_layers(cond: Mapping[str, Any]) -> tuple[list, dict]:
    """The seabed's layer stack for a case's heterogeneity and clutter
    presets, and the parameters used."""
    s = cond["sediment"]
    q = dict(HETEROGENEITY[s["heterogeneity"]])
    c = dict(CLUTTER[cond["clutter"]])
    L: list = []
    if q["share_finer"] > 0 or q["share_coarser"] > 0:
        L.append({"id": "sediment_field", "kind": "sediment_field",
                  "params": {"corr_len_m": q["patch_len"], "finer": s["finer"], "share_finer": q["share_finer"],
                             "coarser": s["coarser"], "share_coarser": q["share_coarser"]}})
    if q["fine_rms"] > 0:
        L.append({"id": "relief_fine", "kind": "microrelief",
                  "params": {"rms_m": q["fine_rms"], "corr_len_m": q["fine_len"]}})
    if q["broad_rms"] > 0:
        L.append({"id": "relief_broad", "kind": "microrelief",
                  "params": {"rms_m": q["broad_rms"], "corr_len_m": q["broad_len"]}})
    if q["pit_cover"] > 0 or q["mounds"] > 0:
        L.append({"id": "bioturbation", "kind": "bioturbation",
                  "params": {"pit_cover": q["pit_cover"], "mounds_per_m2": q["mounds"]}})
    if q["drag"] > 0:
        L.append({"id": "drag_marks", "kind": "drag_marks", "params": {"per_hectare": q["drag"]}})
    if c["cobbles"] > 0:
        L.append({"id": "cobbles", "kind": "cobbles", "params": {"per_m2": c["cobbles"], "material": s["coarser"]}})
    if c["litter"] > 0:
        L.append({"id": "small_litter", "kind": "small_litter",
                  "params": {"per_m2": c["litter"], "material": s["coarser"]}})
    return L, {"heterogeneity": {"preset": s["heterogeneity"], **q}, "clutter": {"preset": cond["clutter"], **c}}


# -------------------------------------------------------------------- world
def _structures(plan: dict, depth_at, lib) -> tuple[list, list]:
    """(explicit entries in the design frame, records) of the basin's
    structures: mooring blocks with riser and line, pontoon piles."""
    out, rec = [], []
    refl = lambda m: float(lib.get(m).reflectivity)                      # noqa: E731

    def chain(name, x, y, yaw, length, bar, **p):
        w = round(3.4 * bar * 1e-3, 3)
        return {"type": "chain", "x": float(x), "y": float(y), "yaw": float(yaw), "length": float(length),
                "width": w, "proud_height": w, "burial": 0.0, "fill_fraction": 1.0,
                "reflectivity": refl("steel_rough"), "material": "steel_rough",
                "params": {"spec": 2, "role": "structure", "name": name, "link_m": round(6.0 * bar * 1e-3, 3),
                           "bar_mm": bar, **p}}
    for blk in plan["blocks"]:
        out.append({"type": "mooring_block", "x": blk["x"], "y": blk["y"],
                    "yaw": float(np.radians(blk["yaw_deg"])), "length": blk["size_m"], "width": blk["size_m"],
                    "proud_height": blk["height_m"], "burial": 0.0, "fill_fraction": 1.0,
                    "reflectivity": refl("concrete"), "material": "concrete",
                    "params": {"spec": 2, "role": "structure", "name": blk["name"]}})
        d = float(depth_at(blk["x"], blk["y"]))
        yaw = np.radians(blk["riser_yaw_deg"])
        lr = blk["riser_length_m"]
        out.append(chain(f"{blk['name']}_riser", blk["x"] + 0.5 * lr * np.cos(yaw), blk["y"] + 0.5 * lr * np.sin(yaw),
                         yaw, lr, blk["bar_mm"], z0_m=blk["height_m"],
                         z1_m=round(max(d - 0.3, blk["height_m"] + 0.5), 2), sag_m=0.0))
        rec.append({"kind": "mooring_block", "name": blk["name"], "x": blk["x"], "y": blk["y"],
                    "size_m": blk["size_m"], "height_m": blk["height_m"], "yaw_deg": blk["yaw_deg"],
                    "tethers": [f"{blk['name']}_riser"]})
        pt = plan["pontoon"]
        if pt is not None:
            e = np.array(pt["end"])
            v = e - (blk["x"], blk["y"])
            dist = float(np.linalg.norm(v))
            if 4.0 < dist < 45.0:
                mid = 0.5 * (e + (blk["x"], blk["y"]))
                dm = float(depth_at(mid[0], mid[1]))
                out.append(chain(f"{blk['name']}_line", mid[0], mid[1], float(np.arctan2(v[1], v[0])), dist,
                                 blk["bar_mm"], z0_m=blk["height_m"], z1_m=round(max(dm - pt["draft_m"], 1.0), 2),
                                 sag_m=blk["line_sag_m"]))
                rec[-1]["tethers"].append(f"{blk['name']}_line")
    pt = plan["pontoon"]
    if pt is not None:
        for k, (x, y) in enumerate(pt["piles"]):
            d = float(depth_at(x, y))
            out.append({"type": "mooring_pile", "x": float(x), "y": float(y), "yaw": 0.0,
                        "length": pt["pile_diameter_m"], "width": pt["pile_diameter_m"],
                        "proud_height": round(d + 0.8, 2), "burial": 0.0, "fill_fraction": 1.0,
                        "reflectivity": refl(pt["pile_material"]), "material": pt["pile_material"],
                        "params": {"spec": 2, "role": "structure", "name": f"pontoon_pile_{k + 1}"}})
        rec.append({"kind": "pontoon", "name": "pontoon", "length_m": pt["length_m"], "width_m": pt["width_m"],
                    "draft_m": pt["draft_m"], "piles": len(pt["piles"]), "pile_material": pt["pile_material"]})
    return out, rec


def plan_world(spec: Mapping[str, Any], case: str, seed: int):
    """``(BuilderState, extras)`` of one world, ready for
    ``build_world_folder``: the basin, the seabed, the structures and the
    pinned litter; ``extras`` is what ``metadata.yaml`` records about how it
    was drawn."""
    from ..builder.state import BuilderState, DepthState, ObjectsState, PathState
    from ..core.types import Wall
    from ..core.geometry import expand_walls
    from .depth.bake import bake_composite
    from .materials import MaterialLibrary
    from .terrain import texture_defaults
    from .world_folder import _resolution_for, build_grid, world_block
    cond = conditions(spec, case)
    seed = int(seed)
    path = PathState.from_mcs_file(resolve_path(spec["path"]))
    xy = path.xy()
    m = float(spec["world"]["margin_m"])
    rect = {"x_min": float(xy[:, 0].min() - m), "y_min": float(xy[:, 1].min() - m),
            "x_max": float(xy[:, 0].max() + m), "y_max": float(xy[:, 1].max() + m)}
    sx, sy = rect["x_max"] - rect["x_min"], rect["y_max"] - rect["y_min"]
    if max(sx, sy) > MAX_SIDE_M:
        raise ValueError(f"campaign world {sx:.0f} x {sy:.0f} m: at most {MAX_SIDE_M:.0f} m a side "
                         "(a shorter path or a smaller world.margin_m)")
    if abs(_resolution_for(sx, sy) - float(spec["world"]["resolution_m"])) > 1e-9:
        raise ValueError(f"campaign.world.resolution_m {spec['world']['resolution_m']}: a world of "
                         f"{sx:.0f} x {sy:.0f} m is built at {_resolution_for(sx, sy)} m")
    rng_b = np.random.default_rng([seed, 4])
    plan = plan_basin(rect, xy, cond, m, rng_b)
    seabed, seabed_rec = seabed_layers(cond)
    fouling = float(rng_b.uniform(*[float(v) for v in cond["objects"]["fouling"]]))
    tex = texture_defaults()
    tex["roughness"]["amplitude"] = 0.0             # the relief is the heterogeneity preset's, nothing else
    tex["materials"]["uniform"] = cond["sediment"]["reference"]
    patch = sorted({cond["sediment"]["finer"], cond["sediment"]["coarser"]} - {cond["sediment"]["reference"]})
    state = BuilderState(
        path=path, limits_rect=rect, walls=plan["walls"],
        depth=DepthState(mode="preset", preset="sloped_plane", params=plan["depth"], texture=tex,
                         material=cond["sediment"]["reference"], layers=plan["layers"] + seabed,
                         seabed={"reference": cond["sediment"]["reference"], "patch_classes": patch,
                                 "heterogeneity": cond["sediment"]["heterogeneity"]}),
        objects=ObjectsState(seed=seed, classes={}, pinned=True),
        world_name=f"{spec['name']}-{case}-s{seed}")
    state.validate_for_build()
    # first pass: the floor as the build will compose it (world frame), for the
    # structures' heights, the depth domain and where litter may fall
    _, _, _, ox, oy = world_block(state)
    grid = build_grid(state)
    walls_w = []
    for w in expand_walls(state.walls):
        w = dict(w)
        for k, d in (("x0", ox), ("y0", oy), ("x1", ox), ("y1", oy)):
            w[k] = float(w[k]) - d
        walls_w.append(Wall(**w))
    comp = bake_composite(state.depth, grid, walls_w, state.world_anchor(), seed, offset=(ox, oy))
    height = comp.height
    xmin, ymin, _, _ = grid.extent
    res = float(grid.resolution)

    def cell(x, y):
        return (int(np.clip((y - ymin) / res, 0, grid.ny - 1)), int(np.clip((x - xmin) / res, 0, grid.nx - 1)))

    def depth_at(xd, yd):                              # design frame in, water depth out
        return -float(height[cell(xd - ox, yd - oy)])
    lib = MaterialLibrary()
    structures, struct_rec = _structures(plan, depth_at, lib)
    report, rasters = dom.domain_report(height, grid, walls_w)
    allowed = rasters["in_domain"] & ~dom.wall_mask(grid, walls_w, ("quay",), margin_m=1.5)
    allowed[:int(2.0 / res)] = allowed[-int(2.0 / res):] = False
    allowed[:, :int(2.0 / res)] = allowed[:, -int(2.0 / res):] = False
    X = xmin + (np.arange(grid.nx) + 0.5) * res
    Y = ymin + (np.arange(grid.ny) + 0.5) * res
    for e in structures:
        if e["type"] == "chain":
            continue
        r = 0.5 * float(np.hypot(e["length"], e["width"])) + 1.0
        j0, i0 = cell(e["x"] - ox - r, e["y"] - oy - r)
        j1, i1 = cell(e["x"] - ox + r, e["y"] - oy + r)
        sub = np.hypot(X[None, i0:i1 + 1] - (e["x"] - ox), Y[j0:j1 + 1, None] - (e["y"] - oy)) <= r
        allowed[j0:j1 + 1, i0:i1 + 1] &= ~sub
    names = list(comp.material_names) if comp.material_id is not None else []
    finer = cond["sediment"]["finer"]

    def sediment_at(x, y):
        if comp.material_id is None:
            return cond["sediment"]["reference"]
        k = int(comp.material_id[cell(x, y)])
        return names[k] if k >= 0 else cond["sediment"]["reference"]
    feats = [pop.Feature(f["kind"], np.asarray(f["points"], float) - (ox, oy), f["name"])
             for f in plan["features"]]
    rng_p = np.random.default_rng([seed, 3])
    # validation instances first (their place is drawn on the population stream)
    occupied, validation = [], []
    wet = np.argwhere(allowed)
    for k, v in enumerate(spec.get("validation_instances") or []):
        if (v.get("class"), v.get("variant")) != ("plastic_box", "nafco_68l"):
            raise ValueError(f"validation_instances[{k}]: only {{class: plastic_box, variant: nafco_68l}}")
        if not len(wet):
            break
        j, i = wet[int(rng_p.integers(len(wet)))]
        x, y = float(X[i]), float(Y[j])
        validation.append({"type": "plastic_box", "x": x + ox, "y": y + oy,
                           "yaw": float(rng_p.uniform(0.0, 2.0 * np.pi)), "length": 0.716, "width": 0.425,
                           "proud_height": 0.323, "burial": 0.0, "fill_fraction": 1.0,
                           "reflectivity": float(lib.get("pp_shell").reflectivity), "material": "pp_shell",
                           "params": {"spec": 2, "variant": "nafco_68l", "wall_mm": 2.0, "state": "flooded",
                                      "role": "validation", "nominal_target": False, "pose": "upright"}})
        occupied.append((x, y, 0.5))
    pspec = population_spec(cond)
    litter, prep = pop.populate(grid, allowed, feats, pspec, rng_p,
                                is_mud=lambda x, y: sediment_at(x, y) == finer, sediment_at=sediment_at,
                                materials=lib, occupied=occupied)
    for e in litter:
        e["x"], e["y"] = round(e["x"] + ox, 3), round(e["y"] + oy, 3)
    state.objects.explicit = structures + validation
    state.objects.placed = litter
    extras = {
        "campaign": {"name": str(spec["name"]), "case": case, "seed": seed,
                     "generator_version": GENERATOR_VERSION},
        "conditions": {**{k: copy.deepcopy(v) for k, v in cond.items()}, "drawn": plan["drawn"], **seabed_rec},
        "run": {**dict(spec.get("run") or {}), "sonar_fouling": round(fouling, 3)},
        "population": {"density_per_ha": pspec.density_per_ha, "target_share": pspec.target_share,
                       "composition": cond["objects"]["composition"], "burial": pspec.burial, **prep},
        "structures": struct_rec + [{"kind": f["kind"], "name": f["name"],
                                     "points": np.round(np.asarray(f["points"], float) - (ox, oy), 2).tolist()}
                                    for f in plan["features"]],
        "uncertain_materials": pspec.uncertain_materials,
        "face_geometry": cond["face_geometry"],
    }
    return state, extras


def build_world(spec: Mapping[str, Any], case: str, seed: int, root: str | Path,
                progress=None) -> dict:
    """Build one (case, seed) under ``root``; the manifest record. An
    existing folder is never touched: ``{"skipped": True}``."""
    from ..core import provenance
    from .world_folder import _safe_name, build_world_folder
    root = Path(root).expanduser()
    t0 = time.time()
    path_name = _safe_name(Path(resolve_path(spec["path"])).stem)
    name = _safe_name(f"{spec['name']}-{case}-s{int(seed)}")
    final = root / path_name / name
    rec: dict[str, Any] = {"case": case, "seed": int(seed), "world": f"{path_name}/{name}"}
    if final.exists():
        rec["skipped"] = True
        if (final / "metadata.yaml").is_file():
            rec["content_digest"] = (yaml.safe_load((final / "metadata.yaml").read_text()) or {}
                                     ).get("content_digest")
        return rec
    state, extras = plan_world(spec, case, seed)
    out = Path(build_world_folder(state, root, progress=progress, extras=extras))
    if out.name != name:
        raise RuntimeError(f"built {out}, expected the folder name {name}")
    meta = yaml.safe_load((out / "metadata.yaml").read_text())
    rec.update(
        skipped=False, content_digest=meta["content_digest"],
        file_digest=provenance.world_digest(out)["sha256"],
        domain={k: meta["domain"][k] for k in ("share_in_domain", "share_too_shallow", "share_too_deep",
                                               "in_domain", "survey_area_m2")},
        out_of_domain=not bool(meta["domain"]["in_domain"]),
        object_counts=meta["object_counts"], detectable_targets=meta["detectable_targets"],
        conditions=extras["conditions"], sonar_fouling=extras["run"]["sonar_fouling"],
        population=extras["population"], build_seconds=round(time.time() - t0, 1))
    return rec


def run_campaign(spec_path: str | Path, seeds, root: str | Path, cases=None, progress=None) -> dict:
    """Every (case, seed) of the specification; the manifest (also written
    to ``<root>/.campaigns/<name>/manifest.yaml``, earlier records kept)."""
    from ..core import provenance
    spec = load_spec(spec_path)
    root = Path(root).expanduser()
    mdir = root / ".campaigns" / str(spec["name"])
    mdir.mkdir(parents=True, exist_ok=True)
    mfile = mdir / "manifest.yaml"
    manifest = (yaml.safe_load(mfile.read_text()) or {}) if mfile.is_file() else {}
    worlds = {(w["case"], int(w["seed"])): w for w in manifest.get("worlds", [])}
    for case in (cases or [str(c["name"]) for c in spec["cases"]]):
        for seed in seeds:
            if progress:
                progress(f"{spec['name']}: case {case}, seed {seed}")
            rec = build_world(spec, case, int(seed), root, progress=None)
            if rec.get("skipped") and (case, int(seed)) in worlds:
                worlds[(case, int(seed))]["skipped_last_run"] = True
            else:
                worlds[(case, int(seed))] = rec
            if progress:
                progress("  skipped (exists)" if rec.get("skipped") else
                         f"  built in {rec['build_seconds']} s, digest {rec['content_digest'][:16]}, "
                         f"in domain {rec['domain']['share_in_domain']:.3f}")
    manifest = {
        "format": "blueboat_campaign_manifest/1",
        "campaign": str(spec["name"]), "specification": spec,
        "specification_file": str(Path(spec_path).resolve()),
        "generator": {"version": GENERATOR_VERSION, "code_digest": provenance.code_digest()},
        "versions": {"python": platform.python_version(), "numpy": np.__version__},
        "worlds": [worlds[k] for k in sorted(worlds)],
    }
    mfile.write_text(yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None), encoding="utf-8")
    return manifest


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="worldgen_campaign",
        description="Build harbour worlds from a campaign specification (blueboat_campaign/1) and seeds.")
    ap.add_argument("--spec", required=True, help="the campaign specification (YAML)")
    ap.add_argument("--seeds", required=True, help="comma-separated seeds, e.g. 1,2,3")
    ap.add_argument("--out", required=True, help="the worlds root (e.g. ~/worlds)")
    ap.add_argument("--cases", default="", help="comma-separated case names (default: all)")
    ap.add_argument("--check", action="store_true", help="validate the specification and stop")
    args = ap.parse_args(argv)
    if args.check:
        spec = load_spec(args.spec)
        print(f"{spec['name']}: {len(spec['cases'])} case(s), path {resolve_path(spec['path'])}")
        return 0
    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    cases = [c for c in args.cases.split(",") if c.strip()] or None
    man = run_campaign(args.spec, seeds, args.out, cases, progress=lambda m: print(m, flush=True))
    bad = [w["world"] for w in man["worlds"] if w.get("out_of_domain")]
    if bad:
        print("OUT OF DOMAIN (under 95 % of the survey water in the 4.5-11.3 m band):", ", ".join(bad))
    print("manifest:", Path(args.out).expanduser() / ".campaigns" / man["campaign"] / "manifest.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
