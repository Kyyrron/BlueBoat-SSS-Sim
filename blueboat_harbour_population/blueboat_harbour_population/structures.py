"""Structures a harbour type needs and the environment lacks (``structures:
auto``): a mooring field (concrete blocks with their riser chains) and the
piles of a pontoon. Objects only: the seabed is not changed (the scour
hollows of the campaign generator's own blocks are terrain layers of the
environment, made before a population is planned).

A structure is never a target and never litter. It is an obstacle for the
litter and, for a mooring block, a source of it.

Entries are in the frozen generator's explicit-instance format, with the
parameters its structure builders read (``worldgen/campaign.py``
``_structures``, ``worldgen/compounds.py``): ``role: structure``, ``name``,
and for a chain ``link_m``, ``bar_mm``, ``z0_m``, ``z1_m``, ``sag_m``.

Pure Python and numpy.
"""

from __future__ import annotations

import numpy as np

from .data import Parameters
from .environment import Environment, Feature
from .params import PlacementParams
from .rng import cos_sin, pick, stream

TWO_PI = 6.283185307179586


def _reflectivity(env: Environment, material: str) -> float:
    return float(getattr(env, "material_reflectivity", {}).get(material, 0.5))


def _path_distance(env: Environment, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    if env.path_xy is None or len(env.path_xy) < 2:
        return np.full(np.shape(x), np.inf)
    return Feature("path", "lane", env.path_xy, geometry="polyline").distance(x, y)


def plan_structures(env: Environment, params: PlacementParams, p: Parameters, seed: int,
                    allowed: np.ndarray) -> tuple[list, list, list, dict]:
    """``(entries, features, occupied, report)`` of the structures to add.
    ``features``: the new source features (kind ``mooring`` / ``pile``);
    ``occupied``: their footprints as obstacles."""
    entries: list = []
    feats: list = []
    occ: list = []
    report = {"mooring_blocks": 0, "piles": 0}
    if params.structures != "auto":
        return entries, feats, occ, report
    xs, ys = env.centres()
    X, Y = np.meshgrid(xs, ys)
    cfg = p("structures.mooring_block")
    keep = float(cfg["path_keep_out_m"])
    # ---- mooring field: where the profile has one and the world has no mooring yet
    per_ha = float(p("structures.per_profile").get(params.profile, 0.0))
    has_mooring = any(f.kind == "mooring" for f in env.features)
    if per_ha > 0.0 and not has_mooring:
        solid = [f for f in env.features if f.kind in ("quay", "breakwater", "pontoon", "pier", "berth")]
        far = np.ones(env.shape, dtype=bool)
        for f in solid:
            far &= f.distance(X, Y) >= 15.0
        deep = (env.depth - float(cfg["clearance_under_keel_m"])) >= float(cfg["height_m"][0])
        ok = allowed & far & deep & (_path_distance(env, X, Y) >= keep + 0.5 * float(cfg["size_m"][1]))
        cells = np.nonzero(ok.ravel())[0]
        n = int(np.floor(per_ha * len(cells) * env.cell_area_ha + 0.5))
        if n > 0 and len(cells):
            U = stream(seed, "structures", "mooring").random((60 * n + 60, 12))
            placed: list = []
            for row in U:
                if len(placed) >= n:
                    break
                c = int(cells[pick(row[0], len(cells))])
                j, i = divmod(c, env.shape[1])
                x = float(xs[i] + (row[1] - 0.5) * env.cell_m)
                y = float(ys[j] + (row[2] - 0.5) * env.cell_m)
                gap = float(cfg["spacing_m"][0]) + row[3] * (float(cfg["spacing_m"][1]) - float(cfg["spacing_m"][0]))
                if any((x - q[0]) * (x - q[0]) + (y - q[1]) * (y - q[1]) < gap * gap for q in placed):
                    continue
                placed.append((x, y, row))
            for k, (x, y, row) in enumerate(placed):
                j, i = env.cell_of(x, y)
                d = float(env.depth[j, i])
                size = round(float(cfg["size_m"][0]) + row[4] * (float(cfg["size_m"][1]) - float(cfg["size_m"][0])), 2)
                h = float(cfg["height_m"][0]) + row[5] * (float(cfg["height_m"][1]) - float(cfg["height_m"][0]))
                h = round(min(h, d - float(cfg["clearance_under_keel_m"])), 2)
                yaw_deg = round(90.0 * float(row[6]), 1)
                name = f"ml_block_{k + 1}"
                bar = round(float(cfg["bar_mm"][0]) + row[7] * (float(cfg["bar_mm"][1]) - float(cfg["bar_mm"][0])), 1)
                lr = round(float(cfg["riser_m"][0]) + row[8] * (float(cfg["riser_m"][1]) - float(cfg["riser_m"][0])), 2)
                ryaw = round(TWO_PI * float(row[9]), 5)
                cx, sx = cos_sin(ryaw)
                entries.append({"type": "mooring_block", "x": round(x, 3), "y": round(y, 3),
                                "yaw": round(float(np.radians(yaw_deg)), 5), "length": size, "width": size,
                                "proud_height": h, "burial": 0.0, "fill_fraction": 1.0,
                                "reflectivity": _reflectivity(env, "concrete"), "material": "concrete",
                                "params": {"spec": 2, "role": "structure", "name": name,
                                           "source": {"feature": name, "use": "structure", "process": "mooring_field"}}})
                w = round(3.4 * bar * 1e-3, 3)
                entries.append({"type": "chain", "x": round(x + 0.5 * lr * cx, 3), "y": round(y + 0.5 * lr * sx, 3),
                                "yaw": ryaw, "length": lr, "width": w, "proud_height": w, "burial": 0.0,
                                "fill_fraction": 1.0, "reflectivity": _reflectivity(env, "steel_rough"),
                                "material": "steel_rough",
                                "params": {"spec": 2, "role": "structure", "name": f"{name}_riser",
                                           "link_m": round(6.0 * bar * 1e-3, 3), "bar_mm": bar, "z0_m": h,
                                           "z1_m": round(max(d - 0.3, h + 0.5), 2), "sag_m": 0.0,
                                           "source": {"feature": name, "use": "structure",
                                                      "process": "mooring_field"}}})
                rad = round(0.5 * size * 1.4142135623730951 + 0.1, 3)
                feats.append(Feature(name, "mooring", [(x, y)], clear_m=rad, attrs={"added": True}))
                occ.append({"x": x, "y": y, "radius": rad, "target": False, "id": name})
            report["mooring_blocks"] = len(placed)
    # ---- piles along the pontoons that have none
    pc = p("structures.pontoon_pile")
    piles_have = [f for f in env.features if f.kind == "pile"]
    k = 0
    for f in sorted((q for q in env.features if q.kind == "pontoon"), key=lambda q: q.id):
        if f.geometry != "polyline" or f.length_m < 4.0:
            continue
        if any(float(f.distance(q.points[0, 0], q.points[0, 1])) < 3.0 for q in piles_have):
            continue                                   # this pontoon already has its piles
        u = stream(seed, "structures", "piles", f.id).random(8)
        dia = round(float(pc["diameter_m"][0]) + u[0] * (float(pc["diameter_m"][1]) - float(pc["diameter_m"][0])), 2)
        mat = pc["materials"][pick(u[1], len(pc["materials"]))]
        half_w = 0.5 * float(f.attrs.get("width_m", 2.5))
        # stations along the centre line: both ends, then every spacing
        seg_a, seg_b = f.points[0], f.points[-1]
        ln = float(np.sqrt((seg_b[0] - seg_a[0]) * (seg_b[0] - seg_a[0]) + (seg_b[1] - seg_a[1]) * (seg_b[1] - seg_a[1])))
        if ln < 1.0:
            continue                                   # a closed outline (a float drawn as an area): no centre line
        ux, uy = (seg_b[0] - seg_a[0]) / ln, (seg_b[1] - seg_a[1]) / ln
        n_st = max(int(np.floor(ln / float(pc["spacing_m"]))) + 1, 2)
        for q in range(n_st):
            t = ln * q / (n_st - 1)
            side = 1.0 if q % 2 == 0 else -1.0
            x = float(seg_a[0] + t * ux - side * (half_w + 0.5 * dia) * uy)
            y = float(seg_a[1] + t * uy + side * (half_w + 0.5 * dia) * ux)
            if not bool(env.inside(x, y)):
                continue
            j, i = env.cell_of(x, y)
            if not env.water[j, i] or float(_path_distance(env, np.array(x), np.array(y))) < keep:
                continue
            k += 1
            name = f"ml_pile_{k}"
            d = float(env.depth[j, i])
            entries.append({"type": "mooring_pile", "x": round(x, 3), "y": round(y, 3), "yaw": 0.0,
                            "length": dia, "width": dia, "proud_height": round(d + float(pc["above_water_m"]), 2),
                            "burial": 0.0, "fill_fraction": 1.0, "reflectivity": _reflectivity(env, mat),
                            "material": mat,
                            "params": {"spec": 2, "role": "structure", "name": name,
                                       "source": {"feature": f.id, "use": "structure", "process": "pontoon_piles"}}})
            feats.append(Feature(name, "pile", [(x, y)], clear_m=round(0.5 * dia, 3), attrs={"added": True}))
            occ.append({"x": x, "y": y, "radius": round(0.5 * dia, 3), "target": False, "id": name})
    report["piles"] = k
    return entries, feats, occ, report
