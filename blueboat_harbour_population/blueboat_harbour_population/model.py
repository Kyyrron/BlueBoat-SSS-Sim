"""``populate(environment, params, seed) -> PlacementPlan``: the population of
one world.

How (docs/population/POPULATION_MODEL.md has the reasons):

1. **Where objects may lie**: survey water, in the altitude band of the
   frozen sensor, clear of the world's border and of structures.
2. **Structures** the harbour type needs and the world lacks (``structures:
   auto``), planned first: they are obstacles and sources.
3. **Sources and budgets** (``sources.build_sources``).
4. **Candidates**: each stream ``(seed, source, class, process)`` draws
   candidates at a fixed dominating density over the source's reach, each
   with a thinning mark, a priority mark, a place and a row of attributes.
   A candidate is *active* when its thinning mark is under (expected count /
   dominating count): a lower level, a smaller class share or a lower
   activity gives a subset of the same objects.
5. **Groups**: some events of a source are groups (a line of fender tyres, a
   dumping pile, a ball of gear, a pair of traps) with their own streams.
6. **Spacing**: a single or a group that lies within the hard-core distance
   of another with a smaller priority mark is deleted (Matern type II): its
   fate depends only on the candidates inside its own radius.
7. **Instances**: size, state, pose, burial for the sediment under it,
   written as entries of the frozen generator's explicit format.

Pure Python, numpy and scipy.spatial (one neighbour search).
"""

from __future__ import annotations

from typing import Any

import numpy as np

from . import __version__ as VERSION
from . import instances as inst
from .data import (CLASSES, DEBRIS, PACKAGE, Parameters, code_digest, default_parameters,
                   evidence_digest)
from .environment import Environment, Feature
from .layers import small_litter_layers
from .params import PlacementParams
from .plan import PlacementPlan, plain
from .report import build_report
from .rng import cos_sin, pick, poisson_from_uniform, stream, weighted_index
from .sources import FAR_FIELD, Source, build_sources, group_chance
from .splits import check_split
from .structures import plan_structures

TWO_PI = 6.283185307179586
#: columns of a single candidate's row: thinning mark, priority mark, cell,
#: four jitters (x, y), yaw, then the instance's attributes
_C_MARK, _C_PRIO, _C_CELL, _C_JIT, _C_YAW, _C_MOVE1, _C_MOVE2, _C_ATTR = 0, 1, 2, 3, 11, 12, 13, 14
N_SINGLE = _C_ATTR + inst.N_ATTR
#: columns of a group event's row
_G_MARK, _G_PRIO, _G_CELL, _G_JX, _G_JY, _G_SIZE, _G_DIR, _G_SCALE, _G_NET = range(9)
N_GROUP = 9
#: columns of a group member's row: placement (4), yaw, class, then attributes
_M_A, _M_B, _M_C, _M_D, _M_YAW, _M_CLASS, _M_ATTR = 0, 1, 2, 3, 4, 5, 6
N_MEMBER = _M_ATTR + inst.N_ATTR
JITTERS = 4


def allowed_mask(env: Environment, params: PlacementParams, p: Parameters) -> tuple[np.ndarray, dict]:
    """Where an object's centre may fall, and the envelope figures of the
    world: the share of its water in the altitude band."""
    lo, hi = (float(v) for v in p("envelope.altitude_m"))
    m = float(p("envelope.heave_margin_m"))
    alt = env.altitude()
    water = env.water & np.isfinite(env.depth)
    band = water & (alt - m >= lo) & (alt + m <= hi)
    base = water & ~env.blocked & ~_too_close(env, p)
    allowed = base if params.outside_band else (base & band)
    n = max(int(np.count_nonzero(water)), 1)
    a = env.cell_area_ha
    info = {"altitude_band_m": [lo, hi], "heave_margin_m": m,
            "transducer_depth_m": float(env.transducer_depth_m),
            "depth_band_m": [round(lo + m + env.transducer_depth_m, 3), round(hi - m + env.transducer_depth_m, 3)],
            "water_ha": round(float(np.count_nonzero(water)) * a, 4),
            "in_band_ha": round(float(np.count_nonzero(band)) * a, 4),
            "share_in_band": round(float(np.count_nonzero(band)) / n, 4),
            "allowed_ha": round(float(np.count_nonzero(allowed)) * a, 4),
            "outside_band_allowed": bool(params.outside_band)}
    return allowed, info


#: the smallest footprint radius of an object of the set [m] (a small debris piece)
MIN_RADIUS_M = 0.25


def _too_close(env: Environment, p: Parameters) -> np.ndarray:
    """Cells whose centre lies nearer to a wall, a structure or an object
    already there than the smallest object could: no object's centre can
    fall there, so no density is put there either."""
    clear = float(p("spacing.clearance_m")) + MIN_RADIUS_M
    out = np.zeros(env.shape, dtype=bool)
    xs, ys = env.centres()
    for f in env.features:
        if f.clear_m <= 0.0:
            continue
        r = f.clear_m + clear
        x0, x1 = f.points[:, 0].min() - r, f.points[:, 0].max() + r
        y0, y1 = f.points[:, 1].min() - r, f.points[:, 1].max() + r
        i0, i1 = np.searchsorted(xs, x0), np.searchsorted(xs, x1, side="right")
        j0, j1 = np.searchsorted(ys, y0), np.searchsorted(ys, y1, side="right")
        if i1 <= i0 or j1 <= j0:
            continue
        X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1])
        out[j0:j1, i0:i1] |= f.distance(X, Y) < r
    for o in env.occupied:
        r = float(o.get("radius", 0.5)) + clear
        i0, i1 = np.searchsorted(xs, float(o["x"]) - r), np.searchsorted(xs, float(o["x"]) + r, side="right")
        j0, j1 = np.searchsorted(ys, float(o["y"]) - r), np.searchsorted(ys, float(o["y"]) + r, side="right")
        if i1 <= i0 or j1 <= j0:
            continue
        X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1])
        dx, dy = X - float(o["x"]), Y - float(o["y"])
        out[j0:j1, i0:i1] |= dx * dx + dy * dy < r * r
    return out


class _World:
    """The environment as the sampler reads it."""

    def __init__(self, env: Environment, allowed: np.ndarray, p: Parameters, params: PlacementParams,
                 extra_occupied: list) -> None:
        self.env, self.allowed = env, allowed
        self.xs, self.ys = env.centres()
        self.nx = env.shape[1]
        self.cell = float(env.cell_m)
        self.clear = float(p("spacing.clearance_m"))
        self.spacing = float(params.min_spacing_m)
        self.other = float(p("spacing.other_m"))
        self.solids = [f for f in env.features if f.clear_m > 0.0]
        self.occupied = list(env.occupied) + list(extra_occupied)
        lo, hi = (float(v) for v in p("envelope.altitude_m"))
        m = float(p("envelope.heave_margin_m"))
        self.alt_lo, self.alt_hi = lo + m, hi - m

    def valid(self, x: float, y: float, rad: float, target: bool) -> bool:
        """Whether an object of footprint radius ``rad`` may lie at (x, y)."""
        env = self.env
        if not bool(env.inside(x, y)):
            return False
        j, i = env.cell_of(x, y)
        if not self.allowed[j, i]:
            return False
        for f in self.solids:
            if float(f.distance(x, y)) < f.clear_m + rad + self.clear:
                return False
        for o in self.occupied:
            dx, dy = x - float(o["x"]), y - float(o["y"])
            need = float(o.get("radius", 0.5)) + rad + self.clear
            if target and o.get("target"):
                need = max(need, self.spacing)
            if dx * dx + dy * dy < need * need:
                return False
        return True

    def cell_xy(self, flat: int) -> tuple[float, float]:
        j, i = divmod(int(flat), self.nx)
        return float(self.xs[i]), float(self.ys[j])

    def here(self, x: float, y: float) -> dict:
        """Depth, altitude and sediment at a point (its planning cell)."""
        env = self.env
        j, i = env.cell_of(x, y)
        d = float(env.depth[j, i])
        alt = d - float(env.transducer_depth_m)
        name = env.sediment_names[int(env.sediment[j, i])]
        return {"depth_m": round(d, 2), "altitude_m": round(alt, 2), "sediment_under": name,
                "on_mud": bool(name in env.mud_classes),
                "in_envelope": bool(self.alt_lo <= alt <= self.alt_hi)}


def _draw(seed: int, keys: tuple, mean_dom: float, wanted: float, cols: int, fixed: bool
          ) -> tuple[np.ndarray, np.ndarray, bool]:
    """The active candidates of one stream: ``(rows, their indices,
    clipped)``. ``wanted``: the expected number of active candidates."""
    if wanted <= 0.0 or mean_dom <= 0.0:
        return np.zeros((0, cols)), np.zeros(0, dtype=np.int64), False
    n = poisson_from_uniform(float(stream(seed, *keys, "count").random()), mean_dom)
    if n == 0:
        return np.zeros((0, cols)), np.zeros(0, dtype=np.int64), False
    U = stream(seed, *keys, "cand").random((n, cols))
    clipped = wanted > mean_dom
    if fixed:
        k = min(int(np.floor(wanted + 0.5)), n)
        idx = np.sort(np.argsort(U[:, 0], kind="stable")[:k])
    else:
        idx = np.nonzero(U[:, 0] < min(wanted / mean_dom, 1.0))[0]
    return U[idx], idx.astype(np.int64), clipped


def _place(w: _World, src: Source, row: np.ndarray, rad: float, target: bool) -> tuple[float, float] | None:
    """A candidate's position: its cell from the source's kernel, then the
    first of its jitters at which the object fits."""
    cum = src.pdf_cum
    k = int(np.searchsorted(cum, row[_C_CELL] * cum[-1], side="right"))
    cx, cy = w.cell_xy(src.cells[min(k, len(cum) - 1)])
    for q in range(JITTERS):
        x = cx + (row[_C_JIT + 2 * q] - 0.5) * w.cell
        y = cy + (row[_C_JIT + 2 * q + 1] - 0.5) * w.cell
        if w.valid(x, y, rad, target):
            return x, y
    return None


def _member_record(src: Source, process: str, index: int, member: int | None, group_id: str | None,
                   dist: float) -> dict:
    return {"hotspot": "" if src.id == FAR_FIELD else src.id,
            "hotspot_distance_m": round(float(dist), 2),
            "source": {"feature": src.id, "use": src.use, "process": process,
                       "index": int(index) if member is None else [int(index), int(member)]},
            "group_id": group_id}


def _singles(w: _World, src: Source, cls: str, wanted: float, seed: int, p: Parameters,
             params: PlacementParams, dom_per_ha: float, units: list, notes: dict) -> None:
    rows, idx, clipped = _draw(seed, (src.id, cls, "single"), dom_per_ha * src.reach_ha, wanted, N_SINGLE,
                               params.count == "fixed")
    if clipped:
        notes["clipped"].append(f"{src.id}/{cls}")
    target = cls != DEBRIS
    for row, k in zip(rows, idx):
        a = row[_C_ATTR:]
        sh = inst.shape(cls, a, p, r6=params.uncertain_materials, face_used_share=params.face_used_share)
        rad = inst.radius(sh)
        xy = _place(w, src, row, rad, target)
        if xy is None:
            notes["no_room"] += 1
            continue
        units.append({"key": f"{src.id}/{cls}/single/{int(k):07d}", "x": xy[0], "y": xy[1], "radius": rad,
                      "prio": float(row[_C_PRIO]), "target": target, "group": None, "source": src,
                      "members": [{"cls": cls, "x": xy[0], "y": xy[1], "yaw": TWO_PI * float(row[_C_YAW]),
                                   "shape": sh, "attr": a, "process": "single", "index": int(k),
                                   "member": None, "move": (float(row[_C_MOVE1]), float(row[_C_MOVE2]))}]})


def _group_members(w: _World, src: Source, cls: str, process: str, row: np.ndarray, k: int, seed: int,
                   p: Parameters, params: PlacementParams, class_weights: dict) -> list:
    """The members of one group event (those that fit)."""
    cfg = p(f"groups.{process}")
    lo, hi = int(cfg["members"][0]), int(cfg["members"][1])
    n = lo + pick(row[_G_SIZE], hi - lo + 1)
    cum = src.pdf_cum
    c = int(np.searchsorted(cum, row[_G_CELL] * cum[-1], side="right"))
    ax, ay = w.cell_xy(src.cells[min(c, len(cum) - 1)])
    ax += (row[_G_JX] - 0.5) * w.cell
    ay += (row[_G_JY] - 0.5) * w.cell
    M = stream(seed, src.id, cls, process, "member", str(int(k))).random((hi + 1, N_MEMBER))
    out: list = []
    if process in ("tyre_line", "trap_pair"):
        if cfg.get("along") == "feature" and src.feature is not None:
            _, _, ux, uy = src.feature.nearest(ax, ay)
        else:
            ux, uy = cos_sin(TWO_PI * float(row[_G_DIR]))
        s_lo, s_hi = (float(v) for v in cfg["spacing_m"])
        lat = float(cfg.get("lateral_m", 0.0))
        gaps = [s_lo + float(M[q, _M_A]) * (s_hi - s_lo) for q in range(n)]
        total = 0.0
        for q in range(1, n):
            total += gaps[q]
        t = -0.5 * total
        for q in range(n):
            if q:
                t += gaps[q]
            off = (2.0 * float(M[q, _M_B]) - 1.0) * lat
            out.append((cls, ax + t * ux - off * uy, ay + t * uy + off * ux, q))
    else:                                           # pile, gear_ball: inside a disc
        r_lo, r_hi = (float(v) for v in cfg["radius_m"])
        R = r_lo + float(row[_G_SCALE]) * (r_hi - r_lo)
        names = list(class_weights)
        for q in range(n):
            ca, sa = cos_sin(TWO_PI * float(M[q, _M_A]))
            r = R * float(np.sqrt(M[q, _M_B]))
            mc = cls
            if cls == "*":
                mc = names[weighted_index(M[q, _M_CLASS], [class_weights[c] for c in names])]
            out.append((mc, ax + r * ca, ay + r * sa, q))
        if process == "gear_ball" and "net_bundle" in params.classes \
                and row[_G_NET] < float(cfg.get("net_chance", 0.0)):
            out.append(("net_bundle", ax, ay, hi))            # the rope and net that bind the ball
    members = []
    for mc, x, y, q in out:
        a = M[q, _M_ATTR:]
        sh = inst.shape(mc, a, p, r6=params.uncertain_materials, face_used_share=params.face_used_share)
        if not w.valid(x, y, inst.radius(sh), mc != DEBRIS):
            continue
        members.append({"cls": mc, "x": x, "y": y, "yaw": TWO_PI * float(M[q, _M_YAW]), "shape": sh,
                        "attr": a, "process": process, "index": int(k), "member": int(q), "move": None})
    return members


def _groups(w: _World, src: Source, cls: str, process: str, events: float, seed: int, p: Parameters,
            params: PlacementParams, dom_per_ha: float, class_weights: dict, units: list, notes: dict) -> None:
    rows, idx, clipped = _draw(seed, (src.id, cls, process), dom_per_ha * src.reach_ha, events, N_GROUP,
                               params.count == "fixed")
    if clipped:
        notes["clipped"].append(f"{src.id}/{cls}/{process}")
    for row, k in zip(rows, idx):
        mem = _group_members(w, src, cls, process, row, int(k), seed, p, params, class_weights)
        if not mem:
            notes["no_room"] += 1
            continue
        cx, cy = 0.0, 0.0
        for m in mem:
            cx += m["x"]
            cy += m["y"]
        cx, cy = cx / len(mem), cy / len(mem)
        rad = 0.0
        for m in mem:
            dx, dy = m["x"] - cx, m["y"] - cy
            rad = max(rad, float(np.sqrt(dx * dx + dy * dy)) + inst.radius(m["shape"]))
        gid = None if len(mem) < 2 else f"{src.id}.{process}.{int(k)}" + ("" if cls == "*" else f".{cls}")
        units.append({"key": f"{src.id}/{cls}/{process}/{int(k):07d}", "x": cx, "y": cy, "radius": rad,
                      "prio": float(row[_G_PRIO]), "target": any(m["cls"] != DEBRIS for m in mem),
                      "group": gid, "process": process, "source": src, "members": mem})


def _hard_core(units: list, w: _World) -> list:
    """Matern type II thinning: a unit is deleted when another unit with a
    smaller priority mark (ties: the smaller key) lies nearer than the
    hard-core distance of the pair. Deleted units still delete."""
    n = len(units)
    if n < 2:
        return units
    from scipy.spatial import cKDTree
    xy = np.array([(u["x"], u["y"]) for u in units])
    R = np.array([u["radius"] for u in units])
    tg = np.array([u["target"] for u in units])
    pr = np.array([u["prio"] for u in units])
    reach = 2.0 * float(R.max()) + max(w.spacing, w.other) + w.clear + 1e-6
    pairs = cKDTree(xy).query_pairs(reach, output_type="ndarray")
    if not len(pairs):
        return units
    i, j = pairs[:, 0], pairs[:, 1]
    dx, dy = xy[i, 0] - xy[j, 0], xy[i, 1] - xy[j, 1]
    d = np.sqrt(dx * dx + dy * dy)
    # between two targets the stated spacing holds between their edges for a
    # group (its disc) and between their centres for two singles
    gap = np.where(tg[i] & tg[j], w.spacing, w.other)
    grp = np.array([u["group"] is not None for u in units])
    need = np.maximum(R[i] + R[j] + w.clear, np.where(grp[i] | grp[j], R[i] + R[j] + gap, gap))
    hit = d < need
    dead = np.zeros(n, dtype=bool)
    keys = [u["key"] for u in units]
    for a, b in zip(i[hit], j[hit]):
        a_first = (pr[a] < pr[b]) or (pr[a] == pr[b] and keys[a] < keys[b])
        dead[b if a_first else a] = True
    return [u for u, k in zip(units, dead) if not k]


def _redistribute(units: list, w: _World, p: Parameters) -> int:
    """Optional: a share of the light single objects moves a few metres down
    the local slope (storms carry crates and nets into the lows), when the
    new place is free. Returns how many moved."""
    cfg = p("redistribution")
    env = w.env
    gy, gx = np.gradient(np.nan_to_num(env.depth, nan=0.0), env.cell_m)
    moved = 0
    for u in sorted(units, key=lambda q: q["key"]):
        m = u["members"][0]
        if u["group"] is not None or len(u["members"]) != 1 or m["cls"] not in cfg["light_classes"]:
            continue
        if m["move"] is None or m["move"][0] >= float(cfg["share"]):
            continue
        j, i = env.cell_of(u["x"], u["y"])
        sx, sy = float(gx[j, i]), float(gy[j, i])
        norm = float(np.sqrt(sx * sx + sy * sy))
        if norm < 1e-4:
            continue
        lo, hi = (float(v) for v in cfg["distance_m"])
        dist = lo + m["move"][1] * (hi - lo)
        x, y = u["x"] + dist * sx / norm, u["y"] + dist * sy / norm
        if not w.valid(x, y, u["radius"], u["target"]):
            continue
        ok = True
        for v in units:
            if v is u:
                continue
            gap = w.spacing if (u["target"] and v["target"]) else w.other
            need = max(u["radius"] + v["radius"] + w.clear, gap)
            dx, dy = x - v["x"], y - v["y"]
            if dx * dx + dy * dy < need * need:
                ok = False
                break
        if ok:
            u["x"], u["y"], m["x"], m["y"] = x, y, x, y
            m["process"] = "single_moved"
            moved += 1
    return moved


def populate(environment: Environment, params: PlacementParams | dict | None = None, seed: int | None = None,
             parameters: Parameters | None = None) -> PlacementPlan:
    """The population plan of ``environment`` for the ``placement`` block
    ``params`` and the placement ``seed`` (default: the block's own).

    The same environment, block and seed give the same plan, byte for byte.
    Raises when the seed or the harbour belongs to another split than the
    block names (``split_override`` lets it through, and is recorded)."""
    p = parameters or default_parameters()
    if not isinstance(params, PlacementParams):
        params = PlacementParams.from_dict(params, p)
    seed = int(params.seed if seed is None else seed)
    if params.mode != "mission_like":
        raise ValueError(f"populate: placement.mode {params.mode!r} (only mission_like is planned here)")
    split_note = check_split(seed, params.split, params.harbour, params.split_override)
    env = environment
    if params.features:
        from .params import _thaw
        env = _with_features(env, [Feature.from_dict(_thaw(f)) for f in params.features])
    known = {f.id for f in env.features}
    for fid, _ in params.feature_tags + params.feature_activity:
        if fid not in known:
            raise KeyError(f"placement: no feature {fid!r} in the environment (features: {sorted(known)})")
    allowed, envelope = allowed_mask(env, params, p)
    # 2. structures first
    s_entries, s_feats, s_occ, s_report = plan_structures(env, params, p, seed, allowed)
    if s_feats:
        env = _with_features(env, s_feats)
    w = _World(env, allowed, p, params, s_occ)
    # 3. sources and budgets
    srcs, src_report = build_sources(env, params, p, allowed, seed)
    level = int(params.density_level)
    pg = group_chance(p, level) if params.groups else 0.0
    dom = float(p("dominating_per_ha"))
    notes: dict[str, Any] = {"clipped": [], "no_room": 0}
    units: list = []
    expected: dict[str, float] = {c: 0.0 for c in CLASSES}
    for src in srcs:
        if not len(src.cells) or src.budget <= 0.0:
            continue
        exp = src.expected
        if not params.debris:
            exp[DEBRIS] = 0.0
        procs = dict(src.groups)
        # a process over every class of the source (a dumping pile)
        star = [(g, float(spec.get("weight", 1.0))) for g, spec in sorted(procs.items()) if spec["class"] == "*"]
        star_single = 1.0
        if star and pg > 0.0:
            total = 0.0
            for c in sorted(exp):
                total += exp[c]
            for g, wt in star:
                pi = min(pg * wt, 0.9)
                cfg = p(f"groups.{g}")
                nbar = 0.5 * (float(cfg["members"][0]) + float(cfg["members"][1]))
                events = total * pi / (1.0 - pi + pi * nbar)
                star_single = star_single * (1.0 - pi) / (1.0 - pi + pi * nbar)
                weights = {c: exp[c] for c in CLASSES if exp.get(c, 0.0) > 0.0}
                if weights:
                    _groups(w, src, "*", g, events, seed, p, params, dom, weights, units, notes)
        for cls in CLASSES:
            lam = exp.get(cls, 0.0)
            if lam <= 0.0:
                continue
            expected[cls] += lam
            mine = [(g, float(spec.get("weight", 1.0))) for g, spec in sorted(procs.items())
                    if spec["class"] == cls]
            denom, pis = 1.0, []
            if mine and pg > 0.0:
                left = 0.9
                for g, wt in mine:
                    pi = min(pg * wt, left)
                    left -= pi
                    cfg = p(f"groups.{g}")
                    nbar = 0.5 * (float(cfg["members"][0]) + float(cfg["members"][1]))
                    pis.append((g, pi, nbar))
                    denom += pi * (nbar - 1.0)
            events = lam * star_single / denom
            share_single = 1.0
            for _, pi, _ in pis:
                share_single -= pi
            _singles(w, src, cls, events * share_single, seed, p, params, dom, units, notes)
            for g, pi, _ in pis:
                _groups(w, src, cls, g, events * pi, seed, p, params, dom, {}, units, notes)
    n_before = 0
    for u in units:
        n_before += len(u["members"])
    units = _hard_core(units, w)
    moved = _redistribute(units, w, p) if params.redistribution else 0
    # 7. instances, in the world's order: by class, then by stream key
    rows = []
    for u in units:
        for m in u["members"]:
            rows.append((CLASSES.index(m["cls"]), u["key"], -1 if m["member"] is None else m["member"], u, m))
    rows.sort(key=lambda r: (r[0], r[1], r[2]))
    entries: list = []
    groups: dict[str, dict] = {}
    min_t = float(p("nominal_target_min_m"))
    for _, _, _, u, m in rows:
        src: Source = u["source"]
        here = w.here(m["x"], m["y"])
        b = inst.burial(m["cls"], m["shape"]["pose"][0], here["on_mud"], m["attr"], p, params.burial)
        dist = 0.0 if src.feature is None else float(src.feature.distance(m["x"], m["y"]))
        rec = _member_record(src, m["process"], m["index"], m["member"], u["group"], dist)
        rec["sediment"] = "mud" if here["on_mud"] else "sand"
        rec.update(here)
        rec["stand_in_for"] = inst.stand_in(m["cls"], src.use, m["attr"], p)
        prm_fill = m["shape"]["params"]
        if "fill" in prm_fill:
            prm_fill["fill_material"] = here["sediment_under"]
        e = inst.entry(m["cls"], m["x"], m["y"], m["yaw"], m["shape"], b, rec,
                       reflectivity=float(env.material_reflectivity.get(m["shape"]["material"], 0.5)),
                       min_target_m=min_t)
        if u["group"] is not None:
            g = groups.setdefault(u["group"], {"id": u["group"], "type": u["process"], "source": src.id,
                                               "use": src.use, "x": round(float(u["x"]), 3),
                                               "y": round(float(u["y"]), 3),
                                               "radius_m": round(float(u["radius"]), 3), "members": [],
                                               "classes": {}})
            g["members"].append(len(entries))
            g["classes"][m["cls"]] = g["classes"].get(m["cls"], 0) + 1
        entries.append(e)
    group_list = [groups[k] for k in sorted(groups)]
    layers, litter = small_litter_layers(env, params, p, seed)
    # plain values only: a numpy scalar in a world file fails the build at its last step
    plan = PlacementPlan(instances=plain(entries), groups=plain(group_list), structures=plain(s_entries),
                         layers=plain(layers), params=plain(params.to_dict()),
                         model={"package": PACKAGE, "version": VERSION, "code_digest": code_digest(),
                                "evidence_digest": evidence_digest(p.path or None)},
                         environment_digest=environment.digest(), seed=seed,
                         litter_class=litter.get("material") if litter.get("mean_per_m2", 0.0) > 0.0 else None)
    plan.report = plain(build_report(env, allowed, envelope, srcs, src_report, expected, plan, p, params,
                                     {"before_spacing": n_before, "no_room": notes["no_room"],
                                      "clipped": notes["clipped"], "moved": moved, "structures": s_report,
                                      "small_litter": litter, "split": split_note}))
    return plan


def _with_features(env: Environment, feats: list) -> Environment:
    import dataclasses
    return dataclasses.replace(env, features=list(env.features) + list(feats))
