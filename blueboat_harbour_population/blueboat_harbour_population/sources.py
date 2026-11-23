"""Sources of litter: which features emit, how much, which classes, and where
their objects lie.

* A feature's **use** is a source tag: the user's (``placement.feature_tags``),
  else the feature's own, else the profile's default for the feature's kind.
  ``none`` makes it no source.
* A source **reaches** the allowed water within the active band of the
  feature (within its own radius for a spot, a lane or a ground). A cell
  reached by n sources belongs to each for 1 / n. A source's **budget** is
  the level's density times the area it owns times its activity: the mean
  density over the reached water is the level's, whatever the geometry, and
  changing a source's use never changes another source's budget.
* The **class mix** of a source is the profile's class weights times the
  tag's emission, centred so that a harbour with the profile's reference
  composition of sources has exactly the profile's weights.
* The **kernel** says where a source's objects lie: a density per unit area
  against the distance to the feature.

Pure Python and numpy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

from .data import DEBRIS, TARGETS, Parameters
from .environment import Environment, Feature
from .params import ACTIVITY_MAX, PlacementParams
from .rng import dexp, poisson_from_uniform, stream

FAR_FIELD = "far_field"


@dataclass
class Source:
    """One emitting feature, with everything the sampler needs."""

    id: str
    feature: Feature | None                   # None: the far field
    use: str
    activity: float
    cells: np.ndarray                         # flat indices of the allowed cells it reaches
    dist: np.ndarray                          # distance of each to the feature [m] (0 for the far field)
    owned_ha: float = 0.0                     # area it owns (cells shared equally with the other sources)
    reach_ha: float = 0.0
    budget: float = 0.0                       # expected sonar-size objects (targets + debris), after retention
    target_share: float = 0.5
    mix: dict = field(default_factory=dict)   # class -> share of its target budget (the selected classes)
    pdf_cum: np.ndarray | None = None         # cumulative kernel weight over ``cells``
    groups: dict = field(default_factory=dict)

    @property
    def expected(self) -> dict:
        """Expected objects per class (debris included)."""
        out = {c: self.budget * self.target_share * s for c, s in self.mix.items()}
        out[DEBRIS] = self.budget * (1.0 - self.target_share)
        return out


# --------------------------------------------------------------------- level
def level_density(p: Parameters, level: int) -> float:
    return float(p("levels.density_per_ha")[int(level)])


def group_chance(p: Parameters, level: int) -> float:
    return float(p("levels.group_chance")[int(level)])


def small_litter_density(p: Parameters, level: int) -> float:
    return float(p("levels.small_litter_per_m2")[int(level)])


def retention(p: Parameters, level: int) -> float:
    return float(p("levels.retention")[int(level)])


# ----------------------------------------------------------------------- use
def default_use(p: Parameters, profile: str, kind: str) -> str:
    return str(p(f"profiles.{profile}.kind_use").get(kind, "none"))


def use_of(p: Parameters, params: PlacementParams, f: Feature) -> str:
    """The source tag of a feature: the user's, else its own, else the
    profile's default for its kind."""
    return params.tag_of(f.id) or f.use or default_use(p, params.profile, f.kind)


def activity_of(p: Parameters, params: PlacementParams, f: Feature, use: str) -> float:
    """Relative emission of a feature: the user's, else the feature's own
    times the default of its use (a dumping spot is a hot spot by nature)."""
    a = params.activity_of(f.id)
    if a is None:
        base = float(p(f"tags.{use}.activity")) if p.has(f"tags.{use}.activity") else 1.0
        a = float(f.activity) * base
    return min(max(float(a), 0.0), ACTIVITY_MAX)


def reach_m(p: Parameters, use: str, geometry: str = "polyline") -> float:
    """How far from its feature a source reaches [m]: the active band for a
    line (a quay, a pontoon), the point reach for a point (a block, an
    outlet), the kernel's own radius for a use with a flat kernel (a spot, a
    lane, a ground)."""
    k = p(f"tags.{use}.kernel")
    if k["near"] == "uniform" and float(k.get("broad_share", 0.0)) == 0.0:
        return float(k.get("scale_m", 0.0))
    if geometry == "point":
        return float(p("point_reach_m"))
    return float(p("active_band_m"))


def patch_field(p: Parameters, use: str, f: Feature | None, x: np.ndarray, y: np.ndarray, seed: int,
                cell_ha: float = 1e-4) -> np.ndarray:
    """Relative emission along a source at the points (x, y): litter is lost
    at particular places (a berth, a ladder, one end of a pontoon; a hot
    area of a fishing ground), not evenly. ``1 - share`` of the emission is
    even, ``share`` sits in Gaussian patches round spots drawn on the
    feature from the stream ``(seed, feature, "spots")``: the same spots at
    every level and for every class. 1 everywhere for a point feature or a
    use without patches."""
    one = np.ones(np.shape(x))
    if f is None or f.geometry == "point" or not p.has(f"tags.{use}.patch"):
        return one
    cfg = p(f"tags.{use}.patch")
    share, sigma = float(cfg["share"]), float(cfg["sigma_m"])
    if share <= 0.0 or not len(one):
        return one
    if f.geometry == "polygon":
        # spots on the reached cells themselves
        n_cells = len(one)
        mean = float(cfg["spots_per_ha"]) * float(cell_ha) * n_cells
        u = stream(seed, f.id, "spots").random(2 + 2 * 64)
        k = min(max(poisson_from_uniform(float(u[0]), mean), 1), 64)
        acc = np.zeros(np.shape(x))
        for q in range(k):
            c = min(int(u[2 + 2 * q] * n_cells), n_cells - 1)
            dx, dy = x - x[c], y - y[c]
            acc = acc + dexp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma))
    else:
        length = f.length_m
        if length <= 0.0:
            return one
        mean = float(cfg["spots_per_100m"]) * length / 100.0
        u = stream(seed, f.id, "spots").random(2 + 64)
        k = min(max(poisson_from_uniform(float(u[0]), mean), 1), 64)
        s = f.along(x, y)
        acc = np.zeros(np.shape(x))
        for q in range(k):
            ds = s - float(u[2 + q]) * length
            acc = acc + dexp(-(ds * ds) / (2.0 * sigma * sigma))
    tot = float(np.cumsum(acc)[-1])
    if tot <= 0.0:
        return one
    return (1.0 - share) + share * acc * (len(one) / tot)


# -------------------------------------------------------------------- kernel
def kernel_profile(p: Parameters, use: str, d: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """``(near, broad, broad share)``: the two parts of a tag's kernel at the
    distances ``d`` (relative densities per unit area, not normalised)."""
    k = p(f"tags.{use}.kernel")
    s = float(k.get("scale_m", 0.0))
    shape = k["near"]
    if shape == "half_normal":
        near = dexp(-(d * d) / (2.0 * s * s))
    elif shape == "exponential":
        near = dexp(-d / s)
    elif shape == "bump":
        m = float(k.get("mode_m", 0.0))
        near = dexp(-((d - m) * (d - m)) / (2.0 * s * s))
    else:                                       # uniform within the scale
        near = (d <= s).astype(np.float64)
    return near, np.ones_like(d), float(k.get("broad_share", 0.0))


def kernel_cumulative(p: Parameters, use: str, d: np.ndarray, patch: np.ndarray | None = None) -> np.ndarray:
    """Cumulative probability of the reached cells (in their own order): the
    near part holds ``1 - broad share`` of the objects, the flat part the
    rest; both follow the source's patches. Sequential sums only."""
    near, broad, beta = kernel_profile(p, use, d)
    if patch is not None:
        near, broad = near * patch, broad * patch
    cn, cb = np.cumsum(near), np.cumsum(broad)
    if not len(d):
        return cn
    if cn[-1] <= 0.0:
        beta = 1.0
    w = np.zeros(len(d))
    if beta < 1.0:
        w = w + (1.0 - beta) * (near / cn[-1])
    if beta > 0.0:
        w = w + beta * (broad / cb[-1])
    return np.cumsum(w)


# ----------------------------------------------------------------------- mix
def centred_affinity(p: Parameters, profile: str) -> dict:
    """Class multipliers g(c) such that a harbour with the profile's
    reference composition of sources has the profile's class weights:
    ``sum_tag r_tag m_tag(c) = W(c)`` with ``m_tag(c)`` proportional to
    ``W(c) g(c) e_tag(c)``. Solved by iterative proportional fitting over the
    seven classes (never over a selection, so that deselecting a class does
    not change the others' multipliers). An empty reference: all ones."""
    ref = p(f"profiles.{profile}.reference")
    W = p(f"profiles.{profile}.weights")
    tot = 0.0
    for c in TARGETS:
        tot += float(W[c])
    W = {c: float(W[c]) / tot for c in TARGETS}
    g = {c: 1.0 for c in TARGETS}
    if not ref:
        return g
    rsum = 0.0
    for t in ref:
        rsum += float(ref[t])
    emits = {t: p(f"tags.{t}.emits") for t in ref}
    for _ in range(300):
        cur = {c: 0.0 for c in TARGETS}
        for t in ref:
            norm = 0.0
            for c in TARGETS:
                norm += W[c] * g[c] * float(emits[t][c])
            if norm <= 0.0:
                continue
            for c in TARGETS:
                cur[c] += (float(ref[t]) / rsum) * W[c] * g[c] * float(emits[t][c]) / norm
        for c in TARGETS:
            if cur[c] > 0.0 and W[c] > 0.0:
                g[c] = g[c] * W[c] / cur[c]
    # a fixed scale, so that the numbers are comparable between profiles
    top = max(g.values())
    return {c: g[c] / top for c in TARGETS}


def tag_mix(p: Parameters, params: PlacementParams, use: str, g: Mapping[str, float]) -> dict:
    """Shares of a source's target budget over the **selected** classes.
    ``realistic``: profile weight x centred multiplier x the tag's emission;
    ``balanced``: equal; a mapping: its weights. The far field takes the
    profile's weights alone."""
    sel = list(params.classes)
    if not sel:
        return {}
    custom = params.weights()
    if params.mix == "balanced":
        raw = {c: 1.0 for c in sel}
    elif custom is not None:
        raw = {c: float(custom.get(c, 0.0)) for c in sel}
    else:
        W = p(f"profiles.{params.profile}.weights")
        if use == FAR_FIELD:
            raw = {c: float(W[c]) for c in sel}
        else:
            e = p(f"tags.{use}.emits")
            raw = {c: float(W[c]) * float(g[c]) * float(e[c]) for c in sel}
    tot = 0.0
    for c in sel:
        tot += raw[c]
    if tot <= 0.0:
        return {}
    return {c: raw[c] / tot for c in sel if raw[c] > 0.0}


def target_share(p: Parameters, params: PlacementParams, use: str) -> float:
    """Share of targets among a source's sonar-size objects: the profile's,
    with its odds divided by the tag's relative emission of debris (the
    tag's over the mean of the profile's reference composition of sources).
    On the reference composition it gives the profile's share; a source
    that sheds rubble (a breakwater toe) has fewer targets, a fishing
    ground more, never 0 or 1. Switching the debris off
    (``clutter.debris: false``) removes the debris part and leaves the
    target part as it is."""
    t = float(p(f"profiles.{params.profile}.target_share"))
    if use == FAR_FIELD or params.mix != "realistic":
        return t
    ref = p(f"profiles.{params.profile}.reference")
    if not ref:
        return t
    mean, rs = 0.0, 0.0
    for tag in ref:
        mean += float(ref[tag]) * float(p(f"tags.{tag}.debris"))
        rs += float(ref[tag])
    mean = mean / rs if rs > 0.0 else 1.0
    rel = float(p(f"tags.{use}.debris")) / mean if mean > 0.0 else 1.0
    if rel <= 0.0:
        return 0.98
    odds = t / (1.0 - t) / rel
    return min(max(odds / (1.0 + odds), 0.02), 0.98)


# ------------------------------------------------------------------- sources
#: features of these kinds are checked first for ties in nothing: the order
#: of the sources is the order of their ids (a stable, meaningless order)
def build_sources(env: Environment, params: PlacementParams, p: Parameters, allowed: np.ndarray,
                  seed: int = 0) -> tuple[list, dict]:
    """The sources of an environment for a placement, with their budgets and
    kernels, and a report (areas, densities). ``allowed``: bool raster, where
    an object's centre may fall."""
    ny, nx = env.shape
    xs, ys = env.centres()
    cell = float(env.cell_m)
    flat_allowed = allowed.ravel()
    level = int(params.density_level)
    lam = level_density(p, level)
    ret = retention(p, level)
    g = centred_affinity(p, params.profile)
    n_reach = np.zeros(ny * nx, dtype=np.int32)
    out: list[Source] = []
    for f in sorted(env.features, key=lambda q: q.id):
        use = use_of(p, params, f)
        if use == "none":
            continue
        if use not in p.tags:
            raise ValueError(f"feature {f.id!r}: no use {use!r}")
        r = reach_m(p, use, f.geometry)
        k = p(f"tags.{use}.kernel")
        if k["near"] == "uniform" and float(k.get("scale_m", 0.0)) == 0.0 and f.geometry != "polygon":
            raise ValueError(f"feature {f.id!r}: the use {use!r} needs an area (a polygon)")
        # the cells within reach: a window round the feature
        x0, y0 = f.points[:, 0].min() - r - f.clear_m - cell, f.points[:, 1].min() - r - f.clear_m - cell
        x1, y1 = f.points[:, 0].max() + r + f.clear_m + cell, f.points[:, 1].max() + r + f.clear_m + cell
        i0 = int(min(max(np.floor((x0 - env.x0) / cell), 0), nx))
        i1 = int(min(max(np.ceil((x1 - env.x0) / cell), 0), nx))
        j0 = int(min(max(np.floor((y0 - env.y0) / cell), 0), ny))
        j1 = int(min(max(np.ceil((y1 - env.y0) / cell), 0), ny))
        if i1 <= i0 or j1 <= j0:
            cells, dist = np.zeros(0, dtype=np.int64), np.zeros(0)
        else:
            X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1])
            d = np.maximum(f.distance(X, Y) - float(f.clear_m), 0.0)
            jj, ii = np.nonzero((d <= r) & allowed[j0:j1, i0:i1])
            cells = (jj + j0).astype(np.int64) * nx + (ii + i0)
            dist = d[jj, ii]
        n_reach[cells] += 1
        out.append(Source(id=f.id, feature=f, use=use, activity=activity_of(p, params, f, use),
                          cells=cells, dist=dist))
    report = {"level": level, "density_per_ha": lam, "retention": ret, "sources": []}
    area = cell * cell / 1e4
    band = 0.0
    for s in out:
        share = 0.0
        if len(s.cells):
            share = float(np.cumsum(1.0 / n_reach[s.cells])[-1])
        s.owned_ha = share * area
        s.reach_ha = len(s.cells) * area
        band += s.owned_ha
        s.budget = lam * s.owned_ha * s.activity / ret
        s.target_share = target_share(p, params, s.use)
        s.mix = tag_mix(p, params, s.use, g)
        cx, cy = xs[s.cells % nx], ys[s.cells // nx]
        s.pdf_cum = kernel_cumulative(p, s.use, s.dist, patch_field(p, s.use, s.feature, cx, cy, seed, area))
        s.groups = p(f"tags.{s.use}.groups") if params.groups else {}
    # the far field: the allowed water no source reaches
    far = np.nonzero(flat_allowed & (n_reach == 0))[0]
    share_far = float(p("far_field_share"))
    if len(far) and share_far > 0.0:
        fs = Source(id=FAR_FIELD, feature=None, use=FAR_FIELD, activity=1.0, cells=far,
                    dist=np.zeros(len(far)))
        fs.owned_ha = fs.reach_ha = len(far) * area
        fs.budget = lam * share_far * fs.owned_ha
        fs.target_share = target_share(p, params, FAR_FIELD)
        fs.mix = tag_mix(p, params, FAR_FIELD, g)
        fs.pdf_cum = np.cumsum(np.ones(len(far)))
        out.append(fs)
    allowed_ha = float(np.count_nonzero(flat_allowed)) * area
    report.update(active_band_ha=band, allowed_ha=allowed_ha, far_field_ha=float(len(far)) * area,
                  centred_affinity={c: round(g[c], 4) for c in TARGETS})
    for s in out:
        report["sources"].append({"id": s.id, "use": s.use, "kind": s.feature.kind if s.feature else FAR_FIELD,
                                  "activity": s.activity, "owned_ha": s.owned_ha, "reach_ha": s.reach_ha,
                                  "length_m": s.feature.length_m if s.feature else 0.0,
                                  "budget": s.budget, "target_share": s.target_share,
                                  "expected": s.expected})
    return out, report
