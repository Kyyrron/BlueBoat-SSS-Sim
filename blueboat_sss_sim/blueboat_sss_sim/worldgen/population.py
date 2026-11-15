"""Harbour litter population: which objects of the set version 2 a world
holds, in what state, and where (2026-11-15, completion round session 02).

Implements section 5 of ``target_families_and_surrogates.md`` (Version 2):

* **how many**: a density of sonar-size litter per hectare of survey water
  (targets plus ``debris``), a share of targets, a composition
  (``realistic`` weights or ``balanced`` equal counts);
* **where**: an inhomogeneous Poisson pattern, ``lambda(x) = 1 + A sum_k
  a_ck exp(-d_k(x) / L)`` over the hotspot features k (quays, pontoons,
  mooring blocks, a fishing berth, outlets, a breakwater), ``a_ck`` the
  class's affinity to the feature's kind; ``A`` is solved so that the stated
  share of the expected count lies within ``near_m`` of a feature;
* **what**: each instance's size, state, rest pose and burial from its class
  card, the burial from the sediment under it.

Everything drawn is returned as a pinned ``objects.explicit`` entry (every
field stated), so a world never depends on a later change of a class
default, and carries its ground truth in ``params`` (role, state, hotspot,
``nominal_target``). No acoustics here (NC #17): sizes, poses, materials'
names. Every number tagged [DESIGN] is the objects document's own design
value; sources: ``docs/seabed_model.md`` par. 10.

ROS-free, Qt-free; numpy and scipy.ndimage.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from .compounds import FIBRE_OF
from .objects import V2_TARGETS

#: counts per hectare of the three presets [objects document 5.1]
DENSITY_PRESETS = {"sparse": 2.0, "nominal": 10.0, "dense": 30.0}
#: count weights of the realistic mode [objects document 5.2, DESIGN]
REALISTIC = {"tire_car": 0.30, "trap_pot": 0.15, "net_bundle": 0.15, "plastic_box": 0.15,
             "concrete_block": 0.10, "pipe_steel": 0.10, "drum_200l": 0.05}
#: burial preset -> scale of every class range [5.4]; the cap
BURIAL_SCALE = {"low": 0.5, "nominal": 1.0, "high": 1.5}
BURIAL_CAP = 0.9
#: an instance buried deeper than this is "effectively buried": it stays in
#: the ground truth and leaves the detectable-target denominator [5.4]
EFFECTIVELY_BURIED = 0.7
#: smallest largest-dimension of a nominal target [m] (the set's size floor)
NOMINAL_TARGET_MIN_M = 0.5
#: kinds of hotspot feature
FEATURE_KINDS = ("quay", "pontoon", "mooring", "berth", "outlet", "breakwater")
#: class affinity to each kind of feature, 0..1 [5.3, DESIGN]
AFFINITY = {
    "tire_car":       {"quay": 1.0, "pontoon": 0.5, "mooring": 0.2, "berth": 0.5, "outlet": 0.3, "breakwater": 0.3},
    "trap_pot":       {"quay": 0.2, "pontoon": 0.5, "mooring": 1.0, "berth": 1.0, "outlet": 0.0, "breakwater": 0.2},
    "net_bundle":     {"quay": 0.5, "pontoon": 0.7, "mooring": 1.0, "berth": 1.0, "outlet": 0.0, "breakwater": 0.5},
    "plastic_box":    {"quay": 0.3, "pontoon": 0.5, "mooring": 0.7, "berth": 1.0, "outlet": 0.3, "breakwater": 0.0},
    "drum_200l":      {"quay": 1.0, "pontoon": 0.2, "mooring": 0.2, "berth": 0.5, "outlet": 0.3, "breakwater": 0.3},
    "concrete_block": {"quay": 1.0, "pontoon": 0.2, "mooring": 0.2, "berth": 0.5, "outlet": 0.3, "breakwater": 0.3},
    "pipe_steel":     {"quay": 1.0, "pontoon": 0.2, "mooring": 0.2, "berth": 0.5, "outlet": 0.3, "breakwater": 0.3},
    "debris":         {"quay": 0.5, "pontoon": 0.5, "mooring": 0.5, "berth": 0.5, "outlet": 0.7, "breakwater": 1.0},
}
#: (sand, mud) upper ends of the burial range U(0, .) per class and pose [4]
BURIAL_MAX = {
    "plastic_box": (0.15, 0.40), "drum_200l:lying": (0.20, 0.50), "drum_200l:standing": (0.10, 0.30),
    "concrete_block": (0.10, 0.30), "trap_pot": (0.20, 0.40), "pipe_steel": (0.30, 0.60),
    "tire_car": (0.30, 0.60), "net_bundle": (0.50, 0.70), "debris": (0.25, 0.50),
}
#: fibre volume fraction of a net bundle per level of bracket R6 [objects
#: document T7: 0.05-0.3, DESIGN]
NET_VOLUME_FRACTION = {"low": 0.05, "nominal": 0.12, "high": 0.30}
#: tyre codes drawn from: section width [mm], aspect ratio [%], rim [inch]
TYRE_WIDTHS_MM = (165, 175, 185, 195, 205, 215, 225, 235, 245)
TYRE_ASPECTS = (55, 60, 65, 70)
TYRE_RIMS_IN = (13, 14, 15, 16, 17)
#: the intensity grid step [m]
CELL_M = 1.0
#: the least clustering: the intensity at a feature is at least 1 + this
#: times the background [DESIGN] (a small basin lies within ``near_m`` of a
#: quay everywhere, where the share alone asks for no clustering at all)
MIN_AMPLITUDE = 1.0


@dataclass(frozen=True)
class PopulationSpec:
    """What a campaign asks for (``objects:`` of its specification)."""

    density_per_ha: float = 10.0
    target_share: float = 0.5
    composition: Any = "realistic"            # realistic | balanced | {class: weight}
    burial: str = "nominal"                   # low | nominal | high
    decay_m: float = 15.0
    share_near: float = 0.7
    near_m: float = 30.0
    min_target_spacing_m: float = 3.0
    uncertain_materials: str = "nominal"      # bracket R6: low | nominal | high
    #: share of the mirror-faced instances (box, drum, pipe) marked as used,
    #: dented litter (``face_state: used``), the others ``new``. None: no
    #: per-object state, the sonar profile's face geometry for all.
    face_used_share: float | None = None

    def __post_init__(self) -> None:
        if not 0.0 <= self.target_share <= 1.0:
            raise ValueError(f"objects.target_share {self.target_share}: [0, 1]")
        if self.burial not in BURIAL_SCALE:
            raise ValueError(f"objects.burial {self.burial!r}: {sorted(BURIAL_SCALE)}")
        if self.uncertain_materials not in FIBRE_OF:
            raise ValueError(f"objects.uncertain_materials {self.uncertain_materials!r}: {sorted(FIBRE_OF)}")
        if self.face_used_share is not None and not 0.0 <= self.face_used_share <= 1.0:
            raise ValueError(f"face_geometry {self.face_used_share}: new | used | a share in [0, 1]")
        if not 0.0 < self.share_near <= 1.0 or self.decay_m <= 0.0:
            raise ValueError("objects.hotspots: share_within in (0, 1], decay_m > 0")
        if self.density_per_ha < 0.0:
            raise ValueError("objects.density_per_ha < 0")
        weights(self.composition)

    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "PopulationSpec":
        d = dict(d or {})
        known = {"set", "composition", "density_per_ha", "target_share", "burial", "hotspots",
                 "min_target_spacing_m", "uncertain_materials", "face_geometry", "fouling"}
        bad = set(d) - known
        if bad:
            raise KeyError(f"objects: unknown key(s) {sorted(bad)} (known: {sorted(known)})")
        if str(d.get("set", "v2")) != "v2":
            raise ValueError(f"objects.set {d['set']!r}: only 'v2'")
        hs = dict(d.get("hotspots") or {})
        bad = set(hs) - {"decay_m", "share_within_30m", "share_within", "near_m"}
        if bad:
            raise KeyError(f"objects.hotspots: unknown key(s) {sorted(bad)}")
        dens = d.get("density_per_ha", 10.0)
        dens = DENSITY_PRESETS[dens] if isinstance(dens, str) else float(dens)
        return cls(density_per_ha=dens, target_share=float(d.get("target_share", 0.5)),
                   composition=d.get("composition", "realistic"), burial=str(d.get("burial", "nominal")),
                   decay_m=float(hs.get("decay_m", 15.0)),
                   share_near=float(hs.get("share_within_30m", hs.get("share_within", 0.7))),
                   near_m=float(hs.get("near_m", 30.0)),
                   min_target_spacing_m=float(d.get("min_target_spacing_m", 3.0)),
                   uncertain_materials=str(d.get("uncertain_materials", "nominal")),
                   face_used_share=face_share(d.get("face_geometry")))


def face_share(v) -> float | None:
    """``face_geometry`` of a specification: ``new`` (0), ``used`` (1), a
    share of used instances in [0, 1], or nothing / ``profile`` (None)."""
    if v is None or v == "profile":
        return None
    if v == "new":
        return 0.0
    if v == "used":
        return 1.0
    return float(v)


def weights(composition) -> dict[str, float]:
    """Normalised count weights of the target classes."""
    if isinstance(composition, str):
        if composition == "realistic":
            w = dict(REALISTIC)
        elif composition == "balanced":
            w = {c: 1.0 for c in V2_TARGETS}
        else:
            raise ValueError(f"objects.composition {composition!r}: realistic | balanced | a mapping")
    else:
        w = {str(k): float(v) for k, v in dict(composition).items()}
        bad = set(w) - set(V2_TARGETS)
        if bad or not w or min(w.values()) < 0.0 or sum(w.values()) <= 0.0:
            raise ValueError(f"objects.composition: weights >= 0 over {V2_TARGETS}, got {composition}")
    tot = sum(w.values())
    return {c: w.get(c, 0.0) / tot for c in V2_TARGETS}


def apportion(n: int, w: Mapping[str, float], rng: np.random.Generator) -> dict[str, int]:
    """``n`` instances over the classes in proportion to ``w``: the largest
    remainders take the instances the whole parts leave, ties in seeded
    random order. Each count is within one of ``n w``."""
    names = list(w)
    exact = np.array([n * w[c] for c in names])
    base = np.floor(exact + 1e-12).astype(int)
    left = int(n - base.sum())
    if left > 0:
        rem = exact - base
        order = np.lexsort((rng.random(len(names)), -np.round(rem, 9)))
        base[order[:left]] += 1
    return {c: int(k) for c, k in zip(names, base)}


@dataclass
class Feature:
    """A hotspot: a polyline (or a point) in world metres."""

    kind: str
    points: np.ndarray
    name: str = ""

    def __post_init__(self) -> None:
        if self.kind not in FEATURE_KINDS:
            raise ValueError(f"hotspot kind {self.kind!r}: {FEATURE_KINDS}")
        self.points = np.asarray(self.points, dtype=np.float64).reshape(-1, 2)

    def distance(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        p = np.stack([np.asarray(x, float), np.asarray(y, float)], axis=-1)
        if len(self.points) == 1:
            return np.linalg.norm(p - self.points[0], axis=-1)
        best = np.full(p.shape[:-1], np.inf)
        for a, b in zip(self.points[:-1], self.points[1:]):
            ab = b - a
            t = np.clip(((p - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
            best = np.minimum(best, np.linalg.norm(p - (a + t[..., None] * ab), axis=-1))
        return best


@dataclass
class Field:
    """The intensity grid of one world: where objects may fall, and how far
    each cell is from each feature."""

    x: np.ndarray                    # (m,) cell centres, world metres
    y: np.ndarray
    dist: np.ndarray                 # (features, m)
    kinds: list
    names: list
    area_ha: float
    amplitude: float = 0.0
    share_near: float = 0.0
    report: dict = field(default_factory=dict)

    def intensity(self, cls: str) -> np.ndarray:
        w = np.ones(self.x.size)
        if self.dist.shape[0] and self.amplitude > 0.0:
            a = np.array([AFFINITY[cls][k] for k in self.kinds])
            w = w + self.amplitude * (a[:, None] * np.exp(-self.dist / self._decay)).sum(axis=0)
        return w


def build_field(allowed: np.ndarray, grid, features: list, spec: PopulationSpec,
                counts: Mapping[str, int]) -> Field:
    """The intensity grid over the cells of ``allowed`` (bool raster of the
    world grid), and the amplitude ``A`` that puts ``spec.share_near`` of
    the expected count within ``spec.near_m`` of a feature (never under
    :data:`MIN_AMPLITUDE` when the world has features)."""
    res = float(grid.resolution)
    step = max(int(round(CELL_M / res)), 1)
    xmin, ymin, _, _ = grid.extent
    ny, nx = allowed.shape
    sub = allowed[step // 2::step, step // 2::step]
    jj, ii = np.nonzero(sub)
    x = xmin + (ii * step + step // 2 + 0.5) * res
    y = ymin + (jj * step + step // 2 + 0.5) * res
    feats = [f if isinstance(f, Feature) else Feature(**f) for f in features]
    dist = (np.array([f.distance(x, y) for f in feats]) if feats and x.size
            else np.zeros((0, x.size)))
    fld = Field(x, y, dist, [f.kind for f in feats], [f.name or f.kind for f in feats],
                float(allowed.sum()) * res * res / 1e4)
    fld._decay = float(spec.decay_m)
    if not feats or not x.size:
        return fld
    near = dist.min(axis=0) <= spec.near_m
    n_tot = max(sum(counts.values()), 1)

    def share(A: float) -> float:
        fld.amplitude = A
        s = 0.0
        for c, n in counts.items():
            if n:
                w = fld.intensity(c)
                s += n * float(w[near].sum() / w.sum())
        return s / n_tot
    lo, hi = 0.0, 1.0
    s0 = share(0.0)
    if s0 >= spec.share_near or not near.any() or near.all():
        A = MIN_AMPLITUDE
    else:
        while share(hi) < spec.share_near and hi < 1e6:
            hi *= 4.0
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if share(mid) < spec.share_near else (lo, mid)
        A = max(0.5 * (lo + hi), MIN_AMPLITUDE)
    fld.amplitude = float(A)
    fld.share_near = float(share(A))
    fld.report = {"amplitude": float(A), "share_within_near_expected": fld.share_near,
                  "share_within_near_uniform": float(s0), "near_m": float(spec.near_m),
                  "decay_m": float(spec.decay_m)}
    return fld


# ----------------------------------------------------------------- instances
def _u(rng, a, b) -> float:
    return float(rng.uniform(a, b))


def _instance(cls: str, rng: np.random.Generator, spec: PopulationSpec) -> dict:
    """Size, state and pose of one instance of ``cls`` (its class card):
    ``{length, width, proud_height, material, params, pose}``; the pose is
    (rest name, roll, pitch) of the class's canonical frame."""
    r6 = spec.uncertain_materials
    p: dict[str, Any] = {"spec": 2}
    pose = ("canonical", 0.0, 0.0)
    if cls == "plastic_box":
        L = _u(rng, 0.50, 0.75)
        W, H = L * _u(rng, 0.55, 0.70), _u(rng, 0.22, 0.35)
        p["wall_mm"] = round(_u(rng, 2.0, 4.0), 2)
        if rng.random() < 0.3:
            p["state"], p["fill"] = "part_filled", round(_u(rng, 0.05, 0.30), 3)
        else:
            p["state"] = "flooded"
        u = rng.random()
        if u < 0.5:
            pose = ("upright", 0.0, 0.0)
        elif u < 0.8:
            pose = ("inverted", np.pi, 0.0)
        elif rng.random() < 0.7:
            pose = ("on_long_side", np.pi / 2.0, 0.0)
        else:
            pose = ("on_end", 0.0, np.pi / 2.0)
        mat = "pp_shell"
    elif cls == "drum_200l":
        L = W = 0.585
        H = 0.890
        p["wall_mm"] = round(_u(rng, 0.9, 1.6), 2)
        if rng.random() < 0.6:
            pose, p["state"] = ("lying", np.pi / 2.0, 0.0), "flooded"
        else:
            pose = ("standing", 0.0, 0.0)
            if rng.random() < 0.3:
                p["state"], p["fill"] = "sediment_inside", round(_u(rng, 0.1, 0.5), 3)
            else:
                p["state"] = "flooded"
        mat = "steel_shell"
    elif cls == "concrete_block":
        d = sorted([_u(rng, 0.5, 1.0), _u(rng, 0.4, 0.8), _u(rng, 0.2, 0.6)], reverse=True)
        L, W, H = d
        p["bevel_mm"] = round(_u(rng, 0.0, 30.0), 1)
        if rng.random() >= 0.8:
            pose = ("on_long_side", np.pi / 2.0, 0.0) if rng.random() < L / (L + W) else ("on_end", 0.0, np.pi / 2.0)
        else:
            pose = ("largest_face_down", 0.0, 0.0)
        mat = "concrete"
    elif cls == "trap_pot":
        if rng.random() < 0.5:
            p["variant"] = "us_wire"
            L = W = 0.61
            H = _u(rng, 0.51, 0.61)
            p["wire_mm"] = 1.2 if rng.random() < 0.5 else 1.6
        else:
            p["variant"], p["r6"] = "jp_frame", r6
            L, W, H = _u(rng, 0.95, 1.00), _u(rng, 0.40, 0.68), _u(rng, 0.40, 0.45)
        pose = ("upright", 0.0, 0.0) if rng.random() < 0.8 else ("on_side", np.pi / 2.0, 0.0)
        mat = "steel_rough"
    elif cls == "pipe_steel":
        L = _u(rng, 1.0, 3.0)
        W = H = _u(rng, 0.10, 0.30)
        p["wall_mm"] = round(_u(rng, 3.0, 8.0), 1)
        p["state"] = "flooded"
        pose = ("lying", 0.0, 0.0)
        mat = "steel_shell"
    elif cls == "tire_car":
        while True:
            ws = int(rng.choice(TYRE_WIDTHS_MM))
            ar = int(rng.choice(TYRE_ASPECTS))
            rim = int(rng.choice(TYRE_RIMS_IN))
            D = rim * 0.0254 + 2.0 * ws * 1e-3 * ar / 100.0
            if 0.55 <= D <= 0.80:
                break
        L = W = D
        H = ws * 1e-3
        p.update(tyre_code=f"{ws}/{ar}R{rim}", rim_m=round(rim * 0.0254, 4), r6=r6)
        pose = ("flat", 0.0, 0.0) if rng.random() < 0.85 else ("on_tread", np.pi / 2.0, 0.0)
        mat = "rubber"
    elif cls == "net_bundle":
        p["r6"] = r6
        if rng.random() < 0.7:
            L = _u(rng, 0.5, 2.0)
            W = min(_u(rng, 0.4, 1.5), L)
            H = min(_u(rng, 0.1, 0.5), 0.5 * W)
            p.update(variant="tangle", strand_mm=round(_u(rng, 10.0, 20.0), 1),
                     volume_fraction=NET_VOLUME_FRACTION[r6])
        else:
            L = W = _u(rng, 0.3, 0.8)
            d = _u(rng, 12.0, 16.0)
            H = d * 1e-3 * int(rng.integers(2, 9))
            p.update(variant="rope_coil", strand_mm=round(d, 1))
        pose = ("flattened", 0.0, 0.0)
        mat = FIBRE_OF[r6]
    elif cls == "debris":
        L = _u(rng, 0.2, 1.2)
        W = min(_u(rng, 0.15, 0.8), L)
        H = min(_u(rng, 0.05, 0.35), W)
        mat = str(rng.choice(["steel_rough", "concrete", "generic"], p=[0.5, 0.4, 0.1]))
        pose = ("flat", 0.0, 0.0)
    else:
        raise KeyError(f"population: no class card for {cls!r}")
    if cls in ("plastic_box", "drum_200l", "pipe_steel") and spec.face_used_share is not None:
        p["face_state"] = "used" if rng.random() < spec.face_used_share else "new"
    return {"length": float(L), "width": float(W), "proud_height": float(H),
            "material": mat, "params": p, "pose": pose}


def _burial(cls: str, pose: str, mud: bool, spec: PopulationSpec, rng) -> float:
    key = f"{cls}:{pose}" if f"{cls}:{pose}" in BURIAL_MAX else cls
    hi = BURIAL_MAX[key][1 if mud else 0]
    return float(min(_u(rng, 0.0, hi) * BURIAL_SCALE[spec.burial], BURIAL_CAP))


def populate(grid, allowed: np.ndarray, features: list, spec: PopulationSpec,
             rng: np.random.Generator, *, is_mud=None, sediment_at=None,
             materials=None, occupied=()) -> tuple[list, dict]:
    """The litter of one world.

    ``allowed``: bool raster of the world grid, True where an object's
    centre may fall (survey water inside the depth domain, clear of
    structures). ``features``: the hotspots (:class:`Feature` or its
    mapping). ``is_mud(x, y) -> bool``: whether the sediment there counts as
    mud for burial; ``sediment_at(x, y) -> name``: its class (the fill of a
    part-filled container). ``occupied``: ``(x, y, radius)`` of instances
    placed before (validation instances): targets keep their spacing from
    them.

    Returns ``(explicit entries, report)``: the entries, targets first then
    ``debris``, in the order drawn (stable per seed); the report has the
    counts asked for and placed, the amplitude and the realised share near a
    hotspot."""
    from .materials import MaterialLibrary
    lib = materials if materials is not None else MaterialLibrary()
    features = [f if isinstance(f, Feature) else Feature(**f) for f in features]
    res = float(grid.resolution)
    xmin, ymin, _, _ = grid.extent
    area_ha = float(allowed.sum()) * res * res / 1e4
    n = int(round(spec.density_per_ha * area_ha))
    n_t = int(round(spec.target_share * n))
    counts = apportion(n_t, weights(spec.composition), rng)
    counts["debris"] = n - n_t
    fld = build_field(allowed, grid, features, spec, counts)
    report = {"area_ha": area_ha, "asked": dict(counts), "placed": {}, "hotspots": dict(fld.report),
              "features": [{"kind": k, "name": nm} for k, nm in zip(fld.kinds, fld.names)]}
    if not fld.x.size or n == 0:
        return [], report
    from scipy import ndimage
    clear = ndimage.distance_transform_edt(allowed) * res       # distance to a forbidden cell
    entries: list = []
    placed_xy: list = [(float(x), float(y), True, float(r)) for x, y, r in occupied]   # (x, y, target, radius)
    half = 0.5 * CELL_M
    for cls in list(V2_TARGETS) + ["debris"]:
        w = fld.intensity(cls)
        cdf = np.cumsum(w)
        got = 0
        for _k in range(counts.get(cls, 0)):
            inst = _instance(cls, rng, spec)
            rad = 0.5 * float(np.hypot(inst["length"], inst["width"]))
            target = cls != "debris"
            for _try in range(400):
                c = int(np.searchsorted(cdf, rng.random() * cdf[-1]))
                x = float(fld.x[c] + rng.uniform(-half, half))
                y = float(fld.y[c] + rng.uniform(-half, half))
                i = int(np.clip((x - xmin) / res, 0, allowed.shape[1] - 1))
                j = int(np.clip((y - ymin) / res, 0, allowed.shape[0] - 1))
                if not allowed[j, i] or clear[j, i] < rad + 0.3:
                    continue
                ok = True
                for px, py, pt, pr in placed_xy:
                    gap = spec.min_target_spacing_m if (target and pt) else 1.0
                    if np.hypot(x - px, y - py) < max(gap, rad + pr + 0.3):
                        ok = False
                        break
                if ok:
                    break
            else:
                continue                                        # no room: reported, not forced
            mud = bool(is_mud(x, y)) if is_mud is not None else False
            name, roll, pitch = inst["pose"]
            p = dict(inst["params"])
            b = _burial(cls, name, mud, spec, rng)
            if "fill" in p:
                p["fill_material"] = str(sediment_at(x, y)) if sediment_at is not None else "sand"
            hot, hd = "", float("inf")
            for k, f in enumerate(features):
                dk = float(f.distance(np.array([x]), np.array([y]))[0])
                if dk < hd:
                    hot, hd = fld.names[k], dk
            p.update(role="target" if target else "clutter",
                     nominal_target=bool(target and max(inst["length"], inst["width"],
                                                        inst["proud_height"]) >= NOMINAL_TARGET_MIN_M),
                     hotspot=hot if hd <= spec.near_m else "",
                     hotspot_distance_m=round(hd, 2) if np.isfinite(hd) else None,
                     pose=name, sediment="mud" if mud else "sand")
            mat = lib.get(inst["material"])
            e = {"type": cls, "x": round(x, 3), "y": round(y, 3),
                 "yaw": round(float(rng.uniform(0.0, 2.0 * np.pi)), 5),
                 "length": round(inst["length"], 4), "width": round(inst["width"], 4),
                 "proud_height": round(inst["proud_height"], 4), "burial": round(b, 4),
                 "fill_fraction": 0.07 if cls == "trap_pot" else 1.0,
                 "reflectivity": float(mat.reflectivity), "material": inst["material"], "params": p}
            if roll != 0.0 or pitch != 0.0:
                e.update(roll_deg=round(float(np.degrees(roll)), 3),
                         pitch_deg=round(float(np.degrees(pitch)), 3), rest=name)
            entries.append(e)
            placed_xy.append((x, y, target, rad))
            got += 1
        report["placed"][cls] = got
    xy = np.array([(e["x"], e["y"]) for e in entries]).reshape(-1, 2)
    if fld.dist.shape[0] and len(xy):
        dmin = np.min([f.distance(xy[:, 0], xy[:, 1]) for f in features], axis=0)
        report["hotspots"]["share_within_near_placed"] = float(np.mean(dmin <= spec.near_m))
    return entries, report
