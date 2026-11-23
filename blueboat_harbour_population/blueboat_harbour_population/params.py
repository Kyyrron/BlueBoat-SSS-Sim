"""The ``placement:`` block: what the user asks for, with the same names and
values in a specification file and on World Builder Screen 5.

    placement:
      mode: mission_like          # manual | mission_like | legacy
      profile: fishing_harbour    # fishing_harbour | trap_harbour | city_harbour | marina | working_port | generic | auto
      classes: [tire_car, ...]    # omitted = the seven target classes
      mix: realistic              # realistic | balanced | {tire_car: 0.4, ...}
      density_level: 5            # 1..10; 5 = a typical documented harbour
      seed: 120345                # the placement seed (the world's own seed seeds the seabed)
      split: dataset              # development | dataset | tuning | evaluation
      harbour: provincetown       # optional; checked against the harbour split
      clutter: {debris: true, small_litter: true}
      structures: auto            # auto | keep | none
      feature_tags: {quay_n: fender_line}          # use of a feature, by its id
      feature_activity: {quay_n: 2.0}              # relative emission of a feature
      features: [{id: lane_1, kind: lane, points: [[..], [..]], use: traffic_lane}]   # features the environment lacks
      advanced: {burial: nominal, groups: true, redistribution: false, min_spacing_m: 3.0, count: poisson,
                 outside_band: false, uncertain_materials: nominal, face_geometry: new}
      split_override: "reason"    # lets a seed or harbour of another split through, and is recorded

Unknown keys raise by name. Pure Python.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .data import TARGETS, Parameters, default_parameters

MODES = ("manual", "mission_like", "legacy")
STRUCTURES = ("auto", "keep", "none")
BURIAL = ("low", "nominal", "high")
R6 = ("low", "nominal", "high")
COUNT = ("poisson", "fixed")
_KEYS = {"mode", "profile", "classes", "mix", "density_level", "seed", "split", "harbour", "clutter",
         "structures", "feature_tags", "feature_activity", "features", "advanced", "split_override",
         "instances"}
_ADVANCED = {"burial": "nominal", "groups": True, "redistribution": False, "min_spacing_m": 3.0,
             "count": "poisson", "outside_band": False, "uncertain_materials": "nominal",
             "face_geometry": "new"}
_CLUTTER = {"debris": True, "small_litter": True}
#: largest relative emission a feature may be given
ACTIVITY_MAX = 5.0


def _face_share(v) -> float | None:
    """``face_geometry``: ``new`` (0), ``used`` (1), a share of used
    instances in [0, 1], or ``profile`` / nothing (None: no per-object state)."""
    if v is None or v == "profile":
        return None
    if v == "new":
        return 0.0
    if v == "used":
        return 1.0
    f = float(v)
    if not 0.0 <= f <= 1.0:
        raise ValueError(f"placement.advanced.face_geometry {v!r}: new | used | profile | a share in [0, 1]")
    return f


@dataclass(frozen=True)
class PlacementParams:
    """A validated ``placement:`` block."""

    mode: str = "mission_like"
    profile: str = "auto"
    classes: tuple = TARGETS
    mix: Any = "realistic"                    # "realistic" | "balanced" | ((class, weight), ...)
    density_level: int = 5
    seed: int = 1
    split: str = "development"
    harbour: str = ""
    debris: bool = True
    small_litter: bool = True
    structures: str = "auto"
    feature_tags: tuple = ()                  # ((feature id, use), ...) sorted
    feature_activity: tuple = ()              # ((feature id, activity), ...) sorted
    features: tuple = ()                      # feature mappings added to the environment
    burial: str = "nominal"
    groups: bool = True
    redistribution: bool = False
    min_spacing_m: float = 3.0
    count: str = "poisson"
    outside_band: bool = False
    uncertain_materials: str = "nominal"
    face_used_share: float | None = 0.0
    split_override: str = ""
    instances: tuple = ()                     # mode manual: explicit instances

    # ------------------------------------------------------------------ I/O
    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None, parameters: Parameters | None = None) -> "PlacementParams":
        p = parameters or default_parameters()
        d = dict(d or {})
        bad = set(d) - _KEYS
        if bad:
            raise KeyError(f"placement: unknown key(s) {sorted(bad)} (known: {sorted(_KEYS)})")
        mode = str(d.get("mode", "mission_like"))
        if mode not in MODES:
            raise ValueError(f"placement.mode {mode!r}: {' | '.join(MODES)}")
        profile = str(d.get("profile", "auto"))
        if profile not in p.profiles:
            raise ValueError(f"placement.profile {profile!r}: {' | '.join(p.profiles)}")
        classes = d.get("classes")
        classes = list(TARGETS) if classes is None else [str(c) for c in classes]
        unknown = [c for c in classes if c not in TARGETS]
        if unknown:
            raise ValueError(f"placement.classes {unknown}: target classes are {list(TARGETS)}")
        classes = tuple(c for c in TARGETS if c in classes)          # the set's own order, no repeats
        mix = d.get("mix", "realistic")
        if isinstance(mix, str):
            if mix not in ("realistic", "balanced"):
                raise ValueError(f"placement.mix {mix!r}: realistic | balanced | a mapping class -> weight")
        else:
            w = {str(k): float(v) for k, v in dict(mix).items()}
            unknown = [c for c in w if c not in TARGETS]
            if unknown or not w or min(w.values()) < 0.0 or sum(w.values()) <= 0.0:
                raise ValueError(f"placement.mix: weights >= 0 over the target classes, got {dict(mix)}")
            mix = tuple((c, w[c]) for c in TARGETS if c in w)
        level = d.get("density_level", 5)
        if isinstance(level, bool) or int(level) != level or not 1 <= int(level) <= 10:
            raise ValueError(f"placement.density_level {level!r}: a whole number from 1 to 10")
        clutter = dict(d.get("clutter") or {})
        bad = set(clutter) - set(_CLUTTER)
        if bad:
            raise KeyError(f"placement.clutter: unknown key(s) {sorted(bad)} (known: {sorted(_CLUTTER)})")
        clutter = {**_CLUTTER, **clutter}
        structures = str(d.get("structures", "auto"))
        if structures not in STRUCTURES:
            raise ValueError(f"placement.structures {structures!r}: {' | '.join(STRUCTURES)}")
        adv = dict(d.get("advanced") or {})
        bad = set(adv) - set(_ADVANCED)
        if bad:
            raise KeyError(f"placement.advanced: unknown key(s) {sorted(bad)} (known: {sorted(_ADVANCED)})")
        adv = {**_ADVANCED, **adv}
        if adv["burial"] not in BURIAL:
            raise ValueError(f"placement.advanced.burial {adv['burial']!r}: {' | '.join(BURIAL)}")
        if adv["count"] not in COUNT:
            raise ValueError(f"placement.advanced.count {adv['count']!r}: {' | '.join(COUNT)}")
        if adv["uncertain_materials"] not in R6:
            raise ValueError(f"placement.advanced.uncertain_materials {adv['uncertain_materials']!r}: "
                             f"{' | '.join(R6)}")
        if not 0.5 <= float(adv["min_spacing_m"]) <= 20.0:
            raise ValueError("placement.advanced.min_spacing_m: 0.5 to 20 m")
        tags = {str(k): str(v) for k, v in dict(d.get("feature_tags") or {}).items()}
        known_tags = set(p.tags) | {"none"}
        for fid, use in tags.items():
            if use not in known_tags:
                raise ValueError(f"placement.feature_tags.{fid}: no use {use!r} (one of {sorted(known_tags)})")
        act = {str(k): float(v) for k, v in dict(d.get("feature_activity") or {}).items()}
        for fid, a in act.items():
            if not 0.0 <= a <= ACTIVITY_MAX:
                raise ValueError(f"placement.feature_activity.{fid} {a}: 0 to {ACTIVITY_MAX}")
        feats = tuple(_freeze(f) for f in (d.get("features") or []))
        return cls(mode=mode, profile=profile, classes=classes, mix=mix, density_level=int(level),
                   seed=int(d.get("seed", 1)), split=str(d.get("split", "development")),
                   harbour=str(d.get("harbour", "") or ""), debris=bool(clutter["debris"]),
                   small_litter=bool(clutter["small_litter"]), structures=structures,
                   feature_tags=tuple(sorted(tags.items())), feature_activity=tuple(sorted(act.items())),
                   features=feats, burial=str(adv["burial"]), groups=bool(adv["groups"]),
                   redistribution=bool(adv["redistribution"]), min_spacing_m=float(adv["min_spacing_m"]),
                   count=str(adv["count"]), outside_band=bool(adv["outside_band"]),
                   uncertain_materials=str(adv["uncertain_materials"]),
                   face_used_share=_face_share(adv["face_geometry"]),
                   split_override=str(d.get("split_override", "") or ""),
                   instances=tuple(_freeze(i) for i in (d.get("instances") or [])))

    def to_dict(self) -> dict:
        """The block as a specification would write it (plain values)."""
        face = self.face_used_share
        d: dict[str, Any] = {
            "mode": self.mode, "profile": self.profile, "classes": list(self.classes),
            "mix": self.mix if isinstance(self.mix, str) else {c: w for c, w in self.mix},
            "density_level": int(self.density_level), "seed": int(self.seed), "split": self.split,
            "clutter": {"debris": bool(self.debris), "small_litter": bool(self.small_litter)},
            "structures": self.structures,
            "advanced": {"burial": self.burial, "groups": bool(self.groups),
                         "redistribution": bool(self.redistribution),
                         "min_spacing_m": float(self.min_spacing_m), "count": self.count,
                         "outside_band": bool(self.outside_band),
                         "uncertain_materials": self.uncertain_materials,
                         "face_geometry": ("profile" if face is None else "new" if face == 0.0
                                           else "used" if face == 1.0 else float(face))},
        }
        if self.harbour:
            d["harbour"] = self.harbour
        if self.feature_tags:
            d["feature_tags"] = dict(self.feature_tags)
        if self.feature_activity:
            d["feature_activity"] = dict(self.feature_activity)
        if self.features:
            d["features"] = [_thaw(f) for f in self.features]
        if self.split_override:
            d["split_override"] = self.split_override
        if self.instances:
            d["instances"] = [_thaw(i) for i in self.instances]
        return d

    # ------------------------------------------------------------ shortcuts
    def weights(self) -> dict | None:
        """The custom class weights, or None for ``realistic`` / ``balanced``."""
        return None if isinstance(self.mix, str) else dict(self.mix)

    def tag_of(self, feature_id: str) -> str | None:
        return dict(self.feature_tags).get(feature_id)

    def activity_of(self, feature_id: str) -> float | None:
        return dict(self.feature_activity).get(feature_id)


def _freeze(obj):
    """A hashable copy of plain YAML data (mappings as sorted item tuples)."""
    if isinstance(obj, Mapping):
        return ("__map__", tuple(sorted((str(k), _freeze(v)) for k, v in obj.items())))
    if isinstance(obj, (list, tuple)):
        return tuple(_freeze(v) for v in obj)
    return obj


def _thaw(obj):
    if isinstance(obj, tuple) and len(obj) == 2 and obj[0] == "__map__":
        return {k: _thaw(v) for k, v in obj[1]}
    if isinstance(obj, tuple):
        return [_thaw(v) for v in obj]
    return obj


def harbour_profile(harbour: str, parameters: Parameters | None = None) -> str | None:
    """The default profile of a candidate harbour (None: not listed)."""
    p = parameters or default_parameters()
    rec = p("harbours").get(str(harbour))
    return rec["profile"] if rec else None
