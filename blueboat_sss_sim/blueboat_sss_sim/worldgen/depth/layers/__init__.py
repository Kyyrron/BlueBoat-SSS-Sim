"""Layer kinds for the depth & seabed editor (Qt-free).

A *layer* is one YAML-plain dict::

    {"id": "L3", "kind": "channel", "name": "River", "enabled": True,
     "opacity": 1.0, "blend": "deepen",
     "mask": None | {mask dict} | [mask dicts],        # worldgen.depth.masks
     "geometry": {"type": "polyline", "points": [[x, y], ...]},
     "params": {...},                                   # the kind's schema
     "targets": {"height": True, "material": True},     # channels it writes
     "seed": None | int}

Every kind is a :class:`LayerSpec` registered in :data:`LAYER_KINDS`: a
function ``fn(grid, geometry, params, rng, ctx) -> LayerOutput``, a label,
a family (for the GUI's add-layer menu), 7-tuple schema rows *with hover
help* (the same rows ``builder.forms.SchemaForm`` renders for the depth
presets), the geometry types it accepts, the channels it can write, the
literature it rests on (``reference``, see ``docs/seabed_model.md``) and
its default blend mode. Adding a kind is one registration: the editor's
menu, property form, compositor and tests pick it up.

The compositor (:mod:`..compose`) evaluates the stack in order; the first
layer is always a ``source`` (today's preset / depth file / recording)
and draws from the terrain RNG stream exactly as before, so a stack with
nothing above its source compiles bit-identically to the legacy path.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from ....core.geo import GeoAnchor
from ....core.types import GridSpec, Wall
from ..masks import normalize_mask, rebase_mask
from ..paint import normalize_strokes

FAMILIES = ("source", "gradient", "landform", "relief", "adjust", "material",
            "paint")
BLEND_MODES = ("normal", "add", "deepen", "shoal")
CHANNELS = ("height", "material", "gain")
GEOMETRY_TYPES = ("polygon", "polyline", "ellipse", "rect", "axis", "radial",
                  "strokes")
LAYER_KEYS = {"id", "kind", "name", "enabled", "opacity", "blend", "mask",
              "geometry", "params", "targets", "seed"}
MATERIAL_CHOICE = "choice:sand|mud|gravel|rocks|seagrass"


@dataclass
class LayerContext:
    """What a layer function may look at besides its own inputs."""

    grid: GridSpec
    height: np.ndarray | None          # the composite below this layer
    walls: list[Wall]
    anchor: GeoAnchor | None
    is_base: bool = False              # layer 0 (the source)
    seed: int = 0
    mask: np.ndarray | None = None     # the layer's own user mask, built
    labels: np.ndarray | None = None   # wall regions, lazily built
    samples_xy: np.ndarray | None = None   # last positioned source
    reach_m: float = 15.0
    svlog_path: str | None = None      # the recording below (svlog source)


@dataclass
class LayerOutput:
    """What a layer function returns; every field optional.

    ``height`` is an absolute target field (z, negative down) and
    ``delta`` an increment on the composite below; a kind returns one of
    them. ``material`` is ``(ids, names)`` with ``ids`` int16 and ``-1``
    where the layer says nothing; ``material_edge`` tells the compositor
    whether a partial mask weight stamps by threshold (``hard``) or by
    seeded dithering (``dithered``). ``gain_db`` is a scalar or field.
    ``default_mask`` is the layer's own extent (multiplied with the user
    mask). ``info`` is merged into the composite's info (survey quality).
    """

    height: np.ndarray | None = None
    delta: np.ndarray | None = None
    material: tuple[np.ndarray, list[str]] | None = None
    material_edge: str = "dithered"
    gain_db: float | np.ndarray | None = None
    default_mask: np.ndarray | None = None
    #: Blend mode this *result* must be composited with, overriding the
    #: layer's configured one. A kind sets it only when its own parameters
    #: invert what the configured mode means -- a dredged basin whose
    #: design floor is above the bed cannot express itself through a
    #: minimum, and would otherwise be a silent no-op.
    blend: str | None = None
    info: dict[str, Any] | None = None


LayerFn = Callable[[GridSpec, dict | None, dict, np.random.Generator,
                    LayerContext], LayerOutput]


@dataclass(frozen=True)
class LayerSpec:
    kind: str
    fn: LayerFn
    label: str
    family: str
    schema: tuple                       # 7-tuple rows with help
    geometry: tuple[str, ...]           # accepted geometry types; () = none
    channels: tuple[str, ...]
    reference: str                      # docs/seabed_model.md entries
    consequence: str = ""               # one sentence on the sonar image
    blend: str = "normal"
    validate_params: Callable[[Mapping[str, Any]], dict] | None = None


LAYER_KINDS: dict[str, LayerSpec] = {}


def register(spec: LayerSpec) -> LayerSpec:
    if spec.family not in FAMILIES:
        raise ValueError(f"layer kind {spec.kind}: family '{spec.family}' "
                         f"unknown {FAMILIES}")
    if spec.blend not in BLEND_MODES:
        raise ValueError(f"layer kind {spec.kind}: blend '{spec.blend}'")
    for g in spec.geometry:
        if g not in GEOMETRY_TYPES:
            raise ValueError(f"layer kind {spec.kind}: geometry '{g}'")
    for c in spec.channels:
        if c not in CHANNELS:
            raise ValueError(f"layer kind {spec.kind}: channel '{c}'")
    for row in spec.schema:
        if len(row) < 7 or not str(row[6]).strip().endswith("."):
            raise ValueError(f"layer kind {spec.kind}: schema row {row[0]!r} "
                             "needs a one-sentence help ending with '.'")
    LAYER_KINDS[spec.kind] = spec
    return spec


# --------------------------------------------------------------- params
def coerce_params(schema: tuple, params: Mapping[str, Any] | None,
                  where: str) -> dict[str, Any]:
    """Schema defaults updated by ``params``; unknown keys raise by name,
    values are coerced to the row's kind (float / int / bool / choice)."""
    params = dict(params or {})
    allowed = {row[0] for row in schema}
    bad = set(params) - allowed
    if bad:
        raise KeyError(f"Unknown parameter(s) in {where}: {sorted(bad)} "
                       f"(accepted: {sorted(allowed)})")
    out: dict[str, Any] = {}
    for key, _label, kind, default, lo, hi, *_rest in schema:
        v = params.get(key, default)
        if kind == "int":
            v = int(v)
        elif kind == "bool":
            v = bool(v)
        elif str(kind).startswith("choice:"):
            v = str(v)
            choices = str(kind).split(":", 1)[1].split("|")
            if v not in choices:
                raise ValueError(f"{where}.{key} '{v}' not in {choices}")
        else:
            v = float(v)
        out[key] = v
    return out


def _pt(p: Any, where: str) -> list[float]:
    try:
        return [float(p[0]), float(p[1])]
    except Exception as exc:
        raise ValueError(f"{where}: expected an [x, y] point, got {p!r}") from exc


def normalize_geometry(g: Mapping[str, Any] | None, accepted: Sequence[str],
                       where: str) -> dict[str, Any] | None:
    """Validate a geometry dict against the kind's accepted types."""
    if not accepted:
        if g:
            raise ValueError(f"{where}: this layer kind takes no geometry")
        return None
    if not g:
        raise ValueError(f"{where}: needs a geometry of type "
                         f"{' | '.join(accepted)}")
    t = str(g.get("type", ""))
    if t not in accepted:
        raise ValueError(f"{where}.type '{t}' not accepted "
                         f"({' | '.join(accepted)})")
    keys = {"polygon": {"points"}, "polyline": {"points"},
            "ellipse": {"centre", "rx_m", "ry_m", "angle_deg"},
            "rect": {"x_min", "y_min", "x_max", "y_max"},
            "axis": {"a", "b"}, "radial": {"centre", "radius_m"},
            "strokes": {"strokes"}}[t]
    bad = set(g) - keys - {"type"}
    if bad:
        raise KeyError(f"Unknown key(s) in {where} ({t}): {sorted(bad)} "
                       f"(accepted: {sorted(keys | {'type'})})")
    out: dict[str, Any] = {"type": t}
    if t in ("polygon", "polyline"):
        pts = [_pt(p, where) for p in (g.get("points") or [])]
        need = 3 if t == "polygon" else 2
        if len(pts) < need:
            raise ValueError(f"{where} ({t}) needs at least {need} points")
        out["points"] = pts
    elif t == "ellipse":
        out["centre"] = _pt(g.get("centre", (0.0, 0.0)), where)
        out["rx_m"] = float(g.get("rx_m", 5.0))
        out["ry_m"] = float(g.get("ry_m", out["rx_m"]))
        out["angle_deg"] = float(g.get("angle_deg", 0.0))
        if out["rx_m"] <= 0.0 or out["ry_m"] <= 0.0:
            raise ValueError(f"{where}: ellipse radii must be > 0")
    elif t == "rect":
        for k in ("x_min", "y_min", "x_max", "y_max"):
            if k not in g:
                raise KeyError(f"{where} (rect) needs {k}")
            out[k] = float(g[k])
        if out["x_max"] <= out["x_min"] or out["y_max"] <= out["y_min"]:
            raise ValueError(f"{where}: rectangle is empty")
    elif t == "axis":
        out["a"] = _pt(g.get("a"), where)
        out["b"] = _pt(g.get("b"), where)
        if np.hypot(out["b"][0] - out["a"][0], out["b"][1] - out["a"][1]) < 1e-6:
            raise ValueError(f"{where}: axis endpoints coincide")
    elif t == "radial":
        out["centre"] = _pt(g.get("centre", (0.0, 0.0)), where)
        out["radius_m"] = float(g.get("radius_m", 10.0))
        if out["radius_m"] <= 0.0:
            raise ValueError(f"{where}: radius_m must be > 0")
    elif t == "strokes":
        out["strokes"] = normalize_strokes(g.get("strokes"), "depth")
    return out


def normalize_layer(d: Mapping[str, Any], i: int,
                    where: str = "depth.layers") -> dict[str, Any]:
    w = f"{where}[{i}]"
    if not isinstance(d, Mapping):
        raise TypeError(f"{w} is not a mapping: {d!r}")
    bad = set(d) - LAYER_KEYS
    if bad:
        raise KeyError(f"Unknown key(s) in {w}: {sorted(bad)} "
                       f"(accepted: {sorted(LAYER_KEYS)})")
    kind = str(d.get("kind", ""))
    if kind not in LAYER_KINDS:
        raise ValueError(f"{w}.kind '{kind}' unknown (available: "
                         f"{sorted(LAYER_KINDS)})")
    spec = LAYER_KINDS[kind]
    opacity = float(d.get("opacity", 1.0))
    if not 0.0 <= opacity <= 1.0:
        raise ValueError(f"{w}.opacity must be within 0..1, got {opacity}")
    blend = str(d.get("blend", spec.blend))
    if blend not in BLEND_MODES:
        raise ValueError(f"{w}.blend '{blend}' unknown {BLEND_MODES}")
    if spec.validate_params is not None:
        params = spec.validate_params(d.get("params") or {})
    else:
        params = coerce_params(spec.schema, d.get("params"), f"{w}.params")
    targets = {c: True for c in spec.channels}
    for k, v in dict(d.get("targets") or {}).items():
        if k not in spec.channels:
            raise KeyError(f"{w}.targets: '{k}' is not a channel of "
                           f"'{kind}' (writes {list(spec.channels)})")
        targets[k] = bool(v)
    seed = d.get("seed")
    return {"id": str(d.get("id") or f"L{i}"), "kind": kind,
            "name": str(d.get("name") or spec.label),
            "enabled": bool(d.get("enabled", True)), "opacity": opacity,
            "blend": blend,
            "mask": normalize_mask(d.get("mask"), f"{w}.mask"),
            "geometry": normalize_geometry(d.get("geometry"), spec.geometry,
                                           f"{w}.geometry"),
            "params": params, "targets": targets,
            "seed": None if seed is None else int(seed)}


def normalize_layers(layers: Sequence[Mapping[str, Any]] | None,
                     where: str = "depth.layers") -> list[dict[str, Any]]:
    out = [normalize_layer(d, i, where) for i, d in enumerate(layers or [])]
    ids = [L["id"] for L in out]
    if len(set(ids)) != len(ids):
        dup = sorted({x for x in ids if ids.count(x) > 1})
        raise ValueError(f"{where}: duplicate layer id(s) {dup}")
    return out


def rebase_layer(layer: Mapping[str, Any], ox: float, oy: float
                 ) -> dict[str, Any]:
    """A layer shifted from the design frame into the world frame
    (world = design - offset), exactly like walls and strokes."""
    L = dict(layer)
    g = L.get("geometry")
    if g:
        g = dict(g)
        if "points" in g:
            g["points"] = [[x - ox, y - oy] for x, y in g["points"]]
        for k in ("a", "b", "centre"):
            if k in g:
                g[k] = [g[k][0] - ox, g[k][1] - oy]
        for k, d in (("x_min", ox), ("x_max", ox), ("y_min", oy), ("y_max", oy)):
            if k in g:
                g[k] = float(g[k]) - d
        if "strokes" in g:
            g["strokes"] = [dict(s, points=[[x - ox, y - oy]
                                            for x, y in s["points"]])
                            for s in g["strokes"]]
        L["geometry"] = g
    L["mask"] = rebase_mask(L.get("mask"), ox, oy)
    return L


def schema_rows() -> list[tuple]:
    """Every layer kind's schema rows (for the schema-help check)."""
    return [r for spec in LAYER_KINDS.values() for r in spec.schema]


# The kinds register themselves on import (order = menu order).
from . import source, gradients, landforms, relief, adjust, material, survey, paint_ops  # noqa: E402,F401
