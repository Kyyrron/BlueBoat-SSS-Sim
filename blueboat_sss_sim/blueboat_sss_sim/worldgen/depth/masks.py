"""Selections for the layer editor: a *mask* is a 0..1 weight raster.

A mask dict selects where a layer acts, exactly like a layer mask in an
image editor, except the primitives are surveyable things::

    {"kind": "polygon", "points": [[x, y], ...], "feather_m": 3.0}
    {"kind": "distance_to_wall", "distance_m": 6.0, "wall_names": ["quay"]}
    {"kind": "depth_range", "min_m": 2.0, "max_m": 5.0, "feather_m": 1.0}
    {"kind": "gradient", "geometry": {"type": "axis", "a": [..], "b": [..]}}
    {"kind": "coverage", "reach_m": 15.0}          # where a survey reached

Kinds: ``all``, ``rect``, ``ellipse``, ``polygon``, ``polyline_band``,
``distance_to_wall``, ``depth_range`` (on the composite below the layer),
``gradient`` (axis or radial ramp), ``brush`` (mask strokes) and
``coverage``. Every mask takes ``feather_m`` (a C1 smoothstep on the
distance outside the selection), ``invert`` and -- when several masks are
listed -- ``op`` (``union`` | ``intersect`` | ``subtract``) combining it
with the masks before it. A feather never crosses a wall: the weight is
zeroed in wall-regions that hold none of the selection.

Unknown keys raise by name, the same quiet-wrong-answer guard as every
other config in this package.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from ...core.types import GridSpec, Wall
from . import geometry as geo
from .paint import _stroke_weight, normalize_strokes, smoothstep

MASK_KINDS = ("all", "rect", "ellipse", "polygon", "polyline_band",
              "distance_to_wall", "depth_range", "gradient", "brush",
              "coverage")
MASK_OPS = ("union", "intersect", "subtract")
_COMMON = {"kind", "feather_m", "invert", "op"}
_FIELDS: dict[str, set[str]] = {
    "all": set(),
    "rect": {"x_min", "y_min", "x_max", "y_max"},
    "ellipse": {"centre", "rx_m", "ry_m", "angle_deg"},
    "polygon": {"points"},
    "polyline_band": {"points", "width_m"},
    "distance_to_wall": {"distance_m", "wall_names"},
    "depth_range": {"min_m", "max_m"},
    "gradient": {"geometry", "profile"},
    "brush": {"strokes"},
    "coverage": {"reach_m"},
}
GRADIENT_PROFILES = ("linear", "smoothstep")


@dataclass
class MaskContext:
    """What a mask may look at besides its own geometry."""

    height: np.ndarray | None = None          # composite below the layer
    samples_xy: np.ndarray | None = None      # last positioned source
    reach_m: float = 15.0
    labels: np.ndarray | None = None          # wall regions (lazy)
    extra: dict[str, Any] = field(default_factory=dict)


def _pt(p: Any, where: str) -> list[float]:
    try:
        return [float(p[0]), float(p[1])]
    except Exception as exc:
        raise ValueError(f"{where}: expected an [x, y] point, got {p!r}") from exc


def normalize_mask(mask: Any, where: str = "mask") -> list[dict[str, Any]] | None:
    """``None`` | one dict | a list of dicts -> a canonical list (or None)."""
    if mask is None:
        return None
    items = list(mask) if isinstance(mask, (list, tuple)) else [mask]
    out: list[dict[str, Any]] = []
    for j, m in enumerate(items):
        w = f"{where}[{j}]"
        if not isinstance(m, Mapping):
            raise TypeError(f"{w} is not a mapping: {m!r}")
        kind = str(m.get("kind", ""))
        if kind not in MASK_KINDS:
            raise ValueError(f"{w}.kind '{kind}' unknown (accepted: "
                             f"{list(MASK_KINDS)})")
        bad = set(m) - _COMMON - _FIELDS[kind]
        if bad:
            raise KeyError(f"Unknown key(s) in {w} ({kind}): {sorted(bad)} "
                           f"(accepted: {sorted(_COMMON | _FIELDS[kind])})")
        rec: dict[str, Any] = {"kind": kind,
                               "feather_m": float(m.get("feather_m", 0.0)),
                               "invert": bool(m.get("invert", False))}
        if rec["feather_m"] < 0.0:
            raise ValueError(f"{w}.feather_m must be >= 0")
        op = str(m.get("op", "union"))
        if op not in MASK_OPS:
            raise ValueError(f"{w}.op '{op}' unknown {MASK_OPS}")
        rec["op"] = op
        if kind == "rect":
            for k in ("x_min", "y_min", "x_max", "y_max"):
                if k not in m:
                    raise KeyError(f"{w} ({kind}) needs {k}")
                rec[k] = float(m[k])
            if rec["x_max"] <= rec["x_min"] or rec["y_max"] <= rec["y_min"]:
                raise ValueError(f"{w}: rectangle is empty")
        elif kind == "ellipse":
            rec["centre"] = _pt(m.get("centre", (0.0, 0.0)), w)
            rec["rx_m"] = float(m.get("rx_m", 1.0))
            rec["ry_m"] = float(m.get("ry_m", rec["rx_m"]))
            rec["angle_deg"] = float(m.get("angle_deg", 0.0))
            if rec["rx_m"] <= 0.0 or rec["ry_m"] <= 0.0:
                raise ValueError(f"{w}: ellipse radii must be > 0")
        elif kind in ("polygon", "polyline_band"):
            pts = [_pt(p, w) for p in (m.get("points") or [])]
            need = 3 if kind == "polygon" else 2
            if len(pts) < need:
                raise ValueError(f"{w} ({kind}) needs at least {need} points")
            rec["points"] = pts
            if kind == "polyline_band":
                rec["width_m"] = float(m.get("width_m", 2.0))
                if rec["width_m"] <= 0.0:
                    raise ValueError(f"{w}.width_m must be > 0")
        elif kind == "distance_to_wall":
            rec["distance_m"] = float(m.get("distance_m", 5.0))
            names = m.get("wall_names")
            rec["wall_names"] = ([str(n) for n in names]
                                 if names else None)
        elif kind == "depth_range":
            rec["min_m"] = float(m.get("min_m", -1e9))
            rec["max_m"] = float(m.get("max_m", 1e9))
        elif kind == "gradient":
            g = dict(m.get("geometry") or {})
            t = str(g.get("type", ""))
            if t == "axis":
                rec["geometry"] = {"type": "axis", "a": _pt(g.get("a"), w),
                                   "b": _pt(g.get("b"), w)}
            elif t == "radial":
                rec["geometry"] = {"type": "radial",
                                   "centre": _pt(g.get("centre"), w),
                                   "radius_m": float(g.get("radius_m", 10.0))}
            else:
                raise ValueError(f"{w}.geometry.type must be axis | radial")
            prof = str(m.get("profile", "linear"))
            if prof not in GRADIENT_PROFILES:
                raise ValueError(f"{w}.profile '{prof}' unknown "
                                 f"{GRADIENT_PROFILES}")
            rec["profile"] = prof
        elif kind == "brush":
            rec["strokes"] = normalize_strokes(m.get("strokes"), "depth")
            for s in rec["strokes"]:
                s["value"] = 0.0 if not s["erase"] else None
        elif kind == "coverage":
            r = m.get("reach_m")
            rec["reach_m"] = None if r is None else float(r)
        out.append(rec)
    return out


def _feathered(inside: np.ndarray, grid: GridSpec, feather: float,
               walls: Sequence[Wall], ctx: MaskContext) -> np.ndarray:
    """A boolean selection -> 0..1 weight with a C1 feather outside it,
    never crossing a wall."""
    feather = max(float(feather), 0.0)
    if feather <= 0.0:
        return inside.astype(np.float64)
    d = geo.edt_outside(inside, grid.resolution)
    w = geo.feather_weight(d, feather)
    if walls:
        if ctx.labels is None:
            ctx.labels = geo.wall_labels(grid, walls)
        w = geo.confine_to_regions(w, ctx.labels, inside)
    return w


def _one(m: dict[str, Any], grid: GridSpec, walls: Sequence[Wall],
         ctx: MaskContext) -> np.ndarray:
    kind, fe = m["kind"], m["feather_m"]
    if kind == "all":
        return np.ones((grid.ny, grid.nx))
    if kind == "rect":
        inside = geo.rect_inside(grid, m["x_min"], m["y_min"], m["x_max"],
                                 m["y_max"])
        return _feathered(inside, grid, fe, walls, ctx)
    if kind == "ellipse":
        inside = geo.ellipse_rho(grid, m["centre"], m["rx_m"], m["ry_m"],
                                 m["angle_deg"]) <= 1.0
        return _feathered(inside, grid, fe, walls, ctx)
    if kind == "polygon":
        inside = geo.polygon_inside(m["points"], grid)
        return _feathered(inside, grid, fe, walls, ctx)
    if kind == "polyline_band":
        half = 0.5 * m["width_m"]
        d = geo.polyline_distance(m["points"], grid, half + fe)
        w = geo.band_weight(d, half, fe)
        if walls and fe > 0.0:
            if ctx.labels is None:
                ctx.labels = geo.wall_labels(grid, walls)
            w = geo.confine_to_regions(w, ctx.labels, d <= half)
        return w
    if kind == "distance_to_wall":
        d = geo.walls_distance(grid, walls, m["distance_m"] + fe,
                               m["wall_names"])
        return geo.band_weight(d, m["distance_m"], fe)
    if kind == "depth_range":
        if ctx.height is None:
            raise ValueError("a depth_range mask needs the composite below it")
        depth = -np.asarray(ctx.height)
        inside = (depth >= m["min_m"]) & (depth <= m["max_m"])
        return _feathered(inside, grid, fe, walls, ctx)
    if kind == "gradient":
        g = m["geometry"]
        if g["type"] == "axis":
            along, _, length = geo.axis_coords(grid, g["a"], g["b"])
            u = np.clip(along / length, 0.0, 1.0)
        else:
            u = np.clip(geo.radial_distance(grid, g["centre"])
                        / max(g["radius_m"], 1e-9), 0.0, 1.0)
        return smoothstep(u) if m["profile"] == "smoothstep" else u
    if kind == "brush":
        w = np.zeros((grid.ny, grid.nx))
        for s in m["strokes"]:
            f = max(float(s["feather_m"]), 2.0 * grid.resolution)
            ws = _stroke_weight(s, grid, f)
            w = w * (1.0 - ws) if s["erase"] else 1.0 - (1.0 - w) * (1.0 - ws)
        return w
    if kind == "coverage":
        reach = m["reach_m"] if m["reach_m"] is not None else ctx.reach_m
        if ctx.samples_xy is None or len(ctx.samples_xy) == 0:
            raise ValueError("a coverage mask needs a positioned source "
                             "(depth file or recording) below it")
        d = geo.coverage_distance(grid, ctx.samples_xy)
        return geo.band_weight(d, reach, max(fe, reach))
    raise ValueError(f"mask kind '{kind}' unknown")


def build_mask(grid: GridSpec, masks: Sequence[Mapping[str, Any]] | None,
               walls: Sequence[Wall], ctx: MaskContext | None = None
               ) -> np.ndarray | None:
    """Evaluate a (normalised) mask list to a ``[ny, nx]`` weight in 0..1;
    ``None`` in, ``None`` out (= everywhere)."""
    if not masks:
        return None
    ctx = ctx or MaskContext()
    acc: np.ndarray | None = None
    for m in masks:
        w = _one(m, grid, walls, ctx)
        if m["invert"]:
            w = 1.0 - w
        if acc is None:
            acc = w
        elif m["op"] == "union":
            acc = 1.0 - (1.0 - acc) * (1.0 - w)
        elif m["op"] == "intersect":
            acc = acc * w
        else:
            acc = acc * (1.0 - w)
    return np.clip(acc, 0.0, 1.0)


def rebase_mask(masks: Sequence[Mapping[str, Any]] | None, ox: float,
                oy: float) -> list[dict[str, Any]] | None:
    """Masks shifted from the design frame into the world frame
    (world = design - offset), like walls and strokes."""
    if masks is None:
        return None
    out = []
    for m in masks:
        m = dict(m)
        if "points" in m:
            m["points"] = [[x - ox, y - oy] for x, y in m["points"]]
        if "centre" in m:
            m["centre"] = [m["centre"][0] - ox, m["centre"][1] - oy]
        for k, d in (("x_min", ox), ("x_max", ox), ("y_min", oy), ("y_max", oy)):
            if k in m:
                m[k] = float(m[k]) - d
        if "geometry" in m:
            g = dict(m["geometry"])
            for k in ("a", "b", "centre"):
                if k in g:
                    g[k] = [g[k][0] - ox, g[k][1] - oy]
            m["geometry"] = g
        if "strokes" in m:
            m["strokes"] = [dict(s, points=[[x - ox, y - oy]
                                            for x, y in s["points"]])
                            for s in m["strokes"]]
        out.append(m)
    return out
