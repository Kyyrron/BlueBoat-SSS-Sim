"""Seabed recipes: literature-parameterised layer stacks the editor
instantiates over the current depth source (``config/seabed_recipes.yaml``,
format ``blueboat_seabed_recipes/1``).

A recipe's geometry is written in the unit square of the world-limits
rectangle (``points_uv``, ``centre_uv``, ``a_uv`` / ``b_uv``,
``x_min_uv`` ... and radii as ``*_frac`` of the rectangle's shorter side);
``instantiate_recipe`` maps it onto a drawn rectangle in design metres and
returns ordinary layer dicts, validated by ``normalize_layers``. Each
recipe carries a ``label``, a one-sentence ``consequence`` for the sonar
image and a ``reference`` string naming ``docs/seabed_model.md`` entries,
the pattern ``config/sea_states.yaml`` set.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Mapping

import yaml

from .layers import normalize_layers

RECIPES_FORMAT = "blueboat_seabed_recipes/1"
DEFAULT_RECIPES = Path(__file__).resolve().parents[3] / "config" / "seabed_recipes.yaml"


def load_recipes(path: str | Path | None = None) -> dict[str, dict[str, Any]]:
    """``{name: recipe}`` from the YAML file (the shipped one by default)."""
    path = Path(path) if path else DEFAULT_RECIPES
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    if doc.get("format") != RECIPES_FORMAT:
        raise ValueError(f"{path}: format {doc.get('format')!r} is not "
                         f"{RECIPES_FORMAT}")
    out: dict[str, dict[str, Any]] = {}
    for name, rec in (doc.get("recipes") or {}).items():
        for key in ("label", "consequence", "reference", "layers"):
            if key not in rec:
                raise KeyError(f"{path}: recipe '{name}' lacks '{key}'")
        out[str(name)] = dict(rec)
    return out


def _uv_to_xy(uv, rect: Mapping[str, float]) -> list[float]:
    u, v = float(uv[0]), float(uv[1])
    return [rect["x_min"] + u * (rect["x_max"] - rect["x_min"]),
            rect["y_min"] + v * (rect["y_max"] - rect["y_min"])]


def _map_geometry(g: Mapping[str, Any] | None, rect: Mapping[str, float]
                  ) -> dict[str, Any] | None:
    if not g:
        return None
    side = min(rect["x_max"] - rect["x_min"], rect["y_max"] - rect["y_min"])
    out: dict[str, Any] = {}
    for k, v in g.items():
        if k == "points_uv":
            out["points"] = [_uv_to_xy(p, rect) for p in v]
        elif k.endswith("_uv"):
            base = k[:-3]
            if base in ("x_min", "x_max"):
                out[base] = _uv_to_xy((v, 0.0), rect)[0]
            elif base in ("y_min", "y_max"):
                out[base] = _uv_to_xy((0.0, v), rect)[1]
            else:
                out[base] = _uv_to_xy(v, rect)
        elif k.endswith("_frac"):
            out[k[:-5] + "_m"] = float(v) * side
        else:
            out[k] = copy.deepcopy(v)
    return out


def _map_mask(m: Any, rect: Mapping[str, float]) -> Any:
    if m is None:
        return None
    if isinstance(m, list):
        return [_map_mask(x, rect) for x in m]
    m = dict(m)
    mapped = _map_geometry({k: v for k, v in m.items()
                            if k.endswith("_uv") or k.endswith("_frac")
                            or k == "points_uv"}, rect) or {}
    for k in list(m):
        if k.endswith("_uv") or k.endswith("_frac"):
            del m[k]
    m.update(mapped)
    if "geometry" in m:
        m["geometry"] = _map_geometry(m["geometry"], rect)
    return m


def instantiate_recipe(recipe: Mapping[str, Any], rect: Mapping[str, float],
                       first_id: int = 1) -> list[dict[str, Any]]:
    """The recipe's layers in design metres over ``rect`` (ids ``L<n>``
    from ``first_id``, seeds likewise); validated."""
    if not rect:
        raise ValueError("a recipe needs the world-limits rectangle (Screen 2)")
    layers = []
    for k, L in enumerate(recipe.get("layers") or []):
        L = copy.deepcopy(dict(L))
        n = first_id + k
        L["id"] = f"L{n}"
        L.setdefault("seed", n)
        L["geometry"] = _map_geometry(L.get("geometry"), rect)
        L["mask"] = _map_mask(L.get("mask"), rect)
        layers.append(L)
    # Validate as a stack under a placeholder source (never returned).
    from .compose import source_layer
    normalize_layers([source_layer(preset="flat")] + layers)
    return layers
