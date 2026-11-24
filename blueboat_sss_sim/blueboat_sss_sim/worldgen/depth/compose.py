"""The compositor: an ordered layer stack -> height, material and gain
rasters (Qt-free).

``compose`` walks the stack (:mod:`.layers`) in order. Layer 0 is the
``source`` and consumes the terrain RNG stream exactly as the legacy
``build_height`` does; every later layer draws from its own stream
``default_rng([seed, 2, layer.seed or index])``. A layer's height field
``X`` (or ``H + delta``) is blended into the composite ``H`` with the
weight ``w = default_mask * user_mask * opacity``::

    normal   H += (X - H) * w          (assignment when w is None)
    add      H += (X - H) * w          for delta layers; H += X * w otherwise
    deepen   H += (min(H, X) - H) * w  dredging: the deeper wins
    shoal    H += (max(H, X) - H) * w  deposition: the shallower wins

Material layers stamp their ids where ``w >= 0.5`` (``hard``) or where
``w > u`` with ``u`` uniform from the layer's stream (``dithered``: a
feathered edge becomes patchy, as a real sand-mud boundary is); gain
layers multiply ``G`` by ``1 + (10^(dB/10) - 1) * w``.

A stack that is only ``[source]`` -- or ``[source, paint(set)]`` -- is
bit-identical to the legacy preset / file / recording (+ paint) path.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from ...core.geo import GeoAnchor
from ...core.types import GridSpec, Wall
from .layers import (LAYER_KINDS, LayerContext, normalize_layers,
                     rebase_layer)
from .masks import MaskContext, build_mask


@dataclass
class Composite:
    height: np.ndarray
    material_id: np.ndarray | None          # int16, -1 = no overlay
    material_names: list[str]
    gain: np.ndarray | None                 # multiplicative reflectivity
    info: dict[str, Any] = field(default_factory=dict)
    layers: list[dict[str, Any]] = field(default_factory=list)

    @property
    def material_overlay(self) -> tuple[np.ndarray, list[str]] | None:
        if self.material_id is None:
            return None
        return self.material_id, list(self.material_names)


def source_layer(mode: str = "preset", preset: str = "harbour_basin",
                 params: Mapping[str, Any] | None = None, file: str = "",
                 file_kind: str = "", file_values: str = "auto",
                 svlog: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """The ``source`` layer dict for a legacy depth description."""
    return {"id": "source", "kind": "source", "name": "Depth source",
            "params": {"mode": mode, "preset": preset,
                       "params": dict(params or {}), "file": file,
                       "file_kind": file_kind, "file_values": file_values,
                       "svlog": dict(svlog or {})}}


def paint_layer(strokes: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """The ``paint`` (op ``set``) layer dict for legacy depth strokes."""
    return {"id": "paint", "kind": "paint", "name": "Depth brush",
            "geometry": {"type": "strokes",
                         "strokes": [dict(s) for s in strokes]},
            "params": {"op": "set"}}


def stack_from_depth(depth: Any) -> list[dict[str, Any]]:
    """The full layer stack of a ``DepthState`` (or any object with its
    fields): the source, the legacy depth strokes as a ``paint`` layer
    when present, then the authored layers."""
    stack = [source_layer(mode=depth.mode, preset=depth.preset,
                          params=depth.params, file=depth.file,
                          file_kind=depth.file_kind,
                          file_values=depth.file_values, svlog=depth.svlog)]
    if depth.paint:
        stack.append(paint_layer(depth.paint))
    stack.extend(dict(L) for L in (depth.layers or []))
    return stack


def _combine(a: np.ndarray | None, b: np.ndarray | None) -> np.ndarray | None:
    if a is None:
        return b
    if b is None:
        return a
    return a * b


def _sanitize(H: np.ndarray, X: np.ndarray, w: np.ndarray | None
              ) -> tuple[np.ndarray, np.ndarray | None]:
    """A layer's NaN means "I say nothing here": a NaN target takes the
    composite below it and a NaN weight is zero. Without this a landform
    evaluated only within its reach (``polygon_signed_distance`` is NaN
    beyond it) leaked ``NaN * 0`` into the composite and from there into
    the preview pictures (black) and the baked GeoTIFF (2026-09-16)."""
    X = np.where(np.isfinite(X), X, H)
    if w is not None:
        w = np.where(np.isfinite(w), w, 0.0)
    return X, w


def _blend(H: np.ndarray, X: np.ndarray, w: np.ndarray | None, mode: str,
           is_delta: bool) -> np.ndarray:
    X, w = _sanitize(H, X, w)
    if mode == "normal" or (mode == "add" and is_delta):
        if w is None:
            return X
        return H + (X - H) * w
    if mode == "add":
        return H + (X if w is None else X * w)
    T = np.minimum(H, X) if mode == "deepen" else np.maximum(H, X)
    if w is None:
        return T
    return H + (T - H) * w


def compose(grid: GridSpec, layers: Sequence[Mapping[str, Any]],
            walls: Sequence[Wall], anchor: GeoAnchor | None,
            rng: np.random.Generator, seed: int,
            offset: tuple[float, float] = (0.0, 0.0)) -> Composite:
    """Evaluate a layer stack on ``grid``. ``rng`` is the terrain stream
    the source consumes; ``seed`` seeds every other layer; ``offset`` is
    the design->world shift applied to every geometry (world = design -
    offset), so the builder can store layers in its design frame."""
    stack = normalize_layers(layers)
    if not stack or stack[0]["kind"] != "source":
        raise ValueError("depth.layers: the first layer must be a 'source'")
    ox, oy = float(offset[0]), float(offset[1])
    if ox or oy:
        stack = [rebase_layer(L, ox, oy) for L in stack]
    walls = list(walls)
    shape = (grid.ny, grid.nx)
    ctx = LayerContext(grid=grid, height=None, walls=walls, anchor=anchor,
                       seed=int(seed))
    H: np.ndarray | None = None
    M: np.ndarray | None = None
    names: list[str] = []
    G: np.ndarray | None = None
    info: dict[str, Any] = {}
    for i, L in enumerate(stack):
        spec = LAYER_KINDS[L["kind"]]
        if i == 0:
            rng_i, ctx.is_base = rng, True
        else:
            if not L["enabled"]:
                continue
            key = L["seed"] if L["seed"] is not None else i
            rng_i = np.random.default_rng([int(seed), 2, int(key)])
            ctx.is_base = False
        ctx.height = H
        mctx = MaskContext(height=H, samples_xy=ctx.samples_xy,
                           reach_m=ctx.reach_m, labels=ctx.labels)
        user_w = build_mask(grid, L["mask"], walls, mctx)
        ctx.labels, ctx.mask = mctx.labels, user_w
        out = spec.fn(grid, L["geometry"], L["params"], rng_i, ctx)
        if out.info:
            info.update(out.info)
        w = _combine(out.default_mask, user_w)
        if L["opacity"] < 1.0:
            w = np.full(shape, L["opacity"]) if w is None else w * L["opacity"]
        t = L["targets"]
        if t.get("height", False) and (out.height is not None or out.delta is not None):
            is_delta = out.height is None
            X = out.height if not is_delta else (H + out.delta)
            if H is None:
                H = np.array(X, dtype=np.float64, copy=True)
            else:
                H = _blend(H, X, w, out.blend or L["blend"], is_delta)
        if t.get("material", False) and out.material is not None:
            ids, onames = out.material
            sel = ids >= 0
            if w is not None:
                wm = np.where(np.isfinite(w), w, 0.0)
                if out.material_edge == "hard":
                    sel &= wm >= 0.5
                else:
                    sel &= wm > rng_i.uniform(size=shape)
            if sel.any():
                if M is None:
                    M = np.full(shape, -1, dtype=np.int16)
                lut = np.empty(len(onames), dtype=np.int16)
                for k, nm in enumerate(onames):
                    if nm not in names:
                        names.append(nm)
                    lut[k] = names.index(nm)
                M[sel] = lut[ids[sel]]
        if t.get("gain", False) and out.gain_db is not None:
            if G is None:
                G = np.ones(shape)
            gdb = np.nan_to_num(np.asarray(out.gain_db, dtype=np.float64))
            lin = 10.0 ** (gdb / 10.0) - 1.0
            wg = 1.0 if w is None else np.where(np.isfinite(w), w, 0.0)
            G = G * (1.0 + lin * wg)
    if H is None:
        raise ValueError("depth.layers produced no height")
    return Composite(height=H, material_id=M, material_names=names, gain=G,
                     info=info, layers=stack)
