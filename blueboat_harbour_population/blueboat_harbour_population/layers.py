"""Small litter (bottles, cans: clutter class C2) as seabed layers.

The frozen simulator draws small litter as a seabed layer (kind
``small_litter``, ``worldgen/depth/layers/heterogeneity.py``): hard items of
0.1 to 0.4 m scattered at a density per square metre, each a short ridge of
cells of a hard seabed class. A plan adds two such layers with **reserved
ids** (:data:`RESERVED_PREFIX`), each with its own ``seed`` so that its
pattern does not depend on its place in the stack:

* a base layer over the whole world;
* a band layer along the quays and pontoons, where small litter is denser.

The world mean follows the density level and never exceeds the frozen
sensor's verified clutter bracket (the R2 ``high`` preset).

The world's seabed declaration (``world.seabed.patch_classes``) must list
the hard class, or the sensor's near-nadir correction steps over every item:
the adapter and the world builder do that from ``plan.litter_class``.

Pure Python and numpy.
"""

from __future__ import annotations

import numpy as np

from .data import Parameters
from .environment import Environment
from .params import PlacementParams
from .rng import stream
from .sources import small_litter_density

#: ids of the layers a plan adds start with this; nothing else may use it
RESERVED_PREFIX = "ml_"
#: kinds of feature along which small litter is denser
BAND_KINDS = ("quay", "pontoon", "breakwater", "berth", "pier")


def layer_seed(seed: int, layer_id: str) -> int:
    """The layer's own stream key (a positive 31-bit integer)."""
    return int(stream(seed, "layers", layer_id).random() * 2147483647.0) + 1


def small_litter_layers(env: Environment, params: PlacementParams, p: Parameters, seed: int
                        ) -> tuple[list, dict]:
    """``(layers, info)``: the seabed layers of the plan and what they hold.
    No layer when ``clutter.small_litter`` is off."""
    level = int(params.density_level)
    mean = small_litter_density(p, level)
    info = {"enabled": bool(params.small_litter), "level": level, "mean_per_m2": mean,
            "material": env.coarse_class, "base_per_m2": 0.0, "band_per_m2": 0.0, "band_share": 0.0}
    if not params.small_litter or mean <= 0.0:
        # one layer at zero density all the same: a layer-less world is built by another path than a world
        # with layers (its terrain is another draw), and the planning environment is computed with layers
        info["mean_per_m2"] = 0.0
        lid = f"{RESERVED_PREFIX}litter_base"
        return [{"id": lid, "kind": "small_litter", "name": "Small litter (mission-like, none)",
                 "seed": 1, "params": {"per_m2": 0.0, "material": env.coarse_class}}], info
    cfg = p("small_litter.band")
    items = p("small_litter.items")
    width = float(cfg["width_m"])
    target = min(float(cfg["factor"]) * mean, float(cfg["cap_per_m2"]))
    lines = [f for f in sorted(env.features, key=lambda q: q.id)
             if f.kind in BAND_KINDS and f.geometry == "polyline"]
    share = 0.0
    if lines and target > mean:
        xs, ys = env.centres()
        X, Y = np.meshgrid(xs, ys)
        near = np.zeros(env.shape, dtype=bool)
        for f in lines:
            near |= f.distance(X, Y) <= width
        share = float(np.count_nonzero(near)) / float(near.size)
    if share <= 0.0 or share >= 1.0:
        base, extra = mean, 0.0
    else:
        # the band holds ``target``, the rest of the world what is left of the mean -- but never less than the
        # floor share of the mean: small litter lies everywhere, more of it along the quays
        base = max((mean - share * target) / (1.0 - share), float(cfg.get("floor_share", 0.0)) * mean)
        extra = max((mean - base * (1.0 - share)) / share - base, 0.0)
    common = {"length_min_m": float(items["length_min_m"]), "length_max_m": float(items["length_max_m"]),
              "height_min_m": float(items["height_min_m"]), "height_max_m": float(items["height_max_m"]),
              "material": env.coarse_class}
    layers = []
    if base > 0.0:
        lid = f"{RESERVED_PREFIX}litter_base"
        layers.append({"id": lid, "kind": "small_litter", "name": "Small litter (mission-like)",
                       "seed": layer_seed(seed, lid), "params": {"per_m2": round(base, 4), **common}})
    if extra > 0.0:
        lid = f"{RESERVED_PREFIX}litter_band"
        mask = [{"kind": "polyline_band", "op": "union", "feather_m": 0.0,
                 "points": [[round(float(x), 3), round(float(y), 3)] for x, y in f.points],
                 "width_m": round(2.0 * width, 3)} for f in lines]
        layers.append({"id": lid, "kind": "small_litter", "name": "Small litter along the quays (mission-like)",
                       "seed": layer_seed(seed, lid), "mask": mask,
                       "params": {"per_m2": round(extra, 4), **common}})
    info.update(base_per_m2=round(base, 4), band_per_m2=round(base + extra, 4), band_share=round(share, 4),
                band_width_m=width, layers=[L["id"] for L in layers])
    return layers, info
