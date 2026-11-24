"""One instance of a class: its size, state, resting pose and burial, written
as an entry of the frozen generator's explicit-instance format.

The class cards are those of the frozen simulator (``worldgen/population.py``
``_instance``), which implement section 4 of the targets document; their
numbers are in ``data/evidence_parameters.yaml`` under ``classes`` and a test
compares every entry made here with the simulator's catalogue and builders.
An entry pins every field the generator would otherwise draw
(``objects.PINNED_FIELDS``), carries ``params.spec: 2`` and the ground truth
that the generator publishes (``role``, ``nominal_target``, ``pose``,
``hotspot``), and the placement record of this package (``source``,
``group_id``, ``stand_in_for``, ``depth_m``, ``altitude_m``,
``sediment_under``, ``on_mud``).

No acoustics here: sizes, poses and material *names* only.

Each instance is made from one row of uniform numbers (``N_ATTR`` columns),
so that it depends on nothing but its own row.

Pure Python and numpy.
"""

from __future__ import annotations

from typing import Any, Mapping

from .data import DEBRIS, TARGETS, Parameters
from .rng import pick, weighted_index

#: uniform numbers one instance consumes
N_ATTR = 14
_A_SIZE1, _A_SIZE2, _A_SIZE3, _A_WALL, _A_STATE, _A_FILL, _A_POSE1, _A_POSE2 = range(8)
_A_VARIANT, _A_BURIAL, _A_FACE, _A_MATERIAL, _A_STAND_IN, _A_EXTRA = range(8, 14)
#: classes whose flat faces can be new or used (bracket of the frozen sensor)
_FACE_CLASSES = ("plastic_box", "drum_200l", "pipe_steel")


def _u(a: float, lo: float, hi: float) -> float:
    return float(lo) + float(a) * (float(hi) - float(lo))


def tyre_codes(p: Parameters) -> list:
    """Every (width mm, aspect %, rim inch, diameter m) of the card whose
    outer diameter lies in the card's range, in a fixed order."""
    s = p("classes.tire_car.size")
    lo, hi = s["diameter"]
    out = []
    for ws in s["widths_mm"]:
        for ar in s["aspects"]:
            for rim in s["rims_in"]:
                d = rim * 0.0254 + 2.0 * ws * 1e-3 * ar / 100.0
                if lo <= d <= hi:
                    out.append((int(ws), int(ar), int(rim), d))
    return out


def shape(cls: str, a, p: Parameters, *, r6: str = "nominal", face_used_share: float | None = 0.0) -> dict:
    """Size, state, material and pose of one instance of ``cls`` from its
    row of uniforms ``a``: ``{length, width, proud_height, material, params,
    pose: (name, roll_deg, pitch_deg), fill_fraction}``."""
    prm: dict[str, Any] = {"spec": 2}
    pose = ("canonical", 0.0, 0.0)
    fill_fraction = 1.0
    if cls == "plastic_box":
        s = p("classes.plastic_box.size")
        L = _u(a[_A_SIZE1], *s["length"])
        W = L * _u(a[_A_SIZE2], *s["width_ratio"])
        H = _u(a[_A_SIZE3], *s["height"])
        prm["wall_mm"] = round(_u(a[_A_WALL], *s["wall_mm"]), 2)
        st = p("classes.plastic_box.states")
        if a[_A_STATE] < float(st["part_filled"]):
            prm["state"] = "part_filled"
            prm["fill"] = round(_u(a[_A_FILL], *p("classes.plastic_box.fill")), 3)
        else:
            prm["state"] = "flooded"
        po = p("classes.plastic_box.poses")
        name = ("upright", "inverted", "on_long_side", "on_end")[
            weighted_index(a[_A_POSE1], [po["upright"], po["inverted"], po["on_long_side"], po["on_end"]])]
        pose = {"upright": ("upright", 0.0, 0.0), "inverted": ("inverted", 180.0, 0.0),
                "on_long_side": ("on_long_side", 90.0, 0.0), "on_end": ("on_end", 0.0, 90.0)}[name]
        mat = p("classes.plastic_box.material")
    elif cls == "drum_200l":
        s = p("classes.drum_200l.size")
        L = W = float(s["diameter"])
        H = float(s["height"])
        prm["wall_mm"] = round(_u(a[_A_WALL], *s["wall_mm"]), 2)
        if a[_A_POSE1] < float(p("classes.drum_200l.poses")["lying"]):
            pose = ("lying", 90.0, 0.0)
            prm["state"] = "flooded"
        else:
            pose = ("standing", 0.0, 0.0)
            if a[_A_STATE] < float(p("classes.drum_200l.states")["standing_sediment_inside"]):
                prm["state"] = "sediment_inside"
                prm["fill"] = round(_u(a[_A_FILL], *p("classes.drum_200l.fill")), 3)
            else:
                prm["state"] = "flooded"
        mat = p("classes.drum_200l.material")
    elif cls == "concrete_block":
        s = p("classes.concrete_block.size")
        L, W, H = sorted([_u(a[_A_SIZE1], *s["a"]), _u(a[_A_SIZE2], *s["b"]), _u(a[_A_SIZE3], *s["c"])],
                         reverse=True)
        prm["bevel_mm"] = round(_u(a[_A_WALL], *s["bevel_mm"]), 1)
        if a[_A_POSE1] >= float(p("classes.concrete_block.poses")["largest_face_down"]):
            pose = ("on_long_side", 90.0, 0.0) if a[_A_POSE2] < L / (L + W) else ("on_end", 0.0, 90.0)
        else:
            pose = ("largest_face_down", 0.0, 0.0)
        mat = p("classes.concrete_block.material")
    elif cls == "trap_pot":
        s = p("classes.trap_pot.size")
        if a[_A_VARIANT] < float(p("classes.trap_pot.variants")["us_wire"]):
            prm["variant"] = "us_wire"
            L = W = float(s["us_wire"]["side"])
            H = _u(a[_A_SIZE3], *s["us_wire"]["height"])
            wires = s["us_wire"]["wire_mm"]
            prm["wire_mm"] = float(wires[pick(a[_A_WALL], len(wires))])
        else:
            prm["variant"], prm["r6"] = "jp_frame", r6
            L = _u(a[_A_SIZE1], *s["jp_frame"]["length"])
            W = _u(a[_A_SIZE2], *s["jp_frame"]["width"])
            H = _u(a[_A_SIZE3], *s["jp_frame"]["height"])
        pose = (("upright", 0.0, 0.0) if a[_A_POSE1] < float(p("classes.trap_pot.poses")["upright"])
                else ("on_side", 90.0, 0.0))
        mat = p("classes.trap_pot.material")
        fill_fraction = float(p("classes.trap_pot.fill_fraction"))
    elif cls == "pipe_steel":
        s = p("classes.pipe_steel.size")
        L = _u(a[_A_SIZE1], *s["length"])
        W = H = _u(a[_A_SIZE2], *s["diameter"])
        prm["wall_mm"] = round(_u(a[_A_WALL], *s["wall_mm"]), 1)
        prm["state"] = "flooded"
        pose = ("lying", 0.0, 0.0)
        mat = p("classes.pipe_steel.material")
    elif cls == "tire_car":
        codes = tyre_codes(p)
        ws, ar, rim, D = codes[pick(a[_A_EXTRA], len(codes))]
        L = W = D
        H = ws * 1e-3
        prm.update(tyre_code=f"{ws}/{ar}R{rim}", rim_m=round(rim * 0.0254, 4), r6=r6)
        pose = (("flat", 0.0, 0.0) if a[_A_POSE1] < float(p("classes.tire_car.poses")["flat"])
                else ("on_tread", 90.0, 0.0))
        mat = p("classes.tire_car.material")
    elif cls == "net_bundle":
        s = p("classes.net_bundle.size")
        prm["r6"] = r6
        if a[_A_VARIANT] < float(p("classes.net_bundle.variants")["tangle"]):
            t = s["tangle"]
            L = _u(a[_A_SIZE1], *t["length"])
            W = min(_u(a[_A_SIZE2], *t["width"]), L)
            H = min(_u(a[_A_SIZE3], *t["height"]), 0.5 * W)
            prm.update(variant="tangle", strand_mm=round(_u(a[_A_WALL], *t["strand_mm"]), 1),
                       volume_fraction=float(p("classes.net_bundle.volume_fraction")[r6]))
        else:
            t = s["rope_coil"]
            L = W = _u(a[_A_SIZE1], *t["across"])
            d = _u(a[_A_WALL], *t["rope_mm"])
            lo, hi = int(t["layers"][0]), int(t["layers"][1])
            H = d * 1e-3 * (lo + pick(a[_A_EXTRA], hi - lo + 1))
            prm.update(variant="rope_coil", strand_mm=round(d, 1))
        pose = ("flattened", 0.0, 0.0)
        mat = p("classes.net_bundle.material")[r6]
    elif cls == DEBRIS:
        s = p("classes.debris.size")
        L = _u(a[_A_SIZE1], *s["length"])
        W = min(_u(a[_A_SIZE2], *s["width"]), L)
        H = min(_u(a[_A_SIZE3], *s["height"]), W)
        mats = p("classes.debris.materials")
        names = list(mats)
        mat = names[weighted_index(a[_A_MATERIAL], [mats[n] for n in names])]
        pose = ("flat", 0.0, 0.0)
    else:
        raise KeyError(f"no class card for {cls!r}")
    if cls in _FACE_CLASSES and face_used_share is not None:
        prm["face_state"] = "used" if a[_A_FACE] < float(face_used_share) else "new"
    return {"length": float(L), "width": float(W), "proud_height": float(H), "material": str(mat),
            "params": prm, "pose": pose, "fill_fraction": fill_fraction}


def radius(sh: Mapping[str, Any]) -> float:
    """Footprint radius of a shaped instance [m]: half its plan diagonal."""
    L, W = float(sh["length"]), float(sh["width"])
    return 0.5 * (L * L + W * W) ** 0.5


def burial(cls: str, pose_name: str, on_mud: bool, a, p: Parameters, preset: str = "nominal") -> float:
    """Burial fraction: U(0, the card's upper end for the sediment) times the
    preset's scale, capped."""
    card = p(f"classes.{cls}.burial")
    lo_hi = card.get(pose_name, card.get("default"))
    if lo_hi is None:
        raise KeyError(f"classes.{cls}.burial: no range for the pose {pose_name!r}")
    hi = float(lo_hi[1 if on_mud else 0])
    scale = float(p("burial.scale")[preset])
    return float(min(_u(a[_A_BURIAL], 0.0, hi) * scale, float(p("burial.cap"))))


def stand_in(cls: str, use: str, a, p: Parameters) -> str | None:
    """What the instance stands for, when its source emits objects the
    object set lacks (a city quay's bicycles as ``trap_pot``), else None."""
    if use in ("far_field",) or not p.has(f"tags.{use}.stand_in"):
        return None
    rec = p(f"tags.{use}.stand_in").get(cls)
    if not rec:
        return None
    u = float(a[_A_STAND_IN])
    share = float(rec["share"])
    if u >= share:
        return None
    names = list(rec["of"])
    return names[weighted_index(u / share if share > 0 else 0.0, [rec["of"][n] for n in names])]


def entry(cls: str, x: float, y: float, yaw: float, sh: Mapping[str, Any], burial_fraction: float,
          record: Mapping[str, Any], *, reflectivity: float = 0.5, min_target_m: float = 0.5) -> dict:
    """The explicit-instance entry of the frozen generator
    (``worldgen/objects.py`` ``place_objects_v2``): every pinned field, the
    resting pose only for an instance that rests on a side, and ``params``
    with the ground truth and the placement ``record``."""
    name, roll, pitch = sh["pose"]
    prm = dict(sh["params"])
    target = cls != DEBRIS
    biggest = max(float(sh["length"]), float(sh["width"]), float(sh["proud_height"]))
    prm.update(role="target" if target else "clutter",
               nominal_target=bool(target and biggest >= float(min_target_m)),
               pose=name)
    prm.update(record)
    e: dict[str, Any] = {
        "type": cls, "x": round(float(x), 3), "y": round(float(y), 3), "yaw": round(float(yaw), 5),
        "length": round(float(sh["length"]), 4), "width": round(float(sh["width"]), 4),
        "proud_height": round(float(sh["proud_height"]), 4), "burial": round(float(burial_fraction), 4),
        "fill_fraction": float(sh["fill_fraction"]), "reflectivity": float(reflectivity),
        "material": str(sh["material"]), "params": prm}
    if roll != 0.0 or pitch != 0.0:
        e.update(roll_deg=float(roll), pitch_deg=float(pitch), rest=name)
    return e


def is_target_class(cls: str) -> bool:
    return cls in TARGETS
