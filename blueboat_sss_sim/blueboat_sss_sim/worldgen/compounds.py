"""Every catalog object as 3-D parts (2026-11-11).

An object is a list of :class:`Part`: a solid primitive (``solids.Solid``),
its pose in the object's **envelope** frame and, when it differs from the
object's own, a material. A class that is one solid is one part (its catalog
primitive). The classes that are not -- a pallet's boards, a bicycle's tubes,
a chain's links -- are built here, inside the stored footprint
(length x width x proud height) and from the instance's own seeded values
(``object_id``, ``fill_fraction``: the draws its footprint mask makes), so a
world always gives the same shape and nothing is written into it (NC #10).

Geometry and material names only (NC #17): how a part sounds is its
material's physics. The envelope is what places the object (the catalog
primitive, else its bounding box); parts may reach below it, where the seabed
hides them. Since 2026-11-15 the classes are the object set version 2; the
builders of the 22 deleted classes (pallet, crate, trolley, bicycle, anchor,
rope, coils) went with them.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core.types import PlacedObject
from .objects import CATALOG
from .solids import Solid, rot_y, rot_z

_I3 = np.eye(3)
_Z3 = np.zeros(3)


@dataclass(frozen=True)
class Part:
    """One primitive of an object: ``R @ p + t`` takes a point of the solid's
    own frame into the envelope frame. ``material`` None = the object's."""

    solid: Solid
    R: np.ndarray = field(default_factory=lambda: _I3)
    t: np.ndarray = field(default_factory=lambda: _Z3)
    material: str | None = None


def envelope(o: PlacedObject) -> Solid:
    """The solid that places the object: its catalog primitive, else the box
    of its stored footprint and proud height."""
    return CATALOG[o.type].solid(o) or Solid("box", o.length, o.width,
                                             o.proud_height)


def parts(o: PlacedObject) -> list[Part]:
    """The object's parts in its envelope frame. An instance that says
    ``params.spec: 2`` takes the version 2 construction of its class
    (the section "version 2" below), when the class has one."""
    if _spec(o) >= 2 and o.type in _BUILDERS_V2:
        return _BUILDERS_V2[o.type](o)
    build = _BUILDERS.get(o.type)
    return build(o) if build else [Part(envelope(o))]


def _params(o: PlacedObject) -> dict:
    return dict(getattr(o, "params", None) or {})


def _spec(o: PlacedObject) -> int:
    try:
        return int(_params(o).get("spec", 1))
    except (TypeError, ValueError):
        return 1


# ------------------------------------------------------------------ helpers
def _box(sx, sy, sz, x=0.0, y=0.0, z=0.0, material=None) -> Part:
    return Part(Solid("box", sx, sy, sz), _I3, np.array([x, y, z]), material)


def _rod(p0, p1, r, material=None) -> Part:
    """A cylinder of radius ``r`` from ``p0`` to ``p1``."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    yaw = float(np.arctan2(d[1], d[0]))
    pitch = -float(np.arcsin(np.clip(d[2] / max(L, 1e-9), -1.0, 1.0)))
    return Part(Solid("cylinder", r, L, axis=0), rot_z(yaw) @ rot_y(pitch),
                0.5 * (p0 + p1), material)


def _lattice_face(origin, eu, ev, lu, lv, pitch, bar, thick) -> list[Part]:
    """Bars of width ``bar`` at ``pitch`` on the rectangle ``lu`` x ``lv``
    spanned by the unit axes ``eu``, ``ev`` around ``origin``: a wire or
    moulded mesh panel, ``thick`` deep."""
    R = np.column_stack([eu, ev, np.cross(eu, ev)])
    out = []
    nu, nv = max(int(round(lu / pitch)), 1), max(int(round(lv / pitch)), 1)
    for i in range(nu + 1):                        # bars along ev
        out.append(Part(Solid("box", bar, lv, thick), R,
                        origin + (i / nu - 0.5) * lu * eu))
    for j in range(nv + 1):                        # bars along eu
        out.append(Part(Solid("box", lu, bar, thick), R,
                        origin + (j / nv - 0.5) * lv * ev))
    return out


def _line(o: PlacedObject, dashes: int) -> list[Part]:
    """Rope (``dashes`` 0) or chain: rods along the class's own seeded curve
    (``objects._mask_line_segments``); a chain is one rod per link."""
    L, W, H = o.length, o.width, o.proud_height
    rng = np.random.default_rng(o.object_id)
    wav = max(L / max(dashes, 1), 0.05)
    amp, ph = 1.5 * W, rng.uniform(0, 2 * np.pi)
    r = 0.5 * min(W, H)

    def pt(x):
        return np.array([x, amp * np.sin(2 * np.pi * x / (4 * wav) + ph), -0.5 * H + r])
    if dashes:
        m = np.arange(np.floor(-0.5 * L / wav), np.ceil(0.5 * L / wav))
        ends = [(max((i + 0.35) * wav, -0.5 * L), min((i + 1.0) * wav, 0.5 * L)) for i in m]
        return [_rod(pt(a), pt(b), r) for a, b in ends if b > a]
    x = np.linspace(-0.5 * L, 0.5 * L, max(int(np.ceil(L / 0.25)), 8) + 1)
    return [_rod(pt(a), pt(b), r) for a, b in zip(x[:-1], x[1:])]


def _debris(o: PlacedObject) -> list[Part]:
    """A main lump and a seeded side lobe (``objects._mask_blob``'s wobble)."""
    L, W, H = o.length, o.width, o.proud_height
    th = (0.5 * np.pi - np.random.default_rng(o.object_id).uniform(0, 6.28)) / 3.0
    return [Part(Solid("ellipsoid", 0.4 * L, 0.4 * W, H), _I3, np.array([0.0, 0.0, -0.5 * H])),
            Part(Solid("ellipsoid", 0.25 * L, 0.25 * W, 0.6 * H), _I3,
                 np.array([0.25 * L * np.cos(th), 0.25 * W * np.sin(th), -0.5 * H]))]


#: tread and belt package of a car tyre [m]: tread 8 mm + undertread and belts
TYRE_TREAD_M = 0.012


def _tyre_car(o: PlacedObject) -> list[Part]:
    """A steel-belted tread band around the rubber body."""
    sol = envelope(o)                              # annulus r_out, r_in, height
    return [Part(Solid("annulus", sol.a, sol.a - TYRE_TREAD_M, sol.c), _I3, _Z3, "tyre_tread"),
            Part(Solid("annulus", sol.a - TYRE_TREAD_M, sol.b, sol.c))]


# ================================================================ version 2
# The object set version 2 (2026-11-15, completion round session 02;
# target_families_and_surrogates.md). Geometry and material names only
# (NC #17). Dimensions and their sources: docs/seabed_model.md par. 10.
# ``params`` read here travel with a pinned entry (objects._CONSTRUCTION_PARAMS).

#: R6 level -> the fibre material of nets and ropes (worldgen/materials.py)
FIBRE_OF = {"low": "fibre_low", "nominal": "fibre", "high": "fibre_high"}
#: belt width over section width of a radial car tyre (one measured tyre:
#: 197 mm on 241 mm)
TYRE_BELT_SHARE = 0.82
#: strands of a net bundle: at most this many, beyond it they get thicker
NET_MAX_STRANDS = 200


def _rng(o: PlacedObject, tag: int) -> np.random.Generator:
    """The instance's own stream: its id and position, so a shape survives a
    reordering of the world's object list."""
    return np.random.default_rng([3, int(tag), int(o.object_id),
                                  int(round(o.x * 1000.0)) & 0xFFFFFFFF,
                                  int(round(o.y * 1000.0)) & 0xFFFFFFFF])


def _up(o: PlacedObject) -> tuple[int, float]:
    """(axis, sign) of the envelope axis that points up in the world for the
    instance's rest pose (yaw does not matter)."""
    from .solids import rotation_rpy
    v = rotation_rpy(o.roll, o.pitch, 0.0).T @ np.array([0.0, 0.0, 1.0])
    ax = int(np.argmax(np.abs(v)))
    return ax, float(np.sign(v[ax]))


def _fill(o: PlacedObject, inner: tuple[float, float, float], kind: str = "box") -> list[Part]:
    """Sediment that settled inside a flooded container: ``params.fill`` of
    the inner height, lying on the side that is down for the rest pose, of
    the material ``params.fill_material`` (a seabed class: it scatters as
    the seabed does). ``kind`` cylinder: an upright drum's column (only when
    its axis is vertical)."""
    p = _params(o)
    f = float(np.clip(float(p.get("fill", 0.0) or 0.0), 0.0, 0.95))
    if f <= 0.0:
        return []
    mat = str(p.get("fill_material", "sand"))
    ax, sgn = _up(o)
    size = list(inner)
    c = [0.0, 0.0, 0.0]
    if kind == "cylinder":
        if ax != 2:
            return []
        h = f * inner[2]
        return [Part(Solid("cylinder", 0.5 * inner[0], h, axis=2), _I3,
                     np.array([0.0, 0.0, -sgn * 0.5 * (inner[2] - h)]), mat)]
    size[ax] = f * inner[ax]
    c[ax] = -sgn * 0.5 * (inner[ax] - size[ax])
    return [_box(*size, *c, material=mat)]


def _box_v2(o: PlacedObject) -> list[Part]:
    """T1: the closed shell (the validated primitive), and the sediment in
    it when the instance is part filled."""
    w = float(_params(o).get("wall_mm", 2.0)) * 1e-3 + 0.003
    return [Part(envelope(o))] + _fill(
        o, (o.length - 2.0 * w, o.width - 2.0 * w, o.proud_height - 2.0 * w))


def _drum_v2(o: PlacedObject) -> list[Part]:
    """T2, a 200 L steel drum (JIS Z 1600): the body, two rolling hoops 300 mm
    apart, the two chimes standing over the recessed heads. The stored
    footprint is the diameter over the hoops (585 mm for a 568 mm body)."""
    sol = envelope(o)                              # cylinder: radius, height, axis z
    r, H = sol.a, sol.b
    k = H / 0.890
    rb = r * 0.568 / 0.585
    rec, chime, hoop = 0.010 * k, 0.015 * k, 0.020 * k
    out = [Part(Solid("cylinder", rb, H - 2.0 * rec, axis=2))]
    for z in (-0.150 * k, 0.150 * k):
        out.append(Part(Solid("annulus", r, rb - 0.004, hoop), _I3, np.array([0.0, 0.0, z])))
    for sg in (-1.0, 1.0):
        out.append(Part(Solid("annulus", r, rb - 0.004, chime), _I3,
                        np.array([0.0, 0.0, sg * 0.5 * (H - chime)])))
    w = float(_params(o).get("wall_mm", 1.2)) * 1e-3 + 0.003
    return out + _fill(o, (2.0 * (rb - w), 2.0 * (rb - w), H - 2.0 * (rec + w)), "cylinder")


def _block_v2(o: PlacedObject) -> list[Part]:
    """T3: a block whose upper edges are chipped by ``bevel_mm`` (a step)."""
    L, W, H = o.length, o.width, o.proud_height
    b = float(np.clip(float(_params(o).get("bevel_mm", 0.0)) * 1e-3, 0.0, 0.3 * min(L, W, H)))
    if b < 1e-3:
        return [Part(envelope(o))]
    return [_box(L, W, H - b, 0.0, 0.0, -0.5 * b), _box(L - 2.0 * b, W - 2.0 * b, H)]


def _tyre_v2(o: PlacedObject) -> list[Part]:
    """T6, bracket R6 (``params.r6``). ``low`` (belts hidden): one rubber
    body. ``nominal``: the tread band over the steel belts
    (:data:`TYRE_BELT_SHARE` of the section width) on the rubber body.
    ``high`` (belts visible): nominal plus the two steel bead rings. The rim
    diameter is the instance's (``rim_m``, from the tyre code)."""
    sol = envelope(o)                              # annulus r_out, r_in, height
    p = _params(o)
    ro, H = sol.a, sol.c
    ri = float(np.clip(0.5 * float(p.get("rim_m", 2.0 * sol.b)), 0.3 * ro, ro - 0.05))
    lvl = str(p.get("r6", "nominal"))
    if lvl == "low":
        return [Part(Solid("annulus", ro, ri, H))]
    t = TYRE_TREAD_M
    hb = TYRE_BELT_SHARE * H
    hs = 0.5 * (H - hb)
    out = [Part(Solid("annulus", ro, ro - t, hb), _I3, _Z3, "tyre_tread"),
           Part(Solid("annulus", ro - t, ri, H))]
    for sg in (-1.0, 1.0):                         # the shoulders, rubber
        out.append(Part(Solid("annulus", ro, ro - t, hs), _I3,
                        np.array([0.0, 0.0, sg * 0.5 * (hb + hs)])))
    if lvl == "high":
        d = 0.006                                  # 25 wires of about 1 mm
        for sg in (-1.0, 1.0):
            out.append(Part(Solid("annulus", ri, ri - d, d), _I3,
                            np.array([0.0, 0.0, sg * 0.5 * (H - d)]), "metal"))
    return out


def _chain_v2(o: PlacedObject) -> list[Part]:
    """S2: one rod per link along a line that may hang. ``z0_m`` / ``z1_m``:
    heights of its two ends over the seabed at the object's position (a
    riser from a block top, a line to a pontoon); ``sag_m``: how far the
    middle hangs under the straight line; never under the bed. ``link_m``:
    the link pitch; ``bar_mm``: the bar of the link -- the rod's radius is
    one bar, its width the two bars of a link seen side on. A rope line is
    the same curve with the object's fibre material."""
    p = _params(o)
    L = o.length
    link = float(np.clip(float(p.get("link_m", 0.12)), 0.03, 1.0))
    bar = float(np.clip(float(p.get("bar_mm", 20.0)) * 1e-3, 0.002, 0.2))
    z0, z1, sag = (float(p.get(k, 0.0) or 0.0) for k in ("z0_m", "z1_m", "sag_m"))
    base = -0.5 * o.proud_height + bar

    def pt(u):
        z = z0 + (z1 - z0) * u - 4.0 * sag * u * (1.0 - u)
        return np.array([(u - 0.5) * L, 0.0, base + max(z, 0.0)])
    u = np.linspace(0.0, 1.0, 513)
    P = np.array([pt(x) for x in u])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    n = int(np.clip(round(s[-1] / link), 1, 2000))
    us = np.interp(np.linspace(0.0, s[-1], n + 1), s, u)
    return [_rod(pt(a), pt(b), bar) for a, b in zip(us[:-1], us[1:])]


def _cage(L, W, H, pitch, bar, material=None) -> list[Part]:
    """Six lattice panels of a closed cage."""
    ex, ey, ez = _I3
    out = []
    for sg in (-1.0, 1.0):
        out += _lattice_face(sg * 0.5 * H * ez, ex, ey, L, W, pitch, bar, bar)
        out += _lattice_face(sg * 0.5 * W * ey, ex, ez, L, H, pitch, bar, bar)
        out += _lattice_face(sg * 0.5 * L * ex, ey, ez, W, H, pitch, bar, bar)
    if material is not None:
        out = [Part(q.solid, q.R, q.t, material) for q in out]
    return out


def _frame(L, W, H, r) -> list[Part]:
    """The twelve edges of a box as rods."""
    out = []
    for a in (-0.5, 0.5):
        for b in (-0.5, 0.5):
            out.append(_rod((-0.5 * L, a * W, b * H), (0.5 * L, a * W, b * H), r))
            out.append(_rod((a * L, -0.5 * W, b * H), (a * L, 0.5 * W, b * H), r))
            out.append(_rod((a * L, b * W, -0.5 * H), (a * L, b * W, 0.5 * H), r))
    return out


def _funnel(c, n, e1, e2, r_mouth, r_ring, depth, r_rod, material=None) -> list[Part]:
    """An entrance funnel: six rods from a mouth of radius ``r_mouth`` around
    ``c`` on a wall (inward unit normal ``n``) to a ring ``depth`` inside."""
    c, n, e1, e2 = (np.asarray(v, float) for v in (c, n, e1, e2))
    # the ring's frame must be a proper rotation (det +1): ``[e1, e2, n]`` is a
    # mirror for half the walls, which turned the ring's mesh inside out
    # (raycast_test [O3], 2026-11-15); its in-plane axes do not matter
    out = [Part(Solid("annulus", r_ring + r_rod, r_ring - r_rod, 2.0 * r_rod),
                np.column_stack([e1, np.cross(n, e1), n]), c + depth * n, material)]
    for a in np.arange(6) * (np.pi / 3.0):
        d = np.cos(a) * e1 + np.sin(a) * e2
        out.append(_rod(c + r_mouth * d, c + depth * n + r_ring * d, r_rod, material))
    return out


def _trap_pot(o: PlacedObject) -> list[Part]:
    """T4. ``variant: us_wire`` (default): the wire crab pot -- a frame of
    3/16 in rod, six mesh panels and the partition between its two chambers,
    an entrance funnel in each side of the lower chamber, a ballast rim of
    rebar on the bottom, a zinc anode. The 38 mm mesh of 1.2-1.6 mm wire
    (``wire_mm``) is drawn as a square mesh of twice the pitch and twice the
    bar: the same fill, a third fewer parts. ``variant: jp_frame``: a frame
    of 10 mm rebar covered with netting (34 mm twine mesh, drawn at twice
    the pitch and bar) of the instance's fibre (``r6``), a funnel in each
    end."""
    p = _params(o)
    L, W, H = o.length, o.width, o.proud_height
    ex, ey, ez = _I3
    if str(p.get("variant", "us_wire")) == "jp_frame":
        fib = FIBRE_OF.get(str(p.get("r6", "nominal")), "fibre")
        out = _frame(L, W, H, 0.005) + _cage(L, W, H, 0.068, 0.004, fib)
        for sg in (-1.0, 1.0):
            out += _funnel(sg * 0.5 * L * ex, -sg * ex, ey, ez, 0.4 * min(W, H),
                           0.08, 0.25 * L, 0.003, fib)
        return out
    bar = 2.0e-3 * float(np.clip(float(p.get("wire_mm", 1.4)), 0.8, 3.0))
    pitch = 0.076
    out = _frame(L, W, H, 0.0024) + _cage(L, W, H, pitch, bar)
    zp = 0.05 * H                                   # the partition, just over the middle
    out += _lattice_face(zp * ez, ex, ey, L, W, pitch, bar, bar)
    zf = 0.5 * (-0.5 * H + zp)                      # funnels: the middle of the lower chamber
    rm = min(0.11, 0.45 * (zp + 0.5 * H))
    for c, n, e1 in ((np.array([0.5 * L, 0.0, zf]), -ex, ey), (np.array([-0.5 * L, 0.0, zf]), ex, ey),
                     (np.array([0.0, 0.5 * W, zf]), -ey, ex), (np.array([0.0, -0.5 * W, zf]), ey, ex)):
        out += _funnel(c, n, e1, ez, rm, 0.05, 0.15, 0.0015)
    zb = -0.5 * H + 0.005                           # ballast rim: 10 mm rebar
    for a in (-0.5, 0.5):
        out.append(_rod((-0.5 * L, a * W, zb), (0.5 * L, a * W, zb), 0.005))
        out.append(_rod((a * L, -0.5 * W, zb), (a * L, 0.5 * W, zb), 0.005))
    out.append(_rod((-0.05, 0.0, zp + 0.03), (0.05, 0.0, zp + 0.03), 0.015, "metal"))   # anode
    return out


def _net_bundle(o: PlacedObject) -> list[Part]:
    """T7. A net bundle is a seeded tangle of strands under a flattened dome
    (half an ellipsoid of the stored footprint and height): the fibre volume
    fraction ``volume_fraction`` (bracket R6: 0.05 / 0.12 / 0.3) sets how
    many strands of ``strand_mm`` there are, at most
    :data:`NET_MAX_STRANDS`; beyond that they get thicker. A tangle, not an
    envelope: sound passes between the strands, so the shadow is partial.
    ``variant: rope_coil``: layers of three rings of rope ``strand_mm``
    thick (12-16 mm mooring line)."""
    p = _params(o)
    L, W, H = o.length, o.width, o.proud_height
    d = float(np.clip(float(p.get("strand_mm", 14.0)) * 1e-3, 0.004, 0.06))
    if str(p.get("variant", "tangle")) == "rope_coil":
        ro = 0.5 * min(L, W)
        nl = int(np.clip(round(H / d), 2, 12))
        out = []
        for i in range(nl):
            for j in range(3):
                rj = ro - (j + 0.5) * 1.2 * d
                if rj - 0.5 * d <= 0.02:
                    break
                out.append(Part(Solid("annulus", rj + 0.5 * d, rj - 0.5 * d, d), _I3,
                                np.array([0.0, 0.0, -0.5 * H + (i + 0.5) * H / nl])))
        return out
    phi = float(np.clip(float(p.get("volume_fraction", 0.12)), 0.01, 0.5))
    rng = _rng(o, 7)
    a, b = 0.5 * L, 0.5 * W
    vol = (2.0 / 3.0) * np.pi * a * b * H
    ell = 0.35 * max(L, W)                          # strand length
    r = 0.5 * d
    n = int(np.ceil(phi * vol / (np.pi * r * r * ell)))
    if n > NET_MAX_STRANDS:
        n = NET_MAX_STRANDS
        r = float(np.sqrt(phi * vol / (np.pi * ell * n)))
    out = []
    for _ in range(n):
        # a midpoint inside the dome, a direction close to horizontal
        while True:
            q = rng.uniform(-1.0, 1.0, 3)
            q[2] = abs(q[2])
            if q @ q <= 1.0:
                break
        m = q * np.array([a, b, H])
        az, el = rng.uniform(0.0, 2.0 * np.pi), rng.normal(0.0, np.radians(15.0))
        dv = np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
        h = 0.5 * ell * rng.uniform(0.5, 1.0)
        for _k in range(12):                        # both ends inside the dome
            e = np.array([m - h * dv, m + h * dv]) / np.array([a, b, H])
            if (e[:, 2] >= 0.0).all() and ((e * e).sum(axis=1) <= 1.0).all():
                break
            h *= 0.8
        z = np.array([0.0, 0.0, -0.5 * H + r])
        out.append(_rod(m - h * dv + z, m + h * dv + z, r))
    return out


_BUILDERS = {
    # the earlier construction of kept classes (an instance without ``spec: 2``)
    "chain": lambda o: _line(o, 12), "debris": _debris, "tire_car": _tyre_car,
    # new classes of the object set version 2 (one construction)
    "trap_pot": _trap_pot, "net_bundle": _net_bundle,
}

#: version 2 constructions of classes that existed before (``params.spec: 2``)
_BUILDERS_V2 = {
    "plastic_box": _box_v2, "drum_200l": _drum_v2, "concrete_block": _block_v2,
    "tire_car": _tyre_v2, "chain": _chain_v2,
}
