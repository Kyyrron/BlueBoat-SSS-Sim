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
hides them. Hollow bodies (pipe, drum, crate) stay solid.

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
    """The object's parts in its envelope frame."""
    build = _BUILDERS.get(o.type)
    return build(o) if build else [Part(envelope(o))]


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


def _mesh_bar(fill: float, pitch: float) -> float:
    """Bar width of a square mesh of ``pitch`` whose solid fraction is
    ``fill``: 1 - (1 - w/p)^2 = fill."""
    return pitch * (1.0 - np.sqrt(1.0 - float(np.clip(fill, 0.02, 0.95))))


def _basket(lx, ly, lz, cx, cy, cz, pitch, bar, thick) -> list[Part]:
    """Four lattice walls and a lattice floor (open towards +z)."""
    ex, ey, ez = _I3
    c = np.array([cx, cy, cz])
    out = _lattice_face(c - 0.5 * lz * ez, ex, ey, lx, ly, pitch, bar, thick)
    for s in (-1.0, 1.0):
        out += _lattice_face(c + s * 0.5 * ly * ey, ex, ez, lx, lz, pitch, bar, thick)
        out += _lattice_face(c + s * 0.5 * lx * ex, ey, ez, ly, lz, pitch, bar, thick)
    return out


# ----------------------------------------------------------------- builders
def _pallet(o: PlacedObject) -> list[Part]:
    """Deck boards on three stringers on three bottom boards (JIS T11)."""
    L, W, H = o.length, o.width, o.proud_height
    tb = min(0.022, H / 5.0)                       # board thickness
    wb = 0.10                                      # board width
    n = int(np.clip(round(o.fill_fraction * W / wb), 3, np.floor(W / wb)))
    ys = (np.arange(n) + 0.5) / n * W - 0.5 * W
    out = [_box(L, wb, tb, 0.0, y, 0.5 * (H - tb)) for y in ys]
    ws = 0.09
    out += [_box(ws, W, H - 2.0 * tb, x) for x in (-0.5 * (L - ws), 0.0, 0.5 * (L - ws))]
    out += [_box(L, wb, tb, 0.0, y, -0.5 * (H - tb))
            for y in (-0.5 * (W - wb), 0.0, 0.5 * (W - wb))]
    return out


def _mesh_crate(o: PlacedObject) -> list[Part]:
    pitch = 0.05
    return _basket(o.length, o.width, o.proud_height, 0.0, 0.0, 0.0, pitch,
                   _mesh_bar(o.fill_fraction, pitch), 0.004)


def _trolley(o: PlacedObject) -> list[Part]:
    """Lying on its side: a wire basket, two chassis tubes, the handle and
    four castors (plastic)."""
    L, W, H = o.length, o.width, o.proud_height
    pitch, r = 0.04, 0.012
    out = _basket(0.8 * L, 0.75 * W, H, 0.1 * L, 0.125 * W, 0.0, pitch,
                  _mesh_bar(o.fill_fraction, pitch), 0.004)
    y0 = -0.5 * W + 0.05
    for z in (-0.4 * H, 0.4 * H):
        out.append(_rod((-0.5 * L, y0, z), (0.5 * L, y0, z), r))
        for x in (-0.35 * L, 0.35 * L):
            out.append(_rod((x, y0, z - 0.015), (x, y0, z + 0.015), 0.05, "plastic"))
    out.append(_rod((-0.5 * L + r, 0.5 * W - r, -0.5 * H), (-0.5 * L + r, 0.5 * W - r, 0.5 * H), r))
    return out


def _bicycle(o: PlacedObject) -> list[Part]:
    """Lying flat: the frame's plane just over the bed (x along the bike, y
    its height), the handlebar standing the proud height."""
    L, W, H = o.length, o.width, o.proud_height
    rw = 0.5 * W - 0.02                            # wheel radius
    xa = 0.5 * L - rw - 0.02                       # axle x
    z0, r = -0.5 * H + 0.03, 0.016
    wb = 2.0 * xa
    P = {"A": (-xa, 0.0), "F": (xa, 0.0), "BB": (-0.10 * wb, -0.10 * W),
         "ST": (-0.20 * wb, 0.42 * W), "HT": (0.28 * wb, 0.42 * W),
         "HB": (0.32 * wb, 0.25 * W)}
    p3 = {k: np.array([x, y, z0]) for k, (x, y) in P.items()}
    out = [_rod(p3[a], p3[b], r) for a, b in
           (("BB", "ST"), ("ST", "HT"), ("BB", "HB"), ("A", "BB"), ("A", "ST"),
            ("HT", "HB"), ("HB", "F"))]
    for x in (-xa, xa):
        c = np.array([x, 0.0, z0])
        out.append(Part(Solid("annulus", rw, rw - 0.035, 0.035), _I3, c, "rubber"))
        out.append(Part(Solid("annulus", rw - 0.035, rw - 0.05, 0.02), _I3, c))
    out.append(_rod(p3["HT"] - [0, 0, 0.03], p3["HT"] + [0, 0, H - 0.03], 0.012))
    out.append(_rod(p3["BB"] - [0, 0, 0.005], p3["BB"] + [0, 0, 0.005], 0.09))
    out.append(_box(0.25, 0.04, 0.14, P["ST"][0], P["ST"][1] + 0.04, z0 + 0.04, "plastic"))
    return out


def _anchor(o: PlacedObject) -> list[Part]:
    """Shank along x, a stock across it, two flukes rising from the crown."""
    L, W, H = o.length, o.width, o.proud_height
    r = min(0.10 * W, 0.45 * H)
    z = -0.5 * H + r
    xc = 0.3 * L
    out = [_rod((-0.5 * L, 0.0, z), (0.5 * L, 0.0, z), r),
           _rod((xc, -0.5 * W, z), (xc, 0.5 * W, z), r)]
    lf = min(0.35 * L, max(H - 2.0 * r, 0.02) / 0.5)        # rising 30 deg
    Rf = rot_y(-np.radians(30.0))
    for y in (-0.3 * W, 0.3 * W):
        c = np.array([xc - 0.5 * lf * np.cos(np.radians(30.0)), y, z + 0.25 * lf])
        out.append(Part(Solid("box", lf, 0.3 * W, 0.02), Rf, c))
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


def _coil(o: PlacedObject) -> list[Part]:
    """Concentric rope rings under the class's dome (``objects._mask_coil``)."""
    L, H = o.length, o.proud_height
    n = int(round(np.random.default_rng(o.object_id).uniform(2.5, 4.5)))
    out, w = [], 0.5 * L / n
    for i in range(n):
        h = max(H * np.sqrt(1.0 - ((i + 0.5) / n) ** 2), 0.02)
        out.append(Part(Solid("annulus", (i + 1) * w, i * w + 0.2 * w, h), _I3,
                        np.array([0.0, 0.0, -0.5 * H + 0.5 * h])))
    return out


def _lump(o: PlacedObject) -> list[Part]:
    """Net ball: the class's dome, an ellipsoid half sunk in the bed."""
    L, W, H = o.length, o.width, o.proud_height
    return [Part(Solid("ellipsoid", 0.5 * L, 0.5 * W, H), _I3, np.array([0.0, 0.0, -0.5 * H]))]


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


_BUILDERS = {
    "pallet_wood": _pallet, "mesh_crate": _mesh_crate,
    "shopping_trolley": _trolley, "bicycle": _bicycle, "anchor": _anchor,
    "rope": lambda o: _line(o, 0), "chain": lambda o: _line(o, 12),
    "rope_coil": _coil, "net_ball": _lump, "debris": _debris,
    "tire_car": _tyre_car,
}
