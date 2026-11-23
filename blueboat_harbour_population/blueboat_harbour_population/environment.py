"""The environment a population is planned in: where the water is, how deep,
on what sediment, and which features of the harbour can be sources of litter.

An :class:`Environment` is plain arrays and lists in one metric frame (the
world builder's *design* frame when it comes from the simulator). It knows
nothing of the simulator: ``sim_adapter`` builds one from a builder state, a
campaign specification or a world folder; :func:`synthetic_basin` builds one
for tests and for the examples of the documentation.

The planning grid is coarse (1 m): intensities, budgets and the depth band
are decided per cell; an object's position is continuous inside its cell and
is checked against the walls analytically.

Pure Python and numpy (scipy.ndimage for one distance transform).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

#: geometry of a feature
GEOMETRIES = ("polyline", "polygon", "point")
#: kinds of feature the environment adapters produce (what a thing IS; its
#: *use*, the source tag, is the profile's or the user's)
KINDS = ("quay", "pontoon", "breakwater", "berth", "pier", "slipway", "outlet", "mooring",
         "pile", "area", "lane", "point")
#: sediment classes that count as mud (burial ranges; the ``on_mud`` flag)
MUD_CLASSES = ("mud", "soft_mud")
#: the transducer's depth under the surface of the frozen simulator [m]
#: (``SonarModelConfig.sensor_depth_m``; the adapter overwrites it with the
#: simulator's own value)
TRANSDUCER_DEPTH_M = 0.22


@dataclass
class Feature:
    """One thing of the harbour that litter can come from, or lie around."""

    id: str                                   # stable: a wall's name, a structure's name, a spec's id
    kind: str
    points: np.ndarray                        # (n, 2) metres
    geometry: str = ""                        # "" = from the number of points and the kind
    use: str = ""                             # explicit use tag ("" = the profile's default for the kind)
    activity: float = 1.0                     # relative emission (one side of a basin dirtier than the other)
    #: half-width of the solid body objects must stay clear of [m] (a quay
    #: wall's half thickness; 0 for a floating pontoon or a drawn line)
    clear_m: float = 0.0
    attrs: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=np.float64).reshape(-1, 2)
        if self.kind not in KINDS:
            raise ValueError(f"feature {self.id!r}: kind {self.kind!r} (one of {KINDS})")
        if not self.geometry:
            self.geometry = ("point" if len(self.points) == 1 else
                             "polygon" if self.kind == "area" else "polyline")
        if self.geometry not in GEOMETRIES:
            raise ValueError(f"feature {self.id!r}: geometry {self.geometry!r} (one of {GEOMETRIES})")
        if self.geometry == "polygon" and len(self.points) < 3:
            raise ValueError(f"feature {self.id!r}: a polygon needs three points")
        if not str(self.id):
            raise ValueError("a feature needs an id")

    # ------------------------------------------------------------ geometry
    @property
    def length_m(self) -> float:
        """Length of a polyline (0 for a point; the perimeter of a polygon)."""
        p = self.points
        if len(p) < 2:
            return 0.0
        if self.geometry == "polygon":
            p = np.vstack([p, p[:1]])
        d = np.diff(p, axis=0)
        return float(np.sqrt(d[:, 0] * d[:, 0] + d[:, 1] * d[:, 1]).sum())

    def distance(self, x, y) -> np.ndarray:
        """Distance of points to the feature [m] (0 inside a polygon)."""
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        p = self.points
        if self.geometry == "point":
            dx, dy = x - p[0, 0], y - p[0, 1]
            return np.sqrt(dx * dx + dy * dy)
        q = np.vstack([p, p[:1]]) if self.geometry == "polygon" else p
        best = np.full(x.shape, np.inf)
        for a, b in zip(q[:-1], q[1:]):
            best = np.minimum(best, _segment_distance(x, y, a, b))
        if self.geometry == "polygon":
            best = np.where(_inside(x, y, p), 0.0, best)
        return best

    def along(self, x, y) -> np.ndarray:
        """Distance along the feature [m] of the point of it nearest to each
        (x, y): 0 at its first point (0 for a point feature)."""
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        p = self.points
        if self.geometry == "point":
            return np.zeros(x.shape)
        q = np.vstack([p, p[:1]]) if self.geometry == "polygon" else p
        best = np.full(x.shape, np.inf)
        out = np.zeros(x.shape)
        s0 = 0.0
        for a, b in zip(q[:-1], q[1:]):
            abx, aby = float(b[0] - a[0]), float(b[1] - a[1])
            l2 = abx * abx + aby * aby
            ln = float(np.sqrt(l2))
            if l2 <= 0.0:
                continue
            t = np.clip(((x - a[0]) * abx + (y - a[1]) * aby) / l2, 0.0, 1.0)
            dx, dy = x - (a[0] + t * abx), y - (a[1] + t * aby)
            d = dx * dx + dy * dy
            closer = d < best
            out = np.where(closer, s0 + t * ln, out)
            best = np.where(closer, d, best)
            s0 += ln
        return out

    def nearest(self, x: float, y: float) -> tuple[float, float, float, float]:
        """``(px, py, ux, uy)``: the point of the feature nearest to (x, y)
        and the unit direction of the feature there (1, 0 for a point)."""
        p = self.points
        if self.geometry == "point":
            return float(p[0, 0]), float(p[0, 1]), 1.0, 0.0
        q = np.vstack([p, p[:1]]) if self.geometry == "polygon" else p
        out, dbest = (float(q[0, 0]), float(q[0, 1]), 1.0, 0.0), np.inf
        for a, b in zip(q[:-1], q[1:]):
            abx, aby = float(b[0] - a[0]), float(b[1] - a[1])
            l2 = abx * abx + aby * aby
            t = 0.0 if l2 <= 0.0 else min(max(((x - a[0]) * abx + (y - a[1]) * aby) / l2, 0.0), 1.0)
            px, py = float(a[0]) + t * abx, float(a[1]) + t * aby
            d = (x - px) * (x - px) + (y - py) * (y - py)
            if d < dbest:
                ln = np.sqrt(l2)
                out, dbest = (px, py, abx / ln if ln > 0 else 1.0, aby / ln if ln > 0 else 0.0), d
        return out

    def to_dict(self) -> dict:
        d = {"id": self.id, "kind": self.kind, "geometry": self.geometry,
             "points": [[round(float(x), 3), round(float(y), 3)] for x, y in self.points]}
        if self.use:
            d["use"] = self.use
        if self.activity != 1.0:
            d["activity"] = float(self.activity)
        if self.clear_m:
            d["clear_m"] = round(float(self.clear_m), 3)
        if self.attrs:
            d["attrs"] = dict(self.attrs)
        return d

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Feature":
        known = {"id", "kind", "geometry", "points", "use", "activity", "clear_m", "attrs"}
        bad = set(d) - known
        if bad:
            raise KeyError(f"feature {d.get('id')!r}: unknown key(s) {sorted(bad)} (known: {sorted(known)})")
        return cls(id=str(d["id"]), kind=str(d["kind"]), points=np.asarray(d["points"], float),
                   geometry=str(d.get("geometry", "")), use=str(d.get("use", "") or ""),
                   activity=float(d.get("activity", 1.0)), clear_m=float(d.get("clear_m", 0.0)),
                   attrs=dict(d.get("attrs") or {}))


def _segment_distance(x: np.ndarray, y: np.ndarray, a, b) -> np.ndarray:
    ax, ay, bx, by = float(a[0]), float(a[1]), float(b[0]), float(b[1])
    abx, aby = bx - ax, by - ay
    l2 = abx * abx + aby * aby
    if l2 <= 0.0:
        dx, dy = x - ax, y - ay
        return np.sqrt(dx * dx + dy * dy)
    t = np.clip(((x - ax) * abx + (y - ay) * aby) / l2, 0.0, 1.0)
    dx, dy = x - (ax + t * abx), y - (ay + t * aby)
    return np.sqrt(dx * dx + dy * dy)


def _inside(x: np.ndarray, y: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Even-odd rule, comparisons and one division per edge."""
    inside = np.zeros(x.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x0, y0 = float(poly[i, 0]), float(poly[i, 1])
        x1, y1 = float(poly[(i + 1) % n, 0]), float(poly[(i + 1) % n, 1])
        if y0 == y1:
            continue
        cross = ((y0 > y) != (y1 > y)) & (x < x0 + (y - y0) * (x1 - x0) / (y1 - y0))
        inside ^= cross
    return inside


@dataclass
class Environment:
    """Everything :func:`populate` may look at."""

    x0: float                                 # the min corner of cell (0, 0) [m]
    y0: float
    cell_m: float
    water: np.ndarray                         # bool (ny, nx): survey water (wet, reachable by the boat)
    depth: np.ndarray                         # float (ny, nx): water depth, positive down [m]
    sediment: np.ndarray                      # int (ny, nx): index into ``sediment_names``
    sediment_names: list
    features: list = field(default_factory=list)
    #: cells where no object centre may fall whatever its size (dry land is
    #: already outside ``water``): the world's border strip, a structure
    blocked: np.ndarray | None = None
    #: things already in the world that objects keep clear of:
    #: ``{"x", "y", "radius", "target": bool, "id": str}``
    occupied: list = field(default_factory=list)
    path_xy: np.ndarray | None = None         # the survey path, for keep-outs and the world card
    transducer_depth_m: float = TRANSDUCER_DEPTH_M
    #: sediment classes that count as mud here
    mud_classes: tuple = MUD_CLASSES
    #: the hard seabed class the cells under a piece of small litter take
    #: (the world's coarser class)
    coarse_class: str = "gravel"
    #: reflectivity of each material name, written into the instances (the
    #: simulator's material library; the ray-cast sensor does not read it)
    material_reflectivity: dict = field(default_factory=dict)
    name: str = ""
    #: free record of where it came from (never read by the model)
    origin: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.water = np.asarray(self.water, dtype=bool)
        self.depth = np.asarray(self.depth, dtype=np.float64)
        self.sediment = np.asarray(self.sediment, dtype=np.int16)
        if self.water.ndim != 2 or self.depth.shape != self.water.shape or self.sediment.shape != self.water.shape:
            raise ValueError("environment: water, depth and sediment must be 2-D rasters of one shape")
        if self.blocked is None:
            self.blocked = np.zeros(self.water.shape, dtype=bool)
        self.blocked = np.asarray(self.blocked, dtype=bool)
        self.sediment_names = [str(s) for s in self.sediment_names]
        self.features = [f if isinstance(f, Feature) else Feature.from_dict(f) for f in self.features]
        ids = [f.id for f in self.features]
        if len(set(ids)) != len(ids):
            dup = sorted({i for i in ids if ids.count(i) > 1})
            raise ValueError(f"environment: feature ids must be unique, repeated: {dup}")
        if self.path_xy is not None:
            self.path_xy = np.asarray(self.path_xy, dtype=np.float64).reshape(-1, 2)
        self.mud_classes = tuple(str(m) for m in self.mud_classes)

    # ------------------------------------------------------------- the grid
    @property
    def shape(self) -> tuple[int, int]:
        return self.water.shape

    @property
    def cell_area_ha(self) -> float:
        return self.cell_m * self.cell_m / 1e4

    def centres(self) -> tuple[np.ndarray, np.ndarray]:
        """``(x of the columns, y of the rows)`` of the cell centres."""
        ny, nx = self.shape
        return (self.x0 + (np.arange(nx) + 0.5) * self.cell_m,
                self.y0 + (np.arange(ny) + 0.5) * self.cell_m)

    def cell_of(self, x, y) -> tuple[np.ndarray, np.ndarray]:
        """``(row, column)`` of points, clipped to the grid."""
        ny, nx = self.shape
        i = np.clip(np.floor((np.asarray(x, float) - self.x0) / self.cell_m).astype(np.int64), 0, nx - 1)
        j = np.clip(np.floor((np.asarray(y, float) - self.y0) / self.cell_m).astype(np.int64), 0, ny - 1)
        return j, i

    def inside(self, x, y) -> np.ndarray:
        ny, nx = self.shape
        x, y = np.asarray(x, float), np.asarray(y, float)
        return ((x >= self.x0) & (x < self.x0 + nx * self.cell_m)
                & (y >= self.y0) & (y < self.y0 + ny * self.cell_m))

    def altitude(self) -> np.ndarray:
        """Sonar altitude over the floor [m]: depth minus the transducer's."""
        return self.depth - float(self.transducer_depth_m)

    def is_mud(self, j, i) -> np.ndarray:
        mud = np.array([n in self.mud_classes for n in self.sediment_names], dtype=bool)
        return mud[self.sediment[j, i]]

    def feature(self, fid: str) -> Feature:
        for f in self.features:
            if f.id == fid:
                return f
        raise KeyError(f"environment: no feature {fid!r}")

    # --------------------------------------------------------------- digest
    def digest(self) -> str:
        """16 hex digits of what the plan depends on: the rasters quantised
        (depth to the millimetre), the features and the fixed objects. Two
        environments with the same digest give the same plan."""
        h = hashlib.sha256()
        head = {"x0": round(float(self.x0), 3), "y0": round(float(self.y0), 3),
                "cell_m": round(float(self.cell_m), 6), "shape": list(self.shape),
                "sediment_names": self.sediment_names, "mud": list(self.mud_classes),
                "coarse_class": self.coarse_class,
                "materials": {str(k): round(float(v), 6) for k, v in sorted(self.material_reflectivity.items())},
                "transducer_depth_m": round(float(self.transducer_depth_m), 4),
                "features": [f.to_dict() for f in self.features],
                "occupied": [{"x": round(float(o["x"]), 3), "y": round(float(o["y"]), 3),
                              "radius": round(float(o.get("radius", 0.5)), 3),
                              "target": bool(o.get("target", False))} for o in self.occupied],
                "path": ([[round(float(x), 2), round(float(y), 2)] for x, y in self.path_xy]
                         if self.path_xy is not None else None)}
        h.update(json.dumps(head, sort_keys=True, separators=(",", ":")).encode())
        h.update(np.ascontiguousarray(self.water).astype(np.uint8).tobytes())
        h.update(np.ascontiguousarray(self.blocked).astype(np.uint8).tobytes())
        d = np.where(self.water, np.rint(np.nan_to_num(self.depth) * 1000.0), 0.0).astype(np.int32)
        h.update(np.ascontiguousarray(d).tobytes())
        h.update(np.ascontiguousarray(np.where(self.water, self.sediment, -1)).astype(np.int16).tobytes())
        return h.hexdigest()[:16]

    # ------------------------------------------------------------------ I/O
    def save(self, path: str | Path) -> Path:
        """One ``.npz`` (rasters) with the rest as a JSON string inside."""
        path = Path(path)
        meta = {"format": "blueboat_population_environment/1", "x0": float(self.x0), "y0": float(self.y0),
                "cell_m": float(self.cell_m), "sediment_names": self.sediment_names,
                "mud_classes": list(self.mud_classes), "coarse_class": self.coarse_class,
                "material_reflectivity": dict(self.material_reflectivity),
                "transducer_depth_m": float(self.transducer_depth_m), "name": self.name,
                "features": [f.to_dict() for f in self.features],
                "occupied": [dict(o) for o in self.occupied],
                "path_xy": self.path_xy.tolist() if self.path_xy is not None else None,
                "origin": self.origin}
        np.savez_compressed(path, water=self.water, depth=self.depth.astype(np.float32),
                            sediment=self.sediment, blocked=self.blocked,
                            meta=np.array(json.dumps(meta, default=str)))
        return path

    @classmethod
    def load(cls, path: str | Path) -> "Environment":
        with np.load(Path(path), allow_pickle=False) as z:
            meta = json.loads(str(z["meta"]))
            if meta.get("format") != "blueboat_population_environment/1":
                raise ValueError(f"{path}: not an environment file ({meta.get('format')!r})")
            return cls(x0=meta["x0"], y0=meta["y0"], cell_m=meta["cell_m"], water=z["water"],
                       depth=z["depth"].astype(np.float64), sediment=z["sediment"],
                       sediment_names=meta["sediment_names"], features=meta["features"],
                       blocked=z["blocked"], occupied=meta.get("occupied") or [],
                       path_xy=meta.get("path_xy"), transducer_depth_m=meta["transducer_depth_m"],
                       mud_classes=tuple(meta.get("mud_classes") or MUD_CLASSES),
                       coarse_class=str(meta.get("coarse_class", "gravel")),
                       material_reflectivity=dict(meta.get("material_reflectivity") or {}),
                       name=meta.get("name", ""), origin=meta.get("origin") or {})


def block_reduce(mask: np.ndarray, values: np.ndarray, classes: np.ndarray, factor: int
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A fine raster reduced by ``factor`` in both axes: ``(water, depth,
    class)`` of each block. A block is water when more than half of its
    cells are; its depth is the **median** over its water cells (small
    litter and single stones do not move it); its class the most frequent
    one there. Rows and columns that do not fill a block are dropped."""
    f = int(factor)
    ny, nx = (mask.shape[0] // f) * f, (mask.shape[1] // f) * f
    if ny == 0 or nx == 0:
        raise ValueError("block_reduce: the raster is smaller than one block")
    m = mask[:ny, :nx].reshape(ny // f, f, nx // f, f).transpose(0, 2, 1, 3).reshape(ny // f, nx // f, f * f)
    v = values[:ny, :nx].reshape(ny // f, f, nx // f, f).transpose(0, 2, 1, 3).reshape(ny // f, nx // f, f * f)
    c = classes[:ny, :nx].reshape(ny // f, f, nx // f, f).transpose(0, 2, 1, 3).reshape(ny // f, nx // f, f * f)
    water = m.sum(axis=2) * 2 > f * f
    vv = np.where(m, v, np.nan)
    # median over the water cells: sort (NaN last) and take the middle one(s)
    s = np.sort(vv, axis=2)
    n = m.sum(axis=2)
    lo = np.clip((n - 1) // 2, 0, f * f - 1)
    hi = np.clip(n // 2, 0, f * f - 1)
    a = np.take_along_axis(s, lo[..., None], axis=2)[..., 0]
    b = np.take_along_axis(s, hi[..., None], axis=2)[..., 0]
    depth = np.where(n > 0, 0.5 * (a + b), np.nan)
    k = int(classes.max()) + 1 if classes.size else 1
    counts = np.stack([((c == q) & m).sum(axis=2) for q in range(k)], axis=2)
    cls = counts.argmax(axis=2).astype(np.int16)      # ties: the lowest index
    return water, depth, cls


def synthetic_basin(width_m: float = 100.0, height_m: float = 80.0, *, depth_m: float = 6.0,
                    grade: float = 0.0, quays: Sequence[str] = ("south",), pontoon: bool = False,
                    mooring: int = 0, mud_patch: bool = False, cell_m: float = 1.0,
                    name: str = "synthetic") -> Environment:
    """A rectangular test basin: a floor sloping north by ``grade``, quay
    walls on the named sides (``south``, ``north``, ``west``, ``east``), an
    optional pontoon from the first quay and mooring blocks in the open
    water. Features are named as the campaign generator names them
    (``quay_south``, ``pontoon``, ``block_1``). Deterministic, no seed."""
    nx, ny = int(round(width_m / cell_m)), int(round(height_m / cell_m))
    xs = (np.arange(nx) + 0.5) * cell_m
    ys = (np.arange(ny) + 0.5) * cell_m
    X, Y = np.meshgrid(xs, ys)
    depth = depth_m + grade * (Y - 0.5 * height_m)
    water = np.ones((ny, nx), dtype=bool)
    blocked = np.zeros((ny, nx), dtype=bool)
    b = max(int(round(2.0 / cell_m)), 1)
    blocked[:b] = blocked[-b:] = True
    blocked[:, :b] = blocked[:, -b:] = True
    sed = np.zeros((ny, nx), dtype=np.int16)
    names = ["sand"]
    if mud_patch:
        names.append("soft_mud")
        sed[(X > 0.6 * width_m) & (Y > 0.5 * height_m)] = 1
    inset = 3.0
    ends = {"south": ((0.0, inset), (width_m, inset)), "north": ((0.0, height_m - inset), (width_m, height_m - inset)),
            "west": ((inset, 0.0), (inset, height_m)), "east": ((width_m - inset, 0.0), (width_m - inset, height_m))}
    feats: list[Feature] = []
    for s in quays:
        a, e = ends[s]
        f = Feature(f"quay_{s}", "quay", [a, e], clear_m=0.15)
        feats.append(f)
        water &= f.distance(X, Y) > 0.15 + 0.5 * cell_m          # the wall's own cells
        # behind the wall is not survey water
        if s == "south":
            water &= Y > inset
        elif s == "north":
            water &= Y < height_m - inset
        elif s == "west":
            water &= X > inset
        else:
            water &= X < width_m - inset
    occupied = []
    if pontoon and quays:
        a, e = ends[quays[0]]
        ax, ay = 0.5 * (a[0] + e[0]), 0.5 * (a[1] + e[1])
        n = {"south": (0.0, 1.0), "north": (0.0, -1.0), "west": (1.0, 0.0), "east": (-1.0, 0.0)}[quays[0]]
        feats.append(Feature("pontoon", "pontoon", [(ax + 0.3 * n[0], ay + 0.3 * n[1]),
                                                    (ax + 18.0 * n[0], ay + 18.0 * n[1])]))
    for k in range(int(mooring)):
        x = width_m * (0.3 + 0.4 * (k % 2)) + 6.0 * (k // 2)
        y = height_m * (0.55 + 0.2 * ((k // 2) % 2))
        feats.append(Feature(f"block_{k + 1}", "mooring", [(x, y)], clear_m=1.5))
        occupied.append({"x": x, "y": y, "radius": 2.1, "target": False, "id": f"block_{k + 1}"})
    return Environment(x0=0.0, y0=0.0, cell_m=cell_m, water=water, depth=depth, sediment=sed,
                       sediment_names=names, features=feats, blocked=blocked, occupied=occupied,
                       name=name, origin={"kind": "synthetic"})
