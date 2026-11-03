"""The CPU reference visibility oracle: a BVH over triangles, numpy only.

Slow, simple and obviously correct, by design (``raycast_design.md`` §10): the
GPU backend must match it. Every query of the ray-cast sensor -- closest hit
along a ray, occlusion of a segment -- goes through :class:`TriangleBVH`, so
there is **one** visibility oracle for every path leg (the property whose
absence caused the pass-4 failures, ``raycast_handoff_from_pass4.md`` §1-2).

Construction is a *linear* BVH: triangles are sorted by the Morton code of
their centroids and grouped ``leaf_size`` at a time; the tree over the sorted
leaves is the implicit complete binary tree, whose node bounds are reduced
level by level. Split quality only affects speed, never the answer.

Traversal is breadth-first over (ray, node) pairs ("wavefront"), vectorised
with numpy, in chunks of rays so memory stays bounded. The ray-triangle test
is the **watertight** one of Woop, Benthin & Wald (JCGT 2(1), 2013), in
float64: a ray through a shared edge cannot leak through a closed mesh, which
Moller & Trumbore (1997, doi:10.1080/10867651.1997.10487468) -- kept below as
a reference -- does not guarantee.

ROS-free, Qt-free.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Rays traversed together; bounds the size of the (ray, node) pair arrays.
_CHUNK = 1024
#: (ray, triangle) pairs tested at once; bounds the leaf-test arrays
#: (~300 MB at 1e6 pairs in float64 temporaries).
_PAIRS = 1_000_000
#: Determinant below which a ray is treated as parallel to a triangle.
_DET_EPS = 1e-14


def _expand_bits(v: np.ndarray) -> np.ndarray:
    """Spread the low 21 bits of ``v`` to every third bit (63-bit Morton)."""
    v = v.astype(np.uint64) & np.uint64(0x1FFFFF)
    v = (v | (v << np.uint64(32))) & np.uint64(0x1F00000000FFFF)
    v = (v | (v << np.uint64(16))) & np.uint64(0x1F0000FF0000FF)
    v = (v | (v << np.uint64(8))) & np.uint64(0x100F00F00F00F00F)
    v = (v | (v << np.uint64(4))) & np.uint64(0x10C30C30C30C30C3)
    v = (v | (v << np.uint64(2))) & np.uint64(0x1249249249249249)
    return v


def morton_order(points: np.ndarray) -> np.ndarray:
    """Indices that sort ``points`` (n, 3) along a 3-D Morton curve."""
    lo = points.min(axis=0)
    span = np.maximum(points.max(axis=0) - lo, 1e-12)
    q = np.clip(((points - lo) / span) * ((1 << 21) - 1), 0, (1 << 21) - 1)
    q = q.astype(np.uint64)
    code = (_expand_bits(q[:, 0]) << np.uint64(2)) | \
        (_expand_bits(q[:, 1]) << np.uint64(1)) | _expand_bits(q[:, 2])
    return np.argsort(code, kind="stable")


@dataclass
class Hits:
    """Closest-hit result per ray. ``tri`` is -1 where nothing was hit
    (``t`` = inf). ``u``, ``v`` are barycentric coordinates on ``tri``."""

    t: np.ndarray
    tri: np.ndarray
    u: np.ndarray
    v: np.ndarray


def moller_trumbore(orig: np.ndarray, dirs: np.ndarray, v0: np.ndarray,
                    e1: np.ndarray, e2: np.ndarray, t_min: float
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorised ray-triangle test over aligned (ray, triangle) pairs.

    All arrays are (m, 3). Returns ``(t, u, v)`` with ``t`` = inf where the
    pair does not intersect at a distance greater than ``t_min``. Both faces
    count (the scene is not assumed closed, e.g. the terrain sheet)."""
    p = np.cross(dirs, e2)
    det = np.einsum("ij,ij->i", e1, p)
    ok = np.abs(det) > _DET_EPS
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    s = orig - v0
    u = np.einsum("ij,ij->i", s, p) * inv
    q = np.cross(s, e1)
    v = np.einsum("ij,ij->i", dirs, q) * inv
    t = np.einsum("ij,ij->i", e2, q) * inv
    hit = ok & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t > t_min)
    return np.where(hit, t, np.inf), u, v


def watertight(orig: np.ndarray, dirs: np.ndarray, v0: np.ndarray,
               v1: np.ndarray, v2: np.ndarray, t_min: float
               ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Watertight ray-triangle test (Woop, Benthin & Wald, "Watertight
    ray/triangle intersection", JCGT 2(1):65-82, 2013) over aligned
    (ray, triangle) pairs, in float64.

    The triangle is sheared into ray space and tested with three 2-D edge
    functions. Two triangles sharing an edge evaluate that edge from the
    same two vertices in opposite order, i.e. to the exact negation, so a
    ray through a shared edge or vertex hits at least one of them: no ray
    leaks through a closed mesh (Moller-Trumbore's round-off rejects both
    on an edge -- measured: 1 of 7572 axis-aligned probe rays through a
    cylinder in the Shiraishi world). Returns ``(t, u, v)`` like
    :func:`moller_trumbore` (``u``, ``v`` = the weights of ``v1``, ``v2``)."""
    m = orig.shape[0]
    ad = np.abs(dirs)
    kz = np.argmax(ad, axis=1)
    kx = (kz + 1) % 3
    ky = (kx + 1) % 3
    swap = dirs[np.arange(m), kz] < 0.0          # keep the winding
    kx, ky = np.where(swap, ky, kx), np.where(swap, kx, ky)
    rows = np.arange(m)
    dz = dirs[rows, kz]
    sx = dirs[rows, kx] / dz
    sy = dirs[rows, ky] / dz
    sz = 1.0 / dz
    a, b, c = v0 - orig, v1 - orig, v2 - orig

    def shear(p):
        pz = p[rows, kz]
        return p[rows, kx] - sx * pz, p[rows, ky] - sy * pz, sz * pz
    ax, ay, az = shear(a)
    bx, by, bz = shear(b)
    cx, cy, cz = shear(c)
    U = cx * by - cy * bx
    V = ax * cy - ay * cx
    W = bx * ay - by * ax
    outside = (((U < 0) | (V < 0) | (W < 0))
               & ((U > 0) | (V > 0) | (W > 0)))
    det = U + V + W
    ok = ~outside & (det != 0.0)
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    t = (U * az + V * bz + W * cz) * inv
    hit = ok & (t > t_min)
    return np.where(hit, t, np.inf), V * inv, W * inv


def _slab(orig: np.ndarray, inv_d: np.ndarray, bmin: np.ndarray,
          bmax: np.ndarray, t_max: np.ndarray) -> np.ndarray:
    """Ray-AABB overlap on [0, t_max] for aligned (ray, box) pairs."""
    with np.errstate(invalid="ignore"):
        t0 = (bmin - orig) * inv_d
        t1 = (bmax - orig) * inv_d
    # 0 * inf = nan where the ray is parallel and on a slab plane: treat
    # that axis as non-limiting (nan never wins a max/min below).
    tlo = np.nanmax(np.where(np.isnan(t0), -np.inf, np.minimum(t0, t1)),
                    axis=1)
    thi = np.nanmin(np.where(np.isnan(t1), np.inf, np.maximum(t0, t1)),
                    axis=1)
    # An empty box (an unused leaf slot: bmin = +inf, bmax = -inf) gives
    # tlo = -inf, thi = +inf above and must never count as a hit.
    valid = (bmin <= bmax).all(axis=1)
    return valid & (tlo <= thi) & (thi >= 0.0) & (tlo <= t_max)


class TriangleBVH:
    """Linear BVH over a triangle soup, with vectorised queries."""

    def __init__(self, vertices: np.ndarray, triangles: np.ndarray,
                 leaf_size: int = 8) -> None:
        self.vertices = np.ascontiguousarray(vertices, dtype=np.float64)
        self.triangles = np.ascontiguousarray(triangles, dtype=np.int64)
        n = self.triangles.shape[0]
        if n == 0:
            raise ValueError("TriangleBVH needs at least one triangle")
        tri_v = self.vertices[self.triangles]               # (n, 3, 3)
        self._v0 = tri_v[:, 0]
        self._v1 = tri_v[:, 1]
        self._v2 = tri_v[:, 2]
        order = morton_order(tri_v.mean(axis=1))
        self.leaf_size = int(leaf_size)
        n_leaf = -(-n // self.leaf_size)
        p2 = 1
        while p2 < n_leaf:
            p2 *= 2
        # Leaf slots hold triangle ids in Morton order, -1 = empty slot.
        slots = np.full(p2 * self.leaf_size, -1, dtype=np.int64)
        slots[:n] = order
        self.leaf_tris = slots.reshape(p2, self.leaf_size)
        tmin = tri_v.min(axis=1)
        tmax = tri_v.max(axis=1)
        valid = self.leaf_tris >= 0
        idx = np.where(valid, self.leaf_tris, 0)
        lmin = np.where(valid[..., None], tmin[idx], np.inf).min(axis=1)
        lmax = np.where(valid[..., None], tmax[idx], -np.inf).max(axis=1)
        # Conservative padding: a ray meeting a box exactly on its boundary
        # (a vertex on a flat leaf) must not be rounded out of the slab test
        # (measured: 1 of 5186 vertex probes on a capsule). Padding only adds
        # candidates; the triangle test decides.
        live = np.isfinite(lmin).all(axis=1)
        pad = np.zeros_like(lmin)
        pad[live] = 1e-9 * (1.0 + np.maximum(np.abs(lmin[live]),
                                             np.abs(lmax[live])))
        lmin = lmin - pad
        lmax = lmax + pad
        # levels[0] = leaves, levels[-1] = root (one node).
        self.levels: list[tuple[np.ndarray, np.ndarray]] = [(lmin, lmax)]
        while lmin.shape[0] > 1:
            lmin = lmin.reshape(-1, 2, 3).min(axis=1)
            lmax = lmax.reshape(-1, 2, 3).max(axis=1)
            self.levels.append((lmin, lmax))

    @property
    def n_triangles(self) -> int:
        return int(self.triangles.shape[0])

    # ----------------------------------------------------------- traversal
    def _candidates(self, orig: np.ndarray, dirs: np.ndarray,
                    t_max: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(ray, leaf) pairs whose leaf box the ray segment overlaps."""
        with np.errstate(divide="ignore"):
            inv_d = 1.0 / dirs
        top = len(self.levels) - 1
        rmin, rmax = self.levels[top]
        ray = np.arange(orig.shape[0])
        node = np.zeros(orig.shape[0], dtype=np.int64)
        keep = _slab(orig, inv_d, rmin[node], rmax[node], t_max)
        ray, node = ray[keep], node[keep]
        for lev in range(top - 1, -1, -1):
            bmin, bmax = self.levels[lev]
            ray = np.repeat(ray, 2)
            node = (np.repeat(node, 2) * 2) + np.tile([0, 1], node.size)
            keep = _slab(orig[ray], inv_d[ray], bmin[node], bmax[node],
                         t_max[ray])
            ray, node = ray[keep], node[keep]
        return ray, node

    def closest(self, orig: np.ndarray, dirs: np.ndarray,
                t_max: np.ndarray | float = np.inf,
                t_min: float = 1e-9) -> Hits:
        """Closest hit along each ray ``orig + t dirs``, t in (t_min, t_max].
        ``dirs`` need not be unit; ``t`` is in units of ``|dirs|``."""
        orig = np.atleast_2d(np.asarray(orig, dtype=np.float64))
        dirs = np.atleast_2d(np.asarray(dirs, dtype=np.float64))
        m = orig.shape[0]
        tm = np.broadcast_to(np.asarray(t_max, dtype=np.float64), (m,)).copy()
        out = Hits(np.full(m, np.inf), np.full(m, -1, dtype=np.int64),
                   np.zeros(m), np.zeros(m))
        for a in range(0, m, _CHUNK):
            b = min(a + _CHUNK, m)
            o, d, t_lim = orig[a:b], dirs[a:b], tm[a:b]
            ray, leaf = self._candidates(o, d, t_lim)
            if ray.size == 0:
                continue
            # Leaf tests in bounded batches (long grazing rays overlap many
            # leaves), each merged into the per-ray best.
            bt = np.full(b - a, np.inf)
            btr = np.full(b - a, np.iinfo(np.int64).max)
            bu = np.zeros(b - a)
            bv = np.zeros(b - a)
            step = max(1, _PAIRS // self.leaf_size)
            for s in range(0, ray.size, step):
                rs, ls = ray[s:s + step], leaf[s:s + step]
                r = np.repeat(rs, self.leaf_size)
                tr = self.leaf_tris[ls].ravel()
                ok = tr >= 0
                r, tr = r[ok], tr[ok]
                t, u, v = watertight(o[r], d[r], self._v0[tr],
                                     self._v1[tr], self._v2[tr], t_min)
                t = np.where(t <= t_lim[r], t, np.inf)
                fin = np.isfinite(t)
                r, tr, t, u, v = r[fin], tr[fin], t[fin], u[fin], v[fin]
                if r.size == 0:
                    continue
                # Per ray: smallest t, ties broken by the lower triangle id
                # so the answer does not depend on traversal order.
                key = np.lexsort((tr, t, r))
                r, tr, t, u, v = r[key], tr[key], t[key], u[key], v[key]
                first = np.ones(r.size, dtype=bool)
                first[1:] = r[1:] != r[:-1]
                r, tr, t, u, v = (x[first] for x in (r, tr, t, u, v))
                better = (t < bt[r]) | ((t == bt[r]) & (tr < btr[r]))
                r, tr, t, u, v = (x[better] for x in (r, tr, t, u, v))
                bt[r], btr[r], bu[r], bv[r] = t, tr, u, v
            hit = np.isfinite(bt)
            sel = a + np.flatnonzero(hit)
            out.t[sel] = bt[hit]
            out.tri[sel] = btr[hit]
            out.u[sel] = bu[hit]
            out.v[sel] = bv[hit]
        return out

    def occluded(self, p0: np.ndarray, p1: np.ndarray,
                 eps: float = 1e-6) -> np.ndarray:
        """True where the open segment p0 -> p1 (shortened by ``eps`` metres
        at both ends) crosses any triangle."""
        p0 = np.atleast_2d(np.asarray(p0, dtype=np.float64))
        p1 = np.atleast_2d(np.asarray(p1, dtype=np.float64))
        d = p1 - p0
        length = np.linalg.norm(d, axis=1)
        unit = d / np.maximum(length, 1e-300)[:, None]
        h = self.closest(p0, unit, t_max=np.maximum(length - eps, 0.0),
                         t_min=eps)
        return h.tri >= 0

    # ---------------------------------------------------------- reference
    def brute_force_closest(self, orig: np.ndarray, dirs: np.ndarray,
                            t_max: np.ndarray | float = np.inf,
                            t_min: float = 1e-9) -> Hits:
        """Every ray against every triangle (tests only: O(rays x tris))."""
        orig = np.atleast_2d(np.asarray(orig, dtype=np.float64))
        dirs = np.atleast_2d(np.asarray(dirs, dtype=np.float64))
        m, n = orig.shape[0], self.n_triangles
        tm = np.broadcast_to(np.asarray(t_max, dtype=np.float64), (m,))
        out = Hits(np.full(m, np.inf), np.full(m, -1, dtype=np.int64),
                   np.zeros(m), np.zeros(m))
        step = max(1, 400_000 // max(n, 1))
        for a in range(0, m, step):
            b = min(a + step, m)
            r = np.repeat(np.arange(a, b), n)
            tr = np.tile(np.arange(n), b - a)
            t, u, v = watertight(orig[r], dirs[r], self._v0[tr],
                                 self._v1[tr], self._v2[tr], t_min)
            t = np.where(t <= tm[r], t, np.inf).reshape(b - a, n)
            j = np.argmin(t, axis=1)        # lowest id among ties
            tt = t[np.arange(b - a), j]
            hit = np.isfinite(tt)
            rows = np.arange(a, b)[hit]
            out.t[rows] = tt[hit]
            out.tri[rows] = j[hit]
            uu = u.reshape(b - a, n)[np.arange(b - a), j]
            vv = v.reshape(b - a, n)[np.arange(b - a), j]
            out.u[rows] = uu[hit]
            out.v[rows] = vv[hit]
        return out
