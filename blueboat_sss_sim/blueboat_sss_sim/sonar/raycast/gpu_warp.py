"""GPU visibility oracle on NVIDIA Warp (Phase 4; ``raycast_design.md`` §9).

A drop-in for :class:`bvh_cpu.TriangleBVH`'s queries -- ``closest`` and
``occluded`` with the same semantics (``t`` in units of ``|dirs|``, the open
segment shortened by ``eps`` at both ends) -- so the direct path, the
eigenray chains, transmission and diffraction run unchanged on the GPU
(``RaycastConfig.backend: warp``). The CPU BVH stays the reference: gate G1
(``test/raycast_test.py``) holds the two equal on hits and runs the Phase-1
leak probe through this oracle (Warp's ``mesh_query_ray`` watertightness is
not documented).

Numerics: candidate traversal is a Warp BVH over the triangles' boxes in
float32 (shifted to a local origin, boxes padded by ``BOX_PAD_M`` so no
triangle is missed); every candidate is tested by the CPU's watertight
algorithm in float64 on the global coordinates (the reference's inputs) with the CPU's ``eps`` and lowest-id tie rule, so hit
decisions equal the reference's. The module is compiled **without FMA
contraction** (``fuse_fp: False``): fused, the edge function
``a·b − c·d`` is not antisymmetric, both triangles of a shared edge read the
same sign and a ray through the edge passes between them (375 of 22 049
leak probes, G1 2026-11-04); unfused, every operation is the IEEE one numpy
performs. Warp's own ``mesh_query_ray`` is not used:
it leaked 24 of 22 049 vertex/edge probes (G1, 2026-11-04).

Imported only when the backend is selected: the geometric renderer, the
node's default path and the smoke test never import Warp. Runs in the
ray-cast venv (``~/ros2_ws/sim_runs/raycast/venv``) until Phase 6.

ROS-free, Qt-free.
"""

from __future__ import annotations

import numpy as np
import warp as wp

from .bvh_cpu import Hits

BOX_PAD_M = 1.0e-5        # AABB padding: float32 traversal never misses a triangle
#: The traversal's distance cutoff past a query's last accepted hit distance
#: (``bvh_query_next``'s ``max_dist``): a node is skipped only when its float32
#: slab entry lies beyond t_max (1 + CUT_REL) + CUT_ABS_M. The traversal ray
#: is inside every padded box that holds an accepted hit at the hit distance
#: (the BOX_PAD_M premise), so its exact entry is <= t_hit; the float32 entry
#: carries ~3 roundings relative (2e-7) and the float32 ray's origin and
#: direction differ from the float64 test's by ~1e-5 m + 6e-8 t. Hit decisions
#: are therefore those of the full traversal; the cutoff only stops it at the
#: segment's end instead of the scene's (2026-11-04).
CUT_REL = 1.0e-6
CUT_ABS_M = 1.0e-3
_CUT_K64 = wp.constant(wp.float64(1.0 + CUT_REL))
_CUT_A64 = wp.constant(wp.float64(CUT_ABS_M))
_CUT_K32 = wp.constant(wp.float32(1.0 + CUT_REL))
_CUT_A32 = wp.constant(wp.float32(CUT_ABS_M))
_CHUNK = 4_000_000

wp.config.quiet = True
wp.init()
wp.set_module_options({"fuse_fp": False, "enable_backward": False})   # watertightness: see the docstring; no adjoints

_INF = wp.constant(wp.float64(1.0e300))


@wp.func
def _watertight(o: wp.vec3d, d: wp.vec3d, p0: wp.vec3d, p1: wp.vec3d,
                p2: wp.vec3d) -> wp.vec3d:
    """Woop, Benthin & Wald (JCGT 2013) in float64: ``bvh_cpu.watertight``
    line for line. Returns (t, u, v); t = 1e300 on a miss."""
    ax = wp.abs(d[0])
    ay = wp.abs(d[1])
    az = wp.abs(d[2])
    kz = int(0)
    if ay > ax and ay >= az:
        kz = 1
    elif az > ax and az > ay:
        kz = 2
    if ax >= ay and ax >= az:
        kz = 0
    # Warp: a variable reassigned inside a branch must be declared with a
    # type constructor, or the reassignment does not take effect
    kx = int((kz + 1) % 3)
    ky = int((kz + 2) % 3)
    if d[kz] < wp.float64(0.0):
        tmp = int(kx)
        kx = ky
        ky = tmp
    dz = d[kz]
    sx = d[kx] / dz
    sy = d[ky] / dz
    sz = wp.float64(1.0) / dz
    a = p0 - o
    b = p1 - o
    c = p2 - o
    Ax = a[kx] - sx * a[kz]
    Ay = a[ky] - sy * a[kz]
    Bx = b[kx] - sx * b[kz]
    By = b[ky] - sy * b[kz]
    Cx = c[kx] - sx * c[kz]
    Cy = c[ky] - sy * c[kz]
    U = Cx * By - Cy * Bx
    V = Ax * Cy - Ay * Cx
    W = Bx * Ay - By * Ax
    zero = wp.float64(0.0)
    neg = (U < zero) or (V < zero) or (W < zero)
    pos = (U > zero) or (V > zero) or (W > zero)
    det = U + V + W
    if (neg and pos) or det == zero:
        return wp.vec3d(_INF, zero, zero)
    inv = wp.float64(1.0) / det
    t = (U * (sz * a[kz]) + V * (sz * b[kz]) + W * (sz * c[kz])) * inv
    return wp.vec3d(t, V * inv, W * inv)


@wp.func
def _watertight32(o: wp.vec3, d: wp.vec3, p0: wp.vec3, p1: wp.vec3, p2: wp.vec3) -> wp.vec3:
    """The same test in float32 (watertight too: unfused arithmetic, shared
    vertex coordinates); for sampling uses that need no reference-exact hit
    decisions (diffraction's visibility samples). (t, u, v); t = 1e30 on a
    miss."""
    ax = wp.abs(d[0])
    ay = wp.abs(d[1])
    az = wp.abs(d[2])
    kz = int(0)
    if ay > ax and ay >= az:
        kz = 1
    elif az > ax and az > ay:
        kz = 2
    if ax >= ay and ax >= az:
        kz = 0
    kx = int((kz + 1) % 3)
    ky = int((kz + 2) % 3)
    if d[kz] < 0.0:
        tmp = int(kx)
        kx = ky
        ky = tmp
    dz = d[kz]
    sx = d[kx] / dz
    sy = d[ky] / dz
    sz = 1.0 / dz
    a = p0 - o
    b = p1 - o
    c = p2 - o
    Ax = a[kx] - sx * a[kz]
    Ay = a[ky] - sy * a[kz]
    Bx = b[kx] - sx * b[kz]
    By = b[ky] - sy * b[kz]
    Cx = c[kx] - sx * c[kz]
    Cy = c[ky] - sy * c[kz]
    U = Cx * By - Cy * Bx
    V = Ax * Cy - Ay * Cx
    W = Bx * Ay - By * Ax
    neg = (U < 0.0) or (V < 0.0) or (W < 0.0)
    pos = (U > 0.0) or (V > 0.0) or (W > 0.0)
    det = U + V + W
    if (neg and pos) or det == 0.0:
        return wp.vec3(1.0e30, 0.0, 0.0)
    inv = 1.0 / det
    t = (U * (sz * a[kz]) + V * (sz * b[kz]) + W * (sz * c[kz])) * inv
    return wp.vec3(t, V * inv, W * inv)


@wp.kernel
def _closest_k(bvh: wp.uint64, V: wp.array(dtype=wp.vec3d),
               T: wp.array(dtype=wp.vec3i), orig: wp.array(dtype=wp.vec3d),
               dirs: wp.array(dtype=wp.vec3d), o32: wp.array(dtype=wp.vec3),
               d32: wp.array(dtype=wp.vec3), tmin: wp.array(dtype=wp.float64),
               tmax: wp.array(dtype=wp.float64), out_t: wp.array(dtype=wp.float64),
               out_f: wp.array(dtype=int), out_u: wp.array(dtype=wp.float64),
               out_v: wp.array(dtype=wp.float64)):
    i = wp.tid()
    q = wp.bvh_query_ray(bvh, o32[i], d32[i])
    idx = int(0)
    best_t = wp.float64(tmax[i])
    best_f = int(-1)
    bu = wp.float64(0.0)
    bv = wp.float64(0.0)
    dl = wp.length(dirs[i])                 # t is in units of dirs; traversal in metres
    cut = wp.float32(best_t * dl * _CUT_K64 + _CUT_A64)
    while wp.bvh_query_next(q, idx, cut):
        tri = T[idx]
        r = _watertight(orig[i], dirs[i], V[tri[0]], V[tri[1]], V[tri[2]])
        t = r[0]
        if t > tmin[i] and t <= tmax[i]:
            if best_f < 0 or t < best_t or (t == best_t and idx < best_f):
                best_t = t
                best_f = idx
                bu = r[1]
                bv = r[2]
                cut = wp.float32(best_t * dl * _CUT_K64 + _CUT_A64)
    out_f[i] = best_f
    out_t[i] = best_t
    out_u[i] = bu
    out_v[i] = bv


@wp.kernel
def _occluded_k(bvh: wp.uint64, V: wp.array(dtype=wp.vec3d),
                T: wp.array(dtype=wp.vec3i), a: wp.array(dtype=wp.vec3d),
                dirs: wp.array(dtype=wp.vec3d), o32: wp.array(dtype=wp.vec3),
                d32: wp.array(dtype=wp.vec3), tmin: wp.float64,
                tmax: wp.array(dtype=wp.float64), out: wp.array(dtype=wp.int32)):
    i = wp.tid()
    out[i] = 0
    q = wp.bvh_query_ray(bvh, o32[i], d32[i])
    idx = int(0)
    cut = wp.float32(tmax[i] * _CUT_K64 + _CUT_A64)   # unit dirs
    while wp.bvh_query_next(q, idx, cut):
        tri = T[idx]
        r = _watertight(a[i], dirs[i], V[tri[0]], V[tri[1]], V[tri[2]])
        if r[0] > tmin and r[0] <= tmax[i]:
            out[i] = 1
            break


class WarpBVH:
    """The scene's triangles on the GPU: a Warp BVH over padded triangle
    boxes for candidate traversal (float32) and the CPU's watertight test in
    float64 for every candidate, so hit decisions equal the reference's
    (Warp's own ``mesh_query_ray`` leaked 24 of 22 049 vertex/edge probes,
    G1 2026-11-04, and is not used)."""

    def __init__(self, vertices: np.ndarray, triangles: np.ndarray,
                 device: str = "cuda:0") -> None:
        v = np.asarray(vertices, dtype=np.float64)
        self.origin = 0.5 * (v.min(axis=0) + v.max(axis=0))
        self.device = device
        self.triangles = np.asarray(triangles, dtype=np.int64)
        vl = v - self.origin
        tv = vl[self.triangles]
        lo = tv.min(axis=1) - BOX_PAD_M
        hi = tv.max(axis=1) + BOX_PAD_M
        # the float64 test runs on the GLOBAL coordinates (the reference's
        # inputs): a shifted frame loses sub-ulp offsets near a coordinate
        # plane (56 leak probes on a cone's base rim, G1 2026-11-04); only
        # the float32 traversal boxes use the local frame
        self._V = wp.array(v, dtype=wp.vec3d, device=device)
        #: float32 vertices in the local frame (``_watertight32`` users)
        self._V32 = wp.array(vl.astype(np.float32), dtype=wp.vec3, device=device)
        self._T = wp.array(self.triangles.astype(np.int32), dtype=wp.vec3i,
                           device=device)
        self._lo = wp.array(lo.astype(np.float32), dtype=wp.vec3, device=device)
        self._hi = wp.array(hi.astype(np.float32), dtype=wp.vec3, device=device)
        self.bvh = wp.Bvh(self._lo, self._hi)

    @property
    def n_triangles(self) -> int:
        return int(self.triangles.shape[0])

    def _rays(self, orig, dirs):
        ol = orig - self.origin
        L = np.linalg.norm(dirs, axis=1, keepdims=True)
        dn = dirs / np.maximum(L, 1e-300)
        return (wp.array(orig, dtype=wp.vec3d, device=self.device),
                wp.array(dirs, dtype=wp.vec3d, device=self.device),
                wp.array(ol.astype(np.float32), dtype=wp.vec3, device=self.device),
                wp.array(dn.astype(np.float32), dtype=wp.vec3, device=self.device))

    def closest(self, orig: np.ndarray, dirs: np.ndarray,
                t_max: np.ndarray | float = np.inf, t_min: float = 1e-9) -> Hits:
        orig = np.atleast_2d(np.asarray(orig, dtype=np.float64))
        dirs = np.atleast_2d(np.asarray(dirs, dtype=np.float64))
        m = orig.shape[0]
        tm = np.broadcast_to(np.asarray(t_max, dtype=np.float64), (m,))
        tm = np.where(np.isfinite(tm), tm, 1.0e300)
        out = Hits(np.full(m, np.inf), np.full(m, -1, dtype=np.int64),
                   np.zeros(m), np.zeros(m))
        for a in range(0, m, _CHUNK):
            b = min(a + _CHUNK, m)
            n = b - a
            o64, d64, o32, d32 = self._rays(orig[a:b], dirs[a:b])
            ot = wp.zeros(n, dtype=wp.float64, device=self.device)
            of = wp.zeros(n, dtype=int, device=self.device)
            ou = wp.zeros(n, dtype=wp.float64, device=self.device)
            ov = wp.zeros(n, dtype=wp.float64, device=self.device)
            wp.launch(_closest_k, n, [self.bvh.id, self._V, self._T, o64, d64, o32, d32,
                                      wp.array(np.full(n, float(t_min)), dtype=wp.float64,
                                               device=self.device),
                                      wp.array(tm[a:b], dtype=wp.float64, device=self.device),
                                      ot, of, ou, ov], device=self.device)
            f = of.numpy()
            hit = f >= 0
            sel = a + np.flatnonzero(hit)
            out.t[sel] = ot.numpy()[hit]
            out.tri[sel] = f[hit]
            out.u[sel] = ou.numpy()[hit]
            out.v[sel] = ov.numpy()[hit]
        return out

    def occluded(self, p0: np.ndarray, p1: np.ndarray, eps: float = 1e-6
                 ) -> np.ndarray:
        """``TriangleBVH.occluded``: the open segment, shortened by ``eps``
        at both ends, crosses a triangle."""
        p0 = np.atleast_2d(np.asarray(p0, dtype=np.float64))
        p1 = np.atleast_2d(np.asarray(p1, dtype=np.float64))
        p0 = np.broadcast_to(p0, p1.shape) if p0.shape[0] == 1 else p0
        d = p1 - p0
        L = np.linalg.norm(d, axis=1)
        unit = d / np.maximum(L, 1e-300)[:, None]
        tmx = np.maximum(L - eps, 0.0)
        m = p1.shape[0]
        out = np.zeros(m, dtype=bool)
        for a in range(0, m, _CHUNK):
            b = min(a + _CHUNK, m)
            o64, d64, o32, d32 = self._rays(p0[a:b], unit[a:b])
            o = wp.zeros(b - a, dtype=wp.int32, device=self.device)
            wp.launch(_occluded_k, b - a, [self.bvh.id, self._V, self._T, o64, d64,
                                           o32, d32, wp.float64(eps),
                                           wp.array(tmx[a:b], dtype=wp.float64,
                                                    device=self.device), o],
                      device=self.device)
            out[a:b] = o.numpy().astype(bool)
        return out


# ---------------------------------------------------------------- aperture
@wp.kernel
def _near_k(d: wp.array(dtype=wp.vec3), e_a: wp.vec3, u: wp.array(dtype=float),
            w: wp.array(dtype=float), k: float, sg: float, out_re: wp.array(dtype=float),
            out_im: wp.array(dtype=float)):
    """(1/L) int exp(sg i k (|d - u e_a| - |d|)) |d| / |d - u e_a| du by the
    given Gauss-Legendre nodes (``beam.Aperture._near``), with the stable
    difference r - R = (u^2 - 2 u (d.e_a)) / (r + R). ``sg`` = -1 (the
    carrier's convention; +1 = the pre-2026-11-13 sum, ``legacy_phase``)."""
    i = wp.tid()
    x = d[i]
    R = wp.length(x)
    da = wp.dot(x, e_a)
    re = float(0.0)
    im = float(0.0)
    for j in range(u.shape[0]):
        uj = u[j]
        q = uj * uj - 2.0 * uj * da
        r = wp.sqrt(wp.max(R * R + q, 1.0e-18))
        dr = q / (r + R)
        a = R / wp.max(r, 1.0e-9) * w[j]
        re += a * wp.cos(k * dr)
        im += a * wp.sin(k * dr)
    out_re[i] = 0.5 * re
    out_im[i] = sg * 0.5 * im


def near_pattern(d: np.ndarray, e_a: np.ndarray, u: np.ndarray, w: np.ndarray,
                 k: float, device: str = "cuda:0", legacy_phase: bool = False) -> np.ndarray:
    """The near-field along-track factor on the GPU (``Aperture._near``)."""
    n = d.shape[0]
    re = wp.zeros(n, dtype=float, device=device)
    im = wp.zeros(n, dtype=float, device=device)
    wp.launch(_near_k, n, [wp.array(np.ascontiguousarray(d, dtype=np.float32),
                                    dtype=wp.vec3, device=device),
                           wp.vec3(*[float(x) for x in e_a]),
                           wp.array(u.astype(np.float32), device=device),
                           wp.array(w.astype(np.float32), device=device),
                           float(k), 1.0 if legacy_phase else -1.0, re, im], device=device)
    return re.numpy().astype(np.float64) + 1j * im.numpy().astype(np.float64)
