"""The ray-cast sensor's GPU pipeline (``backend: warp``; Phase 4, owner
2026-11-04: "convert everything to GPU computation").

The CPU modules (``renderer``, ``multipath``, ``chains``, ``diffraction``,
``scatterers``, ``receiver``) stay the reference; this module computes the
same quantities on the device, one fused kernel per stage with the
contributors as the thread axis, and is held to the reference by the G-gates
(``test/raycast_test.py``):

* **scatterers** -- the world-fixed lattice of ``scatterers.py``, the same
  splitmix64 hash bit for bit (a scatterer set is a pure function of the
  world seed and its lattice cell, so the GPU draws the CPU's set exactly),
  generated on the device and kept there;

Device data that depends only on the renderer (the sheet grid, the per-
triangle tables, the band table) is uploaded once (:class:`GpuScene`). The
scene's triangles and BVH are the oracle's (``gpu_warp.WarpBVH``), shared.

Imported only for ``backend: warp``; the geometric renderer, the node's
default path and the smoke test never import Warp. ROS-free, Qt-free.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import warp as wp

from . import gpu_warp as _gw          # wp.init, module options (fuse_fp off)
from .scatterers import (REFINE_STRETCH, SALT_AMP, SALT_JX, SALT_JY, SALT_PHASE, Scatterers,
                         wall_is_quay)
from .scene_mesh import KIND_APRON
from .chains import BOUNCE_OFFSET_M as _BOUNCE_OFFSET_M

# bit-for-bit with numpy where it matters; no adjoints (never differentiated; Warp's
# backward codegen also lacks in-place float64 matrix element writes)
wp.set_module_options({"fuse_fp": False, "enable_backward": False})

_K1 = wp.constant(wp.uint64(0x9E3779B97F4A7C15))
_K2 = wp.constant(wp.uint64(0xC2B2AE3D27D4EB4F))
_K3 = wp.constant(wp.uint64(0x165667B19E3779F9))
_M1 = wp.constant(wp.uint64(0xBF58476D1CE4E5B9))
_M2 = wp.constant(wp.uint64(0x94D049BB133111EB))
_INV53 = wp.constant(wp.float64(1.0 / float(1 << 53)))
_TWO_PI = wp.constant(wp.float64(2.0 * np.pi))
_S_JX = wp.constant(wp.uint64(SALT_JX))
_S_JY = wp.constant(wp.uint64(SALT_JY))
_S_PH = wp.constant(wp.uint64(SALT_PHASE))
_S_AMP = wp.constant(wp.uint64(SALT_AMP))
_KIND_APRON = wp.constant(int(KIND_APRON))
_STRETCH = wp.constant(wp.float64(REFINE_STRETCH))
_BOFF = wp.constant(wp.float64(_BOUNCE_OFFSET_M))   # chains.BOUNCE_OFFSET_M: bounce points' lift


# ===================================================================== hash
@wp.func
def _mix(x: wp.uint64) -> wp.uint64:
    x = (x ^ (x >> wp.uint64(30))) * _M1
    x = (x ^ (x >> wp.uint64(27))) * _M2
    return x ^ (x >> wp.uint64(31))


@wp.func
def hash_uniform(seed: wp.uint64, a: wp.int64, b: wp.int64, c: wp.int64,
                 salt: wp.uint64) -> wp.float64:
    """``scatterers.hash_uniform`` bit for bit: uniform [0, 1)."""
    h = seed ^ (salt * _K3)
    h = _mix(h ^ (wp.uint64(a) * _K1))
    h = _mix(h ^ (wp.uint64(b) * _K2))
    h = _mix(h ^ (wp.uint64(c) * _K3))
    return wp.float64(h >> wp.uint64(11)) * _INV53


@wp.func
def rayleigh_factor(seed: wp.uint64, a: wp.int64, b: wp.int64, c: wp.int64) -> wp.float64:
    u = hash_uniform(seed, a, b, c, _S_AMP)
    return wp.sqrt(-wp.log(wp.max(wp.float64(1.0) - u, wp.float64(1e-300))))


# ==================================================================== sheet
@wp.struct
class Sheet:
    """The seabed sheet's height grid (``SheetSampler`` / ``SheetSpecular``)."""
    z: wp.array2d(dtype=wp.float64)
    nx: int
    ny: int
    ox: wp.float64
    oy: wp.float64
    res: wp.float64
    zmin: wp.float64
    nq: int
    edge: int                  # outside == "edge"


@wp.func
def sheet_locate(s: Sheet, x: wp.float64, y: wp.float64) -> wp.vec2d:
    """``SheetSampler._locate``: (z, sheet triangle as a float, -1 beyond
    the grid)."""
    fx = (x - s.ox) / s.res
    fy = (y - s.oy) / s.res
    inside = (fx >= wp.float64(0.0)) and (fy >= wp.float64(0.0)) and \
        (fx <= wp.float64(s.nx - 1)) and (fy <= wp.float64(s.ny - 1))
    cx = wp.clamp(fx, wp.float64(0.0), wp.float64(s.nx - 1) - wp.float64(1e-9))
    cy = wp.clamp(fy, wp.float64(0.0), wp.float64(s.ny - 1) - wp.float64(1e-9))
    i = int(cx)
    j = int(cy)
    u = cx - wp.float64(i)
    v = cy - wp.float64(j)
    za = s.z[j, i]
    zb = s.z[j, i + 1]
    zc = s.z[j + 1, i + 1]
    zd = s.z[j + 1, i]
    z = wp.float64(0.0)
    tri = int(0)
    q = j * (s.nx - 1) + i
    if u >= v:
        z = za + u * (zb - za) + v * (zc - zb)
        tri = q
    else:
        z = za + u * (zc - zd) + v * (zd - za)
        tri = s.nq + q
    if not inside:
        tri = -1
        if s.edge == 0:
            z = s.zmin
    return wp.vec2d(z, wp.float64(tri))


# ============================================================== scatterers
@wp.struct
class Tri:
    """Per-triangle tables of the scene mesh."""
    normal: wp.array(dtype=wp.vec3d)      # mesh.normals() (unflipped)
    material: wp.array(dtype=wp.int32)
    object_id: wp.array(dtype=wp.int32)
    kind: wp.array(dtype=wp.int32)


@wp.func
def refine_nx(t: Tri, tri: int, max_refine: int) -> int:
    """``SheetSampler.sample``'s n_x = ceil(sqrt(1 + z_x^2) / REFINE_STRETCH) (capped)."""
    if tri < 0:
        return 1
    n = t.normal[tri]
    nz = wp.max(wp.abs(n[2]), wp.float64(1e-3))
    return wp.min(int(wp.ceil(wp.sqrt(n[2] * n[2] + n[0] * n[0]) / nz / _STRETCH)), max_refine)


@wp.func
def refine_ny(t: Tri, tri: int, max_refine: int) -> int:
    if tri < 0:
        return 1
    n = t.normal[tri]
    nz = wp.max(wp.abs(n[2]), wp.float64(1e-3))
    return wp.min(int(wp.ceil(wp.sqrt(n[2] * n[2] + n[1] * n[1]) / nz / _STRETCH)), max_refine)


@wp.func
def sheet_point(seed: wp.uint64, p: wp.int64, q: wp.int64, k: int, n: int, nxs: int, nys: int,
                h: wp.float64) -> wp.vec2d:
    """The k-th of a cell's ``n`` jittered points (``SheetSampler.sample``):
    the cell's one point when n == 1, else its stratified nxs x nys sub-grid."""
    if n > 1:
        sub = wp.int64(k + 1)
        x = (wp.float64(p) + (wp.float64(k % nxs) + hash_uniform(seed, p, q, sub, _S_JX)) /
             wp.float64(nxs)) * h
        y = (wp.float64(q) + (wp.float64(k // nxs) + hash_uniform(seed, p, q, sub, _S_JY)) /
             wp.float64(nys)) * h
        return wp.vec2d(x, y)
    return wp.vec2d((wp.float64(p) + hash_uniform(seed, p, q, wp.int64(0), _S_JX)) * h,
                    (wp.float64(q) + hash_uniform(seed, p, q, wp.int64(0), _S_JY)) * h)


@wp.func
def sheet_keep(cull: int, x: wp.float64, y: wp.float64, z: wp.float64, P: wp.vec3d,
               e_a: wp.vec3d, r_max: wp.float64, R_tab: wp.array(dtype=wp.float64),
               w_tab: wp.array(dtype=wp.float64)) -> int:
    """Whether a sheet point survives the fast variant's culls, ``cull`` a
    bit mask: 1 = ``sampling.cull_unreachable`` (nothing above the still
    waterline), 2 = ``sampling.band_cull`` (inside the beam band at its own
    range, the walls' ``keep``)."""
    if (cull & 1) != 0 and z > wp.float64(0.0):
        return 0
    if (cull & 2) != 0:
        d = wp.vec3d(x, y, z) - P
        R = wp.length(d)
        if not ((R <= r_max) and (wp.abs(wp.dot(d, e_a)) <= interp_table(R_tab, w_tab, R))):
            return 0
    return 1


@wp.kernel
def _sheet_counts(s: Sheet, t: Tri, seed: wp.uint64, h: wp.float64, q0: wp.int64,
                  p_lo: wp.array(dtype=wp.int64), n_p: wp.array(dtype=wp.int32),
                  max_refine: int, nsub: wp.array2d(dtype=wp.int32), cull: int,
                  P: wp.vec3d, e_a: wp.vec3d, r_max: wp.float64,
                  R_tab: wp.array(dtype=wp.float64), w_tab: wp.array(dtype=wp.float64)):
    """Per lattice cell (row r, column c): how many scatterers it holds
    (1, or the steep-cell refinement count, 0 outside the row's span); with
    ``cull`` != 0 (the bit mask of ``sheet_keep``) only those it keeps."""
    r, c = wp.tid()
    if c >= int(n_p[r]):
        nsub[r, c] = 0
        return
    p = p_lo[r] + wp.int64(c)
    q = q0 + wp.int64(r)
    x = (wp.float64(p) + hash_uniform(seed, p, q, wp.int64(0), _S_JX)) * h
    y = (wp.float64(q) + hash_uniform(seed, p, q, wp.int64(0), _S_JY)) * h
    tri = int(sheet_locate(s, x, y)[1])
    nxs = refine_nx(t, tri, max_refine)
    nys = refine_ny(t, tri, max_refine)
    n = nxs * nys
    if cull != 0:
        kept = int(0)
        for k in range(n):
            xy = sheet_point(seed, p, q, k, n, nxs, nys, h)
            kept += sheet_keep(cull, xy[0], xy[1], sheet_locate(s, xy[0], xy[1])[0], P, e_a,
                               r_max, R_tab, w_tab)
        n = kept
    nsub[r, c] = n


@wp.kernel
def _sheet_emit(s: Sheet, t: Tri, seed: wp.uint64, h: wp.float64, q0: wp.int64,
                p_lo: wp.array(dtype=wp.int64), n_p: wp.array(dtype=wp.int32),
                nsub: wp.array2d(dtype=wp.int32), start: wp.array2d(dtype=wp.int32),
                out_mat: int, base: int, max_refine: int, pos: wp.array(dtype=wp.vec3d),
                nrm: wp.array(dtype=wp.vec3d),
                area: wp.array(dtype=wp.float64), phase: wp.array(dtype=wp.float64),
                ray: wp.array(dtype=wp.float64), tri_o: wp.array(dtype=wp.int32),
                mat_o: wp.array(dtype=wp.int32), oid_o: wp.array(dtype=wp.int32),
                kind_o: wp.array(dtype=wp.int32), cull: int, P: wp.vec3d, e_a: wp.vec3d,
                r_max: wp.float64, R_tab: wp.array(dtype=wp.float64),
                w_tab: wp.array(dtype=wp.float64)):
    """Writes each cell's ``nsub`` scatterers at its scan slot. With ``cull``
    != 0 the cell's full sub-grid is walked and only the points ``sheet_keep``
    keeps are written; each keeps its share of the cell (1/n of the full
    sub-grid), as without culling."""
    r, c = wp.tid()
    m = int(nsub[r, c])                 # written by this cell
    if m == 0:
        return
    p = p_lo[r] + wp.int64(c)
    q = q0 + wp.int64(r)
    o = int(start[r, c]) - m + base     # exclusive start (inclusive scan)
    n = m                               # the cell's full sub-grid
    nxs = int(1)
    nys = int(1)
    if n > 1 or cull != 0:
        x0 = (wp.float64(p) + hash_uniform(seed, p, q, wp.int64(0), _S_JX)) * h
        y0 = (wp.float64(q) + hash_uniform(seed, p, q, wp.int64(0), _S_JY)) * h
        tri0 = int(sheet_locate(s, x0, y0)[1])
        nxs = refine_nx(t, tri0, max_refine)
        nys = refine_ny(t, tri0, max_refine)
        if cull != 0:
            n = nxs * nys
    j = int(0)
    for k in range(n):
        sub = wp.int64(0)
        if n > 1:
            sub = wp.int64(k + 1)
        xy = sheet_point(seed, p, q, k, n, nxs, nys, h)
        x = xy[0]
        y = xy[1]
        zt = sheet_locate(s, x, y)
        z = zt[0]
        tri = int(zt[1])
        if cull != 0:
            if sheet_keep(cull, x, y, z, P, e_a, r_max, R_tab, w_tab) == 0:
                continue
        nv = wp.vec3d(wp.float64(0.0), wp.float64(0.0), wp.float64(1.0))
        mat = out_mat
        oid = -1
        kind = _KIND_APRON
        if tri >= 0:
            nv = t.normal[tri]
            mat = int(t.material[tri])
            oid = int(t.object_id[tri])
            kind = int(t.kind[tri])
        if nv[2] < wp.float64(0.0):
            nv = -nv
        i = o + j
        j += 1
        pos[i] = wp.vec3d(x, y, z)
        nrm[i] = nv
        area[i] = h * h / wp.max(wp.abs(nv[2]), wp.float64(1e-3)) / wp.float64(n)
        phase[i] = _TWO_PI * hash_uniform(seed, p, q, sub, _S_PH)
        ray[i] = rayleigh_factor(seed, p, q, sub)
        tri_o[i] = tri
        mat_o[i] = mat
        oid_o[i] = oid
        kind_o[i] = kind


@wp.func
def interp_table(xs: wp.array(dtype=wp.float64), ys: wp.array(dtype=wp.float64),
                 x: wp.float64) -> wp.float64:
    """``np.interp`` (clamped ends) on an increasing table, binary search."""
    n = xs.shape[0]
    if x <= xs[0]:
        return ys[0]
    if x >= xs[n - 1]:
        return ys[n - 1]
    lo = int(0)
    hi = int(n - 1)
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    t = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + t * (ys[hi] - ys[lo])


@wp.kernel
def _wall_emit(seed: wp.uint64, h: wp.float64, P: wp.vec3d, e_a: wp.vec3d, r_max: wp.float64,
               R_tab: wp.array(dtype=wp.float64), w_tab: wp.array(dtype=wp.float64),
               tri_id: wp.array(dtype=wp.int32), A: wp.array(dtype=wp.vec3d),
               e1: wp.array(dtype=wp.vec3d), e2: wp.array(dtype=wp.vec3d),
               v1: wp.array(dtype=wp.vec2d), v2: wp.array(dtype=wp.vec2d),
               ip0: wp.array(dtype=wp.int64), iq0: wp.array(dtype=wp.int64),
               nip: wp.array(dtype=wp.int64), cell0: wp.array(dtype=wp.int64),
               t: Tri, s_: Sheet, cull: int, offset: wp.float64,
               write: int, base: int, counter: wp.array(dtype=wp.int32),
               pos: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3d),
               area: wp.array(dtype=wp.float64), phase: wp.array(dtype=wp.float64),
               ray: wp.array(dtype=wp.float64), tri_o: wp.array(dtype=wp.int32),
               mat_o: wp.array(dtype=wp.int32), oid_o: wp.array(dtype=wp.int32),
               kind_o: wp.array(dtype=wp.int32)):
    """``scatterers.planar_sample`` with the renderer's band ``keep``: one
    thread per lattice cell of every wall triangle (cells limited to the
    r_max box around P, which ``keep`` would reject anyway). ``write`` 0
    counts, 1 emits (output order is not the CPU's; the set is). ``cull``
    (``sampling.cull_unreachable``) drops points above the still waterline
    and points buried below the seabed by more than ``offset``, in both
    passes alike."""
    g = wp.tid()
    # the triangle of this cell: binary search on cell0 (exclusive starts)
    lo = int(0)
    hi = int(cell0.shape[0] - 1)
    gg = wp.int64(g)
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if cell0[mid] <= gg:
            lo = mid
        else:
            hi = mid
    k = lo
    loc = gg - cell0[k]
    pp = ip0[k] + loc % nip[k]
    qq = iq0[k] + loc // nip[k]
    tt = wp.int64(tri_id[k])
    s = (wp.float64(pp) + hash_uniform(seed, tt, pp, qq, _S_JX)) * h
    r = (wp.float64(qq) + hash_uniform(seed, tt, pp, qq, _S_JY)) * h
    a0 = v1[k]
    b0 = v2[k]
    den = a0[0] * b0[1] - a0[1] * b0[0]
    a = (s * b0[1] - r * b0[0]) / den
    b = (a0[0] * r - a0[1] * s) / den
    if not ((a >= wp.float64(0.0)) and (b >= wp.float64(0.0)) and (a + b <= wp.float64(1.0))):
        return
    x = A[k] + s * e1[k] + r * e2[k]
    d = x - P
    R = wp.length(d)
    if not ((R <= r_max) and (wp.abs(wp.dot(d, e_a)) <= interp_table(R_tab, w_tab, R))):
        return
    if cull == 1:
        if x[2] > wp.float64(0.0):
            return
        if x[2] < sheet_locate(s_, x[0], x[1])[0] - offset:
            return
    i = wp.atomic_add(counter, 0, 1) + base
    if write == 0:
        return
    tri = int(tri_id[k])
    pos[i] = x
    nrm[i] = t.normal[tri]
    area[i] = h * h
    phase[i] = _TWO_PI * hash_uniform(seed, tt, pp, qq, _S_PH)
    ray[i] = rayleigh_factor(seed, tt, pp, qq)
    tri_o[i] = tri
    mat_o[i] = int(t.material[tri])
    oid_o[i] = int(t.object_id[tri])
    kind_o[i] = int(t.kind[tri])


@dataclass
class GpuScatterers:
    """A scatterer set on the device (struct of arrays, ``Scatterers``'s
    fields)."""
    pos: wp.array
    normal: wp.array
    area: wp.array
    phase: wp.array
    rayleigh: wp.array
    tri: wp.array
    material: wp.array
    object_id: wp.array
    kind: wp.array

    def __len__(self) -> int:
        return int(self.area.shape[0])

    def numpy(self) -> Scatterers:
        """The set as a CPU ``Scatterers`` (tests; never on the render path)."""
        return Scatterers(self.pos.numpy(), self.normal.numpy(), self.area.numpy(),
                          self.phase.numpy(), self.tri.numpy().astype(np.int64),
                          self.material.numpy().astype(np.int64),
                          self.object_id.numpy().astype(np.int64),
                          self.kind.numpy().astype(np.uint8), self.rayleigh.numpy())


def _last(a) -> int:
    """The last element of a non-empty device array (an inclusive scan's
    total), copied alone instead of the whole array."""
    return int(a[-1:].numpy()[0])


def _alloc(n: int, device: str) -> list:
    z = lambda dt: wp.empty(n, dtype=dt, device=device)          # noqa: E731
    return [z(wp.vec3d), z(wp.vec3d), z(wp.float64), z(wp.float64), z(wp.float64),
            z(wp.int32), z(wp.int32), z(wp.int32), z(wp.int32)]


# ================================================================ the scene
class GpuScene:
    """The renderer's device data, uploaded once (``RaycastRenderer``'s
    ``_gpu``)."""

    def __init__(self, rr, device: str = "cuda:0") -> None:
        self.rr = rr
        self.device = device
        m = rr.mesh
        sh = rr.sheet
        g = sh.g
        s = Sheet()
        s.z = wp.array(np.ascontiguousarray(sh.z, dtype=np.float64), dtype=wp.float64,
                       device=device)
        s.nx, s.ny = int(g.nx), int(g.ny)
        s.ox, s.oy, s.res = wp.float64(g.origin_x), wp.float64(g.origin_y), wp.float64(g.resolution)
        s.zmin = wp.float64(sh.zmin)
        s.nq = int(sh.nq)
        s.edge = 1 if sh.outside == "edge" else 0
        self.sheet = s
        t = Tri()
        t.normal = wp.array(np.ascontiguousarray(sh.tri_normal, dtype=np.float64),
                            dtype=wp.vec3d, device=device)
        t.material = wp.array(m.material.astype(np.int32), dtype=wp.int32, device=device)
        t.object_id = wp.array(m.object_id.astype(np.int32), dtype=wp.int32, device=device)
        t.kind = wp.array(m.kind.astype(np.int32), dtype=wp.int32, device=device)
        self.tri = t
        self.R_tab = wp.array(rr._R_tab.astype(np.float64), dtype=wp.float64, device=device)
        self.w_tab = wp.array(rr._w_tab.astype(np.float64), dtype=wp.float64, device=device)
        self.seed = wp.uint64(int(rr.seed) & 0xFFFFFFFFFFFFFFFF)
        self.h = float(rr._rc.sampling.scatterer_spacing_m)
        self.max_refine = int(rr._rc.sampling.max_refine)
        self.out_mat = int(sh.out_mat)
        #: sampling.cull_unreachable (fast variant): 1 = drop unreachable points
        self.cull = int(bool(rr._rc.sampling.cull_unreachable))
        #: the sheet's cull bit mask (``sheet_keep``): 1 unreachable, 2 band
        self.sheet_cull = self.cull | (2 if bool(rr._rc.sampling.band_cull) else 0)
        self.offset = float(rr._rc.sampling.surface_offset_m)
        self._wall_rows = np.zeros(0, dtype=np.int64)
        self._obj_groups: list = []
        self._walls = self._wall_lattices()

    # -------------------------------------------------------------- walls
    def _wall_lattices(self):
        """Per wall triangle, its in-plane frame (``planar_sample``), whether
        its wall is a quay, and its lowest vertex z, as column arrays (the
        plan below is vectorised: 2026-11-10, a Python loop over every wall
        triangle cost ~5 ms per ping-side even with no quay in range)."""
        rr = self.rr
        m = rr.mesh
        tris = rr._walls
        if tris.size == 0:
            return None
        tv = m.vertices[m.triangles[tris]]
        A, B, C = tv[:, 0], tv[:, 1], tv[:, 2]
        e1 = B - A
        L1 = np.linalg.norm(e1, axis=1)
        ok = L1 > 0
        if not ok.any():
            return None
        A, B, C, L1, nrm = A[ok], B[ok], C[ok], L1[ok], m.normals()[tris][ok]
        e1 = (B - A) / L1[:, None]
        e2 = np.cross(nrm, e1)
        cx, cy = np.einsum("ij,ij->i", C - A, e1), np.einsum("ij,ij->i", C - A, e2)
        zero = np.zeros_like(L1)
        P2x = np.stack([zero, L1, cx], axis=1)          # the triangle in its plane
        P2y = np.stack([zero, zero, cy], axis=1)
        out = dict(t=tris[ok].astype(np.int32), A=A, e1=e1, e2=e2,
                   nn=np.cross(e1, e2),
                   v1=np.stack([L1, zero], axis=1), v2=np.stack([cx, cy], axis=1),
                   lo=np.stack([P2x.min(axis=1), P2y.min(axis=1)], axis=1),
                   hi=np.stack([P2x.max(axis=1), P2y.max(axis=1)], axis=1),
                   quay=np.asarray(wall_is_quay(rr, tris), bool)[ok],
                   zlo=tv[ok][:, :, 2].min(axis=1))
        # objects.mode solid (2026-11-11): the rough objects' own triangles are
        # sampled like wall triangles. Grouped per object with a bounding
        # sphere, so a ping plans only the objects it can reach (an object
        # mesh can hold thousands of triangles); the walls are always planned.
        # No object triangle (objects.mode raster): no group, the plan of before.
        oid = m.object_id[tris][ok]
        self._wall_rows = np.flatnonzero(oid < 0)
        self._obj_groups = []
        for o in np.unique(oid[oid >= 0]):
            rows = np.flatnonzero(oid == o)
            pts = tv[ok][rows].reshape(-1, 3)
            c = pts.mean(axis=0)
            self._obj_groups.append((rows, c, float(np.linalg.norm(pts - c, axis=1).max())))
        return out

    def _wall_plan(self, P: np.ndarray, e_a: np.ndarray):
        """The walls' lattice launch (cells within the r_max box) and the
        count of band-kept scatterers, or None. The cell ranges are a
        conservative superset; ``_wall_emit`` applies the exact per-point
        tests, so the emitted set does not depend on them."""
        L = self._walls
        if L is None:
            return None
        rr, h, dev = self.rr, self.h, self.device
        rmax = float(rr.r_max)
        W = float(rr._w_tab[-1]) + h          # the band's largest half-width (+ a cell)
        P = np.asarray(P, float)
        e_a = np.asarray(e_a, float)
        if self._obj_groups:
            # only the objects this ping can reach (their bounding spheres
            # against the range and the band), plus every wall
            sel = [self._wall_rows]
            for rows, c, rad in self._obj_groups:
                dc = c - P
                if (np.linalg.norm(dc) - rad <= rmax) and (abs(float(dc @ e_a)) - rad <= W):
                    sel.append(rows)
            rows = np.concatenate(sel) if len(sel) > 1 else sel[0]
            if rows.size == 0:
                return None
            L = {k: v[rows] for k, v in L.items()}
        dPA = P - L["A"]
        sp = np.einsum("ij,ij->i", dPA, L["e1"])
        rp = np.einsum("ij,ij->i", dPA, L["e2"])
        dn = np.einsum("ij,ij->i", dPA, L["nn"])
        keep = np.abs(dn) <= rmax
        if self.cull:
            # scatterers.cull_wall_triangle, vectorised (the CPU reference uses it)
            keep &= ~((L["zlo"] > 0.0) | (L["quay"] & (dn <= 0.0)))
        lo, hi = L["lo"], L["hi"]
        l0, h0 = np.maximum(lo[:, 0], sp - rmax), np.minimum(hi[:, 0], sp + rmax)
        l1, h1 = np.maximum(lo[:, 1], rp - rmax), np.minimum(hi[:, 1], rp + rmax)
        keep &= (h0 >= l0) & (h1 >= l1)
        # the beam band crosses the plane in the strip |c0 + s a1 + r a2| <= W
        # (keep needs |d . e_a| <= w(R) <= W): the box cut to the strip's
        # extent, still a superset of the kept cells
        a1, a2 = L["e1"] @ e_a, L["e2"] @ e_a
        c0 = -(dPA @ e_a)
        with np.errstate(divide="ignore", invalid="ignore"):
            on1 = np.abs(a1) > 1e-9
            ss = np.stack([(tt - c0 - r_ * a2) / np.where(on1, a1, 1.0)
                           for tt in (-W, W) for r_ in (l1, h1)])
            l0 = np.where(on1, np.maximum(l0, ss.min(axis=0)), l0)
            h0 = np.where(on1, np.minimum(h0, ss.max(axis=0)), h0)
            on2 = np.abs(a2) > 1e-9
            rs = np.stack([(tt - c0 - s_ * a1) / np.where(on2, a2, 1.0)
                           for tt in (-W, W) for s_ in (l0, h0)])
            l1 = np.where(on2, np.maximum(l1, rs.min(axis=0)), l1)
            h1 = np.where(on2, np.minimum(h1, rs.max(axis=0)), h1)
        keep &= ~(~on1 & ~on2 & (np.abs(c0) > W))
        keep &= (h0 >= l0) & (h1 >= l1)
        if not keep.any():
            return None
        k = np.flatnonzero(keep)
        ip0, ip1 = np.floor(l0[k] / h).astype(np.int64), np.floor(h0[k] / h).astype(np.int64)
        iq0, iq1 = np.floor(l1[k] / h).astype(np.int64), np.floor(h1[k] / h).astype(np.int64)
        nip = ip1 - ip0 + 1
        cell0 = np.concatenate([[0], np.cumsum(nip * (iq1 - iq0 + 1))]).astype(np.int64)
        arr = lambda v, dt: wp.array(np.ascontiguousarray(v), dtype=dt, device=dev)  # noqa: E731
        common = [self.seed, wp.float64(h), wp.vec3d(*P), wp.vec3d(*e_a), wp.float64(rmax),
                  self.R_tab, self.w_tab,
                  arr(L["t"][k], wp.int32), arr(L["A"][k], wp.vec3d),
                  arr(L["e1"][k], wp.vec3d), arr(L["e2"][k], wp.vec3d),
                  arr(L["v1"][k], wp.vec2d), arr(L["v2"][k], wp.vec2d),
                  arr(ip0, wp.int64), arr(iq0, wp.int64),
                  arr(nip, wp.int64), arr(cell0, wp.int64), self.tri,
                  self.sheet, self.cull, wp.float64(self.offset)]
        total = int(cell0[-1])
        counter = wp.zeros(1, dtype=wp.int32, device=dev)
        wp.launch(_wall_emit, total, common + [0, 0, counter] + _alloc(1, dev), device=dev)
        n = int(counter.numpy()[0])
        return (common, total, n) if n else None

    def _sheet_args(self, P, e_a) -> list:
        """The cull arguments the sheet's count and emit kernels share."""
        return [self.sheet_cull, wp.vec3d(*np.asarray(P, float)),
                wp.vec3d(*np.asarray(e_a, float)), wp.float64(self.rr.r_max),
                self.R_tab, self.w_tab]

    def _sheet_plan(self, lo: np.ndarray, hi: np.ndarray, y0: float, P, e_a):
        """The sheet's per-cell scatterer counts (steep cells refined) and
        their scan, or None."""
        h, dev = self.h, self.device
        q0 = int(np.floor(y0 / h))
        ok = hi > lo
        p_lo = np.where(ok, np.floor(np.where(ok, lo, 0.0) / h), 0).astype(np.int64)
        p_hi = np.where(ok, np.floor(np.where(ok, hi, 0.0) / h), -1).astype(np.int64)
        n_p = np.maximum(p_hi - p_lo + 1, 0).astype(np.int32)
        nrow = n_p.size
        maxw = int(n_p.max()) if nrow else 0
        if maxw == 0:
            return None
        P_lo = wp.array(p_lo, dtype=wp.int64, device=dev)
        N_p = wp.array(n_p, dtype=wp.int32, device=dev)
        nsub = wp.empty((nrow, maxw), dtype=wp.int32, device=dev)
        wp.launch(_sheet_counts, (nrow, maxw),
                  [self.sheet, self.tri, self.seed, wp.float64(h), wp.int64(q0), P_lo, N_p,
                   self.max_refine, nsub] + self._sheet_args(P, e_a), device=dev)
        flat = nsub.flatten()
        start = wp.empty_like(flat)
        wp.utils.array_scan(flat, start, inclusive=True)
        n = _last(start)
        return (q0, P_lo, N_p, nsub, start.reshape((nrow, maxw)), (nrow, maxw), n) if n else None

    def scatterers(self, P: np.ndarray, e_a: np.ndarray, e_n: np.ndarray) -> GpuScatterers:
        """``RaycastRenderer._scatterers`` on the device: the sheet's band
        region and the walls' band-kept lattices, counted first and written
        into one allocation."""
        lo, hi, y0 = self.rr._sheet_region(P, e_a, e_n)
        sp = self._sheet_plan(lo, hi, y0, P, e_a)
        wpl = self._wall_plan(P, e_a)
        ns = sp[-1] if sp else 0
        nw = wpl[2] if wpl else 0
        out = _alloc(ns + nw, self.device)
        if sp:
            q0, P_lo, N_p, nsub, start, dims, _ = sp
            wp.launch(_sheet_emit, dims,
                      [self.sheet, self.tri, self.seed, wp.float64(self.h), wp.int64(q0), P_lo, N_p,
                       nsub, start, self.out_mat, 0, self.max_refine] + out
                      + self._sheet_args(P, e_a),
                      device=self.device)
        if wpl:
            common, total, _ = wpl
            counter = wp.zeros(1, dtype=wp.int32, device=self.device)
            wp.launch(_wall_emit, total, common + [1, ns, counter] + out, device=self.device)
        return GpuScatterers(*out)


# ================================================================== complex
# Generic (typing.Any): one definition for vec2d (float64) and vec2f (float32);
# the precision of a call is its arguments'.
@wp.func
def cmul(a: Any, b: Any):
    return type(a)(a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0])


@wp.func
def cdiv(a: Any, b: Any):
    d = b[0] * b[0] + b[1] * b[1]
    return type(a)((a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d)


@wp.func
def cabs(a: Any):
    return wp.sqrt(a[0] * a[0] + a[1] * a[1])


@wp.func
def cabs2(a: Any):
    return a[0] * a[0] + a[1] * a[1]


@wp.func
def cconj(a: Any):
    return type(a)(a[0], -a[1])


@wp.func
def csqrt(z: Any):
    """Principal square root (Re >= 0; the cut on the negative real axis
    gives +i sqrt|x| for a +0 imaginary part), as numpy's."""
    x = z[0]
    y = z[1]
    zero = type(x)(0.0)
    two = type(x)(2.0)
    half = type(x)(0.5)
    r = wp.sqrt(x * x + y * y)
    out = type(z)(zero, zero)
    if r == zero:
        return out
    if x >= zero:
        t = wp.sqrt(half * (r + x))
        out = type(z)(t, y / (two * t))
    else:
        t = wp.sqrt(half * (r - x))
        im = t
        if y < zero:
            im = -t
        out = type(z)(wp.abs(y) / (two * t), im)
    return out


@wp.func
def cexpi(ph: wp.float64) -> wp.vec2d:
    return wp.vec2d(wp.cos(ph), wp.sin(ph))


@wp.func
def cexpi32(ph: wp.float32) -> wp.vec2f:
    return wp.vec2f(wp.cos(ph), wp.sin(ph))


# ============================================================ seabed law
@wp.struct
class Law:
    """``scattering.DiffuseLaw`` on the device: per law (sediment) the J&R
    parameters and its I_K table; per material its law and scale."""
    a_rho: wp.array(dtype=wp.float64)
    ap: wp.array(dtype=wp.vec2d)
    ch2: wp.array(dtype=wp.float64)
    alpha: wp.array(dtype=wp.float64)
    s_v: wp.array(dtype=wp.float64)
    ik_off: wp.array(dtype=wp.int32)
    ik_n: wp.array(dtype=wp.int32)
    ik_lq0: wp.array(dtype=wp.float64)
    ik_dlq: wp.array(dtype=wp.float64)
    ik_slope: wp.array(dtype=wp.float64)
    ik_beta: wp.array(dtype=wp.float64)
    ik_lv: wp.array(dtype=wp.float64)
    law_of: wp.array(dtype=wp.int32)
    scale: wp.array(dtype=wp.float64)
    kr2: wp.array(dtype=wp.float64)
    keep: wp.array(dtype=wp.float64)
    cov: wp.array(dtype=wp.float64)
    k: wp.float64
    lambert: int
    mu: wp.float64


@wp.func
def tiny(x: Any):
    """The smallest positive guard of x's precision (the CPU's 1e-300 in
    float64; 1e-37 in float32, where 1e-300 underflows)."""
    if wp.float64(type(x)(1e-300)) == wp.float64(0.0):
        return type(x)(1e-37)
    return type(x)(1e-300)


@wp.func
def sin_theta_p(ap: Any, cos_th: Any):
    """``seabed_ssa.sin_theta_p``: the decaying branch."""
    c2 = cos_th * cos_th
    a2 = cmul(ap, ap)
    s = csqrt(type(ap)(type(cos_th)(1.0) - a2[0] * c2, -a2[1] * c2))
    if cdiv(s, ap)[1] < type(cos_th)(0.0):
        s = -s
    return s


@wp.func
def reflection(a_rho: Any, ap: Any, sin_th: Any):
    """``seabed_ssa.reflection``: J&R 8.49, the critical-angle guard."""
    one = type(sin_th)(1.0)
    zero = type(sin_th)(0.0)
    cos_th = wp.sqrt(wp.max(one - sin_th * sin_th, zero))
    stp = sin_theta_p(ap, cos_th)
    if cabs(stp) < type(sin_th)(1e-15):
        return type(ap)(one, zero)
    z = cdiv(type(ap)(a_rho * ap[0] * sin_th, a_rho * ap[1] * sin_th), stp)
    return cdiv(type(ap)(z[0] - one, z[1]), type(ap)(z[0] + one, z[1]))


@wp.func
def ik_eval(L: Law, j: int, q: Any):
    """``seabed_ssa.IkTable.__call__``: log-log interpolation with the SPM and
    Kirchhoff asymptotes outside the table."""
    lq = wp.log(wp.max(q, tiny(q)))
    off = int(L.ik_off[j])
    n = int(L.ik_n[j])
    lq0 = type(q)(L.ik_lq0[j])
    dlq = type(q)(L.ik_dlq[j])
    lq1 = lq0 + type(q)(n - 1) * dlq
    v = type(q)(0.0)
    if lq < lq0:
        v = type(q)(L.ik_lv[off]) + type(q)(L.ik_slope[j]) * (lq - lq0)
    elif lq > lq1:
        v = type(q)(L.ik_lv[off + n - 1]) - (type(q)(2.0) / type(q)(L.ik_beta[j])) * (lq - lq1)
    else:
        f = (lq - lq0) / dlq
        i = wp.min(int(f), n - 2)
        t = f - type(q)(i)
        v0 = type(q)(L.ik_lv[off + i])
        v = v0 + t * (type(q)(L.ik_lv[off + i + 1]) - v0)
    return wp.exp(v)


@wp.func
def ssa_sigma(L: Law, j: int, sin_i: Any, sin_s: Any, cphi: Any):
    """``SSALaw.sigma``: roughness (small-slope) + volume, in the precision of
    the angles."""
    one = type(sin_i)(1.0)
    two = type(sin_i)(2.0)
    half = type(sin_i)(0.5)
    si = wp.clamp(sin_i, type(sin_i)(1e-6), one)
    ss = wp.clamp(sin_s, type(sin_i)(1e-6), one)
    ci = wp.sqrt(one - si * si)
    cs = wp.sqrt(one - ss * ss)
    a_rho = type(sin_i)(L.a_rho[j])
    apd = L.ap[j]
    ap = wp.vector(type(sin_i)(apd[0]), type(sin_i)(apd[1]))
    k = type(sin_i)(L.k)
    Vi = reflection(a_rho, ap, si)
    Vs = reflection(a_rho, ap, ss)
    ap2 = cmul(ap, ap)
    B = cdiv(cmul(sin_theta_p(ap, ci), sin_theta_p(ap, cs)), type(ap)(ap2[0] * a_rho, ap2[1] * a_rho))
    one_m = one - one / a_rho
    inv = cdiv(type(ap)(one, type(sin_i)(0.0)), type(ap)(ap2[0] * a_rho, ap2[1] * a_rho))
    G = type(ap)(one_m * (ci * cs * cphi - B[0]) - one + inv[0], one_m * (-B[1]) + inv[1])
    A = cmul(cmul(type(ap)(one + Vi[0], Vi[1]), type(ap)(one + Vs[0], Vs[1])), G)
    A = type(ap)(half * A[0], half * A[1])
    dK2 = k * k * (ci * ci + cs * cs - two * ci * cs * cphi)
    dK2 = wp.max(dK2, (type(sin_i)(1e-9) * k) * (type(sin_i)(1e-9) * k))
    dkz = k * (si + ss)
    q = half * type(sin_i)(L.ch2[j]) * dkz * dkz * wp.pow(dK2, -type(sin_i)(L.alpha[j]))
    rough = (k * k * k * k * cabs2(A) / (type(sin_i)(6.283185307179586) * dK2 * dkz * dkz)) * ik_eval(L, j, q)
    ap_m2 = cdiv(type(ap)(one, type(sin_i)(0.0)), ap2)
    Pi = csqrt(type(ap)(ap_m2[0] - (one - si * si), ap_m2[1]))
    if Pi[1] < type(sin_i)(0.0):
        Pi = -Pi
    Ps = csqrt(type(ap)(ap_m2[0] - (one - ss * ss), ap_m2[1]))
    if Ps[1] < type(sin_i)(0.0):
        Ps = -Ps
    den = two * k * a_rho * a_rho * (Pi[1] + Ps[1])
    vol = (cabs2(type(ap)(one + Vi[0], Vi[1])) * cabs2(type(ap)(one + Vs[0], Vs[1])) *
           type(sin_i)(L.s_v[j]) / wp.max(den, tiny(den)))
    return rough + vol


@wp.func
def law_factor(L: Law, mat: int, ci: Any, cs: Any):
    """``DiffuseLaw._factor``: the material's reflection scale, its diffuse
    share ``1 - C`` (a smooth material), the fouled fraction."""
    a = type(ci)(L.scale[mat])
    q = type(ci)(L.kr2[mat])
    if q > type(ci)(0.0):
        a = a * (type(ci)(1.0) - wp.exp(-type(ci)(0.25) * q * (ci + cs) * (ci + cs)))
    return a * type(ci)(L.keep[mat]) + type(ci)(L.cov[mat])


@wp.func
def sigma_mono(L: Law, cos_i: Any, mat: int):
    """``DiffuseLaw.sigma(cos_i, cos_i, mat)``."""
    ci = wp.clamp(cos_i, type(cos_i)(0.0), type(cos_i)(1.0))
    if L.lambert == 1:
        return type(cos_i)(L.mu) * ci * ci
    if ci <= type(cos_i)(0.0):
        return type(cos_i)(0.0)
    return ssa_sigma(L, int(L.law_of[mat]), ci, ci, type(cos_i)(-1.0)) * law_factor(L, mat, ci, ci)


@wp.func
def sigma_bistatic(L: Law, n: Any, d_in: Any, d_out: Any, mat: int):
    """``DiffuseLaw.sigma_bistatic`` (``paths.bistatic_angles`` geometry), in
    the precision of the vectors."""
    si = -wp.dot(n, d_in)
    ss = wp.dot(n, d_out)
    zero = type(si)(0.0)
    ki = d_in + si * n
    ks = d_out - ss * n
    den = wp.length(ki) * wp.length(ks)
    cphi = type(si)(-1.0)
    if den > type(si)(1e-12):
        cphi = wp.dot(ki, ks) / wp.max(den, tiny(den))
    cphi = wp.clamp(cphi, type(si)(-1.0), type(si)(1.0))
    if not ((si > zero) and (ss > zero)):
        return zero
    si = wp.clamp(si, zero, type(si)(1.0))
    ss = wp.clamp(ss, zero, type(si)(1.0))
    if L.lambert == 1:
        return type(si)(L.mu) * si * ss
    return ssa_sigma(L, int(L.law_of[mat]), si, ss, cphi) * law_factor(L, mat, si, ss)


# ================================================================== aperture
@wp.struct
class Beam:
    """``beam.Aperture`` (near field) and the Gauss-Legendre ladder."""
    L: wp.float64
    H: wp.float64
    k: wp.float64
    vfloor: wp.float64
    near: int
    gl_n: wp.array(dtype=wp.int32)
    gl_off: wp.array(dtype=wp.int32)
    gl_u: wp.array(dtype=wp.float32)       # 0.5 L x (node positions, m)
    gl_w: wp.array(dtype=wp.float32)


@wp.func
def sinc_x(x: Any):
    if x == type(x)(0.0):
        return type(x)(1.0)
    return wp.sin(x) / x


@wp.func
def pattern(B: Beam, d64: wp.vec3d, e_a64: wp.vec3d, e_n64: wp.vec3d, e_v64: wp.vec3d) -> wp.vec2d:
    """``Aperture.pattern`` in float32: the near-field line integral along
    track (node count from the point's own phase gradient, the CPU's rule
    per point; the stable difference r - R), the vertical strip's sinc with
    the floor added in power, the baffle."""
    d = wp.vec3f(d64)
    e_a = wp.vec3f(e_a64)
    R = wp.length(d)
    Rs = wp.max(R, 1.0e-9)
    s_n = wp.dot(d, wp.vec3f(e_n64)) / Rs
    if not (s_n > 0.0):
        return wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    s_a = wp.dot(d, e_a) / Rs
    s_v = wp.dot(d, wp.vec3f(e_v64)) / Rs
    k = wp.float32(B.k)
    Lap = wp.float32(B.L)
    vert = sinc_x(0.5 * k * wp.float32(B.H) * s_v)
    vf = wp.float32(B.vfloor)
    if vf > 0.0:
        vert = wp.sqrt(vert * vert + vf * vf)
    re = float(0.0)
    im = float(0.0)
    if B.near == 0:
        re = sinc_x(0.5 * k * Lap * s_a)
    else:
        grad = k * (wp.abs(s_a) + Lap / (2.0 * wp.max(R, 1.0e-3)))
        need = int(wp.ceil(grad * Lap / 6.2831853 * 6.0)) + 16
        need = wp.clamp(need, 32, 4096)
        g = int(0)
        while g < B.gl_n.shape[0] - 1 and int(B.gl_n[g]) < need:
            g = g + 1
        off = int(B.gl_off[g])
        nn = int(B.gl_n[g])
        da = wp.dot(d, e_a)
        for j in range(nn):
            uj = B.gl_u[off + j]
            q = uj * uj - 2.0 * uj * da
            r = wp.sqrt(wp.max(R * R + q, 1.0e-18))
            dr = q / (r + R)
            a = R / wp.max(r, 1.0e-9) * B.gl_w[off + j]
            re += a * wp.cos(k * dr)
            im += a * wp.sin(k * dr)
        re = 0.5 * re
        im = 0.5 * im
    return wp.vec2d(wp.float64(re * vert), wp.float64(im * vert))


# ================================================================ visibility
@wp.struct
class Oracle:
    """The scene oracle on the device (``gpu_warp.WarpBVH`` + z = 0)."""
    bvh: wp.uint64
    V: wp.array(dtype=wp.vec3d)
    V32: wp.array(dtype=wp.vec3)            # local frame, float32 (sampling uses)
    T: wp.array(dtype=wp.vec3i)
    origin: wp.vec3d
    zs: wp.float64
    #: ``precision: fp32`` (fast variant): seg_blocked runs seg_blocked32
    use32: int


@wp.func
def seg_blocked32(o: Oracle, a: wp.vec3d, b: wp.vec3d, eps: wp.float64) -> int:
    """``seg_blocked`` with the float32 watertight test in the BVH's local
    frame (positions to ~1e-5 m at 150 m from its centre, far below the
    1 mm surface offset of every leg end)."""
    al = wp.vec3(a - o.origin)
    d = wp.vec3(b - o.origin) - al
    L = wp.length(d)
    if L <= 0.0:
        return 0
    u = d / L
    e = wp.float32(eps)
    tmx = wp.max(L - e, 0.0)
    q = wp.bvh_query_ray(o.bvh, al, u)
    idx = int(0)
    hit = int(0)
    cut = tmx * _gw._CUT_K32 + _gw._CUT_A32
    while wp.bvh_query_next(q, idx, cut):
        tri = o.T[idx]
        r = _gw._watertight32(al, u, o.V32[tri[0]], o.V32[tri[1]], o.V32[tri[2]])
        if r[0] > e and r[0] <= tmx:
            hit = 1
            break
    return hit


@wp.func
def seg_blocked(o: Oracle, a: wp.vec3d, b: wp.vec3d, eps: wp.float64) -> int:
    """``WarpBVH.occluded``: the open segment a -> b, shortened by ``eps`` at
    both ends, crosses a triangle (1) or not (0). The float32 test when the
    oracle says so (``precision: fp32``): every visibility test of the direct,
    pair, facet and floor-coherent kernels passes here."""
    if o.use32 == 1:
        return seg_blocked32(o, a, b, eps)
    d = b - a
    L = wp.length(d)
    if L <= wp.float64(0.0):
        return 0
    u = d / L
    tmx = wp.max(L - eps, wp.float64(0.0))
    q = wp.bvh_query_ray(o.bvh, wp.vec3(a - o.origin), wp.vec3(u))
    idx = int(0)
    hit = int(0)
    cut = wp.float32(tmx * _gw._CUT_K64 + _gw._CUT_A64)
    while wp.bvh_query_next(q, idx, cut):
        tri = o.T[idx]
        r = _gw._watertight(a, u, o.V[tri[0]], o.V[tri[1]], o.V[tri[2]])
        if r[0] > eps and r[0] <= tmx:
            hit = 1
            break
    return hit


@wp.func
def seg_occluded32(o: Oracle, a: wp.vec3d, b: wp.vec3d, eps: wp.float64) -> int:
    """``seg_occluded`` with the float32 watertight test in the local frame
    (diffraction's visibility samples: their edges are located to 2^-7 of a
    zone, far above float32's ~1e-6 m over a leg)."""
    if (wp.max(a[2], b[2]) > o.zs + eps) and (wp.min(a[2], b[2]) < o.zs - eps):
        return 1
    return seg_blocked32(o, a, b, eps)


@wp.func
def seg_occluded(o: Oracle, a: wp.vec3d, b: wp.vec3d, eps: wp.float64) -> int:
    """``SceneOracle.occluded``: blocked by a triangle or crossing the sea
    surface."""
    if (wp.max(a[2], b[2]) > o.zs + eps) and (wp.min(a[2], b[2]) < o.zs - eps):
        return 1
    return seg_blocked(o, a, b, eps)


@wp.struct
class Layers:
    """``SceneOracle.layer``: flooded thin shells transmit (fluid layer)."""
    on: int
    mask: wp.array(dtype=wp.int32)
    rho: wp.array(dtype=wp.float64)
    cr: wp.array(dtype=wp.float64)
    d: wp.array(dtype=wp.float64)
    k1: wp.float64
    normal: wp.array(dtype=wp.vec3d)       # mesh.normals() (unflipped)


@wp.func
def seg_closest(o: Oracle, a: wp.vec3d, u: wp.vec3d, tmin: wp.float64,
                tmax: wp.float64) -> wp.vec2d:
    """``WarpBVH.closest`` for one ray (unit direction): (t, triangle as a
    float; -1 = no hit), the lowest-index tie rule."""
    q = wp.bvh_query_ray(o.bvh, wp.vec3(a - o.origin), wp.vec3(u))
    idx = int(0)
    best_t = tmax
    best_f = int(-1)
    cut = wp.float32(best_t * _gw._CUT_K64 + _gw._CUT_A64)
    while wp.bvh_query_next(q, idx, cut):
        tri = o.T[idx]
        r = _gw._watertight(a, u, o.V[tri[0]], o.V[tri[1]], o.V[tri[2]])
        t = r[0]
        if t > tmin and t <= tmax:
            if best_f < 0 or t < best_t or (t == best_t and idx < best_f):
                best_t = t
                best_f = idx
                cut = wp.float32(best_t * _gw._CUT_K64 + _gw._CUT_A64)
    return wp.vec2d(best_t, wp.float64(best_f))


@wp.func
def ccos(z: wp.vec2d) -> wp.vec2d:
    return wp.vec2d(wp.cos(z[0]) * wp.cosh(z[1]), -wp.sin(z[0]) * wp.sinh(z[1]))


@wp.func
def csin(z: wp.vec2d) -> wp.vec2d:
    return wp.vec2d(wp.sin(z[0]) * wp.cosh(z[1]), wp.cos(z[0]) * wp.sinh(z[1]))


@wp.func
def layer_T(cos1: wp.float64, rho: wp.float64, cr: wp.float64, d: wp.float64,
            k1: wp.float64) -> wp.vec2d:
    """``scene_mesh.layer_transmission`` for one crossing."""
    c1 = wp.clamp(wp.abs(cos1), wp.float64(1e-6), wp.float64(1.0))
    s1 = wp.sqrt(wp.float64(1.0) - c1 * c1)
    s2 = cr * s1
    c2 = csqrt(wp.vec2d(wp.float64(1.0) - s2 * s2, wp.float64(0.0)))
    if c2[1] < wp.float64(0.0):
        c2 = -c2
    k2z = wp.vec2d(k1 / cr * c2[0], k1 / cr * c2[1])
    den = c2
    if not (cabs(c2) > wp.float64(1e-12)):
        den = wp.vec2d(wp.float64(1e-12), wp.float64(0.0))
    mm = cdiv(wp.vec2d(rho * cr * c1, wp.float64(0.0)), den)
    inv_m = cdiv(wp.vec2d(wp.float64(1.0), wp.float64(0.0)), mm)
    arg = wp.vec2d(k2z[0] * d, k2z[1] * d)
    sm = mm + inv_m
    # -0.5 i (m + 1/m) sin(k2z d)
    t2 = cmul(wp.vec2d(wp.float64(0.0), wp.float64(-0.5)), cmul(sm, csin(arg)))
    T = cdiv(wp.vec2d(wp.float64(1.0), wp.float64(0.0)), ccos(arg) + t2)
    T = cmul(T, cexpi(-k1 * c1 * d))
    return cconj(T)


@wp.func
def seg_transmit(o: Oracle, Ly: Layers, a: wp.vec3d, b: wp.vec3d, eps: wp.float64) -> wp.vec2d:
    """``SceneOracle.transmit``: 0 across the sea surface or an opaque
    triangle, else the product of the thin layers' factors."""
    if (wp.max(a[2], b[2]) > o.zs + eps) and (wp.min(a[2], b[2]) < o.zs - eps):
        return wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    T = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
    p0 = a
    for it in range(12):
        d = b - p0
        L = wp.length(d)
        u = d / wp.max(L, wp.float64(1e-300))
        h = seg_closest(o, p0, u, eps, wp.max(L - eps, wp.float64(0.0)))
        tri = int(h[1])
        if tri < 0:
            break
        if Ly.on == 0 or int(Ly.mask[tri]) == 0:
            T = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
            break
        cos1 = wp.abs(wp.dot(Ly.normal[tri], u))
        T = cmul(T, layer_T(cos1, Ly.rho[tri], Ly.cr[tri], Ly.d[tri], Ly.k1))
        p0 = p0 + (h[0] + wp.float64(10.0) * eps) * u
    return T


# ================================================================== receiver
@wp.struct
class Rx:
    """``receiver.BinGrid`` + ``Envelope`` on the device."""
    n: int
    start_m: wp.float64
    bin_m: wp.float64
    dev_c: wp.float64
    omega: wp.float64
    kk: int                                   # taps: centre +- kk
    env_t0: wp.float64
    env_dt: wp.float64
    env_s: wp.array(dtype=wp.float64)
    ncopy: int
    field: wp.array2d(dtype=wp.vec2d)        # (ncopy, n) privatised
    mean: wp.array2d(dtype=wp.float64)
    coh: wp.array2d(dtype=wp.vec2d)          # the coherent contributors' field


@wp.func
def env_at(x: Rx, t: wp.float64) -> wp.float64:
    """``Envelope.__call__`` (np.interp, 0 outside)."""
    n = x.env_s.shape[0]
    f = (t - x.env_t0) / x.env_dt
    if f < wp.float64(0.0) or f > wp.float64(n - 1):
        return wp.float64(0.0)
    i = wp.min(int(f), n - 2)
    w = f - wp.float64(i)
    return x.env_s[i] + w * (x.env_s[i + 1] - x.env_s[i])


@wp.func
def tap_centre(x: Rx, tau: wp.float64) -> int:
    return int(wp.floor((wp.float64(0.5) * x.dev_c * tau - x.start_m) / x.bin_m))


@wp.func
def t_bin(x: Rx, b: int) -> wp.float64:
    return wp.float64(2.0) * (x.start_m + (wp.float64(b) + wp.float64(0.5)) * x.bin_m) / x.dev_c


@wp.func
def carrier(x: Rx, tau: wp.float64) -> wp.vec2d:
    """exp(-i fmod(omega tau, 2 pi)): the reduction in float64, the
    cos / sin of the reduced phase in float32 (4e-7 rad)."""
    ph = wp.float32(wp.mod(x.omega * tau, _TWO_PI))
    return wp.vec2d(wp.float64(wp.cos(ph)), -wp.float64(wp.sin(ph)))


# =============================================================== boundaries
_MAXB = 13                                   # P + up to 12 bounces
_vec13 = wp.types.vector(length=_MAXB, dtype=wp.float64)
_mat13 = wp.types.matrix(shape=(_MAXB, 3), dtype=wp.float64)
_BK_S = wp.constant(0)                       # surface
_BK_F = wp.constant(1)                       # floor (the sheet)
_BK_W = wp.constant(2)                       # wall face
_BK_O = wp.constant(3)                       # object face


@wp.struct
class SpecSheet:
    """``specular.SheetSpecular`` grids: heights, smoothed normals, the
    per-triangle material / dry flags and the per-material interface."""
    z: wp.array2d(dtype=wp.float64)
    vn: wp.array2d(dtype=wp.vec3d)
    nx: int
    ny: int
    ox: wp.float64
    oy: wp.float64
    res: wp.float64
    zmin: wp.float64
    nq: int
    edge: int
    tri_mat: wp.array(dtype=wp.int32)
    tri_dry: wp.array(dtype=wp.int32)
    out_mat: int
    lam: wp.float64
    dx: wp.float64
    # MaterialSurface per material: 0 sediment (J&R rms rule), 1 half-space + const rms
    s_mode: wp.array(dtype=wp.int32)
    s_arho: wp.array(dtype=wp.float64)
    s_ap: wp.array(dtype=wp.vec2d)
    s_w2: wp.array(dtype=wp.float64)
    s_g2: wp.array(dtype=wp.float64)
    s_rms: wp.array(dtype=wp.float64)


@wp.func
def spec_locate(s: SpecSheet, x: wp.float64, y: wp.float64) -> wp.vec4d:
    """``SheetSpecular._locate`` -> (z, tri, and the smoothed normal packed
    as nx, ny; nz from |n| = 1 is recomputed by ``spec_normal``)."""
    fx = (x - s.ox) / s.res
    fy = (y - s.oy) / s.res
    inside = (fx >= wp.float64(0.0)) and (fy >= wp.float64(0.0)) and \
        (fx <= wp.float64(s.nx - 1)) and (fy <= wp.float64(s.ny - 1))
    cx = wp.clamp(fx, wp.float64(0.0), wp.float64(s.nx - 1) - wp.float64(1e-9))
    cy = wp.clamp(fy, wp.float64(0.0), wp.float64(s.ny - 1) - wp.float64(1e-9))
    i = int(cx)
    j = int(cy)
    u = cx - wp.float64(i)
    v = cy - wp.float64(j)
    za = s.z[j, i]
    zb = s.z[j, i + 1]
    zc = s.z[j + 1, i + 1]
    zd = s.z[j + 1, i]
    z = wp.float64(0.0)
    tri = int(0)
    q = j * (s.nx - 1) + i
    if u >= v:
        z = za + u * (zb - za) + v * (zc - zb)
        tri = q
    else:
        z = za + u * (zc - zd) + v * (zd - za)
        tri = s.nq + q
    if not inside:
        tri = -1
        if s.edge == 0:
            z = s.zmin
    return wp.vec4d(z, wp.float64(tri), u, v)


@wp.func
def spec_normal(s: SpecSheet, x: wp.float64, y: wp.float64) -> wp.vec3d:
    """The smoothed normal of ``SheetSpecular._locate`` (bilinear in the
    clipped cell, (0, 0, 1) beyond the grid unless outside == edge)."""
    fx = (x - s.ox) / s.res
    fy = (y - s.oy) / s.res
    inside = (fx >= wp.float64(0.0)) and (fy >= wp.float64(0.0)) and \
        (fx <= wp.float64(s.nx - 1)) and (fy <= wp.float64(s.ny - 1))
    if (not inside) and s.edge == 0:
        return wp.vec3d(wp.float64(0.0), wp.float64(0.0), wp.float64(1.0))
    cx = wp.clamp(fx, wp.float64(0.0), wp.float64(s.nx - 1) - wp.float64(1e-9))
    cy = wp.clamp(fy, wp.float64(0.0), wp.float64(s.ny - 1) - wp.float64(1e-9))
    i = int(cx)
    j = int(cy)
    u = cx - wp.float64(i)
    v = cy - wp.float64(j)
    one = wp.float64(1.0)
    n = ((one - u) * (one - v) * s.vn[j, i] + u * (one - v) * s.vn[j, i + 1]
         + u * v * s.vn[j + 1, i + 1] + (one - u) * v * s.vn[j + 1, i])
    return n / wp.length(n)


@wp.func
def floor_info(s: SpecSheet, tri: int) -> wp.vec2i:
    """(material, dry) at a sheet triangle (``FloorBoundary.info``)."""
    if tri < 0:
        return wp.vec2i(s.out_mat, 0)
    return wp.vec2i(int(s.tri_mat[tri]), int(s.tri_dry[tri]))


@wp.func
def floor_factor(s: SpecSheet, k: wp.float64, mat: int, sin_psi: wp.float64,
                 R_e: wp.float64) -> wp.vec2d:
    """``FloorBoundary.factor``: V(psi) exp(-1/2 (2 k sigma sin psi)^2), in
    float32."""
    sp = wp.float32(sin_psi)
    ap = s.s_ap[mat]
    V = reflection(wp.float32(s.s_arho[mat]), wp.vec2f(wp.float32(ap[0]), wp.float32(ap[1])), sp)
    sig = wp.float32(s.s_rms[mat])
    if s.s_mode[mat] == 0:
        r_f = wp.sqrt(0.5 * wp.float32(s.lam) * wp.max(wp.float32(R_e), 1.0e-6))
        kl = wp.max(3.14159265 / r_f, 3.14159265 / wp.float32(s.dx))
        g2 = wp.float32(s.s_g2[mat])
        sig = wp.sqrt(6.2831853 * wp.float32(s.s_w2[mat]) * wp.pow(kl, 2.0 - g2) / (g2 - 2.0))
    g = 2.0 * wp.float32(k) * sig * sp
    e = wp.exp(-0.5 * wp.min(g * g, 1400.0))
    return wp.vec2d(wp.float64(V[0] * e), wp.float64(V[1] * e))


@wp.struct
class Faces:
    """``chains.FaceBoundary`` tables (walls' faces, then smooth objects')."""
    c: wp.array(dtype=wp.vec3d)
    n: wp.array(dtype=wp.vec3d)
    u: wp.array(dtype=wp.vec3d)
    v: wp.array(dtype=wp.vec3d)
    hu: wp.array(dtype=wp.float64)
    hv: wp.array(dtype=wp.float64)
    npoly: wp.array(dtype=wp.int32)
    poly_off: wp.array(dtype=wp.int32)
    poly: wp.array(dtype=wp.vec2d)
    mode: wp.array(dtype=wp.int32)          # 0 interface (a_rho, ap), 1 constant R
    arho: wp.array(dtype=wp.float64)
    ap: wp.array(dtype=wp.vec2d)
    rconst: wp.array(dtype=wp.float64)
    rms: wp.array(dtype=wp.float64)
    wall: wp.array(dtype=wp.int32)          # wall index (own-face rule), -1 for objects


@wp.func
def face_valid(F: Faces, f: int, Q: wp.vec3d) -> int:
    d = Q - F.c[f]
    du = wp.dot(d, F.u[f])
    dv = wp.dot(d, F.v[f])
    np_ = int(F.npoly[f])
    if np_ > 0:
        o = int(F.poly_off[f])
        for i in range(np_):
            a = F.poly[o + i]
            b = F.poly[o + (i + 1) % np_]
            if (b[0] - a[0]) * (dv - a[1]) - (b[1] - a[1]) * (du - a[0]) < wp.float64(-1e-12):
                return 0
        return 1
    if wp.abs(du) <= F.hu[f] and wp.abs(dv) <= F.hv[f]:
        return 1
    return 0


@wp.func
def eckart_amp(k: wp.float64, rms: wp.float64, sin_psi: wp.float64) -> wp.float64:
    """sqrt(waveguide.eckart_c(arcsin(sin_psi))), in float32."""
    if rms <= wp.float64(0.0):
        return wp.float64(1.0)
    g = 2.0 * wp.float32(k) * wp.float32(rms) * wp.sin(wp.asin(wp.clamp(wp.float32(sin_psi), 0.0, 1.0)))
    return wp.float64(wp.sqrt(wp.exp(-wp.min(g * g, 700.0))))


@wp.func
def face_factor(F: Faces, f: int, k: wp.float64, sin_psi: wp.float64) -> wp.vec2d:
    e = eckart_amp(k, F.rms[f], sin_psi)
    if F.mode[f] == 1:
        return wp.vec2d(F.rconst[f] * e, wp.float64(0.0))
    ap = F.ap[f]
    V = reflection(wp.float32(F.arho[f]), wp.vec2f(wp.float32(ap[0]), wp.float32(ap[1])),
                   wp.float32(sin_psi))
    return wp.vec2d(wp.float64(V[0]) * e, wp.float64(V[1]) * e)


@wp.struct
class Chain:
    """One chain to one contributor (registers)."""
    length: wp.float64
    depart: wp.vec3d
    arrive: wp.vec3d
    factor: wp.vec2d
    ok: int
    m: int
    Q: _mat13                               # bounce points
    N: _mat13                               # bounce-plane normals


@wp.func
def _row(M: _mat13, j: int) -> wp.vec3d:
    return wp.vec3d(M[j, 0], M[j, 1], M[j, 2])


@wp.func
def _set_row(M: _mat13, j: int, v: wp.vec3d) -> _mat13:
    """Row j of M set to v (Warp passes value types by value: the caller
    assigns the result)."""
    M[j, 0] = v[0]
    M[j, 1] = v[1]
    M[j, 2] = v[2]
    return M


@wp.func
def unit3(v: wp.vec3d) -> wp.vec3d:
    return v / wp.max(wp.length(v), wp.float64(1e-300))


@wp.func
def reflect3(p: wp.vec3d, q: wp.vec3d, n: wp.vec3d) -> wp.vec3d:
    return p - wp.float64(2.0) * wp.dot(p - q, n) * n


@wp.func
def chain_finish(P: wp.vec3d, X: wp.vec3d, c: Chain, NP: _mat13, kinds: wp.array(dtype=wp.int32),
                 faces: wp.array(dtype=wp.int32), off: int, m: int, S: SpecSheet, F: Faces,
                 k: wp.float64, rms_s: wp.float64) -> Chain:
    """The tail of ``chains.build`` once the bounce points ``c.Q`` and the
    planes' normals ``NP`` are known: validity (each bounce on its face,
    both legs on the water side), leg lengths, boundary factors, depart /
    arrive."""
    for j in range(m):
        c.N = _set_row(c.N, j, _row(NP, j))
    cum = _vec13()
    total = wp.float64(0.0)
    prev = P
    for j in range(m + 1):
        nxt2 = X
        if j < m:
            nxt2 = _row(c.Q, j)
        total = total + wp.length(prev - nxt2)
        cum[j] = total
        prev = nxt2
    for j in range(m):
        q = _row(c.Q, j)
        nrm = _row(NP, j)
        pa = P
        if j > 0:
            pa = _row(c.Q, j - 1)
        pb = X
        if j < m - 1:
            pb = _row(c.Q, j + 1)
        kd = int(kinds[off + j])
        v = int(1)
        if kd == _BK_F:
            zt = spec_locate(S, q[0], q[1])
            v = 1 - floor_info(S, int(zt[1]))[1]
        elif kd == _BK_W or kd == _BK_O:
            v = face_valid(F, int(faces[off + j]), q)
        if v == 0 or not (wp.dot(pa - q, nrm) > wp.float64(0.0)) or not (wp.dot(pb - q, nrm) > wp.float64(0.0)):
            c.ok = 0
    fac = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
    for j in range(m):
        q = _row(c.Q, j)
        nrm = _row(NP, j)
        pa = P
        if j > 0:
            pa = _row(c.Q, j - 1)
        inc = unit3(q - pa)
        sin_psi = wp.clamp(-wp.dot(inc, nrm), wp.float64(0.0), wp.float64(1.0))
        d1 = cum[j]
        d2 = total - d1
        R_e = wp.float64(2.0) * d1 * d2 / wp.max(d1 + d2, wp.float64(1e-300))
        kd = int(kinds[off + j])
        bf = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
        if kd == _BK_S:
            bf = wp.vec2d(-eckart_amp(k, rms_s, sin_psi), wp.float64(0.0))
        elif kd == _BK_F:
            zt = spec_locate(S, q[0], q[1])
            bf = floor_factor(S, k, floor_info(S, int(zt[1]))[0], sin_psi, R_e)
        else:
            bf = face_factor(F, int(faces[off + j]), k, sin_psi)
        if c.ok == 0:
            bf = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
        fac = cmul(fac, bf)
    c.factor = fac
    c.length = total
    if m > 0:
        c.depart = unit3(_row(c.Q, 0) - P)
        c.arrive = unit3(X - _row(c.Q, m - 1))
    else:
        c.depart = unit3(X - P)
        c.arrive = c.depart
    return c


@wp.struct
class Planes:
    QP: _mat13
    NP: _mat13


@wp.func
def seq_planes(P: wp.vec3d, X: wp.vec3d, kinds: wp.array(dtype=wp.int32),
               faces: wp.array(dtype=wp.int32), off: int, m: int, S: SpecSheet,
               F: Faces) -> Planes:
    """``b.plane(mid)`` for every boundary: the surface z = 0, a face's own
    plane, the floor's plane under the P-X midpoint (smoothed normal)."""
    pl = Planes()
    mx = wp.float64(0.5) * (X[0] + P[0])
    my = wp.float64(0.5) * (X[1] + P[1])
    for j in range(m):
        kd = int(kinds[off + j])
        if kd == _BK_S:
            pl.QP = _set_row(pl.QP, j, wp.vec3d(mx, my, wp.float64(0.0)))
            pl.NP = _set_row(pl.NP, j, wp.vec3d(wp.float64(0.0), wp.float64(0.0), wp.float64(-1.0)))
        elif kd == _BK_F:
            zt = spec_locate(S, mx, my)
            pl.QP = _set_row(pl.QP, j, wp.vec3d(mx, my, zt[0]))
            pl.NP = _set_row(pl.NP, j, spec_normal(S, mx, my))
        else:
            f = int(faces[off + j])
            pl.QP = _set_row(pl.QP, j, F.c[f])
            pl.NP = _set_row(pl.NP, j, F.n[f])
    return pl


@wp.struct
class Unf:
    Q: _mat13
    ok: int


@wp.func
def unfold_into(A: wp.vec3d, B: wp.vec3d, QP: _mat13, NP: _mat13, j0: int, j1: int,
                Q: _mat13) -> Unf:
    """``chains._unfold`` of the segment A -> planes j0..j1-1 -> B: ``Q``
    with its rows j0..j1-1 set to the bounce points, and ok."""
    I = _mat13()
    I = _set_row(I, 0, A)
    nn = j1 - j0
    for t in range(nn):
        I = _set_row(I, t + 1, reflect3(_row(I, t), _row(QP, j0 + t), _row(NP, j0 + t)))
    u = Unf()
    u.Q = Q
    u.ok = 1
    nxt = B
    for tt in range(nn):
        t = nn - 1 - tt
        j = j0 + t
        a = _row(I, t + 1)
        d = nxt - a
        nrm = _row(NP, j)
        den = wp.dot(d, nrm)
        if not (wp.abs(den) > wp.float64(1e-300)):
            den = wp.float64(1e-300)
        tp = wp.dot(_row(QP, j) - a, nrm) / den
        if not ((tp > wp.float64(1e-9)) and (tp < wp.float64(1.0) - wp.float64(1e-9))):
            u.ok = 0
        q = a + tp * d
        u.Q = _set_row(u.Q, j, q)
        nxt = q
    return u


@wp.func
def build_planar(P: wp.vec3d, X: wp.vec3d, kinds: wp.array(dtype=wp.int32),
                 faces: wp.array(dtype=wp.int32), off: int, m: int, S: SpecSheet,
                 F: Faces, k: wp.float64, rms_s: wp.float64) -> Chain:
    """``chains.build(..., refine=False)`` for one contributor."""
    c = Chain()
    c.m = m
    pl = seq_planes(P, X, kinds, faces, off, m, S, F)
    u = unfold_into(P, X, pl.QP, pl.NP, 0, m, c.Q)
    c.Q = u.Q
    c.ok = u.ok
    return chain_finish(P, X, c, pl.NP, kinds, faces, off, m, S, F, k, rms_s)


_MAXF = 6                                     # floor bounces per chain (order 12)
_vec12 = wp.types.vector(length=2 * _MAXF, dtype=wp.float64)
_vec6i = wp.types.vector(length=_MAXF + 2, dtype=wp.int32)
_mat12 = wp.types.matrix(shape=(2 * _MAXF, 2 * _MAXF), dtype=wp.float64)


@wp.func
def image_run(A: wp.vec3d, QP: _mat13, NP: _mat13, j0: int, j1: int, rev: int) -> wp.vec3d:
    """``chains._image_through`` over planes j0..j1-1 (reversed if rev)."""
    out = A
    nn = j1 - j0
    for t in range(nn):
        j = j0 + t
        if rev == 1:
            j = j1 - 1 - t
        out = reflect3(out, _row(QP, j), _row(NP, j))
    return out


@wp.struct
class Grad:
    G: _vec12
    L: wp.float64


@wp.func
def newton_grad(P: wp.vec3d, X: wp.vec3d, xy: _vec12, nf: int, cuts: _vec6i, QP: _mat13,
                NP: _mat13, S: SpecSheet) -> Grad:
    """``build``'s grad(): the path-length gradient over the floor bounces'
    (x, y) (smoothed sheet slope) and the length."""
    G = _vec12()
    Lsum = wp.float64(0.0)
    for f in range(nf):
        x = xy[2 * f]
        y = xy[2 * f + 1]
        zt = spec_locate(S, x, y)
        vn = spec_normal(S, x, y)
        Sf = wp.vec3d(x, y, zt[0])
        # fixed points: P (f = 0) or the previous floor; X (last) or the next floor
        prevp = P
        if f > 0:
            zp = spec_locate(S, xy[2 * f - 2], xy[2 * f - 1])
            prevp = wp.vec3d(xy[2 * f - 2], xy[2 * f - 1], zp[0])
        nextp = X
        if f < nf - 1:
            zn = spec_locate(S, xy[2 * f + 2], xy[2 * f + 3])
            nextp = wp.vec3d(xy[2 * f + 2], xy[2 * f + 3], zn[0])
        Aj = image_run(prevp, QP, NP, int(cuts[f]) + 1, int(cuts[f + 1]), 0)
        Bj = image_run(nextp, QP, NP, int(cuts[f + 1]) + 1, int(cuts[f + 2]), 1)
        g = unit3(Sf - Aj) + unit3(Sf - Bj)
        zx = -vn[0] / vn[2]
        zy = -vn[1] / vn[2]
        G[2 * f] = g[0] + g[2] * zx
        G[2 * f + 1] = g[1] + g[2] * zy
    for r in range(nf + 1):
        a = P
        if r > 0:
            za = spec_locate(S, xy[2 * r - 2], xy[2 * r - 1])
            a = wp.vec3d(xy[2 * r - 2], xy[2 * r - 1], za[0])
        bb = X
        if r < nf:
            zb = spec_locate(S, xy[2 * r], xy[2 * r + 1])
            bb = wp.vec3d(xy[2 * r], xy[2 * r + 1], zb[0])
        Lsum = Lsum + wp.length(bb - image_run(a, QP, NP, int(cuts[r]) + 1, int(cuts[r + 1]), 0))
    res = Grad()
    res.G = G
    res.L = Lsum
    return res


@wp.struct
class Sol:
    x: _vec12
    det: wp.float64


@wp.func
def solve_lu(A: _mat12, b: _vec12, n: int) -> Sol:
    """Gaussian elimination with partial pivoting on the leading n x n block
    (LAPACK's getrf/getrs method): x with A x = b, and det(A)."""
    x = _vec12()
    det = wp.float64(1.0)
    for p in range(n):
        piv = p
        mx = wp.abs(A[p, p])
        for r in range(p + 1, n):
            if wp.abs(A[r, p]) > mx:
                mx = wp.abs(A[r, p])
                piv = r
        if piv != p:
            for c in range(n):
                tmp = A[p, c]
                A[p, c] = A[piv, c]
                A[piv, c] = tmp
            tb = b[p]
            b[p] = b[piv]
            b[piv] = tb
            det = -det
        d = A[p, p]
        det = det * d
        if d != wp.float64(0.0):
            for r in range(p + 1, n):
                f = A[r, p] / d
                for c in range(p, n):
                    A[r, c] = A[r, c] - f * A[p, c]
                b[r] = b[r] - f * b[p]
    for qq in range(n):
        r = n - 1 - qq
        acc = b[r]
        for c in range(r + 1, n):
            acc = acc - A[r, c] * x[c]
        x[r] = acc / A[r, r]
    sol = Sol()
    sol.x = x
    sol.det = det
    return sol


@wp.func
def build_refined(P: wp.vec3d, X: wp.vec3d, kinds: wp.array(dtype=wp.int32),
                  faces: wp.array(dtype=wp.int32), off: int, m: int, S: SpecSheet,
                  F: Faces, k: wp.float64, rms_s: wp.float64, lam: wp.float64,
                  max_iter: int) -> Chain:
    """``chains.build`` with floor bounces: the flat-floor image solution,
    then the damped Newton solve of the floor bounces' (x, y) (Fermat), the
    planar runs between the fixed points unfolded exactly."""
    c = Chain()
    c.m = m
    pl = seq_planes(P, X, kinds, faces, off, m, S, F)
    QP = pl.QP
    NP = pl.NP
    u0 = unfold_into(P, X, QP, NP, 0, m, c.Q)
    c.Q = u0.Q
    good = u0.ok
    cuts = _vec6i()
    cuts[0] = -1
    nf = int(0)
    for j in range(m):
        if int(kinds[off + j]) == _BK_F and nf < _MAXF:
            nf = nf + 1
            cuts[nf] = j
    cuts[nf + 1] = m
    xy = _vec12()
    for f in range(nf):
        q = _row(c.Q, int(cuts[f + 1]))
        xy[2 * f] = q[0]
        xy[2 * f + 1] = q[1]
    gr = newton_grad(P, X, xy, nf, cuts, QP, NP, S)
    G = gr.G
    h = wp.float64(2e-3)
    conv = int(0)
    n2 = 2 * nf
    for it in range(max_iter):
        gmax = wp.float64(0.0)
        for qi in range(n2):
            gmax = wp.max(gmax, wp.abs(G[qi]))
        if good == 1 and gmax < wp.float64(1e-12):
            conv = 1
        if conv == 1 or good == 0:
            break
        H = _mat12()
        for col in range(n2):
            xp = xy
            xm = xy
            xp[col] = xy[col] + h
            xm[col] = xy[col] - h
            Gp = newton_grad(P, X, xp, nf, cuts, QP, NP, S).G
            Gm = newton_grad(P, X, xm, nf, cuts, QP, NP, S).G
            for r in range(n2):
                H[r, col] = (Gp[r] - Gm[r]) / (wp.float64(2.0) * h)
        sol = solve_lu(H, G, n2)
        step = sol.x
        if not (wp.abs(sol.det) > wp.float64(1e-12)):
            for qi in range(n2):
                step[qi] = G[qi]                 # identity Hessian
        sn = wp.float64(0.0)
        for qi in range(n2):
            step[qi] = -step[qi]
            sn = sn + step[qi] * step[qi]
        sn = wp.sqrt(sn)
        sc = wp.min(wp.float64(1.0), wp.float64(0.5) / wp.max(sn, wp.float64(1e-300)))
        dL = wp.float64(0.0)
        sn2 = wp.float64(0.0)
        for qi in range(n2):
            step[qi] = step[qi] * sc
            dL = dL + G[qi] * step[qi]
            sn2 = sn2 + step[qi] * step[qi]
            xy[qi] = xy[qi] + step[qi]
        G = newton_grad(P, X, xy, nf, cuts, QP, NP, S).G
        if wp.abs(dL) < lam / wp.float64(200.0) and wp.sqrt(sn2) < wp.float64(1e-3):
            conv = 1
    if conv == 0:
        good = 0
    # floors: the converged points and the smoothed normal there
    for f in range(nf):
        j = int(cuts[f + 1])
        zt = spec_locate(S, xy[2 * f], xy[2 * f + 1])
        QP = _set_row(QP, j, wp.vec3d(xy[2 * f], xy[2 * f + 1], zt[0]))
        NP = _set_row(NP, j, spec_normal(S, xy[2 * f], xy[2 * f + 1]))
        c.Q = _set_row(c.Q, j, _row(QP, j))
    # planar runs between the fixed points
    for r in range(nf + 1):
        j0 = int(cuts[r]) + 1
        j1 = int(cuts[r + 1])
        if j1 > j0:
            a = P
            if r > 0:
                a = _row(c.Q, int(cuts[r]))
            bb = X
            if r < nf:
                bb = _row(c.Q, int(cuts[r + 1]))
            ur = unfold_into(a, bb, QP, NP, j0, j1, c.Q)
            c.Q = ur.Q
            if ur.ok == 0:
                good = 0
    c.ok = good
    return chain_finish(P, X, c, NP, kinds, faces, off, m, S, F, k, rms_s)


@wp.func
def build_chain(P: wp.vec3d, X: wp.vec3d, kinds: wp.array(dtype=wp.int32),
                faces: wp.array(dtype=wp.int32), off: int, m: int, S: SpecSheet,
                F: Faces, k: wp.float64, rms_s: wp.float64, lam: wp.float64,
                max_iter: int, refine: int) -> Chain:
    """``chains.build``: refined when asked and the sequence has a floor."""
    has_floor = int(0)
    for j in range(m):
        if int(kinds[off + j]) == _BK_F:
            has_floor = 1
    if refine == 1 and has_floor == 1:
        return build_refined(P, X, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter)
    return build_planar(P, X, kinds, faces, off, m, S, F, k, rms_s)


# ====================================================== short chains (m <= 2)
@wp.struct
class ChainS:
    """A chain of at most two bounces (scalars: no runtime-indexed arrays)."""
    length: wp.float64
    depart: wp.vec3d
    arrive: wp.vec3d
    factor: wp.vec2d
    ok: int
    m: int
    q0: wp.vec3d
    q1: wp.vec3d
    n0: wp.vec3d
    n1: wp.vec3d


@wp.struct
class Pl:
    q: wp.vec3d
    n: wp.vec3d


@wp.func
def plane_of(kd: int, f: int, mx: wp.float64, my: wp.float64, S: SpecSheet, F: Faces) -> Pl:
    """``b.plane(mid)`` for one boundary."""
    pl = Pl()
    if kd == _BK_S:
        pl.q = wp.vec3d(mx, my, wp.float64(0.0))
        pl.n = wp.vec3d(wp.float64(0.0), wp.float64(0.0), wp.float64(-1.0))
    elif kd == _BK_F:
        pl.q = wp.vec3d(mx, my, spec_locate(S, mx, my)[0])
        pl.n = spec_normal(S, mx, my)
    else:
        pl.q = F.c[f]
        pl.n = F.n[f]
    return pl


@wp.func
def cross_plane(a: wp.vec3d, nxt: wp.vec3d, q: wp.vec3d, n: wp.vec3d) -> wp.vec4d:
    """The crossing of a -> nxt with the plane (q, n): (point, t)."""
    d = nxt - a
    den = wp.dot(d, n)
    if not (wp.abs(den) > wp.float64(1e-300)):
        den = wp.float64(1e-300)
    t = wp.dot(q - a, n) / den
    p = a + t * d
    return wp.vec4d(p[0], p[1], p[2], t)


@wp.func
def t_ok(t: wp.float64) -> int:
    if (t > wp.float64(1e-9)) and (t < wp.float64(1.0) - wp.float64(1e-9)):
        return 1
    return 0


@wp.func
def floor_point(S: SpecSheet, x: wp.float64, y: wp.float64) -> wp.vec3d:
    return wp.vec3d(x, y, spec_locate(S, x, y)[0])


@wp.func
def grad1(P: wp.vec3d, X: wp.vec3d, xy: wp.vec2d, before: int, qa: wp.vec3d, na: wp.vec3d,
          after: int, qb: wp.vec3d, nb: wp.vec3d, S: SpecSheet) -> wp.vec2d:
    """``build``'s grad() for one floor bounce: the planar boundary (at most
    one) before it images P, the one after it images X."""
    Sf = floor_point(S, xy[0], xy[1])
    vn = spec_normal(S, xy[0], xy[1])
    A = P
    if before == 1:
        A = reflect3(P, qa, na)
    Bm = X
    if after == 1:
        Bm = reflect3(X, qb, nb)
    g = unit3(Sf - A) + unit3(Sf - Bm)
    zx = -vn[0] / vn[2]
    zy = -vn[1] / vn[2]
    return wp.vec2d(g[0] + g[2] * zx, g[1] + g[2] * zy)


@wp.func
def build_small(P: wp.vec3d, X: wp.vec3d, kinds: wp.array(dtype=wp.int32),
                faces: wp.array(dtype=wp.int32), off: int, m: int, S: SpecSheet, F: Faces,
                k: wp.float64, rms_s: wp.float64, lam: wp.float64, max_iter: int,
                refine: int) -> ChainS:
    """``chains.build`` for m <= 2 with at most one floor bounce, in scalars:
    the planar image solution, and for a floor bounce (refine) the damped
    Newton solve of its (x, y) with the same gradient, finite-difference
    Hessian, step rule and convergence test as the general solver."""
    c = ChainS()
    c.m = m
    c.ok = 1
    mx = wp.float64(0.5) * (X[0] + P[0])
    my = wp.float64(0.5) * (X[1] + P[1])
    k0 = int(kinds[off])
    f0 = int(faces[off])
    p0 = plane_of(k0, f0, mx, my, S, F)
    k1 = int(-1)
    f1 = int(-1)
    p1 = Pl()
    if m == 2:
        k1 = int(kinds[off + 1])
        f1 = int(faces[off + 1])
        p1 = plane_of(k1, f1, mx, my, S, F)
    # planar unfolding
    I1 = reflect3(P, p0.q, p0.n)
    if m == 1:
        r = cross_plane(I1, X, p0.q, p0.n)
        c.q0 = wp.vec3d(r[0], r[1], r[2])
        c.ok = t_ok(r[3])
    else:
        I2 = reflect3(I1, p1.q, p1.n)
        r1 = cross_plane(I2, X, p1.q, p1.n)
        c.q1 = wp.vec3d(r1[0], r1[1], r1[2])
        r0 = cross_plane(I1, c.q1, p0.q, p0.n)
        c.q0 = wp.vec3d(r0[0], r0[1], r0[2])
        c.ok = t_ok(r1[3]) * t_ok(r0[3])
    c.n0 = p0.n
    c.n1 = p1.n
    jf = int(-1)
    if k0 == _BK_F:
        jf = 0
    elif k1 == _BK_F:
        jf = 1
    if refine == 1 and jf >= 0:
        # the planar neighbour of the floor (m = 2): before it (jf = 1) or after (jf = 0)
        before = int(0)
        after = int(0)
        qa = p1.q
        na = p1.n
        qb = p1.q
        nb = p1.n
        if m == 2 and jf == 1:
            before = 1
            qa = p0.q
            na = p0.n
        if m == 2 and jf == 0:
            after = 1
        q = c.q0
        if jf == 1:
            q = c.q1
        xy = wp.vec2d(q[0], q[1])
        G = grad1(P, X, xy, before, qa, na, after, qb, nb, S)
        h = wp.float64(2e-3)
        conv = int(0)
        for it in range(max_iter):
            if c.ok == 1 and wp.max(wp.abs(G[0]), wp.abs(G[1])) < wp.float64(1e-12):
                conv = 1
            if conv == 1 or c.ok == 0:
                break
            Gxp = grad1(P, X, wp.vec2d(xy[0] + h, xy[1]), before, qa, na, after, qb, nb, S)
            Gxm = grad1(P, X, wp.vec2d(xy[0] - h, xy[1]), before, qa, na, after, qb, nb, S)
            Gyp = grad1(P, X, wp.vec2d(xy[0], xy[1] + h), before, qa, na, after, qb, nb, S)
            Gym = grad1(P, X, wp.vec2d(xy[0], xy[1] - h), before, qa, na, after, qb, nb, S)
            h00 = (Gxp[0] - Gxm[0]) / (wp.float64(2.0) * h)
            h10 = (Gxp[1] - Gxm[1]) / (wp.float64(2.0) * h)
            h01 = (Gyp[0] - Gym[0]) / (wp.float64(2.0) * h)
            h11 = (Gyp[1] - Gym[1]) / (wp.float64(2.0) * h)
            det = h00 * h11 - h01 * h10
            sx = G[0]
            sy = G[1]
            if wp.abs(det) > wp.float64(1e-12):
                sx = (h11 * G[0] - h01 * G[1]) / det
                sy = (h00 * G[1] - h10 * G[0]) / det
            sx = -sx
            sy = -sy
            sn = wp.sqrt(sx * sx + sy * sy)
            sc = wp.min(wp.float64(1.0), wp.float64(0.5) / wp.max(sn, wp.float64(1e-300)))
            sx = sx * sc
            sy = sy * sc
            dL = G[0] * sx + G[1] * sy
            xy = wp.vec2d(xy[0] + sx, xy[1] + sy)
            G = grad1(P, X, xy, before, qa, na, after, qb, nb, S)
            if wp.abs(dL) < lam / wp.float64(200.0) and wp.sqrt(sx * sx + sy * sy) < wp.float64(1e-3):
                conv = 1
        if conv == 0:
            c.ok = 0
        Sf = floor_point(S, xy[0], xy[1])
        vn = spec_normal(S, xy[0], xy[1])
        if jf == 0:
            c.q0 = Sf
            c.n0 = vn
            if m == 2:
                r = cross_plane(reflect3(Sf, p1.q, p1.n), X, p1.q, p1.n)
                c.q1 = wp.vec3d(r[0], r[1], r[2])
                if t_ok(r[3]) == 0:
                    c.ok = 0
        else:
            c.q1 = Sf
            c.n1 = vn
            r = cross_plane(reflect3(P, p0.q, p0.n), Sf, p0.q, p0.n)
            c.q0 = wp.vec3d(r[0], r[1], r[2])
            if t_ok(r[3]) == 0:
                c.ok = 0
    # validity: each bounce on its face, both legs on the water side
    for j in range(m):
        q = c.q0
        nrm = c.n0
        kd = k0
        f = f0
        pa = P
        pb = X
        if m == 2:
            pb = c.q1
        if j == 1:
            q = c.q1
            nrm = c.n1
            kd = k1
            f = f1
            pa = c.q0
            pb = X
        v = int(1)
        if kd == _BK_F:
            v = 1 - floor_info(S, int(spec_locate(S, q[0], q[1])[1]))[1]
        elif kd == _BK_W or kd == _BK_O:
            v = face_valid(F, f, q)
        if v == 0 or not (wp.dot(pa - q, nrm) > wp.float64(0.0)) or not (wp.dot(pb - q, nrm) > wp.float64(0.0)):
            c.ok = 0
    # legs, factors
    l0 = wp.length(c.q0 - P)
    total = l0
    l1 = wp.float64(0.0)
    if m == 2:
        l1 = wp.length(c.q1 - c.q0)
        total = total + l1
        total = total + wp.length(X - c.q1)
    else:
        total = total + wp.length(X - c.q0)
    fac = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
    for j in range(m):
        q = c.q0
        nrm = c.n0
        kd = k0
        f = f0
        pa = P
        d1 = l0
        if j == 1:
            q = c.q1
            nrm = c.n1
            kd = k1
            f = f1
            pa = c.q0
            d1 = l0 + l1
        inc = unit3(q - pa)
        sin_psi = wp.clamp(-wp.dot(inc, nrm), wp.float64(0.0), wp.float64(1.0))
        d2 = total - d1
        R_e = wp.float64(2.0) * d1 * d2 / wp.max(d1 + d2, wp.float64(1e-300))
        bf = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
        if kd == _BK_S:
            bf = wp.vec2d(-eckart_amp(k, rms_s, sin_psi), wp.float64(0.0))
        elif kd == _BK_F:
            bf = floor_factor(S, k, floor_info(S, int(spec_locate(S, q[0], q[1])[1]))[0], sin_psi, R_e)
        else:
            bf = face_factor(F, f, k, sin_psi)
        if c.ok == 0:
            bf = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
        fac = cmul(fac, bf)
    c.factor = fac
    c.length = total
    c.depart = unit3(c.q0 - P)
    if m == 2:
        c.arrive = unit3(X - c.q1)
    else:
        c.arrive = unit3(X - c.q0)
    return c


@wp.func
def blocked_small(o: Oracle, P: wp.vec3d, Xv: wp.vec3d, c: ChainS, eps: wp.float64,
                  offset: wp.float64) -> int:
    """``check_visibility`` legs of a short chain."""
    e0 = c.q0 + offset * c.n0
    if seg_blocked(o, P, e0, eps) == 1 or wp.max(P[2], e0[2]) > wp.float64(1e-9):
        return 1
    prev = e0
    if c.m == 2:
        e1 = c.q1 + offset * c.n1
        if seg_blocked(o, e0, e1, eps) == 1 or wp.max(e0[2], e1[2]) > wp.float64(1e-9):
            return 1
        prev = e1
    if seg_blocked(o, prev, Xv, eps) == 1 or wp.max(prev[2], Xv[2]) > wp.float64(1e-9):
        return 1
    return 0


@wp.func
def transmit_small(o: Oracle, Ly: Layers, P: wp.vec3d, Xv: wp.vec3d, c: ChainS,
                   eps: wp.float64, offset: wp.float64) -> wp.vec2d:
    """``_transmit_blocked`` over a short chain's legs."""
    e0 = c.q0 + offset * c.n0
    T = seg_transmit(o, Ly, P, e0, eps)
    prev = e0
    if c.m == 2:
        e1 = c.q1 + offset * c.n1
        T = cmul(T, seg_transmit(o, Ly, e0, e1, eps))
        prev = e1
    return cmul(T, seg_transmit(o, Ly, prev, Xv, eps))


@wp.func
def n_floors(kinds: wp.array(dtype=wp.int32), off: int, m: int) -> int:
    nf = int(0)
    for j in range(m):
        if int(kinds[off + j]) == _BK_F:
            nf = nf + 1
    return nf


@wp.func
def chain_blocked(o: Oracle, P: wp.vec3d, Xv: wp.vec3d, c: Chain, eps: wp.float64,
                  offset: wp.float64) -> int:
    """``chains.check_visibility`` for a bounce chain: any leg blocked by a
    triangle or rising above z = 0."""
    prev = P
    for j in range(c.m + 1):
        end = Xv
        if j < c.m:
            end = _row(c.Q, j) + offset * _row(c.N, j)
        if seg_blocked(o, prev, end, eps) == 1 or wp.max(prev[2], end[2]) > wp.float64(1e-9):
            return 1
        prev = end
    return 0


@wp.func
def own_bad(kinds: wp.array(dtype=wp.int32), faces: wp.array(dtype=wp.int32), off: int,
            m: int, onfl: int, wallof: int, F: Faces) -> int:
    """``Multipath._own_mask``: the sequence's last bounce is the
    contributor's own surface (a seabed scatterer's floor, a wall
    scatterer's own wall)."""
    if m == 0:
        return 0
    kd = int(kinds[off + m - 1])
    if kd == _BK_F and onfl == 1:
        return 1
    if kd == _BK_W and wallof >= 0 and int(F.wall[int(faces[off + m - 1])]) == wallof:
        return 1
    return 0


@wp.struct
class ChainOut:
    """A kept chain evaluated for one contributor: valid, unfolded length,
    arrival direction, pattern and the factor with its leg transmission."""
    ok: int
    length: wp.float64
    arrive: wp.vec3d
    b: wp.vec2d
    factor: wp.vec2d


@wp.func
def eval_chain(P: wp.vec3d, X: wp.vec3d, Xv: wp.vec3d, kinds: wp.array(dtype=wp.int32),
               faces: wp.array(dtype=wp.int32), off: int, m: int, S: SpecSheet, F: Faces,
               k: wp.float64, rms_s: wp.float64, lam: wp.float64, max_iter: int, B: Beam,
               e_a: wp.vec3d, e_n: wp.vec3d, e_v: wp.vec3d, o: Oracle, Ly: Layers,
               eps: wp.float64, offset: wp.float64) -> ChainOut:
    """The refined chain (the short scalar path for m <= 2 with at most one
    floor, the general one otherwise), its pattern, visibility of every leg
    and, where a leg is blocked only by thin shells, its transmission
    (``chains.check_visibility`` / ``_transmit_blocked``). ``Xv`` is the
    contributor lifted by the surface offset; bounce points are lifted by
    ``chains.BOUNCE_OFFSET_M``, as the reference does."""
    out = ChainOut()
    out.ok = 0
    fac = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    T = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
    if m <= 2 and n_floors(kinds, off, m) <= 1:
        cs = build_small(P, X, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter, 1)
        if not (cabs(cs.factor) > wp.float64(0.0)) or cs.ok == 0:
            return out
        b = pattern(B, cs.depart * cs.length, e_a, e_n, e_v)
        if not (cabs(b) > wp.float64(0.0)):
            return out
        if blocked_small(o, P, Xv, cs, eps, _BOFF) == 1:
            if Ly.on == 0:
                return out
            T = transmit_small(o, Ly, P, Xv, cs, eps, _BOFF)
            if not (cabs(T) > wp.float64(0.0)):
                return out
        out.length = cs.length
        out.arrive = cs.arrive
        out.b = b
        fac = cs.factor
    else:
        ch = build_chain(P, X, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter, 1)
        if not (cabs(ch.factor) > wp.float64(0.0)) or ch.ok == 0:
            return out
        b = pattern(B, ch.depart * ch.length, e_a, e_n, e_v)
        if not (cabs(b) > wp.float64(0.0)):
            return out
        if chain_blocked(o, P, Xv, ch, eps, _BOFF) == 1:
            if Ly.on == 0:
                return out
            prev = P
            for j in range(ch.m + 1):
                end = Xv
                if j < ch.m:
                    end = _row(ch.Q, j) + _BOFF * _row(ch.N, j)
                T = cmul(T, seg_transmit(o, Ly, prev, end, eps))
                prev = end
            if not (cabs(T) > wp.float64(0.0)):
                return out
        out.length = ch.length
        out.arrive = ch.arrive
        out.b = b
        fac = ch.factor
    out.factor = cmul(fac, T)
    out.ok = 1
    return out


# =================================================================== kernels
@wp.kernel
def _direct_k(pos: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3d),
              area: wp.array(dtype=wp.float64), mat: wp.array(dtype=wp.int32),
              P: wp.vec3d, e_a: wp.vec3d, e_n: wp.vec3d, e_v: wp.vec3d, B: Beam, L: Law,
              o: Oracle, r_max: wp.float64, floor_pw: wp.float64, offset: wp.float64,
              eps: wp.float64, Ly: Layers, R_o: wp.array(dtype=wp.float64),
              b_o: wp.array(dtype=wp.vec2d), amp0_o: wp.array(dtype=wp.float64),
              state_o: wp.array(dtype=wp.int32), T_o: wp.array(dtype=wp.vec2d)):
    """The direct chain of ``Multipath.diffuse``: length, pattern, the
    scattering-amplitude bound amp0, candidacy (|b|^4 >= floor, front-lit)
    and visibility of the candidates (state bit 0 cand, bit 1 visible)."""
    i = wp.tid()
    X = pos[i]
    n = nrm[i]
    d = X - P
    R = wp.length(d)
    R_o[i] = R
    T_o[i] = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
    if not ((R > wp.float64(1e-3)) and (R <= r_max)):
        amp0_o[i] = wp.float64(0.0)
        b_o[i] = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
        state_o[i] = 4
        return
    ci = wp.clamp(wp.dot(n, -d) / R, wp.float64(0.0), wp.float64(1.0))
    sig0 = wp.float64(wp.max(sigma_mono(L, wp.float32(ci), int(mat[i])), wp.float32(0.0)))
    amp0_o[i] = wp.sqrt(sig0 * area[i])
    b = pattern(B, d, e_a, e_n, e_v)
    b_o[i] = b
    st = int(0)
    b2 = cabs2(b)
    if b2 * b2 >= floor_pw and ci > wp.float64(0.0):
        st = 1
        Xv = X + offset * n
        if seg_occluded(o, P, Xv, eps) == 0:
            st = 3
        elif Ly.on == 1:
            # ``chains._transmit_blocked``: a leg blocked only by thin
            # shells transmits (visible again, T kept; bit 3 = transmitted)
            T = seg_transmit(o, Ly, P, Xv, eps)
            T_o[i] = T
            if cabs(T) > wp.float64(0.0):
                st = 3 | 8
    state_o[i] = st


@wp.kernel
def _level_k(R: wp.array(dtype=wp.float64), b: wp.array(dtype=wp.vec2d),
             amp0: wp.array(dtype=wp.float64),
             D: wp.array(dtype=wp.vec2d), alpha_np: wp.float64, c: wp.float64, x: Rx,
             level: wp.array2d(dtype=wp.float64)):
    """The direct reverberation per bin (the selection's reference level):
    (|D| |b|^2 amp0 leg^2)^2 at bin(2R/c)."""
    i = wp.tid()
    dabs = cabs(D[i])                      # 0 unless visible (after diffraction)
    if dabs == wp.float64(0.0):
        return
    r = R[i]
    leg = wp.exp(-alpha_np * r) / wp.max(r, wp.float64(1e-6))
    p = dabs * cabs2(b[i]) * amp0[i] * leg * leg
    bb = wp.clamp(tap_centre(x, wp.float64(2.0) * r / c), 0, x.n - 1)
    wp.atomic_add(level, i % x.ncopy, bb, p * p)


@wp.kernel
def _bound_k(pos: wp.array(dtype=wp.vec3d), onfl: wp.array(dtype=wp.int32),
             wallof: wp.array(dtype=wp.int32), state: wp.array(dtype=wp.int32),
             R_d: wp.array(dtype=wp.float64), best: wp.array(dtype=wp.float64),
             amp0: wp.array(dtype=wp.float64), P: wp.vec3d, e_a: wp.vec3d, e_n: wp.vec3d,
             e_v: wp.vec3d, B: Beam, kinds: wp.array(dtype=wp.int32),
             faces: wp.array(dtype=wp.int32), m: int, S: SpecSheet, F: Faces,
             k: wp.float64, rms_s: wp.float64, alpha_np: wp.float64, c: wp.float64, x: Rx,
             refine: int, lam: wp.float64, max_iter: int,
             one_o: wp.array(dtype=wp.float64), bound: wp.array2d(dtype=wp.float64)):
    """``Multipath._bound`` of one sequence (on the flat-floor image solution,
    or refined for a kept floor sequence): per contributor the one-way
    amplitude ``one`` and its energy bound, summed per bin."""
    i = wp.tid()
    one_o[i] = wp.float64(0.0)
    if (int(state[i]) & 4) != 0:
        return
    if own_bad(kinds, faces, 0, m, int(onfl[i]), int(wallof[i]), F) == 1:
        return
    X = pos[i]
    fac = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    ln = wp.float64(0.0)
    dep = wp.vec3d(wp.float64(0.0), wp.float64(0.0), wp.float64(1.0))
    if m <= 2 and n_floors(kinds, 0, m) <= 1:
        cs = build_small(P, X, kinds, faces, 0, m, S, F, k, rms_s, lam, max_iter, refine)
        fac = cs.factor
        ln = cs.length
        dep = cs.depart
    else:
        ch = build_chain(P, X, kinds, faces, 0, m, S, F, k, rms_s, lam, max_iter, refine)
        fac = ch.factor
        ln = ch.length
        dep = ch.depart
    if not (cabs(fac) > wp.float64(0.0)):
        return
    b = pattern(B, dep * ln, e_a, e_n, e_v)
    one = cabs(cmul(b, fac)) / wp.max(ln, wp.float64(1e-9))
    one_o[i] = one
    absb = wp.exp(-alpha_np * (ln + R_d[i]))
    pw = one * best[i] * absb * amp0[i]
    bb = wp.clamp(tap_centre(x, (ln + R_d[i]) / c), 0, x.n - 1)
    wp.atomic_add(bound, i % x.ncopy, bb, pw * pw)


@wp.kernel
def _best_k(best: wp.array(dtype=wp.float64), one: wp.array(dtype=wp.float64)):
    i = wp.tid()
    best[i] = wp.max(best[i], one[i])


import os as _os
#: ``paths.reselect_every``: a reused selection is dropped once the sensor
#: turned this much since it was made [deg]
_RESELECT_YAW_DEG = 2.0
_SEQ_TIMING = bool(_os.environ.get("BLUEBOAT_GPU_SEQ_TIMING"))   # per-sequence bound timing (debug)





# ===================================================== floor coherent channel
# ``Multipath.floor_coherent`` on the device (2026-11-04): the window-region
# patches of the seabed sheet (``SheetSpecular.patches``), their amplitude
# (``amplitudes_dirs``) through the out and back chains, the energy bound on
# the host (the same rule), then visibility, thin-shell transmission and the
# coherent deposit.
@wp.struct
class SheetWin:
    """``SheetSpecular``'s window and patch rules."""
    tri_n: wp.array(dtype=wp.vec3d)          # tri_normal (flipped up)
    n1: wp.float64
    n2: wp.float64
    eps: wp.float64                           # patch_phase_rad
    a_min: wp.float64
    a_max: wp.float64
    k: wp.float64


@wp.func
def spec_zone(X: wp.vec3d, nrm: wp.vec3d, A: wp.vec3d, B: wp.vec3d, k: wp.float64) -> wp.vec2d:
    """``SheetSpecular.zone``: (Fresnel-zone index N, R_e); N = 1e300 for a
    back-facing bisector (the window is 0 there, as for the CPU's inf)."""
    da = A - X
    db = B - X
    ra = wp.length(da)
    rb = wp.length(db)
    ua = da / wp.max(ra, wp.float64(1e-300))
    ub = db / wp.max(rb, wp.float64(1e-300))
    bis = ua + ub
    bn = wp.length(bis)
    sin_psi = wp.clamp(wp.float64(0.5) * bn, wp.float64(1e-6), wp.float64(1.0))
    bh = bis / wp.max(bn, wp.float64(1e-300))
    tan_ = bh - wp.dot(bh, nrm) * nrm
    dif = ub - ua
    ep = dif - wp.dot(dif, nrm) * nrm
    epn = wp.length(ep)
    if epn > wp.float64(1e-9):
        ep = ep / wp.max(epn, wp.float64(1e-300))
    else:
        ep = wp.vec3d(wp.float64(0.0), wp.float64(0.0), wp.float64(0.0))
    mu_p = wp.dot(tan_, ep)
    mu2 = wp.dot(tan_, tan_)
    mu_t2 = wp.max(mu2 - mu_p * mu_p, wp.float64(0.0))
    Re = wp.float64(2.0) * ra * rb / wp.max(ra + rb, wp.float64(1e-300))
    N = k * Re * (mu_p * mu_p + sin_psi * sin_psi * mu_t2) / wp.float64(3.141592653589793)
    if not (wp.dot(bh, nrm) > wp.float64(0.0)):
        N = wp.float64(1e300)
    return wp.vec2d(N, Re)


@wp.func
def spec_window(N: wp.float64, n1: wp.float64, n2: wp.float64) -> wp.float64:
    """``specular.window``: 1 to n1, raised cosine to 0 at n2."""
    t = wp.clamp((N - n1) / wp.max(n2 - n1, wp.float64(1e-12)), wp.float64(0.0), wp.float64(1.0))
    return wp.float64(0.5) * (wp.float64(1.0) + wp.cos(wp.float64(3.141592653589793) * t))


@wp.func
def spec_rms(S: SpecSheet, mat: int, R_e: wp.float64) -> wp.float64:
    """``MaterialSurface.rms_height``: the J&R rule above k_low for a
    sediment, the constant otherwise."""
    if S.s_mode[mat] != 0:
        return S.s_rms[mat]
    r_f = wp.sqrt(wp.float64(0.5) * S.lam * wp.max(R_e, wp.float64(1e-6)))
    kl = wp.max(wp.float64(3.141592653589793) / r_f, wp.float64(3.141592653589793) / S.dx)
    g2 = S.s_g2[mat]
    return wp.sqrt(wp.float64(6.283185307179586) * S.s_w2[mat] * wp.pow(kl, wp.float64(2.0) - g2)
                   / (g2 - wp.float64(2.0)))


@wp.struct
class FcCh:
    """One out or back chain of the floor channel."""
    ok: int
    length: wp.float64
    depart: wp.vec3d
    arrive: wp.vec3d
    factor: wp.vec2d


@wp.func
def fc_chain(P: wp.vec3d, X: wp.vec3d, kinds: wp.array(dtype=wp.int32),
             faces: wp.array(dtype=wp.int32), off: int, m: int, S: SpecSheet, F: Faces,
             k: wp.float64, rms_s: wp.float64, lam: wp.float64, max_iter: int) -> FcCh:
    """``chains.build`` (no oracle, refined): direct, the short path or the
    general one; ok only for a valid chain with a non-zero factor."""
    r = FcCh()
    r.ok = 0
    if m == 0:
        d = X - P
        L = wp.length(d)
        r.length = L
        r.depart = d / wp.max(L, wp.float64(1e-300))
        r.arrive = r.depart
        r.factor = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
        r.ok = 1
        return r
    if m <= 2 and n_floors(kinds, off, m) <= 1:
        cs = build_small(P, X, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter, 1)
        if cs.ok == 1 and cabs(cs.factor) > wp.float64(0.0):
            r.ok = 1
        r.length = cs.length
        r.depart = cs.depart
        r.arrive = cs.arrive
        r.factor = cs.factor
        return r
    ch = build_chain(P, X, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter, 1)
    if ch.ok == 1 and cabs(ch.factor) > wp.float64(0.0):
        r.ok = 1
    r.length = ch.length
    r.depart = ch.depart
    r.arrive = ch.arrive
    r.factor = ch.factor
    return r


@wp.func
def fc_vis(o: Oracle, Ly: Layers, P: wp.vec3d, X: wp.vec3d, Xv: wp.vec3d,
           kinds: wp.array(dtype=wp.int32), faces: wp.array(dtype=wp.int32), off: int, m: int,
           S: SpecSheet, F: Faces, k: wp.float64, rms_s: wp.float64, lam: wp.float64,
           max_iter: int, eps: wp.float64) -> wp.vec2d:
    """``check_visibility`` + ``_transmit_blocked`` of one chain from ``Xv``:
    the leg transmission T (1 when clear, 0 when blocked by an opaque
    triangle or leaving the water)."""
    one = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
    zero = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    if m == 0:
        if seg_occluded(o, P, Xv, eps) == 0:
            return one
        if Ly.on == 0:
            return zero
        return seg_transmit(o, Ly, P, Xv, eps)
    if m <= 2 and n_floors(kinds, off, m) <= 1:
        cs = build_small(P, X, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter, 1)
        if blocked_small(o, P, Xv, cs, eps, _BOFF) == 0:
            return one
        if Ly.on == 0:
            return zero
        return transmit_small(o, Ly, P, Xv, cs, eps, _BOFF)
    ch = build_chain(P, X, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter, 1)
    if chain_blocked(o, P, Xv, ch, eps, _BOFF) == 0:
        return one
    if Ly.on == 0:
        return zero
    T = one
    prev = P
    for j in range(ch.m + 1):
        end = Xv
        if j < ch.m:
            end = _row(ch.Q, j) + _BOFF * _row(ch.N, j)
        T = cmul(T, seg_transmit(o, Ly, prev, end, eps))
        prev = end
    return T


@wp.kernel
def _fc_vert_k(S: SpecSheet, W: SheetWin, i0: int, j0: int, nxb: int, A: wp.vec3d, B: wp.vec3d,
               inside: wp.array(dtype=wp.int32), Rev: wp.array(dtype=wp.float64)):
    """``candidate_cells``: the bbox lattice's vertices inside the window."""
    t = wp.tid()
    jj = t // nxb
    ii = t - jj * nxb
    x = S.ox + wp.float64(i0 + ii) * S.dx
    y = S.oy + wp.float64(j0 + jj) * S.dx
    z = spec_locate(S, x, y)[0]
    vn = spec_normal(S, x, y)
    zr = spec_zone(wp.vec3d(x, y, z), vn, A, B, W.k)
    inside[t] = 0
    if zr[0] <= W.n2:
        inside[t] = 1
    Rev[t] = zr[1]


@wp.kernel
def _fc_cell_k(inside: wp.array(dtype=wp.int32), nxb: int, ncx: int,
               c: wp.array(dtype=wp.int32)):
    t = wp.tid()
    jc = t // ncx
    ic = t - jc * ncx
    v = jc * nxb + ic
    c[t] = int(inside[v] | inside[v + nxb] | inside[v + 1] | inside[v + nxb + 1])


@wp.kernel
def _fc_drow_k(c: wp.array(dtype=wp.int32), ncx: int, ncy: int, d1: wp.array(dtype=wp.int32)):
    t = wp.tid()
    jc = t // ncx
    v = c[t]
    if jc > 0:
        v = v | c[t - ncx]
    if jc < ncy - 1:
        v = v | c[t + ncx]
    d1[t] = v


@wp.kernel
def _fc_dcol_k(d1: wp.array(dtype=wp.int32), ncx: int, d: wp.array(dtype=wp.int32)):
    t = wp.tid()
    jc = t // ncx
    ic = t - jc * ncx
    v = d1[t]
    if ic > 0:
        v = v | d1[t - 1]
    if ic < ncx - 1:
        v = v | d1[t + 1]
    d[t] = v


@wp.kernel
def _fc_count_k(d: wp.array(dtype=wp.int32), dscan: wp.array(dtype=wp.int32),
                Rev: wp.array(dtype=wp.float64), nxb: int, ncx: int, W: SheetWin, dx: wp.float64,
                cell: wp.array(dtype=wp.vec2i), nsub: wp.array(dtype=wp.int32),
                reps: wp.array(dtype=wp.int32)):
    """Per candidate cell (row-major, as np.nonzero): its n x n tiling."""
    t = wp.tid()
    if d[t] == 0:
        return
    slot = dscan[t] - 1
    jc = t // ncx
    ic = t - jc * ncx
    Re = Rev[jc * nxb + ic]
    a_t = wp.clamp(wp.sqrt(wp.float64(4.0) * W.eps * wp.max(Re, wp.float64(1e-3)) / W.k),
                   W.a_min, W.a_max)
    ns = wp.max(int(wp.ceil(dx / a_t)), 1)
    cell[slot] = wp.vec2i(ic, jc)
    nsub[slot] = ns
    reps[slot] = ns * ns


@wp.kernel
def _fc_patch_k(cell: wp.array(dtype=wp.vec2i), nsub: wp.array(dtype=wp.int32),
                rscan: wp.array(dtype=wp.int32), i0: int, j0: int, S: SpecSheet, W: SheetWin,
                Xc: wp.array(dtype=wp.vec3d), Nn: wp.array(dtype=wp.vec3d),
                Sm: wp.array(dtype=wp.vec3d), U: wp.array(dtype=wp.vec3d),
                V: wp.array(dtype=wp.vec3d), Ar: wp.array(dtype=wp.float64),
                Mt: wp.array(dtype=wp.int32), Kp: wp.array(dtype=wp.int32)):
    """``SheetSpecular.patches``: each candidate cell's sub-patches, on the
    sheet triangle holding their centres (dry ones flagged off)."""
    q = wp.tid()
    ns = nsub[q]
    start = rscan[q] - ns * ns
    ic = cell[q][0] + i0
    jc = cell[q][1] + j0
    a = S.dx / wp.float64(ns)
    for kk in range(ns * ns):
        sx = kk % ns
        sy = kk // ns
        x = S.ox + (wp.float64(ic) + (wp.float64(sx) + wp.float64(0.5)) / wp.float64(ns)) * S.dx
        y = S.oy + (wp.float64(jc) + (wp.float64(sy) + wp.float64(0.5)) / wp.float64(ns)) * S.dx
        loc = spec_locate(S, x, y)
        tri = int(loc[1])
        vn = spec_normal(S, x, y)
        nrm = wp.vec3d(wp.float64(0.0), wp.float64(0.0), wp.float64(1.0))
        if tri >= 0:
            nrm = W.tri_n[tri]
        elif S.edge == 1:
            nrm = vn
        zx = -nrm[0] / nrm[2]
        zy = -nrm[1] / nrm[2]
        i = start + kk
        Xc[i] = wp.vec3d(x, y, loc[0])
        Nn[i] = nrm
        Sm[i] = vn
        U[i] = wp.vec3d(a, wp.float64(0.0), a * zx)
        V[i] = wp.vec3d(wp.float64(0.0), a, a * zy)
        Ar[i] = a * a / nrm[2]
        fi = floor_info(S, tri)
        Mt[i] = fi[0]
        Kp[i] = 1 - fi[1]


@wp.func
def _sinc_h(x: wp.float64) -> wp.float64:
    """np.sinc(x / pi) = sin(x) / x."""
    if x == wp.float64(0.0):
        return wp.float64(1.0)
    return wp.sin(x) / x


@wp.kernel
def _fc_amp_k(Xc: wp.array(dtype=wp.vec3d), Nn: wp.array(dtype=wp.vec3d),
              Sm: wp.array(dtype=wp.vec3d), U: wp.array(dtype=wp.vec3d),
              V: wp.array(dtype=wp.vec3d), Ar: wp.array(dtype=wp.float64),
              Mt: wp.array(dtype=wp.int32), Kp: wp.array(dtype=wp.int32), P: wp.vec3d,
              kinds: wp.array(dtype=wp.int32), faces: wp.array(dtype=wp.int32), off_b: int,
              m_o: int, m_b: int, S: SpecSheet, F: Faces, W: SheetWin, k: wp.float64,
              rms_s: wp.float64, lam: wp.float64, max_iter: int, B: Beam, e_a: wp.vec3d,
              e_n: wp.vec3d, e_v: wp.vec3d, alpha_np: wp.float64, c: wp.float64,
              r_max: wp.float64, a_o: wp.array(dtype=wp.vec2d), tau_o: wp.array(dtype=wp.float64),
              live_o: wp.array(dtype=wp.int32)):
    """``floor_coherent``'s per-patch amplitude (no visibility yet): the out
    chain (offset 0) and the back chain (offset ``off_b``) to the patch
    centre, ``amplitudes_dirs``, the two patterns and the spreading."""
    i = wp.tid()
    live_o[i] = 0
    if Kp[i] == 0:
        return
    X = Xc[i]
    co = fc_chain(P, X, kinds, faces, 0, m_o, S, F, k, rms_s, lam, max_iter)
    cb = fc_chain(P, X, kinds, faces, off_b, m_b, S, F, k, rms_s, lam, max_iter)
    if co.ok == 0 or cb.ok == 0:
        return
    ra = co.length
    rb = cb.length
    if ra > r_max or rb > r_max:
        return
    inc = co.arrive
    out = -cb.arrive
    n = Nn[i]
    q = W.k * (inc - out)
    qu = wp.dot(U[i], q)
    qv = wp.dot(V[i], q)
    nd = wp.dot(n, out - inc)
    cos_b = wp.float64(0.5) * nd
    if not ((-wp.dot(n, inc) > wp.float64(0.0)) and (wp.dot(n, out) > wp.float64(0.0))):
        return
    area = Ar[i] * _sinc_h(wp.float64(0.5) * qu) * _sinc_h(wp.float64(0.5) * qv)
    sin_th = wp.clamp(-wp.dot(n, inc), wp.float64(0.0), wp.float64(1.0))
    mat = Mt[i]
    Vr = reflection(S.s_arho[mat], S.s_ap[mat], sin_th)
    Re = wp.float64(2.0) * ra * rb / wp.max(ra + rb, wp.float64(1e-300))
    g = W.k * wp.abs(nd) * spec_rms(S, mat, Re)
    sq = wp.exp(wp.float64(-0.5) * wp.min(g * g, wp.float64(1400.0)))
    A_img = X - inc * ra
    B_img = X + out * rb
    w = spec_window(spec_zone(X, Sm[i], A_img, B_img, W.k)[0], W.n1, W.n2)
    sc = (W.k / wp.float64(6.283185307179586)) * wp.max(cos_b, wp.float64(0.0)) * area * w * sq
    amp = wp.vec2d(-Vr[1] * sc, Vr[0] * sc)               # i k / (2 pi) ... V
    if not (w > wp.float64(0.0)) or (amp[0] == wp.float64(0.0) and amp[1] == wp.float64(0.0)):
        return
    bo = pattern(B, co.depart * ra, e_a, e_n, e_v)
    bb = pattern(B, cb.depart * rb, e_a, e_n, e_v)
    L = ra + rb
    a = cmul(cmul(cmul(cmul(amp, bo), bb), co.factor), cb.factor)
    s2 = wp.exp(-alpha_np * L) / (ra * rb)
    a_o[i] = wp.vec2d(a[0] * s2, a[1] * s2)
    tau_o[i] = L / c
    live_o[i] = 1


@wp.kernel
def _fc_dep_k(Xc: wp.array(dtype=wp.vec3d), Nn: wp.array(dtype=wp.vec3d),
              live: wp.array(dtype=wp.int32), a_i: wp.array(dtype=wp.vec2d),
              tau_i: wp.array(dtype=wp.float64), P: wp.vec3d, kinds: wp.array(dtype=wp.int32),
              faces: wp.array(dtype=wp.int32), off_b: int, m_o: int, m_b: int, S: SpecSheet,
              F: Faces, k: wp.float64, rms_s: wp.float64, lam: wp.float64, max_iter: int,
              o: Oracle, Ly: Layers, offset: wp.float64, eps: wp.float64, x: Rx,
              count: wp.array(dtype=wp.int32)):
    """The kept pair's patches: both chains' visibility from the lifted
    centre and their transmission, then the coherent deposit."""
    i = wp.tid()
    if live[i] == 0:
        return
    X = Xc[i]
    Xv = X + offset * Nn[i]
    To = fc_vis(o, Ly, P, X, Xv, kinds, faces, 0, m_o, S, F, k, rms_s, lam, max_iter, eps)
    if not (cabs(To) > wp.float64(0.0)):
        return
    Tb = fc_vis(o, Ly, P, X, Xv, kinds, faces, off_b, m_b, S, F, k, rms_s, lam, max_iter, eps)
    if not (cabs(Tb) > wp.float64(0.0)):
        return
    wp.atomic_add(count, 0, 1)
    tau = tau_i[i]
    ca = cmul(cmul(cmul(a_i[i], To), Tb), carrier(x, tau))
    cp = i % x.ncopy
    cen = tap_centre(x, tau)
    for jj in range(-x.kk, x.kk + 1):
        bb = cen + jj
        if bb < 0 or bb >= x.n:
            continue
        w = env_at(x, t_bin(x, bb) - tau)
        if w == wp.float64(0.0):
            continue
        v = wp.vec2d(ca[0] * w, ca[1] * w)
        wp.atomic_add(x.field, cp, bb, v)
        wp.atomic_add(x.coh, cp, bb, v)


# ======================================================= pair kernel factory
def _chain_tables(maxc: int):
    """Per-contributor chain / pair tables of a pair kernel holding ``maxc``
    chains (static sizes: a light variant for the common case, a heavy one
    for deep image combs)."""
    maxp = maxc * (maxc + 1) // 2
    return (wp.types.vector(length=maxc, dtype=wp.float64),
            wp.types.vector(length=maxc, dtype=wp.int32),
            wp.types.matrix(shape=(maxc, 3), dtype=wp.float64),
            wp.types.vector(length=maxp, dtype=wp.float64))


def _unique_kernel(fn):
    k = wp.kernel(fn, module="unique", enable_backward=False)
    wp.set_module_options({"fuse_fp": False}, module=k.module)
    return k


def _make_pairs_k(maxc: int):
    _vecC, _vecCi, _matC3, _vecP = _chain_tables(maxc)
    _MAXC = wp.constant(maxc)

    def _pairs_k(pos: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3d),
                 area: wp.array(dtype=wp.float64), mat: wp.array(dtype=wp.int32),
                 phase: wp.array(dtype=wp.float64), ray: wp.array(dtype=wp.float64),
                 onfl: wp.array(dtype=wp.int32), wallof: wp.array(dtype=wp.int32),
                 R_d: wp.array(dtype=wp.float64),
                 b_d: wp.array(dtype=wp.vec2d), state: wp.array(dtype=wp.int32),
                 D: wp.array(dtype=wp.vec2d), Tt: wp.array(dtype=wp.vec2d), Ly: Layers, P: wp.vec3d, e_a: wp.vec3d, e_n: wp.vec3d,
                 e_v: wp.vec3d, B: Beam, L: Law, o: Oracle, S: SpecSheet, F: Faces,
                 kinds: wp.array(dtype=wp.int32), faces: wp.array(dtype=wp.int32),
                 koff: wp.array(dtype=wp.int32), nk: int, k: wp.float64, rms_s: wp.float64,
                 alpha_np: wp.float64, c: wp.float64, floor_pw: wp.float64, offset: wp.float64,
                 eps: wp.float64, lam: wp.float64, max_iter: int, x: Rx,
                 npairs_o: wp.array(dtype=wp.int32)):
        """Every contributor through every pair of its chains (direct + the kept
        sequences), ``Multipath.diffuse``'s deposit: pairs (i, j) and (j, i)
        folded (reciprocity), the field ``det f`` and the contributor's own
        coherent mean ``|sum det w|^2`` per bin (one thread holds every pair of
        its scatterer, so the cross terms are summed exactly)."""
        t = wp.tid()
        npairs_o[t] = 0
        st0 = int(state[t])
        if (st0 & 4) != 0:
            return
        X = pos[t]
        nv = nrm[t]
        Xv = X + offset * nv
        # chain table: 0 = direct
        Lc = _vecC()
        vis = _vecCi()
        Fre = _vecC()
        Fim = _vecC()
        bre = _vecC()
        bim = _vecC()
        Arr = _matC3()
        nc = int(1)
        Lc[0] = R_d[t]
        u0 = unit3(X - P)
        Arr[0, 0] = u0[0]
        Arr[0, 1] = u0[1]
        Arr[0, 2] = u0[2]
        dd0 = D[t]
        vis[0] = 0
        if cabs(dd0) > wp.float64(0.0):
            vis[0] = 1
        dd0 = cmul(dd0, Tt[t])                    # D and T ride on the direct chain's factor
        Fre[0] = dd0[0]
        Fim[0] = dd0[1]
        bre[0] = b_d[t][0]
        bim[0] = b_d[t][1]
        for s in range(nk):
            if nc >= _MAXC:
                break
            off = int(koff[s])
            m = int(koff[s + 1]) - off
            if own_bad(kinds, faces, off, m, int(onfl[t]), int(wallof[t]), F) == 1:
                continue
            ce = eval_chain(P, X, Xv, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter, B,
                            e_a, e_n, e_v, o, Ly, eps, offset)
            if ce.ok == 0:
                continue
            Lc[nc] = ce.length
            vis[nc] = 1
            Fre[nc] = ce.factor[0]
            Fim[nc] = ce.factor[1]
            bre[nc] = ce.b[0]
            bim[nc] = ce.b[1]
            Arr[nc, 0] = ce.arrive[0]
            Arr[nc, 1] = ce.arrive[1]
            Arr[nc, 2] = ce.arrive[2]
            nc = nc + 1
        # pairs
        f = wp.vec2d(ray[t] * wp.cos(phase[t]), ray[t] * wp.sin(phase[t]))
        pre = _vecP()
        pim = _vecP()
        ptau = _vecP()
        np_ = int(0)
        mt = int(mat[t])
        for i in range(nc):
            if vis[i] == 0:
                continue
            for j in range(i, nc):
                if vis[j] == 0:
                    continue
                bi = wp.vec2d(bre[i], bim[i])
                bj = wp.vec2d(bre[j], bim[j])
                two = cmul(bi, bj)
                if cabs2(two) < floor_pw:
                    continue
                ai = wp.vec3d(Arr[i, 0], Arr[i, 1], Arr[i, 2])
                aj = wp.vec3d(Arr[j, 0], Arr[j, 1], Arr[j, 2])
                sig = wp.float64(sigma_bistatic(L, wp.vec3f(nv), wp.vec3f(ai), wp.vec3f(-aj), mt))
                Lt = Lc[i] + Lc[j]
                w = wp.float64(2.0)
                if i == j:
                    w = wp.float64(1.0)
                amp = w * wp.exp(-alpha_np * Lt) / (Lc[i] * Lc[j]) * wp.sqrt(sig * area[t])
                det = cmul(cmul(two, wp.vec2d(Fre[i], Fim[i])), wp.vec2d(Fre[j], Fim[j]))
                det = wp.vec2d(det[0] * amp, det[1] * amp)
                if det[0] == wp.float64(0.0) and det[1] == wp.float64(0.0):
                    continue
                pre[np_] = det[0]
                pim[np_] = det[1]
                ptau[np_] = Lt / c
                np_ = np_ + 1
        npairs_o[t] = np_
        cp = t % x.ncopy
        # field: sum det f carrier w; mean: |sum det carrier w|^2 per bin
        for p in range(np_):
            tau = ptau[p]
            cr = carrier(x, tau)
            dp = cmul(wp.vec2d(pre[p], pim[p]), cr)
            fp = cmul(dp, f)
            cen = tap_centre(x, tau)
            for jj in range(-x.kk, x.kk + 1):
                bb = cen + jj
                if bb < 0 or bb >= x.n:
                    continue
                w = env_at(x, t_bin(x, bb) - tau)
                if w == wp.float64(0.0):
                    continue
                wp.atomic_add(x.field, cp, bb, wp.vec2d(fp[0] * w, fp[1] * w))
                acc = cabs2(dp) * w * w
                for q in range(p + 1, np_):
                    tq = ptau[q]
                    if wp.abs(tap_centre(x, tq) - cen) > 2 * x.kk + 1:
                        continue
                    wq = env_at(x, t_bin(x, bb) - tq)
                    if wq == wp.float64(0.0):
                        continue
                    dq = cmul(wp.vec2d(pre[q], pim[q]), carrier(x, tq))
                    acc = acc + wp.float64(2.0) * (dp[0] * dq[0] + dp[1] * dq[1]) * w * wq
                wp.atomic_add(x.mean, cp, bb, acc)

    return _unique_kernel(_pairs_k)


def _make_facet_pairs_k(maxc: int):
    _vecC, _vecCi, _matC3, _vecP = _chain_tables(maxc)
    _MAXC = wp.constant(maxc)

    def _facet_pairs_k(X: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3d),
                       pu: wp.array(dtype=wp.vec3d), pv: wp.array(dtype=wp.vec3d),
                       pa: wp.array(dtype=wp.float64), pb: wp.array(dtype=wp.float64),
                       state: wp.array(dtype=wp.int32), P: wp.vec3d, e_a: wp.vec3d, e_n: wp.vec3d,
                       e_v: wp.vec3d, B: Beam, o: Oracle, Ly: Layers, S: SpecSheet, F: Faces,
                       kinds: wp.array(dtype=wp.int32), faces: wp.array(dtype=wp.int32),
                       koff: wp.array(dtype=wp.int32), is_out: wp.array(dtype=wp.int32), nk: int,
                       refl: wp.float64, rms: wp.float64, k: wp.float64, rms_s: wp.float64,
                       alpha_np: wp.float64, c: wp.float64, floor_pw: wp.float64,
                       offset: wp.float64, eps: wp.float64, lam: wp.float64, max_iter: int, x: Rx,
                       ndep_o: wp.array(dtype=wp.int32)):
        """``Multipath.facets``' deposit for one patch: its chains (direct + the
        kept sequences), the PO bistatic amplitude for every (out, back) pair
        with the out chain free of object faces (PO at the first bounce in
        time), deposited as coherent contributors."""
        t = wp.tid()
        ndep_o[t] = 0
        if (int(state[t]) & 4) != 0:
            return
        Xp = X[t]
        nv = nrm[t]
        Xv = Xp + offset * nv
        Lc = _vecC()
        vis = _vecCi()
        outf = _vecCi()
        Fre = _vecC()
        Fim = _vecC()
        bre = _vecC()
        bim = _vecC()
        Arr = _matC3()
        nc = int(0)
        for s in range(nk + 1):
            if nc >= _MAXC:
                break
            ce = ChainOut()
            if s == 0:
                ce.ok = 0
                Ld = wp.length(Xp - P)
                u0 = unit3(Xp - P)
                b0 = pattern(B, Xp - P, e_a, e_n, e_v)
                if cabs(b0) > wp.float64(0.0):
                    T0 = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
                    okd = int(1)
                    if seg_occluded(o, P, Xv, eps) == 1:
                        okd = 0
                        if Ly.on == 1:
                            T0 = seg_transmit(o, Ly, P, Xv, eps)
                            if cabs(T0) > wp.float64(0.0):
                                okd = 1
                    if okd == 1:
                        ce.ok = 1
                        ce.length = Ld
                        ce.arrive = u0
                        ce.b = b0
                        ce.factor = T0
            else:
                off = int(koff[s - 1])
                m = int(koff[s]) - off
                ce = eval_chain(P, Xp, Xv, kinds, faces, off, m, S, F, k, rms_s, lam, max_iter, B,
                                e_a, e_n, e_v, o, Ly, eps, offset)
            if ce.ok == 0:
                continue
            Lc[nc] = ce.length
            vis[nc] = 1
            outf[nc] = 1
            if s > 0:
                outf[nc] = int(is_out[s - 1])
            Fre[nc] = ce.factor[0]
            Fim[nc] = ce.factor[1]
            bre[nc] = ce.b[0]
            bim[nc] = ce.b[1]
            Arr[nc, 0] = ce.arrive[0]
            Arr[nc, 1] = ce.arrive[1]
            Arr[nc, 2] = ce.arrive[2]
            nc = nc + 1
        cp = t % x.ncopy
        nd = int(0)
        for i in range(nc):
            if outf[i] == 0:
                continue
            for j in range(nc):
                two = cmul(wp.vec2d(bre[i], bim[i]), wp.vec2d(bre[j], bim[j]))
                if cabs2(two) < floor_pw:
                    continue
                ai = wp.vec3d(Arr[i, 0], Arr[i, 1], Arr[i, 2])
                aj = wp.vec3d(Arr[j, 0], Arr[j, 1], Arr[j, 2])
                po = po_pair_amp(nv, pu[t], pv[t], pa[t], pb[t], ai, -aj, k, refl, rms)
                Lt = Lc[i] + Lc[j]
                amp = cmul(cmul(cmul(two, wp.vec2d(Fre[i], Fim[i])), wp.vec2d(Fre[j], Fim[j])), po)
                sc = wp.exp(-alpha_np * Lt) / (Lc[i] * Lc[j])
                amp = wp.vec2d(amp[0] * sc, amp[1] * sc)
                if amp[0] == wp.float64(0.0) and amp[1] == wp.float64(0.0):
                    continue
                nd = nd + 1
                tau = Lt / c
                ca = cmul(amp, carrier(x, tau))
                cen = tap_centre(x, tau)
                for jj in range(-x.kk, x.kk + 1):
                    bb = cen + jj
                    if bb < 0 or bb >= x.n:
                        continue
                    w = env_at(x, t_bin(x, bb) - tau)
                    if w == wp.float64(0.0):
                        continue
                    v = wp.vec2d(ca[0] * w, ca[1] * w)
                    wp.atomic_add(x.field, cp, bb, v)
                    wp.atomic_add(x.coh, cp, bb, v)
        ndep_o[t] = nd

    return _unique_kernel(_facet_pairs_k)


_PAIR_KERNELS: dict = {}

#: chains per contributor of the light and heavy pair kernels
PAIR_SIZES = (4, 32)


def pair_kernel(kind: str, nchains: int):
    """The pair kernel (``pairs`` or ``facets``) holding ``nchains`` chains
    per contributor: the light variant when it suffices, else the heavy one;
    beyond it an explicit error, never a silent truncation."""
    size = next((m for m in PAIR_SIZES if nchains <= m), None)
    if size is None:
        raise ValueError(f"GPU pipeline: {nchains} chains per contributor > {PAIR_SIZES[-1]}")
    key = (kind, size)
    k = _PAIR_KERNELS.get(key)
    if k is None:
        k = _PAIR_KERNELS[key] = (_make_pairs_k if kind == "pairs" else _make_facet_pairs_k)(size)
    return k


@wp.kernel
def _reduce_k(src: wp.array2d(dtype=wp.float64), out: wp.array(dtype=wp.float64)):
    b = wp.tid()
    s = wp.float64(0.0)
    for c in range(src.shape[0]):
        s = s + src[c, b]
    out[b] = s


@wp.kernel
def _reduce_c_k(src: wp.array2d(dtype=wp.vec2d), out: wp.array(dtype=wp.vec2d)):
    b = wp.tid()
    s = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    for c in range(src.shape[0]):
        s = s + src[c, b]
    out[b] = s


# ================================================================ diffraction
_NE = 20                                      # transitions per direction (<= coarse)
_NDIR = 16                                    # base directions (<= sectors)
_NSTK = 8                                     # DFS stack (max depth 6 + 1)
_vecE = wp.types.vector(length=_NE, dtype=wp.float64)
_matDE = wp.types.matrix(shape=(_NDIR, _NE), dtype=wp.float64)
_vecD = wp.types.vector(length=_NDIR, dtype=wp.int32)
_matSE = wp.types.matrix(shape=(_NSTK, _NE), dtype=wp.float64)
_vecS = wp.types.vector(length=_NSTK, dtype=wp.float64)
_vecSi = wp.types.vector(length=_NSTK, dtype=wp.int32)


@wp.struct
class Disc:
    """``diffraction.FresnelDisc`` + its tabulated F, and the oracle rules
    of ``Diffraction`` (eps, end margin)."""
    n1: wp.float64
    n2: wp.float64
    sectors: int
    coarse: int
    bisect: int
    lam: wp.float64
    eps: wp.float64
    end_margin: wp.float64
    ds: wp.float64
    F_re: wp.array(dtype=wp.float64)
    F_im: wp.array(dtype=wp.float64)
    full: wp.vec2d
    use32: int                               # 1: float32 visibility samples (default)
    F32_re: wp.array(dtype=wp.float32)       # the table in float32 (chord integrals)
    F32_im: wp.array(dtype=wp.float32)
    ds32: wp.float32


@wp.func
def disc_F(Dc: Disc, sv: wp.float64) -> wp.vec2d:
    """``Diffraction._F``: np.interp of the cumulative table (clamped)."""
    n = Dc.F_re.shape[0]
    f = sv / Dc.ds
    if f <= wp.float64(0.0):
        return wp.vec2d(Dc.F_re[0], Dc.F_im[0])
    if f >= wp.float64(n - 1):
        return wp.vec2d(Dc.F_re[n - 1], Dc.F_im[n - 1])
    i = wp.min(int(f), n - 2)
    t = f - wp.float64(i)
    return wp.vec2d(Dc.F_re[i] + t * (Dc.F_re[i + 1] - Dc.F_re[i]),
                    Dc.F_im[i] + t * (Dc.F_im[i + 1] - Dc.F_im[i]))


@wp.func
def disc_F32(Dc: Disc, sv: wp.float32) -> wp.vec2f:
    """``disc_F`` in float32 (the chord integrals' samples)."""
    n = Dc.F32_re.shape[0]
    f = sv / Dc.ds32
    if f <= 0.0:
        return wp.vec2f(Dc.F32_re[0], Dc.F32_im[0])
    if f >= wp.float32(n - 1):
        return wp.vec2f(Dc.F32_re[n - 1], Dc.F32_im[n - 1])
    i = wp.min(int(f), n - 2)
    t = f - wp.float32(i)
    return wp.vec2f(Dc.F32_re[i] + t * (Dc.F32_re[i + 1] - Dc.F32_re[i]),
                    Dc.F32_im[i] + t * (Dc.F32_im[i + 1] - Dc.F32_im[i]))


@wp.func
def diff_visible(o: Oracle, Dc: Disc, A: wp.vec3d, B: wp.vec3d, xi: wp.vec3d) -> int:
    """``Diffraction._visible``: A -> xi and xi -> B clear (B pulled back by
    the end margin)."""
    if Dc.use32 == 1:
        if seg_occluded32(o, A, xi, Dc.eps) == 1:
            return 0
    elif seg_occluded(o, A, xi, Dc.eps) == 1:
        return 0
    Bi = B
    if Dc.end_margin > wp.float64(0.0):
        d = B - xi
        L = wp.length(d)
        Bi = xi + d * (wp.max(L - Dc.end_margin, wp.float64(1e-6)) / wp.max(L, wp.float64(1e-300)))
    if Dc.use32 == 1:
        if seg_occluded32(o, xi, Bi, Dc.eps) == 1:
            return 0
    elif seg_occluded(o, xi, Bi, Dc.eps) == 1:
        return 0
    return 1


@wp.struct
class Frame:
    C: wp.vec3d
    de: wp.float64
    e1: wp.vec3d
    e2: wp.vec3d


@wp.func
def diff_frame(A: wp.vec3d, B: wp.vec3d, d1: wp.float64) -> Frame:
    """``Diffraction._frame``."""
    d = B - A
    L = wp.length(d)
    u = d / L
    dc = wp.clamp(d1, wp.float64(1e-4), L - wp.float64(1e-4))
    fr = Frame()
    fr.de = dc * (L - dc) / L
    a = wp.vec3d(wp.float64(1.0), wp.float64(0.0), wp.float64(0.0))
    if wp.abs(u[2]) < wp.float64(0.9):
        a = wp.vec3d(wp.float64(0.0), wp.float64(0.0), wp.float64(1.0))
    e1 = wp.cross(u, a)
    fr.e1 = e1 / wp.length(e1)
    fr.e2 = wp.cross(u, fr.e1)
    fr.C = A + dc * u
    return fr


@wp.struct
class Row:
    st0: int
    cnt: int
    e: _vecE


@wp.func
def diff_row(o: Oracle, Dc: Disc, A: wp.vec3d, B: wp.vec3d, fr: Frame, th: wp.float64,
             v0: int) -> Row:
    """``Diffraction._edges`` along the direction at angle ``th``: the
    coarse samples, every transition bisected. Sample 0 is the disc centre
    for every direction: its visibility ``v0`` is computed once per leg."""
    dirv = wp.cos(th) * fr.e1 + wp.sin(th) * fr.e2
    K = Dc.coarse + 1
    step = Dc.n2 / wp.float64(Dc.coarse)
    lde = Dc.lam * fr.de
    rw = Row()
    rw.cnt = 0
    prev = int(0)
    s_prev = wp.float64(0.0)
    for kk in range(K):
        sc = wp.float64(kk) * step
        if kk == K - 1:
            sc = Dc.n2
        v = v0
        if kk > 0:
            v = diff_visible(o, Dc, A, B, fr.C + wp.sqrt(lde * sc) * dirv)
        if kk == 0:
            rw.st0 = v
        elif v != prev and rw.cnt < _NE:
            a_s = s_prev
            b_s = sc
            for it in range(Dc.bisect):
                m = wp.float64(0.5) * (a_s + b_s)
                vm = diff_visible(o, Dc, A, B, fr.C + wp.sqrt(lde * m) * dirv)
                if vm == prev:
                    a_s = m
                else:
                    b_s = m
            rw.e[rw.cnt] = wp.float64(0.5) * (a_s + b_s)
            rw.cnt = rw.cnt + 1
        prev = v
        s_prev = sc
    return rw


@wp.func
def chord_H(Dc: Disc, s0: wp.float64, s1: wp.float64, t0: wp.float64, dth: wp.float64) -> wp.vec2d:
    """``Diffraction._chord_H`` for one chord (its own sample count), in
    float32 (angles < 2 pi, table values O(1))."""
    m = wp.min(wp.max(16, int(wp.ceil(wp.float64(4.0) * wp.abs(s1 - s0))) + 16), 512)
    r0 = wp.sqrt(wp.max(wp.float32(s0), 0.0))
    r1 = wp.sqrt(wp.max(wp.float32(s1), 0.0))
    a0 = wp.float32(t0)
    da = wp.float32(dth)
    p0x = r0 * wp.cos(a0)
    p0y = r0 * wp.sin(a0)
    p1x = r1 * wp.cos(a0 + da)
    p1y = r1 * wp.sin(a0 + da)
    ex = p1x - p0x
    ey = p1y - p0y
    num = p0x * ey - p0y * ex
    rmax = wp.sqrt(wp.float32(Dc.n2))
    re = float(0.0)
    im = float(0.0)
    mf = wp.float32(m)
    for j in range(m):
        t = a0 + (wp.float32(j) + 0.5) / mf * da
        den = wp.cos(t) * ey - wp.sin(t) * ex
        r = 0.5 * (r0 + r1)
        if wp.abs(den) > 1.0e-12:
            r = num / den
        r = wp.clamp(r, 0.0, rmax)
        F = disc_F32(Dc, r * r)
        re += F[0]
        im += F[1]
    return wp.vec2d(wp.float64(re / mf), wp.float64(im / mf))


@wp.func
def row_value(Dc: Disc, st0: int, cnt: int, e: _vecE) -> wp.vec2d:
    """``Diffraction._value`` (float32 table)."""
    re = float(0.0)
    im = float(0.0)
    st = st0
    lo = wp.float32(0.0)
    for j in range(cnt):
        if st == 1:
            a = disc_F32(Dc, wp.float32(e[j]))
            b = disc_F32(Dc, lo)
            re += a[0] - b[0]
            im += a[1] - b[1]
        lo = wp.float32(e[j])
        st = 1 - st
    if st == 1:
        a = disc_F32(Dc, wp.float32(Dc.n2))
        b = disc_F32(Dc, lo)
        re += a[0] - b[0]
        im += a[1] - b[1]
    return wp.vec2d(wp.float64(re), wp.float64(im))


@wp.func
def same_sector(Dc: Disc, t0: wp.float64, dth: wp.float64, st0: int, cnt: int, e0: _vecE,
                e1: _vecE) -> wp.vec2d:
    """``Diffraction._sector``'s straight-chord branch (rows agreeing)."""
    lin = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    st = st0
    lo0 = wp.float64(0.0)
    lo1 = wp.float64(0.0)
    for j in range(cnt + 1):
        h0 = Dc.n2
        h1 = Dc.n2
        if j < cnt:
            h0 = e0[j]
            h1 = e1[j]
        if st == 1:
            lin = lin + chord_H(Dc, h0, h1, t0, dth) - chord_H(Dc, lo0, lo1, t0, dth)
        lo0 = h0
        lo1 = h1
        st = 1 - st
    return lin


@wp.func
def diff_factor(o: Oracle, Dc: Disc, A: wp.vec3d, B: wp.vec3d, d1: wp.float64, v0: int) -> wp.vec2d:
    """``Diffraction.factor`` for one leg: the base directions' rows, then
    every sector integrated (straight chords where its bounding rows agree,
    bisected with a new direction where they do not, up to depth 6, then
    the trapezoid), depth first on an explicit stack."""
    fr = diff_frame(A, B, d1)
    N = Dc.sectors
    dth0 = _TWO_PI / wp.float64(N)
    bst = _vecD()
    bcn = _vecD()
    be = _matDE()
    for kd in range(N):
        rw = diff_row(o, Dc, A, B, fr, wp.float64(kd) * dth0, v0)
        bst[kd] = rw.st0
        bcn[kd] = rw.cnt
        for j in range(_NE):
            be[kd, j] = rw.e[j]
    total = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    for kd in range(N):
        # stack: (t0, dth, depth, row0, row1)
        St0 = _vecS()
        Sdt = _vecS()
        Sdp = _vecSi()
        Sa_st = _vecSi()
        Sa_cn = _vecSi()
        Sb_st = _vecSi()
        Sb_cn = _vecSi()
        Sa_e = _matSE()
        Sb_e = _matSE()
        k2 = (kd + 1) % N
        St0[0] = wp.float64(kd) * dth0
        Sdt[0] = dth0
        Sdp[0] = 6
        Sa_st[0] = bst[kd]
        Sa_cn[0] = bcn[kd]
        Sb_st[0] = bst[k2]
        Sb_cn[0] = bcn[k2]
        for j in range(_NE):
            Sa_e[0, j] = be[kd, j]
            Sb_e[0, j] = be[k2, j]
        top = int(1)
        while top > 0:
            top = top - 1
            t0 = St0[top]
            dth = Sdt[top]
            dep = Sdp[top]
            wgt = dth / dth0
            ea = _vecE()
            eb = _vecE()
            for j in range(_NE):
                ea[j] = Sa_e[top, j]
                eb[j] = Sb_e[top, j]
            if Sa_st[top] == Sb_st[top] and Sa_cn[top] == Sb_cn[top]:
                v = same_sector(Dc, t0, dth, Sa_st[top], Sa_cn[top], ea, eb)
                total = total + wp.vec2d(v[0] * wgt, v[1] * wgt)
            elif dep <= 0 or top + 2 > _NSTK:
                va = row_value(Dc, Sa_st[top], Sa_cn[top], ea)
                vb = row_value(Dc, Sb_st[top], Sb_cn[top], eb)
                total = total + wp.vec2d(wp.float64(0.5) * (va[0] + vb[0]) * wgt,
                                         wp.float64(0.5) * (va[1] + vb[1]) * wgt)
            else:
                tm = t0 + wp.float64(0.5) * dth
                rm = diff_row(o, Dc, A, B, fr, tm, v0)
                a_st = Sa_st[top]
                a_cn = Sa_cn[top]
                b_st = Sb_st[top]
                b_cn = Sb_cn[top]
                # right half (tm, rm .. row1) at top, left half (t0, row0 .. rm) above it
                St0[top] = tm
                Sdt[top] = wp.float64(0.5) * dth
                Sdp[top] = dep - 1
                Sa_st[top] = rm.st0
                Sa_cn[top] = rm.cnt
                Sb_st[top] = b_st
                Sb_cn[top] = b_cn
                for j in range(_NE):
                    Sa_e[top, j] = rm.e[j]
                    Sb_e[top, j] = eb[j]
                St0[top + 1] = t0
                Sdt[top + 1] = wp.float64(0.5) * dth
                Sdp[top + 1] = dep - 1
                Sa_st[top + 1] = a_st
                Sa_cn[top + 1] = a_cn
                Sb_st[top + 1] = rm.st0
                Sb_cn[top + 1] = rm.cnt
                for j in range(_NE):
                    Sa_e[top + 1, j] = ea[j]
                    Sb_e[top + 1, j] = rm.e[j]
                top = top + 2
    return cdiv(total, wp.vec2d(Dc.full[0] * wp.float64(N), Dc.full[1] * wp.float64(N)))


@wp.func
def diff_classify(o: Oracle, Dc: Disc, A: wp.vec3d, B: wp.vec3d, d1: wp.float64, v0: int) -> int:
    """``Diffraction.classify``: 1 every ring sample and the centre clear,
    0 the centre and every ring to n1 zones blocked, -1 an edge in the
    disc."""
    fr = diff_frame(A, B, d1)
    all_clear = v0
    inner_blocked = 1 - v0
    # early exits, the same answer: a lit centre is an edge at its first
    # blocked sample; a blocked centre is an edge at the first visible inner
    # sample, and deep shadow once the inner rings (zones <= n1) are all
    # blocked -- the outer samples cannot change either
    for zi in range(5):
        if all_clear == 0 and inner_blocked == 0:
            return -1
        if v0 == 0 and zi >= 3:
            return 0
        z = wp.float64(0.5)
        if zi == 1:
            z = wp.float64(2.0)
        elif zi == 2:
            z = Dc.n1
        elif zi == 3:
            z = wp.float64(0.5) * (Dc.n1 + Dc.n2)
        elif zi == 4:
            z = Dc.n2
        r = wp.sqrt(Dc.lam * fr.de * z)
        for ai in range(8):
            t = wp.float64(ai) * _TWO_PI / wp.float64(8.0)
            vis = diff_visible(o, Dc, A, B, fr.C + r * (wp.cos(t) * fr.e1 + wp.sin(t) * fr.e2))
            if vis == 0:
                all_clear = 0
            if z <= Dc.n1 and vis == 1:
                inner_blocked = 0
            if all_clear == 0 and inner_blocked == 0:
                return -1
    if all_clear == 1:
        return 1
    if inner_blocked == 1:
        return 0
    return -1


@wp.kernel
def _diff_hit_k(pos: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3d),
                state: wp.array(dtype=wp.int32), P: wp.vec3d, offset: wp.float64, o: Oracle,
                cell: wp.float64, slot: wp.array(dtype=wp.int32),
                keys: wp.array(dtype=wp.int64), vals: wp.array(dtype=wp.int32),
                frac_o: wp.array(dtype=wp.float64)):
    """Blocked candidates: the hit fraction along P -> X (``SceneOracle.
    closest``, the sea surface included; 0.5 L when nothing is hit) and the
    nine neighbour-cell entries of the lit-side table."""
    i = wp.tid()
    frac_o[i] = wp.float64(-1.0)
    st = int(state[i])
    if not ((st & 1) != 0 and (st & 2) == 0):
        return
    Xv = pos[i] + offset * nrm[i]
    d = Xv - P
    L = wp.length(d)
    u = d / L
    tmax = L - wp.float64(1e-6)
    h = seg_closest(o, P, u, wp.float64(1e-6), tmax)
    t = wp.float64(0.5) * L
    if int(h[1]) >= 0:
        t = h[0]
    if u[2] > wp.float64(0.0) and P[2] < o.zs:
        ts = (o.zs - P[2]) / u[2]
        if ts > wp.float64(1e-6) and ts <= tmax and (int(h[1]) < 0 or ts < h[0]):
            t = ts
    fr = wp.clamp(t / L, wp.float64(0.02), wp.float64(0.98))
    frac_o[i] = fr
    kx = wp.int64(wp.floor(Xv[0] / cell))
    ky = wp.int64(wp.floor(Xv[1] / cell))
    base = int(slot[i]) * 9 - 9                # inclusive scan
    c = int(0)
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            keys[base + c] = ((kx + wp.int64(dx) + wp.int64(1073741824)) << wp.int64(32)) | \
                (ky + wp.int64(dy) + wp.int64(1073741824))
            vals[base + c] = i
            c = c + 1


@wp.kernel
def _blocked_flag_k(state: wp.array(dtype=wp.int32), flag: wp.array(dtype=wp.int32)):
    i = wp.tid()
    st = int(state[i])
    flag[i] = 0
    if (st & 1) != 0 and (st & 2) == 0:
        flag[i] = 1


@wp.kernel
def _run_min_k(keys: wp.array(dtype=wp.int64), vals: wp.array(dtype=wp.int32),
               frac: wp.array(dtype=wp.float64), n: int, ukeys: wp.array(dtype=wp.int64),
               umin: wp.array(dtype=wp.float64), runid: wp.array(dtype=wp.int32)):
    """Per run of equal keys (sorted): the minimum fraction, at the run's
    index (``runid`` = inclusive scan of run starts)."""
    j = wp.tid()
    if j > 0 and keys[j] == keys[j - 1]:
        return
    r = int(runid[j]) - 1
    m = frac[int(vals[j])]
    jj = j + 1
    while jj < n and keys[jj] == keys[j]:
        m = wp.min(m, frac[int(vals[jj])])
        jj = jj + 1
    ukeys[r] = keys[j]
    umin[r] = m


@wp.kernel
def _run_start_k(keys: wp.array(dtype=wp.int64), start: wp.array(dtype=wp.int32)):
    j = wp.tid()
    start[j] = 0
    if j == 0 or keys[j] != keys[j - 1]:
        start[j] = 1


@wp.kernel
def _diff_legs_k(pos: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3d),
                 state: wp.array(dtype=wp.int32), Tt: wp.array(dtype=wp.vec2d),
                 frac: wp.array(dtype=wp.float64), P: wp.vec3d, offset: wp.float64,
                 cell: wp.float64, ukeys: wp.array(dtype=wp.int64),
                 umin: wp.array(dtype=wp.float64), nu: int, o: Oracle, Dc: Disc,
                 D: wp.array(dtype=wp.vec2d), cls_o: wp.array(dtype=wp.int32),
                 dfrac_o: wp.array(dtype=wp.float64), cen_o: wp.array(dtype=wp.int32)):
    """``diffract_direct``'s legs: the blocked candidates (their own hit
    fraction) and the clear, untouched candidates whose cell is in the
    table (the lit side, the cell's smallest fraction); each classified.
    ``cls_o`` 2 = not a diffraction leg."""
    i = wp.tid()
    cls_o[i] = 2
    st = int(state[i])
    if (st & 1) == 0:
        return
    fr = wp.float64(-1.0)
    Xv = pos[i] + offset * nrm[i]
    if (st & 2) == 0:
        fr = frac[i]
    elif Tt[i][0] == wp.float64(1.0) and Tt[i][1] == wp.float64(0.0) and nu > 0:
        kx = wp.int64(wp.floor(Xv[0] / cell))
        ky = wp.int64(wp.floor(Xv[1] / cell))
        q = ((kx + wp.int64(1073741824)) << wp.int64(32)) | (ky + wp.int64(1073741824))
        lo = int(0)
        hi = int(nu - 1)
        while lo < hi:
            mid = (lo + hi) // 2
            if ukeys[mid] < q:
                lo = mid + 1
            else:
                hi = mid
        if ukeys[lo] == q:
            fr = umin[lo]
    if not (fr > wp.float64(0.0)):
        return
    L = wp.length(Xv - P)
    d1 = fr * L
    dfrac_o[i] = d1
    v0 = diff_visible(o, Dc, P, Xv, diff_frame(P, Xv, d1).C)    # the disc centre, once per leg
    cen_o[i] = v0
    c = diff_classify(o, Dc, P, Xv, d1, v0)
    cls_o[i] = c
    if c == 1:
        D[i] = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
    elif c == 0:
        D[i] = wp.vec2d(wp.float64(0.0), wp.float64(0.0))


@wp.kernel
def _diff_factor_k(idx: wp.array(dtype=wp.int32), pos: wp.array(dtype=wp.vec3d),
                   nrm: wp.array(dtype=wp.vec3d), d1: wp.array(dtype=wp.float64),
                   P: wp.vec3d, offset: wp.float64, o: Oracle, Dc: Disc,
                   D: wp.array(dtype=wp.vec2d), cen: wp.array(dtype=wp.int32)):
    """The full factor for the legs with an edge in the disc (``cen``: the
    disc centre's visibility, from classify)."""
    j = wp.tid()
    i = int(idx[j])
    Xv = pos[i] + offset * nrm[i]
    D[i] = diff_factor(o, Dc, P, Xv, d1[i], int(cen[i]))


# ==================================================================== facets
@wp.kernel
def _seg_blocked_probe_k(o: Oracle, a: wp.array(dtype=wp.vec3d), b: wp.array(dtype=wp.vec3d),
                         eps: wp.float64, out: wp.array(dtype=wp.int32)):
    """``seg_blocked`` per segment (tests only: the fp32 / fp64 parity of
    ``precision``; defined here so it compiles with this module's options)."""
    i = wp.tid()
    out[i] = seg_blocked(o, a[i], b[i], eps)


# ------------------------------------------- closed-form straight-edge diffraction
@wp.struct
class Edges:
    """``diffraction.EdgeSet`` near this ping (``diffraction.method: edge``):
    the free edges, their boxes (compacted), the window and the F(v) table."""
    a: wp.array(dtype=wp.vec3d)
    b: wp.array(dtype=wp.vec3d)
    box: wp.array(dtype=wp.int32)
    n: int
    centre: wp.array(dtype=wp.vec3d)
    ax_u: wp.array(dtype=wp.vec3d)
    ax_n: wp.array(dtype=wp.vec3d)
    ax_z: wp.array(dtype=wp.vec3d)
    half: wp.array(dtype=wp.vec3d)
    nbox: int
    lam: wp.float64
    n1: wp.float64
    n2: wp.float64
    margin: wp.float64
    F_re: wp.array(dtype=wp.float64)
    F_im: wp.array(dtype=wp.float64)
    v0: wp.float64
    dv: wp.float64


@wp.func
def seg_closest_sh(P: wp.vec3d, X: wp.vec3d, a: wp.vec3d, b: wp.vec3d) -> wp.vec2d:
    """``diffraction.segments_closest`` for one pair: (leg parameter, distance)."""
    d1 = X - P
    d2 = b - a
    r = P - a
    A = wp.dot(d1, d1)
    E = wp.dot(d2, d2)
    F = wp.dot(d2, r)
    C = wp.dot(d1, r)
    B = wp.dot(d1, d2)
    den = A * E - B * B
    s = wp.float64(0.0)
    if den > wp.float64(1e-12) * A * E:
        s = wp.clamp((B * F - C * E) / den, wp.float64(0.0), wp.float64(1.0))
    t = (B * s + F) / wp.max(E, wp.float64(1e-300))
    if t < wp.float64(0.0):
        s = wp.clamp(-C / A, wp.float64(0.0), wp.float64(1.0))
    elif t > wp.float64(1.0):
        s = wp.clamp((B - C) / A, wp.float64(0.0), wp.float64(1.0))
    t = wp.clamp(t, wp.float64(0.0), wp.float64(1.0))
    return wp.vec2d(s, wp.length(P + s * d1 - (a + t * d2)))


@wp.func
def _slab(p: wp.float64, d: wp.float64, h: wp.float64) -> wp.vec2d:
    if wp.abs(d) < wp.float64(1e-300):
        if wp.abs(p) <= h:
            return wp.vec2d(wp.float64(-1e300), wp.float64(1e300))
        return wp.vec2d(wp.float64(1e300), wp.float64(-1e300))
    t1 = (-h - p) / d
    t2 = (h - p) / d
    return wp.vec2d(wp.min(t1, t2), wp.max(t1, t2))


@wp.func
def seg_crosses_box(P: wp.vec3d, X: wp.vec3d, E: Edges, k: int) -> int:
    """``diffraction.segment_crosses_box`` (the box shrunk by 1e-4 m)."""
    q = P - E.centre[k]
    d = X - P
    hv = E.half[k]
    shr = wp.float64(1e-4)
    su = _slab(wp.dot(E.ax_u[k], q), wp.dot(E.ax_u[k], d), wp.max(hv[0] - shr, wp.float64(0.0)))
    sn = _slab(wp.dot(E.ax_n[k], q), wp.dot(E.ax_n[k], d), wp.max(hv[1] - shr, wp.float64(0.0)))
    sz = _slab(wp.dot(E.ax_z[k], q), wp.dot(E.ax_z[k], d), wp.max(hv[2] - shr, wp.float64(0.0)))
    t0 = wp.max(wp.max(su[0], sn[0]), sz[0])
    t1 = wp.min(wp.min(su[1], sn[1]), sz[1])
    if t0 < t1 and t1 > wp.float64(0.0) and t0 < wp.float64(1.0):
        return 1
    return 0


@wp.kernel
def _edge_factor_k(pos: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3d),
                   state: wp.array(dtype=wp.int32), Tt: wp.array(dtype=wp.vec2d),
                   P: wp.vec3d, offset: wp.float64, E: Edges,
                   D: wp.array(dtype=wp.vec2d), cnt: wp.array(dtype=wp.int32)):
    """``EdgeDiffraction.leg_factors`` on the device: the direct leg's factor
    from its nearest free edge (a shadowed leg: of the one box it crosses)."""
    i = wp.tid()
    st = int(state[i])
    if (st & 1) == 0:
        return
    Xv = pos[i] + offset * nrm[i]
    shadow = int(0)
    if (st & 2) == 0:
        shadow = 1
    elif not (Tt[i][0] == wp.float64(1.0) and Tt[i][1] == wp.float64(0.0)):
        return
    ncross = int(0)
    own = int(-1)
    for k in range(E.nbox):
        if seg_crosses_box(P, Xv, E, k) == 1:
            ncross += 1
            own = k
    if shadow == 1 and ncross != 1:
        return
    if shadow == 0 and ncross != 0:
        return
    L = wp.length(Xv - P)
    best = wp.float64(1e300)
    for e in range(E.n):
        if shadow == 1 and int(E.box[e]) != own:
            continue
        sh = seg_closest_sh(P, Xv, E.a[e], E.b[e])
        d1 = sh[0] * L
        d2 = L - d1
        if d1 > E.margin and d2 > E.margin:
            deff = d1 * d2 / wp.max(L, wp.float64(1e-300))
            best = wp.min(best, sh[1] * sh[1] / (E.lam * wp.max(deff, wp.float64(1e-300))))
    if best >= E.n2:
        return
    v = wp.sqrt(wp.float64(2.0) * best)
    if shadow == 0:
        v = -v
    u = (v - E.v0) / E.dv
    j = wp.clamp(int(wp.floor(u)), 0, E.F_re.shape[0] - 2)
    f = u - wp.float64(j)
    Fv = wp.vec2d(E.F_re[j] + f * (E.F_re[j + 1] - E.F_re[j]),
                  E.F_im[j] + f * (E.F_im[j + 1] - E.F_im[j]))
    tw = wp.clamp((best - E.n1) / (E.n2 - E.n1), wp.float64(0.0), wp.float64(1.0))
    w = wp.float64(0.5) * (wp.float64(1.0) + wp.cos(wp.float64(3.141592653589793) * tw))
    if shadow == 1:
        D[i] = w * Fv
    else:
        D[i] = wp.vec2d(wp.float64(1.0) + w * (Fv[0] - wp.float64(1.0)), w * Fv[1])
    wp.atomic_add(cnt, 0, 1)


@wp.func
def po_pair_amp(n: wp.vec3d, pu: wp.vec3d, pv: wp.vec3d, pa: wp.float64, pb: wp.float64,
                inc: wp.vec3d, out: wp.vec3d, k: wp.float64, refl: wp.float64,
                rms: wp.float64) -> wp.vec2d:
    """``multipath.po_pair`` for one patch."""
    q = k * (inc - out)
    qu = wp.dot(pu, q)
    qv = wp.dot(pv, q)
    nd = wp.dot(n, out - inc)
    if not ((-wp.dot(n, inc) > wp.float64(0.0)) and (wp.dot(n, out) > wp.float64(0.0))):
        return wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    ar = pa * pb * sinc_x(wp.float64(0.5) * qu * pa) * sinc_x(wp.float64(0.5) * qv * pb)
    mag = k / _TWO_PI * wp.float64(0.5) * nd * ar * refl
    if rms > wp.float64(0.0):
        g = k * wp.abs(nd) * rms
        mag = mag * wp.exp(wp.float64(-0.5) * wp.min(g * g, wp.float64(1400.0)))
    return wp.vec2d(wp.float64(0.0), mag)            # i k / 2 pi ...


@wp.kernel
def _facet_direct_k(X: wp.array(dtype=wp.vec3d), P: wp.vec3d, e_a: wp.vec3d, e_n: wp.vec3d,
                    e_v: wp.vec3d, B: Beam, r_max: wp.float64, R_o: wp.array(dtype=wp.float64),
                    best_o: wp.array(dtype=wp.float64), state_o: wp.array(dtype=wp.int32)):
    """The patches' direct chain: length, ``best`` = |b| / L (selection);
    patches beyond r_max are out (state 4)."""
    i = wp.tid()
    d = X[i] - P
    R = wp.length(d)
    R_o[i] = R
    state_o[i] = 0
    best_o[i] = wp.float64(0.0)
    if R > r_max:
        state_o[i] = 4
        return
    b = pattern(B, d, e_a, e_n, e_v)
    best_o[i] = cabs(b) / wp.max(R, wp.float64(1e-9))




@wp.kernel
def _diff_report_k(cls: wp.array(dtype=wp.int32), D: wp.array(dtype=wp.vec2d),
                   edge_flag: wp.array(dtype=wp.int32), counts: wp.array(dtype=wp.int32)):
    """Device-side report and edge list: counts[0] legs, counts[1] legs with
    |D| neither 0 nor 1 (the CPU report's "edges"); edge_flag marks the legs
    needing the full factor (class -1)."""
    i = wp.tid()
    c = int(cls[i])
    edge_flag[i] = 0
    if c == 2:
        return
    wp.atomic_add(counts, 0, 1)
    if c == -1:
        edge_flag[i] = 1


@wp.kernel
def _edge_count_k(cls: wp.array(dtype=wp.int32), D: wp.array(dtype=wp.vec2d),
                  counts: wp.array(dtype=wp.int32)):
    i = wp.tid()
    if int(cls[i]) == 2:
        return
    a = cabs(D[i])
    if a > wp.float64(0.0) and a != wp.float64(1.0):
        wp.atomic_add(counts, 1, 1)


@wp.kernel
def _compact_k(flag: wp.array(dtype=wp.int32), scan: wp.array(dtype=wp.int32),
               out: wp.array(dtype=wp.int32)):
    i = wp.tid()
    if int(flag[i]) == 1:
        out[int(scan[i]) - 1] = i


@wp.kernel
def _diffuse_report_k(state: wp.array(dtype=wp.int32), npairs: wp.array(dtype=wp.int32),
                      counts: wp.array(dtype=wp.int32)):
    """counts[0] contributors in range (state bit 2 clear), counts[1] pairs."""
    i = wp.tid()
    if (int(state[i]) & 4) == 0:
        wp.atomic_add(counts, 0, 1)
    wp.atomic_add(counts, 1, int(npairs[i]))


@wp.kernel
def _init_k(state: wp.array(dtype=wp.int32), R: wp.array(dtype=wp.float64),
            b: wp.array(dtype=wp.vec2d), kind: wp.array(dtype=wp.int32),
            tri: wp.array(dtype=wp.int32), wall_index: wp.array(dtype=wp.int32),
            k_seabed: int, k_rough: int, k_apron: int, k_wall: int,
            D: wp.array(dtype=wp.vec2d), best: wp.array(dtype=wp.float64),
            onfl: wp.array(dtype=wp.int32), wallof: wp.array(dtype=wp.int32)):
    """D (1 visible, 0 else), best = |b| / L (the direct chain), and the
    contributor's own-surface flags."""
    i = wp.tid()
    st = int(state[i])
    D[i] = wp.vec2d(wp.float64(0.0), wp.float64(0.0))
    if (st & 2) != 0:
        D[i] = wp.vec2d(wp.float64(1.0), wp.float64(0.0))
    best[i] = cabs(b[i]) / wp.max(R[i], wp.float64(1e-9))
    kd = int(kind[i])
    onfl[i] = 0
    if kd == k_seabed or kd == k_rough or kd == k_apron:
        onfl[i] = 1
    wallof[i] = -1
    if kd == k_wall and int(tri[i]) >= 0:
        wallof[i] = int(wall_index[int(tri[i])])


class GpuMultipath:
    """``Multipath.diffuse`` on the device (the CPU ``Multipath`` supplies
    the boundaries, the sequence enumeration and the selection rule; the
    per-contributor work runs here)."""

    NCOPY = 64

    def __init__(self, mp) -> None:
        from .scene_mesh import (KIND_APRON as KA, KIND_ROUGH_OBJECT as KR,
                                 KIND_SEABED as KS, KIND_WALL as KW)
        self.mp = mp
        rr = mp.rr
        self.rr = rr
        if getattr(rr, "_gpu", None) is None:
            rr._gpu = GpuScene(rr)
        self.gs = rr._gpu
        dev = self.gs.device
        self.dev = dev
        self.kinds_const = (int(KS), int(KR), int(KA), int(KW))
        self.wall_index = wp.array(rr.mesh.wall_index.astype(np.int32), dtype=wp.int32, device=dev)
        self.beam = self._beam()
        self.law = self._law()
        self.oracle = self._oracle()
        self.spec = self._spec()
        self.faces = self._faces(list(mp.faces) + list(mp.object_faces))
        self.layers = self._layers()
        self.alpha_np = float(rr.alpha) * float(np.log(10.0) / 20.0)
        self.lam = float(mp.lam)
        #: diffraction's visibility samples in float32 (False: the float64
        #: reference test; the G-gates compare the two)
        self.diffraction_fp32 = True
        self.max_iter = int(mp.floor.max_iter) if mp.floor is not None else 20
        #: paths.reselect_every: the last selection per (side, stage)
        self._sel_cache: dict = {}
        #: stage timing (``last_timing``) synchronises the device at every
        #: stage boundary; False (the live node) lets the host run ahead of the
        #: GPU. No value depends on it: GPU work stays in stream order.
        self.timing = True
        self._env_key = None                # the envelope ``_env_s`` was uploaded for
        self._env_s = None

    # ------------------------------------------------------------ tables
    def _beam(self) -> Beam:
        rr, dev = self.rr, self.dev
        ap = rr.aperture
        sizes = [32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 448, 512,
                 640, 768, 896, 1024, 1280, 1536, 1792, 2048, 2560, 3072, 3584, 4096]
        us, ws, offs = [], [], []
        o = 0
        for n in sizes:
            x, w = np.polynomial.legendre.leggauss(n)
            us.append(0.5 * ap.length_m * x)
            ws.append(w)
            offs.append(o)
            o += n
        B = Beam()
        B.L, B.H, B.k = wp.float64(ap.length_m), wp.float64(ap.height_m), wp.float64(ap.k)
        B.vfloor = wp.float64(ap.vertical_floor)
        B.near = 1 if rr._rc.beam.near_field else 0
        B.gl_n = wp.array(np.array(sizes, np.int32), dtype=wp.int32, device=dev)
        B.gl_off = wp.array(np.array(offs, np.int32), dtype=wp.int32, device=dev)
        B.gl_u = wp.array(np.concatenate(us).astype(np.float32), dtype=wp.float32, device=dev)
        B.gl_w = wp.array(np.concatenate(ws).astype(np.float32), dtype=wp.float32, device=dev)
        return B

    def _law(self) -> Law:
        rr, dev = self.rr, self.dev
        dl = rr.law
        L = Law()
        L.k = wp.float64(rr.k)
        L.lambert = 1 if dl.cfg.law == "lambert" else 0
        L.mu = wp.float64(dl._mu)
        laws = dl._laws or []
        f64 = lambda v: wp.array(np.asarray(v, np.float64), dtype=wp.float64, device=dev)  # noqa: E731
        i32 = lambda v: wp.array(np.asarray(v, np.int32), dtype=wp.int32, device=dev)      # noqa: E731
        if not laws:                       # lambert: placeholders
            laws_p = []
        else:
            laws_p = laws
        L.a_rho = f64([lw.p.a_rho for lw in laws_p] or [1.0])
        L.ap = wp.array(np.array([[lw.p.a_p.real, lw.p.a_p.imag] for lw in laws_p] or [[1.0, 0.0]]),
                        dtype=wp.vec2d, device=dev)
        L.ch2 = f64([lw.p.ch2 for lw in laws_p] or [0.0])
        L.alpha = f64([lw.p.alpha for lw in laws_p] or [0.5])
        L.s_v = f64([lw.s_v for lw in laws_p] or [0.0])
        offs, ns, lq0, dlq, slope, beta, lv = [], [], [], [], [], [], []
        o = 0
        for lw in laws_p:
            t = lw.table
            offs.append(o)
            ns.append(t.lq.size)
            lq0.append(t.lq[0])
            dlq.append(t.lq[1] - t.lq[0])
            slope.append(t._lo_slope)
            beta.append(t.beta)
            lv.append(t.lv)
            o += t.lq.size
        L.ik_off = i32(offs or [0])
        L.ik_n = i32(ns or [2])
        L.ik_lq0 = f64(lq0 or [0.0])
        L.ik_dlq = f64(dlq or [1.0])
        L.ik_slope = f64(slope or [0.0])
        L.ik_beta = f64(beta or [1.0])
        L.ik_lv = f64(np.concatenate(lv) if lv else [0.0, 0.0])
        L.law_of = i32(dl._law_of)
        L.scale = f64(dl._scale)
        L.kr2 = f64(dl._kr2)
        L.keep = f64(dl._keep)
        L.cov = f64(dl._cov)
        return L

    def _oracle(self) -> Oracle:
        bvh = self.rr.oracle.bvh
        if not hasattr(bvh, "_V"):
            raise TypeError("the GPU pipeline needs the GPU oracle (backend: warp)")
        o = Oracle()
        o.bvh = bvh.bvh.id
        o.V = bvh._V
        o.V32 = bvh._V32
        o.T = bvh._T
        o.origin = wp.vec3d(*bvh.origin)
        o.zs = wp.float64(self.rr.mesh.surface_z)
        o.use32 = int(self.rr._rc.precision == "fp32")
        return o

    def _layers(self) -> Layers:
        rr, dev = self.rr, self.dev
        lay = rr.oracle.layer
        Ly = Layers()
        nt = rr.mesh.n_triangles
        Ly.on = 0 if lay is None else 1
        f64 = lambda v: wp.array(np.asarray(v, np.float64), dtype=wp.float64, device=dev)  # noqa: E731
        Ly.mask = wp.array((np.zeros(nt, np.int32) if lay is None else lay["mask"].astype(np.int32)),
                           dtype=wp.int32, device=dev)
        Ly.rho = f64(np.ones(nt) if lay is None else lay["rho"])
        Ly.cr = f64(np.ones(nt) if lay is None else lay["c"])
        Ly.d = f64(np.zeros(nt) if lay is None else lay["d"])
        Ly.k1 = wp.float64(0.0 if lay is None else lay["k1"])
        Ly.normal = wp.array(np.ascontiguousarray(rr.mesh.normals(), np.float64), dtype=wp.vec3d,
                             device=dev)
        return Ly

    def _spec(self) -> SpecSheet:
        sp, dev, m = self.mp.sheet, self.dev, self.rr.mesh
        g = sp.g
        S = SpecSheet()
        S.z = wp.array(np.ascontiguousarray(sp.z, np.float64), dtype=wp.float64, device=dev)
        S.vn = wp.array(np.ascontiguousarray(sp.vnormal, np.float64), dtype=wp.vec3d, device=dev)
        S.nx, S.ny = int(g.nx), int(g.ny)
        S.ox, S.oy, S.res = wp.float64(g.origin_x), wp.float64(g.origin_y), wp.float64(g.resolution)
        S.zmin = wp.float64(sp.zmin)
        S.nq = int(sp.nq)
        S.edge = 1 if sp.outside == "edge" else 0
        S.tri_mat = wp.array(m.material.astype(np.int32), dtype=wp.int32, device=dev)
        S.tri_dry = wp.array(np.asarray(m.dry, bool).astype(np.int32), dtype=wp.int32, device=dev)
        S.out_mat = int(sp.out_mat)
        S.lam = wp.float64(sp.lam)
        S.dx = wp.float64(sp.dx)
        surf = sp.surf
        mode, arho, ap, w2, g2, rms = [], [], [], [], [], []
        for i in range(len(surf.names)):
            p = surf.sediment[i]
            if p is not None:
                mode.append(0); arho.append(p.a_rho); ap.append([p.a_p.real, p.a_p.imag])
                w2.append(p.w2); g2.append(p.gamma2); rms.append(0.0)
            else:
                a_rho, nu = surf.half_space[i]
                a = complex(nu / (1.0 + 0j))
                mode.append(1); arho.append(a_rho); ap.append([a.real, a.imag])
                w2.append(0.0); g2.append(3.0); rms.append(surf.rms[i])
        S.s_mode = wp.array(np.array(mode, np.int32), dtype=wp.int32, device=dev)
        S.s_arho = wp.array(np.array(arho, np.float64), dtype=wp.float64, device=dev)
        S.s_ap = wp.array(np.array(ap, np.float64), dtype=wp.vec2d, device=dev)
        S.s_w2 = wp.array(np.array(w2, np.float64), dtype=wp.float64, device=dev)
        S.s_g2 = wp.array(np.array(g2, np.float64), dtype=wp.float64, device=dev)
        S.s_rms = wp.array(np.array(rms, np.float64), dtype=wp.float64, device=dev)
        return S

    def _faces(self, faces: list) -> Faces:
        """Device tables of ``faces`` (FaceBoundary); index = position."""
        dev, surf = self.dev, self.mp.sheet.surf
        F = Faces()
        n = max(len(faces), 1)
        c = np.zeros((n, 3)); nn = np.zeros((n, 3)); uu = np.zeros((n, 3)); vv = np.zeros((n, 3))
        hu = np.zeros(n); hv = np.zeros(n); npoly = np.zeros(n, np.int32); poff = np.zeros(n, np.int32)
        mode = np.zeros(n, np.int32); arho = np.ones(n); ap = np.zeros((n, 2)); ap[:, 0] = 1.0
        rconst = np.zeros(n); rms = np.zeros(n); wall = np.full(n, -1, np.int32)
        polys = []
        o = 0
        owner = dict(zip(map(id, self.mp.faces), self.mp.face_wall))
        for i, f in enumerate(faces):
            c[i], nn[i], uu[i], vv[i] = f.c, f.n, f.u, f.v
            hu[i] = f.hu if np.isfinite(f.hu) else 1e300
            hv[i] = f.hv if np.isfinite(f.hv) else 1e300
            if f.poly is not None:
                npoly[i] = f.poly.shape[0]
                poff[i] = o
                polys.append(f.poly)
                o += f.poly.shape[0]
            rms[i] = f.rms
            if getattr(f, "rconst", None) is not None:
                mode[i] = 1
                rconst[i] = float(np.real(f.rconst))
            else:
                mi = int(f.material)
                p = surf.sediment[mi]
                if p is not None:
                    arho[i], ap[i] = p.a_rho, [p.a_p.real, p.a_p.imag]
                else:
                    a_rho, nu = surf.half_space[mi]
                    arho[i], ap[i] = a_rho, [nu, 0.0]
            wall[i] = owner.get(id(f), -1)
        F.c = wp.array(c, dtype=wp.vec3d, device=dev)
        F.n = wp.array(nn, dtype=wp.vec3d, device=dev)
        F.u = wp.array(uu, dtype=wp.vec3d, device=dev)
        F.v = wp.array(vv, dtype=wp.vec3d, device=dev)
        F.hu = wp.array(hu, dtype=wp.float64, device=dev)
        F.hv = wp.array(hv, dtype=wp.float64, device=dev)
        F.npoly = wp.array(npoly, dtype=wp.int32, device=dev)
        F.poly_off = wp.array(poff, dtype=wp.int32, device=dev)
        F.poly = wp.array(np.concatenate(polys) if polys else np.zeros((1, 2)), dtype=wp.vec2d, device=dev)
        F.mode = wp.array(mode, dtype=wp.int32, device=dev)
        F.arho = wp.array(arho, dtype=wp.float64, device=dev)
        F.ap = wp.array(ap, dtype=wp.vec2d, device=dev)
        F.rconst = wp.array(rconst, dtype=wp.float64, device=dev)
        F.rms = wp.array(rms, dtype=wp.float64, device=dev)
        F.wall = wp.array(wall, dtype=wp.int32, device=dev)
        self._face_index = {id(f): i for i, f in enumerate(faces)}
        return F

    def _rx(self, grid) -> Rx:
        env = grid.env
        x = Rx()
        x.n = int(grid.n)
        x.start_m = wp.float64(grid.start_m)
        x.bin_m = wp.float64(grid.bin_m)
        x.dev_c = wp.float64(grid.device_c)
        x.omega = wp.float64(grid.omega)
        x.kk = int(grid._k)
        x.env_t0 = wp.float64(env._t[0])
        x.env_dt = wp.float64(env._t[1] - env._t[0])
        if self._env_key is not env:        # constant per renderer: upload once
            self._env_s = wp.array(env._s.astype(np.float64), dtype=wp.float64, device=self.dev)
            self._env_key = env
        x.env_s = self._env_s
        x.ncopy = self.NCOPY
        x.field = wp.zeros((self.NCOPY, x.n), dtype=wp.vec2d, device=self.dev)
        x.mean = wp.zeros((self.NCOPY, x.n), dtype=wp.float64, device=self.dev)
        x.coh = wp.zeros((self.NCOPY, x.n), dtype=wp.vec2d, device=self.dev)
        return x

    def _seq_arrays(self, seqs: list):
        kinds, faces, off = [], [], [0]
        for seq in seqs:
            for bd in seq:
                if bd.kind == "S":
                    kinds.append(0); faces.append(-1)
                elif bd.kind == "F":
                    kinds.append(1); faces.append(-1)
                else:
                    kinds.append(2 if bd.kind == "W" else 3)
                    faces.append(self._face_index[id(bd)])
            off.append(len(kinds))
        if not kinds:
            kinds, faces = [0], [-1]
        dev = self.dev
        return (wp.array(np.array(kinds, np.int32), dtype=wp.int32, device=dev),
                wp.array(np.array(faces, np.int32), dtype=wp.int32, device=dev),
                wp.array(np.array(off, np.int32), dtype=wp.int32, device=dev))

    def _sum_bins(self, arr2d) -> np.ndarray:
        out = wp.empty(arr2d.shape[1], dtype=wp.float64, device=self.dev)
        wp.launch(_reduce_k, arr2d.shape[1], [arr2d, out], device=self.dev)
        return out.numpy()

    # ------------------------------------------------------------ diffuse
    def _tick(self, name: str) -> None:
        """Stage wall time (``last_timing``, seconds; synchronised). Nothing
        (no sync) when ``timing`` is off."""
        if not self.timing:
            return
        import time
        wp.synchronize_device(self.dev)
        t = time.perf_counter()
        self.last_timing[name] = self.last_timing.get(name, 0.0) + t - self._t
        self._t = t

    # ------------------------------------------------- selection reuse
    def _cached_selection(self, key, tag, P, e_a, extra=None):
        """``paths.reselect_every`` (fast variant): ``(kept, report)`` of a
        side's earlier selection still valid here, or None, plus the record
        to store when a fresh selection is made (None when reuse is off).
        Valid while fewer than ``reselect_every`` pings used it, the sensor
        moved < ``reselect_move_m`` and turned < 2 deg since, and the set of
        boundaries in range is the same. ``key`` None (or reselect_every 1):
        no reuse, and nothing is stored."""
        pc = self.mp.pc
        n_max = int(pc.reselect_every)
        if key is None or n_max <= 1:
            return None, None
        bnds = tuple(id(b) for b in self.mp.boundaries(P) + list(extra or []))
        yaw = float(np.arctan2(e_a[1], e_a[0]))
        ent = self._sel_cache.get((key, tag))
        if ent is not None:
            dyaw = abs((yaw - ent["yaw"] + np.pi) % (2.0 * np.pi) - np.pi)
            if (ent["bnds"] == bnds and ent["uses"] < n_max
                    and float(np.linalg.norm(np.asarray(P, float) - ent["P"])) < float(pc.reselect_move_m)
                    and dyaw < np.radians(_RESELECT_YAW_DEG)):
                ent["uses"] += 1
                return (ent["kept"], dict(ent["rep"], reused=True)), None
        return None, dict(bnds=bnds, P=np.array(P, float), yaw=yaw, uses=1)

    def _store_selection(self, key, tag, record, kept, rep) -> None:
        if record is not None:
            self._sel_cache[(key, tag)] = dict(record, kept=list(kept), rep=dict(rep))

    def diffuse(self, P, e_a, e_n, e_v, grid, key=None) -> dict:
        """The diffuse field (direct + multipath pairs) of one ping-side.
        ``key`` (the side) enables the selection reuse of ``paths.reselect_every``."""
        import time
        mp, rr, dev = self.mp, self.rr, self.dev
        self.last_timing = {}
        if self.timing:
            wp.synchronize_device(dev)
        self._t = time.perf_counter()
        gsc = self.gs.scatterers(P, e_a, e_n)
        n = len(gsc)
        self._tick("scatterers")
        if n == 0:
            return {"contributors": 0}
        Pv, ea, en, ev = (wp.vec3d(*np.asarray(v, float)) for v in (P, e_a, e_n, e_v))
        R = wp.empty(n, dtype=wp.float64, device=dev)
        b = wp.empty(n, dtype=wp.vec2d, device=dev)
        amp0 = wp.empty(n, dtype=wp.float64, device=dev)
        state = wp.empty(n, dtype=wp.int32, device=dev)
        Tt = wp.empty(n, dtype=wp.vec2d, device=dev)
        offset = float(rr._rc.sampling.surface_offset_m)
        eps = 1e-6
        wp.launch(_direct_k, n, [gsc.pos, gsc.normal, gsc.area, gsc.material, Pv, ea, en, ev,
                                 self.beam, self.law, self.oracle, wp.float64(rr.r_max),
                                 wp.float64(rr._floor), wp.float64(offset), wp.float64(eps),
                                 self.layers, R, b, amp0, state, Tt], device=dev)
        self._tick("direct")
        D = wp.empty(n, dtype=wp.vec2d, device=dev)
        best = wp.empty(n, dtype=wp.float64, device=dev)
        onfl = wp.empty(n, dtype=wp.int32, device=dev)
        wallof = wp.empty(n, dtype=wp.int32, device=dev)
        ks, kr, ka, kw = self.kinds_const
        wp.launch(_init_k, n, [state, R, b, gsc.kind, gsc.tri, self.wall_index, ks, kr, ka, kw,
                               D, best, onfl, wallof], device=dev)
        self._tick("direct")
        drep = self._diffraction(P, gsc, state, D, Tt)
        self._tick("diffraction")
        x = self._rx(grid)
        reuse, record = self._cached_selection(key, "diffuse", P, e_a)
        if reuse is not None:           # level only feeds the selection
            kept, rep = reuse
        else:
            level2 = wp.zeros((self.NCOPY, x.n), dtype=wp.float64, device=dev)
            wp.launch(_level_k, n, [R, b, amp0, D, wp.float64(self.alpha_np), wp.float64(rr.c),
                                    x, level2], device=dev)
            level = self._sum_bins(level2)
            self._tick("level")
            kept, rep = self._select(P, Pv, ea, en, ev, gsc, R, best, amp0, state, onfl, wallof,
                                     x, level, grid)
            self._store_selection(key, "diffuse", record, kept, rep)
        self._tick("select")
        kinds, faces, koff = self._seq_arrays(kept)
        npairs = wp.empty(n, dtype=wp.int32, device=dev)
        wp.launch(pair_kernel("pairs", len(kept) + 1), n, [gsc.pos, gsc.normal, gsc.area, gsc.material, gsc.phase,
                                gsc.rayleigh, onfl, wallof, R, b, state, D, Tt, self.layers,
                                Pv, ea, en, ev,
                                self.beam, self.law, self.oracle, self.spec, self.faces,
                                kinds, faces, koff, len(kept), wp.float64(rr.k),
                                wp.float64(mp.surface.rms), wp.float64(self.alpha_np),
                                wp.float64(rr.c), wp.float64(rr._floor), wp.float64(offset),
                                wp.float64(eps), wp.float64(self.lam), self.max_iter, x, npairs],
                  device=dev)
        fo = wp.empty(x.n, dtype=wp.vec2d, device=dev)
        wp.launch(_reduce_c_k, x.n, [x.field, fo], device=dev)
        fn = fo.numpy()
        grid.field += fn[:, 0] + 1j * fn[:, 1]
        grid.mean_diffuse += self._sum_bins(x.mean)
        self._tick("pairs")
        cnt = wp.zeros(2, dtype=wp.int32, device=dev)
        wp.launch(_diffuse_report_k, n, [state, npairs, cnt], device=dev)
        cn = cnt.numpy()
        rep.update(contributors=int(cn[0]), pairs=int(cn[1]), **drep)
        mp.last_report["diffuse"] = rep
        return rep

    # -------------------------------------------------------------- facets
    def _patch_arrays(self, i: int):
        """Device copies of smooth solid i's PO patches (static, cached),
        with its amplitude scale and the per-ping work arrays."""
        if not hasattr(self, "_patch_cache"):
            self._patch_cache = {}
        g = self._patch_cache.get(i)
        if g is None:
            p = self.rr._patches(i)
            if p is None or len(p) == 0:
                self._patch_cache[i] = g = ()
            else:
                v3 = lambda a: wp.array(np.ascontiguousarray(a, np.float64), dtype=wp.vec3d, device=self.dev)  # noqa: E731
                f1 = lambda a: wp.array(np.ascontiguousarray(a, np.float64), dtype=wp.float64, device=self.dev)  # noqa: E731
                n = len(p)
                amp0 = f1((self.rr.k / (2.0 * np.pi)) * self._pab(i) * float(self.rr._facet_R[i]))
                work = tuple(wp.empty(n, dtype=t, device=self.dev)
                             for t in (wp.float64, wp.float64, wp.int32, wp.int32))
                g = (v3(p.centre), v3(p.normal), v3(p.u), v3(p.v), f1(p.a), f1(p.b), n,
                     np.asarray(p.centre), amp0) + work
                self._patch_cache[i] = g
        return g

    def facets(self, P, e_a, e_n, e_v, grid, level: np.ndarray, key=None) -> dict:
        """``Multipath.facets`` on the device (``key``: the selection reuse,
        per side and object, as in :meth:`diffuse`)."""
        from . import chains as ch
        mp, rr, dev = self.mp, self.rr, self.dev
        k = rr.k
        Pv, ea, en, ev = (wp.vec3d(*np.asarray(v, float)) for v in (P, e_a, e_n, e_v))
        offset = float(rr._rc.sampling.surface_offset_m)
        eps = 1e-6
        x = self._rx(grid)
        n_dep = 0
        reps = []
        for i, o in enumerate(rr._facet_objs):
            c0 = np.array([o.x, o.y, 0.0])
            reach = float(np.hypot(o.length, o.width)) + o.proud_height
            if np.hypot(*(c0[:2] - P[:2])) - reach > rr.r_max:
                continue
            g = self._patch_arrays(i)
            if not g:
                continue
            Xp, Np, Up, Vp, Ap, Bp, n, Xh, amp0, R, best, state, ndep = g
            if not np.any(np.linalg.norm(Xh - P, axis=1) <= rr.r_max):
                continue
            refl = float(rr._facet_R[i])
            rms = float(rr._facet_sigma[i])
            wp.launch(_facet_direct_k, n, [Xp, Pv, ea, en, ev, self.beam, wp.float64(rr.r_max),
                                           R, best, state], device=dev)
            oid = int(o.object_id)
            extra = [f for f in mp.object_faces if f.owner != oid
                     and np.linalg.norm(f.c - P) <= rr.r_max]
            reuse, record = self._cached_selection(key, ("facet", i), P, e_a, extra)
            if reuse is not None:
                kept, rep = reuse
            else:
                zeros = wp.zeros(n, dtype=wp.int32, device=dev)
                minus = wp.full(n, -1, dtype=wp.int32, device=dev)
                kept, rep = self._select_set(P, Pv, ea, en, ev, Xp, n, R, best, amp0, state, zeros,
                                             minus, x, level, grid, extra,
                                             int(mp.pc.max_object_bounces))
                self._store_selection(key, ("facet", i), record, kept, rep)
            reps.append(rep)
            kinds, faces, koff = self._seq_arrays(kept)
            is_out = wp.array(np.array([int(all(b_.kind != "O" for b_ in q)) for q in kept] or [1],
                                       np.int32), dtype=wp.int32, device=dev)
            wp.launch(pair_kernel("facets", len(kept) + 1), n, [Xp, Np, Up, Vp, Ap, Bp, state, Pv, ea, en, ev, self.beam,
                                          self.oracle, self.layers, self.spec, self.faces, kinds,
                                          faces, koff, is_out, len(kept), wp.float64(refl),
                                          wp.float64(rms), wp.float64(k), wp.float64(mp.surface.rms),
                                          wp.float64(self.alpha_np), wp.float64(rr.c),
                                          wp.float64(rr._floor), wp.float64(offset), wp.float64(eps),
                                          wp.float64(self.lam), self.max_iter, x, ndep], device=dev)
            if self.timing:                 # a count for the reports: a device sync
                n_dep += int(ndep.numpy().sum())
        fo = wp.empty(x.n, dtype=wp.vec2d, device=dev)
        wp.launch(_reduce_c_k, x.n, [x.field, fo], device=dev)
        fn = fo.numpy()
        grid.field += fn[:, 0] + 1j * fn[:, 1]
        wp.launch(_reduce_c_k, x.n, [x.coh, fo], device=dev)
        cn = fo.numpy()
        grid.coherent += cn[:, 0] + 1j * cn[:, 1]
        mp.last_report["facets"] = reps
        return {"deposits": n_dep}

    # ------------------------------------------------- floor coherent
    def _sheetwin(self) -> SheetWin:
        sp = self.mp.sheet
        W = SheetWin()
        W.tri_n = wp.array(np.ascontiguousarray(sp.tri_normal, np.float64), dtype=wp.vec3d,
                           device=self.dev)
        W.n1, W.n2 = wp.float64(sp.n1), wp.float64(sp.n2)
        W.eps = wp.float64(sp.eps)
        W.a_min, W.a_max = wp.float64(sp.a_min), wp.float64(sp.a_max)
        W.k = wp.float64(sp.k)
        return W

    def _fc_patches(self, A, B, bbox):
        """``SheetSpecular.patches(A, B, bbox)`` on the device: (count, arrays)."""
        sp, dev = self.mp.sheet, self.dev
        g = sp.g
        i0 = int(np.floor((bbox[0] - g.origin_x) / sp.dx))
        i1 = int(np.ceil((bbox[2] - g.origin_x) / sp.dx))
        j0 = int(np.floor((bbox[1] - g.origin_y) / sp.dx))
        j1 = int(np.ceil((bbox[3] - g.origin_y) / sp.dx))
        nxb, nyb = i1 - i0 + 1, j1 - j0 + 1
        if nxb < 2 or nyb < 2:
            return 0, None
        W = self._sw
        inside = wp.empty(nxb * nyb, dtype=wp.int32, device=dev)
        Rev = wp.empty(nxb * nyb, dtype=wp.float64, device=dev)
        wp.launch(_fc_vert_k, nxb * nyb, [self.spec, W, i0, j0, nxb, wp.vec3d(*A), wp.vec3d(*B),
                                          inside, Rev], device=dev)
        ncx, ncy = nxb - 1, nyb - 1
        nc = ncx * ncy
        c = wp.empty(nc, dtype=wp.int32, device=dev)
        wp.launch(_fc_cell_k, nc, [inside, nxb, ncx, c], device=dev)
        d1 = wp.empty(nc, dtype=wp.int32, device=dev)
        wp.launch(_fc_drow_k, nc, [c, ncx, ncy, d1], device=dev)
        d = wp.empty(nc, dtype=wp.int32, device=dev)
        wp.launch(_fc_dcol_k, nc, [d1, ncx, d], device=dev)
        dscan = wp.empty(nc, dtype=wp.int32, device=dev)
        wp.utils.array_scan(d, dscan, inclusive=True)
        ncell = _last(dscan)
        if ncell == 0:
            return 0, None
        cell = wp.empty(ncell, dtype=wp.vec2i, device=dev)
        nsub = wp.empty(ncell, dtype=wp.int32, device=dev)
        reps = wp.empty(ncell, dtype=wp.int32, device=dev)
        wp.launch(_fc_count_k, nc, [d, dscan, Rev, nxb, ncx, W, wp.float64(sp.dx), cell, nsub, reps],
                  device=dev)
        rscan = wp.empty(ncell, dtype=wp.int32, device=dev)
        wp.utils.array_scan(reps, rscan, inclusive=True)
        n = _last(rscan)
        arr = dict(Xc=wp.empty(n, dtype=wp.vec3d, device=dev), Nn=wp.empty(n, dtype=wp.vec3d, device=dev),
                   Sm=wp.empty(n, dtype=wp.vec3d, device=dev), U=wp.empty(n, dtype=wp.vec3d, device=dev),
                   V=wp.empty(n, dtype=wp.vec3d, device=dev), Ar=wp.empty(n, dtype=wp.float64, device=dev),
                   Mt=wp.empty(n, dtype=wp.int32, device=dev), Kp=wp.empty(n, dtype=wp.int32, device=dev))
        wp.launch(_fc_patch_k, ncell, [cell, nsub, rscan, i0, j0, self.spec, W, arr["Xc"], arr["Nn"],
                                       arr["Sm"], arr["U"], arr["V"], arr["Ar"], arr["Mt"], arr["Kp"]],
                  device=dev)
        return n, arr

    def floor_coherent(self, P, e_a, e_n, e_v, grid, level, key=None) -> dict:
        """``Multipath.floor_coherent`` on the device: the same sequences,
        bound rule and order loop; the patches, amplitudes, visibility and
        deposits per patch on the GPU. ``key`` (the side) reuses which
        image pairs were kept, as ``paths.reselect_every`` does for the
        diffuse selection (fast variant): a reused ping evaluates only those
        pairs, exactly, and skips the dropped ones and the bound test."""
        from . import chains as ch
        mp, rr, dev = self.mp, self.rr, self.dev
        sp = mp.sheet
        if getattr(self, "_sw", None) is None:
            self._sw = self._sheetwin()
        bnds = [mp.surface] + ([mp.floor] if mp.floor is not None else [])
        seqs = ch.sequences(bnds, int(mp.pc.max_order), 0)
        outs = [q for q in seqs if all(b is mp.surface for b in q) and len(q) <= 1]
        backs = [q for q in seqs if not (q and q[-1] is mp.floor)]
        bbox = rr._band_bbox(P, e_a, e_n)
        thr = mp._threshold(grid, level)
        z0, _, _ = sp._locate(np.array([P[0]]), np.array([P[1]]))
        X0 = np.array([[P[0], P[1], float(z0[0])]])
        Pv, ea, en, ev = (wp.vec3d(*np.asarray(v, float)) for v in (P, e_a, e_n, e_v))
        x = self._rx(grid)
        offset = wp.float64(rr._rc.sampling.surface_offset_m)
        count = wp.zeros(1, dtype=wp.int32, device=dev)
        pairs, top = 0, 0
        reuse, record = self._cached_selection(key, "floor", P, e_a)
        allowed = None if reuse is None else set(reuse[0])
        kept_pairs = []
        for order in range(0, int(mp.pc.max_order) + 1):
            kept_any = False
            for i_so, so in enumerate(outs):
                for i_sb, sb in enumerate(backs):
                    if len(so) + len(sb) != order:
                        continue
                    if allowed is not None and (order, i_so, i_sb) not in allowed:
                        continue
                    co = ch.build(P, X0, so, None, mp.lam)
                    cb = ch.build(P, X0, sb, None, mp.lam)
                    if not (np.abs(co.factor[0]) > 0 and np.abs(cb.factor[0]) > 0):
                        continue
                    A_img = X0[0] - co.arrive[0] * co.length[0]
                    B_img = X0[0] - cb.arrive[0] * cb.length[0]
                    n, pa = self._fc_patches(A_img, B_img, bbox)
                    if n == 0:
                        continue
                    kinds, faces, _ = self._seq_arrays([so, sb])
                    a_d = wp.empty(n, dtype=wp.vec2d, device=dev)
                    tau_d = wp.empty(n, dtype=wp.float64, device=dev)
                    live_d = wp.empty(n, dtype=wp.int32, device=dev)
                    common = [kinds, faces, len(so), len(so), len(sb), self.spec, self.faces]
                    wp.launch(_fc_amp_k, n, [pa["Xc"], pa["Nn"], pa["Sm"], pa["U"], pa["V"], pa["Ar"],
                                             pa["Mt"], pa["Kp"], Pv] + common
                              + [self._sw, wp.float64(rr.k), wp.float64(mp.surface.rms),
                                 wp.float64(self.lam), self.max_iter, self.beam, ea, en, ev,
                                 wp.float64(self.alpha_np), wp.float64(rr.c), wp.float64(rr.r_max),
                                 a_d, tau_d, live_d], device=dev)
                    live = live_d.numpy().astype(bool)
                    if not live.any():
                        continue
                    a = a_d.numpy()[live]
                    tau = tau_d.numpy()[live]
                    if allowed is None:
                        est = abs((a[:, 0] + 1j * a[:, 1]).sum()) ** 2
                        b_ = int(mp._bins(grid, np.array([np.median(tau)]))[0])
                        if order > 0 and est < thr[b_]:
                            continue
                    wp.launch(_fc_dep_k, n, [pa["Xc"], pa["Nn"], live_d, a_d, tau_d, Pv] + common[:5]
                              + [self.spec, self.faces, wp.float64(rr.k), wp.float64(mp.surface.rms),
                                 wp.float64(self.lam), self.max_iter, self.oracle, self.layers,
                                 offset, wp.float64(1e-6), x, count], device=dev)
                    pairs += 1
                    kept_any = True
                    kept_pairs.append((order, i_so, i_sb))
            if order > 0 and not kept_any and allowed is None:
                break
            top = order
        fo = wp.empty(x.n, dtype=wp.vec2d, device=dev)
        wp.launch(_reduce_c_k, x.n, [x.field, fo], device=dev)
        fn = fo.numpy()
        grid.field += fn[:, 0] + 1j * fn[:, 1]
        wp.launch(_reduce_c_k, x.n, [x.coh, fo], device=dev)
        cn = fo.numpy()
        grid.coherent += cn[:, 0] + 1j * cn[:, 1]
        rep = dict(pairs=pairs, deposits=int(count.numpy()[0]), order_reached=top,
                   reused=allowed is not None)
        self._store_selection(key, "floor", record, kept_pairs, rep)
        mp.last_report["floor"] = rep
        return rep

    def _pab(self, i: int) -> np.ndarray:
        p = self.rr._patches(i)
        return np.asarray(p.a, np.float64) * np.asarray(p.b, np.float64)

    # ---------------------------------------------------------- selection
    def _select(self, P, Pv, ea, en, ev, gsc, R, best, amp0, state, onfl, wallof, x, level, grid):
        return self._select_set(P, Pv, ea, en, ev, gsc.pos, len(gsc), R, best, amp0, state, onfl,
                                wallof, x, level, grid, None, 0)

    def _select_set(self, P, Pv, ea, en, ev, pos, n, R, best, amp0, state, onfl, wallof, x, level,
                    grid, extra, max_obj):
        """``Multipath.select_chains``: the same enumeration and rule, the
        bound summed on the device."""
        from . import chains as ch
        mp, rr, dev = self.mp, self.rr, self.dev
        bnds = mp.boundaries(P) + list(extra or [])
        levels = ch.sequences_by_order(bnds, int(mp.pc.max_order), int(mp.pc.max_wall_bounces),
                                       int(max_obj))
        n_seq = 0
        thr = mp._threshold(grid, level)
        bound_sum = np.zeros(x.n)
        kept, order_reached, n_dropped = [], 0, 0
        one = wp.empty(n, dtype=wp.float64, device=dev)
        bound2 = wp.zeros((self.NCOPY, x.n), dtype=wp.float64, device=dev)
        for order, level_seqs in enumerate(levels, start=1):
            any_kept = False
            n_seq += len(level_seqs)
            for seq in level_seqs:
                kinds, faces, _ = self._seq_arrays([seq])

                def bound(refine, _k=kinds, _f=faces, _m=len(seq)):
                    bound2.zero_()
                    wp.launch(_bound_k, n, [pos, onfl, wallof, state, R, best, amp0, Pv, ea, en,
                                            ev, self.beam, _k, _f, _m, self.spec, self.faces,
                                            wp.float64(rr.k), wp.float64(mp.surface.rms),
                                            wp.float64(self.alpha_np), wp.float64(rr.c), x, refine,
                                            wp.float64(self.lam), self.max_iter, one, bound2],
                              device=dev)
                    return self._sum_bins(bound2)
                if _SEQ_TIMING:
                    import time as _t
                    wp.synchronize_device(dev)
                    _t0 = _t.perf_counter()
                seq_sum = bound(0)                 # the flat-floor image solution decides
                if _SEQ_TIMING:
                    wp.synchronize_device(dev)
                    print("seq", "+".join(b_.kind for b_ in seq), f"{(_t.perf_counter() - _t0) * 1e3:.1f} ms",
                          flush=True)
                if np.any(seq_sum > thr):
                    if any(bd.refine() for bd in seq):
                        seq_sum = bound(1)         # the kept sequence, refined
                    kept.append(seq)
                    bound_sum += seq_sum
                    wp.launch(_best_k, n, [best, one], device=dev)
                    any_kept = True
                else:
                    n_dropped += 1
            if not any_kept:
                break
            order_reached = order
            thr = mp._threshold(grid, level + bound_sum)
        return kept, dict(order_reached=order_reached, sequences=n_seq, kept=len(kept),
                          dropped=n_dropped, via=["+".join(
                              "surface" if b_.kind == "S" else "floor" if b_.kind == "F"
                              else (f"wall:{b_.name}" if b_.kind == "W" else f"object:{b_.name}")
                              for b_ in s) for s in kept])

    # --------------------------------------------------------- diffraction
    def _disc(self) -> Disc:
        dif = self.mp.diffraction
        d = dif.disc
        if d.sectors > _NDIR or d.coarse > _NE:
            raise ValueError(f"GPU diffraction: sectors <= {_NDIR}, coarse <= {_NE}")
        Dc = Disc()
        Dc.n1, Dc.n2 = wp.float64(d.n1), wp.float64(d.n2)
        Dc.sectors, Dc.coarse, Dc.bisect = int(d.sectors), int(d.coarse), int(d.bisect)
        Dc.lam = wp.float64(dif.lam)
        Dc.eps = wp.float64(dif.eps)
        Dc.end_margin = wp.float64(dif.end_margin)
        Dc.ds = wp.float64(dif._s[1] - dif._s[0])
        Dc.F_re = wp.array(dif._cum.real.astype(np.float64), dtype=wp.float64, device=self.dev)
        Dc.F_im = wp.array(dif._cum.imag.astype(np.float64), dtype=wp.float64, device=self.dev)
        Dc.full = wp.vec2d(float(dif.full.real), float(dif.full.imag))
        Dc.use32 = int(self.diffraction_fp32)
        Dc.F32_re = wp.array(dif._cum.real.astype(np.float32), dtype=wp.float32, device=self.dev)
        Dc.F32_im = wp.array(dif._cum.imag.astype(np.float32), dtype=wp.float32, device=self.dev)
        Dc.ds32 = wp.float32(dif._s[1] - dif._s[0])
        return Dc

    def _diffraction(self, P, gsc, state, D, Tt) -> dict:
        """``Multipath.diffract_direct`` on the device: hit fractions of the
        blocked candidates, the lit-side neighbour table (sorted keys, the
        smallest fraction per cell), classification, and the full factor
        for the legs with an edge in the disc."""
        mp, dev = self.mp, self.dev
        if mp.diffraction is None:
            return {"diffracted": 0}
        from .diffraction import EdgeDiffraction
        if isinstance(mp.diffraction, EdgeDiffraction):
            return self._edge_diffraction(P, gsc, state, D, Tt)
        if getattr(self, "_disc_s", None) is None:
            self._disc_s = self._disc()
        Dc = self._disc_s
        n = len(gsc)
        offset = wp.float64(self.rr._rc.sampling.surface_offset_m)
        Pv = wp.vec3d(*np.asarray(P, float))
        flag = wp.empty(n, dtype=wp.int32, device=dev)
        wp.launch(_blocked_flag_k, n, [state, flag], device=dev)
        slot = wp.empty(n, dtype=wp.int32, device=dev)
        wp.utils.array_scan(flag, slot, inclusive=True)
        nb = _last(slot)
        if nb == 0:
            return {"diffracted": 0}
        ne = 9 * nb
        keys = wp.empty(2 * ne, dtype=wp.int64, device=dev)       # radix sort scratch: 2x
        vals = wp.empty(2 * ne, dtype=wp.int32, device=dev)
        frac = wp.empty(n, dtype=wp.float64, device=dev)
        cell = wp.float64(self.rr._rc.diffraction.neighbour_cell_m)
        wp.launch(_diff_hit_k, n, [gsc.pos, gsc.normal, state, Pv, offset, self.oracle, cell, slot,
                                   keys, vals, frac], device=dev)
        self._tick("diffraction.hits")
        wp.utils.radix_sort_pairs(keys, vals, ne)
        start = wp.empty(ne, dtype=wp.int32, device=dev)
        wp.launch(_run_start_k, ne, [keys, start], device=dev)
        runid = wp.empty(ne, dtype=wp.int32, device=dev)
        wp.utils.array_scan(start, runid, inclusive=True)
        nu = _last(runid)
        ukeys = wp.empty(max(nu, 1), dtype=wp.int64, device=dev)
        umin = wp.empty(max(nu, 1), dtype=wp.float64, device=dev)
        wp.launch(_run_min_k, ne, [keys, vals, frac, ne, ukeys, umin, runid], device=dev)
        self._tick("diffraction.table")
        cls = wp.empty(n, dtype=wp.int32, device=dev)
        d1 = wp.zeros(n, dtype=wp.float64, device=dev)
        cen = wp.zeros(n, dtype=wp.int32, device=dev)
        wp.launch(_diff_legs_k, n, [gsc.pos, gsc.normal, state, Tt, frac, Pv, offset, cell, ukeys,
                                    umin, nu, self.oracle, Dc, D, cls, d1, cen], device=dev)
        counts = wp.zeros(2, dtype=wp.int32, device=dev)
        eflag = wp.empty(n, dtype=wp.int32, device=dev)
        wp.launch(_diff_report_k, n, [cls, D, eflag, counts], device=dev)
        escan = wp.empty(n, dtype=wp.int32, device=dev)
        wp.utils.array_scan(eflag, escan, inclusive=True)
        ne_ = _last(escan)
        self._tick("diffraction.classify")
        if ne_:
            edge = wp.empty(ne_, dtype=wp.int32, device=dev)
            wp.launch(_compact_k, n, [eflag, escan, edge], device=dev)
            wp.launch(_diff_factor_k, ne_, [edge, gsc.pos, gsc.normal, d1, Pv, offset,
                                            self.oracle, Dc, D, cen], device=dev)
        wp.launch(_edge_count_k, n, [cls, D, counts], device=dev)
        cn = counts.numpy()
        self._tick("diffraction.factor")
        self._diff_counts = dict(legs=int(cn[0]), edges_legs=ne_, blocked=nb)
        self._last_cls = cls
        return {"diffracted": int(cn[0]), "edges": int(cn[1])}

    #: points of the F(v) table of the edge method (linear interpolation:
    #: < 1e-7 off scipy's Fresnel integrals)
    EDGE_TABLE_N = 65537

    def _edge_diffraction(self, P, gsc, state, D, Tt) -> dict:
        """``diffraction.method: edge``: the closed-form factor of the nearest
        free straight edge (``EdgeDiffraction``), no ray casts."""
        from .diffraction import fresnel_F
        dif, dev = self.mp.diffraction, self.dev
        ed = dif.edges
        near = ed.near(np.asarray(P, float), dif.reach)
        if near.size == 0:
            return {"diffracted": 0}
        if getattr(self, "_edge_tab", None) is None:
            vmax = float(np.sqrt(2.0 * dif.n2)) + 0.5
            v = np.linspace(-vmax, vmax, self.EDGE_TABLE_N)
            F = fresnel_F(v)
            f1 = lambda a: wp.array(np.ascontiguousarray(a, np.float64), dtype=wp.float64, device=dev)  # noqa: E731
            self._edge_tab = (f1(F.real), f1(F.imag), -vmax, float(v[1] - v[0]))
        boxes, inv = np.unique(ed.box[near], return_inverse=True)
        v3 = lambda a: wp.array(np.ascontiguousarray(a, np.float64).reshape(-1, 3), dtype=wp.vec3d, device=dev)  # noqa: E731
        E = Edges()
        E.a, E.b = v3(ed.a[near]), v3(ed.b[near])
        E.box = wp.array(inv.astype(np.int32), dtype=wp.int32, device=dev)
        E.n = int(near.size)
        E.centre = v3(ed.centre[boxes])
        E.ax_u, E.ax_n, E.ax_z = (v3(ed.axes[boxes, k]) for k in range(3))
        E.half = v3(ed.half[boxes])
        E.nbox = int(boxes.size)
        E.lam, E.n1, E.n2 = wp.float64(dif.lam), wp.float64(dif.n1), wp.float64(dif.n2)
        E.margin = wp.float64(dif.end_margin)
        E.F_re, E.F_im, v0, dv = self._edge_tab
        E.v0, E.dv = wp.float64(v0), wp.float64(dv)
        cnt = wp.zeros(1, dtype=wp.int32, device=dev)
        wp.launch(_edge_factor_k, len(gsc), [gsc.pos, gsc.normal, state, Tt, wp.vec3d(*np.asarray(P, float)),
                                              wp.float64(self.rr._rc.sampling.surface_offset_m), E, D, cnt],
                  device=dev)
        self._tick("diffraction.edge")
        n = int(cnt.numpy()[0])
        return {"diffracted": n, "edges": n}
