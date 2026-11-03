"""World-fixed diffuse scatterers (``raycast_design.md`` §2, §4.2; notes 01, 04).

A rough surface's incoherent field is represented by point scatterers whose
mean-square amplitude is the bistatic cross-section times the area each one
stands for (the point-based model, Brown et al. 2017). Two properties are
load-bearing:

* **World-fixed.** Positions and phases are a pure function of (world seed,
  lattice cell) -- a splitmix64 hash, the same family as the geometric
  renderer's scatterer phases -- so consecutive pings re-illuminate the same
  scatterers (the along-track correlation the corpus measures), the CPU and
  GPU paths draw identical sets, and re-sampling converges to one
  realisation instead of drawing new speckle.
* **Stratified.** One jittered point per lattice cell, never Poisson: the
  sum of ``|a|^2`` per bin is then a quadrature of the area integral with a
  small, smoothly falling error. Random positions would leave a ~1/sqrt(N)
  fluctuation of the per-bin mean (16 % at N = 40), a spurious K-like texture
  that AL02 shows is a *scene* property, never a sampling one.

The seabed sheet (terrain + rough objects) is sampled on a world lattice in
x-y with the sheet's own triangle split (so every point lies on the mesh),
refined on steep cells; any other triangle (walls, the incoherent share of
smooth solids, apron skirts) on a lattice in its own plane. Both are clipped
to the region the caller passes (the beam strip), so a 50 m quay costs only
the part in the beam.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .scene_mesh import KIND_APRON, TriangleScene

_K1 = np.uint64(0x9E3779B97F4A7C15)
_K2 = np.uint64(0xC2B2AE3D27D4EB4F)
_K3 = np.uint64(0x165667B19E3779F9)
_M1 = np.uint64(0xBF58476D1CE4E5B9)
_M2 = np.uint64(0x94D049BB133111EB)
_INV53 = 1.0 / float(1 << 53)

#: Salts of the independent streams.
SALT_JX, SALT_JY, SALT_PHASE, SALT_SUB, SALT_AMP = 1, 2, 3, 4, 5


def _mix(x: np.ndarray) -> np.ndarray:
    with np.errstate(over="ignore"):
        x = (x ^ (x >> np.uint64(30))) * _M1
        x = (x ^ (x >> np.uint64(27))) * _M2
        return x ^ (x >> np.uint64(31))


def hash_uniform(seed: int, a: np.ndarray, b: np.ndarray, c: np.ndarray,
                 salt: int) -> np.ndarray:
    """Uniform [0, 1) as a pure function of (seed, a, b, c, salt)."""
    with np.errstate(over="ignore"):
        h = np.uint64(seed & 0xFFFFFFFFFFFFFFFF) ^ (np.uint64(salt) * _K3)
        h = _mix(h ^ (np.asarray(a).astype(np.int64).view(np.uint64) * _K1))
        h = _mix(h ^ (np.asarray(b).astype(np.int64).view(np.uint64) * _K2))
        h = _mix(h ^ (np.asarray(c).astype(np.int64).view(np.uint64) * _K3))
    return (h >> np.uint64(11)).astype(np.float64) * _INV53


def rayleigh_factor(seed: int, a, b, c) -> np.ndarray:
    """|f| for f = (X + iY)/sqrt(2): |f|^2 ~ Exp(1), so |f| = sqrt(-ln u),
    world-fixed through the hash (its phase is the scatterer's phase)."""
    u = hash_uniform(seed, a, b, c, SALT_AMP)
    return np.sqrt(-np.log(np.maximum(1.0 - u, 1e-300)))


@dataclass
class Scatterers:
    """A set of point scatterers on the scene's surfaces."""

    pos: np.ndarray          # (n, 3) world position on the mesh
    normal: np.ndarray       # (n, 3) unit normal of the triangle it lies on
    area: np.ndarray         # (n,) surface area it stands for [m^2]
    phase: np.ndarray        # (n,) world-fixed random phase [rad]
    tri: np.ndarray          # (n,) triangle index, -1 = beyond the grid
    material: np.ndarray     # (n,) material index (mesh.material_names)
    object_id: np.ndarray    # (n,) -1 = seabed / wall
    kind: np.ndarray         # (n,) triangle kind
    #: |f| of the complex circular Gaussian factor f = (X + iY)/sqrt(2)
    #: (Brown, Johnson, Olson 2017, eq. 3): Rayleigh, E|f|^2 = 1, so every
    #: scatterer's return is itself Gaussian and the field is Gaussian at
    #: any scatterer density -- a sparse lattice cannot fake K-like tails.
    rayleigh: np.ndarray

    def __len__(self) -> int:
        return int(self.area.size)

    @staticmethod
    def concat(parts: list["Scatterers"]) -> "Scatterers":
        parts = [p for p in parts if len(p)]
        if not parts:
            z3, z1 = np.zeros((0, 3)), np.zeros(0)
            zi = np.zeros(0, dtype=np.int64)
            return Scatterers(z3, z3, z1, z1, zi, zi, zi, zi.astype(np.uint8), z1)
        return Scatterers(*(np.concatenate([getattr(p, f) for p in parts])
                            for f in ("pos", "normal", "area", "phase", "tri",
                                      "material", "object_id", "kind",
                                      "rayleigh")))


class SheetSampler:
    """Scatterers on the seabed sheet (+ the apron's flat continuation)."""

    def __init__(self, mesh: TriangleScene, scene, seed: int,
                 spacing_m: float, max_refine: int = 8) -> None:
        g = scene.grid
        self.g = g
        self.mesh = mesh
        self.seed = int(seed)
        self.h = float(spacing_m)
        self.max_refine = int(max_refine)
        self.nq = (g.nx - 1) * (g.ny - 1)
        nv = g.nx * g.ny
        self.z = mesh.vertices[:nv, 2].reshape(g.ny, g.nx)
        self.outside = getattr(scene, "outside", "fill")
        self.zmin = float(self.z.min())
        n = mesh.normals()
        self.tri_normal = n
        self.out_mat = mesh.material[mesh.kind == KIND_APRON][0] \
            if (mesh.kind == KIND_APRON).any() else 0

    def _locate(self, x: np.ndarray, y: np.ndarray):
        """z, triangle index and normal of the sheet at (x, y)."""
        g = self.g
        fx = (x - g.origin_x) / g.resolution
        fy = (y - g.origin_y) / g.resolution
        inside = (fx >= 0) & (fy >= 0) & (fx <= g.nx - 1) & (fy <= g.ny - 1)
        cx = np.clip(fx, 0, g.nx - 1 - 1e-9)
        cy = np.clip(fy, 0, g.ny - 1 - 1e-9)
        i = cx.astype(np.int64)
        j = cy.astype(np.int64)
        u, v = cx - i, cy - j
        za, zb = self.z[j, i], self.z[j, i + 1]
        zc, zd = self.z[j + 1, i + 1], self.z[j + 1, i]
        lower = u >= v                        # triangle (a, b, c)
        z = np.where(lower, za + u * (zb - za) + v * (zc - zb),
                     za + u * (zc - zd) + v * (zd - za))
        q = j * (g.nx - 1) + i
        tri = np.where(lower, q, self.nq + q)
        if self.outside != "edge":
            z = np.where(inside, z, self.zmin)
        tri = np.where(inside, tri, -1)
        return z, tri

    def sample(self, x0: float, x1: float, row_lo: np.ndarray,
               row_hi: np.ndarray, y0: float) -> Scatterers:
        """Scatterers in the region given row by row: lattice rows q from
        ``y0`` (row q covers y in [(q0+q) h, (q0+q+1) h)), each spanning x in
        ``[row_lo[q], row_hi[q]]``."""
        h = self.h
        q0 = int(np.floor(y0 / h))
        ps, qs = [], []
        for k, (lo, hi) in enumerate(zip(row_lo, row_hi)):
            if not hi > lo:
                continue
            p = np.arange(int(np.floor(lo / h)), int(np.floor(hi / h)) + 1)
            ps.append(p)
            qs.append(np.full(p.size, q0 + k))
        if not ps:
            return Scatterers.concat([])
        p = np.concatenate(ps)
        q = np.concatenate(qs)
        sub = np.zeros(p.size, dtype=np.int64)
        x = (p + hash_uniform(self.seed, p, q, sub, SALT_JX)) * h
        y = (q + hash_uniform(self.seed, p, q, sub, SALT_JY)) * h
        z, tri = self._locate(x, y)
        nz = np.where(tri >= 0, self.tri_normal[np.maximum(tri, 0), 2], 1.0)
        J = 1.0 / np.maximum(np.abs(nz), 1e-3)
        n_sub = np.minimum(np.ceil(J).astype(np.int64) ** 2,
                           self.max_refine ** 2)
        # Steep cells: n_sub jittered points instead of one, each standing
        # for its share of the cell (its own triangle's slope factor).
        rep = n_sub > 1
        if rep.any():
            pr = np.repeat(p[rep], n_sub[rep])
            qr = np.repeat(q[rep], n_sub[rep])
            starts = np.cumsum(n_sub[rep]) - n_sub[rep]
            kk = np.arange(pr.size) - np.repeat(starts, n_sub[rep])
            nr = np.repeat(n_sub[rep], n_sub[rep])
            xr = (pr + hash_uniform(self.seed, pr, qr, kk + 1, SALT_JX)) * h
            yr = (qr + hash_uniform(self.seed, pr, qr, kk + 1, SALT_JY)) * h
            zr, tr = self._locate(xr, yr)
            p = np.concatenate([p[~rep], pr])
            q = np.concatenate([q[~rep], qr])
            sub = np.concatenate([sub[~rep], kk + 1])
            x = np.concatenate([x[~rep], xr])
            y = np.concatenate([y[~rep], yr])
            z = np.concatenate([z[~rep], zr])
            tri = np.concatenate([tri[~rep], tr])
            n_sub = np.concatenate([np.ones((~rep).sum(), dtype=np.int64), nr])
        nrm = np.where((tri >= 0)[:, None],
                       self.tri_normal[np.maximum(tri, 0)],
                       np.array([0.0, 0.0, 1.0]))
        nrm = np.where(nrm[:, 2:3] < 0, -nrm, nrm)      # sheet faces up
        area = h * h / np.maximum(np.abs(nrm[:, 2]), 1e-3) / n_sub
        ph = 2.0 * np.pi * hash_uniform(self.seed, p, q, sub, SALT_PHASE)
        ray = rayleigh_factor(self.seed, p, q, sub)
        m = self.mesh
        tri_c = np.maximum(tri, 0)
        mat = np.where(tri >= 0, m.material[tri_c], self.out_mat)
        oid = np.where(tri >= 0, m.object_id[tri_c], -1)
        kind = np.where(tri >= 0, m.kind[tri_c], KIND_APRON).astype(np.uint8)
        return Scatterers(np.column_stack([x, y, z]), nrm, area, ph, tri,
                          mat, oid, kind, ray)


def planar_sample(mesh: TriangleScene, tris: np.ndarray, seed: int,
                  spacing_m: float, keep) -> Scatterers:
    """Scatterers on arbitrary triangles: a lattice in each triangle's own
    plane anchored at its first vertex, one jittered point per cell whose
    jittered position falls inside the triangle, filtered by ``keep(pos)``
    (the beam strip). World-fixed through (seed, triangle, cell)."""
    h = float(spacing_m)
    tv = mesh.vertices[mesh.triangles[tris]]
    nrm_all = mesh.normals()[tris]
    out = []
    for t, (A, B, C), n in zip(tris, tv, nrm_all):
        e1 = B - A
        L1 = np.linalg.norm(e1)
        if L1 <= 0:
            continue
        e1 = e1 / L1
        e2 = np.cross(n, e1)
        P = np.array([[0.0, 0.0], [L1, 0.0],
                      [(C - A) @ e1, (C - A) @ e2]])
        lo, hi = P.min(axis=0), P.max(axis=0)
        ip = np.arange(int(np.floor(lo[0] / h)), int(np.floor(hi[0] / h)) + 1)
        iq = np.arange(int(np.floor(lo[1] / h)), int(np.floor(hi[1] / h)) + 1)
        if ip.size * iq.size > 50_000_000:
            raise MemoryError(f"triangle {t}: {ip.size * iq.size} lattice cells")
        pp, qq = (a.ravel() for a in np.meshgrid(ip, iq))
        tt = np.full(pp.size, int(t))
        s = (pp + hash_uniform(seed, tt, pp, qq, SALT_JX)) * h
        r = (qq + hash_uniform(seed, tt, pp, qq, SALT_JY)) * h
        # barycentric inside test in the plane
        v0, v1 = P[1] - P[0], P[2] - P[0]
        den = v0[0] * v1[1] - v0[1] * v1[0]
        a = (s * v1[1] - r * v1[0]) / den
        b = (v0[0] * r - v0[1] * s) / den
        ins = (a >= 0) & (b >= 0) & (a + b <= 1)
        pos = A + np.outer(s[ins], e1) + np.outer(r[ins], e2)
        sel = keep(pos)
        if not sel.any():
            continue
        pi_, qi = pp[ins][sel], qq[ins][sel]
        k = int(sel.sum())
        out.append(Scatterers(
            pos[sel], np.tile(n, (k, 1)), np.full(k, h * h),
            2.0 * np.pi * hash_uniform(seed, np.full(k, int(t)), pi_, qi,
                                       SALT_PHASE),
            np.full(k, int(t), dtype=np.int64),
            np.full(k, int(mesh.material[t]), dtype=np.int64),
            np.full(k, int(mesh.object_id[t]), dtype=np.int64),
            np.full(k, mesh.kind[t], dtype=np.uint8),
            rayleigh_factor(seed, np.full(k, int(t)), pi_, qi)))
    return Scatterers.concat(out)
