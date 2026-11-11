"""Diffraction at shadow edges: ray-traced Fresnel-Kirchhoff per leg (note 06;
``raycast_design.md`` §3, §4.8; gates D1, D2).

For a leg from A to B whose first Fresnel zones are partly blocked by the
scene, the field ratio is the Kirchhoff integral over a plane perpendicular to
the leg at distance ``d1`` from A (``d2`` from B), through the occluder:

    U / U0 = int V(xi) E(xi) w(xi) dA  /  int E(xi) w(xi) dA,
    E = exp(-i pi |xi|^2 / (lambda d_eff)),   d_eff = d1 d2 / (d1 + d2),

with ``V(xi)`` = 1 where both A -> xi and xi -> B are clear (the one scene
oracle, so the occluder is the real geometry: a box edge, a wall top, a
pontoon draft, a pile of rough objects). The ratio form returns exactly 1 for
a clear disc and is Babinet-consistent; for a straight edge it is the Fresnel
integral ``F(v) = ((1 + i)/2) int_v^inf exp(-i pi t^2 / 2) dt`` (gate D1:
|F| within 0.5 dB, phase within 0.1 rad, v in [-2, 3]). The kernel's sign
follows this renderer's ``exp(-i omega tau)`` convention: a longer path
carries ``exp(-i k dL)``, and the ratio is then exactly the textbook F(v).

Quadrature: polar, in the zone index ``s = |xi|^2 / (lambda d_eff)`` (the
phase is ``-pi s``) and the angle. A smooth window ``w(s)`` (1 inside ``n1``
zones, raised cosine to 0 at ``n2``) suppresses the truncation of the disc,
as the specular channel's window does. Per angular sector, ``V`` is sampled
at coarse ``s`` and every visibility transition is located by bisection, so
V is integrated as a step function with exact edges against a tabulated
``int w(s) e^{-i pi s} ds`` -- the edge position, not the sample spacing, sets
the accuracy. Sample counts are numerical settings (Phase 5).

Monostatic two-way: the out and back legs pass the same edge, so a pair of
chains carries the product of their legs' factors.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FresnelDisc:
    n1: float = 6.0            # zones of full weight
    n2: float = 20.0           # zone where the taper reaches 0
    sectors: int = 16          # edge-sampling directions (D1 study: 0.20 dB)
    coarse: int = 20           # coarse samples in s per sector
    bisect: int = 7            # bisection steps per transition

    def table(self, n_fine: int = 200001):
        s = np.linspace(0.0, self.n2, n_fine)
        t = np.clip((s - self.n1) / (self.n2 - self.n1), 0.0, 1.0)
        w = 0.5 * (1.0 + np.cos(np.pi * t))
        f = w * np.exp(-1j * np.pi * s)
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (f[1:] + f[:-1])
                                               * np.diff(s))])
        return s, cum


#: points per oracle call when samples are stacked (WarpBVH's own chunk)
_POINTS_PER_CALL = 4_000_000


class Diffraction:
    """Fresnel-Kirchhoff leg factors against the scene oracle."""

    def __init__(self, oracle, lam: float, disc: FresnelDisc | None = None,
                 eps: float = 1e-6, end_margin_m: float = 0.0) -> None:
        self.oracle = oracle
        self.lam = float(lam)
        self.disc = disc or FresnelDisc()
        self.eps = float(eps)
        #: geometry within this distance of B (a contributor's own
        #: supporting surface) is not an occluder: the scattering law covers it
        self.end_margin = float(end_margin_m)
        self._s, self._cum = self.disc.table()
        self.full = self._cum[-1]


    def _F(self, s: np.ndarray) -> np.ndarray:
        return (np.interp(s, self._s, self._cum.real)
                + 1j * np.interp(s, self._s, self._cum.imag))

    def _visible(self, A, B, xi) -> np.ndarray:
        """Both segments A -> xi and xi -> B clear (and in the water)."""
        o = self.oracle
        v = ~o.occluded(A, xi, self.eps)
        if v.any():
            i = np.flatnonzero(v)
            Bi = B[i]
            if self.end_margin > 0.0:
                d = Bi - xi[i]
                L = np.linalg.norm(d, axis=1)
                Bi = xi[i] + d * (np.maximum(L - self.end_margin, 1e-6)
                                  / np.maximum(L, 1e-300))[:, None]
            v[i] = ~o.occluded(xi[i], Bi, self.eps)
        return v

    def _frame(self, A, B, d1):
        d = B - A
        L = np.linalg.norm(d, axis=1)
        u = d / L[:, None]
        d1 = np.clip(np.asarray(d1, float), 1e-4, L - 1e-4)
        de = d1 * (L - d1) / L
        a = np.where(np.abs(u[:, 2:3]) < 0.9, np.array([[0, 0, 1.0]]),
                     np.array([[1.0, 0, 0]]))
        e1 = np.cross(u, a)
        e1 /= np.linalg.norm(e1, axis=1, keepdims=True)
        e2 = np.cross(u, e1)
        return A + d1[:, None] * u, de, e1, e2

    def _edges(self, A, B, C, de, dirv):
        """Visibility along one direction of the disc for every leg: the
        initial state and the transition zone indices (coarse samples, each
        transition bisected). Returns (state0 (n,), list of (n,) arrays of
        transition s, NaN where a leg has fewer transitions)."""
        n = A.shape[0]
        disc = self.disc
        s_c = np.linspace(0.0, disc.n2, disc.coarse + 1)
        K = s_c.size
        rad = np.sqrt(np.outer(self.lam * de, s_c))
        xi = C[:, None, :] + rad[:, :, None] * dirv[:, None, :]
        V = self._visible(np.repeat(A, K, axis=0), np.repeat(B, K, axis=0),
                          xi.reshape(-1, 3)).reshape(n, K)
        # every transition of every leg bisected together (one visibility
        # query per bisection step, not one per coarse interval): the same
        # arithmetic per transition, in a few large oracle calls
        ii, kk = np.nonzero(V[:, 1:] != V[:, :-1])      # row-major: k ascending per leg
        cnt = np.bincount(ii, minlength=n).astype(np.int64)
        edges = []
        if ii.size:
            a_s = s_c[kk].astype(float)
            b_s = s_c[kk + 1].astype(float)
            va = V[ii, kk]
            for _ in range(disc.bisect):
                m = 0.5 * (a_s + b_s)
                xm = C[ii] + np.sqrt(self.lam * de[ii] * m)[:, None] * dirv[ii]
                same = self._visible(A[ii], B[ii], xm) == va
                a_s = np.where(same, m, a_s)
                b_s = np.where(same, b_s, m)
            e = 0.5 * (a_s + b_s)
            first = np.r_[0, np.cumsum(cnt)[:-1]]
            rank = np.arange(ii.size) - first[ii]
            for r in range(int(rank.max()) + 1):
                sel = rank == r
                col = np.full(n, np.nan)
                col[ii[sel]] = e[sel]
                edges.append(col)
        return V[:, 0].copy(), edges, cnt

    def factor(self, A: np.ndarray, B: np.ndarray, d1: np.ndarray,
               max_depth: int = 6) -> np.ndarray:
        """U/U0 for legs A -> B (n, 3) with the plane at ``d1`` from A.

        Edges are sampled on ``sectors`` directions. Across a sector whose
        two bounding directions share their transition structure, each edge
        is the straight chord (in the disc plane) between its two points,
        and H(s) = int_0^s w e^{-i pi s'} ds' is averaged along it with a
        sub-angle count proportional to the zone span (no rays): exact for a
        straight edge. A sector whose directions disagree (an edge leaving
        the window, an occluder's corner) is bisected with a new ray
        direction, up to ``max_depth`` times, then trapezoid."""
        A = np.atleast_2d(np.asarray(A, float))
        B = np.atleast_2d(np.asarray(B, float))
        n = A.shape[0]
        C, de, e1, e2 = self._frame(A, B, d1)
        N = self.disc.sectors
        dth = 2.0 * np.pi / N
        th = np.arange(N + 1) * dth

        def row(t, idx):
            return self._edges(A[idx], B[idx], C[idx], de[idx],
                               np.cos(t) * e1[idx] + np.sin(t) * e2[idx])

        rows = self._rows(A, B, C, de, e1, e2, th[:N])
        rows.append(rows[0])
        out = np.zeros(n, dtype=complex)
        for k in range(N):
            out += self._sector(th[k], dth, rows[k], rows[k + 1],
                                np.arange(n), row, max_depth)
        return out / (N * self.full)

    def _rows(self, A, B, C, de, e1, e2, thetas) -> list:
        """``_edges`` for every leg on each direction of ``thetas``, the
        directions stacked into one call per chunk of legs (the same rows as
        one call per direction)."""
        n, N = A.shape[0], len(thetas)
        per_leg = N * (self.disc.coarse + 1)
        chunk = max(1, _POINTS_PER_CALL // per_leg)
        st = [np.zeros(n, dtype=bool) for _ in range(N)]
        ed: list = [[] for _ in range(N)]
        ct = [np.zeros(n, dtype=np.int64) for _ in range(N)]
        # scalar cos / sin per direction, as the per-direction calls took them
        c = np.array([np.cos(t) for t in thetas])
        s_ = np.array([np.sin(t) for t in thetas])
        for a in range(0, n, chunk):
            sl = slice(a, min(a + chunk, n))
            m = sl.stop - sl.start
            rep = lambda x: np.tile(x[sl], (N, 1)) if x.ndim == 2 else np.tile(x[sl], N)  # noqa: E731
            dirv = (np.repeat(c, m)[:, None] * np.tile(e1[sl], (N, 1))
                    + np.repeat(s_, m)[:, None] * np.tile(e2[sl], (N, 1)))
            st0, edges, cnt = self._edges(rep(A), rep(B), rep(C), rep(de), dirv)
            for k in range(N):
                ks = slice(k * m, (k + 1) * m)
                st[k][sl] = st0[ks]
                ct[k][sl] = cnt[ks]
                for j, e in enumerate(edges):
                    while len(ed[k]) <= j:
                        ed[k].append(np.full(n, np.nan))
                    ed[k][j][sl] = e[ks]
        # a column that holds no transition for any leg of a direction is dropped,
        # as one call per direction would never have created it
        return [(st[k], [e for e in ed[k] if not np.all(np.isnan(e))], ct[k])
                for k in range(N)]

    def _value(self, r) -> np.ndarray:
        st0, edges, cnt = r
        n2 = self.disc.n2
        acc = np.zeros(st0.size, dtype=complex)
        st = st0.copy()
        lo = np.zeros(st0.size)
        for e in edges:
            has = ~np.isnan(e)
            ee = np.where(has, e, n2)
            acc += np.where(has & st, self._F(ee) - self._F(lo), 0.0)
            lo = np.where(has, ee, lo)
            st = np.where(has, ~st, st)
        acc += np.where(st, self._F(np.full(st0.size, n2)) - self._F(lo), 0.0)
        return acc

    @staticmethod
    def _sub(r, sel):
        st0, edges, cnt = r
        return st0[sel], [e[sel] for e in edges], cnt[sel]

    def _sector(self, t0, dth, r0, r1, idx, row, depth) -> np.ndarray:
        """Integral over [t0, t0 + dth] (weight = dth / sector width) for
        the legs ``idx``; r0, r1 are the rows of those legs."""
        n = idx.size
        n2 = self.disc.n2
        same = (r0[0] == r1[0]) & (r0[2] == r1[2])
        out = np.zeros(n, dtype=complex)
        wgt = dth / (2.0 * np.pi / self.disc.sectors)
        if same.any():
            s = np.flatnonzero(same)
            a0 = self._sub(r0, s)
            a1 = self._sub(r1, s)
            st = a0[0].copy()
            lo0 = np.zeros(s.size)
            lo1 = np.zeros(s.size)
            m = max(len(a0[1]), len(a1[1]))
            lin = np.zeros(s.size, dtype=complex)
            for j in range(m + 1):
                if j < m:
                    e0 = a0[1][j] if j < len(a0[1]) else np.full(s.size, np.nan)
                    e1_ = a1[1][j] if j < len(a1[1]) else np.full(s.size, np.nan)
                    has = ~np.isnan(e0) & ~np.isnan(e1_)
                    h0 = np.where(has, e0, n2)
                    h1 = np.where(has, e1_, n2)
                else:
                    has = np.ones(s.size, dtype=bool)
                    h0 = h1 = np.full(s.size, n2)
                seg = (self._chord_H(h0, h1, t0, dth)
                       - self._chord_H(lo0, lo1, t0, dth))
                lin += np.where(has & st, seg, 0.0)
                lo0 = np.where(has, h0, lo0)
                lo1 = np.where(has, h1, lo1)
                st = np.where(has, ~st, st)
            out[s] = lin * wgt
        diff = np.flatnonzero(~same)
        if diff.size:
            if depth <= 0:
                out[diff] = 0.5 * (self._value(self._sub(r0, diff))
                                   + self._value(self._sub(r1, diff))) * wgt
            else:
                tm = t0 + 0.5 * dth
                rm = row(tm, idx[diff])
                d0 = self._sub(r0, diff)
                d1 = self._sub(r1, diff)
                out[diff] = (self._sector(t0, 0.5 * dth, d0, rm, idx[diff], row,
                                          depth - 1)
                             + self._sector(tm, 0.5 * dth, rm, d1, idx[diff], row,
                                            depth - 1))
        return out

    def _chord_H(self, s0: np.ndarray, s1: np.ndarray, t0: float,
                 dth: float, m: int | None = None) -> np.ndarray:
        """Mean over the sector [t0, t0 + dth] of H(s(theta)), the edge
        taken as the straight chord (in the disc plane) between its points
        at the two bounding directions -- exact for a straight edge."""
        r0 = np.sqrt(np.maximum(s0, 0.0))
        r1 = np.sqrt(np.maximum(s1, 0.0))
        p0 = np.stack([r0 * np.cos(t0), r0 * np.sin(t0)], axis=-1)
        p1 = np.stack([r1 * np.cos(t0 + dth), r1 * np.sin(t0 + dth)], axis=-1)
        if m is None:
            # >= 8 samples per period of e^{-i pi s} (period 2 in s) along
            # EACH chord's own zone span, never fewer than 16: per leg, so a
            # leg's factor does not depend on the other legs of the call
            mm = np.minimum(np.maximum(16, np.ceil(4.0 * np.abs(np.asarray(s1) - np.asarray(s0)))
                                       + 16), 512).astype(np.int64)
            if mm.size == 0:
                return np.zeros(np.shape(s0), dtype=complex)
            out = np.empty(np.shape(s0), dtype=complex)
            for mv in np.unique(mm):
                sel = mm == mv
                out[sel] = self._chord_H(np.asarray(s0)[sel], np.asarray(s1)[sel], t0, dth, int(mv))
            return out
        if s0.ndim == 1 and s0.size * m > _POINTS_PER_CALL:
            k = max(1, _POINTS_PER_CALL // m)          # memory: chunks of chords, same m
            return np.concatenate([self._chord_H(s0[a:a + k], s1[a:a + k], t0, dth, m)
                                   for a in range(0, s0.size, k)])
        # every sample angle of every chord in one interpolation (the angles
        # from scalar cos / sin as before); accumulated in the same order, so
        # the result is the per-sample loop's to the bit
        tj = [t0 + (j + 0.5) / m * dth for j in range(m)]
        c_ = np.array([np.cos(t) for t in tj]).reshape((m,) + (1,) * s0.ndim)
        s_ = np.array([np.sin(t) for t in tj]).reshape((m,) + (1,) * s0.ndim)
        e = p1 - p0
        den = c_ * e[..., 1] - s_ * e[..., 0]
        num = p0[..., 0] * e[..., 1] - p0[..., 1] * e[..., 0]
        big = np.abs(den) > 1e-12
        r = np.where(big, num / np.where(big, den, 1.0), 0.5 * (r0 + r1))
        r = np.clip(r, 0.0, np.sqrt(self.disc.n2))
        F = self._F(r * r)
        acc = np.zeros(s0.shape, dtype=complex)
        for j in range(m):
            acc += F[j]
        return acc / m

    def classify(self, A, B, d1, n_ang: int = 8) -> np.ndarray:
        """Per leg: 1 = every ring sample (0.5 .. n2 zones) and the centre
        clear; 0 = the centre and every ring to ``n1`` zones blocked (deep
        shadow: |F| <= |F(sqrt(2 n1))|, -24 dB at n1 = 6, set to 0); -1 =
        an edge in the disc (needs :meth:`factor`)."""
        A = np.atleast_2d(np.asarray(A, float))
        B = np.atleast_2d(np.asarray(B, float))
        n = A.shape[0]
        C, de, e1, e2 = self._frame(A, B, d1)
        disc = self.disc
        zones = [0.5, 2.0, disc.n1, 0.5 * (disc.n1 + disc.n2), disc.n2]
        angs = np.arange(n_ang) * 2 * np.pi / n_ang
        # the centre and every ring sample of a leg in one oracle call per
        # chunk of legs (independent samples: the same answer as one call each)
        zz = np.repeat(np.asarray(zones, float), n_ang)
        tt = np.tile(angs, len(zones))
        inner = np.r_[True, zz <= disc.n1]               # centre + rings to n1
        S = 1 + zz.size
        out = np.empty(n, dtype=np.int64)
        chunk = max(1, _POINTS_PER_CALL // S)
        for a in range(0, n, chunk):
            sl = slice(a, min(a + chunk, n))
            m = sl.stop - sl.start
            r = np.sqrt(self.lam * de[sl][:, None] * zz[None, :])          # (m, S-1)
            off = (r[:, :, None] * (np.cos(tt)[None, :, None] * e1[sl][:, None, :]
                                    + np.sin(tt)[None, :, None] * e2[sl][:, None, :]))
            xi = np.concatenate([C[sl][:, None, :], C[sl][:, None, :] + off], axis=1)
            vis = self._visible(np.repeat(A[sl], S, axis=0), np.repeat(B[sl], S, axis=0),
                                xi.reshape(-1, 3)).reshape(m, S)
            all_clear = vis.all(axis=1)
            inner_blocked = ~vis[:, inner].any(axis=1)
            out[sl] = np.where(all_clear, 1, np.where(inner_blocked, 0, -1))
        return out

    def leg_factors(self, A, B, d1) -> np.ndarray:
        """Classification, then the full factor where an edge is in the
        disc."""
        cls = self.classify(A, B, d1)
        out = cls.astype(complex)
        e = np.flatnonzero(cls < 0)
        if e.size:
            out[e] = self.factor(A[e], B[e], np.asarray(d1)[e])
        return out


# ===================================================== closed-form straight edges
# The fast variant's diffraction method (``diffraction.method: edge``,
# 2026-11-10, docs/engineering/raycast_fast_optimize_plan.md). The quantity is
# the one above, the per-leg ratio U/U0 of the windowed Kirchhoff integral,
# evaluated in closed form for the nearest straight edge of the scene's wall
# boxes instead of being ray-traced over the disc: for a half-plane it is
# exactly F(v) (gate D1), v = the signed clearance of the edge from the leg in
# units of sqrt(lambda d_eff / 2) (+ in the edge's geometric shadow). The same
# window weights it (1 inside n1 zones, a raised cosine to 0 at n2), so a lit
# leg gets 1 + w (F - 1), a shadowed one w F, and both are continuous across
# the shadow boundary (F(0) = 1/2) and at n2. Only straight edges with water
# on both sides diffract: a wall box's vertical corners clipped to the water
# column, its top edges when it lies submerged (a sill), its bottom edges where
# it floats above the seabed (a pontoon). The waterline of a wall that rises
# out of the water, and its toe on the seabed, are closed corners. Objects and
# seabed relief get no factor (as with diffraction off), nor does a leg whose
# shadow is cast by anything but exactly one wall box.

#: horizontal free edges are split into pieces of at most this length [m], each
#: kept where the water is clear beneath it (the seabed under a sill varies)
EDGE_PIECE_M = 0.25


def fresnel_F(v):
    """The half-plane ratio ``F(v) = ((1+i)/2) int_v^inf exp(-i pi t^2/2) dt``
    in this renderer's ``exp(-i omega tau)`` convention (gate D1's form)."""
    from scipy.special import fresnel
    S, C = fresnel(np.asarray(v, dtype=np.float64))
    return 0.5 * (1.0 + 1.0j) * ((0.5 - C) - 1.0j * (0.5 - S))


def edge_window(s, n1: float, n2: float):
    """``FresnelDisc``'s taper as a function of the zone index s."""
    t = np.clip((np.asarray(s, np.float64) - n1) / (n2 - n1), 0.0, 1.0)
    return 0.5 * (1.0 + np.cos(np.pi * t))


@dataclass
class EdgeSet:
    """The scene's free straight edges and the wall boxes that own them."""
    a: np.ndarray            # (E, 3) edge start
    b: np.ndarray            # (E, 3) edge end
    box: np.ndarray          # (E,) owning box index
    centre: np.ndarray       # (W, 3) box centre
    axes: np.ndarray         # (W, 3, 3) rows: along the wall, across it, up
    half: np.ndarray         # (W, 3) half extents along those axes

    def near(self, P, reach: float) -> np.ndarray:
        """Indices of the edges within ``reach`` of P."""
        if self.a.shape[0] == 0:
            return np.zeros(0, dtype=np.int64)
        d = self.b - self.a
        t = np.clip(np.einsum("ij,ij->i", P - self.a, d)
                    / np.maximum(np.einsum("ij,ij->i", d, d), 1e-300), 0.0, 1.0)
        dist = np.linalg.norm(self.a + t[:, None] * d - P, axis=1)
        return np.flatnonzero(dist <= reach)


def free_edges(rr) -> EdgeSet:
    """The wall boxes of the renderer's mesh (``scene_mesh._wall_box``, the
    very vertices the oracle traces) and their free edges in the water column."""
    from .scene_mesh import _wall_box
    m = rr.mesh
    walls = getattr(rr._scene, "walls", None) or []
    margin = max(float(rr._rc.sampling.surface_offset_m), 0.01)

    def seabed(xy):
        return rr.sheet._locate(np.atleast_1d(xy[..., 0]), np.atleast_1d(xy[..., 1]))[0]

    A, B, own, cen, axs, hal = [], [], [], [], [], []
    for wi, w in enumerate(walls):
        tri = np.flatnonzero(m.wall_index == wi)
        if tri.size == 0:
            continue
        vm = np.unique(m.vertices[m.triangles[tri]].reshape(-1, 3), axis=0)
        v, _ = _wall_box(w, float(vm[:, 2].min()))
        if not np.allclose(np.unique(v, axis=0), vm, atol=1e-9):
            raise RuntimeError(f"wall {w.name!r}: mesh box differs from _wall_box")
        z_base, z_top = float(v[0, 2]), float(v[4, 2])
        bi = len(cen)
        ux, uy = w.unit
        cen.append(v.mean(axis=0))
        axs.append(np.array([[ux, uy, 0.0], [-uy, ux, 0.0], [0.0, 0.0, 1.0]]))
        hal.append(np.array([0.5 * w.length, 0.5 * float(w.thickness), 0.5 * (z_top - z_base)]))
        for k in range(4):                                  # vertical corners
            x, y = v[k, 0], v[k, 1]
            z0 = max(z_base, float(seabed(np.array([x, y]))[0]) + margin)
            z1 = min(z_top, -margin)
            if z1 > z0 + 1e-6:
                A.append([x, y, z0]); B.append([x, y, z1]); own.append(bi)
        for z, below in ((z_top, False), (z_base, True)):   # top / bottom rims
            if z >= -margin:
                continue
            base = 4 if not below else 0
            for k in range(4):
                p0, p1 = v[base + k], v[base + (k + 1) % 4]
                n = max(int(np.ceil(np.linalg.norm(p1 - p0) / EDGE_PIECE_M)), 1)
                ts = np.linspace(0.0, 1.0, n + 1)
                pts = p0 + ts[:, None] * (p1 - p0)
                clear = seabed(pts[:, :2]) < z - margin
                for j in range(n):
                    if clear[j] and clear[j + 1]:
                        A.append(pts[j]); B.append(pts[j + 1]); own.append(bi)
    f3 = lambda x: np.asarray(x, np.float64).reshape(-1, 3)        # noqa: E731
    return EdgeSet(f3(A), f3(B), np.asarray(own, np.int64), f3(cen),
                   np.asarray(axs, np.float64).reshape(-1, 3, 3), f3(hal))


def segments_closest(P, X, a, b):
    """Closest points of the segments P->X (n legs) and a->b (n edges):
    ``(s, h)`` = the leg parameter in [0, 1] and the distance (Ericson,
    Real-Time Collision Detection 5.1.9)."""
    d1, d2, r = X - P, b - a, P - a
    A = np.einsum("ij,ij->i", d1, d1)
    E = np.einsum("ij,ij->i", d2, d2)
    Fv = np.einsum("ij,ij->i", d2, r)
    C = np.einsum("ij,ij->i", d1, r)
    Bv = np.einsum("ij,ij->i", d1, d2)
    den = A * E - Bv * Bv
    s = np.where(den > 1e-12 * A * E, np.clip((Bv * Fv - C * E) / np.where(den > 0, den, 1.0),
                                              0.0, 1.0), 0.0)
    t = (Bv * s + Fv) / np.maximum(E, 1e-300)
    lo, hi = t < 0.0, t > 1.0
    s = np.where(lo, np.clip(-C / A, 0.0, 1.0), np.where(hi, np.clip((Bv - C) / A, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    h = np.linalg.norm(P + s[:, None] * d1 - (a + t[:, None] * d2), axis=1)
    return s, h


def segment_crosses_box(P, X, centre, axes, half, shrink: float = 1e-4):
    """Does the segment P->X (n legs) pass through the oriented box (slab
    test, the box shrunk by ``shrink`` so a leg ending on a face does not)?"""
    p = np.einsum("jk,ik->ij", axes, P - centre)
    d = np.einsum("jk,ik->ij", axes, X - P)
    hh = np.maximum(half - shrink, 0.0)
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-hh - p) / d
        t2 = (hh - p) / d
    par = np.abs(d) < 1e-300
    tlo = np.where(par, np.where(np.abs(p) <= hh, -np.inf, np.inf), np.minimum(t1, t2))
    thi = np.where(par, np.where(np.abs(p) <= hh, np.inf, -np.inf), np.maximum(t1, t2))
    t0, t1_ = tlo.max(axis=1), thi.min(axis=1)
    return (t0 < t1_) & (t1_ > 0.0) & (t0 < 1.0)


class EdgeDiffraction:
    """``diffraction.method: edge``: the closed-form leg factor of the
    nearest free straight edge (module comment above); the CPU reference of
    ``gpu_render._edge_factor_k``."""

    def __init__(self, rr, lam: float, n1: float, n2: float, end_margin_m: float) -> None:
        self.lam = float(lam)
        self.n1, self.n2 = float(n1), float(n2)
        self.end_margin = float(end_margin_m)
        self.edges = free_edges(rr)
        self.reach = float(rr.r_max) + 1.0

    def leg_factors(self, P, X, blocked, lit_ok, chunk: int = 20000):
        """Legs P -> X[i]: ``(idx, D)`` for those an edge diffracts.
        ``blocked``: the leg is shadowed; ``lit_ok``: a clear leg that may
        take a lit-side factor (untouched by transmission)."""
        E = self.edges
        near = E.near(P, self.reach)
        cand = np.flatnonzero(blocked | lit_ok)
        if near.size == 0 or cand.size == 0:
            return np.zeros(0, np.int64), np.zeros(0, np.complex128)
        used = np.unique(E.box[near])
        out_i, out_d = [], []
        for c0 in range(0, cand.size, chunk):
            idx = cand[c0:c0 + chunk]
            Xi = X[idx]
            n = idx.size
            Pn = np.broadcast_to(P, (n, 3))
            L = np.linalg.norm(Xi - P, axis=1)
            cross = np.zeros((n, used.size), dtype=bool)
            for j, bx in enumerate(used):
                cross[:, j] = segment_crosses_box(Pn, Xi, E.centre[bx], E.axes[bx], E.half[bx])
            shadow = blocked[idx]
            ncross = cross.sum(axis=1)
            own = np.where(ncross == 1, used[np.argmax(cross, axis=1)], -1)
            best_s = np.full(n, np.inf)
            for e in near:
                s, h = segments_closest(Pn, Xi, np.broadcast_to(E.a[e], (n, 3)),
                                        np.broadcast_to(E.b[e], (n, 3)))
                d1 = s * L
                d2 = L - d1
                ok = (d1 > self.end_margin) & (d2 > self.end_margin)
                ok &= np.where(shadow, own == E.box[e], ncross == 0)
                deff = d1 * d2 / np.maximum(L, 1e-300)
                z = np.where(ok, h * h / (self.lam * np.maximum(deff, 1e-300)), np.inf)
                best_s = np.minimum(best_s, z)
            hit = best_s < self.n2
            if not hit.any():
                continue
            s_ = best_s[hit]
            v = np.sqrt(2.0 * s_) * np.where(shadow[hit], 1.0, -1.0)
            F = fresnel_F(v)
            w = edge_window(s_, self.n1, self.n2)
            out_i.append(idx[hit])
            out_d.append(np.where(shadow[hit], w * F, 1.0 + w * (F - 1.0)))
        if not out_i:
            return np.zeros(0, np.int64), np.zeros(0, np.complex128)
        return np.concatenate(out_i), np.concatenate(out_d)
