"""Two-way multipath: every contributor through every pair of one-way
eigenray chains (``raycast_design.md`` §2, §2.2; Phase 2 "surface", "bottom",
"wall", "comb").

For a contributor at X and the chain set {c_0 = direct, c_1, ...} (surface,
floor, wall faces and their alternations, ``chains.py``), the two-way field is
the sum over ordered pairs (out c_i, back c_j):

    A_ij = b(d_i) b(d_j) F_i F_j exp(-alpha (L_i + L_j) ln10/20) / (L_i L_j) a_ij

* ``b``: the aperture pattern at each chain's departure direction (one pose
  until Phase 3 gives the receiver its own);
* ``F``: the chain's boundary factors; ``L``: its unfolded length;
* ``a_ij``: the contributor's amplitude for incidence along ``c_i``'s arrival
  and scattering back along ``c_j``: ``f sqrt(sigma_bistatic dA)`` for a
  diffuse scatterer (one world-fixed factor ``f`` for all of its pairs, so
  its paths interfere -- Lloyd's mirror, the image comb -- while different
  scatterers stay incoherent: ``BinGrid.add_group``), the bistatic PO patch
  amplitude for a smooth solid (``specular.py`` normalisation).

**Which chains.** The sequences up to ``max_order`` bounces are enumerated by
length. A sequence is kept while its energy bound -- the sum over
contributors of ``|b F / L|^2 (best partner)^2 sigma dA`` per range bin,
before any visibility ray -- exceeds ``(noise / TVG + reverberation) -
cutoff_db`` in some bin (design §2.2); the highest order kept is reported.
Visibility rays are cast only for kept sequences.

**No double counting.** A physical path is one scattering event plus
specular bounces. For the floor's mean field (``specular.py``), which is
itself the floor's specular reflection, the PO bounce is the FIRST floor
bounce in time: its out chains carry no floor bounce (only the surface
image), its back chains any surface/floor alternation. A chain whose last
bounce is the contributor's own surface (a seabed scatterer's floor, a wall
scatterer's own face) is degenerate and dropped.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

import numpy as np

from . import chains as ch
from . import propagation as _prop
from .receiver import BinGrid
from .scene_mesh import KIND_APRON, KIND_ROUGH_OBJECT, KIND_SEABED, KIND_WALL

LN10_OVER_20 = np.log(10.0) / 20.0


def _cell_key(kx: np.ndarray, ky: np.ndarray) -> np.ndarray:
    """One positive int64 per (kx, ky) neighbour cell (|k| < 2^30: cells
    >= 1 mm over 1000 km; positive, so signed and unsigned orders agree), the
    same packing as the GPU pipeline's."""
    return ((np.asarray(kx, np.int64) + (1 << 30)) << 32) | (np.asarray(ky, np.int64) + (1 << 30))


def po_pair(nrm, pu, pv, pa, pb, inc, out, k: float, refl=1.0,
            rms: float = 0.0) -> np.ndarray:
    """Bistatic Kirchhoff amplitude of planar patches (``facets.po_bistatic``
    integral, ``i k / 2 pi`` prefactor of this renderer's convention): incident
    propagation direction ``inc``, leaving direction ``out``, the material
    reflection ``refl`` and the Eckart amplitude of RMS ``rms``. No beam, no
    spreading, no carrier."""
    q = k * (inc - out)
    qu = np.einsum("ij,ij->i", pu, q)
    qv = np.einsum("ij,ij->i", pv, q)
    nd = np.einsum("ij,ij->i", nrm, out - inc)
    front = ((-np.einsum("ij,ij->i", nrm, inc) > 0.0)
             & (np.einsum("ij,ij->i", nrm, out) > 0.0))
    ar = pa * pb * np.sinc(0.5 * qu * pa / np.pi) * np.sinc(0.5 * qv * pb / np.pi)
    po = (1j * k / (2.0 * np.pi)) * np.where(front, 0.5 * nd, 0.0) * ar * refl
    if rms > 0.0:
        g = k * np.abs(nd) * rms
        po = po * np.exp(-0.5 * np.minimum(g * g, 1400.0))
    return po


class Multipath:
    """The multipath stage of one :class:`RaycastRenderer`."""

    def __init__(self, rr) -> None:
        self.rr = rr
        pc = rr._rc.paths
        self.pc = pc
        self.lam = 2.0 * np.pi / rr.k
        sheet = rr.specular
        if sheet is None:
            from .specular import SheetSpecular
            sc = rr._rc.specular
            sheet = SheetSpecular(rr.mesh, rr._scene, rr.k, rr.freq_hz,
                                  sc.window_n1, sc.window_n2,
                                  sc.patch_phase_rad, sc.patch_min_m,
                                  sc.patch_max_m,
                                  sediments=getattr(rr.law, "sediments", None),
                                  lib=rr.lib,
                                  legacy_phase=bool(rr._rc.paths.legacy_reflection_phase))
        self.sheet = sheet
        self.surface = ch.SurfaceBoundary(rr.k, pc.surface_rms_m)
        self.floor = ch.FloorBoundary(sheet) if pc.floor_bounces else None
        self.faces, self.face_wall = self._wall_faces() if pc.walls else ([], [])
        self.object_faces = self._object_faces() if pc.object_faces else []
        self.last_report: dict = {}
        dc = rr._rc.diffraction
        self.diffraction = None
        if dc.enabled and dc.method == "edge":
            # the fast variant's closed-form straight edges (diffraction.py)
            from .diffraction import EdgeDiffraction
            self.diffraction = EdgeDiffraction(rr, self.lam, dc.window_n1, dc.window_n2,
                                               dc.end_margin_m)
        elif dc.enabled:
            from .diffraction import Diffraction, FresnelDisc
            self.diffraction = Diffraction(
                rr.oracle, self.lam,
                FresnelDisc(dc.window_n1, dc.window_n2, dc.sectors, dc.coarse,
                            dc.bisect), end_margin_m=dc.end_margin_m)

    # ------------------------------------------------------------ geometry
    def _wall_faces(self):
        """The four vertical faces of every wall's Gazebo box (the scene's
        wall triangles), wet part only, as planar bounce boundaries with the
        wall material's interface (concrete, ``_ACOUSTIC``) and RMS."""
        rr = self.rr
        mesh = rr.mesh
        faces, owner = [], []
        surf = self.sheet.surf
        walls = getattr(rr._scene, "walls", None) or []
        if not walls:
            return faces, owner
        n_all = mesh.normals()
        for wi, w in enumerate(walls):
            tri = np.flatnonzero(mesh.wall_index == wi)
            if tri.size == 0:
                continue
            mi = int(mesh.material[tri[0]])
            refl = (lambda s, _m=mi: surf.reflection(_m, s))
            # the wall's own face roughness (design §5; Wall.roughness_m)
            rms = float(getattr(w, "roughness_m", 0.0) or 0.0)
            nrm = n_all[tri]
            vert = np.abs(nrm[:, 2]) < 1e-6
            # group the vertical triangles by normal (two per face)
            done = np.zeros(tri.size, dtype=bool)
            for a in np.flatnonzero(vert):
                if done[a]:
                    continue
                same = vert & (np.einsum("ij,j->i", nrm, nrm[a]) > 1.0 - 1e-9)
                done |= same
                pts = mesh.vertices[mesh.triangles[tri[same]]].reshape(-1, 3)
                n = nrm[a]
                v = np.array([0.0, 0.0, 1.0])
                u = np.cross(v, n)
                u = u / np.linalg.norm(u)
                cu = pts @ u
                z0, z1 = float(pts[:, 2].min()), min(float(pts[:, 2].max()), 0.0)
                if z1 <= z0:
                    continue
                c = (0.5 * (cu.min() + cu.max()) * u
                     + float(pts[0] @ n) * n + np.array([0, 0, 0.5 * (z0 + z1)]))
                faces.append(ch.FaceBoundary(
                    f"{wi}.{len(owner)}", c, n, u, v, 0.5 * (cu.max() - cu.min()),
                    0.5 * (z1 - z0), refl, rms, rr.k))
                faces[-1].material = mi          # the interface, for the GPU tables
                owner.append(wi)
        return faces, owner

    def _object_faces(self):
        """Planar faces of every smooth solid (coplanar triangle groups at
        least 2 cm across), as GO bounce boundaries for back chains (corner
        paths, Hwang et al. 2015): convex-hull polygon, the object's own
        reflection and RMS."""
        rr = self.rr
        mesh = rr.mesh
        from .scene_mesh import KIND_SMOOTH_OBJECT
        nrm_all = mesh.normals()
        faces = []
        # an object that is one solid: a compound's many small faces are not
        # tracked as bounce mirrors
        idx_of = {int(o.object_id): i for i, o in enumerate(rr._facet_objs)
                  if rr._facet_faces[i]}
        for oid in np.unique(mesh.object_id[mesh.kind == KIND_SMOOTH_OBJECT]):
            if int(oid) not in idx_of:
                continue
            i = idx_of[int(oid)]
            tri = np.flatnonzero((mesh.kind == KIND_SMOOTH_OBJECT)
                                 & (mesh.object_id == oid))
            nrm = nrm_all[tri]
            off = np.einsum("ij,ij->i", nrm, mesh.vertices[mesh.triangles[tri, 0]])
            done = np.zeros(tri.size, dtype=bool)
            for a in range(tri.size):
                if done[a]:
                    continue
                same = (~done & (np.einsum("ij,j->i", nrm, nrm[a]) > 1 - 1e-9)
                        & (np.abs(off - off[a]) < 1e-6))
                done |= same
                pts = mesh.vertices[mesh.triangles[tri[same]]].reshape(-1, 3)
                n = nrm[a]
                t = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
                u = np.cross(n, t)
                u /= np.linalg.norm(u)
                v = np.cross(n, u)
                c = pts.mean(axis=0)
                q = np.column_stack([(pts - c) @ u, (pts - c) @ v])
                if np.ptp(q[:, 0]) < 0.02 or np.ptp(q[:, 1]) < 0.02:
                    continue
                hull = _hull2d(q)
                R = float(rr._facet_R[i])
                faces.append(ch.FaceBoundary(
                    f"{int(oid)}.{len(faces)}", c, n, u, v, np.inf, np.inf,
                    (lambda s_, _R=R: np.full(np.shape(s_), _R, dtype=complex)),
                    float(rr._facet_sigma[i]), rr.k, kind="O", poly=hull,
                    owner=int(oid)))
                faces[-1].rconst = R             # the constant reflection, for the GPU tables
        return faces

    def boundaries(self, P: np.ndarray) -> list:
        out = [self.surface]
        if self.floor is not None:
            out.append(self.floor)
        rmax = self.rr.r_max
        for f in self.faces:
            d = P - f.c
            du = np.clip(d @ f.u, -f.hu, f.hu)
            dv = np.clip(d @ f.v, -f.hv, f.hv)
            near = f.c + du * f.u + dv * f.v
            if np.linalg.norm(P - near) <= rmax and (P - f.c) @ f.n > 0.0:
                out.append(f)
        return out

    def _threshold(self, grid: BinGrid, level: np.ndarray) -> np.ndarray:
        """Per bin: (noise before TVG + level) x 10^(-cutoff/10)."""
        rr = self.rr
        r_dev = grid.start_m + (np.arange(grid.n) + 0.5) * grid.bin_m
        tvg = np.maximum(r_dev, 1e-3) ** (float(rr._rc.receiver.tvg_k) / 10.0)
        noise = float(rr._cfg.noise_floor) / tvg
        return (noise + level) * 10.0 ** (-float(self.pc.cutoff_db) / 10.0)

    def _bins(self, grid: BinGrid, tau: np.ndarray) -> np.ndarray:
        return np.clip(np.floor((0.5 * grid.device_c * tau - grid.start_m)
                                / grid.bin_m).astype(np.int64), 0, grid.n - 1)

    def _own_mask(self, seq: list, on_floor: np.ndarray,
                  wall_of: np.ndarray) -> np.ndarray:
        """Contributors for which ``seq`` is degenerate: its last bounce is
        their own surface (a seabed scatterer's floor, a wall scatterer's own
        wall)."""
        bad = np.zeros(on_floor.size, dtype=bool)
        if not seq:
            return bad
        last = seq[-1]
        if last is self.floor:
            bad |= on_floor
        elif last.kind == "W":
            wi = self.face_wall[self.faces.index(last)]
            bad |= wall_of == wi
        return bad

    def _build(self, P, X, seq, bad, refine: bool = True) -> ch.Chain:
        """``chains.build`` for the contributors not in ``bad``; the others
        get a zero factor and are invisible (never built)."""
        n = X.shape[0]
        if not bad.any():
            return ch.build(P, X, seq, None, self.lam, refine=refine)
        idx = np.flatnonzero(~bad)
        c = ch.build(P, X[idx], seq, None, self.lam, refine=refine)
        def full(a, fill=0.0):
            out = np.full((n,) + a.shape[1:], fill, dtype=a.dtype)
            out[idx] = a
            return out
        return ch.Chain(c.seq, full(c.length, np.inf), full(c.depart),
                        full(c.arrive), full(c.factor), full(c.visible, False),
                        [full(q) for q in c.bounces], c.iterations, c.via,
                        [full(q) for q in c.normals], c.order)

    def diffract_direct(self, P, X, c: ch.Chain, cand: np.ndarray) -> dict:
        """Fresnel-Kirchhoff on the direct chain: the blocked legs among
        ``cand`` and the clear ones whose contributor lies near a blocked
        one (the lit side of the edge). ``c.D`` gets the complex factor
        (1 clear, 0 blocked where not evaluated); ``c.visible`` is opened
        where |D| > 0."""
        n = X.shape[0]
        c.D = np.where(c.visible, 1.0 + 0j, 0.0 + 0j)
        dif = self.diffraction
        if dif is None:
            return {"diffracted": 0}
        from .diffraction import EdgeDiffraction
        if isinstance(dif, EdgeDiffraction):
            Tl = getattr(c, "T", None)
            untouched = np.ones(n, dtype=bool) if Tl is None else (Tl == 1.0)
            idx, D = dif.leg_factors(P, X, cand & ~c.visible, cand & c.visible & untouched)
            c.D[idx] = D
            c.visible[idx] = np.abs(D) > 0.0
            return {"diffracted": int(idx.size), "edges": int(np.sum(
                (np.abs(D) > 0) & (np.abs(D) != 1.0)))}
        blocked = np.flatnonzero(cand & ~c.visible)
        if blocked.size == 0:
            return {"diffracted": 0}
        d = X[blocked] - P
        L = np.linalg.norm(d, axis=1)
        h = self.rr.oracle.closest(np.broadcast_to(P, d.shape), d / L[:, None],
                                   t_max=L - 1e-6, t_min=1e-6)
        t_hit = np.where(np.isfinite(h.t), h.t, 0.5 * L)
        frac_b = np.clip(t_hit / L, 0.02, 0.98)
        # lit-side candidates: clear legs within a cell of a blocked one, each
        # taking the nearest obstruction among those blocked neighbours (the
        # smallest hit fraction: order-free, so the GPU pipeline draws the
        # same set and planes)
        cell = float(self.rr._rc.diffraction.neighbour_cell_m)
        key_b = np.floor(X[blocked, :2] / cell).astype(np.int64)
        off = np.array([(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)], np.int64)
        k9 = (key_b[:, None, :] + off[None, :, :]).reshape(-1, 2)
        lk = _cell_key(k9[:, 0], k9[:, 1])
        f9 = np.repeat(frac_b, 9)
        o = np.lexsort((f9, lk))
        lk, f9 = lk[o], f9[o]
        first = np.r_[True, lk[1:] != lk[:-1]]
        uk, uf = lk[first], f9[first]                 # sorted keys, min fraction each
        Tl = getattr(c, "T", None)
        untouched = np.ones(n, dtype=bool) if Tl is None else (Tl == 1.0)
        clear = np.flatnonzero(cand & c.visible & untouched)
        kc = np.floor(X[clear, :2] / cell).astype(np.int64)
        qk = _cell_key(kc[:, 0], kc[:, 1])
        pos = np.clip(np.searchsorted(uk, qk), 0, max(uk.size - 1, 0))
        hit = (uk.size > 0) & (uk[pos] == qk)
        fr_c = np.where(hit, uf[pos], -1.0)
        lit = clear[fr_c > 0]
        idx = np.concatenate([blocked, lit])
        frac = np.concatenate([frac_b, fr_c[fr_c > 0]])
        Lall = np.linalg.norm(X[idx] - P, axis=1)
        D = dif.leg_factors(np.broadcast_to(P, (idx.size, 3)), X[idx],
                            frac * Lall)
        c.D[idx] = D
        c.visible[idx] = np.abs(D) > 0.0
        return {"diffracted": int(idx.size), "edges": int(np.sum(
            (np.abs(D) > 0) & (np.abs(D) != 1.0)))}

    def _bound(self, c: ch.Chain, e_a, e_n, e_v, grid: BinGrid, direct,
               best: np.ndarray, amp0: np.ndarray):
        """(pattern b, one-way amplitude, per-bin energy bound) of chain
        ``c``, or None when no contributor has a live factor."""
        rr = self.rr
        live = np.flatnonzero(np.abs(c.factor) > 0.0)
        if live.size == 0:
            return None
        b = np.zeros(c.length.shape[0], dtype=complex)
        # near field: the image transducer radiates toward X along the
        # departure direction over the UNFOLDED length
        b[live] = rr.aperture.pattern(
            c.depart[live] * c.length[live][:, None], e_a, e_n, e_v,
            rr._rc.beam.near_field)
        one = np.abs(b * c.factor) / np.maximum(c.length, 1e-9)
        Lp = np.where(np.isfinite(c.length), c.length, 0.0)
        absb = np.exp(-rr.alpha * LN10_OVER_20 * (Lp + direct.length))
        pw = (one * best * absb * amp0) ** 2
        bins = self._bins(grid, (Lp + direct.length) / rr.c)
        return b, one, np.bincount(bins, weights=pw, minlength=grid.n)

    def select_chains(self, P, X, e_a, e_n, e_v, grid, direct, level, amp0,
                      on_floor, wall_of, seqs_filter=None, extra=None,
                      max_obj: int = 0) -> tuple:
        """Chains kept by the energy bound, beside the prebuilt ``direct``
        chain (pattern ``b`` attached). ``amp0``: each contributor's
        scattering-amplitude bound (sqrt(sigma dA) or |PO|); ``level``: the
        per-bin reference (the direct reverberation). Returns (kept chains,
        each with its pattern ``b``, report)."""
        rr = self.rr
        bnds = self.boundaries(P) + list(extra or [])
        levels = ch.sequences_by_order(bnds, int(self.pc.max_order),
                                       int(self.pc.max_wall_bounces), int(max_obj))
        n_seq = 0
        kept = [(direct, [])]
        best = np.abs(direct.b) / np.maximum(direct.length, 1e-9)
        thr = self._threshold(grid, level)
        bound_sum = np.zeros(grid.n)
        order_reached = 0
        n_dropped = 0
        for order, level_seqs in enumerate(levels, start=1):
            any_kept = False
            if seqs_filter is not None:
                level_seqs = [s for s in level_seqs if seqs_filter(s)]
            n_seq += len(level_seqs)
            for seq in level_seqs:
                bad = self._own_mask(seq, on_floor, wall_of)
                if bad.all():
                    n_dropped += 1
                    continue
                # the bound on the flat-floor image solution; the Newton
                # refinement (most of the cost) only for a kept sequence
                floors = any(bd.refine() for bd in seq)
                c = self._build(P, X, seq, bad, refine=False)
                bound = self._bound(c, e_a, e_n, e_v, grid, direct, best, amp0)
                if bound is None:
                    n_dropped += 1
                    continue
                b, one, seq_sum = bound
                if np.any(seq_sum > thr):
                    if floors:
                        c = self._build(P, X, seq, bad)
                        b, one, seq_sum = self._bound(c, e_a, e_n, e_v, grid,
                                                      direct, best, amp0) or (
                            np.zeros(X.shape[0], dtype=complex),
                            np.zeros(X.shape[0]), np.zeros(grid.n))
                    c.b = b
                    kept.append((c, seq))
                    bound_sum += seq_sum
                    best = np.maximum(best, one)
                    any_kept = True
                else:
                    n_dropped += 1
            if not any_kept:
                break
            order_reached = order
            thr = self._threshold(grid, level + bound_sum)
        return kept, dict(order_reached=order_reached, sequences=n_seq,
                          kept=len(kept) - 1, dropped=n_dropped,
                          via=[c.via for c, _ in kept[1:]])

    # ------------------------------------------------------------- diffuse
    def diffuse(self, P, e_a, e_n, e_v, grid: BinGrid,
                chunk: int = 60_000) -> dict:
        """Every diffuse scatterer through every kept chain pair."""
        rr = self.rr
        s = rr._scatterers(P, e_a, e_n)
        if len(s) == 0:
            return {"contributors": 0}
        d = s.pos - P
        R = np.linalg.norm(d, axis=1)
        keep = (R > 1e-3) & (R <= rr.r_max)
        level = np.zeros(grid.n)
        idx = np.flatnonzero(keep)
        X = s.pos[idx]
        nrm = s.normal[idx]
        mat = s.material[idx]
        area = s.area[idx]
        f = s.rayleigh[idx] * np.exp(1j * s.phase[idx])
        kind = s.kind[idx]
        on_floor = np.isin(kind, (KIND_SEABED, KIND_ROUGH_OBJECT, KIND_APRON))
        tri = s.tri[idx]
        wall_of = np.where(kind == KIND_WALL,
                           rr.mesh.wall_index[np.maximum(tri, 0)], -1)
        # scattering-amplitude bound: the monostatic sigma at the direct
        # geometry (bistatic pairs near backscatter for horizontal images)
        cos_i = np.clip(np.einsum("ij,ij->i", nrm, -d[idx]) / R[idx], 0.0, 1.0)
        sig0 = np.maximum(rr.law.sigma(cos_i, cos_i, mat), 0.0)
        amp0 = np.sqrt(sig0 * area)
        offset = rr._rc.sampling.surface_offset_m
        Xv = X + offset * nrm
        direct = ch.build(P, X, [], None, self.lam)
        direct.b = rr.aperture.pattern(X - P, e_a, e_n, e_v,
                                       rr._rc.beam.near_field)
        ch.check_visibility(P, Xv, direct, rr.oracle, idx=np.flatnonzero(
            (np.abs(direct.b) ** 4 >= rr._floor) & (cos_i > 0.0)))
        cand = (np.abs(direct.b) ** 4 >= rr._floor) & (cos_i > 0.0)
        direct.visible &= cand
        drep = self.diffract_direct(P, Xv, direct, cand)
        # the reference level: the direct reverberation per bin
        p0 = np.where(direct.visible, (np.abs(direct.D) * np.abs(direct.b) ** 2 * amp0
                                       * _prop.leg_amplitude(direct.length, rr.alpha) ** 2) ** 2, 0.0)
        level = level + np.bincount(self._bins(grid, 2.0 * direct.length / rr.c),
                                    weights=p0, minlength=grid.n)
        kept, rep = self.select_chains(P, X, e_a, e_n, e_v, grid, direct,
                                       level, amp0, on_floor, wall_of)
        for c, seq in kept[1:]:
            ch.check_visibility(P, Xv, c, rr.oracle,
                                idx=np.flatnonzero(c.visible & (np.abs(c.b) > 0)))
        floor_db = rr._floor
        n_pairs = 0
        for a in range(0, X.shape[0], chunk):
            sl = slice(a, min(a + chunk, X.shape[0]))
            dets, taus, groups = [], [], []
            # reciprocity: the pair (j, i) has the pair (i, j)'s cross-section
            # (sigma_bistatic is reciprocal to 1e-15), length and amplitude, so
            # it is folded into (i, j) with weight 2 -- the same field and the
            # same coherent group sum at half the pairs
            for ii, (ci, si_) in enumerate(kept):
                for cj, sj in kept[ii:]:
                    wgt = 1.0 if cj is ci else 2.0
                    ok = ci.visible[sl] & cj.visible[sl]
                    two = ci.b[sl] * cj.b[sl]
                    ok &= np.abs(two) ** 2 >= floor_db
                    if not ok.any():
                        continue
                    j = np.flatnonzero(ok)
                    g = a + j
                    sig = rr.law.sigma_bistatic(nrm[g], ci.arrive[g],
                                                -cj.arrive[g], mat[g])
                    L = ci.length[g] + cj.length[g]
                    dd = 1.0
                    for c_ in (ci, cj):
                        for att in ("D", "T"):
                            a_ = getattr(c_, att, None)
                            if a_ is not None:
                                dd = dd * a_[g]
                    det = (wgt * dd * two[j] * ci.factor[g] * cj.factor[g]
                           * np.exp(-rr.alpha * LN10_OVER_20 * L)
                           / (ci.length[g] * cj.length[g])
                           * np.sqrt(sig * area[g]))
                    nz = det != 0.0
                    dets.append(det[nz])
                    taus.append(L[nz] / rr.c)
                    groups.append(g[nz])
            if dets:
                det = np.concatenate(dets)
                n_pairs += det.size
                grp = np.concatenate(groups)
                grid.add_group(det, f[grp], np.concatenate(taus), grp)
        rep.update(contributors=int(X.shape[0]), pairs=int(n_pairs), **drep)
        self.last_report["diffuse"] = rep
        return rep

    # --------------------------------------------------------- smooth solids
    def facets(self, P, e_a, e_n, e_v, grid: BinGrid, level: np.ndarray) -> dict:
        """The PO patches of every smooth solid in reach through every kept
        chain pair (bistatic Kirchhoff, coherent)."""
        rr = self.rr
        k = rr.k
        n_dep = 0
        reps = []
        for i, o in enumerate(rr._facet_objs):
            c0 = np.array([o.x, o.y, 0.0])
            reach = float(np.hypot(o.length, o.width)) + o.proud_height
            if np.hypot(*(c0[:2] - P[:2])) - reach > rr.r_max:
                continue
            p = rr._patches(i)
            if p is None or len(p) == 0:
                continue
            X = p.centre
            R = np.linalg.norm(X - P, axis=1)
            sel = np.flatnonzero(R <= rr.r_max)
            if sel.size == 0:
                continue
            X = X[sel]
            nrm, pu, pv, pa, pb = (p.normal[sel], p.u[sel], p.v[sel],
                                   p.a[sel], p.b[sel])
            refl = float(rr._facet_R[i])
            rms = float(rr._facet_sigma[i])
            rtab = rr._facet_tab[i]
            amp0 = (k / (2.0 * np.pi)) * pa * pb * refl
            none = np.zeros(X.shape[0], dtype=bool)
            direct = ch.build(P, X, [], None, self.lam)
            direct.b = rr.aperture.pattern(X - P, e_a, e_n, e_v,
                                           rr._rc.beam.near_field)
            oid = int(o.object_id)
            extra = [f for f in self.object_faces if f.owner != oid
                     and np.linalg.norm(f.c - P) <= rr.r_max]
            kept, rep = self.select_chains(P, X, e_a, e_n, e_v, grid, direct,
                                           level, amp0, none,
                                           np.full(X.shape[0], -1), extra=extra,
                                           max_obj=int(self.pc.max_object_bounces))
            reps.append(rep)
            Xv = X + rr._rc.sampling.surface_offset_m * nrm
            for c, seq in kept:
                ch.check_visibility(P, Xv, c, rr.oracle,
                                    idx=np.flatnonzero(c.visible & (np.abs(c.b) > 0)))
            # PO at the first bounce in time: object faces only after it
            outs = [(c_, q_) for c_, q_ in kept if all(b.kind != "O" for b in q_)]
            for ci, _ in outs:
                for cj, _ in kept:
                    ok = ci.visible & cj.visible
                    two = ci.b * cj.b
                    ok &= np.abs(two) ** 2 >= rr._floor
                    if not ok.any():
                        continue
                    j = np.flatnonzero(ok)
                    rf = refl
                    if rtab is not None:    # a shell wall: |R| at the half-angle of the pair
                        dv = -cj.arrive[j] - ci.arrive[j]
                        ch_ = np.einsum("ij,ij->i", nrm[j], dv) / np.maximum(
                            np.linalg.norm(dv, axis=1), 1e-12)
                        rf = float(rr._facet_gain[i]) * rtab.reflection(ch_)
                    po = po_pair(nrm[j], pu[j], pv[j], pa[j], pb[j],
                                 ci.arrive[j], -cj.arrive[j], k, rf, rms)
                    L = ci.length[j] + cj.length[j]
                    tt = 1.0
                    for c_ in (ci, cj):
                        a_ = getattr(c_, "T", None)
                        if a_ is not None:
                            tt = tt * a_[j]
                    amp = (tt * two[j] * ci.factor[j] * cj.factor[j] * po
                           * np.exp(-rr.alpha * LN10_OVER_20 * L)
                           / (ci.length[j] * cj.length[j]))
                    nz = amp != 0.0
                    grid.add(amp[nz], L[nz] / rr.c, diffuse=False)
                    n_dep += int(nz.sum())
        self.last_report["facets"] = reps
        return {"deposits": n_dep}

    # ------------------------------------------------- the floor's mean field
    def floor_coherent(self, P, e_a, e_n, e_v, grid: BinGrid,
                       level: np.ndarray, beam: bool = True,
                       visibility: bool = True) -> dict:
        """The seabed sheet's mean field through transducer images: out
        chains without floor bounces (direct, surface), back chains any
        surface/floor alternation (the PO bounce is the first floor bounce
        in time, so no physical path counts twice)."""
        rr = self.rr
        sp = self.sheet
        bnds = [self.surface] + ([self.floor] if self.floor is not None else [])
        seqs = ch.sequences(bnds, int(self.pc.max_order), 0)
        outs = [s for s in seqs if all(b is self.surface for b in s) and len(s) <= 1]
        backs = [s for s in seqs if not (s and s[-1] is self.floor)]
        bbox = rr._band_bbox(P, e_a, e_n)
        thr = self._threshold(grid, level)
        # provisional specular point: the sheet under P
        z0, _, _ = sp._locate(np.array([P[0]]), np.array([P[1]]))
        X0 = np.array([[P[0], P[1], float(z0[0])]])
        n_dep, pairs, top = 0, 0, 0
        for order in range(0, int(self.pc.max_order) + 1):
            kept_any = False
            for so in outs:
                for sb in backs:
                    if len(so) + len(sb) != order:
                        continue
                    co = ch.build(P, X0, so, None, self.lam)
                    cb = ch.build(P, X0, sb, None, self.lam)
                    if not (np.abs(co.factor[0]) > 0 and np.abs(cb.factor[0]) > 0):
                        continue
                    A_img = X0[0] - co.arrive[0] * co.length[0]
                    B_img = X0[0] - cb.arrive[0] * cb.length[0]
                    p = sp.patches(A_img, B_img, bbox)
                    if len(p) == 0:
                        continue
                    co = ch.build(P, p.centre, so, None, self.lam)
                    cb = ch.build(P, p.centre, sb, None, self.lam)
                    amp, w = sp.amplitudes_dirs(p, co.arrive, -cb.arrive,
                                                co.length, cb.length)
                    live = np.flatnonzero((w > 0) & (amp != 0)
                                          & (np.abs(co.factor) > 0)
                                          & (np.abs(cb.factor) > 0)
                                          & (co.length <= rr.r_max)
                                          & (cb.length <= rr.r_max))
                    if live.size == 0:
                        continue
                    if beam:
                        bo = rr.aperture.pattern(
                            co.depart[live] * co.length[live][:, None], e_a,
                            e_n, e_v, rr._rc.beam.near_field)
                        bb = rr.aperture.pattern(
                            cb.depart[live] * cb.length[live][:, None], e_a,
                            e_n, e_v, rr._rc.beam.near_field)
                    else:
                        bo = bb = np.ones(live.size)
                    L = co.length[live] + cb.length[live]
                    a = (amp[live] * bo * bb * co.factor[live] * cb.factor[live]
                         * np.exp(-rr.alpha * LN10_OVER_20 * L)
                         / (co.length[live] * cb.length[live]))
                    # bound: |sum| of the pair vs the threshold at its bin
                    tau = L / rr.c
                    est = abs(a.sum()) ** 2
                    b_ = int(self._bins(grid, np.array([np.median(tau)]))[0])
                    if order > 0 and est < thr[b_]:
                        continue
                    Xv = p.centre[live] + rr._rc.sampling.surface_offset_m * p.normal[live]
                    co.visible[:] = True
                    cb.visible[:] = True
                    sub = np.arange(live.size)
                    co2 = ch.Chain(co.seq, co.length[live], co.depart[live],
                                   co.arrive[live], co.factor[live],
                                   np.ones(live.size, bool),
                                   [q[live] for q in co.bounces], 0, co.via,
                                   [q[live] for q in co.normals], co.order)
                    cb2 = ch.Chain(cb.seq, cb.length[live], cb.depart[live],
                                   cb.arrive[live], cb.factor[live],
                                   np.ones(live.size, bool),
                                   [q[live] for q in cb.bounces], 0, cb.via,
                                   [q[live] for q in cb.normals], cb.order)
                    if visibility:
                        ch.check_visibility(P, Xv, co2, rr.oracle, idx=sub)
                        ch.check_visibility(P, Xv, cb2, rr.oracle, idx=sub)
                    vis = co2.visible & cb2.visible
                    if vis.any():
                        # legs blocked only by flooded thin shells transmit
                        # (check_visibility sets T): the amplitude carries
                        # it, as in the diffuse and facet paths (2026-11-04;
                        # before, a transmitting leg deposited unattenuated)
                        for cc in (co2, cb2):
                            if getattr(cc, "T", None) is not None:
                                a = a * cc.T
                        grid.add(a[vis], tau[vis], diffuse=False)
                        n_dep += int(vis.sum())
                    pairs += 1
                    kept_any = True
            if order > 0 and not kept_any:
                break
            top = order
        rep = dict(pairs=pairs, deposits=n_dep, order_reached=top)
        self.last_report["floor"] = rep
        return rep


def _hull2d(q: np.ndarray) -> np.ndarray:
    """Convex hull (CCW) of 2-D points, Andrew's monotone chain."""
    pts = np.unique(np.round(q, 12), axis=0)
    if pts.shape[0] < 3:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])
