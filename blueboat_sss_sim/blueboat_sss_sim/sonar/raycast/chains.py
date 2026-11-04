"""Specular eigenray chains: the image method over planar and locally planar
boundaries, every leg tested by the one scene oracle (note 02; design §2).

A one-way path from the transducer ``P`` to a contributor ``X`` that reflects
specularly off boundaries ``b_1 .. b_n`` (in order) is found by unfolding:

    I_0 = P,   I_j = M_j(I_{j-1})       (the mirror image in b_j's plane)
    Q_n = b_n  crossing of  I_n -> X,  Q_j = b_j crossing of  I_j -> Q_{j+1}

so ``|X - I_n|`` is the path length and the legs ``P -> Q_1 -> ... -> Q_n ->
X`` are straight (Allen & Berkley 1979; Jensen et al. 2011 §2.4). An image is
valid only where every reflection point lies on its physical face, both of
its legs lie on the water side of that face, and every leg is clear (the
one oracle, so a quay hides what is behind it for every path, ghosts
included -- pass-4 failure B1).

Boundaries:

* ``SurfaceBoundary``: the sea surface z = 0 (exact plane), pressure release
  ``-1`` times the Eckart amplitude ``sqrt(C)``, ``C = exp(-(2 k s sin psi)^2)``
  for the surface RMS ``s`` (``waveguide.eckart_c``);
* ``FloorBoundary``: the seabed sheet. Its reflection point is the
  stationary point of the path length over the sheet (Fermat: a damped
  Newton solve in ``build``, at most ``max_iter`` passes), its plane the
  local tangent plane there (the smoothed sheet normal); the factor is
  ``V(psi) sqrt(C)`` of
  ``specular.MaterialSurface`` -- the same interface and the same
  coherent-sum RMS rule as the floor's mean-field channel, with the Fresnel
  radius of the unfolded path at the bounce;
* ``FaceBoundary``: a planar rectangle (a wall's face, a smooth object's
  planar face), the material's fluid half-space ``V(psi)`` times the Eckart
  amplitude of its own RMS. The reflection point must lie inside the
  rectangle (gate P3: no ghost beyond a wall's end).

The incoherent share ``1 - C`` of each bounce (forward scatter around the
specular direction, a double-scattering term) is not rendered -- an optional
term, off, as for the surface (``paths.py``).

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..waveguide import eckart_c


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-300)


def _reflect(p: np.ndarray, q: np.ndarray, n: np.ndarray) -> np.ndarray:
    """Mirror points ``p`` (n, 3) in the planes through ``q`` with unit
    normals ``n``."""
    d = np.einsum("ij,ij->i", p - q, n)
    return p - 2.0 * d[:, None] * n


# ================================================================ boundaries
class SurfaceBoundary:
    kind = "S"
    name = "surface"

    def __init__(self, k: float, rms_m: float, z: float = 0.0) -> None:
        self.k, self.rms, self.z = float(k), float(rms_m), float(z)

    def plane(self, xy: np.ndarray):
        n = xy.shape[0]
        q = np.column_stack([xy[:, 0], xy[:, 1], np.full(n, self.z)])
        nrm = np.tile([0.0, 0.0, -1.0], (n, 1))      # faces the water
        return q, nrm

    def refine(self) -> bool:
        return False

    def valid(self, Q: np.ndarray) -> np.ndarray:
        return np.ones(Q.shape[0], dtype=bool)

    def factor(self, Q, sin_psi, R_e) -> np.ndarray:
        psi = np.arcsin(np.clip(sin_psi, 0.0, 1.0))
        C = eckart_c(psi, self.rms, self.k) if self.rms > 0.0 else 1.0
        return -np.sqrt(C) * np.ones(Q.shape[0], dtype=complex)


class FloorBoundary:
    kind = "F"
    name = "floor"

    def __init__(self, sheet, max_iter: int = 20) -> None:
        self.sheet = sheet              # specular.SheetSpecular
        self.max_iter = int(max_iter)

    def plane(self, xy: np.ndarray):
        z, tri, vn = self.sheet._locate(xy[:, 0], xy[:, 1])
        return np.column_stack([xy[:, 0], xy[:, 1], z]), vn

    def refine(self) -> bool:
        return True

    def info(self, Q: np.ndarray):
        z, tri, vn = self.sheet._locate(Q[:, 0], Q[:, 1])
        m = self.sheet.mesh
        tri_c = np.maximum(tri, 0)
        mat = np.where(tri >= 0, m.material[tri_c], self.sheet.out_mat)
        dry = np.where(tri >= 0, m.dry[tri_c], False)
        return mat, dry

    def valid(self, Q: np.ndarray) -> np.ndarray:
        mat, dry = self.info(Q)
        return ~dry

    def factor(self, Q, sin_psi, R_e) -> np.ndarray:
        mat, _ = self.info(Q)
        surf = self.sheet.surf
        out = np.zeros(Q.shape[0], dtype=complex)
        for mi in np.unique(mat):
            s = mat == mi
            V = surf.reflection(int(mi), sin_psi[s])
            sig = surf.rms_height(int(mi), R_e[s], self.sheet.lam, self.sheet.dx)
            g = 2.0 * self.sheet.k * sig * sin_psi[s]
            out[s] = V * np.exp(-0.5 * np.minimum(g * g, 1400.0))
        return out


class FaceBoundary:
    """A planar rectangle: centre ``c``, unit normal ``n`` (water side),
    in-plane unit axes ``u``, ``v`` with half extents ``hu``, ``hv``."""

    kind = "W"

    def __init__(self, name: str, c, n, u, v, hu: float, hv: float,
                 reflection, rms_m: float, k: float, kind: str = "W",
                 poly: np.ndarray | None = None, owner: int = -1) -> None:
        self.kind = kind                  # "W" wall face, "O" object face
        self.poly = None if poly is None else np.asarray(poly, float)
        self.owner = int(owner)           # object id of an "O" face
        self.name = name
        self.c = np.asarray(c, float)
        self.n = np.asarray(n, float)
        self.u = np.asarray(u, float)
        self.v = np.asarray(v, float)
        self.hu, self.hv = float(hu), float(hv)
        self.reflection = reflection      # callable sin_psi -> complex V
        self.rms = float(rms_m)
        self.k = float(k)

    def plane(self, xy: np.ndarray):
        n = xy.shape[0]
        return np.tile(self.c, (n, 1)), np.tile(self.n, (n, 1))

    def refine(self) -> bool:
        return False

    def valid(self, Q: np.ndarray) -> np.ndarray:
        d = Q - self.c
        du, dv = d @ self.u, d @ self.v
        if self.poly is not None:
            # inside a convex polygon (vertices CCW in (u, v) about c)
            ok = np.ones(Q.shape[0], dtype=bool)
            pv = self.poly
            for i in range(pv.shape[0]):
                a, b = pv[i], pv[(i + 1) % pv.shape[0]]
                ok &= ((b[0] - a[0]) * (dv - a[1]) - (b[1] - a[1]) * (du - a[0])
                       >= -1e-12)
            return ok
        return (np.abs(du) <= self.hu) & (np.abs(dv) <= self.hv)

    def factor(self, Q, sin_psi, R_e) -> np.ndarray:
        psi = np.arcsin(np.clip(sin_psi, 0.0, 1.0))
        C = eckart_c(psi, self.rms, self.k) if self.rms > 0.0 else 1.0
        return self.reflection(sin_psi) * np.sqrt(C)


# ==================================================================== chains
@dataclass
class Chain:
    """One one-way chain (boundary sequence) to each of n contributors."""

    seq: tuple                  # boundary names
    length: np.ndarray          # (n,)
    depart: np.ndarray          # (n, 3) unit direction leaving P
    arrive: np.ndarray          # (n, 3) unit propagation direction at X
    factor: np.ndarray          # (n,) complex product of boundary factors
    visible: np.ndarray         # (n,) bool
    bounces: list = field(default_factory=list)   # [(n, 3)] reflection points
    iterations: int = 0
    via: str = ""               # ground-truth tag: "surface", "wall:<name>", ...
    normals: list = field(default_factory=list)   # [(n, 3)] bounce-plane normals
    order: int = 0


def _unfold(A: np.ndarray, B: np.ndarray, planes: list):
    """Straight-line unfolding of the segment A -> (planes, in order) -> B.
    Returns (length, [bounce points], ok)."""
    imgs = [A]
    for q, nrm in planes:
        imgs.append(_reflect(imgs[-1], q, nrm))
    L = np.linalg.norm(B - imgs[-1], axis=1)
    pts = [None] * len(planes)
    ok = np.ones(A.shape[0], dtype=bool)
    nxt = B
    for j in range(len(planes) - 1, -1, -1):
        q, nrm = planes[j]
        a = imgs[j + 1]
        d = nxt - a
        den = np.einsum("ij,ij->i", d, nrm)
        t = np.einsum("ij,ij->i", q - a, nrm) / np.where(
            np.abs(den) > 1e-300, den, 1e-300)
        ok &= (t > 1e-9) & (t < 1.0 - 1e-9)
        pts[j] = a + t[:, None] * d
        nxt = pts[j]
    return L, pts, ok


def _image_through(A: np.ndarray, planes: list) -> np.ndarray:
    for q, nrm in planes:
        A = _reflect(A, q, nrm)
    return A


def build(P: np.ndarray, X: np.ndarray, seq: list, oracle=None,
          lam: float = 3.4e-3, eps: float = 1e-6, offset: float = 1e-4,
          contributor_on: object | None = None, refine: bool = True) -> Chain:
    """The chain ``P -> seq[0] -> ... -> seq[-1] -> X`` for every X.

    Planar boundaries (surface, faces) are exact images. Floor bounces are
    the stationary points of the path length over the sheet (Fermat): a
    damped Newton solve on their (x, y) -- analytic gradient with the
    smoothed sheet slope, finite-difference Hessian -- from the flat-floor
    image solution, until the path moves less than ``lambda / 200``; a chain
    that does not converge is dropped (``Chain.iterations`` reports the
    passes). On a floor with several specular points for one (P, X) the
    solve returns the one nearest the flat-floor image point -- one eigenray
    per chain, a stated limitation.

    ``contributor_on``: the boundary X itself lies on (the floor for a
    seabed scatterer): a chain whose last bounce is that boundary is
    degenerate (its reflection point is X) and is rejected.

    ``refine=False`` stops at the flat-floor image solution (no Newton): the
    geometry an energy BOUND needs (``Multipath.select_chains``), exact on a
    flat floor and within the floor's relief otherwise."""
    X = np.atleast_2d(np.asarray(X, float))
    n = X.shape[0]
    P = np.asarray(P, float)
    PP = np.broadcast_to(P, X.shape).copy()
    if not seq:
        d = X - P
        L = np.linalg.norm(d, axis=1)
        u = _unit(d)
        vis = (np.ones(n, dtype=bool) if oracle is None
               else ~oracle.occluded(PP, X, eps))
        return Chain((), L, u, u, np.ones(n, dtype=complex), vis, [], 0, "")
    m = len(seq)
    ok = np.ones(n, dtype=bool)
    if contributor_on is not None and seq[-1] is contributor_on:
        ok[:] = False
    mid = 0.5 * (X[:, :2] + P[None, :2])
    planes = [b.plane(mid) for b in seq]
    fl = [j for j, b in enumerate(seq) if b.refine()]
    # initial: every boundary a plane (floors horizontal under the midpoint)
    _, Q, good = _unfold(PP, X, planes)
    it = 0
    if fl and refine:
        # fixed points: P, the floor bounces, X; planar runs between them
        cuts = [-1] + fl + [m]
        runs = [[planes[k] for k in range(cuts[r] + 1, cuts[r + 1])]
                for r in range(len(cuts) - 1)]
        xy = np.stack([Q[j][:, :2] for j in fl], axis=1)       # (n, f, 2)
        nf = len(fl)
        conv = np.zeros(n, dtype=bool)
        fb = seq[fl[0]]

        def grad(xy_):
            S, zx, zy = [], [], []
            for f in range(nf):
                z, _, vn = fb.sheet._locate(xy_[:, f, 0], xy_[:, f, 1])
                S.append(np.column_stack([xy_[:, f, 0], xy_[:, f, 1], z]))
                zx.append(-vn[:, 0] / vn[:, 2])
                zy.append(-vn[:, 1] / vn[:, 2])
            fixed = [PP] + S + [X]
            G = np.zeros((n, nf, 2))
            L = np.zeros(n)
            for f in range(nf):
                Aj = _image_through(fixed[f], runs[f])
                Bj = _image_through(fixed[f + 2], runs[f + 1][::-1])
                g = _unit(S[f] - Aj) + _unit(S[f] - Bj)
                G[:, f, 0] = g[:, 0] + g[:, 2] * zx[f]
                G[:, f, 1] = g[:, 1] + g[:, 2] * zy[f]
            for r in range(nf + 1):
                L += np.linalg.norm(fixed[r + 1]
                                    - _image_through(fixed[r], runs[r]), axis=1)
            return G, L

        G, Lc = grad(xy)
        h = 2e-3
        maxit = max(b.max_iter for b in seq if b.refine())
        for it in range(1, maxit + 1):
            # already stationary (a flat floor's image solution): no Hessian
            conv |= good & (np.abs(G).reshape(n, -1).max(axis=1) < 1e-12)
            act = ~conv & good
            if not act.any():
                break
            Hm = np.zeros((n, 2 * nf, 2 * nf))
            for c in range(2 * nf):
                e = np.zeros((nf, 2))
                e[c // 2, c % 2] = h
                Gp, _ = grad(xy + e)
                Gm, _ = grad(xy - e)
                Hm[:, :, c] = ((Gp - Gm) / (2 * h)).reshape(n, 2 * nf)
            g2 = G.reshape(n, 2 * nf)
            det_ok = np.abs(np.linalg.det(Hm)) > 1e-12
            Hs = np.where(det_ok[:, None, None], Hm, np.eye(2 * nf)[None])
            step = -np.linalg.solve(Hs, g2[..., None])[..., 0]
            sn = np.linalg.norm(step, axis=1)
            step *= np.minimum(1.0, 0.5 / np.maximum(sn, 1e-300))[:, None]
            step = np.where(act[:, None], step, 0.0)
            new = xy + step.reshape(n, nf, 2)
            Gn, Ln = grad(new)
            # path change predicted by the step: converged below lambda/200
            dL = np.abs(np.einsum("ij,ij->i", g2, step))
            conv |= act & (dL < lam / 200.0) & (np.linalg.norm(step, axis=1) < 1e-3)
            xy, G, Lc = new, Gn, Ln
        good &= conv
        for f, j in enumerate(fl):
            z, _, vn = fb.sheet._locate(xy[:, f, 0], xy[:, f, 1])
            planes[j] = (np.column_stack([xy[:, f, 0], xy[:, f, 1], z]), vn)
        # planar bounce points between the fixed points, exactly
        fixed = [PP] + [planes[j][0] for j in fl] + [X]
        Q = [None] * m
        for r in range(nf + 1):
            idxs = list(range(cuts[r] + 1, cuts[r + 1]))
            if idxs:
                _, pts, okr = _unfold(fixed[r], fixed[r + 1],
                                      [planes[k] for k in idxs])
                good &= okr
                for k, pt in zip(idxs, pts):
                    Q[k] = pt
        for f, j in enumerate(fl):
            Q[j] = planes[j][0]
    # validity: each bounce on its face, legs on the water side
    pts = [PP] + Q + [X]
    for j, b in enumerate(seq):
        q, nrm = planes[j]
        ok &= good & b.valid(Q[j])
        ok &= np.einsum("ij,ij->i", pts[j] - Q[j], nrm) > 0.0
        ok &= np.einsum("ij,ij->i", pts[j + 2] - Q[j], nrm) > 0.0
    legs = [np.linalg.norm(pts[j] - pts[j + 1], axis=1) for j in range(m + 1)]
    cum = np.cumsum(np.vstack(legs), axis=0)
    total = cum[-1]
    fac = np.ones(n, dtype=complex)
    for j, b in enumerate(seq):
        q, nrm = planes[j]
        inc = _unit(Q[j] - pts[j])
        sin_psi = np.clip(-np.einsum("ij,ij->i", inc, nrm), 0.0, 1.0)
        d1 = cum[j]
        d2 = total - d1
        R_e = 2.0 * d1 * d2 / np.maximum(d1 + d2, 1e-300)
        fac = fac * np.where(ok, b.factor(Q[j], sin_psi, R_e), 0.0)
    depart = _unit(Q[0] - P)
    arrive = _unit(X - Q[-1])
    via = "+".join("surface" if b.kind == "S" else
                   ("floor" if b.kind == "F" else
                    (f"wall:{b.name}" if b.kind == "W" else f"object:{b.name}"))
                   for b in seq)
    c = Chain(tuple(b.name for b in seq), total, depart, arrive, fac, ok.copy(),
              Q, it, via, [pl[1] for pl in planes], m)
    if oracle is not None:
        check_visibility(P, X, c, oracle, eps, offset)
    return c


def check_visibility(P, X, c: Chain, oracle, eps: float = 1e-6,
                     offset: float = 1e-4, idx: np.ndarray | None = None
                     ) -> None:
    """Clear ``c.visible`` where any leg is blocked (the scene oracle) or
    leaves the water; only contributors ``idx`` (default: every one still
    visible) are tested."""
    X = np.atleast_2d(np.asarray(X, float))
    P = np.asarray(P, float)
    if idx is None:
        idx = np.flatnonzero(c.visible)
    if idx.size == 0:
        return
    if not c.bounces:
        blocked = oracle.occluded(np.broadcast_to(P, (idx.size, 3)), X[idx], eps)
        c.visible[idx[blocked]] = False
        _transmit_blocked(P, X, c, oracle, eps, offset, idx[blocked])
        return
    prev = np.broadcast_to(P, (idx.size, 3))
    m = len(c.bounces)
    bad = np.zeros(idx.size, dtype=bool)
    for j in range(m + 1):
        if j < m:
            end = c.bounces[j][idx] + offset * c.normals[j][idx]
        else:
            end = X[idx]
        live = ~bad
        if live.any():
            b = oracle.bvh.occluded(prev[live], end[live], eps)
            b |= np.maximum(prev[live][:, 2], end[live][:, 2]) > 1e-9
            bad[np.flatnonzero(live)[b]] = True
        prev = end
    c.visible[idx[bad]] = False
    _transmit_blocked(P, X, c, oracle, eps, offset, idx[bad])


def _transmit_blocked(P, X, c: Chain, oracle, eps, offset, idx) -> None:
    """Legs blocked only by flooded thin shells transmit: ``c.T`` (complex,
    1 elsewhere) gets the product of every leg's layer factors and the
    contributor becomes visible again (``scene_mesh.layer_transmission``)."""
    if getattr(oracle, "layer", None) is None or idx.size == 0:
        return
    if getattr(c, "T", None) is None:
        c.T = np.ones(c.length.size, dtype=complex)
    T = np.ones(idx.size, dtype=complex)
    prev = np.broadcast_to(np.asarray(P, float), (idx.size, 3))
    m = len(c.bounces)
    for j in range(m + 1):
        end = (c.bounces[j][idx] + offset * c.normals[j][idx]) if j < m else X[idx]
        T *= oracle.transmit(prev, end, eps)
        prev = end
    c.T[idx] = T
    c.visible[idx] = np.abs(T) > 0.0


def sequences(boundaries: list, max_order: int, max_faces: int = 1,
              max_object_faces: int = 0) -> list:
    """Every boundary sequence up to ``max_order`` bounces: no boundary
    twice in a row (planes; the sheet is not re-entrant at these scales) and
    at most ``max_faces`` planar faces per chain. Ordered by length."""
    out = [[]]
    frontier = [[]]
    for _ in range(max_order):
        new = []
        for s in frontier:
            for b in boundaries:
                if s and s[-1] is b:
                    continue
                if b.kind == "W" and sum(x.kind == "W" for x in s) >= max_faces:
                    continue
                if b.kind == "O" and sum(x.kind == "O" for x in s) >= max_object_faces:
                    continue
                new.append(s + [b])
        out += new
        frontier = new
    return out
