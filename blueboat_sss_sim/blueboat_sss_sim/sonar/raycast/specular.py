"""The coherent (mean-field) return of the seabed sheet: the bottom path's
specular channel (``raycast_design.md`` §2, §14; notes 03, 05, 08).

A rough interface reflects a coherent part of the incident field -- the
mean field -- and scatters the rest. The incoherent part is the diffuse
scatterers' job (``scattering.DiffuseLaw``: the J&R small-slope law of the
fluctuating field); this module renders the mean field. It is what images
the first bottom return (NC #4) wherever the floor is smooth on the scale of
its Fresnel zone, and the floor's specular arrivals of the image comb.

**Physical optics, not an image construction.** The mean field is the
Kirchhoff integral over the sheet's own triangles, sampled by planar
parallelogram patches whose linear phase is integrated exactly (the
``sinc``s of ``facets.po_bistatic``), with the Kirchhoff prefactor
``i k / 2 pi`` in this renderer's ``exp(-i omega tau)`` convention, so that
an infinite flat floor returns exactly the image source ``V / (2 h)``,
amplitude and phase (gate S4/S5). A sloping or curved floor, a bank facing
the sonar, a floor seen through the vertical pattern's null -- all follow
from the geometry, with no specular point searched for.

Per patch (Tx at A, Rx at B; monostatic A = B):

    a = i (k / 2 pi) cos_b |u x v| sinc(q.u / 2) sinc(q.v / 2) V(theta) sqrt(C) w

* ``q = k (inc - out)``, ``cos_b = n.(out - inc) / 2`` (``facets.po_bistatic``);
* ``V(theta)``: the flat-interface pressure reflection coefficient at the
  patch's own incidence angle (the tangent-plane rule): J&R 8.49,
  fluid-fluid, for the sediments of ``seabed_ssa.SEDIMENTS`` (with the
  profile's bounded overrides); the material's fluid half-space otherwise;
* ``C = exp(-(dk_z sigma_h)^2)``, ``dk_z = k |n.(out - inc)|``: Eckart's
  coherent fraction generalised to the bistatic geometry (Eckart 1953;
  ``waveguide.eckart_c`` for the specular case). ``sigma_h`` is NOT a new
  constant: for a sediment it is the RMS height of the same J&R power-law
  spectrum the diffuse law uses, over the scales the coherent sum cannot
  resolve -- above ``K_low = max(pi / r_F, pi / dx)``, ``r_F = sqrt(lambda
  R / 2)`` the two-way first Fresnel radius and ``dx`` the sheet's cell (its
  Nyquist: the mesh carries every larger scale, so nothing is counted
  twice): ``sigma_h^2 = 2 pi w2 K_low^(2 - gamma2) / (gamma2 - 2)``. Other
  materials take their own RMS (``worldgen.materials.surface_roughness_m``);
* ``w``: a smooth stationary-phase window. The Kirchhoff integral of a smooth
  surface is carried by the neighbourhood of its stationary points; away
  from them the phase rotates and the contributions cancel, but a sampled
  sum cancels only approximately. So patches are weighted by
  ``w(N)`` = 1 inside ``N1`` Fresnel zones of the specular direction and a
  raised-cosine taper to 0 at ``N2``; ``N`` is the flat-facet phase excess
  over pi from the bisector's tilt against the smoothed sheet normal, in and
  across the plane of incidence (:meth:`SheetSpecular.zone`). N1, N2 and the patch size are numerical settings
  (``SpecularConfig``), converged in Phase 5; never reduced for speed.

Dry triangles are silent (NC #13). Every leg is tested by the scene oracle.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np

from . import seabed_ssa as _ssa
from .scene_mesh import KIND_APRON

_log = logging.getLogger(__name__)


def sediment_rms_height(p: _ssa.SedimentParams, k_low: np.ndarray) -> np.ndarray:
    """RMS height [m] of the J&R power-law spectrum ``W(K) = w2 K^-gamma2``
    (J&R eq. 6.8, two-dimensional) over ``K > k_low``:
    ``sigma^2 = int_{k_low}^inf W(K) 2 pi K dK = 2 pi w2 k_low^(2-gamma2) /
    (gamma2 - 2)`` (finite for gamma2 > 2)."""
    g2 = float(p.gamma2)
    if g2 <= 2.0:
        raise ValueError(f"{p.name}: gamma2 {g2} <= 2 has no finite RMS")
    kl = np.asarray(k_low, dtype=np.float64)
    return np.sqrt(2.0 * math.pi * p.w2 * kl ** (2.0 - g2) / (g2 - 2.0))


def k_low(R_eff: np.ndarray, lam: float, dx: float) -> np.ndarray:
    """``max(pi / r_F, pi / dx)``: the smallest wavenumber of the
    roughness the coherent sum treats statistically."""
    r_f = np.sqrt(0.5 * lam * np.maximum(np.asarray(R_eff, float), 1e-6))
    return np.maximum(np.pi / r_f, np.pi / float(dx))


def coherent_amplitude(dkz: np.ndarray, sigma_h: np.ndarray) -> np.ndarray:
    """``sqrt(C) = exp(-1/2 (dk_z sigma_h)^2)`` (Eckart 1953, bistatic form;
    the specular case is ``waveguide.eckart_c``)."""
    g = np.asarray(dkz, float) * np.asarray(sigma_h, float)
    return np.exp(-0.5 * np.minimum(g * g, 1400.0))


def window(N: np.ndarray, n1: float, n2: float) -> np.ndarray:
    """1 for N <= n1, raised-cosine taper to 0 at n2."""
    N = np.asarray(N, dtype=np.float64)
    t = np.clip((N - n1) / max(n2 - n1, 1e-12), 0.0, 1.0)
    return 0.5 * (1.0 + np.cos(np.pi * t))


@dataclass
class SheetPatches:
    """Planar parallelogram patches on the seabed sheet."""

    centre: np.ndarray     # (n, 3)
    normal: np.ndarray     # (n, 3) facet normal (up)
    smooth: np.ndarray     # (n, 3) smoothed normal (window only)
    u: np.ndarray          # (n, 3) edge vector of the x step
    v: np.ndarray          # (n, 3) edge vector of the y step
    area: np.ndarray       # (n,) |u x v|
    tri: np.ndarray        # (n,) sheet triangle, -1 beyond the grid
    material: np.ndarray   # (n,) material index

    def __len__(self) -> int:
        return int(self.area.size)


class MaterialSurface:
    """Per material index: the flat-interface reflection ``V(sin theta)``
    and the coherent-sum RMS height rule."""

    def __init__(self, material_names, frequency_hz: float,
                 sediments: dict | None = None, lib=None) -> None:
        from ...worldgen.materials import (MaterialLibrary, WATER_DENSITY_KGM3,
                                           WATER_SOUND_SPEED_MPS,
                                           surface_roughness_m)
        from .scattering import base_material
        lib = lib if lib is not None else MaterialLibrary()
        sed = sediments if sediments is not None else _ssa.SEDIMENTS
        self.names = list(material_names)
        self.sediment: list[_ssa.SedimentParams | None] = []
        self.half_space: list[tuple[float, float] | None] = []
        self.rms: list[float] = []
        for n in self.names:
            if n in sed:
                self.sediment.append(sed[n])
                self.half_space.append(None)
                self.rms.append(0.0)
                continue
            try:
                m = lib.get(base_material(n))
            except KeyError:
                m = None
            # any material with acoustic constants: its own fluid half-space
            # (a layered or flooded object material never forms the seabed
            # sheet; as a floor its half-space is the honest approximation)
            if m is not None and m.density_kgm3 > 0.0 and m.sound_speed_mps > 0.0:
                self.sediment.append(None)
                self.half_space.append((m.density_kgm3 / WATER_DENSITY_KGM3,
                                        m.sound_speed_mps / WATER_SOUND_SPEED_MPS))
                self.rms.append(float(surface_roughness_m(m)))
                continue
            # No verified interface parameters (gravel, rocks, seagrass, the
            # apron's "outside"): the sand interface, as the diffuse law does.
            if n not in ("outside",):
                _log.warning("ray-cast specular: no verified interface "
                             "parameters for %r; sand is used (gap)", n)
            self.sediment.append(sed["sand"])
            self.half_space.append(None)
            self.rms.append(0.0)

    def reflection(self, mat: int, sin_th: np.ndarray) -> np.ndarray:
        p = self.sediment[mat]
        if p is not None:
            return _ssa.reflection(p, sin_th)
        a_rho, nu = self.half_space[mat]
        q = _ssa.SedimentParams("half", a_rho, nu, 0.0, 3.0, 1e-6, 0.0)
        return _ssa.reflection(q, sin_th)

    def rms_height(self, mat: int, R_eff: np.ndarray, lam: float,
                   dx: float) -> np.ndarray:
        p = self.sediment[mat]
        if p is not None:
            return sediment_rms_height(p, k_low(R_eff, lam, dx))
        return np.full(np.shape(R_eff), self.rms[mat])


class SheetSpecular:
    """The seabed sheet's coherent return (monostatic and bistatic)."""

    def __init__(self, mesh, scene, k: float, frequency_hz: float,
                 n1: float, n2: float, patch_phase_rad: float,
                 patch_min_m: float, patch_max_m: float,
                 sediments: dict | None = None, lib=None) -> None:
        g = scene.grid
        self.g = g
        self.mesh = mesh
        self.k = float(k)
        self.lam = 2.0 * np.pi / self.k
        self.dx = float(g.resolution)
        self.n1, self.n2 = float(n1), float(n2)
        self.eps = float(patch_phase_rad)
        self.a_min, self.a_max = float(patch_min_m), float(patch_max_m)
        nv = g.nx * g.ny
        self.nq = (g.nx - 1) * (g.ny - 1)
        z = mesh.vertices[:nv, 2].reshape(g.ny, g.nx)
        self.z = z
        self.outside = getattr(scene, "outside", "fill")
        self.zmin = float(z.min())
        # Smoothed vertex normals (central differences): the window only.
        zy, zx = np.gradient(z, self.dx)
        n = np.stack([-zx, -zy, np.ones_like(z)], axis=-1)
        self.vnormal = n / np.linalg.norm(n, axis=-1, keepdims=True)
        tn = mesh.normals()
        self.tri_normal = np.where(tn[:, 2:3] < 0, -tn, tn)
        self.surf = MaterialSurface(mesh.material_names, frequency_hz,
                                    sediments, lib)
        self.out_mat = (int(mesh.material[mesh.kind == KIND_APRON][0])
                        if (mesh.kind == KIND_APRON).any() else 0)

    # ---------------------------------------------------------- sampling
    def _locate(self, x, y):
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
        lower = u >= v
        zz = np.where(lower, za + u * (zb - za) + v * (zc - zb),
                      za + u * (zc - zd) + v * (zd - za))
        q = j * (g.nx - 1) + i
        tri = np.where(lower, q, self.nq + q)
        if self.outside != "edge":
            zz = np.where(inside, zz, self.zmin)
        tri = np.where(inside, tri, -1)
        # smoothed normal, bilinear in the clipped cell
        vn = ((1 - u)[:, None] * (1 - v)[:, None] * self.vnormal[j, i]
              + u[:, None] * (1 - v)[:, None] * self.vnormal[j, i + 1]
              + u[:, None] * v[:, None] * self.vnormal[j + 1, i + 1]
              + (1 - u)[:, None] * v[:, None] * self.vnormal[j + 1, i])
        vn = np.where(inside[:, None] | (self.outside == "edge"), vn,
                      np.array([0.0, 0.0, 1.0]))
        vn = vn / np.linalg.norm(vn, axis=1, keepdims=True)
        return zz, tri, vn

    def zone(self, X, nrm, A, B):
        """Fresnel-zone index of each point from the specular direction of
        the surface ``nrm`` there, between A and B: the phase excess over pi
        of a flat facet, ``N = k R_e (mu_p^2 + sin^2(psi) mu_t^2) / pi``.
        ``mu_p``, ``mu_t``: the bisector's tangential tilt in and across the
        plane of incidence; ``psi`` the specular grazing angle
        (``|u_A + u_B| = 2 sin psi``); ``R_e = 2 R_a R_b / (R_a + R_b)``.
        Monostatic (psi = 90 deg) it is ``2 k R (1 - cos mu) / pi`` to second
        order, isotropic. Across the plane of incidence a bistatic zone is
        1/sin(psi) wider in angle (gate S4/S5, oblique)."""
        da, db = A - X, B - X
        ra = np.linalg.norm(da, axis=1)
        rb = np.linalg.norm(db, axis=1)
        ua = da / np.maximum(ra, 1e-300)[:, None]
        ub = db / np.maximum(rb, 1e-300)[:, None]
        bis = ua + ub
        bn = np.linalg.norm(bis, axis=1)
        sin_psi = np.clip(0.5 * bn, 1e-6, 1.0)
        bh = bis / np.maximum(bn, 1e-300)[:, None]
        tan_ = bh - np.einsum("ij,ij->i", bh, nrm)[:, None] * nrm
        dif = ub - ua
        ep = dif - np.einsum("ij,ij->i", dif, nrm)[:, None] * nrm
        epn = np.linalg.norm(ep, axis=1)
        ok = epn > 1e-9
        ep = np.where(ok[:, None], ep / np.maximum(epn, 1e-300)[:, None], 0.0)
        mu_p = np.einsum("ij,ij->i", tan_, ep)
        mu2 = np.einsum("ij,ij->i", tan_, tan_)
        mu_t2 = np.maximum(mu2 - mu_p * mu_p, 0.0)
        Re = 2.0 * ra * rb / np.maximum(ra + rb, 1e-300)
        N = self.k * Re * (mu_p * mu_p + sin_psi ** 2 * mu_t2) / np.pi
        # back-facing bisector: outside any window
        N = np.where(np.einsum("ij,ij->i", bh, nrm) > 0.0, N, np.inf)
        return N, Re

    def candidate_cells(self, A, B, bbox):
        """Sheet cells (j, i) in ``bbox`` = (x0, y0, x1, y1) whose smoothed
        normal at some corner lies inside the window (dilated by one cell:
        the normal varies inside a cell). Beyond the grid (the apron) the
        lattice continues at the sheet's spacing."""
        g = self.g
        i0 = int(np.floor((bbox[0] - g.origin_x) / self.dx))
        i1 = int(np.ceil((bbox[2] - g.origin_x) / self.dx))
        j0 = int(np.floor((bbox[1] - g.origin_y) / self.dx))
        j1 = int(np.ceil((bbox[3] - g.origin_y) / self.dx))
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1))
        x = g.origin_x + ii.ravel() * self.dx
        y = g.origin_y + jj.ravel() * self.dx
        zz, _, vn = self._locate(x, y)
        X = np.column_stack([x, y, zz])
        N, Re = self.zone(X, vn, A, B)
        inside = N <= self.n2
        m = inside.reshape(ii.shape)
        # cell (j, i) = vertex (j, i) .. (j+1, i+1); any corner inside,
        # then dilate by one cell
        c = m[:-1, :-1] | m[1:, :-1] | m[:-1, 1:] | m[1:, 1:]
        d = c.copy()
        d[1:, :] |= c[:-1, :]
        d[:-1, :] |= c[1:, :]
        d[:, 1:] |= d[:, :-1].copy()
        d[:, :-1] |= d[:, 1:].copy()
        jc, ic = np.nonzero(d)
        return ic + i0, jc + j0, Re.reshape(ii.shape)[:-1, :-1][jc, ic]

    def patches(self, A, B, bbox) -> SheetPatches:
        """Window-region patches: each candidate cell tiled by n x n
        parallelograms (n from the quadratic-phase criterion
        ``k a^2 / (4 R) <= eps``, a in [a_min, a_max]), each on the sheet
        triangle that holds its centre."""
        ic, jc, Re = self.candidate_cells(A, B, bbox)
        if ic.size == 0:
            return SheetPatches(*(np.zeros((0, 3)),) * 3,
                                *(np.zeros((0, 3)),) * 2,
                                np.zeros(0), np.zeros(0, np.int64),
                                np.zeros(0, np.int64))
        a_t = np.clip(np.sqrt(4.0 * self.eps * np.maximum(Re, 1e-3) / self.k),
                      self.a_min, self.a_max)
        n_sub = np.maximum(np.ceil(self.dx / a_t).astype(np.int64), 1)
        reps = n_sub * n_sub
        cid = np.repeat(np.arange(ic.size), reps)
        starts = np.cumsum(reps) - reps
        kk = np.arange(cid.size) - starts[cid]
        ns = n_sub[cid]
        sx = kk % ns
        sy = kk // ns
        a = self.dx / ns
        g = self.g
        x = g.origin_x + (ic[cid] + (sx + 0.5) / ns) * self.dx
        y = g.origin_y + (jc[cid] + (sy + 0.5) / ns) * self.dx
        zz, tri, vn = self._locate(x, y)
        nrm = np.where((tri >= 0)[:, None], self.tri_normal[np.maximum(tri, 0)],
                       np.array([0.0, 0.0, 1.0]))
        if self.outside == "edge":
            # beyond the grid the heights continue the edge: flat outward
            nrm = np.where((tri >= 0)[:, None], nrm, vn)
        zx = -nrm[:, 0] / nrm[:, 2]
        zy = -nrm[:, 1] / nrm[:, 2]
        u = np.column_stack([a, np.zeros_like(a), a * zx])
        v = np.column_stack([np.zeros_like(a), a, a * zy])
        area = a * a / nrm[:, 2]
        m = self.mesh
        tri_c = np.maximum(tri, 0)
        mat = np.where(tri >= 0, m.material[tri_c], self.out_mat)
        keep = ~np.where(tri >= 0, m.dry[tri_c], False)
        X = np.column_stack([x, y, zz])
        return SheetPatches(X[keep], nrm[keep], vn[keep], u[keep], v[keep],
                            area[keep], tri[keep], mat[keep])

    # ------------------------------------------------------------ field
    def amplitudes(self, p: SheetPatches, A, B):
        """Per patch: the complex coherent amplitude (no beam, no spreading,
        no carrier), the leg lengths, the leg directions, and the window,
        between the points A (source) and B (receiver)."""
        da = p.centre - A
        ra = np.linalg.norm(da, axis=1)
        inc = da / ra[:, None]
        db = B - p.centre
        rb = np.linalg.norm(db, axis=1)
        out = db / rb[:, None]
        amp, w = self.amplitudes_dirs(p, inc, out, ra, rb)
        return amp, ra, rb, inc, out, w

    def amplitudes_dirs(self, p: SheetPatches, inc, out, ra, rb,
                        parts: bool = False):
        """The same from the arriving propagation direction ``inc``, the
        leaving direction ``out`` (toward the receiver) and the unfolded leg
        lengths (image paths: the source at ``X - inc ra``, the receiver at
        ``X + out rb``). Returns (amplitude, window)."""
        q = self.k * (inc - out)
        qu = np.einsum("ij,ij->i", p.u, q)
        qv = np.einsum("ij,ij->i", p.v, q)
        nd = np.einsum("ij,ij->i", p.normal, out - inc)
        cos_b = 0.5 * nd
        front = ((-np.einsum("ij,ij->i", p.normal, inc) > 0.0)
                 & (np.einsum("ij,ij->i", p.normal, out) > 0.0))
        area = (p.area * np.sinc(0.5 * qu / np.pi) * np.sinc(0.5 * qv / np.pi))
        # Tangent-plane rule: V at the patch's own incidence angle (sine of
        # the grazing angle of the incident ray on the facet); at the
        # stationary point it is the specular grazing angle.
        sin_th = np.clip(-np.einsum("ij,ij->i", p.normal, inc), 0.0, 1.0)
        V = np.zeros(len(p), dtype=complex)
        sq = np.zeros(len(p))
        Re = 2.0 * ra * rb / np.maximum(ra + rb, 1e-300)
        dkz = self.k * np.abs(nd)
        for mi in np.unique(p.material):
            s = p.material == mi
            V[s] = self.surf.reflection(int(mi), sin_th[s])
            sq[s] = coherent_amplitude(dkz[s], self.surf.rms_height(
                int(mi), Re[s], self.lam, self.dx))
        A_img = p.centre - inc * ra[:, None]
        B_img = p.centre + out * rb[:, None]
        w = window(self.zone(p.centre, p.smooth, A_img, B_img)[0],
                   self.n1, self.n2)
        base = (1j * self.k / (2.0 * np.pi)) * np.where(
            front, np.clip(cos_b, 0.0, None), 0.0) * area * V * w
        if parts:
            # the fit basis: the amplitude without sqrt(C), and what sqrt(C)
            # is recomputed from
            return base, w, dict(dkz=dkz, Re=Re, mat=p.material.copy())
        return base * sq, w
