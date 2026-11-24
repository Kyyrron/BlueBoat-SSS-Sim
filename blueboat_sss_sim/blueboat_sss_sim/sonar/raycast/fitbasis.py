"""Exact parametric re-evaluation of a ray-cast ping for the fit (Phase A).

The parameters a field calibration fits for this sensor -- the 450 kHz
roughness spectrum (``w2``, ``gamma2`` per sediment), the large-angle
vertical level ``f_v``, the device TVG ``k`` (the system gain acts in the
shared stage) -- change no geometry: visibility, delays, envelope taps, the
along-track aperture and the vertical strip's sinc are fixed per pose. So
one render per pose is decomposed into a basis from which the mean power for
any parameter set follows by sums, instead of re-rendering:

* **diffuse direct field** (monostatic): ``E|a|^2 = |b_a|^4 (s^2 + f_v^2)^2
  L^4 sigma(theta) dA`` per scatterer (``s`` the strip sinc, ``L`` the leg
  amplitude). Scatterers are aggregated per (range bin, angle node,
  material) into ``S4 = sum |b_a|^4 s^4 L^4 dA w^2``, ``S2`` (``s^2``),
  ``S0`` (1), with linear weights on two angle nodes (0.05 deg apart), so
  ``mean_b = sum_k sigma_k (S4 + 2 f_v^2 S2 + f_v^4 S0)``. Scatterers whose
  pass of the two-way pattern floor depends on ``f_v`` are kept one by one,
  packed (float32 scalars and a dense row of envelope taps): near nadir a
  bin consists of them only, so any quantisation of their floor threshold
  moves the whole bin (an aggregated 128-level version was 0.45 dB off);
* **the floor's mean field** (``specular.py``): per patch ``a = K (s^2 +
  f_v^2) sqrt(C(w2, gamma2))``, ``K`` fixed, sqrt(C) recomputed from the
  spectrum (``specular.MaterialSurface.rms_height``) -- exact;
* TVG ``r^(k/10)`` after, as the renderer applies it.

Only scenes without smooth solids or walls, and without kept multipath
chains (open-water flat strata: the surface RMS cuts every chain), are
supported; anything else raises -- never a silent approximation. The basis
is validated against full renders (``test/raycast_test.py``, fit basis).

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ...core.types import Pose3D, Side
from ..config import SPEED_OF_SOUND_MPS
from . import beam as _beam
from . import propagation as _prop
from .receiver import BinGrid

THETA_STEP_DEG = 0.05
F_V_MAX = 0.3            # BeamConfig.vertical_floor_amp upper bound



@dataclass
class PingBasis:
    n: int
    r_dev: np.ndarray
    altitude_m: float
    specular: np.ndarray | None
    pose: Pose3D
    floor: float
    theta_nodes: np.ndarray
    # aggregated diffuse entries
    a_bin: np.ndarray
    a_node: np.ndarray
    a_mat: np.ndarray
    a_S4: np.ndarray
    a_S2: np.ndarray
    a_S0: np.ndarray
    # conditional diffuse contributors (their floor pass depends on f_v),
    # kept exactly, packed: per contributor its scalars and its envelope taps
    # as one dense row (the taps are contiguous bins from ``c_start``)
    c_along4: np.ndarray       # float32 (n,)
    c_s2: np.ndarray           # float32 (n,)
    c_L4A: np.ndarray          # float32 (n,)
    c_theta: np.ndarray        # float32 (n,) degrees
    c_mat: np.ndarray          # int16 (n,)
    c_start: np.ndarray        # int32 (n,)
    c_w2: np.ndarray           # float32 (n, K)
    # floor mean field
    h_K: np.ndarray
    h_s2: np.ndarray
    h_dkz: np.ndarray
    h_Re: np.ndarray
    h_mat: np.ndarray
    h_tap_i: np.ndarray
    h_tap_b: np.ndarray
    h_tap_wc: np.ndarray
    lam: float
    dx: float

    def power(self, law, surf, f_v: float, tvg_k: float) -> np.ndarray:
        """``Ping.power`` (the renderer's mean, TVG applied) for the
        parameter set carried by ``law`` (a ``scattering.DiffuseLaw``) and
        ``surf`` (a ``specular.MaterialSurface`` of the same sediments)."""
        f2 = float(f_v) ** 2
        c = np.cos(np.radians(self.theta_nodes))
        mats = np.unique(np.concatenate([self.a_mat, self.c_mat])).astype(np.int64)
        sig = np.zeros((self.theta_nodes.size, max(int(mats.max()) + 1, 1)
                        if mats.size else 1))
        # the node table is the same for every basis: computed once per law
        # (the same call on the same nodes, so the same values)
        memo = law.__dict__.setdefault("_basis_sigma", {})
        nodes_key = self.theta_nodes.tobytes()
        for m in mats:
            col = memo.get((nodes_key, int(m)))
            if col is None:
                col = memo[(nodes_key, int(m))] = law.sigma(c, c, np.full(c.size, m, dtype=np.int64))
            sig[:, m] = col
        out = np.zeros(self.n)
        if self.a_bin.size:
            w = sig[self.a_node, self.a_mat] * (self.a_S4 + 2.0 * f2 * self.a_S2
                                                + f2 * f2 * self.a_S0)
            out += np.bincount(self.a_bin, weights=w, minlength=self.n)
        if self.c_along4.size:
            a4 = self.c_along4.astype(np.float64)
            g = a4 * (self.c_s2 + f2) ** 2
            on = g >= self.floor
            th = self.c_theta.astype(np.float64) / THETA_STEP_DEG
            node = np.minimum(th.astype(np.int64), self.theta_nodes.size - 2)
            t = th - node
            m = self.c_mat.astype(np.int64)
            sg = (1.0 - t) * sig[node, m] + t * sig[node + 1, m]
            pn = np.where(on, g * self.c_L4A * sg, 0.0)
            K = self.c_w2.shape[1]
            cols = self.c_start[:, None] + np.arange(K)[None, :]
            ok = (cols >= 0) & (cols < self.n)
            v = pn[:, None] * self.c_w2
            out += np.bincount(cols[ok], weights=v[ok], minlength=self.n)
        if self.h_K.size:
            sq = np.zeros(self.h_K.size)
            for m in np.unique(self.h_mat):
                s = self.h_mat == m
                sig_h = surf.rms_height(int(m), self.h_Re[s], self.lam, self.dx)
                g = self.h_dkz[s] * sig_h
                sq[s] = np.exp(-0.5 * np.minimum(g * g, 1400.0))
            a = self.h_K * (self.h_s2 + f2) * sq
            v = a[self.h_tap_i] * self.h_tap_wc
            E = (np.bincount(self.h_tap_b, weights=v.real, minlength=self.n)
                 + 1j * np.bincount(self.h_tap_b, weights=v.imag, minlength=self.n))
            out += np.abs(E) ** 2
        if tvg_k:
            out = out * np.maximum(self.r_dev, 1e-9) ** (float(tvg_k) / 10.0)
        return out


def build_basis(rr, side: Side, pose: Pose3D) -> PingBasis:
    """Decompose one ping of renderer ``rr`` (module doc)."""
    if rr._facet_objs or (getattr(rr._scene, "walls", None) or []):
        raise ValueError("fit basis: scenes with smooth solids or walls are not "
                         "supported (render them in full)")
    full = rr.render(side, pose, 0.0).ping
    if rr.multipath is not None:
        rep = rr.multipath.last_report
        if (rep.get("diffuse", {}).get("order_reached", 0) > 0
                or rep.get("floor", {}).get("order_reached", 0) > 0):
            raise ValueError("fit basis: multipath chains are kept in this scene "
                             f"({rep}); render in full")
    acq = rr._acq
    sensor = rr.sensor_pose(side, pose)
    P = np.array([sensor.x, sensor.y, sensor.z])
    e_a, e_n, e_v = _beam.transducer_frame(sensor.yaw, sensor.pitch, sensor.roll,
                                           side.sign, rr.tilt)
    grid = BinGrid(acq, rr.envelope, rr.omega, SPEED_OF_SOUND_MPS)
    r_dev = grid.start_m + (np.arange(grid.n) + 0.5) * grid.bin_m
    nodes = np.arange(0.0, 90.0 + 1e-9, THETA_STEP_DEG)
    nn = nodes.size
    floor = rr._floor
    # ---- diffuse direct
    s = rr._scatterers(P, e_a, e_n)
    d = s.pos - P
    R = np.linalg.norm(d, axis=1)
    keep = (R <= rr.r_max) & (R > 1e-3)
    cos_i = np.einsum("ij,ij->i", s.normal, -d) / np.maximum(R, 1e-9)
    keep &= cos_i > 0.0
    idx = np.flatnonzero(keep)
    along, sinc_v, front = rr.aperture.parts(d[idx], e_a, e_n, e_v,
                                             rr._rc.beam.near_field)
    along4 = np.abs(along) ** 4 * front
    s2 = sinc_v ** 2
    ub = along4 * (s2 + F_V_MAX ** 2) ** 2 >= floor
    idx, along4, s2 = idx[ub], along4[ub], s2[ub]
    end = s.pos[idx] + rr._rc.sampling.surface_offset_m * s.normal[idx]
    vis = ~rr.oracle.occluded(np.broadcast_to(P, end.shape), end)
    idx, along4, s2 = idx[vis], along4[vis], s2[vis]
    Rk = R[idx]
    L4A = _prop.leg_amplitude(Rk, rr.alpha) ** 4 * s.area[idx]
    theta = np.degrees(np.arccos(np.clip(cos_i[idx], 0.0, 1.0)))
    node = np.minimum((theta / THETA_STEP_DEG).astype(np.int64), nn - 2)
    t = theta / THETA_STEP_DEG - node
    mat = s.material[idx].astype(np.int64)
    always = along4 * s2 * s2 >= floor
    ti, tb, tw, _ = grid.taps(2.0 * Rk / rr.c)
    w2 = tw * tw
    nm = int(max(mat.max() + 1 if mat.size else 1, 1))
    # aggregate the always-passing scatterers
    sel = always[ti]
    ai, ab, aw2 = ti[sel], tb[sel], w2[sel]
    base = along4[ai] * L4A[ai] * aw2
    entries = []
    for wnode, nd in (((1.0 - t[ai]), node[ai]), (t[ai], node[ai] + 1)):
        key = (ab * nn + nd) * nm + mat[ai]
        entries.append((key, base * wnode, s2[ai]))
    keys = np.concatenate([e[0] for e in entries])
    bw = np.concatenate([e[1] for e in entries])
    ss = np.concatenate([e[2] for e in entries])
    # keys are bounded (bin x node x material): a dense bincount gives the
    # unique keys and, accumulating in input order, the same per-key sums as
    # np.unique + bincount(inverse) without the sort (bit for bit)
    nkey = int(grid.n) * nn * nm
    present = np.bincount(keys, minlength=nkey)
    uk = np.flatnonzero(present)
    S4 = np.bincount(keys, weights=bw * ss * ss, minlength=nkey)[uk]
    S2 = np.bincount(keys, weights=bw * ss, minlength=nkey)[uk]
    S0 = np.bincount(keys, weights=bw, minlength=nkey)[uk]
    a_mat = uk % nm
    a_node = (uk // nm) % nn
    a_bin = uk // (nm * nn)
    # conditional ones, exact and packed (dense tap rows)
    cond = np.flatnonzero(~always)
    K = 2 * grid._k + 1
    centre = np.floor((0.5 * grid.device_c * (2.0 * Rk / rr.c) - grid.start_m)
                      / grid.bin_m).astype(np.int64)
    start = centre - grid._k
    cmap = -np.ones(idx.size, dtype=np.int64)
    cmap[cond] = np.arange(cond.size)
    csel = ~always[ti]
    W = np.zeros((cond.size, K), dtype=np.float32)
    rows_ = cmap[ti[csel]]
    W[rows_, tb[csel] - start[ti[csel]]] = w2[csel]
    # ---- the floor's mean field (monostatic)
    hK = np.zeros(0, complex)
    hs2 = hdkz = hRe = np.zeros(0)
    hmat = np.zeros(0, np.int64)
    hti = htb = np.zeros(0, np.int64)
    hwc = np.zeros(0, complex)
    sp = rr.specular
    if sp is not None:
        p = sp.patches(P, P, rr._band_bbox(P, e_a, e_n))
        if len(p):
            dd = p.centre - P
            ra = np.linalg.norm(dd, axis=1)
            inc = dd / ra[:, None]
            base_amp, w, parts = sp.amplitudes_dirs(p, inc, -inc, ra, ra, parts=True)
            live = np.flatnonzero((w > 0) & (base_amp != 0) & (ra <= rr.r_max))
            if live.size:
                al, sv, fr = rr.aperture.parts(dd[live], e_a, e_n, e_v,
                                               rr._rc.beam.near_field)
                endp = (p.centre[live] + rr._rc.sampling.surface_offset_m
                        * p.normal[live])
                vv = ~rr.oracle.occluded(np.broadcast_to(P, endp.shape), endp)
                live, al, sv, fr = live[vv], al[vv], sv[vv], fr[vv]
                hK = (base_amp[live] * al * al * fr
                      * _prop.leg_amplitude(ra[live], rr.alpha) ** 2)
                hs2 = sv * sv
                hdkz = parts["dkz"][live]
                hRe = parts["Re"][live]
                hmat = parts["mat"][live].astype(np.int64)
                hti, htb, htw, car = grid.taps(2.0 * ra[live] / rr.c)
                hwc = htw * car[hti]
    return PingBasis(
        n=grid.n, r_dev=r_dev, altitude_m=float(full.altitude_m),
        specular=None if full.specular is None else np.asarray(full.specular),
        pose=full.pose, floor=float(floor), theta_nodes=nodes,
        a_bin=a_bin, a_node=a_node, a_mat=a_mat, a_S4=S4, a_S2=S2, a_S0=S0,
        c_along4=along4[cond].astype(np.float32), c_s2=s2[cond].astype(np.float32),
        c_L4A=L4A[cond].astype(np.float32), c_theta=theta[cond].astype(np.float32),
        c_mat=mat[cond].astype(np.int16), c_start=start[cond].astype(np.int32),
        c_w2=W,
        h_K=hK, h_s2=hs2, h_dkz=hdkz, h_Re=hRe, h_mat=hmat, h_tap_i=hti,
        h_tap_b=htb, h_tap_wc=hwc,
        lam=(sp.lam if sp is not None else 2 * np.pi / rr.k),
        dx=(sp.dx if sp is not None else 0.1))


def law_and_surface(rr, sediments: dict | None):
    """The diffuse law and the interface rule of renderer ``rr`` for a set
    of sediment overrides (the fitted spectrum)."""
    from .config import SeabedLawConfig
    from .scattering import DiffuseLaw
    from .specular import MaterialSurface
    law = DiffuseLaw(SeabedLawConfig(law=rr._rc.seabed.law,
                                     lambert_mu_db=rr._rc.seabed.lambert_mu_db,
                                     sediments=dict(sediments or {})),
                     rr.mesh.material_names, rr.k, rr.freq_hz, rr.c, lib=rr.lib,
                     objects=list(getattr(rr._scene, "objects", None) or []),
                     split=rr.objects_mode == "solid", fouling=rr.fouling)
    surf = MaterialSurface(rr.mesh.material_names, rr.freq_hz,
                           getattr(law, "sediments", None), rr.lib)
    return law, surf


def _small_int(a: np.ndarray) -> np.ndarray:
    """``a`` in the smallest signed integer dtype that holds its values."""
    if a.size == 0 or not np.issubdtype(a.dtype, np.integer):
        return a
    lo, hi = int(a.min()), int(a.max())
    for t in (np.int8, np.int16, np.int32):
        if np.iinfo(t).min <= lo and hi <= np.iinfo(t).max:
            return a.astype(t)
    return a


def compact_basis(b: PingBasis) -> PingBasis:
    """The same basis in less memory, ``power`` bit for bit: integer arrays
    in the smallest dtype that holds them, and the conditional contributors
    whose taps are all zero dropped (each deposits exact zeros, and the
    others keep their order, so every bin's sum is unchanged). The fit holds
    every basis of every stratum at once (8.7 GB as stored, 2026-11-04)."""
    import dataclasses
    kw = {f.name: getattr(b, f.name) for f in dataclasses.fields(b)}
    for k in ("a_bin", "a_node", "a_mat", "c_mat", "c_start", "h_mat", "h_tap_i", "h_tap_b"):
        kw[k] = _small_int(np.asarray(kw[k]))
    if b.c_w2.size:
        live = np.any(b.c_w2 != 0, axis=1)
        if not live.all():
            for k in ("c_along4", "c_s2", "c_L4A", "c_theta", "c_mat", "c_start", "c_w2"):
                kw[k] = np.ascontiguousarray(kw[k][live])
    return PingBasis(**kw)


def save_basis(b: PingBasis, path) -> None:
    import dataclasses
    d = {}
    for f in dataclasses.fields(b):
        v = getattr(b, f.name)
        if f.name == "pose":
            v = np.array([v.x, v.y, v.z, v.roll, v.pitch, v.yaw])
        elif v is None:
            v = np.zeros(0)
            d["_none_" + f.name] = np.array(1)
        d[f.name] = np.asarray(v)
    np.savez_compressed(path, **d)


def load_basis(path) -> PingBasis:
    import dataclasses
    z = np.load(path)
    kw = {}
    for f in dataclasses.fields(PingBasis):
        v = z[f.name]
        if "_none_" + f.name in z:
            v = None
        elif f.name == "pose":
            v = Pose3D(*[float(x) for x in v])
        elif f.name in ("n",):
            v = int(v)
        elif f.name in ("altitude_m", "floor", "lam", "dx"):
            v = float(v)
        kw[f.name] = v
    return PingBasis(**kw)
