"""The facet layer: analytic high-frequency scattering of smooth solid objects.

NC #17 (2026-09-28). A surface smooth on the scale of the 3.3 mm wavelength
is a mirror at 450 kHz: Lambert's law -- the raster's backscatter law, fitted
on sand -- does not hold for it (Coiras & Groen 2009 eq. 11 caveat; Jackson &
Richardson 2007 par. 13.2). What such an object returns is set by its
geometry and its material, never by a brightness knob:

* **coherent** (Kirchhoff / physical optics, the standard high-frequency
  approximation): every patch of the exposed surface radiates
  ``(k / 2 pi) cos(theta) A sinc sinc exp(2 i k r)``, scaled by the
  material's reflection coefficient ``R`` (impedance, or the fluid-layer
  formula for a thin flooded wall -- ``worldgen/materials.py``) and by the
  Eckart coherent fraction ``sqrt(C)``, ``C = exp(-(2 k sigma cos theta)^2)``
  for the surface's RMS roughness ``sigma``. The patches are summed as
  complex amplitudes in each range bin, with the beam's amplitude pattern
  applied, so ``sigma = 4 pi |sum|^2``. Specular glints, their aspect
  dependence (a box wall glints within about 1 deg of broadside at 450 kHz,
  and nowhere else), edge returns and layover all EMERGE from the geometry;
* **incoherent**: the rest of the energy, ``1 - C``, scattered with the
  raster's own Lambert law, from a coefficient referred to sand's through the
  power reflection coefficient (``rho_sand |R / R_sand|^2``).

In the raster the object only occludes (``facet_bump``, ``renderer.
_ping_geometry``): the seabed under it is hidden by its two-way transmission
(0 when opaque; measured for a flooded shell).

**Units.** The renderer's bin values are the calibrated seabed's, normalised
per unit area of the resolution cell. A physical cross-section ``sigma``
[m^2] is brought onto that scale through ONE literature constant -- sand's
Lambert parameter at 450 kHz, ``sand_lambert_mu_db`` (J&R Fig. 12.3 and
par. 12.1.2) -- at the bin's own grazing angle: the renderer's sand level
``rho_sand sin^p(psi)`` corresponds to ``mu sin^2(psi)`` physically. Nothing
here is fitted to an object.

**Validity.** Patches are ``facet_patch_m`` (1 cm): linear phase across a
patch needs ``k a^2 / 4 r << 1`` (0.05 rad at 2 m) and the beam must not
vary across one (its amplitude width is ~1.5 cm at 2 m): R >= 2 m.
First-order only: no multiple reflection inside an object, no fringe (PTD)
term beyond physical optics, normal-incidence ``R``, and no highlight in the
multipath ghosts (they keep the object's shadow).

ROS-free and Qt-free, like the rest of ``sonar/``.
"""

from __future__ import annotations

from collections import OrderedDict

import numpy as np

from ..core.types import PlacedObject, Pose3D
from ..worldgen.materials import (SAND_REFLECTION, MaterialLibrary,
                                  acoustic_reflection)
from ..worldgen.objects import CATALOG, is_facet_object, object_roughness_m
from ..worldgen.solids import Patches, rot_z, rotation_rpy, tessellate
from . import acoustics
from . import waveguide as _wg
from .config import AcquisitionParams, SonarModelConfig

#: Objects whose patches are kept in memory at once (a 200 L drum is ~21 000
#: patches at 1 cm); the boat revisits few objects at a time.
_CACHE_OBJECTS = 48


def po_amplitude(p: Patches, sensor: np.ndarray, k: float
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Kirchhoff backscatter amplitude of each patch, seen from ``sensor``.

    Returns ``(amplitude, r, cos_theta)``. The amplitudes are normalised so
    that a set of patches summed coherently has cross-section
    ``sigma = 4 pi |sum|^2`` [m^2]: a flat plate of area A at normal
    incidence gives ``k^2 A^2 / pi = 4 pi A^2 / lambda^2`` (Urick). Unit
    reflection coefficient, no beam: both are the caller's. Back-facing
    patches (``cos_theta <= 0``) return 0.
    """
    d = p.centre - sensor
    r = np.sqrt(np.einsum("ij,ij->i", d, d))
    g = d / np.maximum(r, 1e-9)[:, None]
    cos_t = -np.einsum("ij,ij->i", p.normal, g)
    gu = np.einsum("ij,ij->i", p.u, g)
    gv = np.einsum("ij,ij->i", p.v, g)
    # The phase 2 k r is linear across a patch, with gradient 2 k (g.u) and
    # 2 k (g.v): its integral is a b sinc(k a g.u) sinc(k b g.v).
    area = (p.a * p.b * np.sinc(k * p.a * gu / np.pi)
            * np.sinc(k * p.b * gv / np.pi))
    amp = ((k / (2.0 * np.pi)) * np.clip(cos_t, 0.0, None) * area
           * np.exp(2j * k * r))
    return amp, r, cos_t


def object_patches(o: PlacedObject, seabed, patch_m: float,
                   k: float | None = None) -> Patches | None:
    """The exposed surface of a facet object, in world coordinates.

    ``seabed(x, y)`` is the seabed height under the objects (the scene's
    height minus its facet bumps). A rested object sits where the
    rasteriser settled it (``rest_z`` and the banked roll/pitch); a flat one
    stands with its top at the seabed plus its effective (unburied) height,
    exactly as the raster stamped it, and never floats. What lies under the
    seabed -- the buried part, the face it rests on -- is cut.
    """
    spec = CATALOG[o.type]
    sol = spec.solid(o)
    if sol is None:
        return None
    p = tessellate(sol, patch_m, k)
    if o.rested:
        R = rotation_rpy(o.roll, o.pitch, o.yaw)
        centre = np.array([o.x, o.y, o.rest_z])
    else:
        R = rot_z(o.yaw)
        z0 = float(seabed(np.array([o.x]), np.array([o.y]))[0])
        e_z = float(np.abs(R @ sol.extent)[2])
        # The solid never floats and is cut by the seabed when lower: its top
        # is min(effective_height, diameter) over a lying cylinder/capsule
        # (pipe, can, bottle ...), while the raster occluder is
        # effective_height tall (sonar_model.md section 12).
        top = z0 + o.effective_height
        centre = np.array([o.x, o.y, min(top - e_z, z0 + e_z)])
    w = p.transformed(R, centre - R @ sol.centre_offset)
    zs = seabed(w.centre[:, 0], w.centre[:, 1])
    return w.select(w.centre[:, 2] > zs + 1e-4)


def object_reflection(o: PlacedObject, lib: MaterialLibrary,
                      frequency_hz: float) -> float:
    """|R| of this instance: its material's, with the class knobs that
    describe its construction (``plastic_box``: ``wall_mm``, ``flooded``)."""
    mat = lib.get(o.material)
    params = dict(getattr(o, "params", None) or {})
    wall = params.get("wall_mm")
    backing = None
    if params.get("flooded") is False:
        backing = "air"
    return acoustic_reflection(
        mat, wall_m=float(wall) * 1e-3 if wall is not None else None,
        backing=backing, frequency_hz=frequency_hz)


def po_bistatic(p: Patches, inc: np.ndarray, out: np.ndarray,
                path: np.ndarray, k: float
                ) -> tuple[np.ndarray, np.ndarray]:
    """Bistatic Kirchhoff amplitude of each patch (pass 4, the image comb).

    ``inc`` (n, 3): unit propagation direction arriving at each patch;
    ``out`` (n, 3): unit direction from the patch toward the receiver;
    ``path``: total path length source -> patch -> receiver. The tangent-plane
    integral of ``exp(i k (|x - S| + |x - Rx|))`` over a flat ``a x b``
    patch with the obliquity ``n.(out - inc)/2`` -- for ``out = -inc`` it is
    :func:`po_amplitude` exactly (phase gradient ``2 k g``, obliquity
    ``-n.g``). Returns ``(amplitude, cos_b)``; a patch either leg sees from
    behind returns 0."""
    q = k * (inc - out)
    qu = np.einsum("ij,ij->i", p.u, q)
    qv = np.einsum("ij,ij->i", p.v, q)
    cos_b = 0.5 * np.einsum("ij,ij->i", p.normal, out - inc)
    front = ((-np.einsum("ij,ij->i", p.normal, inc) > 0.0)
             & (np.einsum("ij,ij->i", p.normal, out) > 0.0))
    area = (p.a * p.b * np.sinc(qu * p.a / (2.0 * np.pi))
            * np.sinc(qv * p.b / (2.0 * np.pi)))
    amp = ((k / (2.0 * np.pi)) * np.where(front, np.clip(cos_b, 0.0, None), 0.0)
           * area * np.exp(1j * k * path))
    return amp, np.where(front, np.clip(cos_b, 0.0, 1.0), 0.0)


class FacetLayer:
    """Per-renderer state of the facet layer: which objects, their material
    constants, and a bounded cache of their world patches."""

    def __init__(self, scene, acquisition: AcquisitionParams,
                 model: SonarModelConfig, smear_bins: int = 1) -> None:
        self._scene = scene
        # The range envelope a patch is weighted with, in AMPLITUDE: a
        # Gaussian whose power FWHM is the device's range resolution (the
        # renderer's pulse smear, >= 1 bin). A hard bin gate would cut an
        # oblique face at every bin boundary and each cut would ring like
        # an edge -- measured: a box wall 5 deg off broadside summed to
        # -217 dBsm over the whole face but -21 dBsm in one bin. The
        # device's pulse is smooth, and so is this.
        self._sig_bins = np.sqrt(2.0) * max(int(smear_bins), 1) / 2.355
        self._acq = acquisition
        self._cfg = model
        self._k = acoustics.wavenumber(model)
        self._objs = [o for o in getattr(scene, "objects", [])
                      if is_facet_object(o)]
        snap = getattr(scene, "config_snapshot", None) or {}
        lib = MaterialLibrary(snap.get("materials") or None)
        self._rho_sand = float(lib.get("sand").reflectivity)
        self._R = np.array([object_reflection(o, lib, model.sonar_frequency_hz)
                            for o in self._objs])
        self._sigma = np.array([object_roughness_m(o) for o in self._objs])
        # The incoherent residual's Lambert coefficient, referred to sand's
        # through the power reflection coefficient rather than judged.
        self._rho_inc = self._rho_sand * (self._R / SAND_REFLECTION) ** 2
        self._xy = np.array([[o.x, o.y] for o in self._objs]).reshape(-1, 2)
        self._rad = np.array([0.5 * float(np.sqrt(o.length ** 2 + o.width ** 2
                                                  + o.proud_height ** 2))
                              for o in self._objs])
        self._cache: OrderedDict[int, Patches] = OrderedDict()
        self._zref = np.full(len(self._objs), np.nan)
        self._mu = 10.0 ** (float(model.sand_lambert_mu_db) / 10.0)
        self._energy = str(model.image_formation).lower() == "energy"

    @staticmethod
    def applies(scene) -> bool:
        return any(is_facet_object(o) for o in getattr(scene, "objects", []))

    @property
    def objects(self) -> list[PlacedObject]:
        return self._objs

    def _seabed(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        z = self._scene.sample_height(x, y)
        fb = getattr(self._scene, "sample_facet_bump", None)
        return z - fb(x, y) if fb is not None else z

    def patches(self, i: int) -> Patches:
        p = self._cache.get(i)
        if p is None:
            p = object_patches(self._objs[i], self._seabed,
                               float(self._cfg.facet_patch_m), self._k)
            self._cache[i] = p
            while len(self._cache) > _CACHE_OBJECTS:
                self._cache.popitem(last=False)
        else:
            self._cache.move_to_end(i)
        return p

    #: Sub-bins per range bin of the fine grid the patches are summed on
    #: before the envelope is applied (2 mm at 1200 bins over 20 m: the
    #: envelope is constant across one to 1e-3).
    _SUB = 8

    def _range_gate(self, r: np.ndarray, amp: np.ndarray, inc: np.ndarray,
                    start: float, bin_m: float, n: int
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Weight each patch with the pulse envelope of every bin it reaches:
        the coherent amplitude with the envelope ``g``, the incoherent power
        with ``g^2``, ``g`` normalised so that ``sum_b g^2 = 1`` (energy is
        conserved; a flat plate's echo is unchanged).

        Done as a coherent sum on a fine sub-bin grid followed by one short
        convolution with the sampled envelope -- the same sum, reordered
        (patches that share a 1/8-bin cell see the same envelope to 1e-3),
        and ten times cheaper than an envelope per patch."""
        sb = self._sig_bins
        m = int(np.ceil(3.0 * sb))
        sub = self._SUB
        x = (r - start) / bin_m - 0.5                  # continuous bin coord
        lo = int(np.floor(x.min())) - m - 1
        hi = int(np.ceil(x.max())) + m + 2
        nf = (hi - lo) * sub
        fi = np.clip(((x - lo) * sub).astype(np.int64), 0, nf - 1)
        f_re = np.bincount(fi, weights=amp.real, minlength=nf)
        f_im = np.bincount(fi, weights=amp.imag, minlength=nf)
        f_in = np.bincount(fi, weights=inc, minlength=nf)
        # Envelope sampled on the fine grid, centred on a bin centre; its
        # normalisation is the per-patch one averaged over sub-bin offsets.
        t = (np.arange(-m * sub, (m + 1) * sub) + 0.5) / sub
        g = np.exp(-0.5 * (t / sb) ** 2)
        norm = np.sqrt(np.sum(g * g) / sub)
        ga, gp = g / norm, (g / norm) ** 2
        c_re = np.convolve(f_re, ga[::-1], mode="full")
        c_im = np.convolve(f_im, ga[::-1], mode="full")
        c_in = np.convolve(f_in, gp[::-1], mode="full")
        # Output sample j of a full convolution lines the kernel's first tap
        # up with fine cell j - len(g) + 1; bin b's centre is the fine cell
        # whose left edge is at (b - lo) * sub, i.e. the kernel's centre tap
        # (index m * sub) sits there.
        bins = np.arange(lo, hi)
        j = (bins - lo) * sub + (len(g) - 1) - m * sub
        ok = (bins >= 0) & (bins < n) & (j >= 0) & (j < c_re.size)
        out = [np.zeros(n) for _ in range(3)]
        for o_, c_ in zip(out, (c_re, c_im, c_in)):
            o_[bins[ok]] = c_[j[ok]]
        return out[0], out[1], out[2]

    def render(self, sensor: Pose3D, look: float, fwd: float,
               roll_toward: float, horizon: np.ndarray | None,
               ground_m: np.ndarray | None, occluded=None
               ) -> tuple[np.ndarray, np.ndarray, list[int]]:
        """One side's facet returns: ``(diffuse, coherent, lit)``.

        ``diffuse`` and ``coherent`` are per-bin powers on the renderer's
        own scale, ALREADY weighted by the pulse envelope: they are added to
        ``Ping.power`` and ``Ping.specular`` after the renderer's own pulse
        smear (the coherent one then carries the low-CV statistics of a
        steady reflection, like the first bottom return).
        ``horizon`` / ``ground_m`` are the centre line's occluder horizon
        (terrain and every object): a patch below it, in front of its
        object, is hidden. ``occluded``, when given, maps patch centres
        ``(n, 3)`` to a boolean mask of the ones a wall hides (the renderer
        passes the direct path's wall rule); ``None`` -- a scene without
        walls -- changes nothing. ``lit`` lists the objects that returned
        anything.
        """
        acq, cfg = self._acq, self._cfg
        n = acq.num_results
        out_d = np.zeros(n)
        out_s = np.zeros(n)
        lit: list[int] = []
        if not self._objs:
            return out_d, out_s, lit
        k = self._k
        start = acq.range_start_mm / 1000.0
        bin_m = acq.bin_size_m
        r_max = acq.range_max_m
        theta = np.radians(cfg.horizontal_aperture_deg)
        ap = max(float(cfg.array_length_m), 0.0)
        lx, ly = np.cos(look), np.sin(look)
        fx, fy = np.cos(fwd), np.sin(fwd)
        s = np.array([sensor.x, sensor.y, sensor.z])
        rel = self._xy - s[:2]
        across = rel[:, 0] * lx + rel[:, 1] * ly
        along = rel[:, 0] * fx + rel[:, 1] * fy
        sig_max = max(r_max * theta, ap) / 2.355
        cand = np.flatnonzero((across > -self._rad)
                              & (across < r_max + self._rad)
                              & (np.abs(along) < self._rad + 6.0 * sig_max))
        if cand.size == 0:
            return out_d, out_s, lit
        tan_pitch = np.tan(sensor.pitch)
        r_b = start + (np.arange(n) + 0.5) * bin_m
        w_b = np.sqrt(2.0 * np.pi) * np.maximum(r_b * theta, ap) / 2.355
        rng_b = acoustics.net_range_response(r_b, cfg)
        p_l = float(cfg.lambert_exponent)
        for i in cand:
            p = self.patches(int(i))
            if p is None or len(p) == 0:
                continue
            d = p.centre - s
            ac = d[:, 0] * lx + d[:, 1] * ly
            # The fan is pitched with the boat (renderer._ping_geometry):
            # a point dz below the transducer is imaged dz tan(pitch)
            # along-track from where a level fan puts it.
            al = d[:, 0] * fx + d[:, 1] * fy + d[:, 2] * tan_pitch
            r = np.sqrt(np.einsum("ij,ij->i", d, d))
            sig = np.maximum(np.maximum(r * theta, ap) / 2.355, 1e-4)
            azi = np.exp(-0.5 * (al / sig) ** 2)
            # 1e-5 in power: the patches it drops are 50 dB down the beam.
            keep = ((azi > 1e-5) & (ac > 0.0) & (r < r_max)
                    & (np.einsum("ij,ij->i", p.normal, d) < 0.0))
            if not keep.any():
                continue
            q = p.select(keep)
            azi = azi[keep]
            ac = ac[keep]
            horiz = np.hypot(ac, al[keep])
            amp, r, cos_t = po_amplitude(q, s, k)
            front = cos_t > 0.0
            # Terrain and nearer objects hide what lies under their horizon:
            # the centre line's, read in front of this object.
            if horizon is not None and ground_m is not None and front.any():
                y_front = float(ac[front].min()) - 0.1
                i_f = int(np.searchsorted(ground_m, y_front))
                if i_f > 0:
                    elev = -np.arctan2(s[2] - q.centre[:, 2], ac)
                    front &= elev >= float(horizon[i_f - 1]) - 1e-9
            if occluded is not None and front.any():
                front &= ~np.asarray(occluded(q.centre), dtype=bool)
            if not front.any():
                continue
            lit.append(int(i))
            # The vertical beam pattern at the patch's own depression -- the
            # same law the ground samples get, riding on the roll.
            dep = np.arctan2(s[2] - q.centre[:, 2], np.maximum(horiz, 1e-6))
            bw = acoustics.beam_weight(dep, cfg, roll_toward)
            cos_c = np.clip(cos_t, 0.0, 1.0)
            ce = np.exp(-np.minimum((2.0 * k * self._sigma[i] * cos_c) ** 2,
                                    700.0))
            wgt = np.where(front, np.sqrt(azi * bw), 0.0)
            amp = amp * self._R[i] * np.sqrt(ce) * wgt
            inc = np.where(front, (1.0 - ce) * self._rho_inc[i]
                           * cos_c ** p_l * q.area * azi * bw, 0.0)
            s_re, s_im, i_sum = self._range_gate(r, amp, inc, start, bin_m, n)
            if not (s_re.any() or s_im.any() or i_sum.any()):
                continue
            # The calibrated seabed at this object's own depth: the local
            # grazing angle of each bin and the resolution cell it stands
            # for, per unit area -- the scale the renderer's bins are on.
            z_ref = self._zref[i]
            if not np.isfinite(z_ref):
                z_ref = self._zref[i] = float(self._seabed(
                    np.array([self._objs[i].x]), np.array([self._objs[i].y]))[0])
            a_loc = max(s[2] - z_ref, 0.05)
            sin_psi = np.where(r_b > a_loc, a_loc / np.maximum(r_b, 1e-9),
                               1.0)
            norm = (np.full(n, bin_m) if self._energy
                    else bin_m * r_b / np.sqrt(np.maximum(r_b ** 2 - a_loc ** 2,
                                                          (0.5 * bin_m) ** 2)))
            kappa = (self._rho_sand * sin_psi ** p_l
                     / (self._mu * np.maximum(sin_psi, 1e-3) ** 2))
            out_s += (rng_b * kappa * 4.0 * np.pi * (s_re ** 2 + s_im ** 2)
                      / (norm * w_b))
            out_d += rng_b * i_sum / (norm * w_b)
            if cfg.waveguide_facet_comb and front.any():
                c_d = self._comb_pairs(q, front, azi, s, z_ref, roll_toward, i,
                                       start, bin_m, n)
                if c_d is not None:
                    # each image pair is a glint of its own (coherent over the
                    # patches), the pairs add incoherently: the sum fluctuates
                    # like the seabed, so it joins the diffuse channel.
                    c_re, c_im, c_in = c_d
                    out_d += (rng_b * kappa * 4.0 * np.pi * c_re / (norm * w_b)
                              + rng_b * c_in / (norm * w_b))
        return out_d, out_s, lit

    def _comb_pairs(self, q: Patches, front: np.ndarray, azi: np.ndarray,
                    s: np.ndarray, z_ref: float, roll_toward: float, i: int,
                    start: float, bin_m: float, n: int):
        """The object's shallow-water image comb (pass 4): every out/back
        pair of the transducer's surface/floor images up to
        ``waveguide_facet_max_order`` except the direct one, each a bistatic
        Kirchhoff glint (:func:`po_bistatic`) with its legs' one-way beam and
        bounce factors (``sonar/waveguide.py``). Returns the summed
        ``(|coherent|^2, 0, incoherent)`` per bin, pairs added in power."""
        cfg = self._cfg
        k = self._k
        depth = max(-float(z_ref), 0.05)
        z_t = -float(s[2])
        if not (0.0 < z_t < depth):
            return None
        qq = q.select(front)
        az = azi[front]
        im = _wg.image_set(z_t, depth, int(cfg.waveguide_facet_max_order))
        P = qq.centre
        n_img = im.depth.size
        imgs = np.repeat(s[None, :], n_img, axis=0)
        imgs[:, 2] = -im.depth
        floor = _wg.floor_boundary(cfg)
        legs = []
        for kk in range(n_img):
            v = P - imgs[kk]
            l = np.sqrt(np.einsum("ij,ij->i", v, v))
            u = v / np.maximum(l, 1e-9)[:, None]
            horiz = np.hypot(v[:, 0], v[:, 1])
            e = np.arctan2(-v[:, 2], horiz)           # + = going down at the patch
            psi = np.abs(e)
            fac = ((_wg.rayleigh_r(psi, floor[0], floor[1]) ** 2
                    * _wg.eckart_c(psi, floor[2], k)) ** im.n_floor[kk]
                   * _wg.eckart_c(psi, cfg.surface_rms_height_m, k)
                   ** im.n_surface[kk])
            dep = e * (1.0 if im.n_bounce[kk] % 2 == 0 else -1.0)
            bw = acoustics.beam_weight(dep, cfg, roll_toward)
            legs.append((l, u, np.sqrt(np.sqrt(bw) * fac)))   # amplitude/leg
        top = max(float(np.max(w / l)) for l, _, w in legs)
        tot_re = np.zeros(n)
        tot_in = np.zeros(n)
        sig = self._sigma[i]
        p_l = float(cfg.lambert_exponent)
        for a_ in range(n_img):
            la, ua, wa = legs[a_]
            if float(np.max(wa / la)) < 1e-3 * top:
                continue
            for b_ in range(n_img):
                if a_ == im.direct and b_ == im.direct:
                    continue
                lb, ub, wb = legs[b_]
                if float(np.max(wb / lb)) < 1e-3 * top:
                    continue
                amp, cos_b = po_bistatic(qq, ua, -ub, la + lb, k)
                if not np.any(amp):
                    continue
                ce = np.exp(-np.minimum((2.0 * k * sig * cos_b) ** 2, 700.0))
                wgt = wa * wb * np.sqrt(az)
                amp = amp * self._R[i] * np.sqrt(ce) * wgt
                inc = ((1.0 - ce) * self._rho_inc[i] * cos_b ** p_l * qq.area
                       * az * (wa * wb) ** 2)
                s_re, s_im, i_sum = self._range_gate(0.5 * (la + lb), amp, inc,
                                                     start, bin_m, n)
                tot_re += s_re ** 2 + s_im ** 2
                tot_in += i_sum
        return tot_re, None, tot_in
