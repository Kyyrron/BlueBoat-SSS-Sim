"""``RaycastRenderer``: the second sensor behind the ``SonarRenderer`` seam.

Phase 2 build-up (``raycast_plan.md``): the **direct path** (beam, time of
flight, transmission loss, receiver envelope, coherent sum over world-fixed
diffuse scatterers, exact visibility by the scene oracle). Further paths
(surface, bottom, wall, dihedral, image comb), diffraction, transmission and
the PO patches of smooth bodies are added one at a time behind their gates.

Output contract (``raycast_design.md`` §2.1, §11): ``Ping.power`` is the
exact mean ``E|E|^2``; ``Ping.texture`` the realisation over the mean, so the
shared ``noise.apply_ping_noise`` (speckle on, coherent scatterers, one look)
reproduces ``|E|^2``; ``specular``, ``unoccluded`` and ``ambient`` are None
(the shadow floor must come from physics, ``reverberation_db: 0``).

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from ...core.types import Ping, Pose3D, RenderedPing, Side
from ..config import (OMNISCAN_FREQ_HZ, SPEED_OF_SOUND_MPS, AcquisitionParams,
                      SonarModelConfig)
from ..renderer import GeometricRenderer, SonarRenderer
from . import beam as _beam
from . import propagation as _prop
from .config import RaycastConfig
from .receiver import BinGrid, envelope_for
from .scattering import DiffuseLaw
from .scatterers import Scatterers, SheetSampler, planar_sample
from .scene_mesh import (KIND_WALL, SceneOracle, TriangleScene,
                         build_scene_mesh)


class RaycastRenderer(SonarRenderer):
    """3-D ray-cast side-scan renderer (CPU reference)."""

    def __init__(self, scene, acquisition: AcquisitionParams,
                 model: SonarModelConfig, config: RaycastConfig | None = None,
                 world_dir=None, mesh: TriangleScene | None = None,
                 oracle: SceneOracle | None = None) -> None:
        self._scene = scene
        self._acq = acquisition
        self._cfg = model
        self._rc = config or RaycastConfig()
        rc = self._rc
        self.mesh = mesh if mesh is not None else build_scene_mesh(scene, world_dir)
        self.oracle = oracle if oracle is not None else SceneOracle(self.mesh)
        self.water = _prop.Water(rc.water.temperature_c, rc.water.salinity_ppt,
                                 rc.water.ph, rc.water.mean_depth_m,
                                 float(OMNISCAN_FREQ_HZ))
        self.c = self.water.sound_speed
        self.k = self.water.wavenumber
        self.omega = 2.0 * np.pi * float(OMNISCAN_FREQ_HZ)
        self.alpha = self.water.alpha_db_per_m
        # The apertures are physical lengths: derived ONCE from the datasheet
        # widths at the device's nominal 1500 m/s, then radiated at the
        # site's true wavenumber (so the real beam widens a little in warmer,
        # faster water, as the physical array would).
        k_nom = 2.0 * np.pi * float(OMNISCAN_FREQ_HZ) / SPEED_OF_SOUND_MPS
        L = float(rc.beam.aperture_length_m) or _beam.aperture_from_width(
            rc.beam.two_way_width_deg, k_nom, True)
        if not 0.224 <= L <= 0.310:
            raise ValueError(f"aperture length {L:.3f} m outside [0.224, 0.310] m "
                             "(datasheet far field / CAD housing)")
        vf = float(rc.beam.vertical_floor_amp)
        if not 0.0 <= vf <= 0.3:
            raise ValueError(f"vertical_floor_amp {vf} outside [0, 0.3]")
        self.aperture = _beam.Aperture(
            L, _beam.aperture_from_width(rc.beam.height_deg, k_nom, False),
            self.k, vertical_floor=vf)
        self.tilt = np.radians(float(model.beam_tilt_deg))
        self.envelope = envelope_for(acquisition,
                                     rc.receiver.filter_duration_percent,
                                     rc.receiver.bin_integration,
                                     model.max_ping_rate_hz,
                                     rc.receiver.filter_pulse_ratio)
        self.law = DiffuseLaw(rc.seabed, self.mesh.material_names, self.k,
                              float(OMNISCAN_FREQ_HZ), self.c)
        seed = (int(getattr(scene, "seed", 0) or 0) * 1_000_003
                + int(rc.seed)) & 0xFFFFFFFFFFFFFFFF
        self.seed = seed
        self.sheet = SheetSampler(self.mesh, scene, seed,
                                  rc.sampling.scatterer_spacing_m,
                                  rc.sampling.max_refine)
        self._floor = 10.0 ** (rc.beam.floor_db / 10.0)
        # True slant range that can still reach the last bin through the
        # envelope. The device maps time to range with ITS speed of sound,
        # so its last bin lies at range_max * c_true / c_device in true range
        # (20.24 m for a 20 m setting at c = 1518 m/s), plus half the
        # envelope.
        self.r_max = (acquisition.range_max_m / SPEED_OF_SOUND_MPS
                      + 0.5 * self.envelope.half_support_s) * self.c * 1.001
        self._R_tab, self._w_tab = self._strip_table()
        self._walls = np.flatnonzero(self.mesh.kind == KIND_WALL)
        zmin = float(self.mesh.vertices[:, 2].min())
        self._zmin = zmin
        # Smooth solids (NC #17): their coherent PO patches, from the facet
        # layer's own placement and tessellation (sonar/facets.py), cached.
        from ...worldgen.materials import MaterialLibrary
        from ...worldgen.objects import is_facet_object, object_roughness_m
        from ..facets import object_reflection
        self._facet_objs = [o for o in getattr(scene, "objects", [])
                            if is_facet_object(o)]
        snap = getattr(scene, "config_snapshot", None) or {}
        lib = MaterialLibrary(snap.get("materials") or None)
        self._facet_R = [object_reflection(o, lib, float(OMNISCAN_FREQ_HZ))
                         for o in self._facet_objs]
        self._facet_sigma = [object_roughness_m(o) for o in self._facet_objs]
        self._facet_cache: dict[int, object] = {}

    # ------------------------------------------------------------- geometry
    def sensor_pose(self, side: Side, p: Pose3D) -> Pose3D:
        """The transducer pose: the geometric renderer's own formula (the
        lever arm rotated by the full attitude), shared, not copied."""
        return GeometricRenderer._sensor_pose(SimpleNamespace(_cfg=self._cfg),
                                              side, p)

    def _strip_table(self) -> tuple[np.ndarray, np.ndarray]:
        """Half-width ``w(R)`` of the along-track band where the two-way
        along-track pattern ``|b_a|^4`` still exceeds the floor (running max
        over R, so the band never narrows outward)."""
        R = np.concatenate([np.linspace(0.05, 2.0, 40),
                            np.linspace(2.0, self.r_max + 1.0, 120)[1:]])
        e_a, e_n = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])
        e_v = np.cross(e_n, e_a)
        w = np.zeros(R.size)
        for i, r in enumerate(R):
            u = np.linspace(0.0, min(r, r * 0.25 + self.aperture.length_m),
                            4000)
            d = np.outer(u, e_a) + np.outer(np.sqrt(np.maximum(r * r - u * u,
                                                              0.0)), e_n)
            b = np.abs(self.aperture.pattern(d, e_a, e_n, e_v,
                                             self._rc.beam.near_field)) ** 4
            above = np.flatnonzero(b >= self._floor)
            w[i] = u[above[-1]] if above.size else 0.0
        return R, np.maximum.accumulate(w) + 0.01

    def _band_width(self, R: np.ndarray) -> np.ndarray:
        return np.interp(R, self._R_tab, self._w_tab)

    # ---------------------------------------------------------- candidates
    def _sheet_region(self, P: np.ndarray, e_a: np.ndarray, e_n: np.ndarray):
        """Row-wise x-intervals of the band on the seabed sheet: a
        parallelogram along the horizontal look direction, conservative in
        pitch (``e_a`` tilted) and depth."""
        ea_xy = e_a[:2]
        na = np.linalg.norm(ea_xy)
        eh = ea_xy / na
        look = e_n[:2] / max(np.linalg.norm(e_n[:2]), 1e-9)
        dz = abs(P[2] - self._zmin) + 1.0
        W = (float(self._w_tab[-1]) + abs(e_a[2]) * dz) / na
        G = self.r_max
        back = dz                      # a little behind nadir (vertical lobes)
        c = P[:2]
        poly = np.array([c - back * look - W * eh, c + G * look - W * eh,
                         c + G * look + W * eh, c - back * look + W * eh])
        h = self.sheet.h
        y0 = poly[:, 1].min()
        q0 = int(np.floor(y0 / h))
        q1 = int(np.floor(poly[:, 1].max() / h))
        ya = (np.arange(q0, q1 + 1)) * h
        yb = ya + h
        lo = np.full(ya.size, np.inf)
        hi = np.full(ya.size, -np.inf)
        for k in range(4):
            p0, p1 = poly[k], poly[(k + 1) % 4]
            for yy in (ya, yb):
                t = np.where(p1[1] != p0[1], (yy - p0[1]) / (p1[1] - p0[1]
                                                              + 1e-300), -1)
                ok = (t >= 0) & (t <= 1)
                x = p0[0] + t * (p1[0] - p0[0])
                lo = np.where(ok, np.minimum(lo, x), lo)
                hi = np.where(ok, np.maximum(hi, x), hi)
        for v in poly:                  # vertices inside a row's band
            inb = (v[1] >= ya) & (v[1] < yb)
            lo = np.where(inb, np.minimum(lo, v[0]), lo)
            hi = np.where(inb, np.maximum(hi, v[0]), hi)
        return lo, hi, q0 * h

    def _scatterers(self, P: np.ndarray, e_a: np.ndarray, e_n: np.ndarray
                    ) -> Scatterers:
        lo, hi, y0 = self._sheet_region(P, e_a, e_n)
        parts = [self.sheet.sample(0.0, 0.0, lo, hi, y0)]
        if self._walls.size:
            def keep(pos, _P=P, _ea=e_a):
                d = pos - _P
                R = np.linalg.norm(d, axis=1)
                return (R <= self.r_max) & (np.abs(d @ _ea)
                                            <= self._band_width(R))
            parts.append(planar_sample(self.mesh, self._walls, self.seed,
                                       self._rc.sampling.scatterer_spacing_m,
                                       keep))
        return Scatterers.concat(parts)

    # -------------------------------------------------------------- render
    def render(self, side: Side, vehicle_pose: Pose3D, t_sim: float,
               motion=None) -> RenderedPing:
        acq = self._acq
        sensor = self.sensor_pose(side, vehicle_pose)
        P = np.array([sensor.x, sensor.y, sensor.z])
        e_a, e_n, e_v = _beam.transducer_frame(sensor.yaw, sensor.pitch,
                                               sensor.roll, side.sign,
                                               self.tilt)
        grid = BinGrid(acq, self.envelope, self.omega, SPEED_OF_SOUND_MPS)
        self.direct_path(P, e_a, e_n, e_v, grid)
        self.facet_path(P, e_a, e_n, e_v, grid)
        power = grid.mean
        lit = power > 0.0
        texture = np.where(lit, grid.intensity / np.where(lit, power, 1.0), 1.0)
        k_tvg = float(self._rc.receiver.tvg_k)
        if k_tvg:
            if not 0.0 <= k_tvg <= 40.0:
                raise ValueError(f"tvg_k {k_tvg} outside [0, 40]")
            r_dev = grid.start_m + (np.arange(grid.n) + 0.5) * grid.bin_m
            power = power * r_dev ** (k_tvg / 10.0)
        # Transmit ringing: a measured device leakage (55 dB reported at the
        # transducer, -22 dB/m; SonarModelConfig.ringing_*), not scattering.
        # Computed as the geometric renderer does, in the shared steady
        # channel (Ping.specular: Gamma(specular_looks) in the noise stage);
        # the encoder adds calibration_db_offset (this sensor's system gain)
        # back, so the reported level is the measured one.
        specular = None
        if self._cfg.ringing_level_db > 0.0:
            r_dev = grid.start_m + (np.arange(grid.n) + 0.5) * grid.bin_m
            ring_db = (self._cfg.ringing_level_db
                       - self._cfg.ringing_decay_db_per_m * r_dev)
            specular = 10.0 ** ((ring_db - self._cfg.calibration_db_offset)
                                / 10.0)
        down = self.oracle.closest(P, np.array([[0.0, 0.0, -1.0]]))
        altitude = float(down.t[0]) if np.isfinite(down.t[0]) else 0.0
        ping = Ping(side=side, power=power, pose=sensor, altitude_m=altitude,
                    t_sim=t_sim, start_mm=acq.range_start_mm,
                    length_mm=acq.range_length_mm, texture=texture,
                    specular=specular, unoccluded=None, ambient=None)
        return RenderedPing(ping=ping, contacts=[])

    def diffuse_contributions(self, P, e_a, e_n, e_v) -> dict:
        """The direct path's diffuse scatterers, seen: their deterministic
        two-way amplitude ``det`` (pattern, TL, sqrt(sigma dA)), delay
        ``tau`` and world-fixed random factor ``f`` (Brown17). The field is
        ``det * f``; the mean uses ``|det|^2``."""
        s = self._scatterers(P, e_a, e_n)
        empty = dict(det=np.zeros(0, complex), tau=np.zeros(0),
                     f=np.zeros(0, complex), idx=np.zeros(0, np.int64),
                     scatterers=s, candidates=len(s), kept=0)
        if len(s) == 0:
            return empty
        d = s.pos - P
        R = np.linalg.norm(d, axis=1)
        keep = (R <= self.r_max) & (R > 1e-3)
        cos_i = np.einsum("ij,ij->i", s.normal, -d) / np.maximum(R, 1e-9)
        keep &= cos_i > 0.0
        idx = np.flatnonzero(keep)
        b = self.aperture.pattern(d[idx], e_a, e_n, e_v,
                                  self._rc.beam.near_field)
        two = b * b
        ok = np.abs(two) ** 2 >= self._floor
        idx, two = idx[ok], two[ok]
        n_kept = int(idx.size)
        off = self._rc.sampling.surface_offset_m
        end = s.pos[idx] + off * s.normal[idx]
        vis = ~self.oracle.occluded(np.broadcast_to(P, end.shape), end)
        idx, two = idx[vis], two[vis]
        r = R[idx]
        sig = self.law.sigma(cos_i[idx], cos_i[idx], s.material[idx])
        det = two * _prop.leg_amplitude(r, self.alpha) ** 2 * np.sqrt(
            sig * s.area[idx])
        # Brown et al. 2017 eq. 2-3: a = f sqrt(sigma / q_s), f complex
        # circular Gaussian (|f| Rayleigh, world-fixed phase).
        f = s.rayleigh[idx] * np.exp(1j * s.phase[idx])
        return dict(det=det, tau=2.0 * r / self.c, f=f, idx=idx, scatterers=s,
                    candidates=len(s), kept=n_kept)

    def direct_path(self, P: np.ndarray, e_a: np.ndarray, e_n: np.ndarray,
                    e_v: np.ndarray, grid: BinGrid) -> dict:
        """Diffuse scatterers seen directly, deposited (the mean uses
        E|f|^2 = 1). Returns diagnostics (counts) for the tests."""
        c = self.diffuse_contributions(P, e_a, e_n, e_v)
        if c["det"].size:
            grid.add(c["det"] * c["f"], c["tau"], diffuse=True,
                     power=np.abs(c["det"]) ** 2)
        return {"candidates": c["candidates"], "kept": c["kept"],
                "visible": int(c["det"].size)}

    # ------------------------------------------------------- smooth solids
    def _patches(self, i: int):
        p = self._facet_cache.get(i)
        if p is None:
            from ..facets import object_patches
            from ...core.geometry import bilinear_sample
            from .scene_mesh import seabed_height
            zs = seabed_height(self._scene)
            g = self._scene.grid

            def seabed(x, y, _z=zs, _g=g):
                return bilinear_sample(_z, _g, x, y, fill=float(_z.min()))
            patch = float(self._rc.po.patch_m) or float(self._cfg.facet_patch_m)
            # k enters object_patches only through the tessellation's
            # curvature criterion: refined here (PoConfig.curvature_refine).
            p = object_patches(self._facet_objs[i], seabed, patch,
                               self.k * float(self._rc.po.curvature_refine))
            self._facet_cache[i] = p
        return p

    def facet_path(self, P, e_a, e_n, e_v, grid: BinGrid) -> int:
        """Coherent PO patches of every smooth solid in reach (direct path).
        Returns the number of visible patches."""
        from ..waveguide import eckart_c
        n_vis = 0
        for i, o in enumerate(self._facet_objs):
            c = np.array([o.x, o.y, 0.0])
            d0 = c[:2] - P[:2]
            reach = float(np.hypot(o.length, o.width)) + o.proud_height
            if np.hypot(*d0) - reach > self.r_max:
                continue
            p = self._patches(i)
            if p is None or len(p) == 0:
                continue
            n_vis += self.deposit_patches(P, e_a, e_n, e_v, p,
                                          float(self._facet_R[i]),
                                          float(self._facet_sigma[i]), grid,
                                          eckart=eckart_c)
        return n_vis

    def deposit_patches(self, P, e_a, e_n, e_v, p, refl: float,
                        rms_m: float, grid: BinGrid, eckart=None,
                        visibility: bool = True) -> int:
        """Kirchhoff (PO) backscatter of planar patches seen directly:
        ``(k/2pi) cos(theta) a b sinc sinc`` (``facets.po_amplitude``'s
        normalisation, sigma = 4 pi |sum|^2) times the material reflection,
        the Eckart coherent fraction sqrt(C), the two-way pattern and TL, on
        the patches the oracle sees. Deposited as coherent contributors."""
        d = p.centre - P
        R = np.linalg.norm(d, axis=1)
        g = d / np.maximum(R, 1e-9)[:, None]
        cos_t = -np.einsum("ij,ij->i", p.normal, g)
        keep = (cos_t > 0.0) & (R <= self.r_max)
        if not keep.any():
            return 0
        idx = np.flatnonzero(keep)
        b = self.aperture.pattern(d[idx], e_a, e_n, e_v,
                                  self._rc.beam.near_field)
        two = b * b
        ok = np.abs(two) ** 2 >= self._floor
        idx, two = idx[ok], two[ok]
        if visibility and idx.size:
            off = self._rc.sampling.surface_offset_m
            end = p.centre[idx] + off * p.normal[idx]
            vis = ~self.oracle.occluded(np.broadcast_to(P, end.shape), end)
            idx, two = idx[vis], two[vis]
        if idx.size == 0:
            return 0
        k = self.k
        gu = np.einsum("ij,ij->i", p.u[idx], g[idx])
        gv = np.einsum("ij,ij->i", p.v[idx], g[idx])
        area = (p.a[idx] * p.b[idx] * np.sinc(k * p.a[idx] * gu / np.pi)
                * np.sinc(k * p.b[idx] * gv / np.pi))
        amp = (k / (2.0 * np.pi)) * cos_t[idx] * area * refl
        if eckart is not None and rms_m > 0.0:
            psi = np.arcsin(np.clip(cos_t[idx], 0.0, 1.0))
            amp = amp * np.sqrt(eckart(psi, rms_m, k))
        r = R[idx]
        amp = amp * two * _prop.leg_amplitude(r, self.alpha) ** 2
        grid.add(amp.astype(complex), 2.0 * r / self.c, diffuse=False)
        return int(idx.size)

