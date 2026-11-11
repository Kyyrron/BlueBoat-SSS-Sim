"""``RaycastRenderer``: the second sensor behind the ``SonarRenderer`` seam.

Phase 2 build-up (``raycast_plan.md``): the **direct path** (beam, time of
flight, transmission loss, receiver envelope, coherent sum over world-fixed
diffuse scatterers, exact visibility by the scene oracle). Further paths
(surface, bottom, wall, dihedral, image comb), diffraction, transmission and
the PO patches of smooth bodies are added one at a time behind their gates.

Ground truth (2026-11-11): with ``raycast.labels.enabled`` the contacts of a
ping are read off this renderer's own mesh and oracle (``labels.py``); off,
``RenderedPing.contacts`` is empty, as before.

Output contract (``raycast_design.md`` §2.1, §11): ``Ping.power`` is the
exact mean ``E|E|^2``; ``Ping.texture`` the realisation over the mean, so the
shared ``noise.apply_ping_noise`` (speckle on, coherent scatterers, one look)
reproduces ``|E|^2``; ``specular`` carries only the transmit ringing,
``unoccluded`` is None, and ``ambient`` is None (the shadow floor from
physics, ``reverberation_db: 0``) unless ``raycast.floor`` is on (harbour
profiles, 2026-11-10): then the ping's own lit-seabed reference
(``floor.ambient_reference``) for the noise stage's NC #15 floor.

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
from .scatterers import (Scatterers, SheetSampler, cull_wall_triangle, planar_sample,
                         wall_is_quay)
from .scene_mesh import (KIND_SMOOTH_OBJECT, KIND_SOLID, KIND_WALL, SceneOracle, TriangleScene,
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
        #: the shadow floor's smoothed reference slope, per side (floor.py)
        self._floor_state: dict = {}
        #: the measured near-range ringing of this acquisition mode
        #: (raycast.near_range, harbour patch v2), or None: the shared model's
        self._near = (rc.near_range if rc.near_range.applies(
            acquisition, float(getattr(model, "max_ping_rate_hz", 0.0))) else None)
        if rc.near_range.ringing_db is not None and self._near is None:
            import logging
            logging.getLogger(__name__).warning(
                "raycast.near_range was measured at %s bins / %s us, not this acquisition "
                "(%d bins): the shared model's ringing and water column are used",
                rc.near_range.num_results, rc.near_range.pulse_us, acquisition.num_results)
        from ...worldgen.materials import MaterialLibrary
        snap = getattr(scene, "config_snapshot", None) or {}
        #: the scene's materials: the defaults with its world's overrides
        self.lib = MaterialLibrary(snap.get("materials") or None)
        self.mesh = mesh if mesh is not None else build_scene_mesh(
            scene, world_dir, objects_mode=str(rc.objects.mode), lib=self.lib,
            frequency_hz=float(OMNISCAN_FREQ_HZ))
        #: "raster" or "solid": how THIS mesh carries the objects (the mode in
        #: force: a world without depth.npz stays raster; a shared mesh says
        #: what it was built as)
        self.objects_mode = str(getattr(self.mesh, "objects_mode", "raster"))
        self.oracle = (oracle if oracle is not None
                       else SceneOracle(self.mesh, backend=rc.backend))
        self.water = _prop.Water(rc.water.temperature_c, rc.water.salinity_ppt,
                                 rc.water.ph, rc.water.mean_depth_m,
                                 float(OMNISCAN_FREQ_HZ))
        self.c = self.water.sound_speed
        self.k = self.water.wavenumber
        self.omega = 2.0 * np.pi * float(OMNISCAN_FREQ_HZ)
        self.freq_hz = float(OMNISCAN_FREQ_HZ)
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
        from .correction import for_config as _correction_for
        #: the empirical correction layer, or None (off: bit-identical output)
        self._correction = _correction_for(rc.correction)
        if self._correction is not None:
            # an artifact applies only to the physical core it was fitted on
            self._correction.check_core(
                rc, model, accept_engine=bool(rc.correction.accept_engine_knobs))
        vf = float(rc.beam.vertical_floor_amp)
        if not 0.0 <= vf <= 0.3:
            raise ValueError(f"vertical_floor_amp {vf} outside [0, 0.3]")
        self.aperture = _beam.Aperture(
            L, _beam.aperture_from_width(rc.beam.height_deg, k_nom, False),
            self.k, vertical_floor=vf, gpu=(rc.backend == "warp"))
        self.tilt = np.radians(float(model.beam_tilt_deg))
        self.envelope = envelope_for(acquisition,
                                     rc.receiver.filter_duration_percent,
                                     rc.receiver.bin_integration,
                                     model.max_ping_rate_hz,
                                     rc.receiver.filter_pulse_ratio)
        solid = self.objects_mode == "solid"
        objects = list(getattr(scene, "objects", None) or [])
        #: fraction of every object and wall surface covered by sediment and
        #: growth (raycast.objects.fouling): diffuse like the seabed, no mirror
        self.fouling = float(rc.objects.fouling)
        self.law = DiffuseLaw(rc.seabed, self.mesh.material_names, self.k,
                              float(OMNISCAN_FREQ_HZ), self.c, lib=self.lib,
                              objects=objects, split=solid, fouling=self.fouling)
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
        # (20.55 m for a 20 m setting at the profiles' c = 1541 m/s), plus
        # half the envelope.
        self.r_max = (acquisition.range_max_m / SPEED_OF_SOUND_MPS
                      + 0.5 * self.envelope.half_support_s) * self.c * 1.001
        self._R_tab, self._w_tab = self._strip_table()
        # The triangles sampled in their own plane (diffuse scatterers): the
        # walls, and in objects.mode solid every object triangle (none
        # otherwise, so this is the wall list of before).
        self._walls = np.flatnonzero(np.isin(
            self.mesh.kind, (KIND_WALL, KIND_SOLID) + ((KIND_SMOOTH_OBJECT,) if solid else ())))
        zmin = float(self.mesh.vertices[:, 2].min())
        self._zmin = zmin
        # The coherent channel of smooth surfaces (NC #17): Kirchhoff patches,
        # cached. One entry per object and material: its owner, its parts
        # (None in raster mode: the facet layer's own placement and
        # tessellation, sonar/facets.py), |R|, RMS height, and whether its
        # faces also serve as bounce mirrors (an object that is one solid).
        from ...worldgen.objects import is_facet_object, object_roughness_m
        from ..facets import object_reflection
        facets = []
        if solid:
            by_id = {int(o.object_id): o for o in objects}
            for oid, surfaces in self.mesh.parts.items():
                groups: dict = {}
                for sf in surfaces:
                    if sf.coherent:
                        groups.setdefault(sf.material, []).append(sf)
                facets += [(by_id[oid], g, g[0].reflection, g[0].rms_m, len(surfaces) == 1)
                           for g in groups.values()]
        else:
            facets = [(o, None, object_reflection(o, self.lib, float(OMNISCAN_FREQ_HZ)),
                       object_roughness_m(o), True) for o in objects if is_facet_object(o)]
        self._facet_objs = [f[0] for f in facets]
        self._facet_parts = [f[1] for f in facets]
        self._facet_R = [f[2] * (1.0 - self.fouling) for f in facets]
        self._facet_sigma = [f[3] for f in facets]
        self._facet_faces = [f[4] for f in facets]
        self._facet_cache: dict[int, object] = {}
        # The seabed sheet's coherent (mean-field) channel: the bottom path's
        # specular return (specular.py; FBR imaged, NC #4).
        sc = rc.specular
        self.specular = None
        if sc.enabled:
            from .specular import SheetSpecular
            self.specular = SheetSpecular(
                self.mesh, scene, self.k, float(OMNISCAN_FREQ_HZ),
                sc.window_n1, sc.window_n2, sc.patch_phase_rad,
                sc.patch_min_m, sc.patch_max_m,
                sediments=getattr(self.law, "sediments", None), lib=self.lib)
        self._set_layers(self.lib)
        self.multipath = None
        if rc.paths.enabled and int(rc.paths.max_order) > 0:
            from .multipath import Multipath
            self.multipath = Multipath(self)
        #: ``backend: warp`` runs the per-contributor stages on the device
        #: (gpu_render.py); False keeps the CPU reference stages on the GPU
        #: oracle (the G-gates compare the two).
        self.use_gpu_pipeline = rc.backend == "warp"
        if int(rc.paths.reselect_every) > 1 and not self.use_gpu_pipeline:
            raise ValueError("raycast.paths.reselect_every > 1 is a GPU-pipeline path "
                             "(backend: warp); the CPU reference selects every ping")
        self._gpu = None
        self._gpu_mp = None
        #: GPU stage timing (``last_timing``) synchronises at every stage;
        #: the live node turns it off (no value depends on it, 2026-11-10).
        self.stage_timing = True
        #: Ground-truth contacts (``labels.py``, 2026-11-11): on with
        #: ``raycast.labels.enabled``; the node turns them off when nobody
        #: can read them. Never changes ``Ping.power`` or ``Ping.texture``.
        self.labels_on = bool(rc.labels.enabled)
        self._labels = None

    def _set_layers(self, lib) -> None:
        """Flooded thin shells (a material backed by water, e.g. ``pp_shell``,
        unless the instance says ``flooded: false``) transmit through their
        surface triangles by the fluid-layer formula at the crossing angle
        (O6): the measured 0.11 of the field box is a validation number, not
        an input (raycast_design §3)."""
        from ...worldgen.materials import (WATER_DENSITY_KGM3,
                                           WATER_SOUND_SPEED_MPS)
        m = self.mesh
        nt = m.n_triangles
        mask = np.zeros(nt, dtype=bool)
        rho = np.ones(nt)
        cr = np.ones(nt)
        th = np.zeros(nt)
        for o in self._facet_objs:
            mat = lib.get(o.material)
            params = dict(getattr(o, "params", None) or {})
            if (mat.backing or "solid").lower() != "water" or params.get("flooded") is False:
                continue
            if mat.density_kgm3 <= 0 or mat.sound_speed_mps <= 0:
                continue
            wall = params.get("wall_mm")
            d = float(wall) * 1e-3 if wall is not None else float(mat.wall_m)
            sel = (m.kind == KIND_SMOOTH_OBJECT) & (m.object_id == int(o.object_id))
            mask |= sel
            rho[sel] = mat.density_kgm3 / WATER_DENSITY_KGM3
            cr[sel] = mat.sound_speed_mps / WATER_SOUND_SPEED_MPS
            th[sel] = d
        k1 = 2.0 * np.pi * self.freq_hz / WATER_SOUND_SPEED_MPS
        self.oracle.set_layers(mask, rho, cr, th, k1)

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

    def _band_bbox(self, P: np.ndarray, e_a: np.ndarray, e_n: np.ndarray
                   ) -> tuple[float, float, float, float]:
        """Bounding box (x0, y0, x1, y1) of the band parallelogram of
        :meth:`_sheet_region`."""
        ea_xy = e_a[:2]
        na = np.linalg.norm(ea_xy)
        eh = ea_xy / na
        look = e_n[:2] / max(np.linalg.norm(e_n[:2]), 1e-9)
        dz = abs(P[2] - self._zmin) + 1.0
        W = (float(self._w_tab[-1]) + abs(e_a[2]) * dz) / na
        c = P[:2]
        poly = np.array([c - dz * look - W * eh, c + self.r_max * look - W * eh,
                         c + self.r_max * look + W * eh,
                         c - dz * look + W * eh])
        return (float(poly[:, 0].min()), float(poly[:, 1].min()),
                float(poly[:, 0].max()), float(poly[:, 1].max()))

    def _scatterers(self, P: np.ndarray, e_a: np.ndarray, e_n: np.ndarray
                    ) -> Scatterers:
        lo, hi, y0 = self._sheet_region(P, e_a, e_n)
        cull = bool(self._rc.sampling.cull_unreachable)
        sheet = self.sheet.sample(0.0, 0.0, lo, hi, y0)
        if cull:            # nothing above the still waterline (fast variant)
            sheet = sheet.take(sheet.pos[:, 2] <= 0.0)
        if bool(self._rc.sampling.band_cull) and len(sheet):
            # only inside the beam band at its own range (fast variant)
            d = sheet.pos - P
            R = np.linalg.norm(d, axis=1)
            sheet = sheet.take((R <= self.r_max) & (np.abs(d @ e_a) <= self._band_width(R)))
        parts = [sheet]
        walls = self._walls
        if walls.size and cull:
            # whole triangles: above the waterline, or quay faces turned away
            tv = self.mesh.vertices[self.mesh.triangles[walls]]
            dn = np.einsum("ij,ij->i", P - tv[:, 0], self.mesh.normals()[walls])
            zlo = tv[:, :, 2].min(axis=1)
            walls = walls[[not cull_wall_triangle(q, d, z) for q, d, z in
                           zip(wall_is_quay(self, walls), dn, zlo)]]
        if walls.size:
            offset = float(self._rc.sampling.surface_offset_m)

            def keep(pos, _P=P, _ea=e_a):
                d = pos - _P
                R = np.linalg.norm(d, axis=1)
                ok = (R <= self.r_max) & (np.abs(d @ _ea) <= self._band_width(R))
                if cull:    # in the water, not buried under the seabed
                    zs = self.sheet._locate(pos[:, 0], pos[:, 1])[0]
                    ok &= (pos[:, 2] <= 0.0) & (pos[:, 2] >= zs - offset)
                return ok
            parts.append(planar_sample(self.mesh, walls, self.seed,
                                       self._rc.sampling.scatterer_spacing_m,
                                       keep, P, self.r_max))
        return Scatterers.concat(parts)

    def release_device(self) -> None:
        """Drop the GPU pipeline's device buffers (scene scatterers, tables)
        and its reused echo-path selections (``paths.reselect_every``); the
        next render rebuilds them. Results are unchanged (a reused selection
        is simply made afresh). The fit keeps
        renderers cached long after their bases are built, and pinned
        buffers would hold device memory for nothing."""
        self._gpu = None
        self._gpu_mp = None

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
        if self.multipath is not None:
            if self.use_gpu_pipeline:
                if self._gpu_mp is None:
                    from .gpu_render import GpuMultipath
                    self._gpu_mp = GpuMultipath(self.multipath)
                self._gpu_mp.timing = bool(self.stage_timing)
                self._gpu_mp.diffuse(P, e_a, e_n, e_v, grid, key=side)
            else:
                self.multipath.diffuse(P, e_a, e_n, e_v, grid)
            import time as _time
            t0 = _time.perf_counter()
            if self.use_gpu_pipeline:
                self._gpu_mp.facets(P, e_a, e_n, e_v, grid, grid.mean, key=side)
            else:
                self.multipath.facets(P, e_a, e_n, e_v, grid, grid.mean)
            t1 = _time.perf_counter()
            if self.specular is not None:
                if self.use_gpu_pipeline:
                    self._gpu_mp.floor_coherent(P, e_a, e_n, e_v, grid, grid.mean, key=side)
                else:
                    self.multipath.floor_coherent(P, e_a, e_n, e_v, grid, grid.mean)
            #: per-stage wall time of the last render (seconds)
            self.last_timing = dict(getattr(self._gpu_mp, "last_timing", {}) if
                                    self.use_gpu_pipeline else {},
                                    facets=t1 - t0, floor_coherent=_time.perf_counter() - t1)
        else:
            self.direct_path(P, e_a, e_n, e_v, grid)
            self.facet_path(P, e_a, e_n, e_v, grid)
            self.floor_specular_path(P, e_a, e_n, e_v, grid)
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
        if self._near is not None:
            # the device's MEASURED ringing at this acquisition (near_range)
            r_dev = grid.start_m + (np.arange(grid.n) + 0.5) * grid.bin_m
            specular = 10.0 ** ((self._near.ringing_at(r_dev)
                                 - self._cfg.calibration_db_offset) / 10.0)
        elif self._cfg.ringing_level_db > 0.0:
            r_dev = grid.start_m + (np.arange(grid.n) + 0.5) * grid.bin_m
            ring_db = (self._cfg.ringing_level_db
                       - self._cfg.ringing_decay_db_per_m * r_dev)
            specular = 10.0 ** ((ring_db - self._cfg.calibration_db_offset)
                                / 10.0)
        down = self.oracle.closest(P, np.array([[0.0, 0.0, -1.0]]))
        altitude = float(down.t[0]) if np.isfinite(down.t[0]) else 0.0
        if self.objects_mode == "solid":
            # Objects are their own meshes here: the altitude (water column,
            # the correction's u = r / altitude and its domain) is the SEABED
            # under the transducer, not the top of an object the boat passes
            # over -- which would also switch the correction off for those
            # pings (its nadir gate), a 13-15 dB step at u = 1.
            zt, _ = self.sheet._locate(np.array([P[0]]), np.array([P[1]]))
            altitude = max(float(P[2] - zt[0]), 0.0)
        uncorrected = None
        if self._correction is not None:
            # the EMPIRICAL correction layer (correction.py; not physics): the
            # mean only, the texture untouched; the physical mean is kept.
            # Outside its domain it is off and the ping is the physical one.
            from .correction import bin_ranges
            uncorrected = power
            seabed, mat = (self.nadir_terrain(P) if self.objects_mode == "solid"
                           else self.nadir_surface(down))
            power, status = self._correction.correct(
                power, bin_ranges(acq), altitude, acq, nadir_material=mat,
                nadir_is_seabed=seabed, max_ping_rate_hz=self._cfg.max_ping_rate_hz)
        # The NC #15 shadow floor's reference (raycast.floor, harbour patch
        # 2026-11-10): the ping's own lit-seabed level, fitted on its mean
        # (floor.py; CPU, O(bins)). None when the floor is off: the noise
        # stage then has no reference and the output is bit-identical.
        ambient = None
        if float(self._rc.floor.reverberation_db) > 0.0:
            from .correction import bin_ranges as _bin_ranges
            from .floor import ambient_reference
            ambient = ambient_reference(
                power, _bin_ranges(acq), altitude, physical=uncorrected,
                state=self._floor_state.setdefault(side.value, {}))
        ping = Ping(side=side, power=power, pose=sensor, altitude_m=altitude,
                    t_sim=t_sim, start_mm=acq.range_start_mm,
                    length_mm=acq.range_length_mm, texture=texture,
                    specular=specular, unoccluded=None, ambient=ambient)
        if uncorrected is not None:
            ping.power_uncorrected = uncorrected
            ping.correction_status = status
        contacts = []
        if self.labels_on:
            if self._labels is None:
                from .labels import LabelOracle
                self._labels = LabelOracle(self, int(self._rc.labels.max_points))
            contacts = self._labels.contacts(side, P, e_a, e_n, float(acq.bin_size_m))
        return RenderedPing(ping=ping, contacts=contacts)

    def nadir_surface(self, hits) -> tuple[bool, str | None]:
        """``(seabed?, material name)`` of the first surface straight below
        the transducer, from an ``oracle.closest`` result: what the empirical
        correction's domain gate needs (``correction.py``). Never used by the
        physics."""
        from .scene_mesh import KIND_SEABED
        if not np.isfinite(hits.t[0]) or int(hits.tri[0]) < 0:
            return False, None
        m, tri = self.mesh, int(hits.tri[0])
        return (int(m.kind[tri]) == KIND_SEABED and not bool(m.dry[tri]),
                m.material_names[int(m.material[tri])])

    def nadir_terrain(self, P: np.ndarray) -> tuple[bool, str | None]:
        """:meth:`nadir_surface` on the terrain sheet under ``P`` (objects.mode
        solid: an object under the boat does not change what the seabed is)."""
        _, tri = self.sheet._locate(np.array([P[0]]), np.array([P[1]]))
        t = int(tri[0])
        if t < 0:
            return False, None
        m = self.mesh
        return (not bool(m.dry[t]), m.material_names[int(m.material[t])])

    def nadir_below(self, sensor: Pose3D) -> tuple[bool, str | None]:
        """:meth:`nadir_surface` for a transducer pose (the calibration
        engine's correction path, ``calib.cached_render``)."""
        P = np.array([sensor.x, sensor.y, sensor.z])
        if self.objects_mode == "solid":
            return self.nadir_terrain(P)
        return self.nadir_surface(self.oracle.closest(P, np.array([[0.0, 0.0, -1.0]])))

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

    # ------------------------------------------------- coherent floor channel
    def floor_specular_contributions(self, P, e_a, e_n, e_v, B=None,
                                     beam: bool = True,
                                     visibility: bool = True) -> dict:
        """The seabed sheet's mean field between the transducer P and the
        receiver B (default P: the monostatic bottom path), as coherent
        contributors: ``amp`` (pattern, TL, PO, V, sqrt(C), window) and the
        delay. ``beam=False`` drops the pattern (the S4/S5 gate)."""
        sp = self.specular
        if sp is None:
            return dict(amp=np.zeros(0, complex), tau=np.zeros(0), n=0)
        B = P if B is None else np.asarray(B, float)
        p = sp.patches(P, B, self._band_bbox(P, e_a, e_n))
        if len(p) == 0:
            return dict(amp=np.zeros(0, complex), tau=np.zeros(0), n=0)
        amp, ra, rb, inc, out, w = sp.amplitudes(p, P, B)
        keep = (w > 0.0) & (amp != 0.0) & (ra <= self.r_max) & (rb <= self.r_max)
        idx = np.flatnonzero(keep)
        if idx.size and beam:
            bt = self.aperture.pattern(p.centre[idx] - P, e_a, e_n, e_v,
                                       self._rc.beam.near_field)
            br = bt if B is P else self.aperture.pattern(
                p.centre[idx] - B, e_a, e_n, e_v, self._rc.beam.near_field)
            amp = amp.copy()
            amp[idx] = amp[idx] * bt * br
        if idx.size and visibility:
            off = self._rc.sampling.surface_offset_m
            end = p.centre[idx] + off * p.normal[idx]
            vis = ~self.oracle.occluded(np.broadcast_to(P, end.shape), end)
            if B is not P:
                vis &= ~self.oracle.occluded(np.broadcast_to(B, end.shape), end)
            idx = idx[vis]
        a = (amp[idx] * _prop.leg_amplitude(ra[idx], self.alpha)
             * _prop.leg_amplitude(rb[idx], self.alpha))
        return dict(amp=a, tau=(ra[idx] + rb[idx]) / self.c, n=int(idx.size),
                    patches=len(p))

    def floor_specular_path(self, P, e_a, e_n, e_v, grid: BinGrid) -> int:
        c = self.floor_specular_contributions(P, e_a, e_n, e_v)
        if c["n"]:
            grid.add(c["amp"], c["tau"], diffuse=False)
        return c["n"]

    # ------------------------------------------------------- smooth solids
    def _patches(self, i: int):
        p = self._facet_cache.get(i)
        if p is None:
            from ..facets import object_patches
            from ...core.geometry import bilinear_sample
            from .scene_mesh import seabed_height
            # the sheet the mesh was built on: the terrain in objects.mode
            # solid, else the stamped raster (as before)
            zs = (self.sheet.z if self.objects_mode == "solid"
                  else seabed_height(self._scene))
            g = self._scene.grid

            def seabed(x, y, _z=zs, _g=g):
                return bilinear_sample(_z, _g, x, y, fill=float(_z.min()))
            patch = float(self._rc.po.patch_m) or float(self._cfg.facet_patch_m)
            # k enters only through the tessellation's curvature criterion:
            # refined here (PoConfig.curvature_refine).
            kc = self.k * float(self._rc.po.curvature_refine)
            if self._facet_parts[i] is None:
                p = object_patches(self._facet_objs[i], seabed, patch, kc)
            else:
                from .object_mesh import part_patches
                p = part_patches(self._facet_parts[i], seabed, patch, kc)
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
        # Kirchhoff prefactor i k / 2 pi in this exp(-i omega tau)
        # convention (specular.py): the phase consistent with the floor's
        # mean field and the image paths; |amp| as facets.po_amplitude.
        amp = (1j * k / (2.0 * np.pi)) * cos_t[idx] * area * refl
        if eckart is not None and rms_m > 0.0:
            psi = np.arcsin(np.clip(cos_t[idx], 0.0, 1.0))
            amp = amp * np.sqrt(eckart(psi, rms_m, k))
        r = R[idx]
        amp = amp * two * _prop.leg_amplitude(r, self.alpha) ** 2
        grid.add(amp.astype(complex), 2.0 * r / self.c, diffuse=False)
        return int(idx.size)

