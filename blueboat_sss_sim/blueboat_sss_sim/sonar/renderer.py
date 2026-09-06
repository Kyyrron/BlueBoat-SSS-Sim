"""Side-scan sonar renderers.

:class:`SonarRenderer` is the replaceable-backend interface: it turns a
sensor pose + a :class:`~blueboat_sss_sim.worldgen.scene.SceneModel` into one
:class:`~blueboat_sss_sim.core.types.RenderedPing` per side. Everything above
it (ROS node, encoder, dataset tooling) depends only on this interface, so
a GPU / tube-tracing / learning-based backend can be dropped in later.

:class:`GeometricRenderer` (v1) implements the classic heightfield SSS
model used across the synthetic-SSS literature (KTH draping line, UUV
simulator lineage):

1. Sample the seabed along the athwartship ground line of the ping.
2. Compute slant range, depression angle and local incidence per sample.
3. **Horizon culling** gives acoustic shadows: a sample is insonified only
   if its elevation angle (seen from the transducer) exceeds the running
   maximum of all nearer samples. Proud objects therefore cast
   geometrically correct shadows for free, and the nadir water-column gap
   appears naturally because no seabed sample has slant range < altitude.
4. Weight by material backscatter (Lambertian), vertical beam pattern
   (riding on vehicle roll -- the USV surface-motion signature), and the
   residual range response after idealised TVG.
5. Accumulate into ``num_results`` slant-range bins.

4b. **Azimuth-beam integration**: K parallel ground lines spanning the
   0.5 deg along-track footprint, Gaussian-weighted per sample with
   sigma(R) = R*theta/2.355 -- point targets blur along-track
   proportionally to range, as with the real beam.

5. **Wall / surface multipath** (on by default, a no-op without walls):
   each reflecting boundary declared by the world config contributes a
   mirror source, rendered by the same passes and summed into the same
   range bins, so a basin's ghost returns land at their true folded path
   length with per-ghost ground truth attached. See sonar/multipath.py.

6. **Direct wall echo and occlusion**: the ground line is intersected with
   every wall segment analytically; the wet face returns a coherent line
   at its own slant range (specular lobe in depression, cos^2 in azimuth
   obliquity) and every seabed sample behind the face is culled, exactly
   the bright-band-then-dark signature a quay leaves in a real mosaic.

7. **Coherent ground scatterers**: every ground sample carries a random
   phasor fixed to its ground cell (a hash, no raster); the diffuse field
   is summed coherently per bin and the transmit pulse is convolved on the
   complex field. The bin statistic is the textbook single-look Exp(1),
   but the texture is now *correlated* -- along range by the pulse and
   along track by the overlap of consecutive footprints, which the azimuth
   lines are anchored to a ground lattice to preserve -- as the corpus
   measures (lag-1 correlation 0.3-0.7 both ways). The renderer still
   returns the noiseless mean in ``Ping.power``; the texture rides in
   ``Ping.texture`` for the noise stage.

Known simplifications (see docs/sonar_model.md for the full list):
straight rays, static scene, stop-and-hop pings (no intra-ping motion),
and reflections only to first order -- the direct path plus one bounce
per mirror source.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field

import numpy as np

from ..core.types import (GroundTruthContact, Ping, Pose3D, RenderedPing,
                          Side, Wall)
from ..worldgen.objects import CATALOG
from ..worldgen.scene import SceneModel
from . import acoustics
from .config import (SPEED_OF_SOUND_MPS, AcquisitionParams,
                     SonarModelConfig)
from .multipath import bounce_weight, crossing_mask, mirror_sources, point_crosses


class SonarRenderer(abc.ABC):
    """Backend interface: pose -> one rendered ping for one side."""

    @abc.abstractmethod
    def render(self, side: Side, vehicle_pose: Pose3D, t_sim: float) -> RenderedPing:
        """Render one noiseless ping (noise is applied by the caller so that
        renderers stay deterministic and comparable)."""


@dataclass
class _WallHit:
    """Where one ground line meets a wall segment (direct path only)."""

    wall: Wall
    index: int                   # position in scene.walls
    d: float                     # horizontal distance to the face [m]
    x: float
    y: float
    z_foot: float                # seabed height at the foot of the face
    z_top: float                 # min(top_z, 0): the wet part of the face
    z_bottom: float              # max(seabed, wall.z_bottom): where it ends
    cos_az: float                # |look . normal|: 1 broadside, 0 along it


@dataclass
class _PingGeometry:
    """Intermediate per-ping geometry, kept for testability."""

    slant: np.ndarray            # slant range per ground sample [m]
    visible: np.ndarray          # bool, horizon-culling result
    depression: np.ndarray      # ray depression angle below horizontal [rad]
    cos_incidence: np.ndarray
    reflectivity: np.ndarray
    altitude: float
    px: np.ndarray               # world x of each ground sample
    py: np.ndarray               # world y of each ground sample
    seabed_z: np.ndarray         # seabed z at each ground sample
    wall_hits: list = field(default_factory=list)   # _WallHit, nearest first


@dataclass
class _Acc:
    """Per-bin accumulators of one source's azimuth integration."""

    num_d: np.ndarray            # diffuse (Lambert) power, azimuth-weighted
    num_s: np.ndarray            # coherent specular power
    den: np.ndarray              # azimuth weights (mean = num / den)
    re: np.ndarray | None        # coherent scatterer field, real part
    im: np.ndarray | None        # ... imaginary part (None = incoherent pass)
    energy: np.ndarray | None    # sum of squared per-cell amplitudes per bin
                                 # (= E|field|^2 exactly)
    cross: np.ndarray | None     # same-cell amplitude products across the
                                 # bin boundary b|b+1 (field correlation
                                 # between adjacent bins from straddling)
    num_b: np.ndarray            # single-bounce band (seabed -> face ->
                                 # transducer), a *sum* of scatterers landing
                                 # in the bin, ghost passes only
    num_w: np.ndarray            # wall-face echo (coherent), own denominator
    den_w: np.ndarray


# splitmix64-style integer mixing for the scatterer field: a phase per
# ground cell from (ix, iy, salt), no raster, any world size.
_K1 = np.uint64(0x9E3779B97F4A7C15)
_K2 = np.uint64(0xC2B2AE3D27D4EB4F)
_K3 = np.uint64(0xBF58476D1CE4E5B9)
_K4 = np.uint64(0x94D049BB133111EB)


class GeometricRenderer(SonarRenderer):
    """Heightfield ray model with horizon-culling shadows (see module doc)."""

    def __init__(self, scene: SceneModel, acquisition: AcquisitionParams,
                 model: SonarModelConfig) -> None:
        self._scene = scene
        self._direct_sensor: Pose3D | None = None
        self._acq = acquisition
        self._cfg = model

        # Ground-line sample step, coupled to the slant-bin size so that
        # every range bin receives samples at any num_results (600, 1200,
        # ...). The previous fixed 5 cm step left ~50% of far-range bins
        # empty at 600 bins (2.5 cm bins) -- an aliasing artifact that was
        # masked by the noise floor. Floor at 4 mm for cost sanity.
        # Coherent scatterer mode needs several independent scatterers per
        # bin for the per-bin statistic to reach Exp(1); the pulse smear adds
        # more, but the ground step is tightened a little as well.
        self._coherent = bool(model.speckle and model.coherent_scatterers
                              and int(model.speckle_looks) == 1)
        step_frac = 0.30 if self._coherent else 0.45
        self._ds = max(min(model.sample_step_m,
                           step_frac * acquisition.bin_size_m), 0.004)
        seed_words = np.array([int(getattr(scene, "seed", 0) or 0) & 0xFFFFFFFF,
                               int(model.scatterer_seed) & 0xFFFFFFFF],
                              dtype=np.uint64)
        with np.errstate(over="ignore"):
            salt = (seed_words[:1] * _K1) ^ (seed_words[1:] * _K2) ^ _K3
            salt ^= salt >> np.uint64(31)
            salt = salt * _K4
        self._salt = salt[0]
        self._cell_inv = 1.0 / max(float(model.scatterer_cell_m), 1e-3)

        # Along-track (azimuth) beam integration: the 0.5 deg Omniscan beam
        # has a footprint ~ R * theta that a single infinitesimal ground
        # line cannot represent. We integrate K parallel ground lines
        # spanning the footprint at max range and weight each *sample* by a
        # Gaussian in its along-track offset, with sigma(R) = R*theta/2.355
        # -- so far range blurs along-track while near-nadir stays sharp.
        k = max(int(model.alongtrack_beam_lines), 1)
        self._n_lines = k if k % 2 == 1 else k + 1
        self._theta_h = np.radians(model.horizontal_aperture_deg)
        half_fp = 0.5 * acquisition.range_max_m * self._theta_h
        self._line_offsets = (np.linspace(-half_fp, half_fp, self._n_lines)
                              if self._n_lines > 1 else np.array([0.0]))

        # Transmit-pulse ringing: the decaying coherent core right under
        # the transducer (measured on the field corpus — see
        # SonarModelConfig.ringing_level_db). Precomputed per acquisition
        # as linear power at the calibration gain: the encoder adds
        # calibration_db_offset (+ the gain step) back, so the *reported*
        # level lands on the measured 55 dB → 33 dB/1 m decay. Joined to
        # the coherent specular channel, whose low-CV statistics match
        # the steadiness of real ringing.
        if model.ringing_level_db > 0.0:
            r_bins = (acquisition.range_start_mm / 1000.0
                      + (np.arange(acquisition.num_results) + 0.5)
                      * acquisition.bin_size_m)
            ring_db = (model.ringing_level_db
                       - model.ringing_decay_db_per_m * r_bins)
            self._ringing = 10.0 ** ((ring_db - model.calibration_db_offset)
                                     / 10.0)
        else:
            self._ringing = None

        # Ghost renders get their own (by default coarser) azimuth sampling:
        # a multipath ghost is dim and already smeared by the bounce, so
        # paying the direct path's line count once per mirror source buys
        # nothing. This is the cost lever when a basin has several walls.
        gk = max(int(model.ghost_beam_lines), 1)
        self._n_ghost_lines = gk if gk % 2 == 1 else gk + 1
        self._ghost_offsets = (
            np.linspace(-half_fp, half_fp, self._n_ghost_lines)
            if self._n_ghost_lines > 1 else np.array([0.0]))
        # Lattice pitch of the azimuth lines in coherent mode: the line set
        # is snapped to a ground lattice along the heading so that
        # consecutive pings re-illuminate the same scatterer cells with
        # different footprint weights -- the along-track correlation.
        self._line_pitch = (2.0 * half_fp / (self._n_lines - 1)
                            if self._n_lines > 1 else 0.0)

    # ------------------------------------------------------------- helpers
    def smear_bins(self) -> int:
        """Width of the range envelope kernel in bins (1 = none)."""
        cfg = self._cfg
        if not cfg.pulse_smearing:
            return 1
        tau = self._acq.pulse_duration_s(cfg.max_ping_rate_hz)
        w = int(round((SPEED_OF_SOUND_MPS * tau / 2.0) / self._acq.bin_size_m))
        return max(w, int(cfg.range_filter_bins), 1)

    def _line_offsets_for(self, sensor: Pose3D, fwd: float) -> np.ndarray:
        """Along-track offsets of the azimuth lines for this ping."""
        if not self._coherent or self._n_lines <= 1 or self._line_pitch <= 0:
            return self._line_offsets
        s0 = sensor.x * np.cos(fwd) + sensor.y * np.sin(fwd)
        base = np.floor(s0 / self._line_pitch) * self._line_pitch - s0
        j = np.arange(self._n_lines) - (self._n_lines - 1) // 2
        return base + j * self._line_pitch

    def _scatter_phase(self, px: np.ndarray, py: np.ndarray) -> np.ndarray:
        """Ground-fixed random phase per sample (hash of its ground cell)."""
        ix = np.floor(px * self._cell_inv).astype(np.int64)
        iy = np.floor(py * self._cell_inv).astype(np.int64)
        with np.errstate(over="ignore"):
            h = ((ix.astype(np.uint64) * _K1) ^ (iy.astype(np.uint64) * _K2)
                 ^ self._salt)
            h ^= h >> np.uint64(31)
            h *= _K3
            h ^= h >> np.uint64(29)
            h *= _K4
            h ^= h >> np.uint64(32)
        phase = (h >> np.uint64(40)).astype(np.float64) * (2.0 * np.pi / 2.0 ** 24)
        # Group id along the line: a new scatterer whenever the cell changes
        # (samples are ordered along the line, so equal cells are adjacent).
        change = np.ones(ix.shape, dtype=bool)
        change[1:] = (ix[1:] != ix[:-1]) | (iy[1:] != iy[:-1])
        return phase, np.cumsum(change) - 1

    def _wall_hits(self, sensor: Pose3D, dx: float, dy: float,
                   r_max: float) -> list[_WallHit]:
        """Intersections of the horizontal ground ray with every wall."""
        walls = getattr(self._scene, "walls", None) or []
        if not walls or not self._cfg.wall_echo_enabled:
            return []
        hits: list[_WallHit] = []
        for i, w in enumerate(walls):
            ex, ey = w.x1 - w.x0, w.y1 - w.y0
            det = ex * dy - dx * ey
            if abs(det) < 1e-12:
                continue                               # ray parallel to wall
            rx, ry = w.x0 - sensor.x, w.y0 - sensor.y
            d = (ex * ry - rx * ey) / det
            t = (dx * ry - dy * rx) / det
            if d <= 0.05 or d > r_max or t < 0.0 or t > 1.0:
                continue
            hx, hy = sensor.x + d * dx, sensor.y + d * dy
            z_foot = float(self._scene.sample_height(np.array([hx]),
                                                     np.array([hy]))[0])
            nx, ny = w.normal
            hits.append(_WallHit(wall=w, index=i, d=float(d), x=float(hx),
                                 y=float(hy), z_foot=z_foot,
                                 z_top=min(float(w.top_z), 0.0),
                                 z_bottom=max(z_foot, float(w.z_bottom)),
                                 cos_az=abs(dx * nx + dy * ny)))
        hits.sort(key=lambda h: h.d)
        return hits

    # ------------------------------------------------------------------ API
    def render(self, side: Side, vehicle_pose: Pose3D, t_sim: float) -> RenderedPing:
        sensor = self._sensor_pose(side, vehicle_pose)
        acq = self._acq
        self._direct_sensor = sensor

        # Direct path. Look/forward directions are passed explicitly rather
        # than re-derived downstream, because a mirror source's are reflected
        # and cannot be recovered from a yaw plus a side sign.
        look = sensor.yaw + side.sign * np.pi / 2.0
        roll_toward = -side.sign * sensor.roll
        acc, center_geom = self._integrate(
            sensor, look, sensor.yaw, roll_toward, 1.0,
            self._line_offsets_for(sensor, sensor.yaw), self._n_lines, None,
            direct=True)
        den = acc.den
        safe = np.maximum(den, 1e-12)
        lit = den > 1e-12
        power = np.where(lit, acc.num_d / safe, 0.0)
        specular = np.where(lit, acc.num_s / safe, 0.0)
        if acc.den_w.any():
            # Wall-face echo: its own mean per bin (a separate denominator,
            # so the face never dilutes the ground mean in the same bins),
            # joined to the coherent channel like the FBR.
            specular = specular + np.where(
                acc.den_w > 0.0, acc.num_w / np.maximum(acc.den_w, 1e-12), 0.0)
        if self._ringing is not None:
            specular = specular + self._ringing

        texture = None
        if acc.re is not None:
            texture = self._texture(acc)

        power = self._pulse_smear(power)
        specular = self._pulse_smear(specular)
        # The second-bottom-echo ghost images the *direct* response only, so
        # it is applied before wall ghosts are summed in -- the two multipath
        # features stay independent instead of ghosting each other.
        power = self._apply_multipath(power, center_geom.altitude)

        contacts = self._ground_truth_contacts(
            side, sensor, look, sensor.yaw, center_geom,
            center_geom.altitude)
        contacts += self._wall_contacts(side, sensor, center_geom)
        ghost_power, ghost_contacts = self._render_ghosts(
            side, sensor, center_geom.altitude)
        if ghost_power is not None:
            power = power + ghost_power
        contacts += ghost_contacts

        ping = Ping(
            side=side,
            power=power,
            specular=specular,
            pose=sensor,
            altitude_m=center_geom.altitude,
            t_sim=t_sim,
            start_mm=acq.range_start_mm,
            length_mm=acq.range_length_mm,
            texture=texture,
        )
        return RenderedPing(ping=ping, contacts=contacts)

    # --------------------------------------------------- azimuth integration
    def _integrate(self, origin: Pose3D, look: float, fwd: float,
                   roll_toward: float, depression_sign: float,
                   line_offsets: np.ndarray, n_lines: int,
                   wall, *, direct: bool = False
                   ) -> tuple[_Acc, _PingGeometry]:
        """Integrate the azimuth beam for one source (direct or mirrored).

        One ground line per along-track offset, each sample weighted by the
        range-dependent azimuth pattern. The centre line (smallest offset)
        provides altitude and ground-truth geometry. ``direct`` selects the
        real transducer's pass: only it carries the coherent scatterer field
        and the wall face echo / occlusion.
        """
        n = self._acq.num_results
        z = lambda: np.zeros(n, dtype=np.float64)   # noqa: E731
        coherent = direct and self._coherent
        acc = _Acc(num_d=z(), num_s=z(), den=z(),
                   re=z() if coherent else None, im=z() if coherent else None,
                   energy=z() if coherent else None,
                   cross=z() if coherent else None,
                   num_w=z(), den_w=z(), num_b=z())
        center_geom: _PingGeometry | None = None
        best = np.inf
        for u in line_offsets:
            geom = self._ping_geometry(origin, look, fwd,
                                       along_offset=float(u), direct=direct)
            if abs(float(u)) < best:
                best, center_geom = abs(float(u)), geom
            self._shade_into(geom, origin, roll_toward, depression_sign,
                             float(u), n_lines, wall, acc, direct)
        assert center_geom is not None
        return acc, center_geom

    # ------------------------------------------------------ sensor mounting
    def _sensor_pose(self, side: Side, p: Pose3D) -> Pose3D:
        """Vehicle base pose -> transducer pose (rigid mount offset).

        The lever arm is rotated by yaw only: at ±0.20 m athwartship a 10°
        roll displaces the transducer by 3 mm, below the ground-sample
        step, so roll/pitch enter the imagery through the beam pattern and
        the fan plane (``_ping_geometry``), not through the mount offset.
        """
        cfg = self._cfg
        c, s = np.cos(p.yaw), np.sin(p.yaw)
        ox = cfg.mount_x_m
        oy = side.sign * cfg.mount_y_abs_m
        return Pose3D(
            x=p.x + c * ox - s * oy,
            y=p.y + s * ox + c * oy,
            z=p.z - cfg.sensor_depth_m,
            roll=p.roll, pitch=p.pitch, yaw=p.yaw,
        )

    # --------------------------------------------------------- geometry pass
    def _ping_geometry(self, sensor: Pose3D, look: float, fwd: float,
                       along_offset: float = 0.0,
                       direct: bool = False) -> _PingGeometry:
        acq = self._acq
        r_max = acq.range_max_m

        # Athwartship look direction and along-track (heading) direction
        # in the world frame, given explicitly: for a mirror source both are
        # reflected and neither follows from the pose's yaw.
        dx, dy = np.cos(look), np.sin(look)
        fx, fy = np.cos(fwd), np.sin(fwd)

        # Ground-line samples (horizontal distance y_k from the transducer),
        # shifted along-track by `along_offset` for azimuth-beam integration.
        ds = self._ds
        y_k = np.arange(ds, r_max + ds, ds)
        px = sensor.x + dx * y_k + fx * along_offset
        py = sensor.y + dy * y_k + fy * along_offset

        z_k = self._scene.sample_height(px, py)
        if sensor.pitch != 0.0:
            # Pitch rotates the fan plane about the body y axis, so a ground
            # sample `dz` below the transducer is imaged `dz·tan(pitch)`
            # along-track from where a level fan would put it -- the
            # symmetric port/starboard along-track distortion of a pitching
            # platform (Cobra, Oppenheim & Jaffe 1992). One refinement pass
            # re-samples the height at the shifted footprint. Zero pitch is
            # bit-identical (tan 0 = 0 adds exactly nothing).
            shift = (sensor.z - z_k) * np.tan(sensor.pitch)
            px = px + fx * shift
            py = py + fy * shift
            z_k = self._scene.sample_height(px, py)
        rho = self._scene.sample_reflectivity(px, py)

        dz = sensor.z - z_k                      # positive: seabed below sensor
        slant = np.hypot(y_k, dz)
        depression = np.arctan2(dz, y_k)         # 0 horizontal .. pi/2 nadir

        # Horizon culling: elevation angle (negative-down) must exceed the
        # running max of nearer samples to be insonified.
        elevation = -depression
        run_max = np.maximum.accumulate(elevation)
        visible = elevation >= run_max - 1e-9
        # No return from ground at or above the transducer: a dry beach or
        # an awash bank scatters nothing back into the water. (The horizon
        # cull above already hides everything beyond the waterline.) A
        # fully wet profile is untouched -- the mask is all-true there.
        visible &= dz > 0.0

        # Walls (direct path only): a wet face occludes every sample behind
        # it whose ray passes below the wall top -- a submerged sill is
        # looked over, a quay is not. Ghost passes leave this to the
        # crossing test in sonar/multipath.py.
        hits = self._wall_hits(sensor, dx, dy, r_max) if direct else []
        for h in hits:
            if h.z_top <= h.z_bottom:
                continue
            beyond = y_k > h.d
            z_cross = sensor.z + (z_k - sensor.z) * (h.d / np.maximum(y_k, 1e-9))
            # Only rays meeting the face's own band are stopped: a quay's
            # band reaches the seabed, a pontoon's stops at its draft and
            # the seabed beneath and beyond it stays insonified.
            visible &= ~(beyond & (z_cross <= h.wall.top_z)
                         & (z_cross >= h.z_bottom))

        # Local incidence from the along-profile slope.
        slope = np.gradient(z_k, ds)
        # Ray direction (horizontal, vertical) = (y_k, -dz)/slant; surface
        # normal (2D profile) = (-slope, 1)/sqrt(1+slope^2).
        denom = slant * np.sqrt(1.0 + slope ** 2)
        cos_inc = np.clip((dz + y_k * slope) / np.maximum(denom, 1e-9), 0.0, 1.0)

        altitude = float(sensor.z - self._scene.sample_height(
            np.array([sensor.x]), np.array([sensor.y]))[0])

        return _PingGeometry(slant=slant, visible=visible,
                             depression=depression, cos_incidence=cos_inc,
                             reflectivity=rho, altitude=max(altitude, 0.05),
                             px=px, py=py, seabed_z=z_k, wall_hits=hits)

    # ----------------------------------------------------------- shading pass
    def _shade_into(self, g: _PingGeometry, origin: Pose3D,
                    roll_toward_side: float, depression_sign: float,
                    along_offset: float, n_lines: int, wall,
                    acc: _Acc, direct: bool = False) -> None:
        """Accumulate one ground line's weighted contributions into the
        per-bin numerators (diffuse, coherent specular) and denominator
        (azimuth-beam integration).

        Each sample's azimuth weight is Gaussian in its along-track offset
        with sigma(R) = R * theta_h / 2.355: at short range the footprint is
        tiny, so offset lines contribute ~nothing (near-nadir stays sharp);
        at far range the footprint ~ R*theta spans the offsets and the
        result is the along-track average the real 0.5 deg beam sees.

        Diffuse (Lambert) and coherent specular contributions are kept
        separate because their fluctuation statistics differ (see
        sonar/noise.py).

        ``depression_sign`` is ``-1`` for a source mirrored an odd number of
        times in ``z = 0``: that path leaves the real transducer *upward*, so
        the vertical pattern must be evaluated on the other side of
        horizontal. ``wall``, when given, restricts the contribution to the
        samples the finite wall actually reflects toward."""
        acq, cfg = self._acq, self._cfg

        bs = acoustics.backscatter(g.reflectivity, g.cos_incidence,
                                   cfg.lambert_exponent)
        # Specular near-normal-incidence lobe: dominates at/near nadir and
        # produces the bright first-bottom-return line that FBR/bottom
        # tracking downstream locks onto.
        spec = acoustics.specular(g.reflectivity, g.cos_incidence, cfg)
        w = acoustics.beam_weight(depression_sign * g.depression, cfg,
                                  roll_toward_side)
        rng_resp = acoustics.net_range_response(g.slant, cfg)

        env = w * rng_resp * g.visible
        bounce_env = None
        if wall is not None:
            cross = crossing_mask(wall, origin, g.px, g.py, g.seabed_z)
            bw = bounce_weight(wall, origin, g.px, g.py, g.seabed_z,
                               cfg.wall_diffuse_fraction)
            # The folded (image) path meets the face twice.
            env = env * cross * bw * bw
            if self._direct_sensor is not None and cfg.wall_corner_gain > 0.0:
                # Single-bounce band: the transducer insonifies a ground
                # point directly and the echo returns off the face (or the
                # reverse), arriving at the mean of the direct and folded
                # path lengths -- for every point in front of a quay that
                # mean is just beyond the foot's range, so the whole strip
                # folds into a bright band right behind the face line.
                s_ = self._direct_sensor
                r_direct = np.hypot(np.hypot(g.px - s_.x, g.py - s_.y),
                                    s_.z - g.seabed_z)
                mid = 0.5 * (r_direct + g.slant)
                bounce_env = (bs * w * g.visible * cross * bw
                              * acoustics.net_range_response(mid, cfg)
                              * cfg.wall_corner_gain * (self._ds / acq.bin_size_m),
                              mid)
        contrib_d = bs * env
        contrib_s = spec * env

        # Azimuth (along-track) beam weight per sample.
        if n_lines > 1:
            sigma = np.maximum(g.slant * self._theta_h / 2.355, 1e-4)
            az_w = np.exp(-0.5 * (along_offset / sigma) ** 2)
        else:
            az_w = np.ones_like(g.slant)

        # Bin by slant range with azimuth weights.
        start_m = acq.range_start_mm / 1000.0
        bin_m = acq.bin_size_m
        n = acq.num_results
        idx = np.floor((g.slant - start_m) / bin_m).astype(np.int64)
        ok = (idx >= 0) & (idx < n)
        wd = (contrib_d * az_w)[ok]
        acc.num_d += np.bincount(idx[ok], weights=wd, minlength=n)
        acc.num_s += np.bincount(idx[ok], weights=(contrib_s * az_w)[ok],
                                 minlength=n)
        acc.den += np.bincount(idx[ok], weights=az_w[ok], minlength=n)
        if bounce_env is not None:
            b_amp, mid = bounce_env
            idx_b = np.floor((mid - start_m) / bin_m).astype(np.int64)
            ok_b = (idx_b >= 0) & (idx_b < n)
            acc.num_b += np.bincount(idx_b[ok_b], weights=(b_amp * az_w)[ok_b],
                                     minlength=n)

        # Coherent scatterer field: every ground *cell* this line crosses is
        # one scatterer with a phase fixed to the cell; the samples that
        # fall in the same cell and bin add in amplitude (they are the same
        # scatterer), distinct cells add as random phasors. The per-bin
        # energy sum is kept so the field can be normalised to the
        # incoherent mean exactly, and the same-cell product across a bin
        # boundary so the pulse convolution's expectation is exact too.
        if acc.re is not None and ok.any():
            phase, grp = self._scatter_phase(g.px[ok], g.py[ok])
            amp = np.sqrt(np.maximum(wd, 0.0))
            key = grp * n + idx[ok]
            keys, first, inv = np.unique(key, return_index=True,
                                         return_inverse=True)
            a_cell = np.bincount(inv, weights=amp, minlength=keys.size)
            b_cell = keys % n
            g_cell = keys // n
            ph = phase[first]
            acc.re += np.bincount(b_cell, weights=a_cell * np.cos(ph),
                                  minlength=n)
            acc.im += np.bincount(b_cell, weights=a_cell * np.sin(ph),
                                  minlength=n)
            acc.energy += np.bincount(b_cell, weights=a_cell ** 2,
                                      minlength=n)
            straddle = (g_cell[1:] == g_cell[:-1]) & (b_cell[1:] == b_cell[:-1] + 1)
            if straddle.any():
                acc.cross += np.bincount(
                    b_cell[:-1][straddle],
                    weights=(a_cell[:-1] * a_cell[1:])[straddle], minlength=n)

        if direct and g.wall_hits:
            self._face_echo_into(g, origin, roll_toward_side, depression_sign,
                                 along_offset, n_lines, acc)

    # ------------------------------------------------------- speckle texture
    def _texture(self, acc: _Acc) -> np.ndarray:
        """Unit-mean multiplicative texture from the coherent field.

        ``g_b = F_b / sqrt(E_b)`` has unit mean square in every lit bin; the
        pulse/receive envelope is convolved on ``g`` with the amplitude
        kernel, and the result is divided by its *exact* expectation --
        ``(1/w) sum lit`` plus the same-cell cross terms between adjacent
        bins -- so the texture's mean is 1 bin for bin whatever the cell
        size, the sample step or the kernel width."""
        e = acc.energy
        lit = e > 1e-30
        g = np.where(lit, (acc.re + 1j * acc.im) / np.sqrt(np.maximum(e, 1e-30)),
                     0.0)
        c = np.zeros_like(e)
        both = lit[:-1] & lit[1:]
        c[:-1][both] = acc.cross[:-1][both] / np.sqrt(e[:-1][both] * e[1:][both])
        w = self.smear_bins()
        if w > 1:
            g = np.convolve(g, np.full(w, 1.0 / np.sqrt(w)), mode="same")
            expect = (np.convolve(lit.astype(np.float64), np.ones(w), mode="same")
                      + 2.0 * np.convolve(c, np.ones(w - 1), mode="same")) / w
        else:
            expect = lit.astype(np.float64)
        return np.where(expect > 1e-9,
                        (g.real ** 2 + g.imag ** 2) / np.maximum(expect, 1e-9),
                        1.0)

    # ------------------------------------------------------- wall face echo
    def _face_echo_into(self, g: _PingGeometry, origin: Pose3D,
                        roll_toward_side: float, depression_sign: float,
                        along_offset: float, n_lines: int, acc: _Acc) -> None:
        """The nearest wet wall face along this line, as a coherent return.

        The face is vertical, so the incidence angle on it is the ray's
        depression: the return follows the same specular lobe as the nadir
        FBR (``specular_width_deg``) and therefore concentrates at the
        transducer's own depth, i.e. at slant range ~= horizontal distance --
        a bright line a few bins wide. Azimuth obliquity (a quay run along
        rather than across) dims it as cos^2 with a small floor.
        """
        acq, cfg = self._acq, self._cfg
        for h in g.wall_hits:
            if h.z_top <= h.z_bottom:
                continue
            n_pts = max(int(np.ceil((h.z_top - h.z_bottom) / self._ds)), 2)
            z = np.linspace(h.z_bottom, h.z_top, n_pts)
            dz = origin.z - z                      # >0: face point below sensor
            slant = np.hypot(h.d, dz)
            dep = np.arctan2(dz, h.d)
            w = acoustics.beam_weight(depression_sign * dep, cfg,
                                      roll_toward_side)
            sigma = np.radians(max(cfg.specular_width_deg, 1e-3))
            lobe = np.exp(-0.5 * (dep / sigma) ** 2)
            obliq = max(h.cos_az ** 2, cfg.wall_echo_obliquity_floor)
            # Incidence on a vertical face: the angle from its normal, i.e.
            # the depression combined with the azimuth obliquity. A rough
            # face returns diffusely over its whole height (Lambert in that
            # angle) on top of the specular lobe near the transducer's own
            # depth; a pontoon's rounded, fendered floats keep only a
            # fraction of the lobe.
            cos_face = np.clip(np.cos(dep) * h.cos_az, 0.0, 1.0)
            diffuse = cfg.wall_face_diffuse * cos_face ** cfg.lambert_exponent
            spec_frac = (cfg.wall_pontoon_specular
                         if h.wall.kind == "pontoon" else 1.0)
            # Near-range floor: nothing returns from nearer than the seabed
            # itself does -- the sub-altitude range is the transmit-ringing
            # regime, where the range law would otherwise explode a face
            # the boat brushes past (a crossed pontoon line) into a slab
            # brighter than anything real.
            r_eff = np.maximum(slant, g.altitude)
            amp = (h.wall.reflectivity * cfg.wall_echo_strength * w
                   * acoustics.net_range_response(r_eff, cfg)
                   * (spec_frac * obliq * lobe + diffuse))
            if n_lines > 1:
                s_az = np.maximum(slant * self._theta_h / 2.355, 1e-4)
                az_w = np.exp(-0.5 * (along_offset / s_az) ** 2)
            else:
                az_w = np.ones_like(slant)
            idx = np.floor((slant - acq.range_start_mm / 1000.0)
                           / acq.bin_size_m).astype(np.int64)
            ok = (idx >= 0) & (idx < acq.num_results)
            acc.num_w += np.bincount(idx[ok], weights=(amp * az_w)[ok],
                                     minlength=acq.num_results)
            acc.den_w += np.bincount(idx[ok], weights=az_w[ok],
                                     minlength=acq.num_results)
            break                                  # the nearest face hides the rest

    def _wall_contacts(self, side: Side, sensor: Pose3D,
                       g: _PingGeometry) -> list[GroundTruthContact]:
        """Ground truth for the wall face this ping images (at most one)."""
        acq = self._acq
        for h in g.wall_hits:
            if h.z_top <= h.z_bottom:
                continue
            r_foot = float(np.hypot(h.d, sensor.z - h.z_bottom))
            bin_m = acq.bin_size_m
            return [GroundTruthContact(
                object_id=-(h.index + 1), object_type="wall", side=side,
                slant_range_m=h.d,
                extent_bins=float(max(r_foot - h.d, bin_m) / bin_m),
                shadow_bins=float(max(acq.range_max_m - r_foot, 0.0) / bin_m),
                visible=True, wall_name=h.wall.name)]
        return []

    # --------------------------------------------------------- pulse smearing
    def _pulse_smear(self, power: np.ndarray,
                     amplitude: bool = False) -> np.ndarray:
        """Convolve with the transmit-pulse / receive-filter range envelope.

        The device's range resolution is c*tau/2 (tau = pulse duration),
        floored at ``range_filter_bins`` for the receive chain: every
        scatterer is smeared over that many bins, which both widens the FBR
        onset ramp and correlates neighbouring bins -- exactly what makes the
        real first return a multi-bin feature that persistence-based
        detectors key on. Boxcar envelope, unit gain on power; with
        ``amplitude=True`` the kernel is 1/sqrt(w) so that a complex field's
        expected power is smeared identically."""
        w = self.smear_bins()
        if w <= 1:
            return power
        kernel = np.full(w, 1.0 / np.sqrt(w) if amplitude else 1.0 / w)
        return np.convolve(power, kernel, mode="same")

    # ------------------------------------------------------------- multipath
    def _apply_multipath(self, power: np.ndarray,
                         altitude: float) -> np.ndarray:
        """Optional shallow-water second-bottom-echo ghost (off by default).

        In shallow enclosed water the bottom-surface-bottom path re-images
        the seabed displaced by ~one altitude in slant range, producing the
        dim ghost seabed line the real device shows in ports. Modelled as
        the direct response shifted by `altitude` and scaled by
        ``multipath_gain`` (the extra spreading/absorption and the two
        boundary reflection losses are lumped into that single gain --
        a first-order model, documented in docs/sonar_model.md)."""
        cfg = self._cfg
        if not cfg.multipath_enabled or cfg.multipath_gain <= 0.0:
            return power
        shift = int(round(altitude / self._acq.bin_size_m))
        if shift <= 0 or shift >= power.size:
            return power
        ghost = np.zeros_like(power)
        ghost[shift:] = power[: power.size - shift]
        return power + cfg.multipath_gain * ghost

    # -------------------------------------------------------- wall multipath
    def _render_ghosts(self, side: Side, sensor: Pose3D, altitude: float
                       ) -> tuple[np.ndarray | None, list[GroundTruthContact]]:
        """Mirror-source ghosts off the basin walls and the water surface.

        Each virtual source is rendered exactly like the direct path and its
        response summed into the *same* range bins -- the bin it lands in is
        the folded path length, so a ghost sits where its physics puts it.

        **Where the energy goes, and what statistics it carries.** Ghost power
        joins the ping's *diffuse* channel and never the coherent specular
        one. Two consequences, both wanted: the coherent first-bottom-return
        channel that downstream bottom tracking locks onto is untouched by
        this feature (NC #4 holds by construction, not by tuning), and a ghost
        carries fully-developed Exp(1) speckle rather than the low-CV
        coherent statistic -- right, because a ghost of the nadir return has
        bounced off a rough boundary and decorrelated. The direct field's own
        statistics are unchanged either way (NC #5).
        """
        cfg = self._cfg
        if not cfg.wall_multipath_enabled:
            return None, []
        # Enabled by default, and a strict no-op without reflectors: a
        # scene with no walls (including minimal stub scenes that carry
        # no `walls` attribute at all) renders bit-identically.
        walls = getattr(self._scene, "walls", None) or []
        if not walls and not cfg.surface_mirror_enabled:
            return None, []
        sources = mirror_sources(
            walls, sensor, side, self._acq.range_max_m,
            wall_gain=cfg.wall_multipath_gain,
            surface_enabled=cfg.surface_mirror_enabled,
            surface_reflectivity=cfg.surface_reflectivity)
        if not sources:
            return None, []

        total = np.zeros(self._acq.num_results, dtype=np.float64)
        contacts: list[GroundTruthContact] = []
        for src in sources:
            acc, geom = self._integrate(
                src.origin, src.look, src.fwd, src.roll_toward_side,
                src.depression_sign, self._ghost_offsets,
                self._n_ghost_lines, src.wall)
            den = acc.den
            safe = np.maximum(den, 1e-12)
            ghost = np.where(den > 1e-12, (acc.num_d + acc.num_s) / safe, 0.0)
            total += src.gain * self._pulse_smear(ghost)
            if src.wall is not None and acc.num_b.any():
                total += src.gain_single * self._pulse_smear(acc.num_b)
            # Shadow length is governed by the source's height over the
            # seabed at the object, which the fold preserves -- the mirror's
            # own "altitude" would be measured outside the map.
            geom.altitude = altitude
            contacts += self._ground_truth_contacts(
                side, src.origin, src.look, src.fwd, geom, altitude,
                via=src.name, wall=src.wall)
        return total, contacts

    # ------------------------------------------------------------ ground truth
    def _ground_truth_contacts(self, side: Side, sensor: Pose3D,
                               look: float, fwd: float, g: _PingGeometry,
                               altitude: float, via: str = "",
                               wall=None) -> list[GroundTruthContact]:
        """Which scene objects does *this* ping insonify, and where?

        An object is 'in this ping' if its centre lies within half the
        along-track resolution cell (beam footprint + object length) of the
        ping's ground line. Shadow length uses the classic flat-bottom
        approximation  L_s = h_obj * r / altitude.

        Called once for the direct path and once per mirror source. In the
        mirrored case ``sensor``/``look``/``fwd`` describe the virtual
        transducer, so the geometry falls out unchanged and every ghost
        arrives labelled with the object it images and the reflector that
        produced it -- ghosts are ground truth here, not unlabelled clutter.
        """
        acq, cfg = self._acq, self._cfg
        contacts: list[GroundTruthContact] = []
        dxl, dyl = np.cos(look), np.sin(look)
        fwd_x, fwd_y = np.cos(fwd), np.sin(fwd)
        half_beam = np.radians(cfg.horizontal_aperture_deg) / 2.0

        for o in self._scene.objects:
            rx, ry = o.x - sensor.x, o.y - sensor.y
            across = rx * dxl + ry * dyl          # >0: on this side
            along = rx * fwd_x + ry * fwd_y
            if across <= 0.2 or across > acq.range_max_m:
                continue
            footprint_along = across * np.tan(half_beam) + o.footprint_radius
            if abs(along) > footprint_along:
                continue

            z_obj = float(self._scene.sample_height(np.array([o.x]),
                                                    np.array([o.y]))[0])
            slant = float(np.hypot(across, sensor.z - z_obj))
            if slant > acq.range_max_m:
                continue
            # A finite wall only ghosts what it actually reflects toward.
            if wall is not None and not point_crosses(wall, sensor, o.x, o.y,
                                                      z_obj):
                continue

            # Occlusion check against rendered visibility near the object.
            k = int(np.clip(round(across / self._ds) - 1,
                            0, len(g.visible) - 1))
            k0, k1 = max(0, k - 3), min(len(g.visible), k + 4)
            visible = bool(g.visible[k0:k1].any()) and o.effective_height > 0.005

            bin_m = acq.bin_size_m
            extent_bins = max(o.footprint_radius * 2.0, bin_m) / bin_m
            shadow_m = o.effective_height * slant / max(altitude, 0.1)
            contacts.append(GroundTruthContact(
                object_id=o.object_id, object_type=o.type, side=side,
                slant_range_m=slant, extent_bins=float(extent_bins),
                shadow_bins=float(shadow_m / bin_m), visible=visible,
                ghost=bool(via), via=via))
        return contacts
