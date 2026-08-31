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

5. **Wall / surface multipath** (optional, off by default): each
   reflecting boundary declared by the world config contributes a mirror
   source, rendered by the same passes and summed into the same range
   bins, so a basin's ghost returns land at their true folded path length
   with per-ghost ground truth attached. See sonar/multipath.py.

Known simplifications (see docs/sonar_model.md for the full list):
straight rays, static scene, stop-and-hop pings (no intra-ping motion),
and reflections only to first order -- the direct path plus one bounce
per mirror source.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass

import numpy as np

from ..core.types import (GroundTruthContact, Ping, Pose3D, RenderedPing,
                          Side)
from ..worldgen.objects import CATALOG
from ..worldgen.scene import SceneModel
from . import acoustics
from .config import (SPEED_OF_SOUND_MPS, AcquisitionParams,
                     SonarModelConfig)
from .multipath import crossing_mask, mirror_sources, point_crosses


class SonarRenderer(abc.ABC):
    """Backend interface: pose -> one rendered ping for one side."""

    @abc.abstractmethod
    def render(self, side: Side, vehicle_pose: Pose3D, t_sim: float) -> RenderedPing:
        """Render one noiseless ping (noise is applied by the caller so that
        renderers stay deterministic and comparable)."""


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


class GeometricRenderer(SonarRenderer):
    """Heightfield ray model with horizon-culling shadows (see module doc)."""

    def __init__(self, scene: SceneModel, acquisition: AcquisitionParams,
                 model: SonarModelConfig) -> None:
        self._scene = scene
        self._acq = acquisition
        self._cfg = model

        # Ground-line sample step, coupled to the slant-bin size so that
        # every range bin receives samples at any num_results (600, 1200,
        # ...). The previous fixed 5 cm step left ~50% of far-range bins
        # empty at 600 bins (2.5 cm bins) -- an aliasing artifact that was
        # masked by the noise floor. Floor at 4 mm for cost sanity.
        self._ds = max(min(model.sample_step_m, 0.45 * acquisition.bin_size_m),
                       0.004)

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

        # Ghost renders get their own (by default coarser) azimuth sampling:
        # a multipath ghost is dim and already smeared by the bounce, so
        # paying the direct path's line count once per mirror source buys
        # nothing. This is the cost lever when a basin has several walls.
        gk = max(int(model.ghost_beam_lines), 1)
        self._n_ghost_lines = gk if gk % 2 == 1 else gk + 1
        self._ghost_offsets = (
            np.linspace(-half_fp, half_fp, self._n_ghost_lines)
            if self._n_ghost_lines > 1 else np.array([0.0]))

    # ------------------------------------------------------------------ API
    def render(self, side: Side, vehicle_pose: Pose3D, t_sim: float) -> RenderedPing:
        sensor = self._sensor_pose(side, vehicle_pose)
        acq = self._acq

        # Direct path. Look/forward directions are passed explicitly rather
        # than re-derived downstream, because a mirror source's are reflected
        # and cannot be recovered from a yaw plus a side sign.
        look = sensor.yaw + side.sign * np.pi / 2.0
        roll_toward = -side.sign * sensor.roll
        num_d, num_s, den, center_geom = self._integrate(
            sensor, look, sensor.yaw, roll_toward, 1.0,
            self._line_offsets, self._n_lines, None)

        safe = np.maximum(den, 1e-12)
        power = np.where(den > 1e-12, num_d / safe, 0.0)
        specular = np.where(den > 1e-12, num_s / safe, 0.0)

        power = self._pulse_smear(power)
        specular = self._pulse_smear(specular)
        # The second-bottom-echo ghost images the *direct* response only, so
        # it is applied before wall ghosts are summed in -- the two multipath
        # features stay independent instead of ghosting each other.
        power = self._apply_multipath(power, center_geom.altitude)

        contacts = self._ground_truth_contacts(
            side, sensor, look, sensor.yaw, center_geom,
            center_geom.altitude)
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
        )
        return RenderedPing(ping=ping, contacts=contacts)

    # --------------------------------------------------- azimuth integration
    def _integrate(self, origin: Pose3D, look: float, fwd: float,
                   roll_toward: float, depression_sign: float,
                   line_offsets: np.ndarray, n_lines: int,
                   wall) -> tuple[np.ndarray, np.ndarray, np.ndarray,
                                  _PingGeometry]:
        """Integrate the azimuth beam for one source (direct or mirrored).

        One ground line per along-track offset, each sample weighted by the
        range-dependent azimuth pattern. The centre line (offset 0) provides
        altitude and ground-truth geometry.
        """
        acq = self._acq
        num_d = np.zeros(acq.num_results, dtype=np.float64)   # diffuse
        num_s = np.zeros(acq.num_results, dtype=np.float64)   # coherent specular
        den = np.zeros(acq.num_results, dtype=np.float64)
        center_geom: _PingGeometry | None = None
        for u in line_offsets:
            geom = self._ping_geometry(origin, look, fwd, along_offset=float(u))
            if u == 0.0:
                center_geom = geom
            self._shade_into(geom, origin, roll_toward, depression_sign,
                             float(u), n_lines, wall, num_d, num_s, den)
        assert center_geom is not None
        return num_d, num_s, den, center_geom

    # ------------------------------------------------------ sensor mounting
    def _sensor_pose(self, side: Side, p: Pose3D) -> Pose3D:
        """Vehicle base pose -> transducer pose (rigid mount offset)."""
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
                       along_offset: float = 0.0) -> _PingGeometry:
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
        rho = self._scene.sample_reflectivity(px, py)

        dz = sensor.z - z_k                      # positive: seabed below sensor
        slant = np.hypot(y_k, dz)
        depression = np.arctan2(dz, y_k)         # 0 horizontal .. pi/2 nadir

        # Horizon culling: elevation angle (negative-down) must exceed the
        # running max of nearer samples to be insonified.
        elevation = -depression
        run_max = np.maximum.accumulate(elevation)
        visible = elevation >= run_max - 1e-9

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
                             px=px, py=py, seabed_z=z_k)

    # ----------------------------------------------------------- shading pass
    def _shade_into(self, g: _PingGeometry, origin: Pose3D,
                    roll_toward_side: float, depression_sign: float,
                    along_offset: float, n_lines: int, wall,
                    num_d: np.ndarray, num_s: np.ndarray,
                    den: np.ndarray) -> None:
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
        if wall is not None:
            env = env * crossing_mask(wall, origin, g.px, g.py, g.seabed_z)
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
        idx = np.floor((g.slant - start_m) / bin_m).astype(np.int64)
        ok = (idx >= 0) & (idx < acq.num_results)
        num_d += np.bincount(idx[ok], weights=(contrib_d * az_w)[ok],
                             minlength=acq.num_results)
        num_s += np.bincount(idx[ok], weights=(contrib_s * az_w)[ok],
                             minlength=acq.num_results)
        den += np.bincount(idx[ok], weights=az_w[ok],
                           minlength=acq.num_results)

    # --------------------------------------------------------- pulse smearing
    def _pulse_smear(self, power: np.ndarray) -> np.ndarray:
        """Convolve with the transmit-pulse range envelope.

        The device's range resolution is c*tau/2 (tau = pulse duration):
        every scatterer is smeared over that many bins by the transmitted
        pulse, which both widens the FBR onset ramp and correlates
        neighbouring bins -- exactly what makes the real first return a
        multi-bin feature that persistence-based detectors key on. Boxcar
        envelope (rectangular pulse), unit gain."""
        cfg = self._cfg
        if not cfg.pulse_smearing:
            return power
        tau = self._acq.pulse_duration_s(cfg.max_ping_rate_hz)
        w = int(round((SPEED_OF_SOUND_MPS * tau / 2.0) / self._acq.bin_size_m))
        if w <= 1:
            return power
        kernel = np.full(w, 1.0 / w)
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
        sources = mirror_sources(
            self._scene.walls, sensor, side, self._acq.range_max_m,
            wall_gain=cfg.wall_multipath_gain,
            surface_enabled=cfg.surface_mirror_enabled,
            surface_reflectivity=cfg.surface_reflectivity)
        if not sources:
            return None, []

        total = np.zeros(self._acq.num_results, dtype=np.float64)
        contacts: list[GroundTruthContact] = []
        for src in sources:
            num_d, num_s, den, geom = self._integrate(
                src.origin, src.look, src.fwd, src.roll_toward_side,
                src.depression_sign, self._ghost_offsets,
                self._n_ghost_lines, src.wall)
            safe = np.maximum(den, 1e-12)
            ghost = np.where(den > 1e-12, (num_d + num_s) / safe, 0.0)
            total += src.gain * self._pulse_smear(ghost)
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
