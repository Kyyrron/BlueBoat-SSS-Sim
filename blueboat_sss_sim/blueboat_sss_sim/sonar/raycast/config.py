"""The ``raycast:`` section of a sonar profile (strict: unknown keys raise).

Every value is physical and cited, or a numerical setting chosen by the
convergence study (``raycast_design.md`` §6), or one of the fitted
parameters of §8 (with bounds). The shared encoder / noise stages keep
reading the profile's ``model:`` section (``SonarModelConfig``), whose
``calibration_db_offset`` is this sensor's system gain.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Any, Mapping


def _strict(cls, d: Mapping[str, Any] | None, where: str):
    d = dict(d or {})
    known = {f.name for f in fields(cls)}
    bad = set(d) - known
    if bad:
        raise KeyError(f"Unknown raycast.{where} keys: {sorted(bad)}")
    return cls(**d)


@dataclass(frozen=True)
class WaterConfig:
    #: Every corpus site is seawater (owner, 2026-11-03); T and S unknown.
    #: Default = the mean of the per-site NOAA OISST estimates (harbour 28.5,
    #: beach 27.5, pool 26.0 degC; note 07), band +-3 degC; per-site records
    #: override it for corpus replays. Never fitted.
    temperature_c: float = 27.3
    #: Standard seawater (J&R App. B.3), band +-3 ppt.
    salinity_ppt: float = 35.0
    ph: float = 8.0
    mean_depth_m: float = 1.0


@dataclass(frozen=True)
class BeamConfig:
    #: Datasheet: "0.5 deg, the nominal theoretical 2-way 3 dB beam width"
    #: -> a uniform line aperture of ~24.4 cm (beam.aperture_from_width).
    two_way_width_deg: float = 0.5
    #: Along-track aperture [m]; 0 = derived from ``two_way_width_deg``
    #: (24.6 cm). Bounded [0.224, 0.310] m: datasheet far field 15 m / the
    #: CAD housing length 318 mm (owner, 2026-11-03). Fitted (Phase 7) only
    #: if the corpus's near-range footprints constrain it.
    aperture_length_m: float = 0.0
    #: Datasheet "beam height 50 deg", taken as one-way -3 dB.
    height_deg: float = 50.0
    #: Large-angle vertical response (one-way amplitude, added in power to
    #: the strip's sinc): the nadir level the first bottom return (NC #4)
    #: depends on. Unpublished; bounded [0, 0.3]; set by the NC #4 step,
    #: fitted in Phase 7 with roll to separate it from the seabed law.
    vertical_floor_amp: float = 0.0
    #: Pattern evaluation: near-field aperture integral along track.
    near_field: bool = True
    #: Contributors are kept while the two-way pattern exceeds this [dB re
    #: on-axis far-field]; sidelobes feed the shadow floor (§3).
    floor_db: float = -40.0
    #: Amplitude weighting of the along-track aperture (``beam.taper``): 1 =
    #: uniform (default); < 1 = a cosine on that pedestal. The aperture is
    #: then lengthened so that the two-way -3 dB width stays
    #: ``two_way_width_deg`` (``aperture_length_m`` 0). The vendor's drawing
    #: shows a 250 mm window (session 01, ``inputs/beam_spec.md``): 0.819 is
    #: the strongest taper it allows (first side lobe -14.5 dB one-way against
    #: -13.3 dB). Bounded [0.3, 1]. It moves the seabed mean by 0.02-0.10 dB.
    taper_pedestal: float = 1.0
    #: True = the pre-2026-11-13 sign of the near-field pattern's phase (the
    #: conjugate of the carrier's convention: a bug, ``beam.py``). It changes
    #: the coherent echo of extended smooth surfaces (object flashes) and
    #: nothing else (seabed mean < 1e-8 dB). Only the dated profile copies
    #: (``*_20261112_sonar.yaml``) set it, to reproduce the renders the
    #: 2026-11-12 review was made with.
    legacy_phase_sign: bool = False


@dataclass(frozen=True)
class SamplingConfig:
    #: World lattice spacing of the diffuse scatterers [m]. Provisional,
    #: from the Phase-2 S1 sweep (flat sand, mean vs quadrature: rms 0.10 /
    #: 0.025 / 0.006 dB at 10 / 5 / 2.5 mm, ~spacing^2); final value by the
    #: Phase-5 convergence study, never relaxed for speed -- except the
    #: ``fast`` variant (owner, 2026-11-10: 10 mm; the flat-sand density sweep
    #: gives eps_mu p95 0.035 / 0.023 there, about 0.15 dB of mean level, and
    #: a redrawn speckle realisation; raycast_fast_optimize_plan.md phase 2).
    scatterer_spacing_m: float = 0.005
    #: Maximum refinement per axis of a steep sheet cell (a stratified
    #: n_x x n_y sub-grid, n = ceil of the surface stretch along that axis).
    max_refine: int = 8
    #: Offset of a visibility ray's end off the surface [m].
    surface_offset_m: float = 1e-3
    #: FAST-VARIANT ENGINE PATH (2026-11-10; ``ENGINE_KNOBS``): do not create
    #: scatterers no sound can reach -- any point above the still waterline
    #: (z > 0: dry land, wall tops), wall points buried below the local
    #: seabed by more than ``surface_offset_m``, and whole faces of a
    #: ``kind: quay`` wall turned away from the transducer (a quay is backed
    #: by land; pontoons keep every face). Off = today's sampling, exactly.
    cull_unreachable: bool = False
    #: FAST-VARIANT ENGINE PATH (2026-11-10, phase 2; ``ENGINE_KNOBS``): emit
    #: only the seabed-sheet points inside the beam band at their own range
    #: (``R <= r_max`` and ``|(X - P) . e_a| <= w(R)``, the band the walls
    #: already use) instead of the band's widest parallelogram everywhere. An
    #: out-of-band point is never a direct candidate (its two-way pattern is
    #: below the floor), so a ping that keeps no multipath sequence renders
    #: exactly as without it; with a kept wall / surface sequence such a
    #: point could still be lit through the bounce, so there it is
    #: near-exact. Off = today's sampling, exactly.
    band_cull: bool = False
    #: Longest acquisition range [m] this profile may be used at; 0 = no
    #: limit (default). The harbour profiles set 30: beyond it the 8 GB GPU
    #: runs short (40 m: 6.3 GB, 50 m: out of memory), their measured near
    #: range no longer applies (pulse above 50 us), and nothing was verified
    #: there (the thesis surveys at 20 m). A longer range raises, with this
    #: explanation, instead of rendering something unverified. Changes no
    #: render below the limit.
    max_range_m: float = 0.0


@dataclass(frozen=True)
class SeabedLawConfig:
    #: "ssa": the J&R small-slope roughness law + sediment volume (default,
    #: scattering.DiffuseLaw); "lambert": mu sin(theta_i) sin(theta_s), the
    #: integration machinery's test law (Mackenzie 1961, mu = -27 dB).
    law: str = "ssa"
    lambert_mu_db: float = -27.0
    #: Per-sediment overrides of the J&R parameters the 450 kHz law
    #: extrapolates (``seabed_ssa.SEDIMENTS``): ``{name: {w2, gamma2,
    #: sigma2}}``. Fitted in Phase 7 (raycast_design §8), each inside the
    #: bounds of ``seabed_ssa.OVERRIDE_BOUNDS`` (J&R Table 13.1, eq. 14.9);
    #: empty = the literature defaults.
    sediments: Mapping[str, Mapping[str, float]] = field(default_factory=dict)


@dataclass(frozen=True)
class ReceiverConfig:
    #: The receive filter's duration as a multiple of the transmit pulse.
    #: MEASURED (2026-11-03, X2, ``raycast_phase2.md``): the speckle's range
    #: correlation (lags 1-3, structure floor removed) fits a boxcar filter
    #: of 1.43 x the pulse on the calibration sessions (median; validation
    #: ~1.3). It tracks the pulse (0.8-1.5 x over pulses of 44-208 us), not a
    #: fixed share of the round trip (0.14-0.36 %): a CW tone burst with a
    #: near-matched receive filter. Provisional shape (the measured lag-3
    #: tail is heavier than a boxcar's).
    filter_pulse_ratio: float = 1.4
    #: Legacy alternative: a share of the round-trip window (the driver's
    #: ``filter_duration_percent``, 0.0015). Used only when the ratio is 0.
    filter_duration_percent: float = 0.0
    #: One bin of integration in the range envelope.
    bin_integration: bool = True
    #: The device's time-varied gain, ``k log10(r)`` dB on the output (r the
    #: device range, m). Undocumented by the maker; the geometric fit says
    #: partial compensation. Bounded [0, 40] (none .. full spherical two-way
    #: spreading); fitted (raycast_design §8). Provisional value from the
    #: NC #4 step (raycast_phase2.md).
    tvg_k: float = 0.0


@dataclass(frozen=True)
class PoConfig:
    #: The PO patches of smooth solids are tessellated with the facet layer's
    #: own code (``solids.tessellate`` via ``facets.object_patches``) but a
    #: curvature criterion this many times stricter (the sagitta phase falls
    #: by the same factor: 0.05 rad -> 0.003 rad at 16). Measured (Phase 2,
    #: O2): the default criterion leaves a cylinder (a = 5 cm) 0.20 dB under
    #: k a L^2 even at 500 m -- the planar tiling acts as a grating -- and 16x
    #: converges to the exact PO integral within 0.01 dB. A numerical
    #: setting, confirmed by the Phase-5 study.
    curvature_refine: float = 16.0
    #: Patch size along the flat directions [m]; 0 = the shared
    #: ``SonarModelConfig.facet_patch_m`` (1 cm).
    patch_m: float = 0.0


@dataclass(frozen=True)
class SpecularConfig:
    """The seabed sheet's coherent (mean-field) channel (``specular.py``):
    numerical settings only, converged in Phase 5 (raycast_design §6)."""

    enabled: bool = True
    #: Stationary-phase window in Fresnel zones of the specular direction:
    #: weight 1 inside ``window_n1``, raised-cosine taper to 0 at
    #: ``window_n2``. Set by the S4/S5 truncation study (raycast_phase2.md).
    window_n1: float = 8.0
    window_n2: float = 30.0
    #: Patch size from the quadratic-phase criterion k a^2 / (4 R) <= this
    #: [rad], clipped to [patch_min_m, patch_max_m].
    patch_phase_rad: float = 0.02
    patch_min_m: float = 0.002
    patch_max_m: float = 0.02


@dataclass(frozen=True)
class PathsConfig:
    """Two-way multipath (``multipath.py``, ``chains.py``)."""

    enabled: bool = True
    #: Hard cap on specular bounces per one-way chain; the energy cut-off
    #: (``cutoff_db``) normally stops earlier (design §2.2).
    max_order: int = 12
    #: A sequence of chains is kept while its energy bound exceeds the
    #: bin's (noise + reverberation) minus this [dB].
    cutoff_db: float = 10.0
    #: RMS height of the water surface (Eckart coherent fraction of every
    #: surface bounce). JUDGED open-water default, the geometric profile's
    #: ``surface_rms_height_m`` (wind ripple, ~cm: coherent ~0 at the comb's
    #: angles); a calm pool is a site record (pass-4 overlay 3e-4 m). From the
    #: sea state when the node has one (Phase 6).
    surface_rms_m: float = 0.02
    floor_bounces: bool = True
    #: Wall faces as bounce boundaries (ghosts via quays), at most
    #: ``max_wall_bounces`` per chain.
    walls: bool = True
    max_wall_bounces: int = 1
    #: Planar faces of smooth solids as GO bounces after the PO bounce
    #: (corner paths: dihedral, trihedral; O4/O5), at most this many.
    object_faces: bool = True
    max_object_bounces: int = 2
    #: FAST-VARIANT ENGINE PATH (2026-11-10; ``ENGINE_KNOBS``): reuse a
    #: side's echo-path selection (which sequences to evaluate) for up to
    #: this many pings while the transducer moved < ``reselect_move_m``,
    #: turned < 2 deg and the set of boundaries in range is unchanged. The
    #: kept sequences are still evaluated exactly every ping. 1 = select
    #: every ping (today). GPU pipeline only.
    reselect_every: int = 1
    reselect_move_m: float = 0.0
    #: Until 2026-11-14 the complex reflection coefficient of the seabed and
    #: of wall faces entered the coherent sums in the exp(-i omega t) form of
    #: the textbook, against this renderer's exp(-i omega tau) carrier: the
    #: phase of every such bounce was mirrored, its level right
    #: (``specular.MaterialSurface``). True restores that; the dated profile
    #: copies only.
    legacy_reflection_phase: bool = False

    def __post_init__(self) -> None:
        if int(self.reselect_every) < 1:
            raise ValueError(f"raycast.paths.reselect_every {self.reselect_every} < 1")
        if float(self.reselect_move_m) < 0.0:
            raise ValueError(f"raycast.paths.reselect_move_m {self.reselect_move_m} < 0")


@dataclass(frozen=True)
class DiffractionConfig:
    """Fresnel-Kirchhoff at shadow edges (``diffraction.py``); numerical
    settings, converged in Phase 5 (D1 study: 16 directions -> 0.20 dB)."""

    enabled: bool = True
    window_n1: float = 6.0
    window_n2: float = 20.0
    sectors: int = 16
    coarse: int = 20
    bisect: int = 7
    #: geometry within this distance of a contributor is its own support
    #: (covered by the scattering law), never an occluder [m]
    end_margin_m: float = 0.05
    #: clear legs whose contributor lies within this cell of a blocked one
    #: are tested for the lit side of a shadow edge [m]
    neighbour_cell_m: float = 0.25
    #: FAST-VARIANT ENGINE PATH (2026-11-10; ``ENGINE_KNOBS``): "sampled" =
    #: the ray-traced Fresnel-Kirchhoff disc (today); "edge" = the closed
    #: form F(v) of the nearest free straight edge of a wall box
    #: (``diffraction.EdgeDiffraction``: same per-leg ratio, same window,
    #: no ray casts; objects and seabed relief get no factor).
    method: str = "sampled"

    def __post_init__(self) -> None:
        if self.method not in ("sampled", "edge"):
            raise ValueError(f"raycast.diffraction.method {self.method!r}: 'sampled' or 'edge'")


@dataclass(frozen=True)
class CorrectionConfig:
    """The EMPIRICAL correction layer (``correction.py``; NOT physics),
    separately switchable. Off by default: the physical renderer's output is
    then bit-identical. ``artifact``: a ``blueboat_raycast_correction/1`` YAML
    fitted on calibration sessions only (gates amendment 6b)."""
    enabled: bool = False
    artifact: str = ""
    #: EDGE-HOLD (2026-11-10, harbour patch): ``[low, high]`` nadir altitudes
    #: [m] outside the artifact's fitted domain where its table is still
    #: applied, at the altitude clamped to its knots (``Correction.db_at``),
    #: status ``applied:held``; below ``low`` it fades linearly to 0 over
    #: ``altitude_taper_m`` (``applied:taper``) instead of a cliff. ``None``:
    #: off outside the domain, as the artifact states. A profile-level
    #: extension, never an edit of the artifact; evidence in
    #: ``.claude/raycast-live.md`` (harbour profiles).
    altitude_hold_m: Any = None
    altitude_taper_m: float = 1.0
    #: Fade the table in over this slant distance after the first seabed
    #: echo [m] (0 = the table as fitted). The real first return climbs from
    #: the physical model's own level; the table's maximum on the first bin
    #: made a hard nadir edge (``correction.py``; harbour profiles).
    onset_fade_m: float = 0.0
    #: Accept the artifact under the performance-only engine keys
    #: (``PERFORMANCE_KEYS``): the core check then compares the physics
    #: only. Off: strict (a fast render is refused), as before.
    accept_engine_knobs: bool = False

    def __post_init__(self) -> None:
        h = self.altitude_hold_m
        if h is not None:
            if not (isinstance(h, (list, tuple)) and len(h) == 2
                    and 0.0 < float(h[0]) < float(h[1])):
                raise ValueError(f"raycast.correction.altitude_hold_m {h!r}: [low, high], 0 < low < high")
            object.__setattr__(self, "altitude_hold_m", (float(h[0]), float(h[1])))
        if not float(self.altitude_taper_m) >= 0.0:
            raise ValueError(f"raycast.correction.altitude_taper_m {self.altitude_taper_m}: >= 0")
        if not 0.0 <= float(self.onset_fade_m) <= 5.0:
            raise ValueError(f"raycast.correction.onset_fade_m {self.onset_fade_m}: [0, 5] m")


@dataclass(frozen=True)
class NearRangeConfig:
    """The device's MEASURED near-range response for ONE acquisition mode
    (harbour patch v2, 2026-11-10): the transmit ringing (the coherent
    channel, ``Ping.specular``) and the water-column level (the noise
    stage's Exp(1) ``watercolumn_noise``), in reported dB at gain 4. The
    shared model's values (55 dB decaying 22 dB/m, water column 27.8 dB)
    were measured on 600-bin recordings; at 1200 bins / 44.3 us the device
    rings 3x shorter and its water column is ~12 dB quieter, which put the
    processor's FBR threshold (its noise estimate right after the ringing
    + 8 dB) above a correctly levelled seabed. Off (``ringing_db`` None):
    the shared model's, bit-identical. Applies only to the acquisition it
    was measured at (``num_results``, ``pulse_us``); elsewhere the shared
    model's, logged once."""
    #: ``[[r_m, dB], ...]`` piecewise linear in device range, 0 beyond the last point
    ringing_db: Any = None
    #: the Exp(1) water column's linear mean [reported dB]
    watercolumn_db: float = 0.0
    num_results: int = 0
    pulse_us: Any = None

    def __post_init__(self) -> None:
        t = self.ringing_db
        if t is None:
            return
        rr = [float(a) for a, _ in t]
        if len(rr) < 2 or any(b <= a for a, b in zip(rr, rr[1:])) or rr[0] < 0.0:
            raise ValueError("raycast.near_range.ringing_db: >= 2 [r_m, dB] rows, r increasing from >= 0")
        object.__setattr__(self, "ringing_db", tuple((float(a), float(b)) for a, b in t))
        pu = self.pulse_us
        if pu is not None:
            if not (isinstance(pu, (list, tuple)) and len(pu) == 2 and float(pu[0]) <= float(pu[1])):
                raise ValueError(f"raycast.near_range.pulse_us {pu!r}: [low, high]")
            object.__setattr__(self, "pulse_us", (float(pu[0]), float(pu[1])))

    def applies(self, acq, max_ping_rate_hz: float = 0.0) -> bool:
        """On, and ``acq`` is the acquisition it was measured at."""
        if self.ringing_db is None:
            return False
        if self.num_results and int(acq.num_results) != int(self.num_results):
            return False
        if self.pulse_us is not None:
            pulse = float(acq.pulse_duration_s(max_ping_rate_hz)) * 1e6
            if not (self.pulse_us[0] - 0.5 <= pulse <= self.pulse_us[1] + 0.5):
                return False
        return True

    def ringing_at(self, r_m) -> "Any":
        """The ringing [reported dB] at device ranges ``r_m``; -inf beyond the table."""
        import numpy as np
        r = np.asarray(r_m, dtype=np.float64)
        t = np.asarray(self.ringing_db, dtype=np.float64)
        out = np.interp(r, t[:, 0], t[:, 1])
        return np.where(r <= t[-1, 0], out, -np.inf)


@dataclass(frozen=True)
class FloorConfig:
    """The shadow (reverberation) floor of the shared noise stage, NC #15
    (``sonar/noise.py``). ``reverberation_db`` 0 (default): the shadow
    floor comes from the renderer's physics only (raycast_design §3), the
    output bit-identical. ``> 0`` (harbour profiles, 2026-11-10): a culled
    bin holds the seabed's own speckle this many dB under the lit seabed at
    that range, the ping's ambient reference (``floor.ambient_reference``)
    -- the geometric sensor's MEASURED stand-in (the corpus quay, 17-25 dB;
    18) for the fill this sensor does not model yet: volume and incoherent
    surface reverberation, higher-order multipath (test D3 never ran)."""
    reverberation_db: float = 0.0
    #: How the floor meets the echo in a LIT bin (review of 2026-11-11).
    #: "power" (default, as before): an independent Exp(1) floor is added to
    #: the intensity, which fills the speckle's deep nulls -- measured: the
    #: 1st percentile of the plain seabed rises from -20.0 to -16.7 dB re
    #: the mean (real harbour -20.5). "field": the floor is an independent
    #: complex Gaussian FIELD added before detection (``noise.py``), so a lit
    #: bin stays exactly exponential at the summed mean; a shadow is filled
    #: as before. Harbour profiles: field.
    addition: str = "power"

    def __post_init__(self) -> None:
        if not 0.0 <= float(self.reverberation_db) <= 60.0:
            raise ValueError(f"raycast.floor.reverberation_db {self.reverberation_db}: [0, 60]")
        if self.addition not in ("power", "field"):
            raise ValueError(f"raycast.floor.addition {self.addition!r}: 'power' or 'field'")


@dataclass(frozen=True)
class ObjectsConfig:
    """How scene objects reach this sensor's triangle scene (review of
    2026-11-11, ``object_mesh.py``).

    ``raster`` (default, as before): rough and aggregate objects are the
    bumps the world builder stamped into the height raster, at the WORLD's
    resolution -- in a 0.2 m harbour world a pallet is ~20 raised vertices
    and a small weight one; smooth primitives are watertight meshes that
    only mirror.

    ``solid``: the seabed sheet is the terrain alone (the world folder's
    ``depth.npz``, written before any object was stamped) and EVERY object is
    meshed at load from the manifest as its parts (``worldgen.compounds``:
    one primitive, or a pallet's boards, a bicycle's tubes, ...), each with
    its material. Every part scatters diffusely; a smooth one also mirrors,
    and the two shares add up (``scattering.DiffuseLaw``). A world without
    ``depth.npz`` (a legacy bundle) stays ``raster``, said once. Nothing is
    written into a world folder (NC #10). Harbour profiles: solid.

    ``fouling``: the fraction of every object and wall surface covered by
    sediment and growth; it scatters like the seabed and does not mirror.
    0 = clean. Not calibrated (no measurement at 450 kHz was found): the
    sensitivity axis of the aspect response, set per run with the node's
    ``fouling`` parameter.

    ``layer_model`` (2026-11-14): how the wall of a flooded shell (a material
    backed by water, e.g. ``pp_shell``) passes and returns sound. ``fluid``
    (default, as before): a fluid layer, one |R| at every angle -- it stops
    transmitting beyond the wall's longitudinal critical angle. ``elastic``:
    the solid wall with its shear wave (``layers.py``: the exact plate
    solution, tabulated per wall), for the transmission of crossing rays and
    for the reflection of the shell's own patches at their incidence angle.
    Needs the wall material's shear speed (``worldgen.materials.
    elastic_constants``); a shell material without one keeps the fluid wall,
    said once.

    **Face geometry** (2026-11-14, ``object_mesh.perturb_patches``; solid
    mode only). A real object's faces are not ideal planes and cylinders, and
    the width of a mirror flash depends on it. Three GEOMETRIC inputs, all 0
    by default (the ideal primitive, as before); never a brightness:
    ``face_tilt_rms_deg``: RMS slope of a smooth random waviness of every
    mirror surface, along one surface direction [deg]; ``face_corr_len_m``:
    its correlation length [m] (>= 0.05: five patches); the RMS height that
    follows is ``tilt * length / sqrt(2)`` (0.6 mm for 0.5 deg over 0.1 m).
    ``face_draft_deg``: the draft angle of the near-vertical side walls of
    moulded plastic boxes (they lean outward towards the top) [deg]. The
    waviness is a pure function of (world seed, object id, position): the
    same object is the same from ping to ping and from run to run."""
    mode: str = "raster"
    fouling: float = 0.0
    layer_model: str = "fluid"
    face_tilt_rms_deg: float = 0.0
    face_corr_len_m: float = 0.10
    face_draft_deg: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 <= float(self.face_tilt_rms_deg) <= 5.0:
            raise ValueError(f"raycast.objects.face_tilt_rms_deg {self.face_tilt_rms_deg}: [0, 5]")
        if not 0.05 <= float(self.face_corr_len_m) <= 2.0:
            raise ValueError(f"raycast.objects.face_corr_len_m {self.face_corr_len_m}: [0.05, 2]")
        if not 0.0 <= float(self.face_draft_deg) <= 5.0:
            raise ValueError(f"raycast.objects.face_draft_deg {self.face_draft_deg}: [0, 5]")
        if self.mode not in ("raster", "solid"):
            raise ValueError(f"raycast.objects.mode {self.mode!r}: 'raster' or 'solid'")
        if self.layer_model not in ("fluid", "elastic"):
            raise ValueError(f"raycast.objects.layer_model {self.layer_model!r}: 'fluid' or 'elastic'")
        if not 0.0 <= float(self.fouling) <= 1.0:
            raise ValueError(f"raycast.objects.fouling {self.fouling}: [0, 1]")


@dataclass(frozen=True)
class LabelsConfig:
    """Ground-truth contacts of this sensor (``labels.py``, review of
    2026-11-11): which scene objects a ping insonifies, whether they are
    visible, and the shadow each one casts, read off the renderer's own mesh
    by its own visibility oracle. Off (default): ``RenderedPing.contacts``
    stays empty, as before. Never changes ``Ping.power`` or ``Ping.texture``.
    Harbour profiles: on."""
    enabled: bool = False
    #: surface points per object the visibility test samples at most
    max_points: int = 512

    def __post_init__(self) -> None:
        if not 16 <= int(self.max_points) <= 8192:
            raise ValueError(f"raycast.labels.max_points {self.max_points}: [16, 8192]")


#: The fast variant's engine paths (2026-11-10, docs/engineering/
#: raycast_fast_optimize_plan.md): dotted ``raycast`` key -> the default that
#: reproduces today's renderer exactly. The one list the correction
#: fingerprint (left out at their default), the smoke test and the docs use.
ENGINE_KNOBS: dict[str, Any] = {
    "sampling.cull_unreachable": False,
    "sampling.band_cull": False,
    "paths.reselect_every": 1,
    "paths.reselect_move_m": 0.0,
    "diffraction.method": "sampled",
    "precision": "fp64",
}

#: The keys that change only how fast a ping renders, not the physics a
#: correction artifact was fitted on: ``ENGINE_KNOBS`` plus the path-order
#: cap and the scatterer spacing of the live / fast variants (evidence:
#: live bit-identical on the harbour route 176/176 pings and over objects;
#: fast identical by eye, ``[F1]``-``[F7]``; ``raycast_fast_optimize_plan.md``).
#: A harbour statement -- the pool's multipath comb needs orders >= 2.
#: ``Correction.check_core(accept_engine=True)`` compares everything else.
PERFORMANCE_KEYS: tuple[str, ...] = tuple(ENGINE_KNOBS) + (
    "paths.max_order", "sampling.scatterer_spacing_m")

#: Keys of the physical core that a profile may set to another value than the
#: one a correction artifact was fitted with, because the SEABED MEAN inside
#: the artifact's support was measured not to depend on them (session 01 of
#: the simulator completion round, 2026-11-14; ``Correction.check_core(
#: accept_engine=True)`` gives them the artifact's own value, or their default,
#: before comparing):
#: ``paths.surface_rms_m``: sea surface 1 mm against 2 cm -- 0.00 dB at 7.5
#: and 10 m of altitude, <= 0.14 dB at 5 m for r < 3 x altitude
#: (``chevron/harbour_surface.json``); the near-nadir table lives at
#: r < 2.25 x altitude.
#: ``beam.taper_pedestal``: the aperture weighting -- the seabed mean moves by
#: 0.02 to 0.10 dB between 1.0 and 0.819 at 5-10 m of altitude
#: (``beam/taper_seabed.json``).
#: ``sampling.max_range_m``: a guard; it changes no render below the limit.
ACCEPTED_CORE_KEYS: tuple[str, ...] = ("paths.surface_rms_m", "beam.taper_pedestal",
                                       "sampling.max_range_m")


def engine_knob(rc, key: str):
    """The value of the ``ENGINE_KNOBS`` entry ``key`` in ``rc``."""
    obj = rc
    for part in key.split("."):
        obj = getattr(obj, part)
    return obj


@dataclass(frozen=True)
class RaycastConfig:
    seed: int = 0
    #: The system gain [dB]: ``SL + RS`` and the receiver's constants, the
    #: one fitted level constant of this sensor (raycast_design §8). The
    #: shared encoder/noise stage runs with it as ``calibration_db_offset``
    #: (``shared_stage``); the profile's own ``model`` section keeps the
    #: geometric offset its measured noise levels are paired with. Default:
    #: the provisional anchor of the literature spectra (flat sand at the
    #: bath3 geometry matched to the calibrated geometric renderer, TVG k 12;
    #: raycast_phase2.md Appendix D). Bounds [105, 165] dB (the anchors of
    #: every spectrum variant tried, 126.6-143.7, +-20 dB).
    system_gain_db: float = 126.6
    #: The visibility oracle: "cpu" (the numpy BVH, the reference) or
    #: "warp" (the GPU oracle, gpu_warp.py; gate G1 holds it to the CPU).
    backend: str = "cpu"
    #: FAST-VARIANT ENGINE PATH (2026-11-10; ``ENGINE_KNOBS``): the GPU's
    #: per-triangle visibility test, "fp64" (today) or "fp32" (the float32
    #: watertight test in the BVH's local frame, ``gpu_render.seg_blocked32``,
    #: which the default diffraction already uses). GPU only by definition:
    #: the CPU reference stays float64.
    precision: str = "fp64"
    water: WaterConfig = field(default_factory=WaterConfig)
    beam: BeamConfig = field(default_factory=BeamConfig)
    sampling: SamplingConfig = field(default_factory=SamplingConfig)
    seabed: SeabedLawConfig = field(default_factory=SeabedLawConfig)
    receiver: ReceiverConfig = field(default_factory=ReceiverConfig)
    po: PoConfig = field(default_factory=PoConfig)
    specular: SpecularConfig = field(default_factory=SpecularConfig)
    paths: PathsConfig = field(default_factory=PathsConfig)
    diffraction: DiffractionConfig = field(default_factory=DiffractionConfig)
    correction: CorrectionConfig = field(default_factory=CorrectionConfig)
    floor: FloorConfig = field(default_factory=FloorConfig)
    near_range: NearRangeConfig = field(default_factory=NearRangeConfig)
    labels: LabelsConfig = field(default_factory=LabelsConfig)
    objects: ObjectsConfig = field(default_factory=ObjectsConfig)
    #: The scatterer lattice (speckle realisation) follows the RUN's noise
    #: seed instead of the world seed alone (review of 2026-11-11): the node
    #: sets ``seed`` from its ``seed`` parameter before building the
    #: renderer, so two runs of one world draw two speckle fields (dataset
    #: diversity) and ``seed:=<noise_seed>`` replays one exactly. Off
    #: (default): ``seed`` as written, every run of a world alike.
    seed_per_run: bool = False

    def __post_init__(self) -> None:
        if self.precision not in ("fp64", "fp32"):
            raise ValueError(f"raycast.precision {self.precision!r}: 'fp64' or 'fp32'")
        if not 105.0 <= float(self.system_gain_db) <= 165.0:
            raise ValueError(f"raycast.system_gain_db {self.system_gain_db}: [105, 165] dB")

    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "RaycastConfig":
        d = dict(d or {})
        top = {"seed", "system_gain_db", "backend", "precision", "water", "beam", "sampling",
               "seabed", "receiver", "po", "specular", "paths", "diffraction", "correction",
               "floor", "near_range", "labels", "objects", "seed_per_run"}
        bad = set(d) - top
        if bad:
            raise KeyError(f"Unknown raycast keys: {sorted(bad)}")
        return cls(seed=int(d.get("seed", 0)),
                   system_gain_db=float(d.get("system_gain_db", 126.6)),
                   backend=str(d.get("backend", "cpu")),
                   precision=str(d.get("precision", "fp64")),
                   water=_strict(WaterConfig, d.get("water"), "water"),
                   beam=_strict(BeamConfig, d.get("beam"), "beam"),
                   sampling=_strict(SamplingConfig, d.get("sampling"),
                                    "sampling"),
                   seabed=_strict(SeabedLawConfig, d.get("seabed"), "seabed"),
                   receiver=_strict(ReceiverConfig, d.get("receiver"),
                                    "receiver"),
                   po=_strict(PoConfig, d.get("po"), "po"),
                   specular=_strict(SpecularConfig, d.get("specular"),
                                    "specular"),
                   paths=_strict(PathsConfig, d.get("paths"), "paths"),
                   diffraction=_strict(DiffractionConfig, d.get("diffraction"),
                                       "diffraction"),
                   correction=_strict(CorrectionConfig, d.get("correction"),
                                      "correction"),
                   floor=_strict(FloorConfig, d.get("floor"), "floor"),
                   near_range=_strict(NearRangeConfig, d.get("near_range"), "near_range"),
                   labels=_strict(LabelsConfig, d.get("labels"), "labels"),
                   objects=_strict(ObjectsConfig, d.get("objects"), "objects"),
                   seed_per_run=bool(d.get("seed_per_run", False)))


def shared_stage_model(base, system_gain_db: float, reverberation_db: float = 0.0):
    """The ``SonarModelConfig`` the shared noise / encoder stages use with
    this sensor: ``calibration_db_offset`` = the ray-cast system gain, and
    the linear level constants that encode MEASURED reported levels
    (``noise_floor``, ``watercolumn_noise``; geometric units, paired with
    the geometric offset) rescaled by ``10^((offset_geo - gain)/10)`` so
    their reported dB are unchanged. ``reverberation_db`` 0: the shadow
    floor comes from physics (raycast_design §3) unless ``reverberation_db``
    is given (``raycast.floor``, the noise stage only: the node never hands
    it to the renderer's model). Found 2026-11-03 (NC #4
    step): without the rescaling the water column read 45 dB too loud."""
    import dataclasses
    scale = 10.0 ** ((float(base.calibration_db_offset) - float(system_gain_db))
                     / 10.0)
    return dataclasses.replace(
        base, calibration_db_offset=float(system_gain_db),
        noise_floor=float(base.noise_floor) * scale,
        watercolumn_noise=float(base.watercolumn_noise) * scale,
        reverberation_db=float(reverberation_db))


def noise_stage_model(base, rc: "RaycastConfig", acq, max_ping_rate_hz: float = 0.0):
    """The shared NOISE stage's model for this sensor at ``acq``: the shared
    stage (:func:`shared_stage_model`) plus what only the noise stage reads
    -- the shadow floor (``rc.floor``) and, when ``rc.near_range`` applies
    to ``acq``, its measured water column. The renderer keeps
    :func:`shared_stage`'s model, so neither can move its physics."""
    import dataclasses
    m = shared_stage_model(base, float(rc.system_gain_db),
                           reverberation_db=float(rc.floor.reverberation_db))
    if rc.near_range.applies(acq, max_ping_rate_hz):
        m = dataclasses.replace(m, watercolumn_noise=10.0 ** (
            (float(rc.near_range.watercolumn_db) - float(rc.system_gain_db)) / 10.0))
    return m


def shared_stage(model, rc: "RaycastConfig"):
    """The shared encoder/noise stage of this sensor: the profile's ``model``
    section (geometric offset, measured noise levels) re-expressed under
    ``rc.system_gain_db`` (:func:`shared_stage_model`)."""
    return shared_stage_model(model, float(rc.system_gain_db))


def load_profile(path):
    """A ray-cast sonar profile: ``(SonarConfig, RaycastConfig)``. The
    ``acquisition`` / ``model`` sections load exactly as for the geometric
    renderer (``SonarConfig.from_yaml`` ignores the other keys); ``raycast``
    is strict; ``renderer`` must say ``raycast``."""
    import yaml
    from ..config import SonarConfig
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    if str(doc.get("renderer", "raycast")) != "raycast":
        raise ValueError(f"{path}: renderer {doc.get('renderer')!r} is not raycast")
    return SonarConfig.from_yaml(path), RaycastConfig.from_dict(doc.get("raycast"))


def _sonar_config_cls():
    from ..config import SonarConfig
    return SonarConfig


@dataclass
class RaycastSonarConfig(_sonar_config_cls()):
    """A ``SonarConfig`` carrying a ray-cast section: the calibration engine
    (``calibration/seabed.py``) renders its jobs through this sensor when
    the config has a ``raycast`` mapping (fitted ``raycast.*`` parameters
    live there)."""

    raycast: dict = field(default_factory=dict)


def raycast_sonar_config(cfg, raycast: Mapping[str, Any]) -> RaycastSonarConfig:
    """``cfg`` (a ``SonarConfig``) with a strictly validated ray-cast
    section."""
    RaycastConfig.from_dict(raycast)          # raises on unknown keys
    return RaycastSonarConfig(acquisition=cfg.acquisition, model=cfg.model,
                              raycast=_deepcopy(dict(raycast)))


def _deepcopy(d):
    import copy
    return copy.deepcopy(d)


def get_nested(d: Mapping[str, Any], dotted: str):
    """The value of ``raycast.<a>.<b>...`` in a ray-cast mapping, the code
    default (dataclass / ``seabed_ssa.SEDIMENTS``) when the mapping is
    silent."""
    parts = dotted.split(".")[1:] if dotted.startswith("raycast.") else dotted.split(".")
    node: Any = d
    for k in parts:
        if isinstance(node, Mapping) and k in node:
            node = node[k]
        else:
            node = None
            break
    if node is not None:
        return node
    if parts[:2] == ["seabed", "sediments"]:
        from .seabed_ssa import SEDIMENTS
        return getattr(SEDIMENTS[parts[2]], parts[3])
    obj: Any = RaycastConfig()
    for k in parts:
        obj = getattr(obj, k)
    return obj


def set_nested(d: dict, dotted: str, value) -> dict:
    """A copy of ``d`` with ``raycast.<a>.<b>... = value``."""
    out = _deepcopy(d)
    parts = dotted.split(".")[1:] if dotted.startswith("raycast.") else dotted.split(".")
    node = out
    for k in parts[:-1]:
        node = node.setdefault(k, {})
    node[parts[-1]] = value
    return out


#: The provisional ray-cast section of ``config/raycast_default_sonar.yaml``
#: (2026-11-03): the 450 kHz spectra at 1/100 of J&R Table 13.1's SAX99 /
#: Key West values -- the smallest change that held the NC #4 envelope
#: (``raycast_plan.md`` status 2026-11-03, NC4 v3: 0 regressions; SAX99/30
#: left 2), inside the sand envelope -- with its anchor gain (flat sand at
#: the bath3 geometry matched to the calibrated geometric renderer) and the
#: provisional TVG k 12. To be replaced by the Phase-7 fit.
PROVISIONAL_RAYCAST = {
    "system_gain_db": 143.7,
    "receiver": {"tvg_k": 12.0},
    "seabed": {"sediments": {"sand": {"w2": 4.33e-7}, "mud": {"w2": 4.64e-6}}},
}


def default_profile_doc(base_yaml=None) -> dict:
    """The document of ``config/raycast_default_sonar.yaml``:
    ``acquisition`` and ``model`` copied from the geometric comparison
    profile (so the shared stage never drifts from it), ``renderer:
    raycast`` and :data:`PROVISIONAL_RAYCAST`."""
    import yaml
    from ..config import find_default_sonar_yaml
    with open(base_yaml or find_default_sonar_yaml(), encoding="utf-8") as f:
        base = yaml.safe_load(f)
    RaycastConfig.from_dict(PROVISIONAL_RAYCAST)
    return {"renderer": "raycast", "acquisition": base["acquisition"],
            "model": base["model"], "raycast": _deepcopy(PROVISIONAL_RAYCAST)}


def write_default_profile(path, base_yaml=None) -> None:
    """Generate ``config/raycast_default_sonar.yaml``
    (:func:`default_profile_doc`). Regenerate, never hand-edit."""
    import yaml
    doc = default_profile_doc(base_yaml)
    head = ("# GENERATED by blueboat_sss_sim.sonar.raycast.config."
            "write_default_profile -- regenerate, never hand-edit.\n"
            "# The ray-cast sensor's profile (renderer: raycast). acquisition"
            " and model = config/default_sonar.yaml\n# (the shared encoder"
            " / noise stage; model.calibration_db_offset is the level the"
            " measured noise\n# constants are paired with -- the ray-cast"
            " system gain is raycast.system_gain_db). raycast: the\n#"
            " provisional section (PROVISIONAL_RAYCAST): spectra at 1/100 of"
            " J&R Table 13.1, NC #4-held,\n# replaced by the Phase-7 fit."
            " docs/engineering/raycast_design.md §8, §14.\n")
    with open(path, "w", encoding="utf-8") as f:
        f.write(head)
        yaml.safe_dump(doc, f, sort_keys=False)


#: Live-run variants of the default ray-cast profile (owner, 2026-11-09):
#: each switches off only parts that cost GPU time without changing how the
#: image looks (measured, ``~/ros2_ws/sim_runs/raycast/live_opt/``: the
#: 2026-11-09 harbour session's poses, the whole Shiraishijima lawnmower
#: route, a 122-ping waterfall past the pier, and a pass over a tyre, a box
#: and a pipe on flat sand at 5 m). The full model stays
#: ``raycast_default_sonar.yaml`` (the pool's multipath comb needs orders
#: >= 2; the fit and validation used it). The top-level ``variant:`` block
#: is what MCS lists in its Sonar model menu; ``SonarConfig.from_yaml`` and
#: the node ignore it.
#: The harbour site calibration of this sensor (2026-11-07, "hybrid v2",
#: gates amendments 6b-6d; ``docs/engineering/raycast_assessment.md``): the
#: ray-cast deltas of ``~/calib/raycast/correction/
#: raycast_harbour_hybrid_v2_sonar.yaml`` (sha256 22c9c500...) over the
#: default profile -- gain and TVG refitted on the harbour calibration
#: session (PM-cal), SAX99 sand spectrum, the large-angle vertical floor
#: f_v 0.217, the beam height and a calm-harbour surface. On the validation
#: sessions it puts the far-field seabed at real - sim = -0.3..+1.6 dB at
#: the sim's acquisition (toOptimize, 18 m / 1200 / 44.3 us), where the
#: provisional core reads +11 dB. Harbour only: the core was never fitted
#: on the beach or the pool.
HARBOUR_V2_CORE: dict[str, Any] = {
    "system_gain_db": 126.91092351562708,
    "receiver": {"tvg_k": 3.761531248098334},
    "seabed": {"sediments": {"sand": {"w2": 4.33e-05, "gamma2": 3.04}}},
    "beam": {"vertical_floor_amp": 0.217, "height_deg": 47.11664876755567},
    "paths": {"surface_rms_m": 0.001},
}
#: Its EMPIRICAL near-nadir table (``correction.py``), a byte-identical copy
#: of ``~/calib/raycast/correction/blueboat_raycast_correction_2.yaml``
#: (sha256 9557a8a1...): fitted on PM-cal, the first-bottom-return crest
#: the core leaves 5-10 dB cold; status "failed" on its strict gates (P2:
#: 9-10 m still 3.3-3.7 dB cold; P3; S6) while improving every altitude
#: band (|M1| 7-12 -> <= 3.7 dB). Resolved against the package config dir.
HARBOUR_V2_CORRECTION = "raycast_correction_harbour_v2.yaml"
#: The device's near range at the sim's acquisition (1200 bins, 44.3 us,
#: gain 4), MEASURED on harbour_A-2026-07-22-AM (toOptimize, 8372 pings
#: at 18-27 m; the only 1200-bin / 44.3 us harbour data outside the
#: hold-out, so that session no longer validates this table):
#: ``~/ros2_ws/sim_runs/raycast/harbour_patch/near_range/`` (real profile,
#: generator ``make_table.py``). The ringing solves E[10 log10(R G + W E)]
#: = the real mean-of-dB at every range (G Gamma(25), the coherent channel;
#: E Exp(1), the water column), reproducing it to 0.01 dB; the processor's
#: rule then finds the real ringing end (0.14 m) and noise estimate
#: (28.5 dB) instead of 0.47 m / 41 dB.
HARBOUR_NEAR_RANGE: dict[str, Any] = {
    "ringing_db": [[0.0, 52.15], [0.025, 53.76], [0.05, 55.33], [0.075, 50.91],
                   [0.1, 49.49], [0.125, 48.34], [0.15, 36.7], [0.175, 30.77],
                   [0.2, 29.42], [0.25, 27.5], [0.3, 27.79], [0.4, 28.23],
                   [0.5, 27.81], [0.6, 23.53], [0.75, 19.96], [1.0, 17.37],
                   [1.25, 14.81], [1.5, 11.75], [2.0, 4.46]],
    "watercolumn_db": 17.04,
    "num_results": 1200,
    "pulse_us": [44.3, 50.0],
}
HARBOUR_V2_CORRECTION_SHA256 = "9557a8a123635e0c090b136bc7642d8f91b20073c8ca9ee769b6f96af3e2d509"


def _deep_merge(*parts: Mapping[str, Any]) -> dict:
    out: dict = {}
    for part in parts:
        for k, v in part.items():
            if isinstance(v, Mapping) and isinstance(out.get(k), Mapping):
                out[k] = _deep_merge(out[k], v)
            else:
                out[k] = _deepcopy(dict(v)) if isinstance(v, Mapping) else v
    return out


def _harbour(engine: Mapping[str, Any]) -> dict:
    """``engine`` deltas + the harbour patch (2026-11-10): the v2 core, its
    near-nadir table held down to 4.5 m (fading to 0 at 3.5 m) and accepted
    under the performance keys, the NC #15 shadow floor (18 dB) and the
    measured near range of the 1200-bin / 44.3 us mode. Since the review of
    2026-11-11 also: the floor added in the field domain (the speckle's dark
    tail stays exponential), ground-truth contacts (``labels.py``), every
    object as its own mesh on the terrain (``objects.mode: solid``), and the
    speckle realisation of each run from its own noise seed."""
    return _strip(_deep_merge(engine, HARBOUR_V2_CORE, {
        "correction": {"enabled": True, "artifact": HARBOUR_V2_CORRECTION,
                       "altitude_hold_m": [4.5, 11.3],      # taper: the 1.0 m default
                       "onset_fade_m": 0.8,
                       "accept_engine_knobs": True},
        "floor": {"reverberation_db": 18.0, "addition": "field"},
        "near_range": _deepcopy(HARBOUR_NEAR_RANGE),
        "labels": {"enabled": True},
        "objects": {"mode": "solid"},
        "seed_per_run": True},
        _deepcopy(HARBOUR_PRODUCTION_2026_11_14)),
        # the sea surface: the v2 core's mirror-flat 1 mm is dropped, which leaves the
        # engine's default, 2 cm (PathsConfig.surface_rms_m)
        "paths.surface_rms_m")


def _strip(d: Mapping[str, Any], *dotted: str) -> dict:
    """``d`` without the dotted keys (and without a section they leave
    empty): a variant declares only what differs from the default profile."""
    out = _deepcopy(dict(d))
    for key in dotted:
        *path, leaf = key.split(".")
        node, trail = out, []
        for part in path:
            trail.append((node, part))
            node = node.get(part) if isinstance(node, dict) else None
        if isinstance(node, dict):
            node.pop(leaf, None)
            for parent, part in reversed(trail):
                if isinstance(parent.get(part), dict) and not parent[part]:
                    del parent[part]
    return out


#: What session 01 of the simulator completion round added to the harbour
#: profiles (owner's decisions of 2026-11-14; evidence
#: ``~/ros2_ws/sim_runs/sim_acceptance/01_sonar_physics/handoff_01.md``):
#: * a sea surface 2 cm rough instead of 1 mm: a chop of 0.07 m significant
#:   height is 17 mm RMS; no coherent mirror image of the direct path is left
#:   (the cause of the zig-zag bands at low altitude and of the stripes beyond
#:   25 m). Any value above 5 mm gives the same sonar in the thesis domain;
#: * the wall of a flooded shell as a solid plate (``layers.py``, V5);
#: * object faces that are not ideal: a waviness of 0.5 deg RMS slope over
#:   0.10 m (0.6 mm RMS high) and a 1.5 deg draft on moulded boxes. The draft
#:   is sourced (moulders' design guides: 1-2 deg); the waviness is ASSUMED,
#:   bracketed by the two flash profiles below.
HARBOUR_PRODUCTION_2026_11_14: dict[str, Any] = {
    # (the correlation length 0.10 m is the default; the sea surface is
    # handled in _harbour by dropping the core's value)
    "objects": {"layer_model": "elastic", "face_tilt_rms_deg": 0.5,
                "face_draft_deg": 1.5},
    # the harbour profiles refuse ranges above 30 m (SamplingConfig.max_range_m)
    "sampling": {"max_range_m": 30.0},
}
#: The two ends of the flash-width bracket (frozen criteria, R1) and the
#: other end of the shell bracket: production with only these inputs changed.
#: narrow: ideal faces, no draft, and the strongest aperture taper the
#: vendor's 250 mm window allows (first side lobe -14.5 dB against -13.3 dB);
#: wide: dented, used litter -- 2 deg RMS slope over 0.05 m (1.2 mm RMS);
#: opaque shell: the fluid-wall formula of before, which behaves as an opaque
#: body at oblique incidence (a shell with ribs, a lid, trapped air or silt).
FLASH_NARROW: dict[str, Any] = {"beam": {"taper_pedestal": 0.819}}
#: ... and what the narrow end and the opaque-shell end take back to the
#: engine default (ideal faces; the fluid wall)
FLASH_NARROW_DROPS = ("objects.face_tilt_rms_deg", "objects.face_draft_deg")
FLASH_WIDE: dict[str, Any] = {
    "objects": {"face_tilt_rms_deg": 2.0, "face_corr_len_m": 0.05},
}
SHELL_OPAQUE_DROPS = ("objects.layer_model",)


def _harbour_20261112(engine: Mapping[str, Any]) -> dict:
    """The harbour profiles AS THEY RENDERED ON 2026-11-12 (the review
    ``sim-report/simulator_analysis.md`` was made with them): the harbour
    patch of that day, written out here so that it never follows later
    changes of :func:`_harbour`, plus every legacy switch that reproduces a
    since-fixed behaviour (``beam.legacy_phase_sign``, ``beam.py``;
    ``paths.legacy_reflection_phase``, ``specular.MaterialSurface``). For
    regression against that review only (frozen criteria, B6); never a
    production sensor."""
    return _deep_merge(engine, HARBOUR_V2_CORE, {
        "correction": {"enabled": True, "artifact": HARBOUR_V2_CORRECTION,
                       "altitude_hold_m": [4.5, 11.3],
                       "onset_fade_m": 0.8,
                       "accept_engine_knobs": True},
        "floor": {"reverberation_db": 18.0, "addition": "field"},
        "near_range": _deepcopy(HARBOUR_NEAR_RANGE),
        "labels": {"enabled": True},
        "objects": {"mode": "solid"},
        "seed_per_run": True,
        "beam": {"legacy_phase_sign": True},
        "paths": {"legacy_reflection_phase": True}})


_LIVE_ENGINE = {"paths": {"max_order": 1}}
_FAST_ENGINE = {"paths": {"max_order": 1, "reselect_every": 10, "reselect_move_m": 0.3},
                "sampling": {"cull_unreachable": True, "band_cull": True,
                             "scatterer_spacing_m": 0.010},
                "diffraction": {"method": "edge"},
                "precision": "fp32"}
_BRACKET_NOTE = (
    " One end of a sensitivity bracket of the harbour-fast sensor (2026-11-14): the "
    "production profile with ONLY the named physical inputs changed. A thesis result "
    "must hold at both ends.")
_HARBOUR_NOTE = (
    " HARBOUR SITE CALIBRATION, provisional (2026-11-10): the harbour-fitted core "
    "(seabed level within ~2 dB of the real harbour, was +11 dB), its empirical "
    "first-return table, and a textured shadow floor 18 dB under the seabed "
    "(measured at the quay). Publishes ground-truth contacts (labelled datasets), "
    "renders every object as its own 3-D mesh instead of a bump of the world raster, "
    "and each run draws its own speckle from its noise seed (2026-11-11). Harbour "
    "worlds only; never the sim-to-real comparison profile.")

RAYCAST_VARIANTS: dict[str, dict] = {
    "raycast_live_sonar.yaml": {
        "label": "Ray-cast \u2014 live (first-order multipath)",
        "order": 10,
        "description": (
            "The full ray-cast model with first-order multipath only "
            "(paths.max_order 1). Bit-identical to 'Ray-cast' on the harbour "
            "lawnmower route (176/176 pings) and over objects (tyre, box, pipe); "
            "caps the 16 s pier-corner ping at 7.4 s. Not for the pool, whose "
            "multipath comb needs higher orders."),
        "raycast": _deepcopy(_LIVE_ENGINE),
    },
    # The fast variant (owner, 2026-11-10; docs/engineering/
    # raycast_fast_optimize_plan.md): live + the engine paths of ENGINE_KNOBS.
    # Its look against 'Ray-cast' is the owner's live test, not a claim here.
    "raycast_fast_sonar.yaml": {
        "label": "Ray-cast \u2014 fast",
        "order": 20,
        "description": (
            "Live (first-order multipath) + the fast engine paths: no scatterers "
            "where no sound can reach (above the waterline, buried, the land side "
            "of quays); the echo-path choice reused for up to 10 pings / 0.3 m / "
            "2 deg; shadow-edge diffraction by the closed form of the nearest "
            "straight wall edge; float32 visibility tests; scatterers every "
            "10 mm instead of 5 mm, and only inside the beam band at their own "
            "range. A harbour mode: not for the pool, whose multipath comb needs "
            "higher orders."),
        "raycast": _deepcopy(_FAST_ENGINE),
    },
    # The harbour patch (owner, 2026-11-10; plan "Ray-cast harbour realism
    # patch"; .claude/raycast-live.md): the same engines with the harbour
    # site calibration -- level, first return, shadow floor.
    "raycast_harbour_live_sonar.yaml": {
        "label": "Ray-cast \u2014 harbour, live",
        "order": 30,
        "description": "Live (first-order multipath)." + _HARBOUR_NOTE,
        "raycast": _harbour(_LIVE_ENGINE),
        "header": "harbour site calibration (HARBOUR_V2_CORE + its correction + floor)",
    },
    "raycast_harbour_fast_sonar.yaml": {
        "label": "Ray-cast \u2014 harbour, fast",
        "order": 40,
        "description": "Fast (the fast engine paths)." + _HARBOUR_NOTE,
        "raycast": _harbour(_FAST_ENGINE),
        "header": "harbour site calibration (HARBOUR_V2_CORE + its correction + floor)",
    },
    # The sensitivity brackets of the production sensor (owner, 2026-11-14).
    "raycast_harbour_fast_flash_narrow_sonar.yaml": {
        "label": "Ray-cast \u2014 harbour, fast, flash narrow",
        "order": 41,
        "description": "Ideal flat faces, no draft, the strongest aperture taper the "
                       "transducer window allows: the narrowest mirror flash." + _BRACKET_NOTE,
        "raycast": _deep_merge(_strip(_harbour(_FAST_ENGINE), *FLASH_NARROW_DROPS), FLASH_NARROW),
        "header": "BRACKET END (flash narrow) of the harbour-fast production profile",
    },
    "raycast_harbour_fast_flash_wide_sonar.yaml": {
        "label": "Ray-cast \u2014 harbour, fast, flash wide",
        "order": 42,
        "description": "Dented, used litter: faces 2 deg rough over 0.05 m. The widest, "
                       "weakest mirror flash." + _BRACKET_NOTE,
        "raycast": _deep_merge(_harbour(_FAST_ENGINE), FLASH_WIDE),
        "header": "BRACKET END (flash wide) of the harbour-fast production profile",
    },
    "raycast_harbour_fast_shell_opaque_sonar.yaml": {
        "label": "Ray-cast \u2014 harbour, fast, shells opaque",
        "order": 43,
        "description": "Flooded plastic shells as opaque bodies (the fluid-wall formula): "
                       "the deep-shadow end of the shell bracket." + _BRACKET_NOTE,
        "raycast": _strip(_harbour(_FAST_ENGINE), *SHELL_OPAQUE_DROPS),
        "header": "BRACKET END (shells opaque) of the harbour-fast production profile",
    },
    # Frozen copies of the two harbour profiles as they rendered on 2026-11-12
    # (owner, 2026-11-13): regression only. ``listed: False`` = no ``variant``
    # block, so MCS does not offer them.
    "raycast_harbour_live_20261112_sonar.yaml": {
        "label": "Ray-cast \u2014 harbour, live (as of 2026-11-12)",
        "order": 90,
        "description": "The harbour-live profile as it rendered on 2026-11-12. Regression only.",
        "raycast": _harbour_20261112(_LIVE_ENGINE),
        "header": "FROZEN COPY of the harbour-live profile of 2026-11-12, legacy switches on",
        "listed": False,
    },
    "raycast_harbour_fast_20261112_sonar.yaml": {
        "label": "Ray-cast \u2014 harbour, fast (as of 2026-11-12)",
        "order": 91,
        "description": "The harbour-fast profile as it rendered on 2026-11-12. Regression only.",
        "raycast": _harbour_20261112(_FAST_ENGINE),
        "header": "FROZEN COPY of the harbour-fast profile of 2026-11-12, legacy switches on",
        "listed": False,
    },
}


def flat_deltas(delta: Mapping[str, Any], pre: str = "") -> dict:
    """A variant's ``raycast`` deltas as ``{dotted key: value}``, any depth."""
    out = {}
    for k, v in delta.items():
        if isinstance(v, Mapping):
            out.update(flat_deltas(v, f"{pre}{k}."))
        else:
            out[f"{pre}{k}"] = v
    return out


def _merged(base: Mapping[str, Any], delta: Mapping[str, Any]) -> dict:
    out = _deepcopy(dict(base))
    for k, v in delta.items():
        if isinstance(v, Mapping) and isinstance(out.get(k), Mapping):
            out[k] = _merged(out[k], v)
        else:
            out[k] = _deepcopy(v)
    return out


def variant_profile_doc(name: str, base_yaml=None) -> dict:
    """The document of the live variant ``name`` (a key of
    :data:`RAYCAST_VARIANTS`): the default profile with the variant's
    ``raycast`` deltas, and its ``variant`` block (label, order,
    description) for MCS."""
    v = RAYCAST_VARIANTS[name]
    doc = default_profile_doc(base_yaml)
    rc = _merged(doc["raycast"], v["raycast"])
    RaycastConfig.from_dict(rc)                 # raises on unknown keys
    out = {"renderer": "raycast"}
    if v.get("listed", True):                   # MCS lists a profile by this block
        out["variant"] = {"label": v["label"], "order": int(v["order"]),
                          "description": v["description"]}
    out.update(acquisition=doc["acquisition"], model=doc["model"], raycast=rc)
    return out


def write_variant_profiles(config_dir, base_yaml=None) -> list:
    """Generate every :data:`RAYCAST_VARIANTS` profile into ``config_dir``
    (``config/raycast_<variant>_sonar.yaml``). Regenerate, never
    hand-edit. Returns the written paths."""
    import yaml
    from pathlib import Path
    out = []
    for name, v in RAYCAST_VARIANTS.items():
        path = Path(config_dir) / name
        deltas = ", ".join(f"{k}: {vv}" for k, vv in flat_deltas(v["raycast"]).items())
        head = ("# GENERATED by blueboat_sss_sim.sonar.raycast.config."
                "write_variant_profiles -- regenerate, never hand-edit.\n"
                "# A live-run variant of config/raycast_default_sonar.yaml "
                f"(RAYCAST_VARIANTS): raycast {deltas}.\n"
                + (f"# {v['header']}: a provisional site calibration, never the "
                   "sim-to-real comparison profile.\n" if v.get("header") else "")
                + "# Everything else is the default profile. Evidence: "
                "~/ros2_ws/sim_runs/raycast/live_opt/ and .claude/raycast-live.md.\n")
        with open(path, "w", encoding="utf-8") as f:
            f.write(head)
            yaml.safe_dump(variant_profile_doc(name, base_yaml), f,
                           sort_keys=False, allow_unicode=True)
        out.append(path)
    return out
