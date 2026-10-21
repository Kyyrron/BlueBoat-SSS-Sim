"""Sonar configuration.

Two layers, mirroring the real system exactly:

* **Acquisition parameters** -- the same six run-dependent parameters the
  real ``sss_node.py`` exposes (``range_start_mm``, ``range_length_mm``,
  ``msec_per_ping``, ``gain_index``, ``num_results``, ``pulse_len_percent``);
  identical names, identical semantics (``msec_per_ping = 0`` -> free-run).

* **Simulation-only parameters** -- sensor geometry, acoustic model and
  noise knobs that a real operator does not control (the physics of the
  device and of the water). These live under the ``model:`` key of
  ``default_sonar.yaml`` and never leak into the ROS message content.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml

SPEED_OF_SOUND_MPS = 1500.0


def find_default_sonar_yaml() -> Path:
    """The package's default sonar profile, ``config/default_sonar.yaml``.

    World folders carry no frozen ``sonar.yaml`` (unlike the retired
    mission bundles): the acquisition in force is the package profile plus
    explicit per-run overrides. Resolution order: the source tree (three
    levels up from this file), then the installed ament share directory.
    """
    src = Path(__file__).resolve().parents[2] / "config" / "default_sonar.yaml"
    if src.exists():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = (Path(get_package_share_directory("blueboat_sss_sim"))
                / "config" / "default_sonar.yaml")
        if cand.exists():
            return cand
    except Exception:
        pass
    raise FileNotFoundError(
        "config/default_sonar.yaml not found in the source tree or the "
        "installed share directory")
OMNISCAN_FREQ_HZ = 451_127          # observed on the real device
DEVICE_PROCESSING_MS = 3.0          # free-run overhead beyond two-way travel:
                                    # MEASURED 2.5-3.5 ms on every corpus file
                                    # past the 30 m cap (38 m -> 54 ms, 40 ->
                                    # 56, 52 -> 73, 72 -> 99, 80 -> 110,
                                    # 95.6 -> 131, 125.6 -> 170 ms)
MIN_PING_PERIOD_S = 0.050           # spec + corpus: 20 pings/s up to 30 m
                                    # (18.4 m and 20 m recordings sit at
                                    # exactly 50 ms), range-limited beyond
PULSE_MIN_S = 44.3e-6               # shortest transmit pulse the device
                                    # reports (16-27 m at 1200 bins)

AUTO_GAIN_INDEX = -1                # command sentinel: "let the device choose"
CALIBRATION_GAIN_INDEX = 4          # the index the power model is anchored at
MAX_GAIN_INDEX = 7                  # highest index the simulated ladder defines
                                    # (4..7 measured from the field corpus; the
                                    # device's auto-gain uses all four)

#: Model keys retired when the encoder moved to the device's per-ping
#: normalisation. Named explicitly so a bundle frozen under the old encoding
#: fails with an actionable message rather than a bare "unknown key".
_RETIRED_MODEL_KEYS = {
    "base_scale": "the device normalises pwr_results per ping, so there is no "
                  "linear power -> counts scale; absolute level is now carried "
                  "by calibration_db_offset alone",
    "gain_index_step_db": "the gain ladder is now the measured analog_gain "
                          "table in sonar/encoder.py, not a flat dB step",
}


@dataclass
class AcquisitionParams:
    """Run-dependent parameters, identical to the real node's ROS params --
    names, semantics and (since 2026-09-28) defaults, which are also
    ``config/default_sonar.yaml``'s: the device's best acquisition."""

    range_start_mm: int = 0
    range_length_mm: int = 20_000
    msec_per_ping: int = 0                 # 0 = free-run (device behaviour)
    gain_index: int = AUTO_GAIN_INDEX      # resolves to the calibrated 4
    num_results: int = 1200                # the device maximum
    pulse_len_percent: float = 0.00125     # 44.3 us floor up to ~27 m

    def __post_init__(self) -> None:
        g = int(self.gain_index)
        if g != AUTO_GAIN_INDEX and not (0 <= g <= MAX_GAIN_INDEX):
            raise ValueError(
                f"gain_index {g} is outside the simulated gain ladder: use "
                f"0..{MAX_GAIN_INDEX}, or {AUTO_GAIN_INDEX} for device auto-gain. "
                "The real device accepts a wider ladder, but this module only "
                "models the indices it has an analog-gain figure for -- a "
                "silent fallback would misreport analog_gain and max_pwr_db.")

    @property
    def effective_gain_index(self) -> int:
        """The index actually applied, and the one the device *reports*.

        ``gain_index`` is a **command** parameter and ``-1`` is a command-only
        sentinel meaning "device auto-gain"; the real Omniscan resolves it
        internally and reports a concrete index back in the profile, whose
        ``gain_index`` field is ``uint16`` and so can never carry ``-1``.
        The simulator has no AGC, so auto-gain resolves to the index the power
        model is calibrated at (``CALIBRATION_GAIN_INDEX``) -- which keeps
        the reported ``max_pwr_db`` on the measured anchor (NC #6) instead of
        the arbitrary level a literal ``-1`` would report.

        In the field corpus the device's own auto-gain walks 4..7 within every
        recording; a run that fixes the index is the exception, not the rule.
        """
        g = int(self.gain_index)
        return CALIBRATION_GAIN_INDEX if g == AUTO_GAIN_INDEX else g

    @property
    def range_max_m(self) -> float:
        return (self.range_start_mm + self.range_length_mm) / 1000.0

    @property
    def bin_size_m(self) -> float:
        return self.range_length_mm / 1000.0 / max(self.num_results, 1)

    def ping_period_s(self, max_ping_rate_hz: float = 0.0) -> float:
        """Effective ping period -- the device law, measured on the corpus.

        ``period = max(msec_per_ping, 2R/c + DEVICE_PROCESSING_MS,
        1/max_ping_rate_hz)``. The Omniscan 450 pings at 20 pings/s up to
        30 m and is range-limited beyond (spec sheet); all seven corpus
        recordings follow exactly that: 50 ms at 18.4 m and 20 m, then
        two-way travel plus ~3 ms (54 ms at 38 m ... 131 ms at 95.6 m,
        170 ms at 125.6 m). ``max_ping_rate_hz`` is the 20 Hz cap; 0
        disables it (a model knob, kept for experiments -- the device itself
        never exceeds 20 Hz)."""
        two_way = 2.0 * self.range_max_m / SPEED_OF_SOUND_MPS
        free_run = two_way + DEVICE_PROCESSING_MS / 1000.0
        period = max(self.msec_per_ping / 1000.0, free_run)
        if max_ping_rate_hz > 0.0:
            period = max(period, 1.0 / max_ping_rate_hz)
        return period

    def pulse_duration_s(self, max_ping_rate_hz: float = 0.0) -> float:
        """Transmit-pulse length, as the device reports it.

        MEASURED law (2026-09-28, every corpus recording, 13 distinct
        range / bin / pulse settings, within 1 us): ``pulse =
        max(44.3 us, pulse_len_percent * 2R/c)`` -- the command parameter is
        the pulse's share of the two-way travel time. The ROS driver's former
        0.002 gives 106.4 us at 40 m and 212.8 at 80 m; SonarView sends
        1.5 / num_results (0.0025 at 600: 66.5 us at 20 m; 0.00125 at 1200:
        66.5 us at 39.9 m, 159.6 at 95.6 m), and at 1200 bins the 44.3 us
        floor holds below ~27 m. ``max_ping_rate_hz`` is accepted for
        signature compatibility and unused."""
        two_way = 2.0 * self.range_max_m / SPEED_OF_SOUND_MPS
        return max(PULSE_MIN_S, float(self.pulse_len_percent) * two_way)


@dataclass
class SonarModelConfig:
    """Simulation-only sensor/acoustics/noise parameters."""

    # Mounting geometry: a body-frame lever arm [mount_x, +-mount_y,
    # -sensor_depth] from the vehicle pose, whose z = 0 is the boat floating
    # level at rest on the still waterline (the sonar node moves Gazebo's
    # base_link pose onto that datum first, `base_link_rest_z_m`).
    sensor_depth_m: float = 0.15           # transducer below still water
                                           # (JUDGED estimate, unmeasured)
    mount_x_m: float = 0.0                 # fore/aft offset from base_link
    mount_y_abs_m: float = 0.285           # lateral offset magnitude per side
    beam_tilt_deg: float = 20.0            # pattern centre below horizontal
    vertical_aperture_deg: float = 50.0    # Omniscan 450 spec: 50 deg beam height
    horizontal_aperture_deg: float = 0.5   # Omniscan 450 spec: 0.5 deg azimuth

    # Timing.
    max_ping_rate_hz: float = 20.0         # Omniscan 450 cap: 20 pings/s up
                                           # to 30 m, range-limited beyond.
                                           # MEASURED on all seven corpus
                                           # files (see AcquisitionParams.
                                           # ping_period_s). 0 disables the
                                           # cap -- an experiment knob only.

    # Acoustic model.
    lambert_exponent: float = 1.0          # single-power angular law,
                                           # measured over sand on the two
                                           # altitudes together (2026-09-18;
                                           # docs/sonar_model.md §6)
    absorption_db_per_m: float = 0.10      # ~450 kHz seawater. Not separated
                                           # from the spreading term by the
                                           # corpus; held at the physical value
    spreading_exponent: float = 4.0        # two-way INTENSITY spreading, r^-4
    specular_spreading_exponent: float = 2.0   # the coherent (mirror-like)
                                           # component spreads ONCE over the
                                           # folded path -- it is the field of
                                           # an image source, not of N
                                           # independent scatterers. MEASURED:
                                           # with it, the device's reported
                                           # peak level follows altitude as
                                           # 66.6 - 11.3 log10(alt) dB (19
                                           # corpus files at gain 4, rms
                                           # 1.6 dB) and the simulator's as
                                           # 72.1 - 11.5; sharing the diffuse
                                           # r^-4 law made the simulator fall
                                           # 20-27 dB per decade instead.
    tvg_compensation: float = 0.30         # 0..1 fraction of range loss the
                                           # device's own time-varying gain
                                           # removes. RE-MEASURED 2026-09-18 on
                                           # TWO altitudes at once (bath3 at
                                           # 1.40 m, 12371 pings; reflection-
                                           # EvidenceFullharbour at 7.33 m,
                                           # 1445 pings): the seabed's dB-vs-
                                           # slant curve is reproduced to
                                           # 0.9 dB rms at both, and no single-
                                           # altitude fit can separate this
                                           # from the spreading exponent --
                                           # which is why the earlier value was
                                           # 0 ("the stream is pre-TVG"). It is
                                           # not: the device compensates 30 %
                                           # of the modelled loss in dB.
    array_length_m: float = 0.0            # along-track floor on the footprint:
                                           # an unfocused array of length D
                                           # cannot resolve finer than D, and
                                           # a 0.5 deg beam at 450 kHz implies
                                           # D ~ 0.38 m, which would dominate
                                           # r*theta (26 mm at 3 m, 105 mm at
                                           # 12 m) across the whole swath.
                                           # MEASURED 0: the corpus's own
                                           # along-track texture correlation
                                           # (lag-1 0.36-0.66 at 1 m/s, smoke
                                           # [2g]) is what a ~0.1 m footprint
                                           # at 12 m gives; 0.35 m would put
                                           # it at 0.76. The device therefore
                                           # focuses its near field, and the
                                           # r*theta law holds to at least
                                           # 10 m. What happens at 1-3 m is
                                           # UNDETERMINED -- the point-target
                                           # capture at 3 and 12 m (2026-09-18
                                           # review, P6-8) is what would fix
                                           # it, and this knob is where the
                                           # answer goes.
    beam_sidelobe_floor: float = 0.004     # pattern never below this (~-24 dB),
                                           # as real sidelobes. At tilt 20 /
                                           # FWHM 50 the nadir tail alone is
                                           # 0.00436, so it binds only under
                                           # >0.55 deg of roll-up, a smaller
                                           # tilt / aperture, or for upward
                                           # (surface-image) paths
    specular_strength: float = 12.0        # near-normal-incidence specular lobe
                                           # amplitude (relative to Lambert).
                                           # Sized so the FBR clears a +8 dB
                                           # noise-floor threshold even over
                                           # dark bottoms (mud/seagrass), as
                                           # on the real device
    specular_width_deg: float = 8.0        # angular width of the specular lobe
    specular_looks: int = 25               # fluctuation of the *coherent*
                                           # specular return: Gamma(L,1/L),
                                           # CV=1/sqrt(L) (~0.2). The real
                                           # nadir echo is Rician/high-K, far
                                           # steadier than diffuse speckle --
                                           # this is what lets downstream
                                           # bottom tracking lock ping-to-ping
    pulse_smearing: bool = True            # convolve each ping with the
                                           # range envelope the device's own
                                           # settings imply (see
                                           # filter_duration_percent): widens
                                           # + stabilises the FBR onset ramp,
                                           # as the real matched-filter
                                           # output does
    filter_duration_percent: float = 0.0015  # the receive filter's share of
                                           # the two-way travel time -- the
                                           # value the real driver pins
                                           # (Cerulean's documented typical).
                                           # Envelope width in bins =
                                           # pulse/Ts + this * num_results + 1
                                           # (transmit pulse + receive filter
                                           # + one sample of decimation):
                                           # 3 at 600 / 0.002, 5 at 1200 /
                                           # 0.00125 and 20 m. MEASURED: the
                                           # along-range lag-1 correlation of
                                           # the texture is 0.30 / 0.50
                                           # against the corpus's 0.29 / 0.46
    range_filter_bins: int = 0             # DEPRECATED: >0 restores the
                                           # pre-2026-09-28 kernel, pulse
                                           # width floored at this many bins
                                           # (a fixed 3 at every bin count),
                                           # so a frozen legacy profile
                                           # renders as it was frozen. 0 =
                                           # the device-physics width above
    # Transmit-pulse ringing: the bright decaying core right under the
    # transducer that every real ping carries (MEASURED on
    # diffDepthCompensation.svlog: the profile leaves the transducer at
    # ~55 dB — brighter than any seabed return — decaying through 43 dB
    # at 0.27 m to 33 dB by 1 m; the GCS's 0.75 m nadir blank exists for
    # exactly this artefact). Modelled as an exponential decay in the
    # reported-dB domain at the calibration gain, added to the *coherent*
    # channel (low CV, like the real ringing) before noise. Level 0
    # disables it. It lives entirely inside the water column at survey
    # altitudes, so the FBR and the level anchor are untouched — and the
    # processor's find_noise_window_start ringing-settle logic finally
    # has the artefact it was written for.
    ringing_level_db: float = 55.0         # reported dB at r = 0 (gain 4)
    ringing_decay_db_per_m: float = 22.0   # (55-33) dB over the first metre
    alongtrack_beam_lines: int = 5         # ground lines integrated across the
                                           # 0.5 deg azimuth footprint (odd;
                                           # 1 = legacy infinitesimal beam)
    calibration_db_offset: float = 81.1    # ABSOLUTE dB reference at gain index
                                           # 4: reported dB = 10log10(power)
                                           # + this + the analog-gain step.
                                           # The only level constant there is --
                                           # pwr_results itself is normalised
                                           # per ping and carries no level.
                                           # RE-FITTED 2026-09-18 with
                                           # tvg_compensation on the same two
                                           # altitudes (was 97.2, fitted at
                                           # 6-9 m with no TVG term, which
                                           # left the simulator ~18 dB hot at
                                           # the 1.2 m beach altitude). The
                                           # noise floor and water column below
                                           # moved with it by the same 16.1 dB,
                                           # so both still sit where the corpus
                                           # measured them in reported dB.
    max_span_db: float = 90.0              # device clamp on max_pwr_db -
                                           # min_pwr_db. MEASURED: exactly 90.0
                                           # on 8.39% of the field corpus, never
                                           # above it.

    # Shallow-water multipath (optional, enclosed-basin regime realism).
    multipath_enabled: bool = False        # second-bottom-echo ghost on/off
    multipath_gain: float = 0.12           # ghost amplitude relative to direct

    # Wall / surface multipath: mirror-source ghosts off the basin boundaries
    # declared by the *world* config's `walls:` (the world says what walls
    # exist and how reflective they are; these knobs say whether the path is
    # modelled at all). ON by default since 2026-09-01 (user decision): it is
    # a strict no-op in any world without walls, so no wall-less result
    # moves, and a walled world gets its physical reflections without a
    # special profile.
    wall_multipath_enabled: bool = True
    wall_multipath_gain: float = 1.0       # global scale on every ghost, on
                                           # top of each wall's own
                                           # reflectivity (1.0 = as modelled)
    surface_mirror_enabled: bool = False   # also mirror across z = 0. The
                                           # air/water interface is very
                                           # nearly a perfect reflector, but
                                           # the path leaves the transducer
                                           # *upward*, so the beam pattern
                                           # sees it at -depression: -8 dB at
                                           # 20 deg up, -17 dB at 40 deg and
                                           # the sidelobe floor beyond 50.6
    surface_reflectivity: float = 0.9      # pressure-release interface
    # Direct wall echo (closes the former assumption A9): a vertical quay
    # face returns a bright coherent line at its own range and occludes the
    # seabed behind it, as every real harbour log shows.
    wall_echo_enabled: bool = True         # strict no-op in a wall-less world
    wall_echo_strength: float = 30.0       # face-return amplitude at
                                           # broadside, relative to Lambert
                                           # (same order as specular_strength:
                                           # a concrete quay reads at FBR
                                           # level). Not corpus-determined:
                                           # the corpus has quays but no
                                           # geometry ground truth
    wall_echo_obliquity_floor: float = 0.1 # residual return when the beam
                                           # runs along the wall (cos^2 law
                                           # above it)
    wall_face_diffuse: float = 0.5         # Lambert weight of the wet face
                                           # (rough concrete returns over its
                                           # whole height, not only where the
                                           # specular lobe points)
    wall_pontoon_specular: float = 0.15    # specular-lobe fraction on a
                                           # pontoon's rounded, fendered floats
    wall_corner_gain: float = 1.0          # scale of the single-bounce band
                                           # (seabed -> face -> transducer)
                                           # that lands just beyond the face
    wall_diffuse_fraction: float = 0.04    # share of the energy a rough face
                                           # scatters incoherently that still
                                           # reaches the mirror geometry (the
                                           # coherent part follows the
                                           # Rayleigh roughness law)
    ghost_beam_lines: int = 1              # azimuth lines per ghost render
                                           # (1 = infinitesimal beam). Ghosts
                                           # are dim and already blurred by
                                           # the bounce, so the direct path's
                                           # 5 lines are not worth paying
                                           # N times over -- this is the cost
                                           # lever if a basin has many walls

    # Noise model.
    speckle: bool = True
    speckle_looks: int = 1                 # 1 = fully-developed Exp(1) speckle
                                           # (Rayleigh amplitude); >1 = smoother
                                           # multi-look Gamma(L, 1/L) speckle
    coherent_scatterers: bool = True       # single-look speckle from a
                                           # ground-fixed scatterer field
                                           # (random phasor per ground cell,
                                           # summed coherently per bin):
                                           # Exp(1) contrast PLUS the
                                           # along-track (footprint overlap)
                                           # and range (pulse) correlation
                                           # the corpus shows. False = the
                                           # legacy per-bin Exp(1) draw
    scatterer_cell_m: float = 0.01         # ground cell of the scatterer
                                           # field; seed-class parameter
                                           # (changing it changes every image)
    scatterer_seed: int = 0                # extra salt on the field, on top
                                           # of the world seed
    # --- shadow, volume scattering and the waterline (2026-09-19) --------
    reverberation_reference: str = "ambient"
                                           # what the shadow floor is a
                                           # fraction OF. "unoccluded": the
                                           # culled bin's own would-be
                                           # response -- which collapses to
                                           # zero on a back-facing flank or
                                           # over land (cos_inc clips at 0),
                                           # and that collapse is the hard
                                           # black band the 2026-09-19
                                           # comparison found between a
                                           # simulated object and its shadow,
                                           # where the real image runs
                                           # continuously. "ambient": the
                                           # lit seabed level at that range
                                           # on a flat bottom at this ping's
                                           # own altitude -- smooth, always
                                           # defined, and never collapsing.
                                           # The floor is the max of the two,
                                           # so the change can only lighten a
                                           # shadow, never deepen one.
    volume_scattering_coeff: float = 0.0   # sediment volume backscatter
                                           # added to the surface term
                                           # (Bell & Linnett 1997 eq. 12's
                                           # sigma_bv, after Jackson's
                                           # bistatic model). SHIPPED AT 0
                                           # ON PURPOSE: the Lambert
                                           # coefficient this model is
                                           # calibrated on is an *empirical
                                           # total* over the corpus's
                                           # grazing band (J&R par. 12.1),
                                           # so it already absorbs whatever
                                           # volume term that band contains.
                                           # Separating the two needs a
                                           # capture over a known bottom at
                                           # several grazing angles; until
                                           # one exists, a fitted value here
                                           # would trade against
                                           # calibration_db_offset and
                                           # lambert_exponent and collapse
                                           # the [2e] identifiability ridge.
    penumbra_deg: float = 0.0              # angular softening of the horizon
                                           # test (0 = the hard boolean). The
                                           # vertical beam has finite width
                                           # and an occluder's edge
                                           # diffracts, so a shadow boundary
                                           # is not a step. Partial-bin
                                           # coverage already arrives with
                                           # image_formation: energy; this is
                                           # the angular part.
    shore_taper_m: float = 0.0             # depth over which a cell's return
                                           # fades out as the water above it
                                           # thins (0 = the hard cull). NC #13
                                           # is untouched: a cell AT OR ABOVE
                                           # the transducer still returns
                                           # exactly nothing. What this
                                           # softens is the last few
                                           # centimetres of water before it,
                                           # where the grazing angle and the
                                           # insonified thickness both go to
                                           # zero -- the difference between a
                                           # waterline and the per-ping
                                           # sawtooth the hard step draws.

    # --- coherent / incoherent split (2026-09-19) ------------------------
    coherent_split: bool = True           # apply the Rayleigh split
                                           # C = exp(-(2 k sigma cos th)^2)
                                           # (Eckart; the law
                                           # sonar/multipath.bounce_weight
                                           # already uses for a wall). What
                                           # is coherent leaves in the mirror
                                           # direction and does not come
                                           # back, so only (1 - C) scatters
                                           # diffusely. Exactly a no-op for
                                           # every seabed material at
                                           # 450 kHz -- 2 k sigma is 15 for
                                           # the smoothest of them -- and it
                                           # is manufactured surfaces the
                                           # split speaks about. False keeps
                                           # the pre-2026-09-19 behaviour,
                                           # under which a smooth plastic box
                                           # rendered +32 dB over the sand it
                                           # sat on against a measured +2.5.
    sonar_frequency_hz: float = 450_000.0  # the model's one wavenumber (split,
                                           # facets, wall bounce weight); the
                                           # reported ping_hz is separate
    smooth_lobe_width_deg: float = 0.0     # near-normal lobe width on a
                                           # surface the roughness raster
                                           # calls smooth (0 = use the global
                                           # specular_width_deg, which is
                                           # the sand value the corpus fits:
                                           # 8 deg, an RMS slope of 0.14).
                                           # A manufactured shell is not a
                                           # mirror either -- a flooded
                                           # storage box bows -- so what
                                           # comes back is its distribution
                                           # of facet slopes. MEASURED on
                                           # the field box: +5.7 dB at 7 deg
                                           # off broadside against +2.5 dB
                                           # at 19 deg fixes the width at
                                           # ~15 deg (2026-09-19).

    # --- objects: the facet layer (2026-09-28, NC #17) --------------------
    object_facets: bool = True             # smooth solid objects scatter from
                                           # their exact geometry (Kirchhoff +
                                           # the incoherent residual,
                                           # sonar/facets.py); in the raster
                                           # they only occlude. False keeps
                                           # the occlusion and drops their
                                           # scattering -- a diagnostic that
                                           # shows an object's shadow alone.
    facet_patch_m: float = 0.01            # patch size of the tessellation:
                                           # linear phase across a patch and a
                                           # beam that does not vary over it
                                           # (valid from ~2 m slant).
    sand_lambert_mu_db: float = -12.0      # sand's Lambert parameter at
                                           # 450 kHz: the ONE constant tying an
                                           # object's cross-section [m^2] to
                                           # the calibrated seabed. Literature,
                                           # never fitted to an object: Jackson
                                           # & Richardson 2007 Fig. 12.3 fits
                                           # -20.2 dB at 20-40 kHz, and par.
                                           # 12.1.2 gives sand +6-8 dB/decade:
                                           # -20.2 + 7 log10(450/30) = -12.0
                                           # (-13.1 to -10.8 on the slope; the
                                           # site spread is +-5-8 dB). An
                                           # aligned field pass over a surveyed
                                           # target would pin it.

    # --- image formation (2026-09-19) -----------------------------------
    image_formation: str = "energy"          # "mean" | "energy".
                                           # "mean": a bin is the azimuth-
                                           # weighted MEAN of the ground
                                           # samples that land in it -- the
                                           # pre-2026-09-19 behaviour, where a
                                           # fold of a hundred samples into
                                           # one bin (a rising shore, a box's
                                           # near wall) brightens it not at
                                           # all. "energy": a bin is the
                                           # INTEGRAL of its samples' response
                                           # over the resolution cell,
                                           # referenced to one bin of flat
                                           # ground, so layover accumulates as
                                           # the device's does. Flat seabed in
                                           # the far field is unchanged; near
                                           # nadir the residual r/y term adds
                                           # +1.55 dB at 1.4x altitude and
                                           # +0.14 dB at 4x.
    facet_area_jacobian: bool = True      # include sqrt(1+slope^2), the
                                           # surface-area element of the
                                           # heightfield. Without it an
                                           # object's wall contributes one
                                           # raster cell of ground length
                                           # whatever the grid is, so its
                                           # brightness is a function of
                                           # world.resolution; with it the
                                           # contribution converges to the
                                           # wall's true height. Only
                                           # meaningful under "energy".
    layover_gain_max_db: float = 20.0       # cap on how much one bin may
                                           # accumulate over a flat-ground
                                           # bin (0 = uncapped). A vertical
                                           # cliff folds an unbounded ground
                                           # length into one bin; the device's
                                           # own dynamic range does not.
    reverberation_db: float = 18.0         # a shadow's floor, dB under what
                                           # the seabed at the same range
                                           # returns where it is lit. MEASURED
                                           # on the corpus quay: 17-25 dB
                                           # behind the face (wall_contrast,
                                           # 2026-09-18). 0 = the pre-2026-09-18
                                           # behaviour, where a culled bin held
                                           # only the white receiver floor and
                                           # every shadow read 30 dB too deep.
    noise_floor: float = 3.39e-09          # receiver floor, additive linear
                                           # power. MEASURED separately (2026-
                                           # 09-02): the far decile of the
                                           # gain-4 pings of the 72-126 m
                                           # recordings, where nothing but
                                           # noise is left, reads -6.1 dB
                                           # (mean of dB; -3.6 dB as a linear
                                           # mean, which this is) -> 10**((
                                           # -3.6 - calibration_db_offset)/10),
                                           # re-evaluated at the 2026-09-18
                                           # offset (81.1; it was 8.3e-11 at
                                           # 97.2 -- the same -3.6 dB). The former
                                           # 6.79e-7 (35.5 dB) was fitted on
                                           # files that never reached the
                                           # floor and hid the range law
                                           # beyond ~20 m.
    # ---- fault injection (all OFF; they exercise downstream paths) -------
    # CM-2 promises that the downstream stack runs unmodified against
    # simulated data, but a simulator that never walks its gain, never drops
    # a ping, never restarts its counter and never reports channel 255 cannot
    # demonstrate it: those four paths in `sss_processor_node` and in the GCS
    # are exactly the ones a clean simulation leaves untested (2026-09-18
    # review G10). Each knob reproduces something the corpus actually shows.
    fault_gain_walk_pings: int = 0         # >0: step the reported gain index
                                           # 4->5->6->7->4 every N pings, as
                                           # the device's own auto-gain does
                                           # inside every field recording
                                           # (all four indices appear in one
                                           # file). Exercises the processor's
                                           # gain compensation, which a
                                           # constant-gain simulation leaves
                                           # at g = 0 forever.
    fault_counter_reset_pings: int = 0     # >0: restart the ping counter every
                                           # N pings -- what a device power
                                           # cycle (or a relaunched simulator)
                                           # does under a running processor.
                                           # Exercises the counter re-base.
    fault_counter_offset: int = 0          # constant offset added to the
                                           # STARBOARD counter: the two units
                                           # number independently and the
                                           # corpus measures 0, -1 and +60.
                                           # Exercises per-device alignment.
    fault_channel_255: bool = False        # report channel_number 255, so side
                                           # identity falls back to the sign of
                                           # the mounting angle (CM-5). The
                                           # device does this on some packets.
    # `dropped_ping_prob` below is the fifth: each side draws independently,
    # so a non-zero value produces the one-sided rows CM-6 is about.
    gain_drift_amp: float = 0.0            # OFF: deferred to the downstream
    gain_drift_period_s: float = 45.0      # augmentation stage (set >0 to
    dropped_ping_prob: float = 0.0         # re-enable in the base model)
    watercolumn_noise: float = 4.68e-06    # volume reverberation inside the
                                           # water column (r < altitude).
                                           # MEASURED: the 1-3 m band of the
                                           # gain-4 corpus pings reads 25.3 dB
                                           # (mean of dB; 27.8 dB linear mean)
                                           # -> 10**((27.8 - 81.1)/10), the
                                           # 2026-09-18 offset (1.15e-07 at
                                           # the old 97.2 -- the same 27.8 dB).
                                           # It sits ~28 dB (corpus, 6-9 m
                                           # altitude) to ~43 dB (the 3.5 m
                                           # smoke fixture) under the FBR
                                           # peak, the margin growing as the
                                           # altitude falls, so bottom
                                           # tracking locks with margin.

    # Ground-line sampling: renderer step is min(sample_step_m, ~half the
    # slant bin) so every range bin receives samples at any num_results
    # (600, 1200, ...); this value is only the coarse upper bound.
    sample_step_m: float = 0.05

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "SonarModelConfig":
        known = {f.name for f in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        unknown = set(d) - known
        retired = unknown & set(_RETIRED_MODEL_KEYS)
        if retired:
            why = "; ".join(f"{k}: {_RETIRED_MODEL_KEYS[k]}" for k in sorted(retired))
            raise KeyError(
                f"Retired sonar model keys: {sorted(retired)} -- {why}. A "
                "bundle frozen before the encoder moved to the device's "
                "per-ping normalisation cannot be replayed under the current "
                "model; regenerate it (NC #10).")
        if unknown:
            raise KeyError(f"Unknown sonar model keys: {sorted(unknown)}")
        return cls(**{k: v for k, v in d.items()})


@dataclass
class SonarConfig:
    acquisition: AcquisitionParams = field(default_factory=AcquisitionParams)
    model: SonarModelConfig = field(default_factory=SonarModelConfig)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "SonarConfig":
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        acq = doc.get("acquisition", {})
        model = doc.get("model", {})
        known_acq = {f.name for f in AcquisitionParams.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        bad = set(acq) - known_acq
        if bad:
            raise KeyError(f"Unknown acquisition keys: {sorted(bad)}")
        return cls(acquisition=AcquisitionParams(**acq),
                   model=SonarModelConfig.from_dict(model))
