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
                                    # reports (18-27 m at 1200 bins)
PULSE_SAMPLE_PERIODS = {600: 1.2, 1200: 1.5}
                                    # MEASURED pulse length in sample periods
                                    # (2R/c / num_results): 106.4 us @40 m/600,
                                    # 212.8 @80 m/600, 64.3 @38 m/1200,
                                    # 159.6 @95.6 m/1200, 208.4 @125.6 m/1200

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
    """Run-dependent parameters, identical to the real node's ROS params."""

    range_start_mm: int = 0
    range_length_mm: int = 15_000
    msec_per_ping: int = 0                 # 0 = free-run (device behaviour)
    gain_index: int = CALIBRATION_GAIN_INDEX
    num_results: int = 600
    pulse_len_percent: float = 0.002

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

        MEASURED law: the pulse spans a fixed number of *sample periods*
        (``2R/c / num_results``): 1.2 at 600 bins, 1.5 at 1200 bins, with a
        44.3 us floor -- reproducing every ``pulse_duration_sec`` in the
        corpus to within 1 us. ``pulse_len_percent`` keeps its real-node name
        and default and scales the pulse relative to the device default
        (0.002 -> x1.0). ``max_ping_rate_hz`` is accepted for signature
        compatibility and unused: the pulse is set by range and bin count,
        not by the ping period."""
        two_way = 2.0 * self.range_max_m / SPEED_OF_SOUND_MPS
        sample_period = two_way / max(int(self.num_results), 1)
        m = PULSE_SAMPLE_PERIODS.get(int(self.num_results), 1.2)
        scale = float(self.pulse_len_percent) / 0.002
        return max(PULSE_MIN_S, scale * m * sample_period)


@dataclass
class SonarModelConfig:
    """Simulation-only sensor/acoustics/noise parameters."""

    # Mounting geometry (vehicle frame; z below waterline).
    sensor_depth_m: float = 0.15           # transducer below surface
    mount_x_m: float = 0.0                 # fore/aft offset from base_link
    mount_y_abs_m: float = 0.20            # lateral offset magnitude per side
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
    lambert_exponent: float = 1.0          # default when material info absent.
                                           # Lambert's law proper; the corpus
                                           # does not separate it from site
                                           # geometry (see docs/sonar_model.md)
    absorption_db_per_m: float = 0.10      # ~450 kHz seawater. Not separated
                                           # from the spreading term by the
                                           # corpus; held at the physical value
    spreading_exponent: float = 4.0        # two-way INTENSITY spreading, r^-4
    tvg_compensation: float = 0.0          # 0..1 fraction of range loss the
                                           # "device" gain removes. MEASURED 0:
                                           # the profile stream the device
                                           # reports is pre-TVG, so the full
                                           # range loss is present in the data.
                                           # Only the product
                                           # spreading_exponent*(1-this) is
                                           # determined; the split is fixed by
                                           # taking spreading at its physical
                                           # two-way value.
    beam_sidelobe_floor: float = 0.004     # pattern never below this (~-24 dB);
                                           # lets the nadir specular return
                                           # through, as real sidelobes do
    specular_strength: float = 30.0        # near-normal-incidence specular lobe
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
                                           # transmit-pulse range resolution
                                           # c*tau/2 (1.2-1.5 bins on the
                                           # device law) floored at
                                           # range_filter_bins: widens +
                                           # stabilises the FBR onset ramp,
                                           # as the real matched-filter
                                           # output does
    range_filter_bins: int = 3             # receive-chain range envelope
                                           # (matched filter + decimation),
                                           # the floor of the smear kernel.
                                           # Sized from the corpus lag-1
                                           # range correlation 0.30-0.68
                                           # (a 3-bin boxcar on the complex
                                           # field gives 0.44)
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
    calibration_db_offset: float = 97.2    # ABSOLUTE dB reference at gain index
                                           # 4: reported dB = 10log10(power)
                                           # + this + the analog-gain step.
                                           # The only level constant there is --
                                           # pwr_results itself is normalised
                                           # per ping and carries no level.
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
                                           # sees it at -depression and only
                                           # the sidelobe floor passes
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
    noise_floor: float = 8.3e-11           # receiver floor, additive linear
                                           # power. MEASURED separately (2026-
                                           # 09-02): the far decile of the
                                           # gain-4 pings of the 72-126 m
                                           # recordings, where nothing but
                                           # noise is left, reads -6.1 dB
                                           # (mean of dB; -3.6 dB as a linear
                                           # mean, which this is) -> 10**((
                                           # -3.6 - 97.2)/10). The former
                                           # 6.79e-7 (38.5 dB) was fitted on
                                           # files that never reached the
                                           # floor and hid the range law
                                           # beyond ~20 m.
    gain_drift_amp: float = 0.0            # OFF: deferred to the downstream
    gain_drift_period_s: float = 45.0      # augmentation stage (set >0 to
    dropped_ping_prob: float = 0.0         # re-enable in the base model)
    watercolumn_noise: float = 1.15e-07    # volume reverberation inside the
                                           # water column (r < altitude).
                                           # MEASURED: the 1-3 m band of the
                                           # gain-4 corpus pings reads 25.3 dB
                                           # (mean of dB; 27.8 dB linear mean)
                                           # -> 10**((27.8 - 97.2)/10). Stays
                                           # 40 dB under the FBR peak, so
                                           # bottom tracking locks with margin.

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
