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
OMNISCAN_FREQ_HZ = 451_127          # observed on the real device
DEVICE_PROCESSING_MS = 2.0          # free-run overhead beyond two-way travel


@dataclass
class AcquisitionParams:
    """Run-dependent parameters, identical to the real node's ROS params."""

    range_start_mm: int = 0
    range_length_mm: int = 15_000
    msec_per_ping: int = 0                 # 0 = free-run (device behaviour)
    gain_index: int = 4
    num_results: int = 600
    pulse_len_percent: float = 0.002

    @property
    def range_max_m(self) -> float:
        return (self.range_start_mm + self.range_length_mm) / 1000.0

    @property
    def bin_size_m(self) -> float:
        return self.range_length_mm / 1000.0 / max(self.num_results, 1)

    def ping_period_s(self, max_ping_rate_hz: float = 0.0) -> float:
        """Effective ping period.

        Floors: the commanded ``msec_per_ping`` and the physical two-way
        travel time + device processing (22 ms at 15 m, as decoded from the
        real capture's device timestamps). If ``max_ping_rate_hz`` > 0 the
        period is additionally floored at 1/rate -- used to enforce the
        20 Hz maximum ping rate from the Omniscan 450 spec sheet.

        NOTE: the spec-sheet 20 Hz cap and the field capture's observed
        22 ms (45 Hz) free-run at 15 m are in tension; the cap is therefore
        a *configurable* model parameter (``model.max_ping_rate_hz``, 0 to
        disable and reproduce the captured behaviour)."""
        two_way = 2.0 * self.range_max_m / SPEED_OF_SOUND_MPS
        free_run = two_way + DEVICE_PROCESSING_MS / 1000.0
        period = max(self.msec_per_ping / 1000.0, free_run)
        if max_ping_rate_hz > 0.0:
            period = max(period, 1.0 / max_ping_rate_hz)
        return period

    def pulse_duration_s(self, max_ping_rate_hz: float = 0.0) -> float:
        """Device convention: pulse length as a fraction of the ping window."""
        return self.pulse_len_percent * self.ping_period_s(max_ping_rate_hz)


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
    max_ping_rate_hz: float = 20.0         # Omniscan 450 spec-sheet cap.
                                           # CAUTION: the team's own field
                                           # capture shows 22 ms (45 Hz)
                                           # free-run at 15 m; set 0 to
                                           # disable the cap and reproduce
                                           # the captured device behaviour.

    # Acoustic model.
    lambert_exponent: float = 1.7          # default when material info absent
    absorption_db_per_m: float = 0.10      # ~450 kHz seawater
    spreading_exponent: float = 2.0        # amplitude spreading (two-way)
    tvg_compensation: float = 0.90         # 0..1 fraction of range loss the
                                           # "device" gain removes
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
                                           # c*tau/2 (~3 bins at 100 us /
                                           # 25 mm bins): widens + stabilises
                                           # the FBR onset ramp, as the real
                                           # matched-filter output does
    alongtrack_beam_lines: int = 5         # ground lines integrated across the
                                           # 0.5 deg azimuth footprint (odd;
                                           # 1 = legacy infinitesimal beam)
    calibration_db_offset: float = 16.0    # max_pwr_db = 10log10(pwr)+offset
    base_scale: float = 110_000.0          # linear power -> u16 counts at gain 4.
                                           # Calibrated so a flat-seabed FBR
                                           # peaks at ~15-25k counts and speckle
                                           # maxima approach/occasionally clip
                                           # 65535, matching the real capture
                                           # (pwr_results 0-62k, max_pwr_db 63.9)
    gain_index_step_db: float = 3.0        # counts scaling per gain index step

    # Shallow-water multipath (optional, enclosed-basin regime realism).
    multipath_enabled: bool = False        # second-bottom-echo ghost on/off
    multipath_gain: float = 0.12           # ghost amplitude relative to direct

    # Noise model.
    speckle: bool = True
    speckle_looks: int = 1                 # 1 = fully-developed Exp(1) speckle
                                           # (Rayleigh amplitude); >1 = smoother
                                           # multi-look Gamma(L, 1/L) speckle
    noise_floor: float = 0.002             # relative to base_scale
    gain_drift_amp: float = 0.0            # OFF: deferred to the downstream
    gain_drift_period_s: float = 45.0      # augmentation stage (set >0 to
    dropped_ping_prob: float = 0.0         # re-enable in the base model)
    watercolumn_noise: float = 0.002       # relative noise in r < altitude bins
                                           # (must stay well below the first
                                           # bottom return for FBR detection)

    # Ground-line sampling: renderer step is min(sample_step_m, ~half the
    # slant bin) so every range bin receives samples at any num_results
    # (600, 1200, ...); this value is only the coarse upper bound.
    sample_step_m: float = 0.05

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "SonarModelConfig":
        known = {f.name for f in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        unknown = set(d) - known
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
