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


@dataclass(frozen=True)
class SamplingConfig:
    #: World lattice spacing of the diffuse scatterers [m]. Provisional,
    #: from the Phase-2 S1 sweep (flat sand, mean vs quadrature: rms 0.10 /
    #: 0.025 / 0.006 dB at 10 / 5 / 2.5 mm, ~spacing^2); final value by the
    #: Phase-5 convergence study, never relaxed for speed.
    scatterer_spacing_m: float = 0.005
    #: Maximum refinement per lattice cell on steep sheet cells (n^2 points).
    max_refine: int = 8
    #: Offset of a visibility ray's end off the surface [m].
    surface_offset_m: float = 1e-3


@dataclass(frozen=True)
class SeabedLawConfig:
    #: "ssa": the J&R small-slope roughness law + sediment volume (default,
    #: scattering.DiffuseLaw); "lambert": mu sin(theta_i) sin(theta_s), the
    #: integration machinery's test law (Mackenzie 1961, mu = -27 dB).
    law: str = "ssa"
    lambert_mu_db: float = -27.0


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
class RaycastConfig:
    seed: int = 0
    water: WaterConfig = field(default_factory=WaterConfig)
    beam: BeamConfig = field(default_factory=BeamConfig)
    sampling: SamplingConfig = field(default_factory=SamplingConfig)
    seabed: SeabedLawConfig = field(default_factory=SeabedLawConfig)
    receiver: ReceiverConfig = field(default_factory=ReceiverConfig)
    po: PoConfig = field(default_factory=PoConfig)

    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "RaycastConfig":
        d = dict(d or {})
        top = {"seed", "water", "beam", "sampling", "seabed", "receiver", "po"}
        bad = set(d) - top
        if bad:
            raise KeyError(f"Unknown raycast keys: {sorted(bad)}")
        return cls(seed=int(d.get("seed", 0)),
                   water=_strict(WaterConfig, d.get("water"), "water"),
                   beam=_strict(BeamConfig, d.get("beam"), "beam"),
                   sampling=_strict(SamplingConfig, d.get("sampling"),
                                    "sampling"),
                   seabed=_strict(SeabedLawConfig, d.get("seabed"), "seabed"),
                   receiver=_strict(ReceiverConfig, d.get("receiver"),
                                    "receiver"),
                   po=_strict(PoConfig, d.get("po"), "po"))


def shared_stage_model(base, system_gain_db: float):
    """The ``SonarModelConfig`` the shared noise / encoder stages use with
    this sensor: ``calibration_db_offset`` = the ray-cast system gain, and
    the linear level constants that encode MEASURED reported levels
    (``noise_floor``, ``watercolumn_noise``; geometric units, paired with
    the geometric offset) rescaled by ``10^((offset_geo - gain)/10)`` so
    their reported dB are unchanged. ``reverberation_db`` 0: the shadow
    floor comes from physics (raycast_design §3). Found 2026-11-03 (NC #4
    step): without the rescaling the water column read 45 dB too loud."""
    import dataclasses
    scale = 10.0 ** ((float(base.calibration_db_offset) - float(system_gain_db))
                     / 10.0)
    return dataclasses.replace(
        base, calibration_db_offset=float(system_gain_db),
        noise_floor=float(base.noise_floor) * scale,
        watercolumn_noise=float(base.watercolumn_noise) * scale,
        reverberation_db=0.0)
