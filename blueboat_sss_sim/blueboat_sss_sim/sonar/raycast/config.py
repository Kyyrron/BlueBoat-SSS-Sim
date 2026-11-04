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
    water: WaterConfig = field(default_factory=WaterConfig)
    beam: BeamConfig = field(default_factory=BeamConfig)
    sampling: SamplingConfig = field(default_factory=SamplingConfig)
    seabed: SeabedLawConfig = field(default_factory=SeabedLawConfig)
    receiver: ReceiverConfig = field(default_factory=ReceiverConfig)
    po: PoConfig = field(default_factory=PoConfig)
    specular: SpecularConfig = field(default_factory=SpecularConfig)
    paths: PathsConfig = field(default_factory=PathsConfig)
    diffraction: DiffractionConfig = field(default_factory=DiffractionConfig)

    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "RaycastConfig":
        d = dict(d or {})
        top = {"seed", "system_gain_db", "backend", "water", "beam", "sampling", "seabed",
               "receiver", "po", "specular", "paths", "diffraction"}
        bad = set(d) - top
        if bad:
            raise KeyError(f"Unknown raycast keys: {sorted(bad)}")
        return cls(seed=int(d.get("seed", 0)),
                   system_gain_db=float(d.get("system_gain_db", 126.6)),
                   backend=str(d.get("backend", "cpu")),
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
                                       "diffraction"))


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


def write_default_profile(path, base_yaml=None) -> None:
    """Generate ``config/raycast_default_sonar.yaml``: ``acquisition`` and
    ``model`` copied from the geometric comparison profile (so the shared
    stage never drifts from it), ``renderer: raycast`` and
    :data:`PROVISIONAL_RAYCAST`. Regenerate, never hand-edit."""
    import yaml
    from ..config import find_default_sonar_yaml
    base = yaml.safe_load(open(base_yaml or find_default_sonar_yaml(),
                               encoding="utf-8"))
    RaycastConfig.from_dict(PROVISIONAL_RAYCAST)
    doc = {"renderer": "raycast", "acquisition": base["acquisition"],
           "model": base["model"], "raycast": PROVISIONAL_RAYCAST}
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
