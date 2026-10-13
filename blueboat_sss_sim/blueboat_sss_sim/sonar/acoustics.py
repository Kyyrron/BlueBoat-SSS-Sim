"""Acoustic response model for the geometric SSS renderer.

Assumptions (documented, deliberately simple -- see docs/sonar_model.md):

* Straight-ray propagation (no refraction; valid for short shallow ranges).
* Lambertian-family backscatter:  BS = rho * cos(theta_i)^p, where theta_i
  is the local incidence angle and p a material/global exponent.
* Two-way loss = geometric spreading (1/r^k) x absorption (alpha dB/m),
  partially compensated by an idealised device TVG (fraction ``tvg``),
  leaving a realistic residual range-dependent brightness falloff.
* Vertical beam pattern: Gaussian in depression angle around the mounted
  tilt; the pattern rides on the vehicle roll (surface-motion signature,
  contribution C4 of the thesis).

All functions are vectorised over range samples of one ping.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np

from .config import SonarModelConfig


def backscatter(reflectivity: np.ndarray, cos_incidence: np.ndarray,
                lambert_exponent: float,
                volume_coeff: float = 0.0) -> np.ndarray:
    """Per-sample backscatter strength (linear amplitude units).

    ``volume_coeff`` adds a weakly angle-dependent sediment term, Bell &
    Linnett 1997 eq. 12's ``sigma_bv`` beside the surface ``sigma_br``. It
    is 0 by default and deliberately so: the Lambert coefficient this model
    is calibrated on is an empirical total over the corpus's grazing band,
    which already contains whatever volume scattering that band sees.
    """
    surf = reflectivity * np.power(np.clip(cos_incidence, 0.0, 1.0),
                                   lambert_exponent)
    if volume_coeff <= 0.0:
        return surf
    return surf + volume_coeff * reflectivity


def beam_weight(depression_rad: np.ndarray, cfg: SonarModelConfig,
                roll_toward_side: float) -> np.ndarray:
    """Vertical beam pattern weight in [0, 1].

    ``depression_rad``: angle of the ray below horizontal (0 = horizontal,
    pi/2 = straight down). ``roll_toward_side``: vehicle roll projected onto
    this side's look direction (positive = fan pushed downward).

    A real transducer has sidelobes, so the pattern is floored at
    ``beam_sidelobe_floor``: without it, near-nadir rays (~70 deg off the
    beam axis at shallow-water altitudes) are attenuated so hard that the
    first bottom return disappears below the water-column noise and
    downstream FBR/bottom-tracking cannot lock."""
    center = np.radians(cfg.beam_tilt_deg) + roll_toward_side
    sigma = np.radians(cfg.vertical_aperture_deg) / 2.355  # FWHM -> sigma
    main = np.exp(-0.5 * ((depression_rad - center) / max(sigma, 1e-6)) ** 2)
    return np.maximum(main, cfg.beam_sidelobe_floor)


def specular(reflectivity: np.ndarray, cos_incidence: np.ndarray,
             cfg: SonarModelConfig,
             width_deg: np.ndarray | None = None) -> np.ndarray:
    """Near-normal-incidence specular lobe (linear amplitude units).

    Lambert alone underestimates the echo at incidence angles near the
    surface normal; the strong specular/coherent component there is what
    makes the first bottom return the brightest feature of a real SSS ping
    (and what bottom-tracking algorithms lock onto). Modelled as a Gaussian
    in the angle from the normal, ``specular_width_deg`` wide, with
    amplitude ``specular_strength`` weakly modulated by material."""
    if cfg.specular_strength <= 0.0:
        return np.zeros_like(cos_incidence)
    theta_n = np.arccos(np.clip(cos_incidence, 0.0, 1.0))
    sigma = np.radians(max(cfg.specular_width_deg, 1e-3))
    if width_deg is not None:
        sigma = np.radians(np.maximum(width_deg, 1e-3))
    lobe = np.exp(-0.5 * (theta_n / sigma) ** 2)
    return cfg.specular_strength * (0.5 + 0.5 * reflectivity) * lobe


#: Speed of sound the wavenumber is built from, matching
#: ``sonar/config.SPEED_OF_SOUND_MPS`` and ``sonar/multipath.WAVENUMBER``.
_C_WATER_MPS = 1500.0

#: RMS height a cell with no smooth surface on it is treated as having.
#: Any value past COHERENT_ROUGHNESS_LIMIT_M gives C = 0 exactly.
_ROUGH_DEFAULT_M = 0.01


def wavenumber(cfg: SonarModelConfig) -> float:
    """Acoustic wavenumber k = 2 pi f / c [rad/m] (1890 at 450 kHz)."""
    return 2.0 * np.pi * float(cfg.sonar_frequency_hz) / _C_WATER_MPS


def coherent_fraction(cos_incidence: np.ndarray, roughness_m: np.ndarray,
                      cfg: SonarModelConfig) -> np.ndarray:
    """Fraction of the echo that stays coherent off a surface of RMS height
    ``roughness_m`` -- the Eckart/Rayleigh law ``exp(-(2 k sigma cos th)^2)``,
    the same one ``sonar/multipath.bounce_weight`` applies to a wall.

    What is coherent leaves in the *mirror* direction, so it does not come
    back to the transducer; only ``1 - C`` scatters diffusely. At 450 kHz
    this is identically 0 for every natural seabed (``2 k sigma`` is 15 for
    mud, 38 for sand) and appreciable only for manufactured surfaces: the
    field box's injection-moulded shell, at 0.1 mm RMS, keeps 87 % of its
    near-normal echo coherent and therefore sends it away -- which is why it
    images as a faint patch and not as a glint.
    """
    g = 2.0 * wavenumber(cfg) * np.asarray(roughness_m, dtype=np.float64) \
        * np.clip(cos_incidence, 0.0, 1.0)
    return np.exp(-np.minimum(g * g, 700.0))


def two_way_loss(slant_range: np.ndarray, cfg: SonarModelConfig,
                 exponent: float | None = None) -> np.ndarray:
    """Combined spreading + absorption loss (linear, <= 1).

    ``exponent`` overrides the spreading exponent: the diffuse field
    spreads twice (out and back) from every scatterer, r^-4 in intensity,
    while a *coherent* reflection off a flat interface is the field of an
    image source and spreads once over the folded path -- the reason the
    first bottom return falls far more slowly with altitude than the
    seabed around it does (measured: the reported peak falls 11.28 dB per
    decade of altitude over 19 gain-4 corpus recordings, against 40+ for
    the tail).
    """
    r = np.maximum(slant_range, 0.05)
    e = cfg.spreading_exponent if exponent is None else float(exponent)
    spreading = r ** (-e)
    absorption = 10.0 ** (-2.0 * cfg.absorption_db_per_m * r / 10.0)
    return spreading * absorption


def tvg_gain(slant_range: np.ndarray, cfg: SonarModelConfig) -> np.ndarray:
    """Idealised device time-varying gain: removes a fraction ``tvg`` of the
    modelled two-way loss (in dB), so residual falloff remains."""
    loss = two_way_loss(slant_range, cfg)
    return loss ** (-cfg.tvg_compensation)


def net_range_response(slant_range: np.ndarray, cfg: SonarModelConfig,
                       coherent: bool = False) -> np.ndarray:
    """loss x TVG in one call (numerically stable combined form).

    The device's time-varying gain is one curve of range, so it is always
    built from the *diffuse* loss; only the loss itself differs between
    the incoherent field and a coherent (specular) reflection.
    """
    if not coherent or (cfg.specular_spreading_exponent
                        == cfg.spreading_exponent):
        return two_way_loss(slant_range, cfg) ** (1.0 - cfg.tvg_compensation)
    return (two_way_loss(slant_range, cfg,
                         cfg.specular_spreading_exponent)
            * two_way_loss(slant_range, cfg) ** (-cfg.tvg_compensation))


# --------------------------------------------------------- the forward model
class SurfaceResponse(NamedTuple):
    """The per-sample pieces of one surface element's echo, before any
    azimuth weighting, occlusion or binning.

    Kept as separate fields rather than one number because the renderer
    multiplies wall folding, visibility and azimuth weights into the shared
    ``beam`` factor before combining, and because the incoherent and coherent
    channels carry different fluctuation statistics downstream
    (``sonar/noise.py``) and different range laws.
    """

    diffuse: np.ndarray          #: incoherent backscatter cross-section
    coherent: np.ndarray         #: near-normal (rough-surface Kirchhoff) lobe
    beam: np.ndarray             #: vertical beam pattern weight
    range_diffuse: np.ndarray    #: net range response, diffuse spreading law
    range_coherent: np.ndarray   #: net range response, coherent spreading law


def surface_response(reflectivity: np.ndarray, cos_incidence: np.ndarray,
                     depression_rad: np.ndarray, slant: np.ndarray,
                     cfg: SonarModelConfig,
                     roll_toward_side: float = 0.0,
                     roughness_m: np.ndarray | None = None
                     ) -> SurfaceResponse:
    """The one place the per-sample acoustic response is composed.

    Three callers must agree on it or the simulator and the tools that
    *invert* it drift apart silently: ``sonar/renderer.py`` (the forward
    render), ``analysis/backscatter.py`` (the reflectivity inversion) and,
    through that, ``analysis/shading.py`` (the relief inversion). Any new
    term -- volume scattering, a coherent/incoherent roughness split -- is
    added here once and reaches all three.
    """
    diffuse = backscatter(reflectivity, cos_incidence,
                          cfg.lambert_exponent,
                          cfg.volume_scattering_coeff)
    width = None
    incoh = None
    if cfg.coherent_split and roughness_m is not None:
        smooth = np.asarray(roughness_m, dtype=np.float64) > 0.0
        if smooth.any():
            # The Rayleigh factor is scoped to the NEAR-NORMAL lobe, and only
            # there. It says how much of the echo stays coherent, and coherent
            # energy leaves in the mirror direction: a wall 19 deg off normal
            # sends it 38 deg away from the transducer, so that part is lost
            # and only (1 - C) comes back. At wide angles the mirror direction
            # is nowhere near the transducer to begin with -- the Lambert tail
            # is backscatter from roughness that never had a coherent share to
            # lose -- so multiplying it too would be double-counting, and it
            # measurably was: a smooth crate lying flat went 16 dB dark at
            # 25 deg grazing, where the incidence is 65 deg from its normal.
            rough_eff = np.where(smooth, roughness_m, _ROUGH_DEFAULT_M)
            incoh = 1.0 - coherent_fraction(cos_incidence, rough_eff, cfg)
            # The near-normal peak is a ROUGH-surface Kirchhoff lobe whose
            # width is an RMS surface slope, and the 8 deg it carries is
            # sand's, fitted on the corpus. A manufactured surface has no
            # measured slope distribution, and lending it sand's is what
            # produced the glint: a stamped height step always contains one
            # facet exactly normal to the ray, whatever the grid spacing,
            # and that facet then collects the whole lobe. So on a surface
            # the roughness raster calls smooth the lobe is suppressed
            # (width 0), and a width may be set per run once a field capture
            # measures one.
            if cfg.smooth_lobe_width_deg > 0.0:
                width = np.where(smooth, cfg.smooth_lobe_width_deg,
                                 cfg.specular_width_deg)
            else:
                incoh = np.where(smooth, 0.0, incoh)
    coherent = specular(reflectivity, cos_incidence, cfg, width)
    if incoh is not None:
        coherent = coherent * incoh
    rng_d = net_range_response(slant, cfg)
    rng_c = net_range_response(slant, cfg, coherent=True)
    if incoh is not None and np.ndim(rng_c) > 0:
        # The coherent spreading law (`specular_spreading_exponent`, NC #4)
        # says an image source spreads ONCE over the folded path. That is a
        # statement about an *extended* flat interface -- the seabed under
        # the boat, metres across. A stamped object's facet is not one: the
        # first Fresnel zone at 8 m and 3.3 mm is 0.16 m in radius, and the
        # field box's wall is 0.32 m tall, so it is a single zone at best and
        # its echo spreads twice like any finite scatterer. Without this the
        # r^2 bonus lands on every object edge and no lobe width can undo it
        # -- measured +8 dB of residual highlight at a 1 deg lobe.
        rng_c = np.where(np.asarray(roughness_m, dtype=np.float64) > 0.0,
                         rng_d, rng_c)
    return SurfaceResponse(
        diffuse=diffuse,
        coherent=coherent,
        beam=beam_weight(depression_rad, cfg, roll_toward_side),
        range_diffuse=rng_d,
        range_coherent=rng_c,
    )


def combined_power(sr: SurfaceResponse, visible=1.0) -> np.ndarray:
    """``SurfaceResponse`` reduced to the noiseless mean power of a sample.

    The renderer does not call this -- it needs the channels apart, to bin
    them separately -- but it is the definition the renderer's own
    combination must match, and the analytic tools use it directly.
    """
    return ((sr.diffuse * sr.range_diffuse
             + sr.coherent * sr.range_coherent) * sr.beam) * visible
