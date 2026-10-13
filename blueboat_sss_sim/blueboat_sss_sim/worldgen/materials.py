"""Acoustic material definitions for seabed types and object materials.

Values are *relative* acoustic parameters tuned for a 450 kHz side-scan
regime; they parameterise the backscatter model in
:mod:`blueboat_sss_sim.sonar.acoustics`, not a physical scattering theory.
All defaults can be overridden from ``config/materials.yaml``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping

import numpy as np


#: Ceiling of the rendered reflectivity when a baked gain map is in force
#: (+6 dB over 1.0). A material's own value stays in [0, 1]; the survey
#: layer (``depth/layers/survey.py``) measures rock and gravel several dB
#: above sand ([B2] Jackson & Richardson 2007 par. 12.1) and needs the
#: headroom to render them. Without a gain map the clip stays at 1.0, so
#: every existing world is bit-identical.
REFLECTIVITY_MAX = 4.0


@dataclass(frozen=True)
class Material:
    """Acoustic surface description.

    Attributes:
        reflectivity: mean backscatter strength in [0, 1].
        texture_amp:  relative amplitude of fine reflectivity texture.
        texture_cells: spatial frequency of the texture (lattice cells over
            the world; larger = finer grain).
        micro_roughness_m: RMS amplitude of micro-relief added to the
            heightmap [m] (drives incidence-angle variation / graininess).
        (There is no per-material angular exponent. There used to be a
            ``lambert_exp`` field, read by nothing: the renderer applies one
            global ``SonarModelConfig.lambert_exponent``, and the 2026-09-18
            two-altitude corpus fit measures that exponent at 1.0 over sand
            -- against the 1.6 the dead field claimed for it. A per-material
            angular law is a real effect [J&R par. 12.1], but nothing in this
            corpus measures it, so the knob is gone rather than misleading.)
        transmission: fraction of the incident intensity that reaches the
            seabed *behind* an object of this material and comes back --
            the two-way transmission of its shell. 0 (every material but
            ``pp_shell``) is an opaque object casting a black shadow, which
            is what the renderer did for every object before 2026-09-18.
        roughness_m: RMS height of the surface itself [m], the Rayleigh
            roughness that decides how much of the echo stays *coherent*:
            ``C = exp(-(2 k sigma cos theta)^2)`` (Eckart; the same law
            ``sonar/multipath.bounce_weight`` already applies to a wall).
            What is coherent leaves in the mirror direction and does not
            come back; only ``1 - C`` scatters diffusely. At 450 kHz
            (k = 1890 rad/m) every seabed material is far past the limit --
            ``2 k sigma`` is 15 for the smoothest of them, mud -- so
            ``C = 0`` and the split is exactly a no-op for the seabed. It
            is manufactured surfaces that are smooth on this scale, and
            that is why a plastic box reads as a faint target: measured
            +2.5 dB over the sand around it at 19 deg grazing, against the
            +32 dB an unsplit model gave it (2026-09-19).
        rms_slope_deg: width of this material's near-normal (Kirchhoff)
            lobe, in degrees of surface slope. ``None`` uses the global
            ``SonarModelConfig.specular_width_deg``, which is the sand
            value the corpus fits (8 deg = an RMS slope of 0.14). Set per
            material only where a field measurement fixes it.
        density_kgm3, sound_speed_mps: the material's own density and
            longitudinal sound speed -- its acoustic impedance
            ``Z = density * sound_speed``, from which the facet layer
            (``sonar/facets.py``, NC #17) derives the reflection coefficient
            of a smooth object. Physical constants with citations, never
            fitted. 0 = unknown (rough materials, which never reach the
            facet layer).
        backing: what lies behind the surface. ``solid``: the material fills
            the object (a fluid half-space, ``R = (Z - Zw)/(Z + Zw)``).
            ``water``: a thin wall of ``wall_m`` with water on both sides (a
            flooded shell, the fluid-layer formula). ``air``: a thin wall
            with air behind it (a sealed can, air-backed foam): the
            water/air step reflects almost everything, ``|R| = 1``.
        wall_m: wall thickness of a ``water``-backed shell [m].
    """

    name: str
    reflectivity: float
    texture_amp: float
    texture_cells: int
    micro_roughness_m: float
    color: tuple[float, float, float] = (0.6, 0.55, 0.4)  # SDF visual only
    transmission: float = 0.0
    roughness_m: float = 0.0               # 0 = "take micro_roughness_m"
    rms_slope_deg: float | None = None     # None = the global lobe width
    density_kgm3: float = 0.0              # 0 = unknown
    sound_speed_mps: float = 0.0
    backing: str = "solid"                 # solid | water | air
    wall_m: float = 0.0                    # water-backed shell wall [m]


_SEABED_DEFAULTS: dict[str, Material] = {
    "sand":     Material("sand",     0.55, 0.10, 220, 0.010, (0.76, 0.70, 0.50)),
    "mud":      Material("mud",      0.30, 0.06, 150, 0.004, (0.45, 0.40, 0.33)),
    "gravel":   Material("gravel",   0.70, 0.22, 320, 0.025, (0.55, 0.53, 0.50)),
    "rocks":    Material("rocks",    0.85, 0.30, 260, 0.080, (0.42, 0.42, 0.44)),
    "seagrass": Material("seagrass", 0.22, 0.35, 300, 0.030, (0.20, 0.45, 0.25)),
}

#: Surface RMS height above which the coherent (mirror) fraction is zero to
#: float precision at 450 kHz: ``exp(-(2 k sigma)^2)`` with k = 1890 rad/m is
#: under 1e-16 by sigma = 1.6 mm. Every seabed material and every rough
#: object material sits past it, so the coherent/incoherent split is exactly
#: a no-op for them and only manufactured surfaces feel it.
COHERENT_ROUGHNESS_LIMIT_M = 1.6e-3

# Object-material RMS surface heights [m]. Judged from manufacture, not
# measured -- stated here rather than buried, because `roughness_m` is what
# decides whether a target reads as a glint or as a faint patch. The scale
# that matters is the 3.3 mm wavelength at 450 kHz: anything past
# COHERENT_ROUGHNESS_LIMIT_M is acoustically rough and behaves exactly as it
# did before 2026-09-19.
_ROUGHNESS_M: dict[str, float] = {
    # A tyre's return is its TREAD -- grooves and blocks centimetres deep,
    # acoustically rough -- and the steel belt under a rubber skin whose
    # impedance is within 15 % of water's. Rated at the moulded sidewall
    # (0.2 mm, 2026-09-19) it became a mirror of R = 0.07 under the facet
    # layer (NC #17), i.e. invisible, which no SSS image of a tyre shows.
    # Rough, it stays on the raster Lambert path it has always been on.
    "rubber": 3.0e-3,      # tread
    "pvc": 1.0e-4,         # extruded
    "plastic": 1.0e-4,     # injection-moulded
    "metal": 5.0e-5,       # rolled sheet
    "concrete": 2.0e-3,    # form-finished -- acoustically rough
    "brickclay": 1.0e-3,
    "rope": 3.0e-3,        # fibre -- acoustically rough
    "generic": 2.0e-3,     # the rough default: keeps an unstated material
                           # behaving exactly as it did before the split
    "glass": 5.0e-5,
    "wood": 5.0e-4,        # sawn
    "eps": 5.0e-4,         # expanded-bead surface
    "tinplate": 5.0e-5,
    # The 2026-09-07 field box: an injection-moulded polypropylene shell,
    # smooth to a tenth of a millimetre, so 2 k sigma = 0.38 and 87 % of
    # its near-normal echo leaves in the mirror direction instead of
    # coming back. That single number is what separates the measured
    # +2.5 dB highlight from the +32 dB an unsplit model produced.
    "pp_shell": 1.0e-4,
}

_OBJECT_DEFAULTS: dict[str, Material] = {
    "rubber":   Material("rubber",   0.45, 0.05, 0, 0.0, (0.10, 0.10, 0.10)),
    "pvc":      Material("pvc",      0.50, 0.05, 0, 0.0, (0.85, 0.85, 0.80)),
    "plastic":  Material("plastic",  0.40, 0.05, 0, 0.0, (0.90, 0.90, 0.95)),
    "metal":    Material("metal",    0.95, 0.05, 0, 0.0, (0.60, 0.62, 0.65)),
    "concrete": Material("concrete", 0.80, 0.10, 0, 0.0, (0.70, 0.70, 0.68)),
    "brickclay":Material("brickclay",0.75, 0.10, 0, 0.0, (0.65, 0.30, 0.22)),
    "rope":     Material("rope",     0.35, 0.10, 0, 0.0, (0.80, 0.75, 0.55)),
    "generic":  Material("generic",  0.60, 0.10, 0, 0.0, (0.50, 0.50, 0.50)),
    # Target-family additions (target_families_and_surrogates.md):
    "glass":    Material("glass",    0.65, 0.05, 0, 0.0, (0.55, 0.70, 0.60)),
    "wood":     Material("wood",     0.35, 0.10, 0, 0.0, (0.55, 0.40, 0.25)),
    # Air-backed closed-cell foam is a *strong* reflector despite being
    # soft: the water/air impedance step is near-total.
    "eps":      Material("eps",      0.80, 0.05, 0, 0.0, (0.92, 0.92, 0.90)),
    "tinplate": Material("tinplate", 0.90, 0.05, 0, 0.0, (0.72, 0.72, 0.70)),
    # A thin polypropylene shell full of water -- the 2026-09-07 beach
    # target (a 68 L NAFCO storage box, lid on, flooded with water only).
    # It is a FACET object (NC #17): smooth (0.1 mm moulded), so it scatters
    # from its geometry through its reflection coefficient, which
    # `acoustic_reflection` derives from the constants in `_ACOUSTIC` (PP,
    # Selfridge 1985: 905 kg/m3, 2740 m/s; a 2 mm flooded wall -> the
    # fluid-layer formula, |R|^2 = 0.16 = -7.9 dB, transparent at a half
    # wavelength, 3.0 mm). `reflectivity` 0.18 is NOT read for it any more:
    # that number was the same power reflection coefficient used as a
    # Lambert coefficient, which a mirror-smooth surface does not have --
    # and it is what put the box +16 to +25 dB over the sand in every GUI
    # world (2026-09-28). It remains only for a world built before the
    # facet layer, which renders as it was built (SceneModel.stale_rasters).
    # ``transmission`` 0.11: MEASURED, not derived. The 2026-09-07 beach
    # recording puts the box's own shadow 9.6 dB under the sand around it
    # rather than at the receiver floor (2026-09-18 review par. 3.6), and
    # 10^(-9.6/10) = 0.11 is what a flooded shell passes through its walls
    # and back. Re-measured 2026-09-28 through the facet-layer renderer on
    # the two clear passes of object_boiteVsDistance (shadows -8.2 and
    # -7.4 dB at 6.1 and 10.1 m, the box's heading unknown): averaged over
    # headings, 0.11 is within 0.25 dB of them (0.13 is +0.26), so it
    # stands. The lid and floor, met at 70-80 deg from their normals, are
    # forward mirrors (-4 to -8 dB per crossing) -- which is why a shell
    # with nearly transparent walls still shadows by 9 dB.
    "pp_shell": Material("pp_shell", 0.18, 0.05, 0, 0.0, (0.88, 0.90, 0.92),
                         transmission=0.11),
}


for _n, _r in _ROUGHNESS_M.items():
    if _n in _OBJECT_DEFAULTS:
        _OBJECT_DEFAULTS[_n] = replace(_OBJECT_DEFAULTS[_n], roughness_m=_r)


#: The water every reflection coefficient is taken against: seawater density
#: at the model's own sound speed (``sonar/config.SPEED_OF_SOUND_MPS``).
WATER_DENSITY_KGM3 = 1025.0
WATER_SOUND_SPEED_MPS = 1500.0

# Object-material acoustic constants: (density kg/m3, longitudinal sound
# speed m/s, backing, wall m). PHYSICAL, cited, never fitted (NC #17):
# [S] Selfridge 1985, IEEE Trans. Sonics Ultrason. SU-32(3):381, Table I;
# [K] Kinsler, Frey, Coppens & Sanders, Fundamentals of Acoustics (4th ed.),
# App. A10. Only the smooth ones reach the facet layer (``sonar/facets.py``);
# the rough ones are listed for completeness and for a user who overrides
# their roughness.
_ACOUSTIC: dict[str, tuple[float, float, str, float]] = {
    "rubber":    (1100.0, 1600.0, "solid", 0.0),   # hard rubber [K]
    "pvc":       (1380.0, 2380.0, "solid", 0.0),   # PVC, grey [S]
    "plastic":   (950.0, 2430.0, "solid", 0.0),    # HDPE [S]; PP/PE family
    "metal":     (7850.0, 5900.0, "solid", 0.0),   # mild steel [K]
    "concrete":  (2400.0, 3700.0, "solid", 0.0),   # [K]
    "brickclay": (1800.0, 3600.0, "solid", 0.0),   # fired clay brick
    "glass":     (2500.0, 5600.0, "solid", 0.0),   # soda-lime glass [K]
    "wood":      (600.0, 4000.0, "solid", 0.0),    # softwood, along grain
    # Closed-cell foam: its own impedance is negligible, what the wave meets
    # is the air in the cells.
    "eps":       (20.0, 0.0, "air", 0.0),
    # A 0.3 mm tinplate sheet is 0.02 of a wavelength in steel, and the
    # fluid-layer formula gives |R| = 0.99 for it either way; the half-space
    # value (0.94) is used, like any steel.
    "tinplate":  (7850.0, 5900.0, "solid", 0.0),
    # The 2026-09-07 field box: polypropylene [S] (0.905 g/cc, 2.74 km/s),
    # a 2 mm moulded wall, flooded. The class's own `wall_mm` / `flooded`
    # knobs override the last two per instance.
    "pp_shell":  (905.0, 2740.0, "water", 2.0e-3),
}
for _n, (_d, _c, _bk, _w) in _ACOUSTIC.items():
    if _n in _OBJECT_DEFAULTS:
        _OBJECT_DEFAULTS[_n] = replace(
            _OBJECT_DEFAULTS[_n], density_kgm3=_d, sound_speed_mps=_c,
            backing=_bk, wall_m=_w)

#: The medium sand the incoherent term is scaled against: Jackson &
#: Richardson 2007 Table 4.3 (density ratio 1.845, sound-speed ratio 1.1782)
#: -> Z = 3.34 MRayl, R = 0.370. A rough surface's diffuse return scales,
#: to first order, with the power reflection coefficient of the interface
#: (Kirchhoff), so a smooth object's incoherent residual is referred to
#: sand's through |R|^2 instead of through a judged reflectivity.
SAND_DENSITY_KGM3 = 1.845 * WATER_DENSITY_KGM3
SAND_SOUND_SPEED_MPS = 1.1782 * WATER_SOUND_SPEED_MPS


def _half_space_r(density: float, speed: float) -> float:
    zw = WATER_DENSITY_KGM3 * WATER_SOUND_SPEED_MPS
    z = density * speed
    return abs(z - zw) / (z + zw)


#: |R| of the medium sand above (0.370).
SAND_REFLECTION = _half_space_r(SAND_DENSITY_KGM3, SAND_SOUND_SPEED_MPS)


def acoustic_reflection(mat: "Material", *, wall_m: float | None = None,
                        backing: str | None = None,
                        frequency_hz: float = 450_000.0) -> float:
    """Normal-incidence amplitude reflection coefficient |R| of a surface of
    ``mat`` in water -- what the facet layer's coherent term is scaled by.

    ``solid``: a fluid half-space, ``(Z - Zw) / (Z + Zw)``.
    ``water``: a wall of thickness ``d`` with water on both sides, the
    fluid-layer formula (Kinsler et al. par. 6.3; Brekhovskikh):
    ``R_I = (m - 1/m)^2 sin^2(k d) / [4 cos^2(k d) + (m + 1/m)^2 sin^2(k d)]``,
    ``m = Z / Zw``, ``k = 2 pi f / c`` in the wall. Resonant in ``d``: a
    2 mm polypropylene wall gives R_I = 0.15 (-8.3 dB), a half-wavelength
    wall (3.0 mm) is transparent.
    ``air``: a thin wall with air behind it, ``|R| = 1``.
    ``wall_m`` / ``backing`` override the material's own (the
    ``plastic_box`` class knobs). 0 when the material carries no acoustic
    constants.
    """
    bk = (backing or mat.backing or "solid").lower()
    if bk == "air":
        return 1.0
    if mat.density_kgm3 <= 0.0 or mat.sound_speed_mps <= 0.0:
        return 0.0
    if bk == "solid":
        return _half_space_r(mat.density_kgm3, mat.sound_speed_mps)
    if bk != "water":
        raise ValueError(f"material '{mat.name}': backing '{bk}' "
                         "(solid | water | air)")
    d = float(mat.wall_m if wall_m is None else wall_m)
    if d <= 0.0:
        return _half_space_r(mat.density_kgm3, mat.sound_speed_mps)
    m = (mat.density_kgm3 * mat.sound_speed_mps
         / (WATER_DENSITY_KGM3 * WATER_SOUND_SPEED_MPS))
    kd = 2.0 * np.pi * float(frequency_hz) / mat.sound_speed_mps * d
    s2, c2 = np.sin(kd) ** 2, np.cos(kd) ** 2
    r_i = (m - 1.0 / m) ** 2 * s2 / (4.0 * c2 + (m + 1.0 / m) ** 2 * s2)
    return float(np.sqrt(r_i))


def surface_roughness_m(mat: "Material") -> float:
    """The RMS height the Rayleigh split uses: the material's own
    ``roughness_m`` when set, else its ``micro_roughness_m`` (the seabed
    materials, which carry their roughness in that field and are all far
    past ``COHERENT_ROUGHNESS_LIMIT_M``), else the rough default."""
    if mat.roughness_m > 0.0:
        return float(mat.roughness_m)
    if mat.micro_roughness_m > 0.0:
        return float(mat.micro_roughness_m)
    return float(_ROUGHNESS_M["generic"])


class MaterialLibrary:
    """Lookup table of materials, optionally overridden from a YAML mapping."""

    def __init__(self, overrides: Mapping[str, Mapping[str, Any]] | None = None) -> None:
        self._materials: dict[str, Material] = {}
        self._materials.update(_SEABED_DEFAULTS)
        self._materials.update(_OBJECT_DEFAULTS)
        for name, params in (overrides or {}).items():
            base = self._materials.get(name, _OBJECT_DEFAULTS["generic"])
            merged = {
                "name": name,
                "reflectivity": params.get("reflectivity", base.reflectivity),
                "texture_amp": params.get("texture_amp", base.texture_amp),
                "texture_cells": params.get("texture_cells", base.texture_cells),
                "micro_roughness_m": params.get("micro_roughness_m", base.micro_roughness_m),
                "color": tuple(params.get("color", base.color)),
                "transmission": params.get("transmission", base.transmission),
                "roughness_m": params.get("roughness_m", base.roughness_m),
                "rms_slope_deg": params.get("rms_slope_deg", base.rms_slope_deg),
                "density_kgm3": params.get("density_kgm3", base.density_kgm3),
                "sound_speed_mps": params.get("sound_speed_mps",
                                              base.sound_speed_mps),
                "backing": params.get("backing", base.backing),
                "wall_m": params.get("wall_m", base.wall_m),
            }
            self._materials[name] = Material(**merged)

    def get(self, name: str) -> Material:
        try:
            return self._materials[name]
        except KeyError as exc:
            raise KeyError(
                f"Unknown material '{name}'. Known: {sorted(self._materials)}"
            ) from exc

    @property
    def seabed_names(self) -> list[str]:
        return list(_SEABED_DEFAULTS)

    @property
    def names(self) -> list[str]:
        """Every known material, seabed and object alike (the builder's
        per-object material picker)."""
        return sorted(self._materials)
