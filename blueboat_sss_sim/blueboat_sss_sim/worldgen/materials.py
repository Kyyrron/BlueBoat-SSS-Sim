"""Acoustic material definitions for seabed types and object materials.

Values are *relative* acoustic parameters tuned for a 450 kHz side-scan
regime; they parameterise the backscatter model in
:mod:`blueboat_sss_sim.sonar.acoustics`, not a physical scattering theory.
All defaults can be overridden from ``config/materials.yaml``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


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
        lambert_exp: exponent of the cos(theta) backscatter law; higher
            values = more specular-like (rock, metal), lower = diffuse (mud).
    """

    name: str
    reflectivity: float
    texture_amp: float
    texture_cells: int
    micro_roughness_m: float
    lambert_exp: float
    color: tuple[float, float, float] = (0.6, 0.55, 0.4)  # SDF visual only


_SEABED_DEFAULTS: dict[str, Material] = {
    "sand":     Material("sand",     0.55, 0.10, 220, 0.010, 1.6, (0.76, 0.70, 0.50)),
    "mud":      Material("mud",      0.30, 0.06, 150, 0.004, 1.2, (0.45, 0.40, 0.33)),
    "gravel":   Material("gravel",   0.70, 0.22, 320, 0.025, 1.8, (0.55, 0.53, 0.50)),
    "rocks":    Material("rocks",    0.85, 0.30, 260, 0.080, 2.2, (0.42, 0.42, 0.44)),
    "seagrass": Material("seagrass", 0.22, 0.35, 300, 0.030, 1.0, (0.20, 0.45, 0.25)),
}

_OBJECT_DEFAULTS: dict[str, Material] = {
    "rubber":   Material("rubber",   0.45, 0.05, 0, 0.0, 1.5, (0.10, 0.10, 0.10)),
    "pvc":      Material("pvc",      0.50, 0.05, 0, 0.0, 1.8, (0.85, 0.85, 0.80)),
    "plastic":  Material("plastic",  0.40, 0.05, 0, 0.0, 1.6, (0.90, 0.90, 0.95)),
    "metal":    Material("metal",    0.95, 0.05, 0, 0.0, 2.5, (0.60, 0.62, 0.65)),
    "concrete": Material("concrete", 0.80, 0.10, 0, 0.0, 2.0, (0.70, 0.70, 0.68)),
    "brickclay":Material("brickclay",0.75, 0.10, 0, 0.0, 2.0, (0.65, 0.30, 0.22)),
    "rope":     Material("rope",     0.35, 0.10, 0, 0.0, 1.3, (0.80, 0.75, 0.55)),
    "generic":  Material("generic",  0.60, 0.10, 0, 0.0, 1.8, (0.50, 0.50, 0.50)),
}


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
                "lambert_exp": params.get("lambert_exp", base.lambert_exp),
                "color": tuple(params.get("color", base.color)),
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
