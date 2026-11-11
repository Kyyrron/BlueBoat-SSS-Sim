"""Diffuse scattering laws per material (notes 03, 05; NC #17).

``ssa`` (default): sediments (``seabed_ssa.SEDIMENTS``) take the J&R
small-slope roughness law + the sediment volume term. A manufactured surface
(an object part, a wall; ``worldgen.materials``) takes the sand law scaled by
its normal-incidence power reflection coefficient relative to sand's,
``|R_mat / R_sand|^2`` -- the rule the facet layer applies to its incoherent
residual (``facets.FacetLayer``), never a per-object brightness. With
``split`` (``objects.mode: solid``) a smooth material scatters diffusely only
the share its roughness takes out of the mirror, ``1 - C`` with
``C = exp(-(k sigma (cos_i + cos_s))^2)`` (Eckart 1953); its coherent share is
the Kirchhoff patches (``renderer.deposit_patches``). ``fouling`` in [0, 1] is
the fraction of every manufactured surface covered by sediment and growth: it
scatters like the sand law and does not mirror. Gravel and rock, which have
no parameter set here, take the sand law at their published level above sand
(``SEABED_CONTRAST_DB``). A material with neither (rope, generic, seagrass)
falls back to sand, with a warning: a recorded gap, not a tuned value.

``lambert``: ``mu sin(theta_i) sin(theta_s)``, mu = -27 dB (Mackenzie 1961,
doi:10.1121/1.1908482) -- kept as the integration machinery's test law.

``sigma`` is the bistatic cross-section per unit area per unit solid angle,
so a scatterer standing for area dA has ``E|a|^2 = sigma dA``.
"""

from __future__ import annotations

import logging

import numpy as np

from .config import SeabedLawConfig
from . import seabed_ssa as _ssa

_log = logging.getLogger(__name__)

#: Backscatter level of a seabed type above sand at hundreds of kHz [dB]:
#: Jackson & Richardson 2007 Sect. 12.1 (Lambert fits mud -19.7, sand -20.2,
#: gravel -11.7, rock -5.6 dB with their frequency slopes) carried to
#: 450 kHz, the ordering Fonseca et al. 2025 measure at 290-440 kHz
#: (``docs/seabed_model.md`` Sect. 7.1; ``config/materials_literature.yaml``
#: holds the same numbers). A level on the sand law, not a roughness
#: spectrum: the within-class spread is +-5-8 dB.
SEABED_CONTRAST_DB = {"gravel": 3.0, "rocks": 6.5}


def base_material(name: str) -> str:
    """The library material of a mesh material name: ``scene_mesh`` tags a
    wall's (``metal#wall``) and a construction-specific instance's
    (``pp_shell#12``)."""
    return name.partition("#")[0]


def manufactured_surface(name: str, lib, frequency_hz: float, objects=()
                         ) -> tuple[float, float] | None:
    """``(|R|, RMS height [m])`` of a manufactured surface as the mesh names
    it, or None when its material carries no acoustic constants. A wall is
    rough to the diffuse law (RMS 0 here: no split; its own ``roughness_m``
    is the bounce path's); ``<material>#<id>`` is that object's construction
    (``facets.object_reflection``)."""
    from ...worldgen.materials import acoustic_reflection, surface_roughness_m
    base, _, tag = name.partition("#")
    try:
        mat = lib.get(base)
    except KeyError:
        return None
    owner = None if tag in ("", "wall") else next(
        (o for o in objects if int(o.object_id) == int(tag)), None)
    if owner is not None:
        from ...worldgen.objects import object_roughness_m
        from ..facets import object_reflection
        r, rms = object_reflection(owner, lib, frequency_hz), object_roughness_m(owner)
    else:
        r = acoustic_reflection(mat, frequency_hz=frequency_hz)
        rms = 0.0 if tag == "wall" else surface_roughness_m(mat)
    return (float(r), float(rms)) if r > 0.0 else None


class DiffuseLaw:
    """sigma for the direct (monostatic) path, per material index."""

    def __init__(self, cfg: SeabedLawConfig, material_names=None,
                 k: float | None = None, frequency_hz: float = 451127.0,
                 c_water: float = 1500.0, lib=None, objects=(),
                 split: bool = False, fouling: float = 0.0) -> None:
        self.cfg = cfg
        self.names = list(material_names or ["sand"])
        self._mu = 10.0 ** (cfg.lambert_mu_db / 10.0)
        n = len(self.names)
        self._scale = np.ones(n)
        #: (2 k sigma)^2 of a smooth manufactured material (0 = no split)
        self._kr2 = np.zeros(n)
        #: the fouled fraction of a manufactured material, and its complement
        self._cov = np.zeros(n)
        self._keep = np.ones(n)
        self._law_of = np.zeros(n, dtype=np.int64)
        self._laws: list = []
        if cfg.law == "lambert":
            return
        if cfg.law != "ssa":
            raise ValueError(f"seabed law {cfg.law!r} (ssa | lambert)")
        if k is None:
            raise ValueError("the ssa law needs the wavenumber")
        from ...worldgen.materials import (COHERENT_ROUGHNESS_LIMIT_M,
                                           SAND_REFLECTION, MaterialLibrary)
        if not 0.0 <= float(fouling) <= 1.0:
            raise ValueError(f"fouling {fouling} outside [0, 1]")
        tables: dict = {}
        laws: dict = {}
        self.sediments = _ssa.sediments_for(getattr(cfg, "sediments", None))
        for name, p in self.sediments.items():
            beta = round(2.0 * p.alpha, 6)
            tables.setdefault(beta, _ssa.ik_table(beta))
            laws[name] = len(self._laws)
            self._laws.append(_ssa.SSALaw(p, k, frequency_hz, c_water,
                                          table=tables[beta]))
        lib = lib if lib is not None else MaterialLibrary()
        for i, n in enumerate(self.names):
            if n in laws:
                self._law_of[i] = laws[n]
                continue
            self._law_of[i] = laws["sand"]
            surf = manufactured_surface(n, lib, frequency_hz, objects)
            if surf is not None:
                self._scale[i] = (surf[0] / SAND_REFLECTION) ** 2
                self._cov[i], self._keep[i] = fouling, 1.0 - fouling
                if split and 0.0 < surf[1] < COHERENT_ROUGHNESS_LIMIT_M:
                    self._kr2[i] = (2.0 * k * surf[1]) ** 2
            elif n in SEABED_CONTRAST_DB:
                self._scale[i] = 10.0 ** (SEABED_CONTRAST_DB[n] / 10.0)
            elif n != "outside":
                _log.warning("ray-cast: no verified seabed parameters for "
                             "material %r; the sand law is used (gap)", n)

    def _factor(self, mat: np.ndarray, ci: np.ndarray, cs: np.ndarray) -> np.ndarray:
        """What multiplies the law, per scatterer: the material's reflection
        scale, its diffuse share ``1 - C``, the fouled fraction."""
        a = self._scale[mat]
        q = self._kr2[mat]
        if q.any():
            a = np.where(q > 0.0, a * -np.expm1(-0.25 * q * (ci + cs) ** 2), a)
        if self._cov.any():
            a = a * self._keep[mat] + self._cov[mat]
        return a

    def sigma(self, cos_i: np.ndarray, cos_s: np.ndarray,
              material: np.ndarray | None = None) -> np.ndarray:
        """Monostatic backscatter: ``cos_i`` = ``cos_s`` = cosine of the
        angle from the facet normal (= sine of the local grazing angle)."""
        ci = np.clip(np.asarray(cos_i, dtype=np.float64), 0.0, 1.0)
        cs = np.clip(np.asarray(cos_s, dtype=np.float64), 0.0, 1.0)
        if self.cfg.law == "lambert":
            return self._mu * ci * cs
        mat = (np.zeros(ci.shape, dtype=np.int64) if material is None
               else np.asarray(material, dtype=np.int64))
        out = np.zeros(ci.shape)
        law_idx = self._law_of[mat]
        for j, law in enumerate(self._laws):
            m = (law_idx == j) & (ci > 0.0)
            if m.any():
                out[m] = law.sigma(ci[m], cs[m], -np.ones(int(m.sum())))
        return out * self._factor(mat, ci, cs)

    def sigma_bistatic(self, normal: np.ndarray, d_in: np.ndarray,
                       d_out: np.ndarray, material: np.ndarray | None = None
                       ) -> np.ndarray:
        """Bistatic cross-section per unit area per steradian: ``d_in`` the
        propagation direction arriving at the facet, ``d_out`` the direction
        leaving it toward the receiver (J&R 13.1-13.7 geometry; the SSA law
        and the volume term are written for any (theta_i, theta_s, phi)).
        Backscatter (d_out = -d_in) equals :meth:`sigma`. 0 where either
        leg meets the facet from behind."""
        from .paths import bistatic_angles
        si, ss, cphi = bistatic_angles(normal, d_in, d_out)
        front = (si > 0.0) & (ss > 0.0)
        si = np.clip(si, 0.0, 1.0)
        ss = np.clip(ss, 0.0, 1.0)
        if self.cfg.law == "lambert":
            return np.where(front, self._mu * si * ss, 0.0)
        mat = (np.zeros(si.shape, dtype=np.int64) if material is None
               else np.asarray(material, dtype=np.int64))
        out = np.zeros(si.shape)
        law_idx = self._law_of[mat]
        for j, law in enumerate(self._laws):
            m = (law_idx == j) & front
            if m.any():
                out[m] = law.sigma(si[m], ss[m], cphi[m])
        return out * self._factor(mat, si, ss)
