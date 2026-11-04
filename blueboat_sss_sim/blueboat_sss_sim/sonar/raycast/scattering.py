"""Diffuse scattering laws per material (notes 03, 05; NC #17).

``ssa`` (default): sediments (``seabed_ssa.SEDIMENTS``: sand, mud) take the
J&R small-slope roughness law + the sediment volume term; rough manufactured
materials (concrete, rubber, rope, ...; ``worldgen.materials._ACOUSTIC``)
take the sand law scaled by their normal-incidence power reflection
coefficient relative to sand's, ``|R_mat / R_sand|^2`` -- the rule the facet
layer already applies to its incoherent residual (``facets.FacetLayer``),
never a per-object brightness. Seabed materials without verified parameters
(gravel, rocks, seagrass; INDEX "full texts wanted") fall back to sand, with
a warning: a recorded gap, not a tuned value.

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


class DiffuseLaw:
    """sigma for the direct (monostatic) path, per material index."""

    def __init__(self, cfg: SeabedLawConfig, material_names=None,
                 k: float | None = None, frequency_hz: float = 451127.0,
                 c_water: float = 1500.0) -> None:
        self.cfg = cfg
        self.names = list(material_names or ["sand"])
        self._mu = 10.0 ** (cfg.lambert_mu_db / 10.0)
        self._scale = np.ones(len(self.names))
        self._law_of = np.zeros(len(self.names), dtype=np.int64)
        self._laws: list = []
        if cfg.law == "lambert":
            return
        if cfg.law != "ssa":
            raise ValueError(f"seabed law {cfg.law!r} (ssa | lambert)")
        if k is None:
            raise ValueError("the ssa law needs the wavenumber")
        from ...worldgen.materials import (SAND_REFLECTION, MaterialLibrary,
                                           _ACOUSTIC, acoustic_reflection)
        tables: dict = {}
        laws: dict = {}
        self.sediments = _ssa.sediments_for(getattr(cfg, "sediments", None))
        for name, p in self.sediments.items():
            beta = round(2.0 * p.alpha, 6)
            tables.setdefault(beta, _ssa.ik_table(beta))
            laws[name] = len(self._laws)
            self._laws.append(_ssa.SSALaw(p, k, frequency_hz, c_water,
                                          table=tables[beta]))
        lib = MaterialLibrary()
        for i, n in enumerate(self.names):
            if n in laws:
                self._law_of[i] = laws[n]
                continue
            self._law_of[i] = laws["sand"]
            if n in _ACOUSTIC:
                r = acoustic_reflection(lib.get(n), frequency_hz=frequency_hz)
                self._scale[i] = (r / SAND_REFLECTION) ** 2
            elif n != "outside":
                _log.warning("ray-cast: no verified seabed parameters for "
                             "material %r; the sand law is used (gap)", n)

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
        return out * self._scale[mat]

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
        return out * self._scale[mat]
