"""Stochastic degradations applied to rendered pings.

* **Speckle** -- fully-developed speckle: measured intensity of a rough
  surface is exponentially distributed around the mean backscatter
  (multiplicative Exp(1)); the single most important statistical property
  a detector trained on synthetic SSS must see.
* **Noise floor** -- additive electronic/ambient noise (exponential too).
* **Gain drift** -- slow multiplicative wander of the receive chain,
  modelled as a sinusoid + random-walk mixture.
* **Dropped pings** -- Bernoulli loss reproducing occasional missing
  profiles observed on the real link.

Stateless functions except :class:`GainDrift`, which is per-side stateful.
"""

from __future__ import annotations

import numpy as np

from .config import SonarModelConfig


class GainDrift:
    """Slow multiplicative gain wander for one transducer channel."""

    def __init__(self, cfg: SonarModelConfig, rng: np.random.Generator) -> None:
        self._cfg = cfg
        self._phase = rng.uniform(0.0, 2.0 * np.pi)
        self._walk = 0.0
        self._rng = rng

    def value(self, t_sim: float) -> float:
        cfg = self._cfg
        if cfg.gain_drift_amp <= 0:
            return 1.0
        self._walk = 0.995 * self._walk + 0.005 * self._rng.normal(0.0, 1.0)
        s = np.sin(2.0 * np.pi * t_sim / max(cfg.gain_drift_period_s, 1.0)
                   + self._phase)
        return float(1.0 + cfg.gain_drift_amp * (0.7 * s + 0.3 * self._walk))


def apply_ping_noise(power: np.ndarray, watercolumn_mask: np.ndarray,
                     gain: float, cfg: SonarModelConfig,
                     rng: np.random.Generator,
                     specular: np.ndarray | None = None) -> np.ndarray:
    """Return the noisy linear power vector (does not modify inputs).

    Speckle statistics, per component:

    * **Diffuse** (``power``): ``speckle_looks == 1`` gives fully-developed
      speckle -- intensity multiplicatively Exp(1)-distributed, i.e.
      Rayleigh amplitude, the textbook single-look SSS model.
      ``speckle_looks = L > 1`` gives multi-look Gamma(L, 1/L) speckle
      (mean 1, CV 1/sqrt(L)).
    * **Coherent specular** (``specular``, near-nadir first-return lobe):
      the coherent echo does not fully decorrelate, so it fluctuates far
      less than the diffuse field (Rician with high K-factor). Modelled as
      Gamma(``specular_looks``, 1/looks), CV = 1/sqrt(looks) (~0.2 at the
      default 25). This is what makes the real device's first bottom
      return a *stable* feature that bottom-tracking can lock onto
      ping after ping."""
    out = power.astype(np.float64, copy=True)
    if cfg.speckle:
        looks = max(int(cfg.speckle_looks), 1)
        if looks == 1:
            out *= rng.exponential(1.0, size=out.shape)
        else:
            out *= rng.gamma(looks, 1.0 / looks, size=out.shape)
    if specular is not None:
        spec = specular.astype(np.float64, copy=True)
        if cfg.speckle:
            slooks = max(int(cfg.specular_looks), 1)
            spec *= rng.gamma(slooks, 1.0 / slooks, size=spec.shape)
        out += spec
    floor = cfg.noise_floor * rng.exponential(1.0, size=out.shape)
    out += floor
    if watercolumn_mask.any():
        out[watercolumn_mask] += cfg.watercolumn_noise * \
            rng.exponential(1.0, size=int(watercolumn_mask.sum()))
    return out * gain


def ping_dropped(cfg: SonarModelConfig, rng: np.random.Generator) -> bool:
    return bool(rng.uniform() < cfg.dropped_ping_prob)
