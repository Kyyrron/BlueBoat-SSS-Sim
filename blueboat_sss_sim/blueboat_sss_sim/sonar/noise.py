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
                     specular: np.ndarray | None = None,
                     texture: np.ndarray | None = None,
                     unoccluded: np.ndarray | None = None,
                     ambient: np.ndarray | None = None,
                     speckle_fill: np.ndarray | None = None,
                     floor_addition: str = "power") -> np.ndarray:
    """Return the noisy linear power vector (does not modify inputs).

    Speckle statistics, per component:

    * **Diffuse** (``power``): ``speckle_looks == 1`` gives fully-developed
      speckle -- intensity multiplicatively Exp(1)-distributed, i.e.
      Rayleigh amplitude, the textbook single-look SSS model. When the
      renderer supplies ``texture`` (its coherent ground-scatterer field,
      unit mean, Exp(1)-distributed per bin but *correlated* along range and
      along track like the real device's), that texture is used instead of
      an independent draw; the statistic per bin is the same, the
      neighbourhood structure is what changes.
      ``speckle_looks = L > 1`` gives multi-look Gamma(L, 1/L) speckle
      (mean 1, CV 1/sqrt(L)); the texture is not used in that mode.
    * **Coherent specular** (``specular``, near-nadir first-return lobe):
      the coherent echo does not fully decorrelate, so it fluctuates far
      less than the diffuse field (Rician with high K-factor). Modelled as
      Gamma(``specular_looks``, 1/looks), CV = 1/sqrt(looks) (~0.2 at the
      default 25). This is what makes the real device's first bottom
      return a *stable* feature that bottom-tracking can lock onto
      ping after ping.

    ``speckle_fill`` (``Ping.speckle_fill``) marks bins where the texture
    holds its 1.0 fallback but wall ghosts or the single-bounce band carry
    energy (behind a quay): they get their own Exp(1) draw, like any diffuse
    echo. ``ping_level_jitter_db`` > 0 multiplies the echoes (diffuse +
    coherent) by one unit-mean log-normal factor per call -- per side per
    ping, as the corpus's uncorrelated port/starboard anomalies say. Both
    draw from ``rng`` only when active, so a wall-free scene at zero jitter
    consumes exactly the stream it always did.

    ``floor_addition`` (2026-11-11; ``raycast.floor.addition``): how the
    reverberation floor meets the diffuse echo in a LIT bin. ``"power"``
    (default, the behaviour of every profile before): the floor's own Exp(1)
    intensity is added, which fills the speckle's deep nulls (the plain
    seabed's 1st percentile rises from -20.0 to -16.7 dB re the mean with an
    18 dB floor; the real harbour reads -20.5). ``"field"``: the floor is an
    independent complex Gaussian FIELD added before detection, i.e. the
    intensity gains the interference term ``2 sqrt(I_diffuse I_floor) cos(phi)``
    with ``phi`` uniform, so a lit bin stays exponential at the summed mean
    and a shadow (``I_diffuse = 0``) is filled exactly as before. Only bins
    where the reverberation IS the floor change; receiver noise and the water
    column stay power additions. ``phi`` is drawn last, so every other draw
    is the one ``"power"`` makes."""
    if floor_addition not in ("power", "field"):
        raise ValueError(f"floor_addition {floor_addition!r}: 'power' or 'field'")
    out = power.astype(np.float64, copy=True)
    if cfg.speckle:
        looks = max(int(cfg.speckle_looks), 1)
        if looks == 1:
            if texture is not None and cfg.coherent_scatterers:
                out *= texture
                if speckle_fill is not None and np.any(speckle_fill):
                    m = np.asarray(speckle_fill, dtype=bool)
                    out[m] *= rng.exponential(1.0, size=int(m.sum()))
            else:
                out *= rng.exponential(1.0, size=out.shape)
        else:
            out *= rng.gamma(looks, 1.0 / looks, size=out.shape)
    # the diffuse intensity alone (the field the floor interferes with)
    diffuse = out.copy() if floor_addition == "field" else None
    if specular is not None:
        spec = specular.astype(np.float64, copy=True)
        if cfg.speckle:
            slooks = max(int(cfg.specular_looks), 1)
            spec *= rng.gamma(slooks, 1.0 / slooks, size=spec.shape)
        out += spec
    sigma_db = float(cfg.ping_level_jitter_db)
    if sigma_db > 0.0:
        s = sigma_db * np.log(10.0) / 10.0
        jitter = float(np.exp(s * rng.standard_normal() - 0.5 * s * s))
        out *= jitter
        if diffuse is not None:
            diffuse *= jitter
    # Reverberation floor: a shadow is dark, not empty. What the receiver
    # hears where the main lobe is blocked is the same seabed at the same
    # range through the beam's sidelobes, plus the water volume -- so the
    # floor of a shadow follows the ping's own level rather than sitting at
    # the white receiver floor 40+ dB below it. MEASURED on the corpus quay
    # (analysis/wall_contrast): the deep shadow 6-12 m behind a quay face
    # sits 17-25 dB under the seabed at the same slant range, and the
    # 2026-09-18 review measured the simulator's at 30 dB too deep. Speckle
    # is the seabed's own (Exp(1)), not the receiver's white noise.
    # `reverberation_db: 0` restores the pre-2026-09-18 white floor.
    floor = cfg.noise_floor * rng.exponential(1.0, size=out.shape)
    ref = None
    if unoccluded is not None:
        ref = np.asarray(unoccluded, dtype=np.float64)
    if ambient is not None:
        # The culled bin's own would-be response collapses to zero wherever
        # cos_inc clipped at 0 -- a back-facing flank, a cell over land -- so
        # the floor built from it collapses with it. That is the hard black
        # band between a simulated object and its shadow, where the real
        # image runs continuously. The ambient curve (the lit seabed at this
        # range on a flat bottom at this ping's altitude) never collapses;
        # taking the max of the two can only lighten a shadow.
        amb = np.asarray(ambient, dtype=np.float64)
        ref = amb if ref is None else np.maximum(ref, amb)
    cross = None
    if ref is not None and cfg.reverberation_db > 0.0:
        rev = 10.0 ** (-cfg.reverberation_db / 10.0) * ref
        rev_i = rev * rng.exponential(1.0, size=out.shape)
        if diffuse is not None:
            # where the reverberation is the floor, its field meets the echo's
            cross = np.where(rev_i > floor, np.sqrt(diffuse * rev_i), 0.0)
        floor = np.maximum(floor, rev_i)
    out += floor
    if watercolumn_mask.any():
        out[watercolumn_mask] += cfg.watercolumn_noise * \
            rng.exponential(1.0, size=int(watercolumn_mask.sum()))
    if cross is not None:
        out += 2.0 * cross * np.cos(rng.uniform(0.0, 2.0 * np.pi, size=out.shape))
    return out * gain


def ping_dropped(cfg: SonarModelConfig, rng: np.random.Generator) -> bool:
    return bool(rng.uniform() < cfg.dropped_ping_prob)
