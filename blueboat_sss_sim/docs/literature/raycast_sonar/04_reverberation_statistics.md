# 04 — Statistics of reverberation: speckle, K-distribution, correlation, coherent summation

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| AL02 | Abraham & Lyons, "Novel physical interpretations of K-distributed reverberation", *IEEE JOE* 27(4):800–813, 2002 | 10.1109/JOE.2002.804324 | full text (local PDF) |
| LA99 | Lyons & Abraham, "Statistical characterization of high-frequency shallow-water seafloor backscatter", *JASA* 106(3):1307–1315, 1999 | 10.1121/1.428034 | record only — **full text wanted** |
| JP76 | Jakeman & Pusey, "A model for non-Rayleigh sea echo", *IEEE Trans. Antennas Propag.* 24(6):806–814, 1976 | 10.1109/TAP.1976.1141451 | record only |
| War81 | Ward, "Compound representation of high resolution sea clutter", *Electron. Lett.* 17(16):561–563, 1981 | 10.1049/el:19810394 | record only |
| Goo20 | Goodman, *Speckle Phenomena in Optics: Theory and Applications*, 2nd ed., SPIE, 2020 | 10.1117/3.2548484 | record only |
| JR07 | Jackson & Richardson 2007, ch. 16 "Backscatter statistics" (16.2 Gaussian; 16.3 K, Weibull, Rayleigh mixture, Crowther, McDaniel) | 10.1007/978-0-387-36945-7 | full text (local) |
| Brown17 | Brown, Johnson, Olson 2017 (note 01): point-based model of the incoherent field | 10.1121/1.4976584 | record only |

## What they establish

* **Fully developed speckle** (Goo20; JR07 §16.2). Take the sum of many independent scatterers with
  uniform random phase in one resolution cell. Its field is circular complex Gaussian, so the amplitude
  is Rayleigh and the intensity exponential (Exp(1) about the mean; CV = 1 single look). It needs
  "many" scatterers per cell. Below ~10 the intensity PDF departs from exponential, and it is heavier
  tailed when amplitudes vary.
* **K-distribution** (JP76, War81; JR07 §16.3.3). It is the compound of a Gamma-distributed local mean
  ("texture", shape α) and Exp speckle. α → ∞ gives Rayleigh back.
* **Physical origin of K (AL02).** The K-distributed matched-filter envelope arises from a **finite**
  number of scatterers with exponentially distributed sizes, or from homogeneous patches of exponential
  size dominated by interface scattering. The shape parameter is proportional to the number of
  scatterers or patches in the resolution cell, hence to the beamwidth and bandwidth (AL02's own data
  confirm α ∝ array beamwidth). So **K is a property of the scene plus the sensor's resolution, not a
  noise to add.**
* **Correlation of the speckle.** Along range it is set by the pulse ⊛ receive-filter response: two
  samples closer than the range resolution share scatterers. Along track it is set by the overlap of
  consecutive beam footprints: consecutive pings re-illuminate the same scatterers. Both follow when the
  scatterers are fixed in the world and the field is summed coherently.
* **Coherent vs incoherent summation.** The mean of `|Σ a_n e^{iφ_n}|²` over random phases is
  `Σ |a_n|²`, the incoherent sum. Coherent summation changes the *realisation* (speckle, interference,
  fringes, glints), not the mean of the diffuse part. For deterministic phase relations it also changes
  the mean: specular paths, Lloyd interference (note 02), PO glints (note 05). At λ = 3.3 mm every
  path-length difference above ~1 mm is a different phase, so the field is summed as complex amplitude.

## Project measurements this sensor is judged against

These come from `raycast_handoff_from_pass4.md` §4, `calibration_results.md` and the memory
`blueboat-corpus-facts`:

| Statistic | Real corpus | Geometric renderer |
|---|---|---|
| range lag-1 / lag-2 / lag-3 correlation | 0.3–0.7 / 0.10–0.17 / 0.03–0.10 | lag-2/3 0.26 / 0.10 (boxcar kernel) |
| track lag-1 correlation | 0.3–0.7 | gated in smoke `[2g]` |
| 1 m block variance / per-bin variance | pool 0.006–0.013, beach 0.004 | 0.001–0.002 |
| per-ping level std | 0.9–1.5 dB (pool), 0.9–1.0 dB (harbour, beach) | 0.57 dB |

The geometric renderer's hand-added Gamma texture (`texture_k_inv_shape`) was **refuted on the field
data** (pass 3 C, pass 4). So K is not a fitted constant for this sensor. If it appears, it must come
from scene content, as AL02 derive.

## Equations used by the ray-cast sensor

* Point-scatterer field in bin `b`: `E_b = Σ_n a_n s_mf(t_b − τ_n) e^{−iωτ_n}`, with
  `E|a_n|² = σ(θ_i, θ_s, φ) ΔA_n · (beam·TL)²` and the phase `φ_n` hashed from the world position
  (note 01). Mean: `⟨I_b⟩ = Σ_n E|a_n|² |s_mf(t_b − τ_n)|²`.
* Density condition (to be fixed by the Phase 5 study): number of scatterers per range-resolution ×
  footprint cell `N_cell ≥ N_min`. The convergence metric must show the per-bin intensity PDF reaching
  Exp(1) (KS distance below the sampling floor) **before** it is compared with data. That way a sparse
  sampling never fakes a K tail.

## How it maps to the renderer

- `scatterers.py`: density and the world-fixed hash;
- `receiver.py`: the coherent sum, `Ping.power` = mean, `Ping.texture` = realisation/mean;
- the Phase 5 convergence study separates Monte-Carlo error, which falls with density at a fixed
  world seed, from speckle, which is the spread across world seeds.

Large-scale texture (the missing block variance) is a **scene** question for this sensor:
- sediment patches (the existing `worldgen/depth/layers`, material maps);
- sub-grid relief;
- a per-ping level mechanism (surface interference, note 08; source-level jitter, note 09).
It is not a statistic to inject.

## Caveats

* AL02 analyse low-frequency towed-array data. The mechanism transfers; the numbers do not.
* LA99 (high-frequency, shallow water) is the directly relevant data set; its full text is wanted for
  the measured α range at high frequency.

## Full-text reading, 2026-11-03 (LA99, owner-supplied PDF)

* 80 kHz backscatter from 14 shallow-water sites around Sardinia and Sicily (including Posidonia
  seagrass and live-shellfish sites):
  - the amplitude statistics were often non-Rayleigh, with heavier tails;
  - K and Rayleigh-mixture models fit best. A two-component Rayleigh mixture usually sufficed, which
    reads as patches;
  - mean scattering strength did not correlate well with bottom type.
* Consequence: non-Rayleigh statistics are expected from **scene patchiness** (two-component
  mixtures). That supports producing them through scene content, never through sampling. The
  sensor's Gaussian scatterer factor (Brown17) makes sampling-induced tails impossible.

