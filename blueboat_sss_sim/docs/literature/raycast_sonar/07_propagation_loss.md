# 07 — Propagation loss in the water: absorption, spreading, sound speed

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| FG82a | Francois & Garrison, "Sound absorption based on ocean measurements. Part I: Pure water and magnesium sulfate contributions", *JASA* 72(3):896–907, 1982 | 10.1121/1.388170 | record; formula read in JR07 App. B.3 |
| FG82b | Francois & Garrison, "… Part II: Boric acid contribution and equation for total absorption", *JASA* 72(6):1879–1890, 1982 | 10.1121/1.388673 | record; formula read in JR07 App. B.3 |
| AM98 | Ainslie & McColm, "A simplified formula for viscous and chemical absorption in sea water", *JASA* 103(3):1671–1672, 1998 | 10.1121/1.421258 | record only (cross-check formula; text wanted) |
| FS77 | Fisher & Simmons, "Sound absorption in sea water", *JASA* 62(3):558–564, 1977 | 10.1121/1.381574 | record only |
| JR07 | Jackson & Richardson 2007, App. B.2 (sound speed), B.3 (absorption, eqs B.6–B.12) | 10.1007/978-0-387-36945-7 | full text (local) |
| Mac81 | Mackenzie 1981, nine-term sound speed | 10.1121/1.386920 | record |
| BW93 | Bilaniuk & Wong 1993, pure-water sound speed | 10.1121/1.406819 | record |
| OISST | NOAA 0.25° Daily OISST v2.1 (dataset), NOAA NCEI, 2020– ; method: Huang et al., "Improvements of the Daily Optimum Interpolation Sea Surface Temperature (DOISST) Version 2.1", *J. Climate* 34(8):2923–2939, 2021; Reynolds et al., *J. Climate* 20(22):5473–5496, 2007 | dataset 10.25921/RE9P-PT57 (DataCite, verified 2026-11-03); 10.1175/JCLI-D-20-0166.1; 10.1175/2007JCLI1824.1 | data read via ERDDAP 2026-11-03 (records verified) |
| Ric96 | Richards, Heathershaw, Thorne, "The effect of suspended particulate matter on sound attenuation in seawater", *JASA* 100(3):1447–1450, 1996 | 10.1121/1.415991 | record; JR07 App. B.3 summarises it |

## What they establish

* **Absorption, Francois–Garrison** (JR07 B.6–B.12):
  `α [dB/m] = 10⁻³ [A₁P₁f₁f²/(f² + f₁²) + A₂P₂f₂f²/(f² + f₂²) + A₃P₃f²]`.
  The three terms are boric acid, MgSO₄ relaxation and pure-water viscosity, with T [°C], S [ppt],
  z [m], pH and f [kHz]. JR07: "Between 100 and 500 kHz the importance of magnesium sulfate absorption
  decreases, and above 500 kHz absorption is primarily due to water viscosity". Attenuation is 1–3
  orders of magnitude higher at 5–40 ppt than in fresh water. The formula is fitted to open-ocean water
  (S 30–40 ppt) and filtered water; suspended matter adds viscous and scattering losses (Ric96).
* **Spreading.** Spherical per leg, `1/r` in amplitude. This holds for every leg in a homogeneous
  medium. Cylindrical-like range laws in the pool come from the **image sum** (pass 4: the seabed's own
  waveguide), not from a spreading law. In the ray caster they emerge from the paths.
* **Sound speed.** Mackenzie for seawater (the corpus water), Bilaniuk–Wong for fresh water. It sets λ, the bin-to-range mapping
  (the encoder reports `sos_dmps` from `SPEED_OF_SOUND_MPS`) and refraction (note 02).

## The corpus sites' water (owner, 2026-11-03; estimates 2026-11-03)

**Every site was sea water** (the owner: the pool too). The Phase-0 assumption of a fresh, chlorinated
pool, and the "4.6 dB two-way at 20 m between fresh and sea water" that followed from it, are
**withdrawn**.
- **Salinity is unknown** at every site. The value used is standard seawater, **35 ppt** (JR07 App.
  B.3), with a band of ±3 ppt.
- **Temperature is unknown** (the recordings carry GPS and attitude, no water temperature). It is
  estimated per site and date from NOAA's daily OISST v2.1 analysis (Huang et al. 2021, the dataset
  doi:10.25921/RE9P-PT57; read 2026-11-03 through NOAA CoastWatch ERDDAP). The query took the
  nearest ocean cell (10–17 km offshore of the site), at the dataset's 0.25° resolution, so only
  that coarse cell left the machine. The band is **±3 °C**, because enclosed shallow water departs
  from offshore SST. The site positions and the raw cells are kept privately in `~/calib/raycast/`
  (the corpus manifest records no coordinates, and neither does the repository).

| Site and dates | OISST nearest ocean cell | Used | α at S 35, pH 8 [dB/m] | two-way at 20 m | c (J&R B.10) |
|---|---|---|---|---|---|
| harbour_A, 2026-07-21…23 | 28.2 / 28.5 / 28.8 °C | 28.5 ± 3 °C | 0.177 (0.161–0.193) | 7.1 ± 0.65 dB | 1545 ± 10 m/s |
| beach_A, 2026-09-07 | 27.6 °C | 27.5 ± 3 °C | 0.171 (0.156–0.188) | 6.9 ± 0.64 dB | 1542 ± 10 m/s |
| pool_A (outdoor), 2026-10-27…28 | 26.0 / 25.7 °C (preliminary NRT product) | 26.0 ± 3 °C | 0.163 (0.148–0.179) | 6.5 ± 0.62 dB | 1537 ± 10 m/s |

The salinity band moves α by ±0.011 dB/m (≈ ±0.4 dB two-way at 20 m). Neither T nor S is fitted;
Phase 7 reports how every score moves across the bands.

**Consequences.**
- The geometric renderer uses one `absorption_db_per_m: 0.10` for every site ("not separated from
  spreading"). The physical value in the corpus water is 0.16–0.18 dB/m, so 2.5–3 dB two-way more
  loss at 20 m. In the geometric fit that difference is absorbed jointly with TVG and the level.
- **Range scale.** The device maps time to range at its assumed 1500 m/s. Every corpus file reports
  `sos_dmps` = 15000, checked on calibration files of all three sites, 2026-11-03. With water at
  1537–1545 m/s, real images are range-compressed by **2.5–3.0 % (±0.6 %)** relative to the true
  geometry: a wall at 10 m true range appears 25–30 cm closer. The ray-cast sensor reproduces this
  (true speed for the delays, 1500 m/s for the bins). The geometric renderer uses 1500 m/s for both,
  so it does not (recorded in `TODO.md`).
- For reference only: fresh water at the same temperatures would have ≈ 0.04 dB/m (the viscous term
  alone).


## Equations used by the ray-cast sensor

`TL_leg = 20 log₁₀ r + α r` (one way, per leg, α from FG82 at the site's T, S, pH, mean leg depth).
The coherent amplitude factor is `e^{−α r ln10 / 20} / r`. Optional suspended-matter attenuation is a
site term (Ric96), off by default.

## How it maps to the renderer

`propagation.py`: `absorption_fg(f, T, S, z, pH)`, `sound_speed(T, S, z)`; a `site:` block in the
`raycast:` profile section (T, S, pH; optional sound-speed profile).

## Caveats

* AM98 is a cross-check only; F&G is the primary formula, read in JR07.
* F&G at S = 0 reduces to the pure-water term, which is the right limit for a pool (JR07 Fig. B.2).
