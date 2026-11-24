# 08 — Sea surface, bubbles, and water-column volume scattering

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| Eck53 | Eckart, "The scattering of sound from the sea surface", *JASA* 25(3):566–570, 1953 | 10.1121/1.1907123 | record (pass 4); `waveguide.eckart_c` |
| Tho88 | Thorsos 1988 (Kirchhoff validity) | 10.1121/1.396188 | record |
| McD93 | McDaniel, "Sea surface reverberation: a review", *JASA* 94(4):1905–1922, 1993 | 10.1121/1.407514 | record only — **full text wanted** |
| Med77 | Medwin, "Counting bubbles acoustically: a review", *Ultrasonics* 15(1):7–13, 1977 | 10.1016/0041-624X(77)90005-1 | record only |
| CP89 | Commander & Prosperetti, "Linear pressure waves in bubbly liquids: comparison between theory and experiments", *JASA* 85(2):732–746, 1989 | 10.1121/1.397599 | record only |
| TVF94 | Trevorrow, Vagle, Farmer, "Acoustical measurements of microbubbles within ship wakes", *JASA* 95(4):1922–1930, 1994 | 10.1121/1.408706 | record only — **full text wanted** |
| Love77 | Love, "Target strength of an individual fish at any aspect", *JASA* 62(6):1397–1403, 1977 | 10.1121/1.381672 | record only |
| Ric96 | Richards et al. 1996 (suspended particles), note 07 | 10.1121/1.415991 | record |
| MC98 | Medwin & Clay, *Fundamentals of Acoustical Oceanography*, Academic Press, 1998 — "Bubbles" ch. (10.1016/b978-012487570-8/50010-6) and "Waveguides" ch. (10.1016/b978-012487570-8/50013-1) | chapters as listed | record only |

## What they establish

* **Coherent reflection from a rough surface** (Eck53): the coherent reflection coefficient is
  `R_coh = R₀ · exp(−2 k² σ² sin² ψ)` in amplitude, `C = exp(−(2kσ sin ψ)²)` in power, for Gaussian
  heights of RMS σ at grazing angle ψ. It is already implemented as `waveguide.eckart_c` (pass 4). The
  air–water surface has `R₀ ≈ −1` (pressure release). The incoherent remainder `1 − C` scatters around
  the specular direction (Kirchhoff, valid for correlation length `kl ≳ 6`, Tho88; at 450 kHz
  `l ≳ 3 mm`, true for gravity waves and for most capillary waves).
* **Surface reverberation** (McD93, record): backscatter from the surface at high frequency comes from
  roughness (Bragg-scale capillary waves) and, in wind, from near-surface bubble layers. The latter
  dominate at low grazing angles in higher winds.
* **Bubbles** (Med77, CP89, MC98). Resonant bubbles scatter and attenuate strongly. At 450 kHz the
  resonant radius is `a_r ≈ 3.26 / f [Hz] · √(1 + 0.1 z)` m ≈ 7 µm near the surface (Minnaert,
  standard). CP89 give the dispersion and attenuation of a bubbly mixture. Ship wakes (TVF94) hold
  persistent microbubble clouds measurable acoustically for minutes.
* **Fish and suspended particles** (Love77, Ric96): discrete volume scatterers and added attenuation.

## Numbers for our geometry (`phase0_numbers.txt`; transducer depth 0.22 m)

Coherent fraction of the surface leg to a seabed point:

| Surface | 2 m range, 1.5 m deep | 10 m range | 20 m range | pool, 5 m, 0.9 m deep |
|---|---|---|---|---|
| calm pool, σ 0.3 mm (pass 4) | 0.58 | 0.96 | 0.99 | 0.94 |
| wind ripple, σ 2 mm | 0.00 | 0.20 | 0.66 | 0.07 |
| waves, Hs 0.1 m (σ 25 mm) | 0 | 0 | 0 | 0 |

So Lloyd fringes and a coherent image comb exist **only on calm water** (the pool). In waves the
surface paths are incoherent, with their energy redirected by the slope distribution. The geometry of
a wave-modulated surface also moves the specular points; the sea module (`sea/`) gives the surface
`η(x, y, t)` as a pure function of seed, time and position.

Beam vs surface: with the 20° tilt and the 50° height, the upper −3 dB edge is 5° *above* horizontal,
so the surface is inside the main lobe at every range. Surface backscatter is therefore a real
contributor in wind or waves, and negligible on a calm pool.

## Decisions (numbers in `raycast_design.md` §3)

* Calm or rough flat surface: **include**. It is the `z = 0` plane in the mesh, with `R₀ = −1`, the
  Eckart coherent part on specular legs, and incoherent Kirchhoff scattering of `1 − C` (bistatic
  Gaussian-slope law, JR07 13.62 form with the surface's slope variance).
* Wave-modulated surface geometry from `sea/`: **include-optional**. It needs the active sea state in
  the sonar node (additive subscription to `/sim/sea_state`); off by default.
* Surface backscatter law (Bragg/bubble): **include-optional** pending McD93; the calm default is ~0.
* Wake / thruster bubble curtain: **include-optional, off**. The fan plane sits at the transducers,
  0.345 m aft of the hull centre, forward of the thrusters, so the boat's own wake lies behind the
  ping plane in straight runs. It can enter the beam in turns or reversing. Needs TVF94 and a corpus
  check: near-range clutter correlated with turns.
* Fish / particles / volume reverberation: **include as a uniform volume scattering strength `S_v`**
  per site, so the water column before the FBR is physical. The corpus measures that level (the
  geometric `watercolumn_noise`, 25.3 dB reported). `S_v` is fitted inside literature bounds; a primary
  source for coastal high-frequency `S_v` bounds is still needed.

## How it maps to the renderer

`scene_mesh.py` adds the surface plane (and optional wave surface); `scattering.py` the surface laws;
`propagation.py`/`scatterers.py` the volume scatterers, as random points in the water volume within the
beam at density `S_v`, hashed to the world like the seabed scatterers.

## Full-text reading, 2026-11-03 (McD93, TVF94; owner-supplied PDFs)

* **McD93** reviews sea-surface reverberation **at 3–60 kHz**: rough-surface scattering dominates at
  moderate to high grazing angles, resonant subsurface microbubbles at low grazing angles.
  Measurements under similar conditions disagree by up to an order of magnitude, and disparately high
  levels occur in coastal areas.
  - No validated law at 450 kHz.
  - Decision: surface backscatter, when enabled, uses the same small-slope roughness physics with a
    wave spectrum from `sea/` (calm: negligible). The bubble term stays out pending a high-frequency
    source.
* **TVF94** measured ship-wake microbubbles at 28–400 kHz (upward sonars at 25 m depth, under three
  oceanographic vessels):
  - volume backscatter M_v up to **≈ 0.3 m⁻¹** in the near-surface wake core at 120–200 kHz, roughly
    constant over the top 5–6 m;
  - wakes up to 66 m wide (ship at 10 kn), 7–12 m deep, persisting **≈ 7.5 min**;
  - decay by gas diffusion.
  These are large-ship values: an upper bound, not a BlueBoat value. The wake term stays optional;
  its strength for a small electric USV is unknown, so a corpus check for turn-correlated near-range
  clutter comes first.

