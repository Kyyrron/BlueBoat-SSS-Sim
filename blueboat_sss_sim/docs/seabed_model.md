# Seabed model — the shapes and textures the depth & seabed editor authors

*What real harbour, river, canal and beach floors look like, which
literature the World Builder's depth presets, layer kinds and recipes rest
on, and how the layer editor turns those shapes into the rasters the
simulator consumes. Companion to `configuration_guide.md` ("depth",
"terrain") and `developer_guide.md` ("Add a seabed layer kind"). Every
DOI in §8 was resolved through Crossref on 2026-09-09; standards and grey
literature are cited by the issuing agency's own page.*

## Table of contents

1. [Why a model, and what it is not](#1-why-a-model-and-what-it-is-not)
2. [Harbours, fishing ports, marinas, quay aprons](#2-harbours-fishing-ports-marinas-quay-aprons)
3. [Rivers and estuarine channels](#3-rivers-and-estuarine-channels)
4. [Canals](#4-canals)
5. [Beaches, shoreface, tidal flats, rocky shores](#5-beaches-shoreface-tidal-flats-rocky-shores)
6. [Parameter table](#6-parameter-table)
7. [The editing model](#7-the-editing-model)
   - 7.1 [Measured seabed from a recording: reflectivity and relief](#71-measured-seabed-from-a-recording-reflectivity-and-relief)
8. [References](#8-references)

---

## 1. Why a model, and what it is not

The simulator's headline comparison runs over synthetic seabeds, so the
seabeds must be defensible as *plausible instances of the survey
environments*, not just pleasant-looking rasters. This document is the
literature behind the shapes: it says which patterns occur, at what size,
and what they look like on a 300–900 kHz side-scan image. It is the
grounding for

* the depth presets (`worldgen/depth/presets.py`, `DEPTH_PRESETS`),
* the layer kinds of the editor (`worldgen/depth/layers/`) and their
  parameter defaults and ranges,
* the recipes (`config/seabed_recipes.yaml`).

Two caveats bound it. First, the field corpus the sonar model is fitted
against (Shiraishi-jima harbour, 6–9 m altitude, its sediment never
sampled; a beach site over known sand at 0.9–1.7 m) pins sand alone —
`sonar_model.md` A10 — so nothing here calibrates the *acoustic* response
of any other material; the five seabed materials remain
relative (`materials.py`). Second, the renderer's whole material spread is
about 3.7 dB against an 18 dB range law (`.claude/TODO.md`, known
limitations), so material layers act on the image mostly through the
micro-relief and shadows they carry until that is re-measured over a known
bottom.

## 2. Harbours, fishing ports, marinas, quay aprons

**The basin floor is a designed plane.** A basin or channel depth is the
design vessel's maximum draft plus a keel clearance. The Fisheries
Agency's design reference sets the clearance at **1.0 m** (at least 1.0 m
over hard ground), allows **two thirds of the fishing-limit wave height**
when small craft must enter in rough weather, plans channel depths in
**0.5 m steps**, and splits small-craft facilities at **−3.0 m** [24].
OCDI's Part III applies the same draft-plus-clearance rule to commercial
ports [25]. Within the project's operating bands (marina 2–6 m, commercial
basin 7–15 m, fishing port between) the floor is therefore a flat plane at
a round half-metre value — the `basin` layer's `floor_depth`.

**The plane is not flat at the decimetre scale.** Channels and basins are
dredged with an *advance maintenance* increment of **0.6–0.9 m** and a
*dredging tolerance* of **0.3–0.9 m** beyond the design depth [26,
§V-5-3h,i]; the design depth is a minimum, and the actual floor sits
0.3–1.5 m below it with the relief of the dredge passes. Design width is
measured at the *bottom* of the side slopes [26], so a dredged pocket has a
sloped rim of its own width beyond the flat floor (`overdredge_m`,
`relief_m`, `relief_spacing_m`, `rim_slope`). No verified source gives
slope by soil type; the rim slope keeps the range the historical preset
used (1:1.5–1:10, 1:3 default) without a citation.

**Berth scour.** Seven years of biannual bathymetry in a low-energy
Barcelona basin show propeller-jet scour whose **maximum depth is of the
order of the propeller diameter**, most of it formed within the first six
months of berthing [29]; propeller wash at quay walls is the mechanism
[28]; PIANC WG 180 frames the design choice between allowing scour and
protecting the toe with rip-rap or mattresses [27]. Hence the `hole`
layer (berth scour 0.5–3 m deep, a few diameters long along the berth,
optionally with a coarse lag) and the `material_band` belt (a rubble apron
along a protected quay).

**Siltation.** Fines enter a basin by tidal filling, the horizontal
exchange eddy at the mouth and density currents, and settle where the
exchange flow decelerates — the inner corners and the lee of the entrance
[30]. The `siltation` layer is the inward shoaling and inward fining from
a drawn entrance line; the legacy preset's edge ramp is a special case.

**Anthropogenic micro-relief.** Dredge furrows are physical marks that
persist for years (the macrobenthos was still perturbed six years after
dredging stopped) [31]; anchor and chain scour cut arcs and pits wherever
vessels swing [32]. The `furrows` layer makes the parallel lineations
(metre-scale spacing, decimetre depth, along the dredge run).

**Texture and its sonar appearance.** Backscatter strength orders mud <
sand < gravel < rock, roughly 10–15 dB from mud to rock at hundreds of kHz
and the 20–30° grazing angles a hull-mounted 450 kHz side-scan works at
[33, 34]; the pixel statistics are K-distributed with a shape parameter
set by the facies patch size relative to the resolution cell [35]; the
interpretation vocabulary — dark smooth mud, bright grainy gravel, rock
with hard shadows, lineations — is Blondel's and Lurton's [36, 37]. Hence
material **gradients with a transition width** (`material_gradient`) and
**patch fields with a characteristic length** (`material_patches`), and
dithered rather than ruled boundaries.

## 3. Rivers and estuarine channels

**Planform.** Meander wavelength is about 10–14 channel widths and bend
radius 2–3 widths [13]; pools recur every **5–7 widths** [14]. At a bend
the thalweg hugs the outer bank — closest **after the apex** in an
isolated bend, **before the apex** in successive bends — the deepest pool
forms before the apex, and the transverse bed slope is steepest where the
thalweg is nearest the outer bank [16]; the point bar on the inner bank
steers the secondary flow [15]. The `channel` layer therefore extrudes a
cross-section template along a drawn centreline in curvilinear (s, n)
coordinates, its thalweg offset following the *lagged signed curvature*
(`curvature_lag_m`, one width by default), with pools phase-locked to the
bends and a periodic pool–riffle term for straight reaches.

**Cross-section.** Trapezoid or parabola on straights, asymmetric at
bends; alluvial width-to-depth ratios of tens [13]. Templates
`trapezoid`, `parabola`, `asymmetric`.

**Bedforms.** Ripples have wavelengths below 0.6 m; dunes are small
(0.6–5 m), medium (5–10 m), large (10–100 m) or very large (> 100 m), with
height ≈ 0.0677 λ^0.81 [19]; van Rijn's predictor gives Δ/h =
0.11 (D50/h)^0.3 (1 − e^(−0.5T))(25 − T) and λ ≈ 7.3 h [20]. The `dunes`
layer defaults to those laws (height from the wavelength when left at 0).

**Local scour.** Pier scour reaches about 2.4 pier widths,
y_s/a = 2.0 K₁K₂K₃ (y₁/a)^0.35 Fr^0.43 [21]; the `hole` layer at a pile.

**Interpolating soundings in a channel.** Isotropic interpolation smears
the thalweg; a channel must be interpolated in flow-oriented coordinates
[17, 18] — the reason the channel template works in (s, n) and why a
positioned source can be re-interpolated along a drawn centreline in a
later update (`.claude/TODO.md`).

**Texture.** Coarse lag in the thalweg and on riffles, fines on bars and
in slack water [13, 15]: the channel layer can write material as well as
height (`lag_material`).

## 4. Canals

Engineered trapezoids whose bottom width and depth follow the CEMT design
vessel plus keel clearance [22]; on the older navigations the full depth
holds only in the central channel, the bed slopes up toward the banks,
silt settles at the base of those slopes and is stirred by boat traffic,
and marina basins are built to **1.5 m** [23]. The `canal` preset and the
`channel` layer's `trapezoid` template with a `material_region` of mud
cover this. The project's own access filter rules out canals narrower than
25–30 m for this platform (`target_families_and_surrogates.md` §1.1); the
template is kept for wide urban canals and as an object-type source.

## 5. Beaches, shoreface, tidal flats, rocky shores

**Equilibrium profile.** h = A x^(2/3) from the shoreline [1, 2], with
A = 0.067 w^0.44 (fall velocity w in cm/s): ≈ 0.10 m^(1/3) for 0.2 mm sand,
≈ 0.20 for coarse sand [2, 3]. The profile flattens toward the closure
depth d_l ≈ 2.28 H_e − 68.5 H_e²/(g T_e²) [4]. The `gradient` layer's
`dean` profile from a drawn shoreline generalises the `beach_profile`
preset to any shoreline (`depth_end` is the closure cap); on a straight
shoreline the two are identical to machine precision (smoke test [12]).

**Beach state and bars.** Ω = H_b/(w_s T) separates reflective (Ω < 1:
steep, a step at the base of the swash zone, no bar), intermediate (1–6:
rhythmic bar-and-beach, transverse bar and rip, low-tide terrace) and
dissipative (> 6: multiple bars) beaches [5]; a barred profile is the
Dean profile plus a parametric bar perturbation [6]; rip channels cut the
bar at spacings of order 10²–10³ m [7]. The `bar` layer (crest, trough,
rips) and the `shelf` layer (swash step) follow.

**Sheltered beaches** — the Seto Inland Sea case — are steep, narrow,
normally bar-less, and carry morphology inherited from rare storms [8]:
the *Sheltered beach* recipe has no bar; *Open beach* has bar and rips.

**Ripples.** Orbital ripples have λ ≈ 0.62 d₀ (wave orbital diameter) and
steepness ≈ 0.17; anorbital ripples λ ≈ 535 D50 [9]; the orbital →
anorbital sequence runs with depth [10]. The `ripples` layer: λ 0.2–3 m
(default 0.5 m; under 0.2 m the 0.1 m world grid aliases it), height an
absolute 0–0.5 m (default 0.06 m; typically 0.1–0.17 λ), crests
shore-parallel.

**Tidal flats.** Gradients of order 10⁻³, convex-up when accreting and
concave-up when eroding, dissected by creeks [11]: a near-zero `linear`
gradient plus a `channel` creek (the *Tidal flat* recipe).

**Seagrass** beds are patchy at the 1–10 m scale and resolvable by
high-resolution sonar [12]: `material_patches` with a patch length.

**Dry ground** (the `land` layer, 2026-09-18). A survey area is rarely all
water: a beach world has a beach in it, a harbour world has an apron, and a
bank or an island sits in the middle of plenty of them. The layer lifts a
drawn area to a height above the still-water plane (`height_m`, default
+0.3 m) and carries the seabed up to it over a shore face (`shore_m`), so
the renderer's own dry-cell rule (NC #13: a cell at or above the transducer
returns nothing, and the horizon cull hides what lies beyond the waterline)
produces the waterline without a special case.

What makes it usable rather than a polygon is `round_m`: the outline is
**opened and then closed** with a disc of that radius (a morphological
radius filter, [50] Serra 1982 ch. 2, computed here on the Euclidean
distance transform), so nothing along the water's edge is sharper than the
disc. A shoreline drawn as six clicked vertices reads as a shoreline
instead of as a hexagon, and a thin spike narrower than the disc simply
disappears. The default 3 m is about the radius a metre-scale sonar can
resolve anyway.

The older route — a `dredged basin` with a floor above the waterline —
still works and is sometimes what you want (a hard-edged quay apron, a
dredged pond); its `floor_depth` accepts negative values, and because
"deepen" (its default blend, which takes the minimum) cannot express a
floor *above* the bed, such a basin composites normally instead. A beach
face is what `land` is for; a rectangular concrete edge is what the basin
is for.

## 6. Parameter table

| Primitive | Parameter | Default / range | Source |
|---|---|---|---|
| basin | floor depth | draft + 1.0 m, rounded to 0.5 m; 2–15 m | [24, 25] |
| basin | over-dredge relief | 0.3–1.5 m below design, dredge-pass texture | [26] |
| basin | rim slope | 1:3 default, 1:1–1:12 | historical preset range (unsourced) |
| hole (berth) | depth / length | ≈ propeller diameter (0.5–3 m) / 2–5 D | [29, 28, 27] |
| hole (pier) | depth | 1–2.4 × pier width | [21] |
| siltation | from the entrance inward | shoal 0.3–1.5 m, mud fining inward | [30] |
| furrows | spacing / depth | metre-scale / decimetre-scale, along the run | [31] |
| channel | thalweg offset | ∝ lagged signed curvature; pool before the apex | [16, 15] |
| channel | pool–riffle spacing | 5–7 W | [14] |
| channel | meander λ / R | 10–14 W / 2–3 W | [13] |
| dunes | λ / H | 7.3 h / 0.0677 λ^0.81 | [20, 19] |
| ripples | λ / H | 0.62 d₀ or 535 D50 / 0.1–0.17 λ | [9, 10] |
| gradient (dean) | A | 0.067 w^0.44 ≈ 0.10 (0.2 mm) – 0.20 (coarse) | [2, 3] |
| gradient (dean) | closure depth | 2.28 H_e − 68.5 H_e²/(gT_e²) | [4] |
| bar / rip | bar form / rip spacing | Dean + bar perturbation / 10²–10³ m | [6, 7] |
| canal | depth / toe silt | draft + clearance; marina 1.5 m | [22, 23] |
| tidal flat | gradient | ~10⁻³ | [11] |
| material gradient | backscatter order | mud < sand < gravel < rock, 10–15 dB | [33, 34] |
| material patches | patch length | 1–10 m (seagrass), K-statistics | [12, 35] |

## 7. The editing model

The screen is a non-destructive **layer stack** compiled to the same two
rasters the engine always consumed (`worldgen.depth.compose`):

* the **source** at the bottom — a preset, a depth file or a recording,
  exactly as before, consuming the terrain RNG stream as before, so a
  stack with nothing above it is bit-identical to the legacy path;
* the base **depth brush** strokes and the **seabed texture** form with its
  material brush (the legacy `paint`, `material_paint`, `texture`);
* any number of **layers** (`worldgen.depth.layers`): gradients,
  landforms, relief, adjustments, material layers, a reflectivity gain,
  brush-stroke layers with an operation. Each layer has a *geometry* drawn
  on the map, a *mask* (`worldgen.depth.masks`: rectangle, ellipse,
  polygon, band along a line, distance to the walls, depth range of the
  bed below, gradient, mask brush, survey coverage — each with a C1
  feather that never crosses a wall, an `invert`, and `union` /
  `intersect` / `subtract` when several are listed), an *opacity* and a
  *blend* mode: `normal` (toward the layer's field), `add`, `deepen` (the
  deeper wins — dredging), `shoal` (the shallower wins — deposition).
  Layers write one or more *channels*: height, material (stamped where the
  weight is ≥ 0.5 for a hard edge, or where it beats a seeded uniform draw
  for a dithered edge) and a multiplicative reflectivity gain. Every
  layer above the source draws from its own stream
  `default_rng([seed, 2, layer.seed])`, so editing one never moves
  another's texture or the objects.

At build time (`worldgen.depth.bake`) the stack is composed on the build
grid in the world frame and written as `depth_baked.tif` — a float32
north-up EPSG:4326 GeoTIFF whose pixel lattice *is* the grid, ingested
losslessly through the ordinary `depth.mode: file` path — and
`material_map.npz`, consumed through the one additive world-YAML key
`terrain.materials.map`. Downstream, a layered world is a file-mode
world; only `builder_state.yaml` (`depth.layers`) knows it was authored in
layers. The Screen-3 preview composes the same stack in memory on its
coarse grid; smoke test [12] pins preview and bake to float32 rounding
and the ten presets and the paint layer to bit-identity; [11] pins every
layer kind to a one-sentence hover help and a non-empty reference (the
references themselves are listed in §8).

### 7.1 Measured seabed from a recording: reflectivity and relief

A recording carries more than the depth under the boat. Two products of
the same per-bin inversion (`analysis/backscatter.py`, 2026-09-16) bring
what the swath saw into the world:

**Reflectivity.** Every profile bin past the first bottom return is
brought to the calibration gain (the measured 20 log₁₀ ladder), the
renderer's own flat-floor model at reflectivity 1 — Lambert's law
`ρ sinᵖθ_g` [39] on the beam pattern, the range law and the specular
lobe of `sonar_model.md` §3 — is removed, the shadow bins (at the
receiver floor) and the near-nadir cone (3.5 × `specular_width_deg`,
where the data rise above Lambert [33 §12.1.1]) are left out, and the
residual slope against `log₁₀ sin θ_g` is taken out — Coiras et al.'s
beam-pattern regularisation [45 eq. 11] in its one-parameter form, which
a material patch confined to one angle band cannot bend (a per-degree
median ate −0.7 dB of a 2.6 dB mud contrast on the smoke's test). The
cell mean of the dB values, corrected by the single-look speckle bias
`10 log₁₀e (ψ(L) − ln L)` (−2.51 dB), is Lambert's coefficient in the
renderer's units: the simulator renders it back. The
`reflectivity_survey` layer applies it as a gain about the survey median
(rendered as the `reference` material) and classes cells by the
contrasts the literature measures between sediments at hundreds of kHz:
Jackson & Richardson fit `10 log₁₀ μ` at mud −19.7, sand −20.2, gravel
−11.7 and rock −5.6 dB over the compiled measurements below 60° grazing
[33 §12.1.1, Figs 12.2–12.5], with frequency slopes of +3 / +6–8 / +3 /
0 dB per decade [33 §12.1.2]; carried to 450 kHz from that ~30 kHz
compilation, sand sits ~5 dB above mud, gravel ~3 dB and rock 5–8 dB
above sand — the ordering and size the ESAB multifrequency model measures
at 290–440 kHz [41: sandy mud 3–5 dB below sand in the oblique regime,
bedrock and maërl highest]. Hence the defaults mud ≤ −3.5 dB, gravel ≥
+2.5 dB, rocks ≥ +5.5 dB about the median. Two caveats travel with the
layer: the within-class spread of scattering strength is ±5–8 dB ("sediment
name is not a good predictor" — a hard/soft separation is what is
defensible [33 §12.1.1]), and the absolute level inherits the
calibration's uncertainty (`sonar_model.md` §6; the ±6 dB auto-gain caveat
was retired on 2026-09-18, when the two-altitude fit put the seabed level
within 0.6 dB at both altitudes). Within one recording the relative map is
good to the 1–2 dB that averaging ~10³ single-look bins per cell leaves.
The rendered reflectivity ceiling opens to `REFLECTIVITY_MAX` (4.0, +6 dB)
only under a gain map, so every world without one clips at 1.0 as before.

**How it is placed on the grid** (2026-09-18). The inversion works on 1 m
cells and the world is rendered at 0.10 m, so the map has to be resampled
up by a factor of ten. It used to be resampled **nearest-neighbour**,
which is what the 2026-09-18 report *"the reflectivity from svlog is too
sharp"* was looking at: 1 m squares of constant reflectivity with hard
edges between them, a raster artefact rather than a seabed. It is now
**bilinear with a coverage-weighted (normalised) convolution**, so an
unmeasured cell cannot bleed into a measured one, and the survey layer's
own smoothing and feather defaults went from 2 m / 2 m to **3 m / 4 m** —
which is the scale the literature actually supports, given a within-class
spread of ±5–8 dB: a sediment boundary resolved to a metre is a claim the
measurement cannot carry.

**The static table and its literature counterpart.** The shipped material
table (`worldgen/materials.py`) spans 5.9 dB from seagrass to rock, where
the numbers above span about 15. Those values are judgement, kept because
every existing world was built with them, and widening them silently would
change every world's absolute level. `config/materials_literature.yaml`
(2026-09-18) carries the J&R contrasts as an **override** instead — mud
−5, sand 0, gravel +3, rock +6.5 dB, anchored on sand = 0.20 so rock fits
under the renderer's [0, 1] ceiling — together with the 4.4 dB level offset
that anchoring implies. A world that wants literature contrasts pastes it
into its `materials:` section and moves `calibration_db_offset` with it;
a world that wants *this* site's contrasts surveys them instead, which is
what the layer above is for.

**Relief.** On a Lambertian floor brightness is slope; integrating it
across the swath from the FBR depth is the classic shape-from-shading of
side-scan sonar [43, 44, 45], with the altimeter as boundary condition
[46, 47]. `analysis/shading.py` does it in one deterministic pass on the
inverted bins: average against the speckle first (a single-look slope
integrates into a random walk), split reflectivity from slope by scale
(the reflectivity is the linear mean over a 20 m window — an 8 m window
swallowed the smoke's 10 m ripples: correlation 0.58 against 0.91 — and a
dB mean sat below the linear one and drifted the integral by a metre),
invert Lambert with the flat-floor grazing angle, integrate outward and
re-solve the ground range once [43], and keep only the relief about the
along-ray trend — the along-track direction is unobservable from one
line, and the spline through the FBR soundings stays the low-frequency
surface (`worldgen/depth/smooth.py` fuses the two). The shadow bound of
[43, 45] (a shadow run implies the surface lies below the ray) is flagged,
not enforced. Measured on the simulator's own 10 m / 0.4 m ripples under
speckle: correlation 0.87 with the truth, 0.09 m error on a 0.15 m RMS
relief, 0.05 m on a flat floor. The amplitude carries the angular law's
uncertainty: `lambert_exponent` is measured at 1.0 over the beach sand
(§6 of `sonar_model.md`, 2026-09-18), not for other bottoms; Ran et al.
calibrate their shape-from-shading DEM by regression onto the echo
sounder [48 §2.2.3], and `relief_gain: 0` does the same on the reliefs —
the shading relief against the relief the FBR of the other lines measured
where they cross the swath (on the beach corpus that gain is 0.1–0.3: at
3–5° grazing the shading over-reads, and the tie lines say so).

## 8. References

Peer-reviewed and book sources (DOIs resolved through Crossref, 2026-09-09):

1. Bruun, P. (1954). *Coast erosion and the development of beach profiles*. Beach Erosion Board Technical Memorandum 44, U.S. Army Corps of Engineers. (No DOI; https://erdc-library.erdc.dren.mil/jspui/handle/11681/3426)
2. Dean, R. G. (1991). Equilibrium beach profiles: characteristics and applications. *Journal of Coastal Research*, 7(1), 53–84. (No DOI; https://journals.flvc.org/jcr/article/view/78405)
3. Dean, R. G., & Dalrymple, R. A. (2001). *Coastal Processes with Engineering Applications*. Cambridge University Press. https://doi.org/10.1017/CBO9780511754500
4. Hallermeier, R. J. (1980). A profile zonation for seasonal sand beaches from wave climate. *Coastal Engineering*, 4, 253–277. https://doi.org/10.1016/0378-3839(80)90022-8
5. Wright, L. D., & Short, A. D. (1984). Morphodynamic variability of surf zones and beaches: a synthesis. *Marine Geology*, 56(1–4), 93–118. https://doi.org/10.1016/0025-3227(84)90008-2
6. Holman, R. A., Lalejini, D. M., Edwards, K., & Veeramony, J. (2014). A parametric model for barred equilibrium beach profiles. *Coastal Engineering*, 90, 85–94. https://doi.org/10.1016/j.coastaleng.2014.03.005
7. Short, A. D. (1985). Rip-current type, spacing and persistence, Narrabeen Beach, Australia. *Marine Geology*, 65(1–2), 47–71. https://doi.org/10.1016/0025-3227(85)90046-5
8. Jackson, N. L., Nordstrom, K. F., Eliot, I., & Masselink, G. (2002). 'Low energy' sandy beaches in marine and estuarine environments: a review. *Geomorphology*, 48(1–3), 147–162. https://doi.org/10.1016/S0169-555X(02)00179-4
9. Wiberg, P. L., & Harris, C. K. (1994). Ripple geometry in wave-dominated environments. *Journal of Geophysical Research: Oceans*, 99(C1), 775–789. https://doi.org/10.1029/93JC02726
10. Clifton, H. E., & Dingler, J. R. (1984). Wave-formed structures and paleoenvironmental reconstruction. *Marine Geology*, 60(1–4), 165–198. https://doi.org/10.1016/0025-3227(84)90149-X
11. Friedrichs, C. T. (2011). Tidal flat morphodynamics: a synthesis. In *Treatise on Estuarine and Coastal Science* (vol. 3, pp. 137–170). Elsevier. https://doi.org/10.1016/B978-0-12-374711-2.00307-7
12. Lefebvre, A., Thompson, C. E. L., Collins, K. J., & Amos, C. L. (2009). Use of a high-resolution profiling sonar and a towed video camera to map a *Zostera marina* bed, Solent, UK. *Estuarine, Coastal and Shelf Science*, 82(2), 323–334. https://doi.org/10.1016/j.ecss.2009.01.027
13. Leopold, L. B., & Wolman, M. G. (1957). *River channel patterns: braided, meandering, and straight*. U.S. Geological Survey Professional Paper 282-B. https://doi.org/10.3133/pp282B
14. Keller, E. A., & Melhorn, W. N. (1978). Rhythmic spacing and origin of pools and riffles. *Geological Society of America Bulletin*, 89(5), 723–730. https://doi.org/10.1130/0016-7606(1978)89%3C723:RSAOOP%3E2.0.CO;2 (this DOI ends in `;2`, which `check_dois.py`'s pattern truncates; resolved by hand through Crossref on 2026-09-09)
15. Dietrich, W. E., & Smith, J. D. (1983). Influence of the point bar on flow through curved channels. *Water Resources Research*, 19(5), 1173–1192. https://doi.org/10.1029/WR019i005p01173
16. Ahmed, S., Ghosh, S., & Barbhuiya, A. K. (2024). Characteristics of thalweg profile and cross-sectional geometry along alluvial river bends. *ISH Journal of Hydraulic Engineering*, 30(5), 547–558. https://doi.org/10.1080/09715010.2024.2383187
17. Merwade, V. M., Maidment, D. R., & Goff, J. A. (2006). Anisotropic considerations while interpolating river channel bathymetry. *Journal of Hydrology*, 331(3–4), 731–741. https://doi.org/10.1016/j.jhydrol.2006.06.018
18. Merwade, V., Cook, A., & Coonrod, J. (2008). GIS techniques for creating river terrain models for hydrodynamic modeling and flood inundation mapping. *Environmental Modelling & Software*, 23(10–11), 1300–1311. https://doi.org/10.1016/j.envsoft.2008.03.005
19. Ashley, G. M. (1990). Classification of large-scale subaqueous bedforms: a new look at an old problem. *Journal of Sedimentary Petrology*, 60(1), 160–172. https://doi.org/10.2110/jsr.60.160
20. van Rijn, L. C. (1984). Sediment transport, part III: bed forms and alluvial roughness. *Journal of Hydraulic Engineering*, 110(12), 1733–1754. https://doi.org/10.1061/(ASCE)0733-9429(1984)110:12(1733)
21. Arneson, L. A., Zevenbergen, L. W., Lagasse, P. F., & Clopper, P. E. (2012). *Evaluating Scour at Bridges*, 5th ed. (HEC-18, FHWA-HIF-12-003). Federal Highway Administration. https://www.fhwa.dot.gov/engineering/hydraulics/pubs/hif12003.pdf
22. Rijkswaterstaat (2020). *Richtlijnen Vaarwegen 2020*, 2nd revised ed. https://open.rijkswaterstaat.nl/@198385/richtlijnen-vaarwegen-2020-2e-gewijzigde/
23. Canal & River Trust. *Waterway dimensions* (web, fetched 2026-09-09). https://canalrivertrust.org.uk/boating/go-boating/planning-your-boat-route/waterway-dimensions
24. 水産庁 (Fisheries Agency of Japan) (2023). 漁港・漁場の施設の設計参考図書 (2023年版), 第7編 水域施設 (Design reference for fishing-port and fishing-ground facilities, Part 7 Water-area facilities). https://www.jfa.maff.go.jp/j/gyoko_gyozyo/g_thema/attach/pdf/sekkei_kaitei-101.pdf
25. OCDI — Overseas Coastal Area Development Institute of Japan (2020). *Technical Standards and Commentaries for Port and Harbour Facilities in Japan* (English ed.), Part III Waterways and Basins. https://ocdi.or.jp/en/technical-st_v2020/
26. U.S. Army Corps of Engineers (2003). *Coastal Engineering Manual*, EM 1110-2-1100, Part V, Chapter 5 "Navigation Projects", §V-5-3 (channel depth: advance maintenance 0.6–0.9 m, dredging tolerance 0.3–0.9 m; width at the bottom of the side slopes).
27. PIANC (2015). *Guidelines for protecting berthing structures from scour caused by ships*. MarCom Working Group 180, Report 180. https://www.pianc.org/publication/guidelines-for-protecting-berthing-structures-from-scour-caused-by-ships/
28. Hamill, G. A., Johnston, H. T., & Stewart, D. P. J. (1999). Propeller wash scour near quay walls. *Journal of Waterway, Port, Coastal, and Ocean Engineering*, 125(4), 170–175. https://doi.org/10.1061/(ASCE)0733-950X(1999)125:4(170)
29. Mujal-Colilles, A., Gironella, X., Sánchez-Arcilla, A., Puig Polo, C., & Garcia-León, M. (2017). Erosion caused by propeller jets in a low energy harbour basin. *Journal of Hydraulic Research*, 55(1), 121–128. https://doi.org/10.1080/00221686.2016.1252801
30. Winterwerp, J. C. (2005). Reducing harbor siltation. I: Methodology. *Journal of Waterway, Port, Coastal, and Ocean Engineering*, 131(6), 258–266. https://doi.org/10.1061/(ASCE)0733-950X(2005)131:6(258)
31. Boyd, S. E., Limpenny, D. S., Rees, H. L., & Cooper, K. M. (2005). The effects of marine sand and gravel extraction on the macrobenthos at a commercial dredging site (results 6 years post-dredging). *ICES Journal of Marine Science*, 62(2), 145–162. https://doi.org/10.1016/j.icesjms.2004.11.014
32. Broad, A., Rees, M. J., & Davis, A. R. (2020). Anchor and chain scour as disturbance agents in benthic environments: trends in the literature and charting a course to more sustainable boating and shipping. *Marine Pollution Bulletin*, 161, 111683. https://doi.org/10.1016/j.marpolbul.2020.111683
33. Jackson, D. R., & Richardson, M. D. (2007). *High-Frequency Seafloor Acoustics*. Springer. https://doi.org/10.1007/978-0-387-36945-7
34. Applied Physics Laboratory, University of Washington (1994). *APL-UW High-Frequency Ocean Environmental Acoustic Models Handbook*. Technical Report APL-UW TR 9407. https://apps.dtic.mil/sti/citations/ADB199453
35. Lyons, A. P., & Abraham, D. A. (1999). Statistical characterization of high-frequency shallow-water seafloor backscatter. *Journal of the Acoustical Society of America*, 106(3), 1307–1315. https://doi.org/10.1121/1.428034
36. Lurton, X. (2010). *An Introduction to Underwater Acoustics: Principles and Applications*, 2nd ed. Springer. https://doi.org/10.1007/978-3-642-13835-5
37. Blondel, P. (2009). *The Handbook of Sidescan Sonar*. Springer/Praxis. https://doi.org/10.1007/978-3-540-49886-5
38. Makar, A. (2023). Coastal bathymetric sounding in very shallow water using USV: study of public beach in Gdynia, Poland. *Sensors*, 23(9), 4215. https://doi.org/10.3390/s23094215 (survey-method context, from the 2026-09-01 planning appendix)

Added 2026-09-16 for §7.1 (DOIs resolved through Crossref that day; [33] ch. 2 and 12, [45] and [48] read in full from the PDFs):

39. Mackenzie, K. V. (1961). Bottom reverberation for 530- and 1030-cps sound in deep water. *The Journal of the Acoustical Society of America*, 33(11), 1498–1504. https://doi.org/10.1121/1.1908482
40. Fonseca, L., & Mayer, L. (2007). Remote estimation of surficial seafloor properties through the application Angular Range Analysis to multibeam sonar data. *Marine Geophysical Researches*, 28(2), 119–126. https://doi.org/10.1007/s11001-007-9019-4
41. Fonseca, L., Lurton, X., Fezzani, R., & Roche, M. (2025). A simplified semi-empirical model for multifrequency seafloor backscattering angular response (ESAB). *Frontiers in Remote Sensing*, 6, 1619218. https://doi.org/10.3389/frsen.2025.1619218
42. Hamilton, E. L. (1980). Geoacoustic modeling of the sea floor. *The Journal of the Acoustical Society of America*, 68(5), 1313–1340. https://doi.org/10.1121/1.385100
43. Cuschieri, J. M., & Hebert, M. (1990). Three-dimensional map generation from side-scan sonar images. *Journal of Energy Resources Technology*, 112(2), 96–102. https://doi.org/10.1115/1.2905729
44. Johnson, A. E., & Hebert, M. (1996). Seafloor map generation for autonomous underwater vehicle navigation. *Autonomous Robots*, 3(2–3), 145–168. https://doi.org/10.1007/BF00141152
45. Coiras, E., Petillot, Y., & Lane, D. M. (2007). Multiresolution 3-D reconstruction from side-scan sonar images. *IEEE Transactions on Image Processing*, 16(2), 382–390. https://doi.org/10.1109/TIP.2006.888337
46. Xie, Y., Bore, N., & Folkesson, J. (2022). Sidescan only neural bathymetry from large-scale survey. *Sensors*, 22(14), 5092. https://doi.org/10.3390/s22145092
47. Xie, Y., Bore, N., & Folkesson, J. (2023). Neural network normal estimation and bathymetry reconstruction from sidescan sonar. *IEEE Journal of Oceanic Engineering*, 48(1), 218–232. https://doi.org/10.1109/JOE.2022.3194899
48. Ran, C., Zhang, L., Han, S., Zhang, X., Wang, S., & Zhou, X. (2025). Three-dimensional reconstruction of shallow seabed topographic surface based on fusion of side-scan sonar and echo sounding data. *ISPRS Journal of Photogrammetry and Remote Sensing*, 228, 249–269. https://doi.org/10.1016/j.isprsjprs.2025.07.018
49. Bikonis, K., Moszyński, M., & Łubniewski, Z. (2013). Application of shape from shading technique for side scan sonar images. *Polish Maritime Research*, 20(3), 39–44. https://doi.org/10.2478/pomr-2013-0033

Added 2026-09-18 for §5 (the `land` layer's outline rounding):

50. Serra, J. (1982). *Image Analysis and Mathematical Morphology*. Academic Press. (No DOI; ISBN 0-12-637240-3. Opening followed by closing with a structuring element — here a disc of `round_m`, evaluated through the Euclidean distance transform — is the classical radius filter: it removes every protrusion and every intrusion sharper than the element while leaving the body of the set alone, which is exactly the "a drawn polygon must read as a shoreline" requirement.)

Not verifiable on 2026-09-09 and therefore not cited: Moore (1982) M.S.
thesis (the A–D50 table is taken from [3]); Flemming (1988) (the H–λ law
is cited through [19]); USACE EM 1110-2-1613 (publisher returned 403;
[26] carries the same depth increments); the Fisheries Agency reference's
tonnage-class basin-depth table (not in the fetched chapter overview).
