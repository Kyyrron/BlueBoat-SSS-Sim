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
9. [Sediment classes and seabed heterogeneity](#9-sediment-classes-and-seabed-heterogeneity-2026-11-15) (after the references: added 2026-11-15)
10. [The object set version 2](#10-the-object-set-version-2)
11. [Structures, litter population, depth domain](#11-structures-litter-population-depth-domain)
12. [The campaign generator](#12-the-campaign-generator)

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
a nominal 5.9 dB (about 3.7 dB measured in an image under the old range law) against an 18 dB range law (`.claude/TODO.md`, known
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
[33, 34]. Real pixel statistics are heavier-tailed than fully developed
speckle (K-distributed, with a shape parameter set by the facies patch size
relative to the resolution cell [35]); **the simulator does not model
that**: every material, mud to rock, gets the same fully developed Exp(1)
intensity speckle (`sonar/noise.py`, from the coherent ground-scatterer
field of `renderer._texture`; CV 1, correlated along range and track).
Facies differ only in mean level (`reflectivity`), a static reflectivity
texture (`texture_amp`, `texture_cells`) and the micro-relief that throws
shadows (`micro_roughness_m`); per-material K shape parameters are an open
item (`roadmap.md`). The interpretation vocabulary — dark smooth mud, bright grainy gravel, rock
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
| material patches | patch length | 1–10 m (seagrass); K-statistics [35] not modelled | [12, 35] |

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
table (`worldgen/materials.py`) spans a nominal 5.9 dB from seagrass to rock (10·log₁₀ ρ; about 3.7 dB measured in an image under the old range law, `.claude/TODO.md`), where
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

### 7.2 A non-sand floor in a run (pass 4, 2026-11-02)

The pool corpus (painted concrete) is the first site whose floor is not sand. Its level-vs-range shape is
**not** a floor law. It is the seabed's own shallow-water multipath (`docs/sonar_model.md` §14, the seabed
image comb). Once that is in, the painted floor is sand's angular law, −16.4 dB lower, fitted on the pool's
FIT runs.

The renderer carries this as a global, per-run override of the diffuse law (`seabed_level_db`,
`seabed_lambert_exponent`) and of the floor's reflection for the combs (`seabed_material: painted_concrete`).
It does not use a per-cell material law: the material raster is still collapsed to reflectivity. A world
mixing sand and concrete cells would need per-cell law plumbing (`.claude/TODO.md`).

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

Added 2026-11-15 for §9–§12 (completion round, session 02). Each DOI was compared with its Crossref record
(title, authors, year) or its page was opened that day; **[full]** = the cited pages were read in full text,
**[abs]** = abstract or record only. The reading notes, pages and paper requests are in the session's evidence
folder (`~/ros2_ws/sim_runs/sim_acceptance/02_world_builder/stage0/`, this machine only).

51. Abraham, D. A., & Lyons, A. P. (2002). Novel physical interpretations of K-distributed reverberation. *IEEE Journal of Oceanic Engineering*, 27(4), 800–813. https://doi.org/10.1109/JOE.2002.804324 [full] (the counting model of patchy scatterers; its data are at 450–700 Hz)
52. Nickell, L. A., & Atkinson, R. J. A. (1995). Functional morphology of burrows and trophic modes of three thalassinidean shrimp species. *Marine Ecology Progress Series*, 128, 181–197. https://doi.org/10.3354/meps128181 [full]
53. Townsend, E. C., & Fonseca, M. S. (1998). Bioturbation as a potential mechanism influencing spatial heterogeneity of North Carolina seagrass beds. *Marine Ecology Progress Series*, 169, 123–132. https://doi.org/10.3354/meps169123 [full]
54. Dworschak, P. C., & Rodrigues, S. de A. (1997). A modern analogue for the trace fossil *Gyrolithes*. *Lethaia*, 30(1), 41–52. https://doi.org/10.1111/j.1502-3931.1997.tb00443.x [full]
55. Watson, S. J., et al. (2022). The footprint of ship anchoring on the seafloor. *Scientific Reports*, 12, 7500. https://doi.org/10.1038/s41598-022-11627-5 [full] (large ships in 35 m: an upper bound)
56. Watson, S. J., et al. (2020). Anchor and chain scour as disturbance agents: mooring-block chain scars. *Frontiers in Marine Science*, 7, 579626. https://doi.org/10.3389/fmars.2020.579626 [full]
57. Madricardo, F., et al. (2019). Assessing the human footprint on the sea-floor of coastal systems: the case of the Venice Lagoon. *Scientific Reports*, 9, 6615. https://doi.org/10.1038/s41598-019-43027-7 [full]
58. Schernewski, G., et al. (2023). Litter in harbours and marinas of the southern Baltic. *Applied Sciences*, 13(20), 11424. https://doi.org/10.3390/app132011424 [full]
59. NILIM (2022). *Supplemental Explanation Note to the Technical Standards for Port and Harbour Facilities in Japan* (draft). National Institute for Land and Infrastructure Management. https://www.ysk.nilim.go.jp/kakubu/kouwan/sisetu/tscphf.pdf [full, pages cited]
60. Lyons, A. P., & Abraham, D. A. (1999) = [35] above, read in full for §9 (80 kHz: patch contrasts and the cell-size law transfer; shape parameters are upper bounds).
61. Le Chenadec, G., Boucher, J.-M., & Lurton, X. (2007). Angular dependence of K-distributed sonar data. *IEEE Transactions on Geoscience and Remote Sensing*, 45(5), 1224–1235. https://doi.org/10.1109/TGRS.2006.888454 [full]
62. Nakajima, et al. (2007). *Proceedings of the Annual Meeting of the Japanese Society of Fisheries Engineering*, 249–252. https://doi.org/10.18903/pamjsfe.2007.0_249 [full] (sediment of an enclosed Japanese fishing port: sand 7 %, silt 60 %, clay 33 %)
63. Papatheodorou, G., et al. (2021). CEST2021, paper 00599. https://doi.org/10.30955/gnc2021.00599 [full] (a port surveyed by a USV with 455 / 800 kHz side-scan; 56 detected targets in 120 × 160 m)
64. Nippon Steel Drum Co., catalogue sheets reproducing JIS Z 1600 and Z 1601 (drum.nipponsteel.com, en-A1.pdf, en-A2.pdf) [full]; JIS Z 1600:2017 preview (Japanese Standards Association) [abs]
65. Andrews, E. (1947). *Crab pot construction (Chesapeake Bay type)*. Fishery Leaflet 262, US Fish and Wildlife Service [full]; Isaacson, P. A. (1963). *Commercial Fisheries Review*, 25(1), 12–16 [full]; Code of Maryland Regulations 08.02.03.07 [full]
66. Kochi Prefecture fisheries report FY2012, p. 75–77 (two Japanese pots, measured) [full]
67. UN Regulation No. 30, Rev. 3 (2007), §2 and §6.1 (tyre size designation) [full]; Gent, A. N., & Walter, J. D. (eds.) (2006). *The Pneumatic Tire*. NHTSA DOT HS 810 561 [full: ch. 1, cited passages of ch. 3, 6, 17]
68. Prado, J., & Dremière, P. Y. (1990). *Fisherman's Workbook*. FAO / Fishing News Books [full, sections cited] (fibre densities, twine and rope sizes)
69. Friedrichs, C. T., Rennie, S. E., & Brandt, A. (2016). Self-burial of objects on sandy beds by scour. In *Scour and Erosion*, 179–189. https://doi.org/10.1201/9781315375045-23 [full]; Chu, P. C., & Fan, C. (2007). *IEEE Journal of Oceanic Engineering*, 32(1), 34–48. https://doi.org/10.1109/JOE.2007.890942 [full]; Shrestha, et al. (2024). *Applied Ocean Research*, 148, 103948. https://doi.org/10.1016/j.apor.2024.103948 [full]
70. IHO (2024). *Standards for Hydrographic Surveys*, S-44 Edition 6.2.0 (adopted text) [full]
71. Mujal-Colilles et al. (2017) = [29] above, read in full for §11 (final scour depth "up to one propeller diameter").
72. Selfridge, A. R. (1985). Approximate material properties in isotropic materials. *IEEE Transactions on Sonics and Ultrasonics*, 32(3), 381–394. https://doi.org/10.1109/T-SU.1985.31608 [full, Table I] (mild steel: 5.9 and 3.2 mm/µs, 7.80 g/cm³)

Not obtained, and therefore not used (paper requests): APL-UW TR 9407 chapter IV as issued (the DTIC record
refuses the download; [34] is cited through [33] only); Stanic et al. (1988), doi:10.1121/1.396341; Williams
et al. (2002), doi:10.1109/JOE.2002.1040925, and (2009), doi:10.1109/JOE.2009.2018335 (the roughness model
against data above 100 kHz); Lyons et al. (2009), doi:10.1109/JOE.2008.2008410 (patchy seafloors at high
frequency); Broad, Rees & Davis (2020) full text (anchor marks of small boats).

---

## 9. Sediment classes and seabed heterogeneity (2026-11-15)

*Completion round, session 02. Evidence, proxies and the fit: `02_world_builder/` (handoff_02.md, design_note.md).*

**Why.** A world with one uniform sediment has no ping-to-ping correlation and none of the dark and bright
patches of a real harbour floor. The real FIT recording (`reflectionEvidenceFullharbour.svlog`, object-free
stretches) shows, against uniform simulated sand rendered at the same acquisition: a correlation of 0.03–0.15
three pings apart (uniform: 0.00), patch means that spread by 1.05–2.46 dB at 0.5 m and 0.37–2.08 dB at 2 m
(uniform: 0.93 and 0.22 dB), and 7–36 bright regions per 100 m² (uniform: 2.5).

### 9.1 Sediment classes

A seabed patch is a **sediment class**: a row of acoustic parameters, never a grey level (NC #17). The ray-cast
sensor's law per class is `sonar/raycast/seabed_ssa.SEDIMENTS` (density and sound-speed ratios, loss, roughness
spectrum, volume scattering); the legacy geometric renderer reads `worldgen/materials._SEABED_DEFAULTS`.

| Class | Parameters | Level against `sand` at 451 kHz (10° / 20° / 30° of grazing) | Role |
|---|---|---|---|
| `sand` | The calibrated harbour sediment (session 01; [33] SAX99 with the harbour core) | 0 | The **reference**: the only class the harbour sensor's first-return correction is calibrated on |
| `soft_mud` | [33] Table 13.1, Key West (carbonate sand-silt-clay, φ 7.0), as printed | −1.3 / −6.5 / −7.4 dB | The **finer** patch class (dark patches) |
| `fine_sand` | [33] Table 13.1, Quinault (φ 2.95) | −9.7 / −8.9 / −5.8 dB | Available; not in the presets |
| `coarse_sand` | [33] Table 13.1, Panama City 98 (φ 0.62) | +2.6 / +1.6 / −0.7 dB | Available; not in the presets |
| `gravel`, `rocks` | The sand law plus 3.0 / 6.5 dB (`scattering.SEABED_CONTRAST_DB`, as before) | +3.0 / +6.5 dB | `gravel` is the **coarser** patch class (bright patches, rubble, small litter) |

- No parameter set read gives a credible *bright* shell or gravel class through the roughness model: measured
  shell-hash spectra come out about 9 dB darker than sand. The bright class therefore stays the level class
  `gravel`. Stated limit.
- The roughness model is validated to about 100 kHz only [33]; at 450 kHz the classes are an extrapolation, as
  the calibrated `sand` itself is.
- The existing `mud` row cannot serve as a patch class in the harbour profiles (they override its spectrum):
  hence the new name `soft_mud`.
- **Reference sediment.** A world declares `world.seabed: {reference: sand, patch_classes: [...]}` in its
  configuration (the builder state's `depth.seabed`; the campaign generator writes it). Over a declared patch
  class the sensor's near-nadir correction stays on, judged on the reference; without the declaration a patch
  under the boat would switch it off (a 13–15 dB step). A world without it behaves as before.

### 9.2 The generator: six layer kinds

`worldgen/depth/layers/heterogeneity.py`; each is an ordinary layer of the editor (masks, opacity, its own
random stream `[seed, 2, layer]`).

| Kind | What it draws | Parameters | Source |
|---|---|---|---|
| `sediment_field` | A Gaussian random field (correlation `exp(−r²/L²)`), cut at two quantiles: the lowest share becomes the finer class, the highest the coarser one | `corr_len_m`, `finer`, `share_finer`, `coarser`, `share_coarser` | Patchiness as a mixture of scatterer populations [51], [60], [61] |
| `microrelief` | A zero-mean Gaussian random relief of a given RMS height and correlation length; used twice (fine 0.4 m, broad 3 m) | `rms_m`, `corr_len_m` | Fitted (9.4); the fine scale is what a burrowed floor looks like [52]–[54] |
| `bioturbation` | Feeding pits (0.3–1.0 m across, 0.03–0.12 m deep, up to 1.3 % of the floor) and large mounds (0.3–0.4 m across) | `pit_cover`, `pit_radius_m`, `pit_depth_m`, `mounds_per_m2`, … | [52]–[54]; the common mounds (under 15 cm) are smaller than two grid cells and are left to the sediment's roughness |
| `drag_marks` | Troughs with berms along an arc (keel, anchor and chain drags) | `per_hectare`, `width_m`, `depth_m`, `length_m` | [55]–[57]: sizes of large-vessel marks are upper bounds; the defaults for small boats are [DESIGN] |
| `small_litter` | Bottle- and can-size hard bumps of the coarser class; their list is kept (`small_litter_arrays`) | `per_m2`, sizes | [58]: 0.02–3.71 items per m² in 14 harbours |
| `cobbles` | Stones 0.1–0.4 m; with a mask along a quay toe, the blocks of a rubble apron | `per_m2`, sizes | [59]: foundation rubble 10–200 kg per piece |

### 9.3 Statistic definitions (acceptance criterion B2)

Computed by `analysis/seabed_stats.py` on a recording or a rendered session, on object-free stretches (straight
track, no wall or reflector, slant range past 1.4 × altitude):
- **lag-3 correlation**: of the gain-removed level in dB at the same bin, three pings apart, bin means removed;
- **patch spread** at 0.5 m and 2 m: standard deviation of the cell means of the level;
- **dark / bright regions per 100 m²**: connected regions of at least 0.05 m² whose power, smoothed over
  0.15 m, lies 4 dB under / over the per-bin reference (bands across 90 % of the window are rejected).

### 9.4 The fit, and the R2 presets

Fitted on the FIT stretches only (14 stretches, 40 m / 600 bins / 106 µs, window to 29.5 m), on physical
parameters: RMS heights, patch shares. `low` and `high` reproduce the smallest and the largest statistics over
the stretches, `nominal` their median (`worldgen/campaign.HETEROGENEITY`).

| Preset | Fine relief (0.4 m) | Broad relief (3 m) | Finer / coarser share, patch length | Pits, mounds, drag marks |
|---|---|---|---|---|
| `low` | 12 mm | 40 mm | 0 / 0 | none |
| `nominal` | 12 mm | 75 mm | 1 % / 0.5 %, 2 m | pits 0.5 %, 0.01 mounds per m², 4 marks per ha |
| `high` | 20 mm | 200 mm | 8 % / 3 %, 3 m | pits 1.3 %, 0.05 mounds per m², 12 marks per ha |

Generated against real, at the FIT acquisition (worlds built by the campaign generator: flat, 8.3 m, 800
pings, mean of two seeds; real: the 14 FIT stretches). [Batch B of session 02]

| Statistic | Real min / median / max | `uniform` | `low` | `nominal` | `high` |
|---|---|---|---|---|---|
| Correlation 3 pings apart | 0.025 / 0.044 / 0.146 | 0.005 | 0.024 | 0.066 | 0.216 |
| Spread of 0.5 m patch means [dB] | 1.05 / 1.24 / 2.46 | 0.95 | 1.13 | 1.52 | 2.62 |
| Spread of 2 m patch means [dB] | 0.37 / 0.60 / 2.08 | 0.25 | 0.38 | 0.68 | 1.71 |
| Dark regions per 100 m² | 26.4 / 31.6 / 37.1 | 49.8 | 59.4 | 51.4 | 42.1 |
| Bright regions per 100 m² | 6.8 / 9.3 / 36.4 | 2.8 | 8.2 | 15.0 | 47.3 |

Read against the tolerances of acceptance criterion B2 (correlation ±0.05, spreads ±30 %):
- `low` against the real minimum: correlation -0.001, spreads +8 % and +4 %: inside.
- `nominal` against the real median: correlation +0.022, spreads +22 % and +13 %: inside, on the rough side of the median.
- `high` against the real maximum: correlation +0.070 (outside ±0.05), spreads +7 % and -18 % (inside). The largest real stretch lies beside structures.
- The dark-region rate is outside the real range for every preset, the uniform seabed included (below).
- Mean level of a range band against the uniform world, largest shift: `low` -0.22 dB, `nominal` -0.45 dB, `high` -1.17 dB (all in the 16–20 m band).

What the fit cannot do, and why:
- The **dark-region rate** of the simulated picture exceeds the real one already on uniform sand (50 against
  26–37 per 100 m²) at this acquisition: the simulated speckle is more correlated along range (0.23 against
  0.12) than the device's at 40 m / 600 bins. It is a property of the sensor model at that setting, not of the
  seabed, and no seabed parameter can lower it. Session 05 judges it at the thesis setting.
- Four generator parameters cannot match four statistics at three levels exactly; the table states the misses.
- One harbour, one recording: the presets are a bracket, not a calibration.

**Level.** The dominant class is the calibrated `sand`, unchanged. The presets move the mean level of a range
band by the amounts listed above: relief changes the local grazing angle, the patches are few. The regression
worlds of the real harbour keep a uniform seabed (`harbour_wl1p50_blocks`).

## 10. The object set version 2

*`target_families_and_surrogates.md` (Version 2) is the specification; this is what is built.* A class is its
**parts** (`worldgen/compounds.py`: primitives with a material) and its **card** (`worldgen/population.py`:
sizes, states, poses, burial). How a part sounds is its material's physics (NC #17); there is no level anywhere.

| Class | Parts | Material | Sizes [m] | States and poses | Source of the sizes |
|---|---|---|---|---|---|
| `plastic_box` (T1) | The closed shell; a sediment block inside when part filled | `pp_shell`, wall 2–4 mm | L U(0.50, 0.75), W = L × U(0.55, 0.70), H U(0.22, 0.35) | Flooded 0.7, part filled 0.3; upright 0.5, inverted 0.3, on a side 0.2 | Six fish boxes of 42–70 L measure 0.60–0.84 × 0.37–0.51 × 0.18–0.31 m; wall thickness not stated by any source [DESIGN] |
| `drum_200l` (T2) | Body shell, two rolling hoops, two chimes; a sediment column when upright and filled | `steel_shell`, wall 0.9–1.6 mm | 0.568 body, 0.585 over the hoops, 0.890 high; hoops 0.300 apart | Lying 0.6, standing 0.4; flooded (never air-filled) | JIS Z 1600 / Z 1601 [64]; hoop width 20 mm and chime 15 mm [DESIGN] |
| `concrete_block` (T3) | A block with a chipped upper edge (0–30 mm) | `concrete` | U(0.5, 1.0) × U(0.4, 0.8) × U(0.2, 0.6) | Largest face down 0.8 | [DESIGN] |
| `trap_pot` (T4) | `us_wire`: frame of 4.8 mm rod, six mesh panels and the partition, four entrance funnels, a rebar rim, an anode. `jp_frame`: a 10 mm rebar frame under netting, a funnel in each end | `steel_rough`; netting: fibre | US 0.61 × 0.61 × U(0.51, 0.61); Japanese U(0.95, 1.00) × U(0.40, 0.68) × U(0.40, 0.45) | Upright 0.8, on a side 0.2 | [65]: 24 × 24 × 20–24 in, 38 mm mesh, wire 1.2 or 1.6 mm; [66] |
| `pipe_steel` (T5) | A closed flooded shell | `steel_shell`, wall 3–8 mm | Ø U(0.10, 0.30), L U(1.0, 3.0) | Lying | [DESIGN] |
| `tire_car` (T6) | Rubber body; the tread band over the belts on 0.82 of the section width; bead rings (bracket R6) | `rubber`, `tyre_tread`, `metal` | From a tyre code: outer Ø = rim + 2 × width × aspect ratio, within 0.55–0.80 | Flat 0.85, on its tread 0.15 | [67] |
| `net_bundle` (T7) | A seeded tangle of strands under a flattened dome; or a coil of rope 12–16 mm | Fibre (bracket R6) | U(0.5, 2.0) × U(0.4, 1.5) × U(0.1, 0.5); coil 0.3–0.8 across | Flattened | [68]; volume fraction and strand radius [DESIGN] |
| `mooring_block` (S1) | A box | `concrete` | Per instance (the harbour's: 4 × 4 m, 2.0–2.4 m over their hollows) | Fixed | Surveyed |
| `chain` (S2) | One rod per link along a line that may hang between two end heights | `steel_rough` | Per instance | — | Link 6 × bar, width 3.4 × bar [DESIGN] |
| `mooring_pile` (S3) | A cylinder | `concrete`, `steel_rough`, `wood_rough` | Ø 0.3–0.6, to the surface | Fixed | As before |
| `debris` (C1) | Two lumps | `steel_rough`, `concrete`, `generic` | 0.2–1.2 | — | As before |

Stated departures from the specification, each a drawing economy with the same physics:
- **Trap mesh**: a square mesh of twice the pitch (76 mm) and twice the bar of the real 38 mm hexagons: the
  same fill fraction, a third fewer parts.
- **Pipe**: a closed shell, not an open tube: half the mirror patches; the wall is nearly opaque either way
  (|R| 0.99). An end cap flashes when the sonar looks within about 1° of the pipe's axis.
- **Net**: a tangle, not an envelope with a volume fraction: sound passes between the strands, so the shadow
  is partial, and the volume fraction is the number of strands.
- **Thin steel** (wire, rebar, chain links, a pile in seawater) is rated rough (`steel_rough`): thinner than the
  3.3 mm wavelength, or scaled and overgrown, it has no flat facet to mirror from. It scatters diffusely with
  steel's reflection.
- **Fouling** is one value per world (the sonar's `fouling` parameter, recorded as `run.sonar_fouling`), not
  one per object.

**Bracket R6** (materials without a usable constant at 450 kHz; `uncertain_materials: low | nominal | high`):

| | `low` | `nominal` | `high` |
|---|---|---|---|
| Tyre | Belts hidden: the rubber body alone (|R| 0.07) | The tread band over the belts (rubber on steel, |R| 0.93) on the body | Nominal plus the two steel bead rings |
| Net, rope | Fibre |R| 0.1, volume fraction 0.05 | 0.2, 0.12 | 0.4, 0.30 |

The nominal tyre was chosen from rendered pictures (owner's rule: its shape must read easily, without a strong
flash): the tread band gives a small bright arc at the near edge (peak +10 dB over the seabed in a few bins,
+0.3 dB of echo energy) and the shadow; the low end is a shadow alone. The real belts are two plies of
0.15–0.35 mm filaments, not a steel band, so the truth lies between `low` and `nominal`.

**An existing world is not changed by any of this**: a class that existed before builds its version 2 parts
only for an instance that says `params.spec: 2` (the population model writes it).

## 11. Structures, litter population, depth domain

**Structures** (`worldgen/campaign.plan_basin`). Quays on 0–3 sides; a pontoon with four piles; mooring blocks
(2–4 m square, 1.0–2.8 m tall) each in a chain-scar hollow (radius 3–10 m, 0.1–0.4 m deep; rings up to 40 m
across and under 0.5 m deep are reported [56]) with a riser chain to under the surface and, near a pontoon, a
hanging line to it; a fishing berth with its propeller scour (0.3–0.8 m deep: "up to one propeller diameter"
[71], scaled to a fishing boat); a rubble apron 2–4 m wide at the breakwater's toe, blocks 0.2–0.5 m [59]. No
vegetation: a turbid basin of 4.5–11 m is at or below the eelgrass limit. No moored hulls.

**Population** (`worldgen/population.py`; the specification's section 5).

| Quantity | Value | Bracket |
|---|---|---|
| Sonar-size litter (targets + `debris`) | 2 / 10 / 30 per hectare of survey water [57], [63] | — |
| Share of targets | 0.5 | 0.3–0.7 |
| Composition | `realistic` (tyre 0.30, trap 0.15, net 0.15, box 0.15, block 0.10, pipe 0.10, drum 0.05) or `balanced` | — |
| Spatial pattern | `λ(x) = 1 + A Σ a exp(−d / 15 m)` over quays, pontoons, mooring blocks, the berth, outlets, the breakwater; `A` solved so that 0.7 of the expected count lies within 30 m of a feature (never under 1) | 5–40 m; 0.5–0.9 |
| Spacing | At least 3 m between targets | — |
| Burial | Per class and sediment (sand / mud), preset × 0.5 / 1 / 1.5, at most 0.9 [69] | — |
| `effectively_buried` | Burial over 0.7: in the ground truth, out of the detectable-target count | — |
| `nominal_target` | A target whose largest dimension is at least 0.5 m [70] | — |

Everything drawn is written as a pinned `objects.explicit` entry; the counts asked for and placed, the
amplitude and the realised share near a hotspot go into `metadata.yaml` (`population`).

**Depth domain** (`worldgen/domain.py`; acceptance criterion C1). Altitude = depth − 0.22 m (the sonar model's
transducer depth) ∓ 0.10 m of heave margin; survey water = wet cells reachable from the spawn without crossing
a quay; a world is in domain when 95 % of it lies in 4.5–11.3 m. Every world built since 2026-11-15 carries
the report in `metadata.yaml` (`domain`, with the share along the source path's own 20 m swath) and the mask in
`ground_truth_rasters.npz`; `domain.domain_report(terrain, grid, walls, path_xy=...)` gives it for any other
world (a real harbour holds much water its survey path never goes near).

## 12. The campaign generator

`worldgen_campaign --spec <campaign.yaml> --seeds 1,2,3 --out <worlds root>` (`worldgen/campaign.py`;
`config/campaign_example.yaml`; format and every key: `configuration_guide.md` §4). One world per (case,
seed), an ordinary world folder; nothing is overwritten; the same specification and seed give the same
**content digest** (`core/provenance.content_digest`: arrays, manifest and metadata without paths and dates).
Random streams: terrain `[seed, 0]`, objects `[seed, 1]`, layers `[seed, 2, layer]`, population `[seed, 3]`,
basin `[seed, 4]`.

