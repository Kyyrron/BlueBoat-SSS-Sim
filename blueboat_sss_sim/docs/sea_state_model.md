# Sea state model — current and waves on the simulated BlueBoat

What makes the simulated boat *feel* the water: a slowly varying current
that pushes and slows it, and wind waves that make it heave, pitch and
roll — so the controllers work against disturbances and the simulated
side-scan sonar shows the attitude signatures a real harbour run shows.
Implemented in `blueboat_sss_sim/sea/` (ROS-free, smoke section `[10]`)
and driven by `ros/sea_state_node.py`; presets in
`config/sea_states.yaml`; launch arguments on `full_mission_launch.py` and
`sea_state_launch.py`; live control from the Mission Control Station.

Every constant and formula below names its source; the numbered
references are at the end (§10), the research log behind each modelling
decision in §11, and the end-to-end validation — the same survey in calm
water, current, waves and both, run in Gazebo — in
[`sea_state_validation.md`](sea_state_validation.md).

**2026-10-08 revision, in one paragraph.** The current became a pure
function of seed, time and position (it was stepped on the ROS timer's
jittered `dt`, so the same seed did not give the same current), smooth
(C1 instead of Brownian), softly bounded, never restarted by a change of
situation, and weakly varying in space (§3); operators now set a *mean*
speed and direction plus a variation level, waves by strength with an
automatic period, crest length and wakes (§4.3), all resolved here. The
sonar now follows the hull fully: the transducer lever arm turns with
roll and pitch, every ping is rendered at the pose of its own time, and
that pose is published (§7). The effect on the side-scan image is
measured in Gazebo (§9, §12). One modelling discontinuity found on the
way — the hull's hydrostatics switched when the waves started (S11) — was
resolved on 2026-10-09: the catamaran hydrostatics now carry the hull in
**every** run, calm water included (§5).

---

**2026-11-10 revision, in one paragraph.** The real BlueBoat's attitude
(id-150 ATTITUDE, 25 Hz, four live-clock Shiraishi-jima harbour recordings,
`config/corpus_attitude_targets.yaml`) rocks at **~1.1 Hz** on every survey
line whatever the heading (roll centroid 1.11–1.17 Hz, 85–93 % of roll power
in 0.8–2 Hz, roll std 0.91–1.21°, pitch 0.60–0.87°); the simulated boat at the
old `smooth` (Tp 2.8 s) matched the *std* but rocked at **~0.65 Hz** -- the
regular, slow wave pattern seen in the 2026-11-10 harbour sonar runs. Three
causes, three fixes: (1) every wave level now follows the fetch-limited
young-sea relation **Tp = 3.6 √Hs** (§4.1; owner's decision), with a new
site level `harbour_chop` (0.07 m, 0.95 s) that reproduces the corpus
spectrum; (2) the field is discretised to resolve the wave-**slope**
spectrum up to 3 Hz (`hybrid`, §4.2) -- equal-energy bins had lumped the
whole short-wave tail into one component; (3) the hull reads the
Froude-Krylov pressure at its 4.5 cm **draft** (§5, retiring S4's "no
depth decay"). The offline mirror's pitch inertia was also wrong (§5:
5.64 kg·m², not 3.92). The gates now compare motion **spectra** with the
corpus (§9). Wave-run recordings before this date are not comparable bit
for bit.

## 1. Scope and boundary

The sea state acts on the **hull only**. Nothing else in the world moves:
the seabed, the litter objects and the walls are static, the acoustic
scene stays static during a run (`sonar_model.md` A4), and the water
surface stays a flat visual plane in Gazebo. What the boat feels and what
the sonar sees changes; the ground truth does not.

Two mechanisms, both external to the boat's own model, so **no file of
BlueBoat-Control is modified** (CM-3 / NC #8):

| Effect | Path into Gazebo | Who applies it |
|---|---|---|
| Current + wave orbital velocity | ROS `/current` → gz `/ocean_current` (the bridge both launch paths already create) | the boat's own `gz::sim::systems::Hydrodynamics` plugin, through its relative-velocity damping term (`hydrodynamics.xacro`) |
| Hull wrench: the catamaran hydrostatics (every run) + the wave excitation | ROS `/world/<w>/wrench` → gz `ApplyLinkWrench` (in every generated world since 2026-09; injected at runtime into the stock world and older folders) | gz-sim, `Link::AddWorldWrench`, one physics step per message |

The sonar needs nothing new on the wire: Gazebo's `OdometryPublisher`
(`<dimensions>3</dimensions>`) already carries the full 6-DOF pose on
`/blueboat/odom`, which `sss_sim_node` hands to the renderer (§7).

## 2. What Gazebo already provides, and its limits

`blueboat.xacro` gives the hull one collision box on which the
world-level `Buoyancy` system applies graded buoyancy about the flat plane
z = 0, and `hydrodynamics.xacro` attaches the `Hydrodynamics` system with
Fossen-form added mass and linear + quadratic damping [1]. Three
properties of that model matter here and are *measured* by the smoke test
against the xacro's own formula:

* **The buoyancy box has 3 % reserve buoyancy.** `buoyant_correction =
  1.01·((16.01 + 2·0.07) / (1000·1.2·0.91·0.376))^{1/3} = 0.3434` — the
  1.01 scales each *dimension*, so the volume is 1.01³ = 1.030 × the
  displacement — and the box is 0.412 × 0.313 × 0.129 m, floating 97 %
  submerged (waterplane 0.129 m², heave stiffness 1263 N/m, trim: link
  origin at z = −0.111 m; `restoring_summary` `reserve_buoyancy` 0.0303).
  Under a wave crest it can gain at most 3 % of the weight; under a
  trough it loses most of its lift. A wave force computed for *that* box
  would make the boat ride awash and sink by ~Hs/2 on average — an
  artefact of the URDF, not of the water. §5 says how the model avoids
  inheriting it.
* **The damping and added-mass coefficients are the BlueROV2's**
  (`hydrodynamics.xacro` cites von Benzon et al. 2022 [8]; no BlueBoat
  identification exists in the project; BlueBoat-Control is updated by
  additions only (it was frozen at `v1.0`), so one would be a request to
  that module).
  The current drag the controllers feel is therefore
  `xU·u + xUabsU·u|u|` with the ROV's 25.15 N·s/m and 33.8 N·s²/m²:
  0.25 m/s → 8.4 N, 0.5 m/s → 21 N, 0.8 m/s → 42 N against 2 × 40 N of
  modelled thrust. Realistic in order of magnitude for a 16 kg catamaran;
  not identified. Recorded, not fixed (Control's file).
* **The current enters the damping term only** (gz-sim 8
  `Hydrodynamics.cc`: `localLinearVelocity = R⁻¹ (v − v_current)` feeds
  the damping matrix; added mass and Coriolis use the absolute velocity).
  A steady current is therefore exactly a steady drag; an accelerating
  water mass would need a Froude–Krylov term the plugin does not have —
  negligible for tidal currents (S2).

## 3. Current model

**What the operator sets** is the **mean** current: a speed, the compass
direction it comes from, and a variation level. **What the simulator adds
by itself** is the small, smooth wandering around that mean that real
water has (`sea/current.py`):

    V(t, x, y) = [U + σ_V g(ξ_V(t))] · e(θ + σ_θ g(ξ_θ(t)))  +  s · u_eddy(x, y, t)

* **In time — Fossen's Gauss–Markov process, sampled exactly.** Fossen
  [1] §8.3 models a current's speed and direction as first-order
  Gauss–Markov (Ornstein–Uhlenbeck) processes with saturation,
  `V̇ + μV = w`, `V_min ≤ V ≤ V_max`; the UUV Simulator [14] and DAVE [15]
  Gazebo plugins step exactly that once per physics update. Here each of
  ξ_V, ξ_θ is a **unit-variance** process whose knots lie on a fixed
  sim-time grid (Δ = τ/12, clamped 0.5–5 s) and follow the **exact**
  AR(1) recursion `ξ_{j+1} = a ξ_j + √(1−a²) n_j`, `a = e^{−Δ/τ}`
  (Gillespie [16], exact for any step), from a stationary start; the
  normals of knot block b come from `default_rng([seed, stream, τ_ms, b])`
(τ in whole milliseconds, `sea/current.py`). So
  the realisation is a function of `(seed, t)` only — not of how often or
  when the node's timer fires (the previous Euler–Maruyama step used the
  jittered timer `dt` and shared one random stream between the two ends
  of a blend: the same seed did **not** give the same current, and DAVE's
  update has the same step-size dependence [15]).
* **Smooth, not Brownian.** Knots are joined by a Catmull–Rom spline
  (C1): the water's acceleration is continuous and bounded (an OU path is
  nowhere differentiable — at 100 Hz its acceleration is white noise of
  order √(2h/τ)/h² ≈ 180 s⁻² in unit terms, against 0.2 s⁻² now; smoke
  `[10]`). Between knots the variance dips at most 2 % below 1.
* **Bounded smoothly.** `g(ξ) = (B/c) tanh(ξ/B)`, B = 2.5, with `c =
  0.880` restoring unit variance: the fluctuation has the configured σ
  and never exceeds ≈ 2.84 σ (Fossen's saturation without the hard
  clip's plateaus); the speed is clamped at ≥ 0.
* **Never restarted.** A change of situation (a command or a timeline
  keyframe) blends U, θ and σ through the ramp; ξ keeps running, so the
  variation does not reset to zero at every change (it did: each retarget
  built a fresh process). When two states' time constants differ the two
  processes are cross-faded with a variance-preserving weight.
* **In space — weak eddies.** `u_eddy` is a divergence-free field of 8
  random Fourier modes of a stream function (Kraichnan [17]): wavelengths
  log-uniform in 25–100 m, directions uniform, each drifting in phase
  over 5–15 min; unit spatial RMS, bounded by `N·c`. Two lines a few
  metres apart, or the same line sailed back, therefore feel a slightly
  different but repeatable current. The node evaluates the whole current
  **at the boat's position** (a single boat; gz `/ocean_current` is
  uniform). The scale and strength are **judgement values** for a harbour
  basin — no site measurement exists (S2).

**Variation levels** (`current_variation:` in `config/sea_states.yaml`,
fractions of the mean speed U):

| Level | σ_V / U | σ_θ | τ_V / τ_θ | eddy RMS / U | Basis |
|---|---|---|---|---|---|
| steady | 0.05 | 3° | 60 / 120 s | 0 | controlled comparisons |
| natural (default) | 0.10 | 10° | 60 / 120 s | 0.05 | turbulence intensity ~10 % measured at tidal sites [18] |
| strong ("gusty / eddying") | 0.20 | 20° | 60 / 120 s | 0.10 | upper end of [18]; stress level |

Transfer caveat: [18] measures energetic tidal channels (deeper, faster);
a harbour basin is quieter, which is why *natural* is not raised above it.
τ is Fossen-style judgement (minutes), not measured. The legacy speed
presets (weak 0.1 … very_strong 0.8 m/s) are now *natural*-variation
shortcuts; explicit sigma fields in any block still win.

Direction convention: the user's **"from"** (meteorological, like wind:
"from North to South"). Oceanographers report the *set* (toward); the
readback and the markers carry both (`from: NE, toward: SW`). The
current enters Gazebo's damping only (§2): a steady current is a steady
drag, and the boat must crab into a cross-current to hold its line.

## 4. Wave model

### 4.1 Spectrum

Wind waves are synthesised from a **JONSWAP** spectrum (Hasselmann et
al. 1973 [3], fetch-limited — the harbour case), in the (Hs, Tp) form of
DNV-RP-C205 [5] §3.5.5,

    S(ω) = A_γ · (5/16) Hs² ωp⁴ ω⁻⁵ exp(−1.25 (ωp/ω)⁴) · γ^r ,
    r = exp(−(ω − ωp)² / (2 σ² ωp²)),  σ = 0.07 (ω ≤ ωp), 0.09 (ω > ωp),
    A_γ = 1 − 0.287 ln γ ,

with **γ = 1.5 in the presets** since 2026-09-03 (the JONSWAP North-Sea
mean is 3.3; Hasselmann et al. [3] report γ from 1 to 7 with the young,
fetch-limited seas of a small basin at the broad end — the user's review
found the γ 3.3 field "one wave every second and always the same", which
is exactly what a peaked spectrum produces). γ = 1 is the
Pierson–Moskowitz fully developed spectrum [4] — the smoke test checks the
two coincide. The variance `m0 = ∫S dω = Hs²/16` is reproduced within 2 %
(`[10]`). γ and the spreading `s` are per-preset fields (a "custom" wave
state from the MCS sets them explicitly).

**Peak period per preset (2026-11-10): Tp = 3.6 √Hs**
(`spectrum.TP_PER_SQRT_HS`, `tp_fetch_limited`), the fetch-limited young
sea. The fetch-limited growth laws give Tp/√Hs = 3.0–4.0 s/√m for a
harbour (U10 4–10 m/s over 0.3–2 km): JONSWAP [3] (ε̃ = 1.6·10⁻⁷ X̃,
f_m U/g = 3.5 X̃^−0.33) and Kahma & Calkoen [26] eqs (2)–(3) stable
(ε̃ = 9.25·10⁻⁷ X̃^0.766, ω̃p = 11.99 X̃^−0.242) and (6)–(7) unstable
(5.38·10⁻⁷ X̃^0.940, 14.19 X̃^−0.283), with X̃ = gF/U10², ε̃ = g²ε/U10⁴,
ω̃p = ωp U10/g; 3.6 is also DNV-RP-C205 [5] §3.5.10's lower bound and the
value that reproduces the corpus motion (§9). Levels: rippled 0.80 s,
harbour_chop 0.95 s, smooth 1.39 s, slight 2.13 s, rough 2.79 s. Until
2026-11-10 the presets sat *above* the band (2.0–4.5 s, "swell and wake
energy on top"); the corpus shows almost no roll power below 0.5 Hz
(2–3 %), and the long-period energy that does exist (wakes, swell trains)
is the `events:` groups' (§4.1b). `tp_from_hs_dnv` still returns the
mid-band value for reference.

### 4.1b Random wave groups (`sea/events.py`)

What a 1.2 m boat feels most in a harbour is not the stationary chop but
the **groups that arrive at random**: a passing vessel's wake (a few
cycles of 4–8 s waves, minutes apart — Soomere 2007 [13]), a swell train
leaking through the entrance, a gust-raised group. Each wave preset
carries an `events:` block `{rate_per_hour, hs_m, tp_s, duration_s,
from_spread_deg}`: arrivals are a **Poisson process** (exponential
inter-arrival times, mean `3600 / rate_per_hour`), each event its own
short directional train — an 8-frequency × 3-bearing `WaveField` (γ 1.5,
spreading s 8; a 4 × 1 field until 2026-09-05 rang like a metronome) with
Tp jittered ±15 %, a
direction drawn within ±`from_spread_deg` of the sea's from-direction,
seeded phases — multiplied by a Hann envelope of `duration_s`, so it
grows, peaks at `hs_m` and dies without a step. At most three overlap.
Elevation, slope and orbital velocity are the sums over the live groups,
exact zeros in between; the groups belong to the *target* state and live
outside the ramp blend, so a schedule step never restarts the arrivals.
The arrival statistics are an assumption (no wake census exists for the
site) and are stated as such (S9); the group *shape* follows the wake
literature. The readback reports the live group (`waves.event`), the MCS
box shows "wake group 0.25 m from ENE, 18 s left", and `[10]` checks the
rate, the zeros between groups, the envelope bound and seed determinism.

### 4.2 Directional field

`sea/wavefield.py` discretises the spectrum into frequency bins times
`N_d = 12` directions. **Since 2026-11-10 (`hull.wave_response.discretisation:
hybrid`, `spectrum.hybrid_frequencies`)**: 12 equal-energy bins over
0.4–1.5 × the peak, then log-spaced bins (ratio 1.25, ≤ 24 frequencies in
all) up to `f_max_hz` = 3 Hz, each band's component placed where k² equals
the band's slope variance over its variance -- so every band carries its
exact elevation AND slope variance (m0 rescaled exact). Why: equal-energy
bins resolve the *elevation* spectrum, but a 1.2 m catamaran rolls and
pitches on the *slope* spectrum (k² S ∝ f⁻¹ in the tail); for the old 2.8 s
sea the 16 bins ended at 0.89 Hz, the last one carrying the whole tail and
~40–50 % of the discrete slope forcing (a spurious line), with nothing above
it although half the slope variance lies there (OrcaFlex and ITTC
7.5-02-07-01.1 [28] warn that wide tail bins fail to excite a response;
Mérigaud & Ringwood [29] advise against equal-area schemes). 3 Hz is where
the 0.15 m demihull beam averages the waves out (a null near 3.2 Hz) and
the response falls as (f_n/f)² above the ~1.2 Hz resonance. Fossen's
practice [27] is equal Δω with ω drawn inside each interval and A_k =
√(2 S(ω_k) Δω); the per-tick cost of the hull integration (100 Hz,
3.5 ms at 24 × 12 components) bounds us at 24 frequencies, which the slope
check of `[10]` (within 3 % of the continuous slope variance to 3 Hz, no
component > 25 % of it) shows is enough. The wake groups keep their own
8 × 3 equal-energy field (unchanged). Before 2026-11-10, and still without
the `wave_response` block: `N_f = 16` frequency
bins of **equal energy** (component at the energy-weighted centre of each
band, so the discrete sum carries exactly m0 and resolves the peak where
the energy is) times `N_d = 12` directions with the cos^{2s}(Δθ/2)
spreading of Longuet-Higgins, Cartwright & Smith 1963 [6], `s = 3` in
the presets (a short-crested harbour chop — since 2026-09-05; the
earlier six bearings at `s = 6` put three quarters of the energy in two
rays, a long-crested sea whose moment sat about one fixed world axis and
read as "pitch only" on a boat heading into it), a small **random**
frequency offset per component drawn from the seed (±4 %; so one bin's
components never add coherently at every point — random rather than the
earlier fixed ±3 % ladder, whose steps beat with a period of their own
and read as a regular swell), finite-depth Airy dispersion
`ω² = g k tanh(k h)` with `h` the world's mean depth (from the folder's
`depth.npz`), and uniform random phases from the run **seed**:

    η(x, y, t) = Σ_ij a_ij cos(k_i (x cos θ_j + y sin θ_j) − ω_i t + φ_ij),
    a_ij² = 2 (m0 / N_f) D_j .

Surface orbital velocities come from the same components (Airy theory at
z = 0): horizontal `a ω coth(kh)` in phase with the elevation along the
propagation direction, vertical `a ω` in quadrature. Measured in `[10]`:
σ_η = Hs/4 at any point (±12 %), waves "from E" travel west, the orbital
speed scales as σ_η·ωp, same seed → identical field.

### 4.3 Presets and the operator's choices

`config/sea_states.yaml`, placed on the WMO 3700 / Douglas code [7] by Hs
(codes corrected 2026-10-08: 0.35 m is code 2 "smooth", 0.6 m code 3
"slight" — the earlier labels called 0.35 m "slight"); the preset *keys*
are unchanged (they are launch arguments), the labels are plain:

| Key | Label | Hs [m] | Tp [s] | γ / s | wake groups (per h, Hs, Tp, dur) | WMO 3700 |
|---|---|---|---|---|---|---|
| calm | Calm (flat water) | 0 | — | — | none | 0 |
| rippled | Ripples (0.05 m) | 0.05 | 0.80 | 1.5 / 3 | 4, 0.08 m, 5 s, 25 s | 1 |
| harbour_chop | Harbour chop (site, 0.07 m) | 0.07 | 0.95 | 1.5 / 3 | 5, 0.10 m, 5 s, 25 s | 1 |
| smooth | Light chop (0.15 m) | 0.15 | 1.39 | 1.5 / 3 | 6, 0.12 m, 5.5 s, 25 s | 2 |
| slight | Choppy (0.35 m) | 0.35 | 2.13 | 1.5 / 3 | 8, 0.25 m, 6 s, 25 s | 2 |
| rough | Rough (0.6 m) — stress test | 0.60 | 2.79 | 1.5 / 3 | 15, 0.35 m, 7 s, 30 s | 3 |

Periods since 2026-11-10 (Tp = 3.6 √Hs, §4.1; were 2.0 / 2.8 / 3.6 / 4.5 s).
`harbour_chop` is an **inverse estimate**: the height whose offline motion
(`simulate_3dof`) matches the corpus roll std and spectrum (roll 0.84–1.17°
at 1.06–1.12 Hz vs real 0.91–1.21° at 1.11–1.17 Hz; pitch 1.03–1.18° vs real
0.60–0.87° -- the model's pitch response is ~1.4× the real one, a hull
limitation recorded in `TODO.md`, not tuned away), not a wave measurement.
Measured offline per level (one seed, 90 s): rippled roll 0.6° / pitch 0.7°,
smooth 2.3° / 2.2° (heave 2.8 cm), slight 4.5° / 3.4° (7.9 cm), rough
4.3° / 4.0° (11.9 cm) -- the label texts quote these.

Plain-language keys (2026-10-08, additive; resolved in
`sea/presets.py`, never by MCS):

* **`tp_s: auto`** — Tp interpolated from the presets' own (Hs, Tp)
  pairs, `∝ √Hs` beyond them (the steepness scaling of DNV-RP-C205 [5]
  §3.5.10) — so a custom height gets the period the levels imply.
* **`crest: short | long`** — the directional spreading `s`: 3 (today's
  short-crested harbour chop) or 25 (long-crested swell). Goda & Suzuki
  [19] give s_max 10 for wind sea and 75 for swell; 25 is the
  intermediate "swell with a short decay distance" value of secondary
  sources (swell entering a harbour has travelled little). Our `s` is
  frequency-independent (Goda's peaks at the spectral peak).
* **`wakes: none | occasional | frequent`** — the wake-group rate × 0, 1
  or 2.5 of the level's own (of the nearest level's for a custom height).
* **current `variation`** (§3) and **keyframe `ramp_s`** (a timeline
  segment's transition under `hold`).

The file carries a label, a one-sentence plain consequence for the boat
and the sonar, and a reference per entry; MCS shows label and
consequence, never the reference. Directions are chosen per run, not
stored. `rippled`'s peak wavelength (Tp 2.0 s: 6.2 m at 5 m depth, 5.8 m
at 1.5 m — `spectrum.dispersion`) is about five hull lengths, so the boat
contours it (§5) at its few centimetres of height: a legitimate "barely
anything" level.

## 5. Wave → hull coupling (Froude–Krylov, hydrostatic)

For a body much smaller than the wavelength the wave force is dominated
by the pressure the undisturbed wave would exert on the hull — the
Froude–Krylov force; diffraction and radiation are higher order in
`L/λ` (Newman 1977 [2] ch. 6; the long-wave limit of the response
amplitude operator is "the body follows the surface"). At the BlueBoat's
scale (L = 1.2 m, λp = 6.2–26 m for the presets at 5 m depth, 5.8–16 m
at 1.5 m) and draft (0.045 m ≪
λ, so the exponential depth decay is 1), the Froude–Krylov force is the
**hydrostatic buoyancy under the instantaneous wave surface**.

`sea/hull.py` integrates it over the hull footprint: columns over each
hull box in the body frame, rotated to the world by the boat's attitude,
each submerged by `clip(η(x, y, t) − z_bottom, 0, z_top − z_bottom)`;
force `ρ g Σ h dA` upward at the column centroids; moment about the
**link origin** in world axes (the bridge drops the gz `force_offset`
field, so `ApplyLinkWrench` applies the force at the link origin — a
moment taken about any other point would be a slow, mystifying drift).

**Which hull.** The wrench published in every run (since 2026-10-09;
before, only while waves were active) is

    F = H_cat(η) − H_gz(0) + damping ,

`H_cat` the buoyancy of a **catamaran hydrostatic hull** (two demihulls
1.2 × 0.15 × 0.376 m, 0.76 m apart — the BlueBoat 120 in the round —
placed so that their draft at the Gazebo trim reproduces the
displacement: waterplane 0.36 m², draft 0.045 m) under the wave surface,
and `H_gz(0)` the Gazebo buoyancy box (mirrored from the xacro formula)
under the flat plane, i.e. exactly what Gazebo itself keeps applying. The
sum on the hull is therefore `H_cat(η)`: the boat floats on the
catamaran's hydrostatics under the actual surface — the flat plane on
calm water, where no wave field is evaluated (0.14 ms per tick). Both
hulls share the trim by construction, so the replacement is
static-force-free: at rest at the trim the wrench is zero (`[10]`:
5.7e-14 N), and away from it it is the catamaran's restoring (a 2°
bow-up pitch meets +15 N·m).

*Why the catamaran in every run (S11, resolved 2026-10-09).* Until then
the swap happened only with waves on, so calm runs floated on the URDF
box: soft in pitch (C55 5–46 N·m/rad, 3 % reserve buoyancy), it let the
thrust couple trim the boat 2.2° bow-up under way, while the catamaran
trims ~0.2°. Calm-vs-waves comparisons therefore mixed two hulls (a
~0.17 m mean footprint offset at 4.7 m altitude), and a timeline ramping
calm → chop stepped the hydrostatics at its first tick (22.6 N, `[10]`).
The corpus shows the box's trim was right only by coincidence (pitch from
id-150 `ATTITUDE` binned by GPS speed):

| Session | at rest | 0.9–1.2 m/s | dynamic part |
|---|---|---|---|
| `OneLineHarbourClean70m` | +1.21° | +2.08° | +0.9° |
| `fullHarbourCleanExample` | +1.39° | +2.29° | +0.9° |
| `toOptimize-harbourScan` | +1.46° | +2.20° | +0.7° |
| `bath3` (beach, 1.2 m water) | +2.79° | +0.26° | −2.5° |

Most of the field "trim" is a **static** offset present at rest (load
distribution or the IMU's mounting) that differs between sessions; the
speed-dependent part is small and of inconsistent sign (bow-up in the
harbour, bow-down in very shallow water — plausibly squat). A fitted trim
term would model an unknown per-session offset, constant along-track and
irrelevant to any comparison, so none is added. One hull everywhere:
calm-vs-waves isolates the sea. `hull.hydrostatics: gazebo_box_when_calm`
restores the earlier behaviour for old runs and A/B. Since the same day
MCS starts the sea node for **every** empty-world run, a calm, no-current
choice included, so the station's runs all float this hull; only a bare
command-line `Sim_launch.py` without `sea_state_launch.py` beside it stays
on the box (control tuning, no seabed scene). The MCS SEA STATE box flags
a run on the box (readback `hull.model: gazebo_box`).

Resulting stiffness and natural periods (URDF inertia in the **link**
frame, the plugin's added-mass diagonal, `HullBoxes.restoring_summary`,
`[10]`). The xacro writes the base link's `<inertia>` as (2.42, 3.92,
5.64) kg·m² in an inertial frame rolled by `rpy="${pi/2} 0 0"`, which
swaps y and z: Gazebo's **pitch** inertia is 5.64 kg·m², its yaw 3.92.
Until 2026-11-10 this table and the offline mirror took 3.92 as pitch
(0.60 s; corrected below; `sea.motion.URDF_LINK_INERTIA`, smoke `[10]`
re-derives it from Control's xacro):

| Hull | A_wp [m²] | C33 [N/m] | C44 [N·m/rad] | C55 [N·m/rad] | T_heave | T_roll | T_pitch |
|---|---|---|---|---|---|---|---|
| Gazebo box (only without the sea node) | 0.129 | 1263 | 38 | 4–46* | 0.98 s | 1.6 s | 2.2–7.3 s* |
| catamaran (every run with the sea node) | 0.36 | 3530 | 577 | 440 | 0.59 s | 0.42 s | 0.72 s |

\* the 97 %-submerged box's pitch stiffness is strongly non-linear about
the trim (its waterplane engages only as an end emerges). With natural
periods well below a long-period wave band (Tp ≥ ~2 s) and the ROV-class damping, the
catamaran **contours** the surface quasi-statically: heave ≈ η, pitch and
roll ≈ the local wave slope along and across the heading. **Since 2026-11-10
the levels at and below `smooth` are NOT in that regime** (Tp 0.8–1.4 s,
wavelength 1–3 m ≈ the hull): the hull filters the surface (the beam-sea
roll moment peaks at λ = 2 × the demihull spacing, ~1.0 Hz; the head-sea
pitch moment near 0.93 Hz) and responds near its own resonances -- which
is what the corpus shows (a heading-invariant ~1.1 Hz peak, output-only
estimates roll ≈ 1.15–1.3 Hz with ζ ≈ 0.1–0.25, pitch ≈ 1.05–1.2 Hz with
ζ ≈ 0.1–0.2). The Froude–Krylov pressure is therefore read at the
**draft** since 2026-11-10 (`hull.wave_response.draft_decay`): each
component of the surface the hull integrates is attenuated by
cosh(k(h−d))/cosh(kh) at d = the 4.5 cm draft (`WaveField.decay`; e^(−kd)
in deep water: 0.83 at 1 Hz, 0.49 at 2 Hz; Journée & Massie [30] eq. 8.43,
the "Smith effect"); the surface elevation the readback reports stays the
undecayed one. The hull's own natural periods and damping (BlueROV2 added
inertia 0.12 kg·m² against ~3 kg·m² of demihull added inertia; judgement
ζ ≈ 0.4 against the corpus's 0.1–0.25) stay a recorded limitation
(`TODO.md` "hull v2").

Checks in `[10]`: a crest of +0.1 m lifts by `ρ g A_wp η = 353 N`; a
trough of −0.1 m drops the full weight (the shallow-draft hull hangs);
a surface rising toward +x pitches the bow up (M_y < 0), rising toward
+y rolls port-up (M_x > 0); the wave force on an active state is
zero-mean.

**Orbital velocity** is added to the published water velocity, so surge
and sway wave drag (and the small rectified drift through the quadratic
term) emerge from the existing damping — no separate wave-drift force
(S5).

### 5.5 Horizontal force and yaw (2026-09-05)

The pressure under a sloping surface is not vertical: to first order
each submerged column also feels `f_h = −ρ g V ∇η` (Newman [2] ch. 6,
the long-wave Froude–Krylov limit), applied at its mid-depth.
`HullBoxes.submerged_wrench(pose, surface, slope)` adds it when given
the field's slope; the model passes `WaveField.slope` (plus the live
wave groups'). Two demihulls on a short-crested sea then see different
slopes, which yaws the hull, and every crest passing the boat gives it
a surge/sway kick — the sonar sees along-track sampling jitter and
heading wobble, not only altitude ripple. Flat water gives exactly zero,
and without a slope function the wrench is bit-identical to the
vertical-only model (`[10]` checks both, and `Fx = −ρ g V·0.2` on a 0.2
slope to 10 %).

### 5.6 Pose prediction (2026-09-05)

The published wrench carries the catamaran's **restoring stiffness**
(C33 ≈ 3.5 kN/m, C44 ≈ 580 N·m/rad, C55 ≈ 440 N·m/rad) evaluated at the
pose the node last received. `blueboat.xacro` publishes odometry at
20 Hz; the node publishes at 100 Hz. A restoring spring acting on a pose
up to 50 ms stale is a *delayed* spring, and on the 0.59 s heave mode
with the plugin's ζ ≈ 0.03 it pumps energy in: the offline closed loop
(`sea/motion.py::simulate_3dof`, seed 3, 40 s) reproduces a **0.6 s
heave bounce** of 0.27 m peak-to-peak at 'smooth' with half the heave
power above 1 Hz
— the regular, too-frequent "wave" reported in the 2026-09-05 review —
while the same model fed a 100 Hz pose simply contours the sea.
`sea_state_node` therefore propagates the last odometry (pose + the
body-frame twist Gazebo's `OdometryPublisher` sends) to the tick time
with `predict_pose` before evaluating the wrench, and reports the pose
age in the status JSON (`pose_age_ms`, additive).

**2026-10-09: the 100 Hz pose.** Prediction was not enough in roll. With
the catamaran on calm water too (§5), Gazebo showed a **self-sustained
roll of ±0.5° at 3.2 Hz** with nothing exciting it — and the same ring
had been hiding under the wave roll before (≈0.12° rms in its 2.8–3.6 Hz
band under the 1.3° wave roll of the 2026-10-08 runs). A roll-only discrete model at Gazebo's
10 ms step reproduces it exactly when the catamaran's roll increment
(538 of C44's 577 N·m/rad; the box Gazebo keeps applying supplies 39)
acts on 20 Hz odometry whose twist is itself an average over its 50 ms
period, with ~2 physics steps of transport: 3.1 Hz, growing to 1° at 4
steps. The cure is the information, not more damping: blueboat_description's
own `PosePublisher` gives the model pose **every physics step**;
`sea_state_launch.py` bridges it, stamped, to `/sim/sea_state/boat_pose`
(`geometry_msgs/TransformStamped`, sim-only), and
`sea/motion.py::HighRatePose` differences consecutive poses into body
velocity and rates (exact ZYX kinematics). The same model is then stable
up to 4 steps of delay; in Gazebo the calm roll std went from 0.34° to
0.003° and the ring band of the wave roll from 0.8 % of its power to
zero — what remains above 1 Hz (5–6 %) is the chop's own high-frequency
tail. The odometry stays the fallback (`pose_source` in the readback).

### 5.7 Hull damping (2026-09-05)

The Hydrodynamics plugin's coefficients are the BlueROV2's (S6): heave
damping `zW = −18 N·s/m` gives ζ ≈ 0.03 on the catamaran's own
stiffness, so any residual lag rings. A catamaran's radiation plus
viscous damping is far higher; the model adds, whenever it knows the
boat's twist, `F_z = −b33 (v_z − w_orbital)`, `M_x = −b44 p`,
`M_y = −b55 q` with `hull.damping` of `config/sea_states.yaml` sized for
ζ ≈ 0.4: `b = 2ζ√(C (m + a)) − b_plugin` → 245 N·s/m, 20 and 13 N·m·s.
These are judgement values (a decay test on the real boat would anchor
them; `TODO.md`). Measured closed-loop at 'smooth' with the 20 Hz pose
hold: heave dominant period 2.7 s (≈ Tp), no power above 1 Hz, roll std
1.5°, pitch 1.3° — inside the corpus band (§9).

### 5.8 Excess-buoyancy clip (2026-09-05)

A 4.5 cm-draft demihull buried by a crest it did not ride publishes
several weights of buoyancy (5.7 g measured with the boat pinned at the
trim under 'slight'). `WaveExcitation` soft-clips the catamaran's excess
buoyancy `H_hull,z − m g` above `wave_force_clip.knee_g` weights (2.5)
with a C1 tanh knee of `width_g` weights (1.0). A hull that rides its
wave never reaches the knee (a +0.1 m crest is 2.2 weights, checked
exact in `[10]`); it only removes the slam.

## 6. Delivery into Gazebo

* **One-shot wrenches at the physics rate.** `sea_state_node` publishes
  `ros_gz_interfaces/EntityWrench` on `/world/<w>/wrench` at
  `rate_hz = 100 = 1 / max_step_size`; each message is applied for
  exactly one step and consumed once, so the impulse ∫F dt is conserved
  under any transport jitter (a zero-mean wave force gives zero drift),
  and a dead node or bridge leaves **no** force behind. `persistent`
  wrenches (which accumulate until cleared) are never used; `/wrench/clear`
  is sent once at start and stop as hygiene. Each message is scaled by
  `k = Δt_sim / max_step_size` so a lower rate or an RTF dip is corrected,
  not silently halved.
* **Entity** `{name: "blueboat", type: MODEL}` → the model's canonical
  link, `blueboat/base_link` (the link carrying the hydrodynamics and the
  buoyancy box).
* **Plugin presence.** Worlds written by `sdf_writer.py` since 2026-09
  include `gz-sim-apply-link-wrench-system`. For the stock `ocean` world
  and older folders the node calls gz-sim's
  `/world/<w>/entity/system/add` (`gz.msgs.EntityPlugin_V`, served by
  the SystemManager) and confirms the `/world/<w>/wrench` topics appear;
  otherwise it logs once and runs **current-only**, reporting
  `waves: unavailable` with the reason.
  Without the wrench path the hull falls back to Gazebo's box too; the
  readback says which hydrostatics carry the boat (`hull.model`).
* **The pose the wrench uses** is Gazebo's per-step model pose
  (`/sim/sea_state/boat_pose`, stamped, 100 Hz, bridged by
  `sea_state_launch.py`), the 20 Hz odometry only as a fallback
  (`pose_source`; §5.6).
* **Sim time.** Timers run on `/clock`, the wave phase uses the current
  sim time and the boat position of that pose (≤ 10 ms old from the
  100 Hz pose; ≤ 50 ms on the odometry fallback: ≤ 5 cm at 1 m/s against
  wavelengths of metres). The field is a deterministic
  function of `(seed, t_sim, x, y)`: a re-run reproduces the same sea.

## 7. Effects on the side-scan sonar

The renderer (`sonar/renderer.py`, `sonar_model.md` §4) receives the
6-DOF pose per ping and couples it as a real platform does (Cobra,
Oppenheim & Jaffe [9]; Blondel [10] ch. 4; Anstee [20]):

| DOF | Mechanism | Visible as |
|---|---|---|
| roll | tilts the vertical beam pattern toward one side (`acoustics.beam_weight`) **and** (2026-10-08) lifts one transducer / dips the other by `mount_y·sin(roll)` | port/starboard brightness banding along track — the surface-platform signature (thesis contribution C4); 8° roll tilts the far-vs-near brightness by about ±1.1 dB, opposite on the two sides (smoke `[10]`) |
| heave | changes the altitude, slant geometry, water column, surface-multipath delay | a wavy first-bottom-return line, water-column band breathing |
| pitch | rotates the fan plane about the body y axis: each ground sample `dz` below the transducer images `dz·tan(pitch)` along-track; the transducer itself swings `sensor_depth·sin(pitch)` | the **common mode**: both sides' footprints move along-track together; at 5 m the beam's ground speed swings between stopping, stepping back and racing ahead (§12) |
| yaw / yaw rate | turns the fan: the ground at range r moves `r·sin(Δψ)` along-track | the **differential mode**: one side advances while the other lags, growing with range (Lei et al. [21] separate the two modes the same way) |
| surge / sway | per-ping position | irregular along-track sampling; crab angle and cross-track offset in a current |

**The sensor pose (2026-10-08).** `renderer._sensor_pose` rotates the
lever arm `[mount_x, ±mount_y, −sensor_depth]` by the full R(roll, pitch,
yaw) (yaw only before — the docs' "3 mm at 10°" was the horizontal term;
the vertical one is 49.5 mm on the ±0.285 m mount). `sss_sim_node` renders
each ping at the boat's pose **propagated to the ping time** — since
2026-10-09 Gazebo's per-step model pose (`/sim/sea_state/boat_pose`, at
most a physics step old), the 20 Hz odometry only as a fallback — with
its own body rates (`sea.motion.predict_pose`, the function the hull
wrench uses), and publishes the poses it used (and their source) on
`~/ground_truth/sensor`. A level
pose is bit-identical to before (the render fixture holds).

What a surveyor sees, therefore, emerges from the physics chain
**waves → hull wrench → Gazebo motion → transducer pose → ping geometry
→ image**; nothing distorts the image directly. Measured on the same
survey in Gazebo (§12, `sea_state_validation.md`): footprints that step
back over seabed already imaged (a quarter of the pings at light chop),
pings per target three times as uneven, targets imaged twice or broken
up in the waterfall, a jagged seabed line.

Not modelled: intra-ping motion (A5, stop-and-hop pings — over a 20 m
ping's 27 ms a 10°/s yaw rate turns the fan 0.27°, ~9 cm at the far edge
against a ~17 cm footprint, i.e. smearing within one ping; Preston &
Gorton's stability criterion for multibeam sidescan is about exactly
this), the motion-induced Doppler on the pulse, and wave-induced
roughness of the sea surface in the optional surface mirror (S8).

## 8. Assumptions

* **S1 — linear waves, single spectrum.** Airy components with random
  phases; no breaking, no swell partition, no wave–current interaction.
  Fine for Hs ≤ 0.6 m in 3–10 m of water.
* **S2 — current: Gauss–Markov in time, weak eddies in space,
  damping-only.** The temporal part is Fossen's model, sampled exactly;
  the eddy field's 25–100 m scale, 5–15 min evolution and strength are
  judgement values (no site measurement); the node publishes the current
  at the boat (gz `/ocean_current` is uniform — one boat). It enters the
  plugin's damping term only (the plugin's design).
* **S3 — flat visual surface.** Gazebo's water plane does not move; the
  boat moves on an invisible surface. The indicators (§9) show the state.
* **S4 — hydrostatic Froude–Krylov on a box catamaran.** No diffraction,
  no radiation beyond the plugin's added mass and damping plus the
  linear hull damping of §5.7, the Smith-effect depth decay applied at
  the mean draft since 2026-11-10 (one depth for every column), pressure
  taken along world-vertical columns (exact for the level hull,
  small-angle for a tilted one) with the first-order horizontal
  component of §5.5, demihull geometry approximated by boxes.
* **S5 — no explicit second-order wave-drift force.** The first-order
  horizontal Froude–Krylov force (§5.5) is zero-mean; mean drift arises
  only from the quadratic damping acting on the orbital velocity.
* **S10 — the hull damping coefficients and the pose predictor are
  judgement values (§5.6–5.7).** ζ ≈ 0.4 is typical of a small catamaran
  in heave; a decay test on the real boat, or a BlueBoat identification
  in `BlueBoat-Control`, would replace them.
* **S6 — the boat's hydrodynamic coefficients are the BlueROV2's** (§2).
  The response amplitudes in waves inherit that.
* **S7 — no wind on the hull.** gz-sim's `WindEffects` system exists and
  is a roadmap item; the presets describe the sea, not the air.
* **S8 — sea state does not touch the acoustic scene.** Surface roughness
  is not fed into the `surface_reflectivity` of the optional surface
  mirror (`sonar_model.md` §10); the seabed and objects are static.
* **S11 — one hydrostatic hull, no dynamic trim (resolved 2026-10-09).**
  The catamaran carries the hull in every run (§5); dynamic trim/squat
  is in no model — the corpus shows it small and of inconsistent sign —
  and the per-session static offset (load, IMU mounting) is not
  modelled.
* **S12 — stop-and-hop pings.** See §7: intra-ping rotation smears at
  most about half a footprint at the swath edge at 10°/s.
* **S9 — wave groups are a marked Poisson process.** Arrival rate,
  height, period and duration per preset are judgement values sized to a
  small harbour with traffic; no wake census exists for the site. The
  group shape (a Hann-enveloped 4–8 s train) follows the wake literature
  [13]; the arrivals do not.

## 9. Verification and indicators

**Offline (`python3 -m test.smoke_test`, section `[10]`, ~70 checks):**
spectrum variance and peak, PM equivalence, dispersion limits, field
statistics, propagation direction, orbital scaling, seed determinism,
exact no-ops (Hs 0, current none, flat water), hull trim from the xacro
formula, shared trim, crest/trough/slope signs and magnitudes, natural
periods, Gauss–Markov statistics and saturation, presets' labels /
references / Tp band, unknown-key errors, schedule interpolation and
round-trips, command decoding, ramping without a step, JSON readback,
markers, launch resolvers, the generated world's plugin, the renderer
pitch shift, and (2026-09-03) the wave-group process: ~30 arrivals per
hour at rate 30/h, exact zeros between groups, envelope bounded by `hs_m`,
seed determinism, the no-event preset a strict no-op, the model carrying
a group into the surface and the readback; and (2026-09-05) the
horizontal force and its bit-identical zero-slope no-op, the
excess-buoyancy clip (a buried demihull publishes 3.5 weights, the
+0.1 m crest stays exact), pose prediction and damping signs, and the
**closed loop**: `simulate_3dof` with the pose held for 50 ms — the
un-fixed loop rings at 0.6 s with half its heave power above 1 Hz (kept
as a detail line, never asserted), the fixed one must put the heave peak
above 1.5 s with < 10 % of the power above 1 Hz, roll and pitch std in
0.5–2.5°, a non-zero surge force and yaw moment, and stay finite.

**Real-boat SPECTRUM (2026-11-10, `config/corpus_attitude_targets.yaml`,
generated by `python3 -m blueboat_sss_sim.analysis.sea_validation targets`,
never hand-edited):** the four live-clock harbour recordings on their
straight, under-way lines (`analysis.sea_validation.attitude_spectra`: 20 s
Welch windows after a 10 s high-pass): roll std 0.91–1.21°, centroid
1.11–1.17 Hz, 85–93 % of its power in 0.8–2 Hz; pitch std 0.60–0.87°,
centroid 0.91–1.17 Hz; yaw rate 1.7–2.8 °/s with 31–72 % of its power above
1.2 Hz. Smoke `[10]` reproduces one file's numbers exactly and gates
`harbour_chop` offline (roll centroid within 0.15 Hz of the band, ≥ 75 % in
0.8–2 Hz, roll/pitch std 0.6–1.6°); `test/gazebo_sea_test.py` (default
`--waves harbour_chop`) gates the same spectrum live plus wave-rate yaw
jitter (≥ 5 % above 1.2 Hz). The contouring checks (heave std = Hs/4 at Tp)
apply only to seas with a peak wavelength ≥ 4 hull lengths.
**Earlier anchor (2026-09-05, std only):** roll std 0.9–1.3°, pitch std 0.8–1.5° about a
+2° bow-up trim, dominant period 1.0–1.2 s (short wind chop), rates
5–10 °/s. That band is the *measurement*; the pass criterion the gates
enforce is the wider **0.5–2.5°** (roll and pitch std, `simulate_3dof`
in `[10]` and `test/gazebo_sea_test.py`), because the presets' Tp
are longer than the corpus's chop by design (the user's choice was to
keep the preset levels): compare the *amplitudes*, not the period, and
read a value outside 0.9–1.5° as a difference from this site's chop, not
a failure.

**Gazebo gate (2026-10-08, `test/gazebo_sea_test.py`).** The live
null / impulse / motion tests below are now one scripted, recorded run of
four scenarios on the validation world — calm (null: no wave forcing,
the catamaran hydrostatics active, level, no back-scan), current, waves
(impulse: mean hull height = calm's within 3 mm; the same hull: mean trim
within 0.5° of calm's; motion: roll/pitch in the corpus band, heave std =
Hs/4 with its peak near Tp, no 1.6 Hz line, ≤ 60 ms pose age) and both —
plus the sonar consequences. The earlier written limit "heave peak-to-peak <
0.15 m at 'smooth'" contradicted its own preset (Hs 0.15 m: a Gaussian
sea's largest crest-to-trough over ~50 waves is ≈ 1.4 Hs) and is
replaced by the heave-std criterion. Results: §12 and
`sea_state_validation.md`.

**Live Gazebo by hand (once per Gazebo version):**

```bash
ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=$HOME/worlds/<p>/<w> \
    with_control:=true trajectory_file:=<yaml> \
    sea_current:=strong sea_current_from_deg:=90 sea_waves:=slight sea_waves_from_deg:=45
gz topic -e -t /ocean_current                 # ~0.5 m/s westward + orbital ripple
gz topic -f -t /world/generated_ocean/wrench   # ~100 Hz one-shot wrenches
gz topic -e -t /world/generated_ocean/wrench/persistent -d 10   # must stay empty
ros2 topic echo /blueboat/odom --field pose.pose.position.z     # heave ±0.17 m
ros2 topic echo /sim/sea_state                 # the JSON readback MCS shows
```

Acceptance gate for the wrench path: (a) **null test** `sea_waves:=calm
sea_current:=none` — no wave forcing (the hull wrench is ~0 at rest at
the trim; with `hydrostatics: gazebo_box_when_calm` no traffic at all),
odom z at the still-water trim with no drift (a drift is a
torque-reference error); (b) **impulse test**
`slight` for 120 s — mean(z) equal to the trim to < 1 mm and mean(vz) ≈ 0
(a non-zero mean z is a delivery bug, a non-zero mean vz lost impulses);
(c) **motion test** (2026-09-05) `smooth` for 120 s, recording
`/blueboat/odom`: roll and pitch std in 0.5–2.5° (the scripted gate's band), heave std ≈ Hs/4
(the old "peak-to-peak < 0.15 m" was wrong for a 0.15 m Hs, §9), the
heave spectrum peaking near Tp with **no line at 1.6 Hz**
(the 0.6 s ringing of the un-predicted loop), yaw and surge visibly
wave-driven, and `/sim/sea_state` reporting `pose_age_ms` ≤ 60 (the
scripted gate's limit).
Once per Gazebo version: `gz model -m blueboat --link` (canonical link),
the arrow marker's local axis (`marker_arrow_axis`), and the world entity
id used by the injection service (`1` on Harmonic: injected into the stock
`ocean` world live on 2026-10-09). The injection is decided by
`/world/<w>/wrench/persistent`, which only the `ApplyLinkWrench` system
creates — never by `/wrench`, which the launch's own bridge advertises
before the node looks (until 2026-10-09 that skipped the injection in the
stock world: wrenches to nowhere, the hull riding only the orbital drag —
roll/pitch std 0.3/0.2° instead of 1.3/1.7° at `smooth`).

**Indicators.** In the Gazebo GUI: an arrow per active component (blue
current, green waves, pointing where the water goes, length ∝ strength)
and a text summary above the world origin, through gz-gui's
`/marker_array` service (6 s lifetime, refreshed every 2 s, so they vanish
with the node). In RViz: `visualization_msgs/MarkerArray` on
`/sim/sea_state/markers`, frame `world`, riding at the boat. In MCS: the
`SEA STATE` box over the map, fed by `/sim/sea_state` (JSON, latched,
2 Hz) — rows Current / Waves / Boat motion / Next change / Status — with
the *Change sea state…* dialog that publishes on `/sim/sea_state/command`.

## 10. References

1. T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion
   Control*, Wiley, 2011. §8.2 wave spectra and wave-induced forces,
   §8.3 ocean currents (first-order Gauss–Markov model). doi:
   [10.1002/9781119994138](https://doi.org/10.1002/9781119994138)
2. J. N. Newman, *Marine Hydrodynamics*, MIT Press, 1977 (40th-anniversary
   ed. 2018), ch. 6 — Froude–Krylov force, long-wave limit. doi:
   [10.7551/mitpress/4443.001.0001](https://doi.org/10.7551/mitpress/4443.001.0001)
3. K. Hasselmann et al., "Measurements of wind-wave growth and swell
   decay during the Joint North Sea Wave Project (JONSWAP)", *Deutsche
   Hydrographische Zeitschrift*, Ergänzungsheft A8(12), 1–95, 1973 (no
   DOI; hdl: 10013/epic.20654).
4. W. J. Pierson Jr. and L. Moskowitz, "A proposed spectral form for fully
   developed wind seas based on the similarity theory of S. A.
   Kitaigorodskii", *J. Geophys. Res.* 69(24), 5181–5190, 1964. doi:
   [10.1029/JZ069i024p05181](https://doi.org/10.1029/JZ069i024p05181)
5. DNV GL, *Recommended Practice DNV-RP-C205: Environmental conditions
   and environmental loads*, 2017 (revised 2019). §3.5.5 JONSWAP in
   (Hs, Tp) form, §3.5.10 Tp/√Hs band.
6. M. S. Longuet-Higgins, D. E. Cartwright and N. D. Smith, "Observations
   of the directional spectrum of sea waves using the motions of a
   floating buoy", in *Ocean Wave Spectra*, Prentice-Hall, 111–136, 1963
   (cos^{2s} directional spreading).
7. World Meteorological Organization, *Manual on Codes*, WMO-No. 306,
   code table 3700 "State of the sea" (the Douglas sea scale by Hs).
8. M. von Benzon, F. F. Sørensen, E. Uth, J. Jouffroy, J. Liniger and
   S. Pedersen, "An open-source benchmark simulator: Control of a BlueROV2
   underwater robot", *J. Mar. Sci. Eng.* 10(12), 1898, 2022 — the source
   of the hydrodynamic coefficients in `hydrodynamics.xacro`. doi:
   [10.3390/jmse10121898](https://doi.org/10.3390/jmse10121898)
9. D. T. Cobra, A. V. Oppenheim and J. S. Jaffe, "Geometric distortions in
   side-scan sonar images: a procedure for their estimation and
   correction", *IEEE J. Oceanic Eng.* 17(3), 252–268, 1992 — platform
   motion (pitch, yaw, roll, heave) → sonograph distortion. doi:
   [10.1109/48.153442](https://doi.org/10.1109/48.153442)
10. P. Blondel, *The Handbook of Sidescan Sonar*, Springer, 2009. doi:
    [10.1007/978-3-540-49886-5](https://doi.org/10.1007/978-3-540-49886-5)
11. B. Bingham et al., "Toward Maritime Robotic Simulation in Gazebo",
    *Proc. MTS/IEEE OCEANS*, Seattle, 2019 — the VRX wave/buoyancy
    plugins evaluated and not adopted (GPL C++ dependencies, and they
    replace the boat's own plugins, i.e. a change to BlueBoat-Control).
    doi: [10.23919/OCEANS40490.2019.8962724](https://doi.org/10.23919/OCEANS40490.2019.8962724)
13. T. Soomere, "Nonlinear components of ship wake waves", *Applied
    Mechanics Reviews* 60(3), 120–138, 2007 — the shape and grouping of
    ship-wake wave trains in sheltered water, the model behind
    `sea/events.py`. doi: [10.1115/1.2730847](https://doi.org/10.1115/1.2730847)
    (corrected 2026-10-08: the DOI cited before, 10.1007/s10236-007-0113-8,
    does not resolve).
12. Gazebo Sim 8 (Harmonic) system documentation: `Hydrodynamics`
    (`<default_current>`, `/ocean_current`), `Buoyancy`
    (`graded_buoyancy`), `ApplyLinkWrench` (`/world/<w>/wrench*`),
    `EntitySystemAddService` — https://gazebosim.org/api/sim/8/ ; sources
    read at gz-sim tag `gz-sim8`.
14. M. M. M. Manhães, S. A. Scherer, M. Voss, L. R. Douat and T. Rauschenbach,
    "UUV Simulator: A Gazebo-based package for underwater intervention and
    multi-robot simulation", *OCEANS 2016 MTS/IEEE Monterey* — first-order
    Gauss–Markov current plugin (speed + two angles, clamped). doi:
    [10.1109/OCEANS.2016.7761080](https://doi.org/10.1109/OCEANS.2016.7761080)
15. Field Robotics Lab, DAVE project wiki, "Ocean Current" —
    https://github.com/Field-Robotics-Lab/dave/wiki/Ocean-Current (read
    2026-10-08): the UUV Simulator current plus stratified and tidal
    variants; Gauss–Markov update `var = (1 − step·μ) var + noiseAmp·rand`
    per world update (step-size dependent), clamped.
16. D. T. Gillespie, "Exact numerical simulation of the Ornstein–Uhlenbeck
    process and its integral", *Physical Review E* 54, 2084–2091, 1996.
    doi: [10.1103/PhysRevE.54.2084](https://doi.org/10.1103/PhysRevE.54.2084)
17. R. H. Kraichnan, "Diffusion by a random velocity field", *The Physics
    of Fluids* 13(1), 22–31, 1970 — random Fourier modes of a
    divergence-free field. doi: [10.1063/1.1692799](https://doi.org/10.1063/1.1692799)
18. J. Thomson, B. Polagye, V. Durgesh and M. C. Richmond, "Measurements of
    turbulence at two tidal energy sites in Puget Sound, WA", *IEEE J.
    Oceanic Eng.* 37(3), 363–374, 2012 — turbulence intensity typically
    10 % at hub height. doi: [10.1109/JOE.2012.2191656](https://doi.org/10.1109/JOE.2012.2191656)
19. Y. Goda and Y. Suzuki, "Computation of refraction and diffraction of
    sea waves with Mitsuyasu's directional spectrum", *Technical Note of
    the Port and Harbour Research Institute* No. 230, 1975 (no DOI;
    https://www.pari.go.jp/en/1975/12/197512023001.html) — s_max 10 wind
    sea, 75 swell.
20. S. D. Anstee, *The Effects of Towfish Motion on Sidescan Sonar Images*,
    DSTO Materials Research Laboratory MRL-RR-1-94, 1994 (DTIC ADA279252),
    and *… Extension to a Multiple-Beam Device*, MRL-TN-660 (ADA279265) —
    simulated geometric effects of towfish yaw, pitch, heave on the image
    and on seabed coverage; yaw the most damaging; "backscanning" as a
    local distortion measure. Full text not retrieved (DTIC refused);
    used from the abstracts and contents.
21. C. Lei, V. Franchi, H. Rajani, N. Gracias, R. Garcia and H. Wang,
    "Geometric correction of side-scan sonar images with image-consistent
    attitude refinement", arXiv:2604.19901, 2026 (preprint) — pitch as
    the port/starboard common mode, yaw the differential mode.
22. Y. Kim and J. Ryou, "A study of sonar image stabilization of unmanned
    surface vehicle based on motion sensor for inspection of underwater
    infrastructure", *Remote Sensing* 12(21), 3481, 2020 — on a USV waves
    and swell distort the sonar data; per-ping roll correction. doi:
    [10.3390/rs12213481](https://doi.org/10.3390/rs12213481)
23. J. E. Hughes Clarke, "Dynamic motion residuals in swath sonar data:
    ironing out the creases", *International Hydrographic Review* 4(1),
    6–23, 2003 (no DOI) — wave-period attitude residuals ("wobbles") in
    surface-vessel swath sonar.
24. T. Sheffer and H. Guterman, "Geometrical correction of side-scan sonar
    images", *2018 IEEE ICSEE* — yaw, pitch, speed and position correction
    from AUV navigation. doi: [10.1109/ICSEE.2018.8646188](https://doi.org/10.1109/ICSEE.2018.8646188)
25. M. Paravisi, D. H. Santos, V. Jorge, G. Heck, L. M. Gonçalves and A.
    Amory, "Unmanned surface vehicle simulator with realistic environmental
    disturbances", *Sensors* 19(5), 1068, 2019 — waves, wind and current
    degrade USV trajectories in simulation. doi:
    [10.3390/s19051068](https://doi.org/10.3390/s19051068)
26. K. K. Kahma and C. J. Calkoen, "Reconciling discrepancies in the observed
    growth of wind-generated waves", *J. Phys. Oceanogr.* 22(12), 1389–1405,
    1992 -- eqs (2)–(3), (6)–(7): fetch-limited energy and peak frequency,
    stable / unstable stratification (read first-hand, 2026-11-10). doi:
    [10.1175/1520-0485(1992)022<1389:RDITOG>2.0.CO;2](https://doi.org/10.1175/1520-0485(1992)022<1389:RDITOG>2.0.CO;2)
27. T. I. Fossen, *Lecture Notes TTK4190 Guidance, Navigation and Control of
    Vehicles*, ch. 10 "Environmental Forces and Moments" (slides; the book's
    ch. 10, doi:10.1002/9781119575016.ch10, was not available) -- slide 34:
    A_k = √(2 S(ω_k) Δω), repetition after 2π/Δω, ω_k drawn inside each
    interval; MSS example 100 frequencies.
28. ITTC Recommended Procedures 7.5-02-07-01.1 "Laboratory modelling of
    multidirectional irregular wave spectra" (2017) and 7.5-02-07-02.1
    "Seakeeping experiments" (2024) -- component counts, repetition period,
    f_T/f_P > 2 (prefer 3). https://www.ittc.info (no DOI)
29. P. Mérigaud and J. V. Ringwood, "Free-surface time-series generation for
    wave energy applications", *IEEE J. Oceanic Eng.* 43(1), 19–35, 2018 --
    against equal-area discretisation schemes. doi:
    [10.1109/JOE.2017.2691199](https://doi.org/10.1109/JOE.2017.2691199)
30. J. M. J. Journée and W. W. Massie, *Offshore Hydromechanics*, TU Delft,
    2001 (open courseware, no DOI) -- eq. 8.43, pressure decay with depth;
    pp. 6-23/24, a body follows the waves only for λ ≫ its length.
31. D. Tamsett and P. Hogarth, "Sidescan sonar beam function and seabed
    backscatter functions from trace amplitude and vehicle roll data",
    *IEEE J. Oceanic Eng.* 41(1), 2016 -- roll modulates trace amplitude
    through the rotated beam function (our banding mechanism, §7). doi:
    [10.1109/JOE.2015.2390732](https://doi.org/10.1109/JOE.2015.2390732)

Every DOI above was resolved at doi.org and its title checked against
Crossref on 2026-10-08; refs 1-25 re-checked and 26-31 added on 2026-11-10.

## 11. Research log — where each modelling decision comes from

Why the implementation looks the way it does. "Borrow" is what we took;
"differ" is what we deliberately do otherwise, and why that is the right
simplification for a simulator whose job is to make SSS acquisition
realistically hard, reproducibly.

### 11.1 Current

| Source | What it does | What we borrow | What we do differently | Why |
|---|---|---|---|---|
| Fossen [1] §8.3 | Current speed and direction as saturated first-order Gauss–Markov processes about a mean | The model itself: mean + GM fluctuation, saturation | Exact discretisation on a fixed grid, C1 interpolation, smooth (tanh) bound, unit processes scaled by σ | A per-tick Euler step is neither reproducible under a jittery timer nor smooth; the statistics are the same at the knots |
| UUV Simulator [14], DAVE [15] (Gazebo plugins) | The same GM per world update, clamped; DAVE adds depth-stratified and tidal currents | That a Gazebo marine simulator models current this way — the established practice | Seed-keyed, step-independent realisation; no stratification (a 1.2 m surface boat sees only the surface layer); no tidal harmonics (minutes-long surveys) | The thesis compares policies across runs: same seed must mean same water |
| Gillespie [16] | Exact OU update for any step | The AR(1) recursion with a = e^{−Δ/τ} | — | Exactness at any knot spacing |
| Kraichnan [17] | Random Fourier modes give a divergence-free random velocity field | Stream-function modes, 8 of them | Harbour scale (25–100 m) and slow phase drift chosen by judgement | Incompressible by construction, cheap, bounded; makes adjacent / return lines differ repeatably — no site data exists, so it is labelled a judgement |
| Thomson et al. [18] | Turbulence intensity ~10 % at tidal-energy sites | "Natural" variation σ_V = 10 % of U | Applied to a quieter harbour as an upper-bound-ish typical value; the slow GM part only | The only measured scale found; the transfer is stated |

### 11.2 Waves and the hull

| Source | What it does | What we borrow | What we do differently | Why |
|---|---|---|---|---|
| Hasselmann et al. [3], DNV-RP-C205 [5] | JONSWAP spectrum in (Hs, Tp); Tp/√Hs band | Spectrum, variance, the Tp ∝ √Hs scaling (auto period) | γ = 1.5 (young basin sea), presets above the pure wind-sea band | Fetch-limited harbour chop with swell/wake energy |
| Longuet-Higgins et al. [6]; Goda & Suzuki [19] | cos^{2s}(Δθ/2) spreading; s_max 10 wind sea, 75 swell | The spreading form; "long-crested" s = 25 | Frequency-independent s | Plain choice (short chop / long swell) without a frequency-dependent model |
| Soomere [13] | Wake wave trains in sheltered water | Group shape (a few long-period cycles, rising and decaying) | Poisson arrivals and Hann envelope are assumptions | No wake census at the site |
| Newman [2]; Fossen [1] §8.2 | Froude–Krylov force; long-wave RAO limit "the body follows the surface" | Hydrostatic FK on the catamaran hull + first-order horizontal FK | No radiation/diffraction beyond the plugin's added mass and the hull damping | L/λ small; validated in Gazebo: heave std = Hs/4 at the wave period (§12) |
| VRX, Bingham et al. [11]; Paravisi et al. [25] | USV simulators with PM waves, buoyancy, wind, current | Confirmation that environmental forcing visibly degrades USV tracks | Not adopted as code (GPL C++, replaces the boat's own plugins = a change to BlueBoat-Control) | CM-3 / NC #8 |
| Robotics simulators (2026-11-10 survey): VRX (3 Gerstner components, buoyancy at 2 points per hull), gz-waves / asv_wave_sim and Stonefish (FFT seas, hydrostatic pressure per hull-mesh triangle), MARUS (Crest + per-triangle forces), USVSim (osgOcean FFT, per-link buoyancy); HoloOcean and DAVE: no physical waves | Hydrostatic / Froude–Krylov pressure on a hull representation under a synthesized sea, radiation mostly ignored | That this family is the established robotics practice (ours is the same, seeded and validated) | A JONSWAP spectrum resolved to 3 Hz instead of 3 components or a visual FFT grid; the draft decay most of them omit | Real-time cost vs fidelity; ship simulators (Fossen MSS) instead superpose RAOs from panel codes, which a 1.2 m catamaran without hydrodynamic coefficients cannot use |
| Hasselmann et al. [3]; Kahma & Calkoen [26]; DNV [5] | Fetch-limited growth of Hs and Tp | Tp = 3.6 √Hs for every level (2026-11-10) | One rule for all levels; the wind/fetch is not an input | The corpus fixes the site's sea only through the boat's motion |
| ITTC [28]; Mérigaud & Ringwood [29]; Fossen [27] | Spectrum discretisation and component counts | Resolve the response band; avoid wide tail bins | 24 × 12 components (cost-bound) instead of ≥ 100 frequencies | Checked by slope coverage, not by count |
| Journée & Massie [30] | Pressure decay with depth; small-body limit | Draft decay at the mean draft | One depth per column, no diffraction | λ ≈ L at 1 Hz: the whole-body small-body limit fails, the column integration keeps the averaging |

### 11.3 The side-scan sonar on a moving surface platform

| Phenomenon | Source (platform) | What it demonstrates | How it maps here | Reproduced? | Limitation of the comparison |
|---|---|---|---|---|---|
| Geometric distortion from yaw, pitch, heave, sway | Cobra et al. [9] (towfish); Blondel [10] | Sonograph distortion follows the platform's attitude history | The renderer's per-ping geometry follows the pose | Yes (by construction, measured §12) | Towfish dynamics differ from a hull; the geometry is the same |
| Yaw the most damaging; back-scanning (seabed imaged again); coverage effects | Anstee [20] (towfish simulation) | Yaw at range repeats or skips seabed | Footprint back-scan / gap fractions, pings per target | Yes (§12) | Abstract-level source |
| Pitch = common mode, yaw = differential mode between port and starboard | Lei et al. [21] (real SSS datasets, preprint) | The two sides separate the two motions | `footprint_metrics` common / differential std | Yes (smoke `[10]` + §12) | Preprint |
| USV-specific: waves and swell distort the sonar data; roll correction per ping | Kim & Ryou [22] (USV) | Surface vehicles are more exposed than ships | The whole chain, surface platform | Yes (roll banding, smoke `[10]`) | Their sonar was towed; ours hull-mounted |
| Wave-period attitude residuals in swath data | Hughes Clarke [23] (surface vessel, multibeam) | "Wobbles" at the wave period | Heave/roll at the wave period reach every ping | Yes (attitude spectra §12) | Multibeam, not SSS |
| Attitude-based geometric correction | Sheffer & Guterman [24] (AUV) | Navigation + attitude can undo much of it | Our "naive mosaic" deliberately omits it | Demonstrated by omission | AUV |
| Current-induced track deviation and crab | Fossen [1]; Paravisi et al. [25] | Cross-current lines need a crab angle; track error grows | Track metrics in Gazebo | Yes (§12) | Our BlueROV2 damping coefficients (S6) |
| Intra-ping smear | Preston & Gorton (IOA, multibeam sidescan stability criterion) | Fast attitude change within a ping smears it | — | **Not modelled** (S12) | Estimated ≤ half a footprint at 10°/s |

## 12. Validation summary (2026-10-09)

The same survey (two 70 m lines, seed 7) in calm water, a 0.3 m/s
cross-current, *Light chop* and both, run in Gazebo by
`test/gazebo_sea_test.py` on a 5 m and a 1.5 m world, every scenario on the
catamaran hull and the 100 Hz pose — every number, figure and the
step-by-step recipe in [`sea_state_validation.md`](sea_state_validation.md).
In brief:

* **Model checks:** calm carries no wave forcing and stays level (roll
  std 0.00°, no ringing); the realised current matches its mean within
  the run's statistics and never leaves its bound; heave std = Hs/4
  (0.035–0.039 m vs 0.0375) at the wave period (2.6–2.9 s vs Tp 2.8), mean
  hull height = calm's to 0.8 mm (impulse conserved), no 1.6 Hz line, no
  3.2 Hz roll ring; roll/pitch std 1.4–1.7°, inside the gate's 0.5–2.5°
band and the wider field band of `sea_state_validation.md` (above this
site's 0.9–1.5° chop, as the longer preset Tp predicts); the
  same trim in every scenario (−0.15 ± 0.02°). 5 m world 25/25, 1.5 m
  world 24/24 + 1 INFO.
* **Sonar consequences:** at 5 m, 15–19 % of pings step back over imaged
  seabed and target images keep about half their similarity to the calm
  survey (correlation 0.47) without moving their mean position; with the
  current too, 23 % of target passes get no ping; at 1.5 m pitch hardly
  moves the near-range footprint (0.03 m) but yaw still back-scans 8 % of
  the far swath — the altitude law of §7 at work.
* **Against the real boat:** roll/pitch, their rates and the implied
  footprint wobble sit inside the field band; the simulated **yaw rate is
  4–10× lower** than the field's (the yaw-driven effects are therefore
  conservative); the real boat's static per-session trim offset is not
  modelled (§5).
