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
references are at the end (§10).

---

## 1. Scope and boundary

The sea state acts on the **hull only**. Nothing else in the world moves:
the seabed, the litter objects and the walls are static, the acoustic
scene stays static during a run (`sonar_model.md` A5), and the water
surface stays a flat visual plane in Gazebo. What the boat feels and what
the sonar sees changes; the ground truth does not.

Two mechanisms, both external to the boat's own model, so **no file of
BlueBoat-Control is modified** (CM-3 / NC #8):

| Effect | Path into Gazebo | Who applies it |
|---|---|---|
| Current + wave orbital velocity | ROS `/current` → gz `/ocean_current` (the bridge both launch paths already create) | the boat's own `gz::sim::systems::Hydrodynamics` plugin, through its relative-velocity damping term (`hydrodynamics.xacro`) |
| Wave excitation wrench | ROS `/world/<w>/wrench` → gz `ApplyLinkWrench` (in every generated world since 2026-09; injected at runtime into the stock world and older folders) | gz-sim, `Link::AddWorldWrench`, one physics step per message |

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

* **The buoyancy box has 1 % reserve buoyancy.** `buoyant_correction =
  1.01·((16.01 + 2·0.07) / (1000·1.2·0.91·0.376))^{1/3} = 0.3434`, so the
  box is 0.412 × 0.313 × 0.129 m and floats 99 % submerged (waterplane
  0.129 m², heave stiffness 1263 N/m, trim: link origin at z = −0.111 m).
  Under a wave crest it can gain at most 1 % of the weight; under a
  trough it loses most of its lift. A wave force computed for *that* box
  would make the boat ride awash and sink by ~Hs/2 on average — an
  artefact of the URDF, not of the water. §5 says how the model avoids
  inheriting it.
* **The damping and added-mass coefficients are the BlueROV2's**
  (`hydrodynamics.xacro` cites von Benzon et al. 2022 [8]; no BlueBoat
  identification exists in the project — `BlueBoat-Control/.claude/TODO.md`).
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

Fossen [1] §8.3 models a current as mean speed and direction plus
first-order Gauss–Markov (Ornstein–Uhlenbeck) fluctuations,

    V̇_c + μ V_c = w(t),   |V_c| ≤ V_max ,

with white noise `w` and saturation. `sea/current.py` implements exactly
that for the speed perturbation `dv` and the direction perturbation
`dβ` (Euler–Maruyama step, stationary standard deviations `σ_speed`,
`σ_dir`, time constants `τ = 1/μ` of 60 s and 120 s, saturation at ±2σ),
about the preset's mean and the operator's compass direction. The world-
frame velocity is `V (cos θ, sin θ, 0)` with θ the ENU angle of the
*toward* direction (`core/geometry.compass_deg_to_enu_yaw(from + 180)`).

Direction convention: the user's **"from"** (meteorological, like wind:
"from North to South"). Oceanographers report the *set* (toward); the
readback and the markers carry both (`from: NE, toward: SW`) so the two
never get confused.

The current is **spatially uniform** over the world (S2). Harbour basins
have eddies and shear at the mouth; a 2-D field (or Gazebo's
`lookup_current_*` environment table) is a roadmap item.

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

**Peak period per preset**: the DNV-RP-C205 [5] §3.5.10 steepness band
`3.6 √Hs ≤ Tp ≤ 5 √Hs` describes a pure wind sea; a harbour basin open to
a channel carries swell and wake energy on top, so the presets sit above
that band (up to `10 √Hs` for the smallest ripples riding a longer
swell): rippled 2.0 s, smooth 2.8 s, slight 3.6 s, rough 4.5 s. The
`tp_from_hs_dnv` helper still returns the pure wind-sea mid-band value for
reference.

### 4.1b Random wave groups (`sea/events.py`)

What a 1.2 m boat feels most in a harbour is not the stationary chop but
the **groups that arrive at random**: a passing vessel's wake (a few
cycles of 4–8 s waves, minutes apart — Soomere 2007 [13]), a swell train
leaking through the entrance, a gust-raised group. Each wave preset
carries an `events:` block `{rate_per_hour, hs_m, tp_s, duration_s,
from_spread_deg}`: arrivals are a **Poisson process** (exponential
inter-arrival times, mean `3600 / rate_per_hour`), each event its own
short directional train — a 4 × 1 `WaveField` with Tp jittered ±15 %, a
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

`sea/wavefield.py` discretises the spectrum into `N_f = 16` frequency
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

### 4.3 Presets

`config/sea_states.yaml`, placed on the Douglas / WMO 3700 sea-state
code [7] by Hs, harbour-realistic plus one stress level:

| Waves | Hs [m] | Tp [s] | γ / s | wake groups (per h, Hs, Tp, dur) | Douglas | Current | V̄ [m/s] | σ_V | σ_dir |
|---|---|---|---|---|---|---|---|---|---|
| calm | 0 | — | — | none | 0 | none | 0 | 0 | 0 |
| rippled | 0.05 | 2.0 | 1.5 / 6 | 6, 0.08 m, 5 s, 25 s | 1 | weak | 0.10 | 0.02 | 8° |
| smooth | 0.15 | 2.8 | 1.5 / 6 | 12, 0.15 m, 5.5 s, 30 s | 2 | moderate | 0.25 | 0.05 | 10° |
| slight | 0.35 | 3.6 | 1.5 / 6 | 20, 0.25 m, 6 s, 35 s | 3 (lower) | strong | 0.50 | 0.08 | 10° |
| rough (stress) | 0.60 | 4.5 | 1.5 / 6 | 30, 0.35 m, 7 s, 40 s | 3 (upper) | very_strong (stress) | 0.80 | 0.12 | 12° |

A **custom** wave state (Hs, Tp, γ, wake-group rate/height/period) is
set from the MCS dialog: it travels as the explicit numeric fields of a
one-keyframe schedule at launch, or of a live command.

The file carries a label, a one-sentence consequence for the boat and the
sonar, and a reference per preset; MCS shows them as tooltips. Directions
are chosen per run, not stored. `rippled`'s peak wavelength (1.6 m) is
about the hull length: the footprint integration of §5 averages it out
correctly, so it is a legitimate "barely anything" level.

## 5. Wave → hull coupling (Froude–Krylov, hydrostatic)

For a body much smaller than the wavelength the wave force is dominated
by the pressure the undisturbed wave would exert on the hull — the
Froude–Krylov force; diffraction and radiation are higher order in
`L/λ` (Newman 1977 [2] ch. 6; the long-wave limit of the response
amplitude operator is "the body follows the surface"). At the BlueBoat's
scale (L = 1.2 m, λp = 4.5–17 m for the presets) and draft (0.045 m ≪
λ, so the exponential depth decay is 1), the Froude–Krylov force is the
**hydrostatic buoyancy under the instantaneous wave surface**.

`sea/hull.py` integrates it over the hull footprint: columns over each
hull box in the body frame, rotated to the world by the boat's attitude,
each submerged by `clip(η(x, y, t) − z_bottom, 0, z_top − z_bottom)`;
force `ρ g Σ h dA` upward at the column centroids; moment about the
**link origin** in world axes (the bridge drops the gz `force_offset`
field, so `ApplyLinkWrench` applies the force at the link origin — a
moment taken about any other point would be a slow, mystifying drift).

**Which hull.** The wrench published while waves are active is

    F = H_cat(η) − H_gz(0) ,

`H_cat` the buoyancy of a **catamaran hydrostatic hull** (two demihulls
1.2 × 0.15 × 0.376 m, 0.76 m apart — the BlueBoat 120 in the round —
placed so that their draft at the Gazebo trim reproduces the
displacement: waterplane 0.36 m², draft 0.045 m) under the wave surface,
and `H_gz(0)` the Gazebo buoyancy box (mirrored from the xacro formula)
under the flat plane, i.e. exactly what Gazebo itself keeps applying. The
sum on the hull is therefore `H_cat(η)`: the boat floats on the
catamaran's hydrostatics under the actual surface. Both hulls share the
trim by construction, so the replacement is static-force-free, and on
calm water the model publishes nothing at all — Gazebo's own hydrostatics
stand untouched (the strict no-op the presets promise; `[10]` checks the
excitation is *exactly* zero on flat water).

Resulting stiffness and natural periods (URDF inertia, the plugin's
added-mass diagonal, `HullBoxes.restoring_summary`, `[10]`):

| Hull | A_wp [m²] | C33 [N/m] | C44 [N·m/rad] | C55 [N·m/rad] | T_heave | T_roll | T_pitch |
|---|---|---|---|---|---|---|---|
| Gazebo box (calm water) | 0.129 | 1263 | 39 | 5–46* | 0.98 s | 1.6 s | 1.9–5 s* |
| catamaran (waves active) | 0.36 | 3530 | 577 | 443 | 0.59 s | 0.42 s | 0.60 s |

\* the 99 %-submerged box's pitch stiffness is strongly non-linear about
the trim (its waterplane engages only as an end emerges). With natural
periods well below the 1.0–3.3 s wave band and the ROV-class damping, the
catamaran **contours** the surface quasi-statically: heave ≈ η, pitch and
roll ≈ the local wave slope along and across the heading — the physically
expected behaviour of a small catamaran in harbour chop.

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
(`sea/motion.py::simulate_3dof`) reproduces a **0.63 s heave bounce**
of 0.27 m peak-to-peak at 'smooth' with half the heave power above 1 Hz
— the regular, too-frequent "wave" reported in the 2026-09-05 review —
while the same model fed a 100 Hz pose simply contours the sea.
`sea_state_node` therefore propagates the last odometry (pose + the
body-frame twist Gazebo's `OdometryPublisher` sends) to the tick time
with `predict_pose` before evaluating the wrench, and reports the pose
age in the status JSON (`pose_age_ms`, additive).

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
* **Sim time.** Timers run on `/clock`, the wave phase uses the current
  sim time and the boat position the last odom (≤ 50 ms old: ≤ 5 cm at
  1 m/s against wavelengths of metres). The field is a deterministic
  function of `(seed, t_sim, x, y)`: a re-run reproduces the same sea.

## 7. Effects on the side-scan sonar

The renderer (`sonar/renderer.py`, `sonar_model.md` §4) receives the
6-DOF pose per ping and couples it as a real platform does
(Cobra, Oppenheim & Jaffe 1992 [9]; Blondel 2009 [10] ch. 4):

| DOF | Mechanism | Visible as |
|---|---|---|
| roll | tilts the vertical beam pattern toward one side (`acoustics.beam_weight`) | port/starboard brightness banding along track — the surface-platform signature (thesis contribution C4) |
| heave | changes the altitude, slant geometry, water column, surface-multipath delay | altitude ripple, FBR wobble, water-column band breathing |
| pitch | rotates the fan plane about the body y axis: each ground sample at depth `dz` below the transducer images `dz·tan(pitch)` along-track from where a level fan puts it (since 2026-09-02) | symmetric port/starboard along-track compression/expansion of features; `[10]` measures 0.30 m at 5° and 3.5 m altitude, the same on both sides |
| yaw / yaw rate | per-ping look direction — now driven by the wave force too (§5.5) | antisymmetric along-track distortion (one side advances, the other lags) |
| surge / sway | per-ping position — now driven by the wave force too (§5.5) | irregular along-track sampling, crab angle on lines across the current |

Roll reaching the image only through the beam pattern is the physical
truth of a side-looking transducer (its nadir geometry moves by the
mount lever arm × sin roll, millimetres): a rolling boat shows
brightness banding, a pitching, heaving, surging and yawing one shows
geometry. Before 2026-09-05 the field was long-crested about one world
axis and the boat rang in heave, so a run read as "pitch only"; §5.5–5.7
are what changed.

Not modelled: intra-ping motion (A5, stop-and-hop pings), the rotation of
the ±0.20 m mount lever arm by roll/pitch (3 mm at 10°, below the ground
sample), and the motion-induced Doppler on the pulse.

## 8. Assumptions

* **S1 — linear waves, single spectrum.** Airy components with random
  phases; no breaking, no swell partition, no wave–current interaction.
  Fine for Hs ≤ 0.6 m in 3–10 m of water.
* **S2 — spatially uniform current, damping-only.** One vector over the
  world; enters the plugin's damping term only (the plugin's design).
* **S3 — flat visual surface.** Gazebo's water plane does not move; the
  boat moves on an invisible surface. The indicators (§9) show the state.
* **S4 — hydrostatic Froude–Krylov on a box catamaran.** No diffraction,
  no radiation beyond the plugin's added mass and damping plus the
  linear hull damping of §5.7, no Smith-effect depth decay, pressure
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
* **S9 — wave groups are a marked Poisson process.** Arrival rate,
  height, period and duration per preset are judgement values sized to a
  small harbour with traffic; no wake census exists for the site. The
  group shape (a Hann-enveloped 4–8 s train) follows the wake literature
  [13]; the arrivals do not.

## 9. Verification and indicators

**Offline (`python3 -m test.smoke_test`, section `[10]`, ~40 checks):**
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

**Real-boat anchor (corpus `ATTITUDE`, 25 Hz, seven Shiraishi-jima
recordings, 2026-09-05):** roll std 0.9–1.3°, pitch std 0.8–1.5° about a
+2° bow-up trim, dominant period 1.0–1.2 s (short wind chop), rates
5–10 °/s. That band is the live pass criterion below; the presets' Tp
are longer than the corpus's chop by design (the user's choice was to
keep the preset levels), so compare the *amplitudes*, not the period.

**Live Gazebo (manual, once per model change):**

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
sea_current:=none` — no wrench traffic, odom z at the still-water trim
with no drift (a drift is a torque-reference error); (b) **impulse test**
`slight` for 120 s — mean(z) equal to the trim to < 1 mm and mean(vz) ≈ 0
(a non-zero mean z is a delivery bug, a non-zero mean vz lost impulses);
(c) **motion test** (2026-09-05) `smooth` for 120 s, recording
`/blueboat/odom`: roll and pitch std in 0.5–2°, heave peak-to-peak
< 0.15 m, the heave spectrum peaking near Tp with **no line at 1.6 Hz**
(the 0.6 s ringing of the un-predicted loop), yaw and surge visibly
wave-driven, and `/sim/sea_state` reporting `pose_age_ms` ≤ 50.
Once per Gazebo version: `gz model -m blueboat --link` (canonical link),
the arrow marker's local axis (`marker_arrow_axis`), and the world entity
id used by the injection service.

**Indicators.** In the Gazebo GUI: an arrow per active component (blue
current, green waves, pointing where the water goes, length ∝ strength)
and a text summary above the world origin, through gz-gui's
`/marker_array` service (6 s lifetime, refreshed every 2 s, so they vanish
with the node). In RViz: `visualization_msgs/MarkerArray` on
`/sim/sea_state/markers`, frame `world`, riding at the boat. In MCS: the
`SEA STATE` box in the left panel, fed by `/sim/sea_state` (JSON, latched,
2 Hz), with the *Modify situation…* dialog that publishes on
`/sim/sea_state/command`.

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
13. T. Soomere, "Nonlinear ship wake waves as a model of rogue waves and
    a source of danger to the coastal environment: a review", *Oceanologia*
    49(1), 2007 — the shape and grouping of ship-wake wave trains in
    sheltered water, the model behind `sea/events.py`. doi:
    [10.1007/s10236-007-0113-8](https://doi.org/10.1007/s10236-007-0113-8)
12. Gazebo Sim 8 (Harmonic) system documentation: `Hydrodynamics`
    (`<default_current>`, `/ocean_current`), `Buoyancy`
    (`graded_buoyancy`), `ApplyLinkWrench` (`/world/<w>/wrench*`),
    `EntitySystemAddService` — https://gazebosim.org/api/sim/8/ ; sources
    read at gz-sim tag `gz-sim8`.
