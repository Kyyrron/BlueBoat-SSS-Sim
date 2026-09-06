# TODO — BlueBoat-SSS-Sim

Open items only. Settled facts live in `CLAUDE.md`.

Priority marks: **[P0]** blocks use of the package · **[P1]** correctness or
cross-module risk · **[P2]** realism / project value.

---

## World builder (2026-09 rework) — open points

### [P1] Live Gazebo + GUI pass on a builder world
The whole rework is verified by the offline suite (~150 checks) and the
offscreen GUI smoke, but no builder world has yet been opened in a real
Gazebo Harmonic session on this machine, and the wizard has not been
driven interactively. One session of: `world_builder` → build a world from
`wakamatsu_beach` → `sim_world_launch.py world_dir:=…` (walls visible,
limit borders invisible but colliding) → `sss_sim_launch.py` + ping enable
(GCS anchors at the wakamatsu coordinates via the fixed shim) →
`full_mission_launch.py with_control:=true`.

Add, since 2026-09-06: build one world in **recording** depth mode from
`~/ros2_ws/data/SSS_data/sessions/2026_09_04-09_57_09/…svlog` over
`plage_bathymetry1` (the pair that overlaps), and check the Screen-3
coverage line against the seabed the preview draws. That session's first
attempt failed with "no depth sample lands inside the world limits" from
pairing the recording with a path anchored elsewhere — the message now
names both places, but only a live pass says whether that is enough for
the operator.

### [P1] Live Gazebo pass with the sea state (re-run after 2026-09-05)
`sea_state_node` is verified offline (smoke `[10]`: spectrum, hull,
current, schedules, launch resolvers, and since 2026-09-05 the closed
loop under a 20 Hz pose hold) and by importing under rclpy. The user's
live runs before 2026-09-05 showed the boat ringing at ~1.6 Hz ("too
powerful, too regular, too frequent, pitch only"): the offline loop
attributes that to the 20 Hz odometry feeding the restoring spring at
100 Hz, now fixed by pose prediction + hull damping (`sea/motion.py`,
`docs/sea_state_model.md` §5.5–5.8). The gate in §9 has to be run once
on this machine: (a) null test `sea_waves:=calm sea_current:=none` —
`gz topic -e -t /world/<w>/wrench` silent, odom z at the trim with no
drift; (b) impulse test `slight` over 120 s: mean z at the trim to
< 1 mm, mean vz ≈ 0, `gz topic -f` ≈ 100 Hz; (c) **motion test**
`smooth` over 120 s recording `/blueboat/odom`: roll/pitch std 0.5–2°,
heave p2p < 0.15 m, no 1.6 Hz heave line, `/sim/sea_state`
`pose_age_ms` ≤ 50; the MCS "SEA STATE" box live and "Modify situation…"
ramping; the stock-world companion path (`world_name:=ocean`, runtime
`ApplyLinkWrench` injection — the world entity id 1 and the marker arrow
axis are still unverified live). Also confirm `gz model -m blueboat
--link` names the canonical link the wrench targets.

### [P2] Boat may be too light; hull damping is a judgement value
The user's hint on 2026-09-05. The simulated mass (16.01 kg), the
BlueROV2 added mass (heave 14.57 kg) and every damping coefficient are
`blueboat_description`'s (`blueboat.xacro`, `hydrodynamics.xacro`,
CM-3) and the odometry rate (20 Hz) too. The measured self-excitation
was the pose hold, not the mass; but a surveying BlueBoat with two
sonars, batteries and the USBL is ~20 kg, and the extra `hull.damping`
(ζ ≈ 0.4: 245 N·s/m heave, 20 / 13 N·m·s roll / pitch,
`config/sea_states.yaml`) is sized, not measured. A decay test on the
real boat (or a BlueBoat identification in BlueBoat-Control) would anchor
both; revisit `hull.damping` when one exists.

### [P2] Wave-group statistics are a judgement value
`events:` rates / heights / periods per preset (sea/events.py) are sized
for a small harbour with traffic, with no wake census for the site;
a day of counting passing vessels at the study basin would anchor them.
The Hann envelope and the Poisson arrivals are assumptions (S9).

### [P2] Backscatter reflectivity hint is uncalibrated
`depth.svlog.backscatter_layer` (2026-09-05) derives a 0–1 mid-swath
level map from the recording after removing the file's own median range
law; it is relative, gain-dependent (auto-gain moves it between pings)
and never assigned by itself — the "Paint materials from backscatter"
button thresholds it into mud/gravel dots the user can edit. The whole
material spread the renderer can produce is ~3.7 dB against an 18 dB
range gradient (known limitations below), so treat the layer as a
sketch. A recording at fixed gain over a known bottom would let it be
scaled.

### [P2] Curved walls: Gazebo shows chords
A bent wall is flattened to ≤ 0.1 m sagitta chords; in Gazebo they are
that many boxes with slight overlaps at the joints (visual only). The
depth barriers and the sonar see the same chords, so nothing disagrees.

### [P2] Objects v3: lattice classes on their side
A bicycle / crate / trolley on one of its faces keeps its yaw-frame
lattice thinning over the *projected* box footprint; the skeleton is not
re-derived from the rotated frame members. Fine at raster resolution,
noted so it is not "rediscovered".

### [P2] Pontoon face level is a judgement value
`wall_pontoon_specular` 0.15 / `wall_face_diffuse` 0.5 have no corpus
anchor (no pontoon geometry ground truth in the field logs); the quay
numbers are measured (`docs/sonar_model.md` §11).

### [P2] Projected-CRS GeoTIFFs are rejected
`worldgen/depth/ingest.py` reads geographic-CRS (degree) tiepoint+scale
GeoTIFFs through Pillow and rejects UTM/projected files with an actionable
message. Adding `pyproj` would lift this; until then, re-export in
EPSG:4326 or convert to CSV.

### [P2] Cone visual is a cylinder stand-in
SDF `<cone>` exists only from SDFormat 15 (Ionic); on Harmonic
`traffic_cone` renders as an upright cylinder at half the base radius
(`sdf_writer._object_sdf`). Acoustics are mask-driven and correct either
way. Swap to a real `<cone>` when the workspace moves past Harmonic.

### [P2] EPS reflectivity is a judgement call
`eps` ships at 0.80 (air-backed closed-cell foam reflects strongly — the
water/air impedance step is near-total), which is counter-intuitive for
"foam" and unmeasured. Flag from a field capture of an EPS fish box.

### [P2] Depth flood fill ignores wall height
Every wall blocks the nearest-value extension regardless of `top_z` — a
low pontoon stops depth continuation exactly like a quay. Simple and
predictable; revisit only if a submerged-sill case turns up.

### [P2] Explicit object placement has no GUI yet
`objects.explicit` (verbatim instances — matched pairs, RTK-fixed
clusters, the experimental core of `target_families_and_surrogates.md`) is
implemented and smoke-tested in the placement engine and reachable from
world YAML / `builder_state.yaml`, but Screen 4 does not surface it.
Placing instances by clicking the Screen-2 map is the natural upgrade.

---

## Cross-module correctness

### [P1] Processor-written svlogs carry a frozen `time_boot_ms`
Three of the seven corpus files (those with `src = 3`, i.e. written by
`sss_processor_node`) carry one constant `time_boot_ms` (~4.27e9) and
`hdg = 0` on every navigation envelope, so no clock skew can be
estimated and no ping can be interpolated onto the fix track. The
simulator's reader now detects it (`svlog_reader.read_svlog` →
`clock_frozen`) and pairs by file order, as the GCS does — within one
ping period at 25 Hz — and the depth-from-recording quality readout says
`clock file order`. The fix is BlueBoat-SSS's (`_build_global_position_
envelope` and its LOCAL_POSITION_NED twin should stamp the autopilot's
`time_boot_ms`, CM-3); until then every processor recording pays a
≤ 5 cm placement error.

### [P1] Transducer lateral offset disagrees across modules
The simulator mounts transducers at `mount_y_abs_m: 0.20`
(`sonar/config.py`, `config/default_sonar.yaml`); `sss_processor_node.py`
still has all four of `TRANSDUCER_X_OFFSET_M`, `TRANSDUCER_Y_OFFSET_PORT_M`,
`TRANSDUCER_Y_OFFSET_STBD_M` and `TRANSDUCER_SUBMERSION_M` at `0.0` with their
"TODO: measure on the physical BlueBoat" comments intact. Until both use the
same numbers, slant-range-corrected geometry from simulated data carries a
systematic across-track bias.

**Needs a physical measurement, and there is no second source for it**:
`blueboat_description` ships a `sonar_snippets.xacro` but never instantiates it
on the BlueBoat model, so the URDF does not carry the geometry either. Measure
on the hull, then set both repositories (the `BlueBoat-SSS` half is that
repo's change — CM-3) and record who measured it, on which hull, from which
reference point. Do not pick a number to make the two agree: an
agreed-but-wrong offset is worse than a recorded disagreement.

### [P2] 1200-bin handling in the GCS is unverified
The ROS half is settled: at `num_results:=1200` the wire contract, throughput and
`sss_processor_node` all hold (recorded in `CLAUDE.md` §6). What was not exercised
is the **visualization app**. `blueboat_gcs` reads `num_results` from the packet
everywhere it matters (`core/svlog.py` sizes its payload as
`_HEAD_SIZE + 2 * num_results`) and hard-codes no bin count, so it should be
bin-count-agnostic by construction — but that is a code reading, not a run. It is
a Qt app whose acquisition starts from a button, so confirming it needs an
interactive session, and the fix for anything found there is `BlueBoat-SSS`'s
(CM-3).

---

## Realism and project value

### [P2] Seabed-dependent constants need a capture over a known bottom
The Shiraishi-jima corpus fitted the encoding, the range law, the noise floor,
the gain ladder and the level reference (`docs/sonar_model.md` §6). It cannot
fit the constants that speak about the **seabed itself**: `lambert_exponent` is
confounded with an unsampled bottom type, and `specular_strength` /
`specular_width_deg` are set by whatever lies under the boat — the real −6 dB
FBR width varies 7× across altitude bands of one recording. Both are held at
values NC #4 gates rather than fitted.

The blocker is site geometry, not tooling. Two *real* passes of the same water,
at the same range setting and altitude band, disagree by **1.8–3.1 dB RMS**,
against a 0.67 dB reduction floor — a flat-seabed model compared against a
harbour survey cannot do better, because the far swath sits over ground whose
depth is nothing like the depth under the boat. Closing this needs a capture
over a **known, flat** bottom at ≥ 2 altitudes with the acquisition written
down. `sss_calibration_report` runs the comparison the moment one exists; it
refuses to score a pairing whose geometry does not overlap.

### [P2] `calibration_db_offset` is determined only to ±6 dB
The device's auto-gain regulates the real reported level, holding `max_pwr_db`
roughly flat against altitude while the model's follows the range law. Pooled
over 14 (range, altitude) cells at gain index 4 the required offset spans
**11.9 dB**; 97.2 is the ping-weighted mean. A capture at a **fixed** gain
index would pin it. Nothing in the current corpus fixes gain.

### [P2] Gain indices 0–3 are unmeasured
`ANALOG_GAIN_TABLE` carries measured values for 4–7 — the only indices the
device's auto-gain ever selected in seven recordings. 0–3 are the original
estimates and would misreport `analog_gain` and the level if a run used them.

---

## Blocked — not verifiable on this machine

Nothing at present. (The 20 Hz vs 22 ms ping-rate question that sat here was
**settled on 2026-09-02 from the corpus itself**: all seven recordings follow
`period = max(50 ms, 2R/c + 3 ms)` — 50 ms at 18.4 m and 20 m, 131 ms at
95.6 m — which is the spec sheet's "20 pings/s up to 30 m, then range-
limited"; the 22 ms figure was wrong. `sss_svlog_compare` reports the law
residual per file.)

## Open — model fidelity, next

- **Wall multipath strength — settled 2026-09-03.** Ghosts now carry
  `reflectivity²` (two bounces) times the Rayleigh roughness law plus the
  lumped `wall_diffuse_fraction` 0.04, anchored on the
  `fullHarbourCleanExample` quay (`analysis/wall_contrast.py`); what
  remains open is the port/starboard spread of that corpus measurement
  (beyond −5.6 vs −17.4 dB), which a second quay with known geometry
  would resolve.
- **Along-track correlation tail.** The corpus shows lag-10 correlation up to
  0.5 on the 40 m files; the footprint-overlap model gives ~0 beyond lag 3 at
  1 m/s. Likely real bedforms (A7) — needs a seabed micro-relief texture
  below the raster resolution, or the real vehicle speed (the corpus's
  LOCAL_POSITION_NED track is constant, so it cannot be recovered there).

---

## Known limitations (accepted, not bugs)

Recorded so they are not "rediscovered" as defects. These match the explicit
assumption list A1–A10 in `docs/sonar_model.md` §7:

- Static acoustic scene during a run; no intra-ping motion (stop-and-hop pings).
- Straight rays, no refraction. Reflections are first order only: the direct
  path plus the two multipath models (second-bottom-echo ghost, off by
  default; wall/surface mirror sources, on by default and a no-op without
  walls).
- A wall is an analytic vertical plane: it echoes (coherent face line at its
  own range), occludes what lies behind it and mirrors ghosts, but has no
  rubble foot, no texture along its face and no diffraction over its top.
- 2.5-D heightfield scene: no overhangs or cavities; objects are height +
  reflectivity stamps. Wrecks with internal structure would need a mesh path.
- Speckle correlation is geometric only (footprint overlap along track, pulse
  along range); no seabed micro-relief below the raster resolution.
- No biofouling, sediment interaction, or vegetation dynamics.

Settled against a full simulated survey (bundle `~/runs/r3`, seed 7, 906 s
lawnmower, 19 354 pings, mosaic cells 3.4 M):

- **Across-track range, not seabed material, sets large-scale brightness —
  by a wide margin.** Intensity falls **18.5 dB** from just past the bottom
  return to the 12–15 m far edge (measured on the fitted range law at 3.8 m
  mean altitude), against a **3.67 dB** total spread between all five
  materials. Sand→mud is ~0.7 dB and seagrass sits 0.24 dB from sand, so the
  "mud/seagrass image 5–10 dB darker" reading does not hold; the only material
  that separates is **rocks, ~2 dB brighter**. The gradient is the device's own
  pre-TVG range loss, measured against the corpus — dark far-range bands are
  expected output, not a renderer defect. Set `tvg_compensation: 1.0` for a
  flat image; waterfall tiles already remove its smooth part for display.
  **The 3.67 dB material figure and the mosaic numbers below were measured
  under the old range law and have not been re-measured since the fit.**
- **Objects are well resolved in the mosaic; contrast is the limit, not pixel
  size.** At the GCS's `auto_cell_size` (30.5 mm/cell for 15 m / 600 bins)
  manifest objects span **4–106 cells** — a 0.77 m debris object is 26 cells at
  7.9 σ. Of 29 objects, 23 were ensonified and **10 clear ≥3 σ**; the 13 that do
  not are spatially resolved but low-contrast at 4 m altitude (a 2.74 m PVC pipe
  is 90 cells at 2.0 σ). More mosaic resolution would not help — shorter
  altitude would, via shadow length; that is what
  `config/shallow_water_world.yaml` ships for.
  Detector evaluation stays in the **waterfall** domain regardless (root
  `CLAUDE.md` CM-10); there a 0.77 m object spans ~20 bins at 37.5 mm/bin.
