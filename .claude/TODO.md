# TODO — BlueBoat-SSS-Sim

Open items only. Settled facts live in `CLAUDE.md`.

Priority marks: **[P0]** blocks use of the package · **[P1]** correctness or
cross-module risk · **[P2]** realism / project value.

---

## World builder (2026-09 rework) — open points

### [P1] Live Gazebo + GUI pass on a builder world
The whole rework is verified by the offline suite (434 checks as of 2026-10-09) and the
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

### [P2] Gazebo gate: intermittent pose-delivery stalls fail "fresh pose" (2026-10-20)
On 2026-10-20 the gate's inputs had been lost (see `docs/sea_state_validation.md` §2, the
2026-10-20 note) and were rebuilt. Two full campaigns then each failed **one** check, both
times in *waves*: "a fresh pose under every ping".
- 5 m world: 24 / 25. 0.67 % of pings over 60 ms (limit 0.1 %), worst 320 ms.
- 1.5 m world: 23 / 24. 0.08 % over 60 ms, worst 180 ms; the sea node also saw 120 ms.

`waves` re-run alone on the 5 m world passed: 0.00 %, worst 10 ms. 2026-10-09 passed in all
eight scenarios. **Not reproducible, not a model change.** Measured, all on
`2026-10-20_5m/waves`:
- **The renderer is not the cause.** Re-rendered offline at the run's 5227 ping poses, a
  two-side tick takes median 17.7 ms, max 31.6 ms, never near the 50 ms period. The
  late pings' own ticks took 15–21 ms.
- **Gazebo did not stall.** RTF was 1.000. `/blueboat/odom` reached the bag recorder every
  50 ms (stamp and wall) straight through each late ping.
- The late pings come in bursts whose age grows 50 ms per ping (170 → 320 ms). A 310 ms age
  means `sss_sim_node` held neither a fresh 100 Hz pose nor a fresh odometry message
  (`_ping_pose` falls back to odom past 100 ms), while its sim clock and timer ran on. So
  both pose subscriptions to that node stopped being delivered for ~0.3 s.

A second session ran the offline smoke suites on this machine while these campaigns were live.
Those suites put CPU load on the machine but no DDS traffic (they are ROS-free). Whether they
overlapped the two failing *waves* runs, or the clean re-run, was not established. CPU
contention is therefore a candidate too, and the next campaign should run with nothing else
on the machine.

Candidate, **unproven**: reliable-QoS head-of-line blocking in the default RMW (Fast DDS,
no profile). One lost sample holds back the ones behind it until repaired. The pose
subscription (`/sim/sea_state/boat_pose`, depth 50, simulation-only) could go BEST_EFFORT
(a stale pose is worthless; a RELIABLE publisher stays compatible). `/blueboat/odom` is a
cross-module contract (CM-1/CM-4), not to be touched for this.

Next step: record `/sim/sea_state/boat_pose` in the gate's bag (`gazebo_sea_test.py`'s topic
list). Its receive times against its stamps tell delivery stalls from publisher stalls.
Run campaigns until one fails; change the QoS only if the bag shows delivery stalls. Until
then, a lone "fresh pose" failure with every other check passing is this item, not a
regression.

### [x] Live Gazebo pass with the sea state — done 2026-10-08 (world-folder path)
`test/gazebo_sea_test.py` ran the null, impulse and motion tests and the
sonar consequences in Gazebo on `~/worlds/sea_validation/flat_{5p0m,1p5m}`
(results `docs/sea_state_validation.md` §3): no wrench on calm water, mean
hull height = calm's to 0.5 mm, heave std = Hs/4 at the wave period, no
1.6 Hz line, roll/pitch 1.3–1.6°, ≤ 50 ms pose age while surveying (one
start-up sample of 180–240 ms at t ≈ 1.7 s, reported apart). The old
"heave p2p < 0.15 m at smooth" criterion was wrong for a 0.15 m Hs and is
replaced by heave std = Hs/4. **Superseded on 2026-10-09** (one hull,
S11): calm water now carries the catamaran hydrostatics, so "no wrench on
calm water" became "no wave forcing on calm water"; the re-run measured
mean hull height = calm's to 0.8 mm and the gate's pose-age limit is
≤ 60 ms. The stock-world companion path was run live on 2026-10-09 (item
"Live check of the MCS sea UI" below). Still open: the MCS sea UI by hand,
`gz model -m blueboat --link` / the marker arrow axis by hand.

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

### [P2] Reflectivity from a recording: absolute level and the material table
The `reflectivity_survey` layer (2026-09-16, replaces the disc-splat
hint) inverts every bin through the calibrated forward model
(`analysis/backscatter.py`, `docs/seabed_model.md` §7.1). Its *relative*
map is good to 1–2 dB; its *absolute* level carries the calibration's
±6 dB auto-gain caveat (`docs/sonar_model.md` §6), which is why it
renders about the survey median as a reference material rather than as
absolute values. Two things a capture over a known bottom would settle:
the reference material itself, and whether the fixed material table
(sand 0.55, mud 0.30, gravel 0.70, rocks 0.85, seagrass 0.22 — a nominal 5.9 dB spread, ~3.7 dB measured in-image under the old range law) should
be re-anchored to the literature contrasts the layer classifies with
(mud −5, gravel +3, rock +5…8 dB about sand at 450 kHz, [33] §12.1,
[41]). Decision 2026-09-16: the table stays, the rendered ceiling opens
to `REFLECTIVITY_MAX` under a gain map so the measured contrast can be
rendered; re-anchoring would change every existing world with those
materials.

### [P2] Shading relief: amplitude, shadow bound, low grazing angles
`analysis/shading.py` (2026-09-16) reads the relief across the swath from
the shading. Open: (a) the amplitude is uncertain by the undetermined
Lambert exponent — ×0.7 measured against the simulator's own ripples,
0.1–0.3 on the beach corpus where the swath grazes at 3–5° — so the
tie-line calibration (`relief_gain: 0`) is the honest default and a
survey without tie lines gets gain 1.0 with a "default" note; (b) the
shadow bound of Cuschieri & Hebert / Coiras (a shadow run puts the
surface below the ray) is flagged, not enforced; (c) at grazing angles
under ~5° the flat-floor beam model's residual (−8 dB/decade on
`bath1.svlog`) is what the inversion integrates — a beam-tilt fit on a
low-altitude recording (`sss_calibration_report --fit-tilt`) would
narrow it. A rippled bottom with a known bathymetry (a MBES line, or the
`sandbar_ripples` synthetic survey of smoke [4c] flown for real) is the
gate that would move any of the three.

### [P2] Curved walls: Gazebo shows chords
A bent wall is flattened to ≤ 0.1 m sagitta chords; in Gazebo they are
that many boxes with slight overlaps at the joints (cosmetic; each chord box carries visual and collision). The
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
"foam" and unmeasured. Since NC #17 it is read only by non-facet shapes
(lattice classes); on a solid primitive eps renders at |R| = 1 (air
backing). No class defaults to it; it is kept for the planned float/buoy class. Flag from a field capture of an EPS fish box.

### [P2] Depth flood fill ignores wall height
Every wall blocks the nearest-value extension regardless of `top_z` — a
low pontoon stops depth continuation exactly like a quay. Simple and
predictable; revisit only if a submerged-sill case turns up.

### [x] Screen 5 segfault on selecting / turning an object — closed 2026-09-16
Two use-after-free triggers (a selection change and a scatter-row handle
drag rebuilt the scene from inside Qt's own mouse dispatch) and an
unguarded running `QThread` replacement; reproduced offscreen with real
`QTest` events, fixed in `builder/map/canvas.py` (counted guards, deferred
rebuilds, diffed object items, a graveyard, a release signal) and
`builder/pages/place_page.py` (one undo step per gesture, the pin
question after the release, the worker guard, the `pinned` flag, the
review items of that day). `CLAUDE.md` §3.3 (5) carries the rule; the GUI
smoke drives the sequences under `faulthandler` (printed under `[g5]`).

### [P2] Hand-placed objects are not marked in `metadata.yaml`
Screen 5 (2026-09-14) writes `objects.explicit` and can pin Screen 4's
random field, but a consumer reading `metadata.yaml` cannot tell an
authored object from a scattered one except by its `orientation.mode:
fixed` — and a *fully pinned* world reports `fixed` for every object, so
that signal degenerates exactly when it would be most useful. A
`placement: manual | random` key per object would separate authored
ground truth from the random field for the thesis's metrics. Deliberately
deferred: the key is additive to `blueboat_world_meta/1`, and the
decision (2026-09-14) was to leave the cross-module metadata contract
untouched for this change.

---

### [P2] Positioned sources are interpolated isotropically

The layer editor (2026-09-09) can lay a depth file or recording over a
modelled source and blend it by coverage, and its `channel` layer works
in flow-oriented (s, n) coordinates — but a *source* itself is still
interpolated isotropically (`interpolate.py`, `smooth.py`), which smears
a surveyed thalweg across the channel (Merwade et al. 2006, 2008;
`docs/seabed_model.md` §3). A `centreline` option on a positioned source
that re-interpolates its soundings in (s, n) would close this; the
`geometry.polyline_frame` machinery is already there.

### [P2] Material layers act mostly through relief

The seabed editor's material layers (regions, gradients, patches, belts,
the reflectivity gain) render through the same five materials whose whole
nominal spread is 5.9 dB (~3.7 dB measured in-image, old range law) against the 18 dB range law (known limitations below),
so on the image a material edit reads mainly through the micro-relief and
shadows it carries. Re-measuring the material constants over a known
bottom (the [P2] item below) is what would make the gradient a
sand-to-mud contrast rather than a hint.

## Sea state (2026-10-08 upgrade) — open points

### [x] One hull model in every run (S11) — resolved 2026-10-09
The catamaran hydrostatics now carry the hull in every run, calm water
included (`hull.hydrostatics: catamaran`; `gazebo_box_when_calm` keeps the
old swap-only-with-waves behaviour for old runs). Evidence and reasoning:
`docs/sea_state_model.md` §5 (corpus pitch vs speed: the field's trim is a
static per-session offset of +1.2..+2.8° at rest, the dynamic part small
and of inconsistent sign, so no trim term is fitted). Gazebo gate: calm
trim now ≈ waves trim (checked). The MCS empty-world path joined the same
day (user's order): `launch_manager.companion_command` starts
`sea_state_launch.py` for every empty-world launch, a null sea included,
and the SEA STATE box flags a run on the box. Only a bare command-line
`Sim_launch.py` without the companion still floats on the box (control
tuning, no seabed scene).

### [x] The simulated controller could miss "ready" — fixed 2026-10-09 in BlueBoat-Control
`simulation_interface` published `/blueboat/controller_ready` once
(latched); `master_control` subscribes volatile, so a late join never heard
it and the boat sat at the start (two Gazebo-gate starts lost on
2026-10-09). By the user's order `simulation_interface` now re-publishes it
every 1 s like `robot_interface` (QoS unchanged, CM-4). Live check: a
volatile subscriber joining 40 s after launch receives `true`, at 0.93 Hz;
the `--quick` gate re-run started all four scenarios on the first attempt.
`test/gazebo_sea_test.py` keeps its stall detector and relaunch
(`--stall-s`, `--retries`, attempts in `run.yaml`) as a guard.

### [P2] The eddy field's scale and strength are judgement values
`EddyField`: 8 modes, 25–100 m, 5–15 min, RMS = 0.05 U ("natural"). No
site measurement exists; an ADCP transect or a few drifters across the
basin would anchor both (and say whether the mouth needs a jet).

### [P2] Live check of the MCS sea UI by hand (empty-world path done 2026-10-09)
The **empty-world** path was run live on 2026-10-09
(`docs/sea_state_validation.md` §3, "The station's empty-world path"). It
found and fixed a real defect: the injection of `ApplyLinkWrench` into
`ocean` was skipped, because the launch's own bridge advertises
`/wrench`, so the wrench never reached the hull there. Injection (entity
id 1), the calm catamaran hull, a live command and a clean stop are now
checked. Still to see by hand: the redesigned MCS sea dialogs / timeline
planner / "Boat motion" row and the amber "hull: Gazebo box" Status
(verified offscreen and by the MCS smoke test only), and the GUI marker
arrow's axis (`marker_arrow_axis`; headless runs have no marker service).

### [P2] Simulated yaw is 4–10× calmer than the real boat's
Gazebo gate vs corpus: yaw-rate std 1.2 °/s (light chop) against 5.1 °/s on
the cleanest field line (`OneLineHarbourClean70m`) and 9–14 °/s elsewhere,
while roll/pitch rates match (sim 5.7–6.1 / 7.0 °/s vs 6.8 / 5.4 °/s on
that line — `docs/sea_state_validation.md` "Against the real boat"). Candidates: the
BlueROV2 yaw damping of `hydrodynamics.xacro` (`nRabsR −40`), the weak
first-order FK yaw moment of a short boat in short-crested chop, the PID's
heading stiffness, and IMU noise / helm activity in the field figure.
Until resolved, every yaw-driven (differential-mode) effect in
`docs/sea_state_validation.md` is a lower bound.

### [P2] Intra-ping motion is not modelled (S12)
Stop-and-hop pings: a 10°/s yaw over a 20 m ping's 27 ms turns the fan
0.27° (~9 cm at the far edge, about half a footprint). If a corpus waterfall
ever shows within-ping smear at the swath edge, this is where it belongs.

## Cross-module correctness

### [x] `transducer_heading_deg` carried an absolute bearing — closed 2026-09-15

The encoder wrote `(vehicle ∓ 90) mod 360` where the real Omniscan 450
reports the constant mounting angle −90 / +90 on every packet (measured on
100 % of the field corpus, `project_review.md` §3.6). SonarView's channel
fallback reads the sign of that field, so every simulator ping with a
heading in 90–270° was mirrored in SonarView — a CM-2 violation invisible
to our own stack, which routes on `channel_number`. Fixed in
`sonar/encoder.py`; `analysis/svlog_reader._side_from_packet` now applies
the wrapped-sign rule shared with the GCS and the processor (older
simulator logs stay readable), `analysis/backscatter.backscatter_bins`
(2026-09-16; before it `bathymetry.backscatter_samples`)
derives the look direction from `vehicle_heading_deg` + `side.sign·90°`,
and the smoke test gates the wire value in `[2]` and the placement in
`[4c]`. Root `CLAUDE.md` §4.6 records the convention.


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

### [x] Transducer lateral offset agreed at 0.285 m — closed 2026-10-12/13
The disagreement this item recorded (simulator `mount_y_abs_m: 0.20`,
processor `0.0`) is gone. Both now use **0.285 m**, half the BR-101447 hull
spacing (570 mm / 2), the BlueBoat drawing rather than a tape measurement:
- every profile in `config/*_sonar.yaml` (`default_sonar.yaml:36`; edited 2026-10-12);
- `TRANSDUCER_Y_OFFSET_PORT_M` / `_STBD_M` in `sss_processor_node.py` (SSS `994a74b`,
  2026-10-13), mirrored in the GCS's `core/svlog.py`.

The smoke test's render fixture `test/fixtures/render_baseline_v1.npz` was rebaselined
for this change on 2026-10-19. It had been frozen at 0.20, and re-rendering at 0.20
reproduces it bit for bit. The pontoon-crossing check now places the boat from the
profile's mount, so its transducer stays 0.3 m from the line.

### [P2] Transducer fore/aft offset and submersion are not measured
`TRANSDUCER_X_OFFSET_M` = `mount_x_m` = 0.0 (unmeasured, amidships assumed), and
`TRANSDUCER_SUBMERSION_M` = `sensor_depth_m` = 0.15 m (an estimate). The two modules
agree, but neither number is measured. `blueboat_description` ships a
`sonar_snippets.xacro` but never instantiates it, so the URDF carries no geometry.

Measure on the hull, then set both repositories (the `BlueBoat-SSS` half is that repo's
change, CM-3). Record who measured it, on which hull, from which reference point.
Since 2026-10-21 `sensor_depth_m` is unambiguously depth below the **still waterline**
(the Gazebo base_link datum is removed by the node, `base_link_rest_z_m`); measure it
that way, and put the value in a calibration artifact (`installation` class, per session
if the transducers are re-mounted), not in `default_sonar.yaml`
(`docs/engineering/validation_and_experiment_plan.md` stage 1).

### [x] The simulator never exercised the downstream degradation paths — closed 2026-09-18
CM-2 promises the real stack runs unmodified against simulated data, and a
clean simulation could not demonstrate it: constant gain (so the
processor's compensation sat at `g = 0` forever), a counter that never
restarted, no per-device offset, never `channel_number: 255`, no dropped
pings. `config/fault_injection_sonar.yaml` + the four `fault_*` knobs put
all five on the wire on demand (`sonar/encoder.py`, off by default, gated
by smoke `[4b]`). The processor's own half — `GLOBAL_POSITION_INT`
velocities hard-coded to zero, so every `.svlog` it writes claimed a
stationary boat — is fixed in `BlueBoat-SSS` (`sss_processor_node.py`,
from `/mavros/local_position/velocity_local`, which it already
subscribed).

### [P1] 1200-bin handling in the GCS: replay proven, live to confirm
Since 2026-09-28 the default acquisition is 1200 bins / pulse 0.00125 at both
ends — the simulator's default profile and the real `sss_node.py` — so every
station-started Gazebo run and every real run exercises this. The GCS **replay** path is proven on field data
(it produced the 2400-pixel seabed images of `object_boiteVsDistance`, a
1200-bin log); the **live** path is confirmed by the first such run.
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

### [x] `calibration_db_offset` determined only to ±6 dB — retired 2026-09-18
History: on the single-altitude fit the required offset spanned **11.9 dB**
over 14 (range, altitude) cells at gain 4, with 97.2 the ping-weighted mean,
read as auto-gain regulation. The two-altitude re-fit (NC #6) showed it was
a *slope*: with `specular_spreading_exponent` 2.0 the reported peak follows
the device's measured altitude law, and `calibration_db_offset` (now
**81.1**) is determined by the seabed level to well under a dB at both
anchors (`docs/sonar_model.md` A8, §6). A fixed-gain capture would still be
welcome but no longer gates anything.

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

### Closed on 2026-09-18 (the sim review's Part 6)

The 2026-09-18 review (`../sim_review.md`) measured the simulator against
both corpora and its Part 6 update plan is **implemented**: the level and
range law re-fitted on two altitudes (P6-1, and with it the angular law
P6-9), object transmission and the `plastic_box` class (P6-2, P6-3), the
shadow's reverberation floor and the shallow-beach profile (P6-4), wall
occlusion of faces, ghosts and the corner band (P6-5, P6-7), the near-field
knob measured *off* (P6-8), fault injection and the processor's GPS
velocities (P6-10), the GCS's planned path, waterfall scale and contrast
(P6-11/12/13, in `BlueBoat-SSS`), ground truth read off the rendered mask
(P6-14), the documentation (P6-15) and the literature-anchored material
override (P6-16). What each one is gated by is in `CLAUDE.md`'s
"2026-09-18 gates" paragraph; what is *left* is below.

### Closed on 2026-09-19 (the sim-vs-real image-formation rework)

The user's comparison of `sessions/beach_sim` against `beach_sept` measured
four things, all now addressed (`docs/sonar_model.md` §3.1, §6):

* **the object was a glint, not a target** — +21.6…+23.3 dB over the local
  seabed against the device's measured **+2.5**. Three causes, each fixed:
  the bin was a per-sample *mean* so a folded wall did not accumulate but
  its facet still collected the near-normal lobe; that lobe carried *sand's*
  fitted RMS slope on a manufactured surface; and it collected the
  image-source `r²` bonus on a facet that is one Fresnel zone. Now
  **+7.6…+9.2 dB**, and independent of the raster (7.4 → 0.2 dB spread);
* **a hard black band between the object and its shadow** — the shadow floor
  was a fraction of the culled bin's *own* would-be response, which
  collapses where the incidence clips to zero. Referenced to the ambient
  level instead: worst bin **−54 → −19 dB** against the device's −13.6;
* **the waterfall's metres-per-pixel did not match** — the sim ran the 15 m
  default against a 20 m corpus. The default moved to 20 000;
* **simulated recordings claimed a stationary boat** — `mavros_shim_node`
  published no `/mavros/local_position/velocity_local`, which
  `sss_processor_node` reads for `GLOBAL_POSITION_INT`'s vx/vy (measured:
  0.000 m/s on all 1480 fixes of a 191 m track). It publishes it now.

Also: the shading inversion was re-derived exactly for the new forward model
(`phi = arctan(ratio·tan(dep)) − dep`, a single arctangent — no albedo, no
exponent), which took smoke `[4c]`'s ripple correlation from 0.87 to
**0.96**; the GCS live path stopped freezing its display model (below); and
every seabed recipe gained a bedform with a sonar-look preview to author it
by.

### Closed on 2026-09-28 (object physics, NC #17)

The user's comparison of `sessions/2026_09_28-10_27_47` / `-10_40_17`
against `beach_sept/object_boiteVsDistance` found the box still a glint
(+15.9 dB and up against +0.2…+1.1). Causes and fixes, each measured
(`docs/sonar_model.md` §12):

* **the World Builder's path never built the roughness raster**, so the
  2026-09-19 smooth-surface rework had never reached a GUI world (+19.8 /
  +24.6 dB near-edge at the field geometry). One stamping pass now serves
  both builders, `[9]` compares every raster, and `stale_rasters()` names an
  old world on load. **Every GUI world with smooth objects built before
  2026-09-28 must be rebuilt** (`plage`, `plage_1box`, `plage_2boxes`);
* **the ~6 dB residual of the item this replaces** ("the field box is still
  ~6 dB brighter") was not geometry: it was sand's reflectivity carried onto
  the object's face ramp by bilinear sampling, plus Lambert's law applied to
  a mirror. Smooth solids now scatter in the facet layer (Kirchhoff
  patches); the box reads +0.1…+2.9 dB near-edge over 94 % of headings at
  the field geometry, shadows −8.0 / −8.3 (field −8.2 / −7.4), no seam, and
  glints only within ~1° of alignment;
* **`wall_mm` / `flooded` were dead knobs** — now read, and carried by a
  pinned entry; the class draws unburied by default;
* a comparison world, `~/worlds/plage_bathymetry2/plage_box_field`, puts the
  box where the field's lay (world (0.912, 14.306), 1.50 m deep, heading 30°
  — unknown in the field).

### [P1] Re-run the sim-vs-real box comparison in Gazebo

Offline, at the field geometry, the facet layer reproduces the field box
(above). The end-to-end check is a GCS session over `plage_box_field` (the
default profile is 20 m / 1200 since 2026-09-28, so no argument is needed;
the package must be rebuilt first — the install predated all of this),
reduced with
`python3 -m blueboat_sss_sim.analysis.object_signature <session>/seabed_images
--at 0.91 14.31` (the sim's GCS frame is the world frame) against the same
tool on `beach_sept/seabed_images_object_boiteVsDistance --at -15.9 17.7`.
Rebuild `plage_1box` / `plage_2boxes` (Edit) before reusing them: both are
stale.

### [P2] The box's footprint reads 1–4 dB darker than the field's

At the field geometry the model's footprint sits at the shell's
transmission (about −8 to −9 dB), like its shadow; the field's is lighter
(about −5 / −6.5) than its own shadow. With only water inside, the
candidates are what one transmission cannot carry: the lid and floor are
forward mirrors at 70–80° (so the seabed under the box should be *darker*
than behind it), while the interior far wall, the lid's ribs and the
buckles return a little through a nearly transparent near wall. Needs the
surveyed-heading capture below before any second constant is added.

### [P2] Facet layer: first-order physics only

`R` is the normal-incidence value; no multiple reflection inside an object
(a shell's far wall, the dihedral with a rough seabed); no fringe (PTD)
term; facet objects keep their shadows but not their highlights in the
multipath ghosts; patch validity R ≳ 2 m. Each is a bounded extension of
`sonar/facets.py` if a capture ever shows it matters.

### [P2] `sand_lambert_mu_db` is literature, ±5 dB

The one constant putting an object's cross-section on the seabed's scale
(−12.0 dB, J&R Fig. 12.3 + par. 12.1.2). It sets how bright a glint is, not
whether one appears. **The capture that pins it** is the one the 2026-09-18
review asked for: the same `plastic_box` (or a steel drum) at a **surveyed
heading**, imaged in several passes at 4–6 grazing angles with a logged
track, so that one pass is aligned with a face.

### [P2] The across-track edge is 10x sharper than the device's

Measured 0.08–0.12 m against the field box's 1.10–1.84 m. **No physical PSF
at 450 kHz explains that gap**: `cτ/2` is 25 mm and the across-track ground
cell at 19° grazing is comparable, so the model cannot make a 1.1 m
transition and should not be made to. The remaining candidates are platform
motion within the real measurement's 33-row window and the real target's own
geometry. Re-measure the real edge **per ping** rather than on a row mean
before concluding anything; if it survives that, it belongs in the sea-state
model, not here. Since 2026-10-08 the sea state moves the
transducer realistically and the Gazebo gate records every ping's pose
(`~/ground_truth/sensor`): the same 33-row edge measurement on a *waves*
run of `test/gazebo_sea_test.py` would say how much of the gap platform
motion can explain. (`penumbra_deg` exists and ships at 0 for exactly this
reason — it would hide the question rather than answer it.)

### [P2] `smooth_lobe_width_deg` is shipped suppressed, not measured

Since 2026-09-28 this concerns only the smooth *aggregates* left on the
raster (the lattice classes); smooth solids are facet objects. On a surface
the roughness raster calls smooth the near-normal lobe is
switched off entirely, because the 8° it would otherwise carry is *sand's*
RMS slope and nothing measures a manufactured surface's. The field box's two
grazing angles (+5.7 dB at 7°, +2.5 dB at 19°) would solve to ≈15°, but that
pair carries an unknown yaw and burial and a 30° yaw offset absorbs several
dB. The capture above fixes this too.

### [P2] The per-material surface roughness table is judged, not measured

`_ROUGHNESS_M` in `worldgen/materials.py` — 0.1 mm for an injection-moulded
shell, 2 mm for form-finished concrete — is sized from manufacture. It is
what decides whether a target images as a glint or a faint patch, so it is
stated in one table rather than spread through the material entries. A
capture of two known objects of different finish at one grazing angle would
anchor it. `brickclay` (1 mm) is the entry nearest the 1.6 mm line: judged, on the facet side (smoke `[14]`).

### [P2] The beach's shore needs a swash zone, not a model change

Past the waterline the simulator now sits at the reverberation floor rather
than at the receiver's, but the last wet strip is still not the bright
accumulated band the real beach shows. On a 1:8 slope there is no layover to
accumulate (`dr/dy` stays ≈ 1; a fold needs a near-vertical face), so the
real brightness there is the swash zone itself — coarser sediment, bubbles,
steep micro-relief — which is **world authoring**, not acoustics. A
shoreline material/relief band in the beach recipes is the fix.
`shore_taper_m` exists and ships at 0: tuning it would darken the band
further, which is the wrong direction.

### [P1] The reported peak is still ~5 dB hot
The seabed level matches within 0.6 dB at both altitudes and the peak's
altitude *slope* within 0.2 dB/decade, but its level sits 4–7 dB above the
device's law (`66.60 − 11.28·log₁₀ alt`). The fit window deliberately
excludes the near-nadir return, so this is the near-nadir model —
`specular_width_deg`, the beam's near-field behaviour, or the specular /
diffuse split at high grazing. None of the three is determined by this
corpus. A point target at 3 and 12 m, and a fixed-gain pass, would settle
it (the captures the review lists).

### [P2] Pontoon signature has no field anchor
`wall_pontoon_specular` (0.15) and the float-face model are judgement
values: no recording of a pontoon with known geometry exists.
`mooring_pile` (2026-09-18) lets a pontoon line be furnished with the
piles that actually stand beside it, which is the part that needed no
anchor; raising the specular fraction still needs one.

### [P2] Second-bottom ghost and surface mirror are profile-scoped
`config/shallow_beach_sonar.yaml` turns both on for the ≈1.2 m regime,
where they land inside a shadow rather than beyond the swath. Their gains
are geometric, not fitted: a capture at ≥ 3 m altitude at the same site
would separate the fill they produce from the TVG.

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

## 2026-10-21 audit — open follow-ups (`docs/engineering/issue_triage.md`)

### [P1] Staged field calibration on session-split data
The scaffolding exists (`blueboat_sss_sim/calibration/`, `docs/engineering/`); the
fits do not. Next: write the manifest of the existing corpus (roles: the two NC #6
anchors are calibration data), run `sss_calibration quality` on it, then stages 1-4 of
`validation_and_experiment_plan.md` in order. Blocked on the private corpus.

### [P2] `objects.classes.<class>` ignores unknown keys
`count: 3` (instead of `amount: {kind: count, value: 3}`) silently places nothing.
Make class entries strict in a deliberate change (it would reject old hand-written
configs that carry stray keys).

### [P2] Bin-centre convention differs across analysis tools
`i·L/N` (calibration.py, svlog_digest.py), `(i+½)·L/N` (renderer, calibration package),
`i·L/(N−1)` (bathymetry.py). Up to half a bin. Settle on the device's convention once a
target at surveyed range measures it.

### [P2] One noise RNG for both sides
A dropped ping on one side shifts the other side's noise realisation. Split per side
(`SeedSequence.spawn`) when per-side reproducibility matters to augmentation.

### [P2] `[4d]` ghost-lab objects are never stamped; `block_concrete` is not a class
Test-quality fix for the next time `[4d]` is touched.

## Known limitations (accepted, not bugs)

Recorded so they are not "rediscovered" as defects. These match the explicit
assumption list A1–A13 in `docs/sonar_model.md` §7:

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
- One **global** angular exponent, measured over sand. A per-material
  angular law is real [J&R par. 12.1] and unmeasured here; the dead
  `Material.lambert_exp` was deleted rather than wired (2026-09-18).
- The near field is assumed **focused**: the along-track footprint is
  `r · θ` at every range. The corpus rules out an unfocused 0.38 m aperture
  at 12 m; inside ~3 m nothing measures it (`array_length_m`, default 0).
- A material transmits or it does not: one scalar, no thickness, no
  frequency dependence, no internal reverberation. `pp_shell`'s 0.11 is
  the field box's measured shadow (re-measured 2026-09-28 through the facet
  layer), not a model of a shell; its REFLECTION is modelled (the
  fluid-layer formula on the wall thickness, NC #17).

Settled against a full simulated survey (bundle `~/runs/r3`, seed 7, 906 s
lawnmower, 19 354 pings, mosaic cells 3.4 M):

- **Across-track range, not seabed material, sets large-scale brightness —
  by a wide margin.** Intensity falls **18.5 dB** from just past the bottom
  return to the 12–15 m far edge (measured on the fitted range law at 3.8 m
  mean altitude), against a **3.67 dB** total spread between all five
  materials (the table's nominal spread is 5.9 dB). Sand→mud is ~0.7 dB and seagrass sits 0.24 dB from sand, so the
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
