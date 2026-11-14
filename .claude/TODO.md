# TODO — BlueBoat-SSS-Sim

Open items only. Settled facts live in `CLAUDE.md`.

Priority marks: **[P0]** blocks use of the package · **[P1]** correctness or
cross-module risk · **[P2]** realism / project value.

---

## Ray-cast harbour sensor — open after the completion round, session 01 (2026-11-14)

Evidence and commands: `~/ros2_ws/sim_runs/sim_acceptance/01_sonar_physics/handoff_01.md` (this machine only).
Physics and inputs: `docs/sonar_model.md` section 15.

### [P1] The corner path on sand rests on an untested roughness rule
An upright face on sand returns 28 dB more than the face alone through the seabed bounce
(`chains.py:FloorBoundary.factor`: only roughness above the Fresnel-zone wavenumber destroys the mirror, so a
patch close to the floor sees a nearly smooth bottom). Verified as implemented (V4, 0.5 dB); no recording tests
the rule itself. It is the largest term of an upright object's echo. Bracket it for the sensitivity study (a key
that scales the coherent floor bounce between 0 and the rule's value would do it; not added: the code was
frozen for the final batch) and state it with the model-conditional results.

### [P1] The beam could not be measured on the corpus
Main lobe and side lobes on real reflectors: yaw sweeps the beam as fast as travel and the heading is not known
to 0.05 deg on the ping clock (evidence `beam/beam_measurement.md`). The aperture is the vendor's 0.5 deg
two-way statement. A corner reflector at mid-water on a straight line, attitude on the sonar's clock, would
settle it. The elevation pattern and the 20 deg tilt were not re-examined (the Makurazaki roll response could
bound them).

### [P1] A thin plastic shell's shadow is a bracket, not a number
Elastic wall, measured constants: no shadow at harbour grazing; low end of the documented constants: -11 dB;
fluid wall: -13 dB. The real box (beach, SEEN) reads -8 to -11 dB. Missing: polypropylene shear absorption
below 1 MHz, a coupon measurement, ribs / lid / trapped air. Only `plastic_box` is a shell in the catalogue;
drums, tins, pipes, crates and polytanks are solids (request to the catalogue's owner session).

### [P2] Layers and classes not covered by V5
Rubber and the steel-belted tyre wall (no absorption or shear speed at 0.2-1 MHz could be read at a source; the
tread echo stays an upper bound), rope constants.

### [P2] Shadow fill is still the measured 18 dB stand-in
The engine's own physics (side lobes, diffraction, first-order multipath, receiver noise) leaves -23 to -27 dB
behind a 0.6 m object and -38 to -48 dB behind a 2.4 m one, against the corpus quay's 17-25 dB; volume and
incoherent surface reverberation are not modelled and no sourced volume scattering strength at 450 kHz was
obtained (evidence `shadowfill/shadow_fill.md`). The stand-in stays, bracket 17-25 dB. Seen on it: the shadow
reads -18.4 to -19.3 dB at 8 m but -13.8 dB at 14 m in 6 m of water, because the floor follows a smoothed
per-ping ambient level (`floor.ambient_reference`): refer it to the local range if that 4 dB matters.

### [P1] Flash bracket (R1): the wide end removes the box flash at 6 m, it does not widen it to ±5 deg
Atlas of 2026-11-14 (evidence `atlas/atlas_half_widths.json`, handoff 2.18), 5 m of water, PP container's long
face, half-width with the highlight >= 3 dB over the seabed: narrow ±0.5 deg (+8.9 dB), production ±0.25 deg
(+6.5 dB), wide none (+0.7 dB) at 6 m; ±2.5 / ±1.0 / ±5.0 deg at 16 m. Cylinders at 6 m: ±0.5-1.0 to
±2.5-4.5 deg. The acceptance text asks for "about ±1 to ±5 deg for the box face at 6 m"; that ±5 deg was the
legacy renderer's own error at beach geometry (handoff 2.15). Session 05 judges it. If a wider visible flash at
short range is wanted it needs a physical reason (a measured face, ribs), never a level.

### [P1] Live ray-cast run: 99.90 % of nominal pings, not 100 %
Headless run of 2026-11-14 (harbour world, harbour-fast, 20 m, requested factor 2, hold `when_needed`; evidence
`throughput/live/c2_live_harbour_fast_20m/c2_live_summary.json`, handoff 2.17): real-time factor 1.19, ping
numbers consecutive on both sides, but one 470 ms gap between pings 1.4 s after the first one (a 239 ms cycle
at start-up; the node warns "behind the sim clock although the simulation is held") and 44 intervals of 40 or
60 ms among 9 612. To do: one run with `hold_sim_while_rendering:=always` and its cost; find why a ping tick
lands 10 ms early or late (sim step against the timer); or have the recorder start after a warm-up.

### [P2] V3 away from the flash
Ray-cast and independent reference agree on the flash (peak within 0.53 dB, half-width within 0.05 deg) but
differ by 4-11 dB at a few aspects 37-46 dB under the peak: a cylinder end-on (the staircase outline of its
end cap), a plate and a box at 10 deg (cause not isolated). Under the seabed level in every harbour picture.

### [P2] Path selection is redone on most pings among many objects
With 20 objects in reach the set of object faces within range changes at almost every 5 cm step, so the
selection reuse (`paths.reselect_every`) rarely applies. Cheap since the 2026-11-14 pre-test (about 60 ms per
ping-side with 30 objects in the scene); a reuse key on the faces that pass the pre-test would remove the rest,
but changes which pings reuse a selection: prove it on `throughput/facet_cost.py` and V9 first.

### [P2] Face waviness is assumed
0.5 deg over 0.10 m in production, bracket 0 to 2 deg over 0.05 m; no measurement of a real container's face.
A straightedge and feeler gauge on the real containers, or the real heading-response experiment.

### [P2] Port is brighter than starboard on every harbour file
+0.1 to +1.3 dB at u 1.4-2, not consistent in size (evidence `portstbd/`). Not modelled.

### [P2] `owner_measurements.yaml` does not exist
Mount values are the pass-3 ones. The day it exists: values into `config/default_sonar.yaml` `model:`,
regenerate the ray-cast profiles, re-baseline the render fixture, rerun V7. Change requests for the processor
and GCS fore/aft arm (0.0 against -0.345 m): evidence `inputs/lever_arm.md`.

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

Recurred 2026-10-30 (pass-2 verification): once under a concurrent 4-worker
CPU load (2.8 % / 4.6 % of pings over 60 ms in *waves* / *both*) and once
unloaded (0.17 %, worst 90 ms, *waves*); the calm + waves re-run passed
(0.00 %). The gate also checks the sonar stream since 2026-10-30
(`--sonar-config`, device invariants + FBR detection on every profile).

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

## Ray-cast sensor (`renderer: raycast`) — in progress since 2026-11-02

### [P0] Option A plan — independent review of 2026-11-11 (built; the owner's live test is next)
Record: `blueboat_sss_sim/docs/engineering/independent_review_2026-11-11.md` (decision, plan,
as-built, results against their pre-stated tolerances); plan file
`~/.claude/plans/pasted-content-id-5000-independent-lively-flask.md`. The owner chose to keep
and improve the custom ray-cast (HoloOcean / hybrid rejected, `holoocean_reference.md`).

**Done 2026-11-11:** run provenance + stale-install warning (`core/provenance.py`,
`~/ground_truth/run_info`, GCS `metadata.json["simulation"]`); bottom line verified offline with
the processor's own tracker (`analysis/raycast_bottomline.py`: locked 99.7 % on the 11-11 track);
ground-truth contacts (`sonar/raycast/labels.py`, `raycast.labels.enabled`) and `range_scale`
(Perception applies it); the shadow floor added in the field domain (`raycast.floor.addition`);
`seed_per_run`; Gazebo object heights (`sdf_writer.py`); **objects as their own meshes**
(`raycast.objects.mode: solid`, `sonar/raycast/object_mesh.py`: catalog primitives and fine mask
meshes on the terrain sheet). All on in the two harbour profiles only; default / live / fast are
unchanged. Gates: smoke `[18]`, `raycast_test --only L` / `--only O`, guard
`~/ros2_ws/sim_runs/raycast/review_a/guard_a.py --check`.

**Done 2026-11-11, second pass (one surface rule; `.claude/raycast-live.md`):** every object as
parts with materials (`worldgen/compounds.py`; the mask skins are gone), the diffuse share of
smooth surfaces and `raycast.objects.fouling` (node `fouling`), a steel-belted car-tyre tread,
`Wall.material` (World Builder), gravel / rock levels, world material overrides, the nadir onset
fade (`correction.onset_fade_m`), Gazebo visuals from the same parts, and the GCS Record ON race
that left a session without a `.svlog` (BlueBoat-SSS). Justification and references:
`blueboat_sss_sim/docs/simulator_justification.md`.

**Open, in order:**
- **[P0] Owner's live test** (`python3 -m blueboat_sss_sim.core.provenance --check` tells whether
  the install matches the source): MCS "Ray-cast — harbour, fast" on the harbour world, GCS record.
  Look at: the nadir edge (a ramp, no bright line), the bottom line (locked, about 0.3 m late like
  the real tracker), the pallets as boards, `metadata.json["simulation"]`, and Perception's
  dataset-mode boxes (`BlueBoat-Perception/tools/draw_labels.py`). The bicycle tyres stay faint
  (bare rubber, 2 cm proud): try `sonar_fouling:=0.5`. For a meaningful detection test build a
  world with objects above the thesis' own floor (>= 0.5 m, relief >= 0.2 m: car tyres, concrete
  blocks, drums, crates); a world re-saved with Edit shows the compounds in Gazebo too.
- **[P1] WP5 — validation world + `raycast_test --only V`** (range bin, shadow at two altitudes,
  plate / drum aspect, occlusion behind a quay, roll / pitch / heave, seed independence, cost
  record). `[L]` and `[O]` already cover labels, shadow length, material ordering, the surface
  rule, fouling and repeatability on synthetic scenes.
- **[P2] Cost of mirror patches:** a pallet is ~28 000 patches processed whenever it is within
  range, also off the beam (40 vs 35 ms per cycle on the 07:57 track). An exact off-beam skip is
  possible when no wall or object face is in reach.
- **[P2] Not modelled, stated:** hollow bodies (pipe bore, drum, open crate), sheet-pile
  corrugation and fenders, absorption in tyre rubber (no value read at a source), the builder's
  reflectivity-gain layer (seabed patchiness), seagrass, a mirror return from a wall smoother
  than 1.6 mm; extended contact keys for rested objects (Perception's footprint is L × W).
- **[P2] Perception / GCS follow-ups:** Perception should store the run record with a dataset
  (it has the sensor's `run_id`); the GCS replay window does not show the record.
- **Limits to state in the thesis:** `independent_review_2026-11-11.md` §11.

A second `SonarRenderer`: 3-D ray casting with coherent summation and full 6-DoF motion,
beside the unchanged geometric renderer. Plan, gates and the Phase-7 data split:
`blueboat_sss_sim/docs/engineering/raycast_plan.md` (status log at its end). Design:
`raycast_design.md`. Literature: `docs/literature/raycast_sonar/`.

### [P1] Live ray-cast in a slowed simulation (built and run live 2026-11-07)
Node selection (`renderer`, `raycast_backend`), `sonar_renderer:=`, `real_time_factor:=`, the
one-clock plumbing (Control / SSS / Perception / MCS), warp in the node's interpreter, the
start-up warm-up and the exact Gauss-Legendre cache are done; live on `flat_5p0m` x0.08 held
20 Hz (CLAUDE.md §3.1). Open:
- (closed 2026-11-11) `ground_truth/contacts` under the ray-cast: `sonar/raycast/labels.py`, on in
  the harbour profiles (`raycast.labels.enabled`).
- The GCS replay-path sonar (`blueboat_gcs/ros/sonar_source.py`) passes no renderer
  (§11 "both paths").
- Per-scene factors: harbour (Shiraishijima, 20 m / 1200 bins, hybrid v2) **x0.03 holds 20 Hz**, measured end
  to end through the MCS and GCS Apps on 2026-11-07 (`integration_guide.md` §2b). The pool is still to be
  measured. The node logs the factor it needs.
- **GCS bottom tracking on ray-cast pings** (found 2026-11-07): the processor's tracker reads the hybrid's first
  return 1.4–1.6 m late and jumps by metres; the geometric renderer at the same poses is within 2 cm. The
  ray-cast has no sharp first-return line, and NC #4 was never established for it. The images are fine, but
  their georeference near nadir is not. A fix is model work: the owner decides.
- Fixed 2026-11-07: `sss_sim_node.main` swallows rclpy's teardown race. Before, every MCS stop of a ray-cast run
  printed a traceback and "process has died".
- Speed: a cycle is mostly CPU numpy (scatterers, chains, laws, deposits) and costs 2-3x more
  live than offline under CPU contention; `GpuMultipath._beam` could use
  `core/gauss_legendre_cache.leggauss` itself (today the node installs it process-wide).
- **Quay cost (2026-11-09, `.claude/raycast-live.md`).** The first full MCS + GCS run lost 23 % of its pings
  past a quay (port ping-side 1–7 s, 16 s at the pier corner, vs 0.08 s in open water). Done: the node holds
  Gazebo while a cycle overruns (`core/sim_hold.py`, `sonar_hold_sim`), and MCS lists the generated live
  variants (`raycast_live_sonar.yaml` bit-identical, `raycast_fast_sonar.yaml` identical by eye). Verified
  live headless on 2026-11-09 (`raycast-live.md`: 1 548 pings at exactly 50 ms through the pier corner, 0 late,
  `/clock` keeps publishing while paused). Still to see: a full MCS + GCS mission (the `.svlog`, seabed images
  without repeated-ping stripes, the wall time vs the ~1.3 h / ~0.9 h estimate, an MCS stop mid-hold).
  2026-11-09 23:23: an MCS ray-cast launch with the Gazebo GUI froze (the GUI segfaulted at start-up in
  gz-transport's discovery `getenv`; `raycast-live.md`). Fixed: `headless` auto (server only for the
  ray-cast), the launch stops with the reason when Gazebo exits by itself, the hold re-detects a late
  Gazebo. 2026-11-10: server-only hid Gazebo from MCS runs; the GUI is back as a separate respawning
  `gz sim -g` client (a GUI crash no longer reaches the server). To see live: the Gazebo-exit shutdown and
  the GUI client (window opens, a killed `gz sim -g` respawns while `/clock` runs on) — smoke-checked only.
  Engine items left, not done because they change renders or need a study (owner, 2026-11-09: optimise,
  do not change the model):
  - (a) Quay boxes are sampled whole at 5 mm (buried part, back face, top above water, bottom):
    23–38 % of all contributors can receive no sound, yet each pays every multipath bound and pair.
    Dropping them before the bound is NOT exact (1 / 176 route side-pings changed by up to 5 dB: they
    inflate the loose bound); dropping them only from the pairs stage would be. The above-water face is
    also not occluded by the surface in the bound.
  - (b) `GpuMultipath._select_set` costs contributors × candidate sequences (one launch + sync each, no
    pruning of the children of dropped sequences): 71 candidates at a pier corner, ~0.13 s each.
  - (c) Diffraction keys its lit-side table on 2-D xy cells (`neighbour_cell_m`), so blocked points above
    the waterline / below the seabed flag a whole wall column; the factor kernel then runs on every
    wall-face leg near the waterline, toe and box ends (1–5 s per quay ping-side).
  - (d) Every kernel is fp64 on an RTX 4060 (1:64 fp64 rate).

### [P1] Optimise the ray-cast `fast` variant (owner order 2026-11-10) — built, owner look test pending
Plan and as-built notes: `blueboat_sss_sim/docs/engineering/raycast_fast_optimize_plan.md`.
- Built: culling of unreachable points, reuse of the echo-path selection, closed-form edge diffraction
  (diffraction back on), fp32 visibility; only `raycast_fast_sonar.yaml` turns them on (`config.ENGINE_KNOBS`).
- Default and live are unchanged (guard `~/ros2_ws/sim_runs/raycast/fast_opt/guard.py`); `test.raycast_test
  --only F` passes.
- Offline: pier-corner ping-side 2.04 → 0.29 s, quay 0.91 → 0.43 s.
- Open:
  - the owner's live look test (harbour, MCS "Ray-cast — fast" vs "Ray-cast", x0.5);
  - if more speed is wanted, the fp64 chain geometry of the pair and bound kernels (now the dominant cost).
- **Phase 2 built 2026-11-10** (§4b of the plan): fast at 10 mm + `band_cull`, floor-pair reuse, exact
  overhead cuts for every profile, hold margin 5 ms, `real_time_factor` and MCS Slow-down up to x4. Open: the
  owner's live test of fast at x2–x3 (look: speckle grain at 10 mm, the band cull near quays; whether the boat
  still tracks the path above x1).

### [P1] Ray-cast calibration: the harbour-first hybrid (2026-11-07) — testable, EXCLUDED by its rule
Phases 0–6 are built, and the GPU path is complete. Phase A / attempt 2 and Stage 2 failed their
physical gates, so the owner redirected the work to a physical core plus a separate empirical correction
(`docs/engineering/raycast_assessment.md` §13; gates amendments 6b/6c/6d in `~/calib/raycast/`).
- Correction v2 cuts the harbour first-return crest residual from 7–10 to 1.7–1.8 dB.
- Both acquisition groups are still EXCLUDED by the pre-registered rule: P2 fails at 9–10 m, and the S6 speckle
  stability check fails.
- Owner: a known limitation, not a no-go. The testable profile is
  `~/calib/raycast/correction/raycast_harbour_hybrid_v2_sonar.yaml` (outside the repository).

Open:
- NC #4 for the hybrid (not triggered by the rule);
- S7, the quay face;
- noisy rows for every in-domain stratum;
- the before-nadir rise at 9–10 m, which needs the harbour-world replay;
- the crest window's along-track speckle correlation: 0.03–0.09 in the simulation against 0.23–0.62 in real
  recordings.

### [P1] Ray-cast GPU memory: CUDA error 101 above 40 m (diagnosed 2026-11-07, open)
Error 101 is device out-of-memory, not an ordinal bug (`raycast_assessment.md` §13.7).

Peak per ping-side, harbour worst case (the quay inside the swath), one renderer (the live node), measured on an
RTX 4060 8 GB:

| range | 20 m | 25 m | 30 m | 35 m | 40 m | 50 m |
|---|---|---|---|---|---|---|
| peak | 2.3 GB | 3.1 GB | 3.6 GB | 5.7 GB | 6.3 GB | **OOM** |

- An offline replay holding several renderers (`scene_replay.render_rows`) fails at 40 m already.
- The README advice is to keep the ray-cast at ≤ 30 m (20 m is the survey setting).
- Fix candidates, not applied:
  - release every cached renderer's buffers before a retry;
  - process the per-ping scatterer pipeline in range segments. This is exact, and needs G1.

### [P1] Simulator analysis with the beach box validation: `sim-report/` (2026-11-12, evening)
`sim-report/simulator_analysis.md` (+ `.pdf`, `figures/`) takes over `sim-status-report.md` and adds the real
Nafco box of 2026-09-07 (`beach_sept/object_boite*.svlog`), replayed offline on the real boat track with the
geometric, harbour-fast and harbour-live sensors. Evidence: `~/ros2_ws/sim_runs/beach_box_validation/` (README in
the report's section 13). **This opened the ray-cast's sealed beach hold-out (owner's sign-off, 2026-11-12): the
nine `beach_sept/object*` files can no longer serve as an untouched test of the ray-cast.** No simulator source
was changed. Verdict: harbour-fast is suitable for the harbour experiments with stated limits; the flash at an
aligned face (level and width) is not validated by any real datum and needs a heading experiment with the
object's yaw surveyed (report section 11.3).

Corrections to this module's records (**applied 2026-11-14**, session 01 of the completion round: dated
correction notes in `analysis/object_signature.py`, the smoke `[14]` comment and `docs/sonar_model.md`
section 12; `sim_review_work/.../build_field_world.py` is outside the module and keeps the old position):
- **The field box's position was 19 m off.** `sonar_model.md` section 12, `analysis/object_signature.py`
  (docstring, `--at -15.9 17.7`), the entry "the box is still ~6 dB hot" history and
  `sim_review_work/object_physics_2026_09_28/build_field_world.py` place it at GPS 33.9307498 N, 130.73129505 E.
  The pings put it 16.0 m west and 5.5 m north of the beach anchor, in 1.10 m of water (not 1.50 m): the old
  images were georeferenced with the ping's own heading field. The numbers of `sonar_model.md` section 12
  ("34 of 36 headings within 2 dB", transducer 1.35 m over the seabed) describe a geometry the box was not in;
  the report's section 8 replaces them. Smoke `[14]` uses those field numbers as targets: to review.
- **The box is in the last three object runs only** (`object_boiteVsDistance`, `object_boiteEncore`,
  `object_boite_shobbit`); the six earlier `object*` runs pass the same lines with no box there (controls).
- **The worlds `~/worlds/plage_bathymetry2/plage*` and the 2026-09-28 sessions cited below no longer exist.**

Found by the beach validation, all open:
- **Ray-cast:** a hollow shell is opaque. Real box shadow: mean -8 to -11 dB, darkest part -8 to -14 dB;
  harbour-fast: up to 4 dB deeper in the mean, about -20 dB at its darkest (on the 18 dB floor), dark stretch
  0.5-1.1 m shorter. Category "acceptable limitation" in the report.
- **Flash width differs between the renderers**: at 6 m (8 deg grazing) a box face is visible within ~1 deg
  (geometric, Gaussian beam weight) or 3-5 deg (ray-cast, aperture side lobes). On the object ladder at 23-37
  deg the geometric was the wider one for cylinders. Unvalidated either way.
- **Harbour profiles at beach depth** (sonar altitude < 3.5 m): seabed 1-9 dB too dark, as their own rule
  implies. Only matters if beach images must match real levels.
- **Geometric:** +2 to +4 dB from 10 m at the beach; seabed without patches (real control passes show dark
  patches in 13 of 18; the empty simulated world in none).
- **Perception's label rule** (`core/sim_labels.ping_object_bins`): on a rising seabed at low grazing the
  label's shadow part is 2-4 m too long (the flat-floor tip wins over the renderer's mask); 51-77 % of its bins
  are dark. Fine on flat floors. A request for Perception, not an edit here.

### [P1] Sensor comparison for the thesis: `sim-status-report.md` (2026-11-12); the owner decides
The three sensors (geometric, ray-cast harbour-fast, harbour-live) against the real
`toOptimize-harbourScan` run and on a line of 28 test objects: `sim-status-report.md` (git root).
Ranking for harbour-depth water at 20 m: harbour-fast, harbour-live (same pictures), geometric. It
rests on appearance; no measured aspect response exists yet. **Owner's decision (report §8.4):** the
model that makes the detector's training images must be the one the beach heading experiment
bounds; the harbour profiles are not valid at 1–3 m. Three ways: a ray-cast beach calibration and
the sealed hold-out opened once; the geometric everywhere with its gaps closed; or both, stated.
`.claude/CLAUDE.md` §3.1 and `docs/simulator_justification.md` state the sensors' status differently
and were not edited.

Open items the comparison found (numbers and sections in the report §8.6):
- **Geometric:** seabed ~4 dB hot at 1200 bins / 44.3 µs (pulse-length term, already listed);
  nadir crest +8–9 dB and water column +12–14 dB (the 600-bin near range used at 1200 bins);
  speckle without deep fades (dark tail 4 dB short); rough blocks at the receiver's ceiling and
  drawn as a rim (+8 to +12 dB against the real mooring blocks); open-frame classes as blobs with a
  solid's shadow.
- **Ray-cast (both harbour profiles):** a hollow shell is opaque (storage box shadow −13 to −15 dB
  where the field box measured −7.4 / −8.2 dB); the dark stretch behind a solid is 4–28 % shorter
  than the flat-floor geometry (cause not established); 2 of 28 labels mark objects with no visible
  trace (mesh crate 0°, bicycle 90°: Perception's dataset needs a minimum-signal rule); rope has no
  verified parameters (sand law, logged).
- **Harbour-fast:** stripes along range at 39 m (none at 18–27 m).
- **Harbour-live:** rows 10–20 dB too dark inside the shadow of the 2.8 m mooring block, deeper in
  the sea run. Not diagnosed.
- **All three:** no highlight on an upright 200 L drum (the wall-seabed corner path is not
  modelled); `.svlog` time stamps on a 10 ms grid (a 55 ms period is written 50 / 60 ms); uniform
  seabed (ping-to-ping correlation 0.02–0.03, real 0.15).
- **Geometric against ray-cast:** the broadside flash of a pipe or cylinder has a different width
  at the scale of 1° (0.4–1.5° off: geometric still at the ceiling, ray-cast +2.5 to +6.7 dB).
- **Not the sensor (Control, add-only: a request, not an edit):** the boat's heading jumps by ~3°
  in 0.5 s each time it crosses ±180° (due west) and wanders up to 3.3° about a westward line
  (MPC); on an eastward line it holds 0.01°. It images an object twice and changes the aspect of
  mirroring objects. Every westward survey line is affected.

### [P1] Harbour validation runs recorded (2026-11-12)
The real `toOptimize-harbourScan` path on a world of the real harbour, three sensors, calm and with
low waves + current: `~/ros2_ws/sim_runs/harbour_validation/README.md` (world, path, range plan, the
sessions `SSS_data/sessions/harbourval_*`, the real reference in the same format, the object ladder,
the analysis). Recorded with the superproject's `tools/sim_record`. **Correction:** in the first
six runs the third range change was 31 m early (162.9 m instead of 194.4 m; `cost_probe.py` searched
80 m ahead at most): fixed, and repeated as `calm_geometric_r2` / `calm_harbour_fast_r2`. Found on
the way, all open:
- **Harbour-live cannot follow a real path at the real ranges under 2 h.** Whole path, 18–39 m:
  4 h predicted. The time is `diffraction.factor` (sampled edge diffraction): 3–8 s per ping within
  ~15 m of a 4 m mooring block (steep terrain), ~3 s per ping wherever a quay is in range, 0.15 s in
  open water (`world_src/cost_probe*.json`). The fast variant's closed-form edge needs 14 min.
- **A range change during a run leaves a ping gap**: 0.6 s geometric (the GCS's off / set / on),
  ~3 s harbour-fast at x2. The ray-cast rebuilds for the new acquisition at re-enable while Gazebo
  runs on; the hold only covers a render. The real device has no gap. Fix candidate: pause the
  world during the rebuild.
- **At 38–39 m the device pulse is 64 µs**, outside the harbour profiles' measured near range
  (44–50 µs): the shared model's ringing and water column apply there (logged). The real harbour
  run spends 232 of its 478 s at those ranges.
- **The boat spawns heading east, whatever the path.** The world launch has no spawn yaw; a path
  starting southwards was followed 5.8 m wide over its first 35 m (MPC). Worked round with a
  lead-in in the path; a `spawn_yaw` for the world launch would remove it.
- **World facts for this harbour** (the owner's older Shiraishijima worlds differ): the pier's end
  is a floating pontoon, not a quay to the seabed (real `OneLineHarbourClean70m`: the seabed behind
  it is 2–6 dB weaker, not shadowed); the survey's two 4 m "mounds" are the pontoon's real mooring
  blocks (real sonar: 3.9 m along-track, 1.7 m by the shadow) and need no correction, contrary to
  `sim_runs/waves_review/REPORT.md`.

### [P2] Re-stamping a saved rested object is not idempotent (found 2026-11-02)
`objects._rasterize_solid` writes the settled pose back into the object, including the bank tilt.
A second stamp of a saved object on a slope therefore tilts it twice: up to 0.89 m height
difference, measured on a synthetic 4 % slope. No saved world is affected, and nothing re-stamps
today. A tool that ever needs to recover rasters from saved objects must not re-stamp them (the
ray-cast mesh reads the stored raster). `raycast_phase1.md`.

### [P2] Facet layer: small cylinders ~0.2 dB under k a L^2 (found 2026-11-02)
The facet tessellation's curvature criterion (0.05 rad sagitta phase, `solids._curved`) tiles a
cylinder (a = 5 cm) with planar patches that act as a grating. Measured: the PO sum is 0.20 dB under
the closed form even at 500 m, and converges to the exact integral with 4x finer circumferential
patches. Smoke `[14]` tests at 0.5 dB, so it passes. This affects the geometric renderer's smooth
objects (not changed; the ray-cast sensor uses a 16x stricter criterion). `raycast_phase2.md`.

### [P2] Ray-cast: full texts wanted
Ten items are listed at the end of `docs/literature/raycast_sonar/INDEX.md`. None blocks Phases 1–6.

### [P2] Ray-cast: measurements wanted
Updated 2026-11-03 with the owner's answers (`raycast_design.md` §12):
- every site is seawater; salinity and temperature are unknown, so per-site NOAA OISST estimates are
  used, ±3 °C; S 35 ± 3 ppt. A thermometer at the next field session would replace them;
- a temperature profile (refraction);
- the transducer CAD housing is 318 mm (aperture bounded [22.4, 31.0] cm);
- two-unit interference and the transmit waveform are being inferred from the recordings.

### [P1] Real images are range-compressed 2.5–3 % (sound speed), the geometric renderer is not (found 2026-11-03)
Every corpus file reports `sos_dmps` = 15000: the device maps time to range at 1500 m/s. The corpus
water is warm seawater, ~1537–1545 m/s (per-site OISST temperatures, ±10 m/s), so real images show
objects and walls 2.5–3.0 % closer than they are (25–30 cm at 10 m). The geometric renderer uses 1500
m/s for both the travel time and the bins, so it has no such compression. Any comparison that matched
simulated and real ranges, e.g. wall positions in the pool, absorbs this scale somewhere. Not changed
in the geometric renderer. The ray-cast sensor reproduces it (true c for delays, 1500 for bins).
Note 07 of `docs/literature/raycast_sonar/`.

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

- [ ] **Omniscan status frames (id 2194) on `~/raw`.** Since 2026-10-27 the real
  driver forwards every device frame, and SonarView's recordings carry status
  frames at ~0.9 Hz per device. Whether the device sends them to our client
  unsolicited is the open BlueBoat-SSS field check (`sssTODO.md`). If it does,
  emit them here too (layout `<ffI3xB`, the floats' meaning unestablished —
  carry corpus-typical values, documented as such) so the raw stream stays
  CM-2-identical. **Needs the boat.**

(The 20 Hz vs 22 ms ping-rate question that sat here was settled on 2026-09-02
from the corpus — `period = max(50 ms, 2R/c + 3 ms)` — and **re-read on
2026-10-27**: the 50 ms is SonarView's `msec_per_ping` command, every corpus
file being SonarView's; the device free-runs at 29.0 ms at 20 m (the real
driver's 2026-10-27 recordings), so the old "22 ms at 15 m" (= 2R/c + 2 ms) was
the device's free run, not wrong. `msec_per_ping` now defaults to 50 and
`max_ping_rate_hz` to 0; `sss_svlog_compare` still reports the law residual per
file.)

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

### [x] The reported peak is still ~5 dB hot — resolved 2026-10-30 (pass 2)
The device's `max_pwr_db` is mostly not the bottom return (ceiling pile-up at
1–2 m, the bottom return at 3–6 m, the transmit ringing above ~8 m), and
`specular_strength` 12 had been fitted to it. Pass 2 promoted bundle B through
its pre-registered gate: `receiver_ceiling` on (67.9 dB, measured) and
`specular_strength` 12 → 1.75 (the detector-located FBR crest, raised to the
NC #4 envelope bound). The FBR crest bias fell on every session (e.g. +8.8 →
+0.9 dB on the harbour anchor), the peak now sits within 4 dB of the device
law at every altitude (mean +0.7), smoke `[2c]` re-anchored.
`docs/engineering/calibration_results.md`.

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

### [x] Staged field calibration on session-split data — pass 2 done 2026-10-30
Engine `calibration/seabed.py` (`score`, `fit`, `scan`, the NC #4 FBR envelope);
bundle B promoted, A and C provisional in `config/field_candidate_v1_sonar.yaml`.
`docs/engineering/calibration_results.md`; the open items it leaves are below.

### [P1] Pulse-length level law: confounded with the site, heterogeneous within file
The real level rises with the pulse inside a file (+0.91 dB/dB pooled), which
the shipped model ignores: most of its +3 dB at 1200 bins / 44.3 µs. With k = 1
and the level triple refitted (bundle A) the 1200-bin sessions improve strongly
but harbour 07-21 gets 3 dB worse and `lambert_exponent` sits on its 0.5 bound —
each calibration site has one pulse, so k and the site level are one parameter.
The within-file slope also differs by session (beach B2 +0.67, harbour 07-22-AM
+1.21): possibly a receiver-filter floor on the effective pulse at the short
end (hypothesis). Needs: >= 3 altitudes over ONE classified bottom with the
acquisition cycled at each (`validation_and_experiment_plan.md` §6).

### [P2] Within-file altitude residual of the seabed level
Inside the harbour anchor, sim − real rises from −2 dB at 6 m to +2 dB at 14 m
after any refit; deeper harbour water may be a different bottom. Same capture.

### [P2] Real port/starboard level imbalance (~1–1.5 dB) is not modelled
Measured at both sites in every calibration fit (starboard reads lower). A
per-transducer gain term would be a device constant; not added in pass 2.

### [P2] Nadir ringing is a plateau then a fall, not dB-linear
Harbour anchor: ~49.6 dB to 0.5 m, then fast decay; the model's 55 dB −
22 dB/m is 3–6 dB off at 0.5–1 m. A dB-linear refit on the first metre
over-fills the water column further out and lost the FBR at 5–10 m altitude
(NC #4 envelope), so the shipped values stay. Needs a two-segment ringing
shape, then the envelope.

### [P2] K-distributed seabed (texture_k_inv_shape) not promoted
ν ≈ 0.67 matches the calibration sessions' power-minus-dB and per-bin std, but
raised harbour 07-22-AM's FBR crest +1.4 dB and overshoots its power-minus-dB;
track lag-1 also rises (0.74 vs 0.37 real on the beach anchor). Ships off;
on in the candidate profile. A range- or depth-dependent ν, or a smaller K
cell, are the next hypotheses.

### [x] Gazebo visuals stood every object on its own stamp — fixed 2026-11-02
Found on the operator's screenshot of `pool_2w` (the boxed weight 0.55 m above the floor on a
pyramid). `write_world_sdf` placed each object at `scene.sample_height` -- the STAMPED raster,
i.e. the object's own top -- and meshed that raster, so each stamp became a bump. **Visual
only**: the facet layer takes the seabed as `height - facet_bump` (every facet patch spans
floor to top, measured). Fix: `write_world_sdf(terrain_height=...)` -- objects, walls and
`seabed.stl` stand on the pre-stamp terrain (`build_world_folder` passes `depth.npz`'s raster,
`generate_world` the new `generate_scene(terrain_out=...)`; without it the facet stamps are
removed, `height - facet_bump`); smoke check in the limits/SDF block. **Every world built
before 2026-11-02 with objects shows them floating in Gazebo** (sonar unaffected); a re-save
(Edit) gives a corrected `_i` folder. The pool worlds were rebuilt that way:
`~/worlds/pool_line_and_back/{pool_empty,pool_1w,pool_2w}_1`, `scene.npz` / `depth.npz` /
`scene_manifest.yaml` bit-identical to the originals (`~/calib/pass3/worlds_v2/rebuild_report.json`),
box centre -0.072 -> -0.622 m, nested weight -0.300 -> -0.850, bare weight -0.752 -> -0.844.

### [x] Pass 3 (pool_october) done 2026-11-02 — shipped as a candidate, not "v1 stable"
**Road to v1 stable**: `~/calib/pass3/V1_ROADMAP.md` (field week first; the pool TEST runs are spent,
so v1 needs a new reserved hold-out).
The 2026-10-27/28 pool recordings (18 runs) were inspected, split (operator-approved: FIT 10-28
early, VALIDATION 10-27 + 10-28 line runs, TEST the two 2-weight runs), rebuilt as worlds
`~/worlds/pool_line_and_back/{pool_empty,pool_1w,pool_2w}`, scored (phase 3), fixed and fitted
(phase 4: only the installation moved), and tested once (phase 5). Release criteria fail on
TEST: seabed level +9.5 dB (painted floor vs a sand stand-in), FBR crest +1.7 dB, objects
under-detected (5/15 passes vs 14/15) and over-shadowed. Public summary:
`docs/engineering/calibration_results.md` Part II; private: `~/calib/pass3/report_2026-11-02.md`,
`RUNBOOK.md`, `GCS_PLAN.md` (the BlueBoat-SSS half, plan only). Open items it raised are below.

### [x] Pass 4 (2026-11-02): shallow-water image comb, the painted floor, appearance -- nothing promoted
Record: `~/calib/pass4/report_2026-11-02.md`, `docs/engineering/calibration_results.md` Part III,
`docs/engineering/raycast_handoff_from_pass4.md`.
- The default changed only by the P0 fix (a ghost's coherent lobe was ×1000 hot): pool and field
  scores bit-identical, envelope 82 / 0.
- Every new term ships off. The pool overlay is `config/pool_environment_pass4_sonar.yaml`.

Open items it leaves:
- **[P1] Owner decision**: is a non-sand (painted-concrete) floor part of v1, or is the v1 level
  criterion restated for sand sites? With the overlay the pool scores LEVEL +0.65 / +0.18 dB.
- **[P1] Seabed comb vs tracker lock (S, NC #4).** With S on, the pool tracker lock falls up to 0.33
  under the device (FBR detection stays ≥ 0.995). The comb is diffuse-only, and the real pool FBR ramps
  0.6 → 0.9 m: the direct/surface mixed path. Image the coherent FBR lobe too, then re-gate.
- **[P2] Wall comb (W2) strict gate.** VAL beyond 0.5–2 m is +2.4 dB median, but 3/18 sides exceed
  5 dB (pool_mapping1 port 5.4 with the floor fixed). Faces at 3–6 m: the tail is 4–5 dB too strong.
  Check against a quay at two depths (V1 roadmap 1.4).
- **[P2] Object comb cost** ~360 ms per ping-side (live budget 25 ms). Prune pairs by the specular
  condition, or precompute per object.
- **[P2] Shadow zone (O primary)** 3.3 / 4.6 dB darker than real after the echo end, with the comb
  (gate 3).
- **[P2] 10 m vs 20 m step (K) is recorder-dependent?** Pool (our driver) −2.1 dB vs SonarView field
  0 dB at a fixed pulse. Record 10 m and 20 m with both recorders in one session.
  `receive_window_level` is kept (off) for it.
- **[P2] B1 premise refuted by data.** Real pool box rows darken beyond the wall face (VAL median
  −2.85 dB). `ghost_share_split` failed its gate (off). Comb paths need visibility (ray-cast hand-off).
- **[P2] Texture (B3)**: range kernel shape (lags 2–3 too correlated: boxcar vs pulse ⊗ filter); a
  per-ping level fluctuation is missing (0.57 vs 0.9–1.5 dB); scene patchiness. Target table:
  `~/calib/pass4/step1/DB3/REPORT.md`.
- **BlueBoat-SSS (proposed items, not ours to edit; `~/calib/pass4/step1/D5b/REPORT.md`):**
  - autopilot rates hold ~9 s per 30 s re-assert on the water (4 Hz otherwise);
  - the watchdog window starts before logging;
  - the zero-stamp warning repeats on every message;
  - the odom buffer is never pruned after a zero-stamped first sample;
  - superproject §4.4's "~13 Hz until reboot" is contradicted.

### [P1] Texture: the pool's extra fluctuation is not ground-fixed (pass-2 C refuted here)
Pool floor: per-bin std 6.29 dB real vs 4.99 default (power-minus-dB 3.47 vs 2.17), but range
and track lag-1 already match (0.54/0.14 real, 0.55/0.12 default). Every K texture of the scan
(1/nu 0.5-1.5, cell 0.05-0.5 m) closes the spread only by adding correlation
(`~/calib/pass3/phase4/texture_scan/`). Needed: a per-sample heavy-tail mechanism with a physical
basis (fluctuating medium / interfering surface paths in shallow water), or evidence that the
painted floor alone explains it (the next sand session).

### [P2] Near-range ringing: one device shape cannot fit pool and field (R)
Measured over nine acquisitions (`~/calib/pass3/phase4/ringing_profiles.json`): plateau then a
steep fall, -10 dB at 0.09-0.14 m (8-17 mm bins) / 6-11 bins (33-80 mm bins); the shipped
dB-linear 55 - 22 dB/m leaves the pool's water column +19..+23 dB. A two-regime fit improves the
pool (rms 20 -> 5-7 dB) and worsens bath3 / the harbour anchor -> not moved. Separate an
environment water column from the acquisition: the pool's acquisition (20 m / 1200 / 44 us) in
open water.

### [P2] Object echo deficit on the pool (O)
Boxed weight: echo 12.5 dB under the device vs the local floor, no extent (0 vs 0.9 m); on TEST
the box reaches 10 dB on 1/8 passes (device 7/8), the bare weight on 4/7 (7/7), and both cast
shadows on ~40 % of passes (device 0 %). No material/geometry constant closes it (NC #17).
Next: the same weights on a sand bottom (separates object physics from the floor material),
and the waveguide image comb of the wall item (the real echoes carry a tail the renderer lacks).

### [x] Per-ping `vehicle_heading_deg` in REAL recordings is not the boat's heading — fixed 2026-11-02
Measured 2026-11-01 (pass 3 phase 1): offset to the autopilot ATTITUDE yaw differs per
transducer and per run — pool −44…−168°, beach bath3 −77/−56°, object_boiteVsDistance
−107/−120°, harbour fullHarbourCleanExample +90/+60° — while ATTITUDE yaw and GPS heading
agree with the course over ground within ~7°. Fixed in pass 3 phase 4:
`analysis/svlog_reader.profile_heading_deg` (ATTITUDE yaw, MAVLink NED, on the profile
clock or file order when the clock is frozen; the ping field only without ATTITUDE), used by
`pose_track` and `analysis/backscatter._iter_placed_profiles`; smoke `[4c]` checks a file
whose ping headers are 80° off. Any pre-2026-11-02 output of those two tools on a real
recording placed every profile with the wrong look direction.

### [P2] Device level depends on the bin width at a fixed pulse; the simulator does not
Pool, same site/altitude/pulse (44.3 µs), minutes apart: 10 m / 1200 bins reads −2.0…−3.0
dB against 20 m / 1200 at 1–8 m slant (FBR crest −2 dB, leading edge −7 dB, nadir ringing
+5 dB; outside the ±0.6 dB spread of the 20 m runs). The renderer is bin-count invariant
(`sonar_model.md`). Close to energy-per-bin. Pass 3 phase 4 (device stage).

### [P2] Wall echo shape and the beyond-wall level (E): a waveguide image comb (pass 3)
Pool walls placed within ±0.1 m (median) of the measured outline; the simulated face line
sits at the geometric face, but the real wall echo is followed by a ~1 m plateau (−1…−3 dB
under the face peak at near-normal plan incidence) and the simulator is 30 dB under the
device 0.5–2 m past the face (phase 3). Pass 3 phase 4 (`~/calib/pass3/phase4/PHASE4_REPORT.md`
§T): **not a device kernel** (the boxed weight's tail is 11 dB weaker, same pulse), **not
plan obliquity** (present at < 5° incidence), and it **grows** with the face range. Inferred:
the wall is a mirror at near-normal incidence and the 0.9 m surface/floor waveguide (painted
concrete totally reflects below ~66° grazing) adds the transducer image's vertical images;
an offline image comb with the shipped beam (`~/calib/pass3/tools/wall_image_comb.py`)
reproduces the growth with range and the 1–2 m extent, 4–8 dB weak. Next: a renderer
image comb for coherent wall/object echoes (order limited by the beam), proved first against
the closed form, then the pool walls (FIT) and the harbour quay's 7–10 dB "beyond" deficit.

### [~] Transducer installation: 0.22 m deep, −0.345 m (drawing) / −0.60 m vs the pool nav fix
Operator (2026-11-01): the transducers are on the rear-most hull mounting holes, **the same
holes for every corpus**; the GPS sat ~0.10 m behind the bow for the pool corpus and at the
hull centre for the older ones. Measured (pass 3 phase 2, `~/calib/pass3/phase2/transducer_mount.json`):
`BlueBoat.dae` puts the holes 0.030 m above the keel, 0.345 m aft of the hull centre;
`BlueBoat.stl` buoyancy puts the waterline 0.216-0.241 m above the keel at 16-22 kg; the
pool (floor 0.90 m) reproduces every run's FBR altitude with **0.22 m** ± 0.03 m. **Done
2026-11-02 (pass 3 phase 4, bundle G):** `sensor_depth_m` 0.15 → 0.22 and `mount_x_m`
0.0 → −0.345 in `sonar/config.py` and every comparison profile (`field_candidate_v1`
frozen at 0.15/0.0), registry entries with the evidence. Open: (1) the pool walls give
−0.60 m from the navigation fix (FIT; VALIDATION −0.71), i.e. −0.10…−0.21 m from the hull
centre with the GPS where the operator put it, against −0.345 m on the drawing — measure
the acoustic centre / the EKF reference (reciprocal passes over a point target);
(2) `sss_processor_node` / the GCS moved to 0.22 m on 2026-11-02 (operator's order); their
fore/aft is still 0.0 and no georeference uses one — a per-session arm is BlueBoat-SSS's
(`~/calib/pass3/GCS_PLAN.md` item 4).

### [P2] Object edges (D) and walls (E) not calibrated in pass 2
D needs attitude/heave replay over the field-box pass before any `penumbra_deg`;
E (quay "beyond" zone 7–10 dB dark) needs a known structure. The walled-scene
speckle fill (behind a quay) is fixed; nothing else changed.

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

---

## Waves review + renderer picture comparison (2026-11-10) — hand-offs

Evidence: `~/ros2_ws/sim_runs/waves_review/` (`REPORT.md`, `out/`), plan
`~/.claude/plans/pasted-content-id-09ed-see-in-sleepy-hartmanis.md`. Done the same day:
wave levels Tp = 3.6 √Hs + `harbour_chop`, hybrid slope discretisation, draft decay,
corpus attitude-spectrum targets and gates, link-frame inertia mirror, opt-in
`scatterer_anchoring: ground`, `analysis/picture_compare.py`.

### [P1] Owner look test + live gate on the new sea (owner runs it)
Re-run the 2026-11-10 harbour protocol with `harbour_chop` (and `smooth`), geometric and
ray-cast, plus `test/gazebo_sea_test.py` (default `--waves harbour_chop`, spectral criteria).
Then `python3 -m blueboat_sss_sim.analysis.picture_compare` on the new sessions against the
rebuilt real ones (`~/ros2_ws/sim_runs/waves_review/real_sessions/`).

### [P1] Ray-cast, for its session (measured on the harbour runs; not edited here)
- **Shadows are empty**: anchor shadows −55…−65 dB re the seabed in front (A1/A2),
  detector input 0 (77–99 % pure black); real −21…−42 dB with speckle texture (sd
  6–7 dB), geometric −25…−36 dB textured. NC #15's reverberation floor is missing.
- **First return untrackable**: the GCS `row_bottom_m` jumps p90 0.35–0.42 m row to row and
  reads 0.3–0.8 m deep (geometric 0.02–0.03 m). The display model and the georeference use
  it, so ray-cast AI tiles carry row-wise normalisation jitter (NC #4 not established).
- **No ground-truth contacts**: no labelled datasets (CM-11).
- **Absolute level ~+11 dB hot at 20 m / 1200 / 44.3 µs** (harbour, altitude 5.5–7.5 m):
  plain seabed 54.4 / 48.2 / 41.5 dB at 8 / 12 / 16 m slant against real 43.4 / 36.6 /
  31.0 (`toOptimize` 18.4 m segment, alt 4.8 m) and geometric 46.6 / 41.5 / 36.6. In
  Perception's `fixed_gray` detector input (fitted on real data) 71–78 % of the ray-cast's
  plain-seabed pixels saturate at 255 (geometric 20–23 %, real 1.3–1.6 %): the texture the
  detector sees is clipped. The hybrid correction was fitted at 40 m / 600 / 106 µs; its
  transfer to the default acquisition is the known limitation, now measured.
These four are what stands between the ray-cast's better seabed texture and production use.
The geometric's +3–6 dB is consistent with the missing pulse-length level term
(field calibration 2026-10-23: +0.93 dB/dB, provisional in `field_candidate_v1`).

**Patched the same day for the harbour (items 1–3; `.claude/raycast-live.md` "Harbour
profiles", `docs/engineering/raycast_assessment.md` §14):** `config/raycast_harbour_live_sonar.yaml`
/ `raycast_harbour_fast_sonar.yaml` = the v2 harbour core (level), its near-nadir table edge-held
to 4.5 m (first return), the NC #15 floor at 18 dB on a per-ping ambient reference (shadows).
Still open:
- **[P1] Owner look test** of the harbour profiles: the harbour protocol with
  `sonar_config:=config/raycast_harbour_fast_sonar.yaml`, then `picture_compare --fixed-gray`
  (REPORT.md §4). Expected: saturation toward real 1.3–1.6 %, textured shadows ~−20…−28 dB, no
  0.5 m late bottom line. Promote nothing to the default ray-cast profile before it.
- **[P2] Ground-truth contacts** for the ray-cast (item 4, deferred by the owner).
- **[P2] D3**, the physical shadow floor at the corpus quay (blocked by CUDA 101): when it runs,
  the measured 18 dB stand-in can be compared with physics, and the default can stay physics-only.
- **[P3]** The v2 table above 11.3 m (off) and below 3.5 m (off); P2's 9–10 m bands stay 3.3–3.7 dB
  cold near nadir.
- **v2 (same day, after the owner's run):** shadow lines (the floor reference) and the lost bottom
  line (the 600-bin near range at 1200 bins) fixed in the harbour profiles; `raycast_assessment.md`
  §14.1. Open:
  - **[P2] The geometric carries the same 600-bin near range at 1200 bins / 44.3 µs** (ringing end
    0.50 m, noise estimate 40.6 dB, water column 26 dB; real 0.14 m / 28.5 / 14.5). Its sharp crest
    keeps the tracker locked, so nothing breaks, but the water column it draws is ~12 dB too bright.
    The fix (`HARBOUR_NEAR_RANGE` measured, a per-acquisition near range in the shared model) moves the
    comparison profile: NC #6, owner's call.
  - **[P3]** The ray-cast bottom line is now steadier than the real one (jumps p90 0.07 vs 0.88 m);
    heave and waves in a live run add some. Nothing to fit until a live comparison.

### [P2] Hull v2 (not in scope 2026-11-10; feasibility measured)
The model's pitch response is ~1.4× the real one at `harbour_chop` (1.03–1.18° vs
0.60–0.87°) and its natural periods are short (roll 0.42 s / pitch 0.72 s vs the corpus's
~0.8–0.9 s response, output-only ζ ≈ 0.1–0.25 vs judgement 0.4). Causes: BlueROV2 roll/
pitch added inertia (0.12 kg·m² vs ~3.1 / 2.6 kg·m² of demihull strip-theory added inertia),
damping sized, not measured. Routes: (a) Control add-only BlueBoat hydrodynamic profile
(owner order needed), or (b′) an exact delay-free "inertia scaling" of the sea node's own
wrench (Plan-agent design, SSS-Sim only; a literal −ΔI·α term is unstable at ΔI/I ≈ 1.2).
Also `hull.water_density` 1000 → 1025 (sites are seawater), and a dockside roll/pitch decay
test (ITTC 7.5-02-07-02.1 §2.3.5: ~10 cycles, 25 Hz ATTITUDE). Control's report calls 5.64
the yaw inertia; it is pitch in Gazebo (rotated inertial frame) -- Control's TODO.

### [P2] Geometric: make `scatterer_anchoring: ground` the default?
Opt-in since 2026-11-10 (`config/ground_anchored_sonar.yaml`). Default = deliberate
render-fixture rebaseline; decide after the owner's look test of wave runs.

### [P2] Geometric seabed texture is smoother than real (cross-ref B3)
Harbour plain seabed, gain 4, 18–27 m / 1200: 9×9 high-pass std 4.7–4.8 dB (real 5.4–5.5,
ray-cast 5.4), dark tail p1 −16.5 dB re the mean (real −20.2…−20.4, ray-cast −20.0),
intensity contrast 0.97 (real 1.06–1.09). Same diagnosis as B3 ("per-ping level fluctuation
missing", pool per-bin std 6.29 vs 4.99).
