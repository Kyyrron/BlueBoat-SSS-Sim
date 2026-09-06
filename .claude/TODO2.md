# TODO2 — verification pass for the 2026-09-03 pipeline (what to run, what to look at)

Companion to `TODO.md` (which tracks open engineering items). This file is the
**test list** for the whole chain after the 2026-09-03 sessions: ping assembly fix,
walls / pontoons, 3-D object rest poses, sea state (current + waves + wake groups),
MCS sea-state flow, GCS throughput, curved / merged walls. Every offline gate below is
green on this machine; the live items are the ones only a Gazebo / GCS session can
answer. Tick them here; record measured numbers in the module's `TODO.md` or the
relevant doc (the wall and sea numbers belong in `docs/sonar_model.md` §11 and
`docs/sea_state_model.md` §9).

## Important matters first

1. **Rebuild before any live test.** Both changed ROS packages install by copy:
   `cd ~/ros2_ws && colcon build --packages-select blueboat_sss_sim blueboat_sss && source install/setup.bash`.
   A stale install reproduces the old bugs (torn rows, walls that do not block).
2. **Never relaunch the sonar node under a running processor without the fix
   installed** — that was the "one black row in two" cause. With the fix, relaunching
   is safe: the processor re-bases within ≤ 24 pings.
3. **The wave wrench is one-shot by design**: if `sea_state_node` dies, the force stops
   (no runaway). A boat that keeps heaving after the node is gone is a bug.
4. **A wall the boat can drive over is a `pontoon`, never a `quay`.** Re-declare the
   crossed rectangle in `~/worlds/testGPS/shiraishi-jima-port` (Edit → double-click each
   of walls 6–8 → Kind = pontoon); a quay under the hull is a modelling error, not a
   renderer bug.
5. **Do not compare sim and real levels at different ranges**: the ping period, the
   range law and the quay contrasts are all range-dependent. Use the same
   `range_length_mm` and reduce both with `sss_svlog_compare` / `wall_contrast`.
6. **The GCS memory caps are now in samples**, not rows: a 30 m / 1200-bin run keeps
   fewer rows scrollable than before (by design). Raise `waterfall_max_samples` in
   `blueboat_gcs/config/default.yaml` if you need more.

## A. Build and offline gates (5 min)

| # | Command | Expected |
|---|---|---|
| A1 | `cd BlueBoat-SSS-Sim/blueboat_sss_sim && python3 -m test.smoke_test` | `ALL CHECKS PASSED` (~310 checks, ~100 s — the Stop hook's budget is 180 s; `[2f]` needs the corpus at `~/ros2_ws/data/SSS_data`; concurrent runs wait on a lock) |
| A2 | `QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke` | `GUI SMOKE PASSED` |
| A3 | `cd BlueBoat-SSS/blueboat_sss && QT_QPA_PLATFORM=offscreen python3 -m pytest -q` | all pass (31 assembly tests need the sourced workspace; 9 throughput tests) |
| A4 | `python3 -m blueboat_gcs.analysis.ping_bench --pings 4000 --turn` (same dir) | ingest ≤ 2 ms/ping, mosaic render ≤ 15 ms, waterfall ≤ 20 ms, cell ≈ 0.026 → 0.053 m after one budget coarsening, RSS flat |
| A5 | `cd BlueBoat-MCS && ruff check . && QT_QPA_PLATFORM=offscreen python3 smoke_test.py` | `All checks passed!` + `SMOKE TEST PASSED` (`sea state ok` present) |
| A6 | `colcon build --packages-select blueboat_sss_sim blueboat_sss` | no error; `ros2 run blueboat_sss_sim sea_state_node --ros-args -p current:=weak` starts and logs "waiting for Gazebo" |

## B. Ping pipeline (GCS + processor), 15 min live

| # | Test | Expected observation |
|---|---|---|
| B1 | Start Gazebo + `sss_sim_launch.py` on the world, GCS START, enable pinging | `ros2 topic hz /sss_processor/processed` ≈ the ping rate (20 Hz at 15 m), **not** 40 Hz |
| B2 | `ros2 topic echo /sss_processor/processed --field starboard_ping_number` | non-zero on every message |
| B3 | Kill and relaunch **only** `sss_sim_launch.py` while the GCS keeps running | processor log: "ping counter restarted … re-basing"; within 24 pings rows are two-sided again; GCS console shows no "rows are torn" warning after that |
| B4 | GCS waterfall at full zoom (Fit + zoom in) | continuous rows, no alternating black lines on either side; a warning line appears only if > 25 % of recent rows are one-sided |
| B5 | Leave the live view running 10 min (Record OFF), watch `top` | CPU steady (< 50 % of one core), RSS flat once the waterfall cap is reached (≈ 100 k rows × 1200 columns ≈ 480 MB + pixmaps); no "display N pings behind" status at rest |
| B6 | Drag the Contrast slider while streaming | one re-render per drag, no multi-second freeze |
| B7 | Record ON for 2 min, then OFF | session folder complete; no 50 ms hitches every 6 s (seabed images now save on a pool thread) |
| B8 | 30 m / 1200-bin profile (`range_length_mm:=30000 num_results:=1200`) for 5 min | same as B5; the waterfall keeps ~50 k rows |

## C. Sea state (current + waves + wake groups), 30 min live

Launch: `ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=$HOME/worlds/testGPS/shiraishi-jima-port with_control:=true trajectory_file:=$HOME/worlds/testGPS/shiraishi-jima-port/source_path_world.yaml sea_current:=strong sea_current_from_deg:=90 sea_waves:=slight sea_waves_from_deg:=45`

| # | Test | Expected observation |
|---|---|---|
| C1 | Null test: `sea_current:=none sea_waves:=calm` | `gz topic -e -t /world/generated_ocean/wrench` silent for 10 s; `/current` all zeros; odom z at the trim (−0.11 m) with no drift |
| C2 | Injection: `gz topic -l \| grep wrench` after launch | `/world/<w>/wrench`, `/wrench/persistent`, `/wrench/clear` listed (plugin native in new worlds, injected in old ones — the node logs which) |
| C3 | Impulse test: `slight` for 120 s, boat station-keeping | `ros2 topic echo /blueboat/odom` z oscillates ±0.1–0.2 m; mean z equals the trim to < 1 mm; mean vz ≈ 0; `gz topic -f -t /world/<w>/wrench` ≈ 100 Hz |
| C4 | Current direction: `strong from E`, controller off (`with_control:=false`) | boat drifts **west** at ~0.5 m/s |
| C5 | Wake groups: `smooth` for 10 min | the SEA STATE box / `/sim/sea_state` shows "wake group … s left" about every 5 min, each 30 s; heave visibly grows and dies with a 5–6 s period; between groups the ~3 s chop only |
| C6 | Irregularity: watch odom z for 60 s under `slight` | no fixed-period metronome; periods vary 2.5–5 s; groupy amplitude |
| C7 | Sonar coupling: GCS waterfall under `slight` | port/starboard banding (roll), altitude ripple (heave), along-track wobble of point targets (pitch) |
| C8 | Gazebo / RViz indicators | arrow + text markers in the Gazebo GUI (`/marker_array`); `/sim/sea_state/markers` in RViz |
| C9 | Stock world: MCS "Gazebo simulation" without a world, current `strong` | console shows the `[sea]` companion `sea_state_launch.py world_name:=ocean`; the node logs the runtime `ApplyLinkWrench` injection; boat drifts |
| C10 | Confirm once: `gz model -m blueboat --link`; `gz service -i -s /world/<w>/entity/system/add` | canonical link is `blueboat/base_link`; world entity id 1 (else set `target_entity` / fix the injection request) |

## D. Walls and pontoons, 15 min

| # | Test | Expected observation |
|---|---|---|
| D1 | Re-declare walls 6–8 of the test world as `pontoon` (Edit mode → save as `_1`), run the survey | crossing the rectangle: no bright slab in the water column, no shadow; only a faint edge at the nadir-gap edge |
| D2 | Quay (wall 2) in the GCS mosaic | bright band (~0.8 m) at the face, **dark** zone behind it, faint mirrored texture further out — compare with `real_walls.png` |
| D3 | `python3 -m blueboat_sss_sim.analysis.wall_contrast <recorded sim svlog>` | face +15…+25 dB, 1.5–4 m beyond ≤ −8 dB, 6–12 m beyond ≤ −15 dB (the corpus anchor: +21 / −12 / −23) |
| D4 | Draw two walls meeting at a corner within 0.75 m; build | snap ring appears while drawing; `metadata.yaml` has one shared corner point; the Screen-3 depth fill does not leak through the corner |
| D5 | Select a wall, drag its hollow middle handle; build; open in Gazebo | a curve on the canvas; `world.sdf` has `wall_<name>_s01…` boxes following it; `metadata.yaml` walls carry `curve: <name>`; "Straighten wall" removes it |

## E. Objects v3, 10 min

| # | Test | Expected observation |
|---|---|---|
| E1 | Objects page → drum_200l → "let them fall / roll (3-D)", randomness 1.0; build | Info… lists "standing 36 %, inverted 9 %, lying 56 %"; `metadata.yaml` objects carry `rest`, `roll_deg`, `pitch_deg`; Gazebo shows lying drums on their side, none floating, none sunk below the seabed |
| E2 | Same world with the toggle off | identical to a pre-2026-09-03 build (bit-identical `scene.npz`) |
| E3 | Survey over a lying drum at the default size | a glint line at the top, short shadow; not the slab + long shadow of the giant standing drums |

## F. MCS flows, 10 min

| # | Test | Expected observation |
|---|---|---|
| F1 | Launch → Gazebo simulation → (world dialog) → **Sea state** dialog | appears for both empty world and personalized world; presets show descriptions + references; Cancel aborts |
| F2 | Choose "Custom" waves (Hs 0.3, Tp 4, 9 groups/h) | `~/.config/blueboat_mcs/sea_states/custom_waves.yaml` written; the launch line carries `sea_schedule:=…custom_waves.yaml` |
| F3 | Timeline: add two rows, save as `build`; relaunch and pick it | `sea_schedule:=…/build.yaml`; SEA STATE box "Next change: in N s" counts down and the readback changes |
| F4 | Left panel SEA STATE box | visible only during a sim run; Current / Waves / Surface / Next change / Status live within 1 s (latched topic) |
| F5 | "Modify situation…" → strong from S, ramp 10 s → Apply | readback ramps over 10 s; Gazebo arrow turns |
| F6 | Stop Mission in the empty-world case | the `[sea]` companion terminates with the mission (no stray `sea_state_node` in `ps`) |

## G. Not verifiable on this machine / needs field data

- **Pontoon face level** (`wall_pontoon_specular`, `wall_face_diffuse`): no pontoon
  geometry ground truth in the corpus. A recording next to a known pontoon at known
  distance would anchor it (`wall_contrast` on that log).
- **Quay roughness** (`roughness_m` 3 mm): fitted only through the far-ghost level; a
  second quay of known construction (sheet pile vs rubble) would separate roughness
  from reflectivity.
- **Corpus quay port/starboard spread** (beyond −5.6 vs −17.4 dB): the model sits
  between; unknown whether it is the two quays' geometry or the two transducers.
- **BlueBoat hydrodynamic coefficients**: Gazebo applies the BlueROV2's; a tow or
  free-decay test of the BlueBoat (BlueBoat-Control item) would fix the wave response
  amplitudes.
- **Wake-group statistics** (rate / height / period per preset): sized by judgement;
  one day of counting vessel passages at the basin would anchor them.
- **~8 % real-device ping loss** (BlueBoat-SSS `TODO.md`): unrelated to the sim,
  still open.
