# BlueBoat-SSS-Sim

Synthetic side-scan sonar platform for the BlueBoat USV — a ROS 2
(ament_python) package with two separated deliverables:

1. **World Builder** — PySide6 wizard producing **GPS-anchored Gazebo
   worlds** around the MCS Pattern Designer's GPS-anchored survey paths
   (limits + reflective walls on a satellite map, authored bathymetry, a
   litter catalog scattered statistically **or placed by hand**), saved as
   immutable folders `~/worlds/<path_name>/<world_name>/`;
2. **simulated sonar** — a **drop-in replacement** for the real dual
   Cerulean Omniscan 450 SS interface; the downstream stack runs unmodified.

**Purely additive**: modifies no control stack, robot description,
interfaces package, MCS or GCS. Missions are MCS/Control's; this package
**no longer serves `/path_request`** (Control's `path_generation` is the only
server; its `from_yaml` branch reads MCS deployed trajectories directly).
"Package source root" = `BlueBoat-SSS-Sim/blueboat_sss_sim/`, one level
below the git root.

Simulated results follow `../SIM_VALIDITY_CONDITIONS.md` (superproject root).

*On 2026-11-18 the old sections moved verbatim, numbers kept: §1, §2, §7, §8 → `.claude/development.md`;
§3 (3.1–3.5) → `.claude/interface.md`; §4 (NC #1–#17) → `.claude/constraints.md`; §5, §6 → `.claude/data-config.md`.
A citation such as "simCLAUDE.md §3.1" or "NC #15" finds its full text there.*

**Detail loads by itself:** each `.claude/rules/*.md` is a path-scoped rule that imports one `.claude/` topic
file when you read or edit a matching file; the tables below name the same files for reading by hand.

## 0. Frozen (2026-11-18) — read first

- **The code is frozen** at tag `sim-freeze-v1.0-2026-11-18` (the owner creates it), code digest
  **`cc29e08780aac4cd`**, engine digest `ffacf236faa57010`. The digest covers every `*.py` under
  `blueboat_sss_sim/blueboat_sss_sim/` and `blueboat_sss_sim/config/*.yaml` (`core/provenance.py`
  `tree_files()`). **Change none of them**; a change is a new simulator version, by owner decision only.
  `setup.py` also installs `blueboat_sss_sim/docs/*.md` and `docs/engineering/*.md`, so adding or removing a
  file there changes the install (needs a rebuild); module notes go to the repository-root `docs/` instead.
- **Simulator v1.1 (sessions 14 and 16, owner orders; pending the owner's signature and tag
  `sim-v1.1-worldgen-2026-11-21`): world generation only.** Two changes:
  - World Builder Screen 5 has a *Manual | Mission-like* switch (`builder/`). A world planned by the Mission-like
    mode needs v1.1 only to be *edited* (its build uses the frozen generator).
  - The seabed raster of one large world. The generator coarsens the raster of a world over 150 m of side (400 m
    gives 0.30 m), and every validation of the seabed picture was made at 0.10 m. A state may ask for a finer raster:
    `BuilderState.raster_resolution_m` (`builder/state.py`, written only when set), read and checked by
    `worldgen/world_folder.py:world_block` (never finer than 0.10 m, never coarser than the automatic value, at most
    8 million cells: what the ray-cast sensor loads on the 8 GB GPU). Absent, the behaviour is the one of v1.0, bit
    for bit. The wizard has no control for the field; an Edit re-save keeps it; **an install older than v1.1 ignores
    it and re-saves such a world at the coarse raster**.

  Code digest **`ed13b523d1be4f07`**, **engine digest `40764206ab14c647`** (the engine digest covers `worldgen/`: of
  its 86 files only `world_folder.py` differs from v1.0's `ffacf236faa57010`; `sonar/`, `core/`, calibration,
  profiles and sea presets are untouched), generator digest `6a6e3de4e4dfa21c`. Every recorded world rebuilds with
  its content digest. Runs may use v1.0 or v1.1. Evidence and decision records: `~/ros2_ws/sim_runs/worldgen_v11/`
  (`HANDOFF_14.md`), `~/ros2_ws/sim_runs/worldgen_v12/`,
  `~/ros2_ws/sim_runs/sim_acceptance/sim_v1.1_worldgen_decision.md`,
  `~/ros2_ws/sim_runs/sim_acceptance/sim_v1.2_worldgen_decision.md`.
- **How worlds are generated now:** by hand (Screen 5 Manual); by the population model (Screen 5 Mission-like, or
  headless `python3 -m blueboat_harbour_population build --spec <campaign spec with placement:> --seeds ... --out
  ...`, also on a real depth file with an `environment:` block: walls, land, structures, use features and a raster
  resolution since package version 1.1.0, `docs/population/POPULATION_MODEL.md` §12); by session 02's model (`worldgen_campaign`, a spec
  without `placement:`, or `placement.mode: legacy`). The model is the second package of this repository,
  `../blueboat_harbour_population/` (outside the code digest; its own version, digests, evidence file and tests).
  **Read `docs/population/POPULATION_MODEL.md` (the model) and `EVIDENCE.md` (its sources) before touching it**, and
  `builder/CLAUDE.md` before touching Screen 5. Splits: `blueboat_harbour_population/data/splits.yaml` (dataset
  100000–199999, tuning 200000–299999, evaluation 300000–399999, development below; harbours by split).
- **Before producing results:** `python3 -m blueboat_sss_sim.core.provenance --check` (exit 1 = stale
  install) must give code digest `cc29e08780aac4cd` (v1.0) or `ed13b523d1be4f07` (v1.1).
  Rebuild a stale install with
  `cd ~/ros2_ws && rm -rf build/blueboat_sss_sim install/blueboat_sss_sim && colcon build --packages-select blueboat_sss_sim`.
- **Record per run** what the conditions file lists: world content digest (`metadata.yaml`
  `content_digest`), profile SHA-256, sea, controller, bracket ends (R1, R2, R5, R6), fouling, seeds. The
  node publishes most of it on `/side_scan_sonar/ground_truth/run_info`.
- **Envelope (conditions file §2):** production profile `raycast_harbour_fast_sonar.yaml`, 20 m range,
  1200 bins, 44.3 µs, both sides; altitude **4.5–7.0 m**; ground range 5–20 m with 8–20 m reported apart;
  campaign-generator worlds with sediment patches; seas `calm`, `harbour_chop`, `field_matched`
  (`field_matched_x2` is not a rougher sea: a yaw arm for PID and LoS only, with `thrust_gain:=[0.3805,0.6055]`
  for the FIT seas); MPC field gains for the headline, LoS as the disturbance arm, always
  `sim_config:=field_equivalent`; dataset at the R6 low end; ≥ 3 seeds, disjoint dataset and evaluation
  seeds. Never 600 bins or long pulses.
- **Labels come from the rendered picture and the world metadata, never from a contact**:
  `slant_range_m` is the across-track range (up to 6 bins off), `shadow_bins` is geometric (5 to 35 bins too
  long). An offline GCS rebuild's waterfall can run 5.11 s behind ping time: align on the seabed images' `t_s`
  or the ground truth's `t_sim`.
- Known defects and their sizes: `~/ros2_ws/sim_runs/sim_acceptance/10_evidence/SIM_FREEZE_CARD.md` §3. The
  backlog in `.claude/TODO.md` / `TODO2.md` is for a future version only; cross-repository open items:
  superproject `docs/OPEN_ITEMS.md`. Dated history: `BlueBoat-SSS-Sim/docs/HISTORY.md`.

## 1. Architecture in brief (full: `.claude/development.md` §2, `blueboat_sss_sim/docs/architecture.md`)

- **The sonar is not a Gazebo plugin.** Gazebo owns hull dynamics and pose; a ROS 2 Python node renders
  against the same generated scene. One `SceneModel` emits both `world.sdf` and `scene.npz`.
- **Renderers:** the GPU ray-cast (`renderer: raycast`, run in a slowed simulation with Gazebo paused while a
  ping renders; `warp-lang==1.19.0` in the node's `/usr/bin/python3`) with the harbour profiles
  (`raycast_harbour_fast` = production, `raycast_harbour_live` = slow reference, `…_flash_narrow/_wide` and
  `…_corner_low/_high` = bracket ends); the geometric renderer is **legacy**.
- **Worlds:** `worldgen_campaign --spec <yaml> --seeds 1,2,3 --out <root>` builds harbour worlds headless
  (object set version 2: 11 classes; a world naming one of the 22 deleted classes does not load). World
  folders are immutable (NC #10); `metadata.yaml` is the cross-module contract.
- **Sea state:** `sea/` drives `ros/sea_state_node.py` (current, waves, wind into Gazebo).
- **Layering:** `core/`, `worldgen/`, `sonar/`, `analysis/` import no ROS and no Qt; `ros/` is a thin shell;
  `builder/` is the only Qt code.

## 2. Interface (full: `.claude/interface.md`)

Node `side_scan_sonar`: `/side_scan_sonar/{port,starboard}/profile` (`blueboat_interfaces/OmniscanProfile`)
and `…/raw` (`std_msgs/UInt8MultiArray`, byte-valid Ping Protocol), BEST_EFFORT / KEEP_LAST / 10; additive
simulation-only JSON on `/side_scan_sonar/ground_truth/{contacts,sensor,run_info}`; `…/ping/enable`
(`std_msgs/Bool`, pinging is **off at start-up**). Pose from `/sim/sea_state/boat_pose`, fallback
`/blueboat/odom_gz` then `/blueboat/odom`. Parameter names and defaults equal the real `sss_node.py`
(`range_length_mm` 20000, `num_results` 1200, `msec_per_ping` 50, `gain_index` −1, `pulse_len_percent`
0.00125). `mavros_shim_node` publishes the MAVROS topics from odometry. Launch files: `full_mission_launch.py`,
`sss_sim_launch.py`, `sim_world_launch.py`, `sea_state_launch.py` (`blueboat_sss_sim/launch/CLAUDE.md`).

## 3. Non-negotiable constraints (full text, reasons and constants: `.claude/constraints.md`)

1. Interface fidelity is absolute; never define a new message type (additive JSON topics only).
2. `raw` frames are byte-valid Ping Protocol, checksum included (`parse_frame()` is the oracle).
3. Both sides are published from the same timer tick.
4. The first bottom return survives every acoustic change (six coupled pieces).
5. Speckle statistics are load-bearing (diffuse `Exp(1)`, specular `Gamma(L, 1/L)`, correlated).
6. Power calibration is anchored to the field corpus and moves once, on both anchors.
7. Never raise launch's global log level to suppress noise.
8. Never modify the control stack or other packages.
9. The renderer's ground-sample step stays coupled to the slant-bin size.
10. World folders are immutable snapshots.
11. The headline detector is trained on simulated datasets labelled by Perception's dataset mode (CM-11).
12. Limits never reflect. 13. No return from land, no objects on it. 14. Nothing ticked means nothing placed.
15. A shadow is a measurement, not a hole. 16. Dry land is authored, not carved.
17. Objects are rendered from material physics, never tuned per object.

## 4. Data and configuration (full: `.claude/data-config.md`)

`scene.npz` + `scene_manifest.yaml` are the ground truth; `metadata.yaml` (`blueboat_world_meta/1`, schema
version 2) is what downstream reads, never `builder_state.yaml`; `ground_truth_rasters.npz` holds the
seabed classes and domain. Unknown keys raise by name in the sonar sections. Schemas:
`blueboat_sss_sim/docs/configuration_guide.md`.

## 5. Commands

Population model (from `../blueboat_harbour_population/`): `python3 -m blueboat_harbour_population check | populate |
build | batch | card | stats`; its tests `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest` (61; the
adapter tests need the simulator's source root on `PYTHONPATH`, which `test/conftest.py` adds). Screen 5's mode:
`QT_QPA_PLATFORM=offscreen python3 -m test.placement_test` (53 checks, ~3 min; not in the Stop hook).

From the package source root `BlueBoat-SSS-Sim/blueboat_sss_sim/`:

```bash
python3 -m blueboat_sss_sim.core.provenance --check     # identity of the installed simulator
python3 -m test.smoke_test                              # offline gate, ~540 checks, ~260 s, system python
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
```

Running a script by path imports the **installed** package, not the source tree; prefix
`PYTHONPATH=<package source root>` for source measurements. Gazebo gate: `.claude/gazebo-gate.md`. The Stop
hook (`.claude/settings.json`, gitignored) runs the smoke test after an edit of a package `.py` or
`config/*.yaml` file. Running a simulated survey from the command line: superproject skill `sim-run-recording`.

## 6. Read before touching

| Before … | Read |
|---|---|
| producing or interpreting any simulated result | `../SIM_VALIDITY_CONDITIONS.md`, the freeze card |
| selecting the ray-cast, `sonar_renderer:=`, `real_time_factor:=`, or touching `sonar/raycast` | `.claude/raycast-live.md` |
| the pose path, `sensor_depth_m`, `mount_x_m`, comparing wave or Gazebo recordings across dates | `.claude/sensor-pose.md` |
| `worldgen/campaign.py`, `population.py`, `domain.py`, `groundtruth.py`, `compounds.py` v2 | `.claude/worldgen-campaign.md` |
| a launch module | `blueboat_sss_sim/launch/CLAUDE.md` |
| the World Builder | `blueboat_sss_sim/blueboat_sss_sim/builder/CLAUDE.md` |
| calibration or a level constant | `blueboat_sss_sim/blueboat_sss_sim/calibration/CLAUDE.md` |
| the sea state | `blueboat_sss_sim/blueboat_sss_sim/sea/CLAUDE.md`, `docs/sea_state_model.md` |
| any `.svlog` tool | superproject skill `svlog` |
| the physics of the model | `blueboat_sss_sim/docs/sonar_model.md`, `seabed_model.md` |
| development rules (frames, sides, randomness, adding a class) | `.claude/development.md` §8 |

Superproject rules hold here: **never `git commit`, `git push` or `git tag`**; `BlueBoat-Control` and
`BlueBoat-MCS` are changed by additions only, and only on an explicit user order.
