# BlueBoat-SSS-Sim — history

Dated decisions and session outcomes, newest last, one short paragraph each, with a pointer to the evidence.
Detail of every model decision stays in `.claude/` (`constraints.md`, `interface.md`, `raycast-live.md`,
`sensor-pose.md`, `worldgen-campaign.md`) and in the package docs (`blueboat_sss_sim/docs/`). Created 2026-11-18
(session 11).

## Before the completion round

- **2026-09-02.** Speckle comes from a coherent ground-scatterer field: correlated along range and track as the
  corpus measures (NC #5).
- **2026-09-05.** An empty class selection places zero objects (it used to scatter 29) (NC #14).
- **2026-09-15.** `transducer_heading_deg` on the wire is the mounting angle (−90 / +90), as the device reports
  it; older simulated recordings carry the absolute bearing.
- **2026-09-16.** A World Builder use-after-free segfault lost a world. Rule since then: never rebuild the scene
  or open a modal from a slot a canvas item event emitted.
- **2026-09-18/19.** The independent simulator review (`sim_review.md`, superproject) was implemented. It brought
  the specular range law, the ambient-referenced shadow floor (NC #15), the calibration constants of NC #6, and
  the fault-injection profile that tests CM-2.
- **2026-09-28.** Acquisition defaults equal the real `sss_node.py`. Owner's rule: objects are rendered from
  material physics, never tuned per object (NC #17).
- **2026-10-08.** Boat motion is never simulated offline; `simulate_3dof` stays the only offline loop.
- **2026-10-20.** Owner's decision (CM-11): the headline detector is trained on simulated datasets.
- **2026-10-27.** Ping timing follows SonarView's 50 ms command, the same in simulation and in real water.
- **2026-10-30.** Calibration pass: `specular_strength` set to 1.75; bundle A failed
  (`field_candidate_v1_sonar.yaml`, provisional).
- **2026-11-02.** Gazebo visuals stand on the terrain-only raster; worlds built earlier keep the old picture
  until re-saved.
- **2026-11-07.** The ray-cast sensor (`sonar/raycast/`, GPU, NVIDIA Warp) runs in a simulation slowed as a whole.
  It was verified end to end at the Shiraishijima harbour, 20 m / 1200 bins, x0.03, at 20.3 Hz in simulation
  time. GPU memory: 2.3 GB at 20 m, 3.6 GB at 30 m, out of memory at 50 m on the 8 GB GPU.
- **2026-11-09.** Gazebo is paused while a slow ping renders, so no ping is lost (`core/sim_hold.py`).
- **2026-11-10.** Harbour profiles `raycast_harbour_live` / `raycast_harbour_fast` added (site calibration, v2
  core, near-nadir table).
- **2026-11-11.** Independent review: run provenance (`core/provenance.py`, `~/ground_truth/run_info`), ray-cast
  ground-truth contacts (`raycast.labels`), objects meshed from their parts with fouling
  (`docs/engineering/independent_review_2026-11-11.md`).
- **2026-11-12.** Harbour validation of the two profiles on the real `toOptimize-harbourScan` path
  (`~/ros2_ws/sim_runs/harbour_validation/`). This is where the README's old statement "the ray-cast publishes no
  ground-truth contacts yet; its absolute levels are not validated" dates from.

## Completion round (sessions 01–10, 2026-11-13 to 2026-11-18)

Evidence root: `~/ros2_ws/sim_runs/sim_acceptance/` (this machine only). The criteria were pre-registered in
`00_README_order_rules_acceptance_criteria_FROZEN.md`, with amendments in `amendments.md` and data roles in
`split_register.yaml`.

- **01 sonar physics (2026-11-13/14).** The whole-engine verification V1–V8 passed against independent
  references. `raycast_harbour_fast` became the production sensor, `raycast_harbour_live` the slow reference,
  and the geometric renderer legacy. The bracket profiles `…_flash_narrow/_wide` and `…_shell_opaque` were
  added. `01_sonar_physics/handoff_01.md`.
- **02 World Builder (2026-11-15).** The campaign generator `worldgen_campaign`, sediment patches and R2
  presets, and ground truth (`metadata.yaml` schema 2, `ground_truth_rasters.npz`). Object set version 2 has 11
  classes; the other 22 were deleted (owner's order). The R5/R6 brackets were added.
  `02_world_builder/handoff_02.md`, `command_map.md`.
- **02b ray-cast fix (2026-11-16).** Stale sonar poses under the render hold were fixed (CR-03-5), and the C1
  reproducibility check run. The engine digest is `ffacf236faa57010`. `02b_raycast_fix/handoff_02b.md`.
- **03 sea state (2026-11-15/16).** The sea on the identified plant: wind, the yaw-moment stand-in, a kappa
  bracket, the CAD hull (R-8), and the `harbour_chop` / `field_matched` seas. `03_sea_state/handoff_03.md`.
- **04 / 04b / 04c control (2026-11-13/16).** The identified plant (`sim_config:=field_equivalent`) and R-8 were
  applied in BlueBoat-Control, and the closed-loop spread between controllers was diagnosed as a stated
  limitation. `04_control/`, `04b_control_r8/`, `04c_closed_loop/`.
- **06 lever arm (2026-11-15).** One set of transducer offsets across the simulator, processor, GCS, Belief and
  Perception. `06_sss/handoff_06.md` and the follow-ups.
- **09a / 09b (2026-11-16).** The simulator configuration is recorded in every run record, and MCS can pass a sea
  situation file and `thrust_gain`.
- **05 acceptance report (2026-11-17/18).** The literal verdict is **NOT READY**: V9 fails on the wire pot
  (1.06 dB highlight, 1.61 dB shadow), C5 fails (contact field meanings), C3 is not met on two formal points, and
  field validation fails B1–B6. `05_report/SIM_acceptance_report_2026-11-18.md`.
- **10 evidence (2026-11-18).** The pictures, the beach box, the rough sea, the campaign look, a blind check (73 %
  in the fixed mapping, p = 0.008), 15 unregistered recordings, and the level against altitude (undecided, so
  ≤ 7 m). `10_evidence/EVIDENCE_REPORT.md`, `SIM_FREEZE_CARD.md`.

## Freeze

- **2026-11-18. The owner froze the simulator as a model-conditional tool**, tag `sim-freeze-v1.0-2026-11-18`,
  code digest `cc29e08780aac4cd`, production profile `f74ce9206906953a`. The decision is
  `~/ros2_ws/sim_runs/sim_acceptance/freeze_decision.md`. Its conditions are in `../SIM_VALIDITY_CONDITIONS.md`
  (superproject root). A code change is a new simulator version, by owner decision only.
- **2026-11-18, session 11.** Post-freeze clean-up: the acceptance data was thinned, the instruction files were
  split (`.claude/CLAUDE.md` and its topic files), and this file, `docs/TUTORIAL.md` and the shorter README were
  added. No package file changed. `sim_acceptance/11_cleanup/REPORT.md`.
- **2026-11-18, session 13 (owner order, R-3).** `full_mission_launch.py` gained the launch argument
  `path_source` (`mission`, default, or `replanner`), passed to its `master_control` in the launch's own values,
  as `Sim_launch.py` does, so a built world can host a replanner-owned run. Launch files are outside the code
  digest (`cc29e08780aac4cd` unchanged) and the default is unchanged; the change is included in the freeze tag.
  No package file changed. `~/ros2_ws/sim_runs/replanner/handoff_13_R-3.md`.

## Simulator v1.1: world generation (2026-11-21, session 14, owner order)

- **Mission-like placement.** World Builder Screen 5 gained a mode switch, *Manual* (as before) or *Mission-like*:
  a harbour type, the target classes, a density level 1–10 and a seed fill the world with targets, debris, small
  litter and structures as documented harbours are filled. The model is a second package of this repository,
  `blueboat_harbour_population` (outside the code digest, its own version 1.0.0 and digests written into every
  world's builder state; its parameters can be recalibrated without a new simulator version). It also builds worlds
  headless from a campaign specification with a `placement:` block (and an `environment:` block for a real depth
  file), through the frozen generator used as a library.
- **What changed in the package:** `builder/` only (`state.py`: a `placement` record written only when set;
  `placement.py`, `pages/mission_panel.py` new; `pages/place_page.py`: the switch, Manual paths verbatim;
  `worker.py`: the build passes the record to `metadata.yaml`, and two workers; `map/canvas.py`: an optional object
  colour). Code digest v1.1 `bbd4112928645e13`; engine digest `ffacf236faa57010`, generator digest `4de0d3a4…`,
  profiles and sea presets unchanged.
- **Regression:** the 32 recorded worlds with a builder state rebuild identically from v1.0 (`1932b9a`) and v1.1
  (content digest, `builder_state.yaml`, `world.sdf`, `seabed.stl`, `depth_baked.tif`); a fixed scene renders
  bit-identical pings (geometric) and within the GPU's own run-to-run floor (ray-cast, 7.5e-15 against 7.7e-15).
  Evidence: `~/ros2_ws/sim_runs/worldgen_v11/` (`HANDOFF_14.md`). Decision record for the owner:
  `~/ros2_ws/sim_runs/sim_acceptance/sim_v1.1_worldgen_decision.md`; proposed tag `sim-v1.1-worldgen-2026-11-21`.
