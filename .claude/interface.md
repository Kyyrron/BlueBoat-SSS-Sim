# BlueBoat-SSS-Sim — interface detail (sonar node, MAVROS shim, World Builder, CLI, generated worlds, launch files)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 3. Interface (the contract — this is where bugs come from)

Node name `side_scan_sonar`, so private topics resolve under
`/side_scan_sonar/`.

### 3.1 `sss_sim_node` — the drop-in sonar

| Topic | Type | Dir | QoS | Other side |
|---|---|---|---|---|
| `/side_scan_sonar/port/profile` | `blueboat_interfaces/OmniscanProfile` | pub | BEST_EFFORT, KEEP_LAST, 10 | `sss_processor_node` |
| `/side_scan_sonar/starboard/profile` | 〃 | pub | 〃 | 〃 |
| `/side_scan_sonar/port/raw` | `std_msgs/UInt8MultiArray` | pub | 〃 | `sss_processor_node` (rebuilds `.svlog`) |
| `/side_scan_sonar/starboard/raw` | 〃 | pub | 〃 | 〃 |
| `/side_scan_sonar/ground_truth/contacts` | `std_msgs/String` (JSON) | pub | default 10 | offline analysis; only on a cycle with contacts; **simulation-only, additive** |
| `/side_scan_sonar/ground_truth/sensor` | `std_msgs/String` (JSON) | pub | default 10 | every ping cycle: the boat pose at the ping time and each transducer's rendered pose; **simulation-only, additive** (2026-10-08), read by `analysis/sea_validation.py`. Since 2026-11-11 also `run_id` and `range_scale` (true slant range → imaged range: 1500 / c under the ray-cast, 1.0 geometric), which Perception's labels apply |
| `/side_scan_sonar/ground_truth/run_info` | `std_msgs/String` (JSON) | pub | RELIABLE, TRANSIENT_LOCAL, 1 | **run provenance** (2026-11-11, `blueboat_sss_sim_run/1`): renderer, profile path + sha256, world digest, seeds, acquisition, build stamp, whether the installed code is the source tree; at start-up, every enable, and stop (counters). The GCS stores it as `metadata.json["simulation"]`; also written to `run_info_dir` |
| `/side_scan_sonar/ping/enable` | `std_msgs/Bool` | sub | default 10 | operator / launch one-shot |
| `/sim/sea_state/boat_pose` | `geometry_msgs/TransformStamped` | sub | default 50 | `sea_state_launch.py` bridge (param `pose_topic`); the primary pose |
| `/blueboat/odom` | `nav_msgs/Odometry` | sub | default 10 | Gazebo bridge (param `odom_topic`); the fallback pose (optional while `boat_pose` flows) |

Pinging is **off at startup**, like hardware; run-dependent parameters are
re-read on every enable.

Parameters — names, semantics **and defaults** = the real `sss_node.py`
since 2026-09-28; `[4b]` asserts each and reads the real driver's source to
assert the cross-module equality, skipping visibly without it:

| Parameter | Default (sim = real) |
|---|---|
| `range_start_mm` | 0 |
| `range_length_mm` | 20000 |
| `msec_per_ping` | **50** (SonarView's command, 2026-10-27; 0 = free run) |
| `gain_index` | -1 (device auto; the simulator resolves it to its calibrated 4) |
| `num_results` | 1200 (the device maximum, 16.7 mm bins at 20 m) |
| `pulse_len_percent` | 0.00125 |

= the device's **best range resolution**: pulse =
`max(44.3 µs, pulse_len_percent × 2R/c)` (**MEASURED**, every corpus
recording, 13 settings, within 1 µs — `AcquisitionParams.pulse_duration_s`),
so 0.00125 stays on the 44.3 µs floor up to ~27 m. The renderer's range
envelope follows the acquisition (`pulse/Ts + filter_duration_percent × N +
1` bins), as does the along-range lag-1 texture correlation (`[2g]`).
Seabed level is bin-count invariant (0.1 dB, 600 vs 1200), so the
calibration carries over; **smoke gates still measure at the calibration
acquisition** (`_cal_config()`: 600 bins, 0.002, gain 4 — the anchors').

**Ping timing = SonarView's (2026-10-27, owner: acquire exactly like
SonarView, identically sim and real).** Period = `max(msec_per_ping, 2R/c +
3 ms, 1/max_ping_rate_hz)`. The corpus law `max(50 ms, 2R/c + 3 ms)` is
SonarView's 50 ms **command** (every corpus file is SonarView's), not a device
cap: the real driver free-running pinged every 29.0 ms (34.5 Hz) at 20 m. So
`msec_per_ping` defaults to 50 in every profile, the node and
`AcquisitionParams`, and `model.max_ping_rate_hz` to **0** (off) — the period is
unchanged at every range, renders and fixtures with it, and `msec_per_ping: 0`
now free-runs as the device does. Smoke `[4b]` asserts the law and the
real-driver equality.

Launches pass all six from the resolved profile (explicit `sonar_config:=` >
a legacy bundle's frozen `sonar.yaml` > `config/default_sonar.yaml`), each
overridable per run, so acquisition always traces to a named file. Profiles
are each the default with **one idea** changed -- except
`field_candidate_v1_sonar.yaml` (2026-10-30), a calibration artifact composed
into a self-contained profile (provisional, never the comparison profile;
regenerate, never hand-edit); `default_sonar.yaml` is the
sim-vs-real comparison profile (`shallow_beach_sonar.yaml` lightens the
field box's shadow past the device's), `ground_anchored_sonar.yaml`
(2026-11-10) anchors the geometric speckle to the ground under yaw/sway (opt-in,
`.claude/sensor-pose.md`), and `fault_injection_sonar.yaml` is a
**test** profile, never calibration or comparison.

`gain_index: -1` = the device's command-only sentinel; the sim resolves it
to the calibrated index 4 and reports 4. Modelled ladder **0–7** (4–7
measured on the corpus); any other index raises in `AcquisitionParams`.

**Stale installs (2026-11-11).** The package installs by COPY: a session can run code older than
the source tree (the 2026-11-11 harbour session did). The node now warns at start-up, naming the
files, when the install differs from the source it was built from, and every run records its
build (`~/ground_truth/run_info`). Before reading a live result:
`python3 -m blueboat_sss_sim.core.provenance --check` (exit 1 = stale); rebuild with
`cd ~/ros2_ws && rm -rf build/blueboat_sss_sim install/blueboat_sss_sim && colcon build
--packages-select blueboat_sss_sim` (a plain rebuild leaves deleted modules installed).

Sim-only params: `true_odom_topic` (2026-11-15, R-7: `/blueboat/odom_gz`, the preferred
fallback pose while fresh; `odom_topic` stays `/blueboat/odom`, the only pose of a bag replay),
`run_context` (the launch's b64 JSON, recorded as `run_info.simulation_context`, C3),
`scene_dir` (**required**; world folder or legacy bundle),
`run_info_dir` (2026-11-11; where the run record is also written, "" = nowhere),
`sonar_config`, `odom_topic`, `publish_ground_truth`, `seed` (0 = a fresh
entropy seed that is logged and published as `noise_seed`; launch arg
`sonar_seed`), `predict_pose` (default true), `pose_topic` (default
`/sim/sea_state/boat_pose`), `base_link_rest_z_m` (2026-10-21, below),
`renderer` / `raycast_backend` (2026-11-07, below), `hold_sim_while_rendering` /
`gz_world_name` / `real_time_factor` (2026-11-09: the ray-cast pauses Gazebo while a ping cycle
overruns, `core/sim_hold.py`; `full_mission_launch.py sonar_hold_sim:=`), `fouling` (2026-11-11:
the ray-cast's fouled fraction of object and wall surfaces for this run, −1 = the profile's;
launch arg `sonar_fouling`).

**Ray-cast renderer (2026-11-07): moved to `.claude/raycast-live.md`.** Read it before
selecting `renderer: raycast`, launching with `sonar_renderer:=` / `real_time_factor:=`, or
touching `sonar/raycast`. In short: the run is slowed as a whole on `/clock`, and since
2026-11-09 Gazebo is also paused while a slow ping renders (a quay costs 1–16 s a ping), so no
ping is lost; MCS lists the generated live variants `config/raycast_live_sonar.yaml` (first-order
multipath, bit-identical) and `raycast_fast_sonar.yaml` (also no diffraction, identical by eye)
next to the full `raycast_default_sonar.yaml` (`sonar/raycast/config.write_variant_profiles`);
`warp-lang==1.19.0` must import in the node's `/usr/bin/python3`; keep range ≤ 30 m on the 8 GB
GPU; ground-truth contacts only with `raycast.labels.enabled` (the harbour profiles, since
2026-11-11: `sonar/raycast/labels.py`, read off the renderer's own mesh and oracle); FBR lock
measured offline on a recorded track with the processor's own tracker
(`analysis/raycast_bottomline.py`), not yet in a live run.
Those three carry the **provisional core** (anchored at bath3 only: +11 dB at the harbour) and
no shadow floor. **Harbour profiles (2026-11-10):** `raycast_harbour_live_sonar.yaml` /
`raycast_harbour_fast_sonar.yaml` add the harbour site calibration (v2 core, its empirical
near-nadir table edge-held to 4.5 m, the NC #15 floor 18 dB under a smoothed per-ping ambient
reference, the near range -- ringing, water column -- measured at 1200 bins / 44.3 µs; since
2026-11-11 the table fades in over 0.8 m after the first echo, `correction.onset_fade_m`)
-- provisional, harbour worlds only, never the comparison profile; details and limits in
`.claude/raycast-live.md`. **Since 2026-11-14 (completion round, session 01):**
`raycast_harbour_fast` is the **production sensor** for harbour worlds (altitude 4.5–11.3 m,
range ≤ 30 m, refused beyond), `raycast_harbour_live` the slow **reference** for convergence, **(2026-11-18) Results are produced only at 20 m range, 1200 bins / 44.3 µs and 4.5–7.0 m altitude (`SIM_VALIDITY_CONDITIONS.md` §2, superproject root); the code's own limits above are wider than the validated envelope.**
the geometric renderer **legacy**; the generic `raycast_default / live / fast` warn when loaded
on a scene with objects. The harbour profiles now carry a 2 cm sea surface, solid (elastic)
shell walls, face waviness 0.5° / 0.10 m with a 1.5° draft, and two phase-convention fixes;
`…_flash_narrow` / `…_flash_wide` / `…_shell_opaque` are the bracket ends for the sensitivity
study and `raycast_harbour_{fast,live}_20261112_sonar.yaml` the frozen copies of the
2026-11-12 state (regression only). Physics and inputs: `docs/sonar_model.md` section 15;
what to know before an edit: the last entry of `.claude/raycast-live.md`. The whole-engine
verification (V1–V9 of the acceptance criteria) runs on the GPU from
`~/ros2_ws/sim_runs/sim_acceptance/01_sonar_physics/` (this machine only), not in the smoke
gate; smoke `[19]` holds the closed forms and conventions it rests on.

**Installation, vertical datum, pose path: moved to `.claude/sensor-pose.md`.** Read it
before touching the pose path (`sss_sim_node` pose handling, `renderer._sensor_pose` /
`_pitch_centre`, `sea.motion`), `sensor_depth_m` (0.22) / `mount_x_m` (−0.345) /
`base_link_rest_z_m`, or comparing wave-run or Gazebo recordings across dates (several
changes broke bit-for-bit comparability).

Ground-truth JSON per ping cycle:
`{"t_sim": float, "contacts": [{"side","object_id","type","slant_range_m","extent_bins","shadow_bins","visible","ping_number","ghost","via"}]}`
— `ghost`/`via` mark a multipath image (`via: "wall:<name>"` / `"surface"`,
`""` direct); consumers aggregate on `(object_id, via)`; both keys default
to direct, so pre-wall streams parse unchanged. A wall's own face echo is a
contact with `type: "wall"` and a **string** `object_id: "wall:<name>"`
(objects keep integer ids); published only on cycles that have contacts.

### 3.2 `mavros_shim_node` — on by default in the launches

Subscribes `/blueboat/odom`; publishes `/mavros/global_position/compass_hdg`
(`Float64`), `/mavros/imu/data` (`Imu`), `/mavros/local_position/pose`
(`PoseStamped`), `/mavros/local_position/velocity_local` (`TwistStamped`,
world frame — the `.svlog` vx/vy), `/mavros/global_position/global` (`NavSatFix`, BEST_EFFORT
sensor-data QoS, ~5 Hz) from the ENU position about `sim_origin_lat` /
`sim_origin_lon` — **filled by `sss_sim_launch.py` from the world's geo
anchor** when empty; the legacy 43.6961 / 7.3080 default applies only to
unanchored worlds, with a warning. The odom subscription is the node's
entire drive train (f489a21 shipped without it: four topics, zero messages).

### 3.3 World Builder — `world_builder` (GUI, PySide6)

A six-screen wizard, in Create or Edit mode. **Its full description is in
`blueboat_sss_sim/blueboat_sss_sim/builder/CLAUDE.md`**, which loads when
working on files there. Two rules hold everywhere: **never rebuild the scene
or open a modal from a slot that a canvas item event emitted** (the
2026-09-16 use-after-free segfault that lost a world), and a saved world is
immutable — every save, Edit re-saves included, takes a new `_i` folder
(NC #10).

### 3.4 CLI entry points

Console scripts are listed in `setup.py`; each takes `--help`. The `.svlog`
tools (`sss_svlog_compare`, `analysis.svlog_digest`, `analysis.object_signature`,
`analysis.wall_contrast`, `sss_calibration_report`) — which answers which
question, and their caveats — are in the superproject skill
`.claude/skills/svlog/SKILL.md`. Run `sss_svlog_compare` on every model
change; run `svlog_digest` first on any `.svlog`.

### 3.4b Generated worlds (2026-11-15, completion round session 02)

`worldgen_campaign --spec <yaml> --seeds 1,2,3 --out <root>` builds harbour worlds headless from a
`blueboat_campaign/1` specification: a procedural basin, a seabed with sediment patches and relief (R2 presets
fitted on the FIT recording), structures, and litter of the **object set version 2** (11 classes) drawn from
the population model, all pinned, with a content digest that repeats. Every new world, hand-made included, gets
its ground truth: `metadata.yaml` schema version 2 (additive, `format` unchanged) and
`ground_truth_rasters.npz`.

**The catalogue is the object set version 2 (since 2026-11-15, Stage 6, owner's order).** Eleven
classes (`objects.V2_CLASSES`); the 22 others were deleted, and a world that names one does not
load: `SceneModel.load`, the ray-cast mesher and the placement raise, naming the class and its
replacement (`objects.REMOVED_CLASSES`). **The earlier construction of the seven classes that
existed before is kept for an instance without `params.spec: 2`** — a build path, not a kept
class: it keeps every world and every result made before bit-identical (the real harbour world,
the beach-box worlds, session 01's evidence). **New worlds always carry `spec: 2`** (the campaign
generator and every rebuild write it). Session 01's scripts that named deleted classes have
patched copies in `~/ros2_ws/sim_runs/sim_acceptance/02_world_builder/e1_patched/`; which command
moved where: `02_world_builder/command_map.md`. Brackets: R5 (`raycast.objects.corner_floor_coherence`, profiles
`…_corner_low` / `…_corner_high`) and R6 (`uncertain_materials`). **Read `.claude/worldgen-campaign.md` before
touching `worldgen/campaign.py`, `population.py`, `domain.py`, `groundtruth.py`, the version 2 builders of
`compounds.py` or `depth/layers/heterogeneity.py`** (opt-in `spec: 2`, pinned entries, random streams, the
digest, `sand` as the only reference sediment).

### 3.5 Launch files

`full_mission_launch.py`, `sss_sim_launch.py`, `sim_world_launch.py`, `sea_state_launch.py`.
Since 2026-11-15 `full_mission_launch.py` forwards Control's `sim_config` (`field_equivalent` |
`legacy`) and its overrides exactly as `Sim_launch.py` (R-6), derives the odometry topics from it
(R-7) and takes `spawn_yaw`, `with_sonar`, `thrust_gain`, `odom_base_delay`, `sea_kappa`,
`sea_presets_file`; sea situations are `sea_schedule:=<share>/config/sea_situations/<name>.yaml`. Since 2026-11-16 a generated world
starts **paused** (`hold_until_sea_armed:=true`) and the sea node unpauses it once the hull is armed: with the
physical CoG Gazebo's box alone capsized the boat in the first seconds. `field_matched` / `field_matched_x2` (the
2026-09-07 FIT sea) need the plant's asymmetry `thrust_gain:=[0.3805,0.6055]` beside them. **(2026-11-18) `field_matched_x2` is not a rougher sea**: same waves and wind, only the
yaw-moment variance doubled; use it as a yaw arm for PID and LoS only (`SIM_VALIDITY_CONDITIONS.md` §2).
**(2026-11-18, owner order, R-3)** `full_mission_launch.py` takes `path_source` (`mission` default |
`replanner`) and passes it to its `master_control` in the launch's own values, as `Sim_launch.py` does
(after a tuning file, before `control_params_file`). `replanner` makes the run replanner-owned: master_control
holds at the spawn pose until the first accepted `/blueboat/update_path`; `trajectory_file` is still required
but not followed. master_control validates the value (FATAL otherwise). Launch files are outside the code
digest; the default is unchanged. MCS passes it from **Extra args** (`path_source:=replanner`).
What each does, the `ApplyLinkWrench` detection rule and the **optional ROS imports the
offline smoke gate depends on**: `blueboat_sss_sim/launch/CLAUDE.md` (loads when working
there; read it before editing any launch module).

---
