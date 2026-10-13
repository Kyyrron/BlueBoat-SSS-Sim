# ROS 2 interface

The simulated node reproduces the real `sss_node.py` interface exactly.
Namespace: node name `side_scan_sonar` → private topics resolve under
`/side_scan_sonar/...`.

## 1. Topics

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/side_scan_sonar/port/profile` | `blueboat_interfaces/OmniscanProfile` | pub | decoded ping |
| `/side_scan_sonar/starboard/profile` | 〃 | pub | 〃 |
| `/side_scan_sonar/port/raw` | `std_msgs/UInt8MultiArray` | pub | verbatim Ping-Protocol frame |
| `/side_scan_sonar/starboard/raw` | 〃 | pub | 〃 (rebuildable into `.svlog`) |
| `/side_scan_sonar/ping/enable` | `std_msgs/Bool` | sub | `true` start / `false` stop; params re-read on every enable |
| `/side_scan_sonar/ground_truth/contacts` | `std_msgs/String` (JSON) | pub | **simulation-only extra**, additive |
| `/side_scan_sonar/ground_truth/sensor` | `std_msgs/String` (JSON) | pub | **simulation-only extra**, additive (2026-10-08): every ping cycle, `{t_sim, stamp, pose_age_ms, predicted, pose_source: pose_100hz|odom, boat[x,y,z,roll,pitch,yaw], sensors{port|starboard: {pose[6], altitude_m, ping_number|null, dropped}}}` — the vehicle pose at the ping time and the exact transducer pose each side was rendered from (world ENU, m / rad). Read by `analysis/sea_validation.py` |
| `/sim/sea_state/boat_pose` | `geometry_msgs/TransformStamped` | sub (depth 50) | primary pose source since 2026-10-09: Gazebo's per-step model pose (parameter `pose_topic`; `""` = odometry only) |
| `/blueboat/odom` | `nav_msgs/Odometry` | sub | fallback pose source (parameter `odom_topic`) |

`sea_state_node` (simulation only — started by `full_mission_launch.py`
since 2026-09-03 and by `sea_state_launch.py` standalone, e.g. beside
BlueBoat-Control's `Sim_launch.py`; not by `sss_sim_launch.py` —
`docs/sea_state_model.md`):

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/current` | `geometry_msgs/Vector3` | pub | world-frame water velocity **at the boat** (mean current + its smooth variation in time and space + wave orbital velocity), bridged to gz `/ocean_current` for the boat's `Hydrodynamics` plugin; exact zeros for the null preset |
| `/world/<w>/wrench` | `ros_gz_interfaces/EntityWrench` | pub | one-shot hull wrench per physics step on the boat model, bridged to gz `ApplyLinkWrench`: since 2026-10-09 the catamaran hydrostatics in **every** run (calm water too: one message per physics step, ~0 at rest at the trim) plus the wave excitation when there are waves. In a world without the system (BlueBoat-Control's stock `ocean`) the node injects `ApplyLinkWrench` at run time, detected by the system's own `/world/<w>/wrench/persistent` (never by `/wrench`, which this bridge advertises). `hull.hydrostatics: gazebo_box_when_calm` restores the old "nothing while calm" |
| `/world/<w>/wrench/clear` | `ros_gz_interfaces/Entity` | pub | hygiene at start/stop |
| `/sim/sea_state/boat_pose` | `geometry_msgs/TransformStamped` | sub (bridged by `sea_state_launch.py` from gz `/model/blueboat/pose`, blueboat_description's own `PosePublisher`) | **simulation-only, additive** (2026-10-09): the boat's model pose every physics step (100 Hz), Gazebo's sim-time stamp, child frame `blueboat` — the pose the hull wrench and the sonar use (the 20 Hz odometry is the fallback) |
| `/sim/sea_state` | `std_msgs/String` (JSON) | pub | 2 Hz, **TRANSIENT_LOCAL depth 1** (latched; additive since 2026-10-09: `hull{model: catamaran|gazebo_box, active}`): `{t_sim, current{preset,label,mean_speed_mps,from_deg,from,toward,instant{speed_mps,from_deg,vx,vy}}, waves{preset,label,douglas,hs_m,tp_s,from_deg,from,toward,eta_m,status: active|calm|idle|unavailable,reason,events_per_hour,event: null|{hs_m,tp_s,from_deg,t_remaining_s,envelope}}, water_velocity[3], wrench{fz,mx,my,fx,fy,mz}, blend, schedule{name,keyframes,interpolation,t_s,next_t_sim,next}, summary, wrench_messages, pose_age_ms}` — what the MCS "SEA STATE" box shows (`fx,fy,mz` and `pose_age_ms` are additive since 2026-09-05). Additive since 2026-10-08: `current.{variation, sigma_speed_mps, sigma_dir_deg, spatial_rms_mps, spatial_mps}` (the variation around the mean; `instant` is the current at the boat), `waves.{crest, wakes, spread_s}`, and `boat{samples, window_s, heave_std_m, roll_std_deg, pitch_std_deg, roll_max_deg, pitch_max_deg}` (what the sea does to the hull over the last 30 s of odometry) |
| `/sim/sea_state/command` | `std_msgs/String` (JSON) | sub | `{"current": {"preset"|"speed_mps",…,"from_deg"}, "waves": {…}, "ramp_s": 20}` or `{"schedule": <blueboat_sea_schedule/1 doc>, "t0": "now"}`; the node ramps, never steps. Plain-language keys (2026-10-08, additive): current `variation: steady|natural|strong`; waves `tp_s: auto`, `crest: short|long`, `wakes: none|occasional|frequent`; schedule keyframes `ramp_s` (the transition into that keyframe under `hold`) |
| `/sim/sea_state/markers` | `visualization_msgs/MarkerArray` | pub | RViz arrows + text (the Gazebo GUI gets the same through `gz service /marker_array`) |

`mavros_shim_node` additionally republishes odom-derived navigation under
MAVROS names (`/mavros/global_position/global` — NavSatFix about the
world's GPS anchor, ~5 Hz, BEST_EFFORT — plus `compass_hdg`, `imu/data`,
`local_position/pose` and `local_position/velocity_local` (`TwistStamped`,
world frame; `sss_processor_node` fills the `.svlog`'s vx/vy from it)). This package serves **no** `/path_request`
service: `blueboat_control`'s `path_generation` is the only server
(missions belong to MCS/Control).

QoS on the sonar `profile` and `raw` pubs: BEST_EFFORT, KEEP_LAST, depth
10 — identical to the real node (subscribers must match reliability). The
two `ground_truth/*` topics use the default (RELIABLE) QoS, depth 10, and
exist only with `publish_ground_truth: true` (the default).

Pinging is **off at startup**, as on hardware:

```bash
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
```

## 2. Parameters

Run-dependent (names, semantics and — since 2026-09-28 — defaults identical
to the real node; re-read at every enable):

| Parameter | Default (sim = real `sss_node.py`) | Meaning |
|---|---|---|
| `range_start_mm` | 0 | first bin slant range |
| `range_length_mm` | 20000 | swath slant extent (bin = length/n) |
| `msec_per_ping` | 0 | 0 = free run: max(50 ms, 2R/c + 3 ms) |
| `gain_index` | -1 (auto; the simulator resolves it to 4) | gain ladder index |
| `num_results` | 1200 (the device maximum) | bins per ping |
| `pulse_len_percent` | 0.00125 | pulse = max(44.3 µs, this × 2R/c) |

These are the device's best range resolution: at 1200 bins and 0.00125 the
pulse sits on its 44.3 µs floor up to ~27 m (33 mm true range resolution).
`project_synthesis.md` §8.5's 30 m coverage and 15 m revisit passes are
passed per run. The acquisition is also always **explicit**: `sss_sim_launch.py`
passes all six from the resolved sonar profile (explicit `sonar_config:=` >
a legacy bundle's frozen `sonar.yaml` > `config/default_sonar.yaml`), each
overridable per run (`gain_index:=-1`, `range_length_mm:=30000`, …), and
the node warns at every enable when what is in force differs from what the
named profile records.

`gain_index: -1` carries the real device's meaning — auto-gain. It is a
**command-only** sentinel: the profile's `gain_index` is `uint16`, so the
device resolves auto-gain internally and reports a concrete index. The
simulator has no AGC and resolves `-1` to the calibrated index 4, reporting 4.
The modelled ladder is 0–7 (`ANALOG_GAIN_TABLE`); any other value is rejected
when the parameters are read, rather than silently falling back to a gain the
profile would then misreport. Indices **4–7 are measured** from the field
corpus (`analog_gain` 74.55 / 142.8 / 242.025 / 464.625) — the device's own
auto-gain uses all four inside every recording, so a shorter ladder could not
express most real acquisitions; 0–3 are unmeasured estimates.

Simulation-only: `scene_dir` (required), `sonar_config`, `odom_topic`,
`publish_ground_truth` (also gates `~/ground_truth/sensor`), `seed`,
`predict_pose` (default true: each ping rendered at the boat's pose
propagated to the ping time with its own rates; false = the newest sample
as-is), `pose_topic` (default `/sim/sea_state/boat_pose`, the 100 Hz
stamped pose; the 20 Hz odometry is the fallback).

`sea_state_node` parameters: `current`, `current_from_deg`, `waves`,
`waves_from_deg` (preset names from `config/sea_states.yaml` + compass
FROM bearings), `schedule_file`, `seed`, `rate_hz` (100 = the physics
rate), `max_step_size` (read from the world's `world.sdf` when `world_dir`
is given), `world_name` ("" = discover), `world_dir` (mean depth from
`depth.npz`, physics step), `depth_m`, `target_entity` (`blueboat`),
`target_entity_type`, `presets_file`, `enable_waves`, `enable_gz_markers`,
`marker_arrow_axis`, `ramp_s`, `odom_topic`, `pose_topic` (default
`/sim/sea_state/boat_pose`; "" = odometry only). The readback adds
`pose_source` (`pose_100hz` | `odom`) and `hull{model, active}`. The launch files set them
from `sea_*` arguments (`docs/sea_state_model.md` §9).

## 3. OmniscanProfile fields

See `msg_reference/OmniscanProfile.msg`. Notable conventions:
`timestamp_ms` is the device-uptime clock (starts at first enable);
`vehicle_heading_deg` is compass (0 = North, CW); `transducer_heading_deg`
is the transducer's **mounting angle relative to the bow**, a constant
−90 on every port packet and +90 on every starboard packet — exactly what
the real Omniscan 450 reports on 100 % of the field corpus (changed
2026-09-15; it used to carry the absolute bearing `vehicle ∓ 90 mod 360`,
which SonarView's sign-of-heading side fallback mirrored for headings
90–270°); `sos_dmps` = 15000 (dm/s); `ping_hz` = 451127 (acoustic
frequency, not rate).

**Fault injection** (2026-09-18, all off by default). A run under
`config/fault_injection_sonar.yaml` puts on the wire what the corpus shows
and a clean simulation never does: a `gain_index` walking 4→7 inside the
run, a `ping_number` that restarts mid-mission, a starboard counter offset
(+60, the largest the corpus measures), `channel_number: 255` — which
sends side identity to its documented fallback, the sign of the constant
mounting angle (CM-5) — and independently dropped pings per side, i.e.
one-sided rows. Nothing about it is more realistic; it exists so the
downstream stack's gain compensation, counter re-base, side fallback and
one-sided assembly are *exercised*, which is what CM-2 promises and what
a clean stream cannot demonstrate.

## 4. Raw frame byte map (Ping Protocol, msg 2198 OS_MONO_PROFILE)

Little-endian throughout. For `num_results = 600`: total 1262 bytes,
`payload_len` = 1252.

| Offset | Size | Type | Field |
|---|---|---|---|
| 0 | 2 | `u8×2` | magic `'B' 'R'` |
| 2 | 2 | `u16` | payload_len (52 + 2n) |
| 4 | 2 | `u16` | message_id = 2198 |
| 6 | 1 | `u8` | src_device_id |
| 7 | 1 | `u8` | dst_device_id |
| 8 | 4 | `u32` | ping_number |
| 12 | 4 | `u32` | start_mm |
| 16 | 4 | `u32` | length_mm |
| 20 | 4 | `u32` | timestamp_ms |
| 24 | 4 | `u32` | ping_hz (451127) |
| 28 | 2 | `u16` | gain_index |
| 30 | 2 | `u16` | num_results |
| 32 | 2 | `u16` | sos_dmps (15000) |
| 34 | 1 | `u8` | channel_number (**0 = port, 1 = starboard**) |
| 35 | 1 | `u8` | reserved |
| 36 | 4 | `f32` | pulse_duration_sec (~44.3 µs) |
| 40 | 4 | `f32` | analog_gain (74.55 @ idx 4) |
| 44 | 4 | `f32` | max_pwr_db |
| 48 | 4 | `f32` | min_pwr_db |
| 52 | 4 | `f32` | transducer_heading_deg |
| 56 | 4 | `f32` | vehicle_heading_deg |
| 60 | 2n | `u16[n]` | pwr_results — **not counts**; see below |
| 60+2n | 2 | `u16` | checksum = Σ(all previous bytes) mod 2¹⁶ |

**`pwr_results` is normalised per ping, not absolute.** The device rescales
every ping onto its own dB axis and reports the endpoints in `min_pwr_db` /
`max_pwr_db`; recover the physical values with the Cerulean template
`db = min_pwr_db + (raw / 65535) · (max_pwr_db − min_pwr_db)`, which is what
`sss_processor_node` and the GCS apply. Consequences: every ping's array
spans 0–65535 with exactly one bin at full scale and a minimum of 0 (measured
on 68 948 / 68 948 field pings), the span is clamped at 90 dB, the level lives
entirely in the two endpoints, and a receive-gain change moves them while
leaving the counts untouched. Stacking raw arrays from different pings into a
waterfall is therefore wrong — convert to dB first
(`analysis.calibration.scale_to_db`, below; downstream
`sss_helper.scale_to_db`).

`blueboat_sss_sim.sonar.encoder.parse_frame()` decodes and validates frames
(raises on bad magic/id/checksum) — use it in tests and log tooling;
`blueboat_sss_sim.analysis.calibration.scale_to_db()` inverts the
normalisation.

## 5. Ground-truth contacts (simulation extra)

JSON per ping cycle **that has contacts** (a cycle with none publishes
nothing; `ground_truth/sensor` is the every-cycle topic), keyed for
association with `ping_number`:

```json
{"t_sim": 12.480, "contacts": [
  {"side": "port", "object_id": 17, "type": "tire_car",
   "slant_range_m": 6.412, "extent_bins": 9.3, "shadow_bins": 21.7,
   "visible": true, "ping_number": 566, "ghost": false, "via": ""},
  {"side": "port", "object_id": 17, "type": "tire_car",
   "slant_range_m": 13.077, "extent_bins": 9.3, "shadow_bins": 44.1,
   "visible": true, "ping_number": 566, "ghost": true,
   "via": "wall:quay_north"}]}
```

`ghost` / `via` mark a multipath image: the *same* `object_id`, seen down a
folded path off the named reflector, at the range that path length earns.
`via` is `""` on the direct return. Both keys are additive — a consumer
reading only the older fields is unaffected, and one reading a stream from a
bundle generated before wall multipath existed should default them
(`ghost` false, `via` empty). Aggregate on `(object_id, via)`, never
`object_id` alone, or an object merges with its own ghost.

**A wall's own face echo** is a contact too: `"type": "wall"` and a
**string** `"object_id": "wall:<name>"` (scene objects keep their integer
id). A consumer that parses `object_id` as an integer must skip or
special-case `type == "wall"`, as `analysis/contacts.py` does.

`shadow_bins` and `visible` are read **off the rendered visibility mask**
since 2026-09-18, not from the flat-bottom `h·R/altitude` formula: the
shadow is measured from the object's own far edge to where the ping is lit
again, so an object whose shadow ends inside its own footprint reports
`0`, one standing in another's shadow reports `0` and `visible: false`, and
a shadow lengthened by a down-slope or broken by a lattice is labelled as
the picture shows it. A label that contradicts the image teaches the wrong
cue, which is the one thing a training label must not do. A shadow shorter
than the renderer's own ground step is reported as that step (sub-bin, and
no picture shows it either).

Consumed by `analysis.contacts.from_jsonl` (dump the topic to JSONL for
`mission_metrics --source jsonl`); ignorable by everything else.
