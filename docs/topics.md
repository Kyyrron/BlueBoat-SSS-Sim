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
| `/blueboat/odom` | `nav_msgs/Odometry` | sub | pose source (parameter `odom_topic`) |
| `/mission/full_path` | `nav_msgs/Path` | pub (latched) | complete mission for RViz, from `sss_path_generation` (set the display's Durability to Transient Local) |

QoS on all sonar pubs: BEST_EFFORT, KEEP_LAST, depth 10 — identical to the
real node (subscribers must match reliability).

Pinging is **off at startup**, as on hardware:

```bash
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
```

## 2. Parameters

Run-dependent (names, defaults and semantics identical to the real node;
re-read at every enable):

| Parameter | Default | Meaning |
|---|---|---|
| `range_start_mm` | 0 | first bin slant range |
| `range_length_mm` | 15000 | swath slant extent (bin = length/n) |
| `msec_per_ping` | 0 | 0 = free run (two-way + 2 ms) |
| `gain_index` | 4 | gain ladder index (3 dB/step default) |
| `num_results` | 600 | bins per ping |
| `pulse_len_percent` | 0.002 | pulse duration as fraction of period |

Simulation-only: `scene_dir` (required), `sonar_config`, `odom_topic`,
`publish_ground_truth`, `seed`.

## 3. OmniscanProfile fields

See `msg_reference/OmniscanProfile.msg`. Notable conventions:
`timestamp_ms` is the device-uptime clock (starts at first enable);
`vehicle_heading_deg` is compass (0 = North, CW); `transducer_heading_deg`
= vehicle heading ∓ 90° (port −, starboard +, mod 360); `sos_dmps` = 15000
(dm/s); `ping_hz` = 451127 (acoustic frequency, not rate).

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
| 34 | 1 | `u8` | channel_number |
| 35 | 1 | `u8` | reserved |
| 36 | 4 | `f32` | pulse_duration_sec (~44.3 µs) |
| 40 | 4 | `f32` | analog_gain (74.55 @ idx 4) |
| 44 | 4 | `f32` | max_pwr_db |
| 48 | 4 | `f32` | min_pwr_db |
| 52 | 4 | `f32` | transducer_heading_deg |
| 56 | 4 | `f32` | vehicle_heading_deg |
| 60 | 2n | `u16[n]` | pwr_results |
| 60+2n | 2 | `u16` | checksum = Σ(all previous bytes) mod 2¹⁶ |

`blueboat_sss.sonar.encoder.parse_frame()` decodes and validates frames
(raises on bad magic/id/checksum) — use it in tests and log tooling.

## 5. Ground-truth contacts (simulation extra)

JSON per ping-cycle, keyed for association with `ping_number`:

```json
{"t_sim": 12.480, "contacts": [
  {"side": "port", "object_id": 17, "type": "tire_car",
   "slant_range_m": 6.412, "extent_bins": 9.3, "shadow_bins": 21.7,
   "visible": true, "ping_number": 566}]}
```

Consumed by `dataset_recorder_node` for auto-labeling; ignorable by
everything else.
