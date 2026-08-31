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

Run-dependent (names and semantics identical to the real node; re-read at
every enable). **Two defaults deliberately differ** — see below:

| Parameter | Default | Real `sss_node.py` | Meaning |
|---|---|---|---|
| `range_start_mm` | 0 | 0 | first bin slant range |
| `range_length_mm` | 15000 | **20000** | swath slant extent (bin = length/n) |
| `msec_per_ping` | 0 | 0 | 0 = free run (two-way + 2 ms) |
| `gain_index` | 4 | **-1** (auto) | gain ladder index (3 dB/step default) |
| `num_results` | 600 | 600 | bins per ping |
| `pulse_len_percent` | 0.002 | 0.002 | pulse duration as fraction of period |

The simulator's defaults are its *calibration point* — the range and fixed
gain `model.calibration_db_offset` and the acoustic constants are anchored to — not
an attempt to mirror the real node's defaults, which match neither survey
setting the project uses (`project_synthesis.md` §8.5 reserves 30 m for
coverage passes, 15 m for revisit passes). Comparability rests on the
acquisition being **explicit**, not on equal defaults: `sss_sim_launch.py`
passes all six from the mission bundle's frozen `sonar.yaml`, each overridable
per run (`gain_index:=-1`, `range_length_mm:=30000`, …), and the node warns at
every enable when what is in force differs from what the bundle records.

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
waterfall is therefore wrong — convert to dB first, as
`dataset/waterfall.py` does.

`blueboat_sss_sim.sonar.encoder.parse_frame()` decodes and validates frames
(raises on bad magic/id/checksum) — use it in tests and log tooling;
`blueboat_sss_sim.analysis.calibration.scale_to_db()` inverts the
normalisation.

## 5. Ground-truth contacts (simulation extra)

JSON per ping-cycle, keyed for association with `ping_number`:

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

Consumed by `dataset_recorder_node` for auto-labeling and by
`analysis.contacts.from_jsonl`; ignorable by everything else.
