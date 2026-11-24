# Dataset and calibration contracts

The formats a field calibration produces and consumes, and the conventions
that make their numbers comparable. Existing cross-module file contracts
(`.svlog`, GCS sessions, world folders, trajectories, poslogs) stay as the
superproject's `file-interfaces` skill describes them; this page adds only
the calibration layer on top.

## 1. Raw vs derived data

- **Raw is write-once (CM-7).** `.svlog` files, run bags, position CSVs
  and controller `.npy` logs are never edited, renamed or "cleaned" in
  place. A correction is a derived file that names its source and hash.
- **Tools refuse to write into a recording's tree.**
  `sss_calibration quality --out` exits if the output directory lies
  inside one. Keep every `--out` outside `~/ros2_ws/data/SSS_data`.
  `svlog_compare` and `calibration_cli` do not check this yet.
- **Derived products** (quality reports, reductions, fits, artifacts,
  effective profiles) carry:
  - the `format` they follow;
  - their inputs by name and SHA-256, never by absolute path;
  - no position fixes. The quality report drops the digest's `first_fix`.
- **Nothing private enters the repository.** No recording, track or GPS
  fix belongs in source, docs or tests. Tests synthesise their
  recordings at run time.

## 2. Session manifest (`blueboat_session_manifest/1`)

YAML, validated by `calibration/manifest.py`
(`sss_calibration validate-manifest M.yaml [--root DIR] [--check-files]
[--verify-hashes]`). Print an example with `sss_calibration example
manifest`.

```yaml
format: blueboat_session_manifest/1
dataset: {name, owner, description}
sessions:
  - id: S1                         # unique
    role: calibration              # calibration | validation | test | exploratory
    site: beach_A
    date: 2026-11-02               # YYYY-MM-DD
    group: beach_A/2026-11-02      # sessions of one group must share a role
    regime: {altitude_band_m: [1.0, 2.0], bottom: sand, bottom_label_source: grab sample,
             gain_mode: fixed_4, sea_state: calm, range_m: 20}
    platform: {vehicle: blueboat, mount_measurement: <ref>}
    clock: {sonar: device uptime, nav: autopilot boot, frozen_time_boot: false}
    quality_flags: []
    recordings:
      - {path: S1/pass01.svlog, kind: svlog, sha256: <hex>, purpose: seabed_level,
         quality_flags: [], exclude: false}
```

**The rules:**
- **Split by whole session, never by ping.**
- A recording path or a SHA-256 under two roles is leakage, and so is one
  `group` (site-day) under two roles.
- Paths are relative to the manifest root: no absolute paths, no `..`.
- A missing file is reported as `unavailable`. It is never invented, and
  it is an error only when `--verify-hashes` needs it.
- `regime.forward_model` says how a session is rendered for scoring:
  `flat_sand` (the default when absent: the seabed engine's flat sand at the
  stratum altitude) or `scene` (scored against the session's own world, as
  the pool of pass 3 is). Any other value is refused. The flat-sand engine
  (`calibration.seabed.manifest_recordings`) never selects a `scene`
  session, so adding one leaves every flat-sand score unchanged (smoke `[16]`).

## 3. Clock domains

| Clock | Source | Notes |
|---|---|---|
| Sonar `timestamp_ms` | each Omniscan's own uptime (real); `round(t_sim·1000)` from the first enable (sim) | per device, so sort per side; it runs backwards in file order across sides |
| Ping counters | per device; offsets 0, −1 and +60 have been measured | align before pairing (CM-6); the quality report gives the first-ping offset |
| MAVLink `time_boot_ms` | autopilot | **frozen** in processor-written files (`read_svlog().clock_frozen`), then pair by file order |
| ROS `header.stamp` | node clock; sim time under Gazebo | a real-launch MAVROS stamp can be 0 (Control §6); use bag receive times |
| Sim pose | Gazebo per-step pose, sim-time stamped | the sonar uses it if < 0.1 s old, predicted to the tick (≤ 0.15 s) |

A timing-latency calibration (stage 1) estimates the offset between the
sonar and navigation clocks; it is not modelled today.

## 4. Frames and units

**Frames and angles:**
- **World:** ENU, z = 0 at the still surface, depths negative; world
  (0, 0) is the GPS anchor (equirectangular, `core/geo.py`).
- **Yaw:** ENU internally; compass degrees on hardware-facing fields.
- **Body:** +y = port, `Side.PORT.sign = +1`. Port looks along
  heading + 90°. On the wire, `transducer_heading_deg` is the mounting
  angle (−90 port, +90 starboard).
- **Vertical datum (2026-10-21):** the renderer's vehicle z = 0 is the
  boat floating level at rest. `sensor_depth_m` is the transducer face
  below the still waterline. Gazebo's base_link floats at
  `base_link_rest_z_m` (−0.111 m) and is moved onto that datum by the node.
  A real-boat depth inversion uses the same `sensor_depth_m` plus
  `datum_offset_m` (tide/draft).

**Range bins and levels:**
- **Bins:** slant bin `i` has centre `start + (i + ½)·L/N` in the renderer
  and the calibration package. Some analysis tools use `i·L/N` or
  `i·L/(N−1)` (triage N2): state the convention with any sub-bin result.
- **Levels:** `pwr_results` are per-ping normalised counts. Invert each
  ping as `dB = min + raw/65535·(max − min)` before averaging. Absolute
  level lives in the endpoints at the reported `gain_index`.

## 5. Calibration artifact (`blueboat_calibration_artifact/1`)

YAML or JSON, validated by `calibration/artifact.py`
(`sss_calibration validate-artifact A.yaml`). Print an example with
`sss_calibration example artifact`; it is a placeholder, so it validates
with a warning and must never be applied.

| Key | Required | Content |
|---|---|---|
| `format`, `id`, `created` | yes | format string; unique id; ISO date |
| `parameters` | yes | `name: {value, unit, status: measured\|joint\|estimated, uncertainty: {sd \| interval \| equivalence_range \| unidentified}}`; name registered and `tunable`, unit = registry unit, value within bounds |
| `sessions` | no | `session_id: {name: {...}}`, only `installation` / `session_nuisance` classes, only calibration sessions |
| `data` | yes | `calibration_sessions` (non-empty), `validation_sessions` (disjoint), `manifest` (name/hash) |
| `fit` | yes | `method`, `objective` (non-empty), `bounds`, optional identifiability summary |
| `assumptions` | yes | non-empty list |
| `validation` | yes | `[{metric, session (a validation session), value, acceptance, passed}]` |
| optional | — | `created_by`, `description`, `software`, `scope`, `base_profile`, `excluded`, `notes` |

The document is rejected for any of these:

- a key named like a position fix (`lat`, `lon`, `gps`, `track`, …);
- any numeric list longer than 64 entries;
- an unknown top-level key.

Per-session values that all deviate one way from the global value raise a
**warning**. Artifacts are write-once.

## 6. Effective profile (`blueboat_effective_profile/1`)

A normal sonar profile, written by `sss_calibration effective-profile`, in
which **every** acquisition and model field is stated, plus a
`provenance:` block. That block holds `sources` (profile, artifact id,
session, run overrides), `from_layer` (each non-default value's layer) and
`not_applied` (artifact values outside the sonar profile, with their
owner). `SonarConfig.from_yaml` reads only `acquisition:` and `model:`, so
the file runs as is: `sonar_config:=<file>`.

## 7. Quality report (`blueboat_svlog_quality/1`)

Per recording:

- **Identity:** `file` (name only), `size_bytes`, optional `sha256`.
- **Content:** the digest (packet census, per-side acquisition, gain
  census, pulse, ping period vs the device law, counter range and
  restarts, channel-255 count, FBR median and detection rate, nav counts,
  clock regime), plus the three normalisation invariants.
- **Verdict:** `flags` (`error` / `warn` / `info`, with code and message)
  and `usable` (no `error`).

The summary file is `quality_summary.json`.

| Flag | Level | Meaning |
|---|---|---|
| `invariant_*` | error < 99 %, warn < 100 % | per-ping normalisation broken (a real log had 1 of 4236 pings with two tied full-scale bins) |
| `<side>_gain_walk` | warn | several gain indices; level fits use the per-ping gain |
| `<side>_mixed_acquisition` | warn | several range/bin settings; reduce per setting |
| `<side>_counter_restart` | warn | device or sim restart under the recorder |
| `<side>_period` | warn | ping period > 5 ms off the device law |
| `<side>_weak_fbr` | warn | bottom found on < 90 % of sampled pings |
| `clock_frozen` | warn | pair navigation by file order |
| `no_gps`, `one_side` | warn | |
| `<side>_channel_255`, `counter_offset_first` | info | |

## 8. Simulation ground truth, for datasets

`~/ground_truth/contacts` and `~/ground_truth/sensor` (`docs/topics.md`).
A dataset built from a simulated run should record:

- the world folder (name + manifest hash);
- the effective sonar profile;
- the node's `noise_seed` (now on the sensor topic);
- the sea-state seed;
- `base_link_rest_z_m`.

It should exclude, or flag, pings whose `out_of_map_from_m` falls inside
the labelled range.
