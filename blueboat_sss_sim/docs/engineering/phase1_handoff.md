# Phase-1 handoff (Cloud stage, 2026-10-21) → the Computer session

This stage had the source tree but **none of the private field data**.
Nothing here is a field calibration and no physical constant was fitted.
The next session will not share this filesystem: everything below is in
the committed tree (branch `claude/optimistic-hypatia-rx6ghs` of
BlueBoat-SSS-Sim).

## 1. What changed

Paths are relative to `blueboat_sss_sim/` unless they start with `.claude/`
or are `README.md` (both at the repo root). Evidence for every fix:
[`issue_triage.md`](issue_triage.md).

| Change | Files | Behaviour impact |
|---|---|---|
| **Facet objects occluded by walls** (F1) | `sonar/renderer.py`, `sonar/facets.py` | scenes with walls and smooth objects behind them: their glint is gone, as their label already said. No walls → identical |
| **Wall ghost first leg read on the near side** (F2) | `sonar/renderer.py` | land or apron behind a quay no longer deletes the ghost. Flat basins identical |
| **Gazebo vertical datum** (F3) | `sea/motion.py`, `ros/sss_sim_node.py` | Gazebo runs: transducers 0.111 m shallower (0.15 m below water, as calibrated). `base_link_rest_z_m:=0.0` restores. Offline renders identical |
| **Atomic explicit object entries** (F4) | `worldgen/objects.py` | hand-written entries naming a `material` without `reflectivity`, or giving `params` size knobs. Builder worlds identical |
| **Wire span ≤ 90 dB in float32** (F5) | `sonar/encoder.py` | `min_pwr_db` of clamped pings moves by float32 ulps. Counts identical |
| **Recorded noise seed** (F6) | `ros/sss_sim_node.py`, `launch/sss_sim_launch.py` | `seed` 0 → a logged concrete seed, `noise_seed` on `ground_truth/sensor`; `sonar_seed` launch arg |
| `full_mission_launch.py` forwards `sonar_config`, `sonar_seed` (F7) | `launch/full_mission_launch.py` | additive |
| Calibration report noise parity (F8) | `analysis/calibration_cli.py` | shadows in `sss_calibration_report` now match a live run |
| `reverberation_db: 18.0` stated in the default profile (F9) | `config/default_sonar.yaml` | none (same value) |
| **Out-of-map reporting** (F10) | `core/types.py`, `sonar/renderer.py`, node | diagnostics only: `Ping.out_of_map_from_m`, per side on `ground_truth/sensor`, one warning |
| **Calibration package** | `blueboat_sss_sim/calibration/` (registry, artifact, manifest, quality, identifiability, `__main__`), `config/parameter_registry.yaml`, `setup.py` (`sss_calibration`) | new, ROS-free |
| Tests | `test/smoke_test.py` `[15]` (6 checks), `[16]` (14 checks) | |
| Docs | `docs/engineering/*` (8 files); `docs/{sonar_model,configuration_guide,topics,architecture,developer_guide,roadmap}.md`; `README.md`; `.claude/CLAUDE.md`; `.claude/TODO.md` | |

## 2. Commands run and results (this container: Python 3.13, numpy 2.4.4, no ROS)

| Command (from `blueboat_sss_sim/`) | Result |
|---|---|
| `python3 -m test.smoke_test`, pristine tree, as found | crashed in `[12]`: `ImportError: libEGL.so.1` (PySide6 QtGui) after 381 PASS / 0 FAIL — environment |
| `apt-get install -y libegl1` | fixed the import |
| `python3 -m test.smoke_test`, pristine tree | `ALL CHECKS PASSED`: 426 PASS, 0 FAIL, 4 SKIP, 6 min 07 s |
| `python3 -m test.smoke_test`, **final tree** | `ALL CHECKS PASSED`: **446 PASS** (426 + 20 new), 0 FAIL, 4 SKIP, 6 min 10 s |
| `QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke`, pristine and final | `GUI SMOKE PASSED` both (120 PASS lines; the logged "run.svlog: only 0 pings" traceback is pre-existing and expected) |
| `python3 -m blueboat_sss_sim.calibration registry --check` | 64 parameters, 0 drifted, 0 implicit |
| `… identifiability --param model.tvg_compensation --param model.lambert_exponent --param model.calibration_db_offset --altitude 1.40 --altitude 7.33` | condition number 29.5, no ridge (one altitude: 3.5e4, every pair \|corr\| ≈ 1) |
| `… quality --out <scratch> <3 attached .svlog>` | `object_boiteEncore`: usable, warn (1/4236 tied full-scale bins, gain walk 4–6, mixed ranges). `fullHarbourCleanExample`: usable, warn (gain walk 4–7, two ranges). `2026-09-18-17-50-54` (inferred simulated, pre-fix encoder): flagged, 628 pings span > 90 dB by float32 rounding (F5) |
| `python3 -m py_compile` node + launch files | ok (no ROS to run them) |

**The four SKIPs, and why:**
- `[2f]`: no corpus at `~/ros2_ws/data/SSS_data`.
- `[4b]`: no `BlueBoat-SSS` checkout, so the real driver comparison cannot
  run.
- `[10]`: no `simple_launch`, so the bridge checks cannot run.
- `[13]`: no `rclpy`, so the node pose fallback cannot run.

**Not run here:**
- the Gazebo gate (`test/gazebo_sea_test.py`);
- `colcon build`;
- any ROS launch.

## 3. Where things live

- Design: [`field_calibration_architecture.md`](field_calibration_architecture.md)
- Contracts and schemas: [`dataset_and_calibration_contracts.md`](dataset_and_calibration_contracts.md)
- Registry: `config/parameter_registry.yaml` (truth) and [`parameter_registry.md`](parameter_registry.md) (view)
- Metrics, order and experiments: [`validation_and_experiment_plan.md`](validation_and_experiment_plan.md)
- Code: `blueboat_sss_sim/calibration/`; the existing reductions in `analysis/calibration.py`, `analysis/svlog_compare.py`, `analysis/wall_contrast.py`, `analysis/object_signature.py`
- `python3 -m blueboat_sss_sim.calibration --help`; `… example artifact|manifest` prints templates

## 4. Blocked by private data (exact evidence needed)

| Decision / fit | Evidence needed |
|---|---|
| `sensor_depth_m`, `mount_x_m` (installation) | dock measurement of each transducer face below the waterline at survey load; reciprocal passes over a surveyed point target |
| sonar ↔ navigation latency (not modelled yet) | the same reciprocal passes at two speeds |
| yaw response 4–10× low (reported) | run bags/poslogs with commanded thrust and measured yaw rate in calm water; the same manoeuvres in Gazebo |
| gain indices 0–3 | deep-water pings at each forced gain index |
| seabed law per bottom class (`lambert_exponent`, level) | ≥ 3 altitudes over each independently classified flat bottom, fixed gain, reciprocal headings |
| FBR peak +4–7 dB hot; `specular_width_deg`, `specular_looks` | nadir returns at ≥ 5 altitudes over one classified bottom |
| object edge 10× sharp; `smooth_lobe_width_deg`; µ_sand | reference targets of known size/material at surveyed yaw at 3, 6, 12 m |
| sediment-dependent speckle | long straight lines per bottom class |
| any validation claim | held-out sessions that no fit touched (manifest roles) |

## 5. Runbook for the Computer session

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS-Sim/blueboat_sss_sim
python3 -m test.smoke_test                          # expect ALL CHECKS PASSED; [2f], [4b] run there
python3 -m blueboat_sss_sim.calibration registry --check
# 1. describe the corpus: one manifest, whole-session roles
python3 -m blueboat_sss_sim.calibration example manifest > ~/calib/corpus_manifest.yaml   # then edit
python3 -m blueboat_sss_sim.calibration validate-manifest ~/calib/corpus_manifest.yaml \
    --root ~/ros2_ws/data/SSS_data --check-files --verify-hashes --json ~/calib/manifest_check.json
# 2. quality of every recording (out OUTSIDE the corpus tree)
python3 -m blueboat_sss_sim.calibration quality --manifest ~/calib/corpus_manifest.yaml \
    --root ~/ros2_ws/data/SSS_data --out ~/calib/quality --hash
# 3. before each fit: is it identifiable with the sessions you have?
python3 -m blueboat_sss_sim.calibration identifiability --param model.tvg_compensation \
    --param model.lambert_exponent --param model.calibration_db_offset --altitude <alt1> --altitude <alt2> --out ~/calib/ident
# 4. fit stage by stage (validation plan §2) with analysis/calibration.py reductions;
#    write the result as an artifact, validate, compose, and validate on held-out sessions
python3 -m blueboat_sss_sim.calibration validate-artifact ~/calib/cal-<id>.yaml
python3 -m blueboat_sss_sim.calibration effective-profile --artifact ~/calib/cal-<id>.yaml --out ~/calib/eff-<id>.yaml
ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=<w> sonar_config:=$HOME/calib/eff-<id>.yaml sonar_seed:=<n>
# 5. Gazebo check of the datum fix: ground_truth/sensor sensors.*.pose z at rest ~= -0.15 m
python3 -m test.gazebo_sea_test --world-dir ~/worlds/sea_validation/flat_5p0m --out <dir> --quick
```

**Recommended order:**
1. run the smoke suite with the corpus;
2. write the manifest of the existing corpus (the two NC #6 anchors are
   **calibration** data);
3. run the quality reports;
4. check the datum fix in Gazebo;
5. stage 1 (geometry), with new dock measurements;
6. stage 2 (dynamics, from Control's run bags);
7. stages 3–4.

Write each stage as its own artifact.

## 6. Calibration guardrails — constants NOT changed in this stage

These still hold the values documented in `sonar_model.md` §6:

| Group | Constants |
|---|---|
| Level and range law | `calibration_db_offset` 81.1, `tvg_compensation` 0.30, `spreading_exponent` 4.0, `lambert_exponent` 1.0 |
| Coherent lobe | `specular_spreading_exponent` 2.0, `specular_strength` 12, `specular_width_deg` 8, `specular_looks` 25 |
| Beam and mounting | `beam_tilt_deg` 20, `vertical_aperture_deg` 50, `horizontal_aperture_deg` 0.5, `beam_sidelobe_floor` 0.004, `array_length_m` 0, `sensor_depth_m` 0.15, `mount_x_m` 0, `mount_y_abs_m` 0.285 |
| Water and floors | `absorption_db_per_m` 0.10, `noise_floor` 3.39e-9, `watercolumn_noise` 4.68e-6, `reverberation_db` 18 (now stated in the YAML, same value) |
| Device | `ringing_level_db` 55, `ringing_decay_db_per_m` 22, `max_span_db` 90, the gain table, `filter_duration_percent` 0.0015, the pulse floor 44.3 µs, `max_ping_rate_hz` 20 |
| Objects and walls | `sand_lambert_mu_db` −12, every material constant and transmission, every wall constant |
| Acquisition defaults | all six; the render fixture was not rebaselined |

## 7. Residual risks, by priority

| P | Risk | Evidence | Acceptance gate |
|---|---|---|---|
| P0 | Datum fix verified offline only | node `py_compile`; `[15]` helper checks | Gazebo: at rest, `sensors.*.pose` z = −0.150 ± 0.01 m |
| P0 | Calibration still rests on two recordings, one bottom (sand) | `sonar_model.md` §6 | held-out sessions per regime within the repeatability floor |
| P1 | Yaw response, FBR peak, object edge residuals | memo, TODO | stage 2 / 4 / 5 metrics on held-out data |
| P1 | No fitter yet; identifiability is local and noise-free | `calibration/identifiability.py` | first fit reports uncertainty + ridge check in its artifact |
| P2 | Triage N1–N9 (strict class keys, bin-centre convention, per-side RNG, odometry staleness, `[4d]` lab) | `issue_triage.md` | — |
