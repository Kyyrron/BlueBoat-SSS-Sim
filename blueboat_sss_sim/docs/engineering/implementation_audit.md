# Implementation audit — what the simulator is, as built (2026-10-21)

The audit read the source tree and ran its gates; this page states what
the code does. Where a design document says otherwise, the code wins and
the difference is in [`issue_triage.md`](issue_triage.md). Deeper
references: `docs/architecture.md` (module map), `docs/sonar_model.md`
(physics, A1–A13, the calibration record §6), `docs/topics.md` (wire
contracts), `.claude/CLAUDE.md` (the non-negotiables).

## 1. What was inspected and run

**Read:**
- `ros/sss_sim_node.py` (pose sources, prediction, tick, publishing, seeds)
- `sonar/` (renderer, acoustics, facets, multipath, noise, encoder, config)
- `worldgen/` (scene, objects, materials, the out-of-map rule, the world folder)
- `sea/` (hull trim, motion, high-rate pose)
- the launch resolvers
- `analysis/` (calibration, calibration CLI, digest, reader, compare)
- the smoke suite's structure and fixtures
- `config/*.yaml` and `docs/*.md`
- the BlueBoat xacro (Control), for the base_link trim

**Run (Python 3.13, numpy 2.4.4, scipy 1.17.1, no ROS):**

- `python3 -m test.smoke_test`, on the pristine tree and after each stage (§6)
- the read-only digest of the three attached `.svlog` samples
- deterministic reproductions of every confirmed defect
- the new `sss_calibration` commands

## 2. Execution and data flow per ping

```
Gazebo base_link pose (100 Hz /sim/sea_state/boat_pose; 20 Hz /blueboat/odom fallback)
  -> _ping_pose: freshest source (< 0.1 s), predicted to the tick (sea.motion.predict_pose, dt <= 0.15 s)
  -> waterline_pose: z - base_link_rest_z_m                 (2026-10-21; -0.111 m default)
  -> GeometricRenderer.render(side, pose, t_sim), per side:
       _sensor_pose: lever arm [mount_x, ±mount_y, -sensor_depth] rotated by roll/pitch/yaw
       _integrate: 5 ground lines (azimuth beam), each _ping_geometry:
           heights (pitch refinement) -> facet-object occluders -> horizon cull (= shadow)
           -> transmitting crests -> land mask (NC #13) -> wall bands (direct path)
           -> incidence, roughness, area element, slant intervals; out_of_map_from_m
       _shade_into: acoustics.surface_response (diffuse ρ·cos^p + coherent lobe, beam, range law, TVG)
       layover cap -> power / specular -> wall face echo -> ringing -> texture
       -> pulse smear -> facet layer (Kirchhoff patches; terrain, object AND wall occlusion)
       -> second-bottom ghost -> ground-truth contacts (read off the rendered mask) -> wall contacts
       -> wall / surface mirror ghosts (first leg read on the near side, 2026-10-21)
  -> drop? (dropped_ping_prob) -> water-column mask -> noise.apply_ping_noise
     (texture × diffuse + Gamma(25) × coherent + max(receiver floor, reverberation floor) + water column)
  -> PingEncoder.encode: dB = 10log10 P + 81.1 + gain step; per-ping normalisation, 90 dB clamp, uint16
  -> publish profile + raw frame per side (same tick, NC #3); then contacts JSON (if any) and sensor JSON
```

## 3. Configuration precedence

| What | Order (later wins) | Where |
|---|---|---|
| Sonar profile file | package default → a legacy bundle's frozen `sonar.yaml` → explicit `sonar_config:=` | `launch/sss_sim_launch.resolve_sonar_config`; `full_mission_launch.py` forwards `sonar_config` since 2026-10-21 |
| Acquisition (6 params) | dataclass default → profile `acquisition:` → non-empty launch override → ROS param at each enable | `resolve_acquisition`; node re-reads on every enable and warns on divergence |
| Model | dataclass default → profile `model:` (unknown keys raise) | `SonarConfig.from_yaml` |
| Calibrated composition | defaults → profile → calibration artifact → artifact session values → run acquisition | `calibration.artifact.effective_config` (new; writes a self-contained profile) |
| GPS anchor of the shim | launch args → world manifest `geo.anchor` → legacy constant (warning) | `resolve_world_anchor` |
| Vertical datum | Gazebo base_link z − `base_link_rest_z_m` | node, 2026-10-21 |

## 4. Contracts touched by this audit

All additive; no topic, type, QoS or wire field changed.

| Contract | Change |
|---|---|
| `~/ground_truth/sensor` JSON | new keys: `base_link_rest_z_m`, `noise_seed`, `sensors.<side>.out_of_map_from_m`; `sensors.*.pose` now on the waterline datum (`boat` remains the received pose) |
| node params | new `base_link_rest_z_m`; `seed` 0 now resolves to a logged concrete seed |
| launch args | `sss_sim_launch.py`: `sonar_seed`; `full_mission_launch.py`: `sonar_config`, `sonar_seed` |
| raw frame / `OmniscanProfile` | `min_pwr_db` of a clamped ping may move by float32 ulps so the float32 pair never spans more than 90 dB |
| `Ping` (Python) | new optional `out_of_map_from_m` |
| new files | `config/parameter_registry.yaml`; `blueboat_sss_sim/calibration/`; console script `sss_calibration` |

## 5. Random streams

| Consumer | Stream | Recorded? |
|---|---|---|
| Terrain / objects | `[seed, 0]` / `[seed, 1]` with `depth:`; one legacy stream without | world manifest `seed` + config |
| Depth layers | `[seed, 2, layer.seed or index]` | manifest config |
| Rest pose | `[2, id, x_mm, y_mm]` | via object pose |
| Object shape wiggle | `default_rng(object_id)` | positional (triage #8) |
| Builder preview terrain | `default_rng(0)` (≠ build, triage #22) | — |
| Speckle texture phases | hash of (world seed, `scatterer_seed`) | world seed + profile |
| **Sensor noise, drops, drift** | node `seed`; 0 → entropy | **now logged + `noise_seed`** |
| Sea state | `sea_seed` → schedule → 1 | `/sim/sea_state` JSON |
| Calibration CLI | `--seed` (default 7) | report |

## 6. Gates and their results in this environment

| Command (from `blueboat_sss_sim/`) | Result |
|---|---|
| `python3 -m test.smoke_test`, pristine tree, before installing `libegl1` | crashed in `[12]` on `ImportError: libEGL.so.1` (PySide6 QtGui) after 381 PASS, 0 FAIL — environment, not code |
| same, pristine tree, after `apt-get install libegl1` | `ALL CHECKS PASSED`: 426 PASS lines, 0 FAIL, 4 visible SKIP (no corpus `[2f]`; no real `sss_node.py` `[4b]`; no ROS in `[10]` and `[13]`), 6 min 07 s |
| same, after this audit's changes | `ALL CHECKS PASSED`: 446 PASS lines (426 + 20 new in `[15]`/`[16]`), 0 FAIL, the same 4 SKIP, 6 min 10 s |
| `QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke`, before and after | `GUI SMOKE PASSED` both (120 PASS lines) |

The 439 count in `.claude/CLAUDE.md` is from a machine with the corpus and
the BlueBoat-SSS checkout; `[2f]` and the real-driver half of `[4b]` add
the difference.

## 7. Known limitations

This list is not exhaustive. See A1–A13 and the triage.

- **Model:** straight rays, constant 1500 m/s, first-order reflections,
  stop-and-hop pings, a static 2.5-D scene (A1–A6).
- **Speckle:** sediment-independent, Exp(1) diffuse / Gamma(25) coherent.
- **Seabed law:** one global angular exponent, measured over sand only.
- **First bottom return:** the reported peak is still +4–7 dB hot (the
  documented residual).
- **Object edges:** about 10× sharper across-track than the field box.
- **Gain:** no AGC; gain indices 0–3 unmeasured.
- **Yaw:** simulated yaw rate reported 4–10× below field.
- **Installation:** `sensor_depth_m` and `mount_x_m` are unmeasured
  estimates.
- **Fixed constants:** the BlueROV2 hydrodynamic coefficients in Gazebo
  and the hull damping are judged values.
