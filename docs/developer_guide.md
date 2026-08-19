# Developer guide

For anyone extending the package. Read `architecture.md` first.

## 1. Conventions

* **Frames.** World = ENU, z = 0 at the water surface, depths negative in
  the heightfield. Headings: internal math uses ENU yaw (rad, CCW from
  +x); anything user/hardware-facing uses compass degrees
  (`enu_yaw_to_compass_deg`, 0 = North, CW) — conversion lives in
  `core/geometry.py` only.
* **Sides.** `Side.PORT.sign = +1` (+y in body frame), `STARBOARD = −1`.
  Transducer heading = vehicle heading ∓ 90°.
* **Randomness.** Every stochastic component takes a
  `numpy.random.Generator`; nothing touches the global RNG. World content
  derives entirely from `world.seed`, so bundles are reproducible.
* **Typing/docs.** Full type hints, module docstrings state contracts;
  `core/` and the science modules must stay importable without ROS.
* **No new messages.** The package reuses `blueboat_interfaces`. If you
  need new data channels, prefer JSON on `std_msgs/String` (as the
  ground-truth topic does) or add messages in `blueboat_interfaces`
  itself.

## 2. Code tour (read in this order)

1. `core/types.py` — every dataclass that crosses module boundaries.
2. `worldgen/scene.py` — `SceneModel`: the single source of truth;
   `generate_scene` composes `terrain.py` + `objects.py`.
3. `sonar/renderer.py` — `GeometricRenderer.render()` is ~100 lines and
   contains the entire acoustic pipeline for one ping; `acoustics.py`
   holds the pure formulas.
4. `sonar/encoder.py` — quantisation + Ping-Protocol framing;
   `parse_frame` is its own unit test oracle.
5. `dataset/` — pure functions from ping streams to YOLO tiles.
6. `ros/sss_sim_node.py` — the thin shell wiring 1–5 to topics/timers.

## 3. Common extensions

### Add a litter object type
In `worldgen/objects.py`: add a `CATALOG` entry (name, material, size and
burial priors, mask primitive). If none of the existing primitives (box,
lying/upright cylinder, annulus, capsule, dashed line, cross, blob) fits,
add a mask function next to them — it receives a local grid and returns a
boolean footprint plus a height profile. The labeler discovers classes
automatically; no other file changes.

### Add a survey pattern
Implement `my_pattern(...) -> WaypointTrajectory` in
`mission/patterns.py` and register it in `build_pattern`. Time
parameterisation and the `RequestPath` service come for free.

### Replace the acoustic model
Subclass `SonarRenderer` (`render(side, pose, t_sim) -> RenderedPing`) and
instantiate it in `sss_sim_node._start_pinging`. Everything downstream
(noise, encoding, topics, dataset) is renderer-agnostic. Keep
ground-truth contact emission if you want auto-labeling to survive.

### Tune realism
All noise/physics knobs are in the sonar YAML (`configuration_guide.md
§2`); zeroing the noise group yields clean-physics images for ablation.

## 4. Testing

* `python3 -m test.smoke_test` — offline end-to-end (world → render →
  encode → decode → label → export), asserts frame byte-parity with the
  field capture, YOLO validity, shallow-regime altitudes, and produces a
  visual waterfall preview in `/tmp/blueboat_sss_smoke/`. Run it after any
  change; it needs only numpy/scipy/yaml/PIL.
* Frame-level: `parse_frame(encode(...))` round-trips are the contract
  for the raw stream; extend those checks if you touch `encoder.py`.
* In-graph: `ros2 topic hz /side_scan_sonar/port/profile` should match
  `ping_period_s()` (≈45 Hz per side at 15 m free-run… i.e. 22 ms), and
  `ros2 topic echo --once .../raw | head` should start with `[66, 82,`
  (`'B'`, `'R'`).

## 5. Performance notes

`GeometricRenderer` costs one heightfield line sample (~600 points at the
default `sample_step_m`) plus O(n) numpy ops per ping per side — a few
hundred µs; free-run dual-channel at 45 Hz uses well under one core.
`sample_step_m` and scene `resolution` are the levers if you enlarge
worlds. World generation is seconds; the STL is decimated to ≤200
vertices/axis purely for Gazebo visuals (acoustics always uses the
full-resolution raster).

## 6. Gotchas

* The recorder keys ground truth by `(side, ping_number)`; if you add a
  second sonar node instance, namespace it or the keys collide.
* `dropped_ping_prob` advances the encoder's counter on drops
  (hardware-faithful) — don't "fix" that.
* `world.resolution` coarser than ~half a range bin causes visible
  stair-stepping in shadows.
* Launch files follow the project's `simple_launch` style; keep new ones
  consistent.
