# Sensor installation and pose path

Moved from `.claude/CLAUDE.md` §3.1 on 2026-11-10. Read before touching the pose path
(`ros/sss_sim_node.py` pose handling, `renderer._sensor_pose` / `_pitch_centre`,
`sea.motion`), the mount / depth constants, or comparing wave-run or Gazebo recordings
across dates.

**Installation (2026-11-02, field-calibration pass 3).** `sensor_depth_m` **0.22**
(measured) and `mount_x_m` **−0.345** (the drawing, vs base_link = the hull centre), in the
code default and every comparison profile; `field_candidate_v1_sonar.yaml` keeps its pass-2
0.15 / 0.0. A replay of a real recording takes the fore/aft arm relative to *that corpus's*
navigation fix (`calibration.scene_replay.with_installation`). `sss_processor_node` moved
to 0.22 m the same day; its fore/aft stays 0.0, unused by any georeference (superproject §4.3).

**Vertical datum (2026-10-21).** The renderer's vehicle z = 0 is the boat
floating level on the still water, so `sensor_depth_m` is depth below the
still waterline -- what every offline and calibration render assumes.
Gazebo's base_link rests 0.111 m below that
(`sea.motion.GAZEBO_BASE_REST_Z_M`, the URDF box's equilibrium); the node
now subtracts `base_link_rest_z_m` from every received pose before
rendering (`0.0` = the old ~0.26 m submersion). `ground_truth/sensor`'s
`boat` stays the received pose; `sensors.*.pose` is the rendered one.
Gazebo recordings made before 2026-10-21 are not comparable bit for bit.

**The sonar follows the hull (2026-10-08; 100 Hz pose 2026-10-09).** Each
ping is rendered at the boat's pose propagated to the ping time with its
own rates (`sea.motion.predict_pose`, the function the hull wrench uses):
Gazebo's per-step model pose, stamped (`/sim/sea_state/boat_pose`, bridged
by `sea_state_launch.py` from blueboat_description's own `PosePublisher`;
`sea.motion.HighRatePose` differences the rates), the 20 Hz odometry only
as a fallback. The sea-state node uses the same pose for the hull wrench:
on the 20 Hz odometry the catamaran's stiff roll restoring rang at 3.2 Hz
(`docs/sea_state_model.md` §5.6). `renderer._sensor_pose` rotates the lever arm
`[mount_x, ±mount_y, −sensor_depth]` by the full attitude (yaw only
before). Level poses are bit-identical to before, so the render fixture
and every calm-water comparison stand; any wave-run recording made before
2026-10-08 is not comparable bit for bit. **Pitch moves the beam centre,
not the samples (2026-10-30):** the coherent scatterer field is a ground
lattice, and shifting every sample by `dz·tan(pitch)` re-drew the speckle
each ping (track lag-1 0.76 → 0.24 under a 0.4°/ping pitch walk);
`renderer._pitch_centre` keeps the lines on the lattice and weights them
about the shifted centre, only the relief residual moves samples (smoke
`[10]`). Wave-run recordings made before it are not comparable bit for bit.

**Waves and speckle under motion (2026-11-10).** The wave levels were re-derived
(Tp = 3.6 √Hs, new `harbour_chop`), the field resolves the slope spectrum to 3 Hz
(`hull.wave_response.discretisation: hybrid`) and the hull reads the pressure at its
draft: wave-run recordings made before 2026-11-10 are not comparable bit for bit
(`docs/sea_state_model.md`, "2026-11-10 revision"). Separately, the geometric renderer's
coherent field re-draws its speckle under **yaw** (and sway on oblique headings): each
azimuth line reads one 1 cm row of cells, so 0.1° of yaw moves a 10 m sample onto other
cells (track lag-1 0.73 → 0.08 on a flat-sand walk; 0.16 in the harbour wave run vs
0.37–0.44 real). `model.scatterer_anchoring: ground` (opt-in,
`config/ground_anchored_sonar.yaml`) reads every cell of the line's footprint strip:
yaw then keeps the physical footprint-overlap correlation (0.49 vs 0.74 level ≈
(1 − 0.1/0.5)²), sway / roll / heave ≥ 0.85 × level, same cost and mean level (smoke
`[10]`). The default stays `line` (render fixture unchanged); making `ground` the
default is a deliberate rebaseline -- the owner's call after a look test.

