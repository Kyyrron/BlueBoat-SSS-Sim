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

**Two odometries (2026-11-15, R-7 of Control's `REQUESTS.md`).** Since Control's session 04,
`/blueboat/odom` in a field-equivalent run is what the real boat's software sees: field-rate
(4 Hz stretches, 25 Hz bursts) and delayed, relayed by `simulation_interface` from Gazebo's
own `/blueboat/odom_gz` (20 Hz, true). The sonar node and the sea-state node still render /
push on the 100 Hz model pose first; their **fallback is `/blueboat/odom_gz`** while it is
fresh (< 1 s, parameter `true_odom_topic`), then `/blueboat/odom` (`odom_topic`; the only
pose of a bag replay, so `sss_sim_launch.py`'s own default stays `/blueboat/odom`). The MAVROS
shim stays on `/blueboat/odom`: processing and georeferencing see the delayed, field-like pose,
the sensor and the sea the true one -- the realistic split. `full_mission_launch.py` derives
the topics from `sim_config` (`resolve_sim_config`, smoke `[10]`): with the raw bridge
(field-equivalent with the control stack) the fallback is `odom_gz`; legacy runs and runs
without the control stack bridge `/blueboat/odom` directly (`odom_raw:=false`; before
2026-11-15 a `with_control:=false` run had no `/blueboat/odom` at all). The sonar's
`run_info` records which source it used (`pose_source`) and the launch's run context
(`simulation_context`: sim_config, plant, odometry profile, tuning-file and schedule SHA-256).
Motion during a ping stays an explicit assumption: the ping is rendered at the pose
propagated to its own time, one pose per ping.

**The pose at the ping's own stamp (2026-11-16, session 02b, CR-03-5).** rclpy's executor runs
the ping timer before subscriptions and takes one message per subscription per wake, while its
`/clock` (depth 1) is always current: after any stall (a ray-cast render, a second Gazebo on the
machine) the pose cache trailed the clock by up to 100 ms (session 03: 0.4-0.7 % of ray-cast
pings; the gate's intermittent "fresh pose" failures since 2026-10-20, load-related). Each tick
now first takes the pose messages already delivered (`_drain_pose_queues`, rclpy's own
`take_message`), and a source newer than the ping gives its newest sample at or before the stamp
(64 x 100 Hz / 16 x odom_gz history, `_at_or_before`). Single-threaded on purpose: a
`MultiThreadedExecutor` let the poses run ahead of the clock and starved the ping timer (245 pings
in 295 s). Fresh-pose pings are bit-identical (offline check, 20 000 queries). Evidence and
before/after: `~/ros2_ws/sim_runs/sim_acceptance/02b_raycast_fix/handoff_02b.md` §4.
