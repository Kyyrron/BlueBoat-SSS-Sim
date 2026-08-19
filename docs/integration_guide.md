# Integration guide

How to run `blueboat_sss_sim` alongside the existing BlueBoat simulator, and
how sim ↔ real swapping works. Nothing in the existing packages is
modified.

## 1. Install

```bash
cd ~/ros2_ws/src
# (copy/clone blueboat_sss_sim here, next to blueboat_description etc.)
cd ~/ros2_ws
rosdep install --from-paths src -yi          # numpy/scipy/yaml/PIL/simple_launch
colcon build --packages-select blueboat_sss_sim
source install/setup.bash
```

Requires the existing `blueboat_interfaces` package (for
`OmniscanProfile` and `RequestPath`) — already in your workspace.

## 2. Quick start (three commands)

```bash
# 1. Generate a self-contained mission bundle (world + trajectory + sonar cfg)
ros2 run blueboat_sss_sim generate_mission \
    --config $(ros2 pkg prefix blueboat_sss_sim)/share/blueboat_sss_sim/config/default_mission.yaml \
    --out ~/runs/r1 --seed 7

# 2. Inspect the world (optional)
ros2 launch blueboat_sss_sim sim_world_launch.py mission_dir:=$HOME/runs/r1

# 3. Full autonomous mission: Gazebo + robot + control stack + sonar + dataset
ros2 launch blueboat_sss_sim full_mission_launch.py mission_dir:=$HOME/runs/r1
```

The YOLO dataset accumulates in `~/runs/r1/dataset/` and is finalized
(`dataset.yaml`) on shutdown (Ctrl-C).

## 3. Composition with the existing launch files

`full_mission_launch.py` includes `blueboat_description/world_launch.py`
with a `world:=<bundle>/world.sdf` argument. If your `world_launch.py`
does not yet expose a `world` argument, either:

* add one line declaring the argument and pass it to the Gazebo process
  (the generated `world.sdf` embeds the same plugin block as the stock
  world, so everything else is unchanged), or
* set `use_existing_world_launch:=false`, which starts
  `ign gazebo -r <bundle>/world.sdf` directly and you launch the robot
  spawn separately as you do today.

The generated world uses `ignition-gazebo-*` plugin names (Fortress). For
Garden/Harmonic set `gazebo_plugin_prefix: gz` in the mission YAML.

## 4. Mission path service vs. the existing one

`sss_path_generation` serves the **same** `RequestPath` service on
`/path_request` as the existing `path_generation.py` — the unmodified
`master_control.py` / `path_publisher.py` track generated survey missions
with zero changes. Start exactly one of the two:

* survey missions → `sss_path_generation`
  (`trajectory_file:=<bundle>/trajectory.yaml`), as `full_mission_launch`
  does;
* the original analytic paths → the existing node, and skip
  `with_mission_path`.

## 5. Sim ↔ real swap

The sonar interface is identical, so the swap is one node choice:

| | Real boat | Simulation |
|---|---|---|
| sonar node | `sss_node.py` (hardware) | `blueboat_sss_sim sss_sim_node` |
| pose source | vehicle nav | `/blueboat/odom` from Gazebo |
| everything downstream | unchanged | unchanged |

The `raw` topics remain byte-valid Ping Protocol, so `.svlog`
reconstruction and any Cerulean tooling work on simulated data too. The
only observable differences: the extra `~/ground_truth/contacts` topic
(additive) and simulated `timestamp_ms` starting at first enable.

## 6. Standalone sonar (no full stack)

Drive the boat any way you like (teleop, existing missions) and run just
the sonar + recorder:

```bash
ros2 launch blueboat_sss_sim sss_sim_launch.py mission_dir:=$HOME/runs/r1
```

`auto_ping:=false` if you want to enable pinging manually, exactly as an
operator would on the real system.

## 7. Batch dataset generation

`randomize: true` in the mission YAML draws seed, litter density and
pattern parameters per bundle:

```bash
for i in $(seq 1 20); do
  ros2 run blueboat_sss_sim generate_mission --config my_mission.yaml \
      --out ~/runs/batch/$i --seed $i
done
```

Each bundle is fully reproducible from its seed; `mission_snapshot.yaml`
records the resolved configuration. Point all recorders at one shared
`dataset_dir` to accumulate a single training set (tile names are prefixed
by `run_name`, so set it per run).

## 8. Verifying an install

`python3 -m test.smoke_test` from the package source root runs the whole
ROS-free pipeline (world → render → encode → decode → label → export) and
writes a visual waterfall preview to `/tmp/blueboat_sss_smoke/`.
