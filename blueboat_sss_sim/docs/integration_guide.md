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

`full_mission_launch.py` starts Gazebo on the **bundle's** `world.sdf`
itself, because `blueboat_description/world_launch.py` hard-codes the
stock world and declares no `world` argument. It reproduces that launch
file's three responsibilities against the bundle:

* `sl.gz_launch(<bundle>/world.sdf, "-r")` — which also registers the
  world name (`generated_ocean`) that the model bridges resolve against;
* the `/clock` and `/ocean_current` bridges;
* `blueboat_description/upload_rov_launch.py`, which spawns the robot and
  bridges `/blueboat/odom`, `pose_gt`, `joint_states` and
  `cmd_thruster{1,2}`.

No file in `blueboat_description` is modified. Pass
`use_stock_world:=true` to load the stock description world instead — the
boat spawns and drives, but the seabed and objects the sonar renders
against are not in the Gazebo scene.

The generated world uses `gz-sim-*` plugin names (Garden/Harmonic), which
is the `gazebo_plugin_prefix: gz` default. Set `ignition` in the mission
YAML for a Fortress machine.

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

## 8. Measuring a mission

Detection metrics come out of the renderer's own ground truth, with no ROS
and no field session:

```bash
# The bundle's intended path -- reproducible from the seed alone
ros2 run blueboat_sss_sim mission_metrics --bundle ~/runs/r3 --out ~/metrics/r3

# The path a real run actually tracked, from its recorded .svlog
ros2 run blueboat_sss_sim mission_metrics --bundle ~/runs/r3 \
    --source svlog --input ~/sessions/2026-08-30/run.svlog --out ~/metrics/r3_real

# What the node actually published (a dump of ground_truth/contacts;
# that topic carries no pose, so aspect is reported as unavailable)
ros2 run blueboat_sss_sim mission_metrics --bundle ~/runs/r3 \
    --source jsonl --input contacts.jsonl --out ~/metrics/r3_pub
```

Writes `metrics.json` + `metrics.md` into `--out`; the bundle and the
recording are only ever read. Every object in `scene_manifest.yaml` is
accounted for as detected, ensonified-but-below-criterion, or never
ensonified, and the criterion in force (`geometric` / `resolved` /
`shadowed`) is named in both files. The `content_digest` covers everything
except provenance, so the same bundle regenerated from its seed into a
different directory compares equal.

The `.svlog` path needs the position track the recording carries as mavlink
`LOCAL_POSITION_NED` (id 150). In simulation that comes from
`mavros_shim_node`, so launch with `with_mavros_shim:=true` if the run is
going to be measured this way.

## 9. Verifying an install

`python3 -m test.smoke_test` from the package source root runs the whole
ROS-free pipeline (world → render → encode → decode → label → export) and
writes a visual waterfall preview to `/tmp/blueboat_sss_smoke/`.
