# Launch files — `blueboat_sss_sim/launch/`

Moved from the module root `.claude/CLAUDE.md` §3.5 on 2026-11-10.

`full_mission_launch.py` (Gazebo on the world folder + bridges + spawn +
sonar/shim; `headless:=true` runs the Gazebo server only (`gz sim -s`, for
scripted runs); the default `auto` runs it so for the ray-cast too, with the GUI as a
separate respawning `gz sim -g` client (2026-11-10; the Gazebo GUI can segfault at start-up
in gz-transport's discovery `getenv`, which in one process froze an MCS run on 2026-11-09);
Gazebo exiting by itself now stops the launch with its reason; `with_control:=true trajectory_file:=<yaml>` adds the unmodified
control stack with **`blueboat_control`'s own `path_generation`** on
`trajectory:=from_yaml:<file>` — dev convenience, operational missions are
MCS's), `sss_sim_launch.py`, `sim_world_launch.py`, `sea_state_launch.py`
(also **standalone next to BlueBoat-Control's `Sim_launch.py`**,
`world_name:=ocean` — MCS starts it there for every empty-world run, a calm
sea included, since 2026-10-09). In such a world the node injects
`ApplyLinkWrench` at run time; it must detect the system by
`/world/<w>/wrench/persistent`, **never** by `/wrench`, which the launch's own
bridge advertises (that test skipped the injection until 2026-10-09, so the
stock world's wrenches went nowhere; `sea_state_node.wrench_system_loaded`).

The smoke test calls launch-free module-scope resolvers directly
(`resolve_acquisition`, `resolve_sonar_config`, `resolve_world_anchor`,
`trajectory_total_time`, `world_name_from_sdf`, `wrench_bridges`,
`gz_args_for`, `parse_real_time_factor`, `read_max_step_size`). **The three launch modules the smoke test imports
(`sss_sim_launch`, `full_mission_launch`, `sea_state_launch`) import their
ROS symbols optionally** (`sim_world_launch.py`, never imported offline,
does not) (`try: from
simple_launch import … except ImportError`): module-scope `simple_launch` →
`launch` → `lark` (absent from the workspace venv, the Stop hook's usual
`python3`) would kill the offline gate. Keep it so.

**2026-11-15 (session 03, R-6 / R-7).** `full_mission_launch.py` forwards Control's simulation
configuration exactly as `Sim_launch.py` does: `sim_config` (`field_equivalent` default |
`legacy`) and the overrides `plant`, `odom_profile`, `odom_seed`, `tuning_file` (first in both
nodes' parameter lists; then the launch drops `odom_profile`), `gain_profile`, plus
`control_params_file` (after the launch values, master_control only: the `experiment` branch's
meaning), `thrust_gain`, `odom_base_delay`, `spawn_yaw`, `with_sonar`, `sea_kappa`,
`sea_presets_file`. `SIM_CONFIGS` here mirrors Control's; smoke `[10]` parses Control's file
with `ast` and compares. The resolver `resolve_sim_config` is module-scope and launch-free (the
smoke test calls it); it also sets the odometry topics (R-7) and the run context (C3, passed
`b64:`-encoded because launch_ros reads a string parameter as YAML). `sss_sim_launch.py` gained
`with_sonar`, `odom_topic` (default stays `/blueboat/odom` for bag replays), `true_odom_topic`,
`run_context`; `sea_state_launch.py` `sea_presets_file`, `sea_plant`, `sea_kappa`, the two odometry
topics and `run_context`. The six `SEA_ARGS` are unchanged.

**Start hold (2026-11-16, session 03).** `full_mission_launch.py` starts a generated world's Gazebo **paused**
(`hold_until_sea_armed:=true`, the default: `gz_args_for(..., paused=True)` drops only `-r`) and passes
`release_world_on_arm:=true` to `sea_state_launch.py`; the sea node unpauses the world once it has armed the hull
(`core/sim_hold.StartHold`), on any bootstrap failure, or after `release_deadline_s` (60 s wall) -- a run can never
stay paused because the node failed. Why: since session 04b the identified plant carries the physical CoG (+0.1376),
on which Gazebo's buoyancy box alone capsizes the boat in the ~4 s before the sea node arms (it flipped and righted
itself in every run until then). No physics change; the 20 s start cut of the motion statistics stays (A-07). Not with
`use_stock_world` (Control's `world_launch.py` starts Gazebo running): that path still has the capsize. Never start a
world paused without a sea node in the launch -- nothing would unpause it.
