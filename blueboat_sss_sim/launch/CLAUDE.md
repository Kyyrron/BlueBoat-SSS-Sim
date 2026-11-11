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
