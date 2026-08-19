"""Launch the simulated Side Scan Sonar stack.

Composable with the existing simulator launch: start your simulator as
usual, then include (or run alongside) this launch with the mission bundle
directory. Nothing in the existing launch files needs modification.

The simulator's output stops at SSS data (profiles + raw + ground-truth
contacts) per the target pipeline; the YOLO dataset recorder is a
downstream AI-stage tool and is therefore **off by default** (task 3).
Enable it explicitly with ``with_recorder:=true`` only when you want the
sim itself to write a dataset.

Arguments
---------
mission_dir     mission bundle directory (from `generate_mission`); provides
                scene, sonar config and trajectory. Required.
with_recorder   also run the YOLO dataset recorder            (default false)
with_mavros_shim  republish heading/imu under MAVROS names    (default false)
with_mission_path run the RequestPath mission service         (default false)
                  (leave false if another path_generation runs;
                  the two serve the same service name)
dataset_dir     recorder output (default <mission_dir>/dataset)
auto_ping       enable pinging automatically at startup       (default true)
quiet           route helper one-shot output to log           (default true)
"""

from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time=True)

sl.declare_arg("mission_dir", default_value="")
sl.declare_arg("with_recorder", default_value=False)
sl.declare_arg("with_mavros_shim", default_value=False)
sl.declare_arg("with_mission_path", default_value=False)
sl.declare_arg("dataset_dir", default_value="")
sl.declare_arg("auto_ping", default_value=True)
sl.declare_arg("quiet", default_value=True)


def launch_setup():
    mission_dir = sl.arg("mission_dir")
    if not mission_dir:
        raise RuntimeError("launch argument 'mission_dir' is required")
    dataset_dir = sl.arg("dataset_dir") or f"{mission_dir}/dataset"
    quiet = bool(sl.arg("quiet"))

    # Simulated sonar (drop-in replacement for the real sss_node.py).
    sl.node("blueboat_sss_sim", "sss_sim_node", output="screen",
            parameters={
                "scene_dir": mission_dir,
                "sonar_config": f"{mission_dir}/sonar.yaml",
                "odom_topic": "/blueboat/odom",
                "publish_ground_truth": True,
            })

    if sl.arg("with_recorder"):
        sl.node("blueboat_sss_sim", "dataset_recorder_node", output="screen",
                parameters={"output_dir": dataset_dir})

    if sl.arg("with_mavros_shim"):
        sl.node("blueboat_sss_sim", "mavros_shim_node")

    if sl.arg("with_mission_path"):
        sl.node("blueboat_sss_sim", "sss_path_generation", output="screen",
                parameters={"trajectory_file": f"{mission_dir}/trajectory.yaml"})

    if sl.arg("auto_ping"):
        # One-shot ping enable a few seconds after the graph is up (same
        # command an operator issues on the real system). Its publisher
        # chatter ("beginning loop", "publishing #1: ...") goes to the log
        # file when quiet (task 1).
        from launch.actions import ExecuteProcess, TimerAction
        sl.add_action(TimerAction(period=5.0, actions=[ExecuteProcess(
            cmd=["ros2", "topic", "pub", "--once",
                 "/side_scan_sonar/ping/enable", "std_msgs/msg/Bool",
                 "data: true"],
            output="log" if quiet else "screen")]))

    return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function=launch_setup)
