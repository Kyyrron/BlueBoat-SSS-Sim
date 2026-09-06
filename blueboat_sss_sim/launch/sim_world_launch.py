"""Launch Gazebo alone on a generated world -- for visual inspection of a
world folder before committing to a full run.

Arguments
---------
world_dir     world folder containing world.sdf (required)
gz_cmd        Gazebo executable, must match the installed generation:
              'gz sim' (Garden/Harmonic) or 'ign gazebo' (Fortress).
                                                    (default 'gz sim')
paused        start paused                          (default false)
"""

from simple_launch import SimpleLauncher

sl = SimpleLauncher()

sl.declare_arg("world_dir", default_value="")
sl.declare_arg("gz_cmd", default_value="gz sim")
sl.declare_arg("paused", default_value=False)


def launch_setup():
    world_dir = sl.arg("world_dir")
    if not world_dir:
        raise RuntimeError("launch argument 'world_dir' is required")

    from launch.actions import ExecuteProcess
    cmd = str(sl.arg("gz_cmd")).split()
    if not sl.arg("paused"):
        cmd.append("-r")
    cmd.append(f"{world_dir}/world.sdf")
    sl.add_action(ExecuteProcess(cmd=cmd, output="screen"))

    return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function=launch_setup)
