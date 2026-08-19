"""Launch Gazebo alone on a generated world -- for visual inspection of a
world bundle before committing to a full mission run.

Arguments
---------
mission_dir   mission/world bundle directory containing world.sdf (required)
gz_cmd        Gazebo executable: 'ign gazebo' (Fortress) or 'gz sim'
              (Garden/Harmonic).                    (default 'ign gazebo')
paused        start paused                          (default false)
"""

from simple_launch import SimpleLauncher

sl = SimpleLauncher()

sl.declare_arg("mission_dir", default_value="")
sl.declare_arg("gz_cmd", default_value="gz sim") 
sl.declare_arg("paused", default_value=False)


def launch_setup():
    mission_dir = sl.arg("mission_dir")
    if not mission_dir:
        raise RuntimeError("launch argument 'mission_dir' is required")

    from launch.actions import ExecuteProcess
    cmd = str(sl.arg("gz_cmd")).split()
    if not sl.arg("paused"):
        cmd.append("-r")
    cmd.append(f"{mission_dir}/world.sdf")
    sl.add_action(ExecuteProcess(cmd=cmd, output="screen"))

    return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function=launch_setup)
