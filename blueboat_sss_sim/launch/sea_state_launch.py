"""Sea state (current + waves) for a running Gazebo BlueBoat simulation.

Starts ``sea_state_node`` and the two extra ros -> gz bridges it needs for
the wave wrench (``/world/<w>/wrench`` and ``/world/<w>/wrench/clear``).
Included by ``full_mission_launch.py``; also runs **standalone next to
BlueBoat-Control's own ``Sim_launch.py``** (stock world ``ocean``):

    ros2 launch blueboat_sss_sim sea_state_launch.py world_name:=ocean \\
        sea_current:=strong sea_current_from_deg:=90 \\
        sea_waves:=slight sea_waves_from_deg:=45

The node injects gz-sim's ``ApplyLinkWrench`` system into the running
world when it is absent (the stock world, older world folders), so no
neighbour package and no saved world is modified.

Arguments
---------
sea_current / sea_current_from_deg   preset (none | weak | moderate |
                  strong | very_strong) and compass FROM direction
sea_waves / sea_waves_from_deg       preset (calm | rippled | smooth |
                  slight | rough) and compass FROM direction
sea_schedule      timeline YAML (``blueboat_sea_schedule/1``); overrides
                  the four above                             (default "")
sea_seed          wave phases + current fluctuations (0 = schedule's / 1)
world_name        gz world name the bridges are created for. Default
                  ``ocean`` (the stock world); ``full_mission_launch.py``
                  passes the generated world's name. Empty = the node
                  discovers it and runs its own bridge.
world_dir         world folder (mean depth, physics step)   (default "")
with_current_bridge   also bridge ROS /current -> gz /ocean_current;
                  default false because both simulator launch paths
                  already do (two bridges would double-publish)
rate_hz           publish rate = physics rate                (default 100)
quiet             unused here; accepted for symmetry with the siblings

``/current`` (``geometry_msgs/Vector3``) is bridged by
``blueboat_description/launch/world_launch.py`` and by
``full_mission_launch.py``; the ``Hydrodynamics`` plugin on the boat
subscribes gz ``/ocean_current`` unconditionally.
"""

# ROS is needed to *launch*, not to import this module (see the same guard in
# sss_sim_launch.py). `wrench_bridges` below does need it -- it returns
# GazeboBridge objects -- and says so rather than failing obscurely.
try:
    from simple_launch import GazeboBridge, SimpleLauncher
except ImportError:                       # no ROS on this interpreter
    GazeboBridge = SimpleLauncher = None

sl = SimpleLauncher(use_sim_time=True) if SimpleLauncher is not None else None

if sl is not None:
    sl.declare_arg("sea_current", default_value="none")
    sl.declare_arg("sea_current_from_deg", default_value=0.0)
    sl.declare_arg("sea_waves", default_value="calm")
    sl.declare_arg("sea_waves_from_deg", default_value=0.0)
    sl.declare_arg("sea_schedule", default_value="")
    sl.declare_arg("sea_seed", default_value=0)
    sl.declare_arg("world_name", default_value="ocean")
    sl.declare_arg("world_dir", default_value="")
    sl.declare_arg("with_current_bridge", default_value=False)
    sl.declare_arg("rate_hz", default_value=100.0)
    sl.declare_arg("quiet", default_value=True)


def wrench_bridges(world_name: str) -> list:
    """The two ros -> gz bridges of the wave wrench. Module scope and
    launch-free (a list of ``GazeboBridge``) so the smoke test can pin the
    message pairs -- ``EntityWrench`` is not in simple_launch's ``msg_map``
    and needs the explicit ``gz_msg``; without it the bridge object is
    silently broken. This one *does* need ROS on the path (it builds
    GazeboBridge objects); it says so rather than failing on a None."""
    if GazeboBridge is None:
        raise RuntimeError(
            "wrench_bridges() needs simple_launch (ROS) on this interpreter; "
            "the rest of this module imports without it")
    return [
        GazeboBridge(f"/world/{world_name}/wrench", f"/world/{world_name}/wrench",
                     "ros_gz_interfaces/EntityWrench", GazeboBridge.ros2gz,
                     gz_msg="gz.msgs.EntityWrench"),
        GazeboBridge(f"/world/{world_name}/wrench/clear",
                     f"/world/{world_name}/wrench/clear",
                     "ros_gz_interfaces/Entity", GazeboBridge.ros2gz,
                     gz_msg="gz.msgs.Entity"),
    ]


#: The boat's 100 Hz Gazebo pose, stamped (2026-10-09): what the hull
#: wrench and the sonar use instead of the 20 Hz odometry. Simulation-only,
#: additive; the source is blueboat_description's own PosePublisher.
BOAT_POSE_TOPIC = "/sim/sea_state/boat_pose"


def pose_bridge(model: str = "blueboat"):
    """gz ``/model/<model>/pose`` (``gz.msgs.Pose``, every physics step) ->
    ROS ``BOAT_POSE_TOPIC`` as ``geometry_msgs/TransformStamped``, which
    keeps the sim-time stamp and the child frame (the hull is told apart
    from any nested model). Needs ROS on the path, like wrench_bridges."""
    if GazeboBridge is None:
        raise RuntimeError("pose_bridge() needs simple_launch (ROS) on this interpreter")
    return GazeboBridge(f"/model/{model}/pose", BOAT_POSE_TOPIC,
                        "geometry_msgs/TransformStamped", GazeboBridge.gz2ros,
                        gz_msg="gz.msgs.Pose")


def launch_setup():
    world_name = str(sl.arg("world_name"))
    bridges = []
    if world_name:
        bridges += wrench_bridges(world_name)
        bridges.append(pose_bridge("blueboat"))
    if sl.arg("with_current_bridge"):
        bridges.append(GazeboBridge("/ocean_current", "/current",
                                    "geometry_msgs/Vector3", GazeboBridge.ros2gz))
    if bridges:
        sl.create_gz_bridge(bridges, name="sea_state_bridge")

    sl.node("blueboat_sss_sim", "sea_state_node", output="screen",
            parameters={
                "current": str(sl.arg("sea_current")),
                "current_from_deg": float(sl.arg("sea_current_from_deg")),
                "waves": str(sl.arg("sea_waves")),
                "waves_from_deg": float(sl.arg("sea_waves_from_deg")),
                "schedule_file": str(sl.arg("sea_schedule")),
                "seed": int(sl.arg("sea_seed")),
                "world_name": world_name,
                "world_dir": str(sl.arg("world_dir")),
                "rate_hz": float(sl.arg("rate_hz")),
            })
    return sl.launch_description()


generate_launch_description = (
    sl.launch_description(opaque_function=launch_setup)
    if sl is not None else None)
