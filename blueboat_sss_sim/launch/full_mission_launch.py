"""End-to-end simulated mission.

Starts, in one command:
  1. Gazebo on the mission bundle's generated ``world.sdf``, with the
     ``/clock`` and ``/ocean_current`` bridges and the robot spawn
     (``blueboat_description``'s ``upload_rov_launch.py``);
  2. the existing control stack (``simulation_interface``,
     ``master_control``, ``path_publisher``) unchanged -- with
     ``path_publisher``'s ``total_time`` sized automatically from the
     mission bundle's trajectory metadata, so RViz shows (and any
     consumer tracks) the *whole* mission, not the first 120 s;
  3. this package's mission path service (replacing ``path_generation.py``);
  4. the simulated SSS (``sss_sim_launch.py``).

The simulator's output stops at SSS data; the dataset recorder is a
downstream tool and is not started here.

Arguments
---------
mission_dir       mission bundle directory (required)
controller_type   controller for master_control        (default 'PID')
use_stock_world   load blueboat_description's stock world.sdf via its
                  world_launch.py instead of the bundle's  (default false)
quiet             filter known-noisy startup lines      (default true)

range_start_mm / range_length_mm / msec_per_ping / gain_index /
num_results / pulse_len_percent
                  per-run acquisition overrides, forwarded to
                  ``sss_sim_launch.py``. Empty (the default) means "use the
                  mission bundle's frozen ``sonar.yaml`` value" -- the bundle
                  is the authority for what a run acquired at (NC #10).

World loading
-------------
``blueboat_description``'s ``world_launch.py`` hard-codes the stock
``world.sdf`` and declares no ``world`` argument, so it cannot load a
bundle world. This launch file therefore does that bring-up itself:
``sl.gz_launch`` on the bundle's ``world.sdf`` -- which also registers the
world name (``generated_ocean``) that the model bridges resolve against --
then the same ``/clock`` and ``/ocean_current`` bridges and the same
``upload_rov_launch.py`` include that the stock path provides.

Verbosity policy (lesson learned)
---------------------------------
An earlier revision raised the *global* launch log level to WARNING.
That silenced every process's stdout/rosout on screen -- including
master_control, path_publisher and the SSS nodes -- because launch
routes all child output through its own INFO-level loggers. Reverted.

The current approach attaches a ``logging.Filter`` to launch's screen
handler that drops only known-noisy lines by content (process
started/finished bookkeeping, gz bridge creation spam, ``create``'s
world-name polling, the one-shot ping-enable publisher chatter),
wherever they originate -- including inside included launch files.
Everything else stays on screen: first-party INFO from master_control,
simulation_interface, path_publisher, path_generation and
side_scan_sonar, every ERROR, and Python tracebacks. The one deliberate
exception is a WARN -- kdl_parser's "root link ... has an inertia"
line from robot_state_publisher, which is on the list by name.
"""

import logging
import os

from launch.actions import SetEnvironmentVariable
from simple_launch import GazeboBridge, SimpleLauncher

# ---------------------------------------------------------------------------
# Targeted screen-noise filter (see module docstring).
# ---------------------------------------------------------------------------
_NOISY_SNIPPETS = (
    "process started with pid",
    "process has finished cleanly",
    "Creating GZ->ROS Bridge",
    "Creating ROS->GZ Bridge",
    "Requesting list of world names",
    "publisher: beginning loop",
    "publishing #",
    "signal_handler(signum",
    "The root link blueboat/base_link has an inertia specified in the URDF",
    "Waiting messages on topic [robot_description]",
    "Entity creation successful",
    "Passing message from ROS",
)


class _ScreenNoiseFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:  # True = keep
        try:
            msg = record.getMessage()
        except Exception:
            return True
        return not any(s in msg for s in _NOISY_SNIPPETS)


def _install_quiet_filter() -> None:
    try:
        import launch.logging as launch_logging
        handler = launch_logging.launch_config.get_screen_handler()
        if not any(isinstance(f, _ScreenNoiseFilter) for f in handler.filters):
            handler.addFilter(_ScreenNoiseFilter())
    except Exception:
        pass  # cosmetic only -- never break the launch over it


sl = SimpleLauncher(use_sim_time=True)

sl.declare_arg("mission_dir", default_value="")
sl.declare_arg("controller_type", default_value="PID")
sl.declare_arg("use_stock_world", default_value=False)
sl.declare_arg("quiet", default_value=True)

# The six run-dependent acquisition parameters, forwarded to sss_sim_launch.
# Empty = use the mission bundle's frozen sonar.yaml value (NC #10).
ACQUISITION_ARGS = ("range_start_mm", "range_length_mm", "msec_per_ping",
                    "gain_index", "num_results", "pulse_len_percent")
for _a in ACQUISITION_ARGS:
    sl.declare_arg(_a, default_value="",
                   description=f"{_a} override; empty = use the mission "
                               "bundle's frozen sonar.yaml value")


def _mission_total_time(mission_dir: str) -> float:
    """Whole-mission time window for path_publisher: stored duration
    (written by generate_mission) x 10% controller margin + 30 s."""
    import yaml
    try:
        with open(f"{mission_dir}/trajectory.yaml", "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        duration = float(doc.get("duration_s", 0.0))
        if duration <= 0.0:  # older bundle without metadata: recompute
            import numpy as np
            wps = np.asarray(doc["waypoints"], dtype=float)
            seg = np.diff(wps, axis=0)
            duration = float(np.hypot(seg[:, 0], seg[:, 1]).sum()
                             / max(float(doc.get("speed", 1.0)), 1e-6))
        return duration * 1.1 + 30.0
    except Exception:
        return 600.0  # safe fallback


def launch_setup():
    mission_dir = sl.arg("mission_dir")
    if not mission_dir:
        raise RuntimeError("launch argument 'mission_dir' is required")
    world_file = f"{mission_dir}/world.sdf"
    quiet = bool(sl.arg("quiet"))
    if quiet:
        _install_quiet_filter()

    total_time = _mission_total_time(mission_dir)

    # 1. Gazebo + robot.
    if sl.arg("use_stock_world"):
        sl.include("blueboat_description", "world_launch.py",
                   launch_arguments={"sliders": False})
    else:
        # The bundle's world.sdf references seabed.stl relatively; put the
        # bundle on the resource path so the mesh resolves wherever Gazebo
        # is started from.
        sl.add_action(SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.pathsep.join(filter(None, [
                mission_dir, os.environ.get("GZ_SIM_RESOURCE_PATH", "")]))))
        # gz_launch reads <world name> out of the SDF and registers it, which
        # is what makes GazeboBridge.model_prefix() resolve; a bare Gazebo
        # process would leave the bridges without a world name.
        sl.gz_launch(world_file, "-r")
        sl.create_gz_bridge([
            GazeboBridge.clock(),
            GazeboBridge("/ocean_current", "/current",
                         "geometry_msgs/Vector3", GazeboBridge.ros2gz)])
        sl.include("blueboat_description", "upload_rov_launch.py",
                   launch_arguments={"sliders": False, "thr": "thrusters_ur"})

    # 2. Existing control stack, untouched (first-party: stays on screen).
    #    path_publisher's window now covers the whole mission.
    sl.node("blueboat_control", "simulation_interface.py")
    sl.node("blueboat_control", "path_publisher.py", output="screen",
            parameters={"total_time": total_time})
    sl.node("blueboat_control", "master_control.py", output="screen",
            parameters={"controller_type": sl.arg("controller_type"),
                        "simulation": True})

    # 3. Mission trajectory served on the same RequestPath interface.
    #    Also latches the complete path on /mission/full_path for RViz.
    sl.node("blueboat_sss_sim", "sss_path_generation", output="screen",
            parameters={"trajectory_file": f"{mission_dir}/trajectory.yaml"})

    # 4. Simulated SSS (no dataset recorder -- downstream stage).
    sl.include("blueboat_sss_sim", "sss_sim_launch.py",
               launch_arguments={"mission_dir": mission_dir,
                                 "with_recorder": False,
                                 "with_mission_path": False,
                                 "quiet": quiet,
                                 **{a: sl.arg(a) for a in ACQUISITION_ARGS}})

    return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function=launch_setup)
