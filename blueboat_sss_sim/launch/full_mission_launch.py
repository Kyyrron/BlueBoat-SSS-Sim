"""Dev end-to-end simulation on a world folder.

Starts, in one command:
  1. Gazebo on the world folder's ``world.sdf``, with the ``/clock`` and
     ``/ocean_current`` bridges and the robot spawn
     (``blueboat_description``'s ``upload_rov_launch.py``);
  2. the simulated SSS + MAVROS shim (``sss_sim_launch.py``), the shim
     anchored at the world's own GPS anchor;
  3. optionally (``with_control:=true``) the existing control stack
     unchanged -- ``simulation_interface``, ``master_control``,
     ``path_publisher`` and **``blueboat_control``'s own
     ``path_generation``** driven by ``trajectory:=from_yaml:<file>``.
     This package no longer serves ``/path_request``: missions belong to
     MCS/Control, and the deployed-trajectory interface is exactly the
     real boat's.

This launch is a development convenience for exercising a world
end-to-end without the Mission Control Station; operational missions are
launched by MCS.

Arguments
---------
world_dir         world folder (required)
with_control      also start the control stack          (default false)
trajectory_file   local-frame trajectory YAML for path_generation's
                  from_yaml branch (required with with_control; an MCS
                  ``.deployed/<name>.yaml`` works as-is). The file is
                  watched: it may appear after launch.
controller_type   controller for master_control        (default 'PID')
use_stock_world   load blueboat_description's stock world.sdf via its
                  world_launch.py instead of the world's  (default false)
quiet             filter known-noisy startup lines      (default true)
sim_origin_lat / sim_origin_lon
                  MAVROS-shim origin override; empty = the world's anchor
with_mavros_shim  forwarded to sss_sim_launch.py        (default true)

sea_current / sea_current_from_deg / sea_waves / sea_waves_from_deg /
sea_schedule / sea_seed
                  the sea state (``sea_state_launch.py``): current and
                  wave presets from ``config/sea_states.yaml`` with their
                  compass FROM directions, or a timeline YAML. Defaults
                  ``none`` / ``calm`` -- a strict no-op (zero water
                  velocity, no wrench), so an unchanged command line runs
                  exactly as before. The node is always started, so the
                  situation can be modified live from MCS.

range_start_mm / range_length_mm / msec_per_ping / gain_index /
num_results / pulse_len_percent
                  per-run acquisition overrides, forwarded to
                  ``sss_sim_launch.py``. Empty (the default) means "use
                  the resolved sonar profile's value".

World loading
-------------
``blueboat_description``'s ``world_launch.py`` hard-codes the stock
``world.sdf`` and declares no ``world`` argument, so it cannot load a
generated world. This launch file therefore does that bring-up itself:
``sl.gz_launch`` on the world's ``world.sdf`` -- which also registers the
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

sl.declare_arg("world_dir", default_value="")
sl.declare_arg("with_control", default_value=False)
sl.declare_arg("trajectory_file", default_value="")
sl.declare_arg("controller_type", default_value="PID")
sl.declare_arg("use_stock_world", default_value=False)
sl.declare_arg("quiet", default_value=True)
sl.declare_arg("with_mavros_shim", default_value=True)
sl.declare_arg("sim_origin_lat", default_value="")
sl.declare_arg("sim_origin_lon", default_value="")

# The six run-dependent acquisition parameters, forwarded to sss_sim_launch.
ACQUISITION_ARGS = ("range_start_mm", "range_length_mm", "msec_per_ping",
                    "gain_index", "num_results", "pulse_len_percent")
for _a in ACQUISITION_ARGS:
    sl.declare_arg(_a, default_value="",
                   description=f"{_a} override; empty = use the resolved "
                               "sonar profile's value")

# The sea state, forwarded to sea_state_launch.py (same names).
SEA_ARGS = {"sea_current": "none", "sea_current_from_deg": 0.0,
            "sea_waves": "calm", "sea_waves_from_deg": 0.0,
            "sea_schedule": "", "sea_seed": 0}
for _a, _d in SEA_ARGS.items():
    sl.declare_arg(_a, default_value=_d)

#: The stock world of blueboat_description (use_stock_world).
STOCK_WORLD_NAME = "ocean"


def world_name_from_sdf(world_file: str, default: str = STOCK_WORLD_NAME) -> str:
    """The ``<world name="...">`` of an SDF file -- the name gz-sim
    registers its ``/world/<name>/...`` topics and services under, which
    the sea-state wrench bridges are created for. Module scope and
    launch-free so the smoke test can exercise it."""
    import re
    try:
        with open(world_file, "r", encoding="utf-8") as f:
            m = re.search(r'<world\s+name\s*=\s*"([^"]+)"', f.read())
        return m.group(1) if m else default
    except OSError:
        return default


def trajectory_total_time(trajectory_file: str) -> float:
    """Whole-mission time window for path_publisher, from either schema:
    ``points: [[t, x, y, yaw], ...]`` (MCS) or ``waypoints`` + ``speed``
    (legacy), x 10% controller margin + 30 s. Module scope and launch-free
    so the smoke test can exercise it."""
    import yaml
    try:
        with open(trajectory_file, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        duration = float(doc.get("duration_s", 0.0))
        if duration <= 0.0 and "points" in doc:
            pts = doc["points"]
            duration = float(pts[-1][0]) - float(pts[0][0])
        if duration <= 0.0 and "waypoints" in doc:
            import numpy as np
            wps = np.asarray(doc["waypoints"], dtype=float)
            seg = np.diff(wps, axis=0)
            duration = float(np.hypot(seg[:, 0], seg[:, 1]).sum()
                             / max(float(doc.get("speed", 1.0)), 1e-6))
        if duration > 0.0:
            return duration * 1.1 + 30.0
    except Exception:
        pass
    return 600.0  # safe fallback (file not written yet: it is watched)


def launch_setup():
    world_dir = sl.arg("world_dir")
    if not world_dir:
        raise RuntimeError("launch argument 'world_dir' is required")
    world_file = f"{world_dir}/world.sdf"
    quiet = bool(sl.arg("quiet"))
    if quiet:
        _install_quiet_filter()

    # 1. Gazebo + robot.
    if sl.arg("use_stock_world"):
        sl.include("blueboat_description", "world_launch.py",
                   launch_arguments={"sliders": False})
    else:
        # The world.sdf references seabed.stl relatively; put the folder on
        # the resource path so the mesh resolves wherever Gazebo starts.
        sl.add_action(SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.pathsep.join(filter(None, [
                world_dir, os.environ.get("GZ_SIM_RESOURCE_PATH", "")]))))
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

    # 2. Optional control stack, untouched -- including blueboat_control's
    #    OWN path_generation on its from_yaml branch (the real-boat
    #    deployed-trajectory interface; the file is watched, so it may be
    #    deployed after launch).
    if sl.arg("with_control"):
        trajectory_file = sl.arg("trajectory_file")
        if not trajectory_file:
            raise RuntimeError(
                "with_control:=true needs trajectory_file:=<local-frame "
                "trajectory YAML> (an MCS .deployed/<name>.yaml works as-is)")
        sl.node("blueboat_control", "simulation_interface.py")
        sl.node("blueboat_control", "path_generation.py", output="screen",
                parameters={"trajectory": f"from_yaml:{trajectory_file}"})
        sl.node("blueboat_control", "path_publisher.py", output="screen",
                parameters={"total_time":
                            trajectory_total_time(trajectory_file)})
        sl.node("blueboat_control", "master_control.py", output="screen",
                parameters={"controller_type": sl.arg("controller_type"),
                            "simulation": True})

    # 3. Simulated SSS + MAVROS shim, anchored at the world's GPS anchor.
    sl.include("blueboat_sss_sim", "sss_sim_launch.py",
               launch_arguments={"world_dir": world_dir,
                                 "quiet": quiet,
                                 "with_mavros_shim": sl.arg("with_mavros_shim"),
                                 "sim_origin_lat": sl.arg("sim_origin_lat"),
                                 "sim_origin_lon": sl.arg("sim_origin_lon"),
                                 **{a: sl.arg(a) for a in ACQUISITION_ARGS}})

    # 4. Sea state: current + waves on the hull (docs/sea_state_model.md).
    #    The wrench bridges are created for the world's own name; the
    #    node injects ApplyLinkWrench into the running world if the SDF
    #    predates it (worlds built before 2026-09, the stock world).
    world_name = (STOCK_WORLD_NAME if sl.arg("use_stock_world")
                  else world_name_from_sdf(world_file))
    sl.include("blueboat_sss_sim", "sea_state_launch.py",
               launch_arguments={"world_name": world_name,
                                 "world_dir": world_dir,
                                 "quiet": quiet,
                                 **{a: sl.arg(a) for a in SEA_ARGS}})

    return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function=launch_setup)
