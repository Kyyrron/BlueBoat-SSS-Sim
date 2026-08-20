"""End-to-end simulated mission.

Starts, in one command:
  1. Gazebo on the mission bundle's generated ``world.sdf`` (via
     ``blueboat_description``'s world launch or directly);
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
use_existing_world_launch  include blueboat_description world_launch.py
                  with a `world` argument               (default true)
quiet             filter known-noisy startup lines      (default true)

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
Everything else, notably first-party INFO and all warnings/errors from
any node, stays on screen.
"""

import logging

from simple_launch import SimpleLauncher

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
sl.declare_arg("use_existing_world_launch", default_value=True)
sl.declare_arg("quiet", default_value=True)


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
    if sl.arg("use_existing_world_launch"):
        sl.include("blueboat_description", "world_launch.py",
                   launch_arguments={"sliders": False,
                                     "world": world_file})
    else:
        from launch.actions import ExecuteProcess
        sl.add_action(ExecuteProcess(
            cmd=["gz", "sim", "-r", world_file],
            output="log" if quiet else "screen"))

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
                                 "quiet": quiet})

    return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function=launch_setup)
