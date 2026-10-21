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
     MCS/Control, and the deployed-trajectory interface is exactly a
     real-robot run's.

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
note              free-text tag put into the run's position-log file name,
                  ``{date}-{note}-poslog.csv`` -- exactly the argument
                  ``Sim_launch.py`` and the real-robot
                  ``BlueBoat_launch.py`` carry, forwarded to
                  ``simulation_interface``        (default 'sim')
data_dir          root for that CSV and the controller .npy; empty resolves
                  to $BLUEBOAT_DATA_DIR, else the sourced workspace
                                                  (default '')
use_stock_world   load blueboat_description's stock world.sdf via its
                  world_launch.py instead of the world's  (default false)
quiet             filter known-noisy startup lines      (default true)
sim_origin_lat / sim_origin_lon
                  MAVROS-shim origin override; empty = the world's anchor
with_mavros_shim  forwarded to sss_sim_launch.py        (default true)
headless          run the Gazebo server only (``gz sim -s``), no GUI --
                  scripted runs such as ``test/gazebo_sea_test.py``
                                                  (default false)
gain_profile      forwarded to ``master_control`` only when set: ``field``
                  flies the real-boat tuning branch in simulation, ``sim``
                  the simulation branch (BlueBoat-Control's parameter,
                  2026-10-21)                     (default '' = not passed)
control_params_file
                  a ROS 2 parameters YAML appended to ``master_control``'s
                  parameters (e.g. ``mpc_Q_diag``) -- scripted campaigns
                  such as BlueBoat-Experiments    (default '' = none)
with_sonar        start the simulated sonar node (and its auto ping);
                  forwarded to ``sss_sim_launch.py``. ``false`` keeps the
                  MAVROS shim and drops the renderer's CPU cost
                                                  (default true)

sea_current / sea_current_from_deg / sea_waves / sea_waves_from_deg /
sea_schedule / sea_seed
                  the sea state (``sea_state_launch.py``): current and
                  wave presets from ``config/sea_states.yaml`` with their
                  compass FROM directions, or a timeline YAML. Defaults
                  ``none`` / ``calm`` -- zero water velocity and no wave
                  forcing; since 2026-10-09 the hull wrench still carries
                  the catamaran hydrostatics (~0 at rest at the trim; one
                  hull in every run). The node is always started, so the
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

# ROS is needed to *launch*, not to read the resolvers below. Importing
# `simple_launch` pulls in `launch` -> `lark`, which the workspace venv does
# not carry, so a plain `python3 -m test.smoke_test` used to die here before
# reporting a single check (2026-09-18). The import is therefore optional:
# under ROS every line behaves exactly as before, and without it the module
# still yields its constants and its pure resolvers -- which is what
# `CLAUDE.md` means by "module-scope, launch-free resolvers the smoke test
# exercises directly".
try:
    from launch.actions import SetEnvironmentVariable
    from simple_launch import GazeboBridge, SimpleLauncher
except ImportError:                       # no ROS on this interpreter
    SetEnvironmentVariable = GazeboBridge = SimpleLauncher = None

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


sl = SimpleLauncher(use_sim_time=True) if SimpleLauncher is not None else None


def _declare(name, **kw) -> None:
    """Declare a launch argument under ROS; a no-op without it, so the
    constants and the pure helpers below stay importable either way."""
    if sl is not None:
        sl.declare_arg(name, **kw)


_declare("world_dir", default_value="")
_declare("with_control", default_value=False)
_declare("trajectory_file", default_value="")
_declare("controller_type", default_value="PID")
# Run-log identity, same names and meaning as Sim_launch.py / BlueBoat_launch.py
# (simulation_interface names {date}-{note}-poslog.csv from 'note').
_declare("note", default_value="sim")
_declare("data_dir", default_value="")
_declare("use_stock_world", default_value=False)
_declare("quiet", default_value=True)
_declare("with_mavros_shim", default_value=True)
_declare("sim_origin_lat", default_value="")
_declare("sim_origin_lon", default_value="")
_declare("headless", default_value=False)
# Additive (2026-10-21): scripted-campaign pass-throughs; the defaults leave
# every node's parameters exactly as before.
_declare("gain_profile", default_value="")
_declare("control_params_file", default_value="")
_declare("with_sonar", default_value=True)

# The six run-dependent acquisition parameters, forwarded to sss_sim_launch.
ACQUISITION_ARGS = ("range_start_mm", "range_length_mm", "msec_per_ping",
                    "gain_index", "num_results", "pulse_len_percent")
for _a in ACQUISITION_ARGS:
    _declare(_a, default_value="",
             description=f"{_a} override; empty = use the resolved "
                         "sonar profile's value")

# The sea state, forwarded to sea_state_launch.py (same names).
SEA_ARGS = {"sea_current": "none", "sea_current_from_deg": 0.0,
            "sea_waves": "calm", "sea_waves_from_deg": 0.0,
            "sea_schedule": "", "sea_seed": 0}
for _a, _d in SEA_ARGS.items():
    _declare(_a, default_value=_d)

#: The stock world of blueboat_description (use_stock_world).
STOCK_WORLD_NAME = "ocean"


def gz_args_for(headless) -> str:
    """Gazebo command-line flags: run immediately; server only when
    ``headless`` (launch-free, exercised by the smoke test)."""
    return "-r -s" if str(headless).strip().lower() in ("1", "true", "yes") else "-r"


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


def _trajectory_duration(trajectory_file: str) -> float:
    """Duration of one trajectory file, from either schema:
    ``points: [[t, x, y, yaw], ...]`` (MCS) or ``waypoints`` + ``speed``
    (legacy); 0.0 when the file is missing or says nothing."""
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
        return max(duration, 0.0)
    except Exception:
        return 0.0


def trajectory_total_time(trajectory_file: str) -> float:
    """Whole-mission time window for path_publisher, x 10% controller
    margin + 30 s. Module scope and launch-free so the smoke test can
    exercise it.

    path_publisher builds its request window ONCE, at startup, so the
    duration has to be right at launch time -- and an MCS deployed file
    is not there then: MCS starts this launch, *then* deletes
    ``.deployed/<name>.yaml`` and rewrites it only after its GPS fit
    converges. Read at launch, the window fell back to 600 s and RViz drew
    the first 600 s of the 1317 s ``plage_bathymetry2`` loop (2026-09-28).
    A file under ``.deployed/`` therefore takes its duration from the
    designer file it is deployed from, ``<trajectories>/<name>.yaml``
    (MCS's ``deployed_path(src.parent, src.stem)``), which always exists
    and whose sample times ``deploy_mission`` copies unchanged; then from
    the file itself; then the 600 s fallback."""
    from pathlib import Path
    f = Path(trajectory_file)
    candidates = [f]
    if f.parent.name == ".deployed":
        candidates.insert(0, f.parent.parent / f.name)
    for cand in candidates:
        duration = _trajectory_duration(str(cand))
        if duration > 0.0:
            return duration * 1.1 + 30.0
    return 600.0  # safe fallback (file not written yet: it is watched)


def master_control_parameters(base: dict, gain_profile="", params_file=""):
    """``master_control``'s parameters: ``base`` unchanged when neither
    pass-through is set (the pre-2026-10-21 call, exactly); otherwise
    ``gain_profile`` joins the dict and a parameters file follows it.
    Module scope and launch-free so the smoke test can exercise it."""
    gp = str(gain_profile or "").strip()
    pf = str(params_file or "").strip()
    if not gp and not pf:
        return base
    params = [dict(base, gain_profile=gp) if gp else dict(base)]
    if pf:
        params.append(pf)
    return params


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
        sl.gz_launch(world_file, gz_args_for(sl.arg("headless")))
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
        sl.node("blueboat_control", "simulation_interface.py",
                parameters={"controller_type": sl.arg("controller_type"),
                            "note": sl.arg("note"),
                            "data_dir": sl.arg("data_dir")})
        sl.node("blueboat_control", "path_generation.py", output="screen",
                parameters={"trajectory": f"from_yaml:{trajectory_file}"})
        sl.node("blueboat_control", "path_publisher.py", output="screen",
                parameters={"total_time":
                            trajectory_total_time(trajectory_file)})
        sl.node("blueboat_control", "master_control.py", output="screen",
                parameters=master_control_parameters(
                    {"controller_type": sl.arg("controller_type"),
                     "simulation": True,
                     "data_dir": sl.arg("data_dir")},
                    sl.arg("gain_profile"), sl.arg("control_params_file")))

    # 3. Simulated SSS + MAVROS shim, anchored at the world's GPS anchor.
    sl.include("blueboat_sss_sim", "sss_sim_launch.py",
               launch_arguments={"world_dir": world_dir,
                                 "quiet": quiet,
                                 "with_mavros_shim": sl.arg("with_mavros_shim"),
                                 "sim_origin_lat": sl.arg("sim_origin_lat"),
                                 "sim_origin_lon": sl.arg("sim_origin_lon"),
                                 "with_sonar": sl.arg("with_sonar"),
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


generate_launch_description = (
    sl.launch_description(opaque_function=launch_setup)
    if sl is not None else None)
