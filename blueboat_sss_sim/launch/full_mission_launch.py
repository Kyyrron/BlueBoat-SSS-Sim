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
sonar_config      sonar profile YAML, forwarded to sss_sim_launch.py
                  (default '': the world's frozen copy, else the package
                  default -- the same resolution as there)
sonar_seed        sensor-noise seed, forwarded (default 0 = fresh, logged)
sonar_renderer    ``geometric`` | ``raycast`` | '' (the profile's key),
                  forwarded to sss_sim_launch.py         (default '')
sonar_fouling     ray-cast: fouled fraction of object and wall surfaces,
                  0..1, forwarded       (default -1: the profile's)
real_time_factor  Gazebo's real-time factor, [0.01, 4.0] (default 1.0; the
                  cap was 1.0 until 2026-11-10: with the ray-cast's hold,
                  above x1 only the time between pings runs faster, and the
                  unmodified control stack must keep up -- PID is light,
                  the MPC is not measured there).
                  Below 1 the whole simulation runs slower than real time
                  on one clock -- for the ray-cast sensor, which cannot
                  render a ping inside the 50 ms period (owner, 2026-11-07).
                  Applied as ``gz sim -z <rtf / max_step_size>`` (the world's
                  physics step; 0.01 s -> 10 Hz at x0.1), which overrides
                  the world's own ``<real_time_factor>`` without touching the
                  immutable world folder. Every node runs on /clock (and the
                  control nodes get ``time_source: ros``), so a slowed run
                  records exactly what a real-speed one does.
sonar_hold_sim    ``auto`` | ``when_needed`` | ``always`` | ``off``, forwarded to
                  sss_sim_launch.py with this world's name and the
                  real_time_factor (default ``auto``: the ray-cast pauses
                  Gazebo while a ping cycle overruns, so no ping is skipped
                  whatever a quay costs -- 2026-11-09, ``core/sim_hold.py``)
headless          ``true``: the Gazebo server only (``gz sim -s``), no GUI --
                  scripted runs such as ``test/gazebo_sea_test.py``.
                  ``auto`` (default): for the ray-cast sonar, the server
                  (``-s``) plus the GUI as a **separate, respawning client**
                  (``gz sim -g``, process ``gz_gui``, 2026-11-10); for the
                  geometric one, one ``gz sim`` process as before.
                  ``false``: one process (server + GUI) whatever the sonar.
                  Why the split: the Gazebo GUI can crash at start-up (a
                  segfault in gz-transport's discovery thread, ``getenv``
                  racing the GUI's own environment writes -- upstream,
                  intermittent), and in one process the ``gz sim`` wrapper
                  then stops the server too (2026-11-09: a frozen ray-cast
                  run). As a client, a crash loses only the window, which
                  comes back ~3 s later; the run never notices.
                  If Gazebo exits by itself, this launch logs why and shuts
                  down (nothing runs without it; before, the stack froze
                  silently with no /clock).

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
    from launch.actions import (EmitEvent, ExecuteProcess, LogInfo,
                                RegisterEventHandler, SetEnvironmentVariable)
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from simple_launch import GazeboBridge, SimpleLauncher
except ImportError:                       # no ROS on this interpreter
    SetEnvironmentVariable = GazeboBridge = SimpleLauncher = None
    EmitEvent = LogInfo = RegisterEventHandler = OnProcessExit = Shutdown = None
    ExecuteProcess = None

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
    # gz_gui on a hybrid AMD + NVIDIA laptop: Mesa's EGL probes every DRM
    # device and skips the NVIDIA one it has no driver for (cosmetic; the GUI
    # renders on the display GPU, the ray-cast uses CUDA, not EGL).
    "libEGL warning",
    "pci id for fd",
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
_declare("headless", default_value="auto")
_declare("sonar_config", default_value="")
_declare("sonar_seed", default_value="0")
_declare("sonar_renderer", default_value="")
_declare("sonar_fouling", default_value="-1")
_declare("real_time_factor", default_value="1.0")
_declare("sonar_hold_sim", default_value="auto")

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


#: The slow-down range of ``real_time_factor`` (owner, 2026-11-07).
RTF_MIN, RTF_MAX = 0.01, 4.0


def parse_real_time_factor(value) -> float:
    """``real_time_factor`` as a float in [RTF_MIN, RTF_MAX]; raises with
    the accepted range otherwise (launch-free, exercised by the smoke test)."""
    try:
        rtf = float(str(value).strip() or 1.0)
    except ValueError:
        raise ValueError(f"real_time_factor {value!r} is not a number") from None
    if not RTF_MIN - 1e-12 <= rtf <= RTF_MAX + 1e-12:
        raise ValueError(f"real_time_factor {rtf} outside [{RTF_MIN}, {RTF_MAX}]")
    return rtf


def read_max_step_size(world_file: str, default: float = 0.01) -> float:
    """The world's ``<max_step_size>`` [s] (the regex sea_state_node uses)."""
    import re
    try:
        with open(world_file, "r", encoding="utf-8") as f:
            m = re.search(r"<max_step_size>\s*([0-9.eE+-]+)\s*</max_step_size>",
                          f.read())
        return float(m.group(1)) if m else default
    except OSError:
        return default


def gz_args_for(headless, real_time_factor=1.0, max_step_size=0.01) -> str:
    """Gazebo command-line flags: run immediately; server only when
    ``headless``; below real time, ``-z`` = the update rate that gives
    ``real_time_factor`` at this physics step (launch-free, exercised by
    the smoke test). At 1.0 the flags are exactly as before."""
    args = "-r -s" if str(headless).strip().lower() in ("1", "true", "yes") else "-r"
    rtf = parse_real_time_factor(real_time_factor)
    if abs(rtf - 1.0) > 1e-12:
        args += f" -z {rtf / float(max_step_size):g}"
    return args


def profile_renderer(sonar_config) -> str:
    """The ``renderer:`` key of a sonar profile (a path, or a bare name
    resolved against the package ``config/`` as ``sss_sim_launch`` does);
    "" when there is none or it cannot be read. Launch-free."""
    from pathlib import Path
    name = str(sonar_config or "").strip()
    if not name:
        return ""
    path = Path(os.path.expanduser(name))
    if not path.exists():
        try:
            from blueboat_sss_sim.sonar.config import find_default_sonar_yaml
            path = Path(find_default_sonar_yaml()).parent / path.name
        except Exception:  # noqa: BLE001
            return ""
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        return str(doc.get("renderer") or "").strip().lower()
    except (OSError, ValueError):
        return ""


def resolve_headless(headless, renderer_kind: str = "") -> bool:
    """``headless`` as a bool: true/false as given; ``auto`` (or empty) =
    server only for the ray-cast sonar (2026-11-09: the Gazebo GUI can crash
    at start-up and competes for the GPU), the GUI for the geometric one.
    Launch-free, exercised by the smoke test."""
    v = str(headless).strip().lower()
    if v in ("", "auto"):
        return str(renderer_kind or "").strip().lower() == "raycast"
    return v in ("1", "true", "yes")


def gazebo_gui_mode(headless, renderer_kind: str = "") -> str:
    """How the Gazebo GUI runs: ``"none"`` (``headless:=true``),
    ``"embedded"`` (one ``gz sim`` process, server + GUI: ``headless:=false``,
    or ``auto`` with the geometric sonar -- exactly as before) or ``"client"``
    (``auto`` with the ray-cast: the server runs ``-s`` and the GUI is a
    separate ``gz sim -g`` process, so a GUI crash cannot stop the server;
    2026-11-10). The server runs ``-s`` exactly when
    ``resolve_headless`` says so, i.e. for ``none`` and ``client``.
    Launch-free, exercised by the smoke test."""
    v = str(headless).strip().lower()
    if v in ("", "auto"):
        return ("client" if str(renderer_kind or "").strip().lower() == "raycast"
                else "embedded")
    return "none" if v in ("1", "true", "yes") else "embedded"


def gz_gui_client_cmd(gz_version=None) -> list:
    """The command of the separate GUI client: ``gz sim -g`` pinned to the
    same library version the server launch uses (``GZ_VERSION``, Harmonic's
    8 by default). Launch-free."""
    version = str(gz_version or os.environ.get("GZ_VERSION", "") or "").strip()
    version = {"harmonic": "8", "garden": "7", "fortress": "6",
               "ionic": "9"}.get(version.lower(), version) or "8"
    return ["gz", "sim", "-g", "--force-version", version]


def gazebo_exit_actions(process_name, returncode, shutting_down: bool):
    """What to do when a process exits: for Gazebo (``gazebo``) exiting by
    itself, the reason to log and that the launch must stop; None otherwise
    (another process, or the launch already shutting down). Launch-free."""
    if shutting_down or "gazebo" not in str(process_name or ""):
        return None
    return (f"Gazebo exited by itself (return code {returncode}): nothing runs "
            "without it (no /clock, no physics), so this launch stops now. If its "
            "console shows a segfault in gz::transport ... Discovery::RecvMessages / "
            "getenv, the Gazebo GUI crashed at start-up (an upstream gz-transport "
            "race): relaunch, or run the GUI as a separate client (headless:=auto, "
            "the default for the ray-cast sonar) or not at all (headless:=true).")


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


def launch_setup():
    world_dir = sl.arg("world_dir")
    if not world_dir:
        raise RuntimeError("launch argument 'world_dir' is required")
    world_file = f"{world_dir}/world.sdf"
    quiet = bool(sl.arg("quiet"))
    if quiet:
        _install_quiet_filter()

    # Gazebo exiting by itself (its GUI crashing at start-up, 2026-11-09)
    # left every other node waiting for a /clock that never came: say why
    # and stop the launch instead.
    def _on_exit(event, context):
        msg = gazebo_exit_actions(getattr(event.action, "name", ""),
                                  getattr(event, "returncode", None),
                                  bool(context.is_shutdown))
        if msg is None:
            return None
        return [LogInfo(msg=msg), EmitEvent(event=Shutdown(reason="Gazebo exited"))]
    sl.add_action(RegisterEventHandler(OnProcessExit(on_exit=_on_exit)))

    rtf = parse_real_time_factor(sl.arg("real_time_factor"))
    if rtf != 1.0:
        logging.getLogger("full_mission_launch").warning(
            "real_time_factor %.3f: the simulation runs %.1fx %s than real "
            "time (one clock; records are identical to a real-speed run)",
            rtf, max(rtf, 1.0 / rtf), "faster" if rtf > 1.0 else "slower")

    # 1. Gazebo + robot.
    if sl.arg("use_stock_world"):
        if rtf != 1.0:
            raise RuntimeError("real_time_factor != 1 needs the world's own "
                               "world.sdf (use_stock_world:=false)")
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
        renderer_kind = (str(sl.arg("sonar_renderer") or "").strip().lower()
                         or profile_renderer(sl.arg("sonar_config")))
        headless = resolve_headless(sl.arg("headless"), renderer_kind)
        gui_mode = gazebo_gui_mode(sl.arg("headless"), renderer_kind)
        sl.gz_launch(world_file, gz_args_for(headless, rtf,
                                             read_max_step_size(world_file)))
        if gui_mode == "client":
            # 2026-11-10: the GUI in its own process. Named without
            # "gazebo", so its exit never trips the Gazebo-exit shutdown
            # (gazebo_exit_actions); it inherits GZ_SIM_RESOURCE_PATH above.
            logging.getLogger("full_mission_launch").warning(
                "ray-cast sonar: Gazebo server + a separate GUI client (a GUI crash "
                "no longer stops the run; it respawns). headless:=true hides it, "
                "headless:=false = one process as before.")
            sl.add_action(ExecuteProcess(cmd=gz_gui_client_cmd(), name="gz_gui",
                                         output="screen", respawn=True,
                                         respawn_delay=3.0))
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
                            "data_dir": sl.arg("data_dir"),
                            # sim-time watchdog/republish (Control,
                            # 2026-11-07): one clock at any real-time factor
                            "time_source": "ros"})
        sl.node("blueboat_control", "path_generation.py", output="screen",
                parameters={"trajectory": f"from_yaml:{trajectory_file}"})
        sl.node("blueboat_control", "path_publisher.py", output="screen",
                parameters={"total_time":
                            trajectory_total_time(trajectory_file)})
        sl.node("blueboat_control", "master_control.py", output="screen",
                parameters={"controller_type": sl.arg("controller_type"),
                            "simulation": True,
                            "data_dir": sl.arg("data_dir"),
                            "time_source": "ros"})

    # The world's own name: the sea-state wrench bridges and the sonar's
    # hold (/world/<name>/control) address it.
    world_name = (STOCK_WORLD_NAME if sl.arg("use_stock_world")
                  else world_name_from_sdf(world_file))

    # 3. Simulated SSS + MAVROS shim, anchored at the world's GPS anchor.
    sl.include("blueboat_sss_sim", "sss_sim_launch.py",
               launch_arguments={"world_dir": world_dir,
                                 "quiet": quiet,
                                 "with_mavros_shim": sl.arg("with_mavros_shim"),
                                 "sim_origin_lat": sl.arg("sim_origin_lat"),
                                 "sim_origin_lon": sl.arg("sim_origin_lon"),
                                 "sonar_config": sl.arg("sonar_config"),
                                 "sonar_seed": sl.arg("sonar_seed"),
                                 "sonar_renderer": sl.arg("sonar_renderer"),
                                 "sonar_fouling": sl.arg("sonar_fouling"),
                                 "sonar_hold_sim": sl.arg("sonar_hold_sim"),
                                 "gz_world_name": world_name,
                                 "real_time_factor": str(rtf),
                                 **{a: sl.arg(a) for a in ACQUISITION_ARGS}})

    # 4. Sea state: current + waves on the hull (docs/sea_state_model.md).
    #    The wrench bridges are created for the world's own name; the
    #    node injects ApplyLinkWrench into the running world if the SDF
    #    predates it (worlds built before 2026-09, the stock world).
    sl.include("blueboat_sss_sim", "sea_state_launch.py",
               launch_arguments={"world_name": world_name,
                                 "world_dir": world_dir,
                                 "quiet": quiet,
                                 **{a: sl.arg(a) for a in SEA_ARGS}})

    return sl.launch_description()


generate_launch_description = (
    sl.launch_description(opaque_function=launch_setup)
    if sl is not None else None)
