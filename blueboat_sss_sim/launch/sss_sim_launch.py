"""Launch the simulated Side Scan Sonar stack over a world folder.

Composable with the existing simulator launch: start Gazebo on the
world's ``world.sdf`` as usual (``sim_world_launch.py`` or the full
``full_mission_launch.py``), then include or run this alongside. Nothing
in any neighbour package needs modification.

Arguments
---------
world_dir       world folder (from the ``world_builder`` GUI or
                ``generate_world``); provides the scene. Required.
sonar_config    sonar profile YAML (default: the world folder's own frozen
                ``sonar.yaml`` if one exists -- legacy mission bundles --
                else the package ``config/default_sonar.yaml``)
with_mavros_shim  republish heading/imu/GPS under MAVROS names (default
                  true; the BlueBoat GCS's GPS-anchored map needs
                  /mavros/global_position/global + compass_hdg to anchor)
sim_origin_lat  geographic latitude of the ENU world origin (default "":
                taken from the world's own geo anchor; the legacy
                43.6961 fallback applies only to unanchored worlds)
sim_origin_lon  geographic longitude, same resolution as the latitude
sonar_seed      sensor-noise seed for the node (default 0: a fresh seed from
                OS entropy, which the node logs and reports as
                ``noise_seed`` on ~/ground_truth/sensor, so any run can be
                replayed by passing that value here)
auto_ping       enable pinging automatically at startup       (default true)
quiet           route helper one-shot output to log           (default true)

Acquisition parameters
----------------------
The six run-dependent device parameters are passed to the node
**explicitly**, read from the resolved sonar profile, so a run's
acquisition is always traceable to a named file rather than to node
defaults -- which is what makes a sim-vs-real comparison meaningful (root
CLAUDE.md CM-2). The shipped default profile, the node's declared defaults
and the real ``sss_node.py``'s are the same six values since 2026-09-28
(20 m, gain -1 = device auto, 1200 bins, pulse 0.00125).

Each may still be overridden per run, exactly as on the real system:

    range_start_mm  range_length_mm  msec_per_ping
    gain_index      num_results      pulse_len_percent

An empty override (the default) means "take the profile's value".
"""

# ROS is needed to *launch*, not to read the resolvers below. Importing
# `simple_launch` pulls in `launch` -> `lark`, which the workspace venv does
# not carry, so a plain `python3 -m test.smoke_test` used to die here before
# reporting a single check (2026-09-18). The import is therefore optional:
# under ROS every line behaves exactly as before, and without it the module
# still yields its constants and its pure resolvers -- which is what
# `CLAUDE.md` means by "module-scope, launch-free resolvers the smoke test
# exercises directly".
try:
    from simple_launch import SimpleLauncher
except ImportError:                       # no ROS on this interpreter
    SimpleLauncher = None

sl = SimpleLauncher(use_sim_time=True) if SimpleLauncher is not None else None

# The six run-dependent parameters of the real sss_node.py, in its own order.
ACQUISITION_ARGS = ("range_start_mm", "range_length_mm", "msec_per_ping",
                    "gain_index", "num_results", "pulse_len_percent")

#: Legacy origin (the GCS bench simulator's) for worlds without a geo anchor.
LEGACY_ORIGIN = (43.6961, 7.3080)

if sl is not None:
    sl.declare_arg("world_dir", default_value="")
    sl.declare_arg("sonar_config", default_value="")
    sl.declare_arg("sonar_seed", default_value="0",
                   description="sensor-noise seed; 0 = fresh, logged and "
                               "reported as noise_seed")
    sl.declare_arg("with_mavros_shim", default_value=True)
    sl.declare_arg("sim_origin_lat", default_value="")
    sl.declare_arg("sim_origin_lon", default_value="")
    sl.declare_arg("auto_ping", default_value=True)
    sl.declare_arg("quiet", default_value=True)
    for _a in ACQUISITION_ARGS:
        sl.declare_arg(_a, default_value="",
                       description=f"{_a} override; empty = use the resolved "
                                   "sonar profile's value")


def resolve_sonar_config(world_dir, sonar_config=None) -> str:
    """The sonar profile in force: explicit > the folder's own frozen
    ``sonar.yaml`` (legacy bundles) > the package default. Module scope,
    launch-free, so the smoke test exercises the same resolution."""
    from pathlib import Path

    from blueboat_sss_sim.sonar.config import find_default_sonar_yaml

    if sonar_config:
        return str(sonar_config)
    frozen = Path(world_dir) / "sonar.yaml"
    if frozen.exists():
        return str(frozen)
    return str(find_default_sonar_yaml())


def resolve_acquisition(sonar_yaml, overrides=None):
    """Profile acquisition values, with non-empty overrides applied.

    Returns a ``{param: value}`` mapping ready to hand to the node. Kept at
    module scope, with no launch dependencies, so the offline smoke test can
    exercise the same resolution the launch performs.
    """
    from dataclasses import asdict, fields

    from blueboat_sss_sim.sonar.config import AcquisitionParams, SonarConfig

    acq = asdict(SonarConfig.from_yaml(sonar_yaml).acquisition)
    cast = {f.name: f.type for f in fields(AcquisitionParams)}
    for key, raw in (overrides or {}).items():
        if raw is None or str(raw) == "":
            continue
        acq[key] = float(raw) if cast[key] in ("float", float) else int(raw)
    AcquisitionParams(**acq)                  # validates the gain ladder early
    return acq


def resolve_world_anchor(world_dir):
    """The world's GPS anchor ``(lat0, lon0)`` from its manifest, or None.

    Module scope and launch-free like ``resolve_acquisition``, for the same
    reason. A world built by the ``world_builder`` GUI always carries one;
    a legacy bundle or a bare ``generate_world`` output may not.
    """
    from pathlib import Path

    import yaml

    manifest = Path(world_dir) / "scene_manifest.yaml"
    if not manifest.exists():
        return None
    with open(manifest, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    a = (doc.get("geo") or {}).get("anchor") or {}
    if "lat0" in a and "lon0" in a:
        return float(a["lat0"]), float(a["lon0"])
    return None


def launch_setup():
    world_dir = sl.arg("world_dir")
    if not world_dir:
        raise RuntimeError("launch argument 'world_dir' is required")
    quiet = bool(sl.arg("quiet"))

    # Simulated sonar (drop-in replacement for the real sss_node.py).
    # Acquisition comes from the resolved profile, passed explicitly, so
    # the run images what a named file records, never silent defaults.
    sonar_yaml = resolve_sonar_config(world_dir, sl.arg("sonar_config"))
    acquisition = resolve_acquisition(
        sonar_yaml, {a: sl.arg(a) for a in ACQUISITION_ARGS})
    sl.node("blueboat_sss_sim", "sss_sim_node", output="screen",
            parameters={
                "scene_dir": world_dir,
                "sonar_config": sonar_yaml,
                "odom_topic": "/blueboat/odom",
                "publish_ground_truth": True,
                "seed": int(str(sl.arg("sonar_seed") or 0)),
                **acquisition,
            })

    if sl.arg("with_mavros_shim"):
        # Origin priority: explicit launch args > the world's own anchor >
        # the legacy constant (with a warning -- GPS from such a run is
        # geographically meaningless).
        lat, lon = sl.arg("sim_origin_lat"), sl.arg("sim_origin_lon")
        if str(lat) == "" or str(lon) == "":
            anchor = resolve_world_anchor(world_dir)
            if anchor is not None:
                lat, lon = anchor
            else:
                lat, lon = LEGACY_ORIGIN
                import logging
                logging.getLogger("sss_sim_launch").warning(
                    "world %s carries no geo anchor; the MAVROS shim will "
                    "publish GPS about the legacy origin %s", world_dir,
                    LEGACY_ORIGIN)
        sl.node("blueboat_sss_sim", "mavros_shim_node",
                parameters={"sim_origin_lat": float(lat),
                            "sim_origin_lon": float(lon)})

    if sl.arg("auto_ping"):
        # One-shot ping enable a few seconds after the graph is up (same
        # command an operator issues on the real system). Its publisher
        # chatter ("beginning loop", "publishing #1: ...") goes to the log
        # file when quiet.
        from launch.actions import ExecuteProcess, TimerAction
        sl.add_action(TimerAction(period=5.0, actions=[ExecuteProcess(
            cmd=["ros2", "topic", "pub", "--once",
                 "/side_scan_sonar/ping/enable", "std_msgs/msg/Bool",
                 "data: true"],
            output="log" if quiet else "screen")]))

    return sl.launch_description()


generate_launch_description = (
    sl.launch_description(opaque_function=launch_setup)
    if sl is not None else None)
