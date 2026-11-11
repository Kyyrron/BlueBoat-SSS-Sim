#!/usr/bin/env python3
"""Simulated Side Scan Sonar node -- drop-in replacement for ``sss_node.py``.

Publishes exactly the topics, message types, QoS and control semantics of
the real node (see that file's docstring), but renders pings against the
generated :class:`SceneModel` instead of driving Omniscan 450 hardware:

Topics
------
Pub  ~/port/profile         blueboat_interfaces/OmniscanProfile
Pub  ~/port/raw             std_msgs/UInt8MultiArray   (byte-valid Ping Protocol)
Pub  ~/starboard/profile    blueboat_interfaces/OmniscanProfile
Pub  ~/starboard/raw        std_msgs/UInt8MultiArray
Pub  ~/ground_truth/contacts  std_msgs/String (JSON)   [simulation extra]
Pub  ~/ground_truth/sensor    std_msgs/String (JSON)   [simulation extra]:
                            per ping cycle, the vehicle pose and both
                            transducer poses the ping was rendered from,
                            ``run_id`` and ``range_scale`` (true slant range
                            -> imaged range; 1.0 for the geometric sonar)
Pub  ~/ground_truth/run_info  std_msgs/String (JSON)   [simulation extra,
                            latched: RELIABLE / TRANSIENT_LOCAL / 1]: what
                            produced this run -- renderer, profile path and
                            sha256, world digest, seeds, acquisition, the
                            build stamp and whether the installed code is the
                            source tree (``core/provenance.py``); at start-up,
                            every enable and stop (with counters). Also
                            written to ``run_info_dir`` (2026-11-11)
Sub  ~/ping/enable          std_msgs/Bool   true=start, false=stop
Sub  <odom_topic>           nav_msgs/Odometry (default /blueboat/odom); the
                            fallback pose -- optional while the 100 Hz
                            <pose_topic> is flowing

Run-dependent parameters (identical names/semantics to the real node)
----------------------------------------------------------------------
range_start_mm, range_length_mm, msec_per_ping, gain_index, num_results,
pulse_len_percent -- re-read each time pinging is enabled.

These ROS parameters are the node's acquisition interface, exactly as on the
real node -- names, semantics and (since 2026-09-28) defaults: 20 m, device
auto-gain, 1200 bins, pulse 0.00125, the device's best range resolution, and
(since 2026-10-27) SonarView's 50 ms per ping.
``sss_sim_launch.py`` still passes all six from the resolved sonar profile, so
a run's acquisition is traceable to a named file; a profile whose
``acquisition:`` block disagrees with the parameters actually in force is
reported as a warning at every ping enable.

``gain_index: -1`` is accepted with the real device's meaning (auto-gain) and
resolves to the calibrated index 4, which is what the profile then reports --
the device likewise resolves auto-gain internally and reports a concrete index,
since the ``gain_index`` field is ``uint16`` and cannot carry ``-1``.

Simulation-only parameters
--------------------------
scene_dir (required), sonar_config (YAML with the ``model:`` section),
odom_topic, publish_ground_truth, seed, predict_pose, pose_topic,
base_link_rest_z_m, renderer, raycast_backend, hold_sim_while_rendering,
gz_world_name, real_time_factor, fouling (ray-cast: the fouled fraction of
object and wall surfaces for this run, ``raycast.objects.fouling``; -1 = the
profile's).

Renderer selection (2026-11-07, ``raycast_design.md`` §11). ``renderer``
``""`` takes the profile's top-level ``renderer:`` key, else ``geometric``;
``raycast`` renders with the 3-D ray-cast sensor
(``sonar/raycast``, imported only then) from the profile's ``raycast:``
section, with the shared encoder/noise stage re-expressed under its system
gain (``shared_stage_model``). Re-read on every enable. ``raycast_backend``:
``auto`` (default) = ``warp`` when the ``warp`` module imports, else the
profile's; ``""`` = the profile's; or ``cpu`` / ``warp``. The geometric path
is byte-identical to before.

Slow-down (owner, 2026-11-07). The ray-cast costs far more than the 50 ms
ping period, so the simulation runs with Gazebo's real-time factor below 1
(``full_mission_launch.py real_time_factor:=``). Everything here is on sim
time already (timer, stamps, pose freshness), so a slowed run pings at the
same sim-time rate as a real-speed one. A ray-cast profile is stamped at its
tick's sim time (both sides), so the stamp does not depend on how long the
render took in wall time. When a tick comes more than 1.5 periods after the
last one in sim time, the renderer did not keep up at this factor: a warning
gives the achieved sim-time rate and the factor that would hold the period.

Holding the simulation (2026-11-09, ``core/sim_hold.py``). A ray-cast ping
with a quay in range costs 1-16 s instead of ~0.1 s; at a fixed slow-down
the ticks in between are skipped (the 2026-11-09 harbour run lost 23 % of
its pings). ``hold_sim_while_rendering`` ``auto`` (default: ``when_needed``
for the ray-cast, ``off`` for the geometric renderer) pauses the Gazebo
world ``gz_world_name`` through gz-transport just before the next tick
(the wall budget ``ping period / real_time_factor`` minus
``max(hold_margin_s, 0.1 x budget)``) and resumes it as soon
as the render ends, so every ping keeps its sim-time period whatever it
costs; ``always`` pauses around every render, ``off`` never. Without a
world name or a reachable ``/world/<name>/control`` (a standalone sonar, a
bag replay) the node pings as before, said once in the log.
``~/ground_truth/sensor`` reports ``sim_held`` per cycle (additive).

``seed`` 0 draws a fresh noise seed from OS entropy at start-up -- and logs
it and puts it on ``~/ground_truth/sensor`` (``noise_seed``), so any run can
be replayed with ``seed:=<that value>`` (2026-10-21; before, the entropy
seed was never recorded).

Vertical datum (2026-10-21). Gazebo's ``blueboat/base_link`` floats at rest
``base_link_rest_z_m`` above the still waterline (default
``sea.motion.GAZEBO_BASE_REST_Z_M``, the URDF buoyancy box's equilibrium,
-0.111 m); the renderer takes z = 0 as the waterline. Every received pose
is re-referenced (``sea.motion.waterline_pose``) before it is rendered, so
the transducers sit ``sensor_depth_m`` below the still water as in every
offline and calibration render. ``base_link_rest_z_m:=0.0`` restores the
pre-fix behaviour (transducers ~0.111 m deeper).

Platform motion (2026-10-08; 100 Hz pose 2026-10-09). Each ping is
rendered at the vehicle pose *at the ping time*: Gazebo's per-step model
pose (``pose_topic``, default ``/sim/sea_state/boat_pose``, bridged by
``sea_state_launch.py``; at most a physics step old), else the 20 Hz
odometry, propagated by its own body-frame rates with
``sea.motion.predict_pose`` -- the function the sea-state node uses for
the hull wrench -- so a rolling or pitching boat is not imaged with an
attitude up to 50 ms old (``predict_pose:=false`` restores the newest
sample as-is). The renderer then places each transducer by the full
attitude (``GeometricRenderer._sensor_pose``).

The extra ``~/ground_truth/*`` topics are additive: downstream consumers
of the real interface never see them unless they subscribe.
"""

from __future__ import annotations

import dataclasses
import hashlib
import importlib.util
import json
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String, UInt8MultiArray

from blueboat_interfaces.msg import OmniscanProfile

from ..core import provenance
from ..core.geometry import quat_to_rpy
from ..core.sim_hold import GzWorldControl, RenderHold, resolve_hold_mode
from ..sea.motion import (GAZEBO_BASE_REST_Z_M, HighRatePose, predict_pose,
                          waterline_pose)
from ..core.types import Pose3D, Side
from ..sonar.config import (AUTO_GAIN_INDEX, SPEED_OF_SOUND_MPS,
                            AcquisitionParams, SonarConfig, SonarModelConfig)
from ..sonar.encoder import PingEncoder
from ..sonar.noise import GainDrift, apply_ping_noise, ping_dropped
from ..sonar.renderer import GeometricRenderer, SonarRenderer
from ..worldgen.scene import SceneModel


#: Ticks after an enable the overrun diagnostic ignores (one-off start costs).
OVERRUN_GRACE_TICKS = 5
#: Wall seconds between looks for a Gazebo world that was not up at enable.
HOLD_RETRY_S = 2.0


class _SideChannel:
    """One transducer channel: renderer side + encoder + drift + publishers."""

    def __init__(self, node: "SideScanSonarSimNode", side: Side,
                 qos: QoSProfile, rng: np.random.Generator) -> None:
        self.side = side
        self.profile_pub = node.create_publisher(
            OmniscanProfile, f"~/{side.value}/profile", qos)
        self.raw_pub = node.create_publisher(
            UInt8MultiArray, f"~/{side.value}/raw", qos)
        self.encoder: Optional[PingEncoder] = None
        self.drift: Optional[GainDrift] = None
        self.rng = rng


class SideScanSonarSimNode(Node):
    """See module docstring."""

    def __init__(self) -> None:
        super().__init__("side_scan_sonar")

        # ---- Run-dependent parameters (identical to the real node, names
        # and defaults; the launches pass the resolved profile's values) ----
        self.declare_parameter("range_start_mm", 0)
        self.declare_parameter("range_length_mm", 20000)
        self.declare_parameter("msec_per_ping", 50)     # SonarView's interval
        self.declare_parameter("gain_index", -1)
        self.declare_parameter("num_results", 1200)
        self.declare_parameter("pulse_len_percent", 0.00125)

        # ---- Simulation-only parameters ------------------------------------
        self.declare_parameter("scene_dir", "")
        self.declare_parameter("sonar_config", "")
        self.declare_parameter("odom_topic", "/blueboat/odom")
        self.declare_parameter("publish_ground_truth", True)
        self.declare_parameter("seed", 0)
        self.declare_parameter("predict_pose", True)
        self.declare_parameter("pose_topic", "/sim/sea_state/boat_pose")
        self.declare_parameter("base_link_rest_z_m", GAZEBO_BASE_REST_Z_M)
        # Additive (2026-11-07): the renderer (module doc).
        self.declare_parameter("renderer", "")
        self.declare_parameter("raycast_backend", "auto")
        self.declare_parameter("fouling", -1.0)
        # Additive (2026-11-09): hold the simulation while a slow ping
        # renders (module doc, core/sim_hold.py).
        self.declare_parameter("hold_sim_while_rendering", "auto")
        self.declare_parameter("gz_world_name", "")
        self.declare_parameter("real_time_factor", 1.0)
        # Additive (2026-11-10): how long before the next tick the hold
        # pauses an overrunning render (the run-loop cost per held ping).
        self.declare_parameter("hold_margin_s", 0.005)
        # Additive (2026-11-11): where each run's provenance record is also
        # written ("" = nowhere; the topic ~/ground_truth/run_info carries it
        # either way). Never a world folder (NC #10).
        self.declare_parameter("run_info_dir", "~/.cache/blueboat_sss_sim/runs")

        scene_dir = self.get_parameter("scene_dir").value
        if not scene_dir:
            raise RuntimeError("parameter 'scene_dir' is required "
                               "(directory containing scene.npz + manifest)")
        self._scene = SceneModel.load(scene_dir)
        self.get_logger().info(
            f"scene loaded: {self._scene.grid.nx}x{self._scene.grid.ny} @ "
            f"{self._scene.grid.resolution} m, {len(self._scene.objects)} "
            f"objects, beyond the map: '{self._scene.outside}'")
        self._oom_warned = False
        stale = self._scene.stale_rasters()
        if stale:
            # SceneModel.load says the same through Python logging, which
            # the ROS console does not show: repeat it where the operator
            # looks.
            self.get_logger().warning(
                f"world {scene_dir} predates the current object model (no "
                f"{', '.join(stale)} raster): its objects render as they did "
                "when it was built. Rebuild it (World Builder -> Edit World "
                "-> save) for the current model.")

        sonar_cfg_path = self.get_parameter("sonar_config").value
        self._sonar_cfg_path: str = sonar_cfg_path or ""
        self._file_acq: Optional[AcquisitionParams] = None
        if sonar_cfg_path:
            file_cfg = SonarConfig.from_yaml(sonar_cfg_path)
            self._model_cfg: SonarModelConfig = file_cfg.model
            self._file_acq = file_cfg.acquisition
        else:
            self._model_cfg = SonarModelConfig()

        seed = int(self.get_parameter("seed").value)
        if not seed:
            # OS entropy, as before -- but a concrete, recorded value, so a
            # run's noise can be replayed (`seed:=<noise_seed>`).
            seed = int(np.random.SeedSequence().entropy) % (2 ** 63 - 1) or 1
        self._noise_seed = seed
        self._rng = np.random.default_rng(seed)
        self._base_rest_z = float(
            self.get_parameter("base_link_rest_z_m").value)
        self.get_logger().info(
            f"sensor noise seed {seed} (replay with seed:={seed}); base_link "
            f"rest z {self._base_rest_z:+.4f} m above the still waterline")

        # ---- Publishers (best-effort: matches real node) --------------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._channels = {
            Side.PORT: _SideChannel(self, Side.PORT, qos, self._rng),
            Side.STARBOARD: _SideChannel(self, Side.STARBOARD, qos, self._rng),
        }
        gt_on = bool(self.get_parameter("publish_ground_truth").value)
        self._gt_pub = (self.create_publisher(String, "~/ground_truth/contacts", 10)
                        if gt_on else None)
        self._sensor_pub = (self.create_publisher(String, "~/ground_truth/sensor", 10)
                            if gt_on else None)
        # Run provenance (2026-11-11, module doc): latched, so a recorder
        # that starts after the sonar still gets it; re-published at every
        # enable and stop.
        self._run_pub = (self.create_publisher(
            String, "~/ground_truth/run_info",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
            if gt_on else None)
        self._run_id = (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
                        + f"-{seed & 0xFFFF:04x}")
        self._run_static = self._provenance(scene_dir)
        self._labels_wanted = gt_on
        self._floor_addition = "power"
        self._rc = None                     # the RaycastConfig in force (ray-cast only)
        self._run_t: Optional[tuple] = None  # (wall, sim) at the last enable
        self._n_cycles = self._n_pings = self._n_dropped = self._n_held = 0
        self._walls: list[float] = []

        # ---- Inputs ----------------------------------------------------------
        self._pose: Optional[Pose3D] = None
        self._pose_t = 0.0                         # odometry stamp, sim time
        self._twist_body = (np.zeros(3), np.zeros(3))
        self._predict = bool(self.get_parameter("predict_pose").value)
        self._hr = HighRatePose()
        self._pose_source = "odom"
        if str(self.get_parameter("pose_topic").value):
            self.create_subscription(TransformStamped,
                                     str(self.get_parameter("pose_topic").value),
                                     self._on_boat_pose, 50)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._on_odom, 10)
        self.create_subscription(Bool, "~/ping/enable", self._on_ping_enable, 10)

        # ---- Ping engine state ------------------------------------------------
        self._renderer: Optional[SonarRenderer] = None
        self._acq: Optional[AcquisitionParams] = None
        self._timer = None
        self._t0: Optional[float] = None
        # The renderer in force and the encoder/noise model it pairs with:
        # the profile's own model for the geometric renderer (the same
        # object, so that path is byte-identical), the shared stage under the
        # ray-cast system gain otherwise.
        self._renderer_kind = "geometric"
        self._stage_cfg: SonarModelConfig = self._model_cfg
        self._raycast_key = None            # (acq, rc) the cached renderer was built for
        self._raycast_shared: dict = {}     # mesh + oracle, reused across enables
        self._period = 0.0
        self._last_tick_t: Optional[float] = None
        self._render_wall = 0.0
        self._render_only: Optional[float] = None   # the ray-cast's two renders [s]
        self._overruns = 0
        self._ticks = 0                     # ticks since the last enable
        self._hold: Optional[RenderHold] = None    # set per enable (_setup_hold)
        self._world_control = None                 # GzWorldControl, kept across enables
        self._hold_note = ""                       # the last "why no hold" logged
        self._hold_retry: Optional[tuple] = None   # (kind, period) while Gazebo is not up
        self._hold_next_check = 0.0                # wall time of the next retry

        # A ray-cast run builds and warms its renderer now, before the first
        # enable: the mesh, the GPU scene and the first render's one-off
        # setup take seconds to a minute, which would otherwise stall the
        # first ping cycle (module doc).
        if self._resolve_renderer_kind() == "raycast":
            self._prewarm_raycast()
        self._publish_run_info("startup")

        self.get_logger().info(
            "side_scan_sonar (SIMULATED) ready, ping OFF. Toggle with:\n"
            "  ros2 topic pub --once /side_scan_sonar/ping/enable "
            "std_msgs/msg/Bool 'data: true'")

    # ---------------------------------------------------------------- inputs
    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._pose = Pose3D(p.x, p.y, p.z, roll, pitch, yaw)
        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        # Gazebo's OdometryPublisher reports the twist in the child (body)
        # frame -- the convention predict_pose expects.
        self._twist_body = (np.array([lin.x, lin.y, lin.z]),
                            np.array([ang.x, ang.y, ang.z]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._pose_t = stamp if stamp > 0.0 else self._now()

    def _on_ping_enable(self, msg: Bool) -> None:
        if msg.data:
            self._start_pinging()
        else:
            self._stop_pinging()

    # ------------------------------------------------------------- lifecycle
    def _acquisition(self) -> AcquisitionParams:
        """The six run-dependent parameters as they stand now."""
        return AcquisitionParams(
            range_start_mm=self._int("range_start_mm"),
            range_length_mm=self._int("range_length_mm"),
            msec_per_ping=self._int("msec_per_ping"),
            gain_index=self._int("gain_index"),
            num_results=self._int("num_results"),
            pulse_len_percent=float(self.get_parameter("pulse_len_percent").value),
        )

    def _prewarm_raycast(self) -> None:
        """Build the ray-cast renderer for the current acquisition and render
        one cycle (discarded: no noise drawn, nothing published), so the
        first enable reuses it (``_build_raycast``'s cache) and pings at once.
        A failure is logged; the enable then tries again."""
        t0 = time.perf_counter()
        try:
            r = self._build_raycast(self._acquisition())
            for side in self._channels:
                r.render(side, Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0)
        except Exception as e:  # noqa: BLE001 -- the enable reports and retries
            self.get_logger().warning(
                f"ray-cast warm-up failed ({type(e).__name__}: {e}); the first "
                "enable will build it again")
            return
        self._renderer, self._renderer_kind = r, "raycast"
        self.get_logger().info(
            f"ray-cast renderer warmed up in {time.perf_counter() - t0:.1f} s")

    def _start_pinging(self) -> None:
        acq = self._acquisition()
        self._acq = acq
        self._warn_if_acquisition_diverges(acq)
        kind = self._resolve_renderer_kind()
        if kind == "raycast":
            try:
                self._renderer = self._build_raycast(acq)
            except Exception as e:  # noqa: BLE001 -- any build failure leaves the sonar off, loudly
                self._renderer = None
                self.get_logger().error(
                    f"ray-cast renderer could not be built ({type(e).__name__}: {e}); "
                    "pinging stays OFF. Fix the profile/backend, or enable with "
                    "renderer:=geometric.")
                return
        else:
            self._renderer = GeometricRenderer(self._scene, acq, self._model_cfg)
            self._stage_cfg = self._model_cfg
            self._floor_addition, self._rc = "power", None
        self._renderer_kind = kind
        for ch in self._channels.values():
            # One encoder per side for the node's whole life: the real
            # device numbers pings from power-up and a re-enable never
            # restarts its counter (a restarted counter puts duplicate ping
            # numbers into one .svlog, which the GCS keys rows on).
            if ch.encoder is None:
                ch.encoder = PingEncoder(ch.side, acq, self._stage_cfg)
            else:
                ch.encoder.reconfigure(acq, self._stage_cfg)
            ch.drift = GainDrift(self._stage_cfg, self._rng)
        if self._t0 is None:
            self._t0 = self._now()

        period = acq.ping_period_s(self._stage_cfg.max_ping_rate_hz)
        self._period = period
        self._last_tick_t = None
        self._overruns = 0
        self._ticks = 0
        self._setup_hold(kind, period)
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(period, self._tick)
        self._n_cycles = self._n_pings = self._n_dropped = self._n_held = 0
        self._walls = []
        self._run_t = (time.monotonic(), self._now())
        self._publish_run_info("enable")
        gain = (f"{acq.gain_index} (auto -> {acq.effective_gain_index})"
                if acq.gain_index == AUTO_GAIN_INDEX else str(acq.gain_index))
        self.get_logger().info(
            f"pinging started (renderer {kind}, range {acq.range_start_mm}-"
            f"{acq.range_start_mm + acq.range_length_mm} mm, gain "
            f"{gain}, n={acq.num_results}, period {period*1000:.0f} ms)")

    def _profile_doc(self) -> dict:
        """The sonar profile as a mapping ({} without one)."""
        if not self._sonar_cfg_path:
            return {}
        import yaml
        with open(self._sonar_cfg_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _resolve_renderer_kind(self) -> str:
        """``renderer`` param > the profile's ``renderer:`` key > geometric."""
        kind = str(self.get_parameter("renderer").value or "").strip().lower()
        if not kind:
            try:
                kind = str(self._profile_doc().get("renderer") or "").strip().lower()
            except (OSError, ValueError) as e:
                self.get_logger().warning(f"sonar profile unreadable ({e}): geometric")
                kind = ""
        kind = kind or "geometric"
        if kind not in ("geometric", "raycast"):
            self.get_logger().error(f"unknown renderer {kind!r}: geometric")
            kind = "geometric"
        return kind

    def _build_raycast(self, acq: AcquisitionParams):
        """The ray-cast renderer for this acquisition (lazy import; the mesh
        and oracle are built once per node and reused on every enable)."""
        from ..core import gauss_legendre_cache
        from ..sonar.raycast.config import RaycastConfig, shared_stage_model
        from ..sonar.raycast.renderer import RaycastRenderer
        # The GPU beam tables' Gauss-Legendre nodes, cached exactly on disk
        # (61 s per start on the system numpy otherwise).
        gauss_legendre_cache.install()
        doc = self._profile_doc()
        section = doc.get("raycast")
        if not isinstance(section, dict):
            raise ValueError(
                f"profile {self._sonar_cfg_path or '(none)'} has no 'raycast:' section "
                "(use config/raycast_default_sonar.yaml; the launches pick it for "
                "sonar_renderer:=raycast)")
        rc = RaycastConfig.from_dict(section)
        want = str(self.get_parameter("raycast_backend").value or "").strip().lower()
        if want == "auto":
            if importlib.util.find_spec("warp") is not None:
                want = "warp"
            else:
                want = ""
                self.get_logger().warning(
                    "ray-cast: the 'warp' module is not importable in this Python, so the "
                    f"profile's backend '{rc.backend}' runs (the CPU reference is very slow; "
                    "install warp-lang into the ROS environment for the GPU path)")
        if want:
            rc = dataclasses.replace(rc, backend=want)
        fouling = float(self.get_parameter("fouling").value)
        if fouling >= 0.0:
            rc = dataclasses.replace(
                rc, objects=dataclasses.replace(rc.objects, fouling=fouling))
        if rc.seed_per_run:
            # the speckle realisation follows this run's noise seed (replay
            # with seed:=<noise_seed>, which ~/ground_truth/run_info records)
            rc = dataclasses.replace(rc, seed=int(self._noise_seed) % (2 ** 62))
        self._rc = rc
        self._floor_addition = str(rc.floor.addition)
        key = (acq, rc)
        if self._raycast_key == key and self._renderer is not None \
                and self._renderer_kind == "raycast":
            return self._renderer
        if self._renderer is not None and hasattr(self._renderer, "release_device"):
            self._renderer.release_device()
        t0 = time.perf_counter()
        # The mesh and its oracle are reused across enables -- as long as the
        # profile meshes the objects the same way (raycast.objects).
        shared = (self._raycast_shared
                  if self._raycast_shared.get("objects") == rc.objects else {})
        r = RaycastRenderer(self._scene, acq, self._stage_model(rc, shared_stage_model, acq), rc,
                            world_dir=self.get_parameter("scene_dir").value,
                            mesh=shared.get("mesh"), oracle=shared.get("oracle"))
        self._raycast_shared = dict(mesh=r.mesh, oracle=r.oracle, objects=rc.objects)
        # No device sync per GPU stage in a live run (only offline timing
        # wants it; no value depends on it).
        r.stage_timing = False
        # Ground-truth contacts are computed only when someone can read them.
        r.labels_on = bool(rc.labels.enabled and self._labels_wanted)
        self._raycast_key = key
        self.get_logger().info(
            f"ray-cast renderer ready (backend {rc.backend}, system gain "
            f"{rc.system_gain_db:.1f} dB, built in {time.perf_counter() - t0:.1f} s). "
            "Run the simulation slowed down (real_time_factor:=) so it holds the ping "
            "period in sim time.")
        return r

    def _stage_model(self, rc, shared_stage_model, acq) -> SonarModelConfig:
        from ..sonar.raycast.config import noise_stage_model
        model = shared_stage_model(self._model_cfg, rc.system_gain_db)
        # raycast.floor (NC #15 shadow floor) and raycast.near_range (the
        # measured water column) are the NOISE stage's only (harbour patch,
        # 2026-11-10): the renderer keeps its own model, so its multipath
        # selection cannot move with them. Off: the same model, bit-identical.
        self._stage_cfg = noise_stage_model(self._model_cfg, rc, acq,
                                            float(self._model_cfg.max_ping_rate_hz))
        return model

    def _warn_if_acquisition_diverges(self, acq: AcquisitionParams) -> None:
        """Report a bundle that records different acquisition than is in force.

        The bundle is an immutable record of what a run acquired at (NC #10),
        so a mismatch means the ground truth and the pings describe different
        surveys. Warn rather than raise: overriding acquisition per run is
        legitimate device behaviour, and the operator is told which knobs moved.
        """
        if self._file_acq is None:
            return
        differing = [
            f"{f.name}: bundle {getattr(self._file_acq, f.name)} "
            f"!= in force {getattr(acq, f.name)}"
            for f in dataclasses.fields(AcquisitionParams)
            if getattr(self._file_acq, f.name) != getattr(acq, f.name)
        ]
        if differing:
            self.get_logger().warning(
                f"acquisition differs from the bundle's frozen sonar.yaml "
                f"({self._sonar_cfg_path}): " + "; ".join(differing) +
                ". The bundle records the survey it was generated for; "
                "regenerate it if this override is the intended acquisition.")

    def _setup_hold(self, kind: str, period: float) -> None:
        """Hold the simulation while a slow ping renders (module doc): the
        world control for ``gz_world_name``, or none -- with the reason said
        once -- so the node pings exactly as before."""
        self._hold = None
        self._hold_retry = None
        try:
            mode = resolve_hold_mode(
                str(self.get_parameter("hold_sim_while_rendering").value), kind)
        except ValueError as e:
            self.get_logger().error(f"{e}: holding the simulation is OFF")
            return
        if mode == "off" or kind != "raycast":
            return
        world = str(self.get_parameter("gz_world_name").value or "").strip()
        note = ""
        if not world:
            note = ("no gz_world_name (a standalone sonar or a bag replay): pings are "
                    "not held -- a slow render skips ticks")
        else:
            try:
                if (self._world_control is None
                        or self._world_control.world_name != world):
                    self._world_control = GzWorldControl(world)
                if not self._world_control.available():
                    # Gazebo not up yet (a slow world load) or gone: try again
                    # from the ping timer every HOLD_RETRY_S (_tick).
                    self._hold_retry = (kind, period)
                    self._hold_next_check = time.monotonic() + HOLD_RETRY_S
                    note = (f"{self._world_control.service} is not advertised (no Gazebo "
                            "world of that name yet): pings are not held; retrying every "
                            f"{HOLD_RETRY_S:g} s")
            except ImportError as e:
                note = f"gz-transport bindings unavailable ({e}): pings are not held"
        if note:
            if note != self._hold_note:
                self.get_logger().info(f"hold_sim_while_rendering: {note}")
            self._hold_note = note
            return
        rtf = min(max(float(self.get_parameter("real_time_factor").value), 1e-3), 4.0)
        budget = period / rtf
        self._hold = RenderHold(self._world_control, mode, budget,
                                margin_s=float(self.get_parameter("hold_margin_s").value),
                                log=lambda m: self.get_logger().warning(
                                    f"hold_sim_while_rendering: {m}"))
        self.get_logger().info(
            f"holding the simulation ({mode}) while a ray-cast cycle overruns: world "
            f"'{world}', wall budget {budget * 1000:.0f} ms per ping "
            f"(x{rtf:.3f}); every ping keeps its {period * 1000:.0f} ms sim-time period")

    def release_hold(self) -> None:
        """Resume the world if a hold paused it (stop / shutdown)."""
        if self._hold is not None:
            self._hold.resume()

    def _stop_pinging(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.release_hold()
        if self._run_t is not None:
            self._publish_run_info("stop")
            self._run_t = None
        self.get_logger().info("pinging stopped")

    # ------------------------------------------------------------------ tick
    def _on_boat_pose(self, msg: TransformStamped) -> None:
        child = msg.child_frame_id.split("/")[-1]
        if child and child != "blueboat":
            return
        tr, q = msg.transform.translation, msg.transform.rotation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._hr.update(st if st > 0.0 else self._now(),
                        Pose3D(tr.x, tr.y, tr.z, roll, pitch, yaw))

    def _ping_pose(self, t_now: float) -> Optional[tuple[Pose3D, float]]:
        """The vehicle pose at the ping time and the source's age [s]:
        the 100 Hz model pose when it is fresh, else the odometry; None
        when neither exists (no ping that cycle)."""
        hr = self._hr.state()
        if hr is not None and t_now - hr[3] < 0.1:
            pose, v_body, w_body, t_hr = hr
            age = t_now - t_hr
            self._pose_source = "pose_100hz"
            if not self._predict:
                return pose, age
            return predict_pose(pose, v_body, w_body, age), age
        if self._pose is None:
            return None
        self._pose_source = "odom"
        age = t_now - self._pose_t
        if not self._predict:
            return self._pose, age
        v_body, w_body = self._twist_body
        return predict_pose(self._pose, v_body, w_body, age), age

    def _tick(self) -> None:
        if self._renderer is None or self._acq is None:
            return
        now_time = self.get_clock().now()
        t_now = now_time.nanoseconds * 1e-9
        self._check_overrun(t_now)
        got = self._ping_pose(t_now)
        if got is None:
            return
        boat, age = got
        # Gazebo's base_link -> the renderer's waterline datum (module doc).
        pose = waterline_pose(boat, self._base_rest_z)
        t_sim = t_now - (self._t0 or 0.0)
        gt_payload = []
        sensors: dict = {}
        raycast = self._renderer_kind == "raycast"
        pre: Optional[dict] = None
        stamp = None
        w0 = time.perf_counter()
        if raycast:
            # Both sides rendered before either is published (NC #3: one
            # tick, both sides), stamped at the tick's sim time; a failed
            # render (a transient device allocation) skips the whole cycle.
            stamp = now_time.to_msg()
            renderer = self._renderer
            if (self._hold is None and self._hold_retry is not None
                    and time.monotonic() >= self._hold_next_check):
                self._setup_hold(*self._hold_retry)   # Gazebo came up late?

            def render_both():
                return {s: renderer.render(s, pose, t_sim) for s in self._channels}
            try:
                # Held (2026-11-09): Gazebo pauses while this cycle overruns
                # its wall budget, so the next tick keeps its sim time.
                pre = (self._hold.run(render_both) if self._hold is not None
                       else render_both())
                self._render_only = time.perf_counter() - w0
            except Exception as e:  # noqa: BLE001 -- one lost cycle, never a dead node
                self.get_logger().error(
                    f"ray-cast render failed ({type(e).__name__}: {e}); cycle skipped",
                    throttle_duration_sec=10.0)
                if hasattr(self._renderer, "release_device"):
                    self._renderer.release_device()
                return
        for ch in self._channels.values():
            rendered = (pre[ch.side] if pre is not None
                        else self._renderer.render(ch.side, pose, t_sim))
            sp = rendered.ping.pose
            oom = rendered.ping.out_of_map_from_m
            sensors[ch.side.value] = {
                "pose": [round(float(v), 5) for v in
                         (sp.x, sp.y, sp.z, sp.roll, sp.pitch, sp.yaw)],
                "altitude_m": round(float(rendered.ping.altitude_m), 4),
                "ping_number": None, "dropped": False,
                # Additive (2026-10-21): where this side's swath leaves the
                # authored map, null while it stays inside (A13).
                "out_of_map_from_m": (None if oom is None
                                      else round(float(oom), 2))}
            if oom is not None and not self._oom_warned:
                self._oom_warned = True
                self.get_logger().warning(
                    f"the {ch.side.value} swath leaves the world map "
                    f"{oom:.1f} m out: beyond it the sonar sees the scene's "
                    f"'{self._scene.outside}' rule (a flat phantom plain at "
                    "the deepest cell under 'fill', the repeated edge under "
                    "'edge'), not authored seabed. Keep surveys range_max "
                    "inside the limits (docs/sonar_model.md A13); "
                    "ground_truth/sensor reports it per ping. (Logged once.)")
            if ping_dropped(self._stage_cfg, ch.rng):
                # Device counter still advances on the real hardware.
                if ch.encoder is not None:
                    ch.encoder._ping_number += 1  # noqa: SLF001 deliberate
                sensors[ch.side.value]["dropped"] = True
                self._n_dropped += 1
                continue
            wc = self._watercolumn_mask(rendered.ping.altitude_m)
            noisy = apply_ping_noise(
                rendered.ping.power, wc,
                ch.drift.value(t_sim) if ch.drift else 1.0,
                self._stage_cfg, ch.rng,
                specular=rendered.ping.specular,
                texture=rendered.ping.texture,
                unoccluded=rendered.ping.unoccluded,
                ambient=rendered.ping.ambient,
                speckle_fill=rendered.ping.speckle_fill,
                floor_addition=self._floor_addition)
            rendered.ping.power = noisy
            enc = ch.encoder.encode(rendered.ping)  # type: ignore[union-attr]
            self._publish(ch, enc, stamp)
            sensors[ch.side.value]["ping_number"] = int(enc.ping_number)
            self._n_pings += 1
            if self._gt_pub is not None and rendered.contacts:
                gt_payload += [{
                    "side": c.side.value,
                    # A wall face's own echo is labelled by the wall's name;
                    # scene objects keep their integer id.
                    "object_id": (f"wall:{c.wall_name}"
                                  if c.object_type == "wall" else c.object_id),
                    "type": c.object_type,
                    "slant_range_m": round(c.slant_range_m, 3),
                    "extent_bins": round(c.extent_bins, 1),
                    "shadow_bins": round(c.shadow_bins, 1),
                    "visible": c.visible,
                    "ping_number": enc.ping_number,
                    # Additive: a multipath image of the object named by
                    # object_id, and the reflector that produced it. Absent
                    # keys mean "direct path" for a consumer reading an
                    # older stream, so nothing downstream has to change.
                    "ghost": c.ghost,
                    "via": c.via,
                } for c in rendered.contacts]
        if self._gt_pub is not None and gt_payload:
            m = String()
            m.data = json.dumps({"t_sim": round(t_sim, 3),
                                 "contacts": gt_payload})
            self._gt_pub.publish(m)
        if self._sensor_pub is not None:
            # Every cycle, contacts or not: what the validation tooling
            # (analysis/sea_validation.py) reads to know exactly where each
            # ping was taken from. Pose = [x, y, z, roll, pitch, yaw], world
            # ENU, metres / radians. `boat` is the pose as received (Gazebo's
            # base_link, predicted to the ping time); `sensors.*.pose` is
            # what was rendered, on the waterline datum `base_link_rest_z_m`
            # below it (additive keys `base_link_rest_z_m`, `noise_seed`).
            m = String()
            m.data = json.dumps({
                "t_sim": round(t_sim, 4), "stamp": round(t_now, 4),
                "pose_age_ms": round(1000.0 * age, 1), "predicted": self._predict,
                "pose_source": self._pose_source,
                "boat": [round(float(v), 5) for v in
                         (boat.x, boat.y, boat.z, boat.roll, boat.pitch, boat.yaw)],
                "base_link_rest_z_m": round(self._base_rest_z, 5),
                "noise_seed": self._noise_seed,
                # Additive (2026-11-11): the run this ping belongs to
                # (~/ground_truth/run_info), and the factor from TRUE slant
                # range to the range the image shows it at. The ray-cast
                # times echoes with the water's sound speed and bins them
                # with the device's 1500 m/s, as the real Omniscan does, so a
                # target at true range R is imaged at R * range_scale; 1.0
                # for the geometric sonar. Label geometry must apply it.
                "run_id": self._run_id,
                "range_scale": round(self._range_scale(), 6),
                # Additive (2026-11-09): Gazebo was paused while this cycle
                # rendered (hold_sim_while_rendering, module doc).
                "sim_held": bool(self._hold is not None and raycast
                                 and self._hold.last_held),
                "sensors": sensors})
            self._sensor_pub.publish(m)
        self._render_wall = time.perf_counter() - w0
        self._n_cycles += 1
        if self._hold is not None and raycast and self._hold.last_held:
            self._n_held += 1
        if len(self._walls) < 200_000:
            self._walls.append(self._render_wall)

    def _publish(self, ch: _SideChannel, enc, stamp=None) -> None:
        msg = OmniscanProfile()
        # stamp None (geometric): the publish time, as before; the ray-cast
        # passes its tick's sim time (module doc).
        msg.header.stamp = self.get_clock().now().to_msg() if stamp is None else stamp
        msg.header.frame_id = enc.frame_id
        msg.side = ch.side.value
        msg.ping_number = enc.ping_number
        msg.start_mm = enc.start_mm
        msg.length_mm = enc.length_mm
        msg.timestamp_ms = enc.timestamp_ms
        msg.ping_hz = enc.ping_hz
        msg.gain_index = enc.gain_index
        msg.num_results = enc.num_results
        msg.sos_dmps = enc.sos_dmps
        msg.channel_number = enc.channel_number
        msg.pulse_duration_sec = enc.pulse_duration_sec
        msg.analog_gain = enc.analog_gain
        msg.max_pwr_db = enc.max_pwr_db
        msg.min_pwr_db = enc.min_pwr_db
        msg.transducer_heading_deg = enc.transducer_heading_deg
        msg.vehicle_heading_deg = enc.vehicle_heading_deg
        msg.pwr_results = [int(v) for v in enc.pwr_results]
        ch.profile_pub.publish(msg)

        raw = UInt8MultiArray()
        raw.data = list(enc.raw_frame)
        ch.raw_pub.publish(raw)

    def _check_overrun(self, t_now: float) -> None:
        """Warn (throttled) when this tick comes more than 1.5 ping periods
        of sim time after the last one: the renderer did not keep up at the
        simulation's current speed. Log only; nothing published changes."""
        last, self._last_tick_t = self._last_tick_t, t_now
        self._ticks += 1
        # The first ticks after an enable carry one-off costs (first render,
        # caches, the GPU's first launch): not a sustained rate.
        if last is None or self._period <= 0.0 or self._ticks <= OVERRUN_GRACE_TICKS:
            return
        dt = t_now - last
        if dt <= 1.5 * self._period:
            return
        self._overruns += 1
        render = max(self._render_wall, 1e-6)
        hold = self._period / render
        if self._hold is not None and self._hold.active:
            # Holding should make this impossible: the pause came too late
            # (a busy interpreter delaying the timer, a slow pause request).
            self.get_logger().warning(
                f"{self._renderer_kind} sonar behind the sim clock although the "
                f"simulation is held ({self._hold.mode}): {dt * 1000:.0f} ms of sim time "
                f"between pings against a {self._period * 1000:.0f} ms period "
                f"({self._overruns} late ticks since enable; this cycle took "
                f"{render * 1000:.0f} ms of wall time). If it repeats, set "
                "hold_sim_while_rendering:=always.",
                throttle_duration_sec=10.0)
            return
        self.get_logger().warning(
            f"{self._renderer_kind} sonar behind the sim clock: {dt * 1000:.0f} ms of sim "
            f"time between pings against a {self._period * 1000:.0f} ms period "
            f"({1.0 / dt:.1f} Hz achieved, {self._overruns} late ticks since enable); one "
            f"ping cycle takes {render * 1000:.0f} ms of wall time"
            + (f" ({self._render_only * 1000:.0f} ms rendering)"
               if self._render_only is not None else "")
            + f", so a real-time factor of x{min(hold, 1.0):.3f} or lower holds the period.",
            throttle_duration_sec=10.0)

    # --------------------------------------------------------------- helpers
    def _watercolumn_mask(self, altitude_m: float) -> np.ndarray:
        assert self._acq is not None
        r = (self._acq.range_start_mm / 1000.0
             + (np.arange(self._acq.num_results) + 0.5) * self._acq.bin_size_m)
        return r < altitude_m

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ provenance
    def _range_scale(self) -> float:
        """True slant range -> imaged range (module doc, ``range_scale``)."""
        c = getattr(self._renderer, "c", None) if self._renderer_kind == "raycast" else None
        return float(SPEED_OF_SOUND_MPS) / float(c) if c else 1.0

    def _provenance(self, scene_dir: str) -> dict:
        """The part of the run record that cannot change while the node
        lives: build stamp, install-vs-source check, world and profile
        digests. A stale install is said loudly, once, with the file names."""
        log = self.get_logger()
        out: dict = {"build": {}, "source_check": {"matches": None},
                     "world": {"dir": str(scene_dir)}, "profile": {}}
        try:
            out["build"] = provenance.build_info()
            chk = provenance.source_check()
            out["source_check"] = {
                "matches": chk["matches"], "running_digest": chk["running_digest"],
                "stale_files": (chk["differing"] + chk["only_in_source"]
                                + chk["only_in_install"])[:40]}
            msg = provenance.stale_message(chk)
            if msg:
                log.warning(msg)
            out["world"] = provenance.world_digest(scene_dir)
            out["profile"] = provenance.profile_info(self.get_parameter("sonar_config").value)
        except Exception as e:  # noqa: BLE001 -- provenance must never stop the sonar
            log.warning(f"run provenance incomplete ({type(e).__name__}: {e})")
        return out

    def _run_info(self, event: str) -> dict:
        """The run record (``~/ground_truth/run_info``, JSON,
        ``blueboat_sss_sim_run/1``): what produced the pings of this run."""
        st = self._run_static
        rc = self._rc
        doc: dict = {
            "schema": "blueboat_sss_sim_run/1", "run_id": self._run_id, "event": event,
            "wall_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "renderer": self._renderer_kind if self._renderer is not None
            else self._resolve_renderer_kind(),
            "backend": getattr(rc, "backend", None),
            "profile": dict(st.get("profile") or {}),
            "world": {**(st.get("world") or {}),
                      "seed": int(getattr(self._scene, "seed", 0) or 0),
                      "objects": len(self._scene.objects)},
            "seeds": {"noise": int(self._noise_seed),
                      "scatterer": getattr(self._renderer, "seed", None)},
            "acquisition": (dataclasses.asdict(self._acq) if self._acq is not None else None),
            "range_scale": round(self._range_scale(), 6),
            "build": st.get("build"), "source_check": st.get("source_check"),
            "hold_mode": getattr(self._hold, "mode", "off") if self._hold is not None else "off",
            "real_time_factor": float(self.get_parameter("real_time_factor").value),
            "gz_world_name": str(self.get_parameter("gz_world_name").value or ""),
        }
        try:
            doc["profile"]["variant"] = (self._profile_doc().get("variant") or {}).get("label")
        except (OSError, ValueError):
            pass
        if rc is not None:
            sec = json.dumps(dataclasses.asdict(rc), sort_keys=True, default=repr)
            doc["raycast"] = {
                "resolved_sha256": hashlib.sha256(sec.encode()).hexdigest(),
                "seed": int(rc.seed), "seed_per_run": bool(rc.seed_per_run),
                "labels": bool(getattr(self._renderer, "labels_on", False)),
                "objects_mode": getattr(self._renderer, "objects_mode", None),
                "fouling": float(rc.objects.fouling),
                "floor_addition": str(rc.floor.addition),
                "correction": {"enabled": bool(rc.correction.enabled),
                               "artifact": str(rc.correction.artifact)}}
        if event == "stop" and self._run_t is not None:
            w = np.asarray(self._walls, dtype=np.float64)
            wall = time.monotonic() - self._run_t[0]
            sim = self._now() - self._run_t[1]
            doc["counters"] = {
                "cycles": int(self._n_cycles), "pings": int(self._n_pings),
                "dropped": int(self._n_dropped), "late_ticks": int(self._overruns),
                "held_cycles": int(self._n_held),
                "cycle_wall_ms_p50": round(float(np.percentile(w, 50)) * 1e3, 2) if w.size else None,
                "cycle_wall_ms_p90": round(float(np.percentile(w, 90)) * 1e3, 2) if w.size else None,
                "wall_s": round(wall, 2), "sim_s": round(sim, 2),
                "rtf_effective": round(sim / wall, 4) if wall > 0 else None}
        return doc

    def _publish_run_info(self, event: str) -> None:
        try:
            doc = self._run_info(event)
            text = json.dumps(doc)
        except Exception as e:  # noqa: BLE001 -- never stop the sonar for its record
            self.get_logger().warning(f"run info not built ({type(e).__name__}: {e})")
            return
        if self._run_pub is not None:
            m = String()
            m.data = text
            self._run_pub.publish(m)
        d = str(self.get_parameter("run_info_dir").value or "").strip()
        if d:
            try:
                out = Path(d).expanduser()
                out.mkdir(parents=True, exist_ok=True)
                (out / f"{self._run_id}.json").write_text(json.dumps(doc, indent=1))
            except OSError as e:
                self.get_logger().warning(f"run info not written to {d} ({e})",
                                          throttle_duration_sec=60.0)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SideScanSonarSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl-C, or a launch teardown (rclpy's own SIGINT handler)
    except Exception:  # noqa: BLE001
        # rclpy's SIGINT handler can shut the context down while the executor
        # builds its next wait set ("failed to initialize wait set: the given
        # context is not valid"); a ~1 s ray-cast render in the timer callback
        # makes that window wide (seen at every MCS stop of a ray-cast run,
        # 2026-11-07). A teardown race, not a failure: re-raise only while
        # the context is still up.
        if rclpy.ok():
            raise
    finally:
        # Never leave Gazebo paused behind a dead sonar (2026-11-09).
        try:
            node.release_hold()
        except Exception:  # noqa: BLE001 -- teardown must go on
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
