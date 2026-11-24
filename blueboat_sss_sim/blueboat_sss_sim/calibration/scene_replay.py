"""Ping-for-ping replay of a recording's poses through a WORLD (pass 3, 2026-11-01).

The seabed engine (``seabed.py``) renders flat sand at a stratum's altitude; a
session with ``regime.forward_model: scene`` (the pool) is scored against its own
world instead: every selected real ping is rendered at its own (registered) pose,
acquisition and gain, through the same noise call and wire encoder as the node,
and decoded like the device -- so one reduction applies to both sides of the pair.

Two modes, as in the seabed engine:

* ``noisy`` -- ``apply_ping_noise`` + ``PingEncoder`` + ``scale_to_db``, one
  ``numpy.random.Generator`` per call (seeded), poses rendered in order;
* ``expected`` -- the noise-free mean intensity (``seabed.expected_power``)
  mapped to reported dB (``seabed._to_reported_db``); deterministic.

On a flat sand scene with the seabed engine's poses, ``render_rows`` reproduces
``seabed._render_noisy`` bit for bit (smoke ``[16]``), so the two engines can be
compared stratum by stratum.

Pure library: no ROS, no file I/O. Pose convention: world ENU metres, ``z`` the
vehicle's height (0 = floating level on the still water), roll/pitch/yaw in the
renderer's FLU frame (MAVLink FRD attitude: ``roll = roll, pitch = -pitch``,
``yaw_enu = pi/2 - yaw_ned``).
"""

from __future__ import annotations

import copy
import dataclasses
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class ReplayPose:
    """One ping to render: the vehicle pose and the ping's own acquisition."""

    side: str                 # "port" | "starboard"
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float                # ENU, radians
    range_mm: int
    num_results: int
    pulse_us: float
    gain: int = 4


def _acq(cfg, p: ReplayPose):
    from .seabed import acquisition_for
    return acquisition_for(cfg, p.range_mm, p.num_results, p.pulse_us, p.gain)


def render_rows(scene, cfg, poses: Sequence[ReplayPose], *, mode: str = "noisy",
                seed: int = 0, world_dir=None, cache: dict | None = None) -> dict:
    """Render ``poses`` in ``scene`` with the sonar config ``cfg``.

    Returns ``rows`` (n, N) reported dB (float32; NaN-padded if the bin count
    varies), ``fbr`` (detector bin or -1), ``max_db`` (n,), ``altitude`` (the
    renderer's geometric altitude, m) and ``out_of_map`` (count of pings whose
    beam left the scene grid). Renderers are built once per acquisition and
    side, in the order the poses come, so a fixed ``seed`` is reproducible.

    A ``cfg`` carrying a ray-cast section (``sonar.raycast.config.
    raycast_sonar_config``, the seabed engine's hook) renders with the
    ray-cast sensor and its shared-stage model, one scene mesh and oracle for
    every renderer of the call; ``world_dir`` (that sensor only) gives its
    mesh the world's terrain-only raster. Otherwise the geometric renderer,
    unchanged.

    ``cache`` (optional, one dict per scene and config, reused across calls)
    keeps the renderers and a copy of every rendered ping keyed by its
    acquisition and pose. A render is deterministic per pose; the noise is
    drawn per call, after it, in the same order, so the rows equal those of
    an uncached call. Without it nothing changes.
    """
    from ..analysis.calibration import fbr_bin, scale_to_db
    from ..core.types import Pose3D, Side
    from ..sonar.encoder import PingEncoder
    from ..sonar.noise import apply_ping_noise
    from ..sonar.renderer import GeometricRenderer
    from .seabed import _to_reported_db, expected_power
    if mode not in ("noisy", "expected"):
        raise ValueError(f"mode {mode!r} (noisy | expected)")
    model = cfg.model
    noise_model = None                       # the noise stage's, when it differs
    floor_addition = "power"                 # raycast.floor.addition, as in the node
    rc = getattr(cfg, "raycast", None)       # None: the geometric renderer
    if rc is not None:
        from ..sonar.raycast.config import RaycastConfig, shared_stage_model
        from ..sonar.raycast.renderer import RaycastRenderer
        rcfg = RaycastConfig.from_dict(rc)
        floor_addition = str(rcfg.floor.addition)
        model = shared_stage_model(cfg.model, rcfg.system_gain_db)
        from ..sonar.raycast.config import noise_stage_model
        # raycast.floor / near_range: the noise stage only, as in the node
        noise_model = (lambda acq_: noise_stage_model(cfg.model, rcfg, acq_,
                                                      float(cfg.model.max_ping_rate_hz)))
        shared: dict = {}

        def make(acq):
            r = RaycastRenderer(scene, acq, model, rcfg, world_dir=world_dir,
                                mesh=shared.get("mesh"), oracle=shared.get("oracle"))
            shared.update(mesh=r.mesh, oracle=r.oracle)
            return r
    else:
        def make(acq):
            return GeometricRenderer(scene, acq, model)
    if rc is not None:
        # the ray-cast sensor on the GPU: a render waits and retries when the
        # device is momentarily full (another GPU process), never fails a
        # multi-hour scoring run on a transient allocation
        from ..sonar.raycast.calib import _build_retrying

        def render(r, sd, ps):
            return _build_retrying(lambda: r.render(sd, ps, 0.0).ping, r.release_device)
    else:
        def render(r, sd, ps):
            return r.render(sd, ps, 0.0).ping
    rng = np.random.default_rng(seed)
    if cache is None:
        rcache: dict = {}
        pcache = None
    else:
        rcache = cache.setdefault("renderers", {})
        pcache = cache.setdefault("pings", {})
    nmax = max((p.num_results for p in poses), default=0)
    rows = np.full((len(poses), nmax), np.nan, np.float32)
    fbr = np.full(len(poses), -1, int)
    mx = np.full(len(poses), np.nan)
    alt = np.full(len(poses), np.nan)
    oom = 0
    for k, p in enumerate(poses):
        key = (p.side, p.range_mm, p.num_results, round(p.pulse_us, 3), p.gain)
        if key not in rcache:
            acq = _acq(cfg, p)
            side = Side.PORT if p.side == "port" else Side.STARBOARD
            rcache[key] = (acq, side, make(acq),
                           PingEncoder(side, acq, model),
                           (np.arange(acq.num_results) + 0.5) * acq.bin_size_m)
        acq, side, rr, enc, bins = rcache[key]
        pose = Pose3D(float(p.x), float(p.y), float(p.z), float(p.roll),
                      float(p.pitch), float(p.yaw))
        if pcache is None:
            pg = render(rr, side, pose)
        else:
            pk = (key, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)
            if pk not in pcache:
                pcache[pk] = render(rr, side, pose)
            pg = copy.deepcopy(pcache[pk])      # the noisy path replaces pg.power
        if pg.out_of_map_from_m is not None:
            oom += 1
        alt[k] = pg.altitude_m
        if mode == "noisy":
            pg.power = apply_ping_noise(pg.power, bins < pg.altitude_m, 1.0,
                                        noise_model(acq) if noise_model else model, rng,
                                        specular=pg.specular,
                                        texture=pg.texture,
                                        unoccluded=pg.unoccluded,
                                        ambient=pg.ambient,
                                        speckle_fill=pg.speckle_fill,
                                        floor_addition=floor_addition)
            e = enc.encode(pg)
            db = scale_to_db(e.pwr_results, e.min_pwr_db, e.max_pwr_db)
            mx[k] = float(e.max_pwr_db)
        else:
            db = _to_reported_db(expected_power(pg, model, bins), model, acq,
                                 p.gain)
            mx[k] = float(np.max(db))
        rows[k, :db.size] = db
        i = fbr_bin(np.asarray(db, dtype=np.float64))
        fbr[k] = -1 if i is None else int(i)
    return dict(rows=rows, fbr=fbr, max_db=mx, altitude=alt, out_of_map=oom)


def with_installation(cfg, *, sensor_depth_m: float | None = None,
                      mount_x_m: float | None = None,
                      mount_y_abs_m: float | None = None):
    """``cfg`` with a session's installation terms (transducer depth below the
    still waterline, fore/aft and lateral lever arm) replaced -- the acoustic
    model untouched. What a per-session ``installation`` artifact carries."""
    kw = {k: v for k, v in (("sensor_depth_m", sensor_depth_m),
                            ("mount_x_m", mount_x_m),
                            ("mount_y_abs_m", mount_y_abs_m)) if v is not None}
    return dataclasses.replace(cfg, model=dataclasses.replace(cfg.model, **kw))
