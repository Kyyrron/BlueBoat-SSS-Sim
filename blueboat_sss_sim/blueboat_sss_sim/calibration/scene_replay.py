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
                seed: int = 0) -> dict:
    """Render ``poses`` in ``scene`` with the sonar config ``cfg``.

    Returns ``rows`` (n, N) reported dB (float32; NaN-padded if the bin count
    varies), ``fbr`` (detector bin or -1), ``max_db`` (n,), ``altitude`` (the
    renderer's geometric altitude, m) and ``out_of_map`` (count of pings whose
    beam left the scene grid). Renderers are built once per acquisition and
    side, in the order the poses come, so a fixed ``seed`` is reproducible.
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
    rng = np.random.default_rng(seed)
    cache: dict = {}
    nmax = max((p.num_results for p in poses), default=0)
    rows = np.full((len(poses), nmax), np.nan, np.float32)
    fbr = np.full(len(poses), -1, int)
    mx = np.full(len(poses), np.nan)
    alt = np.full(len(poses), np.nan)
    oom = 0
    for k, p in enumerate(poses):
        key = (p.side, p.range_mm, p.num_results, round(p.pulse_us, 3), p.gain)
        if key not in cache:
            acq = _acq(cfg, p)
            side = Side.PORT if p.side == "port" else Side.STARBOARD
            cache[key] = (acq, side, GeometricRenderer(scene, acq, model),
                          PingEncoder(side, acq, model),
                          (np.arange(acq.num_results) + 0.5) * acq.bin_size_m)
        acq, side, rr, enc, bins = cache[key]
        pose = Pose3D(float(p.x), float(p.y), float(p.z), float(p.roll),
                      float(p.pitch), float(p.yaw))
        pg = rr.render(side, pose, 0.0).ping
        if pg.out_of_map_from_m is not None:
            oom += 1
        alt[k] = pg.altitude_m
        if mode == "noisy":
            pg.power = apply_ping_noise(pg.power, bins < pg.altitude_m, 1.0,
                                        model, rng, specular=pg.specular,
                                        texture=pg.texture,
                                        unoccluded=pg.unoccluded,
                                        ambient=pg.ambient,
                                        speckle_fill=pg.speckle_fill)
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
