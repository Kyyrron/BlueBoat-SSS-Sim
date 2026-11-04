"""The ray-cast sensor inside the field-calibration engine
(``calibration/seabed.py``, renderer hook): expected-mode levels through the
exact fit basis (``fitbasis.py``), noisy rows through full renders.

Renderers are cached per process by scene geometry, acquisition and the
geometry-relevant part of the ray-cast section; fit bases per (renderer,
side, pose). The fitted keys (``system_gain_db``, ``receiver.tvg_k``,
``beam.vertical_floor_amp``, ``seabed.sediments``) never enter a cache key:
they change no geometry. Full noisy renders are cached per pose too (the
engine's two seeds differ only in the shared noise stage).

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

import copy
import json
from types import SimpleNamespace

import numpy as np

from ...core.types import Pose3D, Side
from .config import RaycastConfig, shared_stage_model

_FITTED = (("system_gain_db",), ("receiver", "tvg_k"),
           ("beam", "vertical_floor_amp"), ("seabed", "sediments"),
           ("backend",))                  # the oracle backend changes no geometry
_RENDERERS: dict = {}
_BASES: dict = {}
_PINGS: dict = {}
_MAX_RENDERERS = 8


def _geometry_rc(rc: dict) -> dict:
    g = copy.deepcopy(dict(rc or {}))
    for path in _FITTED:
        node = g
        for k in path[:-1]:
            node = node.get(k, {}) if isinstance(node, dict) else {}
        if isinstance(node, dict):
            node.pop(path[-1], None)
    return g


def _acq_key(acq) -> tuple:
    return (acq.range_start_mm, acq.range_length_mm, acq.num_results,
            round(acq.pulse_len_percent, 9), acq.msec_per_ping)


def renderer_for(depth_m: float, half_m: float, acq, model, rc: dict):
    """A cached renderer on the engine's flat scene."""
    from ...calibration.seabed import flat_scene
    from .renderer import RaycastRenderer
    g = _geometry_rc(rc)
    key = (round(float(depth_m), 6), round(float(half_m), 3), _acq_key(acq),
           json.dumps(g, sort_keys=True), round(float(model.sensor_depth_m), 6),
           round(float(model.beam_tilt_deg), 6), round(float(model.mount_x_m), 6),
           round(float(model.mount_y_abs_m), 6))
    rr = _RENDERERS.get(key)
    if rr is None:
        if len(_RENDERERS) >= _MAX_RENDERERS:
            old = next(iter(_RENDERERS))
            _RENDERERS.pop(old)                  # bases hold arrays only: kept
            for k in [k for k in _PINGS if k[0] == old]:
                _PINGS.pop(k)
        # the backend is no part of the key (it changes no geometry) but the
        # renderer is built with the one asked for
        base = RaycastConfig.from_dict({**g, "backend": (rc or {}).get("backend", "cpu")})
        m = shared_stage_model(model, base.system_gain_db)
        rr = RaycastRenderer(flat_scene(depth_m, half_m), acq, m, base)
        _RENDERERS[key] = rr
    return key, rr


def _stored_basis(bk, build):
    """A basis from the on-disk store ``$BLUEBOAT_RAYCAST_BASES`` (keyed by
    a hash of its geometry key), built and saved when absent; built only,
    without the variable."""
    import hashlib
    import os
    from .fitbasis import load_basis, save_basis
    root = os.environ.get("BLUEBOAT_RAYCAST_BASES")
    if not root:
        return build()
    h = hashlib.sha256(repr(bk).encode()).hexdigest()[:24]
    path = os.path.join(os.path.expanduser(root), h + ".npz")
    if os.path.exists(path):
        return load_basis(path)
    b = build()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".part.npz"
    save_basis(b, tmp)
    os.replace(tmp, path)
    return b


def ringing(model, gain_db: float, r_dev: np.ndarray):
    """The measured transmit ringing in this sensor's linear units (its
    reported level stays the measured one for any system gain)."""
    if model.ringing_level_db <= 0.0:
        return None
    ring_db = model.ringing_level_db - model.ringing_decay_db_per_m * r_dev
    return 10.0 ** ((ring_db - float(gain_db)) / 10.0)


def expected_level(cfg, acq, side_name: str, alt: float, gain: int, replay,
                   rc: dict) -> dict:
    """The engine's ``_render_expected`` for the ray-cast sensor: the mean
    reported dB over the replay poses, from the fit basis."""
    from ...analysis.calibration import fbr_bin
    from ...calibration.seabed import _to_reported_db, expected_power
    from .fitbasis import build_basis, law_and_surface
    rcfg = RaycastConfig.from_dict(rc)
    model = shared_stage_model(cfg.model, rcfg.system_gain_db)
    half = acq.range_length_mm / 1000.0 + 5.0
    key, rr = renderer_for(alt + cfg.model.sensor_depth_m, half, acq, cfg.model, rc)
    bins = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
    if replay is None:
        poses = [(Side.PORT, Pose3D(0.0, 0.0, 0.0))]
    else:
        side = Side(side_name)
        poses = [(side, Pose3D(0.0, 0.0, float(dz), float(r), float(p), 0.0))
                 for r, p, dz in zip(*replay)]
    law, surf = law_and_surface(rr, rcfg.seabed.sediments)
    spec = ringing(model, rcfg.system_gain_db, bins)
    acc = np.zeros(acq.num_results)
    level_db0 = None
    for side, pose in poses:
        bk = (key, side.value, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)
        b = _BASES.get(bk)
        if b is None:
            b = _BASES[bk] = _stored_basis(bk, lambda: build_basis(rr, side, pose))
        p = b.power(law, surf, rcfg.beam.vertical_floor_amp, rcfg.receiver.tvg_k)
        pg = SimpleNamespace(power=p, specular=spec, unoccluded=None, ambient=None,
                             altitude_m=b.altitude_m)
        db = _to_reported_db(expected_power(pg, model, bins), model, acq, gain)
        if level_db0 is None:
            level_db0 = db
        acc += 10.0 ** (db / 10.0)
    level = 10.0 * np.log10(acc / len(poses))
    i = fbr_bin(level_db0) if replay is None else fbr_bin(level)
    return dict(level=level, fbr_alt=float("nan") if i is None else i * acq.bin_size_m)


def noisy_renderer(cfg, acq, scene_depth: float, half: float, rc: dict):
    """(renderer, shared-stage model, ping cache key) for the engine's noisy
    mode: a renderer with the full (fitted) ray-cast section."""
    from ...calibration.seabed import flat_scene
    from .renderer import RaycastRenderer
    rcfg = RaycastConfig.from_dict(rc)
    model = shared_stage_model(cfg.model, rcfg.system_gain_db)
    key = ("noisy", round(scene_depth, 6), round(half, 3), _acq_key(acq),
           json.dumps(rc, sort_keys=True))
    hit = _RENDERERS.get(key)
    if hit is None:
        if len(_RENDERERS) >= _MAX_RENDERERS:
            old = next(iter(_RENDERERS))
            _RENDERERS.pop(old)
            for k in [k for k in _PINGS if k[0] == old]:
                _PINGS.pop(k)
        hit = _RENDERERS[key] = RaycastRenderer(flat_scene(scene_depth, half), acq,
                                                model, rcfg)
    return hit, model, key


def cached_render(rr, key, side: Side, pose: Pose3D):
    """A full render, cached per pose (the engine's seeds share it)."""
    pk = (key, side.value, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)
    pg = _PINGS.get(pk)
    if pg is None:
        pg = _PINGS[pk] = rr.render(side, pose, 0.0).ping
    return copy.copy(pg)
