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
           ("backend",),                  # the oracle backend changes no geometry
           ("seabed", "law"), ("seabed", "lambert_mu_db"),   # the law: applied in power(), not stored
           ("correction",))   # the empirical layer: applied after the basis (expected_level), never stored
_RENDERERS: dict = {}
_BASES: dict = {}
_PINGS: dict = {}
_MAX_RENDERERS = 8
#: noisy-mode pings kept (a 1200-bin ping with its texture is ~50 kB)
_MAX_PINGS = 4000


def _prune_empty(d):
    """``d`` without empty sub-mappings (recursively)."""
    if not isinstance(d, dict):
        return d
    out = {}
    for k, v in d.items():
        v = _prune_empty(v)
        if isinstance(v, dict) and not v:
            continue
        out[k] = v
    return out


def _geometry_rc(rc: dict) -> dict:
    """The geometry part of a ray-cast section: the fitted keys removed, and
    the sub-mappings they leave empty too -- whether a fitted key's parent
    exists says nothing about the geometry (2026-11-04: ``set_params`` adds
    ``beam: {vertical_floor_amp}``, so the fit's keys differed from the
    builder's for the same scene and every basis was built twice)."""
    g = copy.deepcopy(dict(rc or {}))
    for path in _FITTED:
        node = g
        for k in path[:-1]:
            node = node.get(k, {}) if isinstance(node, dict) else {}
        if isinstance(node, dict):
            node.pop(path[-1], None)
    return _prune_empty(g)


def _acq_key(acq) -> tuple:
    return (acq.range_start_mm, acq.range_length_mm, acq.num_results,
            round(acq.pulse_len_percent, 9), acq.msec_per_ping)


def renderer_key(depth_m: float, half_m: float, acq, model, rc: dict) -> tuple:
    """The cache key of ``renderer_for`` (geometry only)."""
    g = _geometry_rc(rc)
    return (round(float(depth_m), 6), round(float(half_m), 3), _acq_key(acq),
            json.dumps(g, sort_keys=True), round(float(model.sensor_depth_m), 6),
            round(float(model.beam_tilt_deg), 6), round(float(model.mount_x_m), 6),
            round(float(model.mount_y_abs_m), 6))


def renderer_for(depth_m: float, half_m: float, acq, model, rc: dict):
    """A cached renderer on the engine's flat scene."""
    from ...calibration.seabed import flat_scene
    from .renderer import RaycastRenderer
    g = _geometry_rc(rc)
    key = renderer_key(depth_m, half_m, acq, model, rc)
    rr = _RENDERERS.get(key)
    if rr is None:
        if len(_RENDERERS) >= _MAX_RENDERERS:
            old = next(iter(_RENDERERS))
            _RENDERERS.pop(old)                  # bases hold arrays only: kept
            for k in [k for k in _PINGS if k[0] == old]:
                _PINGS.pop(k)
            import gc
            gc.collect()                         # renderers are reference cycles
        # the backend is no part of the key (it changes no geometry) but the
        # renderer is built with the one asked for
        base = RaycastConfig.from_dict({**g, "backend": (rc or {}).get("backend", "cpu")})
        m = shared_stage_model(model, base.system_gain_db)
        rr = RaycastRenderer(flat_scene(depth_m, half_m), acq, m, base)
        _RENDERERS[key] = rr
    return key, rr


#: Multipath correction of the expected seabed power (Phase A attempt 2,
#: 2026-11-05): per (renderer key, side) a per-bin factor P_full / P_direct
#: measured by full renders at the site's sea-surface roughness (the fit
#: bases hold the direct paths only; ``build_basis`` refuses kept chains).
#: Gain and TVG cancel in the ratio; empty = no correction (the default).
_MP_CORR: dict = {}
MP_HITS = {"hit": 0, "miss": 0}       # poses served with / without a correction while any is set


def set_multipath_correction(key, side: str, factor) -> None:
    """Install (or, with ``factor`` None, remove) a multipath correction."""
    if factor is None:
        _MP_CORR.pop((key, side), None)
    else:
        _MP_CORR[(key, side)] = np.asarray(factor, dtype=np.float64)


#: per renderer key, what ``fitbasis.law_and_surface`` reads of a renderer
#: (the law settings, material names, k, frequency, sound speed): the fit
#: needs a full renderer only to build a missing basis (2026-11-04: with 8
#: cached renderers and ~150 strata every evaluation rebuilt every flat mesh)
_LAWCTX: dict = {}


#: the law and interface per (law context, sediments): every stratum of one
#: fit evaluation shares the parameter set, so they share one law (and its
#: sigma-node memo, ``PingBasis.power``)
_LAWS: dict = {}
_MAX_LAWS = 4


def _law_for(key, sediments):
    from .fitbasis import law_and_surface
    ctx = _LAWCTX[key]
    lk = (json.dumps(sediments or {}, sort_keys=True, default=repr), repr(ctx._rc.seabed),
          tuple(ctx.mesh.material_names), ctx.k, ctx.freq_hz, ctx.c)
    hit = _LAWS.get(lk)
    if hit is None:
        if len(_LAWS) >= _MAX_LAWS:
            _LAWS.pop(next(iter(_LAWS)))
        hit = _LAWS[lk] = law_and_surface(ctx, sediments)
    return hit


def _law_context(rr) -> SimpleNamespace:
    return SimpleNamespace(_rc=SimpleNamespace(seabed=copy.deepcopy(rr._rc.seabed)),
                           mesh=SimpleNamespace(material_names=list(rr.mesh.material_names)),
                           k=rr.k, freq_hz=rr.freq_hz, c=rr.c)


#: how a transient device out-of-memory surfaces from Warp: an allocation
#: failure, or a CUDA error raised right after one (Warp's scan reports a
#: failed temporary allocation as "error 101: invalid device ordinal",
#: preceded by "error 2: out of memory" on stderr; 2026-11-04)
_DEVICE_OOM = ("Failed to allocate", "out of memory", "CUDA error 2:", "CUDA error 101:")


def _build_retrying(build, release, attempts: int = 12):
    """``build()``, retried when the device is momentarily out of memory
    (another GPU process): release this renderer's buffers, wait, retry,
    each retry logged. A multi-hour run must not die on a transient
    allocation (2026-11-04, three times)."""
    import gc
    import logging
    import time
    for k in range(attempts):
        try:
            return build()
        except RuntimeError as e:
            if not any(m in str(e) for m in _DEVICE_OOM) or k == attempts - 1:
                raise
            logging.getLogger(__name__).warning(
                "device out of memory (%s): retry %d of %d after releasing buffers", e, k + 1,
                attempts - 1)
            release()
            gc.collect()
            time.sleep(min(30.0 * (k + 1), 300.0))


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
    from .fitbasis import build_basis, compact_basis
    rcfg = RaycastConfig.from_dict(rc)
    model = shared_stage_model(cfg.model, rcfg.system_gain_db)
    half = acq.range_length_mm / 1000.0 + 5.0
    depth = alt + cfg.model.sensor_depth_m
    key = renderer_key(depth, half, acq, cfg.model, rc)
    holder: dict = {}

    def rr_(dz: float = 0.0):
        k_ = key if dz == 0.0 else renderer_key(depth + dz, half, acq, cfg.model, rc)
        if k_ not in holder:
            holder[k_] = renderer_for(depth + dz, half, acq, cfg.model, rc)[1]
            _LAWCTX.setdefault(k_, _law_context(holder[k_]))
        return holder[k_]
    if key not in _LAWCTX:
        rr_()
    bins = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
    if replay is None:
        poses = [(Side.PORT, Pose3D(0.0, 0.0, 0.0), 0.0)]
    else:
        # seabed-relative replay (2026-11-06): the replayed dz is the
        # recording's measured altitude variation (FBR altitude - median:
        # bathymetry and heave). This sensor has a sea surface, so dz moves
        # the SEABED (a flat scene at depth + dz) and the transducer stays at
        # its depth below the surface. Moving the vehicle by dz instead lifted
        # the transducer (0.22 m deep) toward and out of the water: Lloyd's-
        # mirror cancellation near the surface, an empty ping above it
        # (7 of 40 pings of a harbour validation block). The geometric
        # renderer has no surface: there the two conventions are identical.
        side = Side(side_name)
        poses = [(side, Pose3D(0.0, 0.0, 0.0, float(r), float(p), 0.0), float(dz))
                 for r, p, dz in zip(*replay)]
    law, surf = _law_for(key, rcfg.seabed.sediments)
    from .correction import for_config as _correction_for
    corr_layer = _correction_for(rcfg.correction)
    if corr_layer is not None:
        corr_layer.check_core(rcfg, cfg.model,
                              accept_engine=bool(rcfg.correction.accept_engine_knobs))
    spec = ringing(model, rcfg.system_gain_db, bins)
    acc = np.zeros(acq.num_results)
    level_db0 = None
    for side, pose, dz in poses:
        kz = key if dz == 0.0 else renderer_key(depth + dz, half, acq, cfg.model, rc)
        bk = (kz, side.value, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)
        b = _BASES.get(bk)
        if b is None:
            b = _stored_basis(bk, lambda: _build_retrying(
                lambda: build_basis(rr_(dz), side, pose), lambda: rr_(dz).release_device()))
            b = _BASES[bk] = compact_basis(b)     # power() bit for bit, less memory
        p = b.power(law, surf, rcfg.beam.vertical_floor_amp, rcfg.receiver.tvg_k)
        corr = _MP_CORR.get((key, side.value))
        if corr is not None:
            p = p * corr
            MP_HITS["hit"] += 1
        elif _MP_CORR:
            MP_HITS["miss"] += 1
        if corr_layer is not None:
            # the empirical correction layer (correction.py), as the renderer
            # applies it: the engine's flat scene is sand seabed under nadir
            p = corr_layer.apply(p, bins, b.altitude_m, acq, nadir_material="sand",
                                 nadir_is_seabed=True,
                                 max_ping_rate_hz=cfg.model.max_ping_rate_hz)
        pg = SimpleNamespace(power=p, specular=spec, unoccluded=None, ambient=None,
                             altitude_m=b.altitude_m)
        db = _to_reported_db(expected_power(pg, model, bins), model, acq, gain)
        if level_db0 is None:
            level_db0 = db
        acc += 10.0 ** (db / 10.0)
    if holder:
        # nothing of the renderers is needed any more (the law context is
        # cached apart): keep neither their device buffers nor their ~0.3 GB
        # of host arrays (the fit holds every basis at once; OOM 2026-11-04)
        for k_, r_ in holder.items():
            r_.release_device()
            _RENDERERS.pop(k_, None)
        holder.clear()
        # a renderer is a reference cycle (renderer <-> multipath): without a
        # collection the popped ones stay resident (~0.2 GB each at 40 m; 63
        # of them were the fit's missing 6 GB, OOM 2026-11-04)
        import gc
        gc.collect()
    level = 10.0 * np.log10(acc / len(poses))
    i = fbr_bin(level_db0) if replay is None else fbr_bin(level)
    return dict(level=level, fbr_alt=float("nan") if i is None else i * acq.bin_size_m)


def _physical(rc) -> dict:
    """A ray-cast section without its empirical ``correction``."""
    return {k: v for k, v in dict(rc or {}).items() if k != "correction"}


def noisy_key(acq, scene_depth: float, half: float, rc: dict, material: str = "sand") -> tuple:
    """The cache key of ``noisy_renderer`` (and of its pings), computed
    without building the renderer."""
    return ("noisy", round(scene_depth, 6), round(half, 3), _acq_key(acq),
            json.dumps(_physical(rc), sort_keys=True), material)


def noisy_model(cfg, rc: dict):
    """The shared-stage model ``noisy_renderer`` returns, without the renderer."""
    return shared_stage_model(cfg.model, RaycastConfig.from_dict(_physical(rc)).system_gain_db)


def noisy_renderer(cfg, acq, scene_depth: float, half: float, rc: dict,
                   material: str = "sand"):
    """(renderer, shared-stage model, ping cache key) for the engine's noisy
    mode: a renderer with the full (fitted) ray-cast section, on a flat
    bottom of ``material``. The renderer is the PHYSICAL one: the section's
    ``correction`` is no part of its key or its construction -- the
    empirical layer is applied after the cached render
    (``cached_render(..., correction=)``, as ``expected_level`` applies it
    after the basis), so a physical and a corrected arm share every render
    and its texture."""
    from ...calibration.seabed import flat_scene
    from .renderer import RaycastRenderer
    rc = _physical(rc)
    rcfg = RaycastConfig.from_dict(rc)
    model = shared_stage_model(cfg.model, rcfg.system_gain_db)
    key = noisy_key(acq, scene_depth, half, rc, material)
    hit = _RENDERERS.get(key)
    if hit is None:
        if len(_RENDERERS) >= _MAX_RENDERERS:
            # the evicted renderer's pings stay cached (keyed by its scene):
            # the seabed-relative replay alternates between more depths than
            # the cache holds, and the second seed must find its pings
            _RENDERERS.pop(next(iter(_RENDERERS)))
            import gc
            gc.collect()                         # renderers are reference cycles
        while len(_PINGS) > _MAX_PINGS:
            _PINGS.pop(next(iter(_PINGS)))       # oldest first
        hit = _RENDERERS[key] = RaycastRenderer(flat_scene(scene_depth, half, material=material),
                                                acq, model, rcfg)
    # device buffers for the renderer in use only: the cache keeps up to 8,
    # a 40 m flat scene's scatterers alone are ~0.4 GB of an 8 GB device
    for k, r in _RENDERERS.items():
        if k != key and getattr(r, "_gpu", None) is not None:
            r.release_device()
    return hit, model, key


def correction_for_rc(rc: dict):
    """The empirical correction a ray-cast section enables, or None."""
    from .correction import for_config
    return for_config(RaycastConfig.from_dict(rc).correction)


#: (renderer key, artifact source, version) whose physical core was checked
_CORE_OK: set = set()


def cached_render(rr, key, side: Side, pose: Pose3D, correction=None):
    """A full render, cached per pose (the engine's seeds share it). ``rr``
    is the renderer or a zero-argument callable returning it, called only on
    a cache miss: a corrected arm replaying a physical arm's pings builds no
    renderer. With a ``correction`` (``correction_for_rc``) the empirical
    layer is applied to the returned copy exactly as the renderer applies it
    (its domain gate on the transducer's nadir included, from the context
    stored with the ping); the cached physical ping is never modified."""
    pk = (key, side.value, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)
    hit = _PINGS.get(pk)
    if hit is None:
        r = rr() if callable(rr) else rr
        pg = _build_retrying(lambda: r.render(side, pose, 0.0).ping, r.release_device)
        # what the correction needs later, without the renderer: its core,
        # acquisition and the surface straight below the transducer
        hit = _PINGS[pk] = (pg, (r._rc, r._cfg, r._acq, r.nadir_below(pg.pose)))
    pg, (rc_, model_, acq_, (seabed, mat)) = hit
    out = copy.copy(pg)
    if correction is not None:
        from .correction import bin_ranges
        ck = (key, correction.source, correction.version)
        if ck not in _CORE_OK:
            correction.check_core(rc_, model_, accept_engine=bool(getattr(
                getattr(rc_, "correction", None), "accept_engine_knobs", False)))
            _CORE_OK.add(ck)
        out.power, out.correction_status = correction.correct(
            pg.power, bin_ranges(acq_), pg.altitude_m, acq_, nadir_material=mat,
            nadir_is_seabed=seabed, max_ping_rate_hz=model_.max_ping_rate_hz)
        out.power_uncorrected = pg.power
    return out
