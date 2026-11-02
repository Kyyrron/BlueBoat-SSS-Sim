"""Seabed scoring and fitting against field recordings.

The engine behind ``python3 -m blueboat_sss_sim.calibration score`` and
``... fit`` (2026-10-24). One reduction is applied to real and simulated
pings alike (the ``analysis.calibration`` principle: one reduction, both
sides of the pair).

**Strata.** Every gain-``g`` ping of a recording is decoded to reported dB
(``scale_to_db``), its first bottom return found by the downstream detector
(``fbr_bin``) and its altitude taken as ``FBR bin x L/N``. Pings group by
(file, side, range, bins, pulse) x altitude band (0.2 m below 3 m, 1.0 m
above); a stratum needs :data:`MIN_PINGS`. A stratum is **core** when its
median altitude lies inside the file's own FBR altitude p10-p90 (detector
misfires and the ends of a file's altitude range are scored but kept out of
the session summaries). The file's range is per side, over every ping the
detector finds (any gain), widened by :data:`CORE_MARGIN_M` -- the pass-1
summaries' definition.

**Forward model.** Flat sand (reflectivity 0.55) at the stratum's altitude
(matched once on the simulated FBR, as the pass-1 scorer did), the stratum's
own acquisition (range, bins, and ``pulse_len_percent`` giving its pulse),
gain ``g``, the profile under test. Two modes:

* ``noisy`` -- the node's noise call and the wire encoder (``PingEncoder``),
  decoded and reduced like the real pings; :data:`N_SIM` pings per seed,
  seeds :data:`SEEDS`. Everything (crest, texture, level) can be scored.
* ``expected`` -- the noise-free mean intensity: ``power + specular`` plus
  the mean of the floors ``E[max(aX, bY)] = a + b - ab/(a + b)`` for
  independent Exp(1) ``X, Y`` (receiver floor ``a``, reverberation floor
  ``b``), plus the water-column mean; deterministic and smooth in the
  model's constants, so it is what the level fit evaluates. Texture, jitter
  and gain drift are unit-mean and drop out. It scores the level and the
  nadir profile only.

**Replay.** ``none`` renders a level boat on a straight line (pass 1).
``attitude`` renders each simulated ping at a real ping's roll and pitch
(ATTITUDE, paired on the profile clock -- or by file order on a frozen
clock -- like ``analysis.backscatter``) and its FBR altitude about the
stratum median, on the side the stratum was recorded on. MAVLink attitude
is FRD; the renderer's body frame is FLU, so ``roll = roll``,
``pitch = -pitch``.

**Metrics** per stratum (positive = simulator brighter): seabed LEVEL bias
(dB of the power average, tail ``[max(1.4 alt, 1 m), min(0.725 R, R - 1 m)]``
where the real dB mean >= 0 dB) and its shape RMS, block-bootstrap CI and
sub-bands; the dB-mean bias; power-minus-dB and per-bin std (real and sim);
texture lag-1 and high-pass std; the FBR crest (median of the per-ping max
dB in ``[FBR - 2 bins, FBR + max(1 m, alt/2)]``) and leading edge
(``[FBR - 2, FBR + 4 bins]``); FBR detection rate; the nadir profile; the
reported maximum against the corpus law ``66.60 - 11.28 log10 alt``.

**Fit.** Least squares (scipy, imported lazily) of registered tunable
parameters within their registry bounds, on ``expected``-mode residuals of
the level tail or the nadir profile, every stratum weighted equally, the
rendered altitudes frozen at the starting point so the residual is smooth.
Validation and test sessions are refused unless the caller passes a reason
(``allow_reuse``), which is recorded. The output is a report and a DRAFT
``blueboat_calibration_artifact/1`` holding fitted values only, with the
base profile it was fitted on written next to it.

Read-only on recordings; nothing is written inside a recording's directory
(``io_guard``). No position fix or track leaves this module: strata carry
dB rows and attitude only.
"""

from __future__ import annotations

import dataclasses
import datetime as _dt
import hashlib
import json
import math
import os
import time
from concurrent.futures import Executor, ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml

SCORES_FORMAT = "blueboat_seabed_scores/1"
FIT_FORMAT = "blueboat_seabed_fit/1"
#: Bump when the real-side reduction changes (invalidates the cache).
REDUCTION_VERSION = 3

MIN_PINGS = 100
N_SIM = 120
N_EXPECTED = 24
SEEDS = (7, 8)
SIGNAL_DB = 0.0
PEAK_LAW = (66.60, -11.28)
BOOT = 300
BLOCK = 50
CREST_MIN_M = 1.0
CREST_ALT_FRACTION = 0.5
LEADING_BINS = 4
REGIME_SPLIT_M = 3.0
NADIR_WINDOWS_M = ((0.0, 0.15), (0.45, 0.55), (0.95, 1.05))
SAND_REFLECTIVITY = 0.55
SPEED_OF_SOUND = 1500.0
SUB_BAND_EDGES = (3, 5, 10, 15, 20, 30, 40, 60, 80, 130)
CORE_MARGIN_M = 0.1
#: Nadir-fit window: slant ranges up to min(this, 0.7 x altitude).
NADIR_MAX_M = 0.6


def band_of(alt: float) -> float:
    w = 0.2 if alt < REGIME_SPLIT_M else 1.0
    return round(math.floor(alt / w) * w, 3)


def regime_of(alt: float) -> str:
    return "low (<3 m)" if alt < REGIME_SPLIT_M else "high (>=3 m)"


# ===================================================================== real
@dataclass
class Stratum:
    """One (file, side, acquisition, altitude band) group of real pings."""

    session: str
    role: str
    file: str
    side: str
    range_mm: int
    num_results: int
    pulse_us: float
    gain: int
    band: float
    alt: float
    rows: np.ndarray                      # (n, N) reported dB, float32
    max_db: np.ndarray                    # (n,)
    fbr: np.ndarray                       # (n,) detector bin
    alts: np.ndarray                      # (n,) FBR altitude [m]
    roll: np.ndarray | None               # (n,) renderer convention [rad]
    pitch: np.ndarray | None
    step_m: float
    speed_mps: float | None
    core: bool
    index: np.ndarray | None = None       # (n,) profile index in the file

    @property
    def n(self) -> int:
        return int(self.rows.shape[0])

    @property
    def bin_m(self) -> float:
        return self.range_mm / 1000.0 / self.num_results

    @property
    def key(self) -> tuple:
        return (self.file, self.side, self.range_mm, self.num_results,
                self.pulse_us, self.gain, self.band)

    def label(self) -> str:
        return (f"{self.file}:{self.side}:{self.range_mm // 1000}m/"
                f"{self.num_results}/{self.pulse_us:g}us@{self.band:g}m")


def _attitude_arrays(st, profiles) -> tuple[np.ndarray, np.ndarray] | None:
    """Per-profile (roll, pitch) in the renderer's FLU convention, or None."""
    from ..analysis.svlog_reader import pair_by_order
    att = st.attitude
    if not att:
        return None
    a_roll = np.array([a.roll for a in att], dtype=np.float64)
    a_pitch = np.array([a.pitch for a in att], dtype=np.float64)
    if st.clock_frozen:
        j = pair_by_order(np.array([p.order for p in profiles]),
                          np.array([a.order for a in att]))
        return a_roll[j], -a_pitch[j]
    t = np.array([a.time_boot_ms + st.skew_ms for a in att], dtype=np.float64)
    o = np.argsort(t, kind="stable")
    pt = np.array([p.timestamp_ms for p in profiles], dtype=np.float64)
    return (np.interp(pt, t[o], a_roll[o]), -np.interp(pt, t[o], a_pitch[o]))


def reduce_recording(path: str | Path, *, rel: str, session: str, role: str,
                     gain: int = 4, min_pings: int = MIN_PINGS
                     ) -> tuple[list[Stratum], dict]:
    """Group a recording's gain-``gain`` pings into strata (module doc)."""
    from ..analysis.calibration import fbr_bin, scale_to_db
    from ..analysis.svlog_reader import read_svlog
    st = read_svlog(path, with_power=True)
    profiles = st.profiles
    spd = [math.hypot(g.vx_mps, g.vy_mps) for g in st.gps]
    speed = float(np.median(spd)) if spd and np.median(spd) > 0.05 else None
    att = _attitude_arrays(st, profiles)
    groups: dict[tuple, list[int]] = {}
    dbs: dict[int, np.ndarray] = {}
    fbrs: dict[int, int] = {}
    tried = {"port": 0, "starboard": 0}
    found = {"port": 0, "starboard": 0}
    side_alts: dict[str, list[float]] = {"port": [], "starboard": []}
    for k, p in enumerate(profiles):
        if p.pwr_results is None or not p.num_results:
            continue
        db = scale_to_db(p.pwr_results, p.min_pwr_db, p.max_pwr_db)
        i = fbr_bin(db)
        if i is not None:
            side_alts[p.side.value].append(
                p.start_mm / 1000.0 + i * p.length_mm / 1000.0 / p.num_results)
        if int(p.gain_index) != gain:
            continue
        tried[p.side.value] += 1
        if i is None:
            continue
        found[p.side.value] += 1
        alt = i * p.length_mm / 1000.0 / p.num_results
        key = (p.side.value, int(p.length_mm), int(p.num_results),
               round(float(p.pulse_duration_sec) * 1e6, 1), band_of(alt))
        groups.setdefault(key, []).append(k)
        dbs[k] = db.astype(np.float32)
        fbrs[k] = int(i)
    core_range = {s: ((float(np.percentile(a, 10)) - CORE_MARGIN_M,
                       float(np.percentile(a, 90)) + CORE_MARGIN_M)
                      if a else (np.nan, np.nan))
                  for s, a in side_alts.items()}
    period: dict[tuple, float | None] = {}
    for side, L in {(g[0], g[1]) for g in groups}:
        ts = np.sort([p.timestamp_ms for p in profiles
                      if p.side.value == side and p.length_mm == L])
        dt = np.diff(ts.astype(float))
        dt = dt[(dt > 0) & (dt < 2000)]
        period[(side, L)] = float(np.median(dt)) / 1000.0 if dt.size else None
    strata: list[Stratum] = []
    for key, idx in sorted(groups.items()):
        if len(idx) < min_pings:
            continue
        side, L, N, pulse_us, band = key
        idx = sorted(idx)
        alts = np.array([fbrs[k] * L / 1000.0 / N for k in idx])
        alt = float(np.median(alts))
        per = period.get((side, L))
        step = (speed * per) if (speed and per) else 0.05
        strata.append(Stratum(
            session=session, role=role, file=rel, side=side, range_mm=L,
            num_results=N, pulse_us=pulse_us, gain=gain, band=band, alt=alt,
            rows=np.stack([dbs[k] for k in idx]),
            max_db=np.array([profiles[k].max_pwr_db for k in idx]),
            fbr=np.array([fbrs[k] for k in idx]), alts=alts,
            roll=None if att is None else att[0][idx].astype(np.float32),
            pitch=None if att is None else att[1][idx].astype(np.float32),
            step_m=float(step), speed_mps=speed,
            core=bool(core_range[side][0] <= alt <= core_range[side][1]),
            index=np.array(idx, dtype=np.int64)))
    meta = dict(
        speed_mps=speed, clock_frozen=bool(st.clock_frozen),
        attitude=bool(att is not None),
        attitude_std_deg=(None if att is None else
                          [round(float(np.degrees(np.std(att[0]))), 2),
                           round(float(np.degrees(np.std(att[1]))), 2)]),
        fbr_detection={s: (round(found[s] / tried[s], 4) if tried[s] else None)
                       for s in tried},
        core_alt_range_m={s: [round(v[0], 3), round(v[1], 3)]
                          for s, v in core_range.items()},
        strata=len(strata))
    return strata, meta


_ARRAYS = ("rows", "max_db", "fbr", "alts", "roll", "pitch", "index")


def _cache_key(path: Path, gain: int) -> str:
    s = path.stat()
    h = hashlib.sha256(f"{path.resolve()}|{s.st_size}|{s.st_mtime_ns}|{gain}|"
                       f"{REDUCTION_VERSION}|{MIN_PINGS}".encode())
    return h.hexdigest()[:20]


def _save_cached(cache: Path, key: str, strata: list[Stratum], meta: dict
                 ) -> None:
    cache.mkdir(parents=True, exist_ok=True)
    arrays: dict[str, np.ndarray] = {}
    head = []
    for i, s in enumerate(strata):
        d = dataclasses.asdict(s)
        for a in _ARRAYS:
            v = d.pop(a)
            if v is not None:
                arrays[f"{i}_{a}"] = v
        head.append(d)
    tmp = cache / f".{key}.partial.npz"
    np.savez_compressed(tmp, header=np.array(json.dumps(
        {"strata": head, "meta": meta})), **arrays)
    os.replace(tmp, cache / f"{key}.npz")


def _load_cached(cache: Path, key: str) -> tuple[list[Stratum], dict] | None:
    f = cache / f"{key}.npz"
    if not f.exists():
        return None
    with np.load(f) as z:
        hdr = json.loads(str(z["header"]))
        out = []
        for i, d in enumerate(hdr["strata"]):
            for a in _ARRAYS:
                k = f"{i}_{a}"
                d[a] = z[k] if k in z.files else None
            out.append(Stratum(**d))
    return out, hdr["meta"]


def _reduce_job(job: dict) -> tuple[str, list[Stratum], dict]:
    path = Path(job["path"])
    cache = Path(job["cache"]) if job.get("cache") else None
    key = _cache_key(path, job["gain"])
    if cache is not None:
        got = _load_cached(cache, key)
        if got is not None:
            strata, meta = got
            for s in strata:          # roles live in the manifest, not cache
                s.session, s.role = job["session"], job["role"]
            return job["rel"], strata, meta
    strata, meta = reduce_recording(path, rel=job["rel"],
                                    session=job["session"], role=job["role"],
                                    gain=job["gain"])
    if cache is not None:
        _save_cached(cache, key, strata, meta)
    return job["rel"], strata, meta


def manifest_recordings(manifest: Mapping[str, Any],
                        roles: Sequence[str] = ("calibration", "validation"),
                        sessions: Sequence[str] | None = None
                        ) -> list[tuple[str, str, str]]:
    """``(session, role, relative path)`` of every scored ``.svlog``:
    excluded recordings (merged duplicates) never, simulated sessions only
    when their role is asked for, and never a session whose
    ``regime.forward_model`` is not ``flat_sand`` (pass 3: the pool is scored
    against its own world, not against flat sand at the stratum altitude)."""
    from .manifest import forward_model
    out = []
    for s in manifest.get("sessions") or []:
        if s.get("role") not in roles:
            continue
        if forward_model(s) != "flat_sand":
            continue
        if sessions and s["id"] not in sessions:
            continue
        for r in s.get("recordings") or []:
            if r.get("kind") == "svlog" and not r.get("exclude"):
                out.append((str(s["id"]), str(s["role"]), str(r["path"])))
    return out


def load_strata(manifest: Mapping[str, Any], root: Path, *,
                roles: Sequence[str] = ("calibration", "validation"),
                sessions: Sequence[str] | None = None, gain: int = 4,
                cache: Path | None = None, executor: Executor | None = None
                ) -> tuple[list[Stratum], dict]:
    jobs = [dict(path=str(root / rel), rel=rel, session=sid, role=role,
                 gain=gain, cache=str(cache) if cache else None)
            for sid, role, rel in manifest_recordings(manifest, roles,
                                                      sessions)]
    run = executor.map if executor is not None else map
    strata: list[Stratum] = []
    meta: dict[str, dict] = {}
    for rel, st, m in run(_reduce_job, jobs):
        strata.extend(st)
        meta[rel] = m
    return strata, meta


# ====================================================================== sim
def acquisition_for(cfg, range_mm: int, num_results: int, pulse_us: float,
                    gain: int):
    """The stratum's acquisition: its range and bins, and the
    ``pulse_len_percent`` that gives its pulse (the 44.3 us floor below)."""
    pct = ((pulse_us * 1e-6) / (2.0 * range_mm / 1000.0 / SPEED_OF_SOUND)
           if pulse_us > 44.4 else 0.00125)
    return dataclasses.replace(cfg.acquisition, range_length_mm=int(range_mm),
                               num_results=int(num_results),
                               gain_index=int(gain), range_start_mm=0,
                               pulse_len_percent=pct)


def flat_scene(depth_m: float, half_m: float, res: float = 0.1,
               reflectivity: float = SAND_REFLECTIVITY):
    from ..core.types import GridSpec
    from ..worldgen.scene import SceneModel
    n = int(round(2 * half_m / res)) + 1
    g = GridSpec(origin_x=-half_m, origin_y=-half_m, resolution=res, nx=n,
                 ny=n)
    return SceneModel(grid=g, height=np.full((n, n), -float(depth_m)),
                      reflectivity=np.full((n, n), reflectivity),
                      material_id=np.zeros((n, n), dtype=np.uint8),
                      material_names=["sand"], objects=[], walls=[], seed=0,
                      config_snapshot={})


def expected_power(ping, model, bins: np.ndarray) -> np.ndarray:
    """Mean intensity of the node's noisy ping (``apply_ping_noise``) at
    unit gain: every multiplicative term is unit-mean, the two floors are
    the mean of a max of independent exponentials (module doc)."""
    p = np.asarray(ping.power, dtype=np.float64).copy()
    if ping.specular is not None:
        p += np.asarray(ping.specular, dtype=np.float64)
    a = float(model.noise_floor)
    ref = None
    if ping.unoccluded is not None:
        ref = np.asarray(ping.unoccluded, dtype=np.float64)
    if ping.ambient is not None:
        amb = np.asarray(ping.ambient, dtype=np.float64)
        ref = amb if ref is None else np.maximum(ref, amb)
    if ref is not None and model.reverberation_db > 0.0:
        b = 10.0 ** (-model.reverberation_db / 10.0) * ref
        s = a + b
        floor = np.where(s > 0, s - a * b / np.where(s > 0, s, 1.0), 0.0)
    else:
        floor = np.full_like(p, a)
    p += floor
    p[bins < ping.altitude_m] += float(model.watercolumn_noise)
    return p


def _to_reported_db(power: np.ndarray, model, acq, gain: int) -> np.ndarray:
    """Linear power -> reported dB exactly as ``PingEncoder`` maps it
    (without the per-ping normalisation, which a mean has no use for)."""
    from ..sonar.encoder import ceiling_at, gain_step_db, soft_ceiling_db
    db = (10.0 * np.log10(np.maximum(power, 1e-30))
          + model.calibration_db_offset + gain_step_db(gain))
    if model.receiver_ceiling:
        db = soft_ceiling_db(db, ceiling_at(
            acq.pulse_duration_s(model.max_ping_rate_hz), model),
            model.receiver_ceiling_knee_db)
    return db


def _crest(db: np.ndarray, i: int, bin_m: float, alt: float) -> float:
    w = int(math.ceil(max(CREST_MIN_M, CREST_ALT_FRACTION * alt) / bin_m))
    return float(db[max(i - 2, 0):i + w + 1].max())


def _leading(db: np.ndarray, i: int) -> float:
    return float(db[max(i - 2, 0):i + LEADING_BINS + 1].max())


def _replay_block(st_roll, st_pitch, st_alts, st_index, alt_med, n: int,
                  contiguous: bool
                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """``n`` (roll, pitch, dz) triples from a stratum: a time-contiguous
    block (noisy mode: consecutive simulated pings must be consecutive real
    pings, or the attitude jumps between them and decorrelates the texture
    along track -- a stratum holds one altitude band, so neighbouring
    entries can be seconds apart) or evenly spread (expected mode: the
    distribution is what matters)."""
    if st_roll is None:
        return None
    m = len(st_roll)
    if contiguous:
        if st_index is not None and m > n:
            ix = np.asarray(st_index)
            start = int(np.argmin(ix[n - 1:] - ix[:m - n + 1]))
        else:
            start = max((m - n) // 2, 0)
        idx = (start + np.arange(n)) % m
    else:
        idx = np.unique(np.linspace(0, m - 1, min(n, m)).round().astype(int))
    return (np.asarray(st_roll, dtype=np.float64)[idx],
            np.asarray(st_pitch, dtype=np.float64)[idx],
            np.asarray(st_alts, dtype=np.float64)[idx] - alt_med)


def _render_noisy(cfg, acq, side_name: str, alt: float, step: float,
                  n: int, seed: int, replay,
                  reflectivity: float = SAND_REFLECTIVITY) -> dict:
    from ..analysis.calibration import fbr_bin, scale_to_db
    from ..core.types import Pose3D, Side
    from ..sonar.encoder import PingEncoder
    from ..sonar.noise import apply_ping_noise
    from ..sonar.renderer import GeometricRenderer
    model = cfg.model
    half = acq.range_length_mm / 1000.0 + n * step + 5.0
    scene = flat_scene(alt + model.sensor_depth_m, half,
                       reflectivity=reflectivity)
    rr = GeometricRenderer(scene, acq, model)
    side = Side.PORT if replay is None else Side(side_name)
    enc = PingEncoder(side, acq, model)
    rng = np.random.default_rng(seed)
    bins = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
    rows, mx, fbr = [], [], []
    for k in range(n):
        x = -0.5 * n * step + k * step
        if replay is None:
            pose = Pose3D(x, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            pose = Pose3D(x, 0.0, float(replay[2][k]), float(replay[0][k]),
                          float(replay[1][k]), 0.0)
        pg = rr.render(side, pose, 0.0).ping
        pg.power = apply_ping_noise(pg.power, bins < pg.altitude_m, 1.0, model,
                                    rng, specular=pg.specular,
                                    texture=pg.texture,
                                    unoccluded=pg.unoccluded,
                                    ambient=pg.ambient,
                                    speckle_fill=pg.speckle_fill)
        e = enc.encode(pg)
        db = scale_to_db(e.pwr_results, e.min_pwr_db, e.max_pwr_db)
        i = fbr_bin(db)
        rows.append(db.astype(np.float32))
        mx.append(float(e.max_pwr_db))
        fbr.append(-1 if i is None else int(i))
    fbr_a = np.array(fbr)
    ok = fbr_a >= 0
    return dict(rows=np.array(rows), mx=np.array(mx), fbr=fbr_a,
                fbr_alt=(float(np.median(fbr_a[ok]) * acq.bin_size_m)
                         if ok.any() else float("nan")))


def _render_expected(cfg, acq, side_name: str, alt: float, gain: int,
                     replay) -> dict:
    from ..analysis.calibration import fbr_bin
    from ..core.types import Pose3D, Side
    from ..sonar.renderer import GeometricRenderer
    model = cfg.model
    half = acq.range_length_mm / 1000.0 + 5.0
    scene = flat_scene(alt + model.sensor_depth_m, half)
    rr = GeometricRenderer(scene, acq, model)
    bins = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
    if replay is None:
        poses = [(Side.PORT, Pose3D(0.0, 0.0, 0.0))]
    else:
        side = Side(side_name)
        poses = [(side, Pose3D(0.0, 0.0, float(dz), float(r), float(p), 0.0))
                 for r, p, dz in zip(*replay)]
    acc = np.zeros(acq.num_results)
    level_db0 = None
    for side, pose in poses:
        pg = rr.render(side, pose, 0.0).ping
        db = _to_reported_db(expected_power(pg, model, bins), model, acq, gain)
        if level_db0 is None:
            level_db0 = db
        acc += 10.0 ** (db / 10.0)
    level = 10.0 * np.log10(acc / len(poses))
    i = fbr_bin(level_db0) if replay is None else fbr_bin(level)
    return dict(level=level,
                fbr_alt=float("nan") if i is None else i * acq.bin_size_m)


def sim_job(job: Mapping[str, Any]) -> dict:
    """Render one stratum's simulated counterpart (module doc).

    ``job``: cfg, side, range_mm, num_results, pulse_us, gain, alt, step,
    mode ('noisy' | 'expected' | 'both'), replay (None or the
    ``(roll, pitch, alts)`` arrays of the stratum), alt_used (skip the FBR
    match: the fit freezes it), n_sim, seeds."""
    t0 = time.time()
    cfg = job["cfg"]
    acq = acquisition_for(cfg, job["range_mm"], job["num_results"],
                          job["pulse_us"], job["gain"])
    bin_m = acq.bin_size_m
    target = float(job["alt"])
    n = int(job.get("n_sim", N_SIM))
    seeds = tuple(job.get("seeds", SEEDS))
    rep_src = job.get("replay")
    out: dict[str, Any] = dict(key=job["key"],
                               pulse_us=acq.pulse_duration_s(
                                   cfg.model.max_ping_rate_hz) * 1e6)
    mode = job.get("mode", "noisy")
    if mode in ("noisy", "both"):
        rep = (None if rep_src is None else
               _replay_block(*rep_src, target, n, contiguous=True))
        used = job.get("alt_used_noisy")
        if used is None:
            a = _render_noisy(cfg, acq, job["side"], target, job["step"], n,
                              seeds[0], rep)
            used = target
            if np.isfinite(a["fbr_alt"]) and abs(a["fbr_alt"] - target) > bin_m:
                used = max(target + (target - a["fbr_alt"]), 0.2)
                a = _render_noisy(cfg, acq, job["side"], used, job["step"], n,
                                  seeds[0], rep)
        else:
            a = _render_noisy(cfg, acq, job["side"], used, job["step"], n,
                              seeds[0], rep)
        b = _render_noisy(cfg, acq, job["side"], used, job["step"], n,
                          seeds[1], rep)
        lin = lambda x: 10.0 * np.log10(np.mean(
            10.0 ** (x.astype(np.float64) / 10.0), axis=0))
        crest, lead = [], []
        for r in (a, b):
            for db, i in zip(r["rows"], r["fbr"]):
                if i >= 0:
                    crest.append(_crest(db, int(i), bin_m, target))
                    lead.append(_leading(db, int(i)))
        det = np.concatenate([a["fbr"], b["fbr"]]) >= 0
        out.update(rendered_alt=used, fbr_alt=a["fbr_alt"],
                   fbr_alt_seed2=b["fbr_alt"],
                   mean=[a["rows"].mean(0), b["rows"].mean(0)],
                   lin=[lin(a["rows"]), lin(b["rows"])], rows=a["rows"],
                   mx=np.concatenate([a["mx"], b["mx"]]),
                   crest=np.array(crest), leading=np.array(lead),
                   fbr_detection=float(det.mean()))
    if mode in ("expected", "both"):
        rep = (None if rep_src is None else
               _replay_block(*rep_src, target, N_EXPECTED, contiguous=False))
        used = job.get("alt_used_expected")
        if used is None:
            e = _render_expected(cfg, acq, job["side"], target, job["gain"], rep)
            used = target
            if np.isfinite(e["fbr_alt"]) and abs(e["fbr_alt"] - target) > bin_m:
                used = max(target + (target - e["fbr_alt"]), 0.2)
                e = _render_expected(cfg, acq, job["side"], used, job["gain"],
                                     rep)
        else:
            e = _render_expected(cfg, acq, job["side"], used, job["gain"], rep)
        out.update(exp_level=e["level"], exp_rendered_alt=used,
                   exp_fbr_alt=e["fbr_alt"])
    out["secs"] = time.time() - t0
    return out


def make_job(st: Stratum, cfg, *, mode: str, replay: str,
             n_sim: int = N_SIM, seeds: Sequence[int] = SEEDS,
             alt_used: Mapping[str, float] | None = None) -> dict:
    rep = None
    if replay == "attitude" and st.roll is not None:
        rep = (st.roll, st.pitch, st.alts, st.index)
    elif replay not in ("none", "attitude"):
        raise ValueError(f"replay {replay!r}: none | attitude")
    j = dict(key=st.key, cfg=cfg, side=st.side, range_mm=st.range_mm,
             num_results=st.num_results, pulse_us=st.pulse_us, gain=st.gain,
             alt=round(st.alt, 2) if rep is None else st.alt,
             step=st.step_m, mode=mode, replay=rep, n_sim=n_sim,
             seeds=tuple(seeds))
    for k, v in (alt_used or {}).items():
        j[k] = v
    return j


# ================================================================== scoring
def tail_mask(r: np.ndarray, alt: float, L_m: float, real_mean: np.ndarray):
    lo, hi = max(1.4 * alt, 1.0), min(0.725 * L_m, L_m - 1.0)
    return (r >= lo) & (r <= hi) & (real_mean >= SIGNAL_DB), lo, hi


def power_average_db(rows: np.ndarray) -> np.ndarray:
    return 10.0 * np.log10(np.mean(10.0 ** (rows.astype(np.float64) / 10.0),
                                   axis=0))


def _boot_bias(real_rows, linear, sim_curve, mask, rng):
    n = real_rows.shape[0]
    if n < 2 * BLOCK:
        return None
    blocks = [np.arange(s, min(s + BLOCK, n)) for s in range(0, n, BLOCK)]
    x = (10.0 ** (real_rows[:, mask].astype(np.float64) / 10.0) if linear
         else real_rows[:, mask])
    out = []
    for _ in range(BOOT):
        pick = rng.integers(0, len(blocks), len(blocks))
        m = x[np.concatenate([blocks[j] for j in pick])].mean(0)
        if linear:
            m = 10.0 * np.log10(m)
        out.append(float(np.mean(sim_curve[mask] - m)))
    return [round(float(np.percentile(out, 2.5)), 3),
            round(float(np.percentile(out, 97.5)), 3)]


def _nadir(lin: np.ndarray, r: np.ndarray, alt: float) -> dict:
    out = {}
    for lo, hi in NADIR_WINDOWS_M:
        m = (r >= lo) & (r <= hi)
        if m.any() and hi < 0.9 * alt:
            out[f"{lo:g}-{hi:g}"] = round(float(
                10.0 * np.log10(np.mean(10.0 ** (lin[m] / 10.0)))), 2)
    return out


def score_stratum(st: Stratum, sim: Mapping[str, Any],
                  rng: np.random.Generator, ci: bool = True) -> dict:
    """Every metric of the module doc for one stratum (sim - real);
    ``ci=False`` skips the block bootstrap (grid scans)."""
    from ..analysis.svlog_compare import texture_stats
    L_m = st.range_mm / 1000.0
    N = st.num_results
    r = (np.arange(N) + 0.5) * L_m / N
    real_mean = st.rows.mean(0)
    real_lin = power_average_db(st.rows)
    mask, lo, hi = tail_mask(r, st.alt, L_m, real_mean)
    res: dict[str, Any] = dict(
        session=st.session, role=st.role, file=st.file, side=st.side,
        L=st.range_mm, N=N, pulse_us=st.pulse_us, gain=st.gain, band=st.band,
        alt=round(st.alt, 4), n=st.n, core=st.core, regime=regime_of(st.alt),
        step=round(st.step_m, 4), speed=st.speed_mps,
        attitude_replayed=bool(sim.get("replayed", False)),
        sim_pulse_us=round(float(sim["pulse_us"]), 1),
        tail_window_m=[round(lo, 2), round(hi, 2)], tail_bins=int(mask.sum()))
    real_crest = np.array([_crest(db, int(i), st.bin_m, st.alt)
                           for db, i in zip(st.rows, st.fbr)])
    real_lead = np.array([_leading(db, int(i))
                          for db, i in zip(st.rows, st.fbr)])
    res["crest_real_db"] = round(float(np.median(real_crest)), 2)
    res["leading_real_db"] = round(float(np.median(real_lead)), 2)
    res["nadir_real_db"] = _nadir(real_lin, r, st.alt)
    res["peak_real_db"] = round(float(np.median(st.max_db)), 2)
    res["peak_law_db"] = round(PEAK_LAW[0] + PEAK_LAW[1]
                               * math.log10(max(st.alt, 0.05)), 2)
    if "exp_level" in sim:
        res["exp_rendered_alt"] = round(float(sim["exp_rendered_alt"]), 3)
        res["exp_fbr_alt"] = round(float(sim["exp_fbr_alt"]), 3)
        res["nadir_sim_expected_db"] = _nadir(sim["exp_level"], r, st.alt)
        if mask.sum() >= 10:
            de = sim["exp_level"] - real_lin
            be = float(np.mean(de[mask]))
            res.update(level_expected_bias_db=round(be, 3),
                       level_expected_shape_rms_db=round(float(np.sqrt(
                           np.mean((de[mask] - be) ** 2))), 3),
                       sub_bands_expected=_sub_bands(de, r, mask, st.alt))
    if "lin" in sim:
        res.update(sim_rendered_alt=round(float(sim["rendered_alt"]), 3),
                   sim_fbr_alt=round(float(sim["fbr_alt"]), 3),
                   sim_fbr_detection=round(float(sim["fbr_detection"]), 4))
        if mask.sum() >= 10:
            dl = sim["lin"][0] - real_lin
            bl = float(np.mean(dl[mask]))
            d0 = sim["mean"][0] - real_mean
            d1 = sim["mean"][1] - real_mean
            bias = float(np.mean(d0[mask]))
            res.update(
                level_bias_db=round(bl, 3),
                level_shape_rms_db=round(float(np.sqrt(np.mean(
                    (dl[mask] - bl) ** 2))), 3),
                level_rms_db=round(float(np.sqrt(np.mean(dl[mask] ** 2))), 3),
                level_bias_ci95_db=(_boot_bias(st.rows, True, sim["lin"][0],
                                               mask, rng) if ci else None),
                level_seed_diff_db=round(float(np.mean(
                    (sim["lin"][1] - real_lin)[mask])) - bl, 3),
                dbmean_bias_db=round(bias, 3),
                dbmean_shape_rms_db=round(float(np.sqrt(np.mean(
                    (d0[mask] - bias) ** 2))), 3),
                dbmean_bias_ci95_db=(_boot_bias(st.rows, False,
                                                sim["mean"][0], mask, rng)
                                     if ci else None),
                dbmean_seed_diff_db=round(float(np.mean(d1[mask])) - bias, 3),
                fluct_real_db=round(float(np.mean(
                    (real_lin - real_mean)[mask])), 3),
                fluct_sim_db=round(float(np.mean(
                    (sim["lin"][0] - sim["mean"][0])[mask])), 3),
                perbin_std_real_db=round(float(
                    st.rows[:, mask].std(0).mean()), 3),
                perbin_std_sim_db=round(float(
                    sim["rows"][:, mask].std(0).mean()), 3),
                sub_bands=_sub_bands(dl, r, mask, st.alt))
        if len(sim["crest"]):
            res["crest_sim_db"] = round(float(np.median(sim["crest"])), 2)
            res["crest_bias_db"] = round(res["crest_sim_db"]
                                         - res["crest_real_db"], 2)
            res["leading_sim_db"] = round(float(np.median(sim["leading"])), 2)
            res["leading_bias_db"] = round(res["leading_sim_db"]
                                           - res["leading_real_db"], 2)
        res["nadir_sim_db"] = _nadir(sim["lin"][0], r, st.alt)
        res["peak_sim_db"] = round(float(np.median(sim["mx"])), 2)
        tr = texture_stats(st.rows.astype(np.float64))
        ts = texture_stats(sim["rows"].astype(np.float64))
        res["texture"] = {k: [round(tr.get(k, float("nan")), 3),
                              round(ts.get(k, float("nan")), 3)]
                          for k in ("range_corr_lag1", "track_corr_lag1",
                                    "hp_std_db")}
        res["texture_track_comparable"] = st.speed_mps is not None
    return res


def _sub_bands(d: np.ndarray, r: np.ndarray, mask: np.ndarray, alt: float
               ) -> dict[str, float]:
    out = {}
    edges = [1.4 * alt, *SUB_BAND_EDGES]
    for e0, e1 in zip(edges[:-1], edges[1:]):
        m = mask & (r >= e0) & (r < e1)
        if m.sum() >= 5:
            out[f"{e0:.1f}-{e1:g}"] = round(float(np.mean(d[m])), 2)
    return out


SUMMARY_KEYS = ("level_bias_db", "level_expected_bias_db", "crest_bias_db",
                "leading_bias_db", "dbmean_bias_db", "fluct_real_db",
                "fluct_sim_db", "perbin_std_real_db", "perbin_std_sim_db",
                "sim_fbr_detection")


def summarise(results: Sequence[Mapping[str, Any]], core_only: bool = True
              ) -> dict[str, dict]:
    """Ping-weighted means per session, per acquisition (bins, pulse) and
    per altitude regime; the minimum simulated FBR detection rate too."""
    def agg(rows):
        out: dict[str, Any] = dict(strata=len(rows),
                                   pings=int(sum(r["n"] for r in rows)))
        for k in SUMMARY_KEYS:
            v = [(r[k], r["n"]) for r in rows if r.get(k) is not None]
            if v:
                w = np.array([x[1] for x in v], dtype=float)
                out[k] = round(float(np.average([x[0] for x in v],
                                                weights=w)), 3)
        det = [r["sim_fbr_detection"] for r in rows
               if r.get("sim_fbr_detection") is not None]
        if det:
            out["sim_fbr_detection_min"] = round(float(min(det)), 4)
        return out
    rows = [r for r in results if r.get("core") or not core_only]
    by: dict[str, dict] = {"session": {}, "acquisition": {}, "regime": {},
                           "role": {}}
    for name, keyf in (("session", lambda r: r["session"]),
                       ("acquisition", lambda r: f"{r['N']} bins / "
                                                 f"{r['pulse_us']:g} us"),
                       ("regime", lambda r: r["regime"]),
                       ("role", lambda r: r["role"])):
        groups: dict[str, list] = {}
        for r in rows:
            groups.setdefault(keyf(r), []).append(r)
        by[name] = {k: agg(v) for k, v in sorted(groups.items())}
    by["session_role"] = {r["session"]: r["role"] for r in rows}
    return by


# ================================================================= profiles
def load_config(profile: str | Path | None = None,
                artifact: str | Path | None = None,
                sets: Sequence[str] = (), allow_base_mismatch: bool = False):
    """``(SonarConfig, description)``: a profile (default: the shipped one),
    optionally composed with an artifact (``effective_config``), then
    ``--set model.x=v`` overrides (exploration and priors; recorded)."""
    from ..sonar.config import SonarConfig, find_default_sonar_yaml
    from .artifact import effective_config, load_artifact
    prof = Path(profile).expanduser() if profile else find_default_sonar_yaml()
    desc: dict[str, Any] = {"profile": str(prof), "profile_sha256":
                            hashlib.sha256(prof.read_bytes()).hexdigest()}
    if artifact:
        doc = load_artifact(artifact)
        eff = effective_config(prof, artifact=doc,
                               allow_base_mismatch=allow_base_mismatch)
        cfg = eff.config
        desc["artifact"] = {"path": str(artifact), "id": doc.get("id")}
    else:
        cfg = SonarConfig.from_yaml(prof)
    desc["set"] = apply_sets(cfg, sets)
    return cfg, desc


def apply_sets(cfg, sets: Iterable[str]) -> dict[str, Any]:
    from .registry import load_registry
    reg = load_registry()
    done = {}
    for s in sets:
        name, _, raw = str(s).partition("=")
        name = name.strip()
        if not raw:
            raise ValueError(f"--set {s!r}: expected model.<field>=<value>")
        value = yaml.safe_load(raw)
        sect, _, key = name.partition(".")
        if sect not in ("model", "acquisition") or not hasattr(
                getattr(cfg, sect), key):
            raise KeyError(f"--set {name}: no such sonar parameter")
        if name in reg.parameters:
            reg.parameters[name].check_value(value)
        setattr(getattr(cfg, sect), key, value)
        done[name] = value
    return done


def set_params(cfg, names: Sequence[str], values: Sequence[float]):
    c = dataclasses.replace(cfg, acquisition=dataclasses.replace(
        cfg.acquisition), model=dataclasses.replace(cfg.model))
    for n, v in zip(names, values):
        sect, _, key = n.partition(".")
        cur = getattr(getattr(c, sect), key)
        setattr(getattr(c, sect), key, type(cur)(v) if isinstance(
            cur, (int, float)) and not isinstance(cur, bool) else v)
    return c


# ==================================================================== score
def score(strata: Sequence[Stratum], cfg, *, mode: str = "both",
          replay: str = "attitude", executor: Executor | None = None,
          n_sim: int = N_SIM, seeds: Sequence[int] = SEEDS,
          curves_out: Path | None = None, ci: bool = True) -> list[dict]:
    jobs = [make_job(s, cfg, mode=mode, replay=replay, n_sim=n_sim,
                     seeds=seeds) for s in strata]
    run = executor.map if executor is not None else map
    sims = list(run(sim_job, jobs))
    rng = np.random.default_rng(12345)
    out = []
    for k, (s, sm) in enumerate(zip(strata, sims)):
        sm = dict(sm, replayed=jobs[k]["replay"] is not None)
        res = score_stratum(s, sm, rng, ci=ci)
        if curves_out is not None:
            curves_out.mkdir(parents=True, exist_ok=True)
            f = curves_out / f"curve_{k + 1:03d}.npz"
            arrays = dict(r=(np.arange(s.num_results) + 0.5) * s.bin_m,
                          real=s.rows.mean(0),
                          real_lin=power_average_db(s.rows))
            if "lin" in sm:
                arrays.update(sim0=sm["mean"][0], sim1=sm["mean"][1],
                              sim_lin=sm["lin"][0])
            if "exp_level" in sm:
                arrays["sim_expected"] = sm["exp_level"]
            np.savez_compressed(f, **arrays)
            res["curve_file"] = f.name
        out.append(res)
    return out


# ====================================================================== fit
@dataclass
class FitResult:
    params: list[str]
    x0: list[float]
    x: list[float]
    bounds: list[list[float]]
    cost0: float
    cost: float
    rms0_db: float
    rms_db: float
    n_residuals: int
    strata: list[str]
    cov: list[list[float]] | None
    sd: list[float | None]
    corr: list[list[float]] | None
    scans: dict[str, list[list[float]]] = field(default_factory=dict)
    per_stratum0: list[float] = field(default_factory=list)
    per_stratum: list[float] = field(default_factory=list)
    nfev: int = 0
    message: str = ""

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


def _residual_selector(metric: str, st: Stratum, max_bins: int,
                       nadir_max_m: float = NADIR_MAX_M):
    """(indices into the level curve, the real values there)."""
    L_m = st.range_mm / 1000.0
    r = (np.arange(st.num_results) + 0.5) * L_m / st.num_results
    real_lin = power_average_db(st.rows)
    if metric == "level":
        m, _, _ = tail_mask(r, st.alt, L_m, st.rows.mean(0))
    elif metric == "nadir":
        m = r <= min(nadir_max_m, 0.7 * st.alt)
    else:
        raise ValueError(f"metric {metric!r}: level | nadir")
    idx = np.flatnonzero(m)
    if idx.size > max_bins:
        idx = idx[np.linspace(0, idx.size - 1, max_bins).round().astype(int)]
    return idx, real_lin[idx]


class _Evaluator:
    def __init__(self, strata, cfg, params, metric, replay, executor,
                 max_bins, nadir_max_m=NADIR_MAX_M):
        self.strata = list(strata)
        self.cfg, self.params, self.metric = cfg, list(params), metric
        self.replay, self.ex = replay, executor
        sel = [_residual_selector(metric, s, max_bins, nadir_max_m)
               for s in self.strata]
        keep = [i for i, (idx, _) in enumerate(sel) if idx.size >= 5]
        self.strata = [self.strata[i] for i in keep]
        self.sel = [sel[i] for i in keep]
        self.alt_used: list[float] | None = None
        self.nfev = 0

    def curves(self, x) -> list[np.ndarray]:
        cfg = set_params(self.cfg, self.params, x)
        jobs = []
        for k, s in enumerate(self.strata):
            au = (None if self.alt_used is None
                  else {"alt_used_expected": self.alt_used[k]})
            jobs.append(make_job(s, cfg, mode="expected", replay=self.replay,
                                 alt_used=au))
        run = self.ex.map if self.ex is not None else map
        sims = list(run(sim_job, jobs))
        if self.alt_used is None:
            self.alt_used = [float(sm["exp_rendered_alt"]) for sm in sims]
        self.nfev += 1
        return [sm["exp_level"] for sm in sims]

    def residuals(self, x) -> np.ndarray:
        out = []
        for (idx, real), lvl in zip(self.sel, self.curves(x)):
            out.append((lvl[idx] - real) / math.sqrt(idx.size))
        return np.concatenate(out)

    def per_stratum(self, x) -> list[float]:
        return [round(float(np.mean(lvl[idx] - real)), 3)
                for (idx, real), lvl in zip(self.sel, self.curves(x))]


def fit(strata: Sequence[Stratum], cfg, params: Sequence[str], *,
        metric: str = "level", replay: str = "attitude",
        executor: Executor | None = None, max_bins: int = 24,
        scan_points: int = 5, nadir_max_m: float = NADIR_MAX_M) -> FitResult:
    """Least squares of ``params`` (module doc); local covariance from the
    Jacobian at the solution (residuals treated as independent -- they are
    not, so the sd understates), and a conditional 1-D scan of each
    parameter about the solution."""
    from scipy.optimize import least_squares
    from .registry import load_registry
    reg = load_registry()
    lo, hi, x0 = [], [], []
    for p in params:
        e = reg.get(p)
        if not e.tunable:
            raise ValueError(f"{p} is not tunable (class {e.cls}); a fit "
                             "cannot set it")
        b = e.bounds or [-np.inf, np.inf]
        lo.append(float(b[0]))
        hi.append(float(b[1]))
        sect, _, key = p.partition(".")
        x0.append(float(getattr(getattr(cfg, sect), key)))
    ev = _Evaluator(strata, cfg, params, metric, replay, executor, max_bins,
                    nadir_max_m)
    if not ev.strata:
        raise ValueError("no stratum has enough residual bins")
    r0 = ev.residuals(x0)
    ps0 = ev.per_stratum(x0)
    sol = least_squares(ev.residuals, np.array(x0), bounds=(lo, hi),
                        x_scale="jac", diff_step=1e-3, method="trf")
    x = [float(v) for v in sol.x]
    J = sol.jac
    n, k = J.shape
    s2 = float(np.sum(sol.fun ** 2)) / max(n - k, 1)
    cov = sd = corr = None
    try:
        c = np.linalg.inv(J.T @ J) * s2
        cov = c.tolist()
        sd = [float(math.sqrt(max(c[i, i], 0.0))) for i in range(k)]
        d = np.sqrt(np.clip(np.diag(c), 1e-300, None))
        corr = (c / np.outer(d, d)).tolist()
    except np.linalg.LinAlgError:
        sd = [None] * k
    scans: dict[str, list[list[float]]] = {}
    for i, p in enumerate(params):
        width = 3.0 * (sd[i] if sd and sd[i] else 0.05 * (hi[i] - lo[i]))
        pts = []
        for v in np.linspace(max(lo[i], x[i] - width),
                             min(hi[i], x[i] + width), scan_points):
            xx = list(x)
            xx[i] = float(v)
            pts.append([float(v), float(0.5 * np.sum(ev.residuals(xx) ** 2))])
        scans[p] = pts
    return FitResult(
        params=list(params), x0=x0, x=x, bounds=[lo, hi],
        cost0=float(0.5 * np.sum(r0 ** 2)), cost=float(sol.cost),
        rms0_db=float(math.sqrt(np.mean(np.square(ps0)))),
        rms_db=float(math.sqrt(np.mean(np.square(ev.per_stratum(x))))),
        n_residuals=int(n), strata=[s.label() for s in ev.strata],
        cov=cov, sd=sd, corr=corr, scans=scans, per_stratum0=ps0,
        per_stratum=ev.per_stratum(x), nfev=ev.nfev,
        message=str(sol.message))


def check_fit_roles(strata: Sequence[Stratum], allow_reuse: str | None
                    ) -> list[str]:
    """Refuse a fit on anything but calibration sessions unless a reason
    is given; return the reused sessions (to be recorded)."""
    reused = sorted({s.session for s in strata if s.role != "calibration"})
    if reused and not (allow_reuse or "").strip():
        raise PermissionError(
            f"sessions {reused} are not calibration sessions; fitting on them "
            "turns a hold-out into training data -- pass allow_reuse "
            "(--allow-reuse \"<reason>\") to do it deliberately, recorded")
    return reused


def draft_artifact(result: FitResult, *, artifact_id: str,
                   base_profile: Path, manifest_path: str,
                   calibration_sessions: Sequence[str],
                   validation_sessions: Sequence[str], metric: str,
                   replay: str, assumptions: Sequence[str],
                   reused: Sequence[str] = (), reuse_reason: str = "",
                   status: str = "joint") -> dict:
    from .registry import load_registry
    reg = load_registry()
    params = {}
    for i, p in enumerate(result.params):
        unc: dict[str, Any] = {}
        if result.sd and result.sd[i] is not None:
            unc["sd"] = round(float(result.sd[i]), 4)
        else:
            unc["unidentified"] = "singular Jacobian at the solution"
        params[p] = dict(value=round(float(result.x[i]), 4),
                         unit=reg.get(p).unit,
                         status=status if len(result.params) > 1
                         else "estimated",
                         uncertainty=unc,
                         note=("formal sd from the local Jacobian, residuals "
                               "treated as independent: it understates"))
    notes = []
    if reused:
        notes.append(f"fitted on non-calibration sessions {list(reused)}: "
                     f"{reuse_reason}")
    return {
        "format": "blueboat_calibration_artifact/1",
        "id": artifact_id,
        "created": _dt.date.today().isoformat(),
        "created_by": "blueboat_sss_sim.calibration.seabed fit",
        "description": f"DRAFT: {metric} fit of {result.params}",
        "software": {"package": "blueboat_sss_sim"},
        "base_profile": {"name": base_profile.name,
                         "sha256": hashlib.sha256(
                             base_profile.read_bytes()).hexdigest()},
        "parameters": params,
        "data": {"manifest": str(manifest_path),
                 "calibration_sessions": sorted(set(calibration_sessions)),
                 "validation_sessions": sorted(set(validation_sessions)
                                               - set(calibration_sessions))},
        "fit": {"method": "scipy.optimize.least_squares (trf), expected-mode "
                          f"forward model, replay {replay}",
                "objective": (f"{metric}: mean over strata of the squared "
                              "sim - real power-average dB residual, each "
                              "stratum weighted equally"),
                "bounds": {p: [result.bounds[0][i], result.bounds[1][i]]
                           for i, p in enumerate(result.params)},
                "cost": [round(result.cost0, 4), round(result.cost, 4)],
                "per_stratum_rms_db": [round(result.rms0_db, 3),
                                       round(result.rms_db, 3)]},
        "assumptions": list(assumptions),
        "validation": [],
        **({"notes": "; ".join(notes)} if notes else {}),
    }


def write_base_profile(cfg, desc: Mapping[str, Any], path: Path) -> Path:
    """The composed configuration a fit started from, self-contained and
    write-once, so a draft artifact can name it as its base profile."""
    from .artifact import EffectiveConfig, write_effective_profile
    eff = EffectiveConfig(config=cfg, provenance={}, not_applied={})
    return write_effective_profile(eff, path, sources=dict(desc))


# ============================================================ NC #4 envelope
#: The FBR operating envelope (2026-10-30): the corpus's acquisitions and the
#: altitudes they were used at. A model change that fits a field statistic
#: must keep the downstream detector finding the first bottom return here
#: wherever the shipped profile does (NC #4) -- fitting the specular
#: strength to one 7 m crest alone lost it at 7-21 m in the harbour.
ENVELOPE_ACQS = ((20000, 1200, 44.3), (40000, 1200, 64.3),
                 (40000, 600, 106.4), (80000, 600, 212.8))
ENVELOPE_ALTS = (0.6, 1.0, 1.5, 2.0, 3.0, 5.0, 7.0, 10.0, 14.0, 18.0, 22.0)
ENVELOPE_MIN_DETECTION = 0.95


#: Seabed reflectivities the envelope runs over: sand (the calibration
#: bottom) and the darkest seabed material of the World Builder (seagrass,
#: 0.22; mud is 0.30) -- the shipped specular strength was sized for "dark
#: mud/seagrass" (default_sonar.yaml), and NC #4 holds over every bottom.
ENVELOPE_REFLECTIVITIES = (SAND_REFLECTIVITY, 0.22)


def _envelope_job(job: Mapping[str, Any]) -> tuple:
    cfg, (L, N, pu), alt = job["cfg"], job["acq"], job["alt"]
    acq = acquisition_for(cfg, L, N, pu, 4)
    r = _render_noisy(cfg, acq, "port", alt, 0.05, job["n"], job["seed"], None,
                      reflectivity=job["refl"])
    return (L, N, pu), alt, job["refl"], float((r["fbr"] >= 0).mean())


def fbr_envelope(cfg, *, executor: Executor | None = None, n: int = 60,
                 seed: int = 11, acqs=ENVELOPE_ACQS, alts=ENVELOPE_ALTS,
                 reflectivities=ENVELOPE_REFLECTIVITIES
                 ) -> dict[tuple, float]:
    """Detection rate of the downstream FBR detector on ``n`` noisy, encoded
    level pings over a flat bottom, per (acquisition, altitude,
    reflectivity) of the envelope (altitudes beyond 0.7 x range skipped)."""
    jobs = [dict(cfg=cfg, acq=a, alt=h, n=n, seed=seed, refl=rf)
            for rf in reflectivities for a in acqs
            for h in alts if h < 0.7 * a[0] / 1000.0]
    run = executor.map if executor is not None else map
    return {(a, h, rf): d for a, h, rf, d in run(_envelope_job, jobs)}


def envelope_regressions(candidate: Mapping[tuple, float],
                         reference: Mapping[tuple, float],
                         floor: float = ENVELOPE_MIN_DETECTION) -> list:
    """Envelope cases the reference detects (>= floor) and the candidate
    does not."""
    return sorted((k, reference[k], candidate.get(k, 0.0)) for k in reference
                  if reference[k] >= floor and candidate.get(k, 0.0) < floor)


# ================================================================ anchor gate
def anchor_level_bias(path: str | Path, cfg, *, band: tuple[float, float],
                      max_pings: int = 300, gain: int = 4,
                      side: str | None = None) -> dict:
    """Cheap corpus gate: the first ``max_pings`` gain-``gain`` pings of a
    recording whose FBR altitude lies in ``band`` (one acquisition: the
    first such ping's), against the level-boat expected model. Reads the
    file only up to the last ping it needs."""
    from ..analysis.calibration import fbr_bin, scale_to_db
    from ..analysis.svlog_reader import _side_from_packet, iter_packets
    from ..sonar.encoder import OS_MONO_PROFILE_ID, parse_frame
    rows, alts, acq_key = [], [], None
    for pkt in iter_packets(path):
        if pkt.packet_id != OS_MONO_PROFILE_ID:
            continue
        d = parse_frame(pkt.raw)
        if int(d["gain_index"]) != gain or not int(d["num_results"]):
            continue
        if side and _side_from_packet(d).value != side:
            continue
        key = (int(d["length_mm"]), int(d["num_results"]),
               round(float(d["pulse_duration_sec"]) * 1e6, 1))
        if acq_key is not None and key != acq_key:
            continue
        db = scale_to_db(d["pwr_results"], float(d["min_pwr_db"]),
                         float(d["max_pwr_db"]))
        i = fbr_bin(db)
        if i is None:
            continue
        alt = i * key[0] / 1000.0 / key[1]
        if not (band[0] <= alt < band[1]):
            continue
        acq_key = key
        rows.append(db.astype(np.float32))
        alts.append(alt)
        if len(rows) >= max_pings:
            break
    if not rows:
        return dict(n=0)
    L, N, pulse_us = acq_key
    st = Stratum(session="", role="", file=str(Path(path).name), side="port",
                 range_mm=L, num_results=N, pulse_us=pulse_us, gain=gain,
                 band=band[0], alt=float(np.median(alts)),
                 rows=np.stack(rows), max_db=np.zeros(len(rows)),
                 fbr=np.zeros(len(rows), dtype=int), alts=np.array(alts),
                 roll=None, pitch=None, step_m=0.05, speed_mps=None,
                 core=True)
    sm = sim_job(make_job(st, cfg, mode="expected", replay="none"))
    r = (np.arange(N) + 0.5) * L / 1000.0 / N
    real_lin = power_average_db(st.rows)
    m, _, _ = tail_mask(r, st.alt, L / 1000.0, st.rows.mean(0))
    return dict(n=len(rows), alt=round(st.alt, 3), acquisition=list(acq_key),
                tail_bins=int(m.sum()),
                level_bias_db=float(np.mean((sm["exp_level"] - real_lin)[m]))
                if m.sum() >= 10 else None)


# ====================================================================== CLI
def _executor(workers: int) -> Executor | None:
    return ProcessPoolExecutor(workers) if workers > 1 else None


def _read_manifest(path: str | Path) -> dict:
    with open(Path(path).expanduser(), "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _guard(out: Path, root: Path, extra: Iterable[Path] = ()) -> None:
    from .io_guard import refuse_inside
    refuse_inside(out, roots=[root, *extra])


def cmd_score(a) -> int:
    out = Path(a.out).expanduser().resolve()
    root = Path(a.root).expanduser().resolve()
    cache = Path(a.cache).expanduser().resolve() if a.cache else None
    _guard(out, root)
    if cache is not None:
        _guard(cache, root)
    out.mkdir(parents=True, exist_ok=True)
    if (out / "scores.json").exists():
        raise SystemExit(f"{out / 'scores.json'} exists; scores are "
                         "write-once -- choose a new --out")
    cfg, desc = load_config(a.profile, a.artifact, a.set or [],
                            a.allow_base_mismatch)
    man = _read_manifest(a.manifest)
    t0 = time.time()
    ex = _executor(a.workers)
    try:
        strata, meta = load_strata(man, root, roles=a.roles.split(","),
                                   sessions=a.session or None, gain=a.gain,
                                   cache=cache, executor=ex)
        print(f"{len(strata)} strata from {len(meta)} recordings "
              f"({time.time() - t0:.0f} s)", flush=True)
        res = score(strata, cfg, mode=a.mode, replay=a.replay, executor=ex,
                    n_sim=a.n_sim, curves_out=out / "curves")
    finally:
        if ex is not None:
            ex.shutdown()
    doc = dict(format=SCORES_FORMAT, label=a.label, config=desc,
               mode=a.mode, replay=a.replay, gain=a.gain, n_sim=a.n_sim,
               seeds=list(SEEDS), min_pings=MIN_PINGS, signal_db=SIGNAL_DB,
               peak_law=list(PEAK_LAW), manifest=str(a.manifest),
               files=meta, summary=summarise(res),
               summary_all=summarise(res, core_only=False), strata=res,
               seconds=round(time.time() - t0, 1))
    (out / "scores.json").write_text(json.dumps(doc, indent=1, default=float)
                                     + "\n")
    _print_summary(doc["summary"])
    print(f"wrote {out / 'scores.json'} ({len(res)} strata, "
          f"{time.time() - t0:.0f} s)")
    return 0


def _print_summary(summ: Mapping[str, Any]) -> None:
    cols = ("level_bias_db", "level_expected_bias_db", "crest_bias_db",
            "fluct_sim_db", "fluct_real_db", "perbin_std_sim_db",
            "perbin_std_real_db", "sim_fbr_detection_min")
    for group in ("session", "acquisition", "regime"):
        print(f"-- by {group} (core strata, ping-weighted)")
        for k, v in summ[group].items():
            vals = " ".join(f"{c.replace('_db', '')}={v.get(c, float('nan')):+.2f}"
                            for c in cols if c in v)
            print(f"  {k:<34} n={v['pings']:>6} {vals}")


def cmd_fit(a) -> int:
    out = Path(a.out).expanduser().resolve()
    root = Path(a.root).expanduser().resolve()
    _guard(out, root)
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"{out} is not empty; a fit is write-once -- choose "
                         "a new --out")
    cache = Path(a.cache).expanduser().resolve() if a.cache else None
    if cache is not None:
        _guard(cache, root)
    cfg, desc = load_config(a.profile, a.artifact, a.set or [],
                            a.allow_base_mismatch)
    man = _read_manifest(a.manifest)
    roles = {s["id"]: s["role"] for s in man.get("sessions") or []}
    sessions = a.session or [k for k, v in roles.items()
                             if v == "calibration"]
    ex = _executor(a.workers)
    t0 = time.time()
    try:
        strata, meta = load_strata(man, root, roles=tuple(set(roles.values())),
                                   sessions=sessions, gain=a.gain, cache=cache,
                                   executor=ex)
        if not a.all_strata:
            strata = [s for s in strata if s.core]
        reused = check_fit_roles(strata, a.allow_reuse)
        print(f"fit {a.param} on {len(strata)} strata of {sessions} "
              f"({a.metric}, replay {a.replay})", flush=True)
        res = fit(strata, cfg, a.param, metric=a.metric, replay=a.replay,
                  executor=ex, max_bins=a.max_bins,
                  nadir_max_m=a.nadir_max_m)
    finally:
        if ex is not None:
            ex.shutdown()
    out.mkdir(parents=True, exist_ok=True)
    base = write_base_profile(cfg, desc, out / "base_profile.yaml")
    doc = dict(format=FIT_FORMAT, config=desc, metric=a.metric,
               nadir_max_m=a.nadir_max_m,
               replay=a.replay, sessions=sessions, reused=reused,
               reuse_reason=a.allow_reuse or "", files=meta,
               result=res.to_dict(), seconds=round(time.time() - t0, 1))
    (out / "fit.json").write_text(json.dumps(doc, indent=1, default=float)
                                  + "\n")
    val = [k for k, v in roles.items() if v == "validation"]
    art = draft_artifact(
        res, artifact_id=a.artifact_id or f"draft-{a.metric}-"
        f"{_dt.datetime.now().strftime('%Y%m%d-%H%M%S')}",
        base_profile=base, manifest_path=str(a.manifest),
        calibration_sessions=sorted({s.session for s in strata}),
        validation_sessions=val, metric=a.metric, replay=a.replay,
        assumptions=[
            "flat sand seabed (reflectivity 0.55) under every scored window",
            f"gain index {a.gain} only; altitude = detector FBR x L/N",
            "the base profile's other constants held fixed (base_profile)",
            *[f"set before the fit: {k} = {v}" for k, v in
              desc.get("set", {}).items()]],
        reused=reused, reuse_reason=a.allow_reuse or "")
    from .artifact import save_artifact, validate_artifact
    rep = validate_artifact(art)
    save_artifact(art, out / "draft_artifact.yaml")
    print(json.dumps(dict(params=res.params, x0=res.x0, x=res.x, sd=res.sd,
                          corr=res.corr, rms_db=[res.rms0_db, res.rms_db],
                          nfev=res.nfev), indent=1))
    print(f"draft artifact: errors {rep.errors} warnings {rep.warnings}")
    print(f"wrote {out} ({time.time() - t0:.0f} s)")
    return 0 if rep.ok else 1


def cmd_scan(a) -> int:
    """Score a grid of parameter values (cartesian over every --grid) on
    the same strata; per value, the session/acquisition/regime summaries.
    For metrics a least-squares fit cannot use (the crest is a max
    statistic, the texture a noise statistic): common random numbers make
    the comparison between grid points paired."""
    import itertools
    out = Path(a.out).expanduser().resolve()
    root = Path(a.root).expanduser().resolve()
    _guard(out, root)
    cache = Path(a.cache).expanduser().resolve() if a.cache else None
    if cache is not None:
        _guard(cache, root)
    if (out / "scan.json").exists():
        raise SystemExit(f"{out / 'scan.json'} exists; choose a new --out")
    cfg, desc = load_config(a.profile, a.artifact, a.set or [],
                            a.allow_base_mismatch)
    names, values = [], []
    for g in a.grid:
        name, _, vals = g.partition("=")
        names.append(name.strip())
        values.append([yaml.safe_load(v) for v in vals.split(",")])
    from .registry import load_registry
    reg = load_registry()
    for n_, vs in zip(names, values):
        for v in vs:
            if n_ in reg.parameters:
                reg.parameters[n_].check_value(v)
    man = _read_manifest(a.manifest)
    ex = _executor(a.workers)
    t0 = time.time()
    points = []
    try:
        strata, meta = load_strata(man, root, roles=a.roles.split(","),
                                   sessions=a.session or None, gain=a.gain,
                                   cache=cache, executor=ex)
        if not a.all_strata:
            strata = [s for s in strata if s.core]
        for combo in itertools.product(*values):
            c = set_params(cfg, names, combo)
            res = score(strata, c, mode=a.mode, replay=a.replay, executor=ex,
                        n_sim=a.n_sim, ci=False)
            summ = summarise(res, core_only=False)
            points.append(dict(values=dict(zip(names, combo)), summary=summ,
                               strata=[{k: r.get(k) for k in (
                                   "session", "file", "side", "N",
                                   "pulse_us", "band", "n", "texture",
                                   "texture_track_comparable",
                                   *SUMMARY_KEYS)}
                                   for r in res]))
            print(f"{dict(zip(names, combo))}: ({time.time() - t0:.0f} s)",
                  flush=True)
            _print_summary(summ)
    finally:
        if ex is not None:
            ex.shutdown()
    out.mkdir(parents=True, exist_ok=True)
    (out / "scan.json").write_text(json.dumps(dict(
        format="blueboat_seabed_scan/1", config=desc, grid=dict(zip(
            names, values)), mode=a.mode, replay=a.replay, files=meta,
        points=points, seconds=round(time.time() - t0, 1)), indent=1,
        default=float) + "\n")
    print(f"wrote {out / 'scan.json'}")
    return 0


def cmd_envelope(a) -> int:
    """NC #4: the profile's FBR envelope against the shipped profile's
    (or ``--reference``); exit 1 on any regression. Synthetic: no field data."""
    from ..sonar.config import SonarConfig, find_default_sonar_yaml
    cfg, desc = load_config(a.profile, a.artifact, a.set or [],
                            a.allow_base_mismatch)
    ref_cfg = SonarConfig.from_yaml(a.reference or find_default_sonar_yaml())
    ex = _executor(a.workers)
    try:
        ref = fbr_envelope(ref_cfg, executor=ex, n=a.n)
        env = fbr_envelope(cfg, executor=ex, n=a.n)
    finally:
        if ex is not None:
            ex.shutdown()
    reg = envelope_regressions(env, ref)
    doc = dict(format="blueboat_fbr_envelope/1", config=desc,
               reference=str(a.reference or "default_sonar.yaml"),
               min_detection=ENVELOPE_MIN_DETECTION,
               cases=[dict(acquisition=list(k[0]), altitude_m=k[1],
                           reflectivity=k[2], reference=ref[k],
                           detection=env.get(k)) for k in sorted(ref)],
               regressions=[dict(acquisition=list(k[0]), altitude_m=k[1],
                                 reflectivity=k[2], reference=b, detection=c)
                            for k, b, c in reg])
    if a.out:
        out = Path(a.out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"{len(env)} envelope cases, {len(reg)} regression(s) against "
          f"{doc['reference']}")
    for k, b, c in reg:
        print(f"  {k[0][0] // 1000} m / {k[0][1]} bins / {k[0][2]:g} us at "
              f"{k[1]:g} m, reflectivity {k[2]:g}: {c:.2f} (reference {b:.2f})")
    return 1 if reg else 0


def add_parsers(sub) -> None:
    def common(p):
        p.add_argument("--manifest", required=True)
        p.add_argument("--root", required=True,
                       help="the corpus root the manifest paths are under")
        p.add_argument("--out", required=True)
        p.add_argument("--profile", help="sonar profile (default: shipped)")
        p.add_argument("--artifact", help="calibration artifact to compose")
        p.add_argument("--allow-base-mismatch", action="store_true")
        p.add_argument("--set", action="append", metavar="model.X=V",
                       help="override after composition (recorded)")
        p.add_argument("--session", action="append",
                       help="restrict to these session ids")
        p.add_argument("--gain", type=int, default=4)
        p.add_argument("--replay", choices=("none", "attitude"),
                       default="attitude")
        p.add_argument("--workers", type=int,
                       default=max((os.cpu_count() or 2) - 2, 1))
        p.add_argument("--cache", help="reduction cache directory "
                                       "(outside the corpus)")
    s = sub.add_parser("score", help="score a profile on real sessions")
    common(s)
    s.add_argument("--roles", default="calibration,validation")
    s.add_argument("--mode", choices=("noisy", "expected", "both"),
                   default="both")
    s.add_argument("--n-sim", type=int, default=N_SIM)
    s.add_argument("--label", default="")
    s.set_defaults(fn=cmd_score)
    f = sub.add_parser("fit", help="least-squares fit on calibration "
                                   "sessions (draft artifact)")
    common(f)
    f.add_argument("--param", action="append", required=True)
    f.add_argument("--metric", choices=("level", "nadir"), default="level")
    f.add_argument("--max-bins", type=int, default=24)
    f.add_argument("--nadir-max-m", type=float, default=NADIR_MAX_M,
                   help="nadir metric: slant window min(this, 0.7 alt)")
    f.add_argument("--all-strata", action="store_true",
                   help="fit non-core strata too")
    f.add_argument("--allow-reuse", metavar="REASON",
                   help="fit on validation/test sessions, deliberately")
    f.add_argument("--artifact-id")
    f.set_defaults(fn=cmd_fit)
    g = sub.add_parser("scan", help="score a grid of parameter values")
    common(g)
    g.add_argument("--grid", action="append", required=True,
                   metavar="model.X=v1,v2,...")
    g.add_argument("--roles", default="calibration")
    g.add_argument("--mode", choices=("noisy", "expected", "both"),
                   default="noisy")
    g.add_argument("--n-sim", type=int, default=N_SIM)
    g.add_argument("--all-strata", action="store_true")
    g.set_defaults(fn=cmd_scan)
    v = sub.add_parser("envelope", help="NC #4: FBR envelope vs the shipped "
                                        "profile (synthetic, no field data)")
    v.add_argument("--profile")
    v.add_argument("--artifact")
    v.add_argument("--allow-base-mismatch", action="store_true")
    v.add_argument("--set", action="append", metavar="model.X=V")
    v.add_argument("--reference", help="reference profile (default: shipped)")
    v.add_argument("--n", type=int, default=60, help="pings per case")
    v.add_argument("--workers", type=int,
                   default=max((os.cpu_count() or 2) - 2, 1))
    v.add_argument("--out", help="JSON report")
    v.set_defaults(fn=cmd_envelope)
