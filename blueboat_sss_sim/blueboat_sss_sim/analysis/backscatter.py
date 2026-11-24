"""Seabed reflectivity from a recording -- the calibrated forward model
inverted per bin.

Every profile bin past the first bottom return is turned into an estimate
of the renderer's own ``reflectivity`` at the ground point it insonified,
by removing what the simulator would have reported for a flat floor of
reflectivity 1 at the same slant range, altitude and roll:

    reported dB   = 10 log10(power) + calibration_db_offset + gain_step_db
                    (``sonar/encoder.py``, the wire convention)
    power (rho=1) = sin^p(theta_g) . beam_weight . net_range_response
                    (``sonar/acoustics.py`` on the flat-floor geometry of
                    ``renderer._ping_geometry``: cos(theta_i) = sin(theta_g))
    rho_hat       = 10^((dB - gain_step - model_dB) / 10)

so ``10 log10(rho_hat)`` is Lambert's coefficient [B1] *in the renderer's
units* -- a world whose reflectivity map carries it renders the recording
back. The absolute level inherits the calibration's own AGC caveat
(``docs/sonar_model.md`` par. 6: +-6 dB); within one recording the
relative map is good to the 1-2 dB the speckle statistics allow.

Gates (all per bin): bins at or below the receiver floor plus a margin
are shadow / no return and carry nothing; the near-nadir cone inside
``2 x specular_width_deg`` of the normal is the specular rise Lambert
does not describe ([B2] par. 12.1.1, "data rise above Lambert's law for
steep grazing angles"); grazing angles under ``min_grazing_deg`` are
down-weighted, the inversion being ill-conditioned there.

Angle normalisation: Lambert's exponent is not determined by the corpus
(par. 6) and the device's auto-gain walks slowly with the scene, so by
default the model-inverted estimate is regularised against the recording
itself in the spirit of Coiras et al. 2007 [S3] eq. (11) (the beam
pattern as the median over equal grazing angles) -- but in its
**one-parameter form**: the recording's per-angle medians are fitted
with a single slope in ``log10 sin(theta_g)`` (what a wrong Lambert
exponent or TVG fraction leaves behind) and that slope alone is removed.
A per-degree median would also follow a material patch that happens to
occupy one angle band and eat its contrast (measured on the smoke's mud
capsule: -1.9 dB instead of -2.6); a slope cannot. ``"model"`` keeps the
calibrated model alone (what the identity gate on simulated data checks).

Speckle: a single-look bin is Exp(1)-distributed about its mean, so the
mean of its dB sits ``10 log10(e) . (psi(L) - ln L)`` below the mean's dB
(-2.51 dB at L = 1); the per-cell dB mean is corrected by that constant
(``speckle_looks``), which is what keeps the estimator streaming -- no
sorted medians over millions of bins -- and robust enough: a few bright
object bins move a cell's dB mean by well under a decibel.

ROS-free; imports only the sonar model, the calibration helpers and the
recording reader. Results are cached per (file, mtime, size, anchor,
cell, config) like ``bathymetry.load_track``.
"""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from ..core.geo import GeoAnchor
from ..core.geometry import compass_deg_to_enu_yaw
from ..core.types import Side
from ..sonar import acoustics
from ..sonar.config import CALIBRATION_GAIN_INDEX, SonarModelConfig
from ..sonar.encoder import ANALOG_GAIN_TABLE, gain_step_db
from .calibration import scale_to_db
from .svlog_reader import ProfileRecord, SvlogStreams, pair_by_order, read_svlog

__all__ = ["InversionConfig", "BinSamples", "MuRaster", "flat_floor_model_db",
           "invert_bins", "backscatter_bins", "mu_raster", "summarize",
           "resample_to_grid", "receiver_floor_db"]

DB_PER_NEPER = 10.0 / math.log(10.0)


@dataclass(frozen=True)
class InversionConfig:
    """Knobs of the per-bin inversion (documented in the module doc)."""

    floor_margin_db: float = 4.0        # shadow: within this of the floor
    min_grazing_deg: float = 2.0        # below: weight x 0.25
    low_grazing_weight: float = 0.25
    specular_exclusion_factor: float = 3.5   # theta_n < f . specular_width:
                                        # the lobe still adds +140 % at 2.5
                                        # sigma, +7 % at 3.5 (measured on the
                                        # smoke's uniform floor)
    angle_norm: str = "empirical"       # "empirical" (Coiras eq. 11) | "model"
    angle_bin_deg: float = 1.0
    speckle_looks: int | None = 1       # dB-mean bias correction; None = off
    gain_index_ref: int = CALIBRATION_GAIN_INDEX

    def key(self) -> tuple:
        return (self.floor_margin_db, self.min_grazing_deg,
                self.low_grazing_weight, self.specular_exclusion_factor,
                self.angle_norm, self.angle_bin_deg, self.speckle_looks,
                self.gain_index_ref)


def speckle_db_bias(looks: int | None) -> float:
    """``E[10 log10 X] - 10 log10 E[X]`` for a Gamma(L, 1/L) speckle law:
    ``10 log10(e) (psi(L) - ln L)``; 0 when ``looks`` is ``None``."""
    if looks is None or looks <= 0:
        return 0.0
    from scipy.special import digamma
    return float(DB_PER_NEPER * (digamma(looks) - math.log(looks)))


# ---------------------------------------------------------------- model
def flat_floor_model_db(slant_m: np.ndarray, altitude_m: float,
                        cfg: SonarModelConfig, roll_toward_side: float = 0.0,
                        include_specular: bool = False,
                        pulse_s: float | None = None) -> np.ndarray:
    """Reported dB (calibration gain) of the renderer's per-bin mean for a
    flat floor of reflectivity 1 at ``altitude_m`` -- diffuse only unless
    ``include_specular``. NaN inside the water column (``slant <= a``).
    ``pulse_s`` is the recording's transmit pulse, so the pulse-level terms
    (``acoustics.pulse_level_factors``) invert as they render."""
    r = np.asarray(slant_m, dtype=np.float64)
    a = float(altitude_m)
    with np.errstate(invalid="ignore", divide="ignore"):
        sin_g = np.where(r > a, a / np.maximum(r, 1e-9), np.nan)
        grazing = np.arcsin(np.clip(sin_g, 0.0, 1.0))
        cos_inc = sin_g                       # flat floor: theta_i = 90 - theta_g
        # Same composition the renderer uses (acoustics.surface_response), so
        # the inversion cannot drift from the forward model it inverts.
        sr = acoustics.surface_response(np.ones_like(r), cos_inc, grazing, r,
                                        cfg, roll_toward_side, pulse_s=pulse_s)
        power = sr.diffuse * sr.beam * sr.range_diffuse
        if include_specular:
            # The coherent lobe carries its own spreading law, exactly as the
            # renderer applies it (acoustics.two_way_loss).
            power = power + sr.coherent * sr.beam * sr.range_coherent
        db = 10.0 * np.log10(np.maximum(power, 1e-300)) + cfg.calibration_db_offset
    return np.where(r > a, db, np.nan)


def receiver_floor_db(cfg: SonarModelConfig) -> float:
    """The calibrated receiver floor in reported dB at the calibration
    gain (``noise_floor`` through the encoder's level constant)."""
    return 10.0 * math.log10(cfg.noise_floor) + cfg.calibration_db_offset


def _slant_axis(start_mm: int, length_mm: int, num_results: int) -> np.ndarray:
    n = max(int(num_results), 1)
    return (float(start_mm) + np.arange(n) / max(n - 1, 1) * float(length_mm)) / 1000.0


def invert_bins(pwr_results: np.ndarray, min_pwr_db: float, max_pwr_db: float,
                start_mm: int, length_mm: int, num_results: int,
                gain_index: int, altitude_m: float, cfg: SonarModelConfig,
                inv: InversionConfig = InversionConfig(), *,
                floor_db: float | None = None,
                roll_toward_side: float = 0.0,
                pulse_s: float | None = None) -> dict[str, np.ndarray]:
    """One profile -> per-bin arrays: ``i_lin`` (rho-hat, NaN where the bin
    carries nothing), ``weight``, ``grazing_deg``, ``ground_m`` (horizontal
    distance from the transducer), ``db_g4`` (reported dB brought to the
    calibration gain), ``model_db``, ``shadow`` (bool: at the floor).
    ``floor_db`` defaults to the calibrated receiver floor."""
    if int(gain_index) not in ANALOG_GAIN_TABLE:
        raise ValueError(f"gain_index {gain_index} outside the modelled ladder")
    n = int(num_results)
    r = _slant_axis(start_mm, length_mm, n)
    a = float(altitude_m)
    db = scale_to_db(pwr_results, float(min_pwr_db), float(max_pwr_db))
    # Brought to the calibration gain, where the model's level constant
    # lives (``gain_step_db`` is already relative to that index).
    db_g4 = db - gain_step_db(int(gain_index))
    floor = receiver_floor_db(cfg) if floor_db is None else float(floor_db)
    # The model carries the specular lobe too (at rho = 1, where it is
    # S*(0.5+0.5) -- not proportional to rho, hence the exclusion cone
    # below): what leaks past the cone is then a second-order bias.
    model = flat_floor_model_db(r, a, cfg, roll_toward_side,
                                include_specular=True, pulse_s=pulse_s)
    with np.errstate(invalid="ignore", divide="ignore"):
        sin_g = np.where(r > a, a / np.maximum(r, 1e-9), np.nan)
        grazing = np.degrees(np.arcsin(np.clip(sin_g, 0.0, 1.0)))
        ground = np.sqrt(np.maximum(r * r - a * a, 0.0))
    past = r > a
    shadow = past & (db_g4 <= floor + inv.floor_margin_db)
    nadir = past & ((90.0 - grazing)
                    < inv.specular_exclusion_factor * cfg.specular_width_deg)
    ok = past & ~shadow & ~nadir & np.isfinite(model)
    # The floor is additive noise: take it out of the signal before the
    # ratio (a bin a few dB above it is mostly floor).
    lin_sig = 10.0 ** (db_g4 / 10.0) - 10.0 ** (floor / 10.0)
    i_lin = np.full(n, np.nan)
    i_lin[ok] = np.maximum(lin_sig[ok], 1e-12) / 10.0 ** (model[ok] / 10.0)
    weight = np.where(ok, np.sin(np.radians(np.nan_to_num(grazing)))
                      * acoustics.beam_weight(np.radians(np.nan_to_num(grazing)),
                                              cfg, roll_toward_side), 0.0)
    weight = np.where(ok & (grazing < inv.min_grazing_deg),
                      weight * inv.low_grazing_weight, weight)
    return {"i_lin": i_lin, "weight": weight, "grazing_deg": grazing,
            "ground_m": ground, "db_g4": db_g4, "model_db": model,
            "shadow": shadow, "slant_m": r}


# --------------------------------------------------------------- streams
def _profile_roll(streams: SvlogStreams, profiles: list[ProfileRecord],
                  index: np.ndarray) -> np.ndarray:
    """Vehicle roll [rad] for the profiles at ``index``, from the attitude
    stream on the profile clock (live) or by file order (frozen clock);
    zeros without attitude."""
    att = streams.attitude
    if not att or len(index) == 0:
        return np.zeros(len(index))
    if streams.clock_frozen:
        j = pair_by_order(np.array([profiles[i].order for i in index]),
                          np.array([a.order for a in att]))
        return np.array([att[k].roll for k in j], dtype=np.float64)
    t = np.array([a.time_boot_ms + streams.skew_ms for a in att], dtype=np.float64)
    o = np.argsort(t, kind="stable")
    t, roll = t[o], np.array([att[i].roll for i in o], dtype=np.float64)
    pt = np.array([profiles[i].timestamp_ms for i in index], dtype=np.float64)
    return np.interp(pt, t, roll)


def _iter_placed_profiles(path: str | Path, anchor: GeoAnchor,
                          cfg: SonarModelConfig, inv: InversionConfig,
                          floor_db: float
                          ) -> Iterator[tuple[ProfileRecord, float, float,
                                              float, dict[str, np.ndarray]]]:
    """Yield ``(profile, east, north, look_enu, bins)`` for every profile
    that has a GPS position and a fused altitude."""
    from .bathymetry import load_track
    from .svlog_reader import gps_track, profile_heading_deg
    s = read_svlog(path, with_power=True)
    track, _ = load_track(path)
    alt_by_ping = {int(k): float(a) for k, a in zip(track.ping_number,
                                                     track.altitude_m)
                   if np.isfinite(a)}
    _, idx, lat, lon = gps_track("", streams=s)
    if len(idx) == 0:
        return
    e_all, n_all = anchor.to_en(lat, lon)
    rolls = _profile_roll(s, s.profiles, idx)
    # the autopilot's heading, never the ping's own vehicle_heading_deg
    # (an unreferenced device heading on real recordings, 2026-11-01)
    heads = profile_heading_deg(s, idx)
    off = track.counter_offset
    for j, i in enumerate(idx):
        p = s.profiles[i]
        key = int(p.ping_number) - (off if p.side is Side.STARBOARD else 0)
        alt = alt_by_ping.get(key)
        if alt is None or int(p.gain_index) not in ANALOG_GAIN_TABLE:
            continue
        look = compass_deg_to_enu_yaw(float(heads[j])) + p.side.sign * np.pi / 2.0
        bins = invert_bins(p.pwr_results, p.min_pwr_db, p.max_pwr_db,
                           p.start_mm, p.length_mm, p.num_results,
                           p.gain_index, alt, cfg, inv, floor_db=floor_db,
                           roll_toward_side=-p.side.sign * float(rolls[j]),
                           pulse_s=(float(p.pulse_duration_sec)
                                    if p.pulse_duration_sec > 0 else None))
        yield p, float(e_all[j]), float(n_all[j]), float(look), bins


#: A file's 1st percentile is accepted as its floor only when it sits this
#: far under the median: single-look speckle alone puts p1 about 18 dB
#: below the median of a signal-only swath (Exp(1): p1 = -20 dB about the
#: mean), so a pile-up deeper than this cannot be speckle -- it is the
#: receiver / ambient floor the shadow gate is looking for.
FLOOR_PILEUP_DB = 25.0


def _file_floor_db(path: str | Path, cfg: SonarModelConfig,
                   inv: InversionConfig, step: int = 10) -> float:
    """The floor the shadow gate uses, at the calibration gain: the
    recording's own 1st percentile of the reported dB past nadir (every
    ``step``-th profile) when that is a genuine pile-up
    (``FLOOR_PILEUP_DB`` under the median), else the calibrated receiver
    floor. Never below the calibrated floor."""
    from .bathymetry import load_track
    s = read_svlog(path, with_power=True)
    track, _ = load_track(path)
    alt_by_ping = {int(k): float(a) for k, a in zip(track.ping_number,
                                                     track.altitude_m)
                   if np.isfinite(a)}
    off = track.counter_offset
    vals = []
    for p in s.profiles[::max(step, 1)]:
        key = int(p.ping_number) - (off if p.side is Side.STARBOARD else 0)
        alt = alt_by_ping.get(key)
        if alt is None or int(p.gain_index) not in ANALOG_GAIN_TABLE:
            continue
        r = _slant_axis(p.start_mm, p.length_mm, p.num_results)
        db = scale_to_db(p.pwr_results, p.min_pwr_db, p.max_pwr_db) \
            - gain_step_db(int(p.gain_index))
        vals.append(db[r > alt])
    if not vals:
        return receiver_floor_db(cfg)
    v = np.concatenate(vals)
    if v.size == 0:
        return receiver_floor_db(cfg)
    p1, p50 = np.percentile(v, [1.0, 50.0])
    if float(p50 - p1) < FLOOR_PILEUP_DB:
        return receiver_floor_db(cfg)
    return max(float(p1), receiver_floor_db(cfg))


# ------------------------------------------------------------- samples
@dataclass
class BinSamples:
    """Every inverted bin placed on the ground (design/world metres)."""

    east: np.ndarray
    north: np.ndarray
    i_lin: np.ndarray
    weight: np.ndarray
    grazing_deg: np.ndarray
    ping: np.ndarray
    side: np.ndarray            # int8: +1 port, -1 starboard
    floor_db: float = 0.0

    def __len__(self) -> int:
        return len(self.i_lin)


def backscatter_bins(path: str | Path, anchor: GeoAnchor, *,
                     cfg: SonarModelConfig | None = None,
                     inv: InversionConfig = InversionConfig(),
                     bin_step: int = 1) -> BinSamples:
    """Per-bin reflectivity samples of a recording, placed abeam of the
    track along each side's look direction (the vehicle heading rotated
    ``side.sign x 90``, never the packet's mounting angle). Every
    ``bin_step``-th bin; bins that carry nothing are left out."""
    cfg = cfg or SonarModelConfig()
    floor = _file_floor_db(path, cfg, inv)
    E, N, I, W, G, P, S = [], [], [], [], [], [], []
    for p, e0, n0, look, b in _iter_placed_profiles(path, anchor, cfg, inv, floor):
        sel = np.flatnonzero(np.isfinite(b["i_lin"]))[::max(bin_step, 1)]
        if sel.size == 0:
            continue
        g = b["ground_m"][sel]
        E.append(e0 + g * np.cos(look))
        N.append(n0 + g * np.sin(look))
        I.append(b["i_lin"][sel])
        W.append(b["weight"][sel])
        G.append(b["grazing_deg"][sel])
        P.append(np.full(sel.size, int(p.ping_number), dtype=np.int64))
        S.append(np.full(sel.size, int(p.side.sign), dtype=np.int8))
    if not E:
        z = np.empty(0)
        return BinSamples(z, z, z, z, z, np.empty(0, np.int64),
                          np.empty(0, np.int8), floor)
    return BinSamples(np.concatenate(E), np.concatenate(N), np.concatenate(I),
                      np.concatenate(W), np.concatenate(G), np.concatenate(P),
                      np.concatenate(S), floor)


# -------------------------------------------------------------- raster
@dataclass
class MuRaster:
    """Lambert coefficient per ground cell, ``[ny, nx]`` row 0 = min north
    (NaN where nothing was insonified), with its provenance."""

    mu_db: np.ndarray
    count: np.ndarray
    spread_db: np.ndarray
    origin_x: float
    origin_y: float
    cell_m: float
    n_pings: int = 0
    n_bins: int = 0
    floor_db: float = 0.0
    angle_norm: str = "empirical"
    gain_index_ref: int = CALIBRATION_GAIN_INDEX
    angle_curve: dict[str, list[float]] = field(default_factory=dict)
    angle_slope_db_per_decade: float = 0.0

    @property
    def extent(self) -> tuple[float, float, float, float]:
        ny, nx = self.mu_db.shape
        return (self.origin_x, self.origin_y,
                self.origin_x + (nx - 1) * self.cell_m,
                self.origin_y + (ny - 1) * self.cell_m)


def _angle_curve(path, anchor, cfg, inv, floor) -> tuple[np.ndarray, np.ndarray, float]:
    """Per-grazing-angle medians of the model-inverted rho-hat (dB) on
    ``angle_bin_deg`` bins (from a 0.5 dB histogram), the count-weighted
    slope of those medians against ``log10 sin(theta_g)``, and the
    correction to remove: that slope alone, zero-mean over the angles that
    carry data, so the file's level is untouched. Returns ``(angle_deg,
    correction_db, slope_db_per_decade)``."""
    step = float(inv.angle_bin_deg)
    n_ang = int(np.ceil(90.0 / step)) + 1
    db_lo, db_hi, db_step = -60.0, 40.0, 0.5
    n_db = int((db_hi - db_lo) / db_step) + 1
    hist = np.zeros((n_ang, n_db), dtype=np.int64)
    for _p, _e, _n, _look, b in _iter_placed_profiles(path, anchor, cfg, inv, floor):
        ok = np.isfinite(b["i_lin"]) & (b["weight"] > 0.0)
        if not ok.any():
            continue
        ai = np.clip(np.round(b["grazing_deg"][ok] / step).astype(int), 0, n_ang - 1)
        di = np.clip(np.round((10.0 * np.log10(b["i_lin"][ok]) - db_lo)
                              / db_step).astype(int), 0, n_db - 1)
        np.add.at(hist, (ai, di), 1)
    counts = hist.sum(axis=1)
    med = np.full(n_ang, np.nan)
    for k in np.flatnonzero(counts >= 50):
        c = np.cumsum(hist[k])
        med[k] = db_lo + float(np.searchsorted(c, 0.5 * c[-1])) * db_step
    angles = np.arange(n_ang) * step
    have = np.isfinite(med)
    if have.sum() < 2:
        return angles, np.zeros(n_ang), 0.0
    w = counts.astype(np.float64)
    x_all = np.log10(np.sin(np.radians(np.maximum(angles, 0.5))))
    x = x_all[have]
    slope = float(np.polyfit(x, med[have], 1, w=np.sqrt(w[have]))[0]) \
        if have.sum() >= 3 else 0.0
    x_bar = float(np.average(x, weights=w[have]))
    corr = slope * (x_all - x_bar)
    return angles, corr, slope


def _accumulate(path, anchor, cfg, inv, floor, cell_m, corr_fn):
    """Streaming per-cell sums of the (angle-normalised) dB estimate."""
    sums: dict[str, Any] = {"keys": [], "w": [], "wd": [], "wd2": []}
    xs, ys = [], []
    n_pings = n_bins = 0
    chunks = []
    for _p, e0, n0, look, b in _iter_placed_profiles(path, anchor, cfg, inv, floor):
        ok = np.isfinite(b["i_lin"]) & (b["weight"] > 0.0)
        if not ok.any():
            continue
        n_pings += 1
        n_bins += int(ok.sum())
        g = b["ground_m"][ok]
        db = 10.0 * np.log10(b["i_lin"][ok]) - corr_fn(b["grazing_deg"][ok])
        chunks.append((e0 + g * np.cos(look), n0 + g * np.sin(look),
                       db, b["weight"][ok]))
    if not chunks:
        return None
    e = np.concatenate([c[0] for c in chunks]).astype(np.float32)
    n = np.concatenate([c[1] for c in chunks]).astype(np.float32)
    d = np.concatenate([c[2] for c in chunks]).astype(np.float32)
    w = np.concatenate([c[3] for c in chunks]).astype(np.float32)
    del chunks
    ox = math.floor(float(e.min()) / cell_m) * cell_m
    oy = math.floor(float(n.min()) / cell_m) * cell_m
    ci = np.round((e - ox) / cell_m).astype(np.int64)
    cj = np.round((n - oy) / cell_m).astype(np.int64)
    nx, ny = int(ci.max()) + 1, int(cj.max()) + 1
    flat = cj * nx + ci
    W = np.bincount(flat, weights=w, minlength=nx * ny)
    WD = np.bincount(flat, weights=w * d, minlength=nx * ny)
    WD2 = np.bincount(flat, weights=w * d * d, minlength=nx * ny)
    C = np.bincount(flat, minlength=nx * ny)
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.where(W > 0, WD / W, np.nan)
        var = np.where(W > 0, WD2 / W - mean * mean, np.nan)
    return (mean.reshape(ny, nx), np.sqrt(np.maximum(var, 0.0)).reshape(ny, nx),
            C.reshape(ny, nx), ox, oy, n_pings, n_bins)


@functools.lru_cache(maxsize=4)
def _mu_bundle(path: str, mtime_ns: int, size: int, lat0: float, lon0: float,
               cell_m: float, inv_key: tuple, cfg_key: tuple) -> MuRaster:
    inv = InversionConfig(*inv_key)
    cfg = SonarModelConfig(**dict(cfg_key)) if cfg_key else SonarModelConfig()
    anchor = GeoAnchor(lat0=lat0, lon0=lon0)
    floor = _file_floor_db(path, cfg, inv)
    slope = 0.0
    curve: dict[str, list[float]] = {}
    if inv.angle_norm == "empirical":
        angles, corr, slope = _angle_curve(path, anchor, cfg, inv, floor)
        curve = {"grazing_deg": [float(a) for a in angles],
                 "correction_db": [float(c) for c in corr]}
        # The layer above is a slope removal; a per-degree median (Coiras'
        # full form) would follow a material patch confined to one angle
        # band and remove its contrast with it.

        def corr_fn(g: np.ndarray) -> np.ndarray:
            return np.interp(g, angles, corr)
    elif inv.angle_norm == "model":
        def corr_fn(g: np.ndarray) -> np.ndarray:
            return np.zeros_like(g)
    else:
        raise ValueError(f"angle_norm '{inv.angle_norm}' (empirical | model)")
    acc = _accumulate(path, anchor, cfg, inv, floor, cell_m, corr_fn)
    if acc is None:
        raise ValueError(f"{Path(path).name}: no profile bin past the bottom "
                         "return carries a usable level -- nothing to invert")
    mean, spread, count, ox, oy, n_pings, n_bins = acc
    mean = mean - speckle_db_bias(inv.speckle_looks)
    return MuRaster(mean, count, spread, float(ox), float(oy), float(cell_m),
                    n_pings=n_pings, n_bins=n_bins, floor_db=float(floor),
                    angle_norm=inv.angle_norm, gain_index_ref=inv.gain_index_ref,
                    angle_curve=curve, angle_slope_db_per_decade=slope)


def _cfg_key(cfg: SonarModelConfig | None) -> tuple:
    if cfg is None:
        return ()
    d = SonarModelConfig().__dict__
    return tuple(sorted((k, v) for k, v in cfg.__dict__.items()
                        if not isinstance(v, (list, dict, np.ndarray))
                        and d.get(k) != v))


def mu_raster(path: str | Path, anchor: GeoAnchor, cell_m: float = 1.0,
              inv: InversionConfig = InversionConfig(),
              cfg: SonarModelConfig | None = None) -> MuRaster:
    """The recording's Lambert-coefficient raster on a ``cell_m`` lattice
    about ``anchor`` (cached per file / anchor / cell / config)."""
    p = Path(path)
    st = p.stat()
    return _mu_bundle(str(p), st.st_mtime_ns, st.st_size, float(anchor.lat0),
                      float(anchor.lon0), float(cell_m), inv.key(),
                      _cfg_key(cfg))


def resample_to_grid(r: MuRaster, grid, field_name: str = "mu_db",
                     order: int = 1) -> np.ndarray:
    """Resample a raster field onto ``grid`` (a ``GridSpec``); NaN outside
    the raster or where it is empty.

    ``order`` is the spline order of the interpolation: 1 (bilinear, the
    default) or 0 (nearest). A survey raster is metre-scale and the world
    grid is decimetre-scale, so nearest resampling stamped every measured
    cell as a hard 1 m square and the reflectivity map came out blocky
    (reported 2026-09-18). Bilinear costs nothing and is what the measured
    field deserves: the cell value is a mean over ~10^3 bins, not a
    boundary. Empty cells stay NaN and are *not* interpolated across --
    a cell next to a gap keeps its own value rather than borrowing one."""
    src = np.asarray(getattr(r, field_name), dtype=np.float64)
    ny, nx = src.shape
    xs = grid.origin_x + np.arange(grid.nx) * grid.resolution
    ys = grid.origin_y + np.arange(grid.ny) * grid.resolution
    fi = (xs - r.origin_x) / r.cell_m
    fj = (ys - r.origin_y) / r.cell_m
    if order <= 0:
        ci, cj = np.round(fi).astype(int), np.round(fj).astype(int)
        out = np.full((grid.ny, grid.nx), np.nan)
        okx, oky = (ci >= 0) & (ci < nx), (cj >= 0) & (cj < ny)
        if okx.any() and oky.any():
            out[np.ix_(oky, okx)] = src[np.ix_(cj[oky], ci[okx])]
        return out
    from scipy import ndimage
    jj, ii = np.meshgrid(fj, fi, indexing="ij")
    ok = np.isfinite(src)
    # Normalised convolution: interpolate value * validity and validity,
    # then divide, so a NaN neighbour dilutes nothing.
    num = ndimage.map_coordinates(np.where(ok, src, 0.0), [jj, ii], order=1,
                                  mode="nearest")
    den = ndimage.map_coordinates(ok.astype(np.float64), [jj, ii], order=1,
                                  mode="nearest")
    inside = ((ii >= -0.5) & (ii <= nx - 0.5)
              & (jj >= -0.5) & (jj <= ny - 0.5))
    with np.errstate(invalid="ignore", divide="ignore"):
        out = np.where(inside & (den > 0.5), num / np.maximum(den, 1e-9),
                       np.nan)
    return out


def summarize(r: MuRaster, reference: str = "sand",
              min_count: int = 20) -> dict[str, Any]:
    """The metadata block: how much was measured and at what level. The
    percentiles are taken over cells seen by at least ``min_count`` bins
    (a cell grazed by a handful of bins is one speckle draw, not a
    level)."""
    v = r.mu_db[np.isfinite(r.mu_db) & (r.count >= int(min_count))]
    p = (np.percentile(v, [5, 50, 95]) if v.size else [float("nan")] * 3)
    return {"pings": int(r.n_pings), "bins": int(r.n_bins),
            "cells": int(np.isfinite(r.mu_db).sum()), "cell_m": float(r.cell_m),
            "mu_db_p5": float(p[0]), "mu_db_p50": float(p[1]),
            "mu_db_p95": float(p[2]), "floor_db": float(r.floor_db),
            "reference": str(reference), "gain_index_ref": int(r.gain_index_ref),
            "angle_norm": str(r.angle_norm),
            "angle_slope_db_per_decade": float(r.angle_slope_db_per_decade)}
