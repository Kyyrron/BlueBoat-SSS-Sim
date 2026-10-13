"""Depth soundings from a ``.svlog`` recording (ROS-free, Qt-free).

The Omniscan's first bottom return (FBR) is the shortest slant range to
the seabed -- for a side-looking transducer a few centimetres off the
hull's centreline, that is the water depth under the boat. One sounding
per ping, placed at the GPS fix the recording carries in its id-150
``GLOBAL_POSITION_INT`` envelopes, is what this module derives::

    read_svlog  ->  altitude_track  ->  place_soundings  ->  decimate
      (packets)     (per ping_number)    (lat/lon -> EN,     (cell medians
                                          + submersion)       for the fit)

Every step is re-expressed from the documented real-side behaviour
(``sss_processor_node`` / the GCS replay loader) and imports nothing from
``BlueBoat-SSS`` (CM-3 / NC #8); recordings are opened read-only (CM-7).

Detection reuses :func:`blueboat_sss_sim.analysis.calibration.fbr_bin`
(the processor's threshold rule, constants copied); the bin -> metres
conversion is the *real* side's ``start_mm + i/(n-1)*length_mm``. The
two sides detect independently and disagree on 50-85 % of the corpus's
pings by more than 0.3 m, so they are fused robustly rather than gated:
a running median per side rejects gross outliers, agreeing pairs take
the larger of the two (the real tracker's rule -- an early false trigger
is the failure it guards against), a lone or disagreeing side is kept
only when it sits within ``MAX_DEV_M`` of the running fused altitude.

Depth below the still-water surface = FBR altitude + the transducer's
submersion (``sensor_depth_m`` of the sonar config, 0.15 m) + a
``datum_offset_m`` the operator enters for tide / draft (positive when
the water stood higher than the reference level during the survey).
"""

from __future__ import annotations

import functools
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from ..core.geo import GeoAnchor, latlon_to_local_en
from ..core.types import Side
from .calibration import fbr_bin, scale_to_db
from .svlog_reader import ProfileRecord, SvlogStreams, read_svlog

#: Running-median window (pings) per side.
WINDOW_PINGS = 15
#: A side's detection farther than this from its running median is a
#: gross outlier (wall echo, ringing) and is dropped.
MAX_DEV_M = 0.5
#: Two sides within this agree; the larger is taken (the real tracker's
#: ``max(port, stbd)``).
SIDE_AGREE_M = 0.3
#: Minimum pings for a survey to be usable at all.
MIN_PINGS = 20


def sensor_depth_default() -> float:
    """The simulator's transducer submersion (``sonar/config.py``)."""
    from ..sonar.config import SonarModelConfig
    return float(SonarModelConfig().sensor_depth_m)


# ------------------------------------------------------------- detection
def slant_m(i: int, start_mm: float, length_mm: float, num_results: int
            ) -> float:
    """Bin index -> slant range in metres, the real side's formula."""
    return (float(start_mm)
            + float(i) / max(int(num_results) - 1, 1) * float(length_mm)) / 1000.0


def fbr_slant_m(p: ProfileRecord) -> float:
    """First-bottom-return slant range of one profile (NaN on a miss)."""
    if p.pwr_results is None:
        raise ValueError("profile read without power -- use with_power=True")
    db = scale_to_db(p.pwr_results, p.min_pwr_db, p.max_pwr_db)
    i = fbr_bin(db)
    if i is None:
        return float("nan")
    return slant_m(i, p.start_mm, p.length_mm, p.num_results)


def counter_offset(port: list[ProfileRecord], stbd: list[ProfileRecord]) -> int:
    """``stbd.ping_number - port.ping_number`` for pings written next to
    each other (median over the file): the two devices number their
    pings independently with a constant per-power-up offset."""
    if not port or not stbd:
        return 0
    pt = np.array([p.timestamp_ms for p in port], dtype=np.float64)
    st = np.array([p.timestamp_ms for p in stbd], dtype=np.float64)
    order = np.argsort(st, kind="stable")
    st, sn = st[order], np.array([stbd[i].ping_number for i in order])
    j = np.clip(np.searchsorted(st, pt), 0, len(st) - 1)
    pn = np.array([p.ping_number for p in port])
    return int(np.median(sn[j] - pn))


def _running_median(values: np.ndarray, window: int) -> np.ndarray:
    """Median filter over a sequence with NaNs (filled by the global
    median first, so a gap does not drag its neighbours)."""
    from scipy.ndimage import median_filter
    v = np.asarray(values, dtype=np.float64)
    if not np.isfinite(v).any():
        return v.copy()
    fill = np.where(np.isfinite(v), v, float(np.nanmedian(v)))
    return median_filter(fill, size=max(int(window) | 1, 3), mode="nearest")


@dataclass
class AltitudeTrack:
    """Per port ``ping_number`` (starboard aligned by the counter offset):
    the two sides' FBR slant ranges after outlier rejection, the fused
    altitude (NaN when neither side is usable) and a quality flag
    (2 both sides agree, 1 one side, 0 none)."""

    ping_number: np.ndarray
    order: np.ndarray            # packet index of the ping (port, else stbd)
    t_ms: np.ndarray             # device clock of the ping
    heading_deg: np.ndarray      # vehicle heading (compass) off the ping
    port_m: np.ndarray
    stbd_m: np.ndarray
    altitude_m: np.ndarray
    quality: np.ndarray
    counter_offset: int = 0
    n_profiles: int = 0
    n_detected: int = 0

    def __len__(self) -> int:
        return len(self.ping_number)


def altitude_track(profiles: list[ProfileRecord], *, window: int = WINDOW_PINGS,
                   max_dev_m: float = MAX_DEV_M,
                   side_agree_m: float = SIDE_AGREE_M) -> AltitudeTrack:
    """FBR altitudes per ping, both sides fused robustly."""
    port = [p for p in profiles if p.side is Side.PORT]
    stbd = [p for p in profiles if p.side is Side.STARBOARD]
    off = counter_offset(port, stbd)

    def side_series(recs: list[ProfileRecord], shift: int) -> dict[int, tuple]:
        if not recs:
            return {}
        raw = np.array([fbr_slant_m(p) for p in recs])
        med = _running_median(raw, window)
        keep = np.isfinite(raw) & (np.abs(raw - med) <= max_dev_m)
        out = {}
        for p, v, k in zip(recs, raw, keep):
            out[int(p.ping_number) - shift] = (float(v) if k else float("nan"),
                                              p, np.isfinite(v))
        return out

    ps = side_series(port, 0)
    ss = side_series(stbd, off)
    keys = sorted(set(ps) | set(ss))
    n = len(keys)
    pn = np.array(keys, dtype=np.int64)
    order = np.zeros(n, dtype=np.int64)
    t_ms = np.zeros(n, dtype=np.float64)
    hdg = np.zeros(n, dtype=np.float64)
    pm = np.full(n, np.nan)
    sm = np.full(n, np.nan)
    detected = 0
    for k, key in enumerate(keys):
        rec = ps.get(key) or ss.get(key)
        order[k], t_ms[k], hdg[k] = rec[1].order, rec[1].timestamp_ms, \
            rec[1].vehicle_heading_deg
        if key in ps:
            pm[k] = ps[key][0]
            detected += int(ps[key][2])
        if key in ss:
            sm[k] = ss[key][0]
            detected += int(ss[key][2])

    both = np.isfinite(pm) & np.isfinite(sm)
    agree = both & (np.abs(pm - sm) <= side_agree_m)
    alt = np.where(agree, np.maximum(pm, sm), np.nan)
    quality = np.where(agree, 2, 0).astype(np.int8)
    # Lone or disagreeing sides: accept the one nearest the running fused
    # altitude, within max_dev_m of it.
    ref = _running_median(alt, max(window, 21))
    for cand in (pm, sm):
        ok = np.isnan(alt) & np.isfinite(cand) & np.isfinite(ref) \
            & (np.abs(cand - ref) <= max_dev_m)
        better = ok & (np.isnan(alt) | (np.abs(cand - ref) < np.abs(alt - ref)))
        alt[better] = cand[better]
        quality[better] = 1
    return AltitudeTrack(pn, order, t_ms, hdg, pm, sm, alt, quality,
                         counter_offset=off, n_profiles=len(profiles),
                         n_detected=detected)


# -------------------------------------------------------------- placement
@dataclass
class Soundings:
    """Depth soundings in world EN metres (depth positive down, negative =
    above the still-water surface) with their provenance."""

    east: np.ndarray
    north: np.ndarray
    depth: np.ndarray
    t_ms: np.ndarray
    quality: np.ndarray
    lat: np.ndarray
    lon: np.ndarray
    clock_mode: str = "live"          # "live" | "file_order"

    def __len__(self) -> int:
        return len(self.depth)


def place_soundings(track: AltitudeTrack, streams: SvlogStreams,
                    anchor: GeoAnchor, *, sensor_depth_m: float,
                    datum_offset_m: float = 0.0) -> Soundings:
    """Pair every usable altitude with a GPS position and convert to depth."""
    good = np.isfinite(track.altitude_m)
    if not streams.gps:
        raise ValueError("the recording carries no GPS fixes "
                         "(GLOBAL_POSITION_INT) -- soundings cannot be placed")
    if streams.clock_frozen:
        from .svlog_reader import pair_by_order
        idx = pair_by_order(track.order[good],
                            np.array([g.order for g in streams.gps]))
        lat = np.array([streams.gps[j].lat for j in idx])
        lon = np.array([streams.gps[j].lon for j in idx])
        sel = np.flatnonzero(good)
        mode = "file_order"
    else:
        gt = np.array([g.time_boot_ms + streams.skew_ms for g in streams.gps],
                      dtype=np.float64)
        o = np.argsort(gt, kind="stable")
        gt = gt[o]
        glat = np.array([streams.gps[i].lat for i in o])
        glon = np.array([streams.gps[i].lon for i in o])
        inside = good & (track.t_ms >= gt[0]) & (track.t_ms <= gt[-1])
        sel = np.flatnonzero(inside)
        lat = np.interp(track.t_ms[sel], gt, glat)
        lon = np.interp(track.t_ms[sel], gt, glon)
        mode = "live"
    east, north = anchor.to_en(lat, lon)
    depth = track.altitude_m[sel] + float(sensor_depth_m) + float(datum_offset_m)
    return Soundings(np.asarray(east, dtype=np.float64),
                     np.asarray(north, dtype=np.float64), depth,
                     track.t_ms[sel], track.quality[sel], lat, lon, mode)


@dataclass
class Cells:
    """Along-track decimation of soundings: one median per square cell."""

    east: np.ndarray
    north: np.ndarray
    depth: np.ndarray
    count: np.ndarray
    cell_m: float

    def __len__(self) -> int:
        return len(self.depth)


def decimate(s: Soundings, *, cell_m: float | None = None,
             target_n: int = 2500) -> Cells:
    """Cell medians on a ``cell_m`` lattice; when ``cell_m`` is not given
    it starts at 1 m and grows by 1.5x until at most ``target_n`` cells
    remain (a global thin-plate spline stays well under a second)."""
    if len(s) == 0:
        return Cells(np.empty(0), np.empty(0), np.empty(0),
                     np.empty(0, int), float(cell_m or 1.0))
    cell = float(cell_m) if cell_m else 1.0
    while True:
        ci = np.round(s.east / cell).astype(np.int64)
        cj = np.round(s.north / cell).astype(np.int64)
        keys = ci * 1_000_003 + cj
        uniq, inv = np.unique(keys, return_inverse=True)
        if cell_m or len(uniq) <= target_n:
            break
        cell *= 1.5
    n = len(uniq)
    east = np.zeros(n)
    north = np.zeros(n)
    depth = np.zeros(n)
    count = np.zeros(n, dtype=np.int64)
    order = np.argsort(inv, kind="stable")
    bounds = np.flatnonzero(np.diff(inv[order])) + 1
    for k, grp in enumerate(np.split(order, bounds)):
        east[k] = float(s.east[grp].mean())
        north[k] = float(s.north[grp].mean())
        depth[k] = float(np.median(s.depth[grp]))
        count[k] = len(grp)
    return Cells(east, north, depth, count, cell)


def crossover_stats(s: Soundings, cell_m: float = 1.0,
                    min_gap_ms: float = 60_000.0) -> dict[str, float] | None:
    """Depth disagreement where the track revisits a cell after
    ``min_gap_ms`` (a tie line crossing a primary line)."""
    if len(s) == 0:
        return None
    ci = np.round(s.east / cell_m).astype(np.int64)
    cj = np.round(s.north / cell_m).astype(np.int64)
    keys = ci * 1_000_003 + cj
    uniq, inv = np.unique(keys, return_inverse=True)
    res = []
    for k in range(len(uniq)):
        m = inv == k
        t = s.t_ms[m]
        if t.max() - t.min() < min_gap_ms:
            continue
        early = s.depth[m][t < t.min() + 0.25 * min_gap_ms]
        late = s.depth[m][t > t.max() - 0.25 * min_gap_ms]
        if len(early) and len(late):
            res.append(float(np.median(early) - np.median(late)))
    if not res:
        return None
    a = np.abs(np.array(res))
    return {"n": int(len(a)), "median_abs_m": float(np.median(a)),
            "p90_abs_m": float(np.percentile(a, 90)),
            "rms_m": float(np.sqrt(np.mean(np.square(res))))}


# ------------------------------------------------------------------ cache
@functools.lru_cache(maxsize=4)
def _track_bundle(path: str, mtime_ns: int, size: int
                  ) -> tuple[AltitudeTrack, SvlogStreams]:
    """Parse + detect once per (path, mtime, size); the power arrays are
    dropped after detection so the cache stays small."""
    s = read_svlog(path, with_power=True)
    track = altitude_track(s.profiles)
    light = SvlogStreams([], s.positions, s.gps, s.attitude, s.skew_ms,
                         s.clock_frozen, s.n_packets)
    return track, light


def load_track(path: str | Path) -> tuple[AltitudeTrack, SvlogStreams]:
    """Cached :func:`altitude_track` + the recording's nav streams (the
    profiles themselves are not kept)."""
    p = Path(path)
    st = p.stat()
    return _track_bundle(str(p), st.st_mtime_ns, st.st_size)


# ------------------------------------------------------- recording extent
@functools.lru_cache(maxsize=8)
def _extent_bundle(path: str, mtime_ns: int, size: int) -> dict[str, Any]:
    """Header-only pass: where a recording was made, without decoding a
    single power array (0.3 s on an 18 MB survey against ~4 s with the
    bins, so the GUI can ask this on every file choice)."""
    s = read_svlog(path)                     # with_power=False
    lat = np.array([g.lat for g in s.gps], dtype=float)
    lon = np.array([g.lon for g in s.gps], dtype=float)
    ok = np.isfinite(lat) & np.isfinite(lon) & ~((lat == 0.0) & (lon == 0.0))
    lat, lon = lat[ok], lon[ok]
    out: dict[str, Any] = {
        "name": Path(path).name,
        "n_fixes": int(len(lat)),
        "profiles": int(len(s.profiles)),
        "clock_mode": "file_order" if s.clock_frozen else "live",
    }
    if not len(lat):
        return out
    lat0, lon0 = float(np.median(lat)), float(np.median(lon))
    e, n = latlon_to_local_en(lat, lon, lat0, lon0)
    out.update({
        "lat_min": float(lat.min()), "lat_max": float(lat.max()),
        "lon_min": float(lon.min()), "lon_max": float(lon.max()),
        "lat_c": 0.5 * (float(lat.min()) + float(lat.max())),
        "lon_c": 0.5 * (float(lon.min()) + float(lon.max())),
        "span_e_m": float(e.max() - e.min()),
        "span_n_m": float(n.max() - n.min()),
    })
    return out


def recording_extent(path: str | Path) -> dict[str, Any]:
    """Where a ``.svlog`` was recorded: fix count, lat/lon box and centre,
    span in metres, clock regime. Cached per (path, mtime, size).

    ``n_fixes == 0`` means the file carries no usable GPS -- the depth
    source cannot place its soundings and says so.
    """
    p = Path(path)
    st = p.stat()
    return dict(_extent_bundle(str(p), st.st_mtime_ns, st.st_size))


# ---------------------------------------------------------------- survey
@dataclass
class SurveyResult:
    soundings: Soundings
    cells: Cells
    summary: dict[str, Any] = field(default_factory=dict)


def survey_summary(track: AltitudeTrack, s: Soundings, cells: Cells,
                   streams: SvlogStreams, *, sensor_depth_m: float,
                   datum_offset_m: float) -> dict[str, Any]:
    n_ping = int(len(track))
    n_alt = int(np.isfinite(track.altitude_m).sum())
    alt = track.altitude_m[np.isfinite(track.altitude_m)]
    xo = crossover_stats(s, cells.cell_m) if len(s) else None
    return {
        "pings": n_ping,
        "profiles": int(track.n_profiles),
        "detected_fraction": (float(track.n_detected / max(track.n_profiles, 1))),
        "altitude_fraction": float(n_alt / max(n_ping, 1)),
        "altitude_p5_m": float(np.percentile(alt, 5)) if len(alt) else None,
        "altitude_p50_m": float(np.median(alt)) if len(alt) else None,
        "altitude_p95_m": float(np.percentile(alt, 95)) if len(alt) else None,
        "soundings": int(len(s)),
        "cells": int(len(cells)),
        "cell_m": float(cells.cell_m),
        "extent_m": ([float(s.east.min()), float(s.north.min()),
                      float(s.east.max()), float(s.north.max())]
                     if len(s) else None),
        "depth_min_m": float(s.depth.min()) if len(s) else None,
        "depth_max_m": float(s.depth.max()) if len(s) else None,
        "crossover": xo,
        "clock_mode": s.clock_mode if len(s) else (
            "file_order" if streams.clock_frozen else "live"),
        "gps_fixes": int(len(streams.gps)),
        "counter_offset": int(track.counter_offset),
        "sensor_depth_m": float(sensor_depth_m),
        "datum_offset_m": float(datum_offset_m),
    }


def survey_track(path: str | Path, anchor: GeoAnchor, *,
                 sensor_depth_m: float | None = None,
                 datum_offset_m: float = 0.0,
                 cell_m: float | None = None,
                 target_cells: int = 2500) -> SurveyResult:
    """The whole chain for one recording about one world anchor."""
    sd = sensor_depth_default() if sensor_depth_m is None else float(sensor_depth_m)
    track, streams = load_track(path)
    if len(track) < MIN_PINGS:
        raise ValueError(f"{Path(path).name}: only {len(track)} pings -- not "
                         "a usable survey")
    s = place_soundings(track, streams, anchor, sensor_depth_m=sd,
                        datum_offset_m=datum_offset_m)
    if len(s) == 0:
        raise ValueError(f"{Path(path).name}: no ping could be placed (no "
                         "bottom return coincides with a GPS fix)")
    cells = decimate(s, cell_m=cell_m, target_n=target_cells)
    summary = survey_summary(track, s, cells, streams, sensor_depth_m=sd,
                             datum_offset_m=datum_offset_m)
    return SurveyResult(s, cells, summary)


# ------------------------------------------------------------ backscatter
def _place_profiles(s: SvlogStreams):
    from .svlog_reader import gps_track
    return gps_track("", streams=s)


# -------------------------------------------------------------------- CLI
def main(argv: list[str] | None = None) -> int:
    import argparse
    import json

    ap = argparse.ArgumentParser(
        description="FBR depth soundings from a .svlog about a GPS anchor")
    ap.add_argument("svlog")
    ap.add_argument("--lat0", type=float, required=True)
    ap.add_argument("--lon0", type=float, required=True)
    ap.add_argument("--datum-offset", type=float, default=0.0)
    ap.add_argument("--cell", type=float, default=None)
    ap.add_argument("--out", default=None, help="CSV of the soundings")
    args = ap.parse_args(argv)
    res = survey_track(args.svlog, GeoAnchor(args.lat0, args.lon0),
                       datum_offset_m=args.datum_offset, cell_m=args.cell)
    print(json.dumps(res.summary, indent=2))
    if args.out:
        s = res.soundings
        with open(args.out, "w", encoding="utf-8") as f:
            f.write("east,north,depth,lat,lon,t_ms,quality\n")
            for k in range(len(s)):
                f.write(f"{s.east[k]:.3f},{s.north[k]:.3f},{s.depth[k]:.3f},"
                        f"{s.lat[k]:.7f},{s.lon[k]:.7f},{s.t_ms[k]:.0f},"
                        f"{int(s.quality[k])}\n")
        print(f"wrote {len(s)} soundings to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
