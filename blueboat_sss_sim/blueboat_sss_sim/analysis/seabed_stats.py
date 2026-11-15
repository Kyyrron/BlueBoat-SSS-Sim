"""Seabed heterogeneity statistics of a side-scan waterfall (acceptance criterion B2).

What a seabed that is not one uniform sediment does to the picture, as four numbers per *stretch* (an
object-free block of consecutive pings of one side on a straight leg, cut to a slant-range window):

    lag_correlation      correlation of the dB values at the same bin, ``lag`` pings apart (bin means removed):
                         ``analysis.svlog_compare.texture_stats``'s ``track_corr_lag3`` on the stretch's window
    patch_spread_db      std of 10 log10 of the means of L x L cells (along track x slant range) of the linear
                         power relative to its per-bin mean (L = 0.5 m and 2 m)
    regions              connected sets of the contrast map under -4 dB (dark) or over +4 dB (bright), of at least
                         0.05 m2; the contrast is the 2026-11-12 review's: linear power smoothed over
                         0.15 m x 0.15 m over the per-bin reference (median / ln 2, smoothed 0.5 m in range)
    stretch_stats        all of them, as a dict; ``summary`` = minimum, median, maximum over stretches

and how a recording becomes stretches (``load_sides``, ``find_stretches``; rule in :func:`find_stretches`).
Definitions frozen on 2026-11-15, before any field number was computed (completion round, session 02: design
note section 2.4 under ``~/ros2_ws/sim_runs/sim_acceptance/02_world_builder/``). A uniform simulated seabed
gives, with these definitions: lag-3 correlation 0.03-0.06, patch spreads 0.6-0.8 / 0.15-0.36 dB, 0-2.5 dark and
no bright region per 100 m2.

    python3 -m blueboat_sss_sim.analysis.seabed_stats FILE.svlog [--out DIR] [--stretches FILE.yaml]
        --out        quick-look pictures of each side with the stretches boxed, and stats.json
        --stretches  use the annotated stretches of that file instead of finding them (written by --write)
        --write      write the found stretches to FILE.yaml and stop (no statistic is computed)

ROS-free; numpy and scipy.ndimage (matplotlib only for the quick-look).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy import ndimage

LAG_PINGS = 3
PATCH_CELLS_M = (0.5, 2.0)
SMOOTH_M = 0.15
REF_SMOOTH_M = 0.5
REGION_DB = 4.0
MIN_AREA_M2 = 0.05
BAND_FRACTION = 0.9
#: a stretch under this area gives no region rate
MIN_STRETCH_M2 = 25.0

# ---- the stretch rule (find_stretches)
MAX_YAW_RATE_DPS = 3.0
MIN_ADVANCE_M = 0.01
U_MIN = 1.4
WALL_DB = 6.0
WALL_TREND_M = 4.0
REFLECTOR_DB = 12.0
#: a spot this far UNDER the local seabed at the 0.5 m scale is an object's shadow, not sediment (a real shadow
#: sits 17-25 dB under the seabed; the darkest sediment class of the generator is 5-10 dB under the sand)
SHADOW_DB = 12.0
REFLECTOR_SMOOTH_M = 0.5
REFLECTOR_GUARD_M = 1.5
MIN_PINGS = 150
BLOCK_PINGS = 400
SIDES = ("port", "starboard")
STRETCHES_FORMAT = "blueboat_seabed_stretches/1"


@dataclass
class Stretch:
    """An object-free block of one side."""

    db: np.ndarray            # (pings, bins) dB, analog gain removed
    adv_m: np.ndarray         # (pings,) along-track advance per ping [m], > 0
    bin_m: float              # slant-range bin [m]
    slant0_m: float = 0.0     # slant range of the first column [m]
    name: str = ""
    meta: dict = field(default_factory=dict)

    @property
    def area_m2(self) -> float:
        return float(self.adv_m.sum() * self.db.shape[1] * self.bin_m)


# ------------------------------------------------------------------------------------------- statistics
def _corr(a: np.ndarray, b: np.ndarray) -> float:
    a = a.ravel() - a.mean()
    b = b.ravel() - b.mean()
    d = np.sqrt((a * a).mean() * (b * b).mean())
    return float((a * b).mean() / d) if d > 0 else float("nan")


def lag_correlation(db: np.ndarray, lag: int = LAG_PINGS, remove_ping_level: bool = False) -> float:
    """Correlation of the dB values at the same bin ``lag`` pings apart, after removing each bin's mean over
    the stretch. ``remove_ping_level`` also removes each ping's median over the window: what is left is texture
    inside the swath, without ping-level effects (gain steps, roll)."""
    x = np.asarray(db, dtype=np.float64)
    if x.shape[0] <= lag + 8:
        return float("nan")
    x = x - x.mean(axis=0, keepdims=True)
    if remove_ping_level:
        x = x - np.median(x, axis=1, keepdims=True)
        x = x - x.mean(axis=0, keepdims=True)
    return _corr(x[:-lag], x[lag:])


def _relative_power(db: np.ndarray) -> np.ndarray:
    lin = 10.0 ** (np.asarray(db, dtype=np.float64) / 10.0)
    return lin / np.maximum(lin.mean(axis=0, keepdims=True), 1e-300)


def patch_spread_db(db: np.ndarray, adv_m: np.ndarray, bin_m: float, cell_m: float) -> float:
    """Standard deviation [dB] of the means of ``cell_m`` x ``cell_m`` cells (along track x slant range) of the
    linear power relative to its per-bin mean. NaN with fewer than 9 whole cells."""
    rel = _relative_power(db)
    adv_m = np.asarray(adv_m, dtype=np.float64)
    pos = np.concatenate([[0.0], np.cumsum(adv_m)[:-1]])
    row_cell = np.floor(pos / float(cell_m)).astype(np.int64)
    nc = max(int(round(float(cell_m) / float(bin_m))), 1)
    n_col = rel.shape[1] // nc
    if n_col < 1:
        return float("nan")
    means: list[float] = []
    for r in np.unique(row_cell):
        rows = np.flatnonzero(row_cell == r)
        if rows.size < 2 or adv_m[rows].sum() < 0.8 * cell_m:       # a whole cell only
            continue
        blk = rel[rows][:, :n_col * nc].reshape(rows.size, n_col, nc)
        means.extend(blk.mean(axis=(0, 2)).tolist())
    if len(means) < 9:
        return float("nan")
    return float(np.std(10.0 * np.log10(np.maximum(means, 1e-9))))


def contrast_db(db: np.ndarray, adv_m: np.ndarray, bin_m: float, smooth_m: float = SMOOTH_M,
                ref_smooth_m: float = REF_SMOOTH_M) -> np.ndarray:
    """The review's contrast map: smoothed linear power over the robust per-bin reference, in dB."""
    lin = 10.0 ** (np.asarray(db, dtype=np.float64) / 10.0)
    ref = np.median(lin, axis=0) / np.log(2.0)
    ref = ndimage.uniform_filter(ref, max(int(ref_smooth_m / bin_m), 1), mode="nearest")
    kr = max(int(round(smooth_m / float(np.median(adv_m)))), 1)
    kc = max(int(round(smooth_m / bin_m)), 1)
    sm = ndimage.uniform_filter(lin, (kr, kc), mode="nearest")
    return 10.0 * np.log10(np.maximum(sm / np.maximum(ref[None, :], 1e-300), 1e-6))


def regions(con: np.ndarray, adv_m: np.ndarray, bin_m: float, dark: bool = True, thr_db: float = REGION_DB,
            min_area_m2: float = MIN_AREA_M2, band_fraction: float = BAND_FRACTION) -> list[dict]:
    """Connected sets of the contrast map under ``-thr_db`` (``dark``) or over ``+thr_db``, of at least
    ``min_area_m2`` (along track x slant range). A set spanning ``band_fraction`` of the window in range or
    more is a dropped or saturated ping, not a region, and is left out."""
    mask = (con < -abs(thr_db)) if dark else (con > abs(thr_db))
    lab, _ = ndimage.label(mask)
    adv_m = np.asarray(adv_m, dtype=np.float64)
    out = []
    for i, sl in enumerate(ndimage.find_objects(lab), start=1):
        m = lab[sl] == i
        rows = np.arange(sl[0].start, sl[0].stop)
        if (sl[1].stop - sl[1].start) >= band_fraction * con.shape[1]:
            continue
        area = float((m * adv_m[rows][:, None]).sum() * bin_m)
        if area < min_area_m2:
            continue
        v = con[sl][m]
        out.append({"area_m2": area, "row0": int(sl[0].start), "row1": int(sl[0].stop),
                    "col0": int(sl[1].start), "col1": int(sl[1].stop),
                    "extreme_db": float(v.min() if dark else v.max()), "mean_db": float(v.mean())})
    return out


def stretch_stats(s: Stretch) -> dict:
    """The B2 quantities of one stretch."""
    con = contrast_db(s.db, s.adv_m, s.bin_m)
    area = s.area_m2
    dk = regions(con, s.adv_m, s.bin_m, dark=True)
    br = regions(con, s.adv_m, s.bin_m, dark=False)
    per100 = (100.0 / area) if area >= MIN_STRETCH_M2 else float("nan")
    out = {
        "name": s.name, "pings": int(s.db.shape[0]), "bins": int(s.db.shape[1]), "area_m2": round(area, 1),
        "length_m": round(float(np.sum(s.adv_m)), 2), "advance_m": round(float(np.median(s.adv_m)), 4),
        "corr_lag3": lag_correlation(s.db, LAG_PINGS),
        "corr_lag3_ping_level_removed": lag_correlation(s.db, LAG_PINGS, remove_ping_level=True),
        "corr_lag1": lag_correlation(s.db, 1),
        "std_db": float((s.db - s.db.mean(axis=0, keepdims=True)).std()),
        "dark_regions": len(dk), "bright_regions": len(br),
        "dark_per_100m2": len(dk) * per100, "bright_per_100m2": len(br) * per100,
        "dark_area_share": float(sum(r["area_m2"] for r in dk) / area) if area > 0 else float("nan"),
        "bright_area_share": float(sum(r["area_m2"] for r in br) / area) if area > 0 else float("nan"),
    }
    for cell in PATCH_CELLS_M:
        out[f"patch_spread_{cell:g}m_db"] = patch_spread_db(s.db, s.adv_m, s.bin_m, cell)
    return out


B2_KEYS = ("corr_lag3", "patch_spread_0.5m_db", "patch_spread_2m_db", "dark_per_100m2", "bright_per_100m2")


def summary(stats: list[dict], keys: tuple[str, ...] = B2_KEYS) -> dict:
    """Minimum, median and maximum of each statistic over stretches (the R2 presets' targets)."""
    out = {}
    for k in keys:
        v = np.array([s[k] for s in stats if k in s and s[k] is not None and np.isfinite(s[k])],
                     dtype=np.float64)
        out[k] = ({"min": float(v.min()), "median": float(np.median(v)), "max": float(v.max()), "n": int(v.size)}
                  if v.size else {"min": None, "median": None, "max": None, "n": 0})
    return out


# ------------------------------------------------------------------------------------------- recordings
@dataclass
class SideData:
    """One side of a recording at one acquisition setting."""

    t: np.ndarray            # s since the first ping
    ping: np.ndarray         # ping_number
    x: np.ndarray            # east  [m] about the anchor
    y: np.ndarray            # north [m]
    hdg: np.ndarray          # compass heading [deg]
    roll: np.ndarray
    pitch: np.ndarray
    range_m: float
    bins: int
    pulse_us: np.ndarray
    again: np.ndarray        # analog gain as the device reports it
    alt: np.ndarray          # first-return slant range [m] (NaN = none)
    db: np.ndarray           # (n, bins) reported dB
    dropped_settings: dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return int(self.t.size)

    @property
    def bin_m(self) -> float:
        return self.range_m / self.bins


def load_sides(path: str | Path, anchor: tuple[float, float] | None = None,
               setting: tuple[int, int] | None = None) -> dict[str, SideData]:
    """Both sides of a recording at ONE acquisition setting (``setting`` = (range_mm, bins); default: the one
    with most pings). Positions in metres about ``anchor`` (lat, lon; default: the first fix)."""
    from ..calibration.seabed import _attitude_arrays
    from ..core.geo import latlon_to_local_en
    from .calibration import fbr_bin, scale_to_db
    from .svlog_reader import gps_track, profile_heading_deg, read_svlog
    st = read_svlog(path, with_power=True)
    st, idx, lat, lon = gps_track(path, st)
    prof = [st.profiles[i] for i in idx]
    hdg = np.asarray(profile_heading_deg(st, idx), dtype=np.float64)
    att = _attitude_arrays(st, prof)
    roll, pitch = ((np.asarray(att[0]), np.asarray(att[1])) if att is not None
                   else (np.zeros(len(prof)), np.zeros(len(prof))))
    lat0, lon0 = anchor if anchor is not None else (float(lat[0]), float(lon[0]))
    e, n = latlon_to_local_en(np.asarray(lat, float), np.asarray(lon, float), lat0, lon0)
    t0 = float(min(p.timestamp_ms for p in prof))
    out = {}
    for s in SIDES:
        k_all = [i for i, p in enumerate(prof) if p.side.value == s and p.pwr_results is not None]
        counts: dict = {}
        for i in k_all:
            key = (int(prof[i].length_mm), int(prof[i].num_results))
            counts[key] = counts.get(key, 0) + 1
        if not counts:
            continue
        use = tuple(setting) if setting else max(counts, key=counts.get)
        k = np.array([i for i in k_all if (int(prof[i].length_mm), int(prof[i].num_results)) == use])
        if k.size == 0:
            continue
        db = np.stack([scale_to_db(prof[i].pwr_results, prof[i].min_pwr_db, prof[i].max_pwr_db) for i in k]
                      ).astype(np.float32)
        rng = use[0] / 1000.0
        alt = np.full(k.size, np.nan)
        for j in range(k.size):
            b = fbr_bin(db[j].astype(np.float64))
            if b is not None:
                alt[j] = b * rng / use[1]
        ts = np.array([prof[i].timestamp_ms for i in k], float)
        o = np.argsort(ts, kind="stable")
        k, db, alt, ts = k[o], db[o], alt[o], ts[o]
        out[s] = SideData(t=(ts - t0) / 1000.0, ping=np.array([prof[i].ping_number for i in k]), x=e[k], y=n[k],
                          hdg=hdg[k], roll=roll[k], pitch=pitch[k], range_m=rng, bins=int(use[1]),
                          pulse_us=np.array([prof[i].pulse_duration_sec for i in k]) * 1e6,
                          again=np.array([prof[i].analog_gain for i in k], float), alt=alt, db=db,
                          dropped_settings={f"{a} mm / {b} bins": c for (a, b), c in counts.items()
                                            if (a, b) != use})
    return out


def _running_mean(x: np.ndarray, n: int) -> np.ndarray:
    return ndimage.uniform_filter1d(np.asarray(x, float), max(int(n), 1), mode="nearest")


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """[a, b) index runs of True."""
    d = np.diff(np.concatenate([[0], np.asarray(mask).astype(np.int8), [0]]))
    return [(int(a), int(b)) for a, b in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1))]


def _destair(t: np.ndarray, *series: np.ndarray) -> list[np.ndarray]:
    """Series that a slower stream holds between its samples (a recording whose autopilot clock is frozen pairs
    every ping with the latest fix in file order: the FIT harbour file has 3 fixes per second for 18 pings),
    made piecewise linear between their common change points. Returned unchanged when they are not stairs."""
    arr = [np.asarray(v, dtype=np.float64) for v in series]
    change = np.zeros(arr[0].size - 1, dtype=bool)
    for v in arr:
        change |= np.diff(v) != 0.0
    ch = np.flatnonzero(change) + 1
    if ch.size < 2 or ch.size > 0.7 * arr[0].size:
        return arr
    idx = np.concatenate([[0], ch])                   # the first ping of each step carries the new sample
    return [np.interp(t, t[idx], v[idx]) for v in arr]


def track_xy(sd: SideData) -> tuple[np.ndarray, np.ndarray]:
    """The boat's position at every ping, stairs removed."""
    x, y = _destair(sd.t, sd.x, sd.y)
    return x, y


def advance(sd: SideData) -> np.ndarray:
    """Along-track advance per ping [m] (5-ping running mean, as the review)."""
    x, y = track_xy(sd)
    d = np.hypot(np.gradient(x), np.gradient(y))
    return np.clip(_running_mean(d, 5), 0.005, None)


def gain_removed_db(sd: SideData) -> np.ndarray:
    """Reported dB minus the ping's analog gain (20 log10; the reference cancels in every statistic here)."""
    return sd.db.astype(np.float64) - 20.0 * np.log10(np.maximum(sd.again, 1e-6))[:, None]


def straight_mask(sd: SideData) -> np.ndarray:
    dt = np.clip(np.gradient(sd.t), 1e-3, None)
    (hd,) = _destair(sd.t, np.unwrap(np.radians(sd.hdg)))
    rate = np.degrees(np.gradient(hd)) / dt
    rate = _running_mean(rate, max(int(round(1.0 / float(np.median(dt)))), 1))
    gap = np.concatenate([[False], np.diff(sd.t) > 5.0 * np.median(dt)])
    return (np.abs(rate) < MAX_YAW_RATE_DPS) & (advance(sd) > MIN_ADVANCE_M) & ~gap


def cut_stretch(sd: SideData, ping0: int, ping1: int, slant0_m: float, slant1_m: float, name: str = "",
                altitude_m: float | None = None) -> Stretch:
    """The stretch of pings [ping0, ping1) and slant window [slant0, slant1) of ``sd``."""
    binm = sd.bin_m
    c0, c1 = int(round(slant0_m / binm)), int(round(slant1_m / binm))
    adv = advance(sd)[ping0:ping1]
    z = gain_removed_db(sd)[ping0:ping1, c0:c1]
    alt = altitude_m
    if alt is None and np.isfinite(sd.alt[ping0:ping1]).any():
        alt = float(np.nanmedian(sd.alt[ping0:ping1]))
    dur = max(float(sd.t[ping1 - 1] - sd.t[ping0]), 1e-6)
    return Stretch(db=z, adv_m=adv, bin_m=binm, slant0_m=c0 * binm, name=name or f"{ping0}_{ping1}", meta={
        "ping0": int(ping0), "ping1": int(ping1), "t0_s": round(float(sd.t[ping0]), 1),
        "t1_s": round(float(sd.t[ping1 - 1]), 1),
        "altitude_m": None if alt is None else round(float(alt), 2),
        "slant_window_m": [round(c0 * binm, 3), round(c1 * binm, 3)], "range_m": sd.range_m, "bins": sd.bins,
        "roll_rms_deg": round(float(np.degrees(np.std(sd.roll[ping0:ping1]))), 2),
        "speed_mps": round(float(adv.sum() / dur), 2)})


def find_stretches(sd: SideData, name: str = "", max_slant_m: float | None = None) -> list[Stretch]:
    """The object-free blocks of one side.

    Rule: one acquisition setting, consecutive pings; straight (|yaw rate| < MAX_YAW_RATE_DPS over a 1 s running
    mean, the boat advancing); slant-range window from U_MIN x altitude to the end of the range (last 2 %
    dropped), cut before any band that stands WALL_DB over the range trend of the block (a quay or wall face);
    pings within REFLECTOR_GUARD_M along track of a spot more than REFLECTOR_DB over the local seabed (a
    reflector) or more than SHADOW_DB under it (a shadow), both at REFLECTOR_SMOOTH_M, left out, which splits
    the block; at least MIN_PINGS pings; longer runs cut into blocks of at most BLOCK_PINGS. ``max_slant_m``
    caps the window (to compare a recording with a render made at a shorter range)."""
    adv_all = advance(sd)
    z_all = gain_removed_db(sd)
    binm = sd.bin_m
    out = []
    for a, b in _runs(straight_mask(sd)):
        if b - a < MIN_PINGS or not np.isfinite(sd.alt[a:b]).any():
            continue
        alt = float(np.nanmedian(sd.alt[a:b]))
        c0 = int(np.ceil(U_MIN * alt / binm))
        c1 = int(0.98 * sd.bins)
        if max_slant_m is not None:
            c1 = min(c1, int(float(max_slant_m) / binm))
        if c1 - c0 < int(2.0 / binm):
            continue
        prof = np.median(z_all[a:b, c0:c1], axis=0)
        trend = ndimage.median_filter(prof, size=max(int(WALL_TREND_M / binm) | 1, 3), mode="nearest")
        over = ndimage.binary_opening((prof - trend) > WALL_DB, structure=np.ones(max(int(0.3 / binm), 1)))
        if over.any():
            c1 = c0 + int(np.argmax(over)) - int(0.5 / binm)
            if c1 - c0 < int(2.0 / binm):
                continue
        adv = adv_all[a:b]
        con = contrast_db(z_all[a:b, c0:c1], adv, binm, smooth_m=REFLECTOR_SMOOTH_M)
        hot = ((con > REFLECTOR_DB) | (con < -SHADOW_DB)).any(axis=1)
        guard = max(int(round(REFLECTOR_GUARD_M / float(np.median(adv)))), 1)
        hot = ndimage.binary_dilation(hot, structure=np.ones(2 * guard + 1))
        for p, q in _runs(~hot):
            while q - p >= MIN_PINGS:
                e = min(p + BLOCK_PINGS, q)
                if q - e < MIN_PINGS:
                    e = q
                out.append(cut_stretch(sd, a + p, a + e, c0 * binm, c1 * binm,
                                       name=f"{name}_{a + p}_{a + e}", altitude_m=alt))
                p = e
    return out


def stretches_to_doc(recording: str, per_side: dict[str, list[Stretch]], dropped: list[dict] | None = None) -> dict:
    """The annotation of a recording's stretches (written before any statistic is computed)."""
    return {"format": STRETCHES_FORMAT, "recording": str(recording),
            "rule": {"max_yaw_rate_dps": MAX_YAW_RATE_DPS, "u_min": U_MIN, "wall_db": WALL_DB,
                     "reflector_db": REFLECTOR_DB, "shadow_db": SHADOW_DB, "guard_m": REFLECTOR_GUARD_M,
                     "min_pings": MIN_PINGS, "block_pings": BLOCK_PINGS},
            "stretches": [{"side": s, "name": st.name, **st.meta} for s, sts in per_side.items() for st in sts],
            "dropped_by_inspection": list(dropped or [])}


def stretches_from_doc(doc: dict, sides: dict[str, SideData]) -> dict[str, list[Stretch]]:
    """The stretches an annotation names, cut from the loaded sides."""
    if doc.get("format") != STRETCHES_FORMAT:
        raise ValueError(f"stretch file format {doc.get('format')!r} (expected {STRETCHES_FORMAT})")
    out: dict[str, list[Stretch]] = {s: [] for s in sides}
    for row in doc["stretches"]:
        sd = sides[row["side"]]
        out[row["side"]].append(cut_stretch(sd, int(row["ping0"]), int(row["ping1"]),
                                            float(row["slant_window_m"][0]), float(row["slant_window_m"][1]),
                                            name=str(row.get("name", "")), altitude_m=row.get("altitude_m")))
    return out


def quicklook(sd: SideData, stretches: list[Stretch], png: str | Path, title: str = "") -> None:
    """The side's waterfall (gain removed, per-bin median removed) with the stretches boxed and numbered."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    z = gain_removed_db(sd)
    z = z - np.median(z, axis=0, keepdims=True)
    fig, ax = plt.subplots(figsize=(13, 6.5))
    ax.imshow(z.T, aspect="auto", origin="lower", cmap="gray", vmin=-12, vmax=12,
              extent=(0, sd.n, 0, sd.range_m), interpolation="nearest")
    for i, st in enumerate(stretches):
        m = st.meta
        w = m["slant_window_m"]
        ax.add_patch(plt.Rectangle((m["ping0"], w[0]), m["ping1"] - m["ping0"], w[1] - w[0], fill=False,
                                   ec="tab:orange", lw=1.0))
        ax.text(m["ping0"] + 3, w[1] - 0.04 * sd.range_m, str(i), color="tab:orange", fontsize=8)
    ax.set_xlabel("ping index")
    ax.set_ylabel("slant range [m]")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    import argparse
    import json

    import yaml
    ap = argparse.ArgumentParser(description="Seabed heterogeneity statistics of a recording (criterion B2).")
    ap.add_argument("svlog")
    ap.add_argument("--out", default=None, help="folder for quick-looks and stats.json")
    ap.add_argument("--stretches", default=None, help="annotated stretches to use (YAML)")
    ap.add_argument("--write", default=None, help="write the found stretches to this YAML and stop")
    ap.add_argument("--setting", default=None, help="RANGE_MM,BINS (default: the setting with most pings)")
    ap.add_argument("--max-slant", type=float, default=None, help="cap of the slant-range window [m]")
    a = ap.parse_args(argv)
    path = Path(a.svlog)
    setting = tuple(int(v) for v in a.setting.split(",")) if a.setting else None
    sides = load_sides(path, setting=setting)
    if a.stretches:
        per_side = stretches_from_doc(yaml.safe_load(Path(a.stretches).read_text()), sides)
    else:
        per_side = {s: find_stretches(sd, name=f"{path.stem}_{s}", max_slant_m=a.max_slant)
                    for s, sd in sides.items()}
    outd = Path(a.out) if a.out else None
    if outd is not None:
        outd.mkdir(parents=True, exist_ok=True)
        for s, sd in sides.items():
            quicklook(sd, per_side[s], outd / f"{path.stem}_{s}.png", f"{path.name} {s}")
    for s, sd in sides.items():
        print(f"{s}: {sd.n} pings at {sd.range_m:g} m / {sd.bins} bins; other settings {sd.dropped_settings}; "
              f"{len(per_side[s])} stretches")
    if a.write:
        Path(a.write).write_text(yaml.safe_dump(stretches_to_doc(path.name, per_side), sort_keys=False,
                                                default_flow_style=None))
        print("stretches written to", a.write, "(no statistic computed)")
        return 0
    result = {}
    for s in sides:
        rows = [{**st.meta, **stretch_stats(st)} for st in per_side[s]]
        for r in rows:
            print(f"  {s} pings {r['ping0']:5d}-{r['ping1']:5d}  {r['area_m2']:6.1f} m2  lag3 {r['corr_lag3']:.3f} "
                  f"({r['corr_lag3_ping_level_removed']:.3f})  spread {r['patch_spread_0.5m_db']:.2f}/"
                  f"{r['patch_spread_2m_db']:.2f} dB  dark/bright per 100 m2 {r['dark_per_100m2']:.2f}/"
                  f"{r['bright_per_100m2']:.2f}")
        result[s] = {"stretches": rows, "summary": summary(rows)}
    allrows = [r for s in result for r in result[s]["stretches"]]
    result["both_sides"] = summary(allrows)
    print("summary, both sides:", json.dumps(result["both_sides"]))
    if outd is not None:
        (outd / f"{path.stem}_stats.json").write_text(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
