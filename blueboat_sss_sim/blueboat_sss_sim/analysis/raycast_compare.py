"""Statistics that compare a simulated sensor with real recordings, for the
ray-cast go/no-go and the Phase-8 comparison (``docs/engineering/
raycast_plan.md``; gates pre-registered in ``~/calib/raycast/gates.yaml``).

One set of reducers for real rows and for both renderers' rows (reported
dB, pings x bins, one acquisition, one altitude band -- a
``calibration.seabed.Stratum``):

* ``level_tail``: power-average level over the seabed tail (pass-2 window
  ``[max(1.4 alt, 1), min(0.725 R, R - 1)]``) -> the sim - real bias [dB];
* ``profile_rms``: the full-range mean intensity profile (power average per
  bin, from the first return to the end of range, bins with signal) -> rms of
  sim - real [dB];
* ``crest``: the median first-return crest (``seabed._crest``) [dB];
* ``contrast``: speckle contrast, std of the intensity normalised by its per
  bin mean over the tail (1 for fully developed speckle);
* ``k_log10``: log10 of the K-distribution shape from the second moment of the
  normalised intensity (``E I^2 = 2 (1 + 1/nu)``; capped at 3 = Rayleigh);
* ``ks``: two-sample Kolmogorov-Smirnov distance between the real and simulated
  normalised intensities (a distribution fit; real-vs-real split halves give its
  floor);
* ``range_corr_1..3``: correlation of the dB residual about the per-bin mean
  between bins ``lag`` apart (tail);
* ``track_corr_1``: the same between consecutive pings (only when the
  simulated ping spacing is the real one).

``compare`` returns, per statistic, the error of each renderer against the
real rows with a block bootstrap over pings (the replicates kept, so
``pooled`` can combine strata into a session or the whole validation set),
and the improvement
``|e_geometric| - |e_raycast|`` with its interval: **improves** when the
interval's lower end is above 0, **no worse** when its upper end is not below
0 (the owner's definition, 2026-11-03).

ROS-free, Qt-free, numpy only (scipy for the KS distance).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Mapping, Sequence

import numpy as np

SIGNAL_DB = 0.0
LAGS = (1, 2, 3)
STATS = ("level_tail", "profile_rms", "crest", "contrast", "k_log10", "ks",
         "range_corr_1", "range_corr_2", "range_corr_3", "track_corr_1")
#: statistics whose error is the value itself (already a sim-vs-real
#: distance), not a difference of two values
DISTANCES = ("profile_rms", "ks")


def tail_mask(r: np.ndarray, alt: float, L_m: float, real_mean_db: np.ndarray):
    lo, hi = max(1.4 * alt, 1.0), min(0.725 * L_m, L_m - 1.0)
    return (r >= lo) & (r <= hi) & (real_mean_db >= SIGNAL_DB)


def _pavg_db(rows: np.ndarray) -> np.ndarray:
    return 10.0 * np.log10(np.mean(10.0 ** (rows.astype(np.float64) / 10.0),
                                   axis=0))


def _corr(a: np.ndarray, b: np.ndarray) -> float:
    a = a - a.mean()
    b = b - b.mean()
    den = math.sqrt(float((a * a).sum() * (b * b).sum()))
    return float((a * b).sum() / den) if den > 0 else float("nan")


def _norm_intensity(rows: np.ndarray, mask: np.ndarray) -> np.ndarray:
    I = 10.0 ** (rows[:, mask].astype(np.float64) / 10.0)
    return I / np.maximum(I.mean(axis=0, keepdims=True), 1e-300)


def values(rows: np.ndarray, *, mask: np.ndarray, crest: Callable | None = None,
           fbr: np.ndarray | None = None) -> dict:
    """The single-set statistics of ``rows`` (the paired ones -- level,
    profile, KS -- are formed in :func:`errors`)."""
    out = {}
    In = _norm_intensity(rows, mask)
    out["contrast"] = float(In.std())
    m2 = float(np.mean(In * In))
    nu = 1.0 / (m2 / 2.0 - 1.0) if m2 > 2.0 + 1e-9 else np.inf
    out["k_log10"] = float(min(math.log10(nu), 3.0)) if np.isfinite(nu) and nu > 0 else 3.0
    res = rows[:, mask].astype(np.float64)
    res = res - res.mean(axis=0, keepdims=True)
    for lag in LAGS:
        out[f"range_corr_{lag}"] = (_corr(res[:, :-lag], res[:, lag:])
                                    if res.shape[1] > lag + 4 else float("nan"))
    out["track_corr_1"] = (_corr(res[:-1], res[1:]) if res.shape[0] > 3
                           else float("nan"))
    if crest is not None and fbr is not None:
        c = [crest(db, int(i)) for db, i in zip(rows, fbr) if i >= 0]
        out["crest"] = float(np.median(c)) if c else float("nan")
    return out


def errors(real: np.ndarray, sim: np.ndarray, *, r: np.ndarray, alt: float,
           L_m: float, fbr_real=None, fbr_sim=None, crest: Callable | None = None,
           alt_bin: float | None = None, track_comparable: bool = True,
           ks_samples: int = 20000, rng: np.random.Generator | None = None
           ) -> dict:
    """sim - real for every statistic (distances as they are)."""
    from scipy.stats import ks_2samp
    rng = rng or np.random.default_rng(0)
    real_mean = real.mean(axis=0)
    mask = tail_mask(r, alt, L_m, real_mean)
    if mask.sum() < 10:
        return {}
    e = {}
    pr, ps = _pavg_db(real), _pavg_db(sim)
    e["level_tail"] = float(10.0 * np.log10(
        np.mean(10.0 ** (ps[mask] / 10.0)) / np.mean(10.0 ** (pr[mask] / 10.0))))
    full = (r >= (alt_bin if alt_bin is not None else alt)) & (real_mean >= SIGNAL_DB)
    e["profile_rms"] = float(np.sqrt(np.mean((ps[full] - pr[full]) ** 2)))
    vr = values(real, mask=mask, crest=crest, fbr=fbr_real)
    vs = values(sim, mask=mask, crest=crest, fbr=fbr_sim)
    for k in ("contrast", "k_log10", "range_corr_1", "range_corr_2",
              "range_corr_3", "track_corr_1", "crest"):
        if k in vr and k in vs:
            e[k] = vs[k] - vr[k]
    if not track_comparable:
        e.pop("track_corr_1", None)
    a = _norm_intensity(real, mask).ravel()
    b = _norm_intensity(sim, mask).ravel()
    a = rng.choice(a, min(ks_samples, a.size), replace=False)
    b = rng.choice(b, min(ks_samples, b.size), replace=False)
    e["ks"] = float(ks_2samp(a, b).statistic)
    return e


def _blocks(n: int, block: int, rng) -> np.ndarray:
    nb = max(n // block, 1)
    starts = rng.integers(0, max(n - block + 1, 1), size=nb)
    return (starts[:, None] + np.arange(min(block, n))[None, :]).ravel()


@dataclass
class Compared:
    point: dict            # renderer -> stat -> error
    ci: dict               # renderer -> stat -> (lo, hi)
    improvement: dict      # stat -> (point, lo, hi) of |e_geo| - |e_rc|
    boots: dict = field(default_factory=dict)       # renderer -> stat -> (B,) replicate errors
    imp_boots: dict = field(default_factory=dict)   # stat -> (B,) replicate improvements


def pooled(results: Sequence["Compared"], weights: Sequence[float]) -> "Compared":
    """Several strata as one set: each point and each bootstrap replicate the
    weighted mean (ping-weighted, gates.yaml) of the strata's, replicate b of
    every stratum combined (independent resamples per stratum), so the pooled
    interval carries every stratum's sampling error. Statistics a stratum
    lacks are left out of its weight."""
    w = np.asarray(weights, dtype=float)
    out = Compared(point={}, ci={}, improvement={}, boots={}, imp_boots={})
    rends = sorted({k for c in results for k in c.point})

    def wmean(vals):
        v = np.array(vals, dtype=float)
        ok = np.isfinite(v) if v.ndim == 1 else np.all(np.isfinite(v), axis=1)
        if not ok.any():
            return None
        ww = w[ok] / w[ok].sum()
        return np.tensordot(ww, v[ok], axes=1)

    for k in rends:
        out.point[k], out.ci[k], out.boots[k] = {}, {}, {}
        for st in STATS:
            p = wmean([c.point.get(k, {}).get(st, np.nan) for c in results])
            if p is None:
                continue
            B = max(len(c.boots.get(k, {}).get(st, [])) for c in results)
            reps = wmean([c.boots.get(k, {}).get(st, np.full(B, np.nan)) if
                          len(c.boots.get(k, {}).get(st, [])) == B else np.full(B, np.nan)
                          for c in results])
            out.point[k][st] = float(p)
            if reps is not None:
                out.boots[k][st] = reps
                out.ci[k][st] = (float(np.percentile(reps, 2.5)), float(np.percentile(reps, 97.5)))
    if "raycast" in out.point and "geometric" in out.point:
        for st in STATS:
            pr, pg = out.point["raycast"].get(st), out.point["geometric"].get(st)
            br, bg = out.boots["raycast"].get(st), out.boots["geometric"].get(st)
            if pr is None or pg is None or br is None or bg is None:
                continue
            v = np.abs(bg) - np.abs(br)
            out.imp_boots[st] = v
            out.improvement[st] = (abs(pg) - abs(pr), float(np.percentile(v, 2.5)),
                                   float(np.percentile(v, 97.5)))
    return out


def compare(real: np.ndarray, sims: Mapping[str, np.ndarray], *, B: int = 300,
            block: int = 8, seed: int = 20261104, **kw) -> Compared:
    """Errors of each simulated set against ``real`` with a block bootstrap
    over pings (real and simulated rows resampled independently, the same
    resample of the real rows for both renderers), and the improvement of
    ``raycast`` over ``geometric`` when both are given."""
    rng = np.random.default_rng(seed)

    def _for(k):
        """kw with ``fbr_sim`` resolved to renderer ``k``'s detections (a
        mapping per renderer, as the bootstrap below reads it)."""
        kk = dict(kw)
        if isinstance(kk.get("fbr_sim"), Mapping):
            kk["fbr_sim"] = kk["fbr_sim"].get(k)
        return kk
    point = {k: errors(real, s, rng=np.random.default_rng(1), **_for(k))
             for k, s in sims.items()}
    boots = {k: [] for k in sims}
    imp = []
    for _ in range(B):
        ir = _blocks(real.shape[0], block, rng)
        one = {}
        for k, s in sims.items():
            isb = _blocks(s.shape[0], block, rng)
            kw2 = dict(kw)
            for f in ("fbr_real",):
                if kw.get(f) is not None:
                    kw2[f] = np.asarray(kw[f])[ir]
            if kw.get("fbr_sim") is not None and isinstance(kw["fbr_sim"], Mapping):
                kw2["fbr_sim"] = np.asarray(kw["fbr_sim"][k])[isb]
            one[k] = errors(real[ir], s[isb], rng=rng, **kw2)
            boots[k].append(one[k])
        if "raycast" in one and "geometric" in one:
            imp.append({st: abs(one["geometric"].get(st, np.nan))
                        - abs(one["raycast"].get(st, np.nan)) for st in STATS})
    ci, reps = {}, {}
    for k, bl in boots.items():
        ci[k], reps[k] = {}, {}
        for st in STATS:
            v = np.array([b.get(st, np.nan) for b in bl], dtype=float)
            if np.isfinite(v).any():
                reps[k][st] = v
            v = v[np.isfinite(v)]
            ci[k][st] = ((float(np.percentile(v, 2.5)), float(np.percentile(v, 97.5)))
                         if v.size else (float("nan"), float("nan")))
    improvement, imp_reps = {}, {}
    if imp:
        for st in STATS:
            v = np.array([d[st] for d in imp], dtype=float)
            if np.isfinite(v).any():
                imp_reps[st] = v
            v = v[np.isfinite(v)]
            pr = point.get("raycast", {}).get(st)
            pg = point.get("geometric", {}).get(st)
            p = (abs(pg) - abs(pr)) if pr is not None and pg is not None else float("nan")
            improvement[st] = ((p, float(np.percentile(v, 2.5)),
                                float(np.percentile(v, 97.5))) if v.size
                               else (p, float("nan"), float("nan")))
    return Compared(point=point, ci=ci, improvement=improvement, boots=reps,
                    imp_boots=imp_reps)


def verdict(imp: tuple) -> str:
    """'improves' | 'no worse' | 'worse' from (point, lo, hi) of
    ``|e_geo| - |e_rc|``."""
    p, lo, hi = imp
    if not np.isfinite(lo):
        return "n/a"
    if lo > 0.0:
        return "improves"
    if hi >= 0.0:
        return "no worse"
    return "worse"


def split_half_ks(real: np.ndarray, *, r, alt, L_m, rng=None) -> float:
    """The KS distance between the two halves of the real rows (the floor of
    the distribution-fit statistic)."""
    from scipy.stats import ks_2samp
    rng = rng or np.random.default_rng(0)
    mask = tail_mask(r, alt, L_m, real.mean(axis=0))
    if mask.sum() < 10 or real.shape[0] < 20:
        return float("nan")
    h = real.shape[0] // 2
    a = _norm_intensity(real[:h], mask).ravel()
    b = _norm_intensity(real[h:], mask).ravel()
    a = rng.choice(a, min(20000, a.size), replace=False)
    b = rng.choice(b, min(20000, b.size), replace=False)
    return float(ks_2samp(a, b).statistic)
