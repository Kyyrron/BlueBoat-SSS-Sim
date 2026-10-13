"""Seabed relief across the swath from the *shading* of a recording --
photoclinometry anchored on the first bottom return.

The FBR gives one depth per ping, under the boat. Everything else in a
side-scan image is brightness, and on a Lambertian seabed brightness is
slope: a facet tilted toward the transducer returns more, one tilted away
less. Reading that slope out and integrating it across the swath from the
FBR depth is the classic shape-from-shading of side-scan sonar (Cuschieri
& Hebert 1990 [S1]; Johnson & Hebert 1996 [S2]; Coiras, Petillot & Lane
2007 [S3], whose EM alternation of reflectivity / elevation / beam pattern
this deterministic pass is the one-shot version of), the altimeter being
the boundary condition (Xie, Bore & Folkesson 2022 [S4]).

Per profile, on the per-bin reflectivity estimate of
:mod:`.backscatter` (the calibrated flat-floor model already removed):

1. **average first** -- a single-look bin is Exp(1)-speckled, and a
   slope read from one bin integrates into a random walk; the ratio is
   smoothed over ``along_pings`` pings (~2 m of track at 20 Hz / 1 m/s)
   and ``along_range_m`` of range before anything is inverted;
2. **reflectivity vs slope** -- the albedo/slope ambiguity ([S2] par. 4,
   [S3] par. III) is split by scale: the reflectivity is the linear mean
   over ``albedo_window_m`` along range, the slope is what the smoothed
   ratio deviates from it by. The window must be longer than the bedforms it
   should keep: measured on the smoke's 10 m ripples, an 8 m window
   swallows them (correlation 0.58 with the truth), 20 m keeps them
   (0.91); a material patch narrower than the window reads as relief --
   that is the ambiguity, chosen by scale;
3. **Lambert inverted** -- ``cos(theta_i) = sin(theta_g) (I/rho)^(1/p)``,
   so the across-track tilt is ``phi = asin(clip(sin(theta_g)
   (I/rho)^(1/p))) - theta_g``, clamped to ``+-clamp_deg``; a shadow or
   no-return bin carries no slope and is bridged flat (flagged);
4. **integrated** from the nadir depth outward, ``dh/dy = tan(phi)``
   (``h`` up), then the ground range **re-solved once** with the new
   height, ``y = sqrt(r^2 - (a - dh)^2)`` ([S1]'s geometric reasoning;
   [S6]), and integrated again;
5. **detrended** along the ray over ``trend_length_m``: the along-track
   direction is unobservable from one line, so only the *relief* is kept
   and the spline through the FBR soundings stays the low-frequency
   surface (``worldgen.depth.smooth`` fuses the two).

The amplitude is uncertain by the undetermined Lambert exponent
(``docs/sonar_model.md`` par. 6: a factor of ~1-2). Ran et al. 2025 [S6]
calibrate their shape-from-shading DEM by regression onto the echo
sounder's soundings; :func:`calibrate_gain` does the same with the FBR
soundings of the *other* lines that fall inside a swath (the tie lines
``docs/svlog_depth_survey.md`` asks for).

ROS-free; reads only through :mod:`.backscatter` and the recording reader.
"""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy import ndimage

from ..core.geo import GeoAnchor
from ..core.types import Side
from ..sonar.config import SonarModelConfig
from .backscatter import (InversionConfig, _cfg_key, _file_floor_db,
                          _iter_placed_profiles)

__all__ = ["ReliefSamples", "relief_samples", "calibrate_gain",
           "relief_quality", "detrend_along_ray", "soundings_relief"]

FLAG_BRIDGED = 1        # shadow / no-return bins bridged flat
FLAG_CLAMPED = 2        # slope at the clamp, or relief at relief_max_m


@dataclass
class ReliefSamples:
    """Relief per inverted bin, placed on the ground (design/world
    metres). ``dz`` is *depth* relief (positive = deeper than the
    along-ray trend), ``depth_abs`` the integrated absolute depth,
    ``depth_nadir`` the ping's FBR depth the integration started from."""

    east: np.ndarray
    north: np.ndarray
    dz: np.ndarray
    depth_abs: np.ndarray
    depth_nadir: np.ndarray
    weight: np.ndarray
    ping: np.ndarray            # int64
    side: np.ndarray            # int8: +1 port, -1 starboard
    t_ms: np.ndarray            # profile clock
    flags: np.ndarray           # uint8
    ground_m: np.ndarray | None = None   # horizontal distance from the transducer
    gain: float = 1.0           # the relief gain applied downstream

    def __len__(self) -> int:
        return len(self.dz)


def _nan_uniform(a: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    ok = np.isfinite(a)
    num = ndimage.uniform_filter(np.where(ok, a, 0.0), size=size, mode="nearest")
    den = ndimage.uniform_filter(ok.astype(np.float64), size=size, mode="nearest")
    out = np.where(den > 0.2, num / np.maximum(den, 1e-9), np.nan)
    return np.where(ok, out, np.nan)


def _block_mean(a: np.ndarray, k: int) -> np.ndarray:
    """NaN-aware mean over non-overlapping blocks of ``k`` samples along
    the last axis (a trailing partial block is dropped).

    This *is* the along-range averaging step 1 of the module doc, done at
    ingestion rather than afterwards: a block of ``k`` bins that would
    have been smoothed by a ``k``-wide running mean and then read out at
    one coarse cell carries the same information as its mean, at 1/k the
    memory. Keeping every native bin of a 100 000-ping recording is what
    made this pass allocate >10 GB (2026-09-18)."""
    k = max(1, int(k))
    if k == 1:
        return a
    n = (a.shape[-1] // k) * k
    if n == 0:
        return a[..., :0]
    blocks = a[..., :n].reshape(*a.shape[:-1], n // k, k)
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(blocks)
        num = np.where(ok, blocks, 0.0).sum(axis=-1)
        den = ok.sum(axis=-1)
        return np.where(den > 0, num / np.maximum(den, 1), np.nan)


#: The inversion runs on every ping and every bin; only the *output* is
#: decimated, to one sample per this much range. The consumer rasterises
#: onto a half-metre grid, so a denser return is pure memory.
DEFAULT_OUT_SPACING_M = 0.5
#: Along-track rows inverted at a time (a halo of the along-track window is
#: added on each side). Peak memory is set by this, not by the survey.
CHUNK_PINGS = 4096


def _sensor_depth_default() -> float:
    return float(SonarModelConfig().sensor_depth_m)


@functools.lru_cache(maxsize=2)
def _relief_bundle(path: str, mtime_ns: int, size: int, lat0: float,
                   lon0: float, inv_key: tuple, cfg_key: tuple,
                   opts: tuple) -> ReliefSamples:
    """The cached body of :func:`relief_samples` (path + mtime + size +
    anchor + every inversion parameter). The World Builder re-previews on
    every keystroke; without this the whole recording was re-read and
    re-inverted each time, which is what made ``relief: shading``
    unusable on a 180 MB survey (2026-09-18)."""
    return _relief_samples_uncached(
        path, GeoAnchor(lat0=lat0, lon0=lon0),
        cfg=SonarModelConfig(**dict(cfg_key)) if cfg_key else SonarModelConfig(),
        inv=InversionConfig(*inv_key), **dict(opts))


def relief_samples(path: str | Path, anchor: GeoAnchor, *,
                   cfg: SonarModelConfig | None = None,
                   inv: InversionConfig = InversionConfig(),
                   **kw) -> ReliefSamples:
    """Cached wrapper of :func:`_relief_samples_uncached`; see it for the
    parameters. Cached per file (path, mtime, size), anchor and every
    inversion option, so re-previewing a world with the same recording
    costs nothing."""
    p = Path(path)
    st = p.stat()
    return _relief_bundle(str(p), st.st_mtime_ns, st.st_size,
                          float(anchor.lat0), float(anchor.lon0),
                          inv.key(), _cfg_key(cfg),
                          tuple(sorted(kw.items())))


def _relief_samples_uncached(path: str | Path, anchor: GeoAnchor, *,
                   cfg: SonarModelConfig | None = None,
                   inv: InversionConfig = InversionConfig(),
                   along_pings: int = 40, along_range_m: float = 0.25,
                   albedo_window_m: float = 20.0, trend_length_m: float = 15.0,
                   clamp_deg: float = 30.0, relief_max_m: float = 1.5,
                   sensor_depth_m: float | None = None,
                   datum_offset_m: float = 0.0, bin_step: int | None = None,
                   out_spacing_m: float = DEFAULT_OUT_SPACING_M,
                   chunk_pings: int = CHUNK_PINGS,
                   debias: bool = True) -> ReliefSamples:
    """The relief every lit bin of a recording implies (see the module
    doc).

    The inversion runs at the recording's **full** resolution -- every
    ping, every bin -- because its accuracy comes from averaging: the
    along-track window beats single-look speckle down by sqrt(N), and
    decimating the input costs that directly (measured on the smoke's
    ripple survey: a ping stride of 8 raised the relief RMS 14 % and the
    correlation with the truth fell 0.90 -> 0.83).

    Memory is bounded instead by two things that cost nothing:

    * the per-group matrices are built and inverted in along-track
      **chunks** of ``chunk_pings`` rows with a halo the along-track
      window wide, so peak memory is set by the chunk and not by the
      length of the survey;
    * the *output* is decimated to one sample every ``out_spacing_m``
      along range (``bin_step`` overrides it). The consumer rasterises
      these onto a half-metre grid, so emitting every 25 mm bin only
      inflated the return value -- 35 million samples on a 180 MB
      recording, 3 GB of arrays.

    Together with the ``relief_samples`` cache this is what took the
    World Builder's shading mode from an OOM kill (13 GB resident,
    2026-09-18) to a fraction of a gigabyte.
    """
    cfg = cfg or SonarModelConfig()
    p_exp = float(cfg.lambert_exponent)
    sd = _sensor_depth_default() if sensor_depth_m is None else float(sensor_depth_m)
    floor = _file_floor_db(path, cfg, inv)
    # Group profiles by side and acquisition (one matrix each, file order).
    groups: dict[tuple, dict[str, list]] = {}
    for prof, e0, n0, look, b in _iter_placed_profiles(path, anchor, cfg, inv, floor):
        key = (prof.side, int(prof.start_mm), int(prof.length_mm), int(prof.num_results))
        g = groups.setdefault(key, {"i": [], "w": [], "gr": [], "gd": [], "r": None,
                                    "e": [], "n": [], "look": [], "t": [], "pn": [],
                                    "alt": []})
        g["i"].append(b["i_lin"].astype(np.float32))
        g["w"].append(b["weight"].astype(np.float32))
        g["gr"].append(b["grazing_deg"].astype(np.float32))
        g["gd"].append(b["ground_m"].astype(np.float32))
        g["r"] = b["slant_m"]
        g["e"].append(e0); g["n"].append(n0); g["look"].append(look)
        g["t"].append(float(prof.timestamp_ms)); g["pn"].append(int(prof.ping_number))
        # the altitude the bins were inverted with: ground = sqrt(r^2 - a^2)
        past = np.flatnonzero(b["slant_m"] > 0)
        a = float(np.sqrt(max(float(b["slant_m"][-1]) ** 2
                              - float(b["ground_m"][-1]) ** 2, 0.0))) if past.size else float("nan")
        g["alt"].append(a)
    E, N, DZ, DA, DN, W, P, S, T, F, GR = ([] for _ in range(11))
    for (side, _s, _l, nres), g in groups.items():
        n_p = len(g["i"])
        if n_p == 0:
            continue
        r = np.asarray(g["r"], dtype=np.float64)
        n_b = r.size
        bin_m = float(r[1] - r[0]) if n_b > 1 else 0.025
        k_rng = max(1, int(round(along_range_m / bin_m)))
        k_alb = max(k_rng, int(round(albedo_window_m / bin_m)))
        k_trk = max(1, min(int(along_pings), n_p))
        b_out = (max(1, int(round(out_spacing_m / max(bin_m, 1e-6))))
                 if bin_step is None else max(1, int(bin_step)))
        keep_cols = np.zeros(n_b, dtype=bool)
        keep_cols[::b_out] = True
        e_all = np.asarray(g["e"], dtype=np.float64)
        n_all = np.asarray(g["n"], dtype=np.float64)
        look_all = np.asarray(g["look"], dtype=np.float64)
        t_all = np.asarray(g["t"], dtype=np.float64)
        pn_all = np.asarray(g["pn"], dtype=np.int64)
        alt_all = np.asarray(g["alt"], dtype=np.float64)
        clamp = math.radians(clamp_deg)
        # Per-group accumulators for the across-track debias (below).
        bias_sum = np.zeros(n_b); bias_cnt = np.zeros(n_b)
        gE, gN, gDZ, gDA, gDN, gW, gP, gT, gF, gGR, gBI = ([] for _ in range(11))
        halo = int(k_trk)
        step = max(int(chunk_pings), 1)
        for lo0 in range(0, n_p, step):
            hi0 = min(n_p, lo0 + step)
            lo = max(0, lo0 - halo); hi = min(n_p, hi0 + halo)
            sl = slice(lo0 - lo, lo0 - lo + (hi0 - lo0))     # rows to emit
            I = np.stack(g["i"][lo:hi]).astype(np.float64)
            I_s = _nan_uniform(I, (k_trk, k_rng))
            del I
            with np.errstate(invalid="ignore", divide="ignore"):
                rho = _nan_uniform(I_s, (k_trk, k_alb))
                ratio = I_s / np.maximum(rho, 1e-12)
            del I_s, rho
            grazing = np.radians(np.stack(g["gr"][lo:hi]).astype(np.float64))
            ground = np.stack(g["gd"][lo:hi]).astype(np.float64)
            weight = np.stack(g["w"][lo:hi]).astype(np.float64)
            alt = alt_all[lo:hi]
            # Two inverses, because there are two forward models.
            #
            # Under `image_formation: energy` with `facet_area_jacobian` the
            # renderer weights a sample by the ground area it stands for,
            # sqrt(1+s^2), and normalises the bin by one bin of flat ground.
            # For lambert_exponent 1 that collapses to something exact.
            # cos(theta_i) sqrt(1+s^2) = (dz + y s)/r, and the bin's ground
            # extent carries 1/|dr/dy| = r/(y - dz s), so
            #   I ~ rho (dz + y s)/(y - dz s) = rho tan(dep + phi),
            # with phi = arctan(s) the local tilt. Divided by the flat-floor
            # value rho tan(dep), the ratio inverts in closed form:
            #   phi = arctan(ratio tan(dep)) - dep.
            # No albedo, no exponent, no small-angle step -- the whole
            # photoclinometry is one arctangent.
            #
            # The legacy form inverts I ~ rho cos^p(theta_i) through the
            # flat-floor grazing angle. It is kept for a recording paired
            # with a frozen legacy sonar.yaml, and for lambert_exponent != 1,
            # where the area element does not separate.
            jac_inv = (str(getattr(cfg, "image_formation", "mean")).lower()
                       == "energy" and bool(getattr(cfg, "facet_area_jacobian",
                                                    False))
                       and abs(p_exp - 1.0) < 1e-9)
            if jac_inv:
                cos_i = None
                lit = np.isfinite(ratio) & (weight > 0.0)
                flags = np.zeros(ratio.shape, dtype=np.uint8)
            else:
                with np.errstate(invalid="ignore"):
                    # cos(theta_i) as the model's flat-floor grazing angle
                    # implies.
                    cos_i = np.clip(
                        np.sin(grazing)
                        * np.power(np.maximum(ratio, 1e-6), 1.0 / p_exp),
                        0.0, 1.0)
                del ratio
                ratio = None
                lit = np.isfinite(cos_i) & (weight > 0.0)
                flags = np.zeros(cos_i.shape, dtype=np.uint8)
            flags[~lit] |= FLAG_BRIDGED
            past = np.isfinite(ground) & (ground > 0.0)
            h_nadir = -(alt + sd + float(datum_offset_m))         # height, up

            def _integrate(graz_local, y_ground):
                with np.errstate(invalid="ignore"):
                    if jac_inv:
                        phi = (np.arctan(ratio * np.tan(graz_local))
                               - graz_local)
                    else:
                        phi = np.arcsin(cos_i) - graz_local
                over = lit & (np.abs(phi) >= clamp)
                phi = np.where(lit, np.clip(phi, -clamp, clamp), 0.0)
                dy = np.diff(y_ground, axis=1, prepend=0.0) * past
                return np.cumsum(np.tan(phi) * dy, axis=1), over

            # Pass 1 on the flat-floor geometry; pass 2 with the ground range
            # re-solved from the pass-1 height. The grazing angle stays the
            # flat-floor one in both passes: feeding the integrated height
            # back into the angle it is integrated with is unstable (measured:
            # the ripple correlation fell from 0.91 to 0.83 and the flat-floor
            # noise rose from 3.7 to 6.2 cm).
            y = np.where(past, ground, 0.0)
            dh, _ = _integrate(grazing, y)
            with np.errstate(invalid="ignore", divide="ignore"):
                vert = np.maximum(alt[:, None] - dh, 0.05)
                y2 = np.where(past, np.sqrt(np.maximum(r[None, :] ** 2 - vert ** 2,
                                                       0.0)), 0.0)
            dh, over = _integrate(grazing, y2)
            flags[over] |= FLAG_CLAMPED
            h = h_nadir[:, None] + dh
            # Relief about the along-ray trend (the spline carries the rest).
            dy2 = np.diff(y2, axis=1, prepend=0.0) * past
            mean_dy = float(np.mean(dy2[past])) if past.any() else bin_m
            k_trend = max(3, int(round(trend_length_m / max(mean_dy, 1e-6))))
            trend = ndimage.uniform_filter1d(h, size=k_trend, axis=1, mode="nearest")
            dz = -(h - trend)                                   # depth relief
            sel = (past & lit)[sl] & keep_cols[None, :]
            # The debias curve is accumulated over the whole group (below),
            # on the clipped relief so one wild ray cannot tilt it.
            good = (past & lit)[sl]
            dzc = np.clip(dz[sl], -relief_max_m, relief_max_m)
            bias_sum += np.where(good, dzc, 0.0).sum(axis=0)
            bias_cnt += good.sum(axis=0)
            over_r = np.abs(dz[sl]) > relief_max_m
            fl = flags[sl].copy()
            fl[over_r] |= FLAG_CLAMPED
            pi, bi = np.nonzero(sel)
            if pi.size:
                lk = look_all[lo0:hi0][pi]
                y2e = y2[sl][pi, bi]
                gE.append(e_all[lo0:hi0][pi] + y2e * np.cos(lk))
                gN.append(n_all[lo0:hi0][pi] + y2e * np.sin(lk))
                gDZ.append(dz[sl][pi, bi])
                gDA.append(-h[sl][pi, bi])
                gDN.append(-h_nadir[sl][pi])
                gW.append(weight[sl][pi, bi])
                gP.append(pn_all[lo0:hi0][pi])
                gT.append(t_all[lo0:hi0][pi])
                gF.append(fl[pi, bi])
                gGR.append(y2e)
                gBI.append(bi.astype(np.int32))
            del cos_i, ratio, lit, flags, past, grazing, ground, weight, dh, y, y2, h, trend, dz
        if not gDZ:
            continue
        g_dz = np.concatenate(gDZ)
        if debias:
            # Across-track debias -- the relief's angle-varying gain.
            #
            # The along-ray running mean is edge-replicated at both ends of
            # every ray, and the flat-floor grazing angle is systematically
            # wrong by the same amount at the same range on every ping, so
            # each pass leaves an identical dz(range) signature. Assembled,
            # that signature repeats at the survey's line spacing and the
            # depth map shows the boat's own track printed across it
            # (reported 2026-09-18). Subtracting what is common to every
            # ping of the group keeps only what varies along the track --
            # the seabed.
            #
            # Only the part slower than the albedo window is removed. That
            # window is already the method's scale split (step 2): relief
            # slower than it is not separable from reflectivity, so it is
            # not relief the pass can claim, while a bedform inside the
            # band survives. Without the guard the subtraction also removed
            # a range-locked ripple field measured on one straight line
            # (smoke [4c] correlation 0.87 -> 0.03).
            with np.errstate(invalid="ignore"):
                col = np.where(bias_cnt > 0, bias_sum / np.maximum(bias_cnt, 1), 0.0)
            if n_b > 3:
                col = ndimage.uniform_filter1d(
                    col, size=max(3, min(int(k_alb), n_b)), mode="nearest")
            g_dz = g_dz - col[np.concatenate(gBI)]
        over = np.abs(g_dz) > relief_max_m
        g_fl = np.concatenate(gF)
        g_fl[over] |= FLAG_CLAMPED
        E.append(np.concatenate(gE)); N.append(np.concatenate(gN))
        DZ.append(np.clip(g_dz, -relief_max_m, relief_max_m))
        DA.append(np.concatenate(gDA)); DN.append(np.concatenate(gDN))
        W.append(np.concatenate(gW)); P.append(np.concatenate(gP))
        S.append(np.full(g_dz.size, int(side.sign), dtype=np.int8))
        T.append(np.concatenate(gT)); F.append(g_fl); GR.append(np.concatenate(gGR))
        for k in ("i", "w", "gr", "gd"):
            g[k] = None                       # free this group before the next
    if not E:
        z = np.empty(0)
        return ReliefSamples(z, z, z, z, z, z, np.empty(0, np.int64),
                             np.empty(0, np.int8), z, np.empty(0, np.uint8), z)
    return ReliefSamples(np.concatenate(E), np.concatenate(N), np.concatenate(DZ),
                         np.concatenate(DA), np.concatenate(DN), np.concatenate(W),
                         np.concatenate(P), np.concatenate(S), np.concatenate(T),
                         np.concatenate(F), np.concatenate(GR))


def detrend_along_ray(values: np.ndarray, samples: ReliefSamples,
                      trend_length_m: float) -> np.ndarray:
    """``values`` minus their running mean over ``trend_length_m`` along
    each (ping, side) ray -- the same detrending the relief carries, so a
    ground truth sampled at the same points compares like with like."""
    out = np.zeros_like(values, dtype=np.float64)
    key = samples.ping.astype(np.int64) * 4 + (samples.side.astype(np.int64) + 1)
    order = np.lexsort((samples.ground_m, key))
    k_s = key[order]
    starts = np.flatnonzero(np.r_[True, k_s[1:] != k_s[:-1]])
    ends = np.r_[starts[1:], k_s.size]
    for a, b in zip(starts, ends):
        idx = order[a:b]
        g = samples.ground_m[idx]
        v = values[idx]
        if idx.size < 3:
            out[idx] = v - v.mean()
            continue
        step = max(float(np.median(np.diff(g))), 1e-6)
        k = max(3, int(round(trend_length_m / step)))
        out[idx] = v - ndimage.uniform_filter1d(v, size=k, mode="nearest")
    return out


def soundings_relief(east: np.ndarray, north: np.ndarray, depth: np.ndarray,
                     t_ms: np.ndarray, trend_length_m: float) -> np.ndarray:
    """Each FBR sounding's depth minus the running mean of its own line
    over ``trend_length_m`` of track (soundings in time order): the relief
    the altimeter itself measured along the line."""
    n = depth.size
    out = np.zeros(n)
    if n < 3:
        return out
    order = np.argsort(t_ms, kind="stable")
    e, nn, d = east[order], north[order], depth[order]
    step = np.hypot(np.diff(e), np.diff(nn))
    # A jump of more than 5 m between consecutive soundings is a new line.
    breaks = np.flatnonzero(step > 5.0) + 1
    starts = np.r_[0, breaks]
    ends = np.r_[breaks, n]
    rel = np.zeros(n)
    for a, b in zip(starts, ends):
        seg = d[a:b]
        if seg.size < 3:
            rel[a:b] = seg - seg.mean()
            continue
        sp = max(float(np.median(step[a:b - 1])) if b - a > 1 else 0.05, 1e-3)
        k = max(3, int(round(trend_length_m / sp)))
        rel[a:b] = seg - ndimage.uniform_filter1d(seg, size=k, mode="nearest")
    out[order] = rel
    return out


def calibrate_gain(samples: ReliefSamples, soundings: dict | None, *,
                   trend_length_m: float = 15.0, radius_m: float = 0.5,
                   gap_ms: float = 60_000.0, min_pairs: int = 30,
                   lo: float = 0.1, hi: float = 3.0
                   ) -> tuple[float, dict[str, Any]]:
    """The relief gain that best maps the shading relief onto the relief
    the FBR of *other* lines measured where they cross the swath (Ran et
    al. 2025 [S6] par. 2.2.3, a regression of the shading DEM on the
    echo-sounder depths -- here on the two *reliefs*, both detrended over
    ``trend_length_m``, so the integration's slow drift plays no part):
    least squares through the origin over sounding / sample pairs within
    ``radius_m`` and more than ``gap_ms`` apart, clipped to [lo, hi]. 1.0
    with a note when fewer than ``min_pairs`` pairs exist."""
    info: dict[str, Any] = {"gain_pairs": 0, "gain_residual_m": None,
                            "gain_mode": "default"}
    if not soundings or len(samples) == 0:
        return 1.0, info
    try:
        from scipy.spatial import cKDTree
    except ImportError:                                   # pragma: no cover
        return 1.0, info
    se = np.asarray(soundings.get("east", ()), dtype=np.float64)
    sn = np.asarray(soundings.get("north", ()), dtype=np.float64)
    sdp = np.asarray(soundings.get("depth", ()), dtype=np.float64)
    st = np.asarray(soundings.get("t_ms", ()), dtype=np.float64)
    if se.size < 3:
        return 1.0, info
    s_rel = soundings_relief(se, sn, sdp, st, trend_length_m)
    tree = cKDTree(np.column_stack([samples.east, samples.north]))
    P, M, Wt = [], [], []
    for k in range(se.size):
        idx = tree.query_ball_point((se[k], sn[k]), radius_m)
        if not idx:
            continue
        idx = np.asarray(idx)
        far = np.abs(samples.t_ms[idx] - st[k]) > gap_ms
        idx = idx[far & (samples.weight[idx] > 0.0)]
        if idx.size == 0:
            continue
        pred = samples.dz[idx]
        meas = np.full(idx.size, s_rel[k])
        P.append(pred); M.append(meas); Wt.append(samples.weight[idx])
    if not P:
        return 1.0, info
    p = np.concatenate(P); m = np.concatenate(M); w = np.concatenate(Wt)
    info["gain_pairs"] = int(p.size)
    den = float(np.sum(w * p * p))
    if p.size < min_pairs or den < 1e-6:
        info["gain_mode"] = "default (too few tie soundings)"
        return 1.0, info
    gain = float(np.sum(w * p * m) / den)
    gain = float(np.clip(gain, lo, hi))
    resid = m - gain * p
    info["gain_residual_m"] = float(np.sqrt(np.sum(w * resid * resid) / np.sum(w)))
    info["gain_mode"] = "calibrated on tie soundings"
    return gain, info


def relief_quality(samples: ReliefSamples, soundings: dict | None,
                   cell_m: float = 1.0, gap_ms: float = 60_000.0,
                   reach_m: float = 15.0) -> dict[str, Any]:
    """What the relief covered and how consistent it is: cells, the share
    of the surveyed area (cells within ``reach_m`` of a sounding -- the
    reach ``sample_coverage`` uses, so a single line counts too) with a
    relief sample, the relief RMS, the disagreement where two passes more
    than ``gap_ms`` apart saw the same cell (the shading analogue of the
    crossover residual), and the fractions of bridged and clamped bins."""
    n = len(samples)
    out: dict[str, Any] = {"n_samples": int(n), "cells": 0,
                           "coverage_fraction": 0.0, "rms_m": 0.0,
                           "overlap_rms_m": None, "overlap_cells": 0,
                           "shadow_fraction": 0.0, "clamped_fraction": 0.0,
                           "cell_m": float(cell_m)}
    if n == 0:
        return out
    good = samples.weight > 0.0
    out["shadow_fraction"] = float(np.mean((samples.flags & FLAG_BRIDGED) > 0))
    out["clamped_fraction"] = float(np.mean((samples.flags & FLAG_CLAMPED) > 0))
    w = samples.weight[good]
    out["rms_m"] = float(np.sqrt(np.sum(w * samples.dz[good] ** 2) / max(np.sum(w), 1e-9)))
    e, nn = samples.east[good], samples.north[good]
    ox = math.floor(float(e.min()) / cell_m) * cell_m
    oy = math.floor(float(nn.min()) / cell_m) * cell_m
    ci = np.round((e - ox) / cell_m).astype(np.int64)
    cj = np.round((nn - oy) / cell_m).astype(np.int64)
    nx, ny = int(ci.max()) + 1, int(cj.max()) + 1
    flat = cj * nx + ci
    count = np.bincount(flat, minlength=nx * ny).reshape(ny, nx)
    covered = count > 0
    out["cells"] = int(covered.sum())
    if soundings and len(soundings.get("east", ())) >= 1:
        from scipy.spatial import cKDTree
        xs = ox + np.arange(nx) * cell_m
        ys = oy + np.arange(ny) * cell_m
        xx, yy = np.meshgrid(xs, ys)
        tree = cKDTree(np.column_stack([np.asarray(soundings["east"], dtype=float),
                                        np.asarray(soundings["north"], dtype=float)]))
        d, _ = tree.query(np.column_stack([xx.ravel(), yy.ravel()]),
                          distance_upper_bound=reach_m)
        near = np.isfinite(d).reshape(xx.shape)
        out["coverage_fraction"] = (float(np.sum(covered & near) / near.sum())
                                    if near.any() else 0.0)
        out["reach_m"] = float(reach_m)
    # Overlap: cells seen by two passes far apart in time.
    order = np.argsort(flat, kind="stable")
    f_s, t_s, z_s = flat[order], samples.t_ms[good][order], samples.dz[good][order]
    starts = np.flatnonzero(np.r_[True, f_s[1:] != f_s[:-1]])
    ends = np.r_[starts[1:], f_s.size]
    diffs = []
    for a, b in zip(starts, ends):
        if b - a < 4:
            continue
        t = t_s[a:b]
        if t.max() - t.min() <= gap_ms:
            continue
        tm = 0.5 * (t.min() + t.max())
        early, late = z_s[a:b][t <= tm], z_s[a:b][t > tm]
        if early.size and late.size:
            diffs.append(float(early.mean() - late.mean()))
    if diffs:
        d = np.asarray(diffs)
        out["overlap_rms_m"] = float(np.sqrt(np.mean(d * d)))
        out["overlap_cells"] = int(d.size)
    return out
