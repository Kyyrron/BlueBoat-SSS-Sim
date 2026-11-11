"""Real vs simulated side-scan PICTURES, the way the AI sees them.

The detector is fed the GCS's ``seabed_images/`` (256-row tiles in native
slant bins, display-model normalised; ``metadata/*.json`` +
``*_world.npz`` beside each PNG). This module compares sessions -- a real
recording rebuilt by the GCS, the geometric renderer, the ray-cast
renderer -- on those tiles, not on the ``.svlog`` dB alone:

* **features** -- a known seabed feature (a world point, e.g. an anchor
  block) is located in every session through each session's own
  local<->GPS pairing, cropped (near -> far slant range around it) and
  measured: highlight and shadow depth relative to the seabed in front
  (dB, from ``intensity_db``), the shadow's texture, and in the PIXEL
  domain the shadow's mean grey level, its fraction of pure-black pixels
  and the highlight's fraction of saturated pixels;
* **texture** -- plain-seabed speckle statistics per side: lag-1
  correlation along range and along track, 9x9 high-pass std (dB), and
  the pixel histogram (what a CNN's first layer sees);
* **row banding** -- the std of the per-row near-range mean grey level
  and the port/starboard correlation of its fluctuation (roll banding is
  anti-correlated, a common-mode level change correlated);
* **bottom jitter** -- the row-to-row jump of the tracked altitude
  (``row_bottom_m``), which the display model and the georeference use.

ROS- and Qt-free (numpy, PIL for the panels). Never writes into a
session. CLI::

    python3 -m blueboat_sss_sim.analysis.picture_compare \\
        --session real=PATH --session geometric=PATH ... \\
        --anchor 34.4082349219396 133.5245677376026 \\
        --feature A1=50.59,2.46 --feature A2=20.72,1.86 --out OUT
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


# ===================================================================== data
@dataclass
class Tile:
    """One seabed image (lazy arrays)."""

    png: Path
    meta: dict
    _world: dict | None = None
    _pix: np.ndarray | None = None

    @property
    def world(self) -> dict:
        if self._world is None:
            npz = self.png.parent / "metadata" / (self.png.stem + "_world.npz")
            with np.load(npz, allow_pickle=False) as z:
                self._world = {k: z[k] for k in z.files}
        return self._world

    @property
    def pixels(self) -> np.ndarray:
        if self._pix is None:
            from PIL import Image  # noqa: PLC0415
            self._pix = np.asarray(Image.open(self.png), dtype=np.float64)
        return self._pix

    @property
    def pitch(self) -> float:
        return float(self.meta["bin_pitch_m"])

    @property
    def width(self) -> int:
        return int(self.meta["cols"])

    def slant_k(self, j) -> np.ndarray:
        """Bin index k of column(s) j (pixel convention: col 0 = far port,
        last col = far starboard, the seam = the transducer)."""
        j = np.asarray(j)
        half = self.width // 2
        return np.where(j < half, half - 1 - j, j - half)

    def col_of(self, side: str, k) -> np.ndarray:
        half = self.width // 2
        k = np.asarray(k)
        return (half - 1 - k) if side == "port" else (half + k)


@dataclass
class Session:
    name: str
    root: Path
    tiles: list[Tile] = field(default_factory=list)
    offset_xy: tuple[float, float] = (0.0, 0.0)   # world_local - session_local
    offset_n: int = 0


def load_session(name: str, root: str | Path) -> Session:
    root = Path(root).expanduser()
    sb = root / "seabed_images"
    tiles = []
    for png in sorted(sb.glob("seabed_*.png")):
        js = sb / "metadata" / (png.stem + ".json")
        if js.is_file():
            tiles.append(Tile(png, json.loads(js.read_text())))
    if not tiles:
        raise FileNotFoundError(f"no seabed_images/*.png + metadata in {root}")
    return Session(name, root, tiles)


def frame_offset(session: Session, lat0: float, lon0: float) -> tuple[float, float, int]:
    """(dx, dy, n): the translation from the session's local frame (the
    recording's LOCAL_POSITION / odometry, what ``world_x/world_y`` are in)
    to the world frame anchored at (lat0, lon0), from the session's own
    ``.svlog`` GPS fixes paired with its local positions (median of the
    differences; a simulated session gives ~0)."""
    from ..core.geo import latlon_to_local_en  # noqa: PLC0415
    from .svlog_reader import read_svlog  # noqa: PLC0415
    logs = sorted(session.root.glob("*.svlog"))
    if not logs:
        return 0.0, 0.0, 0
    s = read_svlog(logs[0])
    if len(s.gps) < 5 or len(s.positions) < 5:
        return 0.0, 0.0, 0
    if s.clock_frozen:
        po = np.array([p.order for p in s.positions])
        go = np.array([g.order for g in s.gps])
        idx = np.clip(np.searchsorted(po, go) - 1, 0, len(po) - 1)
        lx = np.array([s.positions[i].x for i in idx])
        ly = np.array([s.positions[i].y for i in idx])
    else:
        pt = np.array([p.time_boot_ms for p in s.positions], dtype=np.float64)
        o = np.argsort(pt, kind="stable")
        pt = pt[o]
        px = np.array([p.x for p in s.positions])[o]
        py = np.array([p.y for p in s.positions])[o]
        gt = np.array([g.time_boot_ms for g in s.gps], dtype=np.float64)
        lx, ly = np.interp(gt, pt, px), np.interp(gt, pt, py)
    lat = np.array([g.lat for g in s.gps])
    lon = np.array([g.lon for g in s.gps])
    e, n = latlon_to_local_en(lat, lon, lat0, lon0)
    dx = np.asarray(e, dtype=np.float64) - lx
    dy = np.asarray(n, dtype=np.float64) - ly
    ok = np.isfinite(dx) & np.isfinite(dy)
    return float(np.median(dx[ok])), float(np.median(dy[ok])), int(ok.sum())


# ================================================================ features
def locate(session: Session, x_world: float, y_world: float,
           max_dist_m: float = 0.25) -> list[tuple[int, int, int, float]]:
    """(tile index, row, col, distance) of every tile whose georeferenced
    pixels pass within ``max_dist_m`` of the world point, nearest first."""
    x = x_world - session.offset_xy[0]
    y = y_world - session.offset_xy[1]
    hits = []
    for ti, tile in enumerate(session.tiles):
        rows = tile.meta.get("rows_data") or []
        if rows:   # cheap reject: the tile's track must pass within ~25 m
            tx = np.array([r["x_m"] for r in rows])
            ty = np.array([r["y_m"] for r in rows])
            if np.hypot(tx - x, ty - y).min() > 1.3 * float(rows[0].get("range_m", 20.0)):
                continue
        wx, wy = tile.world["world_x"], tile.world["world_y"]
        d = np.hypot(wx - x, wy - y)
        d = np.where(np.isfinite(d), d, np.inf)
        flat = int(np.argmin(d))
        dm = float(d.ravel()[flat])
        if dm <= max_dist_m:
            i, j = np.unravel_index(flat, d.shape)
            hits.append((ti, int(i), int(j), dm))
    hits.sort(key=lambda h: (abs(h[1] - 128), h[3]))   # centred rows first
    return hits


def crop(tile: Tile, i: int, j: int, before_m: float = 4.0,
         after_m: float = 6.0, fg: dict | None = None) -> dict:
    """The tile around pixel (i, j), columns ordered near -> far slant
    range from ``s_a - before_m`` to ``s_a + after_m`` on the feature's
    side; every row of the tile (the AI tile height)."""
    side = "port" if j < tile.width // 2 else "starboard"
    p = tile.pitch
    ka = int(tile.slant_k(j))
    k = np.arange(max(ka - int(round(before_m / p)), 0),
                  min(ka + int(round(after_m / p)), tile.width // 2 - 1) + 1)
    cols = tile.col_of(side, k)
    db = tile.world["intensity_db"][:, cols].astype(np.float64)
    px = tile.pixels[:, cols]
    gray = fixed_gray(tile, fg)[:, cols] if fg is not None else None
    return {"side": side, "k": k, "ka": ka, "s": (k + 0.5) * p, "s_a": (ka + 0.5) * p,
            "row": i, "pitch": p, "db": db, "px": px, "gray": gray,
            "bottom_m": tile.world["row_bottom_m"].astype(np.float64)}


def _smooth(v: np.ndarray, n: int) -> np.ndarray:
    n = max(int(n), 1)
    if n == 1:
        return v
    return np.convolve(v, np.ones(n) / n, mode="same")


def feature_metrics(c: dict, half_rows_m: float = 0.5) -> dict:
    """Highlight / shadow of the feature in a crop (dB re the seabed in
    front of it, pixel statistics of the shadow and the highlight)."""
    p, s, sa = c["pitch"], c["s"], c["s_a"]
    nr = max(int(round(half_rows_m / p)), 1)
    r0, r1 = max(c["row"] - nr, 0), min(c["row"] + nr + 1, c["db"].shape[0])
    db = c["db"][r0:r1]
    px = c["px"][r0:r1]
    lin = np.where(np.isfinite(db), 10.0 ** (db / 10.0), np.nan)
    prof = 10.0 * np.log10(np.nanmean(lin, axis=0))
    front = (s >= sa - 4.0) & (s <= sa - 2.0)
    if not front.any() or not np.isfinite(prof[front]).any():
        return {"error": "no seabed in front of the feature"}
    ref = float(np.nanmedian(prof[front]))
    hl_win = (s >= sa - 2.0) & (s <= sa + 1.0)
    hl = _smooth(prof, round(0.15 / p))
    hi = float(np.nanmax(hl[hl_win])) - ref if hl_win.any() else float("nan")
    sm = _smooth(prof, round(1.0 / p))
    sh_win = np.flatnonzero((s >= sa - 1.0) & (s <= sa + 5.5))
    if sh_win.size == 0:
        return {"error": "no shadow window"}
    js = int(sh_win[np.nanargmin(sm[sh_win])])
    w = max(int(round(0.5 / p)), 1)
    sl = slice(max(js - w, 0), js + w + 1)
    sh_db = db[:, sl] - ref
    sh_px = px[:, sl]
    hl_px = px[:, hl_win]
    extra = {}
    if c.get("gray") is not None:
        gr = c["gray"][r0:r1]
        extra = {"det_seabed_mean": round(float(gr[:, front].mean()), 1),
                 "det_shadow_mean": round(float(gr[:, sl].mean()), 1),
                 "det_shadow_black_frac": round(float((gr[:, sl] <= 0.5).mean()), 3),
                 "det_highlight_sat_frac": round(float((gr[:, hl_win] >= 254.5).mean()), 3),
                 "det_highlight_p99": round(float(np.percentile(gr[:, hl_win], 99)), 1)}
    return {**extra, "seabed_ref_db": round(ref, 2),
            "highlight_db": round(hi, 2),
            "shadow_mean_db": round(float(sm[js]) - ref, 2),
            "shadow_p10_db": round(float(np.nanpercentile(sh_db, 10)), 2),
            "shadow_texture_db": round(float(np.nanstd(sh_db)), 2),
            "shadow_at_m": round(float(s[js] - sa), 2),
            "shadow_px_mean": round(float(sh_px.mean()), 1),
            "shadow_px_black_frac": round(float((sh_px <= 0.5).mean()), 3),
            "seabed_px_mean": round(float(px[:, front].mean()), 1),
            "highlight_px_sat_frac": round(float((hl_px >= 254.5).mean()), 3)}


# ============================================================ detector input
def load_fixed_gray(path: str | Path) -> dict:
    """Perception's versioned ``fixed_gray`` parameters (JSON, schema
    ``blueboat_perception/detector_input_fixed_gray/1``): the detector's
    input is ``round(255 * clip((db - g - R(s) - lo) / (hi - lo), 0, 1))``
    with ``g`` the row's receive-gain step and ``R(s)`` a fixed range
    reference. Read as DATA (CM-3: the neighbour package is not imported);
    the formula is the one documented in its ``core/preprocess.py``."""
    doc = json.loads(Path(path).expanduser().read_text())
    rr = doc["range_reference"]
    return {"x": np.asarray(rr["log10_slant_m"], np.float64),
            "r": np.asarray(rr["db"], np.float64),
            "lo": float(doc["window_db"]["lo"]), "hi": float(doc["window_db"]["hi"]),
            "name": doc.get("name", "fixed_gray")}


def fixed_gray(tile: Tile, fg: dict) -> np.ndarray:
    """The detector input of a whole tile (float 0..255, NaN -> 0)."""
    w = tile.width
    j = np.arange(w)
    s = (tile.slant_k(j) + 0.5) * tile.pitch
    ref = np.interp(np.log10(np.maximum(s, 1e-3)), fg["x"], fg["r"])
    g = tile.world.get("row_gain_db")
    if g is None:
        g = np.zeros((tile.world["intensity_db"].shape[0], 2))
    gp = np.where((j < w // 2)[None, :], g[:, 0:1], g[:, 1:2])
    y = tile.world["intensity_db"].astype(np.float64) - gp - ref[None, :]
    u = np.clip((y - fg["lo"]) / (fg["hi"] - fg["lo"]), 0.0, 1.0)
    return np.rint(255.0 * np.nan_to_num(u, nan=0.0))


# ================================================================= texture
def _lag1(a: np.ndarray, axis: int) -> float:
    a = a - np.nanmean(a)
    if axis == 1:
        x, y = a[:, :-1], a[:, 1:]
    else:
        x, y = a[:-1], a[1:]
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 50:
        return float("nan")
    x, y = x[m], y[m]
    return float(np.corrcoef(x, y)[0, 1])


def texture_metrics(session: Session, side: str, x0: float = 1.6,
                    x1_frac: float = 0.85, max_tiles: int = 40,
                    fg: dict | None = None) -> dict:
    """Plain-seabed speckle statistics on one side: per tile, the window
    from ``x0`` x the row's bottom to ``x1_frac`` x the range, the slow
    range trend (median over rows) and per-row level removed. Medians over
    the tiles (robust to a feature in a few of them)."""
    from scipy.ndimage import uniform_filter  # noqa: PLC0415
    vals: dict[str, list] = {k: [] for k in
                             ("lag1_range", "lag1_track", "hp_std_db",
                              "px_lag1_range", "px_lag1_track", "px_mean",
                              "px_std", "px_p1", "px_p99", "dark_tail_db",
                             "det_mean", "det_std", "det_p1", "det_p99",
                             "det_black_frac", "det_sat_frac")}
    step = max(len(session.tiles) // max_tiles, 1)
    for tile in session.tiles[::step]:
        h = float(np.nanmedian(tile.world["row_bottom_m"]))
        rng = float((tile.meta.get("rows_data") or [{}])[0].get("range_m", 20.0))
        p = tile.pitch
        k0, k1 = int(x0 * h / p), int(x1_frac * rng / p)
        if k1 - k0 < 64:
            continue
        cols = tile.col_of(side, np.arange(k0, k1))
        db = tile.world["intensity_db"][:, cols].astype(np.float64)
        px = tile.pixels[:, cols]
        if not np.isfinite(db).all():
            continue
        res = db - np.median(db, axis=0, keepdims=True)
        res = res - res.mean(axis=1, keepdims=True)
        vals["lag1_range"].append(_lag1(res, 1))
        vals["lag1_track"].append(_lag1(res, 0))
        vals["hp_std_db"].append(float((res - uniform_filter(res, 9)).std()))
        lin = 10.0 ** (res / 10.0)
        vals["dark_tail_db"].append(float(np.percentile(res, 1)
                                          - 10.0 * np.log10(lin.mean())))
        vals["px_lag1_range"].append(_lag1(px, 1))
        vals["px_lag1_track"].append(_lag1(px, 0))
        vals["px_mean"].append(float(px.mean()))
        vals["px_std"].append(float(px.std()))
        vals["px_p1"].append(float(np.percentile(px, 1)))
        vals["px_p99"].append(float(np.percentile(px, 99)))
        if fg is not None:
            gr = fixed_gray(tile, fg)[:, cols]
            vals["det_mean"].append(float(gr.mean()))
            vals["det_std"].append(float(gr.std()))
            vals["det_p1"].append(float(np.percentile(gr, 1)))
            vals["det_p99"].append(float(np.percentile(gr, 99)))
            vals["det_black_frac"].append(float((gr <= 0.5).mean()))
            vals["det_sat_frac"].append(float((gr >= 254.5).mean()))
    out = {k: (round(float(np.nanmedian(v)), 3) if v else None) for k, v in vals.items()}
    out["tiles"] = len(vals["lag1_range"])
    return out


def row_banding(session: Session, near_m: float = 6.0) -> dict:
    """Per-row mean grey level of the first ``near_m`` of seabed past the
    tracked bottom, per side, with the per-tile trend (25-row boxcar)
    removed: its std and the port/starboard correlation."""
    ps, ss = [], []
    for tile in session.tiles:
        p = tile.pitch
        bottom = tile.world["row_bottom_m"].astype(np.float64)
        nb = int(near_m / p)
        port, stbd = [], []
        for i, b in enumerate(bottom):
            if not np.isfinite(b) or b <= 0:
                port.append(np.nan)
                stbd.append(np.nan)
                continue
            k0 = int(b / p)
            kk = np.arange(k0, min(k0 + nb, tile.width // 2 - 1))
            port.append(tile.pixels[i, tile.col_of("port", kk)].mean())
            stbd.append(tile.pixels[i, tile.col_of("starboard", kk)].mean())
        for arr, acc in ((np.array(port), ps), (np.array(stbd), ss)):
            if np.isfinite(arr).sum() < 64:
                acc.append(np.full(arr.shape, np.nan))
                continue
            a = np.where(np.isfinite(arr), arr, np.nanmean(arr))
            acc.append(arr - _smooth(a, 25))
    P, S = np.concatenate(ps), np.concatenate(ss)
    m = np.isfinite(P) & np.isfinite(S)
    return {"row_std_port_px": round(float(np.nanstd(P)), 2),
            "row_std_stbd_px": round(float(np.nanstd(S)), 2),
            "port_stbd_corr": (round(float(np.corrcoef(P[m], S[m])[0, 1]), 3)
                               if m.sum() > 100 else None)}


def bottom_jitter(session: Session) -> dict:
    """Row-to-row jump of the tracked altitude, one sample per ping."""
    seen, vals = set(), []
    for tile in session.tiles:
        b = tile.world["row_bottom_m"].astype(np.float64)
        pi = tile.world["row_ping_index"]
        prev = None
        for v, k in zip(b, pi):
            key = (tile.meta["image_id"], int(k))
            if key in seen:
                continue
            seen.add(key)
            if prev is not None and np.isfinite(v) and np.isfinite(prev):
                vals.append(abs(v - prev))
            prev = v
    v = np.array(vals)
    if v.size == 0:
        return {}
    return {"p50_m": round(float(np.median(v)), 4),
            "p90_m": round(float(np.percentile(v, 90)), 4),
            "p99_m": round(float(np.percentile(v, 99)), 4)}


# ================================================================== panels
def _resample(img: np.ndarray, pitch_from: float, pitch_to: float) -> np.ndarray:
    """Nearest-neighbour resampling to a common ground pitch (both axes:
    'square' rows span one bin pitch along-track)."""
    f = pitch_from / pitch_to
    h, w = img.shape
    ri = np.clip((np.arange(int(round(h * f))) / f).astype(int), 0, h - 1)
    ci = np.clip((np.arange(int(round(w * f))) / f).astype(int), 0, w - 1)
    return img[np.ix_(ri, ci)]


def compose_panels(rows: list[tuple[str, list[tuple[str, np.ndarray | None]]]],
                   out: Path, pitch_from: dict, pitch_to: float = 0.0225,
                   cell_w: int = 460) -> Path:
    """A grid PNG: one row per feature, one column per session, every crop
    at the common ground pitch (near range on the left, far on the right;
    newest ping on top), labelled."""
    from PIL import Image, ImageDraw  # noqa: PLC0415
    cells = []
    for _feat, cols in rows:
        line = []
        for name, img in cols:
            if img is None:
                line.append(None)
                continue
            r = _resample(img, pitch_from[(_feat, name)], pitch_to)
            line.append(np.clip(r, 0, 255).astype(np.uint8))
        cells.append(line)
    cell_h = max((c.shape[0] for line in cells for c in line if c is not None),
                 default=100)
    cell_h = min(cell_h, 320)
    ncol = max(len(c) for _f, c in rows)
    lab = 22
    canvas = Image.new("L", (ncol * (cell_w + 6) + 6, len(rows) * (cell_h + lab + 6) + 6), 40)
    draw = ImageDraw.Draw(canvas)
    for ri, ((feat, cols), line) in enumerate(zip(rows, cells)):
        y0 = 6 + ri * (cell_h + lab + 6)
        for ci, ((name, _img), c) in enumerate(zip(cols, line)):
            x0 = 6 + ci * (cell_w + 6)
            draw.text((x0, y0 + 4), f"{feat} | {name}", fill=255)
            if c is None:
                draw.text((x0 + 10, y0 + lab + cell_h // 2), "not imaged", fill=160)
                continue
            img = Image.fromarray(c[:cell_h, :cell_w])
            canvas.paste(img, (x0, y0 + lab))
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out)
    return out


# ===================================================================== CLI
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="picture_compare",
                                 description=__doc__.split("\n\n")[0])
    ap.add_argument("--session", action="append", required=True,
                    help="NAME=PATH of a GCS session folder (repeatable)")
    ap.add_argument("--anchor", nargs=2, type=float, required=True,
                    metavar=("LAT0", "LON0"), help="the world's geo anchor")
    ap.add_argument("--feature", action="append", default=[],
                    help="NAME=X,Y world-local point (repeatable)")
    ap.add_argument("--override", action="append", default=[],
                    help="SESSION:FEATURE=X,Y -- the feature's position in that "
                         "session's world (e.g. a real block the depth file "
                         "misplaces)")
    ap.add_argument("--fixed-gray", default="",
                    help="Perception's fixed_gray JSON: also measure and draw "
                         "the detector's input")
    ap.add_argument("--max-dist", type=float, default=0.25)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    sessions = []
    for spec in args.session:
        name, path = spec.split("=", 1)
        s = load_session(name, path)
        s.offset_xy = frame_offset(s, *args.anchor)[:2]
        sessions.append(s)
        print(f"{name}: {len(s.tiles)} tiles, frame offset "
              f"({s.offset_xy[0]:+.2f}, {s.offset_xy[1]:+.2f}) m", flush=True)
    feats = []
    for spec in args.feature:
        name, xy = spec.split("=", 1)
        x, y = (float(v) for v in xy.split(","))
        feats.append((name, x, y))
    over = {}
    for spec in args.override:
        key, xy = spec.split("=", 1)
        sname, fname = key.split(":", 1)
        over[(sname, fname)] = tuple(float(v) for v in xy.split(","))
    fg = load_fixed_gray(args.fixed_gray) if args.fixed_gray else None
    res: dict[str, Any] = {"sessions": {}, "features": {},
                           "overrides": {f"{a}:{b}": v for (a, b), v in over.items()}}
    rows, rows_det, pitch = [], [], {}
    for fname, x, y in feats:
        cols, cols_det, res["features"][fname] = [], [], {}
        for s in sessions:
            fx, fy = over.get((s.name, fname), (x, y))
            hits = locate(s, fx, fy, args.max_dist)
            if not hits:
                cols.append((s.name, None))
                cols_det.append((s.name, None))
                res["features"][fname][s.name] = {"imaged": False}
                continue
            ti, i, j, d = hits[0]
            c = crop(s.tiles[ti], i, j, fg=fg)
            m = feature_metrics(c)
            m.update({"tile": s.tiles[ti].png.name, "row": i, "col": j,
                      "dist_m": round(d, 3), "side": c["side"],
                      "slant_m": round(c["s_a"], 2), "pitch_m": c["pitch"],
                      "passes": len(hits)})
            res["features"][fname][s.name] = m
            cols.append((s.name, c["px"]))
            cols_det.append((s.name, c["gray"]))
            pitch[(fname, s.name)] = c["pitch"]
        rows.append((fname, cols))
        rows_det.append((fname, cols_det))
    for s in sessions:
        res["sessions"][s.name] = {
            "root": str(s.root), "tiles": len(s.tiles),
            "frame_offset_m": [round(v, 3) for v in s.offset_xy],
            "texture_port": texture_metrics(s, "port", fg=fg),
            "texture_starboard": texture_metrics(s, "starboard", fg=fg),
            "row_banding": row_banding(s),
            "bottom_jitter": bottom_jitter(s)}
        print(s.name, json.dumps(res["sessions"][s.name]), flush=True)
    for fname in res["features"]:
        for sname, m in res["features"][fname].items():
            print(fname, sname, json.dumps(m), flush=True)
    (out / "metrics.json").write_text(json.dumps(res, indent=1))
    if rows:
        compose_panels(rows, out / "features.png", pitch)
        if fg is not None:
            compose_panels(rows_det, out / "features_detector_input.png", pitch)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
