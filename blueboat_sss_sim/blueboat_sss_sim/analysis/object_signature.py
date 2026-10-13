"""An object's signature in a recording, pass by pass: real and simulated
sessions reduced by one statistic (NC #17, 2026-09-28).

Reads the GCS's seabed-image grids (``<session>/seabed_images/metadata/
seabed_NNNNN_world.npz`` + ``.json``: the device's own dB per pixel, native
slant bins, with world coordinates in that session's GCS frame -- local
east/north metres about its first GPS fix). A file-level artefact of the
GCS, read and never written (CM-3). For every image whose swath passes
within ``--radius`` of the point, the object is found as the darkest
connected region there (its footprint and shadow, smoothed < -4 dB under the
same range in rows away from it), and reported as linear means against those
rows at the same ranges:

* ``near``    the 0.4 m in front of the dark region -- the highlight;
* ``rowmax``  the median per-row maximum over that window, against the same
              statistic away from the object (a speckle-fair peak);
* ``foot``    the first 0.4 m of the dark region (the footprint, roughly);
* ``shadow``  the rest of it;
* ``length``  the dark region's slant extent, and ``grazing`` at its near
              edge from the row's own altitude (the boat's, not the
              object's: the seabed slopes).

Measured 2026-09-28 with this reduction on the NAFCO box, ``beach_sept/
seabed_images_object_boiteVsDistance`` at (-15.9, 17.7): images 21-23
(5.9 m, 13.5 deg) near +2.2 / +2.1 / -0.2, row-max +2.9 / +1.1 / -0.3,
shadow -6.4 / -6.2 / -6.0 dB; images 32-34 (10.1 m, 9.5 deg) near
+0.7 / +0.3 / +1.4, row-max +2.3 / +1.5 / +1.6, shadow -4.5 / -5.7 /
-6.4 dB (the other passes do not show the box: past ~11 m it sinks into the
bedforms). The simulated sessions of the same day, in worlds built before
the facet layer: near +14.2 / +14.4 (2026_09_28-10_40_17, images 3-4), and
+21 to +29 for the box that rested on its end (2026_09_28-10_27_47). The
region is found automatically, so a pass through bedform troughs can pick
one: read the passes whose length and rows match the object.

    python3 -m blueboat_sss_sim.analysis.object_signature \\
        <session>/seabed_images --at X Y [--radius 2.5] [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import sys
import warnings
from pathlib import Path

import numpy as np
from scipy import ndimage


def _images(folder: Path) -> list[Path]:
    meta = folder / "metadata" if (folder / "metadata").is_dir() else folder
    return sorted(meta.glob("seabed_*_world.npz"))


def _lin(a: np.ndarray) -> float:
    return float(np.mean(10.0 ** (np.asarray(a, dtype=np.float64) / 10.0)))


def pass_signature(npz: Path, x: float, y: float, radius: float = 2.5
                   ) -> dict | None:
    """The object's signature in one seabed image, or ``None`` when the
    image does not reach the point or shows nothing dark there."""
    z = np.load(npz)
    meta = json.loads(npz.with_name(npz.name.replace("_world.npz", ".json"))
                      .read_text())
    X, Y = z["world_x"], z["world_y"]
    d = np.hypot(X - x, Y - y)
    if not np.isfinite(d).any() or float(np.nanmin(d)) > radius:
        return None
    d = np.where(np.isfinite(d), d, np.inf)
    W = X.shape[1]
    r0, c0 = np.unravel_index(np.nanargmin(d), d.shape)
    port = c0 < W // 2
    pick = ((lambda a: a[:, :W // 2][:, ::-1]) if port
            else (lambda a: a[:, W // 2:]))
    A = pick(z["intensity_db"].astype(np.float64))
    D = pick(d)
    bp = float(meta["bin_pitch_m"])
    # Find the dark region against the median of ALL rows (the object holds
    # a small share of them), then measure against the rows just outside it.
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        ref0 = np.nanmedian(A, axis=0)
    sm = ndimage.uniform_filter(np.nan_to_num(A - ref0),
                                size=(max(3, int(0.15 / bp)),
                                      max(3, int(0.3 / bp))))
    # The region must START within `radius` of the point (the GPS error),
    # but may run a shadow's length beyond it; one that reaches `radius +
    # reach` is background structure (a bedform trough, a shore).
    reach = 3.5
    lab, nlab = ndimage.label((D <= radius + reach) & (sm < -4.0))
    if nlab == 0:
        return None
    near_pt = D <= radius
    edge = D > radius + reach - 0.3
    best, best_size = 0, 0.0
    for j, sl in enumerate(ndimage.find_objects(lab), start=1):
        comp = lab[sl] == j
        size = float(comp.sum()) * bp * bp
        if (size < 0.05 or not (near_pt[sl] & comp).any()
                or (edge[sl] & comp).any()):
            continue
        if size > best_size:
            best, best_size = j, size
    if best == 0:
        return None
    rr, cc = np.nonzero(lab == best)
    rlo, rhi = int(rr.min()), int(rr.max()) + 1
    lo, hi = int(cc.min()), int(cc.max()) + 1
    pad = max(int(round(0.5 / bp)), 1)
    rows_far = np.array([q for q in range(A.shape[0])
                         if q < rlo - pad or q >= rhi + pad])
    if rows_far.size < 20:
        return None
    rows = np.arange(rlo, rhi)
    k = lambda m: max(int(round(m / bp)), 1)         # noqa: E731
    n0, n1 = max(lo - k(0.4), 0), lo + k(0.05)
    f1 = min(lo + k(0.4), hi)
    near = 10.0 * np.log10(_lin(A[rows, n0:n1]) / _lin(A[rows_far, n0:n1]))
    rowmax = float(np.median(A[rows, n0:n1].max(axis=1))
                   - np.median(A[rows_far, n0:n1].max(axis=1)))
    foot = 10.0 * np.log10(_lin(A[rows, lo:f1]) / _lin(A[rows_far, lo:f1]))
    shadow = (10.0 * np.log10(_lin(A[rows, f1:hi]) / _lin(A[rows_far, f1:hi]))
              if hi > f1 + 2 else float("nan"))
    alt = float(np.median([meta["rows_data"][int(q)].get("bottom_m", 0.0)
                           for q in rows]))
    slant = (lo + 0.5) * bp
    return {"image": npz.name.replace("_world.npz", ""),
            "side": "port" if port else "starboard",
            "rows": [rlo, rhi], "along_m": (rhi - rlo) * bp,
            "slant_m": slant, "length_m": (hi - lo) * bp,
            "grazing_deg": float(np.degrees(np.arcsin(min(alt / slant, 1.0))))
            if alt > 0.0 else float("nan"),
            "near_db": near, "rowmax_db": rowmax, "foot_db": foot,
            "shadow_db": shadow, "bin_pitch_m": bp}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("folder", type=Path,
                    help="a GCS seabed_images folder (or its metadata/)")
    ap.add_argument("--at", nargs=2, type=float, required=True,
                    metavar=("X", "Y"),
                    help="the object, in that session's GCS frame [m]")
    ap.add_argument("--radius", type=float, default=2.5,
                    help="search radius about the point [m] (GPS error)")
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    files = _images(a.folder)
    if not files:
        print(f"no seabed_*_world.npz under {a.folder}", file=sys.stderr)
        return 2
    rows = [s for f in files
            if (s := pass_signature(f, a.at[0], a.at[1], a.radius))]
    print(f"{len(rows)} pass(es) over ({a.at[0]:.1f}, {a.at[1]:.1f}) in "
          f"{a.folder}")
    for s in rows:
        print(f"  {s['image']} {s['side']:9s} slant {s['slant_m']:5.2f} m  "
              f"grazing {s['grazing_deg']:4.1f}  near {s['near_db']:+5.1f}  "
              f"row-max {s['rowmax_db']:+5.1f}  foot {s['foot_db']:+5.1f}  "
              f"shadow {s['shadow_db']:+5.1f} dB  length {s['length_m']:.2f} m")
    if a.json is not None:
        a.json.write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
