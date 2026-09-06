"""Depth-sample ingestion: CSV soundings and GeoTIFF rasters.

Both readers produce a :class:`DepthSamples` -- scattered (east, north,
depth) points in the world's local ENU frame about its
:class:`~blueboat_sss_sim.core.geo.GeoAnchor`. Depth is metres, positive
down; interpolation onto the scene grid lives in :mod:`.interpolate`.

The GeoTIFF path deliberately avoids GDAL/rasterio (not installed):
Pillow reads the raster and exposes the GeoTIFF tags, and the two tags
that matter for a geographic-CRS export (ModelTiepoint 33922,
ModelPixelScale 33550) are a 30-line affine. Projected-CRS files (UTM
metres) are rejected with an actionable message -- converting them needs
a real projection library.
"""

from __future__ import annotations

import csv as _csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ...core.geo import GeoAnchor

# GeoTIFF tag ids (as exposed by PIL's tag_v2).
TAG_MODEL_PIXEL_SCALE = 33550
TAG_MODEL_TIEPOINT = 33922
TAG_GEO_KEY_DIRECTORY = 34735
_GT_MODEL_TYPE_GEO_KEY = 1024          # 1 = projected, 2 = geographic
#: Common raster nodata sentinels, masked alongside NaN.
_NODATA_SENTINELS = (-9999.0, -32767.0, 3.4028235e38, -3.4028235e38)
#: Cap on samples handed to the interpolator (Qhull cost); rasters are
#: strided down to stay under it.
MAX_SAMPLES = 500_000


@dataclass
class DepthSamples:
    """Scattered depth soundings in world EN metres (depth positive down)."""

    east: np.ndarray
    north: np.ndarray
    depth: np.ndarray

    def __post_init__(self) -> None:
        self.east = np.asarray(self.east, dtype=np.float64).ravel()
        self.north = np.asarray(self.north, dtype=np.float64).ravel()
        self.depth = np.asarray(self.depth, dtype=np.float64).ravel()
        if not (len(self.east) == len(self.north) == len(self.depth)):
            raise ValueError("east/north/depth lengths differ")

    def __len__(self) -> int:
        return len(self.depth)


def _resolve_values(depth: np.ndarray, values: str, source: str) -> np.ndarray:
    """Interpret raw vertical values as depth (positive down).

    ``values``: ``"depth"`` (positive down, used as-is), ``"elevation"``
    (negative below the surface, negated), or ``"auto"`` -- bathymetry
    sources split between the two conventions (GEBCO exports elevation,
    echo-sounder logs depth), so auto decides from the sign of the median.
    """
    if values == "depth":
        return depth
    if values == "elevation":
        return -depth
    if values != "auto":
        raise ValueError(
            f"depth values mode '{values}' unknown (auto | depth | elevation)")
    med = float(np.nanmedian(depth)) if len(depth) else 0.0
    return -depth if med < 0.0 else depth


# ------------------------------------------------------------------- CSV
_LAT_NAMES = {"lat", "latitude"}
_LON_NAMES = {"lon", "lng", "longitude"}
_DEPTH_NAMES = {"depth", "depth_m", "z", "elevation"}


def load_csv(path: str | Path, anchor: GeoAnchor,
             values: str = "auto") -> DepthSamples:
    """Read ``lat, lon, depth`` soundings (with or without a header row).

    A header is detected by the first row failing to parse as numbers; its
    column names are matched case-insensitively against lat/lon/depth
    synonyms, so any column order works. Headerless files are taken as
    ``lat, lon, depth`` in that order. Rows with non-finite values drop.
    """
    path = Path(path)
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        sample = f.read(4096)
        f.seek(0)
        try:
            dialect = _csv.Sniffer().sniff(sample, delimiters=",;\t ")
        except _csv.Error:
            dialect = _csv.excel
        rows = [r for r in _csv.reader(f, dialect) if any(c.strip() for c in r)]
    if not rows:
        raise ValueError(f"{path}: empty depth CSV")

    cols = (0, 1, 2)
    start = 0
    try:
        [float(c) for c in rows[0][:3]]
    except (ValueError, IndexError):
        header = [c.strip().lower() for c in rows[0]]
        start = 1

        def _find(names: set[str], what: str) -> int:
            for i, h in enumerate(header):
                if h in names:
                    return i
            raise ValueError(
                f"{path}: header {header} has no {what} column "
                f"(accepted names: {sorted(names)})")

        cols = (_find(_LAT_NAMES, "latitude"), _find(_LON_NAMES, "longitude"),
                _find(_DEPTH_NAMES, "depth"))
        if header[cols[2]] == "elevation" and values == "auto":
            values = "elevation"

    data = []
    for r in rows[start:]:
        try:
            data.append((float(r[cols[0]]), float(r[cols[1]]),
                         float(r[cols[2]])))
        except (ValueError, IndexError):
            continue
    if not data:
        raise ValueError(f"{path}: no numeric lat/lon/depth rows")
    arr = np.array(data, dtype=np.float64)
    ok = np.isfinite(arr).all(axis=1)
    arr = arr[ok]
    east, north = anchor.to_en(arr[:, 0], arr[:, 1])
    return DepthSamples(east, north,
                        _resolve_values(arr[:, 2], values, str(path)))


# --------------------------------------------------------------- GeoTIFF
def load_geotiff(path: str | Path, anchor: GeoAnchor,
                 values: str = "auto") -> DepthSamples:
    """Read a geographic-CRS GeoTIFF depth/elevation raster via Pillow."""
    from PIL import Image

    path = Path(path)
    with Image.open(path) as img:
        tags = dict(img.tag_v2) if hasattr(img, "tag_v2") else {}
        arr = np.asarray(img, dtype=np.float64)
    if arr.ndim == 3:
        arr = arr[..., 0]
    if arr.ndim != 2:
        raise ValueError(f"{path}: expected a 2-D raster, got shape {arr.shape}")

    scale = tags.get(TAG_MODEL_PIXEL_SCALE)
    tie = tags.get(TAG_MODEL_TIEPOINT)
    if scale is None or tie is None:
        raise ValueError(
            f"{path}: no ModelPixelScale/ModelTiepoint GeoTIFF tags -- "
            "export the raster as a georeferenced (geographic lat/lon) "
            "GeoTIFF, or convert it to a lat,lon,depth CSV.")
    keys = tags.get(TAG_GEO_KEY_DIRECTORY)
    if keys is not None:
        for k in range(4, len(keys) - 3, 4):
            if int(keys[k]) == _GT_MODEL_TYPE_GEO_KEY and int(keys[k + 3]) == 1:
                raise ValueError(
                    f"{path}: projected-CRS GeoTIFF (UTM or similar) is not "
                    "supported without a projection library -- re-export in "
                    "geographic lat/lon (EPSG:4326), or use a CSV.")
    sx, sy = float(scale[0]), float(scale[1])
    if abs(sx) > 1.0 or abs(sy) > 1.0:
        raise ValueError(
            f"{path}: pixel scale ({sx}, {sy}) is not in degrees -- this "
            "looks like a projected-CRS raster; re-export in geographic "
            "lat/lon (EPSG:4326), or use a CSV.")
    # Tiepoint: raster (i, j) -> geographic (lon=X, lat=Y); latitude
    # decreases with row for the (overwhelmingly standard) north-up export.
    ti, tj, _, tx, ty = (float(tie[0]), float(tie[1]), float(tie[2]),
                         float(tie[3]), float(tie[4]))

    ny, nx = arr.shape
    stride = max(1, int(np.ceil(np.sqrt(ny * nx / MAX_SAMPLES))))
    rows = np.arange(0, ny, stride)
    cols = np.arange(0, nx, stride)
    sub = arr[np.ix_(rows, cols)]
    lon = tx + (cols - ti) * sx
    lat = ty - (rows - tj) * sy
    lon_g, lat_g = np.meshgrid(lon, lat)

    depth = sub.ravel()
    ok = np.isfinite(depth)
    for s in _NODATA_SENTINELS:
        ok &= ~np.isclose(depth, s, rtol=1e-6, atol=0.0)
    if not ok.any():
        raise ValueError(f"{path}: raster holds no usable depth values")
    east, north = anchor.to_en(lat_g.ravel()[ok], lon_g.ravel()[ok])
    return DepthSamples(east, north,
                        _resolve_values(depth[ok], values, str(path)))
