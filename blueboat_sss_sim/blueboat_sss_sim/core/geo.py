"""Geodesy for GPS-anchored worlds.

The one conversion convention used across the whole BlueBoat project:
spherical equirectangular about an anchor latitude, WGS-84 equatorial
radius, longitude scaled by cos(lat0) -- the *origin* latitude, never the
point latitude. This file is deliberately the same math as
``BlueBoat-MCS/mcs/core/geo.py`` and the GCS's ``mapping/geo.py`` (which
is a verbatim port of it), so a value converted here matches what MCS
deploys and what the GCS maps, bit for bit. ``mavros_shim_node`` inlines
the same inverse; the smoke test asserts the two agree exactly.

Accuracy degrades beyond ~10 km from the anchor, which is far outside any
survey world this project generates.

All functions accept scalars or NumPy arrays elementwise.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

EARTH_RADIUS_M = 6378137.0  # WGS-84 equatorial radius, used as a sphere


# --------------------------------------------------------------- conversions
def latlon_to_local_en(lat, lon, lat0: float, lon0: float):
    """(lat, lon) -> (east, north) metres about the anchor (lat0, lon0)."""
    east = np.radians(np.asarray(lon) - lon0) * EARTH_RADIUS_M * np.cos(
        np.radians(lat0))
    north = np.radians(np.asarray(lat) - lat0) * EARTH_RADIUS_M
    return east, north


def local_en_to_latlon(east, north, lat0: float, lon0: float):
    """(east, north) metres about the anchor -> (lat, lon). Exact inverse
    of :func:`latlon_to_local_en`."""
    lat = lat0 + np.degrees(np.asarray(north) / EARTH_RADIUS_M)
    lon = lon0 + np.degrees(
        np.asarray(east) / (EARTH_RADIUS_M * np.cos(np.radians(lat0))))
    return lat, lon


# ------------------------------------------------------------------- anchor
@dataclass(frozen=True)
class GeoAnchor:
    """Geographic position of the world's ENU origin (world (0, 0)).

    For a world built from an MCS anchored path this is the path's own
    ``geo_anchor`` -- the GPS of the design frame's (0, 0) -- which makes
    the world frame identical to the path's design frame: path waypoints
    land in world coordinates with no transform.

    ``theta_deg`` exists only for schema parity with the MCS trajectory
    format; every anchor this pipeline produces or accepts is 0 (the MCS
    designer has written 0 since the design frame became local ENU).
    """

    lat0: float
    lon0: float
    theta_deg: float = 0.0

    def __post_init__(self) -> None:
        if abs(self.theta_deg) > 1e-9:
            raise ValueError(
                f"geo anchor theta_deg={self.theta_deg} is unsupported: "
                "the world frame is defined as the anchor's local ENU frame "
                "(theta 0). Re-save the source path with the current MCS "
                "designer, which always writes theta_deg: 0.")

    def to_latlon(self, east, north):
        """World (east, north) -> (lat, lon)."""
        return local_en_to_latlon(east, north, self.lat0, self.lon0)

    def to_en(self, lat, lon):
        """(lat, lon) -> world (east, north)."""
        return latlon_to_local_en(lat, lon, self.lat0, self.lon0)

    def to_dict(self) -> dict[str, float]:
        return {"lat0": float(self.lat0), "lon0": float(self.lon0),
                "theta_deg": float(self.theta_deg)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "GeoAnchor | None":
        """Parse a ``{lat0, lon0[, theta_deg]}`` mapping; None passes through
        (a world without an anchor is legal -- legacy bundles)."""
        if d is None:
            return None
        if "lat0" not in d or "lon0" not in d:
            raise KeyError(f"geo anchor needs lat0 and lon0, got {sorted(d)}")
        return cls(lat0=float(d["lat0"]), lon0=float(d["lon0"]),
                   theta_deg=float(d.get("theta_deg", 0.0)))


# ------------------------------------------------------ reporting distances
#: The 16-point compass, clockwise from north.
COMPASS_POINTS = ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW")


def range_bearing(lat0: float, lon0: float, lat1: float, lon1: float
                  ) -> tuple[float, float, str]:
    """Great-circle distance (m), initial bearing (deg clockwise from
    north) and its 16-point compass name, from (lat0, lon0) to (lat1,
    lon1).

    A **reporting** helper only -- it says how far apart two sites are so
    an error message can name the gap. World coordinates are never
    produced with it: those go through :func:`latlon_to_local_en` and its
    inverse, which stay the one conversion convention. Haversine rather
    than the equirectangular pair because the pairs this reports on are
    exactly the ones far outside the anchor's valid band.
    """
    p0, p1 = np.radians(float(lat0)), np.radians(float(lat1))
    dphi = p1 - p0
    dlam = np.radians(float(lon1) - float(lon0))
    a = (np.sin(dphi / 2.0) ** 2
         + np.cos(p0) * np.cos(p1) * np.sin(dlam / 2.0) ** 2)
    dist = 2.0 * EARTH_RADIUS_M * np.arcsin(np.sqrt(min(1.0, float(a))))
    y = np.sin(dlam) * np.cos(p1)
    x = np.cos(p0) * np.sin(p1) - np.sin(p0) * np.cos(p1) * np.cos(dlam)
    brg = float(np.degrees(np.arctan2(y, x))) % 360.0
    name = COMPASS_POINTS[int(round(brg / 22.5)) % 16]
    return float(dist), brg, name


def format_distance(metres: float) -> str:
    """A distance the way an operator reads it: metres below 1 km, then
    kilometres."""
    m = float(metres)
    if m < 1000.0:
        return f"{m:.0f} m"
    if m < 10000.0:
        return f"{m / 1000.0:.1f} km"
    return f"{m / 1000.0:.0f} km"


def format_latlon(lat: float, lon: float) -> str:
    """``33.930840 N, 130.731300 E`` -- the form the metadata uses."""
    return (f"{abs(float(lat)):.6f} {'N' if lat >= 0 else 'S'}, "
            f"{abs(float(lon)):.6f} {'E' if lon >= 0 else 'W'}")


# ------------------------------------------------- slippy-map tile helpers
# Standard Web-Mercator tile math (OSM/Esri "slippy" scheme), used by the
# world-builder map widget. Same convention as the MCS map and GCS tiles.
def latlon_to_tile_xy(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Fractional tile coordinates of (lat, lon) at ``zoom``."""
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - np.arcsinh(np.tan(np.radians(lat))) / np.pi) / 2.0 * n
    return float(x), float(y)


def tile_xy_to_latlon(x: float, y: float, zoom: int) -> tuple[float, float]:
    """(lat, lon) of the NW corner of tile (x, y) at ``zoom``."""
    n = 2.0 ** zoom
    lon = x / n * 360.0 - 180.0
    lat = np.degrees(np.arctan(np.sinh(np.pi * (1.0 - 2.0 * y / n))))
    return float(lat), float(lon)


def metres_per_pixel(lat: float, zoom: int) -> float:
    """Ground resolution of a 256 px Web-Mercator tile at ``lat``."""
    return 156543.03392 * float(np.cos(np.radians(lat))) / (2.0 ** zoom)
