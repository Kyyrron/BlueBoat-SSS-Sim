"""Raster -> ``QImage`` for the builder's previews (the only place the
GUI converts a numpy array into a picture)."""

from __future__ import annotations

import numpy as np
from PySide6.QtGui import QImage


def raster_to_qimage(arr: np.ndarray, lo: float | None = None,
                     hi: float | None = None, invert: bool = False,
                     north_up: bool = True) -> QImage:
    """Greyscale 8-bit image of a float raster.

    ``lo``/``hi`` fix the stretch (default: the finite min/max);
    ``invert`` maps ``lo`` to white (the "white = shallow" depth
    convention); ``north_up`` flips a ``(ny, nx)`` scene raster (row 0 =
    min y) into image convention (row 0 = max y). Non-finite cells render
    black. The returned image owns its pixels.
    """
    a = np.asarray(arr, dtype=np.float64)
    finite = np.isfinite(a)
    if not finite.any():
        a = np.zeros_like(a)
        finite = np.ones_like(a, dtype=bool)
    lo = float(a[finite].min()) if lo is None else float(lo)
    hi = float(a[finite].max()) if hi is None else float(hi)
    u = np.clip((np.where(finite, a, lo) - lo) / max(hi - lo, 1e-9), 0.0, 1.0)
    if invert:
        u = 1.0 - u
    u = np.where(finite, u, 0.0)
    u8 = (u * 255.0).astype(np.uint8)
    if north_up:
        u8 = np.flipud(u8)
    u8 = np.ascontiguousarray(u8)
    h, w = u8.shape
    return QImage(u8.data, w, h, w, QImage.Format.Format_Grayscale8).copy()


def sonar_shading(height: np.ndarray, cell_m: float,
                  grazing_deg: float = 20.0,
                  heading_deg: float = 90.0) -> np.ndarray:
    """What the sonar makes of a relief field: cos(incidence) in [0, 1].

    The same quantity the renderer shades with (``cos_inc`` from the local
    slope against a ray at ``grazing_deg`` above horizontal, arriving from
    the compass direction ``heading_deg``), reduced to a picture. A ripple
    field is invisible in a plain depth stretch -- 6 cm of relief against
    metres of bathymetry -- and unmistakable here, which is the whole
    difference between authoring bedforms blind and seeing them.

    Qt-free on purpose: the offline suite gates it, and the GUI only turns
    the result into pixels.
    """
    h = np.asarray(height, dtype=np.float64)
    gy, gx = np.gradient(h, max(float(cell_m), 1e-6))
    th = np.radians(90.0 - float(heading_deg))          # compass -> ENU
    g = np.radians(float(grazing_deg))
    # Ray direction: horizontal component along the look bearing, dropping
    # at the grazing angle. Surface normal from the two slopes.
    lx, ly, lz = np.cos(th) * np.cos(g), np.sin(th) * np.cos(g), -np.sin(g)
    nx, ny, nz = -gx, -gy, np.ones_like(h)
    n = np.sqrt(nx * nx + ny * ny + nz * nz)
    cos_inc = -(lx * nx + ly * ny + lz * nz) / np.maximum(n, 1e-12)
    return np.clip(cos_inc, 0.0, 1.0)
