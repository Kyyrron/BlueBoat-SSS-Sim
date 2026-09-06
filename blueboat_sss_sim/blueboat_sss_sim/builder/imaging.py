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
