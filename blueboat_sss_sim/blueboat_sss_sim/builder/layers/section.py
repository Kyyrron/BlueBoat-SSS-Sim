"""A small depth-profile plot along a drawn line (QPainter, no
matplotlib): what a channel template or a beach gradient actually cut."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QLabel

from .. import theme


class SectionView(QLabel):
    def __init__(self, parent=None) -> None:
        super().__init__("(section: drag a line on the map)", parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumHeight(110)
        self.setObjectName("hint")
        self._profile: tuple[np.ndarray, np.ndarray] | None = None

    def clear(self) -> None:
        self._profile = None
        self.setPixmap(QPixmap())
        self.setText("(section: drag a line on the map)")

    def set_profile(self, distance, depth) -> None:
        d = np.asarray(distance, dtype=float)
        z = np.asarray(depth, dtype=float)
        ok = np.isfinite(z)
        if ok.sum() < 2:
            self.clear()
            return
        self._profile = (d[ok], z[ok])
        self.setText("")
        self._redraw()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._profile is not None:
            self._redraw()

    def _redraw(self) -> None:
        d, z = self._profile
        w, h = max(self.width(), 200), max(self.height(), 100)
        img = QImage(w, h, QImage.Format.Format_RGB32)
        img.fill(QColor(theme.BG_PANEL))
        p = QPainter(img)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        m_l, m_r, m_t, m_b = 44, 10, 10, 22
        pw, ph = w - m_l - m_r, h - m_t - m_b
        zmin, zmax = float(z.min()), float(z.max())
        if zmax - zmin < 0.05:
            zmin, zmax = zmin - 0.1, zmax + 0.1
        span = zmax - zmin
        zlo, zhi = zmin - 0.05 * span, zmax + 0.05 * span
        dmax = max(float(d.max()), 1e-6)

        def X(v):
            return m_l + pw * v / dmax

        def Y(v):
            return m_t + ph * (v - zlo) / (zhi - zlo)      # depth grows down

        p.setPen(QPen(QColor(theme.C_GRID_MAJOR), 1))
        p.drawRect(m_l, m_t, pw, ph)
        p.setPen(QPen(QColor(theme.TEXT_DIM), 1))
        p.setFont(QFont("DejaVu Sans", 8))
        p.drawText(QPointF(2, m_t + 10), f"{zlo:.1f} m")
        p.drawText(QPointF(2, m_t + ph), f"{zhi:.1f} m")
        p.drawText(QPointF(m_l, h - 6), "0 m")
        p.drawText(QPointF(w - m_r - 40, h - 6), f"{dmax:.0f} m")
        p.setPen(QPen(QColor(theme.ACCENT), 2))
        pts = [QPointF(X(a), Y(b)) for a, b in zip(d, z)]
        for a, b in zip(pts[:-1], pts[1:]):
            p.drawLine(a, b)
        p.end()
        self.setPixmap(QPixmap.fromImage(img))
