"""Screen-size adaptation helpers shared by every wizard page.

The builder's screens are wide: the depth editor alone carries a layer
panel, a map, a tool bar of twenty-odd controls and two live previews.
Laid out in plain box layouts, the tool bar's *minimum* width became the
window's, so the window opened wider than the screen and could not be
shrunk (Qt refuses to resize a window below its layout minimum).

Three pieces fix that, and nothing here knows anything about worlds:

``FlowLayout``
    A wrapping row: items flow onto the next line when the width runs
    out, so a tool bar's minimum width is its *widest single control*
    rather than the sum of all of them.
``ScaledImage``
    A ``QLabel`` that keeps its source pixmap and rescales it on every
    resize, so previews follow the window instead of freezing at the
    size they happened to have when the worker finished.
``fit_to_screen``
    Clamps a window to the available screen area and centres it.

``split`` builds the horizontal ``QSplitter`` the map screens use, so a
panel can be dragged narrow (or shut) on a small display.

``WrapScroll`` holds a property column whose word-wrapped labels need
their *height for the column's width* -- what ``QScrollArea
(widgetResizable=True)`` never asks a ``QStackedWidget`` for.
"""

from __future__ import annotations

from PySide6.QtCore import QEvent, QPoint, QRect, QSize, Qt, Signal
from PySide6.QtGui import QGuiApplication, QImage, QPixmap
from PySide6.QtWidgets import (QFrame, QLabel, QLayout, QScrollArea,
                               QSizePolicy, QSplitter, QStackedWidget,
                               QWidget)

#: Ideal window size; ``fit_to_screen`` shrinks it to what the screen has.
DEFAULT_WINDOW = (1280, 820)


class FlowLayout(QLayout):
    """Left-to-right layout that wraps onto the next line.

    Qt ships this as an example rather than a class; this is that
    algorithm with the project's spacing and a ``minimumSize`` of the
    widest item, which is the whole point -- it is what lets the window
    shrink.
    """

    def __init__(self, parent: QWidget | None = None, margin: int = 0,
                 h_spacing: int = 6, v_spacing: int = 6) -> None:
        super().__init__(parent)
        self._items: list = []
        self._h = h_spacing
        self._v = v_spacing
        self.setContentsMargins(margin, margin, margin, margin)

    # -- QLayout plumbing ------------------------------------------------
    def addItem(self, item) -> None:          # noqa: N802 (Qt override)
        self._items.append(item)

    def count(self) -> int:
        return len(self._items)

    def itemAt(self, index: int):             # noqa: N802
        if 0 <= index < len(self._items):
            return self._items[index]
        return None

    def takeAt(self, index: int):             # noqa: N802
        if 0 <= index < len(self._items):
            return self._items.pop(index)
        return None

    def expandingDirections(self):            # noqa: N802
        return Qt.Orientation(0)

    def hasHeightForWidth(self) -> bool:      # noqa: N802
        return True

    def heightForWidth(self, width: int) -> int:   # noqa: N802
        return self._do_layout(QRect(0, 0, width, 0), test_only=True)

    def setGeometry(self, rect: QRect) -> None:    # noqa: N802
        super().setGeometry(rect)
        self._do_layout(rect, test_only=False)

    def sizeHint(self) -> QSize:              # noqa: N802
        return self.minimumSize()

    def minimumSize(self) -> QSize:           # noqa: N802
        size = QSize()
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        m = self.contentsMargins()
        return size + QSize(m.left() + m.right(), m.top() + m.bottom())

    # -- the algorithm ---------------------------------------------------
    def _do_layout(self, rect: QRect, test_only: bool) -> int:
        m = self.contentsMargins()
        eff = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom())
        x, y, line_h = eff.x(), eff.y(), 0
        for item in self._items:
            hint = item.sizeHint()
            nx = x + hint.width()
            if nx - 1 > eff.right() and line_h > 0:
                x = eff.x()
                y += line_h + self._v
                nx = x + hint.width()
                line_h = 0
            if not test_only:
                item.setGeometry(QRect(QPoint(x, y), hint))
            x = nx + self._h
            line_h = max(line_h, hint.height())
        return y + line_h - rect.y() + m.bottom()


class ToolBar(QScrollArea):
    """A tool row that wraps instead of widening the window.

    ``add`` takes widgets, ``label`` a caption, ``separator`` a thin
    divider between groups. The old ``addStretch``/``addSpacing`` of a
    box layout have no meaning in a wrapping row: groups are separated
    by ``separator()``, which reads better anyway.

    The bar grows to as many wrapped rows as it needs, up to
    ``MAX_ROWS``, and scrolls past that -- so a very narrow window costs
    the page a bounded strip rather than half its height. Its minimum is
    one row by the widest single control, which is what keeps the window
    freely resizable.
    """

    MAX_ROWS = 3

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self._inner = QWidget()
        self._flow = FlowLayout(self._inner)
        self.setWidget(self._inner)
        self.setSizePolicy(QSizePolicy.Policy.Preferred,
                           QSizePolicy.Policy.Fixed)

    # -- content ---------------------------------------------------------
    def add(self, *widgets: QWidget) -> None:
        for w in widgets:
            self._flow.addWidget(w)
        self._apply_height()

    def label(self, text: str) -> QLabel:
        lbl = QLabel(text)
        self._flow.addWidget(lbl)
        self._apply_height()
        return lbl

    def separator(self) -> None:
        line = QFrame()
        line.setFrameShape(QFrame.Shape.VLine)
        line.setFrameShadow(QFrame.Shadow.Plain)
        line.setFixedWidth(10)
        self._flow.addWidget(line)
        self._apply_height()

    # -- geometry --------------------------------------------------------
    def _row_height(self) -> int:
        return max(self._flow.minimumSize().height(), 24)

    def _wanted_height(self, width: int) -> int:
        row = self._row_height()
        cap = row * self.MAX_ROWS + 6 * (self.MAX_ROWS - 1)
        return max(row, min(self._flow.heightForWidth(max(width, 60)), cap))

    def _apply_height(self) -> None:
        w = self.viewport().width() or self.width()
        h = self._wanted_height(w)
        if h != self.height() or self.minimumHeight() != h:
            self.setFixedHeight(h)

    def minimumSizeHint(self) -> QSize:       # noqa: N802
        return QSize(self._flow.minimumSize().width() + 4, self._row_height())

    def sizeHint(self) -> QSize:              # noqa: N802
        w = self.viewport().width() or self.width()
        return QSize(self._flow.minimumSize().width() + 4,
                     self._wanted_height(w))

    def resizeEvent(self, event) -> None:     # noqa: N802
        super().resizeEvent(event)
        self._apply_height()

    def showEvent(self, event) -> None:       # noqa: N802
        super().showEvent(event)
        self._apply_height()


class ScaledImage(QLabel):
    """Preview label that rescales its pixmap with the window.

    ``set_image`` keeps the full-resolution source; every resize redraws
    from it, so a preview rendered while the window was small stays
    sharp when it is enlarged.
    """

    #: The cursor is over the drawn picture: normalised (u, v), u to the
    #: right and v *upward* from the bottom edge (a north-up raster's
    #: south edge), both in [0, 1); ``hover_left`` when it leaves.
    hover = Signal(float, float)
    hover_left = Signal()

    def __init__(self, placeholder: str = "", min_w: int = 160,
                 min_h: int = 110, parent: QWidget | None = None) -> None:
        super().__init__(placeholder, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(min_w, min_h)
        self.setScaledContents(False)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)
        self.setMouseTracking(True)
        self._placeholder = placeholder
        self._src: QPixmap | None = None

    def picture_rect(self) -> QRect | None:
        """Where the (aspect-preserved, centred) picture is drawn inside
        the label, in label pixels; ``None`` without a picture."""
        pm = self.pixmap()
        if self._src is None or pm is None or pm.isNull():
            return None
        r = self.contentsRect()
        x = r.x() + (r.width() - pm.width()) // 2
        y = r.y() + (r.height() - pm.height()) // 2
        return QRect(x, y, pm.width(), pm.height())

    def picture_uv(self, x: float, y: float) -> tuple[float, float] | None:
        """Label pixel -> normalised picture coordinate (u right, v up),
        or ``None`` outside the drawn picture."""
        r = self.picture_rect()
        if r is None or r.width() <= 0 or r.height() <= 0:
            return None
        u = (x - r.x()) / r.width()
        v = 1.0 - (y - r.y()) / r.height()
        if not (0.0 <= u < 1.0 and 0.0 < v <= 1.0):
            return None
        return u, min(v, 1.0 - 1e-9)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        uv = self.picture_uv(event.position().x(), event.position().y())
        if uv is None:
            self.hover_left.emit()
        else:
            self.hover.emit(*uv)
        super().mouseMoveEvent(event)

    def leaveEvent(self, event) -> None:      # noqa: N802
        self.hover_left.emit()
        super().leaveEvent(event)

    def set_image(self, image) -> None:
        if isinstance(image, QImage):
            self._src = QPixmap.fromImage(image)
        elif isinstance(image, QPixmap):
            self._src = image
        else:                                   # a path
            self._src = QPixmap(str(image))
        self.setText("")
        self._rescale()

    def clear_image(self, text: str | None = None) -> None:
        self._src = None
        self.setPixmap(QPixmap())
        self.setText(self._placeholder if text is None else text)

    def resizeEvent(self, event) -> None:     # noqa: N802
        super().resizeEvent(event)
        self._rescale()

    def showEvent(self, event) -> None:       # noqa: N802
        super().showEvent(event)
        self._rescale()

    def _rescale(self) -> None:
        if self._src is None or self._src.isNull():
            return
        self.setPixmap(self._src.scaled(
            self.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))


class PageScroll(QScrollArea):
    """Holder that lets a page fill the window and scroll when it cannot.

    ``setWidgetResizable(True)`` is deliberately *not* used: it sizes the
    page by ``heightForWidth``, which any word-wrapped label turns into
    "the page's ideal height", so the window would show a scroll bar
    rather than compressing the map. Here the page is given the viewport
    and never less than its own minimum -- it compresses first and
    scrolls only past that point.
    """

    def __init__(self, page: QWidget, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setWidgetResizable(False)
        self.setWidget(page)
        page.show()

    def resizeEvent(self, event) -> None:     # noqa: N802
        super().resizeEvent(event)
        self._fit()

    def showEvent(self, event) -> None:       # noqa: N802
        super().showEvent(event)
        self._fit()

    def _fit(self) -> None:
        page = self.widget()
        if page is None:
            return
        vp = self.viewport().size()
        floor = page.minimumSizeHint().expandedTo(page.minimumSize())
        page.resize(max(vp.width(), floor.width()),
                    max(vp.height(), floor.height()))

    def minimumSizeHint(self) -> QSize:       # noqa: N802
        return QSize(120, 120)


class WrapScroll(QScrollArea):
    """A vertical holder whose page takes the viewport's width and exactly
    the height it needs *at that width*.

    ``setWidgetResizable(True)`` sizes the page from ``sizeHint()``; a
    ``QStackedWidget`` (or any widget whose layout does not forward
    ``heightForWidth``) then reports the *single-line* height of a
    word-wrapped label, and the label is drawn in a box too short for its
    wrapped text -- the Screen-3 recording-quality line (2026-09-16). Here
    the page is resized by hand on every viewport resize, page change and
    layout request: width = viewport (never below the page's minimum),
    height = the current page's ``heightForWidth`` at that width, never
    below its minimum; the vertical bar appears only past that.
    """

    def __init__(self, page: QWidget, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setWidgetResizable(False)
        # The page is never narrower than its own minimum; when the column
        # is dragged below that, a horizontal bar appears rather than the
        # right-hand edge being clipped.
        self.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setWidget(page)
        page.installEventFilter(self)
        if isinstance(page, QStackedWidget):
            page.currentChanged.connect(lambda _i: self.refit())
        page.show()

    @staticmethod
    def _content(page: QWidget) -> QWidget:
        if isinstance(page, QStackedWidget) and page.currentWidget() is not None:
            return page.currentWidget()
        return page

    def wanted_height(self, width: int) -> int:
        """The height the current page needs at ``width``."""
        target = self._content(self.widget())
        lay = target.layout()
        h = target.minimumSizeHint().height()
        if lay is not None and lay.hasHeightForWidth():
            h = max(h, lay.heightForWidth(max(width, 1)))
        else:
            h = max(h, target.sizeHint().height())
        return h

    def refit(self) -> None:
        page = self.widget()
        if page is None:
            return
        w = max(self.viewport().width(), page.minimumSizeHint().width(), 1)
        page.resize(w, self.wanted_height(w))

    def eventFilter(self, obj, event) -> bool:   # noqa: N802
        if obj is self.widget() and event.type() == QEvent.Type.LayoutRequest:
            self.refit()
        return super().eventFilter(obj, event)

    def resizeEvent(self, event) -> None:     # noqa: N802
        super().resizeEvent(event)
        self.refit()

    def showEvent(self, event) -> None:       # noqa: N802
        super().showEvent(event)
        self.refit()

    def minimumSizeHint(self) -> QSize:       # noqa: N802
        page = self.widget()
        w = page.minimumSizeHint().width() if page is not None else 60
        return QSize(w + self.verticalScrollBar().sizeHint().width(), 120)


def split(*panes, stretch: tuple[int, ...] = ()) -> QSplitter:
    """A horizontal splitter over the given widgets.

    Panels become draggable (and collapsible) instead of holding a fixed
    share of a window that may be far narrower than the layout wants.
    """
    sp = QSplitter(Qt.Orientation.Horizontal)
    sp.setChildrenCollapsible(True)
    sp.setHandleWidth(6)
    for pane in panes:
        sp.addWidget(pane)
    for i, factor in enumerate(stretch):
        sp.setStretchFactor(i, factor)
    return sp


def fit_to_screen(win: QWidget, width: int = DEFAULT_WINDOW[0],
                  height: int = DEFAULT_WINDOW[1], frac: float = 0.94) -> None:
    """Resize ``win`` to at most ``frac`` of the available screen, centred.

    ``availableGeometry`` excludes panels and task bars, so the window
    never opens under one.
    """
    screen = win.screen() or QGuiApplication.primaryScreen()
    if screen is None:                          # offscreen/headless
        win.resize(width, height)
        return
    area = screen.availableGeometry()
    w = max(480, min(width, int(area.width() * frac)))
    h = max(360, min(height, int(area.height() * frac)))
    win.resize(w, h)
    win.move(area.x() + (area.width() - w) // 2,
             area.y() + (area.height() - h) // 2)
