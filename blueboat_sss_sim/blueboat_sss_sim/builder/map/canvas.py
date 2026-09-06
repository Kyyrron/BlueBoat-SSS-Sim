"""The world-builder map: ENU north-up scene, satellite tiles, drawing.

Adapted from the MCS Pattern Designer's ``designer_map.py`` (copied,
never imported -- CM-3): the same y-flipped ``QGraphicsView`` over a
metric ENU scene, the same wheel-zoom / scrollbar-panning behaviour
(transform-translate panning compounded with AnchorUnderMouse -- a real
bug the MCS fixed), the same adaptive 1/2/5 grid and scale bar, and the
same avoid-``fitInView`` framing (it silently undoes the y-flip). The
waypoint machinery is replaced by the builder's two drawing tools:

* ``Mode.RECT`` -- drag one axis-aligned rectangle: the world limits;
* ``Mode.WALL`` -- click two points per reflective wall segment;
* ``Mode.DEPTH_BRUSH`` / ``Mode.MATERIAL_BRUSH`` / ``Mode.ERASER`` --
  Screen 3's paint tools: drag a stroke (``worldgen.depth.paint`` stroke
  dicts, kept in design-frame metres like the walls; Ctrl+wheel sets the
  brush radius).

Rectangle and walls stay editable in ``Mode.SELECT`` via constant-pixel
corner/endpoint handles (Ctrl snaps to a 1 m grid while dragging), unless
``set_geometry_editable(False)`` makes them read-only context (Screen 3).
The scene is the world/design frame: (0, 0) is the GPS anchor, +x east,
+y north.
"""

from __future__ import annotations

import math
from enum import Enum, auto

import numpy as np
from PySide6.QtCore import QLineF, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (QBrush, QColor, QFont, QGuiApplication, QPainter,
                           QPainterPath, QPen, QPolygonF, QTransform)
from PySide6.QtWidgets import (QGraphicsEllipseItem, QGraphicsItem,
                               QGraphicsLineItem, QGraphicsPathItem,
                               QGraphicsPixmapItem, QGraphicsRectItem,
                               QGraphicsScene, QGraphicsView)

from ...core.geo import GeoAnchor, latlon_to_tile_xy, tile_xy_to_latlon
from ...core.geometry import WALL_SNAP_M, quad_bezier_polyline
from .. import theme
from .tiles import (MAX_ZOOM, MIN_ZOOM, TILE_SIZE_PX, TileFetcher,
                    zoom_for_resolution)

GRID_SNAP_M = 1.0
MAX_TILES_PER_UPDATE = 96
BRUSH_RADIUS_MIN_M = 0.25
BRUSH_RADIUS_MAX_M = 50.0
#: Depth-brush colour ramp spans this depth range (light = shallow).
BRUSH_DEPTH_RAMP_M = 12.0
STROKE_KINDS = ("depth", "material")


def _cosmetic_pen(color, width: float) -> QPen:
    pen = QPen(QColor(color), width)
    pen.setCosmetic(True)
    return pen


class Mode(Enum):
    SELECT = auto()
    RECT = auto()
    WALL = auto()
    DEPTH_BRUSH = auto()
    MATERIAL_BRUSH = auto()
    ERASER = auto()


BRUSH_MODES = (Mode.DEPTH_BRUSH, Mode.MATERIAL_BRUSH, Mode.ERASER)


def _material_color(name: str) -> QColor:
    """The material's SDF visual colour (``worldgen.materials``), so the
    painted patch matches the Gazebo seabed tint."""
    from ...worldgen.materials import MaterialLibrary
    try:
        r, g, b = MaterialLibrary().get(name).color
    except KeyError:
        r, g, b = (0.6, 0.55, 0.4)
    return QColor(int(255 * r), int(255 * g), int(255 * b), 150)


def _depth_color(depth_m: float) -> QColor:
    """Light-to-dark blue ramp over ``BRUSH_DEPTH_RAMP_M``; a sand tone
    for a depth at or below zero (dry land above the waterline)."""
    if float(depth_m) <= 0.0:
        return QColor(222, 200, 150, 150)
    u = min(max(float(depth_m) / BRUSH_DEPTH_RAMP_M, 0.0), 1.0)
    return QColor(int(150 - 130 * u), int(215 - 180 * u), int(255 - 120 * u),
                  130)


def stroke_color(stroke: dict, kind: str) -> QColor:
    if stroke.get("erase"):
        return QColor(225, 225, 225, 90)
    if kind == "material":
        return _material_color(str(stroke.get("value", "")))
    return _depth_color(float(stroke.get("value", 0.0)))


# ------------------------------------------------------------------ layers
class TileLayer:
    """Satellite tiles placed in the EN scene about the GPS anchor.

    Each tile's NW corner lat/lon converts through the anchor to EN
    metres; at survey scale equirectangular and Web Mercator agree to
    well under a pixel, so a per-tile position + scale is exact enough.
    """

    def __init__(self, scene: QGraphicsScene) -> None:
        self._scene = scene
        self._fetcher = TileFetcher()
        self._fetcher.tile_ready.connect(self._on_tile_ready)
        self._items: dict[tuple, QGraphicsPixmapItem] = {}
        self._zoom = -1
        self._anchor: GeoAnchor | None = None
        self.enabled = True

    def set_anchor(self, anchor: GeoAnchor | None) -> None:
        self._anchor = anchor
        self._clear()

    def _clear(self) -> None:
        for item in self._items.values():
            self._scene.removeItem(item)
        self._items.clear()

    def update_view(self, view_rect: QRectF, px_per_m: float) -> None:
        if self._anchor is None or not self.enabled:
            return
        a = self._anchor
        zoom = zoom_for_resolution(a.lat0, 1.0 / max(px_per_m, 1e-9))
        zoom = max(MIN_ZOOM, min(MAX_ZOOM, zoom))
        if zoom != self._zoom:
            self._clear()
            self._zoom = zoom
        # View rect corners (EN) -> lat/lon -> tile range.
        e0, e1 = view_rect.left(), view_rect.right()
        n0, n1 = view_rect.top(), view_rect.bottom()
        lats, lons = [], []
        for e, n in ((e0, n0), (e0, n1), (e1, n0), (e1, n1)):
            lat, lon = a.to_latlon(e, n)
            lats.append(float(lat))
            lons.append(float(lon))
        x0, y0 = latlon_to_tile_xy(max(lats), min(lons), zoom)
        x1, y1 = latlon_to_tile_xy(min(lats), max(lons), zoom)
        count = 0
        for tx in range(int(math.floor(x0)), int(math.floor(x1)) + 1):
            for ty in range(int(math.floor(y0)), int(math.floor(y1)) + 1):
                if count >= MAX_TILES_PER_UPDATE:
                    return
                count += 1
                key = (zoom, tx, ty)
                if key in self._items:
                    continue
                img = self._fetcher.request(key)
                if img is not None:
                    self._place(key, img)

    def _on_tile_ready(self, z: int, x: int, y: int, img) -> None:
        if z == self._zoom and (z, x, y) not in self._items:
            self._place((z, x, y), img)

    def _place(self, key: tuple, img) -> None:
        from PySide6.QtGui import QPixmap
        if self._anchor is None:
            return
        z, x, y = key
        a = self._anchor
        lat_nw, lon_nw = tile_xy_to_latlon(float(x), float(y), z)
        lat_se, lon_se = tile_xy_to_latlon(float(x + 1), float(y + 1), z)
        e_w, n_n = a.to_en(lat_nw, lon_nw)
        e_e, n_s = a.to_en(lat_se, lon_se)
        item = QGraphicsPixmapItem(QPixmap.fromImage(img))
        item.setZValue(-100)
        item.setPos(float(e_w), float(n_n))
        sx = (float(e_e) - float(e_w)) / TILE_SIZE_PX
        sy = (float(n_s) - float(n_n)) / TILE_SIZE_PX   # negative: y-down image
        item.setTransform(QTransform.fromScale(sx, sy))
        self._scene.addItem(item)
        self._items[key] = item


# ----------------------------------------------------------------- handles
class HandleItem(QGraphicsEllipseItem):
    """Constant-pixel draggable handle reporting moves to the host."""

    R = 6.0

    def __init__(self, host: "BuilderMapView", role: tuple) -> None:
        super().__init__(-self.R, -self.R, 2 * self.R, 2 * self.R)
        self._host = host
        self.role = role                    # ("rect", corner) | ("wall", i, end)
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsMovable
                      | QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges
                      | QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.setZValue(60)
        color = theme.C_LIMITS if role[0] == "rect" else theme.C_WALL
        if role[0] == "wall" and role[2] == "ctrl":
            # Hollow: the midpoint handle that bends a wall (curved walls).
            self.setBrush(QBrush(QColor(0, 0, 0, 0)))
            self.setPen(QPen(QColor(color), 2.0))
        else:
            self.setBrush(QBrush(QColor(color)))
            self.setPen(QPen(QColor("white"), 1.2))

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange \
                and not self._host.syncing:
            value = self._host.constrain_handle(value, self)
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged \
                and not self._host.syncing:
            self._host.on_handle_moved(self)
        return super().itemChange(change, value)


# ------------------------------------------------------------------- view
class BuilderMapView(QGraphicsView):
    """The Screen-2 canvas (also reused read-only elsewhere)."""

    limits_changed = Signal()
    walls_changed = Signal()
    strokes_changed = Signal()
    brush_changed = Signal(float)           # radius [m] after Ctrl+wheel
    wall_double_clicked = Signal(int)       # index into walls()
    mode_changed = Signal(object)
    status = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.syncing = False
        self.geometry_editable = True
        self._mode = Mode.SELECT
        self._panning: QPointF | None = None
        self._rect_drag_start: QPointF | None = None
        self._wall_first: QPointF | None = None
        # Paint strokes per layer (design-frame metres) + the brush.
        self._strokes: dict[str, list[dict]] = {k: [] for k in STROKE_KINDS}
        self._stroke_items: list[QGraphicsPathItem] = []
        self._brush = {"value": 3.0, "radius_m": 2.0, "material": "mud",
                       "feather_m": 1.0}
        self._layer = "depth"               # what the eraser erases
        self._cur_stroke: dict | None = None
        self._cur_item: QGraphicsPathItem | None = None
        # Screen-3 context from a sonar recording: the sailed track's
        # soundings (dots coloured by depth) and a raster overlay (the
        # backscatter reflectivity hint).
        self._track_items: list = []
        self._overlay_item = None

        self._scene = QGraphicsScene(self)
        self._scene.setSceneRect(-5e4, -5e4, 1e5, 1e5)
        self.setScene(self._scene)
        # A wall's midpoint (bend) handle shows only while it is selected.
        self._scene.selectionChanged.connect(self._on_selection_changed)
        self._ctrl_handles: dict[int, HandleItem] = {}
        self._snap_marker: QGraphicsEllipseItem | None = None
        self.setRenderHints(QPainter.RenderHint.Antialiasing
                            | QPainter.RenderHint.SmoothPixmapTransform)
        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QColor(theme.BG_DARK))
        self.scale(6.0, -6.0)               # ENU north-up

        self.tiles = TileLayer(self._scene)
        self._anchor: GeoAnchor | None = None

        # Path polyline (static context).
        self._path_item = self._scene.addPolygon(
            QPolygonF(), _cosmetic_pen(theme.C_PATH, 2.0))
        self._path_item.setZValue(20)
        self._path_item.setBrush(QBrush(Qt.BrushStyle.NoBrush))

        # Limits rectangle + wall lines + their handles.
        pen_rect = _cosmetic_pen(theme.C_LIMITS, 2.0)
        pen_rect.setStyle(Qt.PenStyle.DashLine)
        self._rect_item = QGraphicsRectItem()
        self._rect_item.setPen(pen_rect)
        self._rect_item.setZValue(30)
        self._rect_item.setVisible(False)
        self._scene.addItem(self._rect_item)
        self._limits: dict[str, float] | None = None
        self._walls: list[dict] = []
        self._wall_items: list[QGraphicsLineItem] = []
        self._handles: list[HandleItem] = []
        self._rubber: QGraphicsRectItem | QGraphicsLineItem | None = None

    # ------------------------------------------------------------- data API
    def set_anchor(self, anchor: GeoAnchor | None) -> None:
        self._anchor = anchor
        self.tiles.set_anchor(anchor)
        self._update_tiles()

    def set_path(self, xy: np.ndarray) -> None:
        poly = QPolygonF([QPointF(float(x), float(y)) for x, y in xy])
        self._path_item.setPolygon(poly)

    def set_limits(self, limits: dict | None) -> None:
        self._limits = dict(limits) if limits else None
        self._sync_items()

    def limits(self) -> dict | None:
        return dict(self._limits) if self._limits else None

    @staticmethod
    def _copy_wall(w: dict) -> dict:
        out = dict(w)
        if out.get("ctrl") is not None:
            out["ctrl"] = [float(out["ctrl"][0]), float(out["ctrl"][1])]
        return out

    def set_walls(self, walls: list) -> None:
        self._walls = [self._copy_wall(w) for w in walls]
        self._sync_items()

    def walls(self) -> list[dict]:
        return [self._copy_wall(w) for w in self._walls]

    def straighten_selected_walls(self) -> None:
        """Drop the bend (``ctrl``) of every selected curved wall."""
        changed = False
        for i, it in enumerate(self._wall_items):
            if it.isSelected() and self._walls[i].get("ctrl") is not None:
                self._walls[i].pop("ctrl", None)
                changed = True
        if changed:
            self._sync_items()
            self.walls_changed.emit()

    # ---- wall geometry helpers (design metres) --------------------------
    @staticmethod
    def _wall_mid(w: dict) -> QPointF:
        """The bend handle's position: the through point of a curved wall,
        the chord midpoint of a straight one."""
        if w.get("ctrl") is not None:
            return QPointF(float(w["ctrl"][0]), float(w["ctrl"][1]))
        return QPointF(0.5 * (w["x0"] + w["x1"]), 0.5 * (w["y0"] + w["y1"]))

    @staticmethod
    def _wall_path(w: dict) -> QPainterPath:
        path = QPainterPath(QPointF(w["x0"], w["y0"]))
        if w.get("ctrl") is None:
            path.lineTo(QPointF(w["x1"], w["y1"]))
            return path
        # The dragged point M lies ON the curve (t = 1/2); the Bezier
        # control point is 2M - (P0 + P2)/2 -- the same construction
        # core.geometry.quad_bezier_polyline flattens for the engine.
        mx, my = float(w["ctrl"][0]), float(w["ctrl"][1])
        cx = 2.0 * mx - 0.5 * (w["x0"] + w["x1"])
        cy = 2.0 * my - 0.5 * (w["y0"] + w["y1"])
        path.quadTo(QPointF(cx, cy), QPointF(w["x1"], w["y1"]))
        return path

    def _update_wall_item(self, i: int) -> None:
        w = self._walls[i]
        self.syncing = True
        self._wall_items[i].setPath(self._wall_path(w))
        h = self._ctrl_handles.get(i)
        if h is not None:
            h.setPos(self._wall_mid(w))
        self.syncing = False

    def wall_endpoints(self, exclude: int | None = None) -> list[tuple[float, float]]:
        """Every wall endpoint except those of wall ``exclude``."""
        pts = []
        for i, w in enumerate(self._walls):
            if i == exclude:
                continue
            pts.append((float(w["x0"]), float(w["y0"])))
            pts.append((float(w["x1"]), float(w["y1"])))
        return pts

    def snap_to_endpoint(self, p: QPointF, exclude: int | None = None,
                         tol_m: float = WALL_SNAP_M) -> tuple[QPointF, bool]:
        """Nearest other wall endpoint within ``tol_m``, else ``p``."""
        best, best_d = None, tol_m
        for x, y in self.wall_endpoints(exclude):
            d = math.hypot(x - p.x(), y - p.y())
            if d <= best_d:
                best, best_d = QPointF(x, y), d
        return (best, True) if best is not None else (p, False)

    def _show_snap_marker(self, p: QPointF | None) -> None:
        if self._snap_marker is not None:
            self._scene.removeItem(self._snap_marker)
            self._snap_marker = None
        if p is None:
            return
        r = 9.0
        m = QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        m.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        m.setPen(QPen(QColor(theme.C_WALL), 2.0))
        m.setBrush(QBrush(QColor(0, 0, 0, 0)))
        m.setZValue(95)
        m.setPos(p)
        self._scene.addItem(m)
        self._snap_marker = m

    def _on_selection_changed(self) -> None:
        for i, h in self._ctrl_handles.items():
            if i < len(self._wall_items):
                h.setVisible(self._wall_items[i].isSelected())

    def delete_selected_walls(self) -> None:
        sel = {i for i, it in enumerate(self._wall_items) if it.isSelected()}
        if sel:
            self._walls = [w for i, w in enumerate(self._walls)
                           if i not in sel]
            self._sync_items()
            self.walls_changed.emit()

    def set_geometry_editable(self, editable: bool) -> None:
        """Read-only limits/walls (no handles, no selection) -- Screen 3
        shows them as context under the paint."""
        self.geometry_editable = bool(editable)
        self._sync_items()

    # ------------------------------------------------- survey context API
    def set_track(self, east=None, north=None, depth=None,
                  max_points: int = 4000) -> None:
        """Show (or clear, with ``None``) the soundings of a recording as
        small dots coloured by depth, design-frame metres."""
        import numpy as np
        for it in self._track_items:
            self._scene.removeItem(it)
        self._track_items = []
        if east is None or north is None or len(east) == 0:
            return
        e = np.asarray(east, dtype=float)
        n = np.asarray(north, dtype=float)
        d = (np.asarray(depth, dtype=float) if depth is not None
             else np.full(len(e), 3.0))
        stride = max(1, int(np.ceil(len(e) / max_points)))
        for x, y, z in zip(e[::stride], n[::stride], d[::stride]):
            col = _depth_color(float(z))
            col.setAlpha(220)
            it = self._scene.addEllipse(x - 0.35, y - 0.35, 0.7, 0.7,
                                        QPen(Qt.PenStyle.NoPen), QBrush(col))
            it.setZValue(26)
            self._track_items.append(it)

    def set_overlay(self, hint: dict | None) -> None:
        """A 0-1 raster (``values`` [ny, nx], ``origin`` (x, y) of cell
        (0, 0), ``cell_m``) drawn semi-transparently under the strokes;
        ``None`` clears it."""
        if self._overlay_item is not None:
            self._scene.removeItem(self._overlay_item)
            self._overlay_item = None
        if not hint or hint.get("values") is None:
            return
        import numpy as np
        from PySide6.QtGui import QImage, QPixmap
        vals = np.asarray(hint["values"], dtype=float)
        ny, nx = vals.shape
        if nx == 0 or ny == 0:
            return
        rgba = np.zeros((ny, nx, 4), dtype=np.uint8)
        ok = np.isfinite(vals)
        v = np.clip(np.nan_to_num(vals), 0.0, 1.0)
        rgba[..., 0] = (60 + 195 * v).astype(np.uint8)
        rgba[..., 1] = (60 + 170 * v).astype(np.uint8)
        rgba[..., 2] = (90 + 60 * v).astype(np.uint8)
        rgba[..., 3] = np.where(ok, 140, 0).astype(np.uint8)
        img = QImage(rgba.tobytes(), nx, ny, 4 * nx,
                     QImage.Format.Format_RGBA8888).copy()
        item = self._scene.addPixmap(QPixmap.fromImage(img))
        ox, oy = hint["origin"]
        cell = float(hint["cell_m"])
        # Row 0 is the min-north row; the scene's y axis is north-up, so
        # place the image's bottom edge at the origin and flip vertically.
        item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
        item.setPos(ox - 0.5 * cell, oy - 0.5 * cell + ny * cell)
        item.setScale(cell)
        tr = item.transform()
        tr.scale(1.0, -1.0)
        item.setTransform(tr)
        item.setZValue(24)
        self._overlay_item = item

    # --------------------------------------------------------- paint API
    def set_strokes(self, strokes: list, kind: str = "depth") -> None:
        """Replace one layer's strokes (``worldgen.depth.paint`` dicts,
        design-frame metres)."""
        self._check_kind(kind)
        self._strokes[kind] = [self._copy_stroke(s) for s in strokes or []]
        self._sync_items()

    def strokes(self, kind: str = "depth") -> list[dict]:
        self._check_kind(kind)
        return [self._copy_stroke(s) for s in self._strokes[kind]]

    def undo_stroke(self, kind: str | None = None) -> None:
        """Pop the last stroke of ``kind`` (default: the active layer)."""
        kind = kind or self._layer
        if self._strokes[kind]:
            self._strokes[kind].pop()
            self._sync_items()
            self.strokes_changed.emit()

    def clear_strokes(self, kind: str | None = None) -> None:
        kind = kind or self._layer
        if self._strokes[kind]:
            self._strokes[kind] = []
            self._sync_items()
            self.strokes_changed.emit()

    def set_brush(self, value: float | None = None,
                  radius_m: float | None = None,
                  material: str | None = None,
                  feather_m: float | None = None) -> None:
        if value is not None:
            self._brush["value"] = float(value)
        if feather_m is not None:
            self._brush["feather_m"] = float(max(feather_m, 0.05))
        if radius_m is not None:
            self._brush["radius_m"] = float(min(max(
                radius_m, BRUSH_RADIUS_MIN_M), BRUSH_RADIUS_MAX_M))
        if material is not None:
            self._brush["material"] = str(material)

    @property
    def brush_radius(self) -> float:
        return float(self._brush["radius_m"])

    @property
    def brush_layer(self) -> str:
        """The layer the brush (and the eraser) currently targets."""
        return self._layer

    @staticmethod
    def _check_kind(kind: str) -> None:
        if kind not in STROKE_KINDS:
            raise ValueError(f"stroke kind {kind!r} not in {STROKE_KINDS}")

    @staticmethod
    def _copy_stroke(s: dict) -> dict:
        out = dict(s)
        out["points"] = [[float(x), float(y)] for x, y in s["points"]]
        return out

    # ------------------------------------------------------------- modes
    def set_mode(self, mode: Mode) -> None:
        self._mode = mode
        self._wall_first = None
        self._cancel_stroke()
        if mode is Mode.DEPTH_BRUSH:
            self._layer = "depth"
        elif mode is Mode.MATERIAL_BRUSH:
            self._layer = "material"
        self.viewport().setCursor(
            Qt.CursorShape.CrossCursor if mode is not Mode.SELECT
            else Qt.CursorShape.ArrowCursor)
        self.mode_changed.emit(mode)
        if mode is Mode.RECT:
            self.status.emit("drag the world-limits rectangle")
        elif mode is Mode.WALL:
            self.status.emit("click the wall's two endpoints "
                             "(right-click to finish)")
        elif mode in BRUSH_MODES:
            what = ("erase " + self._layer + " paint" if mode is Mode.ERASER
                    else f"paint {self._layer}")
            self.status.emit(f"{what}: drag on the map · Ctrl+wheel sets "
                             f"the radius ({self.brush_radius:.2f} m) · "
                             "right-click to finish")

    @property
    def mode(self) -> Mode:
        return self._mode

    # ------------------------------------------------------ item rebuilding
    def _sync_items(self) -> None:
        self.syncing = True
        for h in self._handles:
            self._scene.removeItem(h)
        for it in self._wall_items:
            self._scene.removeItem(it)
        for it in self._stroke_items:
            self._scene.removeItem(it)
        self._handles, self._wall_items, self._stroke_items = [], [], []
        editable = self.geometry_editable

        if self._limits:
            r = self._limits
            self._rect_item.setRect(QRectF(
                QPointF(r["x_min"], r["y_min"]),
                QPointF(r["x_max"], r["y_max"])))
            self._rect_item.setVisible(True)
            for corner in ("sw", "se", "ne", "nw") if editable else ():
                h = HandleItem(self, ("rect", corner))
                h.setPos(*self._corner_pos(corner))
                self._scene.addItem(h)
                self._handles.append(h)
        else:
            self._rect_item.setVisible(False)

        self._ctrl_handles = {}
        for i, w in enumerate(self._walls):
            line = QGraphicsPathItem(self._wall_path(w))
            line.setPen(_cosmetic_pen(theme.C_WALL, 3.0))
            line.setZValue(35)
            line.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable,
                         editable)
            self._scene.addItem(line)
            self._wall_items.append(line)
            for end in (0, 1) if editable else ():
                h = HandleItem(self, ("wall", i, end))
                h.setPos(w[f"x{end}"], w[f"y{end}"])
                self._scene.addItem(h)
                self._handles.append(h)
            if editable:
                # Bend handle: visible only while the wall is selected.
                h = HandleItem(self, ("wall", i, "ctrl"))
                h.setPos(self._wall_mid(w))
                h.setVisible(False)
                h.setToolTip("Drag to bend this wall into a curve "
                             "(Straighten removes the bend)")
                self._scene.addItem(h)
                self._handles.append(h)
                self._ctrl_handles[i] = h

        # Paint: material strokes under depth strokes, both under walls.
        for kind, z in (("material", 24), ("depth", 26)):
            for s in self._strokes[kind]:
                item = self._make_stroke_item(s, kind)
                item.setZValue(z)
                self._scene.addItem(item)
                self._stroke_items.append(item)
        self.syncing = False

    @staticmethod
    def _stroke_path(stroke: dict) -> QPainterPath:
        pts = stroke["points"]
        path = QPainterPath()
        if len(pts) == 1:
            r = float(stroke["radius_m"])
            path.addEllipse(QPointF(pts[0][0], pts[0][1]), r, r)
            return path
        path.moveTo(pts[0][0], pts[0][1])
        for x, y in pts[1:]:
            path.lineTo(x, y)
        return path

    def _make_stroke_item(self, stroke: dict, kind: str) -> QGraphicsPathItem:
        """Translucent capsule stroke in scene metres (pen width = the
        brush diameter, round caps/joins), a filled disc for a dot."""
        item = QGraphicsPathItem(self._stroke_path(stroke))
        color = stroke_color(stroke, kind)
        if len(stroke["points"]) == 1:
            item.setPen(QPen(Qt.PenStyle.NoPen))
            item.setBrush(QBrush(color))
        else:
            pen = QPen(color, 2.0 * float(stroke["radius_m"]))
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            item.setPen(pen)
            item.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        return item

    # ------------------------------------------------------------ strokes
    def _begin_stroke(self, world: QPointF) -> None:
        erase = self._mode is Mode.ERASER
        value = (None if erase else self._brush["material"]
                 if self._layer == "material" else self._brush["value"])
        self._cur_stroke = {"points": [[world.x(), world.y()]],
                            "radius_m": self.brush_radius,
                            "value": value, "erase": erase}
        if self._layer == "depth" and not erase:
            # Depth strokes blend into the base over their own feather.
            self._cur_stroke["feather_m"] = float(self._brush["feather_m"])
        self._cur_item = self._make_stroke_item(self._cur_stroke, self._layer)
        self._cur_item.setZValue(28)
        self._scene.addItem(self._cur_item)

    def _extend_stroke(self, world: QPointF) -> None:
        if self._cur_stroke is None:
            return
        lx, ly = self._cur_stroke["points"][-1]
        if math.hypot(world.x() - lx, world.y() - ly) < max(
                0.25 * self.brush_radius, 0.05):
            return
        self._cur_stroke["points"].append([world.x(), world.y()])
        # Redraw as a capsule once it is a real polyline.
        self._scene.removeItem(self._cur_item)
        self._cur_item = self._make_stroke_item(self._cur_stroke, self._layer)
        self._cur_item.setZValue(28)
        self._scene.addItem(self._cur_item)

    def _end_stroke(self) -> None:
        if self._cur_stroke is None:
            return
        stroke, self._cur_stroke = self._cur_stroke, None
        if self._cur_item is not None:
            self._scene.removeItem(self._cur_item)
            self._cur_item = None
        self._strokes[self._layer].append(stroke)
        self._sync_items()
        self.strokes_changed.emit()

    def _cancel_stroke(self) -> None:
        self._cur_stroke = None
        if self._cur_item is not None:
            self._scene.removeItem(self._cur_item)
            self._cur_item = None

    def _corner_pos(self, corner: str) -> tuple[float, float]:
        r = self._limits
        return {"sw": (r["x_min"], r["y_min"]), "se": (r["x_max"], r["y_min"]),
                "ne": (r["x_max"], r["y_max"]), "nw": (r["x_min"], r["y_max"])
                }[corner]

    # -------------------------------------------------------- handle moves
    def constrain_handle(self, pos: QPointF,
                         handle: "HandleItem | None" = None) -> QPointF:
        """Ctrl snaps to the metre grid; a wall *endpoint* additionally
        snaps to any other wall's endpoint within WALL_SNAP_M (endpoint
        snap wins), so corners drawn by hand meet exactly."""
        mods = QGuiApplication.keyboardModifiers()
        x, y = pos.x(), pos.y()
        if (handle is not None and handle.role[0] == "wall"
                and handle.role[2] in (0, 1)):
            snapped, hit = self.snap_to_endpoint(pos, exclude=handle.role[1])
            self._show_snap_marker(snapped if hit else None)
            if hit:
                return snapped
        if mods & Qt.KeyboardModifier.ControlModifier:
            x = round(x / GRID_SNAP_M) * GRID_SNAP_M
            y = round(y / GRID_SNAP_M) * GRID_SNAP_M
        return QPointF(x, y)

    def on_handle_moved(self, handle: HandleItem) -> None:
        x, y = handle.pos().x(), handle.pos().y()
        if handle.role[0] == "rect" and self._limits:
            corner = handle.role[1]
            r = self._limits
            if corner in ("sw", "nw"):
                r["x_min"] = x
            else:
                r["x_max"] = x
            if corner in ("sw", "se"):
                r["y_min"] = y
            else:
                r["y_max"] = y
            if r["x_min"] > r["x_max"]:
                r["x_min"], r["x_max"] = r["x_max"], r["x_min"]
            if r["y_min"] > r["y_max"]:
                r["y_min"], r["y_max"] = r["y_max"], r["y_min"]
            self.syncing = True
            self._rect_item.setRect(QRectF(QPointF(r["x_min"], r["y_min"]),
                                           QPointF(r["x_max"], r["y_max"])))
            for h in self._handles:
                if h.role[0] == "rect" and h is not handle:
                    h.setPos(*self._corner_pos(h.role[1]))
            self.syncing = False
            self.limits_changed.emit()
        elif handle.role[0] == "wall":
            _, i, end = handle.role
            w = self._walls[i]
            if end == "ctrl":
                w["ctrl"] = [float(x), float(y)]
            else:
                w[f"x{end}"], w[f"y{end}"] = x, y
            self._update_wall_item(i)
            self.walls_changed.emit()

    # ---------------------------------------------------------------- input
    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        if self._mode in BRUSH_MODES and (
                event.modifiers() & Qt.KeyboardModifier.ControlModifier):
            self.set_brush(radius_m=self.brush_radius * factor)
            self.brush_changed.emit(self.brush_radius)
            self.status.emit(f"brush radius {self.brush_radius:.2f} m")
            return
        px = abs(self.transform().m11()) * factor
        if 0.05 <= px <= 2000.0:
            self.scale(factor, factor)
        self._update_tiles()

    def mousePressEvent(self, event) -> None:
        world = self.mapToScene(event.position().toPoint())
        if event.button() == Qt.MouseButton.LeftButton:
            if self._mode is Mode.RECT:
                self._rect_drag_start = world
                return
            if self._mode is Mode.WALL:
                self._on_wall_click(world)
                return
            if self._mode in BRUSH_MODES:
                self._begin_stroke(world)
                return
        if event.button() == Qt.MouseButton.RightButton \
                and self._mode is not Mode.SELECT:
            self.set_mode(Mode.SELECT)
            return
        if event.button() in (Qt.MouseButton.MiddleButton,
                              Qt.MouseButton.RightButton):
            self._panning = event.position()
            self.viewport().setCursor(Qt.CursorShape.ClosedHandCursor)
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._panning is not None:
            # 1:1 panning via scrollbar deltas (transform-translate panning
            # compounds with AnchorUnderMouse and races ahead of the cursor).
            delta = event.position() - self._panning
            self._panning = event.position()
            h = self.horizontalScrollBar()
            v = self.verticalScrollBar()
            h.setValue(h.value() - int(round(delta.x())))
            v.setValue(v.value() - int(round(delta.y())))
            return
        world = self.mapToScene(event.position().toPoint())
        if self._mode is Mode.RECT and self._rect_drag_start is not None:
            self._show_rubber_rect(self._rect_drag_start, world)
            return
        if self._mode is Mode.WALL and self._wall_first is not None:
            snapped, hit = self.snap_to_endpoint(world)
            self._show_snap_marker(snapped if hit else None)
            self._show_rubber_line(self._wall_first, snapped)
            return
        if self._mode in BRUSH_MODES and self._cur_stroke is not None:
            self._extend_stroke(world)
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self._panning is not None and event.button() in (
                Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton):
            self._panning = None
            self.viewport().setCursor(Qt.CursorShape.ArrowCursor)
            self._update_tiles()
            return
        if self._mode is Mode.RECT and self._rect_drag_start is not None \
                and event.button() == Qt.MouseButton.LeftButton:
            world = self.mapToScene(event.position().toPoint())
            self._finish_rect(self._rect_drag_start, world)
            self._rect_drag_start = None
            return
        if self._mode in BRUSH_MODES and self._cur_stroke is not None \
                and event.button() == Qt.MouseButton.LeftButton:
            self._extend_stroke(self.mapToScene(event.position().toPoint()))
            self._end_stroke()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:
        # The topmost item may be a handle sitting on the wall: look
        # through the stack for the wall itself (or the handle's wall).
        for item in self.items(event.position().toPoint()):
            if item in self._wall_items:
                self.wall_double_clicked.emit(self._wall_items.index(item))
                return
            if isinstance(item, HandleItem) and item.role[0] == "wall":
                self.wall_double_clicked.emit(int(item.role[1]))
                return
        super().mouseDoubleClickEvent(event)

    # ------------------------------------------------------------- drawing
    def _clear_rubber(self) -> None:
        if self._rubber is not None:
            self._scene.removeItem(self._rubber)
            self._rubber = None

    def _show_rubber_rect(self, a: QPointF, b: QPointF) -> None:
        self._clear_rubber()
        self._rubber = QGraphicsRectItem(QRectF(a, b).normalized())
        self._rubber.setPen(_cosmetic_pen(theme.C_LIMITS, 1.5))
        self._rubber.setZValue(90)
        self._scene.addItem(self._rubber)

    def _show_rubber_line(self, a: QPointF, b: QPointF) -> None:
        self._clear_rubber()
        self._rubber = QGraphicsLineItem(QLineF(a, b))
        self._rubber.setPen(_cosmetic_pen(theme.C_WALL, 1.5))
        self._rubber.setZValue(90)
        self._scene.addItem(self._rubber)

    def _finish_rect(self, a: QPointF, b: QPointF) -> None:
        self._clear_rubber()
        rect = QRectF(a, b).normalized()
        if rect.width() < 5.0 or rect.height() < 5.0:
            self.status.emit("rectangle discarded (smaller than 5 m)")
            self.set_mode(Mode.SELECT)
            return
        self._limits = {"x_min": rect.left(), "y_min": rect.top(),
                        "x_max": rect.right(), "y_max": rect.bottom()}
        self._sync_items()
        self.limits_changed.emit()
        self.set_mode(Mode.SELECT)

    def _on_wall_click(self, world: QPointF) -> None:
        # Both clicks snap to an existing endpoint within WALL_SNAP_M.
        world, _hit = self.snap_to_endpoint(world)
        if self._wall_first is None:
            self._wall_first = world
            return
        self._clear_rubber()
        self._show_snap_marker(None)
        a, b = self._wall_first, world
        self._wall_first = None
        if math.hypot(b.x() - a.x(), b.y() - a.y()) < 0.5:
            self.status.emit("wall discarded (shorter than 0.5 m)")
            return
        self._walls.append({
            "name": self._free_wall_name(),
            "x0": a.x(), "y0": a.y(), "x1": b.x(), "y1": b.y(),
            "top_z": 1.0,
            "reflectivity": 0.65,           # concrete quay default
            "thickness": 0.30,
            "kind": "quay",
            "draft_m": 0.4,
            "roughness_m": 0.003,
        })
        self._sync_items()
        self.walls_changed.emit()
        # stay in WALL mode: several walls in a row is the common case

    def _free_wall_name(self) -> str:
        used = {w["name"] for w in self._walls}
        i = 1
        while f"wall_{i}" in used:
            i += 1
        return f"wall_{i}"

    # ------------------------------------------------------------- painting
    def drawBackground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawBackground(painter, rect)
        self._grid_spacing = self._draw_grid(
            painter, rect, abs(self.transform().m11()))

    def drawForeground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawForeground(painter, rect)
        self._draw_scale_bar(painter, abs(self.transform().m11()),
                             getattr(self, "_grid_spacing", 0.0))
        self._draw_north(painter)

    def _draw_grid(self, painter, rect: QRectF, px_per_m: float) -> float:
        """Adaptive 1/2/5-decade metric grid; returns the spacing chosen.
        Over satellite imagery the lines get a dark halo (MCS recipe)."""
        target_px = 60.0
        raw = target_px / max(px_per_m, 1e-9)
        exp = math.floor(math.log10(max(raw, 1e-9)))
        base = raw / (10 ** exp)
        spacing = 10 ** (exp + 1)
        for mult in (1.0, 2.0, 5.0, 10.0):
            if base <= mult:
                spacing = mult * 10 ** exp
                break
        passes = [
            (_cosmetic_pen(QColor(0, 0, 0, 140), 2.6),
             _cosmetic_pen(QColor(0, 0, 0, 170), 3.0)),
            (_cosmetic_pen(QColor(255, 255, 255, 130), 1.0),
             _cosmetic_pen(QColor(255, 255, 255, 200), 1.3)),
        ] if self._anchor is not None else [
            (_cosmetic_pen(theme.C_GRID, 1.0),
             _cosmetic_pen(theme.C_GRID_MAJOR, 1.0))]
        for pen_minor, pen_major in passes:
            x = math.floor(rect.left() / spacing) * spacing
            i = 0
            while x <= rect.right() and i < 400:
                painter.setPen(pen_major if abs((x / spacing) % 5) < 1e-6
                               else pen_minor)
                painter.drawLine(QLineF(x, rect.top(), x, rect.bottom()))
                x += spacing
                i += 1
            y = math.floor(rect.top() / spacing) * spacing
            i = 0
            while y <= rect.bottom() and i < 400:
                painter.setPen(pen_major if abs((y / spacing) % 5) < 1e-6
                               else pen_minor)
                painter.drawLine(QLineF(rect.left(), y, rect.right(), y))
                y += spacing
                i += 1
        return spacing

    def _draw_scale_bar(self, painter, px_per_m: float,
                        spacing_m: float) -> None:
        if spacing_m <= 0 or px_per_m <= 0:
            return
        bar_px = spacing_m * px_per_m
        x0, y0 = 16.0, self.viewport().height() - 16.0
        painter.save()
        painter.resetTransform()
        painter.setPen(QPen(QColor(230, 235, 240), 2))
        painter.drawLine(QLineF(x0, y0, x0 + bar_px, y0))
        for x in (x0, x0 + bar_px):
            painter.drawLine(QLineF(x, y0 - 4, x, y0 + 4))
        painter.setFont(QFont("DejaVu Sans Mono", 9))
        painter.drawText(QPointF(x0 + bar_px + 8, y0 + 4), f"{spacing_m:g} m")
        painter.restore()

    def _draw_north(self, painter) -> None:
        w = self.viewport().width()
        painter.save()
        painter.resetTransform()
        painter.setFont(QFont("DejaVu Sans", 9, QFont.Weight.Bold))
        painter.setPen(QColor(230, 235, 240))
        painter.drawText(QPointF(w - 34.0, 20.0), "N")
        painter.setPen(_cosmetic_pen(QColor(230, 235, 240), 2.0))
        painter.drawLine(QLineF(w - 22.0, 22.0, w - 22.0, 8.0))
        painter.drawLine(QLineF(w - 26.0, 12.0, w - 22.0, 8.0))
        painter.drawLine(QLineF(w - 18.0, 12.0, w - 22.0, 8.0))
        painter.restore()

    # ------------------------------------------------------------- framing
    def center_on_bounds(self, xs, ys, margin_m: float = 15.0) -> None:
        """Frame the given world extents, preserving the y-up flip
        (fitInView may normalise the scale signs and undo it)."""
        xs, ys = list(xs), list(ys)
        if not xs:
            return
        w = max(max(xs) - min(xs), 1.0) + 2 * margin_m
        h = max(max(ys) - min(ys), 1.0) + 2 * margin_m
        cx = (max(xs) + min(xs)) / 2.0
        cy = (max(ys) + min(ys)) / 2.0
        s = min(self.viewport().width() / w, self.viewport().height() / h)
        s = max(0.05, min(s, 200.0))
        t = self.transform()
        t.setMatrix(s, 0, 0, 0, -s, 0, t.m31(), t.m32(), 1.0)
        self.setTransform(t)
        self.centerOn(cx, cy)
        self._update_tiles()

    def _update_tiles(self) -> None:
        self.tiles.update_view(
            self.mapToScene(self.viewport().rect()).boundingRect(),
            abs(self.transform().m11()))

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._update_tiles()
