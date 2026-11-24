"""The property editor of one authored layer: name, enabled, opacity,
blend, channels, the kind's schema form (hover help on every row), its
geometry and its mask. Geometry and mask shapes are drawn on the map (the
page switches the canvas mode); everything else is edited here."""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox,
                               QFormLayout, QGroupBox, QHBoxLayout, QLabel,
                               QLineEdit, QPushButton, QStackedWidget,
                               QVBoxLayout, QWidget)

from ...worldgen.depth.layers import BLEND_MODES, LAYER_KINDS
from ...worldgen.depth.masks import MASK_KINDS
from ..forms import SchemaForm

#: Mask kinds that need a shape drawn on the map, and the tool that draws it.
MASK_GEOMETRY = {"rect": "rect", "ellipse": "ellipse", "polygon": "polygon",
                 "polyline_band": "polyline", "gradient": "axis",
                 "brush": "brush"}
MASK_KIND_LABELS = {"none": "none (whole area)", "all": "whole area",
                    "rect": "rectangle", "ellipse": "ellipse",
                    "polygon": "polygon", "polyline_band": "band along a line",
                    "distance_to_wall": "within a distance of the walls",
                    "depth_range": "depth range of the bed below",
                    "gradient": "gradient (axis)", "brush": "mask brush strokes",
                    "coverage": "where the survey reached"}
BLEND_HELP = {"normal": "Replaces the bed with the layer under the mask.",
              "add": "Adds the layer's field to the bed.",
              "deepen": "Only deepens: dredging, the deeper of the two wins.",
              "shoal": "Only shoals: deposition, the shallower of the two wins."}


def _spin(lo: float, hi: float, step: float, dec: int, tip: str) -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setDecimals(dec)
    s.setToolTip(tip)
    return s


class LayerProperties(QWidget):
    edited = Signal()
    draw_geometry_requested = Signal()
    draw_mask_requested = Signal(str)      # mask kind
    clear_mask_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._layer: dict[str, Any] | None = None
        self._blocking = False
        self._forms: dict[str, SchemaForm] = {}
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.header = QLabel("")
        self.header.setWordWrap(True)
        self.header.setObjectName("hint")
        lay.addWidget(self.header)

        g = QGroupBox("Layer")
        f = QFormLayout(g)
        self.name = QLineEdit()
        self.name.editingFinished.connect(self._emit)
        f.addRow("Name", self.name)
        self.enabled = QCheckBox("composed")
        self.enabled.setToolTip("Untick to leave the layer out of the seabed "
                                "without deleting it.")
        self.enabled.toggled.connect(self._emit)
        f.addRow("Enabled", self.enabled)
        self.opacity = _spin(0.0, 1.0, 0.05, 2,
                             "How strongly the layer applies: 1 fully, 0.5 "
                             "half-way toward its value.")
        self.opacity.valueChanged.connect(self._emit)
        f.addRow("Opacity", self.opacity)
        self.blend = QComboBox()
        for m in BLEND_MODES:
            self.blend.addItem(m)
        self.blend.setToolTip("\n".join(f"{m}: {h}" for m, h in BLEND_HELP.items()))
        self.blend.currentTextChanged.connect(self._emit)
        f.addRow("Blend", self.blend)
        tw = QWidget()
        tl = QHBoxLayout(tw)
        tl.setContentsMargins(0, 0, 0, 0)
        self.targets: dict[str, QCheckBox] = {}
        for ch, label in (("height", "depth"), ("material", "material"),
                          ("gain", "reflectivity")):
            cb = QCheckBox(label)
            cb.setToolTip(f"Write the layer's {label} channel.")
            cb.toggled.connect(self._emit)
            tl.addWidget(cb)
            self.targets[ch] = cb
        f.addRow("Writes", tw)
        lay.addWidget(g)

        self.g_geom = QGroupBox("Geometry")
        gl = QHBoxLayout(self.g_geom)
        self.geom_label = QLabel("")
        self.geom_label.setObjectName("hint")
        gl.addWidget(self.geom_label, 1)
        self.b_draw = QPushButton("Draw on map…")
        self.b_draw.setToolTip("Draw the layer's shape on the map (click the "
                               "vertices and right-click to finish, or drag).")
        self.b_draw.clicked.connect(self.draw_geometry_requested.emit)
        gl.addWidget(self.b_draw)
        lay.addWidget(self.g_geom)

        self.g_params = QGroupBox("Parameters")
        pl = QVBoxLayout(self.g_params)
        self.stack = QStackedWidget()
        self._empty = QLabel("(no parameters)")
        self._empty.setObjectName("hint")
        self.stack.addWidget(self._empty)
        pl.addWidget(self.stack)
        lay.addWidget(self.g_params)

        self.g_mask = QGroupBox("Mask (selection)")
        ml = QFormLayout(self.g_mask)
        self.mask_kind = QComboBox()
        for k in ("none",) + tuple(k for k in MASK_KINDS if k != "all"):
            self.mask_kind.addItem(MASK_KIND_LABELS.get(k, k), userData=k)
        self.mask_kind.setToolTip("Where the layer acts; a feather softens the "
                                  "edge and never crosses a wall.")
        self.mask_kind.currentIndexChanged.connect(self._on_mask_kind)
        ml.addRow("Kind", self.mask_kind)
        self.mask_feather = _spin(0.0, 100.0, 0.5, 2,
                                  "Width of the smooth edge outside the selection.")
        self.mask_feather.valueChanged.connect(self._emit)
        ml.addRow("Feather [m]", self.mask_feather)
        self.mask_invert = QCheckBox("invert")
        self.mask_invert.setToolTip("Apply the layer everywhere except the selection.")
        self.mask_invert.toggled.connect(self._emit)
        ml.addRow("", self.mask_invert)
        self.mask_width = _spin(0.1, 500.0, 0.5, 2, "Full width of the band along the line.")
        self.mask_distance = _spin(0.1, 500.0, 0.5, 2, "Distance from the walls.")
        self.mask_min = _spin(-5.0, 60.0, 0.25, 2, "Shallowest depth selected.")
        self.mask_max = _spin(-5.0, 60.0, 0.25, 2, "Deepest depth selected.")
        self.mask_reach = _spin(0.5, 500.0, 1.0, 1, "How far from a sounding the "
                                "survey still counts (0 = the source's own reach).")
        for sp in (self.mask_width, self.mask_distance, self.mask_min,
                   self.mask_max, self.mask_reach):
            sp.valueChanged.connect(self._emit)
        self._mask_rows = {"width_m": ("Band width [m]", self.mask_width),
                           "distance_m": ("Distance [m]", self.mask_distance),
                           "min_m": ("Min depth [m]", self.mask_min),
                           "max_m": ("Max depth [m]", self.mask_max),
                           "reach_m": ("Reach [m]", self.mask_reach)}
        self._mask_labels: dict[str, QLabel] = {}
        for key, (label, sp) in self._mask_rows.items():
            lbl = QLabel(label)
            ml.addRow(lbl, sp)
            self._mask_labels[key] = lbl
        mb = QWidget()
        mbl = QHBoxLayout(mb)
        mbl.setContentsMargins(0, 0, 0, 0)
        self.mask_status = QLabel("")
        self.mask_status.setObjectName("hint")
        mbl.addWidget(self.mask_status, 1)
        self.b_mask_draw = QPushButton("Draw mask…")
        self.b_mask_draw.clicked.connect(
            lambda: self.draw_mask_requested.emit(self.mask_kind.currentData()))
        mbl.addWidget(self.b_mask_draw)
        self.b_mask_clear = QPushButton("Clear")
        self.b_mask_clear.clicked.connect(self.clear_mask_requested.emit)
        mbl.addWidget(self.b_mask_clear)
        ml.addRow(mb)
        lay.addWidget(self.g_mask)
        lay.addStretch(1)
        self.show_layer(None)

    # ------------------------------------------------------------- forms
    def _form_for(self, kind: str) -> QWidget:
        if kind not in self._forms:
            spec = LAYER_KINDS[kind]
            if not spec.schema:
                return self._empty
            form = SchemaForm(list(spec.schema))
            form.edited.connect(self._emit)
            self._forms[kind] = form
            self.stack.addWidget(form)
        return self._forms[kind]

    def _emit(self, *_a) -> None:
        if not self._blocking:
            self.edited.emit()

    def _on_mask_kind(self, *_a) -> None:
        self._update_mask_rows()
        self._emit()

    def _update_mask_rows(self) -> None:
        kind = self.mask_kind.currentData()
        wanted = {"polyline_band": {"width_m"}, "distance_to_wall": {"distance_m"},
                  "depth_range": {"min_m", "max_m"},
                  "coverage": {"reach_m"}}.get(kind, set())
        for key, (_, sp) in self._mask_rows.items():
            sp.setVisible(key in wanted)
            self._mask_labels[key].setVisible(key in wanted)
        has = kind not in (None, "none")
        self.mask_feather.setEnabled(has)
        self.mask_invert.setEnabled(has)
        self.b_mask_draw.setVisible(kind in MASK_GEOMETRY)
        self.b_mask_clear.setEnabled(has)

    # ----------------------------------------------------------- show
    def show_layer(self, layer: dict[str, Any] | None) -> None:
        self._blocking = True
        try:
            self._layer = layer
            self.setEnabled(layer is not None)
            if layer is None:
                self.header.setText("")
                self.stack.setCurrentWidget(self._empty)
                return
            spec = LAYER_KINDS[layer["kind"]]
            self.header.setText(f"<b>{spec.label}</b> — {spec.consequence}<br>"
                                f"<i>{spec.reference}</i>")
            self.name.setText(str(layer.get("name", spec.label)))
            self.enabled.setChecked(bool(layer.get("enabled", True)))
            self.opacity.setValue(float(layer.get("opacity", 1.0)))
            self.blend.setCurrentText(str(layer.get("blend", spec.blend)))
            targets = dict(layer.get("targets") or {})
            for ch, cb in self.targets.items():
                cb.setVisible(ch in spec.channels)
                cb.setChecked(bool(targets.get(ch, ch in spec.channels)))
            self.g_geom.setVisible(bool(spec.geometry))
            g = layer.get("geometry")
            self.geom_label.setText(self._describe_geometry(g)
                                    if g else f"none yet — draw a "
                                              f"{' / '.join(spec.geometry)}")
            self.stack.setCurrentWidget(self._form_for(layer["kind"]))
            if layer["kind"] in self._forms:
                self._forms[layer["kind"]].set_values(dict(layer.get("params") or {}))
            m = layer.get("mask")
            m = m[0] if isinstance(m, list) and m else m
            kind = str(m["kind"]) if m else "none"
            idx = self.mask_kind.findData(kind)
            self.mask_kind.setCurrentIndex(max(idx, 0))
            self.mask_feather.setValue(float(m.get("feather_m", 0.0)) if m else 0.0)
            self.mask_invert.setChecked(bool(m.get("invert", False)) if m else False)
            if m:
                self.mask_width.setValue(float(m.get("width_m", 2.0)))
                self.mask_distance.setValue(float(m.get("distance_m", 5.0)))
                self.mask_min.setValue(float(m.get("min_m", 0.0)))
                self.mask_max.setValue(float(m.get("max_m", 10.0)))
                self.mask_reach.setValue(float(m.get("reach_m") or 15.0))
            self._update_mask_rows()
            self.mask_status.setText(self._describe_mask(m))
        finally:
            self._blocking = False

    @staticmethod
    def _describe_geometry(g: dict) -> str:
        t = g.get("type")
        if t in ("polygon", "polyline"):
            return f"{t}, {len(g['points'])} vertices"
        if t == "ellipse":
            return f"ellipse {g['rx_m']:.1f} × {g['ry_m']:.1f} m"
        if t == "rect":
            return (f"rectangle {g['x_max'] - g['x_min']:.1f} × "
                    f"{g['y_max'] - g['y_min']:.1f} m")
        if t == "axis":
            import math
            return f"axis {math.hypot(g['b'][0] - g['a'][0], g['b'][1] - g['a'][1]):.1f} m"
        if t == "radial":
            return f"radial {g['radius_m']:.1f} m"
        if t == "strokes":
            return f"{len(g['strokes'])} strokes"
        return t or ""

    @staticmethod
    def _describe_mask(m: dict | None) -> str:
        if not m:
            return "whole area"
        k = m.get("kind")
        if k in ("rect", "ellipse", "polygon", "polyline_band", "gradient"):
            return f"{MASK_KIND_LABELS.get(k, k)} drawn"
        if k == "brush":
            return f"{len(m.get('strokes') or [])} mask strokes"
        return MASK_KIND_LABELS.get(k, k)

    # ----------------------------------------------------------- read back
    def read_into(self, layer: dict[str, Any]) -> None:
        """Write every non-shape field into ``layer`` (in place)."""
        spec = LAYER_KINDS[layer["kind"]]
        layer["name"] = self.name.text().strip() or spec.label
        layer["enabled"] = bool(self.enabled.isChecked())
        layer["opacity"] = float(self.opacity.value())
        layer["blend"] = str(self.blend.currentText())
        layer["targets"] = {ch: bool(cb.isChecked()) for ch, cb in self.targets.items()
                            if ch in spec.channels}
        if layer["kind"] in self._forms:
            layer["params"] = self._forms[layer["kind"]].values()
        kind = self.mask_kind.currentData()
        old = layer.get("mask")
        old = old[0] if isinstance(old, list) and old else (dict(old) if old else {})
        if kind in (None, "none"):
            layer["mask"] = None
            return
        m: dict[str, Any] = {"kind": kind,
                             "feather_m": float(self.mask_feather.value()),
                             "invert": bool(self.mask_invert.isChecked())}
        if kind == "polyline_band":
            m["width_m"] = float(self.mask_width.value())
        elif kind == "distance_to_wall":
            m["distance_m"] = float(self.mask_distance.value())
        elif kind == "depth_range":
            m["min_m"] = float(self.mask_min.value())
            m["max_m"] = float(self.mask_max.value())
        elif kind == "coverage":
            m["reach_m"] = float(self.mask_reach.value())
        if old.get("kind") == kind:
            # Keep the drawn shape / strokes / geometry of the same kind.
            for key in ("points", "centre", "rx_m", "ry_m", "angle_deg",
                        "x_min", "y_min", "x_max", "y_max", "geometry",
                        "strokes", "profile"):
                if key in old:
                    m[key] = old[key]
        if kind in MASK_GEOMETRY and not self._mask_has_shape(m):
            layer["mask"] = None            # until the shape is drawn
            return
        layer["mask"] = m

    @staticmethod
    def _mask_has_shape(m: dict[str, Any]) -> bool:
        k = m.get("kind")
        if k in ("polygon", "polyline_band"):
            return bool(m.get("points"))
        if k == "ellipse":
            return "centre" in m
        if k == "rect":
            return "x_min" in m
        if k == "gradient":
            return bool(m.get("geometry"))
        if k == "brush":
            return bool(m.get("strokes"))
        return True

    def mask_kind_pending(self) -> str | None:
        """The mask kind chosen in the combo when it still needs a shape."""
        kind = self.mask_kind.currentData()
        if self._layer is None or kind not in MASK_GEOMETRY:
            return None
        m = self._layer.get("mask")
        m = m[0] if isinstance(m, list) and m else m
        if m and m.get("kind") == kind and self._mask_has_shape(m):
            return None
        return kind
