"""Schema-driven parameter forms.

Adapted from the MCS designer's ``panels.py`` (copied, never imported --
CM-3), extended with a ``bool`` kind. A schema is a list of
``(key, label, kind, default, min, max)`` rows with
``kind in {"float", "int", "bool", "choice:a|b|c"}`` -- the same rows the
depth presets (``DEPTH_PRESETS``) and object specs (``params_schema``)
declare, which is what lets a new preset or class appear in the GUI with
no GUI change.
"""

from __future__ import annotations

import re

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog, QDialogButtonBox,
                               QDoubleSpinBox, QFormLayout, QSpinBox,
                               QVBoxLayout, QWidget)


def _make_editor(kind: str, default, minimum, maximum):
    """(widget, getter, setter) for one schema row."""
    if kind == "int":
        w = QSpinBox()
        w.setRange(int(minimum or 0), int(maximum or 10_000))
        w.setValue(int(default))
        return w, (lambda: w.value()), (lambda v: w.setValue(int(v)))
    if kind == "bool":
        w = QCheckBox()
        w.setChecked(bool(default))
        return w, (lambda: w.isChecked()), (lambda v: w.setChecked(bool(v)))
    if kind.startswith("choice:"):
        w = QComboBox()
        w.addItems(kind.split(":", 1)[1].split("|"))
        w.setCurrentText(str(default))
        return w, (lambda: w.currentText()), (lambda v: w.setCurrentText(str(v)))
    w = QDoubleSpinBox()
    w.setDecimals(3)
    w.setRange(float(minimum if minimum is not None else -1e6),
               float(maximum if maximum is not None else 1e6))
    w.setValue(float(default))
    return w, (lambda: w.value()), (lambda v: w.setValue(float(v)))


class SchemaForm(QWidget):
    """A form auto-built from a schema; values() returns the param dict."""

    edited = Signal()

    def __init__(self, schema: list[tuple], values: dict | None = None,
                 parent=None) -> None:
        super().__init__(parent)
        layout = QFormLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        # A narrow property column wraps a long row (label above field)
        # instead of forcing the column -- and the window -- wider.
        layout.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self._getters: dict[str, callable] = {}
        self._setters: dict[str, callable] = {}
        values = values or {}
        for key, label, kind, default, minimum, maximum, *rest in schema:
            widget, getter, setter = _make_editor(
                kind, values.get(key, default), minimum, maximum)
            for sig in ("valueChanged", "currentTextChanged", "toggled"):
                if hasattr(widget, sig):
                    getattr(widget, sig).connect(lambda *_: self.edited.emit())
                    break
            layout.addRow(label, widget)
            # Optional 7th element: one-sentence help, shown as the hover
            # tooltip (and status tip) of both the editor and its label.
            help_text = str(rest[0]) if rest and rest[0] else ""
            if help_text:
                widget.setToolTip(help_text)
                widget.setStatusTip(help_text)
                lab = layout.labelForField(widget)
                if lab is not None:
                    lab.setToolTip(help_text)
            self._getters[key] = getter
            self._setters[key] = setter

    def values(self) -> dict:
        return {k: g() for k, g in self._getters.items()}

    def set_values(self, values: dict) -> None:
        """Push stored values into the editors (unknown keys ignored).
        Emits ``edited`` at most once, after every widget is set."""
        self.blockSignals(True)
        try:
            for k, v in (values or {}).items():
                if k in self._setters:
                    self._setters[k](v)
        finally:
            self.blockSignals(False)
        self.edited.emit()


class SchemaDialog(QDialog):
    """Modal parameter dialog built from a schema."""

    def __init__(self, title: str, schema: list[tuple],
                 values: dict | None = None, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle(title)
        layout = QVBoxLayout(self)
        self.form = SchemaForm(list(schema), values)
        layout.addWidget(self.form)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                                   | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def values(self) -> dict:
        return self.form.values()


_LATLON_RE = re.compile(
    r"^\s*(-?\d+(?:\.\d+)?)\s*[,;\s]\s*(-?\d+(?:\.\d+)?)\s*$")


def parse_latlon(text: str) -> tuple[float, float] | None:
    """Parse Google-Maps-style '33.660196, 130.657780' coordinates."""
    m = _LATLON_RE.match(text)
    if not m:
        return None
    lat, lon = float(m.group(1)), float(m.group(2))
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    return lat, lon
