"""Screen 4 -- objects: which classes, how many, how they scatter.

One collapsible row per ``CATALOG`` entry, generated from the catalog
itself (a class added to ``worldgen/objects.py`` appears here with no GUI
change), each with: count or per-hectare density, optional Gaussian
clustering, size and burial distributions (uniform or gauss -- nothing
fancier, per the design), an orientation rule (random / fixed heading /
relative to the survey track), and any class-specific ``params_schema``
rows. Rows collapse to their title while unticked; a text filter and a
"Selected only" toggle keep the 29-class list navigable.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog,
                               QDialogButtonBox, QDoubleSpinBox, QFormLayout,
                               QGroupBox, QHBoxLayout, QLabel, QLineEdit,
                               QPushButton, QScrollArea, QSpinBox,
                               QVBoxLayout, QWidget)

from ...worldgen.objects import (CATALOG, footprint_profile, rest_poses,
                                 rest_poses_text, shape_description)
from ..forms import SchemaForm
from ..state import BuilderState, ObjectsState
from . import WizardPage


class ClassInfoDialog(QDialog):
    """Small per-class reference card: regular size, typical shape, and a
    top-down preview of the typical object's acoustic footprint (the
    height profile the sonar actually images)."""

    def __init__(self, type_name: str, parent=None) -> None:
        super().__init__(parent)
        spec = CATALOG[type_name]
        self.setWindowTitle(f"About: {spec.display_label}")
        col = QVBoxLayout(self)

        prof, o = footprint_profile(spec)
        u8 = np.ascontiguousarray(
            np.flipud((prof * 255.0).astype(np.uint8)))
        img = QImage(u8.data, u8.shape[1], u8.shape[0], u8.shape[1],
                     QImage.Format.Format_Grayscale8).copy()
        pic = QLabel()
        pic.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pic.setPixmap(QPixmap.fromImage(img).scaled(
            240, 240, Qt.AspectRatioMode.KeepAspectRatio))
        pic.setToolTip("top-down height profile of a typical instance "
                       "(white = tall) - what the sonar images")
        col.addWidget(pic)
        cap = QLabel("typical acoustic footprint (top-down, white = tall)")
        cap.setObjectName("hint")
        cap.setAlignment(Qt.AlignmentFlag.AlignCenter)
        col.addWidget(cap)

        form = QFormLayout()
        lr, wr, hr = spec.length_range, spec.width_range, spec.height_range
        if spec.round_footprint:
            size_txt = (f"Ø {o.length:.2f} m ({lr[0]:.2f}-{lr[1]:.2f}), "
                        f"height {o.proud_height:.2f} m "
                        f"({hr[0]:.2f}-{hr[1]:.2f})")
        else:
            size_txt = (f"{o.length:.2f} × {o.width:.2f} × "
                        f"{o.proud_height:.2f} m  (L {lr[0]:.2f}-{lr[1]:.2f}, "
                        f"W {wr[0]:.2f}-{wr[1]:.2f}, "
                        f"H {hr[0]:.2f}-{hr[1]:.2f})")
        form.addRow("Regular size", QLabel(size_txt))
        shape = QLabel(shape_description(spec))
        shape.setWordWrap(True)
        form.addRow("Typical shape", shape)
        rest = QLabel(rest_poses_text(spec))
        rest.setWordWrap(True)
        rest.setToolTip("How the object can come to rest on the seabed when "
                        "'3-D rest' is on, with the odds of each pose")
        form.addRow("Rest poses", rest)
        form.addRow("Material", QLabel(spec.material))
        br = spec.burial_range
        form.addRow("Default burial", QLabel(f"{br[0]:.0%}-{br[1]:.0%} "
                                             "into the sediment"))
        col.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        buttons.accepted.connect(self.accept)
        buttons.clicked.connect(lambda *_: self.accept())
        col.addWidget(buttons)


class _DistRow(QWidget):
    """kind combo + two spins: spec default / uniform(lo, hi) / gauss(μ, σ)."""

    def __init__(self, lo_hi=(0.5, 1.5), decimals=2, parent=None) -> None:
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.kind = QComboBox()
        self.kind.addItems(["spec default", "uniform", "gauss"])
        self.a = QDoubleSpinBox()
        self.b = QDoubleSpinBox()
        for w in (self.a, self.b):
            w.setDecimals(decimals)
            w.setRange(-100.0, 100.0)
        self.a.setValue(lo_hi[0])
        self.b.setValue(lo_hi[1])
        row.addWidget(self.kind)
        row.addWidget(self.a)
        row.addWidget(self.b)
        self.kind.currentTextChanged.connect(self._relabel)
        self._relabel()

    def _relabel(self) -> None:
        gauss = self.kind.currentText() == "gauss"
        default = self.kind.currentText() == "spec default"
        self.a.setEnabled(not default)
        self.b.setEnabled(not default)
        self.a.setToolTip("mean" if gauss else "low")
        self.b.setToolTip("sigma" if gauss else "high")

    def value(self) -> dict | None:
        k = self.kind.currentText()
        if k == "spec default":
            return None
        if k == "gauss":
            return {"kind": "gauss", "mean": self.a.value(),
                    "sigma": self.b.value()}
        return {"kind": "uniform", "lo": self.a.value(), "hi": self.b.value()}

    def set_value(self, dist: dict | None) -> None:
        if not dist:
            self.kind.setCurrentText("spec default")
            return
        if dist.get("kind") == "gauss":
            self.kind.setCurrentText("gauss")
            self.a.setValue(float(dist.get("mean", 1.0)))
            self.b.setValue(float(dist.get("sigma", 0.1)))
        else:
            self.kind.setCurrentText("uniform")
            self.a.setValue(float(dist.get("lo", 0.5)))
            self.b.setValue(float(dist.get("hi", 1.5)))


class _OrientRow(QWidget):
    """mode combo + heading spin + jitter spin (``orientation`` block).

    ``random`` (the default) leaves both spins disabled; ``fixed`` reads
    the heading as an absolute ENU heading (0 = east, CCW); ``path
    relative`` reads it as an offset from the local survey-track heading
    (0 = along-track, 90 = across-track).
    """

    MODES = (("random", "random"), ("fixed", "fixed"),
             ("path relative", "path_relative"))

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.mode = QComboBox()
        self.mode.addItems([m[0] for m in self.MODES])
        self.heading = QDoubleSpinBox()
        self.heading.setRange(-180.0, 180.0)
        self.heading.setDecimals(1)
        self.heading.setSuffix("°")
        self.jitter = QDoubleSpinBox()
        self.jitter.setRange(0.0, 180.0)
        self.jitter.setDecimals(1)
        self.jitter.setSuffix("° σ")
        self.jitter.setToolTip("gaussian jitter around the heading "
                               "(0 = exact)")
        row.addWidget(self.mode)
        row.addWidget(self.heading)
        row.addWidget(self.jitter)
        self.mode.currentTextChanged.connect(self._relabel)
        self._relabel()

    def mode_key(self) -> str:
        return dict(self.MODES)[self.mode.currentText()]

    def label_text(self) -> str:
        return ("Offset from track [deg]" if self.mode_key() == "path_relative"
                else "Heading [deg]")

    def _relabel(self) -> None:
        random = self.mode_key() == "random"
        self.heading.setEnabled(not random)
        self.jitter.setEnabled(not random)
        self.heading.setToolTip(
            "offset from the local survey-track heading: 0 = along-track, "
            "90 = across-track" if self.mode_key() == "path_relative"
            else "ENU heading of the object's long axis: 0 = east, "
                 "90 = north (CCW)")

    def value(self) -> dict | None:
        """The ``orientation`` block, or None for the all-default case so
        configs written before this knob existed round-trip unchanged."""
        mode = self.mode_key()
        heading = float(self.heading.value())
        jitter = float(self.jitter.value())
        if mode == "random" and heading == 0.0 and jitter == 0.0:
            return None
        return {"mode": mode, "heading_deg": heading, "jitter_deg": jitter}

    def set_value(self, orient: dict | None) -> None:
        orient = orient or {}
        key = str(orient.get("mode", "random"))
        text = {v: k for k, v in self.MODES}.get(key, "random")
        self.mode.setCurrentText(text)
        self.heading.setValue(float(orient.get("heading_deg", 0.0)))
        self.jitter.setValue(float(orient.get("jitter_deg", 0.0)))


class _RestRow(QWidget):
    """3-D resting orientation: ``orientation.rest`` (enabled, randomness).

    Off (the default) keeps the legacy flat stamp: drums stand, pipes lie,
    every existing world is bit-identical. On, each object falls into one
    of its class's physically stable poses (standing / lying / on a face /
    on its side ...) and settles on the local seabed slope -- never
    floating, never sunk beyond its burial. ``randomness`` 0 keeps the
    canonical pose for all of them, 1 draws every pose by its odds.
    """

    def __init__(self, has_poses: bool, parent=None) -> None:
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.enabled = QCheckBox("let them fall / roll (3-D)")
        self.enabled.setToolTip(
            "Give each object a physically stable resting pose (a drum may "
            "lie on its side, a crate on any face, a cone on its flank) and "
            "settle it on the seabed slope; off keeps the flat canonical "
            "pose. Objects never float and never sink beyond their burial.")
        self.randomness = QDoubleSpinBox()
        self.randomness.setRange(0.0, 1.0)
        self.randomness.setSingleStep(0.1)
        self.randomness.setDecimals(2)
        self.randomness.setValue(0.5)
        self.randomness.setPrefix("randomness ")
        self.randomness.setToolTip(
            "0 = every object keeps its canonical pose (just settled on the "
            "slope); 1 = every object draws a pose by the class's odds; in "
            "between, that share of the objects falls over.")
        row.addWidget(self.enabled)
        row.addWidget(self.randomness)
        if not has_poses:
            self.enabled.setEnabled(False)
            self.randomness.setEnabled(False)
            self.enabled.setText("n/a (no 3-D rest poses for this shape)")
        self.enabled.toggled.connect(self.randomness.setEnabled)
        self.randomness.setEnabled(self.enabled.isChecked())

    def value(self) -> dict | None:
        if not self.enabled.isChecked():
            return None
        return {"enabled": True, "randomness": float(self.randomness.value())}

    def set_value(self, rest: dict | None) -> None:
        rest = rest or {}
        self.enabled.setChecked(bool(rest.get("enabled", False)))
        self.randomness.setValue(float(rest.get("randomness", 0.5)))


class _ClassRow(QGroupBox):
    """One catalog class. Collapsed (unchecked) it shows only the title
    checkbox, the one-line hint and **Info…**; every parameter widget
    lives inside ``_body``, which follows the checkbox."""

    def __init__(self, type_name: str, spec, parent=None) -> None:
        super().__init__(spec.display_label, parent)
        self.type_name = type_name
        self.label_text = spec.display_label
        self.setCheckable(True)
        self.setChecked(False)
        outer = QVBoxLayout(self)

        head = QHBoxLayout()
        info = (f"{spec.length_range[0]:.2f}-{spec.length_range[1]:.2f} m, "
                f"{spec.material}")
        if spec.fill_fraction_range:
            info += (f", open-frame fill "
                     f"{spec.fill_fraction_range[0]:.2f}-"
                     f"{spec.fill_fraction_range[1]:.2f}")
        lbl = QLabel(info)
        lbl.setObjectName("hint")
        head.addWidget(lbl, 1)
        b_info = QPushButton("Info…")
        b_info.setToolTip("regular size, typical shape, footprint preview")
        b_info.clicked.connect(
            lambda: ClassInfoDialog(self.type_name, self).exec())
        head.addWidget(b_info)
        outer.addLayout(head)

        self._body = QWidget()
        form = QFormLayout(self._body)
        form.setContentsMargins(0, 0, 0, 0)
        self._form = form
        outer.addWidget(self._body)

        amount = QHBoxLayout()
        self.amount_kind = QComboBox()
        self.amount_kind.addItems(["count", "per hectare"])
        self.amount_value = QDoubleSpinBox()
        self.amount_value.setRange(0.0, 10000.0)
        self.amount_value.setDecimals(1)
        self.amount_value.setValue(5.0)
        amount.addWidget(self.amount_kind)
        amount.addWidget(self.amount_value)
        w_amount = QWidget()
        w_amount.setLayout(amount)
        form.addRow("Amount", w_amount)

        cluster = QHBoxLayout()
        self.cluster_n = QSpinBox()
        self.cluster_n.setRange(0, 50)
        self.cluster_n.setToolTip("0 = uniform (no clustering)")
        self.cluster_sigma = QDoubleSpinBox()
        self.cluster_sigma.setRange(0.5, 50.0)
        self.cluster_sigma.setValue(3.0)
        self.cluster_sigma.setSuffix(" m σ")
        cluster.addWidget(self.cluster_n)
        cluster.addWidget(self.cluster_sigma)
        w_cluster = QWidget()
        w_cluster.setLayout(cluster)
        form.addRow("Clusters (0 = none)", w_cluster)

        self.size = _DistRow((0.8, 1.2))
        form.addRow("Size scale", self.size)
        self.burial = _DistRow(tuple(spec.burial_range))
        form.addRow("Burial (0-0.95)", self.burial)

        self.orient = _OrientRow()
        self._orient_label = QLabel(self.orient.label_text())
        form.addRow(self._orient_label, self.orient)
        self.orient.mode.currentTextChanged.connect(
            lambda *_: self._orient_label.setText(self.orient.label_text()))
        self.rest = _RestRow(bool(rest_poses(spec)))
        form.addRow("Resting pose", self.rest)

        self.params_form = None
        if spec.params_schema:
            self.params_form = SchemaForm(list(spec.params_schema))
            form.addRow(self.params_form)

        # Collapse: the body is hidden while the class is unticked.
        self._body.setVisible(False)
        self.toggled.connect(self._body.setVisible)

    def matches(self, text: str) -> bool:
        t = text.strip().lower()
        return (not t or t in self.label_text.lower()
                or t in self.type_name.lower())

    def to_config(self) -> dict | None:
        if not self.isChecked():
            return None
        kind = ("count" if self.amount_kind.currentText() == "count"
                else "density")
        cfg: dict = {"enabled": True,
                     "amount": {"kind": kind,
                                "value": float(self.amount_value.value())}}
        if self.cluster_n.value() > 0:
            cfg["clusters"] = {"n": int(self.cluster_n.value()),
                               "sigma_m": float(self.cluster_sigma.value())}
        if self.size.value() is not None:
            cfg["size"] = self.size.value()
        if self.burial.value() is not None:
            cfg["burial"] = self.burial.value()
        orient = self.orient.value()
        rest = self.rest.value()
        if rest is not None:
            orient = dict(orient or {"mode": "random", "heading_deg": 0.0,
                                     "jitter_deg": 0.0})
            orient["rest"] = rest
        if orient is not None:
            cfg["orientation"] = orient
        if self.params_form is not None:
            params = self.params_form.values()
            if params:
                cfg["params"] = params
        return cfg

    def from_config(self, cfg: dict | None) -> None:
        self.setChecked(bool(cfg and cfg.get("enabled", True)))
        if not cfg:
            return
        amount = cfg.get("amount", {}) or {}
        self.amount_kind.setCurrentText(
            "count" if amount.get("kind", "count") == "count"
            else "per hectare")
        self.amount_value.setValue(float(amount.get("value", 5.0)))
        clusters = cfg.get("clusters") or {}
        self.cluster_n.setValue(int(clusters.get("n", 0)))
        self.cluster_sigma.setValue(float(clusters.get("sigma_m", 3.0)))
        self.size.set_value(cfg.get("size"))
        self.burial.set_value(cfg.get("burial"))
        self.orient.set_value(cfg.get("orientation"))
        self.rest.set_value((cfg.get("orientation") or {}).get("rest"))
        if self.params_form is not None and cfg.get("params"):
            # values are restored through a rebuilt form
            new = SchemaForm(list(CATALOG[self.type_name].params_schema),
                             cfg["params"])
            self._form.replaceWidget(self.params_form, new)
            self.params_form.deleteLater()
            self.params_form = new


class ObjectsPage(WizardPage):
    TITLE = "Objects"
    HINT = ("Tick the classes to scatter, set how many and how they "
            "spread. Distributions are uniform or gaussian; clustering "
            "draws instances around random cluster centres; orientation "
            "is random, fixed, or relative to the survey track. The seed "
            "makes the whole placement reproducible.")

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        top = QHBoxLayout()
        top.addWidget(QLabel("Seed"))
        self._seed = QSpinBox()
        self._seed.setRange(0, 2_000_000_000)
        self._seed.setValue(1)
        top.addWidget(self._seed)
        top.addSpacing(16)
        self._filter = QLineEdit()
        self._filter.setPlaceholderText("filter classes…")
        self._filter.setClearButtonEnabled(True)
        top.addWidget(self._filter, 1)
        self._selected_only = QCheckBox("Selected only")
        self._selected_only.setToolTip("hide the classes that are not ticked")
        top.addWidget(self._selected_only)
        self._layout.addLayout(top)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        col = QVBoxLayout(inner)
        self._rows: dict[str, _ClassRow] = {}
        for type_name in sorted(CATALOG):
            row = _ClassRow(type_name, CATALOG[type_name])
            self._rows[type_name] = row
            row.toggled.connect(lambda *_: self._apply_filter())
            col.addWidget(row)
        col.addStretch(1)
        scroll.setWidget(inner)
        self._layout.addWidget(scroll, 1)

        self._filter.textChanged.connect(lambda *_: self._apply_filter())
        self._selected_only.toggled.connect(lambda *_: self._apply_filter())

    def _apply_filter(self) -> None:
        text = self._filter.text()
        only = self._selected_only.isChecked()
        for row in self._rows.values():
            row.setVisible(row.matches(text)
                           and (not only or row.isChecked()))

    def enter(self, state: BuilderState) -> None:
        self._seed.setValue(int(state.objects.seed))
        for type_name, row in self._rows.items():
            row.from_config(state.objects.classes.get(type_name))
        self._apply_filter()

    def commit(self, state: BuilderState) -> bool:
        classes = {}
        for type_name, row in self._rows.items():
            cfg = row.to_config()
            if cfg is not None:
                classes[type_name] = cfg
        # ``explicit`` and the pinned field belong to Screen 5; that
        # screen re-scatters when this one's signature stops matching.
        state.objects = ObjectsState(seed=int(self._seed.value()),
                                     classes=classes,
                                     explicit=state.objects.explicit,
                                     placed=state.objects.placed,
                                     placed_signature=(
                                         state.objects.placed_signature),
                                     pinned=state.objects.pinned)
        return True
