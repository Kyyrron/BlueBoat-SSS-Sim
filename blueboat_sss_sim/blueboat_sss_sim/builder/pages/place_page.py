"""Screen 5 -- hand placement: put objects and clusters exactly where you
want them.

Screen 4 says *what* to scatter and *how much*; this screen says *where*.
The whole catalog is offered (not only the classes ticked there), each
with the same knobs -- size and burial distributions, orientation, the
3-D resting pose, the class extras -- plus the one that only makes sense
for a click: how many objects that click drops and how widely they spread.
Arm a class, then left-click the map; the armed class stays armed until
another is picked, so a cluster of clicks lays out a whole debris field.

Everything the world will contain is drawn on the map, to scale and
turned to its heading. The objects Screen 4 scattered are shown as
context in a dimmer colour: they are still *generated* from the classes
and the seed, so moving one means pinning the whole field into verbatim
instances first (one confirmation, then every object is editable
individually). Pinning is faithful -- a pinned world rebuilds the same
objects the scatter produced -- and it is what makes "move that drum two
metres left" possible at all.
"""

from __future__ import annotations

import copy

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QUndoStack
from PySide6.QtWidgets import (QComboBox, QDialog, QDialogButtonBox,
                               QDoubleSpinBox, QFormLayout, QHBoxLayout,
                               QLabel, QLineEdit, QListWidget, QListWidgetItem,
                               QMessageBox, QPushButton, QScrollArea,
                               QSpinBox, QVBoxLayout, QWidget)

from ...worldgen.materials import MaterialLibrary
from ...worldgen.objects import (CATALOG, make_explicit_instances,
                                 rest_poses, typical_instance)
from ..forms import SchemaForm
from ..layout import ToolBar, split
from ..layers.commands import SnapshotCommand
from ..map.canvas import BuilderMapView, Mode
from ..state import BuilderState, ObjectsState, scatter_signature
from ..worker import PlacementWorker, hover_text, is_dry
from . import WizardPage
from .objects_page import ClassInfoDialog, _DistRow, _OrientRow, _RestRow


class ObjectEditDialog(QDialog):
    """Every field of one placed instance, as it will be pinned.

    The values are the verbatim ones the engine uses, so this is the last
    word on an object -- nothing is re-drawn from a distribution after it.
    """

    #: (key, label, minimum, maximum, decimals, help)
    ROWS = (
        ("x", "East [m]", -1e5, 1e5, 3,
         "Position in the world's design frame, east of the GPS anchor."),
        ("y", "North [m]", -1e5, 1e5, 3,
         "Position in the world's design frame, north of the GPS anchor."),
        ("yaw_deg", "Heading [deg]", -180.0, 180.0, 1,
         "Heading of the object's long axis: 0 = east, 90 = north (CCW)."),
        ("length", "Length [m]", 0.01, 50.0, 3,
         "Size along the object's own long axis."),
        ("width", "Width [m]", 0.01, 50.0, 3,
         "Size across the long axis; a round class keeps it equal to the "
         "length."),
        ("proud_height", "Height [m]", 0.0, 20.0, 3,
         "How far the object stands above the seabed before burial."),
        ("burial", "Burial", 0.0, 0.95, 3,
         "Fraction sunk into the sediment; 0 sits proud, 0.95 is nearly "
         "gone."),
        ("fill_fraction", "Fill fraction", 0.02, 1.0, 3,
         "Open-frame classes only: how much of the interior is structure "
         "(a diluted highlight and a broken shadow)."),
        ("reflectivity", "Reflectivity", 0.05, 1.0, 3,
         "Acoustic reflection strength, 0-1."),
        ("roll_deg", "Roll [deg]", -180.0, 180.0, 2,
         "3-D resting tilt about the long axis; leave both tilts at zero "
         "for the flat stamp."),
        ("pitch_deg", "Pitch [deg]", -180.0, 180.0, 2,
         "3-D resting tilt about the short axis."),
    )

    def __init__(self, item: dict, parent=None) -> None:
        super().__init__(parent)
        spec = CATALOG[item["type"]]
        self.setWindowTitle(f"Object: {spec.display_label}")
        self._item = dict(item)
        col = QVBoxLayout(self)
        form = QFormLayout()
        self._spins: dict[str, QDoubleSpinBox] = {}
        values = dict(item)
        # Wrapped into the spin box's range: a heading drawn as 200 deg is
        # -160 deg, not clamped to 180 (which silently turned the object).
        values["yaw_deg"] = float((np.degrees(item.get("yaw", 0.0)) + 180.0)
                                  % 360.0 - 180.0)
        for key, label, lo, hi, dec, help_text in self.ROWS:
            if key == "fill_fraction" and spec.fill_fraction_range is None:
                continue
            w = QDoubleSpinBox()
            w.setRange(lo, hi)
            w.setDecimals(dec)
            w.setValue(float(values.get(key, 0.0)))
            w.setToolTip(help_text)
            form.addRow(label, w)
            self._spins[key] = w
        self._material = QComboBox()
        self._material.addItems(sorted(MaterialLibrary().names))
        self._material.setCurrentText(str(item.get("material", spec.material)))
        self._material.setToolTip(
            "The object's material. Change it *after* the reflectivity: "
            "an explicit instance keeps whatever reflectivity is pinned.")
        form.addRow("Material", self._material)
        col.addLayout(form)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                                   | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        col.addWidget(buttons)

    def item(self) -> dict:
        """The edited entry, ready for ``objects.explicit``."""
        out = dict(self._item)
        for key, w in self._spins.items():
            if key == "yaw_deg":
                out["yaw"] = float(np.radians(w.value()))
            else:
                out[key] = float(w.value())
        out["material"] = self._material.currentText()
        spec = CATALOG[out["type"]]
        if spec.round_footprint:
            out["width"] = out["length"]
        # Writing a tilt at all sends the instance down the 3-D solid
        # path; drop both keys again when it is flat, so a flat object
        # keeps the 2.5-D stamp it was drawn with.
        roll, pitch = out.pop("roll_deg", 0.0), out.pop("pitch_deg", 0.0)
        out.pop("rest", None)
        if abs(roll) > 1e-9 or abs(pitch) > 1e-9:
            out["roll_deg"], out["pitch_deg"] = roll, pitch
            out["rest"] = str(self._item.get("rest", "explicit"))
        return out


class _StampForm(QWidget):
    """The armed class's settings: what one click drops.

    The same widgets as Screen 4 (`_DistRow`, `_OrientRow`, `_RestRow`,
    `SchemaForm`), with the class's amount replaced by the two knobs a
    click needs -- how many objects it drops and how far they spread.
    """

    def __init__(self, type_name: str, parent=None) -> None:
        super().__init__(parent)
        spec = CATALOG[type_name]
        self.type_name = type_name
        form = QFormLayout(self)
        form.setContentsMargins(0, 0, 0, 0)

        cluster = QHBoxLayout()
        self.count = QSpinBox()
        self.count.setRange(1, 200)
        self.count.setValue(1)
        self.count.setToolTip(
            "How many objects one click drops. More than one makes the "
            "click a cluster centre.")
        self.sigma = QDoubleSpinBox()
        self.sigma.setRange(0.0, 50.0)
        self.sigma.setValue(3.0)
        self.sigma.setSuffix(" m σ")
        self.sigma.setToolTip(
            "How widely a cluster spreads about the click; the first "
            "object always lands exactly on it.")
        cluster.addWidget(self.count)
        cluster.addWidget(self.sigma)
        w_cluster = QWidget()
        w_cluster.setLayout(cluster)
        form.addRow("Per click", w_cluster)
        self.count.valueChanged.connect(
            lambda n: self.sigma.setEnabled(n > 1))
        self.sigma.setEnabled(False)

        self.size = _DistRow((0.8, 1.2))
        self.size.setToolTip("Size scale drawn per object, about the "
                             "class's regular size.")
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
        self.material = QComboBox()
        self.material.addItems(sorted(MaterialLibrary().names))
        self.material.setCurrentText(spec.material)
        self.material.setToolTip("Material of the objects this click drops.")
        form.addRow("Material", self.material)
        self.params_form = None
        if spec.params_schema:
            self.params_form = SchemaForm(list(spec.params_schema))
            form.addRow(self.params_form)

    def prefill(self, cfg: dict | None) -> None:
        """Adopt the class's Screen-4 settings, when it was configured
        there: hand-placing a class already in the scatter should not mean
        re-typing its distributions."""
        if not cfg:
            return
        clusters = cfg.get("clusters") or {}
        if clusters.get("sigma_m"):
            self.sigma.setValue(float(clusters["sigma_m"]))
        self.size.set_value(cfg.get("size"))
        self.burial.set_value(cfg.get("burial"))
        self.orient.set_value(cfg.get("orientation"))
        self.rest.set_value((cfg.get("orientation") or {}).get("rest"))
        if cfg.get("material"):
            self.material.setCurrentText(str(cfg["material"]))

    def stamp(self) -> dict:
        """Keyword arguments for ``make_explicit_instances``."""
        orient = self.orient.value() or {"mode": "random", "heading_deg": 0.0,
                                         "jitter_deg": 0.0}
        rest = self.rest.value()
        if rest is not None:
            orient = dict(orient)
            orient["rest"] = rest
        params = self.params_form.values() if self.params_form else None
        return {"count": int(self.count.value()),
                "sigma_m": float(self.sigma.value()),
                "size": self.size.value(), "burial": self.burial.value(),
                "orientation": orient, "material": self.material.currentText(),
                "params": params or None}


class PlacePage(WizardPage):
    TITLE = "Place objects by hand"
    HINT = ("Left: every class in the catalog. Pick one, set what a click "
            "drops (one object or a cluster, its size, burial, heading and "
            "resting pose), then left-click the map — the class stays armed "
            "until you pick another. Right: everything the world will "
            "contain, to scale. Select an object to move or turn it, "
            "double-click to edit every value, Delete to remove it. "
            "Objects scattered by the previous screen are dimmer: moving "
            "one pins that random field so each object becomes editable.")

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._state: BuilderState | None = None
        self._preview: dict | None = None
        self._worker: PlacementWorker | None = None
        #: Rows mirroring the worker's: {"kind", "item"}. The screen's
        #: document -- ``commit`` splits it back into placed/explicit.
        self._rows: list[dict] = []
        self._selected = -1
        self._armed: str | None = None
        self._form: _StampForm | None = None
        self._rng = np.random.default_rng(0)
        self._seed_seen: int | None = None
        self._undo = QUndoStack(self)
        #: Snapshot taken at the first live move of a handle gesture; the
        #: undo step is pushed once, on release.
        self._drag_before: dict | None = None
        #: The random field was pinned on this screen (survives deleting
        #: every pinned object: the classes must not start generating
        #: again).
        self._field_pinned = False
        self._pending = False

        body = split()
        self._layout.addWidget(body, 1)

        # --- left: the catalog + the armed class's settings ---------------
        left_w = QWidget()
        left = QVBoxLayout(left_w)
        left.setContentsMargins(0, 0, 6, 0)
        self._filter = QLineEdit()
        self._filter.setPlaceholderText("filter classes…")
        self._filter.setClearButtonEnabled(True)
        self._filter.textChanged.connect(lambda *_: self._apply_filter())
        left.addWidget(self._filter)
        self._list = QListWidget()
        self._list.setToolTip("Every class the simulator can place. Pick "
                              "one to arm it, then click the map.")
        for type_name in sorted(CATALOG, key=lambda t: CATALOG[t].display_label):
            spec = CATALOG[type_name]
            item = QListWidgetItem(spec.display_label)
            item.setData(Qt.ItemDataRole.UserRole, type_name)
            item.setToolTip(f"{spec.length_range[0]:.2f}–"
                            f"{spec.length_range[1]:.2f} m, {spec.material}")
            self._list.addItem(item)
        self._list.currentItemChanged.connect(lambda *_: self._on_arm())
        left.addWidget(self._list, 1)
        row = QHBoxLayout()
        self._b_info = QPushButton("Info…")
        self._b_info.setToolTip("regular size, typical shape, footprint "
                                "preview")
        self._b_info.setEnabled(False)          # until a class is armed
        self._b_info.clicked.connect(self._show_info)
        row.addWidget(self._b_info)
        row.addStretch(1)
        left.addLayout(row)
        self._form_scroll = QScrollArea()
        self._form_scroll.setWidgetResizable(True)
        left.addWidget(self._form_scroll, 1)
        left_w.setMinimumWidth(240)
        body.addWidget(left_w)
        body.setStretchFactor(0, 0)

        # --- right: tools + the map ---------------------------------------
        right_w = QWidget()
        right = QVBoxLayout(right_w)
        right.setContentsMargins(0, 0, 0, 0)
        body.addWidget(right_w)
        body.setStretchFactor(1, 3)
        bar = ToolBar()
        self._b_place = QPushButton("Place")
        self._b_select = QPushButton("Select")
        for b, tip in ((self._b_place, "Left-click the map to drop the armed "
                                       "class."),
                       (self._b_select, "Select an object to move, turn or "
                                        "edit it.")):
            b.setCheckable(True)
            b.setToolTip(tip)
            bar.add(b)
        self._b_place.clicked.connect(lambda *_: self._set_mode(Mode.OBJECT))
        self._b_select.clicked.connect(lambda *_: self._set_mode(Mode.SELECT))
        bar.separator()
        self._b_edit = QPushButton("Edit…")
        self._b_edit.setToolTip("Every value of the selected object.")
        self._b_edit.clicked.connect(lambda *_: self._edit(self._selected))
        self._b_del = QPushButton("Delete")
        self._b_del.setToolTip("Remove the selected object.")
        self._b_del.clicked.connect(self._delete_selected)
        self._b_undo = QPushButton("Undo")
        self._b_redo = QPushButton("Redo")
        self._b_undo.clicked.connect(self._undo.undo)
        self._b_redo.clicked.connect(self._undo.redo)
        self._undo.canUndoChanged.connect(self._b_undo.setEnabled)
        self._undo.canRedoChanged.connect(self._b_redo.setEnabled)
        self._b_undo.setEnabled(False)
        self._b_redo.setEnabled(False)
        bar.add(self._b_edit, self._b_del, self._b_undo, self._b_redo)
        bar.separator()
        self._b_clear = QPushButton("Clear hand-placed")
        self._b_clear.setToolTip("Remove every object placed on this screen "
                                 "(the scatter is untouched).")
        self._b_clear.clicked.connect(self._clear_manual)
        bar.add(self._b_clear)
        right.addWidget(bar)

        self.map = BuilderMapView()
        self.map.set_geometry_editable(False)     # walls/limits are context
        self.map.object_clicked.connect(self._on_click)
        self.map.object_selected.connect(self._on_select)
        self.map.object_moved.connect(self._on_moved)
        self.map.object_drag_finished.connect(self._on_drag_finished)
        self.map.object_double_clicked.connect(self._edit)
        self.map.mode_changed.connect(self._on_map_mode)
        self.map.status.connect(lambda m: self._status.setText(m))
        self.map.hover.connect(self._on_hover)
        self.map.hover_left.connect(lambda: self._hover.setText(""))
        right.addWidget(self.map, 1)
        self._status = QLabel("")
        self._status.setObjectName("hint")
        right.addWidget(self._status)
        self._hover = QLabel("")
        self._hover.setObjectName("hint")
        self._hover.setToolTip("Depth of the seabed under the cursor "
                               "(the Screen-3 preview).")
        right.addWidget(self._hover)
        self._summary = QLabel("")
        self._summary.setObjectName("hint")
        self._summary.setWordWrap(True)
        right.addWidget(self._summary)

        self._sync_buttons()

    # ----------------------------------------------------------- catalog
    def _apply_filter(self) -> None:
        text = self._filter.text().strip().lower()
        for i in range(self._list.count()):
            it = self._list.item(i)
            type_name = it.data(Qt.ItemDataRole.UserRole)
            it.setHidden(bool(text) and text not in it.text().lower()
                         and text not in type_name.lower())
        cur = self._list.currentItem()
        if cur is not None and cur.isHidden():
            # The armed class vanished from the list: disarm, so the map
            # never places something the operator can no longer see.
            self._list.setCurrentItem(None)
            self._armed, self._form = None, None
            self._form_scroll.takeWidget()
            self._b_info.setEnabled(False)
            self._set_mode(Mode.SELECT)

    def _on_arm(self) -> None:
        it = self._list.currentItem()
        if it is None:
            return
        self._armed = it.data(Qt.ItemDataRole.UserRole)
        form = _StampForm(self._armed)
        if self._state is not None:
            form.prefill(self._state.objects.classes.get(self._armed))
        self._form = form
        self._form_scroll.setWidget(form)
        self._b_info.setEnabled(True)
        self._set_mode(Mode.OBJECT)

    def _show_info(self) -> None:
        if self._armed:
            ClassInfoDialog(self._armed, self).exec()

    # ------------------------------------------------------------- tools
    def _set_mode(self, mode: Mode) -> None:
        if mode is Mode.OBJECT and not self._armed:
            self._on_map_mode(self.map.mode)     # the button follows the map
            return
        self.map.set_mode(mode)
        self._on_map_mode(mode)

    def _on_map_mode(self, mode) -> None:
        """The Place / Select buttons show the map's real mode (a
        right-click on the map drops back to SELECT on its own)."""
        self._b_place.setChecked(mode is Mode.OBJECT)
        self._b_select.setChecked(mode is Mode.SELECT)

    def _sync_buttons(self) -> None:
        editable = 0 <= self._selected < len(self._rows) \
            and self._rows[self._selected]["kind"] != "scatter"
        self._b_edit.setEnabled(editable)
        self._b_del.setEnabled(editable)

    # -------------------------------------------------------- the document
    def _snapshot(self) -> dict:
        return {"rows": copy.deepcopy(self._rows), "selected": self._selected}

    def _push(self, text: str, before: dict,
              merge_key: str | None = None) -> None:
        after = self._snapshot()
        if after == before:
            return
        self._undo.push(SnapshotCommand(self, before, after, text, merge_key))

    def _restore_snapshot(self, snap: dict) -> None:
        self._rows = copy.deepcopy(snap["rows"])
        self._selected = int(snap["selected"])
        if not (0 <= self._selected < len(self._rows)):
            self._selected = -1
        self._refresh_map()

    # ------------------------------------------------------------ pinning
    def _pinned(self) -> bool:
        return any(r["kind"] == "pinned" for r in self._rows)

    def _pin_scatter(self, reason: str) -> bool:
        """Freeze the random field into verbatim instances.

        Screen 4's classes *generate* their objects at build time, so an
        individual one cannot be moved while they do. Pinning writes each
        one out with every value it was drawn with -- the rebuilt world
        contains the same objects -- and the classes stop generating.
        """
        if self._pinned() or not any(r["kind"] == "scatter"
                                     for r in self._rows):
            return True
        n = sum(1 for r in self._rows if r["kind"] == "scatter")
        if QMessageBox.question(
                self, "Pin the scattered objects",
                f"{reason}\n\nThe previous screen scatters {n} objects from "
                "its classes and the seed, so an individual one cannot move "
                "while they do.\n\nPinning writes all "
                f"{n} out as fixed instances — the same objects, in the same "
                "places — and each one then becomes editable here. The "
                "classes stay recorded on Screen 4 but stop generating.\n\n"
                "Pin them now?") != QMessageBox.StandardButton.Yes:
            return False
        for r in self._rows:
            if r["kind"] == "scatter":
                r["kind"] = "pinned"
        self._field_pinned = True
        return True

    # ------------------------------------------------------------- edits
    def _on_click(self, point) -> None:
        if not self._armed or self._form is None:
            return
        before = self._snapshot()
        x, y = float(point.x()), float(point.y())
        stamp = self._form.stamp()
        try:
            items = make_explicit_instances(
                self._armed, x, y, self._rng,
                path_xy=(self._state.path.xy() if self._state else None),
                **stamp)
        except Exception as exc:                        # noqa: BLE001
            QMessageBox.warning(self, "Place objects", str(exc))
            return
        for item in items:
            self._rows.append({"kind": "manual", "item": item})
        self._selected = len(self._rows) - 1
        dry = sum(1 for i in items if is_dry(self._preview, i["x"], i["y"]))
        r = self._state.limits_rect if self._state else None
        outside = sum(1 for i in items if r and not (
            r["x_min"] <= i["x"] <= r["x_max"]
            and r["y_min"] <= i["y"] <= r["y_max"]))
        self._push(f"place {len(items)} × {self._armed}", before)
        self._refresh_map()
        msg = (f"placed {len(items)} × {CATALOG[self._armed].display_label} "
               f"at {x:.1f}, {y:.1f} m")
        if dry:
            msg += (f"  ⚠ {dry} on land — the sonar sees nothing there "
                    "(NC #13)")
        if outside:
            msg += (f"  ⚠ {outside} outside the world limits — the sonar "
                    "never passes there")
        self._status.setText(msg)

    def _on_select(self, index: int) -> None:
        self._selected = int(index)
        self._sync_buttons()
        self._refresh_map(keep_selection=True)

    def _on_moved(self, index: int, pose: dict) -> None:
        """Live, on every mouse move of a handle: the row follows the
        footprint. Nothing else happens here -- this runs inside the
        handle's own itemChange, where a rebuild or a dialog would destroy
        the item Qt is executing on. A scattered object is left alone until
        the release asks about pinning."""
        if not (0 <= index < len(self._rows)):
            return
        row = self._rows[index]
        if row["kind"] == "scatter":
            return
        if self._drag_before is None:
            self._drag_before = self._snapshot()
        row["item"].update({"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]})
        self._summary_line()

    def _on_drag_finished(self, index: int, pose: dict) -> None:
        """The handle was released: one undo step per gesture, or -- for a
        scattered object -- the pin question, on the event loop."""
        if not (0 <= index < len(self._rows)):
            self._drag_before = None
            return
        if self._rows[index]["kind"] == "scatter":
            QTimer.singleShot(0, lambda: self._ask_pin_after_drag(index, pose))
            return
        before, self._drag_before = self._drag_before, None
        if before is None:
            before = self._snapshot()
        self._rows[index]["item"].update(
            {"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]})
        self._push("move object", before)
        self._refresh_map(keep_selection=True)

    def _ask_pin_after_drag(self, index: int, pose: dict) -> None:
        if not (0 <= index < len(self._rows)):
            return
        before = self._snapshot()
        if not self._pin_scatter("You moved a scattered object."):
            self._refresh_map()         # snap it back
            return
        self._rows[index]["item"].update(
            {"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]})
        self._selected = index
        self._push("pin the scattered field", before)
        self._refresh_map()

    def _edit(self, index: int) -> None:
        # Deferred: the request may come from a double-click inside the
        # scene's own dispatch, where a modal dialog must not run.
        QTimer.singleShot(0, lambda: self._edit_now(index))

    def _edit_now(self, index: int) -> None:
        if not (0 <= index < len(self._rows)):
            return
        row = self._rows[index]
        before = self._snapshot()
        if row["kind"] == "scatter":
            if not self._pin_scatter("You edited a scattered object."):
                return
            self._refresh_map()         # the field is pinned even on Cancel
        dlg = ObjectEditDialog(row["item"], self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            if row["kind"] == "pinned":
                self._push("pin the scattered field", before)
            return
        row["item"] = dlg.item()
        self._selected = index
        self._push("edit object", before)
        self._refresh_map()

    def _delete_selected(self) -> None:
        i = self._selected
        if not (0 <= i < len(self._rows)):
            return
        before = self._snapshot()
        if self._rows[i]["kind"] == "scatter" and not self._pin_scatter(
                "You deleted a scattered object."):
            return
        del self._rows[i]
        self._selected = -1
        self._push("delete object", before)
        self._refresh_map()

    def _clear_manual(self) -> None:
        if not any(r["kind"] == "manual" for r in self._rows):
            return
        before = self._snapshot()
        self._rows = [r for r in self._rows if r["kind"] != "manual"]
        self._selected = -1
        self._push("clear hand-placed objects", before)
        self._refresh_map()

    def keyPressEvent(self, event) -> None:
        if event.key() in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
            self._delete_selected()
            return
        super().keyPressEvent(event)

    # ------------------------------------------------------------ drawing
    def _refresh_map(self, keep_selection: bool = False) -> None:
        if not (0 <= self._selected < len(self._rows)):
            self._selected = -1
        items = []
        for i, r in enumerate(self._rows):
            item, spec = r["item"], CATALOG[r["item"]["type"]]
            typ = typical_instance(spec)
            items.append((i, {
                "x": item["x"], "y": item["y"], "yaw": item.get("yaw", 0.0),
                "length": item.get("length", typ.length),
                "width": item.get("width", typ.width),
                "round": spec.round_footprint, "kind": r["kind"],
                "label": spec.display_label,
                "dry": is_dry(self._preview, item["x"], item["y"]),
            }, i == self._selected))
        self.map.set_objects(items)
        self._sync_buttons()
        self._summary_line()

    def _summary_line(self) -> None:
        kinds = {"scatter": 0, "pinned": 0, "manual": 0}
        dry = 0
        for r in self._rows:
            kinds[r["kind"]] += 1
            if is_dry(self._preview, r["item"]["x"], r["item"]["y"]):
                dry += 1
        parts = [f"{len(self._rows)} objects"]
        if kinds["scatter"]:
            parts.append(f"{kinds['scatter']} scattered (not yet editable)")
        if kinds["pinned"]:
            parts.append(f"{kinds['pinned']} pinned")
        if kinds["manual"]:
            parts.append(f"{kinds['manual']} hand-placed")
        line = " · ".join(parts)
        if dry:
            line += f"  ⚠ {dry} on land"
        self._summary.setText(line)

    def _on_hover(self, x: float, y: float) -> None:
        anchor = self._state.path.anchor if self._state is not None else None
        self._hover.setText(hover_text(self._preview, x, y, anchor))

    # ------------------------------------------------------------- worker
    def _run_preview(self) -> None:
        if self._worker is not None and self._worker.isRunning():
            # Never replace a running QThread (Qt aborts the process when
            # one is destroyed mid-run): finish it, then run again.
            self._pending = True
            return
        self._pending = False
        o = self._state.objects
        scattering = not (o.placed or o.pinned) and any(
            bool(c.get("enabled", True)) for c in o.classes.values())
        activity = ("placing the scattered objects…" if scattering
                    else "computing the depth preview…")
        self._summary.setText(f"{self._summary.text()} · {activity}"
                              if self._rows else activity)
        # A copy: Back / Next commit into the live state while the worker
        # still reads it (the bake alone runs tens of seconds).
        self._worker = PlacementWorker(copy.deepcopy(self._state))
        self._worker.ready.connect(self._show)
        self._worker.failed.connect(self._fail)
        self._worker.finished.connect(self._after_worker)
        self._worker.start()

    def _after_worker(self) -> None:
        if self._pending:
            self._run_preview()

    def _show(self, result: dict) -> None:
        if self.sender() is not None and self.sender() is not self._worker:
            return                          # a superseded worker's result
        self._preview = result
        # The worker brings the field Screen 4 scatters (and the depth
        # overlay). The pinned and hand-placed rows stay this page's own --
        # seeded from the state on entry, edited since -- so a move, an edit
        # or a click made while the worker ran survives its result.
        scattered = [{"kind": "scatter", "item": dict(r["item"])}
                     for r in result["objects"] if r["kind"] == "scatter"]
        own = [r for r in self._rows if r["kind"] != "scatter"]
        sel = self._selected
        keep = 0 <= sel < len(self._rows) and self._rows[sel]["kind"] != "scatter"
        own_sel = sum(1 for r in self._rows[:sel] if r["kind"] != "scatter")
        self._rows = scattered + own
        self._selected = len(scattered) + own_sel if keep else -1
        self._undo.clear()
        g, depth = result["grid"], result["depth"]
        lo, hi = float(depth.min()), float(depth.max())
        span = (hi - lo) or 1.0
        self.map.set_overlay({"values": 1.0 - (depth - lo) / span,
                              "origin": (g["origin_x"], g["origin_y"]),
                              "cell_m": g["resolution"]}, key="composite")
        self._refresh_map()
        self._status.setText(result["summary"])

    def _fail(self, msg: str) -> None:
        self._summary.setText(f"preview failed: {msg}")
        QMessageBox.critical(self, "Objects", msg)

    # --------------------------------------------------------- wizard API
    def enter(self, state: BuilderState) -> None:
        self._state = state
        if self._seed_seen != int(state.objects.seed):
            # Reseeded only when the seed changed: leaving and returning
            # must not replay the same random draws for the next clicks.
            self._rng = np.random.default_rng([5, int(state.objects.seed)])
            self._seed_seen = int(state.objects.seed)
        self._field_pinned = bool(state.objects.pinned) or bool(state.objects.placed)
        self.map.set_anchor(state.path.anchor)
        self.map.set_path(state.path.xy())
        self.map.set_limits(state.limits_rect)
        self.map.set_walls(state.walls)
        r = state.limits_rect
        if r:
            self.map.center_on_bounds([r["x_min"], r["x_max"]],
                                      [r["y_min"], r["y_max"]])
        if self._armed and self._form is not None:
            self._form.prefill(state.objects.classes.get(self._armed))
        self._check_stale(state)
        # What the state already holds is shown at once and is this page's
        # document from now on (Edit mode, a resumed autosave, a return
        # here); the worker only adds the scattered field. Until 2026-09-28
        # the list stayed empty until the worker returned -- tens of seconds
        # on a layered or recording world -- and a Back or Next meanwhile
        # committed that empty list: the world lost every object.
        self._rows = ([{"kind": "pinned", "item": dict(i)}
                       for i in state.objects.placed]
                      + [{"kind": "manual", "item": dict(i)}
                         for i in state.objects.explicit])
        self._selected = -1
        self._undo.clear()
        self._refresh_map()
        self._set_mode(Mode.OBJECT if self._armed else Mode.SELECT)
        self._run_preview()

    def _check_stale(self, state: BuilderState) -> None:
        """A pinned field that no longer matches Screen 4.

        The signature records the seed and classes that produced the pin;
        once they change, the pinned objects are a snapshot of settings
        that no longer exist. Which one wins is the operator's call.
        """
        if not state.objects.placed:
            return
        now = scatter_signature(state.objects.seed, state.objects.classes)
        if now == state.objects.placed_signature:
            return
        box = QMessageBox(self)
        box.setWindowTitle("The scatter settings changed")
        box.setText(
            f"The previous screen has changed since these "
            f"{len(state.objects.placed)} objects were pinned.\n\n"
            "Re-scatter to follow the new settings (any edits you made to "
            "the pinned objects are lost), or keep the objects you "
            "arranged and ignore the change.")
        b_new = box.addButton("Re-scatter", QMessageBox.ButtonRole.DestructiveRole)
        box.addButton("Keep the pinned objects", QMessageBox.ButtonRole.AcceptRole)
        box.exec()
        if box.clickedButton() is b_new:
            state.objects = ObjectsState(
                seed=state.objects.seed, classes=dict(state.objects.classes),
                explicit=list(state.objects.explicit))
            self._field_pinned = False
        else:
            state.objects.placed_signature = now

    def commit(self, state: BuilderState) -> bool:
        placed = [r["item"] for r in self._rows if r["kind"] == "pinned"]
        manual = [r["item"] for r in self._rows if r["kind"] == "manual"]
        pinned = self._field_pinned or bool(placed)
        state.objects = ObjectsState(
            seed=state.objects.seed, classes=dict(state.objects.classes),
            explicit=manual, placed=placed,
            placed_signature=(scatter_signature(state.objects.seed,
                                                state.objects.classes)
                              if pinned else ""),
            pinned=pinned)
        return True
