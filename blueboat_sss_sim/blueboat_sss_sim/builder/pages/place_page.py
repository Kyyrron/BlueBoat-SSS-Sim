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

**Mission-like mode (simulator v1.1, 2026-11-21).** A switch at the top of
the map gives the choice: *Manual* (this page as it always was) or
*Mission-like*, where the objects are planned by the package
``blueboat_harbour_population`` from the environment of Screens 1-4, a
harbour type, a density level and a seed (``mission_panel.py``; the
bookkeeping in ``builder/placement.py``). The plan is drawn on the map and is
not editable; *Convert to manual* turns it into ordinary pinned objects.
Nothing of the Manual mode changes: its rows, its random stream, its commit.
"""

from __future__ import annotations

import copy

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QUndoStack
from PySide6.QtWidgets import (QComboBox, QDialog, QDialogButtonBox,
                               QDoubleSpinBox, QFormLayout, QHBoxLayout,
                               QInputDialog, QLabel, QLineEdit, QListWidget,
                               QListWidgetItem, QMessageBox, QPushButton,
                               QScrollArea, QSpinBox, QStackedWidget,
                               QVBoxLayout, QWidget)

from ...core.types import PlacedObject
from ...worldgen.materials import MaterialLibrary
from ...worldgen.objects import (CATALOG, is_facet_object,
                                 make_explicit_instances, rest_poses,
                                 typical_instance)
from .. import placement
from ..forms import SchemaForm
from ..layout import ToolBar, WrapScroll, split
from ..layers.commands import SnapshotCommand
from ..map.canvas import BuilderMapView, Mode
from ..state import BuilderState, ObjectsState, scatter_signature
from ..worker import (EnvironmentWorker, PlacementWorker, PlanWorker,
                      hover_text, is_dry)
from . import WizardPage
from .objects_page import ClassInfoDialog, _DistRow, _OrientRow, _RestRow

#: map indices of the planned objects (Mission-like mode): far above any
#: row of the page, so that every row slot ignores them
PLAN_BASE = 1_000_000
#: pause between a change of a Mission-like control and the plan [ms]
PLAN_DEBOUNCE_MS = 150


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
         "How far the object stands above the seabed before burial. For a "
         "lying cylinder or bottle the scattering solid is only as high as "
         "it is wide: a larger height lengthens the shadow, not the echo."),
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
        self._form = form
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
        # The Reflectivity row only means something to the raster path: a
        # smooth solid is scattered by the facet layer from its material's
        # impedance and its geometry (NC #17). Grey it out then. The slots
        # only toggle widgets -- no scene rebuild, no modal.
        for key in ("proud_height", "burial", "roll_deg", "pitch_deg"):
            self._spins[key].valueChanged.connect(self._sync_reflectivity)
        self._material.currentTextChanged.connect(self._sync_reflectivity)
        self._sync_reflectivity()
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                                   | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        col.addWidget(buttons)

    def _is_facet(self) -> bool:
        """Would the object, as the dialog now describes it, be scattered by
        the facet layer?"""
        sp = self._spins
        roll = float(np.radians(sp["roll_deg"].value()))
        pitch = float(np.radians(sp["pitch_deg"].value()))
        tilted = abs(roll) > 1e-9 or abs(pitch) > 1e-9
        o = PlacedObject(
            object_id=0, type=str(self._item["type"]), x=0.0, y=0.0, yaw=0.0,
            length=sp["length"].value(), width=sp["width"].value(),
            proud_height=sp["proud_height"].value(),
            burial=sp["burial"].value(), reflectivity=0.5,
            material=self._material.currentText(), roll=roll, pitch=pitch,
            rest="explicit" if tilted else "canonical")
        return is_facet_object(o)

    def _sync_reflectivity(self, *_args) -> None:
        w = self._spins["reflectivity"]
        facet = self._is_facet()
        w.setEnabled(not facet)
        tip = ("Not read for this object: a smooth material on a solid "
               "primitive is rendered by the facet layer from the "
               "material's impedance and the geometry (NC #17), never from "
               "a brightness. The value is kept in the entry."
               if facet else "Acoustic reflection strength, 0-1.")
        w.setToolTip(tip)
        label = self._form.labelForField(w)
        if label is not None:
            label.setToolTip(tip)

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
    HINT_MISSION = (
        "Mission-like: the world is filled as a real harbour of the chosen "
        "type is: targets, debris, small litter and the structures it needs, "
        "where such objects lie in documented harbours. Choose the harbour "
        "type, the objects, the density and the seed on the left; the map "
        "shows the plan, coloured by class, and the quays and pontoons "
        "coloured by their use (double-click a wall to change it). Convert "
        "to manual makes every object editable.")

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
        # Mission-like mode: the left pane is a stack (the catalogue, or the
        # Mission-like controls); the map on the right is shared.
        self._mode = placement.MANUAL
        self._hp, self._sa, self._why = placement.import_planner()
        self._panel = None
        self._left = QStackedWidget()
        self._left.addWidget(left_w)
        if self._hp is not None:
            from .mission_panel import MissionPanel
            self._panel = MissionPanel(self._hp)
            scroll = WrapScroll(self._panel)
            # the wizard's dark window shows through (a scroll area's
            # viewport otherwise takes the desktop's palette)
            scroll.viewport().setAutoFillBackground(False)
            self._panel.setAutoFillBackground(False)     # setWidget() turned it on
            self._left.addWidget(scroll)
            self._panel.changed.connect(self._schedule_plan)
            self._panel.convert_requested.connect(self._convert_to_manual)
            self._panel.replan_requested.connect(self._replan)
            self._panel.heat_toggled.connect(lambda *_: self._schedule_plan())
            self._panel.profile.currentIndexChanged.connect(
                lambda *_: self._sync_feature_rows(force=True))
        self._init_mission_state()
        body.addWidget(self._left)
        body.setStretchFactor(0, 0)

        # --- right: tools + the map ---------------------------------------
        right_w = QWidget()
        right = QVBoxLayout(right_w)
        right.setContentsMargins(0, 0, 0, 0)
        body.addWidget(right_w)
        body.setStretchFactor(1, 3)
        bar = ToolBar()
        self._b_manual = QPushButton("Manual")
        self._b_mission = QPushButton("Mission-like")
        for b, tip in ((self._b_manual, "Place objects by hand, class by class "
                                        "(as always)."),
                       (self._b_mission, "Fill the world as a real harbour of a "
                                         "chosen type is filled: targets, debris, "
                                         "small litter and structures, by a "
                                         "density level and a seed.")):
            b.setCheckable(True)
            b.setToolTip(tip)
            bar.add(b)
        self._b_manual.setChecked(True)
        self._b_manual.clicked.connect(
            lambda *_: self._switch_mode(placement.MANUAL))
        self._b_mission.clicked.connect(
            lambda *_: self._switch_mode(placement.MISSION))
        if self._hp is None:
            self._b_mission.setEnabled(False)
            self._b_mission.setToolTip(
                "The package blueboat_harbour_population cannot be imported "
                f"({self._why}): build the workspace (colcon build) to use "
                "this mode.")
        bar.separator()
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
        self.map.wall_double_clicked.connect(self._on_wall_double_clicked)
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
        if mode is Mode.OBJECT and self._mode == placement.MISSION:
            mode = Mode.SELECT                  # nothing is placed by hand here
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
        if self._mode == placement.MISSION:
            return
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
        items += self._plan_items()             # none in Manual mode
        self.map.set_objects(items)
        self._sync_buttons()
        self._summary_line()

    def _summary_line(self) -> None:
        if self._mode == placement.MISSION:
            self._mission_summary()
            return
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
        elif self._env_after_preview:
            self._env_after_preview = False
            self._start_environment()

    def _show(self, result: dict) -> None:
        if self.sender() is not None and self.sender() is not self._worker:
            return                          # a superseded worker's result
        if self._mode == placement.MISSION:
            self._preview_result = result   # for a switch back to Manual
            return
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
        # What a Mission-like record needs and Screen 3 drops (no-ops for a
        # state without a record: every Manual world).
        placement.reassert(state)
        placement.restore_layers(state)
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
        if not getattr(state, "placement", None):
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
        self._init_mission_state()
        if (placement.mode_of(state) == placement.MISSION
                and self._panel is not None):
            self._enter_mission(state)
            return
        self._show_mode(placement.MANUAL)
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
        if self._mode == placement.MISSION:
            return self._commit_mission(state)
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
        self._commit_manual_record(state, placed, manual)
        return True

    # ===================================================== Mission-like mode
    # Everything below serves the Mission-like mode only; in Manual mode none
    # of it runs (the hooks above return at once), so a Manual world is built
    # from exactly the state it always was.
    def _init_mission_state(self) -> None:
        self._env = None                    # the planner's Environment
        # The workers are kept across entries: dropping the last reference
        # to a running QThread aborts the process (the rule of
        # _run_preview). A result of an earlier entry is ignored below.
        if not hasattr(self, "_env_worker"):
            self._env_worker = None
            self._plan_worker = None
        self._env_pending = bool(self._env_worker is not None and self._env_worker.isRunning())
        self._env_after_preview = False
        self._plan_pending = False
        self._plan = None                   # the last plan ...
        self._plan_block = None             # ... and the block it answers
        self._plan_heat = None
        self._stored = None                 # a plan stored in the state (Edit, Back)
        self._stash = None                  # the Manual rows while planning
        self._convert = None                # a plan converted to manual, not committed yet
        self._clear_record = False          # Manual chosen over a stored plan
        self._preview_result = None
        self._feature_key = None
        if not hasattr(self, "_plan_timer"):
            self._plan_timer = QTimer(self)
            self._plan_timer.setSingleShot(True)
            self._plan_timer.setInterval(PLAN_DEBOUNCE_MS)
            self._plan_timer.timeout.connect(self._start_plan)

    def _show_mode(self, mode: str) -> None:
        """Buttons, left pane and tools for ``mode`` (no data changes)."""
        self._mode = mode
        mission = mode == placement.MISSION
        hint = self._layout.itemAt(1).widget() if self._layout.count() > 1 else None
        if isinstance(hint, QLabel):
            hint.setText(self.HINT_MISSION if mission else self.HINT)
        self._b_manual.setChecked(not mission)
        self._b_mission.setChecked(mission)
        self._left.setCurrentIndex(1 if (mission and self._panel is not None) else 0)
        for b in (self._b_place, self._b_select, self._b_edit, self._b_del,
                  self._b_undo, self._b_redo, self._b_clear):
            b.setVisible(not mission)
        if mission:
            self._status.setText("")
            self.map.set_mode(Mode.SELECT)
            self._on_map_mode(Mode.SELECT)
        else:
            self.map.set_overlay(None, key="mission")
            self._sync_buttons()
            self._b_undo.setEnabled(self._undo.canUndo())
            self._b_redo.setEnabled(self._undo.canRedo())

    def _manual_rows(self) -> list:
        """The hand-placed rows, without the structures a plan added."""
        return [r for r in self._rows if r["kind"] == "manual"
                and not placement.is_plan_structure(r["item"])]

    # ------------------------------------------------------------ switching
    def _switch_mode(self, mode: str) -> None:
        if mode == self._mode or self._state is None:
            self._show_mode(self._mode)
            return
        if mode == placement.MISSION:
            if self._panel is None:
                self._show_mode(placement.MANUAL)
                return
            self._stash = {"rows": copy.deepcopy(self._rows), "pinned": self._field_pinned,
                           "selected": self._selected}
            self._rows = [dict(r) for r in self._manual_rows()]
            self._selected = -1
            self._undo.clear()
            self._convert = None
            self._show_mode(placement.MISSION)
            self._panel.set_result(None, busy="reading the environment…")
            self._refresh_map()
            self._start_environment()
            return
        # back to Manual: the rows as they were, the plan forgotten
        self._plan_timer.stop()
        self._env_after_preview = False
        if self._stash is not None:
            self._rows = self._stash["rows"]
            self._field_pinned = self._stash["pinned"]
            self._selected = -1
        else:
            self._rows = [dict(r) for r in self._manual_rows()]
            self._field_pinned = False
        self._clear_record = placement.mode_of(self._state) == placement.MISSION
        self._plan = self._plan_block = self._stored = None
        self._stash = None
        self._undo.clear()
        self._show_mode(placement.MANUAL)
        self._refresh_map()
        self._set_mode(Mode.OBJECT if self._armed else Mode.SELECT)
        if self._preview_result is not None:
            res, self._preview_result = self._preview_result, None
            self._show(res)
        elif not any(r["kind"] == "scatter" for r in self._rows):
            self._run_preview()

    def _enter_mission(self, state: BuilderState) -> None:
        """Screen 5 opens on a state that holds a plan (Edit mode, Back):
        the stored plan is shown as it is and is what Next keeps; the
        controls show what it was made with. Nothing is planned again unless
        a control changes or Re-plan is pressed."""
        rec = dict(state.placement)
        self._stored = rec
        self._stash = None
        self._rows = [{"kind": "manual", "item": dict(i)} for i in state.objects.explicit
                      if not placement.is_plan_structure(i)]
        self._show_mode(placement.MISSION)
        params = dict(rec.get("params") or {})
        self._panel.set_block(params)
        meta = dict(rec.get("metadata") or {})
        self._panel.set_result({"expected_shares": meta.get("weights") or {},
                                "expected": meta.get("expected") or {"total": 0, "targets": 0, "debris": 0},
                                "realised": meta.get("realised") or {"targets": 0, "debris": 0, "groups": 0},
                                "warnings": meta.get("warnings") or []}, busy="reading the environment…")
        self._refresh_map()
        self._start_environment()

    # -------------------------------------------------------- environment
    def _start_environment(self) -> None:
        if self._worker is not None and self._worker.isRunning():
            self._env_after_preview = True      # one heavy build at a time
            return
        if self._env_worker is not None and self._env_worker.isRunning():
            self._env_pending = True
            return
        self._env_pending = False
        st = copy.deepcopy(self._state)
        st.objects.explicit = [dict(r["item"]) for r in self._manual_rows()]
        self._env_worker = EnvironmentWorker(st)
        self._env_worker.ready.connect(self._on_environment)
        self._env_worker.failed.connect(self._on_environment_failed)
        self._env_worker.finished.connect(self._after_environment)
        self._env_worker.start()

    def _after_environment(self) -> None:
        if self._env_pending:
            self._start_environment()

    def _on_environment(self, env) -> None:
        if self.sender() is not None and self.sender() is not self._env_worker:
            return
        if self._env_pending:
            return                          # a newer read was asked for: it follows
        self._env = env
        self._sync_feature_rows(force=True)
        if self._mode != placement.MISSION:
            return
        # the hover readout and the dry-land check read the environment's raster
        self._preview = {"grid": {"origin_x": env.x0 + 0.5 * env.cell_m, "origin_y": env.y0 + 0.5 * env.cell_m,
                                  "resolution": env.cell_m, "nx": env.shape[1], "ny": env.shape[0]},
                         "depth": env.depth}
        if self._stored is not None and self._plan is None:
            same = env.digest() == self._stored.get("environment_digest")
            self._panel.set_stale(not same, "" if same else
                                  "The world has changed since this plan was made (Screens 2-4): its objects may "
                                  "no longer sit where the model would put them. Re-plan follows the world as it "
                                  "is; Next keeps the stored plan.")
            self._redraw_mission()
            return
        self._schedule_plan()

    def _on_environment_failed(self, msg: str) -> None:
        if self._panel is not None:
            self._panel.set_result(None, error=f"the environment could not be read: {msg}")

    def _sync_feature_rows(self, force: bool = False) -> None:
        if self._env is None or self._panel is None:
            return
        from blueboat_harbour_population.sources import default_use
        p = self._hp.default_parameters()
        prof = self._panel.profile.currentData()
        rows = [(f.id, f.kind, default_use(p, prof, f.kind)) for f in self._env.features]
        key = (prof, tuple(rows))
        if force or key != self._feature_key:
            self._feature_key = key
            self._panel.set_features(rows)

    # ---------------------------------------------------------------- plans
    def _schedule_plan(self) -> None:
        if self._mode != placement.MISSION:
            return
        if self._plan is None and self._stored is None:
            self._panel.set_result(None, busy="planning…")
        self._plan_timer.start()

    def _replan(self) -> None:
        self._schedule_plan()

    def _start_plan(self) -> None:
        if self._mode != placement.MISSION or self._env is None:
            return
        if self._plan_worker is not None and self._plan_worker.isRunning():
            self._plan_pending = True
            return
        self._plan_pending = False
        block = self._panel.block()
        self._plan_worker = PlanWorker(self._env, block, heat=self._panel.heat.isChecked())
        self._plan_worker.ready.connect(self._on_plan)
        self._plan_worker.failed.connect(self._on_plan_failed)
        self._plan_worker.finished.connect(self._after_plan)
        self._plan_worker.start()

    def _after_plan(self) -> None:
        if self._plan_pending:
            self._start_plan()

    def _on_plan(self, plan, heat, block) -> None:
        if self.sender() is not None and self.sender() is not self._plan_worker:
            return
        if self._mode != placement.MISSION or self._env is None or plan.environment_digest != self._env.digest():
            return                          # a plan of an earlier entry or mode
        self._plan, self._plan_block, self._plan_heat = plan, block, heat
        self._stored = None
        self._panel.set_stale(False)
        busy = "planning…" if (self._plan_pending or block != self._panel.block()) else ""
        self._panel.set_result(plan.report, busy=busy)
        self._redraw_mission()

    def _on_plan_failed(self, msg: str, block) -> None:
        if self.sender() is not None and self.sender() is not self._plan_worker:
            return
        self._plan = self._plan_block = None
        self._panel.set_result(None, error=msg)
        self._redraw_mission()

    # -------------------------------------------------------------- drawing
    def _plan_entries(self) -> tuple[list, list]:
        """(litter and debris, structures) on the map: the current plan's, or
        the stored one's."""
        if self._plan is not None:
            return self._plan.instances, self._plan.structures
        if self._stored is not None and self._state is not None:
            return (list(self._state.objects.placed),
                    [i for i in self._state.objects.explicit if placement.is_plan_structure(i)])
        return [], []

    def _plan_items(self) -> list:
        if self._mode != placement.MISSION:
            return []
        from blueboat_harbour_population import card
        out = []
        litter, structures = self._plan_entries()
        for k, e in enumerate(list(litter) + list(structures)):
            if e["type"] == "chain":
                continue
            spec = CATALOG[e["type"]]
            typ = typical_instance(spec)
            role = dict(e.get("params") or {}).get("role")
            colour = (card.CLASS_COLOUR_DARK.get(e["type"]) if role != "structure" else "#c9c8c2")
            out.append((PLAN_BASE + k, {
                "x": e["x"], "y": e["y"], "yaw": e.get("yaw", 0.0),
                "length": e.get("length", typ.length), "width": e.get("width", typ.width),
                "round": spec.round_footprint, "kind": "plan", "color": colour,
                "label": f"{spec.display_label} (planned)",
                "dry": is_dry(self._preview, e["x"], e["y"])}, False))
        return out

    def _redraw_mission(self) -> None:
        self._refresh_map()
        self._draw_overlay()

    def _draw_overlay(self) -> None:
        """Features coloured by their use and, when asked, the expected
        density: one raster on the map, at most 400 cells a side."""
        env = self._env
        if env is None or self._mode != placement.MISSION:
            self.map.set_overlay(None, key="mission")
            return
        from blueboat_harbour_population import card
        ny, nx = env.shape
        step = max(1, int(np.ceil(max(nx, ny) / 400.0)))
        xs, ys = env.centres()
        xs, ys = xs[::step], ys[::step]
        X, Y = np.meshgrid(xs, ys)
        rgba = np.zeros((len(ys), len(xs), 4), dtype=np.uint8)
        heat = self._plan_heat if self._panel.heat.isChecked() else None
        if heat is not None:
            h = heat[::step, ::step]
            ok = np.isfinite(h) & (h > 0)
            if ok.any():
                v = np.clip(h / max(float(np.nanpercentile(h[ok], 98)), 1e-9), 0.0, 1.0)
                lo, hi = np.array([205, 226, 251]), np.array([16, 66, 129])
                for c in range(3):
                    rgba[..., c] = np.where(ok, lo[c] + (hi[c] - lo[c]) * v, 0).astype(np.uint8)
                # low densities stay see-through: the tint grows with the density
                rgba[..., 3] = np.where(ok, 30 + 190 * v, 0).astype(np.uint8)
        uses = {}
        if self._plan is not None:
            uses = {f["id"]: f["use"] for f in self._plan.report.get("features", [])}
        elif self._stored is not None:
            uses = {f["id"]: f["use"] for f in (self._stored.get("metadata") or {}).get("features", [])}
        width = max(1.5, 0.75 * env.cell_m * step)
        for f in env.features:
            use = uses.get(f.id)
            if not use or use in ("none", "far_field"):
                continue
            near = f.distance(X, Y) <= (width if f.geometry != "point" else 2.5)
            col = card.USE_COLOUR_DARK.get(use, "#c9c8c2").lstrip("#")
            rgb = [int(col[i:i + 2], 16) for i in (0, 2, 4)]
            for c in range(3):
                rgba[..., c] = np.where(near, rgb[c], rgba[..., c])
            rgba[..., 3] = np.where(near, 230, rgba[..., 3])
        self.map.set_overlay({"rgba": rgba, "origin": (float(xs[0]), float(ys[0])),
                              "cell_m": float(env.cell_m * step)}, key="mission")

    def _mission_summary(self) -> None:
        litter, structures = self._plan_entries()
        n_t = sum(1 for e in litter if e["type"] != "debris")
        line = (f"Mission-like: {n_t} targets, {len(litter) - n_t} debris, "
                f"{sum(1 for e in structures if e['type'] != 'chain')} structures planned")
        if self._manual_rows():
            line += f" · {len(self._manual_rows())} hand-placed kept"
        if self._stored is not None and self._plan is None:
            line += " · the stored plan"
        self._summary.setText(line)

    def _on_wall_double_clicked(self, index: int) -> None:
        if self._mode != placement.MISSION or self._panel is None:
            return
        # Deferred: the signal comes from the scene's own dispatch, where a
        # dialog must not run.
        QTimer.singleShot(0, lambda: self._ask_wall_use(index))

    def _ask_wall_use(self, index: int) -> None:
        walls = self.map.walls()
        if not (0 <= index < len(walls)):
            return
        fid = str(walls[index].get("name", ""))
        if fid not in self._panel.feature_ids():
            return
        choices = self._panel.use_choices()
        use, ok = QInputDialog.getItem(self, "Use of the feature",
                                       f"What is {fid} used for?", [c.replace("_", " ") for c in choices],
                                       0, False)
        if ok:
            self._panel.set_feature_use(fid, choices[[c.replace("_", " ") for c in choices].index(use)])

    # ------------------------------------------------------------- convert
    def _convert_to_manual(self) -> None:
        """The plan becomes ordinary objects: the litter a pinned field
        (editable one by one), the structures hand-placed objects. The
        world records that it started as a Mission-like plan and, at Next,
        whether it was edited since."""
        litter, structures = self._plan_entries()
        if not litter and not structures:
            return
        if self._plan is not None:
            record = self._plan_record(self._plan)
            layers = self._plan.layers
        else:
            record = dict(self._stored or {})
            layers = record.get("layers") or []
        self._rows = ([{"kind": "pinned", "item": dict(e)} for e in litter]
                      + [dict(r) for r in self._manual_rows()]
                      + [{"kind": "manual", "item": dict(e)} for e in structures])
        self._field_pinned = True
        placed = [r["item"] for r in self._rows if r["kind"] == "pinned"]
        manual = [r["item"] for r in self._rows if r["kind"] == "manual"]
        self._convert = {"record": record, "layers": [dict(L) for L in layers],
                         "digest": placement.list_digest(placed + manual)}
        self._plan = self._plan_block = self._stored = None
        self._stash = None
        self._undo.clear()
        self._show_mode(placement.MANUAL)
        self._refresh_map()
        self._set_mode(Mode.SELECT)
        self._status.setText("converted: every planned object can now be moved, edited or deleted")

    def _plan_record(self, plan) -> dict:
        """The record a plan leaves in the state (the adapter's), computed on
        a throw-away copy of the state."""
        st = copy.deepcopy(self._state)
        st.objects.explicit = [dict(r["item"]) for r in self._manual_rows()]
        self._sa.apply_plan(st, plan, run={"sonar_fouling": self._sa.fouling_of(plan.seed)})
        return dict(st.placement)

    # --------------------------------------------------------------- commit
    def _commit_mission(self, state: BuilderState) -> bool:
        """Write the last plan (or keep the stored one). Next is refused
        while the plan on the map is not the one the controls ask for; Back
        ignores the answer, and the last finished plan is kept."""
        manual = [dict(r["item"]) for r in self._manual_rows()]
        if self._plan is not None:
            state.objects = ObjectsState(
                seed=state.objects.seed, classes=dict(state.objects.classes),
                explicit=manual, placed=[], placed_signature="", pinned=True)
            self._sa.apply_plan(state, self._plan,
                                run={"sonar_fouling": self._sa.fouling_of(self._plan.seed)})
            current = self._plan_block == self._panel.block() and not self._plan_pending
            if not current:
                self._status.setText("the plan is being updated: press Next again once it is shown")
            return bool(current)
        if self._stored is not None:
            placement.reassert(state)
            placement.restore_layers(state)
            return True
        self._status.setText("no plan yet: wait for the plan to appear on the map")
        return False

    def _commit_manual_record(self, state: BuilderState, placed: list, manual: list) -> None:
        """After the Manual commit: a plan converted on this screen leaves
        its record and its layers; Manual chosen over a stored plan clears
        it; a converted world updates ``edited``. A state that never had a
        plan is not touched."""
        rec = dict(getattr(state, "placement", None) or {})
        if self._convert is not None:
            conv = self._convert
            state.depth.layers = placement.strip_reserved(state.depth.layers) + placement.unreserve(conv["layers"])
            if conv["record"].get("seabed"):
                state.depth.seabed = placement.plain(dict(conv["record"]["seabed"]))
            new = placement.convert_record(conv["record"], edited=False)
            new["converted_digest"] = conv["digest"]
            new["layers"] = placement.unreserve(conv["layers"])
            state.placement = new
            rec = new
            self._convert = None
            self._clear_record = False
        elif self._clear_record:
            placement.clear(state)
            self._clear_record = False
            return
        if rec.get("mode") == placement.MANUAL and rec.get("origin"):
            edited = placement.list_digest(placed + manual) != rec.get("converted_digest")
            rec = placement.plain(rec)
            rec["edited"] = edited
            rec.setdefault("metadata", {})["edited"] = edited
            state.placement = rec
            placement.reassert(state)
