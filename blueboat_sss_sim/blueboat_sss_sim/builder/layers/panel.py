"""The layer list of Screen 3: three fixed rows (the depth source, the
base depth brush, the seabed texture with its material brush) and the
authored layers above them, with add / recipe / reorder / duplicate /
delete controls. The panel owns no document; the page refreshes it."""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QHBoxLayout, QListWidget, QListWidgetItem,
                               QMenu, QPushButton, QVBoxLayout, QWidget)

from ...worldgen.depth.layers import LAYER_KINDS
from .document import LayerDocument

FIXED_ROWS = (("source", "Depth source"),
              ("brush", "Depth brush (base strokes)"),
              ("texture", "Seabed texture & material brush"))
FAMILY_LABELS = {"gradient": "Gradients", "landform": "Landforms",
                 "relief": "Relief & bedforms", "adjust": "Adjustments",
                 "material": "Materials", "paint": "Brush strokes"}


class LayerPanel(QWidget):
    selected = Signal(str)             # row key: fixed key or layer id
    add_requested = Signal(str)        # layer kind
    recipe_requested = Signal(str)     # recipe name
    remove_requested = Signal(str)
    move_requested = Signal(str, int)
    duplicate_requested = Signal(str)
    toggled = Signal(str, bool)        # layer id, enabled

    def __init__(self, recipes: dict | None = None, parent=None) -> None:
        super().__init__(parent)
        self._refreshing = False
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.list = QListWidget()
        self.list.setToolTip("The seabed is composed from the bottom row up: "
                             "the depth source, the base brush strokes, the "
                             "texture, then every layer above.")
        self.list.currentItemChanged.connect(self._on_current)
        self.list.itemChanged.connect(self._on_item_changed)
        lay.addWidget(self.list, 1)
        row = QHBoxLayout()
        self.b_add = QPushButton("Add layer ▾")
        menu = QMenu(self.b_add)
        for fam, label in FAMILY_LABELS.items():
            sub = menu.addMenu(label)
            for kind, spec in LAYER_KINDS.items():
                if spec.family != fam:
                    continue
                act = sub.addAction(spec.label)
                act.setToolTip(spec.consequence)
                act.triggered.connect(lambda _=False, k=kind:
                                      self.add_requested.emit(k))
        self.b_add.setMenu(menu)
        row.addWidget(self.b_add)
        self.b_recipe = QPushButton("Recipe ▾")
        rmenu = QMenu(self.b_recipe)
        for name, rec in (recipes or {}).items():
            act = rmenu.addAction(str(rec.get("label", name)))
            act.setToolTip(f"{rec.get('consequence', '')}\n{rec.get('reference', '')}")
            act.triggered.connect(lambda _=False, n=name:
                                  self.recipe_requested.emit(n))
        self.b_recipe.setMenu(rmenu)
        self.b_recipe.setEnabled(bool(recipes))
        row.addWidget(self.b_recipe)
        row.addStretch(1)
        self.b_up = QPushButton("▲")
        self.b_down = QPushButton("▼")
        self.b_dup = QPushButton("Duplicate")
        self.b_del = QPushButton("Delete")
        for b, tip in ((self.b_up, "Move the layer up (composed later)"),
                       (self.b_down, "Move the layer down (composed earlier)"),
                       (self.b_dup, "Duplicate the layer"),
                       (self.b_del, "Delete the layer")):
            b.setToolTip(tip)
            row.addWidget(b)
        self.b_up.clicked.connect(lambda: self._emit_layer(self.move_requested, 1))
        self.b_down.clicked.connect(lambda: self._emit_layer(self.move_requested, -1))
        self.b_dup.clicked.connect(lambda: self._emit_layer(self.duplicate_requested))
        self.b_del.clicked.connect(lambda: self._emit_layer(self.remove_requested))
        lay.addLayout(row)

    # ------------------------------------------------------------ refresh
    def refresh(self, doc: LayerDocument, selected_key: str | None) -> None:
        self._refreshing = True
        try:
            self.list.clear()
            # Top of the list = top of the stack: authored layers reversed.
            for L in reversed(doc.layers):
                spec = LAYER_KINDS.get(L["kind"])
                label = spec.label if spec else L["kind"]
                op = float(L.get("opacity", 1.0))
                text = f"{L.get('name', label)}  ·  {label}  ·  {L.get('blend', 'normal')}"
                if op < 1.0:
                    text += f"  {int(round(100 * op))} %"
                it = QListWidgetItem(text)
                it.setData(Qt.ItemDataRole.UserRole, str(L["id"]))
                it.setFlags(it.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                it.setCheckState(Qt.CheckState.Checked if L.get("enabled", True)
                                 else Qt.CheckState.Unchecked)
                if spec:
                    it.setToolTip(spec.consequence + "\n" + spec.reference)
                self.list.addItem(it)
            for key, label in reversed(FIXED_ROWS):
                it = QListWidgetItem(label)
                it.setData(Qt.ItemDataRole.UserRole, key)
                it.setFlags(it.flags() & ~Qt.ItemFlag.ItemIsUserCheckable)
                self.list.addItem(it)
            self.select(selected_key or "source")
        finally:
            self._refreshing = False
        self._update_buttons()

    def select(self, key: str) -> None:
        for i in range(self.list.count()):
            it = self.list.item(i)
            if it.data(Qt.ItemDataRole.UserRole) == key:
                self.list.setCurrentItem(it)
                return
        if self.list.count():
            self.list.setCurrentRow(self.list.count() - 1)

    def current_key(self) -> str | None:
        it = self.list.currentItem()
        return None if it is None else str(it.data(Qt.ItemDataRole.UserRole))

    def is_layer(self, key: str | None) -> bool:
        return bool(key) and key not in {k for k, _ in FIXED_ROWS}

    # ------------------------------------------------------------ signals
    def _on_current(self, cur, _prev) -> None:
        self._update_buttons()
        if self._refreshing or cur is None:
            return
        self.selected.emit(str(cur.data(Qt.ItemDataRole.UserRole)))

    def _on_item_changed(self, it: QListWidgetItem) -> None:
        if self._refreshing:
            return
        key = str(it.data(Qt.ItemDataRole.UserRole))
        if self.is_layer(key):
            self.toggled.emit(key, it.checkState() == Qt.CheckState.Checked)

    def _emit_layer(self, signal, *args) -> None:
        key = self.current_key()
        if self.is_layer(key):
            signal.emit(key, *args)

    def _update_buttons(self) -> None:
        on = self.is_layer(self.current_key())
        for b in (self.b_up, self.b_down, self.b_dup, self.b_del):
            b.setEnabled(on)
