"""Screen 5 -- preview + confirm.

Two maps side by side (objects circled over reflectivity, and the final
depth map), rendered by the same PIL code the saved folder uses -- the
preview *is* the file the folder will store. Confirm asks for the name
and builds the folder in a worker thread; a taken name silently gets the
next ``_i`` suffix (worlds are immutable, Edit re-saves included).
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QHBoxLayout, QInputDialog, QLabel, QMessageBox,
                               QPushButton, QVBoxLayout)

from ..layout import ScaledImage
from ..state import BuilderState
from ..worker import GenerateWorker, PreviewWorker
from . import WizardPage


class PreviewPage(WizardPage):
    TITLE = "Preview & save"
    HINT = ("Left: objects (circled) with walls, limits and the GPS path. "
            "Right: the final depth map (white = shallow). Confirm "
            "generates the world folder under ~/worlds/<path>/<name>.")

    def __init__(self, on_saved=None, parent=None) -> None:
        super().__init__(parent)
        self._on_saved = on_saved
        maps = QHBoxLayout()
        self._img_obj = ScaledImage("(rendering…)", min_w=200, min_h=150)
        self._img_dep = ScaledImage("", min_w=200, min_h=150)
        for img, cap in ((self._img_obj, "objects"), (self._img_dep, "depth")):
            colw = QVBoxLayout()
            colw.addWidget(img, 1)
            lbl = QLabel(cap)
            lbl.setObjectName("hint")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            colw.addWidget(lbl)
            maps.addLayout(colw, 1)
        self._layout.addLayout(maps, 1)
        self._summary = QLabel("")
        self._summary.setObjectName("hint")
        self._layout.addWidget(self._summary)
        self._confirm = QPushButton("Confirm - create world")
        self._confirm.setDefault(True)
        self._confirm.setEnabled(False)
        self._confirm.clicked.connect(self._save)
        self._layout.addWidget(self._confirm)
        self._worker = None
        self._gen = None

    # ------------------------------------------------------------- preview
    def enter(self, state: BuilderState) -> None:
        self._state = state
        self._confirm.setEnabled(False)
        self._summary.setText("rendering preview…")
        self._img_obj.clear_image("(rendering…)")
        self._img_dep.clear_image("")
        self._worker = PreviewWorker(state)
        self._worker.ready.connect(self._show)
        self._worker.failed.connect(self._fail)
        self._worker.start()

    def _show(self, obj_png: str, dep_png: str, summary: str) -> None:
        for lbl, path in ((self._img_obj, obj_png), (self._img_dep, dep_png)):
            lbl.set_image(path)
        self._summary.setText(summary)
        self._confirm.setEnabled(True)

    def _fail(self, msg: str) -> None:
        self._summary.setText(f"preview failed: {msg}")
        QMessageBox.critical(self, "Preview", msg)

    # ---------------------------------------------------------------- save
    def _save(self) -> None:
        name, ok = QInputDialog.getText(
            self, "World name", "Name of the world:",
            text=self._state.world_name or "world")
        if not ok or not name.strip():
            return
        self._state.world_name = name.strip()
        self._confirm.setEnabled(False)
        self._summary.setText("generating world folder…")
        self._gen = GenerateWorker(self._state)
        self._gen.progressed.connect(
            lambda m: self._summary.setText(f"generating: {m}"))
        self._gen.finished_path.connect(self._saved)
        self._gen.failed.connect(self._save_failed)
        self._gen.start()

    def _saved(self, out: str) -> None:
        self._summary.setText(f"saved: {out}")
        QMessageBox.information(self, "World created",
                                f"World folder created:\n{out}")
        if self._on_saved:
            self._on_saved(out)

    def _save_failed(self, msg: str) -> None:
        self._confirm.setEnabled(True)
        self._summary.setText(f"save failed: {msg}")
        QMessageBox.critical(self, "World creation failed", msg)

    def commit(self, state: BuilderState) -> bool:
        return True
