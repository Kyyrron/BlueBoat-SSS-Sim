"""Wizard shell: startup chooser, world library, and the page stack.

``QStackedWidget`` page switching (the GCS pattern -- no ``QWizard``
anywhere in this project), with Back/Next owned by the shell and each
page implementing ``enter(state)`` / ``commit(state)``. Create mode runs
Screen 1..6; Edit mode loads a saved world's ``builder_state.yaml`` and
starts at Screen 2 with everything pre-filled. Saving always creates a
new folder (silent ``_i`` dedupe) -- editing never mutates a world.
"""

from __future__ import annotations

import time

from pathlib import Path

from PySide6.QtWidgets import (QDialog, QHBoxLayout, QLabel, QListWidget,
                               QListWidgetItem, QMainWindow, QMessageBox,
                               QPushButton, QStackedWidget, QVBoxLayout,
                               QWidget)

from .layout import DEFAULT_WINDOW, PageScroll, fit_to_screen

from .pages.depth_page import DepthPage
from .pages.layout_page import LayoutPage
from .pages.objects_page import ObjectsPage
from .pages.path_page import PathPage
from .pages.place_page import PlacePage
from .pages.preview_page import PreviewPage
from .state import BuilderState, autosave_path, clear_autosave, load_autosave


class StartDialog(QDialog):
    """Create Gazebo World / Edit World."""

    CREATE, EDIT, RESUME = 1, 2, 3

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("BlueBoat World Builder")
        self.choice = 0
        col = QVBoxLayout(self)
        title = QLabel("BlueBoat World Builder")
        title.setObjectName("page_title")
        col.addWidget(title)
        col.addWidget(QLabel("GPS-anchored Gazebo worlds for the simulated "
                             "side-scan sonar."))
        row = QHBoxLayout()
        b_create = QPushButton("Create Gazebo World")
        b_edit = QPushButton("Edit World")
        b_create.setDefault(True)
        row.addWidget(b_create)
        row.addWidget(b_edit)
        col.addLayout(row)
        b_create.clicked.connect(lambda: self._pick(self.CREATE))
        b_edit.clicked.connect(lambda: self._pick(self.EDIT))
        # A session that never reached a save left an autosave behind
        # (the wizard writes one on every page change). Offer it rather
        # than making the operator redraw everything.
        self._resume = load_autosave()
        if self._resume is not None:
            when = ""
            try:
                when = time.strftime("%Y-%m-%d %H:%M", time.localtime(
                    autosave_path().stat().st_mtime))
            except OSError:
                pass
            b_res = QPushButton("Resume unsaved session"
                                + (f"  ({when})" if when else ""))
            b_res.setToolTip(
                "The World Builder did not finish saving last time. This "
                "reopens the wizard exactly where it was left, from "
                f"{autosave_path()}.")
            col.addWidget(b_res)
            b_res.clicked.connect(lambda: self._pick(self.RESUME))

    def _pick(self, choice: int) -> None:
        self.choice = choice
        self.accept()


class WorldLibraryDialog(QDialog):
    """Pick a saved world to edit/duplicate (adapted from the MCS
    designer's LibraryDialog; no Rename -- worlds are immutable)."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Saved worlds")
        self.selected_dir: Path | None = None
        col = QVBoxLayout(self)
        self._list = QListWidget()
        col.addWidget(self._list, 1)
        row = QHBoxLayout()
        b_open = QPushButton("Open")
        b_delete = QPushButton("Delete…")
        b_cancel = QPushButton("Cancel")
        b_open.setDefault(True)
        row.addWidget(b_open)
        row.addWidget(b_delete)
        row.addStretch(1)
        row.addWidget(b_cancel)
        col.addLayout(row)
        b_open.clicked.connect(self._open)
        b_delete.clicked.connect(self._delete)
        b_cancel.clicked.connect(self.reject)
        self._list.itemDoubleClicked.connect(lambda _i: self._open())
        self._reload()

    def _reload(self) -> None:
        from ..worldgen.world_folder import list_worlds
        self._list.clear()
        self._worlds = list_worlds()
        for w in self._worlds:
            meta = w["meta"]
            depth = meta.get("depth", {})
            item = QListWidgetItem(
                f"{w['path_name']} / {w['world_name']}   "
                f"({len(meta.get('objects', []))} objects, "
                f"{len(meta.get('walls', []))} walls, "
                f"{depth.get('mean_m', '?')} m mean, "
                f"{meta.get('created', '')[:10]})")
            self._list.addItem(item)

    def _current(self) -> dict | None:
        row = self._list.currentRow()
        return self._worlds[row] if 0 <= row < len(self._worlds) else None

    def _open(self) -> None:
        w = self._current()
        if w is None:
            return
        if not (w["dir"] / "builder_state.yaml").exists():
            QMessageBox.warning(
                self, "Saved worlds",
                "This world carries no builder_state.yaml (created outside "
                "the builder) and cannot be edited here.")
            return
        self.selected_dir = w["dir"]
        self.accept()

    def _delete(self) -> None:
        w = self._current()
        if w is None:
            return
        if QMessageBox.question(
                self, "Delete world",
                f"Permanently delete {w['path_name']}/{w['world_name']}?"
        ) == QMessageBox.StandardButton.Yes:
            import shutil
            shutil.rmtree(w["dir"], ignore_errors=True)
            self._reload()


class WorldBuilderWindow(QMainWindow):
    """The wizard itself."""

    def __init__(self, state: BuilderState | None = None,
                 edit_mode: bool = False, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("BlueBoat World Builder")
        # The window adapts to the screen: never wider or taller than the
        # available area, and never held open by a page's layout minimum
        # (every page rides in a scroll area below).
        self.setMinimumSize(640, 420)
        fit_to_screen(self, *DEFAULT_WINDOW)
        self.state = state or BuilderState()
        self.edit_mode = edit_mode

        self._stack = QStackedWidget()
        self.pages: list = []
        if not edit_mode:
            self.pages.append(PathPage())          # Screen 1 (Create only)
        self.pages += [LayoutPage(), DepthPage(), ObjectsPage(), PlacePage(),
                       PreviewPage(on_saved=self._on_saved)]
        for p in self.pages:
            self._stack.addWidget(PageScroll(p))

        shell = QWidget()
        col = QVBoxLayout(shell)
        col.addWidget(self._stack, 1)
        nav = QHBoxLayout()
        self._b_back = QPushButton("< Back")
        self._b_next = QPushButton("Next >")
        self._b_back.clicked.connect(self._back)
        self._b_next.clicked.connect(self._next)
        nav.addWidget(self._b_back)
        nav.addStretch(1)
        nav.addWidget(self._b_next)
        col.addLayout(nav)
        self.setCentralWidget(shell)

        self._idx = 0
        self._enter(0)

    # ------------------------------------------------------------- nav
    def _enter(self, idx: int) -> None:
        self._idx = idx
        self._stack.setCurrentIndex(idx)
        self.pages[idx].enter(self.state)
        # Crash net: the wizard state only reaches disk when a build
        # succeeds, and the process can die before that (OOM, 2026-09-18).
        self.state.autosave()
        self._b_back.setEnabled(idx > 0)
        self._b_next.setVisible(idx < len(self.pages) - 1)
        self.statusBar().showMessage(
            f"step {idx + 1}/{len(self.pages)} - {self.pages[idx].TITLE}"
            + ("   [editing: every save creates a new world]"
               if self.edit_mode else ""))

    def _next(self) -> None:
        if self.pages[self._idx].commit(self.state):
            self._enter(self._idx + 1)

    def _back(self) -> None:
        self.pages[self._idx].commit(self.state)
        self._enter(self._idx - 1)

    def _on_saved(self, out: str) -> None:
        self.statusBar().showMessage(f"saved: {out}")
        # The world now carries its own builder_state.yaml, so the crash
        # net has nothing left to protect.
        clear_autosave()
