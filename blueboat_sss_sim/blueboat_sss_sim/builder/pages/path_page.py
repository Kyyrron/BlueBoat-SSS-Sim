"""Screen 1 -- pick the MCS GPS-anchored path the world is built around.

The list is exactly the MCS launch dialog's custom-path list filtered to
GPS-anchored entries (``builder.paths``): the same directory, the same
predicate. Create mode only; Edit mode keeps the world's cached path.
"""

from __future__ import annotations

from PySide6.QtWidgets import QLabel, QListWidget, QListWidgetItem, QMessageBox

from ..paths import DEFAULT_TRAJECTORIES_DIR, list_anchored_paths
from ..state import BuilderState, PathState
from . import WizardPage


class PathPage(WizardPage):
    TITLE = "GPS path"
    HINT = ("The world anchors to one of the MCS Pattern Designer's "
            "GPS-anchored paths: its anchor becomes the world origin, and "
            "its waypoints land in world coordinates unchanged. Plain "
            "local paths are not listed - give a path a GPS origin in the "
            "designer first.")

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._list = QListWidget()
        self._layout.addWidget(self._list, 1)
        self._empty = QLabel("")
        self._empty.setObjectName("hint")
        self._layout.addWidget(self._empty)

    def enter(self, state: BuilderState) -> None:
        self._list.clear()
        self._infos = list_anchored_paths()
        for info in self._infos:
            item = QListWidgetItem(
                f"{info.name}    ({info.lat0:.6f}, {info.lon0:.6f})")
            self._list.addItem(item)
            if state.path.name == info.name:
                item.setSelected(True)
                self._list.setCurrentItem(item)
        self._empty.setText(
            "" if self._infos else
            f"No GPS-anchored paths found in {DEFAULT_TRAJECTORIES_DIR}")
        if self._infos and self._list.currentRow() < 0:
            self._list.setCurrentRow(0)

    def commit(self, state: BuilderState) -> bool:
        row = self._list.currentRow()
        if row < 0 or row >= len(self._infos):
            QMessageBox.warning(self, "GPS path",
                                "Select a GPS-anchored path first.")
            return False
        info = self._infos[row]
        try:
            new = PathState.from_mcs_file(info.file)
        except ValueError as exc:
            QMessageBox.critical(self, "GPS path", str(exc))
            return False
        if state.path.name and state.path.name != new.name:
            # A different path invalidates geometry drawn about the old one.
            state.limits_rect = None
            state.walls = []
        state.path = new
        return True
