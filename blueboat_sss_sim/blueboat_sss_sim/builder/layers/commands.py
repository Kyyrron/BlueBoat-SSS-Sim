"""Undo commands of the layer editor: every edit is a snapshot of the
whole Screen-3 document (layers + the base brush strokes), which is small
and makes undo/redo trivially correct. Handle drags merge into one
command per layer."""

from __future__ import annotations

from typing import Any

from PySide6.QtGui import QUndoCommand

MERGE_HANDLE = 1001


class SnapshotCommand(QUndoCommand):
    def __init__(self, page, before: dict[str, Any], after: dict[str, Any],
                 text: str, merge_key: str | None = None) -> None:
        super().__init__(text)
        self._page = page
        self.before = before
        self.after = after
        self._merge_key = merge_key
        self._applied = True          # the edit already happened on push

    def id(self) -> int:
        return MERGE_HANDLE if self._merge_key else -1

    def mergeWith(self, other) -> bool:
        if (isinstance(other, SnapshotCommand) and other._merge_key
                and other._merge_key == self._merge_key):
            self.after = other.after
            return True
        return False

    def redo(self) -> None:
        if self._applied:
            self._applied = False       # the first redo() (on push) is a no-op
            return
        self._page._restore_snapshot(self.after)

    def undo(self) -> None:
        self._page._restore_snapshot(self.before)
