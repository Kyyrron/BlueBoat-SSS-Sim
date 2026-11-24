"""Wizard pages (one module per screen)."""

from __future__ import annotations

from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

from ..state import BuilderState


class WizardPage(QWidget):
    """Base page: a title plus the ``enter``/``commit`` protocol the
    wizard shell drives. ``commit`` returns False to block Next."""

    TITLE = ""
    HINT = ""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._layout = QVBoxLayout(self)
        title = QLabel(self.TITLE)
        title.setObjectName("page_title")
        self._layout.addWidget(title)
        if self.HINT:
            hint = QLabel(self.HINT)
            hint.setObjectName("hint")
            hint.setWordWrap(True)
            self._layout.addWidget(hint)

    def enter(self, state: BuilderState) -> None:
        """Populate widgets from the state (pre-filled in Edit mode)."""

    def commit(self, state: BuilderState) -> bool:
        """Write widgets back into the state; False blocks Next."""
        return True
