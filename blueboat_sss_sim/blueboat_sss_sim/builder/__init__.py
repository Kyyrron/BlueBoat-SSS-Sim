"""World-builder GUI package.

Only :mod:`.app`, :mod:`.wizard`, :mod:`.forms`, :mod:`.worker`,
:mod:`.theme`, :mod:`.map` and :mod:`.pages` import Qt; :mod:`.state` and
:mod:`.paths` are plain Python so the smoke test and headless tooling can
drive the whole build pipeline without PySide6 installed. Keep it that
way -- importing this package must never require Qt.
"""
