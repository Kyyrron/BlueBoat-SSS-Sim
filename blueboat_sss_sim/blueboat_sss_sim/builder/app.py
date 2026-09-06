"""``world_builder`` entry point.

Standalone PySide6 app (no ROS): a startup chooser, then the wizard.
PySide6 is import-guarded because it has no rosdep key -- the rest of the
package must keep working without it.

Run as ``ros2 run blueboat_sss_sim world_builder`` or
``python3 -m blueboat_sss_sim.builder.app``.
"""

from __future__ import annotations

import sys


def main() -> None:
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        sys.exit(
            "world_builder needs PySide6 (not a rosdep package):\n"
            "    pip install 'PySide6>=6.5'\n"
            "The rest of blueboat_sss_sim works without it.")

    from . import theme
    from .state import BuilderState
    from .wizard import StartDialog, WorldBuilderWindow, WorldLibraryDialog

    app = QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)

    start = StartDialog()
    if not start.exec():
        return
    if start.choice == StartDialog.EDIT:
        lib = WorldLibraryDialog()
        if not lib.exec() or lib.selected_dir is None:
            return
        from ..worldgen.world_folder import load_builder_state
        try:
            state = load_builder_state(lib.selected_dir)
        except Exception as exc:
            sys.exit(f"could not load {lib.selected_dir}: {exc}")
        win = WorldBuilderWindow(state=state, edit_mode=True)
    else:
        win = WorldBuilderWindow(state=BuilderState(), edit_mode=False)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
