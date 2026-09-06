"""Screen 2 -- world limits + reflective walls, drawn on the map.

The rectangle becomes the world extent and four invisible collision-only
borders (never acoustic). Each wall line is a ``Wall``: it reflects SSS
pings through the mirror-source model and blocks the boat in Gazebo;
double-click one to edit its name / top height / reflectivity (default
0.65, a concrete quay).
"""

from __future__ import annotations

from PySide6.QtWidgets import (QHBoxLayout, QLabel, QMessageBox, QPushButton,
                               QWidget)

from ..forms import SchemaDialog
from ..map.canvas import BuilderMapView, Mode
from ..state import BuilderState
from . import WizardPage

_WALL_SCHEMA = (
    ("kind", "Kind", "choice:quay|pontoon", "quay", None, None,
     "Quay: a face from the seabed to the top that blocks sonar and the boat; "
     "pontoon: a floating structure the sonar looks under (only its draft "
     "reflects, no shadow behind it)."),
    ("top_z", "Top height above water [m]", "float", 1.0, -5.0, 10.0,
     "World z of the top edge; negative makes a submerged sill that rays "
     "above it look over."),
    ("draft_m", "Draft (pontoon) [m]", "float", 0.4, 0.05, 5.0,
     "How deep a pontoon's floats reach below the waterline; ignored for a quay."),
    ("reflectivity", "Reflectivity (concrete ~0.65)", "float", 0.65, 0.0, 1.0,
     "Energy fraction the face returns per bounce; mirror ghosts carry its square."),
    ("roughness_m", "Face roughness [m]", "float", 0.003, 0.0, 0.1,
     "RMS roughness of the face: at 450 kHz a few millimetres already turn "
     "most of the mirror reflection into faint diffuse scatter."),
    ("thickness", "Thickness [m]", "float", 0.30, 0.05, 5.0,
     "Visual and collision thickness in Gazebo; the sonar treats the face as a plane."),
)


class LayoutPage(WizardPage):
    TITLE = "World limits & walls"
    HINT = ("Draw one rectangle: the world limits (invisible, impassable, "
            "never reflect sonar). Add wall lines for quays and pontoons: "
            "quays reflect SSS pings, shadow the seabed behind them and block "
            "the boat; pontoons float, so the sonar looks under them (set the "
            "kind in a wall's properties). Endpoints within 0.75 m of another "
            "wall's end snap to it and merge in the built world; select a wall "
            "and drag its hollow middle handle to bend it (Straighten undoes). "
            "Drag handles to adjust (Ctrl snaps to 1 m); "
            "double-click a wall for its properties; wheel zooms, "
            "middle/right-drag pans.")

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        bar = QWidget()
        row = QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)
        self._b_select = QPushButton("Select")
        self._b_rect = QPushButton("Draw limits")
        self._b_wall = QPushButton("Add wall")
        self._b_del = QPushButton("Delete wall")
        self._b_straight = QPushButton("Straighten wall")
        self._b_straight.setToolTip("Remove the bend of the selected wall(s)")
        self._b_fit = QPushButton("Fit view")
        for b in (self._b_select, self._b_rect, self._b_wall):
            b.setCheckable(True)
        row.addWidget(self._b_select)
        row.addWidget(self._b_rect)
        row.addWidget(self._b_wall)
        row.addWidget(self._b_del)
        row.addWidget(self._b_straight)
        row.addStretch(1)
        row.addWidget(self._b_fit)
        self._layout.addWidget(bar)

        self.map = BuilderMapView()
        self._layout.addWidget(self.map, 1)
        self._info = QLabel("")
        self._info.setObjectName("hint")
        self._layout.addWidget(self._info)

        self._b_select.clicked.connect(lambda: self.map.set_mode(Mode.SELECT))
        self._b_rect.clicked.connect(lambda: self.map.set_mode(Mode.RECT))
        self._b_wall.clicked.connect(lambda: self.map.set_mode(Mode.WALL))
        self._b_del.clicked.connect(self.map.delete_selected_walls)
        self._b_straight.clicked.connect(self.map.straighten_selected_walls)
        self._b_fit.clicked.connect(self._fit)
        self.map.mode_changed.connect(self._on_mode)
        self.map.status.connect(self._info.setText)
        self.map.limits_changed.connect(self._refresh_info)
        self.map.walls_changed.connect(self._refresh_info)
        self.map.wall_double_clicked.connect(self._edit_wall)
        self._on_mode(Mode.SELECT)

    # ----------------------------------------------------------------- glue
    def _on_mode(self, mode: Mode) -> None:
        self._b_select.setChecked(mode is Mode.SELECT)
        self._b_rect.setChecked(mode is Mode.RECT)
        self._b_wall.setChecked(mode is Mode.WALL)

    def _refresh_info(self) -> None:
        r = self.map.limits()
        walls = self.map.walls()
        if r:
            spawn = self._spawn_text(r)
            self._info.setText(
                f"limits {r['x_max'] - r['x_min']:.0f} × "
                f"{r['y_max'] - r['y_min']:.0f} m · {len(walls)} wall(s) · "
                f"boat spawns at {spawn}")
        else:
            self._info.setText(f"{len(walls)} wall(s) · no limits yet")

    def _spawn_text(self, r: dict) -> str:
        """Where world (0, 0) — the spawn — will land after the build-time
        re-basing (path start when inside the rectangle, else the centre)."""
        xy = self._state.path.xy() if hasattr(self, "_state") else []
        if len(xy):
            x0, y0 = float(xy[0][0]), float(xy[0][1])
            if (r["x_min"] < x0 < r["x_max"]
                    and r["y_min"] < y0 < r["y_max"]):
                return "the path start"
        return "the area centre (path start outside the rectangle)"

    def _edit_wall(self, index: int) -> None:
        walls = self.map.walls()
        w = walls[index]
        dlg = SchemaDialog(f"Wall '{w['name']}'", _WALL_SCHEMA, values=w,
                           parent=self)
        if dlg.exec():
            w.update(dlg.values())
            self.map.set_walls(walls)
            self._refresh_info()

    def _fit(self) -> None:
        xs, ys = [], []
        xy = self._state.path.xy() if hasattr(self, "_state") else None
        if xy is not None and len(xy):
            xs += list(xy[:, 0])
            ys += list(xy[:, 1])
        r = self.map.limits()
        if r:
            xs += [r["x_min"], r["x_max"]]
            ys += [r["y_min"], r["y_max"]]
        if not xs:
            xs, ys = [0.0], [0.0]
        self.map.center_on_bounds(xs, ys)

    # ------------------------------------------------------------- protocol
    def enter(self, state: BuilderState) -> None:
        self._state = state
        self.map.set_anchor(state.path.anchor)
        self.map.set_path(state.path.xy())
        self.map.set_limits(state.limits_rect)
        self.map.set_walls(state.walls)
        self._refresh_info()
        self._fit()

    def commit(self, state: BuilderState) -> bool:
        limits = self.map.limits()
        if not limits:
            QMessageBox.warning(self, "World limits",
                                "Draw the world-limits rectangle first "
                                "('Draw limits', then drag on the map).")
            return False
        # No origin-containment check needed: the generated world is
        # re-based so that world (0, 0) -- where the boat always spawns --
        # is the path's first waypoint (or the rectangle centre), which is
        # inside the drawn limits by construction (BuilderState.spawn_origin).
        xy = state.path.xy()
        if len(xy) and ((xy[:, 0] < limits["x_min"]).any()
                        or (xy[:, 0] > limits["x_max"]).any()
                        or (xy[:, 1] < limits["y_min"]).any()
                        or (xy[:, 1] > limits["y_max"]).any()):
            if QMessageBox.question(
                    self, "World limits",
                    "Part of the GPS path lies outside the rectangle - the "
                    "boat cannot reach it there. Keep this rectangle?"
            ) != QMessageBox.StandardButton.Yes:
                return False
        state.limits_rect = limits
        state.walls = self.map.walls()
        return True
