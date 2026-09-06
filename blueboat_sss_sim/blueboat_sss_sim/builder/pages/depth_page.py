"""Screen 3 -- bathymetry (preset or file) + paint + seabed texture.

The preset list and every preset's parameter form come straight from
``DEPTH_PRESETS`` -- registering a new preset there adds it here with no
GUI change; the seabed texture form likewise renders
``worldgen.terrain.TERRAIN_TEXTURE_SCHEMA``. The file path accepts CSV
(lat, lon, depth) and geographic-CRS GeoTIFF; interpolation is linear
inside the data and extends the nearest known depth outward until a wall
or the world limit stops it. The third source is a sonar recording
(``.svlog``): first-bottom-return soundings along the boat's GPS track,
fitted with a smoothing spline, the local slope continued (damped)
beyond the surveyed strip and a residual-scaled undulation added
(``worldgen.depth.svlog`` / ``smooth``); the sailed track and its
soundings are drawn on the map, and an optional backscatter layer can
seed the material paint on request.

On top of that base, two brushes paint on the map: a **depth brush**
(trenches, mounds, dry banks at depth <= 0 -- exact inside the radius,
blended into the base over a short per-stroke feather, never across a
wall) and a **material brush** (mud patches, gravel beds). The
two previews on the right -- depth and reflectivity -- follow every
change after a 300 ms debounce, in a worker thread
(``DepthPreviewWorker``). Every form row carries a one-sentence hover
help (the schema's 7th element).
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox,
                               QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QMessageBox,
                               QPushButton, QRadioButton, QScrollArea,
                               QStackedWidget, QVBoxLayout, QWidget)

from ...worldgen.depth import DEPTH_PRESETS
from ...worldgen.depth.svlog import SVLOG_DEFAULTS
from ...worldgen.materials import MaterialLibrary
from ...worldgen.terrain import (TERRAIN_TEXTURE_SCHEMA, flatten_texture,
                                 unflatten_texture)
from ..forms import SchemaForm
from ..map.canvas import BuilderMapView, Mode
from ..state import BuilderState, DepthState
from ..worker import DepthPreviewWorker
from . import WizardPage

PREVIEW_DEBOUNCE_MS = 300


class DepthPage(WizardPage):
    TITLE = "Depth & seabed"
    HINT = ("Left: generate a depth pattern, load soundings (CSV "
            "lat,lon,depth / geographic GeoTIFF) or derive the depth from a "
            "sonar recording (.svlog), then the seabed texture. "
            "Middle: paint depth (trenches, banks, dry land at depth <= 0) "
            "and material patches on the map -- Ctrl+wheel sets the brush "
            "radius. Right: live previews of the depth and the "
            "reflectivity. Hover any parameter for what it does.")

    #: Screen-3 rows of the recording source: (key, label, min, max,
    #: step, decimals, help).
    SVLOG_ROWS = (
        ("datum_offset_m", "Datum offset [m]", -3.0, 3.0, 0.01, 2,
         "Tide and draft correction added to every sounding; positive "
         "if the water stood higher than the reference level during the "
         "survey."),
        ("trend_length_m", "Trend length [m]", 2.0, 100.0, 1.0, 1,
         "Distance over which the seabed slope measured at the edge of "
         "the track keeps going before it flattens out."),
        ("texture_gain", "Texture", 0.0, 2.0, 0.1, 2,
         "Gentle seabed undulation between the sounding lines, as a "
         "multiple of how rough the recording itself measured; 0 is "
         "perfectly smooth."),
        ("smoothing", "Smoothing", 0.0, 10.0, 0.1, 2,
         "How strongly the fitted surface averages over noisy soundings; "
         "0 passes through every cell median exactly."),
    )

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._state: BuilderState | None = None
        self._loading = False
        self._worker: DepthPreviewWorker | None = None
        self._pending = False
        self._last_key: str | None = None
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(PREVIEW_DEBOUNCE_MS)
        self._timer.timeout.connect(self._run_preview)

        body = QHBoxLayout()
        self._layout.addLayout(body, 1)

        # --- left: parameters (scrollable) ---------------------------------
        panel = QWidget()
        left = QVBoxLayout(panel)
        left.setContentsMargins(0, 0, 6, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        scroll.setMinimumWidth(330)
        body.addWidget(scroll, 0)

        self._r_preset = QRadioButton("Generated pattern")
        self._r_file = QRadioButton("From depth file")
        left.addWidget(self._r_preset)
        g_preset = QGroupBox()
        gl = QVBoxLayout(g_preset)
        self._preset_combo = QComboBox()
        self._preset_stack = QStackedWidget()
        self._preset_forms: dict[str, SchemaForm] = {}
        for name, (_fn, label, schema) in DEPTH_PRESETS.items():
            self._preset_combo.addItem(label, userData=name)
            form = SchemaForm(list(schema))
            form.edited.connect(self._schedule)
            self._preset_forms[name] = form
            self._preset_stack.addWidget(form)
        self._preset_combo.currentIndexChanged.connect(
            self._preset_stack.setCurrentIndex)
        self._preset_combo.currentIndexChanged.connect(self._schedule)
        gl.addWidget(self._preset_combo)
        gl.addWidget(self._preset_stack)
        left.addWidget(g_preset)

        left.addWidget(self._r_file)
        g_file = QGroupBox()
        fl = QFormLayout(g_file)
        pick = QHBoxLayout()
        self._file_edit = QLineEdit()
        self._file_edit.editingFinished.connect(self._schedule)
        b_browse = QPushButton("Browse…")
        b_browse.clicked.connect(self._browse)
        pick.addWidget(self._file_edit, 1)
        pick.addWidget(b_browse)
        w_pick = QWidget()
        w_pick.setLayout(pick)
        fl.addRow("Depth file", w_pick)
        self._values_combo = QComboBox()
        self._values_combo.addItems(["auto", "depth", "elevation"])
        self._values_combo.currentTextChanged.connect(self._schedule)
        fl.addRow("Values are", self._values_combo)
        left.addWidget(g_file)

        self._r_svlog = QRadioButton("From sonar recording (.svlog)")
        self._r_svlog.setToolTip(
            "Depth under the boat from the recording's first bottom "
            "return, placed at its GPS fixes and smoothed over the area.")
        left.addWidget(self._r_svlog)
        g_svlog = QGroupBox()
        sl = QFormLayout(g_svlog)
        pick_s = QHBoxLayout()
        self._svlog_edit = QLineEdit()
        self._svlog_edit.editingFinished.connect(self._schedule)
        b_browse_s = QPushButton("Browse…")
        b_browse_s.clicked.connect(self._browse_svlog)
        pick_s.addWidget(self._svlog_edit, 1)
        pick_s.addWidget(b_browse_s)
        w_pick_s = QWidget()
        w_pick_s.setLayout(pick_s)
        sl.addRow("Recording", w_pick_s)
        self._svlog_spins: dict[str, QDoubleSpinBox] = {}
        for key, label, lo, hi, step, dec, help_ in self.SVLOG_ROWS:
            sp = QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setSingleStep(step)
            sp.setDecimals(dec)
            sp.setValue(float(SVLOG_DEFAULTS[key]))
            sp.setToolTip(help_)
            sp.setStatusTip(help_)
            sp.valueChanged.connect(self._schedule)
            lbl = QLabel(label)
            lbl.setToolTip(help_)
            sl.addRow(lbl, sp)
            self._svlog_spins[key] = sp
        self._svlog_backscatter = QCheckBox(
            "Import backscatter as a reflectivity hint layer")
        self._svlog_backscatter.setToolTip(
            "Also derive a relative seabed-brightness map from the "
            "recording's mid-swath backscatter and show it on the map; "
            "nothing is assigned until you press the button below.")
        self._svlog_backscatter.toggled.connect(self._schedule)
        sl.addRow(self._svlog_backscatter)
        self._b_paint_bs = QPushButton("Paint materials from backscatter")
        self._b_paint_bs.setToolTip(
            "Turn the darkest fifth of the hint into mud patches and the "
            "brightest fifth into gravel, as material strokes you can "
            "edit or clear.")
        self._b_paint_bs.setEnabled(False)
        self._b_paint_bs.clicked.connect(self._paint_from_backscatter)
        sl.addRow(self._b_paint_bs)
        self._svlog_quality = QLabel("")
        self._svlog_quality.setObjectName("hint")
        self._svlog_quality.setWordWrap(True)
        sl.addRow(self._svlog_quality)
        left.addWidget(g_svlog)
        self._hint_raster: dict | None = None

        g_bed = QGroupBox("Seabed texture")
        bl = QVBoxLayout(g_bed)
        self._tex_form = SchemaForm(list(TERRAIN_TEXTURE_SCHEMA))
        self._tex_form.edited.connect(self._schedule)
        bl.addWidget(self._tex_form)
        left.addWidget(g_bed)
        left.addStretch(1)

        # --- middle: paint map --------------------------------------------
        mid = QVBoxLayout()
        body.addLayout(mid, 3)
        bar = QWidget()
        row = QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)
        self._b_pan = QPushButton("Pan")
        self._b_depth = QPushButton("Depth brush")
        self._b_mat = QPushButton("Material brush")
        self._b_erase = QPushButton("Eraser")
        for b in (self._b_pan, self._b_depth, self._b_mat, self._b_erase):
            b.setCheckable(True)
            row.addWidget(b)
        row.addSpacing(12)
        row.addWidget(QLabel("Depth [m]"))
        self._brush_depth = QDoubleSpinBox()
        self._brush_depth.setRange(-5.0, 60.0)      # <= 0: dry land
        self._brush_depth.setDecimals(2)
        self._brush_depth.setSingleStep(0.25)
        self._brush_depth.setValue(3.0)
        self._brush_depth.setToolTip(
            "Depth painted inside the brush; zero or negative paints dry "
            "land above the waterline.")
        row.addWidget(self._brush_depth)
        row.addWidget(QLabel("Radius [m]"))
        self._brush_radius = QDoubleSpinBox()
        self._brush_radius.setRange(0.25, 50.0)
        self._brush_radius.setDecimals(2)
        self._brush_radius.setSingleStep(0.25)
        self._brush_radius.setValue(2.0)
        row.addWidget(self._brush_radius)
        row.addWidget(QLabel("Feather [m]"))
        self._brush_feather = QDoubleSpinBox()
        self._brush_feather.setRange(0.1, 50.0)
        self._brush_feather.setDecimals(2)
        self._brush_feather.setSingleStep(0.25)
        self._brush_feather.setValue(1.0)
        self._brush_feather.setToolTip(
            "Width of the smooth blend from the painted depth back to the "
            "surrounding seabed, outside the brush radius.")
        row.addWidget(self._brush_feather)
        row.addWidget(QLabel("Material"))
        self._brush_mat = QComboBox()
        self._brush_mat.addItems(MaterialLibrary().seabed_names)
        self._brush_mat.setCurrentText("mud")
        row.addWidget(self._brush_mat)
        row.addStretch(1)
        self._b_undo = QPushButton("Undo")
        self._b_clear = QPushButton("Clear")
        self._b_fit = QPushButton("Fit view")
        row.addWidget(self._b_undo)
        row.addWidget(self._b_clear)
        row.addWidget(self._b_fit)
        mid.addWidget(bar)

        self.map = BuilderMapView()
        self.map.set_geometry_editable(False)
        self.map.set_brush(value=3.0, radius_m=2.0, material="mud",
                           feather_m=1.0)
        mid.addWidget(self.map, 1)
        self._info = QLabel("")
        self._info.setObjectName("hint")
        mid.addWidget(self._info)

        self._b_pan.clicked.connect(lambda: self.map.set_mode(Mode.SELECT))
        self._b_depth.clicked.connect(
            lambda: self.map.set_mode(Mode.DEPTH_BRUSH))
        self._b_mat.clicked.connect(
            lambda: self.map.set_mode(Mode.MATERIAL_BRUSH))
        self._b_erase.clicked.connect(lambda: self.map.set_mode(Mode.ERASER))
        self._b_undo.clicked.connect(lambda: self.map.undo_stroke())
        self._b_clear.clicked.connect(lambda: self.map.clear_strokes())
        self._b_fit.clicked.connect(self._fit)
        self._brush_depth.valueChanged.connect(
            lambda v: self.map.set_brush(value=v))
        self._brush_radius.valueChanged.connect(
            lambda v: self.map.set_brush(radius_m=v))
        self._brush_feather.valueChanged.connect(
            lambda v: self.map.set_brush(feather_m=v))
        self._brush_mat.currentTextChanged.connect(
            lambda m: self.map.set_brush(material=m))
        self.map.brush_changed.connect(self._brush_radius.setValue)
        self.map.mode_changed.connect(self._on_mode)
        self.map.status.connect(self._info.setText)
        self.map.strokes_changed.connect(self._on_strokes_changed)
        self._on_mode(Mode.SELECT)

        # --- right: live previews -----------------------------------------
        right = QVBoxLayout()
        body.addLayout(right, 2)
        self._imgs: dict[str, QLabel] = {}
        for key, cap in (("depth", "depth (white = shallow)"),
                         ("refl", "reflectivity (white = strong)")):
            lbl = QLabel("(no preview yet)")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setMinimumSize(240, 150)
            right.addWidget(lbl, 1)
            c = QLabel(cap)
            c.setObjectName("hint")
            c.setAlignment(Qt.AlignmentFlag.AlignCenter)
            right.addWidget(c)
            self._imgs[key] = lbl
        self._stats = QLabel("")
        self._stats.setObjectName("hint")
        self._stats.setWordWrap(True)
        right.addWidget(self._stats)

        self._r_preset.toggled.connect(g_preset.setEnabled)
        self._r_file.toggled.connect(g_file.setEnabled)
        self._r_svlog.toggled.connect(g_svlog.setEnabled)
        self._r_preset.toggled.connect(self._schedule)
        self._r_file.toggled.connect(self._schedule)
        self._r_svlog.toggled.connect(self._schedule)
        self._r_preset.setChecked(True)
        g_file.setEnabled(False)
        g_svlog.setEnabled(False)
        self._radios = {"preset": self._r_preset, "file": self._r_file,
                        "svlog": self._r_svlog}

    # ------------------------------------------------------------- helpers
    def _browse(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Depth file", str(Path.home()),
            "Depth data (*.csv *.txt *.xyz *.tif *.tiff);;All files (*)")
        if path:
            self._file_edit.setText(path)
            self._schedule()

    def _browse_svlog(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Sonar recording", str(Path.home()),
            "Sonar recordings (*.svlog);;All files (*)")
        if path:
            self._svlog_edit.setText(path)
            self._schedule()

    def _paint_from_backscatter(self) -> None:
        """Threshold the hint raster into mud (dark fifth) and gravel
        (bright fifth) blobs and append them as material strokes."""
        hint = self._hint_raster
        if not hint or hint.get("values") is None:
            return
        strokes = backscatter_material_strokes(hint)
        if not strokes:
            self._info.setText("backscatter hint has no contiguous "
                               "dark/bright patches to paint")
            return
        self.map.set_strokes(self.map.strokes("material") + strokes,
                             "material")
        self._info.setText(f"{len(strokes)} material patches painted from "
                           "the backscatter hint (Undo/Clear as usual)")
        self._on_strokes_changed()

    def _on_mode(self, mode: Mode) -> None:
        self._b_pan.setChecked(mode is Mode.SELECT)
        self._b_depth.setChecked(mode is Mode.DEPTH_BRUSH)
        self._b_mat.setChecked(mode is Mode.MATERIAL_BRUSH)
        self._b_erase.setChecked(mode is Mode.ERASER)

    def _on_strokes_changed(self) -> None:
        # A material stroke on a uniform seabed means "painted": switch
        # the layout so the intent is visible in the form (and the YAML).
        if self.map.strokes("material"):
            flat = self._tex_form.values()
            if flat.get("materials.layout") == "uniform":
                self._tex_form.set_values({"materials.layout": "painted"})
        self._schedule()

    def _fit(self) -> None:
        xs, ys = [], []
        if self._state is not None:
            xy = self._state.path.xy()
            if len(xy):
                xs += list(xy[:, 0])
                ys += list(xy[:, 1])
        r = self.map.limits()
        if r:
            xs += [r["x_min"], r["x_max"]]
            ys += [r["y_min"], r["y_max"]]
        if not xs:
            xs, ys = [0.0], [0.0]
        self.map.center_on_bounds(xs, ys, margin_m=5.0)

    def _collect(self, state: BuilderState) -> DepthState:
        texture = unflatten_texture(self._tex_form.values())
        material = str(texture["materials"]["uniform"])
        common = dict(texture=texture, material=material,
                      paint=self.map.strokes("depth"),
                      material_paint=self.map.strokes("material"))
        if self._r_file.isChecked():
            return DepthState(mode="file", file=self._file_edit.text().strip(),
                              file_values=self._values_combo.currentText(),
                              **common)
        if self._r_svlog.isChecked():
            knobs = {k: float(sp.value()) for k, sp in self._svlog_spins.items()}
            knobs["backscatter_layer"] = bool(self._svlog_backscatter.isChecked())
            return DepthState(mode="svlog",
                              file=self._svlog_edit.text().strip(),
                              svlog=knobs, **common)
        name = self._preset_combo.currentData()
        return DepthState(mode="preset", preset=name,
                          params=self._preset_forms[name].values(), **common)

    # ------------------------------------------------------------- preview
    def _schedule(self, *_args) -> None:
        if self._loading or self._state is None:
            return
        self._timer.start()

    def _run_preview(self) -> None:
        if self._state is None:
            return
        d = self._collect(self._state)
        if d.mode in ("file", "svlog") and not d.file:
            self._stats.setText("choose a depth file to preview"
                                if d.mode == "file"
                                else "choose a sonar recording to preview")
            return
        snap = dataclasses.replace(self._state, depth=d)
        key = repr((dataclasses.asdict(d), snap.limits_rect, snap.walls))
        if key == self._last_key:
            return
        if self._worker is not None and self._worker.isRunning():
            self._pending = True
            return
        self._last_key = key
        self._pending = False
        self._stats.setText("rendering preview…")
        self._worker = DepthPreviewWorker(snap)
        self._worker.ready.connect(self._show)
        self._worker.failed.connect(self._fail)
        self._worker.finished.connect(self._after_worker)
        self._worker.start()

    def _after_worker(self) -> None:
        if self._pending:
            self._timer.start()

    def _show(self, img_depth, img_refl, stats: str, info=None) -> None:
        for key, img in (("depth", img_depth), ("refl", img_refl)):
            lbl = self._imgs[key]
            lbl.setPixmap(QPixmap.fromImage(img).scaled(
                lbl.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))
        self._stats.setText(stats)
        self._show_survey(info if isinstance(info, dict) else {})

    def _show_survey(self, info: dict) -> None:
        """The sailed track + soundings on the map, the quality line, and
        (when asked for) the backscatter hint overlay."""
        from ..worker import survey_stats_line
        # The coverage line (and its warning) belongs to every positioned
        # source, so it is set before the recording-only parts.
        self._svlog_quality.setText(survey_stats_line(info))
        arrays = info.get("soundings_arrays") if info else None
        if not arrays:
            self.map.set_track(None)
            self.map.set_overlay(None)
            self._b_paint_bs.setEnabled(False)
            self._hint_raster = None
            return
        self.map.set_track(arrays["east"], arrays["north"], arrays["depth"])
        self._hint_raster = None
        self.map.set_overlay(None)
        want_hint = (self._r_svlog.isChecked()
                     and self._svlog_backscatter.isChecked()
                     and self._state is not None)
        if want_hint:
            try:
                self._hint_raster = compute_backscatter_hint(
                    self._svlog_edit.text().strip(), self._state)
                self.map.set_overlay(self._hint_raster)
            except Exception as exc:          # noqa: BLE001 -- shown, not raised
                self._info.setText(f"backscatter hint failed: {exc}")
                self._hint_raster = None
        self._b_paint_bs.setEnabled(self._hint_raster is not None)

    def _fail(self, msg: str) -> None:
        self._last_key = None                # retry on the next change
        self._stats.setText(f"preview failed: {msg}")
        # Never leave the previous recording's quality line standing under
        # a failure -- for the "does not overlap" case that line is the
        # very thing the message contradicts.
        self._svlog_quality.setText(msg if self._r_svlog.isChecked() else "")
        self.map.set_track(None)
        self.map.set_overlay(None)
        self._b_paint_bs.setEnabled(False)
        self._hint_raster = None

    # ------------------------------------------------------------- protocol
    def enter(self, state: BuilderState) -> None:
        self._loading = True
        try:
            self._state = state
            d = state.depth
            self._radios.get(d.mode, self._r_preset).setChecked(True)
            idx = self._preset_combo.findData(d.preset)
            if idx >= 0:
                self._preset_combo.setCurrentIndex(idx)
                if d.params:
                    self._preset_forms[d.preset].set_values(d.params)
            self._file_edit.setText(d.file if d.mode != "svlog" else "")
            self._svlog_edit.setText(d.file if d.mode == "svlog" else "")
            self._values_combo.setCurrentText(d.file_values)
            knobs = dict(SVLOG_DEFAULTS)
            knobs.update(d.svlog or {})
            for key, sp in self._svlog_spins.items():
                sp.setValue(float(knobs[key]))
            self._svlog_backscatter.setChecked(bool(knobs["backscatter_layer"]))
            flat = flatten_texture(d.texture)
            if "materials" not in (d.texture or {}):
                # Legacy state: the uniform material lived in `material`.
                flat["materials.uniform"] = d.material
            self._tex_form.set_values(flat)
            self.map.set_anchor(state.path.anchor)
            self.map.set_path(state.path.xy())
            self.map.set_limits(state.limits_rect)
            self.map.set_walls(state.walls)
            self.map.set_strokes(d.paint, "depth")
            self.map.set_strokes(d.material_paint, "material")
            self._fit()
        finally:
            self._loading = False
        self._last_key = None
        self._schedule()

    def commit(self, state: BuilderState) -> bool:
        d = self._collect(state)
        if d.mode in ("file", "svlog"):
            if not d.file:
                QMessageBox.warning(self, "Depth", "Choose a depth file or "
                                    "sonar recording (or switch to a "
                                    "generated pattern).")
                return False
            if not Path(d.file).exists():
                QMessageBox.warning(self, "Depth",
                                    f"File not found: {d.file}")
                return False
        state.depth = d
        return True


# ---------------------------------------------------- backscatter helpers
def compute_backscatter_hint(svlog_path: str, state: BuilderState) -> dict:
    """The recording's reflectivity-hint raster over the drawn limits, in
    the design frame (what the Screen-3 map shows)."""
    from ...core.types import GridSpec
    from ...worldgen.depth.svlog import backscatter_hint
    r = state.limits_rect
    if not r or state.path.anchor is None:
        raise ValueError("draw the world limits first (Screen 2)")
    grid = GridSpec(float(r["x_min"]), float(r["y_min"]), 1.0,
                    int(r["x_max"] - r["x_min"]) + 1,
                    int(r["y_max"] - r["y_min"]) + 1)
    return backscatter_hint(svlog_path, state.path.anchor, grid, cell_m=2.0)


def backscatter_material_strokes(hint: dict, *, dark: str = "mud",
                                 bright: str = "gravel", min_cells: int = 4,
                                 max_blobs: int = 200) -> list[dict]:
    """Dark-fifth -> ``dark``, bright-fifth -> ``bright`` material dot
    strokes (one per contiguous blob), design-frame metres."""
    import numpy as np
    from scipy import ndimage
    vals = hint.get("values")
    if vals is None or not np.isfinite(vals).any():
        return []
    ox, oy = hint["origin"]
    cell = float(hint["cell_m"])
    finite = np.isfinite(vals)
    lo, hi = np.nanpercentile(vals, 20), np.nanpercentile(vals, 80)
    strokes: list[dict] = []
    for mask, name in ((finite & (vals <= lo), dark),
                       (finite & (vals >= hi), bright)):
        labels, n = ndimage.label(mask)
        if n == 0:
            continue
        sizes = ndimage.sum(mask, labels, index=np.arange(1, n + 1))
        order = np.argsort(sizes)[::-1]
        for k in order[:max_blobs]:
            if sizes[k] < min_cells:
                break
            lab = k + 1
            cj, ci = ndimage.center_of_mass(mask, labels, lab)
            radius = float(np.sqrt(sizes[k] * cell * cell / np.pi))
            strokes.append({"points": [[ox + float(ci) * cell,
                                        oy + float(cj) * cell]],
                            "radius_m": max(radius, 0.5), "value": name,
                            "erase": False})
    return strokes
