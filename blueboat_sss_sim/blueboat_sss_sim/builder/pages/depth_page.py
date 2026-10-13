"""Screen 3 -- the depth & seabed editor: a layer stack over a depth source.

The screen is an image-editor-style stack (``worldgen.depth.layers``,
``compose``) with real seabed shapes for layers:

* the **depth source** at the bottom -- a generated pattern
  (``DEPTH_PRESETS``), a depth file (CSV lat,lon,depth / geographic
  GeoTIFF) or a sonar recording (``.svlog``, first-bottom-return
  soundings fitted with a smoothing spline, the damped trend beyond the
  track, a coverage line, and -- one tick -- a ``reflectivity_survey``
  layer carrying the seabed reflectivity the recording measured);
* the **base depth brush** strokes and the **seabed texture** form with
  its material brush (the legacy ``DepthState.paint`` /
  ``material_paint`` / ``texture``, so a world with nothing above them is
  built exactly as before);
* any number of **authored layers** above: gradients with Dean / damped /
  linear profiles, landforms (dredged basin, channel along a drawn
  centreline, scour hole, mound, bar and rips, step, siltation), relief
  (ripples, dunes, furrows, roughness, outcrops, pits), adjustments,
  material regions / gradients / patches / belts, a reflectivity gain and
  brush-stroke layers with an operation -- each with a mask (rectangle,
  ellipse, polygon, band along a line, distance to the walls, depth range,
  gradient, mask brush, survey coverage), a feather that never crosses a
  wall, an opacity and a blend mode (normal / add / deepen / shoal).
  Recipes (``config/seabed_recipes.yaml``) instantiate literature-sized
  stacks over the source.

Geometry is drawn on the map (click vertices and right-click to finish,
or drag), edited by its handles, and stored in the design frame like the
walls; at build time the stack is baked into a GeoTIFF and a material
map (``worldgen.depth.bake``), so downstream sees a plain file-mode
world. The two previews on the right, the composite map overlay and the
section tool follow every change after a 300 ms debounce in a worker
thread (``DepthPreviewWorker``). Every edit is undoable. Every form row
carries a one-sentence hover help (the schema's 7th element).
"""

from __future__ import annotations

import copy
import dataclasses
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QUndoStack
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox,
                               QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QMenu,
                               QMessageBox, QPushButton, QRadioButton,
                               QStackedWidget, QVBoxLayout, QWidget)

from ...worldgen.depth import DEPTH_PRESETS
from ...worldgen.depth.compose import stack_from_depth
from ...worldgen.depth.layers import LAYER_KINDS, normalize_layers
from ...worldgen.depth.layers.survey import SURVEY_LAYER_ID
from ...worldgen.depth.recipes import instantiate_recipe, load_recipes
from ...worldgen.depth.svlog import SVLOG_DEFAULTS
from ...worldgen.materials import MaterialLibrary
from ...worldgen.terrain import (TERRAIN_TEXTURE_SCHEMA, flatten_texture,
                                 unflatten_texture)
from ..forms import SchemaForm
from ..worker import hover_text
from ..layout import WrapScroll, ScaledImage, ToolBar, split
from ..layers.commands import SnapshotCommand
from ..layers.document import LayerDocument
from ..layers.panel import LayerPanel
from ..layers.properties import MASK_GEOMETRY, LayerProperties
from ..layers.section import SectionView
from ..map.canvas import (MODE_FOR_GEOMETRY, BuilderMapView, Mode,
                          _material_color)
from ..state import BuilderState, DepthState
from ..worker import DepthPreviewWorker
from . import WizardPage

PREVIEW_DEBOUNCE_MS = 300
OVERLAY_CHOICES = ("none", "depth", "sonar look", "material",
                   "reflectivity")


class DepthPage(WizardPage):
    TITLE = "Depth & seabed"
    HINT = ("Left: the seabed as a stack -- the depth source at the bottom "
            "(a generated pattern, soundings or a sonar recording), the base "
            "brush strokes and texture, then the layers you add: gradients, "
            "landforms, relief, adjustments, materials, or a recipe. Draw a "
            "layer's shape on the map, mask it, set its blend and opacity. "
            "Middle: brushes, masks and the section tool -- Ctrl+wheel sets "
            "the brush radius. Right: live previews; the overlay shows the "
            "composite on the map. Hover any row for what it does.")

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
        ("relief_gain", "Relief gain", 0.0, 3.0, 0.1, 2,
         "Amplitude of the relief read from the recording's shading; its "
         "true size is uncertain by the sonar's Lambert exponent (a factor "
         "of about 1-2), so 0 calibrates it on the FBR soundings of the "
         "other survey lines crossing each swath."),
        ("relief_max_m", "Relief clamp [m]", 0.1, 5.0, 0.1, 2,
         "Largest relief the shading may add or remove per cell; keeps a "
         "bright object or a wall from becoming a hill."),
    )
    RELIEF_HELP = {
        "synthetic": "A seeded undulation between the lines, sized by how "
                     "rough the soundings measured (every older world).",
        "shading": "The relief the recording's own brightness measures "
                   "across each swath -- sand waves, scour, banks -- "
                   "integrated from the depth under the boat and fused over "
                   "the fitted surface (Coiras 2007; Ran 2025).",
        "none": "No relief between the lines: the smooth fitted surface.",
    }

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._state: BuilderState | None = None
        self._loading = False
        self._worker: DepthPreviewWorker | None = None
        self._pending = False
        self._last_key: str | None = None
        self._preview: dict | None = None
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(PREVIEW_DEBOUNCE_MS)
        self._timer.timeout.connect(self._run_preview)

        # The document: authored layers + the base brush strokes.
        self._doc = LayerDocument()
        self._base_paint: list[dict] = []
        self._base_material_paint: list[dict] = []
        self._selected_key = "source"
        self._stroke_target: tuple = ("base",)      # what the depth brush edits
        self._geom_target: tuple | None = None      # ("geometry", id) | ("mask", id, kind)
        self._undo = QUndoStack(self)
        try:
            self._recipes = load_recipes()
        except Exception:                           # noqa: BLE001
            self._recipes = {}

        body = split()
        self._layout.addWidget(body, 1)

        # --- left: layer panel + the selected row's properties -------------
        left_w = QWidget()
        left = QVBoxLayout(left_w)
        left.setContentsMargins(0, 0, 6, 0)
        self._panel = LayerPanel(self._recipes)
        self._panel.setMaximumHeight(230)
        left.addWidget(self._panel)
        self._prop_stack = QStackedWidget()
        # A WrapScroll, not a resizable QScrollArea: the recording-quality
        # line wraps over several lines and needs its height *at the
        # column's width*, which a QStackedWidget never reports.
        self._prop_scroll = WrapScroll(self._prop_stack)
        left.addWidget(self._prop_scroll, 1)
        left_w.setMinimumWidth(260)
        body.addWidget(left_w)
        body.setStretchFactor(0, 0)

        self._prop_stack.addWidget(self._build_source_page())     # 0
        self._prop_stack.addWidget(self._build_brush_page())      # 1
        self._prop_stack.addWidget(self._build_texture_page())    # 2
        self._props = LayerProperties()
        self._props.edited.connect(self._apply_properties)
        self._props.draw_geometry_requested.connect(self._draw_geometry)
        self._props.draw_mask_requested.connect(self._draw_mask)
        self._props.clear_mask_requested.connect(self._clear_mask)
        self._prop_stack.addWidget(self._props)                   # 3

        self._panel.selected.connect(self._on_select)
        self._panel.add_requested.connect(self._add_layer)
        self._panel.recipe_requested.connect(self._add_recipe)
        self._panel.remove_requested.connect(self._remove_layer)
        self._panel.move_requested.connect(self._move_layer)
        self._panel.duplicate_requested.connect(self._duplicate_layer)
        self._panel.toggled.connect(self._toggle_layer)

        # --- middle: tools + map --------------------------------------------
        mid_w = QWidget()
        mid = QVBoxLayout(mid_w)
        mid.setContentsMargins(0, 0, 0, 0)
        body.addWidget(mid_w)
        body.setStretchFactor(1, 3)
        bar = ToolBar()
        self._b_pan = QPushButton("Pan")
        self._b_depth = QPushButton("Depth brush")
        self._b_mat = QPushButton("Material brush")
        self._b_mask = QPushButton("Mask brush")
        self._b_erase = QPushButton("Eraser")
        self._b_section = QPushButton("Section")
        for b, tip in ((self._b_pan, "Pan and drag handles."),
                       (self._b_depth, "Paint depth strokes into the base "
                                       "brush or the selected brush layer."),
                       (self._b_mat, "Paint seabed material patches."),
                       (self._b_mask, "Paint the selected layer's mask."),
                       (self._b_erase, "Erase strokes of the active brush."),
                       (self._b_section, "Drag a line on the map to plot the "
                                         "depth profile along it.")):
            b.setCheckable(True)
            b.setToolTip(tip)
            bar.add(b)
        bar.separator()
        bar.label("Depth [m]")
        self._brush_depth = QDoubleSpinBox()
        self._brush_depth.setRange(-5.0, 60.0)      # <= 0: dry land
        self._brush_depth.setDecimals(2)
        self._brush_depth.setSingleStep(0.25)
        self._brush_depth.setValue(3.0)
        self._brush_depth.setToolTip(
            "Depth painted inside the brush; zero or negative paints dry "
            "land above the waterline.")
        bar.add(self._brush_depth)
        bar.label("Radius [m]")
        self._brush_radius = QDoubleSpinBox()
        self._brush_radius.setRange(0.25, 50.0)
        self._brush_radius.setDecimals(2)
        self._brush_radius.setSingleStep(0.25)
        self._brush_radius.setValue(2.0)
        bar.add(self._brush_radius)
        bar.label("Feather [m]")
        self._brush_feather = QDoubleSpinBox()
        self._brush_feather.setRange(0.1, 50.0)
        self._brush_feather.setDecimals(2)
        self._brush_feather.setSingleStep(0.25)
        self._brush_feather.setValue(1.0)
        self._brush_feather.setToolTip(
            "Width of the smooth blend from the painted depth back to the "
            "surrounding seabed, outside the brush radius.")
        bar.add(self._brush_feather)
        bar.label("Material")
        self._brush_mat = QComboBox()
        self._brush_mat.addItems(MaterialLibrary().seabed_names)
        self._brush_mat.setCurrentText("mud")
        bar.add(self._brush_mat)
        bar.separator()
        bar.label("Overlay")
        self._overlay = QComboBox()
        self._overlay.addItems(list(OVERLAY_CHOICES))
        self._overlay.setCurrentText("depth")
        self._overlay.setToolTip("Which composite channel to draw over the "
                                 "map: the depth, the material or the "
                                 "reflectivity of the preview.")
        self._overlay.currentTextChanged.connect(lambda _t: self._update_overlay())
        bar.add(self._overlay)
        bar.separator()
        self._b_undo = QPushButton("Undo")
        self._b_redo = QPushButton("Redo")
        self._b_clear = QPushButton("Clear strokes")
        self._b_fit = QPushButton("Fit view")
        bar.add(self._b_undo, self._b_redo, self._b_clear, self._b_fit)
        mid.addWidget(bar)

        self.map = BuilderMapView()
        self.map.set_geometry_editable(False)
        self.map.set_brush(value=3.0, radius_m=2.0, material="mud",
                           feather_m=1.0)
        mid.addWidget(self.map, 1)
        self._info = QLabel("")
        self._info.setObjectName("hint")
        mid.addWidget(self._info)
        # The hover readout: what the preview says at the cursor (map or
        # either preview picture) -- position, GPS, depth, material.
        self._hover = QLabel("")
        self._hover.setObjectName("hint")
        self._hover.setToolTip("Depth and material of the preview under the "
                               "cursor, on the map or on a preview picture.")
        mid.addWidget(self._hover)
        self._section = SectionView()
        mid.addWidget(self._section)

        self._b_pan.clicked.connect(lambda: self.map.set_mode(Mode.SELECT))
        self._b_depth.clicked.connect(
            lambda: self.map.set_mode(Mode.DEPTH_BRUSH))
        self._b_mat.clicked.connect(
            lambda: self.map.set_mode(Mode.MATERIAL_BRUSH))
        self._b_mask.clicked.connect(self._start_mask_brush)
        self._b_erase.clicked.connect(lambda: self.map.set_mode(Mode.ERASER))
        self._b_section.clicked.connect(lambda: self.map.set_mode(Mode.SECTION))
        self._b_undo.clicked.connect(self._undo.undo)
        self._b_redo.clicked.connect(self._undo.redo)
        self._b_clear.clicked.connect(self._clear_strokes)
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
        self.map.hover.connect(self._on_map_hover)
        self.map.hover_left.connect(lambda: self._hover.setText(""))
        self.map.strokes_changed.connect(self._on_strokes_changed)
        self.map.geometry_committed.connect(self._on_geometry_committed)
        self.map.geometry_changed.connect(self._on_geometry_changed)
        self.map.section_drawn.connect(self._on_section)
        self._undo.canUndoChanged.connect(self._b_undo.setEnabled)
        self._undo.canRedoChanged.connect(self._b_redo.setEnabled)
        self._b_undo.setEnabled(False)
        self._b_redo.setEnabled(False)
        self._on_mode(Mode.SELECT)

        # --- right: live previews -----------------------------------------
        right_w = QWidget()
        right = QVBoxLayout(right_w)
        right.setContentsMargins(6, 0, 0, 0)
        body.addWidget(right_w)
        body.setStretchFactor(2, 1)      # the map takes the room, not the strip
        body.setSizes([340, 780, 300])
        self._imgs: dict[str, QLabel] = {}
        for key, cap in (("depth", "depth (white = shallow)"),
                         ("refl", "reflectivity (white = strong)")):
            lbl = ScaledImage("(no preview yet)", min_w=150, min_h=110)
            lbl.hover.connect(self._on_preview_hover)
            lbl.hover_left.connect(lambda: self._hover.setText(""))
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
        self._panel.refresh(self._doc, "source")

    # ------------------------------------------------------ property pages
    def _build_source_page(self) -> QWidget:
        page = QWidget()
        left = QVBoxLayout(page)
        left.setContentsMargins(0, 0, 0, 0)
        self._r_preset = QRadioButton("Generated pattern")
        self._r_file = QRadioButton("From depth file")
        left.addWidget(self._r_preset)
        g_preset = QGroupBox()
        gl = QVBoxLayout(g_preset)
        self._preset_combo = QComboBox()
        # The longest preset label must not set the column's minimum.
        self._preset_combo.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._preset_combo.setMinimumContentsLength(16)
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
        fl.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
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
        sl.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
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
        self._svlog_relief = QComboBox()
        for mode in ("shading", "synthetic", "none"):
            self._svlog_relief.addItem(mode)
        self._svlog_relief.setToolTip("\n".join(
            f"{m}: {h}" for m, h in self.RELIEF_HELP.items()))
        self._svlog_relief.currentTextChanged.connect(self._schedule)
        lbl_r = QLabel("Relief between lines")
        lbl_r.setToolTip(self._svlog_relief.toolTip())
        sl.addRow(lbl_r, self._svlog_relief)
        self._svlog_backscatter = QCheckBox("Reflectivity from backscatter")
        self._svlog_backscatter.setToolTip(
            "Add a 'Reflectivity from recording' layer over the source: "
            "every profile bin is inverted through the calibrated sonar "
            "model into the seabed reflectivity it measured, rendered as a "
            "continuous gain about the reference material and classed "
            "(mud / gravel / rock) by the literature contrasts. It is an "
            "ordinary layer row: mask it, reorder it, edit its thresholds "
            "or delete it; unticking removes it.")
        self._svlog_backscatter.toggled.connect(self._on_backscatter_toggled)
        sl.addRow(self._svlog_backscatter)
        self._svlog_quality = QLabel("")
        self._svlog_quality.setObjectName("hint")
        self._svlog_quality.setWordWrap(True)
        # The text is mirrored into the tooltip by ``_set_quality`` so a
        # long readout is always readable in full.
        sl.addRow(self._svlog_quality)
        left.addWidget(g_svlog)
        left.addStretch(1)

        self._r_preset.toggled.connect(g_preset.setEnabled)
        self._r_file.toggled.connect(g_file.setEnabled)
        self._r_svlog.toggled.connect(g_svlog.setEnabled)
        self._r_preset.toggled.connect(self._schedule)
        self._r_file.toggled.connect(self._schedule)
        self._r_svlog.toggled.connect(self._schedule)
        # The survey layer exists exactly when the recording source is
        # selected and the box is ticked (one source of truth: the
        # document; the box follows it in ``_refresh_all``).
        self._r_svlog.toggled.connect(lambda _on: self._sync_backscatter_layer())
        self._r_preset.setChecked(True)
        g_file.setEnabled(False)
        g_svlog.setEnabled(False)
        self._radios = {"preset": self._r_preset, "file": self._r_file,
                        "svlog": self._r_svlog}
        return page

    def _build_brush_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        self._brush_info = QLabel("")
        self._brush_info.setObjectName("hint")
        self._brush_info.setWordWrap(True)
        lay.addWidget(self._brush_info)
        hint = QLabel("The base depth strokes: exact inside the radius, "
                      "blended into the source over their feather, never "
                      "across a wall. Use the Depth brush and Eraser on the "
                      "map; add a 'Brush strokes' layer for raise / lower / "
                      "smooth / flatten / noise operations.")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        lay.addStretch(1)
        return page

    def _build_texture_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        g_bed = QGroupBox("Seabed texture")
        bl = QVBoxLayout(g_bed)
        self._tex_form = SchemaForm(list(TERRAIN_TEXTURE_SCHEMA))
        self._tex_form.edited.connect(self._schedule)
        bl.addWidget(self._tex_form)
        lay.addWidget(g_bed)
        self._mat_info = QLabel("")
        self._mat_info.setObjectName("hint")
        self._mat_info.setWordWrap(True)
        lay.addWidget(self._mat_info)
        hint = QLabel("The material brush paints patches over this layout; "
                      "material layers above (regions, gradients, patches, "
                      "belts) are stamped over both.")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        lay.addStretch(1)
        return page

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

    def _on_mode(self, mode: Mode) -> None:
        self._b_pan.setChecked(mode is Mode.SELECT)
        self._b_depth.setChecked(mode is Mode.DEPTH_BRUSH)
        self._b_mat.setChecked(mode is Mode.MATERIAL_BRUSH)
        self._b_mask.setChecked(mode is Mode.MASK_BRUSH)
        self._b_erase.setChecked(mode is Mode.ERASER)
        self._b_section.setChecked(mode is Mode.SECTION)

    # ------------------------------------------------------- hover readout
    def _on_map_hover(self, x: float, y: float) -> None:
        anchor = self._state.path.anchor if self._state is not None else None
        self._hover.setText(hover_text(self._preview, x, y, anchor))

    def _on_preview_hover(self, u: float, v: float) -> None:
        """A preview picture is the north-up raster: u -> column, v (from
        the bottom = south) -> row, then the same readout as the map."""
        res = self._preview
        if not res:
            return
        g = res["grid"]
        i = min(int(u * g.nx), g.nx - 1)
        j = min(int(v * g.ny), g.ny - 1)
        self._on_map_hover(g.origin_x + i * g.resolution,
                           g.origin_y + j * g.resolution)

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

    # ------------------------------------------------------ document model
    def _snapshot(self) -> dict:
        return {"layers": self._doc.snapshot(),
                "paint": copy.deepcopy(self._base_paint),
                "material_paint": copy.deepcopy(self._base_material_paint)}

    def _push(self, text: str, before: dict, merge_key: str | None = None) -> None:
        after = self._snapshot()
        if after == before:
            return
        self._undo.push(SnapshotCommand(self, before, after, text, merge_key))

    def _restore_snapshot(self, snap: dict) -> None:
        self._doc.restore(snap["layers"])
        self._base_paint = copy.deepcopy(snap["paint"])
        self._base_material_paint = copy.deepcopy(snap["material_paint"])
        if (self._panel.is_layer(self._selected_key)
                and self._selected_key not in self._doc.ids()):
            self._selected_key = "source"
        self._refresh_all()
        self._schedule()

    def _refresh_all(self) -> None:
        """Panel, property page, canvas strokes and geometries from the
        document (after any edit, undo or load)."""
        self._panel.refresh(self._doc, self._selected_key)
        self._show_selected()
        # The reflectivity box mirrors the document (undo, a panel delete
        # and a load all land here), never the other way round.
        has = SURVEY_LAYER_ID in self._doc.ids()
        if self._svlog_backscatter.isChecked() != has:
            self._svlog_backscatter.blockSignals(True)
            self._svlog_backscatter.setChecked(has)
            self._svlog_backscatter.blockSignals(False)

    # -------------------------------------------- reflectivity from recording
    def _on_backscatter_toggled(self, _on: bool) -> None:
        self._sync_backscatter_layer()

    def _sync_backscatter_layer(self) -> None:
        """Add or remove the ``reflectivity_survey`` row so that it exists
        exactly when the recording source is selected and the box is
        ticked. One undo step either way."""
        if self._loading:
            return
        want = self._r_svlog.isChecked() and self._svlog_backscatter.isChecked()
        have = SURVEY_LAYER_ID in self._doc.ids()
        if want == have:
            return
        before = self._snapshot()
        self._flush_strokes()
        if want:
            self._doc.add("reflectivity_survey", layer_id=SURVEY_LAYER_ID,
                          name="Reflectivity from recording")
            # At the bottom of the authored stack: the user's own material
            # edits above it win over the measurement.
            self._doc.move(SURVEY_LAYER_ID, -len(self._doc.layers))
            self._selected_key = SURVEY_LAYER_ID
            if self._overlay.currentText() != "reflectivity":
                self._overlay.setCurrentText("reflectivity")
        else:
            self._doc.remove(SURVEY_LAYER_ID)
            if self._selected_key == SURVEY_LAYER_ID:
                self._selected_key = "source"
        self._refresh_all()
        self._push("Reflectivity from recording", before)
        self._schedule()

    def _flush_strokes(self) -> None:
        """Canvas stroke lists -> their owners (the base or the selected
        layer), without pushing an undo step."""
        depth = self.map.strokes("depth")
        if self._stroke_target[0] == "base":
            self._base_paint = depth
        else:
            try:
                L = self._doc.get(self._stroke_target[1])
                L["geometry"] = {"type": "strokes", "strokes": depth}
            except KeyError:
                pass
        self._base_material_paint = self.map.strokes("material")
        if self._panel.is_layer(self._selected_key):
            try:
                L = self._doc.get(self._selected_key)
            except KeyError:
                L = None
            m = L.get("mask") if L else None
            m = m[0] if isinstance(m, list) and m else m
            if m and m.get("kind") == "brush":
                m["strokes"] = self.map.strokes("mask")
                L["mask"] = m

    def _show_selected(self) -> None:
        """Load the selected row into the property page and the canvas."""
        key = self._selected_key
        self.map.syncing = True
        if self._panel.is_layer(key):
            L = self._doc.get(key)
            self._prop_stack.setCurrentIndex(3)
            self._props.show_layer(L)
            if L["kind"] == "paint":
                self._stroke_target = ("layer", key)
                self.map.set_strokes((L.get("geometry") or {}).get("strokes") or [],
                                     "depth")
            else:
                self._stroke_target = ("base",)
                self.map.set_strokes(self._base_paint, "depth")
            m = L.get("mask")
            m = m[0] if isinstance(m, list) and m else m
            self.map.set_strokes(m.get("strokes") or [] if m and m.get("kind") == "brush"
                                 else [], "mask")
        else:
            self._prop_stack.setCurrentIndex({"source": 0, "brush": 1,
                                              "texture": 2}.get(key, 0))
            self._stroke_target = ("base",)
            self.map.set_strokes(self._base_paint, "depth")
            self.map.set_strokes([], "mask")
        self.map.set_strokes(self._base_material_paint, "material")
        self.map.syncing = False
        self.map.set_layer_geometries(self._layer_geometries())
        self._brush_info.setText(f"{len(self._base_paint)} base depth strokes")
        self._mat_info.setText(f"{len(self._base_material_paint)} material strokes")

    @staticmethod
    def _mask_shape(m: dict | None) -> dict | None:
        """A drawn mask as a displayable geometry dict."""
        m = m[0] if isinstance(m, list) and m else m
        if not m:
            return None
        k = m.get("kind")
        if k == "polygon" and m.get("points"):
            return {"type": "polygon", "points": m["points"]}
        if k == "polyline_band" and m.get("points"):
            return {"type": "polyline", "points": m["points"]}
        if k == "ellipse" and "centre" in m:
            return {"type": "ellipse", "centre": m["centre"], "rx_m": m["rx_m"],
                    "ry_m": m["ry_m"], "angle_deg": m.get("angle_deg", 0.0)}
        if k == "rect" and "x_min" in m:
            return {"type": "rect", "x_min": m["x_min"], "y_min": m["y_min"],
                    "x_max": m["x_max"], "y_max": m["y_max"]}
        if k == "gradient" and m.get("geometry"):
            return dict(m["geometry"])
        return None

    @staticmethod
    def _shape_into_mask(m: dict, g: dict) -> dict:
        m = dict(m)
        t = g.get("type")
        if t in ("polygon", "polyline"):
            m["points"] = g["points"]
        elif t == "ellipse":
            m.update(centre=g["centre"], rx_m=g["rx_m"], ry_m=g["ry_m"],
                     angle_deg=g.get("angle_deg", 0.0))
        elif t == "rect":
            m.update(x_min=g["x_min"], y_min=g["y_min"], x_max=g["x_max"],
                     y_max=g["y_max"])
        elif t in ("axis", "radial"):
            m["geometry"] = dict(g)
        return m

    def _layer_geometries(self) -> list:
        out = []
        for L in self._doc.layers:
            sel = L["id"] == self._selected_key
            g = L.get("geometry")
            if g and g.get("type") != "strokes":
                out.append((L["id"], g, sel))
            ms = self._mask_shape(L.get("mask"))
            if ms:
                out.append((f"{L['id']}:mask", ms, sel))
        return out

    # ------------------------------------------------------- panel actions
    def _on_select(self, key: str) -> None:
        if key == self._selected_key:
            return
        self._flush_strokes()
        self._selected_key = key
        self._show_selected()

    def _add_layer(self, kind: str) -> None:
        before = self._snapshot()
        self._flush_strokes()
        spec = LAYER_KINDS[kind]
        geometry = ({"type": "strokes", "strokes": []}
                    if spec.geometry == ("strokes",) else None)
        new_id = self._doc.add(kind, geometry=geometry)
        self._selected_key = new_id
        self._refresh_all()
        self._push(f"Add {spec.label}", before)
        if spec.geometry and spec.geometry != ("strokes",):
            self._start_geometry(new_id, spec.geometry[0])
        else:
            self._schedule()

    def _add_recipe(self, name: str) -> None:
        rect = self._state.limits_rect if self._state else None
        if not rect:
            QMessageBox.warning(self, "Recipe", "Draw the world limits first "
                                "(Screen 2).")
            return
        rec = self._recipes.get(name)
        if not rec:
            return
        before = self._snapshot()
        self._flush_strokes()
        ids = self._doc.add_many(instantiate_recipe(rec, rect,
                                                    self._doc.next_number()))
        if ids:
            self._selected_key = ids[-1]
        self._refresh_all()
        self._push(f"Recipe {rec.get('label', name)}", before)
        self._info.setText(f"recipe '{rec.get('label', name)}': "
                           f"{rec.get('consequence', '')}")
        self._schedule()

    def _remove_layer(self, layer_id: str) -> None:
        before = self._snapshot()
        self._flush_strokes()
        self._doc.remove(layer_id)
        if self._selected_key == layer_id:
            self._selected_key = "source"
        self._refresh_all()
        self._push("Delete layer", before)
        self._schedule()

    def _move_layer(self, layer_id: str, delta: int) -> None:
        before = self._snapshot()
        self._flush_strokes()
        self._doc.move(layer_id, delta)
        self._refresh_all()
        self._push("Move layer", before)
        self._schedule()

    def _duplicate_layer(self, layer_id: str) -> None:
        before = self._snapshot()
        self._flush_strokes()
        self._selected_key = self._doc.duplicate(layer_id)
        self._refresh_all()
        self._push("Duplicate layer", before)
        self._schedule()

    def _toggle_layer(self, layer_id: str, enabled: bool) -> None:
        before = self._snapshot()
        self._doc.get(layer_id)["enabled"] = bool(enabled)
        if layer_id == self._selected_key:
            self._props.show_layer(self._doc.get(layer_id))
        self._push("Toggle layer", before)
        self._schedule()

    def _apply_properties(self) -> None:
        if not self._panel.is_layer(self._selected_key):
            return
        before = self._snapshot()
        L = self._doc.get(self._selected_key)
        self._props.read_into(L)
        self._panel.refresh(self._doc, self._selected_key)
        self.map.set_layer_geometries(self._layer_geometries())
        self._push("Edit layer", before)
        pending = self._props.mask_kind_pending()
        if pending:
            self._draw_mask(pending)
        else:
            self._schedule()

    # ------------------------------------------------------- geometry tools
    def _draw_geometry(self) -> None:
        if not self._panel.is_layer(self._selected_key):
            return
        spec = LAYER_KINDS[self._doc.get(self._selected_key)["kind"]]
        types = [t for t in spec.geometry if t != "strokes"]
        if not types:
            return
        if len(types) == 1:
            self._start_geometry(self._selected_key, types[0])
            return
        menu = QMenu(self)
        for t in types:
            menu.addAction(t).triggered.connect(
                lambda _=False, tt=t: self._start_geometry(self._selected_key, tt))
        menu.exec(self._props.b_draw.mapToGlobal(self._props.b_draw.rect().bottomLeft()))

    def _start_geometry(self, layer_id: str, gtype: str) -> None:
        self._geom_target = ("geometry", layer_id)
        self.map.set_mode(MODE_FOR_GEOMETRY[gtype])

    def _draw_mask(self, kind: str) -> None:
        if not self._panel.is_layer(self._selected_key):
            return
        if kind == "brush":
            self._start_mask_brush()
            return
        gtype = MASK_GEOMETRY.get(kind)
        if not gtype:
            return
        self._geom_target = ("mask", self._selected_key, kind)
        self.map.set_mode(MODE_FOR_GEOMETRY[gtype])

    def _start_mask_brush(self) -> None:
        if not self._panel.is_layer(self._selected_key):
            self._info.setText("select a layer to paint its mask")
            self.map.set_mode(Mode.SELECT)
            return
        L = self._doc.get(self._selected_key)
        m = L.get("mask")
        m = m[0] if isinstance(m, list) and m else m
        if not m or m.get("kind") != "brush":
            before = self._snapshot()
            L["mask"] = {"kind": "brush", "strokes": [],
                         "feather_m": float(self._props.mask_feather.value()),
                         "invert": False}
            self._props.show_layer(L)
            self.map.set_strokes([], "mask")
            self._push("Mask brush", before)
        self.map.set_mode(Mode.MASK_BRUSH)

    def _clear_mask(self) -> None:
        if not self._panel.is_layer(self._selected_key):
            return
        before = self._snapshot()
        L = self._doc.get(self._selected_key)
        L["mask"] = None
        self.map.set_strokes([], "mask")
        self._props.show_layer(L)
        self.map.set_layer_geometries(self._layer_geometries())
        self._push("Clear mask", before)
        self._schedule()

    def _on_geometry_committed(self, geom: dict) -> None:
        target, self._geom_target = self._geom_target, None
        if not target:
            return
        before = self._snapshot()
        try:
            L = self._doc.get(target[1])
        except KeyError:
            return
        if target[0] == "geometry":
            L["geometry"] = geom
        else:
            kind = target[2]
            m = L.get("mask")
            m = m[0] if isinstance(m, list) and m else (dict(m) if m else {})
            if m.get("kind") != kind:
                m = {"kind": kind,
                     "feather_m": float(self._props.mask_feather.value()),
                     "invert": bool(self._props.mask_invert.isChecked())}
                if kind == "polyline_band":
                    m["width_m"] = float(self._props.mask_width.value())
                if kind == "gradient":
                    m["profile"] = "linear"
            L["mask"] = self._shape_into_mask(m, geom)
        self._props.show_layer(L)
        self.map.set_layer_geometries(self._layer_geometries())
        self._push("Draw shape", before)
        self._schedule()

    def _on_geometry_changed(self, gid: str, geom: dict) -> None:
        before = self._snapshot()
        layer_id, _, tail = gid.partition(":")
        try:
            L = self._doc.get(layer_id)
        except KeyError:
            return
        if tail == "mask":
            m = L.get("mask")
            m = m[0] if isinstance(m, list) and m else m
            if m:
                L["mask"] = self._shape_into_mask(m, geom)
        else:
            L["geometry"] = geom
        if layer_id == self._selected_key:
            self._props.show_layer(L)
        self._push("Move handle", before, merge_key=gid)
        self._schedule()

    def _on_section(self, line) -> None:
        if not self._preview:
            self._info.setText("section: no preview yet")
            return
        import numpy as np
        from scipy import ndimage
        (x0, y0), (x1, y1) = line
        g = self._preview["grid"]
        n = 200
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, n)
        cols = (xs - g.origin_x) / g.resolution
        rows = (ys - g.origin_y) / g.resolution
        z = ndimage.map_coordinates(self._preview["height"], [rows, cols],
                                    order=1, mode="nearest")
        inside = ((cols >= 0) & (cols <= g.nx - 1) & (rows >= 0)
                  & (rows <= g.ny - 1))
        z = np.where(inside, -z, np.nan)
        dist = np.hypot(xs - x0, ys - y0)
        self._section.set_profile(dist, z)
        self._info.setText(f"section {dist[-1]:.1f} m: depth "
                           f"{np.nanmin(z):.2f}–{np.nanmax(z):.2f} m")

    # --------------------------------------------------------- brushes
    def _on_strokes_changed(self) -> None:
        before = self._snapshot()
        self._flush_strokes()
        # A material stroke on a uniform seabed means "painted": switch
        # the layout so the intent is visible in the form (and the YAML).
        if self.map.strokes("material"):
            flat = self._tex_form.values()
            if flat.get("materials.layout") == "uniform":
                self._tex_form.set_values({"materials.layout": "painted"})
        self._brush_info.setText(f"{len(self._base_paint)} base depth strokes")
        self._mat_info.setText(f"{len(self._base_material_paint)} material strokes")
        self._push("Brush stroke", before)
        self._schedule()

    def _clear_strokes(self) -> None:
        mode = self.map.mode
        kind = ("material" if mode is Mode.MATERIAL_BRUSH
                else "mask" if mode is Mode.MASK_BRUSH else "depth")
        self.map.clear_strokes(kind)

    def _collect(self, state: BuilderState) -> DepthState:
        self._flush_strokes()
        texture = unflatten_texture(self._tex_form.values())
        material = str(texture["materials"]["uniform"])
        common = dict(texture=texture, material=material,
                      paint=copy.deepcopy(self._base_paint),
                      material_paint=copy.deepcopy(self._base_material_paint),
                      layers=self._doc.to_list())
        if self._r_file.isChecked():
            return DepthState(mode="file", file=self._file_edit.text().strip(),
                              file_values=self._values_combo.currentText(),
                              **common)
        if self._r_svlog.isChecked():
            knobs = {k: float(sp.value()) for k, sp in self._svlog_spins.items()}
            knobs["relief"] = self._svlog_relief.currentText()
            # Kept for older states and for the folder's audit record; the
            # layer row is what applies the measurement.
            knobs["backscatter_layer"] = SURVEY_LAYER_ID in self._doc.ids()
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
            self._imgs[key].set_image(img)
        self._stats.setText(stats)
        if self._worker is not None and getattr(self._worker, "result", None):
            self._preview = self._worker.result
            self._update_overlay()
        self._show_survey(info if isinstance(info, dict) else {})

    def _update_overlay(self) -> None:
        """The chosen composite channel drawn over the map."""
        res = self._preview
        choice = self._overlay.currentText()
        if not res or choice == "none":
            self.map.set_overlay(None, key="composite")
            return
        import numpy as np
        g = res["grid"]
        hint: dict = {"origin": (g.origin_x, g.origin_y), "cell_m": g.resolution}
        if choice == "depth":
            depth = -res["height"]
            lo, hi = float(depth.min()), float(depth.max())
            hint["values"] = 1.0 - (depth - lo) / max(hi - lo, 1e-9)
        elif choice == "sonar look":
            # cos(incidence) against a grazing ray -- the quantity the
            # renderer shades with. Ripples are 6 cm against metres of
            # bathymetry, so the depth stretch cannot show them and this
            # can: it is the difference between authoring bedforms blind
            # and seeing them before a build.
            from ..imaging import sonar_shading
            v = sonar_shading(res["height"], g.resolution)
            lo2 = float(np.nanpercentile(v, 2))
            hi2 = float(np.nanpercentile(v, 98))
            hint["values"] = np.clip((v - lo2) / max(hi2 - lo2, 1e-6),
                                     0.0, 1.0)
        elif choice == "reflectivity":
            refl = res["reflectivity"]
            v = np.clip(refl / max(1.0, float(np.nanmax(refl))), 0.0, 1.0)
            rgba = np.zeros(v.shape + (4,), dtype=np.uint8)
            rgba[..., :3] = (40 + 200 * v)[..., None].astype(np.uint8)
            rgba[..., 3] = 150
            hint["rgba"] = rgba
        else:
            ids = res["material_id"]
            rgba = np.zeros(ids.shape + (4,), dtype=np.uint8)
            for i, name in enumerate(res["material_names"]):
                c = _material_color(name)
                sel = ids == i
                rgba[sel, 0], rgba[sel, 1], rgba[sel, 2] = c.red(), c.green(), c.blue()
            rgba[..., 3] = 150
            hint["rgba"] = rgba
        self.map.set_overlay(hint, key="composite")

    def _show_survey(self, info: dict) -> None:
        """The sailed track + soundings on the map, the quality line, and
        the measured-reflectivity line when the survey layer ran."""
        from ..worker import survey_stats_line
        # The coverage line (and its warning) belongs to every positioned
        # source, so it is set before the recording-only parts.
        self._set_quality(survey_stats_line(info))
        arrays = info.get("soundings_arrays") if info else None
        if not arrays:
            self.map.set_track(None)
            self.map.set_overlay(None)
            return
        self.map.set_track(arrays["east"], arrays["north"], arrays["depth"])
        self.map.set_overlay(None)

    def _set_quality(self, text: str) -> None:
        self._svlog_quality.setText(text)
        self._svlog_quality.setToolTip(text)
        self._prop_scroll.refit()

    def _fail(self, msg: str) -> None:
        self._last_key = None                # retry on the next change
        self._stats.setText(f"preview failed: {msg}")
        # Never leave the previous recording's quality line standing under
        # a failure -- for the "does not overlap" case that line is the
        # very thing the message contradicts.
        self._set_quality(msg if self._r_svlog.isChecked() else "")
        self.map.set_track(None)
        self.map.set_overlay(None)

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
            # A fresh state (no recording chosen yet, no relief key) gets
            # the measured relief; an existing recording state without the
            # key keeps the synthetic undulation it was built with.
            fresh = "relief" not in (d.svlog or {}) and not d.file
            self._svlog_relief.setCurrentText(
                "shading" if fresh else str(knobs["relief"]))
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
            self._doc = LayerDocument(d.layers)
            if (d.mode == "svlog" and bool(knobs["backscatter_layer"])
                    and SURVEY_LAYER_ID not in self._doc.ids()):
                # An older state carried the flag alone (the disc-splat
                # hint, retired 2026-09-16): give it the layer that does
                # the job now. Folders are immutable, so nothing on disk
                # changes until the user saves a new one.
                self._doc.add("reflectivity_survey", layer_id=SURVEY_LAYER_ID,
                              name="Reflectivity from recording")
                self._doc.move(SURVEY_LAYER_ID, -len(self._doc.layers))
            self._base_paint = copy.deepcopy(list(d.paint or []))
            self._base_material_paint = copy.deepcopy(list(d.material_paint or []))
            self._undo.clear()
            self._selected_key = "source"
            self._geom_target = None
            self._preview = None
            self._section.clear()
            self._refresh_all()
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
        if d.layers:
            try:
                normalize_layers(stack_from_depth(d))
            except (KeyError, ValueError, TypeError) as exc:
                QMessageBox.warning(self, "Seabed layers", str(exc))
                return False
            missing = [L.get("name", L["kind"]) for L in d.layers
                       if LAYER_KINDS[L["kind"]].geometry and not L.get("geometry")]
            if missing:
                QMessageBox.warning(self, "Seabed layers",
                                    "Draw the shape of: " + ", ".join(missing))
                return False
        state.depth = d
        return True
