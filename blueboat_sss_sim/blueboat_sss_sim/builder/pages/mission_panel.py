"""Screen 5, Mission-like mode: the panel of controls (simulator v1.1,
2026-11-21).

One row is one decision, and what it does shows at once: the expected share
of each class beside its box, the expected number of objects beside the
density slider, the warnings under the controls. With no click at all the
defaults give a good world: the harbour type "Auto, from the features", all
seven target classes, level 5 and a random seed that is shown. The names and
values are those of a specification's ``placement:`` block
(``PlacementParams`` of ``blueboat_harbour_population``); :meth:`block`
returns that block.

The panel holds no model: the page plans (``PlanWorker``) and hands the
report back with :meth:`set_result`. It never opens a dialog and never
touches the map.
"""

from __future__ import annotations

import random

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QFrame,
                               QGridLayout, QHBoxLayout, QLabel, QPushButton, QRadioButton, QSlider,
                               QSpinBox, QVBoxLayout, QWidget)

#: the density slider's three landmarks, in plain words
LEVEL_WORDS = {1: "cleaned", 2: "managed", 3: "lightly littered", 4: "below typical", 5: "typical",
               6: "above typical", 7: "heavily littered", 8: "heavily littered", 9: "neglected",
               10: "neglected"}


def _chip(colour: str) -> QLabel:
    lab = QLabel()
    lab.setFixedSize(10, 10)
    lab.setStyleSheet(f"background: {colour}; border-radius: 2px;")
    return lab


class MissionPanel(QWidget):
    """The Mission-like controls. ``changed`` fires on every change that
    asks for a new plan."""

    changed = Signal()
    convert_requested = Signal()
    replan_requested = Signal()
    heat_toggled = Signal(bool)

    def __init__(self, planner, parent=None) -> None:
        super().__init__(parent)
        self._hp = planner
        self._quiet = 0
        self._feature_rows: dict[str, tuple] = {}
        self._defaults: dict[str, str] = {}
        p = planner.default_parameters()
        from blueboat_harbour_population import card
        self._card = card
        self._tags = ["none"] + [t for t in p.tags if not t.startswith("generic_")]
        self._all_tags = ["none"] + list(p.tags)
        col = QVBoxLayout(self)
        col.setContentsMargins(0, 0, 6, 0)
        col.setSpacing(6)

        form = QFormLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        # ---- harbour type
        self.profile = QComboBox()
        # the long names must not set the panel's width (the screen fits 1000 x 600)
        self.profile.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.profile.setMinimumContentsLength(14)
        order = ["auto"] + [q for q in p.profiles if q != "auto"]
        for name in order:
            self.profile.addItem(str(p(f"profiles.{name}.label")), name)
        self.profile.setToolTip("What kind of harbour this is: it decides the default use of every quay, "
                                "pontoon and mooring, and the mix of objects. 'Auto' uses each feature's "
                                "own use (Advanced).")
        self.profile.currentIndexChanged.connect(self._emit)
        form.addRow("Harbour type", self.profile)
        # ---- classes
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(1)
        self.classes: dict[str, QCheckBox] = {}
        for k, c in enumerate(planner.TARGETS):
            box = QCheckBox(card.CLASS_SHORT[c])
            box.setChecked(True)
            box.setToolTip(f"Place objects of the class {c}. A class that is not ticked is absent; the "
                           "others share the total.")
            box.toggled.connect(self._emit)
            self.classes[c] = box
            cell = QHBoxLayout()
            cell.setSpacing(3)
            cell.addWidget(_chip(card.CLASS_COLOUR_DARK[c]))
            cell.addWidget(box, 1)
            grid.addLayout(cell, k // 2, k % 2)
        wrap = QWidget()
        wrap.setLayout(grid)
        form.addRow("Objects", wrap)
        # ---- mix
        self.mix_real = QRadioButton("Realistic")
        self.mix_bal = QRadioButton("Balanced")
        self.mix_real.setChecked(True)
        self.mix_real.setToolTip("Each class in the share this harbour type has in documented harbours.")
        self.mix_bal.setToolTip("The same number of every ticked class (for results per class).")
        grp = QButtonGroup(self)
        grp.addButton(self.mix_real)
        grp.addButton(self.mix_bal)
        self.mix_real.toggled.connect(self._emit)
        row = QHBoxLayout()
        row.addWidget(self.mix_real)
        row.addWidget(self.mix_bal)
        row.addStretch(1)
        w = QWidget()
        w.setLayout(row)
        form.addRow("Mix", w)
        # ---- density
        self.level = QSlider(Qt.Orientation.Horizontal)
        self.level.setRange(1, 10)
        self.level.setValue(5)
        self.level.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.level.setTickInterval(1)
        self.level.setPageStep(1)
        self.level.setToolTip("1 cleaned · 5 typical · 10 neglected. Level 5 is the density of sonar-size "
                              "litter documented in harbours that were never cleaned (about 10 per "
                              "hectare near the quays); every level stays realistic.")
        self.level.valueChanged.connect(self._on_level)
        form.addRow("Density", self.level)
        self.level_text = QLabel("5 · typical")
        self.level_text.setObjectName("hint")
        self.level_text.setWordWrap(True)
        form.addRow("", self.level_text)
        # ---- seed and split
        self.seed = QSpinBox()
        self.seed.setRange(1, 399999)
        self.seed.setToolTip("The placement seed: the same seed and inputs always give the same world. "
                             "It is separate from the world's own seed (Screen 4), which seeds the seabed.")
        self.seed.valueChanged.connect(self._on_seed)
        self.b_roll = QPushButton("Roll")
        self.b_roll.setToolTip("Another seed of the chosen split.")
        self.b_roll.clicked.connect(self.roll_seed)
        row = QHBoxLayout()
        row.addWidget(self.seed, 1)
        row.addWidget(self.b_roll)
        w = QWidget()
        w.setLayout(row)
        form.addRow("Seed", w)
        self.split = QComboBox()
        for name in planner.data.default_splits()["splits"]:
            self.split.addItem(name)
        self.split.setToolTip("Which worlds this one belongs to. Each split has its own range of seeds, so "
                              "that dataset, tuning and evaluation worlds never share one.")
        self.split.currentIndexChanged.connect(self._on_split)
        self.split_text = QLabel("")
        self.split_text.setObjectName("hint")
        row = QHBoxLayout()
        row.addWidget(self.split, 1)
        row.addWidget(self.split_text)
        w = QWidget()
        w.setLayout(row)
        form.addRow("Split", w)
        # ---- also add
        self.debris = QCheckBox("Debris")
        self.litter = QCheckBox("Small litter")
        self.structures = QCheckBox("Structures it needs")
        for b, tip in ((self.debris, "Scrap and rubble of 0.2 to 1.2 m: real litter that is not a target."),
                       (self.litter, "Bottles and cans as a seabed layer: the seabed's own false alarms. "
                                     "Its density follows the level."),
                       (self.structures, "Mooring blocks with their chains and the piles of a pontoon, "
                                         "where this harbour type has them and the world does not.")):
            b.setChecked(True)
            b.setToolTip(tip)
            b.toggled.connect(self._emit)
        box = QVBoxLayout()
        box.setSpacing(1)
        row = QHBoxLayout()
        row.addWidget(self.debris)
        row.addWidget(self.litter)
        row.addStretch(1)
        box.addLayout(row)
        box.addWidget(self.structures)
        w = QWidget()
        w.setLayout(box)
        form.addRow("Also add", w)
        col.addLayout(form)

        # ---- advanced (closed by default)
        self.b_adv = QPushButton("▸ Advanced")
        self.b_adv.setCheckable(True)
        self.b_adv.setStyleSheet("text-align: left;")
        self.b_adv.setToolTip("Burial, groups, storms, spacing, uncertain materials, the use of each "
                              "feature, the expected-density map.")
        self.b_adv.toggled.connect(self._toggle_advanced)
        col.addWidget(self.b_adv)
        self.adv = QFrame()
        adv = QFormLayout(self.adv)
        adv.setContentsMargins(6, 0, 0, 0)
        adv.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self.burial = QComboBox()
        self.burial.addItems(["low", "nominal", "high"])
        self.burial.setCurrentText("nominal")
        self.burial.setToolTip("How deep objects sink into the sediment: each class's own range, scaled by "
                               "0.5, 1 or 1.5.")
        self.burial.currentIndexChanged.connect(self._emit)
        adv.addRow("Burial", self.burial)
        self.groups = QCheckBox("Lines, piles and gear balls")
        self.groups.setChecked(True)
        self.groups.setToolTip("Let some objects come as groups: a line of fender tyres, a dumping pile, "
                               "a ball of traps and rope. More of them at a higher level.")
        self.groups.toggled.connect(self._emit)
        adv.addRow("Groups", self.groups)
        self.storms = QCheckBox("Move light objects into the lows")
        self.storms.setToolTip("A share of the crates and nets moves a few metres down the local slope, "
                               "as storms carry them.")
        self.storms.toggled.connect(self._emit)
        adv.addRow("Storms", self.storms)
        self.spacing = QDoubleSpinBox()
        self.spacing.setRange(0.5, 20.0)
        self.spacing.setSingleStep(0.5)
        self.spacing.setValue(3.0)
        self.spacing.setSuffix(" m")
        self.spacing.setToolTip("Least distance between two targets that are not in one group, so that "
                                "their echoes do not merge.")
        self.spacing.valueChanged.connect(self._emit)
        adv.addRow("Spacing", self.spacing)
        self.r6 = QComboBox()
        self.r6.addItems(["low", "nominal", "high"])
        self.r6.setCurrentText("nominal")
        self.r6.setToolTip("Uncertain materials (tyre belts, net fibre): the low, nominal or high end of "
                           "their bracket. Datasets are rendered at the low end.")
        self.r6.currentIndexChanged.connect(self._emit)
        adv.addRow("Materials", self.r6)
        self.heat = QCheckBox("Show the expected density")
        self.heat.setToolTip("Colour the map by the density the model expects (objects per hectare).")
        self.heat.toggled.connect(self.heat_toggled.emit)
        adv.addRow("Map", self.heat)
        self.features_title = QLabel("Use of each feature (double-click a wall on the map to set it)")
        self.features_title.setObjectName("hint")
        self.features_title.setWordWrap(True)
        adv.addRow(self.features_title)
        self._adv_form = adv
        self.adv.setVisible(False)
        col.addWidget(self.adv)

        # ---- what comes out
        self.warnings = QLabel("")
        self.warnings.setObjectName("hint")
        self.warnings.setWordWrap(True)
        col.addWidget(self.warnings)
        row = QHBoxLayout()
        self.b_replan = QPushButton("Re-plan")
        self.b_replan.setToolTip("Plan again on the world as it is now (the stored plan was made on an "
                                 "earlier state of it).")
        self.b_replan.setVisible(False)
        self.b_replan.clicked.connect(self.replan_requested.emit)
        self.b_convert = QPushButton("Convert to manual")
        self.b_convert.setToolTip("Turn this plan into an ordinary list of objects that can be moved, "
                                  "edited and deleted one by one. The world then records that it started "
                                  "as a Mission-like plan and whether it was edited.")
        self.b_convert.clicked.connect(self.convert_requested.emit)
        row.addWidget(self.b_replan)
        row.addStretch(1)
        row.addWidget(self.b_convert)
        col.addLayout(row)
        col.addStretch(1)
        self.setMinimumWidth(250)
        self.roll_seed(quiet=True)

    # ----------------------------------------------------------- plumbing
    def _emit(self, *_a) -> None:
        if not self._quiet:
            self.changed.emit()

    def _toggle_advanced(self, on: bool) -> None:
        self.b_adv.setText("▾ Advanced" if on else "▸ Advanced")
        self.adv.setVisible(on)

    def _on_level(self, v: int) -> None:
        self.level_text.setText(f"{v} · {LEVEL_WORDS[v]}")
        self._emit()

    def _on_split(self, *_a) -> None:
        self._check_seed()
        self._emit()

    def _on_seed(self, *_a) -> None:
        self._check_seed()
        self._emit()

    def _check_seed(self) -> None:
        from blueboat_harbour_population import splits
        own = splits.split_of_seed(self.seed.value())
        want = self.split.currentText()
        if own == want:
            self.split_text.setText("✓")
            self.split_text.setToolTip("The seed lies in this split's range.")
        else:
            lo, hi = splits.seed_range(want)
            self.split_text.setText(f"✗ seeds {lo}–{hi}")
            self.split_text.setToolTip(f"This seed belongs to {own or 'no split'}: Roll gives one of "
                                       f"{want}.")

    def roll_seed(self, *_a, quiet: bool = False) -> None:
        """A random seed of the chosen split (the randomness is only in the
        choice: the seed is shown and recorded)."""
        from blueboat_harbour_population import splits
        lo, hi = splits.seed_range(self.split.currentText())
        self._quiet += 1
        try:
            self.seed.setValue(random.SystemRandom().randint(lo, hi))
        finally:
            self._quiet -= 1
        self._check_seed()
        if not quiet:
            self._emit()

    # --------------------------------------------------------------- block
    def block(self) -> dict:
        """The ``placement:`` block the controls describe."""
        tags, act = {}, {}
        for fid, (combo, spin) in self._feature_rows.items():
            use = combo.currentData()
            if use:
                tags[fid] = use
            if abs(spin.value() - 1.0) > 1e-9 and spin.isEnabled():
                act[fid] = round(float(spin.value()), 2)
        blk = {
            "mode": "mission_like",
            "profile": self.profile.currentData(),
            "classes": [c for c, b in self.classes.items() if b.isChecked()],
            "mix": "balanced" if self.mix_bal.isChecked() else "realistic",
            "density_level": int(self.level.value()),
            "seed": int(self.seed.value()),
            "split": self.split.currentText(),
            "clutter": {"debris": self.debris.isChecked(), "small_litter": self.litter.isChecked()},
            "structures": "auto" if self.structures.isChecked() else "keep",
            "advanced": {"burial": self.burial.currentText(), "groups": self.groups.isChecked(),
                         "redistribution": self.storms.isChecked(),
                         "min_spacing_m": round(float(self.spacing.value()), 2),
                         "uncertain_materials": self.r6.currentText()},
        }
        if tags:
            blk["feature_tags"] = tags
        if act:
            blk["feature_activity"] = act
        return blk

    def set_block(self, blk: dict | None) -> None:
        """Show a stored block (no ``changed`` signal)."""
        blk = dict(blk or {})
        self._quiet += 1
        try:
            i = self.profile.findData(blk.get("profile", "auto"))
            self.profile.setCurrentIndex(max(i, 0))
            sel = blk.get("classes")
            for c, b in self.classes.items():
                b.setChecked(sel is None or c in sel)
            (self.mix_bal if blk.get("mix") == "balanced" else self.mix_real).setChecked(True)
            self.level.setValue(int(blk.get("density_level", 5)))
            self.level_text.setText(f"{self.level.value()} · {LEVEL_WORDS[self.level.value()]}")
            if blk.get("split"):
                self.split.setCurrentText(str(blk["split"]))
            if blk.get("seed") is not None:
                self.seed.setValue(int(blk["seed"]))
            cl = dict(blk.get("clutter") or {})
            self.debris.setChecked(bool(cl.get("debris", True)))
            self.litter.setChecked(bool(cl.get("small_litter", True)))
            self.structures.setChecked(blk.get("structures", "auto") == "auto")
            adv = dict(blk.get("advanced") or {})
            self.burial.setCurrentText(str(adv.get("burial", "nominal")))
            self.groups.setChecked(bool(adv.get("groups", True)))
            self.storms.setChecked(bool(adv.get("redistribution", False)))
            self.spacing.setValue(float(adv.get("min_spacing_m", 3.0)))
            self.r6.setCurrentText(str(adv.get("uncertain_materials", "nominal")))
            self._stored_tags = dict(blk.get("feature_tags") or {})
            self._stored_activity = dict(blk.get("feature_activity") or {})
            self._apply_feature_values()
        finally:
            self._quiet -= 1
        self._check_seed()

    # ------------------------------------------------------------ features
    _stored_tags: dict = {}
    _stored_activity: dict = {}

    def set_features(self, feats: list) -> None:
        """The features of the environment: ``(id, kind, default use)``.
        One row each: its use (the first entry is the harbour type's
        default) and its relative activity."""
        keep_tags = {fid: c.currentData() for fid, (c, _s) in self._feature_rows.items() if c.currentData()}
        keep_act = {fid: s.value() for fid, (_c, s) in self._feature_rows.items() if abs(s.value() - 1.0) > 1e-9}
        self._stored_tags = {**self._stored_tags, **keep_tags}
        self._stored_activity = {**self._stored_activity, **keep_act}
        self._quiet += 1
        try:
            for fid, (combo, spin) in list(self._feature_rows.items()):
                self._adv_form.removeRow(combo.parentWidget())
            self._feature_rows = {}
            self._defaults = {}
            for fid, kind, default in feats:
                combo = QComboBox()
                combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
                combo.setMinimumContentsLength(10)
                combo.addItem(f"default ({default.replace('_', ' ')})", "")
                for t in self._tags:
                    combo.addItem(t.replace("_", " "), t)
                combo.setToolTip(f"What {fid} ({kind}) is used for: it decides which objects lie near it. "
                                 "'none' makes it no source of litter.")
                spin = QDoubleSpinBox()
                spin.setRange(0.0, 5.0)
                spin.setSingleStep(0.5)
                spin.setDecimals(1)
                spin.setValue(1.0)
                spin.setToolTip("Relative amount of litter from this feature (one side of a basin can be "
                                "much dirtier than the other).")
                combo.currentIndexChanged.connect(self._emit)
                spin.valueChanged.connect(self._emit)
                row = QHBoxLayout()
                row.setContentsMargins(0, 0, 0, 0)
                row.addWidget(combo, 1)
                row.addWidget(spin)
                w = QWidget()
                w.setLayout(row)
                self._adv_form.addRow(fid, w)
                self._feature_rows[fid] = (combo, spin)
                self._defaults[fid] = default
            self._apply_feature_values()
        finally:
            self._quiet -= 1

    def _apply_feature_values(self) -> None:
        for fid, (combo, spin) in self._feature_rows.items():
            use = self._stored_tags.get(fid, "")
            i = combo.findData(use)
            if i < 0 and use:
                combo.addItem(use.replace("_", " "), use)
                i = combo.findData(use)
            combo.setCurrentIndex(max(i, 0))
            spin.setValue(float(self._stored_activity.get(fid, 1.0)))

    def feature_ids(self) -> list:
        return list(self._feature_rows)

    def set_feature_use(self, fid: str, use: str) -> None:
        """Set one feature's use (from a double-click on the map)."""
        if fid not in self._feature_rows:
            return
        combo = self._feature_rows[fid][0]
        i = combo.findData(use)
        if i >= 0:
            combo.setCurrentIndex(i)

    def use_choices(self) -> list:
        return list(self._tags)

    # -------------------------------------------------------------- result
    def set_result(self, report: dict | None, busy: str = "", error: str = "") -> None:
        """Show what a plan holds: the expected share beside each class, the
        expected and placed counts under the slider, the warnings."""
        card = self._card
        v = self.level.value()
        head = f"{v} · {LEVEL_WORDS[v]}"
        if report is None:
            for c, b in self.classes.items():
                b.setText(card.CLASS_SHORT[c])
            self.level_text.setText(head + (f" — {busy}" if busy else ""))
            self.warnings.setText(("⚠ " + error) if error else "")
            self.b_convert.setEnabled(False)
            return
        shares = dict(report.get("expected_shares") or {})
        for c, b in self.classes.items():
            b.setText(card.CLASS_SHORT[c] + (f" {100.0 * shares[c]:.0f} %" if c in shares else ""))
        ex, rz = report["expected"], report["realised"]
        n = rz["targets"] + rz["debris"]
        self.level_text.setText(
            f"{head} — about {ex['total']:.0f} objects expected "
            f"({ex['targets']:.0f} targets, {ex['debris']:.0f} debris); this seed: {n} "
            f"({rz['targets']} targets, {rz['debris']} debris"
            + (f", {rz['groups']} group(s)" if rz["groups"] else "") + ")"
            + (f" — {busy}" if busy else ""))
        warn = list(report.get("warnings") or [])
        self.warnings.setText("Warnings: none" if not warn and not error else
                              "\n".join(["⚠ " + error] if error else []) + "\n".join("⚠ " + w for w in warn))
        self.b_convert.setEnabled(not busy and not error)

    def set_stale(self, on: bool, text: str = "") -> None:
        self.b_replan.setVisible(bool(on))
        if on and text:
            self.warnings.setText("⚠ " + text)
