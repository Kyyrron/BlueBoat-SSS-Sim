"""Minimal dark theme for the world builder (self-contained; the MCS
theme module is not importable across submodules -- CM-3)."""

from __future__ import annotations

BG_DARK = "#14181d"
BG_PANEL = "#1c2128"
TEXT = "#e6ebf0"
TEXT_DIM = "#969ea8"
ACCENT = "#2f81f7"
OK = "#3fb950"
WARN = "#e3b341"
ERR = "#f85149"

C_GRID = "#2a3038"
C_GRID_MAJOR = "#3a424c"
C_PATH = "#3fb950"          # source GPS path
C_LIMITS = "#fad25c"        # world limits rectangle
C_WALL = "#50aaff"          # reflective walls
C_WALL_SEL = "#f85149"
C_OBJECT = "#ff5050"

STYLESHEET = f"""
QMainWindow, QDialog {{ background: {BG_DARK}; color: {TEXT}; }}
QWidget {{ color: {TEXT}; font-size: 13px; }}
QGroupBox {{
    border: 1px solid {C_GRID_MAJOR}; border-radius: 4px;
    margin-top: 8px; padding-top: 14px; background: {BG_PANEL};
}}
QGroupBox::title {{ subcontrol-origin: margin; left: 8px; color: {TEXT_DIM}; }}
QPushButton {{
    background: {BG_PANEL}; border: 1px solid {C_GRID_MAJOR};
    border-radius: 4px; padding: 5px 14px;
}}
QPushButton:hover {{ border-color: {ACCENT}; }}
QPushButton:checked {{ background: {ACCENT}; color: white; }}
QPushButton:default {{ background: {ACCENT}; color: white; }}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QListWidget {{
    background: {BG_DARK}; border: 1px solid {C_GRID_MAJOR};
    border-radius: 3px; padding: 3px;
}}
QLabel#page_title {{ font-size: 17px; font-weight: bold; }}
QLabel#hint {{ color: {TEXT_DIM}; }}
QStatusBar {{ color: {TEXT_DIM}; }}
QScrollArea {{ border: none; }}
"""
