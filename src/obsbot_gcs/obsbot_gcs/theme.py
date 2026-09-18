"""Shared palette and small painting helpers for the GCS."""

from __future__ import annotations

from PyQt5.QtGui import QColor

BG = QColor("#0d1117")
PANEL = QColor("#161b22")
BORDER = QColor("#30363d")
TEXT = QColor("#c9d1d9")
MUTED = QColor("#8b949e")
ACCENT = QColor("#58a6ff")
OK = QColor("#3fb950")
WARN = QColor("#d29922")
DANGER = QColor("#f85149")
HUD = QColor(88, 166, 255, 200)
HUD_DIM = QColor(139, 148, 158, 120)

STYLESHEET = f"""
QWidget {{
    background: {BG.name()};
    color: {TEXT.name()};
    font-family: "DejaVu Sans Mono", monospace;
    font-size: 12px;
}}
QGroupBox {{
    background: {PANEL.name()};
    border: 1px solid {BORDER.name()};
    border-radius: 6px;
    margin-top: 16px;
    padding-top: 8px;
    font-weight: bold;
    color: {MUTED.name()};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}}
QPushButton {{
    background: {PANEL.name()};
    border: 1px solid {BORDER.name()};
    border-radius: 4px;
    padding: 7px 12px;
    color: {TEXT.name()};
}}
QPushButton:hover {{ border-color: {ACCENT.name()}; }}
QPushButton:pressed {{ background: {BORDER.name()}; }}
QPushButton:checked {{
    background: {WARN.name()};
    color: {BG.name()};
    border-color: {WARN.name()};
    font-weight: bold;
}}
QPushButton#danger:hover {{ border-color: {DANGER.name()}; color: {DANGER.name()}; }}
QLabel#value {{ font-size: 24px; font-weight: bold; color: {TEXT.name()}; }}
QLabel#unit  {{ font-size: 11px; color: {MUTED.name()}; }}
QWidget#speedbar {{
    background: rgba(22, 27, 34, 230);
    border-bottom: 1px solid {BORDER.name()};
}}
QWidget#speedbar QLabel {{ background: transparent; font-size: 18px; font-weight: bold; }}
QWidget#speedbar QLabel#speedvalue {{ color: {ACCENT.name()}; font-size: 24px; min-width: 80px; }}
QSlider::groove:horizontal {{
    height: 10px; background: {BORDER.name()}; border-radius: 5px;
}}
QSlider::sub-page:horizontal {{ background: {ACCENT.name()}; border-radius: 4px; }}
QSlider::handle:horizontal {{
    width: 26px; margin: -9px 0; background: {TEXT.name()};
    border: 2px solid {ACCENT.name()}; border-radius: 13px;
}}
"""
