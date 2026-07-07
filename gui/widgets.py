import math
from pathlib import Path

from PyQt6.QtWidgets import (
    QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QSlider, QComboBox,
    QCheckBox,
)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui import QAction
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush


class AssetCombo(QComboBox):
    """Editable combo that re-scans an assets directory each time the popup opens."""

    def __init__(self, assets_dir: Path, theme: dict, placeholder: str = "type or select…",
                 parent=None):
        super().__init__(parent)
        self._dir = assets_dir
        t = theme
        self.setEditable(True)
        self.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.setStyleSheet(
            f"QComboBox {{"
            f"  background: {t['input_bg']}; color: {t['input_text']};"
            f"  border: 1px solid {t['input_border']}; border-radius: 3px;"
            f"  padding: 4px 8px; font-size: 13px;"
            f"}}"
            f"QComboBox QAbstractItemView {{"
            f"  background: {t['input_bg']}; color: {t['input_text']};"
            f"  border: 1px solid {t['input_border']};"
            f"  selection-background-color: {t['btn_active_bg']};"
            f"  selection-color: {t['btn_active_text']}; outline: none;"
            f"}}"
        )
        self.lineEdit().setPlaceholderText(placeholder)
        self.lineEdit().setStyleSheet(
            f"background: transparent; color: {t['input_text']};"
            f"border: none; padding: 0px; font-size: 13px;"
        )

    def showPopup(self) -> None:
        current = self.currentText()
        self.blockSignals(True)
        self.clear()
        if self._dir.exists():
            self.addItems(sorted(p.stem for p in self._dir.glob("*.json")))
        self.setCurrentText(current)
        self.blockSignals(False)
        super().showPopup()


class TabSelectorWidget(QWidget):
    """Custom tab bar: tab buttons on the left, reload icon pinned to the right."""

    tab_changed    = pyqtSignal(int)
    reload_clicked = pyqtSignal()

    def __init__(self, tabs: list[dict], theme: dict, height: int, parent=None):
        super().__init__(parent)
        self.setFixedHeight(height)

        t = theme
        bg   = t["tab_bar_bg"]
        self.setStyleSheet(f"background: {bg}; border: none;")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        inactive_style = f"""
            QPushButton {{
                background: {t['tab_inactive_bg']};
                color: {t['tab_inactive_text']};
                border: none;
                border-bottom: 3px solid transparent;
                padding: 0px 22px;
                font-size: 14px;
                font-family: 'Helvetica Neue', Arial;
                font-weight: 500;
                height: {height}px;
            }}
            QPushButton:hover:!checked {{
                background: {t['tab_hover_bg']};
                color: {t['tab_hover_text']};
            }}
        """

        self._buttons: list[QPushButton] = []
        for i, tab in enumerate(tabs):
            raw   = tab.get("color", t.get("accent", "#3d7eff"))
            color = t.get(raw, raw)   # resolve token name → hex, or pass through
            btn = QPushButton(tab["label"])
            btn.setCheckable(True)
            btn.setStyleSheet(
                inactive_style +
                f"QPushButton:checked {{"
                f"  background: {t['tab_active_bg']};"
                f"  color: {color};"
                f"  border-bottom: 3px solid {color};"
                f"}}"
            )
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.clicked.connect(lambda _checked, idx=i: self._on_tab(idx))
            layout.addWidget(btn)
            self._buttons.append(btn)

        layout.addStretch()

        reload_style = f"""
            QPushButton {{
                background: {t['reload_bg']};
                color: {t['reload_text']};
                border: none;
                border-left: 1px solid {t['border']};
                font-size: 18px;
                width: 44px;
                height: {height}px;
                padding: 0px;
            }}
            QPushButton:hover {{
                background: {t['reload_hover_bg']};
                color: {t['reload_hover_text']};
            }}
        """
        reload_btn = QPushButton("↺")
        reload_btn.setFixedWidth(44)
        reload_btn.setToolTip("Reload Application")
        reload_btn.setStyleSheet(reload_style)
        reload_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        reload_btn.clicked.connect(self.reload_clicked)
        layout.addWidget(reload_btn)

        self._buttons[0].setChecked(True)

    def _on_tab(self, idx: int) -> None:
        for i, btn in enumerate(self._buttons):
            btn.setChecked(i == idx)
        self.tab_changed.emit(idx)

    def set_active(self, idx: int) -> None:
        self._on_tab(idx)


class MenuBarWidget(QWidget):
    """Thin top bar hosting a native QMenuBar on the left."""

    vehicle_new_triggered     = pyqtSignal()
    vehicle_open_triggered    = pyqtSignal()
    vehicle_save_triggered    = pyqtSignal()
    vehicle_save_as_triggered = pyqtSignal()

    def __init__(self, theme: dict, height: int, parent=None):
        super().__init__(parent)
        self.setFixedHeight(height)

        t = theme
        self.setStyleSheet(f"background: {t['menu_bg']};")

        from PyQt6.QtWidgets import QMenuBar
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        menu_bar = QMenuBar(self)
        menu_bar.setStyleSheet(f"""
            QMenuBar {{
                background: transparent;
                color: {t['menu_text']};
                font-size: 13px;
                font-family: 'Helvetica Neue', Arial;
                padding: 0px 4px;
            }}
            QMenuBar::item {{
                background: transparent;
                padding: 4px 10px;
            }}
            QMenuBar::item:selected {{
                background: {t['tab_hover_bg']};
                color: {t['tab_active_text']};
            }}
            QMenu {{
                background: {t['menu_bg']};
                color: {t['menu_text']};
                border: 1px solid {t['border']};
                font-size: 13px;
            }}
            QMenu::item:selected {{
                background: {t['tab_active_bg']};
                color: {t['tab_active_text']};
            }}
        """)

        file_menu = menu_bar.addMenu("File")
        file_menu.addAction("Reload")
        file_menu.addSeparator()
        file_menu.addAction("Exit")

        vehicle_menu = menu_bar.addMenu("Vehicle")
        act_new     = QAction("New Vehicle",      self)
        act_open    = QAction("Open Vehicle",     self)
        act_save    = QAction("Save Vehicle",     self)
        act_save_as = QAction("Save Vehicle As",  self)
        vehicle_menu.addAction(act_new)
        vehicle_menu.addAction(act_open)
        vehicle_menu.addSeparator()
        vehicle_menu.addAction(act_save)
        vehicle_menu.addAction(act_save_as)
        act_new.triggered.connect(self.vehicle_new_triggered)
        act_open.triggered.connect(self.vehicle_open_triggered)
        act_save.triggered.connect(self.vehicle_save_triggered)
        act_save_as.triggered.connect(self.vehicle_save_as_triggered)

        view_menu = menu_bar.addMenu("View")
        view_menu.addAction("Full Screen")

        help_menu = menu_bar.addMenu("Help")
        help_menu.addAction("About")

        layout.addWidget(menu_bar)
        layout.addStretch()


class StatusBarWidget(QWidget):
    """Thin bottom bar for status messages."""

    def __init__(self, theme: dict, height: int, parent=None):
        super().__init__(parent)
        self.setFixedHeight(height)

        t = theme
        self.setStyleSheet(f"background: {t['status_bg']};")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(0)

        self._label = QLabel("Ready")
        self._label.setStyleSheet(
            f"color: {t['status_text']}; font-size: 13px;"
            f"font-family: 'Helvetica Neue', Arial;"
            f"background: transparent;"
        )
        layout.addWidget(self._label)
        layout.addStretch()

    def set_message(self, text: str) -> None:
        self._label.setText(text)


# ── Shared panel widgets ───────────────────────────────────────────────────────


class SectionHeader(QWidget):
    """Full-width panel title bar: ALL-CAPS label on a dark strip."""

    def __init__(self, title: str, theme: dict, parent=None):
        super().__init__(parent)
        self.setFixedHeight(30)
        self.setStyleSheet(
            f"background: {theme['panel_header_bg']};"
            f"border-bottom: 1px solid {theme['border']};"
        )
        h = QHBoxLayout(self)
        h.setContentsMargins(12, 0, 12, 0)
        lbl = QLabel(title)
        lbl.setStyleSheet(
            f"color: {theme['panel_header_text']}; font-size: 12px;"
            f"letter-spacing: 1px; font-weight: 600; background: transparent;"
        )
        h.addWidget(lbl)
        h.addStretch()


class CollapsibleSection(QWidget):
    """
    Expandable panel section.  Pass ``content`` to embed a widget inside the
    body; omit it to get a placeholder "· · ·" label.

    Pass ``checkable=True`` to add a show/hide checkbox to the left of the
    header.  When the checkbox state changes, ``visibility_changed(bool)`` is
    emitted.

    For mutual-exclusion (accordion) behaviour, connect ``opened`` to the
    ``collapse()`` slots of sibling sections.
    """

    opened             = pyqtSignal()
    visibility_changed = pyqtSignal(bool)  # only fired when checkable=True

    def __init__(self, title: str, theme: dict,
                 content: "QWidget | None" = None,
                 checkable: bool = False,
                 checked: bool = True,
                 parent=None):
        super().__init__(parent)
        self.setStyleSheet("border: none;")

        t = theme
        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)

        self._title    = title
        self._subtitle = ""

        if checkable:
            # Header row = QWidget container (provides bg) + checkbox + button
            hdr_row = QWidget()
            hdr_row.setFixedHeight(36)
            hdr_row.setStyleSheet(
                f"background: {t['collapsible_bg']};"
                f"border-bottom: 1px solid {t['border']};"
            )
            hrl = QHBoxLayout(hdr_row)
            hrl.setContentsMargins(0, 0, 0, 0)
            hrl.setSpacing(0)

            self._vis_cb = QCheckBox()
            self._vis_cb.setChecked(checked)
            self._vis_cb.setFixedWidth(30)
            self._vis_cb.setStyleSheet(
                f"QCheckBox {{ background: transparent; padding-left: 6px; }}"
                f"QCheckBox::indicator {{"
                f"  width: 12px; height: 12px; border-radius: 2px; }}"
                f"QCheckBox::indicator:unchecked {{"
                f"  border: 1px solid {t['border']}; background: {t['input_bg']}; }}"
                f"QCheckBox::indicator:checked {{"
                f"  border: 1px solid {t['accent']}; background: {t['accent']}; }}"
            )
            self._vis_cb.stateChanged.connect(self._on_vis_changed)
            hrl.addWidget(self._vis_cb)

            # Button is transparent — the container row provides the background
            btn_ss = (
                f"QPushButton {{ background: transparent; color: {t['collapsible_text']};"
                f"border: none; text-align: left; padding: 0 12px; font-size: 13px; }}"
                f"QPushButton:hover {{ color: {t['label_bright']}; }}"
                f"QPushButton:checked {{ color: {t['label_bright']};"
                f"border-bottom: 1px solid {t['accent']}; }}"
            )
            self._hdr = QPushButton(self._header_text(False))
            self._hdr.setCheckable(True)
            self._hdr.setStyleSheet(btn_ss)
            self._hdr.setCursor(Qt.CursorShape.PointingHandCursor)
            self._hdr.clicked.connect(self._toggle)
            hrl.addWidget(self._hdr, 1)
            vbox.addWidget(hdr_row)
        else:
            hdr_ss = (
                f"QPushButton {{ background: {t['collapsible_bg']};"
                f"color: {t['collapsible_text']};"
                f"border: none; border-bottom: 1px solid {t['border']};"
                f"text-align: left; padding: 0 12px; font-size: 13px; }}"
                f"QPushButton:hover {{ background: {t['collapsible_hover']};"
                f"color: {t['label_bright']}; }}"
                f"QPushButton:checked {{ background: {t['collapsible_open_bg']};"
                f"color: {t['label_bright']}; border-bottom: 1px solid {t['accent']}; }}"
            )
            self._hdr = QPushButton(self._header_text(False))
            self._hdr.setCheckable(True)
            self._hdr.setFixedHeight(36)
            self._hdr.setStyleSheet(hdr_ss)
            self._hdr.setCursor(Qt.CursorShape.PointingHandCursor)
            self._hdr.clicked.connect(self._toggle)
            vbox.addWidget(self._hdr)

        self._body = QWidget()
        self._body.setVisible(False)
        self._body.setStyleSheet(
            f"background: {t['collapsible_open_bg']};"
            f"border-bottom: 1px solid {t['border']};"
        )
        body_lay = QVBoxLayout(self._body)
        if content is not None:
            body_lay.setContentsMargins(0, 0, 0, 0)
            body_lay.setSpacing(0)
            body_lay.addWidget(content)
        else:
            body_lay.setContentsMargins(14, 10, 14, 10)
            placeholder = QLabel("· · ·")
            placeholder.setStyleSheet(
                f"color: {t['border']}; font-size: 13px; background: transparent;"
            )
            body_lay.addWidget(placeholder)
        vbox.addWidget(self._body)

    def _on_vis_changed(self, state: int) -> None:
        self.visibility_changed.emit(state == Qt.CheckState.Checked.value)

    def _header_text(self, checked: bool) -> str:
        arrow = "▼" if checked else "▶"
        base  = f"{arrow}   {self._title}"
        return f"{base}   {self._subtitle}" if self._subtitle else base

    def set_subtitle(self, text: str) -> None:
        """Set a right-aligned info string displayed in the header (e.g. '450 kg')."""
        self._subtitle = text
        self._hdr.setText(self._header_text(self._hdr.isChecked()))

    def _toggle(self, checked: bool) -> None:
        self._hdr.setText(self._header_text(checked))
        self._body.setVisible(checked)
        if checked:
            self.opened.emit()

    def collapse(self) -> None:
        """Programmatically collapse without emitting opened."""
        self._hdr.setChecked(False)
        self._hdr.setText(self._header_text(False))
        self._body.setVisible(False)

    def expand(self) -> None:
        """Programmatically expand, emitting opened."""
        self._hdr.setChecked(True)
        self._hdr.setText(self._header_text(True))
        self._body.setVisible(True)
        self.opened.emit()

    def body(self) -> QWidget:
        """Return the body widget (useful when no content widget was passed)."""
        return self._body


def make_accordion(sections: "list[CollapsibleSection]") -> None:
    """Wire a list of CollapsibleSections for mutual exclusion: expanding one
    collapses all the others."""
    for i, sec in enumerate(sections):
        others = [s for j, s in enumerate(sections) if j != i]
        sec.opened.connect(lambda _o=others: [o.collapse() for o in _o])


# ── Vehicle control widgets (ported from v8.7) ────────────────────────────────


class ResetSlider(QSlider):
    """QSlider that resets to its construction-time default on double-click."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._reset_value: int = 0

    def mouseDoubleClickEvent(self, event):
        self.setValue(self._reset_value)
        super().mouseDoubleClickEvent(event)


class SnapSlider(ResetSlider):
    """
    QSlider with a stagnancy detent at 10 % of range.
    On mouse release it snaps to 0 (or ±10 if released near the detent).
    Double-click also resets to 0.
    """

    _SNAP_POINT = 10
    _SNAP_BAND  = 4

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.locked: bool = False

    def mouseReleaseEvent(self, event):
        if not self.locked:
            v  = self.value()
            av = abs(v)
            if abs(av - self._SNAP_POINT) <= self._SNAP_BAND:
                self.setValue(self._SNAP_POINT if v >= 0 else -self._SNAP_POINT)
            else:
                self.setValue(0)
        super().mouseReleaseEvent(event)

    def log_value(self) -> float:
        vmax = self.maximum()
        if vmax <= 0:
            return 0.0
        v    = self.value()
        sign = 1.0 if v >= 0 else -1.0
        norm = abs(v) / float(vmax)
        K    = 2.0
        return sign * (math.exp(norm * K) - 1.0) / (math.exp(K) - 1.0)


class SteeringWheelWidget(QWidget):
    """Draws a steering wheel rotated by a given angle (degrees, CW = right)."""

    _RIM_COLOR   = QColor(140, 140, 155)
    _HUB_COLOR   = QColor(55,  55,  65)
    _SPOKE_COLOR = QColor(120, 120, 135)

    def __init__(self, size: int = 120, parent=None):
        super().__init__(parent)
        self._angle_deg = 0.0
        self.setFixedSize(size, size)

    def set_angle(self, angle_deg: float) -> None:
        self._angle_deg = angle_deg
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h    = self.width(), self.height()
        r_outer = min(w, h) / 2.0 - 4.0
        r_hub   = r_outer * 0.18
        r_inner = r_outer * 0.55

        p.translate(w / 2.0, h / 2.0)
        p.rotate(self._angle_deg)

        p.setPen(QPen(self._RIM_COLOR, 5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(int(-r_outer), int(-r_outer), int(r_outer * 2), int(r_outer * 2))

        p.setPen(QPen(self._SPOKE_COLOR, 3))
        for base_deg in (-90.0, 30.0, 150.0):
            rad = math.radians(base_deg)
            cx, cy = math.cos(rad), math.sin(rad)
            p.drawLine(int(r_hub * cx), int(r_hub * cy),
                       int(r_inner * cx), int(r_inner * cy))

        p.setPen(QPen(self._SPOKE_COLOR, 2))
        p.setBrush(QBrush(self._HUB_COLOR))
        p.drawEllipse(int(-r_hub), int(-r_hub), int(r_hub * 2), int(r_hub * 2))
        p.end()
