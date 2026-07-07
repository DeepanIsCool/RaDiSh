from __future__ import annotations

import json
from pathlib import Path

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton,
    QInputDialog, QMessageBox,
)
from PyQt6.QtCore import Qt, pyqtSignal

from gui.widgets import SectionHeader

_VEHICLES_DIR = Path(__file__).parents[3] / "assets" / "vehicles"


class _LiveCombo(QComboBox):
    """Editable combo that re-scans assets/vehicles/ every time the popup opens."""

    def __init__(self, vehicles_dir: Path, theme: dict, parent=None):
        super().__init__(parent)
        self._dir = vehicles_dir
        t = theme
        self.setEditable(True)
        self.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

        # Style the combo box itself and the popup list.
        # Do NOT override ::drop-down or ::down-arrow — let Qt draw the system
        # arrow so it is always visible regardless of platform colour scheme.
        self.setStyleSheet(
            f"QComboBox {{"
            f"  background: {t['input_bg']};"
            f"  color: {t['input_text']};"
            f"  border: 1px solid {t['input_border']};"
            f"  border-radius: 3px;"
            f"  padding: 4px 8px;"
            f"  font-size: 13px;"
            f"}}"
            f"QComboBox QAbstractItemView {{"
            f"  background: {t['input_bg']};"
            f"  color: {t['input_text']};"
            f"  border: 1px solid {t['input_border']};"
            f"  selection-background-color: {t['btn_active_bg']};"
            f"  selection-color: {t['btn_active_text']};"
            f"  outline: none;"
            f"}}"
        )
        self.lineEdit().setPlaceholderText("type or select a vehicle…")
        self.lineEdit().setStyleSheet(
            f"background: transparent;"
            f"color: {t['input_text']};"
            f"border: none; padding: 0px; font-size: 13px;"
        )

    def showPopup(self) -> None:
        """Re-read directory so newly created files appear without a restart."""
        current = self.currentText()
        self.blockSignals(True)
        self.clear()
        names = sorted(p.stem for p in self._dir.glob("*.json"))
        self.addItems(names)
        self.setCurrentText(current)
        self.blockSignals(False)
        super().showPopup()


class VehicleConfigWidget(QWidget):
    """
    Upper-left panel.  Typable dropdown lists .json files from assets/vehicles/.
    Typing a name that exists loads it; typing a new name starts a blank template.
    """

    vehicle_changed = pyqtSignal(str, dict)   # (name, data) — {} when brand-new

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        _VEHICLES_DIR.mkdir(parents=True, exist_ok=True)

        self._current_name: str  = ""
        self._current_data: dict = {}

        t = theme
        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)

        vbox.addWidget(SectionHeader("VEHICLE CONFIGURATION", t))

        body = QWidget()
        body.setStyleSheet(f"background: {t['window_bg']};")
        b = QVBoxLayout(body)
        b.setContentsMargins(14, 14, 14, 12)
        b.setSpacing(8)

        label_ss = (
            f"color: {t['label_dim']}; font-size: 12px;"
            f"letter-spacing: 0.5px; background: transparent;"
        )
        btn_ss = (
            f"QPushButton {{"
            f"  background: {t['btn_bg']}; color: {t['btn_text']};"
            f"  border: 1px solid {t['input_border']}; border-radius: 3px;"
            f"  font-size: 13px; padding: 5px 0px;"
            f"}}"
            f"QPushButton:hover {{"
            f"  background: {t['btn_hover_bg']}; color: {t['btn_hover_text']};"
            f"}}"
            f"QPushButton:pressed {{"
            f"  background: {t['btn_active_bg']}; color: {t['btn_active_text']};"
            f"}}"
            f"QPushButton:disabled {{"
            f"  background: {t['input_bg']}; color: {t['input_border']};"
            f"  border-color: {t['input_border']};"
            f"}}"
        )

        # ── Load / Create label ───────────────────────────────────────────────
        b.addWidget(QLabel("Load / Create", styleSheet=label_ss))

        # ── Typable dropdown ──────────────────────────────────────────────────
        self._combo = _LiveCombo(_VEHICLES_DIR, t)
        self._combo.activated.connect(self._on_activated)
        self._combo.lineEdit().returnPressed.connect(self._on_return_pressed)
        b.addWidget(self._combo)

        # ── Buttons ───────────────────────────────────────────────────────────
        btn_row = QWidget()
        btn_row.setStyleSheet("background: transparent;")
        bh = QHBoxLayout(btn_row)
        bh.setContentsMargins(0, 4, 0, 0)
        bh.setSpacing(6)

        self._save_btn    = QPushButton("Save")
        self._save_as_btn = QPushButton("Save As")
        self._delete_btn  = QPushButton("Delete")
        for btn in (self._save_btn, self._save_as_btn, self._delete_btn):
            btn.setStyleSheet(btn_ss)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            bh.addWidget(btn)
        b.addWidget(btn_row)

        vbox.addWidget(body)

        self._save_btn.clicked.connect(self._on_save)
        self._save_as_btn.clicked.connect(self._on_save_as)
        self._delete_btn.clicked.connect(self._on_delete)

        self._sync_buttons()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _path(self, name: str) -> Path:
        return _VEHICLES_DIR / f"{name}.json"

    def _resolve(self, name: str) -> None:
        name = name.strip()
        if not name:
            return
        self._current_name = name
        p = self._path(name)
        self._current_data = json.loads(p.read_text()) if p.exists() else {}
        self._sync_buttons()
        self.vehicle_changed.emit(self._current_name, self._current_data)

    def _sync_buttons(self) -> None:
        has_name = bool(self._current_name)
        exists   = has_name and self._path(self._current_name).exists()
        self._save_btn.setEnabled(has_name)
        self._save_as_btn.setEnabled(has_name)
        self._delete_btn.setEnabled(exists)

    def _write(self, name: str, data: dict) -> None:
        self._path(name).write_text(json.dumps(data, indent=2))

    def _refresh_combo(self, keep: str = "") -> None:
        self._combo.blockSignals(True)
        self._combo.clear()
        self._combo.addItems(sorted(p.stem for p in _VEHICLES_DIR.glob("*.json")))
        self._combo.setCurrentText(keep) if keep else self._combo.clearEditText()
        self._combo.blockSignals(False)

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_activated(self, _index: int) -> None:
        self._resolve(self._combo.currentText())

    def _on_return_pressed(self) -> None:
        self._resolve(self._combo.currentText())

    def _on_save(self) -> None:
        if not self._current_name:
            return
        self._write(self._current_name, self._current_data)
        self._refresh_combo(keep=self._current_name)
        self._sync_buttons()

    def _on_save_as(self) -> None:
        new_name, ok = QInputDialog.getText(
            self, "Save As", "Vehicle name:", text=self._current_name,
        )
        new_name = new_name.strip()
        if not ok or not new_name:
            return
        self._current_name = new_name
        self._write(new_name, self._current_data)
        self._refresh_combo(keep=new_name)
        self._sync_buttons()

    def _on_delete(self) -> None:
        name = self._current_name
        if not name or not self._path(name).exists():
            return
        reply = QMessageBox.question(
            self, "Delete Vehicle", f"Permanently delete '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        self._path(name).unlink()
        self._current_name = ""
        self._current_data = {}
        self._refresh_combo()
        self._sync_buttons()
