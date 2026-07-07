from __future__ import annotations

import json
import math
from pathlib import Path

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QScrollArea, QLabel, QPushButton,
    QInputDialog, QMessageBox, QCheckBox, QComboBox, QDoubleSpinBox, QFrame,
    QSizePolicy,
)
from PyQt6.QtCore import Qt, pyqtSignal, QPointF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont

from gui.widgets import SectionHeader, CollapsibleSection, AssetCombo, make_accordion
from gui.tabs.vehicle_design.wheel_frame_section import WheelFrameBody, resolve_frame
from simulation.engine import EngineModel
from simulation.transmission import TransmissionModel
from simulation.brakes import BrakeModel
from simulation.mass import engine_mass, transmission_mass, brake_mass, body_mass


_ASSET_ROOT = Path(__file__).parents[3] / "assets"

# Generic load/save-only components (no dedicated config body yet).
_COMPONENTS: list[tuple[str, Path]] = []

_DEFAULT_GEAR_RATIOS: dict[int, list[float]] = {
    1: [3.54],
    2: [3.54, 1.48],
    3: [3.54, 1.48, 0.85],
    4: [3.54, 2.10, 1.48, 0.85],
    5: [3.54, 2.10, 1.48, 1.12, 0.85],
    6: [3.54, 2.10, 1.48, 1.12, 0.85, 0.65],
    7: [3.54, 2.10, 1.48, 1.12, 0.85, 0.65, 0.50],
    8: [3.54, 2.10, 1.48, 1.12, 0.85, 0.65, 0.50, 0.42],
}


class _TemplateBar(QWidget):
    """
    Reusable Load / Create + Save / Save As / Delete toolbar for component
    templates. Driven by two callbacks:
      get_data() -> dict          : snapshot the owning body's config
      set_data(dict) -> None      : push a loaded template into the body's UI
    Templates are JSON files in ``assets_dir``.
    """

    def __init__(self, assets_dir: Path, theme: dict,
                 get_data, set_data, parent=None):
        super().__init__(parent)
        assets_dir.mkdir(parents=True, exist_ok=True)
        self._dir          = assets_dir
        self._get_data     = get_data
        self._set_data     = set_data
        self._current_name = ""

        t = theme
        self.setStyleSheet("background: transparent;")
        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(8)

        label_ss = (f"color: {t['label_dim']}; font-size: 12px;"
                    f"letter-spacing: 0.5px; background: transparent;")
        btn_ss = (
            f"QPushButton {{ background: {t['btn_bg']}; color: {t['btn_text']};"
            f"  border: 1px solid {t['input_border']}; border-radius: 3px;"
            f"  font-size: 13px; padding: 5px 0px; }}"
            f"QPushButton:hover {{ background: {t['btn_hover_bg']}; color: {t['btn_hover_text']}; }}"
            f"QPushButton:pressed {{ background: {t['btn_active_bg']}; color: {t['btn_active_text']}; }}"
            f"QPushButton:disabled {{ background: {t['input_bg']}; color: {t['input_border']};"
            f"  border-color: {t['input_border']}; }}"
        )

        vbox.addWidget(QLabel("Load / Create", styleSheet=label_ss))

        self._combo = AssetCombo(assets_dir, t)
        self._combo.activated.connect(self._on_activated)
        self._combo.lineEdit().returnPressed.connect(self._on_return_pressed)
        vbox.addWidget(self._combo)

        btn_row = QWidget(); btn_row.setStyleSheet("background: transparent;")
        bh = QHBoxLayout(btn_row); bh.setContentsMargins(0, 4, 0, 0); bh.setSpacing(6)
        self._save_btn    = QPushButton("Save")
        self._save_as_btn = QPushButton("Save As")
        self._delete_btn  = QPushButton("Delete")
        for btn in (self._save_btn, self._save_as_btn, self._delete_btn):
            btn.setStyleSheet(btn_ss)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            bh.addWidget(btn)
        vbox.addWidget(btn_row)

        self._save_btn.clicked.connect(self._on_save)
        self._save_as_btn.clicked.connect(self._on_save_as)
        self._delete_btn.clicked.connect(self._on_delete)
        self._sync_buttons()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _path(self, name: str) -> Path:
        return self._dir / f"{name}.json"

    def _sync_buttons(self) -> None:
        has  = bool(self._current_name)
        ex   = has and self._path(self._current_name).exists()
        self._save_btn.setEnabled(has)
        self._save_as_btn.setEnabled(has)
        self._delete_btn.setEnabled(ex)

    def _resolve(self, name: str) -> None:
        name = name.strip()
        if not name:
            return
        self._current_name = name
        p = self._path(name)
        if p.exists():
            self._set_data(json.loads(p.read_text()))
        self._sync_buttons()

    def _write(self, name: str) -> None:
        self._path(name).write_text(json.dumps(self._get_data(), indent=2))

    def _refresh_combo(self, keep: str = "") -> None:
        self._combo.blockSignals(True)
        self._combo.clear()
        self._combo.addItems(sorted(p.stem for p in self._dir.glob("*.json")))
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
        self._write(self._current_name)
        self._refresh_combo(keep=self._current_name); self._sync_buttons()

    def _on_save_as(self) -> None:
        name, ok = QInputDialog.getText(
            self, "Save As", "Template name:", text=self._current_name)
        name = name.strip()
        if not ok or not name:
            return
        self._current_name = name; self._write(name)
        self._refresh_combo(keep=name); self._sync_buttons()

    def _on_delete(self) -> None:
        name = self._current_name
        if not name or not self._path(name).exists():
            return
        if QMessageBox.question(
            self, "Delete Template", f"Permanently delete '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        ) != QMessageBox.StandardButton.Yes:
            return
        self._path(name).unlink()
        self._current_name = ""
        self._refresh_combo(); self._sync_buttons()


def _group_section(title: str, theme: dict, checkable: bool = False,
                   checked: bool = True):
    """Build a collapsible sub-section; returns (section, content_layout)."""
    w = QWidget(); w.setStyleSheet("background: transparent;")
    lay = QVBoxLayout(w)
    lay.setContentsMargins(12, 10, 12, 10); lay.setSpacing(6)
    sec = CollapsibleSection(title, theme, w, checkable=checkable, checked=checked)
    return sec, lay


class _GearRatioWidget(QWidget):
    """
    Interactive bar-chart-style editor for forward gear ratios.
    X axis : gear number 1 … N (discrete, evenly spaced).
    Y axis : ratio value (_RATIO_MIN … _RATIO_MAX).
    Drag a point vertically to change its ratio.
    Emits ratios_changed(list[float]) on release.
    """

    ratios_changed = pyqtSignal(list)

    _RATIO_MAX = 5.5
    _RATIO_MIN = 0.3
    _MARGIN_L  = 32
    _MARGIN_B  = 20
    _MARGIN_T  = 6
    _MARGIN_R  = 8
    _PT_R      = 5

    def __init__(self, theme: dict, parent=None) -> None:
        super().__init__(parent)
        self._t = theme
        self._ratios: list[float] = list(_DEFAULT_GEAR_RATIOS[5])
        self._drag_idx: int = -1
        self.setMinimumHeight(110)
        self.setMaximumHeight(140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.CrossCursor)

    # ── Public API ────────────────────────────────────────────────────────────

    def set_n_gears(self, n: int) -> None:
        n = max(1, min(8, n))
        cur = len(self._ratios)
        if n == cur:
            return
        if n > cur:
            while len(self._ratios) < n:
                self._ratios.append(max(self._RATIO_MIN, self._ratios[-1] * 0.78))
        else:
            self._ratios = self._ratios[:n]
        self.update()

    def get_ratios(self) -> list[float]:
        return list(self._ratios)

    def set_ratios(self, ratios: list[float]) -> None:
        self._ratios = [float(r) for r in ratios]
        self.update()

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _plot_rect(self):
        w, h = self.width(), self.height()
        return (self._MARGIN_L, self._MARGIN_T,
                w - self._MARGIN_L - self._MARGIN_R,
                h - self._MARGIN_T - self._MARGIN_B)

    def _pt_to_px(self, idx: int, ratio: float) -> QPointF:
        x0, y0, pw, ph = self._plot_rect()
        n  = max(1, len(self._ratios))
        px = x0 + pw * (idx / (n - 1)) if n > 1 else x0 + pw / 2
        t  = (ratio - self._RATIO_MIN) / (self._RATIO_MAX - self._RATIO_MIN)
        py = y0 + ph * (1.0 - t)
        return QPointF(px, py)

    def _px_to_ratio(self, py: float) -> float:
        _, y0, _, ph = self._plot_rect()
        t = 1.0 - (py - y0) / ph
        return max(self._RATIO_MIN, min(self._RATIO_MAX,
               self._RATIO_MIN + t * (self._RATIO_MAX - self._RATIO_MIN)))

    def _nearest_idx(self, mx: float, my: float) -> int:
        best_d, best_i = 1e9, -1
        for i, r in enumerate(self._ratios):
            p = self._pt_to_px(i, r)
            d = math.hypot(mx - p.x(), my - p.y())
            if d < best_d:
                best_d, best_i = d, i
        return best_i if best_d < 22 else -1

    # ── Paint ─────────────────────────────────────────────────────────────────

    def paintEvent(self, _) -> None:
        t = self._t
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        x0, y0, pw, ph = self._plot_rect()

        p.fillRect(self.rect(), QColor(t["input_bg"]))

        grid_pen   = QPen(QColor(t["border"])); grid_pen.setWidth(1)
        label_col  = QColor(t["label_dim"])
        font       = QFont(); font.setPixelSize(9); p.setFont(font)

        # Y grid + labels
        for rv in [1.0, 2.0, 3.0, 4.0, 5.0]:
            if rv > self._RATIO_MAX or rv < self._RATIO_MIN:
                continue
            py_v = self._pt_to_px(0, rv).y()
            p.setPen(grid_pen)
            p.drawLine(x0, int(py_v), x0 + pw, int(py_v))
            p.setPen(label_col)
            p.drawText(0, int(py_v) - 5, self._MARGIN_L - 4, 14,
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       f"{rv:.0f}")

        # X grid + gear labels
        for i in range(len(self._ratios)):
            px_v = int(self._pt_to_px(i, self._RATIO_MIN).x())
            p.setPen(grid_pen)
            p.drawLine(px_v, y0, px_v, y0 + ph)
            p.setPen(label_col)
            p.drawText(px_v - 10, y0 + ph + 2, 20, self._MARGIN_B - 2,
                       Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop,
                       str(i + 1))

        # Connecting line
        if len(self._ratios) >= 2:
            line_pen = QPen(QColor(t["accent"])); line_pen.setWidth(2)
            p.setPen(line_pen)
            pts = [self._pt_to_px(i, r) for i, r in enumerate(self._ratios)]
            for i in range(len(pts) - 1):
                p.drawLine(pts[i], pts[i + 1])

        # Points + ratio labels
        for i, r in enumerate(self._ratios):
            px = self._pt_to_px(i, r)
            if i == self._drag_idx:
                p.setBrush(QBrush(QColor(t["accent"])))
                p.setPen(QPen(QColor(t["label_bright"]), 2))
                p.drawEllipse(px, self._PT_R + 2, self._PT_R + 2)
            else:
                p.setBrush(QBrush(QColor(t["slider_handle"])))
                p.setPen(QPen(QColor(t["label_bright"]), 1))
                p.drawEllipse(px, self._PT_R, self._PT_R)
            p.setPen(QColor(t["label_bright"]))
            p.drawText(int(px.x()) - 14, int(px.y()) - 14, 28, 12,
                       Qt.AlignmentFlag.AlignHCenter, f"{r:.2f}")

        p.end()

    # ── Mouse ─────────────────────────────────────────────────────────────────

    def mousePressEvent(self, e) -> None:
        self._drag_idx = self._nearest_idx(e.position().x(), e.position().y())
        self.update()

    def mouseMoveEvent(self, e) -> None:
        if self._drag_idx < 0:
            return
        self._ratios[self._drag_idx] = self._px_to_ratio(e.position().y())
        self.update()

    def mouseReleaseEvent(self, _) -> None:
        self._drag_idx = -1
        self.update()
        self.ratios_changed.emit(self.get_ratios())


class _DriveModesEditor(QWidget):
    """
    Add / remove / edit automatic-transmission drive modes (ECO, CITY, SPORT…).

    Each mode is a shift schedule. Internally stored in model form
    (up_base/up_span/dn_base/dn_span/shift_time, fractions of the idle→redline
    span); the spinners present them as light/full-throttle shift RPM percentages.
    """

    changed = pyqtSignal()

    def __init__(self, theme: dict, parent=None) -> None:
        super().__init__(parent)
        t = theme
        self.setStyleSheet("background: transparent;")

        from simulation.automatic import DEFAULT_DRIVE_MODES, DEFAULT_DRIVE_MODE
        self._modes: dict[str, dict] = {
            n: dict(p) for n, p in DEFAULT_DRIVE_MODES.items()
        }
        self._default: str = DEFAULT_DRIVE_MODE
        self._current: str = DEFAULT_DRIVE_MODE
        self._loading = False

        lbl_ss = (f"color: {t['label_dim']}; font-size: 11px;"
                  f" letter-spacing: 0.4px; background: transparent;")
        spin_ss = (
            f"QDoubleSpinBox {{ background: {t['input_bg']}; color: {t['input_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 2px 4px; }}"
            f"QDoubleSpinBox::up-button, QDoubleSpinBox::down-button"
            f" {{ width: 14px; background: {t['btn_bg']}; }}"
            f"QDoubleSpinBox:focus {{ border-color: {t['accent']}; }}"
        )
        combo_ss = (
            f"QComboBox {{ background: {t['input_bg']}; color: {t['input_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 2px 6px; }}"
            f"QComboBox::drop-down {{ border: none; width: 18px; }}"
            f"QComboBox QAbstractItemView {{ background: {t['input_bg']};"
            f" color: {t['input_text']}; selection-background-color: {t['accent']}; }}"
        )
        btn_ss = (
            f"QPushButton {{ background: {t['btn_bg']}; color: {t['btn_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 3px 8px; }}"
            f"QPushButton:hover {{ background: {t['btn_hover_bg']}; color: {t['btn_hover_text']}; }}"
        )
        chk_ss = (
            f"QCheckBox {{ color: {t['label_bright']}; font-size: 12px;"
            f" background: transparent; spacing: 6px; }}"
            f"QCheckBox::indicator {{ width: 13px; height: 13px;"
            f" border: 1px solid {t['input_border']}; border-radius: 2px;"
            f" background: {t['input_bg']}; }}"
            f"QCheckBox::indicator:checked {{ background: {t['accent']};"
            f" border-color: {t['accent']}; }}"
        )

        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0); v.setSpacing(6)
        v.addWidget(QLabel("Drive Modes", styleSheet=lbl_ss))

        # selector + add/remove
        sel_row = QWidget(); sel_row.setStyleSheet("background: transparent;")
        sh = QHBoxLayout(sel_row); sh.setContentsMargins(0, 0, 0, 0); sh.setSpacing(4)
        self._combo = QComboBox(); self._combo.setStyleSheet(combo_ss)
        self._combo.addItems(list(self._modes.keys()))
        add_btn = QPushButton("Add");  add_btn.setStyleSheet(btn_ss)
        del_btn = QPushButton("Del");  del_btn.setStyleSheet(btn_ss)
        sh.addWidget(self._combo, 1); sh.addWidget(add_btn); sh.addWidget(del_btn)
        v.addWidget(sel_row)

        def _spin(lo, hi, step, val, dec=0, suffix=""):
            s = QDoubleSpinBox(); s.setRange(lo, hi); s.setSingleStep(step)
            s.setValue(val); s.setDecimals(dec); s.setStyleSheet(spin_ss)
            s.setFixedWidth(80)
            if suffix:
                s.setSuffix(suffix)
            return s

        def _row(label_text, widget):
            rw = QWidget(); rw.setStyleSheet("background: transparent;")
            rh = QHBoxLayout(rw); rh.setContentsMargins(0, 0, 0, 0); rh.setSpacing(6)
            rh.addWidget(QLabel(label_text, styleSheet=lbl_ss))
            rh.addStretch(); rh.addWidget(widget)
            v.addWidget(rw)

        self._up_light = _spin(0, 100, 5, 30, 0, " %")
        self._up_full  = _spin(0, 100, 5, 85, 0, " %")
        self._dn_light = _spin(0, 100, 5, 10, 0, " %")
        self._dn_full  = _spin(0, 100, 5, 35, 0, " %")
        self._shift_t  = _spin(0.05, 1.5, 0.05, 0.40, 2, " s")
        _row("Upshift @ light",   self._up_light)
        _row("Upshift @ full",    self._up_full)
        _row("Downshift @ light", self._dn_light)
        _row("Downshift @ full",  self._dn_full)
        _row("Shift time",        self._shift_t)

        self._default_chk = QCheckBox("Boot default"); self._default_chk.setStyleSheet(chk_ss)
        v.addWidget(self._default_chk)

        # wire
        self._combo.currentTextChanged.connect(self._on_select)
        add_btn.clicked.connect(self._on_add)
        del_btn.clicked.connect(self._on_remove)
        for s in (self._up_light, self._up_full, self._dn_light,
                  self._dn_full, self._shift_t):
            s.valueChanged.connect(self._on_spin)
        self._default_chk.toggled.connect(self._on_default_toggled)

        self._load(self._current)

    # ── Conversion between spinner % and model fractions ────────────────────────

    def _load(self, name: str) -> None:
        if name not in self._modes:
            return
        self._loading = True
        self._current = name
        p = self._modes[name]
        self._up_light.setValue(round(p["up_base"] * 100))
        self._up_full.setValue(round((p["up_base"] + p["up_span"]) * 100))
        self._dn_light.setValue(round(p["dn_base"] * 100))
        self._dn_full.setValue(round((p["dn_base"] + p["dn_span"]) * 100))
        self._shift_t.setValue(p["shift_time"])
        self._default_chk.setChecked(name == self._default)
        self._loading = False

    def _store_current(self) -> None:
        ul = self._up_light.value() / 100.0
        uf = self._up_full.value() / 100.0
        dl = self._dn_light.value() / 100.0
        df = self._dn_full.value() / 100.0
        self._modes[self._current] = {
            "up_base": ul, "up_span": max(0.0, uf - ul),
            "dn_base": dl, "dn_span": max(0.0, df - dl),
            "shift_time": self._shift_t.value(),
        }

    # ── Slots ───────────────────────────────────────────────────────────────────

    def _on_select(self, name: str) -> None:
        if self._loading or not name:
            return
        self._load(name)

    def _on_spin(self, _) -> None:
        if self._loading:
            return
        self._store_current()
        self.changed.emit()

    def _on_default_toggled(self, checked: bool) -> None:
        if self._loading:
            return
        if checked:
            self._default = self._current
        elif self._default == self._current:
            # can't have no default — keep this one
            self._loading = True
            self._default_chk.setChecked(True)
            self._loading = False
            return
        self.changed.emit()

    def _on_add(self) -> None:
        name, ok = QInputDialog.getText(self, "Add Drive Mode", "Mode name:")
        name = name.strip().upper()
        if not ok or not name or name in self._modes:
            return
        self._store_current()
        self._modes[name] = dict(self._modes.get("CITY",
            {"up_base": 0.30, "up_span": 0.55, "dn_base": 0.10,
             "dn_span": 0.25, "shift_time": 0.40}))
        self._loading = True
        self._combo.addItem(name)
        self._combo.setCurrentText(name)
        self._loading = False
        self._load(name)
        self.changed.emit()

    def _on_remove(self) -> None:
        if len(self._modes) <= 1:
            return
        name = self._current
        del self._modes[name]
        if self._default == name:
            self._default = next(iter(self._modes))
        self._loading = True
        idx = self._combo.findText(name)
        if idx >= 0:
            self._combo.removeItem(idx)
        self._loading = False
        self._load(self._combo.currentText())
        self.changed.emit()

    # ── Data ─────────────────────────────────────────────────────────────────────

    def get_data(self) -> dict:
        self._store_current()
        return {
            "drive_modes": [{"name": n, **p} for n, p in self._modes.items()],
            "default_drive_mode": self._default,
        }

    def load_data(self, data: dict) -> None:
        keys = ("up_base", "up_span", "dn_base", "dn_span", "shift_time")
        built: dict[str, dict] = {}
        for m in data.get("drive_modes", []):
            name = str(m.get("name", "")).strip()
            if name:
                built[name] = {k: float(m.get(k, 0.0)) for k in keys}
        if not built:
            return
        self._modes = built
        self._default = data.get("default_drive_mode", next(iter(built)))
        self._current = self._default if self._default in built else next(iter(built))
        self._loading = True
        self._combo.clear()
        self._combo.addItems(list(built))
        self._combo.setCurrentText(self._current)
        self._loading = False
        self._load(self._current)
        self.changed.emit()


class TransmissionConfigBody(QWidget):
    """Transmission configuration: type, gears, ratios, drivetrain, drive modes."""

    transmission_changed = pyqtSignal(dict)

    def __init__(self, theme: dict, parent=None) -> None:
        super().__init__(parent)
        t = theme

        lbl_ss = (f"color: {t['label_dim']}; font-size: 11px;"
                  f" letter-spacing: 0.4px; background: transparent;")
        spin_ss = (
            f"QDoubleSpinBox {{ background: {t['input_bg']}; color: {t['input_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 2px 4px; }}"
            f"QDoubleSpinBox::up-button, QDoubleSpinBox::down-button"
            f" {{ width: 14px; background: {t['btn_bg']}; }}"
            f"QDoubleSpinBox:focus {{ border-color: {t['accent']}; }}"
        )
        combo_ss = (
            f"QComboBox {{ background: {t['input_bg']}; color: {t['input_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 2px 6px; }}"
            f"QComboBox::drop-down {{ border: none; width: 18px; }}"
            f"QComboBox QAbstractItemView {{ background: {t['input_bg']};"
            f" color: {t['input_text']}; selection-background-color: {t['accent']}; }}"
        )

        info_ss = (f"color: {t['label_bright']}; font-size: 11px;"
                   f" font-weight: 600; background: transparent;")
        self._loading = False

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(14, 14, 14, 12)
        vbox.setSpacing(8)

        def _spin(lo, hi, step, val, dec=2):
            s = QDoubleSpinBox()
            s.setRange(lo, hi); s.setSingleStep(step)
            s.setValue(val); s.setDecimals(dec)
            s.setStyleSheet(spin_ss); s.setFixedWidth(80)
            return s

        def _row(lay, label_text, widget):
            rw = QWidget(); rw.setStyleSheet("background: transparent;")
            rh = QHBoxLayout(rw)
            rh.setContentsMargins(0, 0, 0, 0); rh.setSpacing(6)
            rh.addWidget(QLabel(label_text, styleSheet=lbl_ss))
            rh.addStretch(); rh.addWidget(widget)
            lay.addWidget(rw)

        vbox.addWidget(_TemplateBar(_ASSET_ROOT / "transmissions", t,
                                    self._build_data, self.load_data))
        self._mass_lbl = QLabel("Mass: — kg", styleSheet=info_ss)
        vbox.addWidget(self._mass_lbl)

        # ── Type & Gears ──────────────────────────────────────────────────────
        sec_g, lg = _group_section("Type & Gears", t)
        self._type_combo = QComboBox()
        self._type_combo.addItems(["Manual", "Automatic"])
        self._type_combo.setStyleSheet(combo_ss)
        _row(lg, "Type", self._type_combo)
        lg.addWidget(QLabel("Forward Gears", styleSheet=lbl_ss))
        self._n_gears = _spin(1, 8, 1, 5, 0)
        _row(lg, "Number of Gears", self._n_gears)
        self._ratio_graph = _GearRatioWidget(t)
        lg.addWidget(self._ratio_graph)

        # ── Drivetrain ────────────────────────────────────────────────────────
        sec_d, ld = _group_section("Drivetrain", t)
        self._rev_ratio   = _spin(0.5, 6.0, 0.05, 3.32)
        self._final_drive = _spin(1.0, 8.0, 0.05, 3.90)
        self._eta         = _spin(0.5, 1.0, 0.01, 0.95)
        self._I_engine    = _spin(0.02, 2.0, 0.01, 0.15)
        self._clutch_max  = _spin(50.0, 1000.0, 10.0, 350.0, 0)
        _row(ld, "Reverse Ratio",     self._rev_ratio)
        _row(ld, "Final Drive",       self._final_drive)
        _row(ld, "Efficiency (η)",    self._eta)
        _row(ld, "Inertia I (kg·m²)", self._I_engine)
        _row(ld, "Clutch Max (Nm)",   self._clutch_max)

        # ── Drive modes (automatic only) ──────────────────────────────────────
        self._modes_editor = _DriveModesEditor(t)
        self._modes_section = CollapsibleSection("Drive Modes", t, self._modes_editor)
        self._modes_editor.changed.connect(self._emit)

        _secs = [sec_g, sec_d, self._modes_section]
        for s in _secs:
            vbox.addWidget(s)
        make_accordion(_secs)
        sec_g.expand()

        # ── Wire ──────────────────────────────────────────────────────────────
        self._n_gears.valueChanged.connect(self._on_n_gears_changed)
        self._ratio_graph.ratios_changed.connect(lambda _: self._emit())
        for w in (self._rev_ratio, self._final_drive, self._eta,
                  self._I_engine, self._clutch_max):
            w.valueChanged.connect(lambda _: self._emit())
        self._type_combo.currentIndexChanged.connect(self._on_type_changed)

        self._apply_type_visibility()

    def _is_automatic(self) -> bool:
        return self._type_combo.currentText().split()[0].lower() == "automatic"

    def _apply_type_visibility(self) -> None:
        self._modes_section.setVisible(self._is_automatic())

    def _on_type_changed(self, _) -> None:
        self._apply_type_visibility()
        self._emit()

    def _on_n_gears_changed(self, _) -> None:
        self._ratio_graph.set_n_gears(int(self._n_gears.value()))
        self._emit()

    def set_mass(self, kg: float) -> None:
        self._mass_lbl.setText(f"Mass: {kg:,.0f} kg")

    def _build_data(self) -> dict:
        data = {
            "trans_type":       self._type_combo.currentText().split()[0].lower(),
            "n_forward_gears":  int(self._n_gears.value()),
            "forward_ratios":   self._ratio_graph.get_ratios(),
            "reverse_ratio":    self._rev_ratio.value(),
            "final_drive":      self._final_drive.value(),
            "eta":              self._eta.value(),
            "I_engine":         self._I_engine.value(),
            "clutch_torque_max": self._clutch_max.value(),
        }
        data.update(self._modes_editor.get_data())
        return data

    def load_data(self, data: dict) -> None:
        self._loading = True
        ttype = data.get("trans_type", "manual")
        self._type_combo.setCurrentText("Automatic" if ttype == "automatic" else "Manual")
        fwd = data.get("forward_ratios", [3.54, 2.10, 1.48, 1.12, 0.85])
        self._n_gears.setValue(len(fwd))
        self._ratio_graph.set_n_gears(len(fwd))
        self._ratio_graph.set_ratios(fwd)
        self._rev_ratio.setValue(float(data.get("reverse_ratio", 3.32)))
        self._final_drive.setValue(float(data.get("final_drive", 3.90)))
        self._eta.setValue(float(data.get("eta", 0.95)))
        self._I_engine.setValue(float(data.get("I_engine", 0.15)))
        self._clutch_max.setValue(float(data.get("clutch_torque_max", 350.0)))
        if data.get("drive_modes"):
            self._modes_editor.load_data(data)
        self._apply_type_visibility()
        self._loading = False
        self._emit()

    def _emit(self) -> None:
        if self._loading:
            return
        self.transmission_changed.emit(self._build_data())


class BrakeConfigBody(QWidget):
    """Brake configuration: max torque, front/rear bias, grip ceiling."""

    brakes_changed = pyqtSignal(dict)

    def __init__(self, theme: dict, parent=None) -> None:
        super().__init__(parent)
        t = theme
        self._loading = False

        lbl_ss = (f"color: {t['label_dim']}; font-size: 11px;"
                  f" letter-spacing: 0.4px; background: transparent;")
        spin_ss = (
            f"QDoubleSpinBox {{ background: {t['input_bg']}; color: {t['input_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 2px 4px; }}"
            f"QDoubleSpinBox::up-button, QDoubleSpinBox::down-button"
            f" {{ width: 14px; background: {t['btn_bg']}; }}"
            f"QDoubleSpinBox:focus {{ border-color: {t['accent']}; }}"
        )
        info_ss = (f"color: {t['label_bright']}; font-size: 11px;"
                   f" font-weight: 600; background: transparent;")

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(14, 14, 14, 12)
        vbox.setSpacing(8)

        vbox.addWidget(_TemplateBar(_ASSET_ROOT / "brakes", t,
                                    self._build_data, self.load_data))
        self._mass_lbl = QLabel("Mass: — kg", styleSheet=info_ss)
        vbox.addWidget(self._mass_lbl)

        def _spin(lo, hi, step, val, dec=2):
            s = QDoubleSpinBox()
            s.setRange(lo, hi); s.setSingleStep(step)
            s.setValue(val); s.setDecimals(dec)
            s.setStyleSheet(spin_ss); s.setFixedWidth(80)
            return s

        sec, bl = _group_section("Friction Brake", t)

        def _row(label_text, widget):
            rw = QWidget(); rw.setStyleSheet("background: transparent;")
            rh = QHBoxLayout(rw)
            rh.setContentsMargins(0, 0, 0, 0); rh.setSpacing(6)
            rh.addWidget(QLabel(label_text, styleSheet=lbl_ss))
            rh.addStretch(); rh.addWidget(widget)
            bl.addWidget(rw)

        self._max_torque = _spin(100.0, 8000.0, 50.0, 3500.0, 0)
        self._front_bias = _spin(0.0, 1.0, 0.05, 0.65)
        self._brake_mu   = _spin(0.3, 1.5, 0.05, 1.0)

        _row("Max Torque (Nm)", self._max_torque)
        _row("Front Bias",      self._front_bias)
        _row("Grip (μ)",        self._brake_mu)

        vbox.addWidget(sec)
        make_accordion([sec])
        sec.expand()

        for w in (self._max_torque, self._front_bias, self._brake_mu):
            w.valueChanged.connect(lambda _: self._emit())

    def set_mass(self, kg: float) -> None:
        self._mass_lbl.setText(f"Mass: {kg:,.0f} kg")

    def load_data(self, data: dict) -> None:
        self._loading = True
        self._max_torque.setValue(float(data.get("max_brake_torque", 3500.0)))
        self._front_bias.setValue(float(data.get("front_bias", 0.65)))
        self._brake_mu.setValue(float(data.get("brake_mu", 1.0)))
        self._loading = False
        self._emit()

    def _build_data(self) -> dict:
        return {
            "max_brake_torque": self._max_torque.value(),
            "front_bias":       self._front_bias.value(),
            "brake_mu":         self._brake_mu.value(),
        }

    def _emit(self) -> None:
        if self._loading:
            return
        self.brakes_changed.emit(self._build_data())


class _VeMapWidget(QWidget):
    """
    Interactive 2-D VE-map editor.
    X axis : RPM (0 … max_rpm), one draggable point every 1000 RPM.
    Y axis : Volumetric Efficiency 0 – 100 %.
    Emits ve_changed(list) with [[rpm, ve_fraction], …] whenever a point moves.
    """

    ve_changed = pyqtSignal(list)

    _MARGIN_L = 32    # px — left margin for VE labels
    _MARGIN_B = 20    # px — bottom margin for RPM labels
    _MARGIN_T = 6     # px — top margin
    _MARGIN_R = 8     # px — right margin
    _PT_R     = 5     # point circle radius

    def __init__(self, theme: dict, parent=None) -> None:
        super().__init__(parent)
        self._t = theme
        self._max_rpm = 6000
        self._points: list[list[float]] = []   # [[rpm, ve_fraction], …]
        self._drag_idx: int = -1
        self.setMinimumHeight(110)
        self.setMaximumHeight(140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.CrossCursor)
        self._reset_to_default()

    # ── Public API ────────────────────────────────────────────────────────────

    def set_max_rpm(self, max_rpm: int) -> None:
        """Rebuild the point list when redline changes."""
        max_rpm = max(1000, int(round(max_rpm / 1000) * 1000))
        if max_rpm == self._max_rpm:
            return
        old = {int(r): v for r, v in self._points}
        new_rpms = list(range(0, max_rpm + 1, 1000))
        new_pts: list[list[float]] = []
        for rpm in new_rpms:
            if rpm in old:
                new_pts.append([float(rpm), old[rpm]])
            else:
                # interpolate from old points
                new_pts.append([float(rpm), self._interp(rpm)])
        self._max_rpm = max_rpm
        self._points  = new_pts
        self.update()

    def get_points(self) -> list[list[float]]:
        return [list(p) for p in self._points]

    def set_points(self, pts: list[list[float]]) -> None:
        self._points = [list(p) for p in pts]
        self._max_rpm = int(self._points[-1][0]) if self._points else self._max_rpm
        self.update()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _reset_to_default(self) -> None:
        defaults = {0: 0.60, 1000: 0.65, 2000: 0.72, 3000: 0.80,
                    4000: 0.85, 5000: 0.80, 6000: 0.68}
        self._points = [[float(r), defaults.get(r, 0.70)]
                        for r in range(0, self._max_rpm + 1, 1000)]

    def _interp(self, rpm: float) -> float:
        pts = self._points
        if not pts:
            return 0.70
        if rpm <= pts[0][0]:
            return pts[0][1]
        if rpm >= pts[-1][0]:
            return pts[-1][1]
        for i in range(len(pts) - 1):
            r0, v0 = pts[i]; r1, v1 = pts[i + 1]
            if r0 <= rpm <= r1:
                return v0 + (v1 - v0) * (rpm - r0) / (r1 - r0)
        return pts[-1][1]

    def _plot_rect(self):
        w, h = self.width(), self.height()
        return (self._MARGIN_L, self._MARGIN_T,
                w - self._MARGIN_L - self._MARGIN_R,
                h - self._MARGIN_T - self._MARGIN_B)

    def _pt_to_px(self, rpm: float, ve: float) -> QPointF:
        x0, y0, pw, ph = self._plot_rect()
        px = x0 + pw * (rpm / self._max_rpm) if self._max_rpm else x0
        py = y0 + ph * (1.0 - ve)
        return QPointF(px, py)

    def _px_to_ve(self, py: float) -> float:
        _, y0, _, ph = self._plot_rect()
        return max(0.0, min(1.0, 1.0 - (py - y0) / ph))

    def _nearest_idx(self, mx: float, my: float) -> int:
        best_d, best_i = 1e9, -1
        for i, (rpm, ve) in enumerate(self._points):
            p = self._pt_to_px(rpm, ve)
            d = math.hypot(mx - p.x(), my - p.y())
            if d < best_d:
                best_d, best_i = d, i
        return best_i if best_d < 20 else -1

    # ── Paint ─────────────────────────────────────────────────────────────────

    def paintEvent(self, _) -> None:
        t  = self._t
        p  = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        x0, y0, pw, ph = self._plot_rect()

        # background
        p.fillRect(self.rect(), QColor(t["input_bg"]))

        # grid lines + VE labels
        grid_pen = QPen(QColor(t["border"])); grid_pen.setWidth(1)
        p.setPen(grid_pen)
        font = QFont(); font.setPixelSize(9)
        p.setFont(font)
        label_col = QColor(t["label_dim"])
        for ve_pct in range(0, 101, 20):
            ve = ve_pct / 100.0
            py = y0 + ph * (1.0 - ve)
            p.drawLine(x0, int(py), x0 + pw, int(py))
            p.setPen(label_col)
            p.drawText(0, int(py) - 5, self._MARGIN_L - 4, 14,
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       f"{ve_pct}%")
            p.setPen(grid_pen)

        # RPM grid + labels
        step = 1000
        for rpm in range(0, self._max_rpm + 1, step):
            px = x0 + pw * (rpm / self._max_rpm) if self._max_rpm else x0
            p.drawLine(int(px), y0, int(px), y0 + ph)
            if rpm % 2000 == 0:
                p.setPen(label_col)
                label = f"{rpm // 1000}k" if rpm > 0 else "0"
                p.drawText(int(px) - 12, y0 + ph + 2, 24, self._MARGIN_B - 2,
                           Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop,
                           label)
                p.setPen(grid_pen)

        # curve
        if len(self._points) >= 2:
            curve_pen = QPen(QColor(t["accent"])); curve_pen.setWidth(2)
            p.setPen(curve_pen)
            pts_px = [self._pt_to_px(r, v) for r, v in self._points]
            for i in range(len(pts_px) - 1):
                p.drawLine(pts_px[i], pts_px[i + 1])

        # points
        for i, (rpm, ve) in enumerate(self._points):
            px = self._pt_to_px(rpm, ve)
            if i == self._drag_idx:
                p.setBrush(QBrush(QColor(t["accent"])))
                p.setPen(QPen(QColor(t["label_bright"]), 2))
                p.drawEllipse(px, self._PT_R + 2, self._PT_R + 2)
            else:
                p.setBrush(QBrush(QColor(t["slider_handle"])))
                p.setPen(QPen(QColor(t["label_bright"]), 1))
                p.drawEllipse(px, self._PT_R, self._PT_R)

        p.end()

    # ── Mouse ─────────────────────────────────────────────────────────────────

    def mousePressEvent(self, e) -> None:
        self._drag_idx = self._nearest_idx(e.position().x(), e.position().y())
        self.update()

    def mouseMoveEvent(self, e) -> None:
        if self._drag_idx < 0:
            return
        new_ve = self._px_to_ve(e.position().y())
        self._points[self._drag_idx][1] = new_ve
        self.update()

    def mouseReleaseEvent(self, _) -> None:
        self._drag_idx = -1
        self.update()
        self.ve_changed.emit(self.get_points())


class EngineConfigBody(QWidget):
    """Engine parameter panel: capacity, RPM limits, k/peak-torque, VE map."""

    engine_changed = pyqtSignal(dict)

    def __init__(self, theme: dict, parent=None) -> None:
        super().__init__(parent)
        t = theme

        lbl_ss = (f"color: {t['label_dim']}; font-size: 11px;"
                  f" letter-spacing: 0.4px; background: transparent;")
        spin_ss = (
            f"QDoubleSpinBox {{ background: {t['input_bg']}; color: {t['input_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 2px 4px; }}"
            f"QDoubleSpinBox::up-button, QDoubleSpinBox::down-button"
            f" {{ width: 14px; background: {t['btn_bg']}; }}"
            f"QDoubleSpinBox:focus {{ border-color: {t['accent']}; }}"
        )
        info_ss = (f"color: {t['label_bright']}; font-size: 11px;"
                   f" font-weight: 600; background: transparent;")

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(14, 14, 14, 12)
        vbox.setSpacing(8)

        def _spin(lo, hi, step, val, dec=1):
            s = QDoubleSpinBox()
            s.setRange(lo, hi); s.setSingleStep(step)
            s.setValue(val); s.setDecimals(dec)
            s.setStyleSheet(spin_ss); s.setFixedWidth(80)
            return s

        def _row(lay, label_text, widget):
            rw = QWidget(); rw.setStyleSheet("background: transparent;")
            rh = QHBoxLayout(rw)
            rh.setContentsMargins(0, 0, 0, 0); rh.setSpacing(6)
            rh.addWidget(QLabel(label_text, styleSheet=lbl_ss))
            rh.addStretch(); rh.addWidget(widget)
            lay.addWidget(rw)

        vbox.addWidget(_TemplateBar(_ASSET_ROOT / "engines", t,
                                    self._build_data, self.load_data))
        self._mass_lbl = QLabel("Mass: — kg", styleSheet=info_ss)
        vbox.addWidget(self._mass_lbl)

        # ── Basics ────────────────────────────────────────────────────────────
        sec_basic, lb = _group_section("Basics", t)
        self._capacity = _spin(0.5, 8.0, 0.1, 2.0)
        self._max_rpm  = _spin(3000, 12000, 500, 6000, 0)
        self._idle_rpm = _spin(400,   1500,  50,  800, 0)
        _row(lb, "Capacity (L)", self._capacity)
        _row(lb, "Max RPM",      self._max_rpm)
        _row(lb, "Idle RPM",     self._idle_rpm)

        # ── Combustion & Power (k ↔ τ ↔ kW ↔ hp, all linked) ──────────────────
        sec_comb, lc = _group_section("Combustion & Power", t)
        self._k_spin   = _spin(10.0,   5000.0, 10.0, 1232.0, 0)  # ×10³ internally
        self._tau_spin = _spin(10.0,   3000.0,  5.0,  200.0, 0)  # Nm
        self._kw_spin  = _spin( 1.0,   2000.0,  1.0,  101.0, 0)  # kW
        self._hp_spin  = _spin( 1.0,   2700.0,  1.0,  135.0, 0)  # hp
        _row(lc, "k  (×10³)",        self._k_spin)
        _row(lc, "Peak Torque (Nm)", self._tau_spin)
        _row(lc, "Peak Power (kW)",  self._kw_spin)
        _row(lc, "Horsepower (hp)",  self._hp_spin)

        # ── Tuning ────────────────────────────────────────────────────────────
        sec_tune, lt = _group_section("Tuning", t)
        self._c_drag = _spin(0.001, 1.0, 0.001, 0.05, 3)
        self._afr    = _spin(10.0, 20.0, 0.1, 14.7)
        _row(lt, "Engine Drag", self._c_drag)
        _row(lt, "Target AFR",  self._afr)

        # ── VE Map ────────────────────────────────────────────────────────────
        sec_ve, lv = _group_section("VE Map", t)
        self._ve_map = _VeMapWidget(t)
        lv.addWidget(self._ve_map)

        _secs = [sec_basic, sec_comb, sec_tune, sec_ve]
        for s in _secs:
            vbox.addWidget(s)
        make_accordion(_secs)
        sec_basic.expand()

        # ── Wire up ───────────────────────────────────────────────────────────
        self._engine   = EngineModel()
        self._updating = False   # guard against recursive signal loops
        self._loading  = False   # guard during template load

        self._capacity.valueChanged.connect(self._on_capacity_changed)
        self._max_rpm.valueChanged.connect(self._on_max_rpm_changed)
        self._k_spin.valueChanged.connect(self._on_k_changed)
        self._tau_spin.valueChanged.connect(self._on_tau_changed)
        self._kw_spin.valueChanged.connect(self._on_kw_changed)
        self._hp_spin.valueChanged.connect(self._on_hp_changed)
        self._ve_map.ve_changed.connect(self._on_ve_changed)

        for w in (self._idle_rpm, self._c_drag, self._afr):
            w.valueChanged.connect(lambda _: self._emit())

        self._update_all_from_k()

    # ── Linking helpers ───────────────────────────────────────────────────────

    def _sync_engine(self) -> None:
        """Push current UI values into self._engine."""
        self._engine.capacity_l = self._capacity.value()
        self._engine.k          = self._k_spin.value() * 1000.0
        self._engine.ve_map     = self._ve_map.get_points()

    def _update_all_from_k(self) -> None:
        """Recompute τ, kW, hp from the current k and update all three spinners."""
        self._sync_engine()
        tau = self._engine.peak_torque_nm()
        kw  = self._engine.peak_power_kw()
        hp  = kw * 1.341
        self._updating = True
        self._tau_spin.setValue(round(tau))
        self._kw_spin.setValue(round(kw))
        self._hp_spin.setValue(round(hp))
        self._updating = False

    def _set_k_then_update(self, new_k: float) -> None:
        """Set the k spinner (suppressing its own callback) then update τ/kW/hp."""
        self._engine.k = new_k
        self._updating = True
        self._k_spin.setValue(new_k / 1000.0)
        self._updating = False
        self._update_all_from_k()

    def _on_capacity_changed(self, _) -> None:
        self._update_all_from_k()
        self._emit()

    def _on_max_rpm_changed(self, _) -> None:
        self._ve_map.set_max_rpm(int(self._max_rpm.value()))
        self._emit()

    def _on_k_changed(self, _) -> None:
        if self._updating:
            return
        self._update_all_from_k()
        self._emit()

    def _on_tau_changed(self, _) -> None:
        if self._updating:
            return
        self._sync_engine()
        self._set_k_then_update(self._engine.k_from_peak_torque(self._tau_spin.value()))
        self._emit()

    def _on_kw_changed(self, _) -> None:
        if self._updating:
            return
        self._sync_engine()
        self._set_k_then_update(self._engine.k_from_peak_power(self._kw_spin.value()))
        self._emit()

    def _on_hp_changed(self, _) -> None:
        if self._updating:
            return
        self._sync_engine()
        self._set_k_then_update(
            self._engine.k_from_peak_power(self._hp_spin.value() / 1.341))
        self._emit()

    def _on_ve_changed(self, pts) -> None:
        self._engine.ve_map = pts
        self._update_all_from_k()
        self._emit()

    # ── Emit ─────────────────────────────────────────────────────────────────

    def set_mass(self, kg: float) -> None:
        self._mass_lbl.setText(f"Mass: {kg:,.0f} kg")

    def load_data(self, data: dict) -> None:
        """Load a template; k is the primary, τ/kW/hp are recomputed from it."""
        self._loading = True
        self._capacity.setValue(float(data.get("capacity_l", 2.0)))
        self._max_rpm.setValue(float(data.get("max_rpm", 6000)))
        self._idle_rpm.setValue(float(data.get("idle_rpm", 800)))
        self._c_drag.setValue(float(data.get("c_drag", 0.05)))
        self._afr.setValue(float(data.get("afr_target", 14.7)))
        ve = data.get("ve_map")
        if ve:
            self._ve_map.set_max_rpm(int(data.get("max_rpm", 6000)))
            self._ve_map.set_points(ve)
        self._updating = True
        self._k_spin.setValue(float(data.get("k", 1232000.0)) / 1000.0)
        self._updating = False
        self._update_all_from_k()        # derive τ/kW/hp from the loaded k
        self._loading = False
        self._emit()

    def _build_data(self) -> dict:
        return {
            "capacity_l":  self._capacity.value(),
            "max_rpm":     self._max_rpm.value(),
            "idle_rpm":    self._idle_rpm.value(),
            "k":           self._k_spin.value() * 1000.0,
            "c_drag":      self._c_drag.value(),
            "afr_target":  self._afr.value(),
            "ve_map":      self._ve_map.get_points(),
        }

    def _emit(self) -> None:
        if self._loading:
            return
        self.engine_changed.emit(self._build_data())


class VehicleBodyBody(QWidget):
    """Visibility toggles and chassis floor shape attributes for the Vehicle Body section."""

    floor_changed = pyqtSignal(dict)

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        t = theme
        self._loading = False

        lbl_ss = (
            f"color: {t['label_dim']}; font-size: 11px;"
            f" letter-spacing: 0.4px; background: transparent;"
        )
        chk_ss = (
            f"QCheckBox {{ color: {t['label_bright']}; font-size: 12px;"
            f" background: transparent; spacing: 6px; }}"
            f"QCheckBox::indicator {{ width: 13px; height: 13px;"
            f" border: 1px solid {t['input_border']}; border-radius: 2px;"
            f" background: {t['input_bg']}; }}"
            f"QCheckBox::indicator:checked {{ background: {t['accent']};"
            f" border-color: {t['accent']}; }}"
        )
        spin_ss = (
            f"QDoubleSpinBox {{ background: {t['input_bg']}; color: {t['input_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 2px 4px; }}"
            f"QDoubleSpinBox::up-button, QDoubleSpinBox::down-button"
            f" {{ width: 14px; background: {t['btn_bg']}; }}"
            f"QDoubleSpinBox:focus {{ border-color: {t['accent']}; }}"
        )
        combo_ss = (
            f"QComboBox {{ background: {t['input_bg']}; color: {t['input_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; padding: 2px 6px; }}"
            f"QComboBox::drop-down {{ border: none; width: 18px; }}"
            f"QComboBox QAbstractItemView {{ background: {t['input_bg']};"
            f" color: {t['input_text']};"
            f" selection-background-color: {t['accent']}; }}"
        )

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(14, 14, 14, 12)
        vbox.setSpacing(6)

        # ── Component mass readout ────────────────────────────────────────────
        info_ss = (f"color: {t['label_bright']}; font-size: 11px;"
                   f" font-weight: 600; background: transparent;")
        self._mass_lbl = QLabel("Mass: — kg", styleSheet=info_ss)
        vbox.addWidget(self._mass_lbl)
        vbox.addWidget(QFrame(styleSheet=f"background:{t['border']}; border:none;",
                               minimumHeight=1, maximumHeight=1))

        # ── Visibility toggles ────────────────────────────────────────────────
        for text, attr, default in [
            ("View Chassis Floor",         "_chk_floor",    True),
            ("View Fuel Tank",             "_chk_fuel",     False),
            ("View Body",                  "_chk_body",     False),
            ("View Windshields / Windows", "_chk_windows",  False),
            ("View Lights",                "_chk_lights",   False),
        ]:
            chk = QCheckBox(text)
            chk.setChecked(default)
            chk.setStyleSheet(chk_ss)
            vbox.addWidget(chk)
            setattr(self, attr, chk)

        sep = QFrame()
        sep.setFixedHeight(1)
        sep.setStyleSheet(f"background: {t['border']}; border: none;")
        vbox.addWidget(sep)

        # ── Chassis Floor attributes ──────────────────────────────────────────
        vbox.addWidget(QLabel("Chassis Floor", styleSheet=lbl_ss))

        def _spin(lo, hi, step, val, dec=2):
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setSingleStep(step)
            s.setValue(val)
            s.setDecimals(dec)
            s.setStyleSheet(spin_ss)
            s.setFixedWidth(72)
            return s

        def _row(layout, label_text, widget):
            rw = QWidget()
            rw.setStyleSheet("background: transparent;")
            rh = QHBoxLayout(rw)
            rh.setContentsMargins(0, 0, 0, 0)
            rh.setSpacing(6)
            rh.addWidget(QLabel(label_text, styleSheet=lbl_ss))
            rh.addStretch()
            rh.addWidget(widget)
            layout.addWidget(rw)

        vbox.addWidget(QLabel("Tractor Overhangs", styleSheet=lbl_ss))
        self._front_overhang  = _spin(0.0, 3.0, 0.05, 0.0)
        self._side_overhang   = _spin(0.0, 1.0, 0.02, 0.0)
        self._rear_overhang   = _spin(0.0, 3.0, 0.05, 0.0)

        _row(vbox, "Front Overhang (m)", self._front_overhang)
        _row(vbox, "Side Overhang (m)",  self._side_overhang)
        _row(vbox, "Rear Overhang (m)",  self._rear_overhang)

        def _corner_row(parent_layout, label_text):
            """Corner combo + sub-panels for bevel and round styles."""
            combo = QComboBox()
            combo.addItems(["Angular", "Bevelled", "Rounded"])
            combo.setStyleSheet(combo_ss)
            _row(parent_layout, label_text, combo)

            bevel_panel = QWidget()
            bevel_panel.setStyleSheet("background: transparent;")
            bvl = QVBoxLayout(bevel_panel)
            bvl.setContentsMargins(8, 2, 0, 2); bvl.setSpacing(4)
            b_depth = _spin(0.01, 1.0, 0.05, 0.10)
            b_angle = _spin(5.0, 85.0, 5.0, 45.0, 1)
            _row(bvl, "  Depth (m)", b_depth); _row(bvl, "  Angle (°)", b_angle)
            parent_layout.addWidget(bevel_panel); bevel_panel.setVisible(False)

            round_panel = QWidget()
            round_panel.setStyleSheet("background: transparent;")
            rnl = QVBoxLayout(round_panel)
            rnl.setContentsMargins(8, 2, 0, 2); rnl.setSpacing(4)
            r_radius = _spin(0.05, 2.0, 0.05, 0.20)
            r_ecc    = _spin(0.10, 3.0, 0.10, 1.00)
            _row(rnl, "  Radius (m)", r_radius); _row(rnl, "  Eccentricity", r_ecc)
            parent_layout.addWidget(round_panel); round_panel.setVisible(False)

            combo.currentIndexChanged.connect(
                lambda idx, bp=bevel_panel, rp=round_panel:
                    (bp.setVisible(idx == 1), rp.setVisible(idx == 2)))

            return combo, b_depth, b_angle, r_radius, r_ecc

        # ── Tractor corners ───────────────────────────────────────────────────
        vbox.addWidget(QLabel("Tractor Corners", styleSheet=lbl_ss))
        (self._front_corner_combo,
         self._front_bevel_depth, self._front_bevel_angle,
         self._front_round_radius, self._front_round_ecc) = _corner_row(vbox, "Front Corner")
        (self._rear_corner_combo,
         self._rear_bevel_depth, self._rear_bevel_angle,
         self._rear_round_radius, self._rear_round_ecc) = _corner_row(vbox, "Rear Corner")

        # ── Trailer group (overhangs + corners) — hidden when no trailer ──────
        self._trailer_group = QWidget()
        self._trailer_group.setStyleSheet("background: transparent;")
        tg = QVBoxLayout(self._trailer_group)
        tg.setContentsMargins(0, 0, 0, 0); tg.setSpacing(6)
        tg.addWidget(QFrame(styleSheet=f"background:{t['border']}; border:none;",
                            minimumHeight=1, maximumHeight=1))

        tg.addWidget(QLabel("Trailer Overhangs", styleSheet=lbl_ss))
        self._tr_front_overhang = _spin(0.0, 3.0, 0.05, 0.0)
        self._tr_side_overhang  = _spin(0.0, 1.0, 0.02, 0.0)
        self._tr_rear_overhang  = _spin(0.0, 3.0, 0.05, 0.0)
        _row(tg, "Front Overhang (m)", self._tr_front_overhang)
        _row(tg, "Side Overhang (m)",  self._tr_side_overhang)
        _row(tg, "Rear Overhang (m)",  self._tr_rear_overhang)

        tg.addWidget(QLabel("Trailer Corners", styleSheet=lbl_ss))
        (self._tr_front_corner_combo,
         self._tr_front_bevel_depth, self._tr_front_bevel_angle,
         self._tr_front_round_radius, self._tr_front_round_ecc) = _corner_row(tg, "Front Corner")
        (self._tr_rear_corner_combo,
         self._tr_rear_bevel_depth, self._tr_rear_bevel_angle,
         self._tr_rear_round_radius, self._tr_rear_round_ecc) = _corner_row(tg, "Rear Corner")

        vbox.addWidget(self._trailer_group)
        self._trailer_group.setVisible(False)

        # Wire every interactive control to emit the full config dict
        for chk in (self._chk_floor, self._chk_fuel, self._chk_body,
                    self._chk_windows, self._chk_lights):
            chk.toggled.connect(lambda _: self._emit())
        for spin in (self._front_overhang, self._side_overhang, self._rear_overhang,
                     self._tr_front_overhang, self._tr_side_overhang, self._tr_rear_overhang,
                     self._front_bevel_depth, self._front_bevel_angle,
                     self._front_round_radius, self._front_round_ecc,
                     self._rear_bevel_depth, self._rear_bevel_angle,
                     self._rear_round_radius, self._rear_round_ecc,
                     self._tr_front_bevel_depth, self._tr_front_bevel_angle,
                     self._tr_front_round_radius, self._tr_front_round_ecc,
                     self._tr_rear_bevel_depth, self._tr_rear_bevel_angle,
                     self._tr_rear_round_radius, self._tr_rear_round_ecc):
            spin.valueChanged.connect(lambda _: self._emit())
        for combo in (self._front_corner_combo, self._rear_corner_combo,
                      self._tr_front_corner_combo, self._tr_rear_corner_combo):
            combo.currentIndexChanged.connect(lambda _: self._emit())

    def set_trailer_enabled(self, enabled: bool) -> None:
        """Show the trailer overhang / corner controls only when a trailer exists."""
        self._trailer_group.setVisible(bool(enabled))

    def set_mass(self, kg: float) -> None:
        self._mass_lbl.setText(f"Mass: {kg:,.0f} kg")

    def _build_floor_data(self) -> dict:
        return {
            "view_chassis_floor":       self._chk_floor.isChecked(),
            "view_fuel_tank":           self._chk_fuel.isChecked(),
            "view_body":                self._chk_body.isChecked(),
            "view_windshields":         self._chk_windows.isChecked(),
            "view_lights":              self._chk_lights.isChecked(),
            "front_overhang":           self._front_overhang.value(),
            "side_overhang":            self._side_overhang.value(),
            "rear_overhang":            self._rear_overhang.value(),
            "trailer_front_overhang":   self._tr_front_overhang.value(),
            "trailer_side_overhang":    self._tr_side_overhang.value(),
            "trailer_rear_overhang":    self._tr_rear_overhang.value(),
            "front_corner":             self._front_corner_combo.currentText().lower(),
            "front_bevel_depth":        self._front_bevel_depth.value(),
            "front_bevel_angle":        self._front_bevel_angle.value(),
            "front_round_radius":       self._front_round_radius.value(),
            "front_round_eccentricity": self._front_round_ecc.value(),
            "rear_corner":              self._rear_corner_combo.currentText().lower(),
            "rear_bevel_depth":         self._rear_bevel_depth.value(),
            "rear_bevel_angle":         self._rear_bevel_angle.value(),
            "rear_round_radius":        self._rear_round_radius.value(),
            "rear_round_eccentricity":  self._rear_round_ecc.value(),
            "trailer_front_corner":             self._tr_front_corner_combo.currentText().lower(),
            "trailer_front_bevel_depth":        self._tr_front_bevel_depth.value(),
            "trailer_front_bevel_angle":        self._tr_front_bevel_angle.value(),
            "trailer_front_round_radius":       self._tr_front_round_radius.value(),
            "trailer_front_round_eccentricity": self._tr_front_round_ecc.value(),
            "trailer_rear_corner":              self._tr_rear_corner_combo.currentText().lower(),
            "trailer_rear_bevel_depth":         self._tr_rear_bevel_depth.value(),
            "trailer_rear_bevel_angle":         self._tr_rear_bevel_angle.value(),
            "trailer_rear_round_radius":        self._tr_rear_round_radius.value(),
            "trailer_rear_round_eccentricity":  self._tr_rear_round_ecc.value(),
        }

    def load_data(self, data: dict) -> None:
        if not data:
            return
        self._loading = True
        chk = {"view_chassis_floor": self._chk_floor, "view_fuel_tank": self._chk_fuel,
               "view_body": self._chk_body, "view_windshields": self._chk_windows,
               "view_lights": self._chk_lights}
        for key, w in chk.items():
            if key in data:
                w.setChecked(bool(data[key]))
        spins = {
            "front_overhang": self._front_overhang, "side_overhang": self._side_overhang,
            "rear_overhang": self._rear_overhang,
            "trailer_front_overhang": self._tr_front_overhang,
            "trailer_side_overhang": self._tr_side_overhang,
            "trailer_rear_overhang": self._tr_rear_overhang,
            "front_bevel_depth": self._front_bevel_depth, "front_bevel_angle": self._front_bevel_angle,
            "front_round_radius": self._front_round_radius, "front_round_eccentricity": self._front_round_ecc,
            "rear_bevel_depth": self._rear_bevel_depth, "rear_bevel_angle": self._rear_bevel_angle,
            "rear_round_radius": self._rear_round_radius, "rear_round_eccentricity": self._rear_round_ecc,
            "trailer_front_bevel_depth": self._tr_front_bevel_depth, "trailer_front_bevel_angle": self._tr_front_bevel_angle,
            "trailer_front_round_radius": self._tr_front_round_radius, "trailer_front_round_eccentricity": self._tr_front_round_ecc,
            "trailer_rear_bevel_depth": self._tr_rear_bevel_depth, "trailer_rear_bevel_angle": self._tr_rear_bevel_angle,
            "trailer_rear_round_radius": self._tr_rear_round_radius, "trailer_rear_round_eccentricity": self._tr_rear_round_ecc,
        }
        for key, w in spins.items():
            if key in data:
                w.setValue(float(data[key]))
        combos = {"front_corner": self._front_corner_combo, "rear_corner": self._rear_corner_combo,
                  "trailer_front_corner": self._tr_front_corner_combo,
                  "trailer_rear_corner": self._tr_rear_corner_combo}
        for key, c in combos.items():
            if key in data:
                c.setCurrentText(str(data[key]).capitalize())
        self._loading = False
        self._emit()

    def _emit(self) -> None:
        if self._loading:
            return
        self.floor_changed.emit(self._build_floor_data())


class ComponentDesignerWidget(QWidget):
    """Right panel: collapsible, mutually exclusive component sections."""

    wheel_frame_changed          = pyqtSignal(dict)         # raw wheel-frame config
    component_visibility_changed = pyqtSignal(str, bool)   # (component_name, visible)
    chassis_floor_changed        = pyqtSignal(dict)         # vehicle body / floor config
    engine_cfg_changed           = pyqtSignal(dict)         # engine model config
    transmission_cfg_changed     = pyqtSignal(dict)         # transmission config
    brakes_cfg_changed           = pyqtSignal(dict)         # brake model config

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background: {theme['window_bg']}; border: none;")

        t = theme
        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)

        vbox.addWidget(SectionHeader("CONFIGURATION PANEL", t))

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setStyleSheet(
            f"QScrollArea {{ background: {t['window_bg']}; border: none; }}"
            f"QScrollBar:vertical {{ background: {t['window_bg']}; width: 6px; }}"
            f"QScrollBar::handle:vertical {{ background: {t['border']}; border-radius: 3px; }}"
            f"QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}"
        )

        inner = QWidget()
        inner.setStyleSheet(f"background: {t['window_bg']};")
        inner_v = QVBoxLayout(inner)
        inner_v.setContentsMargins(0, 0, 0, 0)
        inner_v.setSpacing(0)

        sections: list[CollapsibleSection] = []

        # Latest frame footprint — body mass scales with it.
        self._frame_l: float = 4.0
        self._frame_w: float = 1.8
        self._last_body_cfg: dict = {}

        # ── Frame and Body ────────────────────────────────────────────────────
        # Vehicle Body is a sub-component inside the frame section; its config is
        # saved under the "body" key of the frame template.
        self._vb_body = VehicleBodyBody(t)
        vb_sec = CollapsibleSection("Vehicle Body", t, self._vb_body, checkable=True)
        self._vb_sec = vb_sec
        self._vb_body.floor_changed.connect(self.chassis_floor_changed)
        self._vb_body.floor_changed.connect(self._update_body_mass)
        vb_sec.visibility_changed.connect(
            lambda v: self.component_visibility_changed.emit("Vehicle Body", v)
        )

        self._wf_body = WheelFrameBody(
            t, body_section=vb_sec,
            body_get=self._vb_body._build_floor_data,
            body_set=self._vb_body.load_data)
        self._wf_body.frame_changed.connect(self.wheel_frame_changed)
        wf_sec = CollapsibleSection("Frame and Body", t, self._wf_body, checkable=True)
        self._wf_body.frame_changed.connect(
            lambda cfg, _s=wf_sec: _s.set_subtitle(
                f"{resolve_frame(cfg).get('mass_kg', 0):.0f} kg"
            )
        )
        self._wf_body.frame_changed.connect(self._on_frame_changed_for_body)
        wf_sec.visibility_changed.connect(
            lambda v: self.component_visibility_changed.emit("Wheel Frame", v)
        )
        inner_v.addWidget(wf_sec)
        sections.append(wf_sec)

        # ── Engine ────────────────────────────────────────────────────────────
        self._eng_body = EngineConfigBody(t)
        self._eng_body.engine_changed.connect(self.engine_cfg_changed)
        eng_sec = CollapsibleSection("Engine", t, self._eng_body, checkable=True,
                                     checked=False)
        self._eng_body.engine_changed.connect(
            lambda cfg, b=self._eng_body, s=eng_sec:
                self._show_mass(b, s, engine_mass(cfg)))
        eng_sec.visibility_changed.connect(
            lambda v: self.component_visibility_changed.emit("Engine", v)
        )
        inner_v.addWidget(eng_sec)
        sections.append(eng_sec)

        # ── Transmission ──────────────────────────────────────────────────────
        self._trans_body = TransmissionConfigBody(t)
        self._trans_body.transmission_changed.connect(self.transmission_cfg_changed)
        trans_sec = CollapsibleSection("Transmission", t, self._trans_body,
                                       checkable=True, checked=False)
        self._trans_body.transmission_changed.connect(
            lambda cfg, b=self._trans_body, s=trans_sec:
                self._show_mass(b, s, transmission_mass(cfg)))
        trans_sec.visibility_changed.connect(
            lambda v: self.component_visibility_changed.emit("Transmission", v)
        )
        inner_v.addWidget(trans_sec)
        sections.append(trans_sec)

        # ── Brakes ────────────────────────────────────────────────────────────
        self._brk_body = BrakeConfigBody(t)
        self._brk_body.brakes_changed.connect(self.brakes_cfg_changed)
        brk_sec = CollapsibleSection("Brakes", t, self._brk_body, checkable=True,
                                     checked=False)
        self._brk_body.brakes_changed.connect(
            lambda cfg, b=self._brk_body, s=brk_sec:
                self._show_mass(b, s, brake_mass(cfg)))
        brk_sec.visibility_changed.connect(
            lambda v: self.component_visibility_changed.emit("Brakes", v)
        )
        inner_v.addWidget(brk_sec)
        sections.append(brk_sec)

        # Accordion mutual exclusion (top-level sections)
        make_accordion(sections)

        inner_v.addStretch()
        scroll.setWidget(inner)
        vbox.addWidget(scroll, 1)

    # ── Mass display helpers ───────────────────────────────────────────────────

    @staticmethod
    def _show_mass(body, section, kg: float) -> None:
        """Update a component's in-body readout and header subtitle."""
        body.set_mass(kg)
        section.set_subtitle(f"{kg:.0f} kg")

    def _on_frame_changed_for_body(self, cfg: dict) -> None:
        """Frame footprint changed — body mass depends on it, so recompute.
        Also show/hide the trailer body controls based on the fifth-wheel state."""
        res = resolve_frame(cfg)
        self._frame_l = res.get("frame_length_m", 4.0)
        self._frame_w = res.get("frame_width_m",  1.8)
        self._vb_body.set_trailer_enabled(
            res.get("fifth_wheel", {}).get("enabled", False))
        self._update_body_mass(self._last_body_cfg)

    def _update_body_mass(self, cfg: dict) -> None:
        self._last_body_cfg = cfg
        kg = body_mass(cfg, self._frame_l, self._frame_w)
        self._show_mass(self._vb_body, self._vb_sec, kg)

    def apply_defaults(self) -> None:
        """Push initial configs to the viewport on startup.

        Emit the bodies' own signals (not the widget passthroughs) so the mass
        readouts wired to them populate too. Wheel frame first, so the body's
        footprint-dependent mass uses the right frame dims.
        """
        self._wf_body.frame_changed.emit(self._wf_body._build_data())
        self._vb_body.floor_changed.emit(self._vb_body._build_floor_data())
        self._eng_body.engine_changed.emit(self._eng_body._build_data())
        self._trans_body.transmission_changed.emit(self._trans_body._build_data())
        self._brk_body.brakes_changed.emit(self._brk_body._build_data())
