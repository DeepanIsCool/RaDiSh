from __future__ import annotations

import json
from pathlib import Path

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QSpinBox, QSlider, QComboBox, QFrame, QInputDialog, QMessageBox,
)
from PyQt6.QtCore import Qt, pyqtSignal

from gui.widgets import AssetCombo

_ASSETS_DIR      = Path(__file__).parents[3] / "assets" / "wheelframes"
_AXLE_GAP_M      = 0.05       # minimum clear gap between tyre edges (metres)
_WHEELS_PER_AXLE = [1, 2, 4]
_WHEELS_LABELS    = ["1  (centre)", "2  (std)", "4  (dual)"]
_GROUP_DEFAULTS   = {"front": 15, "middle": 50, "rear": 85}


# ── Public resolver ───────────────────────────────────────────────────────────

def resolve_frame(cfg: dict) -> dict:
    """
    Expand a raw wheel-frame config into a flat canonical dict consumed by the
    viewport and direct-control panel.

    Steering modes : "front" | "rear" | "both"
    Drive modes    : "front" | "rear" | "both"
    Middle group   : always passive (non-steerable, non-drivable)

    Returned keys
    -------------
    frame_length_m, frame_width_m,
    tyre_radius_m, tyre_width_m,
    differential  ("open" | "locked"),
    steering_mode, drive_mode,
    axles : list of {position, steerable, drivable, wheels, group}
    """
    steer = cfg.get("steering_mode", "front")
    drive = cfg.get("drive_mode",    "rear")

    flen_m    = cfg.get("frame_length_m", 4.0)
    tyre_diam = cfg.get("tyre_radius_cm", 33) / 100.0 * 2   # metres

    axles: list[dict] = []
    for gname in ("front", "middle", "rear"):
        g     = cfg.get("groups", {}).get(gname, {})
        count = g.get("axle_count", 0)
        if not count:
            continue
        wheels    = g.get("wheels_per_axle", 2)
        ctr       = g.get("position_pct", _GROUP_DEFAULTS[gname]) / 100.0
        steerable = (gname == "front" and steer in ("front", "both")) or \
                    (gname == "rear"  and steer in ("rear",  "both"))
        drivable  = (gname == "front" and drive in ("front", "both")) or \
                    (gname == "rear"  and drive in ("rear",  "both"))
        # Centre-to-centre step = tyre diameter + user-defined clear gap
        sep_m   = g.get("separation_cm", 5) / 100.0
        step_m  = tyre_diam + sep_m
        spacing = max(0.005, step_m / flen_m) if flen_m > 0 else 0.05
        for i in range(count):
            off = (i - (count - 1) / 2.0) * spacing
            axles.append({
                "position":  max(0.01, min(0.99, ctr + off)),
                "steerable": steerable,
                "drivable":  drivable,
                "wheels":    wheels,
                "group":     gname,
            })

    axles.sort(key=lambda a: a["position"])
    return {
        "frame_length_m": cfg.get("frame_length_m", 4.0),
        "frame_width_m":  cfg.get("frame_width_m",  1.8),
        "tyre_radius_m":  cfg.get("tyre_radius_cm", 33) / 100.0,
        "tyre_width_m":   cfg.get("tyre_width_cm",  20) / 100.0,
        "differential":   cfg.get("differential",   "open"),
        "steering_mode":  steer,
        "drive_mode":     drive,
        "axles":          axles,
    }


# ── Style helpers ─────────────────────────────────────────────────────────────

def _sep(t: dict) -> QFrame:
    f = QFrame(); f.setFixedHeight(1)
    f.setStyleSheet(f"background: {t['border']}; border: none;")
    return f

def _lbl(t: dict) -> str:
    return f"color: {t['label_dim']}; font-size: 12px; letter-spacing: 0.5px; background: transparent;"

def _dim(t: dict) -> str:
    return f"color: {t['label_dim']}; font-size: 11px; background: transparent;"

def _sec(t: dict) -> str:
    return f"color: {t['label_bright']}; font-size: 12px; font-weight: 600; background: transparent;"

def _btn_ss(t: dict) -> str:
    return (
        f"QPushButton {{ background: {t['btn_bg']}; color: {t['btn_text']};"
        f"border: 1px solid {t['input_border']}; border-radius: 3px;"
        f"font-size: 13px; padding: 5px 0px; }}"
        f"QPushButton:hover {{ background: {t['btn_hover_bg']}; color: {t['btn_hover_text']}; }}"
        f"QPushButton:pressed {{ background: {t['btn_active_bg']}; color: {t['btn_active_text']}; }}"
        f"QPushButton:disabled {{ background: {t['input_bg']}; color: {t['input_border']};"
        f"border-color: {t['input_border']}; }}"
    )

def _tog_ss(t: dict) -> str:
    return (
        f"QPushButton {{ background: {t['btn_bg']}; color: {t['btn_text']};"
        f"border: 1px solid {t['input_border']}; border-radius: 3px;"
        f"font-size: 12px; padding: 4px 6px; }}"
        f"QPushButton:checked {{ background: {t['btn_active_bg']}; color: {t['accent']};"
        f"border-color: {t['accent']}; }}"
        f"QPushButton:hover:!checked {{ background: {t['btn_hover_bg']}; color: {t['btn_hover_text']}; }}"
    )

def _spin_ss(t: dict) -> str:
    return (
        f"QSpinBox {{ background: {t['input_bg']}; color: {t['input_text']};"
        f"border: 1px solid {t['input_border']}; border-radius: 3px; padding: 2px 4px; font-size: 12px; }}"
        f"QSpinBox::up-button, QSpinBox::down-button {{ background: {t['btn_bg']}; border: none; width: 16px; }}"
        f"QSpinBox::up-button:hover, QSpinBox::down-button:hover {{ background: {t['btn_hover_bg']}; }}"
    )

def _sl_ss(t: dict) -> str:
    return (
        f"QSlider::groove:horizontal {{ background: {t['slider_groove']}; height: 4px; border-radius: 2px; }}"
        f"QSlider::sub-page:horizontal {{ background: {t['slider_fill']}; height: 4px; border-radius: 2px; }}"
        f"QSlider::handle:horizontal {{ background: {t['slider_handle']};"
        f"width: 12px; height: 12px; border-radius: 6px; margin: -4px 0; }}"
    )

def _cb_ss(t: dict) -> str:
    return (
        f"QComboBox {{ background: {t['input_bg']}; color: {t['input_text']};"
        f"border: 1px solid {t['input_border']}; border-radius: 3px; padding: 2px 6px; font-size: 12px; }}"
        f"QComboBox QAbstractItemView {{ background: {t['input_bg']}; color: {t['input_text']};"
        f"border: 1px solid {t['input_border']}; outline: none;"
        f"selection-background-color: {t['btn_active_bg']}; selection-color: {t['btn_active_text']}; }}"
    )

def _radio_group(labels: list[str], t: dict) -> tuple[QWidget, list[QPushButton]]:
    row = QWidget(); row.setStyleSheet("background: transparent;")
    h = QHBoxLayout(row); h.setContentsMargins(0, 0, 0, 0); h.setSpacing(4)
    btns: list[QPushButton] = []
    for lbl in labels:
        b = QPushButton(lbl); b.setCheckable(True)
        b.setStyleSheet(_tog_ss(t)); b.setCursor(Qt.CursorShape.PointingHandCursor)
        h.addWidget(b); btns.append(b)

    def _enforce(clicked: QPushButton, checked: bool) -> None:
        if not checked:
            clicked.blockSignals(True); clicked.setChecked(True); clicked.blockSignals(False)
            return
        for b in btns:
            if b is not clicked:
                b.blockSignals(True); b.setChecked(False); b.blockSignals(False)

    for b in btns:
        b.toggled.connect(lambda c, btn=b: _enforce(btn, c))
    btns[0].setChecked(True)
    return row, btns

def _slider_row(title: str, lo: int, hi: int, default: int,
                fmt, t: dict, vbox: QVBoxLayout) -> tuple[QSlider, QLabel]:
    rw = QWidget(); rw.setStyleSheet("background: transparent;")
    rv = QVBoxLayout(rw); rv.setContentsMargins(0, 0, 0, 2); rv.setSpacing(2)
    top = QWidget(); top.setStyleSheet("background: transparent;")
    th = QHBoxLayout(top); th.setContentsMargins(0, 0, 0, 0)
    th.addWidget(QLabel(title, styleSheet=_lbl(t))); th.addStretch()
    val_lbl = QLabel(fmt(default), styleSheet=_dim(t)); val_lbl.setFixedWidth(58)
    th.addWidget(val_lbl); rv.addWidget(top)
    sl = QSlider(Qt.Orientation.Horizontal)
    sl.setRange(lo, hi); sl.setValue(default); sl.setStyleSheet(_sl_ss(t))
    sl.valueChanged.connect(lambda v: val_lbl.setText(fmt(v)))
    rv.addWidget(sl); vbox.addWidget(rw)
    return sl, val_lbl


# ── Axle-group row ────────────────────────────────────────────────────────────

class _GroupRow(QWidget):
    """Compact single row: GROUP-NAME  Axles[N]  Sep[cm]  @[pos%]  Wheels[combo]"""
    changed = pyqtSignal()

    def __init__(self, group_name: str, t: dict, parent=None):
        super().__init__(parent)
        self._group_name = group_name
        self.setStyleSheet("background: transparent;")
        h = QHBoxLayout(self); h.setContentsMargins(0, 0, 0, 0); h.setSpacing(6)

        name_lbl = QLabel(group_name.upper()); name_lbl.setFixedWidth(52)
        name_lbl.setStyleSheet(_lbl(t)); h.addWidget(name_lbl)

        self._count = QSpinBox()
        self._count.setRange(0, 6)
        self._count.setValue(0 if group_name == "middle" else 1)
        self._count.setFixedWidth(46); self._count.setStyleSheet(_spin_ss(t))
        self._count.setToolTip("Number of axles in this group")
        self._count.valueChanged.connect(self.changed)
        h.addWidget(self._count)

        self._sep = QSpinBox()
        self._sep.setRange(0, 999); self._sep.setValue(5)
        self._sep.setSuffix(" cm"); self._sep.setFixedWidth(64); self._sep.setStyleSheet(_spin_ss(t))
        self._sep.setToolTip("Clear gap between tyre edges of adjacent axles in this group (cm)")
        self._sep.valueChanged.connect(self.changed)
        h.addWidget(self._sep)

        h.addWidget(QLabel("@", styleSheet=_dim(t)))

        self._pos = QSpinBox()
        self._pos.setRange(1, 99); self._pos.setValue(_GROUP_DEFAULTS[group_name])
        self._pos.setSuffix(" %"); self._pos.setFixedWidth(58); self._pos.setStyleSheet(_spin_ss(t))
        self._pos.setToolTip("Group centre position along frame length")
        self._pos.valueChanged.connect(self.changed)
        h.addWidget(self._pos)

        self._wheels = QComboBox()
        self._wheels.addItems(_WHEELS_LABELS); self._wheels.setCurrentIndex(1)
        self._wheels.setStyleSheet(_cb_ss(t))
        self._wheels.setCursor(Qt.CursorShape.PointingHandCursor)
        self._wheels.currentIndexChanged.connect(self.changed)
        h.addWidget(self._wheels)

    def get(self) -> dict:
        return {
            "axle_count":      self._count.value(),
            "separation_cm":   self._sep.value(),
            "position_pct":    self._pos.value(),
            "wheels_per_axle": _WHEELS_PER_AXLE[self._wheels.currentIndex()],
        }

    def set(self, d: dict) -> None:
        self._count.blockSignals(True)
        self._sep.blockSignals(True)
        self._pos.blockSignals(True)
        self._wheels.blockSignals(True)
        self._count.setValue(d.get("axle_count",    0 if self._group_name == "middle" else 1))
        self._sep.setValue(  d.get("separation_cm", 5))
        self._pos.setValue(  d.get("position_pct",  _GROUP_DEFAULTS[self._group_name]))
        w = d.get("wheels_per_axle", 2)
        self._wheels.setCurrentIndex(_WHEELS_PER_AXLE.index(w) if w in _WHEELS_PER_AXLE else 1)
        self._count.blockSignals(False)
        self._sep.blockSignals(False)
        self._pos.blockSignals(False)
        self._wheels.blockSignals(False)


# ── Main body widget ──────────────────────────────────────────────────────────

class WheelFrameBody(QWidget):
    """
    Wheel Frame designer.  Emits frame_changed(dict) on every config change.

    JSON schema
    -----------
    {
      "frame_length_m": 4.0,
      "frame_width_m":  1.8,
      "tyre_radius_cm": 33,
      "tyre_width_cm":  20,
      "steering_mode":  "front" | "rear" | "both",
      "drive_mode":     "front" | "rear" | "both",
      "differential":   "open"  | "locked",
      "groups": {
        "front":  {"axle_count": 1, "position_pct": 15, "wheels_per_axle": 2},
        "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
        "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 2}
      }
    }
    """

    frame_changed = pyqtSignal(dict)

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        _ASSETS_DIR.mkdir(parents=True, exist_ok=True)
        self._dir = _ASSETS_DIR
        self._current_name = ""
        t = theme

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(14, 14, 14, 12)
        vbox.setSpacing(8)

        # ── Load / Create ─────────────────────────────────────────────────────
        vbox.addWidget(QLabel("Load / Create", styleSheet=_lbl(t)))
        self._combo = AssetCombo(_ASSETS_DIR, t)
        self._combo.activated.connect(self._on_activated)
        self._combo.lineEdit().returnPressed.connect(self._on_return_pressed)
        vbox.addWidget(self._combo)

        sv = QWidget(); sv.setStyleSheet("background: transparent;")
        sh = QHBoxLayout(sv); sh.setContentsMargins(0, 0, 0, 0); sh.setSpacing(6)
        self._save_btn    = QPushButton("Save")
        self._save_as_btn = QPushButton("Save As")
        self._delete_btn  = QPushButton("Delete")
        for b in (self._save_btn, self._save_as_btn, self._delete_btn):
            b.setStyleSheet(_btn_ss(t)); b.setCursor(Qt.CursorShape.PointingHandCursor)
            sh.addWidget(b)
        vbox.addWidget(sv)
        self._save_btn.clicked.connect(self._on_save)
        self._save_as_btn.clicked.connect(self._on_save_as)
        self._delete_btn.clicked.connect(self._on_delete)

        vbox.addWidget(_sep(t))

        # ── Frame dimensions ──────────────────────────────────────────────────
        vbox.addWidget(QLabel("FRAME", styleSheet=_sec(t)))
        self._flen_sl, self._flen_lbl = _slider_row(
            "Length", 10, 200, 40, lambda v: f"{v / 10:.1f} m", t, vbox)
        self._fwid_sl, self._fwid_lbl = _slider_row(
            "Width",   5,  50, 18, lambda v: f"{v / 10:.1f} m", t, vbox)
        self._flen_sl.valueChanged.connect(lambda _: self._emit())
        self._fwid_sl.valueChanged.connect(lambda _: self._emit())

        vbox.addWidget(_sep(t))

        # ── Tyre ──────────────────────────────────────────────────────────────
        vbox.addWidget(QLabel("TYRE", styleSheet=_sec(t)))
        self._tyre_r_sl, self._tyre_r_lbl = _slider_row(
            "Radius", 10, 80, 33, lambda v: f"{v} cm", t, vbox)
        self._tyre_w_sl, self._tyre_w_lbl = _slider_row(
            "Width",   5, 50, 20, lambda v: f"{v} cm", t, vbox)
        self._tyre_r_sl.valueChanged.connect(lambda _: self._emit())
        self._tyre_w_sl.valueChanged.connect(lambda _: self._emit())

        vbox.addWidget(_sep(t))

        # ── Steering mode ─────────────────────────────────────────────────────
        vbox.addWidget(QLabel("STEERING", styleSheet=_sec(t)))
        steer_row, self._steer_btns = _radio_group(["Front", "Rear", "Both"], t)
        vbox.addWidget(steer_row)
        for b in self._steer_btns:
            b.toggled.connect(lambda _: self._emit())

        # ── Drive mode ────────────────────────────────────────────────────────
        vbox.addWidget(QLabel("DRIVE", styleSheet=_sec(t)))
        drive_row, self._drive_btns = _radio_group(["Front", "Rear", "Both"], t)
        # default: Rear drive
        self._drive_btns[1].setChecked(True)
        vbox.addWidget(drive_row)
        for b in self._drive_btns:
            b.toggled.connect(lambda _: self._emit())

        # ── Differential ──────────────────────────────────────────────────────
        diff_row = QWidget(); diff_row.setStyleSheet("background: transparent;")
        dh = QHBoxLayout(diff_row); dh.setContentsMargins(0, 0, 0, 0); dh.setSpacing(4)
        dh.addWidget(QLabel("DIFFERENTIAL", styleSheet=_sec(t))); dh.addStretch()
        self._diff_open   = QPushButton("Open");   self._diff_open.setCheckable(True)
        self._diff_locked = QPushButton("Locked"); self._diff_locked.setCheckable(True)
        self._diff_open.setChecked(True)
        for b in (self._diff_open, self._diff_locked):
            b.setStyleSheet(_tog_ss(t)); b.setCursor(Qt.CursorShape.PointingHandCursor)
            dh.addWidget(b)
        self._diff_open.clicked.connect(lambda: self._set_diff("open"))
        self._diff_locked.clicked.connect(lambda: self._set_diff("locked"))
        vbox.addWidget(diff_row)

        vbox.addWidget(_sep(t))

        # ── Axle groups ───────────────────────────────────────────────────────
        vbox.addWidget(QLabel("AXLE GROUPS", styleSheet=_sec(t)))
        hdr = QWidget(); hdr.setStyleSheet("background: transparent;")
        hh = QHBoxLayout(hdr); hh.setContentsMargins(0, 0, 0, 0); hh.setSpacing(6)
        for lbl in ("Group", "Axles", "Sep", "", "Position", "Wheels / axle"):
            l = QLabel(lbl, styleSheet=_dim(t))
            if lbl == "Group":    l.setFixedWidth(52)
            if lbl == "Axles":    l.setFixedWidth(46)
            if lbl == "Sep":      l.setFixedWidth(64)
            if lbl == "":         l.setFixedWidth(14)
            hh.addWidget(l)
        vbox.addWidget(hdr)

        self._groups: dict[str, _GroupRow] = {}
        for gname in ("front", "middle", "rear"):
            row = _GroupRow(gname, t)
            row.changed.connect(self._emit)
            vbox.addWidget(row)
            self._groups[gname] = row

        vbox.addStretch()
        self._sync_buttons()
        self._emit()

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _active(self, btns: list[QPushButton], keys: list[str]) -> str:
        for k, b in zip(keys, btns):
            if b.isChecked():
                return k
        return keys[0]

    def _set_active(self, btns: list[QPushButton], keys: list[str], key: str) -> None:
        idx = keys.index(key) if key in keys else 0
        for i, b in enumerate(btns):
            b.blockSignals(True); b.setChecked(i == idx); b.blockSignals(False)

    def _set_diff(self, key: str) -> None:
        self._diff_open.blockSignals(True);   self._diff_locked.blockSignals(True)
        self._diff_open.setChecked(key == "open")
        self._diff_locked.setChecked(key == "locked")
        self._diff_open.blockSignals(False);  self._diff_locked.blockSignals(False)
        self._emit()

    def _build_data(self) -> dict:
        _keys = ["front", "rear", "both"]
        return {
            "frame_length_m": self._flen_sl.value() / 10.0,
            "frame_width_m":  self._fwid_sl.value() / 10.0,
            "tyre_radius_cm": self._tyre_r_sl.value(),
            "tyre_width_cm":  self._tyre_w_sl.value(),
            "steering_mode":  self._active(self._steer_btns, _keys),
            "drive_mode":     self._active(self._drive_btns, _keys),
            "differential":   "locked" if self._diff_locked.isChecked() else "open",
            "groups":         {n: g.get() for n, g in self._groups.items()},
        }

    def _load_into_ui(self, data: dict) -> None:
        for sl, key, scale in (
            (self._flen_sl, "frame_length_m", 10),
            (self._fwid_sl, "frame_width_m",  10),
        ):
            sl.blockSignals(True)
            sl.setValue(round(data.get(key, sl.value() / 10) * scale))
            sl.blockSignals(False)
        for sl, key in (
            (self._tyre_r_sl, "tyre_radius_cm"),
            (self._tyre_w_sl, "tyre_width_cm"),
        ):
            sl.blockSignals(True)
            sl.setValue(data.get(key, sl.value()))
            sl.blockSignals(False)
        _keys = ["front", "rear", "both"]
        self._set_active(self._steer_btns, _keys, data.get("steering_mode", "front"))
        self._set_active(self._drive_btns, _keys, data.get("drive_mode",    "rear"))
        self._set_diff(data.get("differential", "open"))
        for name, row in self._groups.items():
            row.set(data.get("groups", {}).get(name, {}))

    def _emit(self) -> None:
        self.frame_changed.emit(self._build_data())

    # ── Load / Save ────────────────────────────────────────────────────────────

    def _path(self, name: str) -> Path:
        return self._dir / f"{name}.json"

    def _resolve_name(self, name: str) -> None:
        name = name.strip()
        if not name:
            return
        self._current_name = name
        p = self._path(name)
        if p.exists():
            self._load_into_ui(json.loads(p.read_text()))
        self._emit(); self._sync_buttons()

    def _sync_buttons(self) -> None:
        has = bool(self._current_name)
        ex  = has and self._path(self._current_name).exists()
        self._save_btn.setEnabled(has)
        self._save_as_btn.setEnabled(has)
        self._delete_btn.setEnabled(ex)

    def _write(self, name: str) -> None:
        self._path(name).write_text(json.dumps(self._build_data(), indent=2))

    def _refresh_combo(self, keep: str = "") -> None:
        self._combo.blockSignals(True); self._combo.clear()
        self._combo.addItems(sorted(p.stem for p in self._dir.glob("*.json")))
        self._combo.setCurrentText(keep) if keep else self._combo.clearEditText()
        self._combo.blockSignals(False)

    def _on_activated(self, _: int) -> None:
        self._resolve_name(self._combo.currentText())

    def _on_return_pressed(self) -> None:
        self._resolve_name(self._combo.currentText())

    def _on_save(self) -> None:
        if not self._current_name:
            return
        self._write(self._current_name)
        self._refresh_combo(keep=self._current_name); self._sync_buttons()

    def _on_save_as(self) -> None:
        name, ok = QInputDialog.getText(self, "Save As", "Template name:", text=self._current_name)
        name = name.strip()
        if not ok or not name:
            return
        self._current_name = name; self._write(name)
        self._refresh_combo(keep=name); self._sync_buttons()

    def _on_delete(self) -> None:
        name = self._current_name
        if not name or not self._path(name).exists():
            return
        if QMessageBox.question(self, "Delete", f"Delete '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        ) != QMessageBox.StandardButton.Yes:
            return
        self._path(name).unlink(); self._current_name = ""
        self._refresh_combo(); self._sync_buttons(); self._emit()
