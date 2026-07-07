from __future__ import annotations

import json
import math
from pathlib import Path

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QSpinBox, QSlider, QComboBox, QFrame, QCheckBox, QInputDialog, QMessageBox,
)
from PyQt6.QtCore import Qt, pyqtSignal

from gui.widgets import AssetCombo, ResetSlider, CollapsibleSection, make_accordion

_ASSETS_DIR      = Path(__file__).parents[3] / "assets" / "wheelframes"
# Frame + Body templates (frame config plus an embedded "body" key).
_FB_DIR          = Path(__file__).parents[3] / "assets" / "frame_and_body"
_AXLE_GAP_M      = 0.05       # minimum clear gap between tyre edges (metres)
_WHEELS_PER_AXLE = [1, 2, 4]
_WHEELS_LABELS   = ["×1", "×2", "×4"]
_GROUP_DEFAULTS   = {"front": 15, "middle": 50, "rear": 85}

_CHASSIS_PRESETS = [
    ("Steel",    "#3e4258"),
    ("Alloy",    "#60636e"),
    ("Carbon",   "#28282e"),
    ("Gunmetal", "#4a4a5a"),
    ("Bronze",   "#6a4a2a"),
]


# ── Mass / inertia estimator ──────────────────────────────────────────────────

def _compute_mass_inertia(
    L: float, W: float, R: float, w_t: float,
    axles: list[dict],
    fw: dict,
) -> tuple[float, float, float, float, float, float]:
    """
    Returns (total_kg, inertia_kgm2, frame_kg, tyres_kg, axle_beams_kg, hitch_kg).

    Mass model:
        Frame  — two C-section steel rails (120×8 mm web + 2×60 mm flanges)
                 plus rectangular cross members (80×6 mm) every 0.8 m
        Axles  — axle beam (20 + 8×n_wheels kg) per axle
        Tyres  — tyre + rim assembly, power-law fit (~12 kg at R=0.33, w=0.20)
        Hitch  — fifth-wheel turntable (90 kg) when enabled

    Inertia model (yaw axis, CG at geometric frame centre):
        Frame  — uniform rectangle  I = m(L²+W²)/12
        Axle   — rod at y-offset   I = m(y²+W²/12)
        Wheels — point masses at   I = m·n·(y²+(W/2)²)
        Hitch  — point mass at hitch offset
    """
    RHO_S = 7_850.0   # steel density  [kg/m³]

    # ── Chassis rails ────────────────────────────────────────────────────────
    A_rail  = (0.120 + 2 * 0.060) * 0.008
    m_rails = 2.0 * RHO_S * A_rail * L

    # ── Cross members (80 × 6 mm, one per 0.8 m) ────────────────────────────
    n_xm = max(2, int(L / 0.8) + 1)
    m_xm = RHO_S * (0.080 * 0.006) * W * n_xm

    m_frame = m_rails + m_xm

    # ── Per-wheel mass (tyre + rim) ──────────────────────────────────────────
    m_wheel = 12.0 * (R / 0.33) ** 1.5 * (w_t / 0.20) ** 0.8

    # ── Axle beams + tyre assemblies (kept separate for breakdown) ───────────
    m_axle_beams = 0.0
    m_tyres      = 0.0
    for axle in axles:
        n_w = axle["wheels"]
        m_axle_beams += 20.0 + 8.0 * n_w
        m_tyres      += m_wheel * n_w

    # ── Fifth-wheel hitch plate ───────────────────────────────────────────────
    m_hitch = 90.0 if fw.get("enabled", False) else 0.0

    m_total = m_frame + m_axle_beams + m_tyres + m_hitch

    # ── Yaw moment of inertia ─────────────────────────────────────────────────
    I_z = m_frame * (L ** 2 + W ** 2) / 12.0

    for axle in axles:
        y_k = (axle["position"] - 0.5) * L
        n_w = axle["wheels"]
        m_beam = 20.0 + 8.0 * n_w
        I_z += m_beam * (y_k ** 2 + W ** 2 / 12.0)
        I_z += m_wheel * n_w * (y_k ** 2 + (W / 2.0) ** 2)

    if fw.get("enabled", False):
        y_hitch = (fw.get("hitch_pct", 1.0) - 0.5) * L
        I_z += m_hitch * y_hitch ** 2

    return m_total, max(1.0, I_z), m_frame, m_tyres, m_axle_beams, m_hitch


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

    # ── Fifth wheel / articulating trailer ────────────────────────────────────
    fw_raw = cfg.get("fifth_wheel", {})
    fw_out: dict = {"enabled": False}
    if fw_raw.get("enabled", False):
        t_len   = fw_raw.get("trailer_length_m", 8.0)
        apos_f  = fw_raw.get("axle_position_pct", 75) / 100.0
        asep_m  = fw_raw.get("axle_separation_cm", 5) / 100.0
        n_axle  = fw_raw.get("axle_count", 2)
        w_axle  = fw_raw.get("wheels_per_axle", 4)
        step_m  = tyre_diam + asep_m
        spacing = max(0.005, step_m / t_len) if t_len > 0 else 0.05
        t_axles: list[dict] = []
        for i in range(n_axle):
            off = (i - (n_axle - 1) / 2.0) * spacing
            t_axles.append({"position": max(0.01, min(0.99, apos_f + off))})
        fw_out = {
            "enabled":          True,
            "hitch_pct":        fw_raw.get("hitch_pct", 100) / 100.0,
            "trailer_length_m": t_len,
            "trailer_width_m":  cfg.get("frame_width_m", 1.8),
            "max_angle_deg":    fw_raw.get("max_angle_deg", 45),
            "kingpin_dist_m":   apos_f * t_len,
            "trailer_axles":    t_axles,
            "wheels":           w_axle,
        }

    L   = cfg.get("frame_length_m", 4.0)
    W   = cfg.get("frame_width_m",  1.8)
    R   = cfg.get("tyre_radius_cm", 33) / 100.0
    w_t = cfg.get("tyre_width_cm",  20) / 100.0
    mass_kg, inertia_kgm2, m_frame, m_tyres, m_axles, m_hitch = \
        _compute_mass_inertia(L, W, R, w_t, axles, fw_out)

    return {
        "frame_length_m":    L,
        "frame_width_m":     W,
        "chassis_color":     cfg.get("chassis_color", _CHASSIS_PRESETS[0][1]),
        "tyre_radius_m":     R,
        "tyre_width_m":      w_t,
        "differential":      cfg.get("differential",   "open"),
        "steering_mode":     steer,
        "drive_mode":        drive,
        "axles":             axles,
        "fifth_wheel":       fw_out,
        "mass_kg":           mass_kg,
        "inertia_kgm2":      inertia_kgm2,
        "mass_frame_kg":     m_frame,
        "mass_tyre_kg":      m_tyres,
        "mass_axle_kg":      m_axles,
        "mass_hitch_kg":     m_hitch,
        "max_wheel_angle_deg": cfg.get("max_wheel_angle_deg", 35),
        "max_torque_nm":       cfg.get("max_torque_nm",      3000),
        "max_wheel_rpm":       cfg.get("max_wheel_rpm",       400),
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

def _chk_ss(t: dict) -> str:
    return (
        f"QCheckBox {{ color: {t['label_dim']}; background: transparent; spacing: 6px; }}"
        f"QCheckBox::indicator {{ width: 15px; height: 15px;"
        f"  border: 1px solid {t['input_border']}; border-radius: 3px;"
        f"  background: {t['input_bg']}; }}"
        f"QCheckBox::indicator:checked {{ background: {t['btn_active_bg']};"
        f"  border-color: {t['accent']}; }}"
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
                fmt, t: dict, vbox: QVBoxLayout) -> tuple[ResetSlider, QLabel]:
    rw = QWidget(); rw.setStyleSheet("background: transparent;")
    rv = QVBoxLayout(rw); rv.setContentsMargins(0, 0, 0, 2); rv.setSpacing(2)
    top = QWidget(); top.setStyleSheet("background: transparent;")
    th = QHBoxLayout(top); th.setContentsMargins(0, 0, 0, 0)
    th.addWidget(QLabel(title, styleSheet=_lbl(t))); th.addStretch()
    val_lbl = QLabel(fmt(default), styleSheet=_dim(t)); val_lbl.setFixedWidth(58)
    th.addWidget(val_lbl); rv.addWidget(top)
    sl = ResetSlider(Qt.Orientation.Horizontal)
    sl.setRange(lo, hi); sl.setValue(default)
    sl._reset_value = default
    sl.setStyleSheet(_sl_ss(t))
    sl.valueChanged.connect(lambda v: val_lbl.setText(fmt(v)))
    rv.addWidget(sl); vbox.addWidget(rw)
    return sl, val_lbl


# ── Axle-group row ────────────────────────────────────────────────────────────

class _GroupRow(QWidget):
    """Single compact row: F/M/R  [axles]  [sep cm]  @ [pos%]  [×N combo]"""
    changed = pyqtSignal()

    _SHORT = {"front": "F", "middle": "M", "rear": "R"}

    def __init__(self, group_name: str, t: dict, parent=None):
        super().__init__(parent)
        self._group_name = group_name
        self.setStyleSheet("background: transparent;")
        h = QHBoxLayout(self); h.setContentsMargins(0, 0, 0, 0); h.setSpacing(6)

        name_lbl = QLabel(self._SHORT.get(group_name, group_name[0].upper()))
        name_lbl.setFixedWidth(16)
        name_lbl.setToolTip(group_name.title())
        name_lbl.setStyleSheet(_lbl(t)); h.addWidget(name_lbl)

        self._count = QSpinBox()
        self._count.setRange(0, 6)
        self._count.setValue(0 if group_name == "middle" else 1)
        self._count.setFixedWidth(42); self._count.setStyleSheet(_spin_ss(t))
        self._count.setToolTip("Number of axles in this group")
        self._count.valueChanged.connect(self.changed)
        h.addWidget(self._count)

        self._sep = QSpinBox()
        self._sep.setRange(0, 999); self._sep.setValue(5)
        self._sep.setSuffix("cm"); self._sep.setFixedWidth(58); self._sep.setStyleSheet(_spin_ss(t))
        self._sep.setToolTip("Clear gap between adjacent axle tyre edges (cm)")
        self._sep.valueChanged.connect(self.changed)
        h.addWidget(self._sep)

        self._pos = QSpinBox()
        self._pos.setRange(1, 99); self._pos.setValue(_GROUP_DEFAULTS[group_name])
        self._pos.setSuffix(""); self._pos.setFixedWidth(52); self._pos.setStyleSheet(_spin_ss(t))
        self._pos.setToolTip("Group centre position along frame (% from front)")
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

    def __init__(self, theme: dict, parent=None,
                 body_section=None, body_get=None, body_set=None):
        super().__init__(parent)
        _FB_DIR.mkdir(parents=True, exist_ok=True)
        self._dir = _FB_DIR
        self._current_name = ""
        self._body_get = body_get      # callable -> body cfg dict (or None)
        self._body_set = body_set      # callable(body cfg dict) (or None)
        t = theme

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(14, 14, 14, 12)
        vbox.setSpacing(8)

        # ── Load / Create ─────────────────────────────────────────────────────
        vbox.addWidget(QLabel("Load / Create", styleSheet=_lbl(t)))
        self._combo = AssetCombo(_FB_DIR, t)
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

        # ── Collapsible sub-section helper ────────────────────────────────────
        def _group(title: str):
            w = QWidget(); w.setStyleSheet("background: transparent;")
            lay = QVBoxLayout(w)
            lay.setContentsMargins(12, 8, 12, 8); lay.setSpacing(6)
            sec = CollapsibleSection(title, t, w)
            return sec, lay

        self._sub_sections: list = []

        # ── Frame ─────────────────────────────────────────────────────────────
        self._sec_frame, lf = _group("Frame")
        self._flen_sl, self._flen_lbl = _slider_row(
            "Length", 10, 200, 40, lambda v: f"{v / 10:.1f} m", t, lf)
        self._fwid_sl, self._fwid_lbl = _slider_row(
            "Width",   5,  50, 18, lambda v: f"{v / 10:.1f} m", t, lf)
        self._flen_sl.valueChanged.connect(lambda _: self._emit())
        self._fwid_sl.valueChanged.connect(lambda _: self._emit())

        lf.addWidget(QLabel("Chassis Color", styleSheet=_lbl(t)))
        self._chassis_color: str = _CHASSIS_PRESETS[0][1]
        color_row = QWidget(); color_row.setStyleSheet("background: transparent;")
        ch = QHBoxLayout(color_row); ch.setContentsMargins(0, 0, 0, 0); ch.setSpacing(4)
        self._chassis_btns: list[tuple[QPushButton, str]] = []
        for _cname, _chex in _CHASSIS_PRESETS:
            _cb = QPushButton()
            _cb.setFixedSize(22, 22)
            _cb.setCheckable(True)
            _cb.setToolTip(_cname)
            _cb.setStyleSheet(
                f"QPushButton {{ background: {_chex}; border: 2px solid transparent; border-radius: 3px; }}"
                f"QPushButton:checked {{ border: 2px solid {t['accent']}; }}"
                f"QPushButton:hover:!checked {{ border: 2px solid {t['label_dim']}; }}"
            )
            _cb.setCursor(Qt.CursorShape.PointingHandCursor)
            self._chassis_btns.append((_cb, _chex))
            ch.addWidget(_cb)
        ch.addStretch()
        lf.addWidget(color_row)
        for _cb, _chex in self._chassis_btns:
            _cb.clicked.connect(lambda _, b=_cb, h=_chex: self._pick_chassis_color(b, h))
        self._chassis_btns[0][0].setChecked(True)
        self._sub_sections.append(self._sec_frame)

        # ── Tyre ──────────────────────────────────────────────────────────────
        self._sec_tyre, lty = _group("Tyre")
        self._tyre_r_sl, self._tyre_r_lbl = _slider_row(
            "Radius", 10, 80, 33, lambda v: f"{v} cm", t, lty)
        self._tyre_w_sl, self._tyre_w_lbl = _slider_row(
            "Width",   5, 50, 20, lambda v: f"{v} cm", t, lty)
        self._tyre_r_sl.valueChanged.connect(lambda _: self._emit())
        self._tyre_w_sl.valueChanged.connect(lambda _: self._emit())
        self._sub_sections.append(self._sec_tyre)

        # ── Steering ──────────────────────────────────────────────────────────
        sec_steer, ls = _group("Steering")
        steer_row, self._steer_btns = _radio_group(["Front", "Rear", "Both"], t)
        ls.addWidget(steer_row)
        self._max_angle_sl, _ = _slider_row(
            "Max angle", 5, 60, 35, lambda v: f"{v}°", t, ls)
        for b in self._steer_btns:
            b.toggled.connect(lambda _: self._emit())
        self._max_angle_sl.valueChanged.connect(lambda _: self._emit())
        self._sub_sections.append(sec_steer)

        # ── Drive (mode, limits, differential) ────────────────────────────────
        sec_drive, ld = _group("Drive")
        drive_row, self._drive_btns = _radio_group(["Front", "Rear", "Both"], t)
        self._drive_btns[1].setChecked(True)
        ld.addWidget(drive_row)
        self._max_torque_sl, _ = _slider_row(
            "Max torque", 1, 200, 30, lambda v: f"{v * 100} Nm", t, ld)
        self._max_rpm_sl, _ = _slider_row(
            "Max RPM", 1, 200, 40, lambda v: f"{v * 10} RPM", t, ld)
        for b in self._drive_btns:
            b.toggled.connect(lambda _: self._emit())
        self._max_torque_sl.valueChanged.connect(lambda _: self._emit())
        self._max_rpm_sl.valueChanged.connect(lambda _: self._emit())

        diff_row = QWidget(); diff_row.setStyleSheet("background: transparent;")
        dh = QHBoxLayout(diff_row); dh.setContentsMargins(0, 0, 0, 0); dh.setSpacing(4)
        dh.addWidget(QLabel("Differential", styleSheet=_lbl(t))); dh.addStretch()
        self._diff_open   = QPushButton("Open");   self._diff_open.setCheckable(True)
        self._diff_locked = QPushButton("Locked"); self._diff_locked.setCheckable(True)
        self._diff_open.setChecked(True)
        for b in (self._diff_open, self._diff_locked):
            b.setStyleSheet(_tog_ss(t)); b.setCursor(Qt.CursorShape.PointingHandCursor)
            dh.addWidget(b)
        self._diff_open.clicked.connect(lambda: self._set_diff("open"))
        self._diff_locked.clicked.connect(lambda: self._set_diff("locked"))
        ld.addWidget(diff_row)
        self._sub_sections.append(sec_drive)

        # ── Axle groups ───────────────────────────────────────────────────────
        self._sec_axle, la = _group("Axle Groups")
        _col_hdr = QWidget(); _col_hdr.setStyleSheet("background: transparent;")
        _ch = QHBoxLayout(_col_hdr); _ch.setContentsMargins(0, 0, 0, 0); _ch.setSpacing(6)
        _ch.addSpacing(16)
        for _txt, _w in (("Axles", 42), ("Sep (cm)", 58), ("Pos (%)", 52)):
            _l = QLabel(_txt, styleSheet=_dim(t)); _l.setFixedWidth(_w); _ch.addWidget(_l)
        _ch.addWidget(QLabel("W/axle", styleSheet=_dim(t)))
        _ch.addStretch()
        la.addWidget(_col_hdr)

        self._groups: dict[str, _GroupRow] = {}
        for gname in ("front", "middle", "rear"):
            row = _GroupRow(gname, t)
            row.changed.connect(self._emit)
            la.addWidget(row)
            self._groups[gname] = row
        self._sub_sections.append(self._sec_axle)

        # ── Articulating Trailer ──────────────────────────────────────────────
        sec_trailer, ltr = _group("Articulating Trailer")
        en_row = QWidget(); en_row.setStyleSheet("background: transparent;")
        en_h = QHBoxLayout(en_row); en_h.setContentsMargins(0, 0, 0, 0); en_h.setSpacing(8)
        en_h.addWidget(QLabel("Enable Trailer", styleSheet=_lbl(t))); en_h.addStretch()
        self._fw_toggle = QCheckBox()
        self._fw_toggle.setStyleSheet(_chk_ss(t))
        en_h.addWidget(self._fw_toggle)
        ltr.addWidget(en_row)

        self._fw_panel = QWidget(); self._fw_panel.setStyleSheet("background: transparent;")
        self._fw_panel.setVisible(False)
        fp = QVBoxLayout(self._fw_panel)
        fp.setContentsMargins(0, 4, 0, 0); fp.setSpacing(8)

        self._fw_hitch_sl,   _ = _slider_row("Hitch position",    50, 150, 100,
                                              lambda v: f"{v} %",        t, fp)
        self._fw_len_sl,     _ = _slider_row("Trailer length",    10, 200, 80,
                                              lambda v: f"{v/10:.1f} m", t, fp)
        self._fw_angle_sl,   _ = _slider_row("Max articulation",   5,  90, 45,
                                              lambda v: f"{v}°",         t, fp)

        ax_row = QWidget(); ax_row.setStyleSheet("background: transparent;")
        ax_h = QHBoxLayout(ax_row)
        ax_h.setContentsMargins(0, 0, 0, 0); ax_h.setSpacing(6)
        ax_h.addWidget(QLabel("Axles", styleSheet=_lbl(t)))
        self._fw_axle_count = QSpinBox()
        self._fw_axle_count.setRange(1, 6); self._fw_axle_count.setValue(2)
        self._fw_axle_count.setFixedWidth(46); self._fw_axle_count.setStyleSheet(_spin_ss(t))
        ax_h.addWidget(self._fw_axle_count)
        ax_h.addSpacing(8)
        ax_h.addWidget(QLabel("Wheels / axle", styleSheet=_lbl(t)))
        self._fw_wheels_cb = QComboBox()
        self._fw_wheels_cb.addItems(_WHEELS_LABELS); self._fw_wheels_cb.setCurrentIndex(2)
        self._fw_wheels_cb.setStyleSheet(_cb_ss(t))
        ax_h.addWidget(self._fw_wheels_cb)
        ax_h.addStretch()
        fp.addWidget(ax_row)

        self._fw_axle_pos_sl, _ = _slider_row("Axle group position", 50, 95, 75,
                                               lambda v: f"{v} %",  t, fp)
        self._fw_axle_sep_sl, _ = _slider_row("Axle separation",      0, 200,  5,
                                               lambda v: f"{v} cm", t, fp)

        ltr.addWidget(self._fw_panel)
        self._sub_sections.append(sec_trailer)

        # ── Vehicle Body sub-section (appended from outside) + accordion ──────
        if body_section is not None:
            self._sub_sections.append(body_section)
        for s in self._sub_sections:
            vbox.addWidget(s)
        vbox.addStretch()
        make_accordion(self._sub_sections)
        self._sec_frame.expand()

        self._fw_toggle.toggled.connect(self._fw_panel.setVisible)
        self._fw_toggle.toggled.connect(lambda _: self._emit())
        for _sl in (self._fw_hitch_sl, self._fw_len_sl, self._fw_angle_sl,
                    self._fw_axle_pos_sl, self._fw_axle_sep_sl):
            _sl.valueChanged.connect(lambda _: self._emit())
        self._fw_axle_count.valueChanged.connect(lambda _: self._emit())
        self._fw_wheels_cb.currentIndexChanged.connect(lambda _: self._emit())

        self._sync_buttons()
        self._emit()

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _pick_chassis_color(self, clicked: QPushButton, hex_color: str) -> None:
        self._chassis_color = hex_color
        for b, _ in self._chassis_btns:
            b.blockSignals(True); b.setChecked(b is clicked); b.blockSignals(False)
        self._emit()

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
            "frame_length_m":    self._flen_sl.value() / 10.0,
            "frame_width_m":     self._fwid_sl.value() / 10.0,
            "chassis_color":     self._chassis_color,
            "tyre_radius_cm":    self._tyre_r_sl.value(),
            "tyre_width_cm":     self._tyre_w_sl.value(),
            "steering_mode":     self._active(self._steer_btns, _keys),
            "drive_mode":        self._active(self._drive_btns, _keys),
            "differential":      "locked" if self._diff_locked.isChecked() else "open",
            "max_wheel_angle_deg": self._max_angle_sl.value(),
            "max_torque_nm":       self._max_torque_sl.value() * 100,
            "max_wheel_rpm":       self._max_rpm_sl.value() * 10,
            "groups":            {n: g.get() for n, g in self._groups.items()},
            "fifth_wheel": {
                "enabled":            self._fw_toggle.isChecked(),
                "hitch_pct":          self._fw_hitch_sl.value(),
                "trailer_length_m":   self._fw_len_sl.value() / 10.0,
                "max_angle_deg":      self._fw_angle_sl.value(),
                "axle_count":         self._fw_axle_count.value(),
                "wheels_per_axle":    _WHEELS_PER_AXLE[self._fw_wheels_cb.currentIndex()],
                "axle_position_pct":  self._fw_axle_pos_sl.value(),
                "axle_separation_cm": self._fw_axle_sep_sl.value(),
            },
        }

    def _load_into_ui(self, data: dict) -> None:
        for sl, key, scale in (
            (self._flen_sl, "frame_length_m", 10),
            (self._fwid_sl, "frame_width_m",  10),
        ):
            sl.blockSignals(True)
            sl.setValue(round(data.get(key, sl.value() / 10) * scale))
            sl.blockSignals(False)
        loaded_hex = data.get("chassis_color", _CHASSIS_PRESETS[0][1])
        self._chassis_color = loaded_hex
        for b, hx in self._chassis_btns:
            b.blockSignals(True); b.setChecked(hx == loaded_hex); b.blockSignals(False)
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
        self._max_angle_sl.blockSignals(True)
        self._max_angle_sl.setValue(int(data.get("max_wheel_angle_deg", 35)))
        self._max_angle_sl.blockSignals(False)
        self._max_torque_sl.blockSignals(True)
        self._max_torque_sl.setValue(max(1, min(200, data.get("max_torque_nm", 3000) // 100)))
        self._max_torque_sl.blockSignals(False)
        self._max_rpm_sl.blockSignals(True)
        self._max_rpm_sl.setValue(max(1, min(200, data.get("max_wheel_rpm", 400) // 10)))
        self._max_rpm_sl.blockSignals(False)
        for name, row in self._groups.items():
            row.set(data.get("groups", {}).get(name, {}))
        fw = data.get("fifth_wheel", {})
        self._fw_toggle.blockSignals(True)
        self._fw_toggle.setChecked(fw.get("enabled", False))
        self._fw_panel.setVisible(fw.get("enabled", False))
        self._fw_toggle.blockSignals(False)
        for _sl, _key, _def in (
            (self._fw_hitch_sl,    "hitch_pct",          100),
            (self._fw_angle_sl,    "max_angle_deg",       45),
            (self._fw_axle_pos_sl, "axle_position_pct",   75),
            (self._fw_axle_sep_sl, "axle_separation_cm",   5),
        ):
            _sl.blockSignals(True); _sl.setValue(int(fw.get(_key, _def))); _sl.blockSignals(False)
        self._fw_len_sl.blockSignals(True)
        self._fw_len_sl.setValue(round(fw.get("trailer_length_m", 8.0) * 10))
        self._fw_len_sl.blockSignals(False)
        self._fw_axle_count.blockSignals(True)
        self._fw_axle_count.setValue(fw.get("axle_count", 2))
        self._fw_axle_count.blockSignals(False)
        _w = fw.get("wheels_per_axle", 4)
        self._fw_wheels_cb.blockSignals(True)
        self._fw_wheels_cb.setCurrentIndex(
            _WHEELS_PER_AXLE.index(_w) if _w in _WHEELS_PER_AXLE else 2)
        self._fw_wheels_cb.blockSignals(False)

    def _update_fwid_min(self) -> None:
        """Lower minimum frame width to 0.1 m when any active group uses single wheels."""
        has_single = any(
            g.get()["axle_count"] > 0 and g.get()["wheels_per_axle"] == 1
            for g in self._groups.values()
        )
        new_min = 1 if has_single else 5   # slider units: tenths of a metre
        if self._fwid_sl.minimum() != new_min:
            cur = self._fwid_sl.value()
            self._fwid_sl.setMinimum(new_min)
            if cur < new_min:
                self._fwid_sl.setValue(new_min)

    def _emit(self) -> None:
        self._update_fwid_min()
        data     = self._build_data()
        resolved = resolve_frame(data)
        self._sec_frame.set_subtitle(f"{resolved['mass_frame_kg']:.0f} kg")
        self._sec_tyre.set_subtitle( f"{resolved['mass_tyre_kg']:.0f} kg")
        self._sec_axle.set_subtitle( f"{resolved['mass_axle_kg']:.0f} kg")
        self.frame_changed.emit(data)

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
            data = json.loads(p.read_text())
            self._load_into_ui(data)
            if self._body_set is not None:
                self._body_set(data.get("body", {}))
        self._emit(); self._sync_buttons()

    def _sync_buttons(self) -> None:
        has = bool(self._current_name)
        ex  = has and self._path(self._current_name).exists()
        self._save_btn.setEnabled(has)
        self._save_as_btn.setEnabled(has)
        self._delete_btn.setEnabled(ex)

    def _write(self, name: str) -> None:
        data = self._build_data()
        if self._body_get is not None:
            data["body"] = self._body_get()
        self._path(name).write_text(json.dumps(data, indent=2))

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
