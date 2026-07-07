from __future__ import annotations

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QFrame, QPushButton,
    QSizePolicy,
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer

from gui.widgets import ResetSlider


# ── Snap-to-zero slider (steering) ────────────────────────────────────────────

class _SnapZeroSlider(ResetSlider):
    """Slider that returns to 0 on release and on double-click."""

    def mouseReleaseEvent(self, event):
        self.setValue(self._reset_value)
        super().mouseReleaseEvent(event)


def _sep(t: dict) -> QFrame:
    f = QFrame()
    f.setFixedHeight(1)
    f.setStyleSheet(f"background: {t['border']}; border: none;")
    return f


# ── Widget ─────────────────────────────────────────────────────────────────────

class DirectControlWidget(QWidget):
    """
    Direct vehicle control panel.

    Two mutually exclusive drive modes (toggled by the Direct / Engine buttons):
      Direct  — torque slider is user-controlled; accelerator is inactive.
      Engine  — accelerator slider drives the engine model; torque slider is
                read-only and mirrors the engine's computed output.

    Signals
    -------
    steering_changed   : float  — steer angle in degrees (−40 … +40)
    torque_changed     : float  — direct torque (Nm), emitted only in Direct mode
    rpm_changed        : float  — direct RPM override
    accelerator_changed: float  — throttle α ∈ [0, 1], emitted only in Engine mode
    mode_changed       : str    — "direct" or "engine" whenever the toggle flips
    """

    steering_changed    = pyqtSignal(float)
    torque_changed      = pyqtSignal(float)
    rpm_changed         = pyqtSignal(float)
    accelerator_changed = pyqtSignal(float)
    mode_changed        = pyqtSignal(str)
    clutch_changed      = pyqtSignal(float)   # engagement e ∈ [0,1]; 1=fully locked
    gear_changed        = pyqtSignal(int)     # -1=R, 0=N, 1..N=forward
    brake_changed       = pyqtSignal(float)   # brake pedal b ∈ [0,1]
    range_changed       = pyqtSignal(str)     # automatic selector: P|R|N|D
    drive_mode_changed  = pyqtSignal(str)     # automatic style: ECO|CITY|SPORT|…

    _MODE_DIRECT = "direct"
    _MODE_ENGINE = "engine"

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background: {theme['window_bg']}; border: none;")
        self._t    = theme
        self._mode = self._MODE_DIRECT

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        body = QWidget()
        body.setStyleSheet(f"background: {theme['window_bg']};")
        self._bv = QVBoxLayout(body)
        self._bv.setContentsMargins(10, 10, 10, 10)
        self._bv.setSpacing(0)
        outer.addWidget(body, 1)

        # ── Mode toggle (always visible) ──────────────────────────────────────
        self._mode_row = self._build_mode_toggle(theme)
        self._bv.addWidget(self._mode_row)
        self._bv.addWidget(_sep(theme))
        self._bv.addSpacing(10)

        # ── Dynamic section (rebuilt on every axle config change) ─────────────
        self._dyn = QWidget()
        self._dyn.setStyleSheet("background: transparent;")
        self._dv = QVBoxLayout(self._dyn)
        self._dv.setContentsMargins(0, 0, 0, 0)
        self._dv.setSpacing(6)
        self._bv.addWidget(self._dyn)
        self._bv.addStretch()

        self._steer_slider:   _SnapZeroSlider | None = None
        self._torque_slider:  ResetSlider     | None = None
        self._rpm_slider:     ResetSlider     | None = None
        self._accel_slider:   ResetSlider     | None = None
        self._clutch_slider:  ResetSlider     | None = None
        self._torque_val_lbl: QLabel          | None = None

        # Transmission state
        self._current_gear:    int  = 0      # default to Neutral
        self._n_forward_gears: int  = 5
        self._gear_btns:       list[QPushButton] = []
        self._trans_type:      str  = "manual"   # "manual" | "automatic"
        self._drive_range:     str  = "N"        # automatic selector P|R|N|D
        self._range_btns:      list = []
        self._auto_gear_lbl              = None
        self._last_frame_cfg:  dict | None = None
        self._drive_mode_names: list = ["ECO", "CITY", "SPORT"]
        self._default_drive_mode: str = "CITY"
        self._drive_mode:      str  = "CITY"      # active automatic style
        self._mode_btns:       list = []

        # Shift animation (clutch-disengage → change gear → clutch-engage)
        self._shift_timer       = QTimer(self)
        self._shift_timer.setInterval(18)           # ~18 ms per step
        self._shift_target_gear: int   = 0
        self._shift_phase:       str   = ""         # "disengaging" | "engaging"
        self._shift_step:        int   = 0
        self._SHIFT_STEPS:       int   = 14         # steps per phase ≈ 250 ms
        self._shift_timer.timeout.connect(self._shift_tick)

        self._show_hint("Configure the Wheel Frame to enable controls.")

    # ── Mode toggle ───────────────────────────────────────────────────────────

    def _build_mode_toggle(self, t: dict) -> QWidget:
        row = QWidget(); row.setStyleSheet("background: transparent;")
        rh  = QHBoxLayout(row)
        rh.setContentsMargins(0, 0, 0, 8); rh.setSpacing(0)

        active_ss = (
            f"QPushButton {{ background: {t['accent']}; color: #ffffff;"
            f" border: 1px solid {t['accent']}; border-radius: 3px;"
            f" font-size: 11px; font-weight: 700; padding: 4px 10px; }}"
        )
        idle_ss = (
            f"QPushButton {{ background: {t['btn_bg']}; color: {t['btn_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 11px; padding: 4px 10px; }}"
            f"QPushButton:hover {{ background: {t['btn_hover_bg']}; color: {t['btn_hover_text']}; }}"
        )
        self._btn_direct = QPushButton("Direct")
        self._btn_engine = QPushButton("Engine")
        self._btn_direct.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn_engine.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn_direct.setStyleSheet(active_ss)   # Direct is default
        self._btn_engine.setStyleSheet(idle_ss)

        self._active_ss = active_ss
        self._idle_ss   = idle_ss

        self._btn_direct.clicked.connect(lambda: self._set_mode(self._MODE_DIRECT))
        self._btn_engine.clicked.connect(lambda: self._set_mode(self._MODE_ENGINE))

        rh.addWidget(self._btn_direct, 1)
        rh.addSpacing(4)
        rh.addWidget(self._btn_engine, 1)
        return row

    def _set_mode(self, mode: str) -> None:
        if mode == self._mode:
            return
        self._mode = mode
        self._btn_direct.setStyleSheet(
            self._active_ss if mode == self._MODE_DIRECT else self._idle_ss)
        self._btn_engine.setStyleSheet(
            self._active_ss if mode == self._MODE_ENGINE else self._idle_ss)
        self._apply_mode_to_controls()
        self.mode_changed.emit(mode)
        # In Direct mode emit 0 torque if no axles set; engine handles its own 0
        if mode == self._MODE_DIRECT and self._torque_slider:
            self.torque_changed.emit(float(self._torque_slider.value()))

    def _apply_mode_to_controls(self) -> None:
        """Grey/activate sliders depending on current mode."""
        is_engine = (self._mode == self._MODE_ENGINE)
        if self._torque_slider:
            self._torque_slider.setEnabled(not is_engine)
        if self._accel_slider:
            self._accel_slider.setEnabled(is_engine)

    # ── Public API ────────────────────────────────────────────────────────────

    def update_axle_controls(self, cfg: dict) -> None:
        """Rebuild sliders whenever the wheel-frame config changes."""
        from gui.tabs.vehicle_design.wheel_frame_section import resolve_frame
        self._last_frame_cfg = cfg
        self._clear()

        res    = resolve_frame(cfg)
        axles  = res["axles"]
        steer  = [a for a in axles if a.get("steerable")]
        driven = [a for a in axles if a.get("drivable")]
        n_sw   = sum(a["wheels"] for a in steer)
        n_dw   = sum(a["wheels"] for a in driven)
        diff   = res.get("differential", "open")

        if not steer and not driven:
            self._show_hint("Configure axle groups in the Wheel Frame section.")
            self.steering_changed.emit(0.0)
            self.torque_changed.emit(0.0)
            return

        t      = self._t
        sec_ss = (f"color: {t['label_bright']}; font-size: 12px;"
                  f"font-weight: 600; background: transparent;")
        dim_ss = f"color: {t['label_dim']}; font-size: 11px; background: transparent;"
        sl_ss  = self._sl_ss()

        max_angle  = int(res.get("max_wheel_angle_deg", 40))
        max_torque = int(res.get("max_torque_nm",      2000))
        max_rpm    = int(res.get("max_wheel_rpm",       400))

        _sname = {"front": "Front", "rear": "Rear", "both": "Front + Rear"}
        steer_lbl = _sname.get(res.get("steering_mode", ""), "")
        drive_lbl = _sname.get(res.get("drive_mode",    ""), "")

        # ── Steering ─────────────────────────────────────────────────────────
        if steer:
            self._dv.addWidget(QLabel("STEERING", styleSheet=sec_ss))
            self._dv.addWidget(QLabel(
                f"{steer_lbl}  ·  {len(steer)} axles  ·  {n_sw} wheels"
                f"  ·  ±{max_angle}°",
                styleSheet=dim_ss))
            sl, row = self._slider_row(_SnapZeroSlider,
                                       -max_angle, max_angle, 0, sl_ss,
                                       lambda v: f"{v:+d}°", t)
            self._steer_slider = sl
            sl.valueChanged.connect(lambda v: self.steering_changed.emit(float(v)))
            self._dv.addWidget(row)
        else:
            self._dv.addWidget(QLabel("No steerable axles", styleSheet=dim_ss))
            self.steering_changed.emit(0.0)

        if steer and driven:
            self._dv.addWidget(_sep(t))

        # ── Drive section ─────────────────────────────────────────────────────
        if driven:
            per_w = f"  ·  {max_torque // max(1, n_dw)} Nm/wheel" if n_dw else ""
            self._dv.addWidget(QLabel("TORQUE", styleSheet=sec_ss))
            self._dv.addWidget(QLabel(
                f"{drive_lbl}  ·  {len(driven)} axles  ·  {n_dw} wheels"
                f"  ·  {diff} diff{per_w}",
                styleSheet=dim_ss))

            # Torque slider (Direct mode)
            sl, row = self._slider_row(ResetSlider,
                                       -max_torque, max_torque, 0, sl_ss,
                                       lambda v: f"{v:+d} Nm", t)
            self._torque_slider  = sl
            # The value label is the last widget in the horizontal layout
            self._torque_val_lbl = row.layout().itemAt(1).widget()
            sl.valueChanged.connect(self._on_torque_slider)
            self._dv.addWidget(row)

            self._dv.addWidget(_sep(t))

            # Accelerator slider (Engine mode)
            self._dv.addWidget(QLabel("ACCELERATOR", styleSheet=sec_ss))
            self._dv.addWidget(QLabel("Engine mode only  ·  0 – 100 %",
                                       styleSheet=dim_ss))
            sl2, row2 = self._slider_row(ResetSlider,
                                          0, 100, 0, sl_ss,
                                          lambda v: f"{v} %", t)
            self._accel_slider = sl2
            sl2.valueChanged.connect(self._on_accel_slider)
            self._dv.addWidget(row2)

            self._dv.addWidget(_sep(t))

            # Brake pedal (active in both modes)
            self._dv.addWidget(QLabel("BRAKE", styleSheet=sec_ss))
            self._dv.addWidget(QLabel("Friction brake  ·  0 – 100 %",
                                       styleSheet=dim_ss))
            slb, rowb = self._slider_row(ResetSlider,
                                          0, 100, 0, sl_ss,
                                          lambda v: f"{v} %", t)
            self._brake_slider = slb
            slb.valueChanged.connect(lambda v: self.brake_changed.emit(v / 100.0))
            self._dv.addWidget(rowb)

            self._dv.addWidget(_sep(t))

            self._dv.addWidget(QLabel("DIRECT RPM", styleSheet=sec_ss))
            self._dv.addWidget(QLabel(
                f"Overrides torque physics when non-zero  ·  ±{max_rpm} RPM",
                styleSheet=dim_ss))
            sl3, row3 = self._slider_row(ResetSlider,
                                          -max_rpm, max_rpm, 0, sl_ss,
                                          lambda v: f"{v:+d} RPM", t)
            self._rpm_slider = sl3
            sl3.valueChanged.connect(lambda v: self.rpm_changed.emit(float(v)))
            self._dv.addWidget(row3)

        else:
            self._dv.addWidget(QLabel("No drivable axles", styleSheet=dim_ss))
            self.torque_changed.emit(0.0)
            self.rpm_changed.emit(0.0)

        if driven:
            self._dv.addWidget(_sep(t))
            self._build_transmission_section(t, dim_ss, sec_ss, sl_ss)

        self._apply_mode_to_controls()

    def update_gear_controls(self, cfg: dict) -> None:
        """Called when transmission config changes — rebuild gear controls."""
        self._n_forward_gears = int(cfg.get("n_forward_gears", 5))
        names = [m.get("name") for m in cfg.get("drive_modes", []) if m.get("name")]
        self._drive_mode_names = names or ["CITY"]
        self._default_drive_mode = cfg.get("default_drive_mode", self._drive_mode_names[0])
        new_type = cfg.get("trans_type", "manual")
        if new_type != self._trans_type:
            # Manual ↔ automatic swaps the whole control set — rebuild the panel.
            self._trans_type = new_type
            if self._last_frame_cfg is not None:
                self.update_axle_controls(self._last_frame_cfg)
            return
        if self._trans_type == "manual":
            self._rebuild_gear_buttons()
        else:
            self._rebuild_mode_buttons()

    def update_drive_state(self, state: dict) -> None:
        """Live readout of the automatic's current gear / lock-up."""
        if self._auto_gear_lbl is None or state.get("trans_type") != "automatic":
            return
        g = state.get("gear", 0)
        rng = state.get("drive_range", "N")
        if rng == "D" and g >= 1:
            lock = "  ·  lock-up" if state.get("lockup") else ""
            self._auto_gear_lbl.setText(f"Gear {g}{lock}")
        else:
            self._auto_gear_lbl.setText({"P": "Park", "R": "Reverse",
                                         "N": "Neutral"}.get(rng, rng))

    # ── Transmission section builder ──────────────────────────────────────────

    def _build_transmission_section(self, t, dim_ss, sec_ss, sl_ss) -> None:
        btn_ss = (
            f"QPushButton {{ background: {t['btn_bg']}; color: {t['btn_text']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 11px; padding: 3px 0px; }}"
            f"QPushButton:hover {{ background: {t['btn_hover_bg']};"
            f" color: {t['btn_hover_text']}; }}"
            f"QPushButton:checked {{ background: {t['accent']}; color: #fff;"
            f" border-color: {t['accent']}; }}"
        )
        shift_ss = (
            f"QPushButton {{ background: {t['btn_bg']}; color: {t['label_bright']};"
            f" border: 1px solid {t['input_border']}; border-radius: 3px;"
            f" font-size: 12px; font-weight: 700; padding: 4px 0px; }}"
            f"QPushButton:hover {{ background: {t['btn_hover_bg']}; }}"
            f"QPushButton:pressed {{ background: {t['accent']}; color: #fff; }}"
        )

        if self._trans_type == "automatic":
            self._build_automatic_section(t, dim_ss, sec_ss, btn_ss)
            return

        # ── Clutch ────────────────────────────────────────────────────────────
        self._dv.addWidget(QLabel("CLUTCH", styleSheet=sec_ss))
        self._dv.addWidget(QLabel(
            "0 = engaged (pedal up)  ·  100 = disengaged (pedal down)",
            styleSheet=dim_ss))
        sl, row = self._slider_row(ResetSlider, 0, 100, 0, sl_ss,
                                   lambda v: f"{v} %", t)
        self._clutch_slider = sl
        sl.valueChanged.connect(self._on_clutch_slider)
        self._dv.addWidget(row)

        self._dv.addWidget(_sep(t))

        # ── Gear selector ─────────────────────────────────────────────────────
        self._dv.addWidget(QLabel("GEAR", styleSheet=sec_ss))
        self._gear_btn_container = QWidget()
        self._gear_btn_container.setStyleSheet("background: transparent;")
        self._gear_btn_layout = QHBoxLayout(self._gear_btn_container)
        self._gear_btn_layout.setContentsMargins(0, 0, 0, 0)
        self._gear_btn_layout.setSpacing(3)
        self._gear_btn_style = btn_ss
        self._gear_btns = []
        self._rebuild_gear_buttons()
        self._dv.addWidget(self._gear_btn_container)

        # ── Shift Up / Down ───────────────────────────────────────────────────
        shift_row = QWidget(); shift_row.setStyleSheet("background: transparent;")
        sh = QHBoxLayout(shift_row)
        sh.setContentsMargins(0, 4, 0, 0); sh.setSpacing(6)
        btn_dn = QPushButton("Shift Down")
        btn_up = QPushButton("Shift Up")
        for b in (btn_dn, btn_up):
            b.setStyleSheet(shift_ss)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            sh.addWidget(b, 1)
        btn_dn.clicked.connect(self._on_shift_down)
        btn_up.clicked.connect(self._on_shift_up)
        self._dv.addWidget(shift_row)

    # ── Automatic selector ─────────────────────────────────────────────────────

    def _build_automatic_section(self, t, dim_ss, sec_ss, btn_ss) -> None:
        """P / R / N / D selector + live gear readout (no clutch, no manual gears)."""
        self._dv.addWidget(QLabel("DRIVE RANGE", styleSheet=sec_ss))
        self._dv.addWidget(QLabel("Torque converter  ·  auto-shifting",
                                   styleSheet=dim_ss))

        row = QWidget(); row.setStyleSheet("background: transparent;")
        rh = QHBoxLayout(row)
        rh.setContentsMargins(0, 0, 0, 0); rh.setSpacing(3)
        self._range_btns = []
        for label in ("P", "R", "N", "D"):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setChecked(label == self._drive_range)
            btn.setStyleSheet(btn_ss)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setFixedHeight(28)
            btn.clicked.connect(lambda _, r=label: self._on_range_btn(r))
            rh.addWidget(btn)
            self._range_btns.append((label, btn))
        self._dv.addWidget(row)

        self._auto_gear_lbl = QLabel(self._drive_range, styleSheet=(
            f"color: {t['label_bright']}; font-size: 13px; font-weight: 700;"
            f" background: transparent; padding-top: 4px;"))
        self._dv.addWidget(self._auto_gear_lbl)

        # ── Transmission style (ECO / CITY / SPORT / …) ───────────────────────
        self._dv.addWidget(_sep(t))
        self._dv.addWidget(QLabel("TRANSMISSION STYLE", styleSheet=sec_ss))
        self._mode_btn_style = btn_ss
        self._mode_btn_container = QWidget()
        self._mode_btn_container.setStyleSheet("background: transparent;")
        self._mode_btn_layout = QHBoxLayout(self._mode_btn_container)
        self._mode_btn_layout.setContentsMargins(0, 0, 0, 0)
        self._mode_btn_layout.setSpacing(3)
        self._dv.addWidget(self._mode_btn_container)
        # Boot in the configured default, and sync the viewport to it.
        if self._drive_mode not in self._drive_mode_names:
            self._drive_mode = (self._default_drive_mode
                                if self._default_drive_mode in self._drive_mode_names
                                else self._drive_mode_names[0])
        self._rebuild_mode_buttons()
        self.drive_mode_changed.emit(self._drive_mode)

    def _rebuild_mode_buttons(self) -> None:
        if not hasattr(self, "_mode_btn_layout") or self._mode_btn_layout is None:
            return
        while self._mode_btn_layout.count():
            item = self._mode_btn_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._mode_btns = []
        if self._drive_mode not in self._drive_mode_names and self._drive_mode_names:
            self._drive_mode = self._drive_mode_names[0]
        for name in self._drive_mode_names:
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.setChecked(name == self._drive_mode)
            btn.setStyleSheet(self._mode_btn_style)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setFixedHeight(26)
            btn.clicked.connect(lambda _, m=name: self._on_mode_btn(m))
            self._mode_btn_layout.addWidget(btn)
            self._mode_btns.append((name, btn))

    def _on_mode_btn(self, name: str) -> None:
        self._drive_mode = name
        for m, btn in self._mode_btns:
            btn.blockSignals(True)
            btn.setChecked(m == name)
            btn.blockSignals(False)
        self.drive_mode_changed.emit(name)

    def _on_range_btn(self, rng: str) -> None:
        self._drive_range = rng
        for r, btn in self._range_btns:
            btn.blockSignals(True)
            btn.setChecked(r == rng)
            btn.blockSignals(False)
        self.range_changed.emit(rng)

    def _rebuild_gear_buttons(self) -> None:
        if not hasattr(self, "_gear_btn_layout"):
            return
        # Clear old buttons
        while self._gear_btn_layout.count():
            item = self._gear_btn_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._gear_btns = []

        # Build: R, N, 1 … N_forward
        specs = [("R", -1), ("N", 0)] + [(str(i), i)
                 for i in range(1, self._n_forward_gears + 1)]
        for label, gear_idx in specs:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setChecked(gear_idx == self._current_gear)
            btn.setStyleSheet(self._gear_btn_style)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setFixedHeight(26)
            btn.clicked.connect(lambda _, g=gear_idx: self._on_gear_btn(g))
            self._gear_btn_layout.addWidget(btn)
            self._gear_btns.append((gear_idx, btn))

    def _set_active_gear(self, gear: int) -> None:
        self._current_gear = gear
        for g_idx, btn in self._gear_btns:
            btn.blockSignals(True)
            btn.setChecked(g_idx == gear)
            btn.blockSignals(False)
        self.gear_changed.emit(gear)

    # ── Shift animation ───────────────────────────────────────────────────────

    def _on_shift_up(self) -> None:
        max_g = self._n_forward_gears
        if self._current_gear < max_g:
            self._start_shift(self._current_gear + 1)

    def _on_shift_down(self) -> None:
        if self._current_gear > -1:
            self._start_shift(self._current_gear - 1)

    def _start_shift(self, target_gear: int) -> None:
        if self._shift_timer.isActive():
            return   # already shifting
        self._shift_target_gear = target_gear
        self._shift_phase       = "disengaging"
        self._shift_step        = 0
        self._shift_timer.start()

    def _shift_tick(self) -> None:
        if self._clutch_slider is None:
            self._shift_timer.stop(); return

        self._shift_step += 1
        frac = self._shift_step / self._SHIFT_STEPS

        if self._shift_phase == "disengaging":
            val = round(min(100, frac * 100))
            self._clutch_slider.setValue(val)
            if self._shift_step >= self._SHIFT_STEPS:
                # Mid-point: change gear
                self._set_active_gear(self._shift_target_gear)
                self._shift_phase = "engaging"
                self._shift_step  = 0

        elif self._shift_phase == "engaging":
            val = round(max(0, 100 - frac * 100))
            self._clutch_slider.setValue(val)
            if self._shift_step >= self._SHIFT_STEPS:
                self._clutch_slider.setValue(0)
                self._shift_timer.stop()

    # ── Clutch slot ───────────────────────────────────────────────────────────

    def _on_clutch_slider(self, v: int) -> None:
        # slider 0 = pedal up = clutch fully engaged (e=1)
        # slider 100 = pedal down = clutch disengaged (e=0)
        e = 1.0 - v / 100.0
        self.clutch_changed.emit(e)

    def _on_gear_btn(self, gear: int) -> None:
        self._set_active_gear(gear)

    def update_engine_torque(self, torque_nm: float) -> None:
        """Called by viewport in Engine mode to mirror computed torque on the slider."""
        if self._mode != self._MODE_ENGINE or self._torque_slider is None:
            return
        # Clamp to slider range and update display without emitting torque_changed
        clamped = max(self._torque_slider.minimum(),
                      min(self._torque_slider.maximum(), round(torque_nm)))
        self._torque_slider.blockSignals(True)
        self._torque_slider.setValue(clamped)
        self._torque_slider.blockSignals(False)
        # Update the value label directly
        for lbl in self._dyn.findChildren(QLabel):
            if lbl.text().endswith(" Nm") or lbl.text().endswith("+0 Nm"):
                pass   # updated via valueChanged which we blocked — set manually
        # Find and update the label in the torque row
        if self._torque_val_lbl:
            self._torque_val_lbl.setText(f"{clamped:+d} Nm")

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_torque_slider(self, v: int) -> None:
        if self._mode == self._MODE_DIRECT:
            self.torque_changed.emit(float(v))

    def _on_accel_slider(self, v: int) -> None:
        if self._mode == self._MODE_ENGINE:
            self.accelerator_changed.emit(v / 100.0)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _clear(self) -> None:
        self._shift_timer.stop()
        while self._dv.count():
            item = self._dv.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._steer_slider   = None
        self._torque_slider  = None
        self._rpm_slider     = None
        self._accel_slider   = None
        self._brake_slider   = None
        self._clutch_slider  = None
        self._torque_val_lbl = None
        self._gear_btns      = []
        self._range_btns     = []
        self._auto_gear_lbl  = None
        self._mode_btns      = []
        self._mode_btn_layout = None

    def _show_hint(self, text: str) -> None:
        t   = self._t
        lbl = QLabel(text)
        lbl.setWordWrap(True)
        lbl.setStyleSheet(
            f"color: {t['border']}; font-size: 11px; background: transparent;")
        self._dv.addWidget(lbl)

    @staticmethod
    def _slider_row(cls, lo: int, hi: int, default: int,
                    sl_ss: str, fmt, t: dict) -> tuple[ResetSlider, QWidget]:
        """Compact horizontal row: [slider ──────────────] value"""
        container = QWidget(); container.setStyleSheet("background: transparent;")
        ch = QHBoxLayout(container)
        ch.setContentsMargins(0, 0, 0, 0); ch.setSpacing(6)

        sl = cls(Qt.Orientation.Horizontal)
        sl.setRange(lo, hi); sl.setValue(default)
        sl._reset_value = default
        sl.setStyleSheet(sl_ss)
        ch.addWidget(sl, 1)

        val_lbl = QLabel(fmt(default))
        val_lbl.setFixedWidth(58)
        val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        val_lbl.setStyleSheet(
            f"color: {t['label_bright']}; font-size: 11px;"
            f" font-weight: 600; background: transparent;")
        ch.addWidget(val_lbl)

        sl.valueChanged.connect(lambda v: val_lbl.setText(fmt(v)))
        return sl, container

    def _sl_ss(self) -> str:
        t = self._t
        return (
            f"QSlider::groove:horizontal {{ background: {t['slider_groove']};"
            f" height: 3px; border-radius: 2px; }}"
            f"QSlider::sub-page:horizontal {{ background: {t['slider_fill']};"
            f" height: 3px; border-radius: 2px; }}"
            f"QSlider::handle:horizontal {{ background: {t['slider_handle']};"
            f" width: 11px; height: 11px; border-radius: 6px; margin: -4px 0; }}"
        )
