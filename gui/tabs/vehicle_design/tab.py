from __future__ import annotations

from pathlib import Path

from PyQt6.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QLabel, QInputDialog, QMessageBox,
)
from PyQt6.QtCore import pyqtSignal, QTimer, Qt

from gui.widgets import CollapsibleSection, SectionHeader
from gui.tabs.vehicle_design.vehicle_config    import VehicleConfigWidget
from gui.tabs.vehicle_design.direct_control    import DirectControlWidget
from gui.tabs.vehicle_design.viewport          import ViewportWidget
from gui.tabs.vehicle_design.component_designer import ComponentDesignerWidget

_VEHICLES_DIR = Path(__file__).parents[3] / "assets" / "vehicles"


class VehicleDesignTab(QWidget):
    """
    4-column layout:
      Control Panel  (200 px)  — drive controls
      Viewport       (square)  — physics viewport
      Telemetry      (surplus) — live telemetry
      Configuration  (300 px)  — component designer
    """

    vehicle_status = pyqtSignal(str)

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background: {theme['window_bg']}; border: none;")

        # Headless vehicle config controller — hidden, used only for logic
        self._vehicle_config = VehicleConfigWidget(theme, self)
        self._vehicle_config.hide()
        self._vehicle_config.vehicle_changed.connect(self._on_vehicle_changed)

        self._left      = _LeftPanel(theme, self)
        self._vsep_l    = _VSep(theme, self)
        self._viewport  = ViewportWidget(theme, self)
        self._vsep_t    = _VSep(theme, self)
        self._telemetry = _TelemetryPanel(theme, self)
        self._vsep_r    = _VSep(theme, self)
        self._right     = ComponentDesignerWidget(theme, self)

        self._right.wheel_frame_changed.connect(self._viewport.set_wheel_frame)
        self._right.wheel_frame_changed.connect(self._left.direct_ctrl.update_axle_controls)
        self._left.direct_ctrl.torque_changed.connect(self._viewport.set_torque)
        self._left.direct_ctrl.steering_changed.connect(self._viewport.set_steer)
        self._left.direct_ctrl.rpm_changed.connect(self._viewport.set_rpm)
        self._left.direct_ctrl.accelerator_changed.connect(self._viewport.set_accelerator)
        self._left.direct_ctrl.mode_changed.connect(self._viewport.set_control_mode)
        self._viewport.engine_torque_changed.connect(
            self._left.direct_ctrl.update_engine_torque)
        self._viewport.state_updated.connect(self._telemetry.update_state)
        self._right.component_visibility_changed.connect(
            self._viewport.set_component_visibility)
        self._right.chassis_floor_changed.connect(self._viewport.set_chassis_floor)
        self._right.engine_cfg_changed.connect(self._viewport.set_engine_cfg)
        self._right.transmission_cfg_changed.connect(self._viewport.set_transmission_cfg)
        self._right.transmission_cfg_changed.connect(
            self._left.direct_ctrl.update_gear_controls)
        self._left.direct_ctrl.clutch_changed.connect(self._viewport.set_clutch)
        self._left.direct_ctrl.gear_changed.connect(self._viewport.set_gear)
        self._right.brakes_cfg_changed.connect(self._viewport.set_brakes_cfg)
        self._left.direct_ctrl.brake_changed.connect(self._viewport.set_brake)
        self._left.direct_ctrl.range_changed.connect(self._viewport.set_drive_range)
        self._left.direct_ctrl.drive_mode_changed.connect(self._viewport.set_drive_mode)
        self._viewport.state_updated.connect(self._left.direct_ctrl.update_drive_state)

        QTimer.singleShot(0, self._right.apply_defaults)

    # ── Vehicle menu slots ────────────────────────────────────────────────────

    def new_vehicle(self) -> None:
        name, ok = QInputDialog.getText(self, "New Vehicle", "Vehicle name:")
        if ok and name.strip():
            self._vehicle_config._resolve(name.strip())

    def open_vehicle(self) -> None:
        _VEHICLES_DIR.mkdir(parents=True, exist_ok=True)
        names = sorted(p.stem for p in _VEHICLES_DIR.glob("*.json"))
        if not names:
            QMessageBox.information(self, "Open Vehicle", "No saved vehicles found.")
            return
        name, ok = QInputDialog.getItem(
            self, "Open Vehicle", "Select vehicle:", names, 0, False)
        if ok and name:
            self._vehicle_config._resolve(name)

    def save_vehicle(self) -> None:
        self._vehicle_config._on_save()

    def save_vehicle_as(self) -> None:
        self._vehicle_config._on_save_as()

    # ── Layout ────────────────────────────────────────────────────────────────

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reflow()

    def showEvent(self, event):
        super().showEvent(event)
        self._reflow()

    def _reflow(self):
        W, H = self.width(), self.height()
        if W <= 0 or H <= 0:
            return
        LEFT  = 200
        RIGHT = 300
        SEPS  = 3

        sq      = min(H, max(0, W - LEFT - RIGHT - SEPS))
        telem_w = max(0, W - LEFT - RIGHT - sq - SEPS)

        x = 0
        self._left.setGeometry(x, 0, LEFT, H);         x += LEFT
        self._vsep_l.setGeometry(x, 0, 1, H);          x += 1
        self._viewport.setGeometry(x, 0, sq, sq);      x += sq
        self._vsep_t.setGeometry(x, 0, 1, H);          x += 1
        self._telemetry.setGeometry(x, 0, telem_w, H); x += telem_w
        self._vsep_r.setGeometry(x, 0, 1, H);          x += 1
        self._right.setGeometry(x, 0, RIGHT, H)

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_vehicle_changed(self, name: str, data: dict) -> None:
        msg = f"{name}  ·  loaded" if data else f"{name}  ·  new"
        self.vehicle_status.emit(msg)


class _LeftPanel(QWidget):
    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background: {theme['window_bg']}; border: none;")

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)

        vbox.addWidget(SectionHeader("CONTROL PANEL", theme))

        self.direct_ctrl = DirectControlWidget(theme)
        vbox.addWidget(self.direct_ctrl, 1)


class _TelemetryPanel(QWidget):
    """Live telemetry column."""

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        t = theme
        self.setStyleSheet(f"background: {t['window_bg']}; border: none;")

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)

        vbox.addWidget(SectionHeader("TELEMETRY PANEL", t))

        body = QWidget()
        body.setStyleSheet(f"background: {t['window_bg']};")
        bv = QVBoxLayout(body)
        bv.setContentsMargins(12, 16, 12, 12)
        bv.setSpacing(4)

        lbl_ss = (f"color: {t['label_dim']}; font-size: 10px;"
                  f" letter-spacing: 0.8px; background: transparent;")
        self._speed_label = QLabel("SPEED")
        self._speed_label.setStyleSheet(lbl_ss)
        self._speed_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        self._speed_val = QLabel("0.0 km/h")
        self._speed_val.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self._speed_val.setStyleSheet(
            f"color: {t['label_bright']}; font-size: 26px; font-weight: 700;"
            f" letter-spacing: 1px; background: transparent;")

        self._mass_label = QLabel("TOTAL MASS")
        self._mass_label.setStyleSheet(lbl_ss)
        self._mass_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        self._mass_val = QLabel("— kg")
        self._mass_val.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self._mass_val.setStyleSheet(
            f"color: {t['label_bright']}; font-size: 26px; font-weight: 700;"
            f" letter-spacing: 1px; background: transparent;")

        self._gear_label = QLabel("GEAR")
        self._gear_label.setStyleSheet(lbl_ss)
        self._gear_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        self._gear_val = QLabel("N")
        self._gear_val.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self._gear_val.setStyleSheet(
            f"color: {t['label_bright']}; font-size: 26px; font-weight: 700;"
            f" letter-spacing: 1px; background: transparent;")

        bv.addWidget(self._speed_label)
        bv.addWidget(self._speed_val)
        bv.addSpacing(18)
        bv.addWidget(self._gear_label)
        bv.addWidget(self._gear_val)
        bv.addSpacing(18)
        bv.addWidget(self._mass_label)
        bv.addWidget(self._mass_val)
        bv.addStretch(1)
        vbox.addWidget(body, 1)

    def update_state(self, state: dict) -> None:
        kmh = abs(state.get("speed_kmh", 0.0))
        self._speed_val.setText(f"{kmh:.1f} km/h")
        self._mass_val.setText(f"{state.get('mass_kg', 0.0):,.0f} kg")
        self._gear_val.setText(self._format_gear(state))

    @staticmethod
    def _format_gear(state: dict) -> str:
        g = state.get("gear", 0)
        if state.get("trans_type") == "automatic":
            rng = state.get("drive_range", "N")
            if rng == "D" and g >= 1:
                return f"D{g}" + ("*" if state.get("lockup") else "")
            return rng
        return "R" if g < 0 else ("N" if g == 0 else str(g))


class _VSep(QFrame):
    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setFixedWidth(1)
        self.setStyleSheet(f"background: {theme['border']}; border: none;")
