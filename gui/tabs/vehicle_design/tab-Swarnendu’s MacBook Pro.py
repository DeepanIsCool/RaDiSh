from PyQt6.QtWidgets import QWidget, QHBoxLayout, QFrame, QVBoxLayout
from PyQt6.QtCore import Qt, pyqtSignal

from gui.widgets import CollapsibleSection
from gui.tabs.vehicle_design.vehicle_config    import VehicleConfigWidget
from gui.tabs.vehicle_design.vehicle_info      import VehicleInfoWidget
from gui.tabs.vehicle_design.direct_control    import DirectControlWidget
from gui.tabs.vehicle_design.viewport          import ViewportWidget
from gui.tabs.vehicle_design.component_designer import ComponentDesignerWidget


class VehicleDesignTab(QWidget):
    """
    1600 × 800 content area split into three columns:
      Left  (400)  — Vehicle Configuration (top) + Direct Control (bottom)
      Center (800) — Viewport
      Right  (400) — Component Designer
    """

    vehicle_status = pyqtSignal(str)   # forwarded to the main status bar

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setFixedSize(1600, 800)
        self.setStyleSheet(f"background: {theme['window_bg']}; border: none;")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # ── Left panel (400 px) ───────────────────────────────────────────────
        left = _LeftPanel(theme)
        left.vehicle_config.vehicle_changed.connect(self._on_vehicle_changed)
        layout.addWidget(left)
        layout.addWidget(_VSep(theme))

        # ── Center viewport (800 px) ──────────────────────────────────────────
        viewport = ViewportWidget(theme)
        layout.addWidget(viewport)
        layout.addWidget(_VSep(theme))

        # ── Right panel (400 px) ──────────────────────────────────────────────
        comp = ComponentDesignerWidget(theme)
        comp.wheel_frame_changed.connect(viewport.set_wheel_frame)
        comp.wheel_frame_changed.connect(left.direct_ctrl.update_axle_controls)
        left.direct_ctrl.torque_changed.connect(viewport.set_torque)
        left.direct_ctrl.steering_changed.connect(viewport.set_steer)
        layout.addWidget(comp)

    def _on_vehicle_changed(self, name: str, data: dict) -> None:
        if data:
            msg = f"{name}  ·  loaded"
        else:
            msg = f"{name}  ·  new"
        self.vehicle_status.emit(msg)


class _LeftPanel(QWidget):
    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setFixedSize(400, 800)
        self.setStyleSheet(f"background: {theme['window_bg']}; border: none;")

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)

        self.vehicle_config = VehicleConfigWidget(theme)
        vbox.addWidget(self.vehicle_config)

        self.direct_ctrl = DirectControlWidget(theme)
        info_sec = CollapsibleSection("VEHICLE INFORMATION", theme, VehicleInfoWidget(theme))
        ctrl_sec = CollapsibleSection("DIRECT CONTROL",      theme, self.direct_ctrl)

        info_sec.opened.connect(ctrl_sec.collapse)
        ctrl_sec.opened.connect(info_sec.collapse)

        vbox.addWidget(info_sec)
        vbox.addWidget(ctrl_sec)
        vbox.addStretch(1)

        ctrl_sec.expand()


class _VSep(QFrame):
    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setFixedWidth(1)
        self.setFixedHeight(800)
        self.setStyleSheet(f"background: {theme['border']}; border: none;")
