from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel


def _row(label: str, dim_ss: str, val_ss: str, vbox: QVBoxLayout) -> QLabel:
    row = QWidget(); row.setStyleSheet("background: transparent;")
    h = QHBoxLayout(row)
    h.setContentsMargins(0, 0, 0, 0); h.setSpacing(0)
    lbl = QLabel(label, styleSheet=dim_ss); lbl.setFixedWidth(72)
    val = QLabel("—", styleSheet=val_ss)
    h.addWidget(lbl); h.addWidget(val); h.addStretch()
    vbox.addWidget(row)
    return val


class VehicleInfoWidget(QWidget):
    """Live vehicle telemetry: mass, speed, heading."""

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        t = theme
        self.setStyleSheet("background: transparent;")

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(16, 14, 16, 14)
        vbox.setSpacing(4)

        dim = f"color: {t['label_dim']}; font-size: 11px; background: transparent;"
        val = (f"color: {t['label_bright']}; font-size: 12px; "
               f"font-weight: 600; background: transparent;")

        self._mass_lbl    = _row("Mass",    dim, val, vbox)
        self._speed_lbl   = _row("Speed",   dim, val, vbox)
        self._heading_lbl = _row("Heading", dim, val, vbox)
        vbox.addStretch()

    def update_state(self, state: dict) -> None:
        mass      = state.get("mass_kg",     0.0)
        speed_ms  = state.get("speed_ms",    0.0)
        speed_kmh = state.get("speed_kmh",   0.0)
        heading   = state.get("heading_deg", 0.0)

        self._mass_lbl.setText(f"{mass:,.0f} kg")
        direction = "REV" if speed_ms < -0.05 else "FWD"
        self._speed_lbl.setText(f"{abs(speed_kmh):.1f} km/h  [{direction}]")
        self._heading_lbl.setText(f"{heading:.1f}°")
