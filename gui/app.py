import sys

from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QStackedWidget, QLabel, QFrame
from PyQt6.QtCore import Qt

from gui.widgets import MenuBarWidget, TabSelectorWidget, StatusBarWidget
from gui.tabs.vehicle_design.tab import VehicleDesignTab


def _hsep(theme: dict) -> QFrame:
    """1 px full-width horizontal separator line."""
    f = QFrame()
    f.setFixedHeight(1)
    f.setStyleSheet(f"background: {theme['border']}; border: none;")
    return f


class MainWindow(QMainWindow):

    def __init__(self, cfg: dict):
        super().__init__()
        app_cfg    = cfg["window"]
        layout_cfg = cfg["layout"]
        theme      = cfg["theme"]
        tabs       = cfg["tabs"]

        self.setWindowTitle(app_cfg["title"])
        self.setMinimumSize(900, 600)
        self.setStyleSheet(f"background: {theme['window_bg']}; border: none;")

        # Hide native menu / status bars — we use custom widgets for full layout control
        self.menuBar().setVisible(False)
        self.statusBar().setVisible(False)

        # ── Root widget (fills entire window) ──────────────────────────────────
        root = QWidget()
        root.setStyleSheet("border: none;")
        self.setCentralWidget(root)

        vbox = QVBoxLayout(root)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)

        # ── Row 1 · Menu Bar (30 px) ───────────────────────────────────────────
        self._menu_bar = MenuBarWidget(theme, layout_cfg["menu_bar_height"])
        vbox.addWidget(self._menu_bar)
        vbox.addWidget(_hsep(theme))          # full-width 1 px separator

        # ── Row 2 · Tab Selector (40 px) ──────────────────────────────────────
        self._tab_bar = TabSelectorWidget(tabs, theme, layout_cfg["tab_selector_height"])
        self._tab_bar.tab_changed.connect(self._on_tab_changed)
        self._tab_bar.reload_clicked.connect(self._reload)
        vbox.addWidget(self._tab_bar)

        # ── Row 4 · Status Bar — created first so it can be connected below ──────
        self._status_bar = StatusBarWidget(theme, layout_cfg["status_bar_height"])

        # ── Row 3 · Tab Content (expands to fill remaining space) ────────────
        self._stack = QStackedWidget()
        self._stack.setStyleSheet(f"background: {theme['content_bg']}; border: none;")
        self._vd_tab: VehicleDesignTab | None = None
        for tab in tabs:
            if tab["id"] == "vehicle_design":
                page = VehicleDesignTab(theme)
                page.vehicle_status.connect(self._status_bar.set_message)
                self._vd_tab = page
            else:
                page = self._make_placeholder(tab["label"], theme)
            page.setProperty("tab_label", tab["label"])
            self._stack.addWidget(page)
        vbox.addWidget(self._stack, 1)

        if self._vd_tab:
            self._menu_bar.vehicle_new_triggered.connect(self._vd_tab.new_vehicle)
            self._menu_bar.vehicle_open_triggered.connect(self._vd_tab.open_vehicle)
            self._menu_bar.vehicle_save_triggered.connect(self._vd_tab.save_vehicle)
            self._menu_bar.vehicle_save_as_triggered.connect(self._vd_tab.save_vehicle_as)

        vbox.addWidget(_hsep(theme))          # full-width 1 px separator
        vbox.addWidget(self._status_bar)

    # ── Slots ──────────────────────────────────────────────────────────────────

    def _on_tab_changed(self, idx: int) -> None:
        self._stack.setCurrentIndex(idx)
        tab_label = self._stack.currentWidget().property("tab_label") or ""
        self._status_bar.set_message(tab_label)

    def _reload(self) -> None:
        """Restart the process in place."""
        import os
        os.execv(sys.executable, [sys.executable] + sys.argv)

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _make_placeholder(label: str, theme: dict) -> QWidget:
        page = QWidget()
        page.setProperty("tab_label", label)
        page.setStyleSheet(f"background: {theme['content_bg']};")

        text = QLabel(label)
        text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        text.setStyleSheet(
            f"color: {theme['border']};"
            f"font-size: 22px;"
            f"font-family: 'Helvetica Neue', Arial;"
            f"font-weight: 300;"
            f"letter-spacing: 2px;"
            f"background: transparent;"
        )

        from PyQt6.QtWidgets import QVBoxLayout
        lay = QVBoxLayout(page)
        lay.addStretch()
        lay.addWidget(text)
        lay.addStretch()

        return page
