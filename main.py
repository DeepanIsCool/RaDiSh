import json
import sys
from pathlib import Path

from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QIcon

from gui.app import MainWindow

CONFIG_PATH = Path(__file__).parent / "config" / "application.json"
ICON_PATH   = Path(__file__).parent / "gui" / "logo.png"


def load_config(path: Path) -> dict:
    with path.open() as f:
        return json.load(f)


def _set_dock_icon(icon_path: Path) -> None:
    """Set the macOS dock icon via AppKit (requires pyobjc-framework-Cocoa)."""
    try:
        from AppKit import NSApplication, NSImage
        image = NSImage.alloc().initWithContentsOfFile_(str(icon_path.resolve()))
        if image:
            NSApplication.sharedApplication().setApplicationIconImage_(image)
    except Exception:
        pass


def main():
    cfg = load_config(CONFIG_PATH)

    app = QApplication(sys.argv)
    app.setApplicationName(cfg["window"]["title"])

    if ICON_PATH.exists():
        app.setWindowIcon(QIcon(str(ICON_PATH)))

    if sys.platform == "darwin":
        _set_dock_icon(ICON_PATH)

    window = MainWindow(cfg)
    window.showMaximized()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
