"""
main_gui.py — ROV Ground Control Station main window.
PyQt6 + pyqtgraph. Run directly: python3 main_gui.py
"""

import csv
import os
import sys
import time
from datetime import datetime

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt6")

import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QKeyEvent
from PyQt6.QtWidgets import (
    QApplication, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
    QPushButton, QRadioButton, QSlider, QSpinBox, QDoubleSpinBox,
    QSplitter, QVBoxLayout, QWidget, QButtonGroup, QCheckBox,
)

from map_widget import MapWidget
from sim_core import ROVSimulator

# ---------------------------------------------------------------------------
# pyqtgraph global config
# ---------------------------------------------------------------------------
pg.setConfigOption("background", "#14171c")
pg.setConfigOption("foreground", "#cbd5e1")

# ---------------------------------------------------------------------------
# Scaled Dark Stylesheet
# ---------------------------------------------------------------------------
DARK_STYLE = """
QMainWindow, QWidget {
    background-color: #0f1115;
    color: #f1f5f9;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    font-size: 14px;
}
QGroupBox {
    border: 1.5px solid #2d3748;
    border-radius: 8px;
    margin-top: 12px;
    padding-top: 14px;
    font-weight: 700;
    font-size: 14px;
    color: #94a3b8;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}
QPushButton {
    background: #1e2430;
    border: 1px solid #3b4556;
    border-radius: 6px;
    padding: 9px 16px;
    color: #f8fafc;
    font-size: 14px;
    font-weight: 600;
}
QPushButton:hover {
    background: #283142;
    border-color: #4a5568;
}
QPushButton:pressed {
    background: #161b24;
}
QRadioButton {
    spacing: 10px;
    font-size: 14px;
    color: #e2e8f0;
}
QRadioButton::indicator {
    width: 17px;
    height: 17px;
}
QSlider::groove:horizontal {
    height: 7px;
    background: #2d3748;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    width: 18px;
    height: 18px;
    background: #38bdf8;
    border-radius: 9px;
    margin: -6px 0;
}
QSpinBox, QDoubleSpinBox {
    background: #1a202c;
    border: 1.5px solid #334155;
    border-radius: 6px;
    padding: 6px 10px;
    color: #f8fafc;
    font-size: 14px;
    font-weight: 600;
}
QCheckBox {
    spacing: 10px;
    font-size: 14px;
    color: #e2e8f0;
}
QCheckBox::indicator {
    width: 17px;
    height: 17px;
}
"""


def _val_label() -> QLabel:
    lb = QLabel("—")
    lb.setStyleSheet("color: #38bdf8; font-family: monospace; font-size: 15px; font-weight: bold;")
    return lb


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROV Ground Control Station")
        self.setMinimumSize(1280, 800)

        self.sim = ROVSimulator()
        self._last_state: dict = {}
        self._held_keys: set[int] = set()

        # CSV logging
        self._csv_file = None
        self._csv_writer = None

        # Depth plot buffers (rolling 30 s)
        self._plot_time: list[float] = []
        self._plot_depth_actual: list[float] = []
        self._plot_depth_target: list[float] = []
        self._t0 = time.monotonic()

        self._build_ui()
        self._setup_timers()

    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        # Left panel (Enlarged dashboard)
        left = QWidget()
        left.setFixedWidth(380)
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(10)
        lv.addWidget(self._build_telemetry_group())
        lv.addWidget(self._build_arm_mode_group())
        lv.addWidget(self._build_targets_group())
        lv.addWidget(self._build_current_group())
        lv.addWidget(self._build_log_group())
        lv.addWidget(self._build_keys_group())
        lv.addStretch()

        # Right panel (Map + Depth Plot)
        self.map_widget = MapWidget()
        self._depth_plot = self._build_depth_plot()

        right_split = QSplitter(Qt.Orientation.Vertical)
        right_split.addWidget(self.map_widget)
        right_split.addWidget(self._depth_plot)
        right_split.setSizes([480, 280])

        main_split = QSplitter(Qt.Orientation.Horizontal)
        main_split.addWidget(left)
        main_split.addWidget(right_split)
        main_split.setSizes([380, 960])
        root.addWidget(main_split)

    # --- Telemetry ------------------------------------------------------
    def _build_telemetry_group(self) -> QGroupBox:
        grp = QGroupBox("Live Telemetry")
        g = QVBoxLayout(grp)
        g.setSpacing(5)

        def row(name):
            h = QHBoxLayout()
            lbl_title = QLabel(name)
            lbl_title.setStyleSheet("color: #94a3b8; font-size: 14px; font-weight: 500;")
            h.addWidget(lbl_title)
            lbl = _val_label()
            h.addWidget(lbl, 0, Qt.AlignmentFlag.AlignRight)
            g.addLayout(h)
            return lbl

        self.lbl_armed   = row("State")
        self.lbl_mode    = row("Mode")
        self.lbl_roll    = row("Roll (°)")
        self.lbl_pitch   = row("Pitch (°)")
        self.lbl_yaw     = row("Yaw (°)")
        self.lbl_depth   = row("Depth (m)")
        self.lbl_battery = row("Battery (V)")
        self.lbl_lat     = row("Latitude")
        self.lbl_lon     = row("Longitude")
        return grp

    # --- Arm & Mode -----------------------------------------------------
    def _build_arm_mode_group(self) -> QGroupBox:
        grp = QGroupBox("Arming & Autopilot Mode")
        g = QVBoxLayout(grp)
        g.setSpacing(8)

        self.arm_btn = QPushButton("DISARMED — Click to Arm")
        self.arm_btn.setStyleSheet(
            "QPushButton { background:#7f1d1d; border:1.5px solid #ef4444; color:#ffffff; "
            "border-radius:6px; padding:11px; font-weight:bold; font-size:14px; }"
            "QPushButton:hover { background:#991b1b; }"
        )
        self.arm_btn.clicked.connect(self._toggle_arm)
        g.addWidget(self.arm_btn)

        self._mode_group = QButtonGroup(self)
        for label, mode in [("Manual (WASD + Watchdog)", "MANUAL"),
                             ("Depth Hold (PID Autopilot)", "DEPTH_HOLD"),
                             ("Heading Hold (PID Autopilot)", "HEADING_HOLD")]:
            rb = QRadioButton(label)
            if mode == "MANUAL":
                rb.setChecked(True)
            rb.toggled.connect(lambda chk, m=mode: chk and self.sim.set_mode(m))
            self._mode_group.addButton(rb)
            g.addWidget(rb)
        return grp

    # --- Autopilot Targets ----------------------------------------------
    def _build_targets_group(self) -> QGroupBox:
        grp = QGroupBox("Autopilot Target Setpoints")
        g = QVBoxLayout(grp)
        g.setSpacing(8)

        h1 = QHBoxLayout()
        lbl1 = QLabel("Target Depth (m)")
        lbl1.setStyleSheet("color: #cbd5e1; font-weight: 500;")
        h1.addWidget(lbl1)
        self.spin_depth = QDoubleSpinBox()
        self.spin_depth.setRange(0, 300)
        self.spin_depth.setSingleStep(0.5)
        self.spin_depth.valueChanged.connect(self.sim.set_target_depth)
        h1.addWidget(self.spin_depth)
        g.addLayout(h1)

        h2 = QHBoxLayout()
        lbl2 = QLabel("Target Heading (°)")
        lbl2.setStyleSheet("color: #cbd5e1; font-weight: 500;")
        h2.addWidget(lbl2)
        self.spin_heading = QSpinBox()
        self.spin_heading.setRange(0, 359)
        self.spin_heading.setWrapping(True)
        self.spin_heading.valueChanged.connect(self.sim.set_target_heading)
        h2.addWidget(self.spin_heading)
        g.addLayout(h2)
        return grp

    # --- Water Current --------------------------------------------------
    def _build_current_group(self) -> QGroupBox:
        grp = QGroupBox("Simulated Water Current (m/s)")
        g = QVBoxLayout(grp)
        g.setSpacing(6)

        def slider_row(label):
            h = QHBoxLayout()
            lbl = QLabel(label)
            lbl.setStyleSheet("color: #cbd5e1; font-weight: 500;")
            h.addWidget(lbl)
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(-20, 20)
            sl.setValue(0)
            lbl_v = QLabel("0.0")
            lbl_v.setFixedWidth(42)
            lbl_v.setStyleSheet("color:#38bdf8; font-family:monospace; font-weight:bold; font-size:14px;")
            sl.valueChanged.connect(
                lambda v, lv=lbl_v: (
                    lv.setText(f"{v/20:.1f}"),
                    self.sim.set_water_current(
                        self.sl_n.value() / 20.0,
                        self.sl_e.value() / 20.0,
                    )
                )
            )
            h.addWidget(sl)
            h.addWidget(lbl_v)
            g.addLayout(h)
            return sl

        self.sl_n = slider_row("North Current")
        self.sl_e = slider_row("East Current")
        return grp

    # --- CSV Log --------------------------------------------------------
    def _build_log_group(self) -> QGroupBox:
        grp = QGroupBox("Telemetry CSV Logger")
        h = QHBoxLayout(grp)
        self.chk_log = QCheckBox("Record CSV")
        self.chk_log.toggled.connect(self._toggle_csv)
        self.lbl_log = QLabel("")
        self.lbl_log.setStyleSheet("color:#38bdf8; font-size:12px; font-family:monospace;")
        h.addWidget(self.chk_log)
        h.addWidget(self.lbl_log)
        return grp

    # --- Key reference --------------------------------------------------
    def _build_keys_group(self) -> QGroupBox:
        grp = QGroupBox("Manual Controls (Watchdog: 1.0s)")
        g = QVBoxLayout(grp)
        g.setSpacing(4)
        for k, v in [("W / S", "Surge (Forward / Reverse)"),
                     ("A / D", "Sway (Port / Starboard)"),
                     ("↑ / ↓", "Heave (Ascend / Descend)"),
                     ("← / →", "Yaw (Turn Left / Right)"),
                     ("Space", "Emergency Arm / Disarm")]:
            h = QHBoxLayout()
            kb = QLabel(f"[{k}]")
            kb.setStyleSheet("color:#38bdf8; font-family:monospace; font-weight:bold; font-size:13px;")
            desc = QLabel(v)
            desc.setStyleSheet("color:#94a3b8; font-size:13px;")
            h.addWidget(kb)
            h.addWidget(desc)
            g.addLayout(h)
        return grp

    # --- Depth plot -----------------------------------------------------
    def _build_depth_plot(self) -> pg.PlotWidget:
        pw = pg.PlotWidget(title="<span style='font-size: 15px; font-weight: bold;'>Live Depth vs Time (Rolling 30s Window)</span>")
        pw.setLabel("left", "<span style='font-size: 13px; font-weight: 500;'>Depth</span>", units="m")
        pw.setLabel("bottom", "<span style='font-size: 13px; font-weight: 500;'>Elapsed Time</span>", units="s")
        pw.invertY(True)
        legend = pw.addLegend(offset=(12, 12))
        legend.setScale(1.1)
        pw.showGrid(x=True, y=True, alpha=0.3)
        self._curve_actual = pw.plot(pen=pg.mkPen("#38bdf8", width=2.8), name="Actual Depth")
        self._curve_target = pw.plot(
            pen=pg.mkPen("#fb923c", width=2.2, style=Qt.PenStyle.DashLine), name="Target Depth"
        )
        return pw

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------
    def _setup_timers(self):
        self._sim_timer = QTimer(self)
        self._sim_timer.timeout.connect(self._on_sim_tick)
        self._sim_timer.start(20)   # 50 Hz physics loop

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._on_ui_update)
        self._ui_timer.start(50)    # 20 Hz UI refresh loop

    def _on_sim_tick(self):
        if self.sim.armed and self.sim.mode == "MANUAL":
            self._apply_keyboard_thrust()
        self._last_state = self.sim.step(0.02)

    def _on_ui_update(self):
        if not self._last_state:
            return
        s = self._last_state
        self._refresh_telemetry(s)
        self._refresh_plot(s)
        self.map_widget.update_position(s["lat"], s["lon"], s["yaw"])
        if self._csv_writer:
            self._write_row(s)

    # ------------------------------------------------------------------
    # Telemetry labels
    # ------------------------------------------------------------------
    def _refresh_telemetry(self, s: dict):
        if s["armed"]:
            self.lbl_armed.setText("ARMED")
            self.lbl_armed.setStyleSheet("color: #22c55e; font-weight: bold; font-family: monospace; font-size: 15px;")
        else:
            self.lbl_armed.setText("DISARMED")
            self.lbl_armed.setStyleSheet("color: #ef4444; font-weight: bold; font-family: monospace; font-size: 15px;")

        self.lbl_mode.setText(s["mode"])
        self.lbl_roll.setText(f"{s['roll']:+.1f}")
        self.lbl_pitch.setText(f"{s['pitch']:+.1f}")
        self.lbl_yaw.setText(f"{s['yaw']:05.1f}")
        self.lbl_depth.setText(f"{s['depth']:.2f}")

        batt = s["battery_voltage"]
        self.lbl_battery.setText(f"{batt:.2f} V")
        color = "#ef4444" if batt < 14.0 else "#38bdf8"
        self.lbl_battery.setStyleSheet(f"color:{color}; font-family:monospace; font-size:15px; font-weight:bold;")

        self.lbl_lat.setText(f"{s['lat']:.5f}")
        self.lbl_lon.setText(f"{s['lon']:.5f}")

    # ------------------------------------------------------------------
    # Depth plot
    # ------------------------------------------------------------------
    def _refresh_plot(self, s: dict):
        now = time.monotonic() - self._t0
        self._plot_time.append(now)
        self._plot_depth_actual.append(s["depth"])
        self._plot_depth_target.append(s["target_depth"])

        cutoff = now - 30.0
        while self._plot_time and self._plot_time[0] < cutoff:
            self._plot_time.pop(0)
            self._plot_depth_actual.pop(0)
            self._plot_depth_target.pop(0)

        self._curve_actual.setData(self._plot_time, self._plot_depth_actual)
        self._curve_target.setData(self._plot_time, self._plot_depth_target)

    # ------------------------------------------------------------------
    # Keyboard handling
    # ------------------------------------------------------------------
    def keyPressEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self._held_keys.add(event.key())
            self.sim.last_manual_input_time = time.monotonic()
        if event.key() == Qt.Key.Key_Space:
            self._toggle_arm()
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self._held_keys.discard(event.key())
        super().keyReleaseEvent(event)

    def _apply_keyboard_thrust(self):
        K = self._held_keys
        surge = (1.0 if Qt.Key.Key_W     in K else 0.0) - (1.0 if Qt.Key.Key_S    in K else 0.0)
        sway  = (1.0 if Qt.Key.Key_D     in K else 0.0) - (1.0 if Qt.Key.Key_A    in K else 0.0)
        heave = (1.0 if Qt.Key.Key_Down  in K else 0.0) - (1.0 if Qt.Key.Key_Up   in K else 0.0)
        yaw   = (1.0 if Qt.Key.Key_Right in K else 0.0) - (1.0 if Qt.Key.Key_Left in K else 0.0)
        if any([surge, sway, heave, yaw]):
            self.sim.set_manual_thrust(surge, sway, heave, yaw)

    # ------------------------------------------------------------------
    # Arm / Disarm
    # ------------------------------------------------------------------
    def _toggle_arm(self):
        if self.sim.armed:
            self.sim.disarm()
            self.arm_btn.setText("DISARMED — Click to Arm")
            self.arm_btn.setStyleSheet(
                "QPushButton { background:#7f1d1d; border:1.5px solid #ef4444; color:#ffffff; "
                "border-radius:6px; padding:11px; font-weight:bold; font-size:14px; }"
                "QPushButton:hover { background:#991b1b; }"
            )
        else:
            self.sim.arm()
            self.arm_btn.setText("ARMED — Click to Disarm")
            self.arm_btn.setStyleSheet(
                "QPushButton { background:#14532d; border:1.5px solid #22c55e; color:#ffffff; "
                "border-radius:6px; padding:11px; font-weight:bold; font-size:14px; }"
                "QPushButton:hover { background:#166534; }"
            )

    # ------------------------------------------------------------------
    # CSV logging
    # ------------------------------------------------------------------
    def _toggle_csv(self, checked: bool):
        if checked:
            fname = f"telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv"
            self._csv_file = open(fname, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "timestamp", "armed", "mode",
                "roll", "pitch", "yaw", "depth",
                "lat", "lon", "battery_voltage",
            ])
            self.lbl_log.setText(fname)
        else:
            self._csv_writer = None
            if self._csv_file:
                self._csv_file.close()
                self._csv_file = None
            self.lbl_log.setText("")

    def _write_row(self, s: dict):
        self._csv_writer.writerow([
            f"{time.monotonic():.3f}", int(s["armed"]), s["mode"],
            f"{s['roll']:.2f}", f"{s['pitch']:.2f}", f"{s['yaw']:.2f}",
            f"{s['depth']:.3f}", f"{s['lat']:.6f}", f"{s['lon']:.6f}",
            f"{s['battery_voltage']:.3f}",
        ])

    def closeEvent(self, event):
        if self._csv_file:
            self._csv_file.close()
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
