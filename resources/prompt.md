Build a clean, modular, native Python desktop application for an ROV Simulator and Ground Control Station using PySide6 and pyqtgraph. Do NOT build a web app or use a browser. Keep the code simple, concise, and easy to explain and modify live during a technical interview.

Project Structure:
- sim_core.py: Pure Python physics, thruster lag, PID controllers, water current, and safety watchdogs (zero GUI imports so it can be unit-tested independently).
- map_widget.py: A native PySide6 QWidget using QPainter and urllib/QNetworkAccessManager to fetch and cache standard 256x256 OpenStreetMap PNG tiles (tile.openstreetmap.org) around a reference lat/lon and draw the ROV position marker, heading arrow, and path trail directly on the canvas.
- main_gui.py: PySide6 main window, pyqtgraph live depth plot, keyboard/joystick handling, and CSV telemetry logger.
- test_sim.py: pytest unit tests verifying safety rules, PID convergence, and heading wrap-around.
- requirements.txt & README.md (include a section disclosing AI tool usage for UI scaffolding).

1. Physics & Control Engine (`sim_core.py` running at dt = 0.02s / 50 Hz):
   - State: armed (bool), mode ("MANUAL", "DEPTH_HOLD", "HEADING_HOLD"), roll (deg), pitch (deg), yaw (0-360 deg), depth (m, >= 0), lat, lon, battery_voltage (starts at 16.8V, decays slowly with time + thrust).
   - Thruster Delay: First-order low-pass filter on commanded [surge, sway, heave, yaw] thrusts: `actual += (dt / (tau + dt)) * (cmd - actual)` with `tau = 0.15s`.
   - Hydrodynamic Drag & Water Current (Bonus): Apply linear + quadratic drag opposing velocity relative to a configurable water current vector `[current_north, current_east]`.
   - PID Controllers:
     * Depth Hold: regulates heave thrust to reach `target_depth`.
     * Heading Hold: regulates yaw thrust to reach `target_heading` using shortest signed angle error `(target - current + 180) % 360 - 180`.
   - Safety Rules (CRITICAL):
     * If `not armed`: immediately set all `cmd` and `actual` thrusts to 0.0 and reset PID integrals.
     * In `MANUAL` mode: if `time.monotonic() - last_manual_input_time > 1.0`, zero all manual thrust commands.

2. Desktop GUI (`main_gui.py`):
   - Responsive layout using two QTimers: a 50 Hz timer for `sim.step(0.02)` and a 20 Hz timer for UI/plot updates.
   - Telemetry readouts: Armed state, Mode, Roll, Pitch, Yaw, Depth, Battery Voltage.
   - Controls: Arm/Disarm button, Mode radio buttons, Target Depth & Target Heading spinboxes, Water Current sliders (bonus), and a "Record CSV Log" toggle (bonus).
   - Manual Input: Keyboard controls (WASD for surge/sway, Up/Down for heave, Left/Right for yaw) + optional pygame/QGamepad joystick polling. Pressing or holding a key updates `last_manual_input_time`; releasing keys or leaving them untouched for >1.0s triggers the watchdog.
   - Live Plots: `pyqtgraph.PlotWidget` showing a scrolling 30-second window of Actual Depth vs Target Depth, alongside the native `map_widget.py` OpenStreetMap view.