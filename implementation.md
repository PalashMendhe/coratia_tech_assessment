# ROV Simulator & Ground Control Station — Implementation Plan

## Goal

Build a **native Python desktop application** (PySide6 + pyqtgraph) that simulates an underwater ROV (Remotely Operated Vehicle) and provides a Ground Control Station (GCS) UI. Clean, modular, and easy to explain/modify live during a technical interview.

---

## Environment

| Item | Version | Status |
|---|---|---|
| Python | 3.14.4 | ✅ |
| PySide6 | 6.10.2 | ✅ |
| pyqtgraph | 0.14.0 | ✅ |
| numpy | 2.3.5 | ✅ |
| pytest | 9.0.2 | ✅ |

---

## File Structure

```
intern_assessment_cortech/
├── resources/
│   ├── TECHNICAL ASSESSMENT.pdf
│   └── prompt.md
├── sim_core.py          ← Physics engine (no GUI imports)
├── map_widget.py        ← OpenStreetMap tile widget
├── main_gui.py          ← Main window + all GUI logic
├── test_sim.py          ← pytest unit tests
├── requirements.txt
└── README.md
```

---

## Architecture Overview

```
sim_core.py (ROVSimulator, 50 Hz)
    │  state dict
    ▼
main_gui.py (MainWindow, PySide6)
    │  keyboard/joystick cmds          lat/lon path       depth history
    │                          ┌──────────────────┐   ┌──────────────────┐
    └──────────────────────────► map_widget.py     │   │ pyqtgraph Plot   │
                                │ (QPainter + OSM) │   │ (30s scrolling)  │
                                └──────────────────┘   └──────────────────┘
    │
    ▼  (optional)
telemetry.csv
```

---

## Component Details

---

### 1. `sim_core.py` — Physics & Control Engine

**Runs at `dt = 0.02 s` (50 Hz)**

#### State fields
| Field | Type | Notes |
|---|---|---|
| `armed` | bool | All thrust → 0 when False |
| `mode` | str | `"MANUAL"` / `"DEPTH_HOLD"` / `"HEADING_HOLD"` |
| `roll` | float (deg) | Simulated gently oscillating noise |
| `pitch` | float (deg) | Simulated gently oscillating noise |
| `yaw` | float 0–360 | Wraps modulo 360 |
| `depth` | float ≥ 0 | Metres, clamped at 0 (surface) |
| `lat`, `lon` | float | WGS-84, start at Bengaluru (12.9716, 77.5946) |
| `battery_voltage` | float | Starts 16.8 V, decays with time + thrust |

#### Thruster Model
```python
# First-order lag, tau = 0.15 s
actual += (dt / (tau + dt)) * (cmd - actual)
```
Four channels: `surge`, `sway`, `heave`, `yaw_torque`.

#### Hydrodynamic Drag + Water Current
```python
# Velocity relative to water current
v_rel_n = v_north - current_north
v_rel_e = v_east  - current_east
F_drag_n = -(k_lin * v_rel_n + k_quad * abs(v_rel_n) * v_rel_n)
F_drag_e = -(k_lin * v_rel_e + k_quad * abs(v_rel_e) * v_rel_e)
```
Linear drag `k_lin ≈ 0.5`, quadratic drag `k_quad ≈ 0.2`.

#### PID Controllers
```python
class PID:
    def __init__(self, kp, ki, kd, out_limit): ...
    def update(self, error, dt) -> float: ...  # anti-windup integral clamp
    def reset(self): ...
```
- **Depth Hold** — error = `target_depth - depth` → heave thrust
- **Heading Hold** — error = `(target - yaw + 180) % 360 - 180` → yaw thrust

#### Safety Watchdogs (CRITICAL)
```python
if not self.armed:
    # zero everything, reset PID integrals
    self.cmd_surge = self.cmd_sway = self.cmd_heave = self.cmd_yaw = 0.0
    self.actual_surge = ... = 0.0
    self._depth_pid.reset(); self._heading_pid.reset()

if self.mode == "MANUAL":
    if monotonic() - self.last_manual_input_time > 1.0:
        self.cmd_surge = self.cmd_sway = self.cmd_heave = self.cmd_yaw = 0.0
```

---

### 2. `map_widget.py` — OpenStreetMap Tile Widget

- Subclass `QWidget`, override `paintEvent`.
- Zoom level **15** tiles (256×256 PNG) from `https://tile.openstreetmap.org/{z}/{x}/{y}.png`.
- Tiles fetched async via `QNetworkAccessManager`, cached as `QPixmap` by `(z, x, y)`.
- Draws 3×3 tile grid centred on ROV position.
- **Path trail**: up to 500 `(lat, lon)` points drawn as a polyline.
- **ROV marker**: red filled circle + heading arrow at `yaw` angle.

Tile math:
```python
def latlon_to_tile(lat, lon, zoom):
    n = 2 ** zoom
    x = int((lon + 180) / 360 * n)
    lat_r = math.radians(lat)
    y = int((1 - math.log(math.tan(lat_r) + 1/math.cos(lat_r)) / math.pi) / 2 * n)
    return x, y
```

---

### 3. `main_gui.py` — Main Window

#### Layout
```
┌────────────────────────────────────────────────────┐
│  Telemetry Panel (left)   │  Map Widget (right)    │
│  ─ Armed / Mode           │  OpenStreetMap tiles   │
│  ─ Roll / Pitch / Yaw     │                        │
│  ─ Depth / Battery        ├────────────────────────│
│  Controls:                │  Depth Plot (bottom)   │
│  ─ Arm/Disarm btn         │  pyqtgraph 30s window  │
│  ─ Mode radio buttons     │  Actual vs Target      │
│  ─ Target Depth spinbox   │                        │
│  ─ Target Heading spinbox │                        │
│  ─ Current N/E sliders    │                        │
│  ─ Record CSV toggle      │                        │
└────────────────────────────────────────────────────┘
```

#### Two QTimers
| Timer | Interval | Task |
|---|---|---|
| `sim_timer` | 20 ms (50 Hz) | `sim.step(0.02)` |
| `ui_timer` | 50 ms (20 Hz) | Update labels, plot, map |

#### Keyboard Controls
| Key | Action |
|---|---|
| `W` / `S` | Surge forward / backward |
| `A` / `D` | Sway left / right |
| `↑` / `↓` | Heave up / down |
| `←` / `→` | Yaw left / right |
| `Space` | Arm / Disarm toggle |

Each key press updates `sim.last_manual_input_time`.

---

### 4. `test_sim.py` — Unit Tests

| Test | Checks |
|---|---|
| `test_disarmed_zeroes_thrust` | All thrust = 0 when `armed=False` |
| `test_manual_watchdog` | Thrusts zero after 1.1 s without input |
| `test_pid_depth_convergence` | Depth Hold reaches target ±0.1 m in 10 s |
| `test_heading_wrap` | Error calc correct across 0/360 wrap |
| `test_battery_drain` | Voltage decreases monotonically under thrust |

```bash
pytest test_sim.py -v
```

---

### 5. `requirements.txt`
```
PySide6>=6.6
pyqtgraph>=0.13
numpy>=1.26
pytest>=7.0
```

---

### 6. `README.md` Sections
- Project overview
- Install & run instructions
- Keyboard controls reference
- **AI Tool Disclosure** (required by assessment)
- Architecture notes

---

## Verification Plan

### Automated Tests
```bash
cd /home/plsh/intern_assessment_cortech
pytest test_sim.py -v
```
Expected: **5 passed, 0 failed**.

### Manual Verification Checklist
- [ ] `python3 main_gui.py` — window opens without crash
- [ ] Press `Space` → armed state toggles
- [ ] Hold `W` → ROV moves, path trail appears on map
- [ ] Switch to **Depth Hold**, set target 5 m → depth converges
- [ ] Switch to **Heading Hold**, set target 90° → yaw converges
- [ ] Release all keys 1.1 s in Manual → thrust returns to 0
- [ ] **Disarm** → thrusts immediately 0, PID integrals reset
- [ ] Enable CSV log → `telemetry.csv` written with correct columns
