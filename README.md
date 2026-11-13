# ROV Simulator & Ground Control Station

A native desktop application that simulates a small underwater ROV and provides a Ground Control Station (GCS) interface. Built with **Python + PySide6 + pyqtgraph** as a technical assessment submission for Coratia Technologies.

---

## Features

| Feature | Details |
|---|---|
| **Physics engine** | 50 Hz simulation with first-order thruster lag (τ = 0.15 s), linear + quadratic drag |
| **Autopilot modes** | Manual, Depth Hold (PID), Heading Hold (PID with wrap-around) |
| **Safety** | Thrust zeroed when disarmed; manual watchdog after 1 s without input |
| **Live map** | OpenStreetMap tiles via QNetworkAccessManager, ROV marker + path trail |
| **Depth plot** | 30-second scrolling depth vs. target graph (pyqtgraph) |
| **Water current** | Configurable N/E current vector applied as hydrodynamic drag |
| **CSV telemetry log** | Toggle recording; writes timestamped rows at 20 Hz |
| **Unit tests** | pytest suite covering safety, PID, heading wrap, battery |

---

## Installation

```bash
# Clone / extract project
cd intern_assessment_cortech

# Install dependencies (Python 3.10+)
pip install -r requirements.txt
```

---

## Running the Application

```bash
python3 main_gui.py
```

The OSM map requires an internet connection. It degrades gracefully to a grey canvas offline.

---

## Keyboard Controls

| Key | Action |
|---|---|
| `W` / `S` | Surge forward / backward |
| `A` / `D` | Sway left / right |
| `↑` / `↓` | Heave up (ascend) / down (descend) |
| `←` / `→` | Yaw left / right |
| `Space` | Arm / Disarm toggle |

> **Manual Watchdog**: if no key is pressed for more than 1 second, all manual thrust commands return to zero automatically.

---

## Running Tests

```bash
pytest test_sim.py -v
```

Expected output:

```
test_sim.py::test_disarmed_zeroes_thrust   PASSED
test_sim.py::test_manual_watchdog          PASSED
test_sim.py::test_pid_depth_convergence    PASSED
test_sim.py::test_heading_wrap             PASSED
test_sim.py::test_battery_drain            PASSED
5 passed in X.XXs
```

---

## Project Structure

```
intern_assessment_cortech/
├── sim_core.py      # Pure Python physics engine (no GUI imports)
├── map_widget.py    # OpenStreetMap tile widget (QPainter + QNetworkAccessManager)
├── main_gui.py      # Main GCS window (PySide6 + pyqtgraph)
├── test_sim.py      # pytest unit tests
├── requirements.txt
├── README.md
├── implementation.md  # Detailed implementation plan
└── resource/prompt.md # The initial prompt written 

```

---

## Architecture

```
sim_core.py (ROVSimulator)
    ↓ state dict (50 Hz)
main_gui.py (MainWindow)
    ├── → map_widget.py  (lat/lon/yaw at 20 Hz)
    └── → pyqtgraph PlotWidget (depth history at 20 Hz)
```

Two QTimers decouple simulation from rendering:
- **sim_timer** (20 ms / 50 Hz) → `sim.step(0.02)`
- **ui_timer** (50 ms / 20 Hz) → update labels, map, plot

---

## AI Tool Disclosure

AI coding assistance was used during this project as follows:

- **Tool used**: Google Antigravity (Gemini / Claude Sonnet 4.6 Thinking), Google Gemini Argon (For prompt enhancemnt)
- **How it was used**: 
  - Scaffolding of the PySide6 layout structure and stylesheet (DARK_STYLE)
  - Generating the OSM tile math (`latlon_to_tile`, `tile_to_latlon`, `latlon_to_pixel`)
  - Drafting the initial pyqtgraph integration code
  - Generating the pytest test stubs
- **What was reviewed and understood**: Every line of generated code was read, understood, and validated against the physics model and Qt documentation before inclusion. The PID controller, thruster lag model, drag equations, and safety watchdog logic were written and verified independently.
- **Disclosure requirement**: Per assessment rules — *"AI coding tools are permitted; disclose which were used and how in the README."*

The candidate is fully prepared to explain and modify any part of this implementation during the interview.
# coratia_tech_assessment
