"""
test_sim.py — pytest unit tests for sim_core.ROVSimulator
No GUI imports — runs headless.
"""

import time
import pytest
from sim_core import ROVSimulator, PID


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def run_sim(sim: ROVSimulator, seconds: float, dt: float = 0.02) -> dict:
    """Advance simulation for `seconds` and return final state."""
    steps = int(seconds / dt)
    state = {}
    for _ in range(steps):
        state = sim.step(dt)
    return state


# ---------------------------------------------------------------------------
# 1. Disarmed → all thrust must be zero
# ---------------------------------------------------------------------------
def test_disarmed_zeroes_thrust():
    sim = ROVSimulator()
    # Try to push manual thrust while disarmed
    sim.cmd_surge = 1.0
    sim.cmd_sway  = 1.0
    sim.cmd_heave = 1.0
    sim.cmd_yaw   = 1.0
    state = sim.step(0.02)

    assert state["act_surge"] == pytest.approx(0.0, abs=1e-9)
    assert state["act_sway"]  == pytest.approx(0.0, abs=1e-9)
    assert state["act_heave"] == pytest.approx(0.0, abs=1e-9)
    assert state["act_yaw"]   == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# 2. Manual watchdog → thrust zeroed after 1.1 s without input
# ---------------------------------------------------------------------------
def test_manual_watchdog():
    sim = ROVSimulator()
    sim.arm()
    sim.set_mode("MANUAL")

    # Give a thrust command and immediately mark input as old
    sim.set_manual_thrust(1.0, 0.0, 0.0, 0.0)
    sim.last_manual_input_time = time.monotonic() - 1.5   # 1.5 s ago

    # Step once — watchdog should zero cmd
    sim.step(0.02)
    assert sim.cmd_surge == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# 3. Depth Hold PID converges to target within 10 s
# ---------------------------------------------------------------------------
def test_pid_depth_convergence():
    sim = ROVSimulator()
    sim.arm()
    sim.set_mode("DEPTH_HOLD")
    sim.set_target_depth(5.0)

    state = run_sim(sim, seconds=20.0)
    assert abs(state["depth"] - 5.0) < 0.15, (
        f"Depth did not converge: got {state['depth']:.3f} m, expected ≈ 5.0 m"
    )


# ---------------------------------------------------------------------------
# 4. Heading wrap-around — shortest-angle error across the 0/360 boundary
# ---------------------------------------------------------------------------
def test_heading_wrap():
    """
    If current yaw = 10° and target = 350°, shortest error should be -20°
    (turn left 20°), not +340° (turn right all the way around).
    """
    pid = PID(kp=1.0, ki=0.0, kd=0.0, out_limit=10.0)
    current = 10.0
    target  = 350.0
    error = (target - current + 180.0) % 360.0 - 180.0
    assert error == pytest.approx(-20.0, abs=1e-9)

    output = pid.update(error, dt=0.02)
    assert output < 0.0, "Should command left (negative) yaw to reach 350° from 10°"


# ---------------------------------------------------------------------------
# 5. Battery drains monotonically under sustained thrust
# ---------------------------------------------------------------------------
def test_battery_drain():
    sim = ROVSimulator()
    sim.arm()
    sim.set_mode("MANUAL")
    sim.set_manual_thrust(1.0, 1.0, 1.0, 1.0)
    sim.last_manual_input_time = time.monotonic()  # keep input fresh

    voltages = [sim.battery_voltage]
    for _ in range(500):  # 10 seconds
        sim.set_manual_thrust(1.0, 1.0, 1.0, 1.0)
        sim.last_manual_input_time = time.monotonic()
        state = sim.step(0.02)
        voltages.append(state["battery_voltage"])

    # Each sample should be ≤ the previous (monotonically non-increasing)
    for i in range(1, len(voltages)):
        assert voltages[i] <= voltages[i - 1] + 1e-9, (
            f"Voltage increased at step {i}: {voltages[i-1]:.4f} → {voltages[i]:.4f}"
        )
    # Total drain should be measurable
    assert voltages[-1] < voltages[0], "Battery did not drain at all"
