"""
sim_core.py — ROV Physics & Control Engine
Pure Python, no GUI imports. Runs at dt=0.02s (50 Hz).
"""

import math
import time


# ---------------------------------------------------------------------------
# Simple PID controller with anti-windup
# ---------------------------------------------------------------------------
class PID:
    def __init__(self, kp: float, ki: float, kd: float, out_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        self._integral += error * dt
        # Anti-windup: clamp integral contribution
        integral_limit = self.out_limit / max(self.ki, 1e-9)
        self._integral = max(-integral_limit, min(integral_limit, self._integral))

        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.out_limit, min(self.out_limit, output))


# ---------------------------------------------------------------------------
# ROV Simulator
# ---------------------------------------------------------------------------
class ROVSimulator:
    # Metres per degree (approximate, good enough for small displacements)
    _M_PER_DEG_LAT = 111_320.0

    def __init__(self, lat0: float = 12.9716, lon0: float = 77.5946):
        # --- Vehicle state ---
        self.armed: bool = False
        self.mode: str = "MANUAL"          # "MANUAL" | "DEPTH_HOLD" | "HEADING_HOLD"

        self.roll: float = 0.0             # deg
        self.pitch: float = 0.0            # deg
        self.yaw: float = 0.0              # deg  [0, 360)
        self.depth: float = 0.0            # m >= 0
        self.lat: float = lat0
        self.lon: float = lon0
        self.battery_voltage: float = 16.8 # V

        # --- Velocities (body frame, m/s) ---
        self._vx: float = 0.0  # surge (forward)
        self._vy: float = 0.0  # sway  (right)
        self._vz: float = 0.0  # heave (down)
        self._vr: float = 0.0  # yaw rate (deg/s)

        # --- Thruster commands & actual (after lag filter) ---
        self.cmd_surge: float = 0.0
        self.cmd_sway: float = 0.0
        self.cmd_heave: float = 0.0
        self.cmd_yaw: float = 0.0

        self._act_surge: float = 0.0
        self._act_sway: float = 0.0
        self._act_heave: float = 0.0
        self._act_yaw: float = 0.0

        # --- Thruster lag time-constant ---
        self._tau: float = 0.15  # seconds

        # --- Drag coefficients ---
        self._k_lin: float = 0.5   # linear drag
        self._k_quad: float = 0.15  # quadratic drag

        # --- Water current (m/s, world frame) ---
        self.current_north: float = 0.0
        self.current_east: float = 0.0

        # --- PID controllers ---
        self._depth_pid = PID(kp=0.8, ki=0.15, kd=1.2, out_limit=1.0)
        self._heading_pid = PID(kp=0.03, ki=0.005, kd=0.01, out_limit=1.0)

        # --- Autopilot targets ---
        self.target_depth: float = 0.0
        self.target_heading: float = 0.0

        # --- Safety timing ---
        self.last_manual_input_time: float = time.monotonic()

        # --- Simulation clock ---
        self._t: float = 0.0

    # ------------------------------------------------------------------
    # Public control interface
    # ------------------------------------------------------------------
    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = False

    def set_mode(self, mode: str):
        assert mode in ("MANUAL", "DEPTH_HOLD", "HEADING_HOLD")
        self.mode = mode
        self._depth_pid.reset()
        self._heading_pid.reset()

    def set_manual_thrust(self, surge: float, sway: float, heave: float, yaw: float):
        """Called by keyboard/joystick handler. Values in [-1, 1]."""
        self.cmd_surge = max(-1.0, min(1.0, surge))
        self.cmd_sway  = max(-1.0, min(1.0, sway))
        self.cmd_heave = max(-1.0, min(1.0, heave))
        self.cmd_yaw   = max(-1.0, min(1.0, yaw))
        self.last_manual_input_time = time.monotonic()

    def set_target_depth(self, d: float):
        self.target_depth = max(0.0, d)

    def set_target_heading(self, h: float):
        self.target_heading = h % 360.0

    def set_water_current(self, north: float, east: float):
        self.current_north = north
        self.current_east = east

    # ------------------------------------------------------------------
    # Main simulation step
    # ------------------------------------------------------------------
    def step(self, dt: float = 0.02) -> dict:
        self._t += dt

        # ---- Safety: disarmed ----
        if not self.armed:
            self.cmd_surge = self.cmd_sway = self.cmd_heave = self.cmd_yaw = 0.0
            self._act_surge = self._act_sway = self._act_heave = self._act_yaw = 0.0
            self._depth_pid.reset()
            self._heading_pid.reset()
            return self._state_dict()

        # ---- Safety: manual watchdog ----
        if self.mode == "MANUAL":
            if time.monotonic() - self.last_manual_input_time > 1.0:
                self.cmd_surge = self.cmd_sway = self.cmd_heave = self.cmd_yaw = 0.0

        # ---- Autopilot overrides ----
        if self.mode == "DEPTH_HOLD":
            depth_err = self.target_depth - self.depth
            self.cmd_heave = self._depth_pid.update(depth_err, dt)

        if self.mode == "HEADING_HOLD":
            heading_err = (self.target_heading - self.yaw + 180.0) % 360.0 - 180.0
            self.cmd_yaw = self._heading_pid.update(heading_err, dt)

        # ---- Thruster lag (first-order low-pass) ----
        alpha = dt / (self._tau + dt)
        self._act_surge += alpha * (self.cmd_surge - self._act_surge)
        self._act_sway  += alpha * (self.cmd_sway  - self._act_sway)
        self._act_heave += alpha * (self.cmd_heave - self._act_heave)
        self._act_yaw   += alpha * (self.cmd_yaw   - self._act_yaw)

        # ---- Thrust forces (scale factor: 1 unit = 30 N / 50 kg → 0.6 m/s²) ----
        force_surge = self._act_surge * 0.6  # m/s²
        force_sway  = self._act_sway  * 0.6
        force_heave = self._act_heave * 0.6
        force_yaw   = self._act_yaw   * 40.0  # deg/s²

        # ---- Water current in body frame ----
        yaw_r = math.radians(self.yaw)
        cos_y, sin_y = math.cos(yaw_r), math.sin(yaw_r)
        # World → body rotation
        cur_body_x =  cos_y * self.current_north + sin_y * self.current_east
        cur_body_y = -sin_y * self.current_north + cos_y * self.current_east

        # ---- Velocity relative to current ----
        vrel_x = self._vx - cur_body_x
        vrel_y = self._vy - cur_body_y
        vrel_z = self._vz  # no vertical current

        # ---- Drag (linear + quadratic) ----
        def drag(v):
            return -(self._k_lin * v + self._k_quad * abs(v) * v)

        # ---- Velocity update ----
        self._vx += (force_surge + drag(vrel_x)) * dt
        self._vy += (force_sway  + drag(vrel_y)) * dt
        self._vz += (force_heave + drag(vrel_z)) * dt
        self._vr += (force_yaw   - 5.0 * self._vr) * dt   # rotational drag

        # ---- Position update ----
        # Body → world frame
        world_north = cos_y * self._vx - sin_y * self._vy
        world_east  = sin_y * self._vx + cos_y * self._vy

        self.lat += (world_north / self._M_PER_DEG_LAT) * dt
        m_per_deg_lon = self._M_PER_DEG_LAT * math.cos(math.radians(self.lat))
        self.lon += (world_east / max(m_per_deg_lon, 1.0)) * dt

        self.depth = max(0.0, self.depth + self._vz * dt)
        self.yaw = (self.yaw + self._vr * dt) % 360.0

        # ---- Simulated roll/pitch (passive oscillation) ----
        self.roll  = 3.0 * math.sin(self._t * 0.5 + 1.0)
        self.pitch = 2.0 * math.sin(self._t * 0.7)

        # ---- Battery decay ----
        thrust_load = (abs(self._act_surge) + abs(self._act_sway) +
                       abs(self._act_heave) + abs(self._act_yaw)) / 4.0
        self.battery_voltage -= (0.00005 + 0.0002 * thrust_load) * dt
        self.battery_voltage = max(10.0, self.battery_voltage)

        return self._state_dict()

    # ------------------------------------------------------------------
    # State snapshot
    # ------------------------------------------------------------------
    def _state_dict(self) -> dict:
        return {
            "armed":           self.armed,
            "mode":            self.mode,
            "roll":            self.roll,
            "pitch":           self.pitch,
            "yaw":             self.yaw,
            "depth":           self.depth,
            "lat":             self.lat,
            "lon":             self.lon,
            "battery_voltage": self.battery_voltage,
            "cmd_surge":       self.cmd_surge,
            "cmd_sway":        self.cmd_sway,
            "cmd_heave":       self.cmd_heave,
            "cmd_yaw":         self.cmd_yaw,
            "act_surge":       self._act_surge,
            "act_sway":        self._act_sway,
            "act_heave":       self._act_heave,
            "act_yaw":         self._act_yaw,
            "target_depth":    self.target_depth,
            "target_heading":  self.target_heading,
        }
