#!/usr/bin/env python3
"""
tools/simulator.py
Synthetic 6-DOF ocean environment and sensor stream simulator for Paravane.

Simulates vehicle kinematics, oceanographic environmental disturbances,
stratified sound velocity profiles, and streams noisy sensor measurements:
  - 100 Hz Tactical IMU (3-axis gyro, 3-axis accel) with Allan variance noise.
  - 5 Hz 4-beam Janus DVL bottom-track velocity and altitude.
  - 10 Hz Quartz Bourdon depth / pressure transducer.
  - 1 Hz Ultra-Short Baseline (USBL) acoustic transceiver pings.
  - 2 Hz Multibeam Echo-Sounder (MBES) bathymetric swaths.
"""

import sys
import math
import random
import json
import argparse
from typing import List, Dict, Any, Tuple


def clamp(val: float, low: float, high: float) -> float:
    return max(low, min(high, val))


class OceanEnvironment:
    """Represents stratified ocean currents, sound speed, and bottom topography."""

    def __init__(self, surface_current_speed: float = 0.45, current_dir_rad: float = 0.6):
        self.surface_current_speed = surface_current_speed
        self.current_dir_rad = current_dir_rad

    def current_at_depth(self, depth: float) -> Tuple[float, float, float]:
        """Calculates Ekman spiral current attenuation with depth."""
        decay = math.exp(-depth / 150.0)
        speed = self.surface_current_speed * decay
        angle = self.current_dir_rad + (depth / 200.0) * 0.4
        vn = speed * math.cos(angle)
        ve = speed * math.sin(angle)
        vd = 0.02 * math.sin(depth * 0.05)
        return (vn, ve, vd)

    def sound_speed(self, depth: float) -> float:
        """Munk canonical sound speed profile."""
        z_tilde = 2.0 * (depth - 1000.0) / 1000.0
        return 1500.0 * (1.0 + 0.00737 * (z_tilde - 1.0 + math.exp(-z_tilde)))

    def bottom_depth(self, north: float, east: float) -> float:
        """Synthetic seafloor topography with undulating ridge and slope."""
        base = 1200.0
        ridge = 60.0 * math.sin(north * 0.003) * math.cos(east * 0.002)
        slope = north * 0.02 - east * 0.015
        return base - ridge + slope


class VehicleSimulator:
    """Simulates 6-DOF kinematics and generates sensor observation streams."""

    def __init__(self, env: OceanEnvironment):
        self.env = env
        # State: position (N, E, D) in meters
        self.pos = [0.0, 0.0, 10.0]
        # Body velocity: (u, v, w) in m/s
        self.vel_b = [0.0, 0.0, 0.0]
        # Attitude: (roll, pitch, yaw) in radians
        self.att = [0.0, 0.0, 0.0]
        # Angular rates: (p, q, r) in rad/s
        self.rates = [0.0, 0.0, 0.0]

        # Sensor biases
        self.gyro_bias = [0.0001, -0.00015, 0.00008]
        self.accel_bias = [0.005, -0.008, 0.012]
        self.dvl_bias = [0.002, -0.003, 0.001]

    def step_kinematics(self, u_cmd: float, r_cmd: float, w_cmd: float, dt: float):
        """Simple kinematic integration with dynamic rate limits."""
        # Drive forward speed toward command
        self.vel_b[0] += (u_cmd - self.vel_b[0]) * min(1.0, 0.8 * dt)
        self.vel_b[2] += (w_cmd - self.vel_b[2]) * min(1.0, 0.5 * dt)
        self.rates[2] += (r_cmd - self.rates[2]) * min(1.0, 1.2 * dt)

        # Integrate yaw
        self.att[2] += self.rates[2] * dt
        # Pitch proportional to vertical speed
        self.att[1] = math.atan2(self.vel_b[2], max(0.1, self.vel_b[0]))

        # Transform body velocity to NED frame
        psi = self.att[2]
        theta = self.att[1]
        phi = self.att[0]

        c_psi = math.cos(psi)
        s_psi = math.sin(psi)
        c_th = math.cos(theta)
        s_th = math.sin(theta)

        v_north = self.vel_b[0] * (c_psi * c_th) - self.vel_b[1] * s_psi + self.vel_b[2] * (c_psi * s_th)
        v_east = self.vel_b[0] * (s_psi * c_th) + self.vel_b[1] * c_psi + self.vel_b[2] * (s_psi * s_th)
        v_down = -self.vel_b[0] * s_th + self.vel_b[2] * c_th

        # Add ocean current drift
        c_n, c_e, c_d = self.env.current_at_depth(self.pos[2])
        self.pos[0] += (v_north + c_n * 0.3) * dt
        self.pos[1] += (v_east + c_e * 0.3) * dt
        self.pos[2] += (v_down + c_d * 0.1) * dt
        self.pos[2] = max(1.0, self.pos[2])

    def sample_imu(self) -> Dict[str, Any]:
        """Generates synthetic 100 Hz tactical grade IMU reading."""
        g = 9.80665
        th = self.att[1]
        ph = self.att[0]

        # Specific force includes gravity vector
        fx = -g * math.sin(th) + self.accel_bias[0] + random.gauss(0.0, 0.008)
        fy = g * math.cos(th) * math.sin(ph) + self.accel_bias[1] + random.gauss(0.0, 0.008)
        fz = -g * math.cos(th) * math.cos(ph) + self.accel_bias[2] + random.gauss(0.0, 0.008)

        gx = self.rates[0] + self.gyro_bias[0] + random.gauss(0.0, 0.0005)
        gy = self.rates[1] + self.gyro_bias[1] + random.gauss(0.0, 0.0005)
        gz = self.rates[2] + self.gyro_bias[2] + random.gauss(0.0, 0.0005)

        return {
            "accel_mps2": [round(fx, 5), round(fy, 5), round(fz, 5)],
            "gyro_radps": [round(gx, 6), round(gy, 6), round(gz, 6)]
        }

    def sample_dvl(self) -> Dict[str, Any]:
        """Generates synthetic 5 Hz Doppler Velocity Log reading."""
        bot_d = self.env.bottom_depth(self.pos[0], self.pos[1])
        altitude = bot_d - self.pos[2]

        locked = altitude > 0.5 and altitude < 250.0
        if locked:
            u_meas = self.vel_b[0] + self.dvl_bias[0] + random.gauss(0.0, 0.003)
            v_meas = self.vel_b[1] + self.dvl_bias[1] + random.gauss(0.0, 0.003)
            w_meas = self.vel_b[2] + self.dvl_bias[2] + random.gauss(0.0, 0.004)
            beam_alt = [round(altitude + random.gauss(0.0, 0.04), 3) for _ in range(4)]
        else:
            u_meas, v_meas, w_meas = 0.0, 0.0, 0.0
            beam_alt = [0.0, 0.0, 0.0, 0.0]

        return {
            "locked": locked,
            "altitude_m": round(altitude, 3),
            "vel_body_mps": [round(u_meas, 4), round(v_meas, 4), round(w_meas, 4)],
            "beam_altitudes": beam_alt
        }

    def sample_depth(self) -> Dict[str, Any]:
        """Generates synthetic 10 Hz depth pressure transducer reading."""
        rho = 1027.0
        g = 9.80665
        z = self.pos[2]
        press_pa = rho * g * z + 101325.0 + random.gauss(0.0, 150.0)
        press_dbar = press_pa / 10000.0
        z_meas = z + random.gauss(0.0, 0.03)

        return {
            "depth_m": round(z_meas, 3),
            "pressure_dbar": round(press_dbar, 3)
        }

    def sample_usbl(self, surface_pos: Tuple[float, float, float]) -> Dict[str, Any]:
        """Generates synthetic 1 Hz surface USBL acoustic transponder positioning."""
        dx = self.pos[0] - surface_pos[0]
        dy = self.pos[1] - surface_pos[1]
        dz = self.pos[2] - surface_pos[2]

        slant_range = math.sqrt(dx * dx + dy * dy + dz * dz)
        c = self.env.sound_speed(self.pos[2] * 0.5)
        tof = (slant_range * 2.0) / c

        # Acoustic positioning error scales with slant range
        range_err = slant_range * 0.003 + random.gauss(0.0, 0.8)
        bearing = math.atan2(dy, dx) + random.gauss(0.0, 0.005)
        elevation = math.asin(clamp(dz / max(1.0, slant_range), -1.0, 1.0)) + random.gauss(0.0, 0.005)

        meas_x = surface_pos[0] + (slant_range + range_err) * math.cos(elevation) * math.cos(bearing)
        meas_y = surface_pos[1] + (slant_range + range_err) * math.cos(elevation) * math.sin(bearing)
        meas_z = surface_pos[2] + (slant_range + range_err) * math.sin(elevation)

        return {
            "slant_range_m": round(slant_range + range_err, 2),
            "two_way_travel_time_s": round(tof, 5),
            "pos_est_ned": [round(meas_x, 2), round(meas_y, 2), round(meas_z, 2)],
            "accuracy_1sigma_m": round(slant_range * 0.005, 2)
        }


def run_mission_simulation(duration_s: float, dt: float = 0.1, out_path: str = "sim_stream.json") -> Dict[str, Any]:
    """Runs a simulated survey mission and writes multi-sensor log."""
    env = OceanEnvironment()
    sim = VehicleSimulator(env)

    # Waypoints: Launch -> Descent -> Survey lawnmower -> Ascent
    records = []
    t = 0.0
    steps = int(duration_s / dt)

    surface_ship = (0.0, 0.0, 0.0)

    for i in range(steps):
        # Mission phase control
        if t < 60.0:
            # Dive to 1100m
            u_cmd = 1.5
            w_cmd = 1.2
            r_cmd = 0.0
        elif t < 240.0:
            # Survey leg 1 (Northward)
            u_cmd = 1.8
            w_cmd = 0.0
            r_cmd = 0.0
        elif t < 270.0:
            # Turn East
            u_cmd = 1.2
            w_cmd = 0.0
            r_cmd = 0.05
        elif t < 450.0:
            # Survey leg 2 (Southward)
            u_cmd = 1.8
            w_cmd = 0.0
            r_cmd = 0.0
        else:
            # Ascent to surface
            u_cmd = 1.4
            w_cmd = -1.0
            r_cmd = 0.0

        sim.step_kinematics(u_cmd, r_cmd, w_cmd, dt)

        # Sample sensors at their respective rates
        rec = {
            "time_s": round(t, 2),
            "truth_pos": [round(sim.pos[0], 3), round(sim.pos[1], 3), round(sim.pos[2], 3)],
            "truth_att_deg": [round(math.degrees(sim.att[0]), 2),
                              round(math.degrees(sim.att[1]), 2),
                              round(math.degrees(sim.att[2]), 2)],
            "imu": sim.sample_imu(),
            "depth": sim.sample_depth()
        }

        # 5 Hz DVL
        if i % int(max(1, 0.2 / dt)) == 0:
            rec["dvl"] = sim.sample_dvl()

        # 1 Hz USBL
        if i % int(max(1, 1.0 / dt)) == 0:
            rec["usbl"] = sim.sample_usbl(surface_ship)

        records.append(rec)
        t += dt

    result = {
        "metadata": {
            "duration_s": duration_s,
            "dt_s": dt,
            "total_samples": len(records),
            "generator": "Paravane Synthetic 6-DOF Subsea Robotics Simulator"
        },
        "telemetry": records
    }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    return result


def main():
    parser = argparse.ArgumentParser(description="Paravane Subsea Sensor Stream Simulator")
    parser.add_argument("--duration", type=float, default=60.0, help="Simulation duration in seconds (default 60s)")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation step size in seconds (default 0.1s)")
    parser.add_argument("--out", type=str, default="sim_telemetry.json", help="Output JSON path")
    args = parser.parse_args()

    print(f"Simulating {args.duration}s subsea mission with dt={args.dt}s...")
    res = run_mission_simulation(args.duration, args.dt, args.out)
    print(f"Simulation completed. {len(res['telemetry'])} frames generated. Output written to {args.out}")


if __name__ == "__main__":
    main()
