"""
accelerometer_fbd_cartesian_simple.py

Reads raw ADXL335-style accelerometer data from serial,
calibrates at startup, and outputs simplified JSON packets
containing Cartesian force vectors for a basic Free Body Diagram MVP.

Expected serial input from ESP32 / Arduino:
    xRaw,yRaw,zRaw

Example:
    2048,2055,2670

Install:
    pip install pyserial
"""

import json
import math
import time
from dataclasses import dataclass, asdict


# ============================================================
# CONFIG
# ============================================================

SERIAL_PORT = "COM3"  # Change this to your serial port
BAUD_RATE = 115200

CALIBRATION_DURATION = 2.0  # seconds

ADC_MAX = 4095
ADC_MIDPOINT = ADC_MAX / 2  # 2047.5 for ESP32 12-bit ADC

MASS_KG = 0.25  # Placeholder mass. Measure and replace later.
GRAVITY_MPS2 = 9.81

SMOOTHING_ALPHA = 0.2

# Hides tiny noise that would otherwise appear as applied force.
APPLIED_FORCE_THRESHOLD_N = 0.05

# 0.02 seconds ~= 50 Hz
OUTPUT_DELAY_SECONDS = 0.02


# ============================================================
# DATA CLASSES
# ============================================================


@dataclass
class Vector3:
    """
    Simple Cartesian 3D vector.

    Coordinate convention:
        +X = horizontal axis
        +Y = horizontal axis
        +Z = up

    For force vectors:
        units are Newtons.
    """

    x: float
    y: float
    z: float

    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)


@dataclass
class AccelerometerCalibration:
    """
    Startup calibration data.

    x0, y0, z0:
        Average raw values while the accelerometer is held still.

    g_scale:
        Approximate raw-count difference corresponding to 1g.

    baseline_axis:
        Axis that was most aligned with gravity during startup calibration.

    baseline_sign:
        +1 or -1 depending on the direction of the resting gravity reading.
    """

    x0: float
    y0: float
    z0: float
    g_scale: float
    midpoint: float
    samples: int
    baseline_axis: str
    baseline_sign: int


@dataclass
class SmoothedAcceleration:
    """
    Tracks smoothed acceleration values in g units.
    """

    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    initialized: bool = False

    def update(self, ax, ay, az, alpha=0.2):
        """
        Simple exponential moving average.
        """

        if not self.initialized:
            self.ax = ax
            self.ay = ay
            self.az = az
            self.initialized = True
        else:
            self.ax = alpha * ax + (1 - alpha) * self.ax
            self.ay = alpha * ay + (1 - alpha) * self.ay
            self.az = alpha * az + (1 - alpha) * self.az

        return self.ax, self.ay, self.az


# ============================================================
# SERIAL INPUT
# ============================================================


def read_raw_accelerometer_line(ser):
    """
    Reads one serial line and parses it as:

        xRaw,yRaw,zRaw

    Returns:
        x_raw, y_raw, z_raw

    Raises:
        ValueError if the line is malformed.
    """

    line = ser.readline().decode(errors="ignore").strip()

    parts = line.split(",")
    if len(parts) != 3:
        raise ValueError(f"Invalid accelerometer line: {line}")

    x_raw, y_raw, z_raw = map(int, parts)
    return x_raw, y_raw, z_raw


# ============================================================
# CALIBRATION
# ============================================================


def calibrate_accelerometer(ser, duration=2.0, midpoint=2047.5):
    """
    Calibrates accelerometer from serial data for a fixed duration.

    Assumptions:
        - Device is held still during calibration.
        - Device is resting flat in the intended demo orientation.
        - Serial lines are formatted as xRaw,yRaw,zRaw.

    Returns:
        AccelerometerCalibration
    """

    xs = []
    ys = []
    zs = []

    start_time = time.time()

    while time.time() - start_time < duration:
        try:
            x_raw, y_raw, z_raw = read_raw_accelerometer_line(ser)

            xs.append(x_raw)
            ys.append(y_raw)
            zs.append(z_raw)

        except ValueError:
            continue

    if not xs:
        raise RuntimeError(
            "Calibration failed: no valid accelerometer samples received."
        )

    x0 = sum(xs) / len(xs)
    y0 = sum(ys) / len(ys)
    z0 = sum(zs) / len(zs)

    deviations = {"x": x0 - midpoint, "y": y0 - midpoint, "z": z0 - midpoint}

    baseline_axis = max(deviations, key=lambda axis: abs(deviations[axis]))
    largest_deviation = deviations[baseline_axis]

    g_scale = abs(largest_deviation)

    if g_scale == 0:
        raise RuntimeError("Calibration failed: g_scale is zero.")

    baseline_sign = 1 if largest_deviation >= 0 else -1

    return AccelerometerCalibration(
        x0=x0,
        y0=y0,
        z0=z0,
        g_scale=g_scale,
        midpoint=midpoint,
        samples=len(xs),
        baseline_axis=baseline_axis,
        baseline_sign=baseline_sign,
    )


# ============================================================
# ACCELERATION PROCESSING
# ============================================================


def raw_to_acceleration_g(x_raw, y_raw, z_raw, calibration):
    """
    Converts raw ADC values into acceleration change from baseline, in g units.

    At rest after calibration:
        ax ≈ 0
        ay ≈ 0
        az ≈ 0

    Returns:
        ax_g, ay_g, az_g
    """

    ax_g = (x_raw - calibration.x0) / calibration.g_scale
    ay_g = (y_raw - calibration.y0) / calibration.g_scale
    az_g = (z_raw - calibration.z0) / calibration.g_scale

    return ax_g, ay_g, az_g


def acceleration_g_to_mps2(accel_g):
    """
    Converts acceleration in g-units to m/s^2.
    """

    return accel_g * GRAVITY_MPS2


# ============================================================
# FORCE VECTOR GENERATION
# ============================================================


def make_gravity_force(mass_kg=MASS_KG):
    """
    Gravity force vector.

    Coordinate convention:
        +Z = up
        -Z = down

    Gravity points downward along -Z.
    """

    magnitude = mass_kg * GRAVITY_MPS2

    return Vector3(x=0.0, y=0.0, z=-magnitude)


def make_normal_force(mass_kg=MASS_KG):
    """
    Normal force vector for a flat surface.

    MVP simplification:
        Object is on a flat surface.
        No vertical acceleration.
        Normal force approximately cancels gravity.
    """

    magnitude = mass_kg * GRAVITY_MPS2

    return Vector3(x=0.0, y=0.0, z=magnitude)


def make_applied_force_from_acceleration(ax_g, ay_g, mass_kg=MASS_KG):
    """
    Estimates horizontal applied force from X/Y acceleration.

    Uses only X and Y axes.

    Formula:
        F = m * a

    Since ax_g and ay_g are in g-units:
        ax_mps2 = ax_g * 9.81
        ay_mps2 = ay_g * 9.81

    For flat-surface MVP:
        applied z force = 0
    """

    ax_mps2 = acceleration_g_to_mps2(ax_g)
    ay_mps2 = acceleration_g_to_mps2(ay_g)

    fx = mass_kg * ax_mps2
    fy = mass_kg * ay_mps2
    fz = 0.0

    applied_force = Vector3(fx, fy, fz)

    if applied_force.magnitude() < APPLIED_FORCE_THRESHOLD_N:
        return Vector3(0.0, 0.0, 0.0)

    return applied_force


def make_net_force(gravity_force, normal_force, applied_force):
    """
    Computes net force.

    MVP simplification:
        On a flat surface, gravity and normal cancel each other.
        Therefore:
            net force = applied force

    Keeping this as a separate function makes it easy to improve later:
        net = gravity + normal + applied + friction + etc.
    """

    return Vector3(x=applied_force.x, y=applied_force.y, z=applied_force.z)


# ============================================================
# JSON PACKET GENERATION
# ============================================================


def vector_payload(name, vector):
    """
    Creates one simplified vector object for JSON.
    """

    return {"name": name, "x": vector.x, "y": vector.y, "z": vector.z}


def build_fbd_packet(ax_g, ay_g, az_g, calibration, mass_kg=MASS_KG):
    """
    Builds simplified JSON packet.

    Includes only:
        - surface
        - tilt_angle
        - vectors:
            gravity
            normal
            applied
            net

    For now:
        surface = flat
        tilt_angle = 0.0
        net = applied
    """

    gravity = make_gravity_force(mass_kg)
    normal = make_normal_force(mass_kg)

    applied = make_applied_force_from_acceleration(
        ax_g=ax_g, ay_g=ay_g, mass_kg=mass_kg
    )

    net = make_net_force(
        gravity_force=gravity, normal_force=normal, applied_force=applied
    )

    return {
        "surface": "flat",
        "tilt_angle": 0.0,
        "vectors": [
            vector_payload("gravity", gravity),
            vector_payload("normal", normal),
            vector_payload("applied", applied),
            vector_payload("net", net),
        ],
    }


def packet_to_json(packet):
    """
    Converts packet dictionary to compact JSON.
    """

    return json.dumps(packet, separators=(",", ":"))


# ============================================================
# MAIN LOOP
# ============================================================


def main():
    """
    Main demo loop.

    Reads serial accelerometer data, calibrates, then prints JSON packets.

    Replace:
        print(json_packet)

    with:
        send_to_visualizer(json_packet)

    when connecting to your 3D visualization tool.
    """

    import serial

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    print("Calibrating accelerometer...")
    print("Keep the device still and flat.")

    calibration = calibrate_accelerometer(
        ser, duration=CALIBRATION_DURATION, midpoint=ADC_MIDPOINT
    )

    print("Calibration complete.")
    print(asdict(calibration))

    smoother = SmoothedAcceleration()

    while True:
        try:
            x_raw, y_raw, z_raw = read_raw_accelerometer_line(ser)

            ax_g, ay_g, az_g = raw_to_acceleration_g(
                x_raw=x_raw, y_raw=y_raw, z_raw=z_raw, calibration=calibration
            )

            ax_g, ay_g, az_g = smoother.update(
                ax=ax_g, ay=ay_g, az=az_g, alpha=SMOOTHING_ALPHA
            )

            packet = build_fbd_packet(
                ax_g=ax_g,
                ay_g=ay_g,
                az_g=az_g,
                calibration=calibration,
                mass_kg=MASS_KG,
            )

            json_packet = packet_to_json(packet)

            print(json_packet)

            time.sleep(OUTPUT_DELAY_SECONDS)

        except ValueError:
            continue

        except KeyboardInterrupt:
            print("Stopped.")
            break


if __name__ == "__main__":
    main()
