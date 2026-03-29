#!/usr/bin/env python3
"""
Stepper Angle Test — Verify IK angles match between ROS and Arduino
====================================================================
Run WITHOUT Arduino connected to see the angle table,
or WITH Arduino to send test positions one by one.

Usage:
    python3 test_stepper_angles.py                    # table only
    python3 test_stepper_angles.py /dev/ttyUSB0       # send to Arduino
    python3 test_stepper_angles.py /dev/ttyUSB0 F4    # go to specific hole
"""

import math
import sys
import time

# ===== Robot geometry (must match Arduino + ROS) =====
L1, L2, d = 200.0, 200.0, 190.0   # mm

STEPS_PER_REV    = 200.0
MICROSTEPS       = 16.0
STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS) / 360.0   # 8.8889

HOME_ANGLE = 90.0   # degrees — both motors point +Y


# ===== IK solver (law of cosines, same as Arduino & ROS) =====
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ik_solve(px_mm, py_mm):
    """IK in mm. Returns (theta_left_deg, theta_right_deg) or None."""
    b1x, b2x = -d / 2.0, d / 2.0

    r1 = math.hypot(px_mm - b1x, py_mm)
    r2 = math.hypot(px_mm - b2x, py_mm)

    rmin, rmax = abs(L1 - L2), L1 + L2
    if r1 < rmin - 0.1 or r1 > rmax + 0.1:
        return None
    if r2 < rmin - 0.1 or r2 > rmax + 0.1:
        return None

    phi1 = math.atan2(py_mm, px_mm - b1x)
    ca1  = _clamp((L1**2 + r1**2 - L2**2) / (2 * L1 * r1), -1, 1)
    th1  = phi1 + math.acos(ca1)

    phi2 = math.atan2(py_mm, px_mm - b2x)
    ca2  = _clamp((L1**2 + r2**2 - L2**2) / (2 * L1 * r2), -1, 1)
    th2  = phi2 - math.acos(ca2)

    return math.degrees(th1), math.degrees(th2)


# ===== Hole positions (world frame → IK frame) =====
import numpy as np

_BASE_LINK_OFFSET = np.array([-0.006403, -0.103113, -0.00474])
_POS_ML = np.array([0.695406926143987, -1.58664693818658, 0.05]) + _BASE_LINK_OFFSET
_POS_MR = np.array([0.885406926135972, -1.58664693819113, 0.07]) + _BASE_LINK_OFFSET
IK_ORIGIN = (_POS_ML + _POS_MR) / 2.0   # metres

HOLES = {
    "F0": (0.724254, -1.440010), "F1": (0.724254, -1.395010), "F2": (0.724254, -1.350010),
    "F3": (0.769254, -1.440010), "F4": (0.769254, -1.395010), "F5": (0.769254, -1.350010),
    "F6": (0.814254, -1.440010), "F7": (0.814254, -1.395010), "F8": (0.814254, -1.350010),
    "L0": (0.594254, -1.600010), "L1": (0.549254, -1.600010), "L2": (0.504254, -1.600010),
    "L3": (0.594254, -1.645010), "L4": (0.549254, -1.645010), "L5": (0.504254, -1.645010),
    "L6": (0.594254, -1.690010), "L7": (0.549254, -1.690010), "L8": (0.504254, -1.690010),
}


def world_to_ik_mm(wx, wy):
    """World (m) → IK plane (mm).  Clamp Y >= 0.1mm to avoid atan2 wrap."""
    ix = (wx - IK_ORIGIN[0]) * 1000.0
    iy = (wy - IK_ORIGIN[1]) * 1000.0
    iy = max(0.1, iy)   # avoid atan2 discontinuity at Y=0
    return ix, iy


def urdf_from_ik(ik_deg):
    """IK degrees → URDF degrees."""
    return 90.0 - ik_deg


# ===== Print table =====
def print_table():
    print()
    print("=" * 100)
    print("STEPPER ANGLE REFERENCE TABLE")
    print("  Home = 90° / 90°  (both cranks pointing +Y)")
    print("  Convention: stepper_deg = IK angle in degrees")
    print("              URDF_deg    = 90 - stepper_deg")
    print("=" * 100)
    print()

    hdr = (f"{'Hole':>4s}  {'IK_X(mm)':>9s} {'IK_Y(mm)':>9s}  │ "
           f"{'StepL°':>7s} {'StepR°':>7s}  │ "
           f"{'URDF_L°':>8s} {'URDF_R°':>8s}  │ "
           f"{'ΔL°':>6s} {'ΔR°':>6s}  │ "
           f"{'StepsL':>7s} {'StepsR':>7s}")
    print(hdr)
    print("─" * 100)

    # Home first
    angles_home = ik_solve(0, 250)
    th1_h, th2_h = angles_home

    for name in (["Home"]
                 + [f"F{i}" for i in range(9)]
                 + ["---"]
                 + [f"L{i}" for i in range(9)]):

        if name == "---":
            print("─" * 100)
            continue

        if name == "Home":
            ix, iy = 0.0, 250.0
        else:
            wx, wy = HOLES[name]
            ix, iy = world_to_ik_mm(wx, wy)

        sol = ik_solve(ix, iy)
        if sol is None:
            print(f"{name:>4s}  {'UNREACHABLE':>20s}")
            continue

        th_L, th_R = sol
        dL = th_L - th1_h
        dR = th_R - th2_h
        uL = urdf_from_ik(th_L)
        uR = urdf_from_ik(th_R)
        sL = round(th_L * STEPS_PER_DEGREE)
        sR = round(th_R * STEPS_PER_DEGREE)

        print(f"{name:>4s}  {ix:9.2f} {iy:9.2f}  │ "
              f"{th_L:7.2f} {th_R:7.2f}  │ "
              f"{uL:8.2f} {uR:8.2f}  │ "
              f"{dL:+6.1f} {dR:+6.1f}  │ "
              f"{sL:7d} {sR:7d}")

    print()


# ===== Interactive serial test =====
def serial_test(port, target_hole=None):
    import serial as _serial

    print(f"\nConnecting to {port} at 115200 baud...")
    ser = _serial.Serial(port, 115200, timeout=2.0)
    time.sleep(2.5)
    ser.reset_input_buffer()

    # Wait for READY
    for _ in range(10):
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"  [Arduino] {line}")
        if "READY" in line:
            break

    def send(cmd):
        print(f"  → {cmd}")
        ser.write((cmd + "\n").encode())
        time.sleep(0.05)
        while ser.in_waiting:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(f"  ← {line}")

    # If a specific hole was requested, go straight there
    if target_hole:
        target_hole = target_hole.upper()
        if target_hole == "HOME":
            send("G28")
            send("M400")
            return
        if target_hole not in HOLES:
            print(f"Unknown hole: {target_hole}")
            return
        wx, wy = HOLES[target_hole]
        ix, iy = world_to_ik_mm(wx, wy)
        sol = ik_solve(ix, iy)
        if sol is None:
            print(f"{target_hole} is unreachable!")
            return
        th_L, th_R = sol
        print(f"\nMoving to {target_hole}: StepperL={th_L:.2f}° StepperR={th_R:.2f}°")
        send(f"A{th_L:.2f} B{th_R:.2f}")
        send("M400")
        send("M114")
        return

    # Interactive mode
    print("\n=== Interactive Angle Test ===")
    print("Commands:  <hole>  (e.g. F4, L0, HOME)")
    print("           q       to quit")
    print("           m114    query position")
    print("           m18     disable motors")
    print("           m17     enable motors")
    print()

    send("M114")   # show starting position

    while True:
        try:
            inp = input("\nTarget > ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not inp:
            continue
        if inp.lower() == "q":
            break
        if inp.lower() == "m114":
            send("M114")
            continue
        if inp.lower() == "m17":
            send("M17")
            continue
        if inp.lower() == "m18":
            send("M18")
            continue
        if inp.upper() == "HOME":
            send("G28")
            send("M400")
            send("M114")
            continue

        hole = inp.upper()
        if hole not in HOLES:
            print(f"  Unknown hole '{hole}'. Use F0-F8, L0-L8, or HOME.")
            continue

        wx, wy = HOLES[hole]
        ix, iy = world_to_ik_mm(wx, wy)
        sol = ik_solve(ix, iy)
        if sol is None:
            print(f"  {hole} unreachable!")
            continue

        th_L, th_R = sol
        urdf_L = urdf_from_ik(th_L)
        urdf_R = urdf_from_ik(th_R)
        print(f"  {hole}: StepperL={th_L:.2f}°  StepperR={th_R:.2f}°  "
              f"(URDF: {urdf_L:.2f}° / {urdf_R:.2f}°)")
        send(f"A{th_L:.2f} B{th_R:.2f}")
        send("M400")
        send("M114")

    send("M18")  # disable motors on exit
    ser.close()
    print("Done.")


# ===== Main =====
if __name__ == "__main__":
    print_table()

    if len(sys.argv) >= 2:
        port = sys.argv[1]
        hole = sys.argv[2] if len(sys.argv) >= 3 else None
        serial_test(port, hole)
    else:
        print("To test with Arduino:  python3 test_stepper_angles.py /dev/ttyUSB0")
        print("To go to a hole:       python3 test_stepper_angles.py /dev/ttyUSB0 F4")
