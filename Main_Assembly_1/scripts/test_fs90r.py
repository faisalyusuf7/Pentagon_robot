#!/usr/bin/env python3
"""
FS90R Continuous Rotation Servo Test — PCA9685
===============================================
The FS90R is a continuous rotation servo:
  - throttle  0.0  → stop
  - throttle  1.0  → full speed CW
  - throttle -1.0  → full speed CCW

Using adafruit_servokit's continuous_servo interface.

Wiring: FS90R signal → PCA9685 channel 0 (change CHANNEL below if different)
"""

import time
from adafruit_servokit import ServoKit

CHANNEL = 0  # PCA9685 channel the FS90R is connected to

kit = ServoKit(channels=16)

def servo_stop():
    """Fully kill PWM output on the channel — no buzz."""
    kit._pca.channels[CHANNEL].duty_cycle = 0

# Make sure servo starts stopped
servo_stop()

print("=" * 45)
print("  FS90R Continuous Rotation Servo Test")
print("=" * 45)
print(f"  Channel: {CHANNEL}")
print()
print("Commands:")
print("  w  → spin clockwise (slow)")
print("  W  → spin clockwise (full speed)")
print("  s  → spin counter-clockwise (slow)")
print("  S  → spin counter-clockwise (full speed)")
print("  x  → stop")
print("  1-9→ set throttle (1=0.1 … 9=0.9)")
print("  r  → reverse last direction")
print("  t  → quick test (CW → stop → CCW → stop)")
print("  q  → quit")
print()

last_throttle = 0.0

try:
    while True:
        key = input(">> ").strip()
        if not key:
            continue

        if key == "w":
            last_throttle = 0.3
            kit.continuous_servo[CHANNEL].throttle = last_throttle
            print(f"  CW slow  (throttle={last_throttle})")

        elif key == "W":
            last_throttle = 1.0
            kit.continuous_servo[CHANNEL].throttle = last_throttle
            print(f"  CW full  (throttle={last_throttle})")

        elif key == "s":
            last_throttle = -0.3
            kit.continuous_servo[CHANNEL].throttle = last_throttle
            print(f"  CCW slow (throttle={last_throttle})")

        elif key == "S":
            last_throttle = -1.0
            kit.continuous_servo[CHANNEL].throttle = last_throttle
            print(f"  CCW full (throttle={last_throttle})")

        elif key == "x":
            last_throttle = 0.0
            servo_stop()
            print("  STOPPED (de-energized)")

        elif key in "123456789":
            last_throttle = int(key) / 10.0
            kit.continuous_servo[CHANNEL].throttle = last_throttle
            print(f"  throttle={last_throttle}")

        elif key == "r":
            last_throttle = -last_throttle
            kit.continuous_servo[CHANNEL].throttle = last_throttle
            print(f"  reversed → throttle={last_throttle}")

        elif key == "t":
            print("  Running quick test sequence…")
            for throttle, label, wait in [
                (0.4, "CW slow", 2),
                (1.0, "CW full", 2),
                (None, "STOP", 1),
                (-0.4, "CCW slow", 2),
                (-1.0, "CCW full", 2),
                (None, "STOP", 1),
            ]:
                if throttle is None:
                    servo_stop()
                else:
                    kit.continuous_servo[CHANNEL].throttle = throttle
                print(f"    {label:10s} (throttle={throttle:+.1f})")
                time.sleep(wait)
            print("  Test complete.")

        elif key == "q":
            servo_stop()
            print("  Stopped & exiting.")
            break

        else:
            print("  Unknown command. Use w/W/s/S/x/1-9/r/t/q")

except KeyboardInterrupt:
    servo_stop()
    print("\n  Stopped by Ctrl+C.")
