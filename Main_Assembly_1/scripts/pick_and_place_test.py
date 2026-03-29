#!/usr/bin/env python3
"""
Pick-and-place test — Servo (PCA9685 ch0) + Solenoid valve (GPIO pin 15).

Hardware:
  Servo channel 0  → pushes suction channel up/down
    0°   = channel DOWN
    180° = channel UP

  Solenoid valve on BOARD pin 15 (inverted logic via 2N2222 + IRLZ44N)
    Valve CLOSED (GPIO HIGH) = suction holds ball
    Valve OPEN   (GPIO LOW)  = suction released, ball drops

Pick-and-place sequence:
  PICK:
    1. Servo → 0°   (channel down onto ball)
    2. Valve CLOSED  (suction grabs ball)
    3. Wait for grip
    4. Servo → 180°  (lift ball, valve stays closed)

  PLACE:
    1. Servo → 0°   (channel down to drop location)
    2. Valve OPEN   (release ball)
    3. Wait
    4. Valve CLOSED  (stop airflow)
    5. Servo → 180°  (retract channel)
"""

import os
import signal
import sys
import time

# Force Blinka to recognise the Jetson Orin Nano (needed under sudo)
os.environ.setdefault("BLINKA_FORCEBOARD", "JETSON_ORIN_NANO")

import Jetson.GPIO as GPIO
from adafruit_servokit import ServoKit
# ── Configuration ──────────────────────────────────────────────────────────────
VALVE_PIN       = 15      # BOARD pin — 1 kΩ → 2N2222 base
SERVO_CHANNEL   = 0       # PCA9685 channel
SERVO_DOWN      = 0       # degrees — channel touching ball
SERVO_UP        = 180     # degrees — channel raised

GRIP_DELAY      = 0.5     # seconds to let suction grip the ball
RELEASE_DELAY   = 0.3     # seconds to let ball release
SERVO_MOVE_TIME = 0.6     # seconds to let servo reach position


# ── Valve helpers (inverted logic) ─────────────────────────────────────────────
def valve_close():
    """Close valve → suction active (GPIO HIGH → MOSFET OFF → valve closed)."""
    GPIO.output(VALVE_PIN, GPIO.HIGH)

def valve_open():
    """Open valve → suction released (GPIO LOW → MOSFET ON → valve open)."""
    GPIO.output(VALVE_PIN, GPIO.LOW)


# ── Servo helpers ──────────────────────────────────────────────────────────────
kit = None  # initialised in main()

def servo_down():
    """Move suction channel down."""
    kit.servo[SERVO_CHANNEL].angle = SERVO_DOWN
    time.sleep(SERVO_MOVE_TIME)

def servo_up():
    """Move suction channel up."""
    kit.servo[SERVO_CHANNEL].angle = SERVO_UP
    time.sleep(SERVO_MOVE_TIME)


# ── High-level actions ─────────────────────────────────────────────────────────
def pick():
    """Lower channel, activate suction, lift ball."""
    print("  [1/3] Servo DOWN — approaching ball")
    servo_down()

    print("  [2/3] Valve CLOSED — suction gripping ball")
    valve_close()
    time.sleep(GRIP_DELAY)

    print("  [3/3] Servo UP — lifting ball")
    servo_up()
    print("  ✓ Ball picked!\n")


def place():
    """Lower channel, release suction, retract."""
    print("  [1/4] Servo DOWN — moving to drop position")
    servo_down()

    print("  [2/4] Valve OPEN — releasing ball")
    valve_open()
    time.sleep(RELEASE_DELAY)

    print("  [3/4] Valve CLOSED — stopping airflow")
    valve_close()

    print("  [4/4] Servo UP — retracting")
    servo_up()
    print("  ✓ Ball placed!\n")


def full_cycle():
    """Run a complete pick → (pause) → place sequence."""
    print("── PICK ──")
    pick()
    wait = 2.0
    print(f"  Holding for {wait}s (move to drop location if needed) …")
    time.sleep(wait)
    print("── PLACE ──")
    place()
    print("── Cycle complete ──\n")


# ── Lifecycle ──────────────────────────────────────────────────────────────────
def cleanup_and_exit(sig=None, frame=None):
    print("\nShutting down — valve closed, servo up")
    valve_close()
    servo_up()
    GPIO.cleanup()
    sys.exit(0)


def main():
    global kit

    # ── Servo (init first — Blinka sets its own GPIO mode internally) ──
    kit = ServoKit(channels=16)
    kit.servo[SERVO_CHANNEL].actuation_range = 180
    kit.servo[SERVO_CHANNEL].angle = SERVO_UP  # start retracted

    # ── GPIO (valve) — reset mode after Blinka touched it ──
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(VALVE_PIN, GPIO.OUT, initial=GPIO.HIGH)  # valve closed at boot
    kit.servo[SERVO_CHANNEL].actuation_range = 180
    kit.servo[SERVO_CHANNEL].angle = SERVO_UP  # start retracted

    signal.signal(signal.SIGINT, cleanup_and_exit)
    signal.signal(signal.SIGTERM, cleanup_and_exit)

    print("=" * 60)
    print("  Pick & Place Test — Servo + Solenoid Valve")
    print("=" * 60)
    print(f"  Servo    : PCA9685 ch {SERVO_CHANNEL}  (0°=down, 180°=up)")
    print(f"  Valve    : BOARD pin {VALVE_PIN}  (inverted logic)")
    print(f"  Initial  : Servo UP, Valve CLOSED")
    print("-" * 60)
    print("Commands:")
    print("  p  → PICK   (down, suction on, lift)")
    print("  d  → PLACE  (down, release, retract)")
    print("  f  → FULL CYCLE  (pick → hold → place)")
    print("  ─── manual overrides ───")
    print("  1  → Servo DOWN only")
    print("  2  → Servo UP only")
    print("  3  → Valve CLOSE (suction on)")
    print("  4  → Valve OPEN  (suction off)")
    print("  s  → Status")
    print("  q  → Quit")
    print("-" * 60)

    try:
        while True:
            cmd = input(">> ").strip().lower()

            if cmd == "p":
                print("── PICK ──")
                pick()

            elif cmd == "d":
                print("── PLACE ──")
                place()

            elif cmd == "f":
                full_cycle()

            elif cmd == "1":
                servo_down()
                print("Servo DOWN (0°)")

            elif cmd == "2":
                servo_up()
                print("Servo UP (180°)")

            elif cmd == "3":
                valve_close()
                print("Valve CLOSED (suction on)")

            elif cmd == "4":
                valve_open()
                print("Valve OPEN (suction off)")

            elif cmd == "s":
                pin_raw = GPIO.input(VALVE_PIN)
                valve_state = "CLOSED (suction on)" if pin_raw else "OPEN (suction off)"
                servo_angle = kit.servo[SERVO_CHANNEL].angle
                print(f"  Servo  : {servo_angle}°")
                print(f"  Valve  : {valve_state}  (pin raw={pin_raw})")

            elif cmd == "q":
                break

            else:
                print("Unknown command. Use p / d / f / 1 / 2 / 3 / 4 / s / q")

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        cleanup_and_exit()


if __name__ == "__main__":
    main()
