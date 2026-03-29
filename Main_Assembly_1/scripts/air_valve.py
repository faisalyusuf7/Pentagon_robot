#!/usr/bin/env python3
"""
Jetson GPIO control for 5 V solenoid valve via MOSFET on pin 15.

Circuit (low-side N-channel MOSFET switch):
  - Solenoid: one terminal → 5 V supply, other terminal → MOSFET drain
  - MOSFET source → GND (shared with Jetson)
  - MOSFET gate  → 220 Ω → Jetson GPIO pin
  - 10 kΩ pull-down on gate → GND
  - Flyback diode across solenoid (cathode on 5 V side)

Inverted logic (via 2N2222 + IRLZ44N):
  GPIO HIGH → valve CLOSED (suction on)
  GPIO LOW  → valve OPEN  (suction off)
"""

import signal
import sys
import time

import RPi.GPIO as GPIO

VALVE_PIN = 15  # BCM pin 22 (= BOARD pin 15) — wired through 220 Ω to MOSFET gate

def cleanup_and_exit(sig=None, frame=None):
    print("\nShutting down — closing valve")
    GPIO.output(VALVE_PIN, GPIO.HIGH)
    GPIO.cleanup()
    sys.exit(0)


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(VALVE_PIN, GPIO.OUT, initial=GPIO.HIGH)

    # Ensure clean shutdown on Ctrl-C or SIGTERM
    signal.signal(signal.SIGINT, cleanup_and_exit)
    signal.signal(signal.SIGTERM, cleanup_and_exit)

    print(f"Valve control on BOARD pin {VALVE_PIN}")
    print("Press Ctrl-C to stop\n")

    print("Commands:")
    print("  o  + ENTER → open valve")
    print("  c  + ENTER → close valve")
    print("  t  + ENTER → toggle (1 s open / 1 s closed loop)")
    print("  q  + ENTER → quit\n")

    try:
        while True:
            cmd = input(">> ").strip().lower()

            if cmd == "o":
                GPIO.output(VALVE_PIN, GPIO.LOW)
                print("Valve OPEN")

            elif cmd == "c":
                GPIO.output(VALVE_PIN, GPIO.HIGH)
                print("Valve CLOSED")

            elif cmd == "t":
                print("Toggling — press Ctrl-C to stop")
                while True:
                    GPIO.output(VALVE_PIN, GPIO.LOW)
                    print("  OPEN")
                    time.sleep(1.0)
                    GPIO.output(VALVE_PIN, GPIO.HIGH)
                    print("  CLOSED")
                    time.sleep(1.0)

            elif cmd == "q":
                break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        cleanup_and_exit()


if __name__ == "__main__":
    main()