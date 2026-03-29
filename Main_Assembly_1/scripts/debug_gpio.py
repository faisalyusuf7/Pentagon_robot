#!/usr/bin/env python3
"""
Jetson GPIO test for 5 V solenoid valve — INVERTED logic via 2N2222 + IRLZ44N.

Circuit (2N2222 level-shifter → IRLZ44N low-side switch):
  Power path:
    +5 V supply → Solenoid(+) → Solenoid(−) → MOSFET Drain
    MOSFET Source → GND
    Flyback diode across solenoid (cathode → +5 V, anode → drain)

  Gate drive (level shifting):
    +5 V → 10 kΩ pull-up → MOSFET Gate
    2N2222 Collector → 220 Ω → MOSFET Gate
    2N2222 Emitter  → GND

  Base drive (GPIO control):
    GPIO pin → 1 kΩ → 2N2222 Base
    10 kΩ pull-down: Base → GND (boot safety)

  *** INVERTED LOGIC ***
    GPIO LOW  → 2N2222 OFF → Gate pulled HIGH (5 V) → MOSFET ON  → Valve ON
    GPIO HIGH → 2N2222 ON  → Gate pulled LOW  (0 V) → MOSFET OFF → Valve OFF
"""

import signal
import sys
import time

import Jetson.GPIO as GPIO

# ── Configuration ──────────────────────────────────────────────────────────────
VALVE_PIN = 15          # BOARD pin 15 — wired through 1 kΩ to 2N2222 base
TOGGLE_PERIOD = 1.0     # seconds ON / OFF during toggle test

# ── Inverted-logic helpers ─────────────────────────────────────────────────────
# Because of the 2N2222 inverter stage the electrical sense is flipped:
#   valve_on()  → GPIO LOW   (2N2222 off → gate high → MOSFET on)
#   valve_off() → GPIO HIGH  (2N2222 on  → gate low  → MOSFET off)

def valve_on():
    """Energise the solenoid (GPIO LOW → MOSFET ON)."""
    GPIO.output(VALVE_PIN, GPIO.LOW)

def valve_off():
    """De-energise the solenoid (GPIO HIGH → MOSFET OFF)."""
    GPIO.output(VALVE_PIN, GPIO.HIGH)


# ── Lifecycle ──────────────────────────────────────────────────────────────────
def cleanup_and_exit(sig=None, frame=None):
    """Ensure valve is OFF and GPIO is released on exit."""
    print("\nShutting down — closing valve (GPIO → HIGH)")
    valve_off()
    GPIO.cleanup()
    sys.exit(0)


def main():
    GPIO.setmode(GPIO.BOARD)
    # Boot safe: initial=HIGH keeps valve OFF until explicitly commanded
    GPIO.setup(VALVE_PIN, GPIO.OUT, initial=GPIO.HIGH)

    signal.signal(signal.SIGINT, cleanup_and_exit)
    signal.signal(signal.SIGTERM, cleanup_and_exit)

    print("=" * 60)
    print("  Solenoid Valve Test — Inverted Logic (2N2222 + IRLZ44N)")
    print("=" * 60)
    print(f"  Control pin : BOARD {VALVE_PIN}")
    print(f"  Logic       : GPIO LOW = Valve ON, GPIO HIGH = Valve OFF")
    print(f"  Initial     : Valve OFF (GPIO HIGH)")
    print("-" * 60)
    print("Commands:")
    print("  o  → Open valve   (energise solenoid)")
    print("  c  → Close valve  (de-energise solenoid)")
    print("  p  → Pulse test   (open 0.5 s, then close)")
    print("  t  → Toggle loop  (1 s on / 1 s off, Ctrl-C to stop)")
    print("  s  → Status       (read current pin state)")
    print("  q  → Quit")
    print("-" * 60)

    try:
        while True:
            cmd = input(">> ").strip().lower()

            if cmd == "o":
                valve_on()
                print("Valve OPEN  (GPIO LOW → MOSFET ON)")

            elif cmd == "c":
                valve_off()
                print("Valve CLOSED (GPIO HIGH → MOSFET OFF)")

            elif cmd == "p":
                print("Pulse: opening for 0.5 s …")
                valve_on()
                time.sleep(0.5)
                valve_off()
                print("Pulse complete — valve closed")

            elif cmd == "t":
                print(f"Toggling every {TOGGLE_PERIOD}s — Ctrl-C to stop")
                try:
                    while True:
                        valve_on()
                        print("  OPEN  (GPIO LOW)")
                        time.sleep(TOGGLE_PERIOD)
                        valve_off()
                        print("  CLOSED (GPIO HIGH)")
                        time.sleep(TOGGLE_PERIOD)
                except KeyboardInterrupt:
                    valve_off()
                    print("\nToggle stopped — valve closed")

            elif cmd == "s":
                raw = GPIO.input(VALVE_PIN)
                state = "OFF (GPIO HIGH)" if raw else "ON (GPIO LOW)"
                print(f"Pin {VALVE_PIN} raw={raw} → Valve {state}")

            elif cmd == "q":
                break

            else:
                print("Unknown command. Use o / c / p / t / s / q")

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        cleanup_and_exit()


if __name__ == "__main__":
    main()