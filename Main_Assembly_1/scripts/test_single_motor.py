#!/usr/bin/env python3
"""
Single Motor Test for Arduino 5-bar driver.

Purpose:
- Move ONLY one motor at a time (left=A or right=B)
- Verify direction and safe range before full five_bar launch

Usage:
  python3 test_single_motor.py /dev/ttyACM0
  python3 test_single_motor.py                # auto-detect port

Serial protocol expected (stepper_angle_driver.ino):
- A<deg>   move left motor only
- B<deg>   move right motor only
- M114     read current positions
- M17/M18  enable/disable motors
- G28      home
"""

import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not installed. Run: pip install pyserial")
    raise SystemExit(1)


def auto_detect_port() -> str | None:
    keywords = ["Arduino", "CH340", "ttyACM", "ttyUSB"]
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = f"{p.description} {p.device}"
        if any(k.lower() in desc.lower() for k in keywords):
            return p.device
    return ports[0].device if ports else None


def read_available(ser: serial.Serial, timeout_s: float = 0.25) -> list[str]:
    t0 = time.time()
    lines: list[str] = []
    while time.time() - t0 < timeout_s:
        if ser.in_waiting:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                lines.append(line)
        else:
            time.sleep(0.01)
    return lines


def send(ser: serial.Serial, cmd: str):
    print(f"-> {cmd}")
    ser.write((cmd + "\n").encode())
    ser.flush()
    lines = read_available(ser, timeout_s=0.35)
    for line in lines:
        print(f"<- {line}")


def send_and_wait_for(ser: serial.Serial, cmd: str, tokens: list[str], timeout_s: float = 8.0):
    """Send command and block until one of tokens appears in serial output."""
    print(f"-> {cmd}")
    ser.write((cmd + "\n").encode())
    ser.flush()

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if ser.in_waiting:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(f"<- {line}")
                for tok in tokens:
                    if tok in line:
                        return line
        else:
            time.sleep(0.01)
    print(f"<- TIMEOUT waiting for: {tokens}")
    return ""


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else auto_detect_port()
    if not port:
        print("No serial port detected. Pass one explicitly, e.g. /dev/ttyACM0")
        raise SystemExit(1)

    print(f"Connecting to {port} @ 115200...")
    ser = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(2.0)
    ser.reset_input_buffer()

    print("Connected. Initial output:")
    for line in read_available(ser, timeout_s=1.0):
        print(f"<- {line}")

    send(ser, "M17")
    send(ser, "M114")

    print("\nSingle-motor test commands:")
    print("  l <deg>   move LEFT motor only, e.g. l 10")
    print("  r <deg>   move RIGHT motor only, e.g. r -10")
    print("  d <deg>   delta move left by +deg and right by -deg from current")
    print("  home      send G28")
    print("  m119      read endstop switch states")
    print("  m114      query position")
    print("  m17/m18   enable/disable")
    print("  q         quit")
    print("\nSafety: start with very small values (2 to 5 deg).")

    left = 0.0
    right = 0.0

    while True:
        try:
            inp = input("test> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not inp:
            continue

        parts = inp.split()
        cmd = parts[0].lower()

        if cmd == "q":
            break

        if cmd == "m114":
            send(ser, "M114")
            continue

        if cmd == "m119":
            send(ser, "M119")
            continue

        if cmd == "m17":
            send(ser, "M17")
            continue

        if cmd == "m18":
            send(ser, "M18")
            continue

        if cmd == "home":
            result = send_and_wait_for(ser, "G28", ["OK HOME", "ERR HOME"], timeout_s=20.0)
            # If G28 failed, still drain motion queue and report position.
            if "ERR HOME" in result:
                print("Homing failed: check endstop wiring/state and homing direction.")
            send_and_wait_for(ser, "M400", ["OK IDLE"], timeout_s=15.0)
            send(ser, "M114")
            left = 0.0
            right = 0.0
            continue

        if cmd in ("l", "r"):
            if len(parts) != 2:
                print("Usage: l <deg>  or  r <deg>")
                continue
            try:
                val = float(parts[1])
            except ValueError:
                print("Invalid number")
                continue

            if cmd == "l":
                left = val
                send(ser, f"A{left:.2f}")
            else:
                right = val
                send(ser, f"B{right:.2f}")

            send(ser, "M400")
            send(ser, "M114")
            continue

        if cmd == "d":
            if len(parts) != 2:
                print("Usage: d <deg>")
                continue
            try:
                delta = float(parts[1])
            except ValueError:
                print("Invalid number")
                continue

            left += delta
            right -= delta
            send(ser, f"A{left:.2f}")
            send(ser, f"B{right:.2f}")
            send(ser, "M400")
            send(ser, "M114")
            continue

        print("Unknown command")

    send(ser, "M18")
    ser.close()
    print("Done.")


if __name__ == "__main__":
    main()
