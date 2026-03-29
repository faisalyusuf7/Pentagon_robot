#!/usr/bin/env python3
"""Read pressure sensor lines from Arduino over USB serial.

Use this when the sensor runs at 5V and Raspberry Pi GPIO cannot safely/cleanly
clock the module directly.
"""

import sys
import time

try:
    import serial
except ImportError:
    print("Missing pyserial. Install with: sudo apt install python3-serial")
    sys.exit(1)

PORT = "/dev/ttyACM0"
BAUD = 9600


def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            # Give board a moment after opening port.
            time.sleep(2.0)
            print("Reading lines (Ctrl+C to stop):")
            while True:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(line)
    except FileNotFoundError:
        print(f"Port not found: {PORT}")
        print("Check: ls /dev/ttyACM* /dev/ttyUSB*")
        sys.exit(1)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()
