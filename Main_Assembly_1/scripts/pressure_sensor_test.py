#!/usr/bin/env python3
"""Raspberry Pi test script for HX710/HX711-style pressure modules.

Expected wiring for your current setup:
- Sensor VCC -> Pi 5V
- Sensor GND -> Pi GND
- Sensor SCK -> GPIO11 (physical pin 23)
- Sensor OUT -> voltage divider midpoint -> GPIO27 (physical pin 13)
  Divider example with your parts: OUT --10k-- node --10k-- GND
"""

import time

import RPi.GPIO as GPIO

PIN_SCK = 11
PIN_OUT = 27
READY_TIMEOUT_S = 2.0
ZERO_SAMPLES = 40

# Ball-state thresholds from observed data:
#   picked   ~131000
#   released ~400000 to 800000
PICKED_MAX = 250000
RELEASED_MIN = 350000
STATE_CONFIRM_SAMPLES = 3


def wait_for_ready(timeout_s=READY_TIMEOUT_S):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if GPIO.input(PIN_OUT) == GPIO.LOW:
            return True
        time.sleep(0.001)
    return False


def read_raw_24():
    if not wait_for_ready():
        raise TimeoutError("OUT stayed HIGH")

    value = 0
    for _ in range(24):
        # Pulse SCK and sample after falling edge. This keeps SCK-high
        # time shorter in Python and is more robust with HX-style devices.
        GPIO.output(PIN_SCK, GPIO.HIGH)
        GPIO.output(PIN_SCK, GPIO.LOW)
        value = (value << 1) | (1 if GPIO.input(PIN_OUT) else 0)

    GPIO.output(PIN_SCK, GPIO.HIGH)
    GPIO.output(PIN_SCK, GPIO.LOW)

    if value & 0x800000:
        value -= (1 << 24)
    return value


def average_raw(samples=ZERO_SAMPLES):
    vals = []
    for _ in range(samples):
        vals.append(read_raw_24())
        time.sleep(0.02)
    return int(sum(vals) / len(vals))


def classify_ball(raw_u24):
    if raw_u24 <= PICKED_MAX:
        return "PICKED"
    if raw_u24 >= RELEASED_MIN:
        return "NOT_PICKED"
    return None


def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_SCK, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_OUT, GPIO.IN, pull_up_down=GPIO.PUD_OFF)


def main():
    setup_gpio()
    print("Pressure sensor Pi test starting")
    print(f"SCK=GPIO{PIN_SCK} (pin 23), OUT=GPIO{PIN_OUT} (pin 13)")
    print(f"Initial OUT level: {GPIO.input(PIN_OUT)} (1=HIGH, 0=LOW)")
    print("Keep sensor at zero pressure for startup zeroing...")

    try:
        zero = average_raw(ZERO_SAMPLES)
        print(f"zero={zero}")
        print("Reading... Ctrl+C to stop")
        print(
            f"Ball thresholds: PICKED<={PICKED_MAX}, NOT_PICKED>={RELEASED_MIN}"
        )

        ones_count = 0
        zeros_count = 0
        ball_state = "UNKNOWN"
        pending_state = None
        pending_count = 0

        while True:
            try:
                raw = read_raw_24()
            except TimeoutError:
                print("TIMEOUT: OUT stayed HIGH (check VCC/GND/SCK/OUT divider)")
                time.sleep(1.0)
                continue

            delta = raw - zero
            raw_u24 = raw & 0xFFFFFF

            candidate = classify_ball(raw_u24)
            if candidate is None:
                pending_state = None
                pending_count = 0
            else:
                if candidate == pending_state:
                    pending_count += 1
                else:
                    pending_state = candidate
                    pending_count = 1

                if pending_count >= STATE_CONFIRM_SAMPLES and ball_state != candidate:
                    ball_state = candidate
                    print(f"STATE CHANGE -> {ball_state}")

            print(
                f"raw={raw} raw_u24={raw_u24} zero={zero} delta={delta} state={ball_state}"
            )

            if raw == -1:
                ones_count += 1
            else:
                ones_count = 0

            if raw == 0:
                zeros_count += 1
            else:
                zeros_count = 0

            if ones_count >= 5:
                print("WARNING: repeated -1 (all 1 bits).")
                print("Check: OUT divider node really goes to GPIO27, and SCK is on GPIO11 pin 23.")
                print("If wiring is correct, SCK 3.3V may be too low for this 5V module.")
            elif zeros_count >= 5:
                print("WARNING: repeated 0 (all 0 bits).")
                print("Check: SCK/OUT may be swapped or OUT node may be shorted to GND.")

            if raw == -8388608:
                print("WARNING: sensor at negative saturation (0x800000)")
            elif raw == 8388607:
                print("WARNING: sensor at positive saturation (0x7FFFFF)")

            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
