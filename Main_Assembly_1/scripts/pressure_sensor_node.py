#!/usr/bin/env python3
"""
ROS 2 Pressure-Sensor Node
===========================

Reads an HX710/HX711-style barometric pressure module wired to
Raspberry Pi GPIO and publishes ball-detection state for the
pick-and-place planner.

Important: this setup is classified using the unsigned 24-bit count
(`raw_u24`), not the sign-extended signed value. On this hardware, a
picked ball drives the reading into the upper part of the 24-bit range,
which must remain visible as a large negative `drop = zero - raw_u24`.

Wiring (same as pressure_sensor_test.py):
  VCC  → Pi 5 V
  GND  → Pi GND
  SCK  → GPIO11 (physical pin 23)
  OUT  → 10 k + 10 k voltage divider → GPIO27 (physical pin 13)

Published topics
----------------
  /ball_detected   std_msgs/Bool    True when ball is gripped
    /pressure_raw    std_msgs/Int32   unsigned 24-bit raw reading

Parameters
----------
  gpio_sck         (int)   BCM pin for SCK           [11]
  gpio_out         (int)   BCM pin for OUT            [27]
    picked_drop_max  (int)   drop <= this → PICKED     [-5000000]
    released_drop_min(int)   drop >= this → NOT_PICKED [-4000000]
    confirm_samples  (int)   matching samples in window   [2]
    sample_window    (int)   recent valid samples kept    [5]
  read_hz          (float) sensor poll rate            [10.0]
  zero_samples     (int)   readings for startup zero   [40]
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

import RPi.GPIO as GPIO


class PressureSensorNode(Node):

    def __init__(self):
        super().__init__("pressure_sensor_node")

        # ---------- parameters ----------
        self.declare_parameter("gpio_sck", 11)
        self.declare_parameter("gpio_out", 27)
        self.declare_parameter("picked_drop_max", -5_000_000)
        self.declare_parameter("released_drop_min", -4_000_000)
        self.declare_parameter("confirm_samples", 2)
        self.declare_parameter("sample_window", 5)
        self.declare_parameter("read_hz", 10.0)
        self.declare_parameter("zero_samples", 40)
        self.declare_parameter("log_every_sample", True)

        self._pin_sck = int(self.get_parameter("gpio_sck").value)
        self._pin_out = int(self.get_parameter("gpio_out").value)
        self._picked_drop_max = int(self.get_parameter("picked_drop_max").value)
        self._released_drop_min = int(self.get_parameter("released_drop_min").value)
        self._confirm = int(self.get_parameter("confirm_samples").value)
        self._sample_window = max(
            self._confirm,
            int(self.get_parameter("sample_window").value),
        )
        self._log_every_sample = bool(self.get_parameter("log_every_sample").value)
        hz = float(self.get_parameter("read_hz").value)
        zero_n = int(self.get_parameter("zero_samples").value)

        # ---------- GPIO ----------
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin_sck, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._pin_out, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

        # ---------- publishers ----------
        self._pub_detected = self.create_publisher(Bool, "/ball_detected", 10)
        self._pub_raw = self.create_publisher(Int32, "/pressure_raw", 10)

        # ---------- state ----------
        self._ball_detected = False
        self._zero = 0
        self._log_counter = 0  # for periodic raw logging
        self._recent_candidates = deque(maxlen=self._sample_window)
        self._invalid_log_counter = 0

        # ---------- startup zero ----------
        self.get_logger().info(
            f"Pressure sensor: SCK=GPIO{self._pin_sck}, OUT=GPIO{self._pin_out}"
        )
        self.get_logger().info(f"Zeroing with {zero_n} samples …")
        self._zero = self._average_raw_u24(zero_n)
        self.get_logger().info(f"Zero = {self._zero}")

        # ---------- timer ----------
        self._timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"Pressure sensor node running at {hz:.0f} Hz  "
            f"(zero={self._zero}, PICKED if drop <= {self._picked_drop_max}, "
            f"NOT_PICKED if drop >= {self._released_drop_min}, "
            f"window={self._sample_window}, confirm={self._confirm})"
        )

    # ---------------------------------------------------------- #
    #  Low-level sensor read (identical to pressure_sensor_test)
    # ---------------------------------------------------------- #
    def _wait_ready(self, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if GPIO.input(self._pin_out) == GPIO.LOW:
                return True
            time.sleep(0.001)
        return False

    def _read_raw24(self):
        if not self._wait_ready():
            return None

        value = 0
        for _ in range(24):
            GPIO.output(self._pin_sck, GPIO.HIGH)
            GPIO.output(self._pin_sck, GPIO.LOW)
            value = (value << 1) | (1 if GPIO.input(self._pin_out) else 0)

        # 25th clock pulse (gain = 128 / next channel select)
        GPIO.output(self._pin_sck, GPIO.HIGH)
        GPIO.output(self._pin_sck, GPIO.LOW)

        # sign-extend 24-bit
        if value & 0x800000:
            value -= 1 << 24
        return value

    def _average_raw_u24(self, n):
        vals = []
        for _ in range(n):
            r = self._read_raw24()
            if r is not None:
                vals.append(r & 0xFFFFFF)
            time.sleep(0.02)
        return int(sum(vals) / len(vals)) if vals else 0

    # ---------------------------------------------------------- #
    #  Periodic tick
    # ---------------------------------------------------------- #
    def _tick(self):
        raw = self._read_raw24()
        if raw is None:
            self.get_logger().warn("Sensor timeout (OUT stayed HIGH)")
            return

        raw_u24 = raw & 0xFFFFFF
        drop = self._zero - raw_u24

        if self._is_invalid_sample(raw_u24):
            self._invalid_log_counter += 1
            if self._invalid_log_counter >= 5:
                self._invalid_log_counter = 0
                self.get_logger().warn(
                    f"Ignoring invalid pressure sample raw_u24={raw_u24} drop={drop}"
                )
            return

        self._invalid_log_counter = 0

        # publish raw
        msg_raw = Int32()
        msg_raw.data = raw_u24
        self._pub_raw.publish(msg_raw)

        if self._log_every_sample:
            self.get_logger().info(
                f"RAW={raw_u24}  zero={self._zero}  drop={drop}  "
                f"detected={self._ball_detected}"
            )
        else:
            # periodic raw logging (every ~1 second)
            self._log_counter += 1
            if self._log_counter >= int(float(self.get_parameter("read_hz").value)):
                self._log_counter = 0
                self.get_logger().info(
                    f"RAW={raw_u24}  zero={self._zero}  drop={drop}  "
                    f"detected={self._ball_detected}"
                )

        candidate = self._classify(drop)
        self._recent_candidates.append(candidate)

        picked_votes = sum(1 for state in self._recent_candidates if state == "PICKED")
        released_votes = sum(1 for state in self._recent_candidates if state == "NOT_PICKED")

        new_detected = self._ball_detected
        if picked_votes >= self._confirm:
            new_detected = True
        elif released_votes >= self._confirm:
            new_detected = False

        if new_detected != self._ball_detected:
            self._ball_detected = new_detected
            self.get_logger().info(
                f"Ball state → {'PICKED' if new_detected else 'NOT_PICKED'}  "
                f"(raw_u24={raw_u24}, drop={drop}, votes={list(self._recent_candidates)})"
            )

        # publish detection
        msg = Bool()
        msg.data = self._ball_detected
        self._pub_detected.publish(msg)

    @staticmethod
    def _is_invalid_sample(raw_u24):
        return raw_u24 == 0 or raw_u24 == 0xFFFFFF

    def _classify(self, drop):
        if drop <= self._picked_drop_max:
            return "PICKED"
        if drop >= self._released_drop_min:
            return "NOT_PICKED"
        return None

    # ---------------------------------------------------------- #
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = PressureSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
