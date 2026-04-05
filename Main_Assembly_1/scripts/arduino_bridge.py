#!/usr/bin/env python3
"""
ROS 2 ↔ Arduino Serial Bridge for 5-Bar Stepper Motors
=======================================================
Subscribes to /joint_states from the IK node, extracts the two motor
URDF angles, converts them to stepper degrees, and sends them to the
Arduino over serial.

Angle convention
----------------
    URDF:      θ_urdf = yaw  when crank points +Y  (home)
    Stepper:   θ_step = 0    when crank points up  (home)
    Mapping:   θ_step = −degrees(θ_urdf − yaw_offset)

Serial protocol  (to Arduino)
------------------------------
    A<left_deg> B<right_deg>\n     → move both motors
    G28\n                          → home (0°/0°)
    M114\n                         → query position
    M400\n                         → wait until idle
"""

import math
import serial
import serial.tools.list_ports
import time
import threading

import re

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool


class ArduinoBridge(Node):

    # Joint names from V3 URDF (must match five_bar_ik_node.py ALL_JOINTS)
    MOTOR_LEFT_JOINT  = "Joint_2"
    MOTOR_RIGHT_JOINT = "Joint_1"

    # URDF angle at physical home (crank up) = motor mounting yaw from URDF rpy
    _YAW_LEFT  =  0.37069   # Joint_2 yaw
    _YAW_RIGHT = -0.40717   # Joint_1 yaw

    def __init__(self):
        super().__init__("arduino_bridge")

        # --- parameters ---
        self.declare_parameter("serial_port", "")          # auto-detect if empty
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("send_rate", 30.0)          # Hz (max update rate)
        self.declare_parameter("auto_detect_keywords",
                               ["Arduino", "CH340", "USB2.0-Ser", "ttyUSB", "ttyACM"])

        port  = self.get_parameter("serial_port").value
        baud  = self.get_parameter("baud_rate").value
        rate  = self.get_parameter("send_rate").value
        kw    = self.get_parameter("auto_detect_keywords").value

        # --- serial connection ---
        self._ser = None
        self._ser_lock = threading.Lock()

        # --- publishers (must be created before _connect which calls _publish_status) ---
        self._status_pub    = self.create_publisher(String,     "/arduino_status",   10)
        self._feedback_pub  = self.create_publisher(JointState, "/motor_feedback",   10)

        self._connect(port, baud, kw)

        # --- latest commanded angles (stepper degrees) ---
        self._last_sent_L = None
        self._last_sent_R = None
        self._pending_L = 0.0
        self._pending_R = 0.0
        self._dirty = False           # new angles not yet sent
        self._angle_tol = 0.05        # degrees — don't resend if change < this

        # --- subscribers ---
        self.create_subscription(JointState, "/joint_states",
                                 self._cb_joint_states, 10)
        self.create_subscription(Bool, "/arduino_enable",
                                 self._cb_enable, 10)
        self.create_subscription(String, "/arduino_raw_cmd",
                                 self._cb_raw_cmd, 10)

        # --- send timer ---
        self._send_timer = self.create_timer(1.0 / rate, self._send_tick)

        # --- serial reader thread ---
        self._reader_thread = threading.Thread(target=self._serial_reader,
                                               daemon=True)
        self._reader_thread.start()

        self._publish_status("Bridge ready — waiting for /joint_states")
        self.get_logger().info(
            f"Arduino bridge node started  port={port or 'auto'}  "
            f"baud={baud}  rate={rate} Hz"
        )

    # ========================================================= #
    #  Serial connection
    # ========================================================= #
    def _connect(self, port, baud, keywords):
        if not port:
            port = self._auto_detect(keywords)
        if not port:
            self.get_logger().warn(
                "No serial port found — bridge will publish but NOT send. "
                "Set serial_port param or connect Arduino.")
            return

        try:
            self._ser = serial.Serial(port, baud, timeout=1.0)
            time.sleep(2.5)          # wait for Arduino reset
            # drain any startup garbage
            self._ser.reset_input_buffer()
            self.get_logger().info(f"Serial opened: {port} @ {baud}")
            self._publish_status(f"Serial connected: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self._ser = None

    @staticmethod
    def _auto_detect(keywords):
        for p in serial.tools.list_ports.comports():
            desc = f"{p.description} {p.device}"
            for kw in keywords:
                if kw.lower() in desc.lower():
                    return p.device
        return None

    def _serial_send(self, cmd: str):
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write((cmd + "\n").encode())
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial write error: {e}")

    # Regex to parse  POS A:85.23 B:90.00
    _POS_RE = re.compile(r"POS A:([\-\d.]+)\s+B:([\-\d.]+)")

    def _serial_reader(self):
        """Background thread: read lines from Arduino and log them."""
        while rclpy.ok():
            with self._ser_lock:
                ser = self._ser
            if ser is None or not ser.is_open:
                time.sleep(0.5)
                continue
            try:
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    continue
                self.get_logger().info(f"[Arduino] {line}")
                self._publish_status(f"[Arduino] {line}")
                # Parse position feedback and republish as JointState
                m = self._POS_RE.search(line)
                if m:
                    step_L = float(m.group(1))
                    step_R = float(m.group(2))
                    # Convert stepper degrees back to URDF radians.
                    # step = -degrees(urdf - yaw)  →  urdf = -radians(step) + yaw
                    urdf_L = math.radians(-step_L) + self._YAW_LEFT
                    urdf_R = math.radians(-step_R) + self._YAW_RIGHT
                    fb = JointState()
                    fb.header.stamp = self.get_clock().now().to_msg()
                    fb.name     = [self.MOTOR_LEFT_JOINT, self.MOTOR_RIGHT_JOINT]
                    fb.position = [urdf_L, urdf_R]
                    self._feedback_pub.publish(fb)
            except Exception:
                time.sleep(0.1)

    # ========================================================= #
    #  Callbacks
    # ========================================================= #
    def _cb_joint_states(self, msg: JointState):
        """Extract motor URDF angles → convert to stepper degrees."""
        try:
            idx_L = msg.name.index(self.MOTOR_LEFT_JOINT)
            idx_R = msg.name.index(self.MOTOR_RIGHT_JOINT)
        except ValueError:
            return  # not our joints

        urdf_L = msg.position[idx_L]   # radians
        urdf_R = msg.position[idx_R]

        # URDF → stepper degrees:  step = -degrees(urdf - yaw_offset)
        # At home (urdf = yaw), step = 0°
        step_L = -math.degrees(urdf_L - self._YAW_LEFT)
        step_R = -math.degrees(urdf_R - self._YAW_RIGHT)

        self._pending_L = step_L
        self._pending_R = step_R
        self._dirty = True

    def _cb_enable(self, msg: Bool):
        """Enable / disable motors via M17 / M18."""
        self._serial_send("M17" if msg.data else "M18")

    def _cb_raw_cmd(self, msg: String):
        """Send an arbitrary command directly to the Arduino (e.g. G28, M114)."""
        cmd = msg.data.strip()
        if cmd:
            self.get_logger().info(f"Raw command → Arduino: {cmd}")
            self._serial_send(cmd)
            # If it's a home command, reset our cached angles
            if cmd.upper() == "G28":
                self._last_sent_L = 0.0
                self._last_sent_R = 0.0
                self._pending_L = 0.0
                self._pending_R = 0.0
                self._dirty = False

    # ========================================================= # #
    #  Periodic sender (rate-limited)
    # ========================================================= #
    def _send_tick(self):
        if not self._dirty:
            return

        L, R = self._pending_L, self._pending_R

        # Skip if angles haven't changed enough
        if (self._last_sent_L is not None and
            abs(L - self._last_sent_L) < self._angle_tol and
            abs(R - self._last_sent_R) < self._angle_tol):
            self._dirty = False
            return

        cmd = f"A{L:.2f} B{R:.2f}"
        self._serial_send(cmd)

        self._last_sent_L = L
        self._last_sent_R = R
        self._dirty = False

    # ========================================================= #
    #  Helpers
    # ========================================================= #
    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _shutdown_motors(self):
        """Send M18 to disable motors and close serial on shutdown."""
        if self._ser and self._ser.is_open:
            try:
                self.get_logger().info("Sending M18 — disabling motors...")
                self._ser.write(b"M18\n")
                self._ser.flush()
                time.sleep(0.2)
                self._ser.close()
                self.get_logger().info("Serial closed, motors disabled.")
            except Exception as e:
                self.get_logger().warn(f"Shutdown serial error: {e}")


def main():
    rclpy.init()
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
