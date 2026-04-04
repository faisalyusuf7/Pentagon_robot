#!/usr/bin/env python3
"""
MKS SERVO42C Bridge — Drop-in replacement for arduino_bridge.py
================================================================
Identical ROS2 interface to the original Arduino serial bridge,
but drives two MKS SERVO42C closed-loop stepper motors directly
via UART (no Arduino needed).

Subscribed (same as arduino_bridge.py):
  /joint_states      (sensor_msgs/JointState)  — from five_bar_ik_node
  /arduino_enable    (std_msgs/Bool)            — enable / disable torque
  /arduino_raw_cmd   (std_msgs/String)          — "G28" home, etc.

Published (same as arduino_bridge.py):
  /arduino_status    (std_msgs/String)          — status / diagnostics

Extra (new):
  /motor_encoder_fb  (sensor_msgs/JointState)   — real encoder angles (14-bit)

Angle convention (unchanged from arduino_bridge.py):
  URDF:    θ_urdf = 0  when crank points +Y  (home)
  Stepper: θ_step = 90 when crank points +Y  (home)
  Mapping: θ_step = 90 − degrees(θ_urdf)

Hardware:
  Motor 1 (left,  motor_joint_left):  MKS SERVO42C addr 0x00
  Motor 2 (right, motor_joint_right): MKS SERVO42C addr 0x01
  Connection: USB-TTL adapter → MKS APT board → 6-pin data cable → motor

Parameters:
  serial_port   (str)   "" = auto-detect, or "/dev/ttyUSB0"
  baud_rate     (int)   38400 (must match motor UART config)
  send_rate     (float) 30.0 Hz  — max command update rate
  addr_left     (int)   0       — UART slave address, left motor
  addr_right    (int)   1       — UART slave address, right motor
  subdivision   (int)   16      — microstep setting (must match motor config)
  max_speed_pct (float) 0.4     — 0.0–1.0 fraction of hardware max speed
  home_angle    (float) 90.0    — stepper angle at URDF home (θ_urdf=0)
  angle_tol     (float) 0.05    — deadband: skip move if delta < this (degrees)
  poll_encoder  (bool)  true    — read encoder feedback at poll_rate Hz
  poll_rate     (float) 5.0     — encoder read frequency (Hz)
"""

import math
import sys
import os
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_msgs.msg import Header

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# Protocol lives in the same scripts/ directory
sys.path.insert(0, os.path.dirname(__file__))
from mks_servo42c_protocol import (
    MksServo42cProtocol, Cmd, Mode, MotorStatus, ENCODER_COUNTS_PER_REV
)


# ─────────────────────────── Per-motor state ──────────────────────────────────

class _Motor:
    def __init__(self, name: str, addr: int, home_angle: float, subdivision: int):
        self.name         = name
        self.proto        = MksServo42cProtocol(addr)
        self.subdivision  = subdivision
        self.home_angle   = home_angle          # stepper degrees at URDF θ=0

        # Last angle we actually SENT to the motor (stepper degrees)
        self.sent_deg: float | None = None      # None = not yet sent (= at home)

        # Real encoder angle (stepper degrees, updated by poll)
        self.encoder_deg:  float = home_angle
        self.encoder_counts: int = 0
        self.stall:        bool  = False
        self.last_move_t:  float = 0.0


# ─────────────────────────── Bridge node ─────────────────────────────────────

class MksServo42cBridge(Node):
    """Drop-in replacement for ArduinoBridge using MKS SERVO42C UART."""

    MOTOR_LEFT_JOINT  = "Joint_2"
    MOTOR_RIGHT_JOINT = "Joint_1"

    def __init__(self):
        super().__init__("arduino_bridge")   # keep same node name for compatibility

        # ── Parameters (same names as arduino_bridge.py where applicable) ────
        self.declare_parameter("serial_port",   "")
        self.declare_parameter("baud_rate",     38400)
        self.declare_parameter("send_rate",     30.0)
        # New MKS-specific parameters
        self.declare_parameter("addr_left",     0)
        self.declare_parameter("addr_right",    1)
        self.declare_parameter("subdivision",   16)
        self.declare_parameter("max_speed_pct", 0.4)
        self.declare_parameter("home_angle",    90.0)
        self.declare_parameter("angle_tol",     0.05)
        self.declare_parameter("poll_encoder",  True)
        self.declare_parameter("poll_rate",     5.0)
        self.declare_parameter("auto_home_on_connect", True)
        self.declare_parameter("auto_home_on_enable",  True)

        port       = self.get_parameter("serial_port").value
        baud       = self.get_parameter("baud_rate").value
        send_rate  = self.get_parameter("send_rate").value
        subdiv     = self.get_parameter("subdivision").value
        home       = self.get_parameter("home_angle").value
        speed      = self.get_parameter("max_speed_pct").value
        self._tol  = self.get_parameter("angle_tol").value
        do_poll    = self.get_parameter("poll_encoder").value
        poll_rate  = self.get_parameter("poll_rate").value
        self._auto_home_on_connect = bool(self.get_parameter("auto_home_on_connect").value)
        self._auto_home_on_enable  = bool(self.get_parameter("auto_home_on_enable").value)

        self._speed = max(0.05, min(1.0, speed))

        # ── Motor objects ────────────────────────────────────────────────────
        self._left  = _Motor("left",  self.get_parameter("addr_left").value,
                             home, subdiv)
        self._right = _Motor("right", self.get_parameter("addr_right").value,
                             home, subdiv)

        # ── Serial ───────────────────────────────────────────────────────────
        self._ser   = None
        self._lock  = threading.Lock()

        # ── Publishers ───────────────────────────────────────────────────────
        self._status_pub  = self.create_publisher(String,     "/arduino_status",   10)
        self._encoder_pub = self.create_publisher(JointState, "/motor_encoder_fb", 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(JointState, "/joint_states",
                                 self._cb_joint_states, 10)
        self.create_subscription(Bool,   "/arduino_enable",  self._cb_enable, 10)
        self.create_subscription(String, "/arduino_raw_cmd", self._cb_raw_cmd, 10)

        # ── Pending command state ─────────────────────────────────────────────
        self._pending_L = home
        self._pending_R = home
        self._dirty     = False

        # ── Connect serial ───────────────────────────────────────────────────
        if HAS_SERIAL:
            self._connect(port, baud)
        else:
            self.get_logger().warn(
                "pyserial not installed — simulation mode. "
                "pip install pyserial"
            )

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(1.0 / send_rate,  self._send_tick)
        if do_poll:
            self.create_timer(1.0 / poll_rate, self._poll_tick)

        self._pub_status("MKS SERVO42C bridge ready — waiting for /joint_states")
        self.get_logger().info(
            f"MKS SERVO42C bridge started  port={port or 'auto'}  "
            f"baud={baud}  rate={send_rate} Hz  "
            f"left_addr=0x{self._left.proto.addr:02X}  "
            f"right_addr=0x{self._right.proto.addr:02X}  "
            f"subdivision={subdiv}  speed={speed:.0%}"
        )

    # ── Serial connection ─────────────────────────────────────────────────────

    def _auto_detect(self) -> str:
        keywords = ["USB", "uart", "CH340", "CP210", "FTDI", "Silicon"]
        for p in serial.tools.list_ports.comports():
            desc = p.description.upper() + p.device.upper()
            if any(k.upper() in desc for k in keywords):
                return p.device
        for p in serial.tools.list_ports.comports():
            if "ttyUSB" in p.device or "ttyACM" in p.device:
                return p.device
        return ""

    def _connect(self, port: str, baud: int):
        if not port:
            port = self._auto_detect()
        if not port:
            self.get_logger().warn(
                "No serial port found — running in simulation mode.\n"
                "Set serial_port parameter, e.g. serial_port:=/dev/ttyUSB0"
            )
            return
        try:
            self._ser = serial.Serial(
                port, baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            time.sleep(0.5)
            self._ser.reset_input_buffer()
            self.get_logger().info(f"Serial opened: {port} @ {baud} baud")
            self._pub_status(f"Serial connected: {port}")
            self._init_motors()
            if self._auto_home_on_connect:
                if self._sync_tracking_from_encoder():
                    self._command_home_now(reason="connect")
                else:
                    self.get_logger().warn("Auto-home skipped on connect: encoder read failed")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self._ser = None

    # ── Motor initialisation ──────────────────────────────────────────────────

    def _init_motors(self):
        """Enable torque and set CR_UART mode on both motors."""
        for m in [self._left, self._right]:
            self._tx_rx(m, m.proto.build_enable(True), 4)
            time.sleep(0.05)
            # Mode should already be saved to flash after one-time setup,
            # but send every boot for safety:
            self._tx_rx(m, m.proto.build_set_mode(Mode.CR_UART), 4)
            time.sleep(0.05)
        self.get_logger().info("Motors initialised (enable + CR_UART mode set)")

    # ── Low-level serial ──────────────────────────────────────────────────────

    def _tx(self, motor: _Motor, frame: bytes) -> bool:
        """Send frame; no response."""
        if self._ser is None:
            self.get_logger().debug(f"[SIM] {motor.name} TX: {frame.hex()}")
            return True
        try:
            with self._lock:
                self._ser.write(frame)
                self._ser.flush()
            return True
        except Exception as e:
            self.get_logger().error(f"Serial write error ({motor.name}): {e}")
            return False

    def _tx_rx(self, motor: _Motor, frame: bytes, resp_len: int) -> bytes | None:
        """Send frame and read expected-length response."""
        if self._ser is None:
            self.get_logger().debug(f"[SIM] {motor.name} TX: {frame.hex()}")
            return None
        try:
            with self._lock:
                self._ser.reset_input_buffer()
                self._ser.write(frame)
                self._ser.flush()
                resp = self._ser.read(resp_len)
            return resp if resp else None
        except Exception as e:
            self.get_logger().error(f"Serial I/O error ({motor.name}): {e}")
            return None

    # ── Move motor ────────────────────────────────────────────────────────────

    def _move_to(self, motor: _Motor, target_deg: float):
        """
        Move motor to target_deg (stepper degrees, absolute from home).

        Computes relative delta from last sent position and issues
        a relative-pulse move frame. Silently skips if within tolerance.
        """
        # First move: origin is home_angle (motor was zeroed at home)
        from_deg = motor.sent_deg if motor.sent_deg is not None else motor.home_angle
        delta    = target_deg - from_deg

        if abs(delta) < self._tol:
            return

        cw     = delta > 0
        pulses = abs(motor.proto.degrees_to_pulses(abs(delta), motor.subdivision))
        if pulses == 0:
            return

        frame = motor.proto.build_move_relative(self._speed, pulses, cw)
        ok    = self._tx(motor, frame)

        if ok:
            motor.sent_deg   = target_deg
            motor.last_move_t = time.time()
            self.get_logger().debug(
                f"{motor.name}: {from_deg:.2f}° → {target_deg:.2f}° "
                f"(Δ{delta:+.2f}°, {pulses} pulses, {'CW' if cw else 'CCW'})"
            )

    def _read_encoder_deg(self, motor: _Motor) -> float | None:
        """Read one encoder sample and return absolute stepper angle in degrees."""
        resp = self._tx_rx(motor, motor.proto.build_read_encoder(), 8)
        if resp is None:
            return None
        result = motor.proto.parse_encoder_response(resp)
        if not result:
            return None
        counts, offset_deg = result
        motor.encoder_counts = counts
        motor.encoder_deg = motor.home_angle + offset_deg
        return motor.encoder_deg

    def _sync_tracking_from_encoder(self) -> bool:
        """Align software tracking with real motor shaft angles."""
        ok = True
        for m in [self._left, self._right]:
            deg = self._read_encoder_deg(m)
            if deg is None:
                ok = False
                continue
            m.sent_deg = deg
        return ok

    def _command_home_now(self, reason: str):
        """Physically command both motors to home_angle from current encoder position."""
        for m in [self._left, self._right]:
            self._move_to(m, m.home_angle)
        self._pending_L = self._left.home_angle
        self._pending_R = self._right.home_angle
        self._dirty = False
        self.get_logger().info(f"Auto-home command sent ({reason})")
        self._pub_status(f"Auto-home command sent ({reason})")

    # ── /joint_states callback ────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState):
        """Extract URDF motor angles and convert to stepper degrees."""
        try:
            idx_L = msg.name.index(self.MOTOR_LEFT_JOINT)
            idx_R = msg.name.index(self.MOTOR_RIGHT_JOINT)
        except ValueError:
            return   # joints not in this message

        urdf_L = msg.position[idx_L]   # radians
        urdf_R = msg.position[idx_R]

        # Same mapping as the original arduino_bridge.py:
        #   step_deg = 90 − degrees(urdf_rad)
        self._pending_L = 90.0 - math.degrees(urdf_L)
        self._pending_R = 90.0 - math.degrees(urdf_R)
        self._dirty = True

    # ── /arduino_enable callback ──────────────────────────────────────────────

    def _cb_enable(self, msg: Bool):
        """Enable or disable motor torque."""
        enable = msg.data
        for m in [self._left, self._right]:
            self._tx_rx(m, m.proto.build_enable(enable), 4)
        state = "enabled" if enable else "disabled"
        self.get_logger().info(f"Motors {state}")
        self._pub_status(f"Motors {state}")

        if enable and self._auto_home_on_enable:
            if self._sync_tracking_from_encoder():
                self._command_home_now(reason="enable")
            else:
                self.get_logger().warn("Auto-home skipped on enable: encoder read failed")

    # ── /arduino_raw_cmd callback ─────────────────────────────────────────────

    def _cb_raw_cmd(self, msg: String):
        """
        Handle raw command strings (mirrors arduino_bridge.py interface).

        Supported:
          G28   — home both motors (reset sent_deg to home_angle)
          stop  — emergency stop both motors
          status — log current state
          M114  — query encoder and log (verbose)
        """
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"Raw command: {cmd}")

        if cmd == "G28":
            # True home: sync from encoder, then physically move to home angle.
            if self._sync_tracking_from_encoder():
                self._command_home_now(reason="G28")
            else:
                self.get_logger().warn("G28 failed: encoder read failed")
                self._pub_status("G28 failed: encoder read failed")

        elif cmd in ("STOP", "M112"):
            for m in [self._left, self._right]:
                self._tx(m, m.proto.build_stop())
                m.sent_deg = m.encoder_deg   # sync to encoder
            self._pub_status("Emergency stop sent")

        elif cmd == "M114":
            self._poll_tick(verbose=True)

        elif cmd == "STATUS":
            self._log_status()

        elif cmd.startswith("M17"):   # enable (compatibility)
            self._cb_enable(Bool(data=True))
        elif cmd.startswith("M18"):   # disable (compatibility)
            self._cb_enable(Bool(data=False))

        else:
            self.get_logger().warn(f"Unknown raw command: {cmd}")

    # ── Rate-limited send timer ───────────────────────────────────────────────

    def _send_tick(self):
        """Send pending motor commands at controlled rate."""
        if not self._dirty:
            return
        L, R = self._pending_L, self._pending_R
        self._move_to(self._left,  L)
        self._move_to(self._right, R)
        self._dirty = False

    # ── Encoder polling ───────────────────────────────────────────────────────

    def _poll_tick(self, verbose: bool = False):
        """Read encoder from both motors and publish feedback."""
        states = {}
        for m in [self._left, self._right]:
            resp = self._tx_rx(m, m.proto.build_read_encoder(), 8)
            if resp is None:
                # Simulation: echo commanded position
                m.encoder_deg = m.sent_deg if m.sent_deg is not None else m.home_angle
            else:
                result = m.proto.parse_encoder_response(resp)
                if result:
                    counts, offset_deg = result
                    # Offset is from motor's zero (set at home → home_angle)
                    m.encoder_deg    = m.home_angle + offset_deg
                    m.encoder_counts = counts

            # Stall detection: encoder lags commanded by >2° for >2 s
            err = abs((m.sent_deg or m.home_angle) - m.encoder_deg)
            if err > 2.0 and (time.time() - m.last_move_t) > 2.0:
                if not m.stall:
                    m.stall = True
                    self.get_logger().warn(
                        f"Stall? {m.name}: "
                        f"cmd={m.sent_deg or m.home_angle:.2f}° "
                        f"enc={m.encoder_deg:.2f}°  err={err:.2f}°"
                    )
            else:
                m.stall = False

            states[m.name] = m.encoder_deg
            if verbose:
                self.get_logger().info(
                    f"[ENCODER] {m.name}: {m.encoder_deg:.3f}°  "
                    f"(counts={m.encoder_counts})"
                )

        # Publish encoder feedback as JointState
        js              = JointState()
        js.header       = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name         = [self.MOTOR_LEFT_JOINT, self.MOTOR_RIGHT_JOINT]
        js.position     = [
            math.radians(self._left.encoder_deg),
            math.radians(self._right.encoder_deg),
        ]
        self._encoder_pub.publish(js)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _log_status(self):
        for m in [self._left, self._right]:
            self.get_logger().info(
                f"[STATUS] {m.name}  "
                f"sent={m.sent_deg or m.home_angle:.2f}°  "
                f"encoder={m.encoder_deg:.2f}°  "
                f"stall={m.stall}"
            )

    def _pub_status(self, text: str):
        msg      = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _shutdown(self):
        """Disable motors and close serial on shutdown."""
        self.get_logger().info("Shutting down — disabling motors...")
        for m in [self._left, self._right]:
            try:
                self._tx(m, m.proto.build_enable(False))
            except Exception:
                pass
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
                self.get_logger().info("Serial closed.")
            except Exception:
                pass


# ─────────────────────────── Entry point ─────────────────────────────────────

def main():
    rclpy.init()
    node = MksServo42cBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
