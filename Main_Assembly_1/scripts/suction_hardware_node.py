#!/usr/bin/env python3
"""
Suction Hardware Node — controls solenoid valve AND servo for pick & place.

Controls the PCA9685 servo directly (same as servotest.py) instead of
publishing to a separate servo_node, to guarantee reliable operation.

Subscribes to /suction_cmd (std_msgs/Bool) from the planner:
    True  → PICK sequence  (servo down → valve close → grip → servo up)
    False → PLACE sequence (servo down → valve open → release → valve close → servo up)

Also provides manual control via /suction_manual (std_msgs/String):
    "servo_down"   → move servo down
    "servo_up"     → move servo up
    "valve_open"   → open valve (release suction)
    "valve_close"  → close valve (engage suction)
    "pick"         → full pick sequence
    "place"        → full place sequence

Publishes /suction_status (std_msgs/String) with state info.

Hardware:
  Solenoid valve (BCM pin 15): inverted logic via 2N2222 + IRLZ44N
    GPIO HIGH → valve CLOSED (suction on)
    GPIO LOW  → valve OPEN  (suction off)
  Servo on PCA9685 channel 0 via adafruit_servokit:
    DOWN = 50°, UP = 180°

NOTE: Pick/place sequences use a timer-based state machine (not threads)
      so all hardware actions happen inside the executor thread.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# ── Hardware imports (optional — runs without hardware in simulation) ──
_GPIO_AVAILABLE = False
_SERVO_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except (ImportError, NotImplementedError, RuntimeError) as e:
    print(f"[suction_hw] RPi.GPIO import failed: {e}")

try:
    from adafruit_servokit import ServoKit
    _SERVO_AVAILABLE = True
except (ImportError, NotImplementedError, RuntimeError) as e:
    print(f"[suction_hw] ServoKit import failed: {e}")


class SuctionHardwareNode(Node):

    # ── Configuration ──
    VALVE_PIN       = 15      # BCM pin 15 (matching air_valve.py)
    SERVO_CHANNEL   = 0
    SERVO_DOWN_ANGLE = 0     # matches servotest.py
    SERVO_UP_ANGLE   = 270    # matches servotest.py
    SERVO_SETTLE    = 2.5     # seconds to wait for servo to reach position
    GRIP_DELAY      = 3.0     # seconds to let suction grip the ball
    RELEASE_DELAY   = 2.0     # seconds to let ball release

    def __init__(self):
        super().__init__("suction_hardware_node")

        # ── Parameters ──
        self.declare_parameter("valve_pin", self.VALVE_PIN)
        self.declare_parameter("servo_channel", self.SERVO_CHANNEL)
        self.declare_parameter("servo_down_angle", self.SERVO_DOWN_ANGLE)
        self.declare_parameter("servo_up_angle", self.SERVO_UP_ANGLE)
        self.declare_parameter("servo_settle", self.SERVO_SETTLE)
        self.declare_parameter("grip_delay", self.GRIP_DELAY)
        self.declare_parameter("release_delay", self.RELEASE_DELAY)

        self._gpio = _GPIO_AVAILABLE
        self.VALVE_PIN = int(self.get_parameter("valve_pin").value)
        self._servo_channel = int(self.get_parameter("servo_channel").value)
        self._servo_down = int(self.get_parameter("servo_down_angle").value)
        self._servo_up = int(self.get_parameter("servo_up_angle").value)
        self.SERVO_SETTLE = float(self.get_parameter("servo_settle").value)
        self.GRIP_DELAY = float(self.get_parameter("grip_delay").value)
        self.RELEASE_DELAY = float(self.get_parameter("release_delay").value)

        # ── GPIO init ──
        if self._gpio:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.VALVE_PIN, GPIO.OUT, initial=GPIO.HIGH)
                self.get_logger().info(
                    f"GPIO ENABLED: valve pin {self.VALVE_PIN} (CLOSED)")
            except Exception as e:
                self.get_logger().error(f"GPIO init failed: {e}")
                self._gpio = False

        if not self._gpio:
            self.get_logger().info("GPIO DISABLED — valve simulation mode")

        # ── Servo init (direct PCA9685 control, same as servotest.py) ──
        self._kit = None
        self._servo_hw = _SERVO_AVAILABLE
        if self._servo_hw:
            try:
                self._kit = ServoKit(channels=16)
                self._kit.servo[self._servo_channel].actuation_range = 270
                self._kit.servo[self._servo_channel].set_pulse_width_range(500, 2500)
                self._kit.servo[self._servo_channel].angle = self._servo_up
                self.get_logger().info(
                    f"Servo ENABLED: ch{self._servo_channel}, "
                    f"down={self._servo_down}°, up={self._servo_up}°, "
                    f"homed to {self._servo_up}°")
            except Exception as e:
                self.get_logger().error(f"Servo init failed: {e}")
                self._servo_hw = False

        if not self._servo_hw:
            self.get_logger().info("Servo DISABLED — simulation mode")

        # ── Publishers ──
        self._status_pub = self.create_publisher(String, "/suction_status", 10)

        # ── Subscribers ──
        self.create_subscription(Bool, "/suction_cmd", self._cb_suction_cmd, 10)
        self.create_subscription(String, "/suction_manual", self._cb_manual, 10)

        # ── Timer-based sequence state machine ──
        # Each sequence is a list of (action_callable, delay_after) steps.
        # A 20 Hz timer advances through steps without blocking the executor.
        self._seq_steps = []          # list of (action, delay_seconds)
        self._seq_index = -1          # -1 = idle
        self._seq_wait_until = 0.0
        self._seq_timer = self.create_timer(0.05, self._tick_sequence)

        self._publish_status("READY — valve CLOSED")
        self.get_logger().info("Suction hardware node ready")

    # ================================================================ #
    #  Low-level hardware control
    # ================================================================ #
    def _valve_close(self):
        """Close valve → suction active (GPIO HIGH)."""
        if self._gpio:
            GPIO.output(self.VALVE_PIN, GPIO.HIGH)
        self.get_logger().info("Valve CLOSED")

    def _valve_open(self):
        """Open valve → suction released (GPIO LOW)."""
        if self._gpio:
            GPIO.output(self.VALVE_PIN, GPIO.LOW)
        self.get_logger().info("Valve OPEN")

    def _pub_servo_down(self):
        """Move servo to DOWN position directly via PCA9685."""
        if self._servo_hw:
            self._kit.servo[self._servo_channel].angle = self._servo_down
        self.get_logger().info(f"Servo → {self._servo_down}° (DOWN)")

    def _pub_servo_up(self):
        """Move servo to UP position directly via PCA9685."""
        if self._servo_hw:
            self._kit.servo[self._servo_channel].angle = self._servo_up
        self.get_logger().info(f"Servo → {self._servo_up}° (UP)")

    # ================================================================ #
    #  Timer-driven sequence state machine
    # ================================================================ #
    def _start_sequence(self, steps):
        """Begin a sequence of (action, delay) steps. Rejects if busy."""
        if self._seq_index >= 0:
            self.get_logger().warn("Sequence already in progress — ignoring")
            return
        self._seq_steps = steps
        self._seq_index = 0
        action, delay = self._seq_steps[0]
        action()
        self._seq_wait_until = time.monotonic() + delay

    def _tick_sequence(self):
        """Timer callback (20 Hz) — advances through sequence steps."""
        if self._seq_index < 0:
            return  # idle

        if time.monotonic() < self._seq_wait_until:
            return  # still waiting on current step

        # Advance to next step
        self._seq_index += 1
        if self._seq_index >= len(self._seq_steps):
            # Sequence complete
            self._seq_index = -1
            self._seq_steps = []
            return

        action, delay = self._seq_steps[self._seq_index]
        action()
        self._seq_wait_until = time.monotonic() + delay

    def _build_pick_steps(self):
        """Return step list for a PICK sequence.
        Only closes the valve (suction on). Servo movement is handled
        by the planner: servo_down before pick, servo_up after sensor confirm."""
        return [
            (lambda: (self._valve_close(),
                      self._publish_status("PICK: valve closed — waiting for ball")),
             0.0),
        ]

    def _build_place_steps(self):
        """Return step list for a PLACE sequence."""
        return [
            (lambda: (self._pub_servo_down(),
                      self._publish_status("PLACE: servo down")),
             self.SERVO_SETTLE),

            (lambda: (self._valve_open(),
                      self._publish_status("PLACE: valve open (releasing)")),
             self.RELEASE_DELAY),

            (lambda: (self._pub_servo_up(),
                      self._publish_status("PLACE: servo up (retracting)")),
             self.SERVO_SETTLE),

            (lambda: (self._valve_close(),
                      self._publish_status("PLACE: valve close")),
             0.1),

            (lambda: self._publish_status("PLACE complete — ball released"),
             0.0),
        ]

    # ================================================================ #
    #  Callbacks
    # ================================================================ #
    def _cb_suction_cmd(self, msg: Bool):
        """Handle suction commands from planner: True=pick, False=place."""
        if msg.data:
            self.get_logger().info("Received: PICK command")
            self._start_sequence(self._build_pick_steps())
        else:
            self.get_logger().info("Received: PLACE command")
            self._start_sequence(self._build_place_steps())

    def _cb_manual(self, msg: String):
        """Handle manual commands for testing."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Manual command: {cmd}")

        if cmd == "servo_down":
            self._pub_servo_down()
            self._publish_status("Servo DOWN (manual)")
        elif cmd == "servo_up":
            self._pub_servo_up()
            self._publish_status("Servo UP (manual)")
        elif cmd == "valve_open":
            self._valve_open()
            self._publish_status("Valve OPEN (manual)")
        elif cmd == "valve_close":
            self._valve_close()
            self._publish_status("Valve CLOSED (manual)")
        elif cmd == "pick":
            self._start_sequence(self._build_pick_steps())
        elif cmd == "place":
            self._start_sequence(self._build_place_steps())
        elif cmd == "home":
            self._pub_servo_up()
            self._publish_status("Servo homed UP (manual)")
        else:
            self.get_logger().warn(
                f"Unknown manual command: '{cmd}'. "
                "Use: servo_down, servo_up, valve_open, valve_close, pick, place, home")

    # ================================================================ #
    #  Helpers
    # ================================================================ #
    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def cleanup(self):
        """Safe shutdown: valve closed, GPIO released."""
        try:
            if self._gpio:
                GPIO.output(self.VALVE_PIN, GPIO.HIGH)  # valve closed
                GPIO.cleanup()
                self._gpio = False
                print("[suction_hw] GPIO cleaned up")
        except Exception:
            pass


def main():
    rclpy.init()
    node = SuctionHardwareNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
