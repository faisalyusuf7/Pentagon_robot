#!/usr/bin/env python3
"""
Servo Node — angle-based servo control via PCA9685 (adafruit_servokit).

Subscribes to /servo_cmd (std_msgs/Bool):
    True  → move to DOWN position (servo_down_angle)
    False → move to UP position   (servo_up_angle)

Publishes /servo_status (std_msgs/String) with current state.

Hardware:
  Standard positional servo on PCA9685 channel 0.
  Uses kit.servo[ch].angle (not continuous_servo).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

_SERVO_AVAILABLE = False
try:
    from adafruit_servokit import ServoKit
    _SERVO_AVAILABLE = True
except (ImportError, NotImplementedError, RuntimeError) as e:
    print(f"[servo_node] ServoKit import failed: {e}")


class ServoNode(Node):

    SERVO_CHANNEL    = 0
    SERVO_DOWN_ANGLE = 50
    SERVO_UP_ANGLE   = 180

    def __init__(self):
        super().__init__("servo_node")

        self.declare_parameter("servo_channel",    self.SERVO_CHANNEL)
        self.declare_parameter("servo_down_angle", self.SERVO_DOWN_ANGLE)
        self.declare_parameter("servo_up_angle",   self.SERVO_UP_ANGLE)

        self._channel    = int(self.get_parameter("servo_channel").value)
        self._down_angle = int(self.get_parameter("servo_down_angle").value)
        self._up_angle   = int(self.get_parameter("servo_up_angle").value)

        self._kit = None
        self._hw = _SERVO_AVAILABLE

        if self._hw:
            try:
                self._kit = ServoKit(channels=16)
                self._kit.servo[self._channel].actuation_range = 180
                # Home to UP on startup
                self._kit.servo[self._channel].angle = self._up_angle
                self.get_logger().info(
                    f"Servo ENABLED: ch{self._channel}, homed to {self._up_angle}°")
            except Exception as e:
                self.get_logger().error(f"Servo init failed: {e}")
                self._hw = False

        if not self._hw:
            self.get_logger().info("Servo hardware DISABLED — simulation mode")

        self._status_pub = self.create_publisher(String, "/servo_status", 10)
        self.create_subscription(Bool, "/servo_cmd", self._cb_servo_cmd, 10)

        self._publish_status(f"READY — servo at {self._up_angle}° (UP)")
        self.get_logger().info("Servo node ready")

    def _cb_servo_cmd(self, msg: Bool):
        if msg.data:
            angle = self._down_angle
            label = "DOWN"
        else:
            angle = self._up_angle
            label = "UP"

        self.get_logger().info(f"Servo → {angle}° ({label})")
        if self._hw:
            self._kit.servo[self._channel].angle = angle
        self._publish_status(f"Servo {label} ({angle}°)")

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
