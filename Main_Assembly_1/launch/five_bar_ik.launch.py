"""
Standalone 5-bar visualisation launch file.

Launches:
  - robot_state_publisher   (reads URDF, publishes TF from /joint_states)
  - five_bar_ik_node.py     (IK solver, publishes ALL joints to /joint_states)
  - rviz2                   (optional, for visualisation)
  - static_transform_publisher  (world → base_link)

No MoveIt.  No ros2_control.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("Main_Assembly_1")
    urdf_file = os.path.join(pkg, "urdf", "Main_Assembly_1.urdf")
    rviz_config = os.path.join(pkg, "rviz", "config.rviz")

    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    use_v2 = LaunchConfiguration('use_planner_v2')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_planner_v2',
            default_value='false',
            description='Use optimised planner v2 instead of v1',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='',
            description='Arduino serial port; empty enables bridge auto-detect',
        ),

        # world → base_link static TF (base_link is root of V3 URDF)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            output="screen",
        ),

        # robot_state_publisher (URDF → TF from /joint_states)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
            output="screen",
        ),

        # IK node — publishes all 5 joint states
        Node(
            package="Main_Assembly_1",
            executable="five_bar_ik_node.py",
            output="screen",
        ),

        # Slider GUI — interactive control of motor angles / IK target
        Node(
            package="Main_Assembly_1",
            executable="slider_gui.py",
            output="screen",
        ),

        # Pick-and-place planner v1 (default)
        Node(
            package="Main_Assembly_1",
            executable="pick_and_place_planner.py",
            output="screen",
            condition=UnlessCondition(use_v2),
        ),

        # Pick-and-place planner v2 (optimised) — use_planner_v2:=true
        Node(
            package="Main_Assembly_1",
            executable="pick_and_place_planner_v2.py",
            output="screen",
            condition=IfCondition(use_v2),
        ),

        # Servo node — disabled; suction_hardware_node controls PCA9685 directly
        # Node(
        #     package="Main_Assembly_1",
        #     executable="servo_node.py",
        #     output="screen",
        # ),

        # Suction hardware — solenoid valve + servo (direct PCA9685 control)
        Node(
            package="Main_Assembly_1",
            executable="suction_hardware_node.py",
            output="screen",
        ),

        # Sci-Fi Pick & Place GUI (disabled)
        # Node(
        #     package="Main_Assembly_1",
        #     executable="pick_place_gui.py",
        #     output="screen",
        # ),

        # Arduino serial bridge — sends joint angles to Arduino + CNC Shield
        # MKS SERVO42C runs in STEP/DIR closed-loop mode (CR_CLOSE):
        #   ROS2 → serial → Arduino → STEP/DIR → MKS SERVO42C (encoder loop internal)
        # Flash arduino/Stepper_angle_v2/Stepper_angle_v2.ino onto Arduino first.
        Node(
            package="Main_Assembly_1",
            executable="arduino_bridge.py",
            parameters=[{
                "serial_port": serial_port,
                "baud_rate":   115200,
            }],
            output="screen",
        ),

        # Pressure sensor — reads HX710 via GPIO, publishes /ball_detected
        Node(
            package="Main_Assembly_1",
            executable="pressure_sensor_node.py",
            parameters=[{
                "gpio_sck":           11,
                "gpio_out":           27,
                "picked_drop_max":   -5000000,
                "released_drop_min": -4000000,
                "confirm_samples":    2,
                "sample_window":      5,
                "read_hz":            10.0,
            }],
            output="screen",
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config] if os.path.isfile(rviz_config) else [],
            output="screen",
        ),
    ])
