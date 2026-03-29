"""
Launch file for Main_Assembly_1 with ros2_control
  - robot_state_publisher  (publishes /robot_description & TF)
  - ros2_control_node      (controller_manager + mock hardware)
  - joint_state_broadcaster
  - joint_trajectory_controller
  - rviz2
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── paths ──────────────────────────────────────────────
    pkg_share = FindPackageShare('Main_Assembly_1')

    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'Main_Assembly_1.urdf'])
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])

    # Read URDF as string via xacro (works on plain .urdf too)
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str
    )

    # ── nodes ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_yaml,
        ],
        output='screen',
    )

    # Spawner nodes – launched after controller_manager is alive
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Delay spawners until controller_manager has started
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        joint_trajectory_controller_spawner,
                    ],
                ),
            ],
        )
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # ── launch description ────────────────────────────────
    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_spawners,
        rviz2,
    ])
