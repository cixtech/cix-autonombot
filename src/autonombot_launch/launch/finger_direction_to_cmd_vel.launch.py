#!/usr/bin/env python3
"""Bridge /handpose/finger_direction (Vector3) -> key_vel (Twist) for twist_mux.

Provided by autonombot_launch; the SDK package cix_ros_noe_handpose (optional) publishes the Vector3 in another process.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "input_topic", default_value="/handpose/finger_direction"
            ),
            DeclareLaunchArgument("output_topic", default_value="key_vel"),
            DeclareLaunchArgument("linear_speed", default_value="0.25"),
            DeclareLaunchArgument("angular_speed", default_value="0.8"),
            DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="0.35"),
            Node(
                package="autonombot_launch",
                executable="finger_direction_to_cmd_vel",
                name="finger_direction_to_cmd_vel",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "linear_speed": LaunchConfiguration("linear_speed"),
                        "angular_speed": LaunchConfiguration("angular_speed"),
                        "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
                    }
                ],
            ),
        ]
    )
