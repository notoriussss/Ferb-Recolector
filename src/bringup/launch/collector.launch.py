"""Launches all collector nodes on the Raspberry Pi.

For now: camera, detector, web viewer, behavior_fsm (approach a cube) and obstacle_guard.
For safety, behavior_fsm starts paused: start it with the "Start" button in the web viewer
or with enabled:=true.

TODO (roles 1 and 4): add motor_driver, ultrasonic, line_sensors, scoop_driver and
color_sensor once implemented. Without motor_driver the real robot does not move; without
ultrasonic, obstacle_guard blocks forward motion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision_cfg = os.path.join(get_package_share_directory("vision"), "config", "hsv_ranges.yaml")
    base_cfg = os.path.join(get_package_share_directory("mobile_base"), "config", "collector.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("enabled", default_value="false"),
            Node(
                package="vision",
                executable="camera_node",
                parameters=[
                    vision_cfg,
                    {"device": ParameterValue(LaunchConfiguration("device"), value_type=str)},
                ],
                output="screen",
            ),
            Node(
                package="vision",
                executable="cube_detector",
                parameters=[vision_cfg],
                output="screen",
            ),
            Node(
                package="vision",
                executable="web_viewer",
                parameters=[vision_cfg, {"port": LaunchConfiguration("port")}],
                output="screen",
            ),
            Node(
                package="mobile_base",
                executable="behavior_fsm",
                parameters=[
                    base_cfg,
                    {"enabled": ParameterValue(LaunchConfiguration("enabled"), value_type=bool)},
                ],
                output="screen",
            ),
            Node(
                package="mobile_base",
                executable="obstacle_guard",
                parameters=[base_cfg],
                output="screen",
            ),
        ]
    )
