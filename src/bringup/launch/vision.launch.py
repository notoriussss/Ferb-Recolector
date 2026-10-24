"""Vision only on the real robot: camera, detector and web viewer. Does not move the robot.

Useful to validate colour detection on the Pi before letting the robot move.

  ros2 launch bringup vision.launch.py            # viewer at http://<pi-ip>:8080
  ros2 launch bringup vision.launch.py device:=1  # another camera (/dev/video1)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("vision"), "config", "hsv_ranges.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="0"),
            DeclareLaunchArgument("port", default_value="8080"),
            Node(
                package="vision",
                executable="camera_node",
                parameters=[
                    cfg,
                    {"device": ParameterValue(LaunchConfiguration("device"), value_type=str)},
                ],
                output="screen",
            ),
            Node(package="vision", executable="cube_detector", parameters=[cfg], output="screen"),
            Node(
                package="vision",
                executable="web_viewer",
                parameters=[cfg, {"port": LaunchConfiguration("port")}],
                output="screen",
            ),
        ]
    )
