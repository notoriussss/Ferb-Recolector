"""Solo visión en el robot real: cámara, detector y visor web. No mueve el robot.

Sirve para validar la detección de colores en la Pi antes de dejar que el robot se mueva.

  ros2 launch bringup vision.launch.py            # visor en http://<ip-de-la-pi>:8080
  ros2 launch bringup vision.launch.py device:=1  # otra cámara (/dev/video1)
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
