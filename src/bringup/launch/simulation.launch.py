"""Launches the collector nodes connected to Webots.

Instead of webots_ros2 we use our own bridge (recolector_sim/webots_bridge), which is the
<extern> controller of the "Phineas" robot and publishes the same topics as the real robot.

Arguments:
  webots      headless (default, no window, for Docker), gui or none
              (none = Webots is already open, for example on Windows)
  webots_url  which Webots to connect to: ipc://1234/Phineas (same machine) or
              tcp://<ip>:1234/Phineas (Webots on another machine or outside Docker)
  world       world to open (default simulation/webots/worlds/pista_ucab_ros.wbt)
  port        web viewer port (default 8080)

Example:
  ros2 launch bringup simulation.launch.py webots:=gui
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_world() -> str:
    # with --symlink-install this file points into the repo: src/bringup/launch/ -> root
    repo = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../.."))
    return os.path.join(repo, "simulation", "webots", "worlds", "pista_ucab_ros.wbt")


def _setup(context):
    mode = LaunchConfiguration("webots").perform(context)
    world = LaunchConfiguration("world").perform(context)
    url = LaunchConfiguration("webots_url").perform(context)
    port = int(LaunchConfiguration("port").perform(context))

    webots_home = os.environ.get("WEBOTS_HOME", "/usr/local/webots")
    controller_lib = os.path.join(webots_home, "lib", "controller")
    env_path = os.environ.get("PYTHONPATH", "")
    env_ld = os.environ.get("LD_LIBRARY_PATH", "")
    bridge_env = {
        "PYTHONPATH": os.pathsep.join(
            p for p in (os.path.join(controller_lib, "python"), env_path) if p
        ),
        "LD_LIBRARY_PATH": os.pathsep.join(p for p in (controller_lib, env_ld) if p),
        "WEBOTS_CONTROLLER_URL": url,
        "WEBOTS_HOME": webots_home,
    }

    vision_cfg = os.path.join(get_package_share_directory("vision"), "config", "hsv_ranges.yaml")
    base_cfg = os.path.join(get_package_share_directory("mobile_base"), "config", "collector.yaml")

    actions = []
    if mode != "none":
        if not os.path.isfile(world):
            raise RuntimeError(f"world {world} does not exist; pass it with world:=<path>")
        cmd = ["webots", "--batch", "--mode=realtime", "--stdout", "--stderr"]
        if mode == "headless":
            cmd = [
                "xvfb-run",
                "-a",
                "-s",
                "-screen 0 1280x720x24",
                *cmd,
                "--no-rendering",
                "--minimize",
            ]
        webots = ExecuteProcess(cmd=[*cmd, world], name="webots", output="screen")
        actions += [
            webots,
            # if Webots closes, everything shuts down
            RegisterEventHandler(
                OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])
            ),
        ]

    actions += [
        Node(
            package="recolector_sim",
            executable="webots_bridge",
            output="screen",
            additional_env=bridge_env,
        ),
        Node(
            package="vision", executable="cube_detector", parameters=[vision_cfg], output="screen"
        ),
        Node(
            package="vision",
            executable="web_viewer",
            parameters=[vision_cfg, {"port": port}],
            output="screen",
        ),
        Node(
            package="mobile_base", executable="behavior_fsm", parameters=[base_cfg], output="screen"
        ),
        Node(
            package="mobile_base",
            executable="obstacle_guard",
            parameters=[base_cfg],
            output="screen",
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("webots", default_value="headless"),
            DeclareLaunchArgument("webots_url", default_value="ipc://1234/Phineas"),
            DeclareLaunchArgument("world", default_value=_default_world()),
            DeclareLaunchArgument("port", default_value="8080"),
            OpaqueFunction(function=_setup),
        ]
    )
