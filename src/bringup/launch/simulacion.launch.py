"""Lanza los nodos del recolector conectados a Webots.

En lugar de webots_ros2 se usa un puente propio (recolector_sim/webots_bridge) que es el
controlador <extern> del robot "Phineas" y publica los mismos tópicos que el robot real.

Argumentos:
  webots      headless (por defecto, sin ventana, para Docker), gui o none
              (none = Webots ya está abierto, por ejemplo en Windows)
  webots_url  a qué Webots conectarse: ipc://1234/Phineas (misma máquina) o
              tcp://<ip>:1234/Phineas (Webots en otra máquina o fuera de Docker)
  world       mundo a abrir (por defecto simulation/webots/worlds/pista_ucab_ros.wbt)
  port        puerto del visor web (por defecto 8080)

Ejemplo:
  ros2 launch bringup simulacion.launch.py webots:=gui
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
    # con --symlink-install este archivo apunta al repo: src/bringup/launch/ -> raíz
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
    base_cfg = os.path.join(get_package_share_directory("mobile_base"), "config", "recolector.yaml")

    actions = []
    if mode != "none":
        if not os.path.isfile(world):
            raise RuntimeError(f"no existe el mundo {world}; pásalo con world:=<ruta>")
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
            # si se cierra Webots, se cierra todo
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
