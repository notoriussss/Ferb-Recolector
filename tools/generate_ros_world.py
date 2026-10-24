#!/usr/bin/env python3
"""Generates simulation/webots/worlds/pista_ucab_ros.wbt from pista_ucab.wbt.

The ROS 2 variant is the same, but Phineas uses the <extern> controller (driven by the
webots_bridge node) and Ferb stays still (<none>). Run it after changing pista_ucab.wbt:

  python3 tools/generate_ros_world.py
"""

from pathlib import Path

WORLDS = Path(__file__).resolve().parent.parent / "simulation" / "webots" / "worlds"
HEADER = (
    "# ROS 2 variant: Phineas uses the <extern> controller (driven by the webots_bridge node)\n"
    "# and Ferb stays still. Generated from pista_ucab.wbt by tools/generate_ros_world.py.\n"
)


def set_controller(text: str, robot: str, controller: str) -> str:
    old = f'  name "{robot}"\n  controller "recolector_teleop"'
    if text.count(old) != 1:
        raise SystemExit(f"could not find the controller of {robot} in pista_ucab.wbt")
    return text.replace(old, f'  name "{robot}"\n  controller "{controller}"')


def main():
    text = (WORLDS / "pista_ucab.wbt").read_text(encoding="utf-8")
    first, rest = text.split("\n", 1)  # the first line is the #VRML_SIM header
    text = f"{first}\n{HEADER}{rest}"
    text = set_controller(text, "Phineas", "<extern>")
    text = set_controller(text, "Ferb", "<none>")
    (WORLDS / "pista_ucab_ros.wbt").write_text(text, encoding="utf-8")
    print("done: simulation/webots/worlds/pista_ucab_ros.wbt")


if __name__ == "__main__":
    main()
