#!/usr/bin/env python3
"""
Generates simulation/webots/worlds/pista_ucab_ros.wbt from pista_ucab.wbt.

The ROS 2 variant is the same, but Phineas uses the <extern> controller
(driven by the webots_bridge node) and Ferb stays still (<none>). Run it
after changing pista_ucab.wbt:

  python3 tools/generate_ros_world.py
"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
WORLDS = REPO_ROOT / "simulation" / "webots" / "worlds"
SOURCE_WORLD = "pista_ucab.wbt"
TARGET_WORLD = "pista_ucab_ros.wbt"

# Controller every robot has in the source world, and the ones it gets here.
SOURCE_CONTROLLER = "recolector_teleop"
CONTROLLER_EXTERN = "<extern>"
CONTROLLER_NONE = "<none>"
PHINEAS = "Phineas"
FERB = "Ferb"

# Comment block written at the top of the generated world (kept identical to
# the committed pista_ucab_ros.wbt so regenerating it produces no diff).
HEADER = (
    "# ROS 2 variant: Phineas uses the <extern> controller "
    "(driven by the webots_bridge node)\n"
    f"# and Ferb stays still. Generated from {SOURCE_WORLD} "
    "by tools/generate_ros_world.py.\n"
)


def set_controller(text: str, robot: str, controller: str) -> str:
    """
    Replace the controller of `robot` in the world text.

    Exits with a message if the robot is not found exactly once.
    """
    old = f'  name "{robot}"\n  controller "{SOURCE_CONTROLLER}"'
    if text.count(old) != 1:
        raise SystemExit(
            f"could not find the controller of {robot} in {SOURCE_WORLD}"
        )
    return text.replace(old, f'  name "{robot}"\n  controller "{controller}"')


def main() -> None:
    """
    Read the source world, swap the controllers and write the ROS world.
    """
    text = (WORLDS / SOURCE_WORLD).read_text(encoding="utf-8")
    # The first line is the #VRML_SIM header.
    first_line, rest = text.split("\n", 1)
    text = f"{first_line}\n{HEADER}{rest}"
    text = set_controller(text, PHINEAS, CONTROLLER_EXTERN)
    text = set_controller(text, FERB, CONTROLLER_NONE)
    (WORLDS / TARGET_WORLD).write_text(text, encoding="utf-8")
    print(f"done: simulation/webots/worlds/{TARGET_WORLD}")


if __name__ == "__main__":
    main()
