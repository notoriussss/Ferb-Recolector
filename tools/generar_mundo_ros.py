#!/usr/bin/env python3
"""Genera simulation/webots/worlds/pista_ucab_ros.wbt a partir de pista_ucab.wbt.

La variante para ROS 2 es igual, pero Phineas usa el controlador <extern> (lo maneja el
nodo webots_bridge) y Ferb queda quieto (<none>). Correr después de cambiar pista_ucab.wbt:

  python3 tools/generar_mundo_ros.py
"""

from pathlib import Path

WORLDS = Path(__file__).resolve().parent.parent / "simulation" / "webots" / "worlds"
HEADER = (
    "# Variante para ROS 2: Phineas usa controlador <extern> (lo maneja el nodo webots_bridge)\n"
    "# y Ferb queda quieto. Generado desde pista_ucab.wbt con tools/generar_mundo_ros.py.\n"
)


def set_controller(text: str, robot: str, controller: str) -> str:
    old = f'  name "{robot}"\n  controller "recolector_teleop"'
    if text.count(old) != 1:
        raise SystemExit(f"no encontré el controlador de {robot} en pista_ucab.wbt")
    return text.replace(old, f'  name "{robot}"\n  controller "{controller}"')


def main():
    text = (WORLDS / "pista_ucab.wbt").read_text(encoding="utf-8")
    first, rest = text.split("\n", 1)  # la primera línea es la cabecera #VRML_SIM
    text = f"{first}\n{HEADER}{rest}"
    text = set_controller(text, "Phineas", "<extern>")
    text = set_controller(text, "Ferb", "<none>")
    (WORLDS / "pista_ucab_ros.wbt").write_text(text, encoding="utf-8")
    print("listo: simulation/webots/worlds/pista_ucab_ros.wbt")


if __name__ == "__main__":
    main()
