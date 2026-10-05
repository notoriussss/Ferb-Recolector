# AGENTS.md

Instrucciones para cualquier asistente de IA que trabaje en este repositorio (Claude Code, Copilot, Codex, Cursor, etc.).

## Qué es este proyecto

Un solo robot: el recolector de Robótica UCAB. Es mecanum, recoge cubos de 2,5 cm dentro del área de la línea negra y los empuja con su pala al cuadrado de entrega, contando los que entrega. Los otros robots de la pista son proyectos de otros equipos y **no** van en este repo. El contexto está en `README.md` y `docs/`.

## El código lo escribe el equipo

Este repo es un proyecto de clase. Los archivos de código empiezan vacíos, con un docstring y un `TODO`.
- No escribas la implementación completa de un archivo si nadie te lo pidió explícitamente.
- Si te piden ayuda, prefiere explicar, revisar o sugerir; deja que la persona escriba.
- No llenes de golpe archivos de otras áreas.

## Antes de tocar algo

1. Lee la guía de tu área en `agents/`.
2. Si cambias un tópico o un mensaje, actualiza primero `docs/arquitectura.md`.
3. No inventes medidas del hardware: si no están en `docs/hardware.md` o en un YAML, deja un `# TODO medir`.

## Stack y comandos

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, `rclpy`.
- Compilar: `colcon build --symlink-install`
- Probar: `colcon test && colcon test-result --verbose`
- Lint y formato: `ruff check src tools && ruff format --check src tools`

## Reglas de código

- Identificadores en inglés; comentarios, docstrings y logs en español.
- Todo parámetro físico (pines, velocidades, umbrales, tiempos) va en YAML, con la unidad en el nombre (`cmd_timeout_s`).
- La lógica pura (cinemática, clasificación de color, máquina de estados) va en módulos sin `rclpy`, con pruebas en `test/`.
- El acceso al hardware va detrás de una clase con versión falsa, para probar en la laptop.
- No usar Nav2, SLAM, Gazebo ni MoveIt (decisión del curso). RPi.GPIO no funciona en la Pi 5.

## Seguridad del robot

- La velocidad pasa por `obstacle_guard` antes de llegar a `motor_driver`.
- `motor_driver` frena si deja de recibir comandos.
- Batería 2S como máximo: el HAT aguanta 10,8 V.

## Git

- Ramas `feature/<area>-<tema>`, `fix/…`, `docs/…`; nunca push a `main`.
- Commits `area: qué`. PR pequeño con la plantilla llena.
- Nada de `build/`, `install/`, `log/`, fotos pesadas ni credenciales.
