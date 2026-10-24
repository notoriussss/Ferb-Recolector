# AGENTS.md

Instructions for any AI assistant working in this repository (Claude Code, Copilot, Codex, Cursor, etc.).

## What this project is

A single robot: the UCAB Robotics collector. It is mecanum, picks up 2.5 cm cubes inside the black-line area and pushes them with its scoop to the delivery square, counting the ones it delivers. The other robots on the track are other teams' projects and do **not** go in this repo. Context is in `README.md` and `docs/`.

## The team writes the code

This repo is a class project. Code files start empty, with a docstring and a `TODO`.
- Do not write a file's full implementation unless someone explicitly asked you to.
- If asked for help, prefer explaining, reviewing or suggesting; let the person write.
- Do not fill in other areas' files all at once.

## Before touching anything

1. Read your area's guide in `agents/`.
2. If you change a topic or a message, update `docs/architecture.md` first.
3. Do not invent hardware measurements: if they are not in `docs/hardware.md` or in a YAML, leave a `# TODO measure`.

## Stack and commands

- Ubuntu 26.04, ROS 2 Lyrical, Python 3.12, `rclpy`.
- Build: `colcon build --symlink-install`
- Test: `colcon test && colcon test-result --verbose`
- Lint and format: `ruff check src tools && ruff format --check src tools`

## Code rules

- Everything technical in English: identifiers, comments, docstrings, logs, states, config values and UI text.
- Every physical parameter (pins, speeds, thresholds, times) goes in YAML, with the unit in the name (`cmd_timeout_s`).
- Pure logic (kinematics, colour classification, state machine) goes in modules without `rclpy`, with tests in `test/`.
- Hardware access goes behind a class with a fake version, so it can be tested on the laptop.
- No magic numbers or strings in the code: a named constant (`UPPER_SNAKE_CASE`, with a comment giving the unit or why), or the YAML if it is tunable. Topic and service names live in `topics.py`, states in `states.py`. `ruff` enforces comparisons (PLR2004).
- Parameters are declared once, as dataclass fields (`mobile_base/params.py`); the field name is the ROS parameter name. A node's default must equal the YAML value, or come from the same constant.
- Docstrings: the quotes go on their own lines and the text sits at the same indentation, for every module, class and function (one-liners too):

  ```python
  def clamp(value, low, high):
      """
      Limit `value` to the closed interval [low, high].
      """
  ```
- Style: 80 columns, one argument or item per line once a call or literal does not fit (end with a trailing comma). One responsibility per module and per function; `ruff` caps complexity at 10 (C901). If a node file grows past one screen of logic, move the pure part out.
- Do not use Nav2, SLAM, Gazebo or MoveIt (course decision). RPi.GPIO does not work on the Pi 5.

## Robot safety

- Velocity goes through `obstacle_guard` before reaching `motor_driver`.
- `motor_driver` brakes if it stops receiving commands.
- 2S battery at most: the HAT handles 10.8 V.

## Git

- Branches `feature/<area>-<topic>`, `fix/…`, `docs/…`; never push to `main`.
- Commits `area: what`. Small PR with the template filled in.
- No `build/`, `install/`, `log/`, heavy photos or credentials.
