# 0001. Python 3.12 + ROS 2 Jazzy on Ubuntu 24.04

**Status:** accepted

## Context
The class whiteboard sets the stack Pi 5 → HAT → OS → ROS 2 → application. We are 4 people with different skill levels and there is little time. The robot runs control at 20–50 Hz, vision at 15 Hz and no heavy computation.

## Decision
- Ubuntu 24.04 LTS, because it is the version officially supported by ROS 2 Jazzy and it runs on the Pi 5.
- ROS 2 Jazzy (LTS until 2029).
- All code in Python 3.12 with `rclpy`. Custom messages in an `ament_cmake` package (the only way to generate interfaces).
- OpenCV for vision and `lgpio` for GPIO.

## Consequences
- The whole team can read and review any package.
- If a node turns out to be slow on the Pi (for example vision), just that node can be rewritten in C++ without touching the rest, because the topics do not change.
- RPi.GPIO does not work on the Pi 5; `lgpio` or `gpiozero` must be used.
