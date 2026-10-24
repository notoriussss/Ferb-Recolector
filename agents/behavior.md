# Guide: behaviour

**Owner:** role 3 · **Files:** `src/mobile_base/mobile_base/{behavior_fsm,obstacle_guard}.py`, `src/recolector_interfaces/`, `docs/architecture.md`, `docs/external-interface.md`

## What you need to know

- The proposed nodes, topics and states are in `docs/architecture.md`. It is a starting point, not an obligation.
- Velocity always goes through `obstacle_guard` before the motor.
- Ultrasonic level idea (from the plan): >30 cm normal, 15–30 cm half speed, <15 cm stopped.
- The delivery square is black and touches the line: the robot must tell "I reached the square" apart from "I am about to leave the area".
- The scoop carries one cube at a time.
- The other robots belong to other teams; whatever is agreed with them goes in `docs/external-interface.md`.
- It is best to write the state machine as pure logic without `rclpy`, so it can be tested with `pytest`.
