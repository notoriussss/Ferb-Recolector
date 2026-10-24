"""
Collector mecanum base: motion, safety filters, behaviour and fleet link.

Nodes: behavior_fsm, obstacle_guard, fleet_link (more to come: motor_driver,
line_sensors, ultrasonic, scoop_driver, color_sensor). Pure logic lives in
modules without rclpy (approach, speed_limits, fleet, peer_avoidance,
kinematics, mathutil); see docs/architecture.md, section "Code layout".
"""
