"""Manual control of the collector for testing the model in Webots.

This is not the robot's logic: it is used to check that motors, scoop and sensors
respond like the real robot before writing the ROS 2 nodes.

Keys (click on the 3D view first):
  W / S        forward / backward
  A / D        strafe left / right (mecanum wheels)
  Q / E        turn left / right
  P            raise / lower the scoop
  + / -        more / less speed
  space        stop
"""

import colorsys
import sys

from controller import Keyboard, Robot

# Hiwonder chassis measurements (see RecolectorUCAB.proto)
WHEEL_RADIUS = 0.0325  # m
LX = 0.0605  # m, half the wheelbase
LY = 0.0655  # m, half the track width
MAX_MOTOR_SPEED = 10.5  # rad/s, TT 1:120 at ~6 V
LIMIT_2S = 0.7  # with a 2S battery, limit to ~70 % so the 6 V TT motors are not overdriven

SCOOP_DOWN = 0.0
SCOOP_UP = -0.7  # rad (~40 degrees)

LINE_THRESHOLD = 120  # simulated TCRT5000: white ~240, black line ~0-30


def read_colors(args):
    for a in args:
        if a.startswith("--colors="):
            return a.split("=", 1)[1].split(",")
    return []


def color_from_hue(r, g, b):
    """Simple hue-based classification, only for console output."""
    h, s, v = colorsys.rgb_to_hsv(r / 255, g / 255, b / 255)
    if s < 0.25 or v < 0.15:
        return "none"
    h *= 360
    if h < 15 or h >= 330:
        return "red" if v > 0.45 else "purple"
    if h < 40:
        return "orange"
    if h < 70:
        return "yellow"
    if h < 170:
        return "green"
    if h < 260:
        return "blue"
    return "purple"


def main():
    robot = Robot()
    step = int(robot.getBasicTimeStep())
    name = robot.getName()
    my_colors = read_colors(sys.argv[1:])

    motors = {}
    for n in ("fl", "fr", "rl", "rr"):
        m = robot.getDevice(f"motor_{n}")
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        motors[n] = m
    servo = robot.getDevice("servo_pala")
    servo.setPosition(SCOOP_DOWN)

    lines = {}
    for n in ("fl", "fr", "rl", "rr"):
        s = robot.getDevice(f"linea_{n}")
        s.enable(step)
        lines[n] = s
    sonar = robot.getDevice("ultrasonido")
    sonar.enable(step)
    scoop_ir = robot.getDevice("ir_pala")
    scoop_ir.enable(step)
    scoop_color = robot.getDevice("color_pala")
    scoop_color.enable(step * 4)
    camera = robot.getDevice("camara")
    camera.enable(step * 4)
    accel = robot.getDevice("acelerometro")
    accel.enable(step)
    gyro = robot.getDevice("giroscopio")
    gyro.enable(step)

    keyboard = Keyboard()
    keyboard.enable(step)

    scale = 0.5
    scoop_up = False
    p_prev = False
    heading = 0.0  # integrated from the gyro: drifts over time, like the real MPU6050
    t_last = 0.0

    print(f"[{name}] ready. My colours: {', '.join(my_colors) or 'unassigned'}")

    while robot.step(step) != -1:
        vx = vy = w = 0.0
        p_now = False
        key = keyboard.getKey()
        while key != -1:
            c = chr(key & 0xFF).upper() if (key & 0xFF) < 128 else ""
            if c == "W" or key == Keyboard.UP:
                vx = 1.0
            elif c == "S" or key == Keyboard.DOWN:
                vx = -1.0
            elif c == "A":
                vy = 1.0
            elif c == "D":
                vy = -1.0
            elif c == "Q" or key == Keyboard.LEFT:
                w = 1.0
            elif c == "E" or key == Keyboard.RIGHT:
                w = -1.0
            elif c == "P":
                p_now = True
            elif c == "+":
                scale = min(1.0, scale + 0.01)
            elif c == "-":
                scale = max(0.1, scale - 0.01)
            key = keyboard.getKey()

        if p_now and not p_prev:
            scoop_up = not scoop_up
            servo.setPosition(SCOOP_UP if scoop_up else SCOOP_DOWN)
        p_prev = p_now

        # Mecanum inverse kinematics (X-roller layout). vx, vy in m/s, w in rad/s.
        v_lin = MAX_MOTOR_SPEED * LIMIT_2S * WHEEL_RADIUS * scale
        vx *= v_lin
        vy *= v_lin
        w *= v_lin / (LX + LY)
        k = LX + LY
        wheels = {
            "fl": (vx - vy - k * w) / WHEEL_RADIUS,
            "fr": (vx + vy + k * w) / WHEEL_RADIUS,
            "rl": (vx + vy - k * w) / WHEEL_RADIUS,
            "rr": (vx - vy + k * w) / WHEEL_RADIUS,
        }
        cap = MAX_MOTOR_SPEED * LIMIT_2S
        for n, v in wheels.items():
            motors[n].setVelocity(max(-cap, min(cap, v)))

        dt = step / 1000.0
        heading += gyro.getValues()[2] * dt

        t = robot.getTime()
        if t - t_last >= 0.5:
            t_last = t
            black = [n for n, s in lines.items() if s.getValue() < LINE_THRESHOLD]
            img = scoop_color.getImage()
            r = g = b = 0
            if img:
                width, height = scoop_color.getWidth(), scoop_color.getHeight()
                for yy in range(height):
                    for xx in range(width):
                        r += scoop_color.imageGetRed(img, width, xx, yy)
                        g += scoop_color.imageGetGreen(img, width, xx, yy)
                        b += scoop_color.imageGetBlue(img, width, xx, yy)
                npx = width * height
                r, g, b = r // npx, g // npx, b // npx
            has_cube = scoop_ir.getValue() > 0.5
            color = color_from_hue(r, g, b) if has_cube else "-"
            mine = "yes" if color in my_colors else "no"
            print(
                f"[{name}] t={t:5.1f}s speed={scale:.2f} scoop={'up' if scoop_up else 'down'} "
                f"black_line={black or 'none'} sonar={sonar.getValue():.2f} m "
                f"cube_in_scoop={'yes' if has_cube else 'no'} color={color}"
                + (f" (mine: {mine})" if has_cube else "")
                + f" gyro_heading={heading * 57.3:6.1f} deg"
            )


if __name__ == "__main__":
    main()
