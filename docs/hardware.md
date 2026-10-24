# Hardware and connections

Values marked **measure** come from photos or datasheets and must be confirmed with the robot in hand.

## Collector

| Part | Model | Notes |
|---|---|---|
| Chassis | LewanSoul / Hiwonder mecanum | 186 × 161 × 91 mm, 65 mm wheels |
| Motors | 4× TT 1:120, 3–6 V | no encoders |
| Computer | Raspberry Pi 5, 8 GB | needs 5 V / 5 A |
| Driver | Cokoino 4WD HAT (2× DRV8833) | **max 10.8 V → 2S battery**; sold for Pi 4, confirm pins on the Pi 5 |
| Pi regulator | XL4015 at 5.1 V | separate from the HAT |
| Battery | 2S Li-ion 18650 (7.4 V) | with fuse and main switch |
| Scoop | 3D printed (see plan, section 6) | one cube at a time |
| Scoop servo | SG90 | 0° down, ≈40° up (**measure**) |
| Camera | Pi camera module or USB webcam | 640 × 480 |
| Line sensor | 1× TCRT5000 | underneath, between the front wheels |
| Ultrasonic | HC-SR04 | front; the 5 V echo needs a divider to 3.3 V |
| Scoop presence | FC-51 (IR) | 1 = cube present |
| Scoop colour | TCS34725 (I2C) | confirms the cube's colour |
| IMU (optional) | MPU6050 (I2C) | approximate turns |

## Pins (to be filled in)

Fill in with what is measured on the HAT and copy the same numbers into `src/mobile_base/config/collector.yaml`.

| Signal | GPIO (BCM) | Status |
|---|---|---|
| DRV8833 #1 / #2 nSLEEP | 12 / 13 | confirmed (CKK0018 schematic) |
| Motor FL (M3) IN1 / IN2 | 24 / 25 | confirmed (DRV8833 #2 A) |
| Motor FR (M4) IN1 / IN2 | 26 / 16 | confirmed (DRV8833 #2 B) |
| Motor RL (M1) IN1 / IN2 | 17 / 27 | confirmed (DRV8833 #1 A) |
| Motor RR (M2) IN1 / IN2 | 22 / 23 | confirmed (DRV8833 #1 B) |
| Scoop servo | ? | measure |
| Bottom TCRT5000 (DO, at 3.3 V) | ? | measure; goes in `tools/teleop_web.yaml` → `line_sensor.pin` |
| HC-SR04 TRIG / ECHO | ? / ? | measure |
| FC-51 | ? | measure |
| I2C SDA / SCL | 2 / 3 | fixed on the Pi |

## Track

| Measurement | Value | Status |
|---|---|---|
| Board | ≈ 1.93 × 1.22 m (from the photo) | measure |
| Tape width | ≈ 19 mm | measure |
| Delivery square | ≈ 24 cm per side | measure |
| Cubes | 2.5 cm, ≈ 9.4 g | measure |
| Colours | red, orange, yellow, green, blue, purple/maroon | confirm maroon |
