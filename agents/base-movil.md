# Guía: base móvil y hardware

**Dueño:** rol 1 · **Archivos:** `src/mobile_base/mobile_base/{motor_driver,line_sensors,ultrasonic,scoop_driver}.py`, `scripts/setup_pi.sh`, `scripts/test_hat.py`, `docs/hardware.md`

## Lo que hay que saber

- Chasis mecanum Hiwonder, ruedas de 65 mm, motores TT 1:120 **sin encoders**: la velocidad es de lazo abierto.
- HAT Cokoino con 2× DRV8833: cada motor usa dos pines (IN1/IN2) con PWM. Máximo 10,8 V → batería 2S.
- La Pi 5 necesita 5 V / 5 A; el regulador va aparte del HAT.
- En la Pi 5 se usa `lgpio` o `gpiozero`; RPi.GPIO no funciona. Los pines del HAT hay que medirlos (fue diseñado para la Pi 4).
- El eco del HC-SR04 sale a 5 V: necesita divisor a 3,3 V.
- Convención de signos REP 103: `linear.x` > 0 adelante, `linear.y` > 0 izquierda, `angular.z` > 0 antihorario.
- Orden de ruedas y sensores en todo el proyecto: fl, fr, rl, rr.

## Para probar

- Primero con las ruedas en el aire (`scripts/test_hat.py`).
- Control directo del HAT, sin ROS (mientras no exista `motor_driver`): `python3 tools/teleop_web.py` y abrir http://localhost:8081 (`--fake` en la laptop). Pines y tope en `tools/teleop_web.yaml`. **No pasa por `obstacle_guard`**: ruedas en el aire. Desde otra máquina: `ssh -L 8081:localhost:8081 <usuario>@<ip-de-la-pi>`.
- Con ROS, el control manual WASD está en el visor web (`web_viewer`, puerto 8080).
- Si una rueda gira al revés, invierte ese motor en el YAML, no en el cableado.
