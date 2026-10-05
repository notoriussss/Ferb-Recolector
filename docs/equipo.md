# Equipo, roles e hitos

Somos 4 personas para un robot. Cada una es **dueña** de un área: decide, revisa los PR que la tocan y mantiene su guía en `agents/`. Todos pueden programar en todas partes.

Cuando se asignen los nombres, actualiza también `.github/CODEOWNERS`.

| Rol | Persona | Qué le toca | Guía |
|---|---|---|---|
| **1. Base móvil y hardware** | _por asignar_ | `motor_driver`, `line_sensors`, `ultrasonic`, `scoop_driver`; armado y cableado; `scripts/setup_pi.sh`, `scripts/test_hat.py`; `docs/hardware.md` | `agents/base-movil.md` |
| **2. Visión y color** | _por asignar_ | `vision` (`camera_node`, `cube_detector`); `color_sensor`; `tools/hsv_tuner.py` | `agents/vision.md` |
| **3. Comportamiento** | _por asignar_ | `behavior_fsm`, `obstacle_guard`, `recolector_interfaces`; `docs/arquitectura.md`; acuerdos con los otros equipos (`docs/interfaz-externa.md`) | `agents/comportamiento.md` |
| **4. Simulación, integración y calidad** | _por asignar_ | `simulation/`, `bringup`, `scripts/setup_dev.sh`, CI; revisión final de `main` | `agents/simulacion-integracion.md` |

## Reglas del equipo

- Cada PR lo revisa alguien que no sea su autor.
- Una tarea = un issue = una rama = un PR.
- Nada se prueba en el robot real sin haber pasado antes por la simulación o por una prueba en la laptop, salvo las pruebas de hardware puro.

## Hitos

| Hito | Demo |
|---|---|
| **H0. Repo listo** | `colcon build` pasa en las 4 laptops y en CI |
| **H1. Se mueve** | el robot (real y simulado) se mueve por teclado en todas las direcciones |
| **H2. No se sale** | da vueltas dentro del área sin cruzar la línea y frena ante una mano |
| **H3. Ve los cubos** | la cámara marca los cubos por color en la pista real; el sensor de la pala confirma el color |
| **H4. Entrega uno** | busca un cubo, lo atrapa con la pala y lo deja en el cuadrado; la cuenta sube |
| **H5. Día de la prueba** | trabaja solo varios minutos en la pista con los otros robots, sin chocar |
