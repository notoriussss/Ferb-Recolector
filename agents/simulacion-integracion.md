# Guía: simulación, integración y calidad

**Dueño:** rol 4 · **Archivos:** `simulation/`, `src/bringup/`, `scripts/setup_dev.sh`, `.github/`

## Lo que hay que saber

- Webots R2025a. En Windows: Webots nativo y ROS 2 Jazzy en WSL2 con `webots_ros2`.
- El mundo `pista_ucab.wbt` trae la pista rectificada de la foto, 14 cubos y dos recolectores (el segundo hace de robot del otro equipo).
- Nombres de dispositivos del PROTO: `motor_fl/fr/rl/rr`, `servo_pala`, `camara`, `linea_fl/fr/rl/rr`, `ultrasonido`, `ir_pala`, `color_pala`, `acelerometro`, `giroscopio`.
- Sensores de línea simulados: ≈240 en blanco, 0–30 en negro.
- Los materiales de las ruedas (fl/rr `Exterior`, fr/rl `Interior`) hacen que la fricción mecanum funcione; no cambiarlos.
- Cuando se mida la pista real, actualizar el mundo y `docs/hardware.md`.

## Pendiente

- `bringup/launch/simulacion.launch.py`: conectar los nodos con Webots usando `webots_ros2_driver`, con los mismos tópicos que el robot real.
- Activar la protección de `main` y la CI en GitHub.
