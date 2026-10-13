# Guía: simulación, integración y calidad

**Dueño:** rol 4 · **Archivos:** `simulation/`, `src/bringup/`, `scripts/setup_dev.sh`, `.github/`

## Lo que hay que saber

- Webots R2025a. Los nodos se conectan con el puente propio `webots_bridge` (controlador `<extern>`), no con `webots_ros2`. Entorno: Docker (`compose.yaml`, `docker/README.md`); en Windows, Webots nativo + `docker compose up sim-externo`.
- `pista_ucab_ros.wbt` se genera desde `pista_ucab.wbt` con `tools/generar_mundo_ros.py`.
- El mundo `pista_ucab.wbt` trae la pista rectificada de la foto, 14 cubos y dos recolectores (el segundo hace de robot del otro equipo).
- Nombres de dispositivos del PROTO: `motor_fl/fr/rl/rr`, `servo_pala`, `camara`, `linea_fl/fr/rl/rr`, `ultrasonido`, `ir_pala`, `color_pala`, `acelerometro`, `giroscopio`.
- Sensores de línea simulados: ≈240 en blanco, 0–30 en negro.
- Los materiales de las ruedas (fl/rr `Exterior`, fr/rl `Interior`) hacen que la fricción mecanum funcione; no cambiarlos.
- Cuando se mida la pista real, actualizar el mundo y `docs/hardware.md`.

## Pendiente

- `webots_bridge` solo publica cámara y ultrasonido: falta `line`, `scoop/has_cube`, `scoop/color` y mover la pala.
- `scripts/setup_dev.sh`: por ahora el entorno recomendado es Docker (`docker/README.md`).
- Activar la protección de `main` y la CI en GitHub.
