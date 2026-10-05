# Guía: visión y color

**Dueño:** rol 2 · **Archivos:** `src/vision/`, `src/mobile_base/mobile_base/color_sensor.py`, `tools/hsv_tuner.py`

## Lo que hay que saber

- OpenCV usa H de 0 a 179. El rojo da la vuelta (cerca de 0 y cerca de 179).
- Colores de los cubos: rojo, naranja, amarillo, verde, azul y morado/vinotinto. Riesgo conocido: vinotinto contra rojo.
- La luz del laboratorio cambia: calibra con la luz real y guarda los rangos en `config/hsv_ranges.yaml`.
- La cámara va en la pala, cerca del suelo e inclinada hacia abajo; la fila del cubo en la imagen da una distancia aproximada.
- El TCS34725 de la pala confirma el color del cubo atrapado.
- Si se acuerdan colores reservados con el otro recolector, se configuran por parámetro (ver `docs/interfaz-externa.md`).
