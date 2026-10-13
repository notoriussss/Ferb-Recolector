# Simulación de los recolectores en Webots

Mundo de Webots con la pista de UCAB Robótica, los 14 cubos y los dos recolectores (Phineas y Ferb) modelados según el plan. Sirve para probar movimientos, sensores y lógica antes de tener el robot armado.

Probado con **Webots R2025a**: carga sin errores, las ruedas mecanum avanzan, van de lado y giran, la pala empuja y retiene un cubo, y los sensores responden.

## Contenido

```
webots-recolector/
├── worlds/pista_ucab.wbt              mundo: mesa, tablero, línea, cuadrado, cubos y 2 robots
├── worlds/textures/pista_ucab.png      línea negra sacada de la foto de la pista (rectificada)
├── protos/RecolectorUCAB.proto         el robot (chasis, ruedas, pala y sensores)
├── protos/CuboMadera.proto             cubo de madera de 2,5 cm
├── controllers/recolector_teleop/      control por teclado para probar el modelo
└── vista_cenital.png                   captura del mundo desde arriba
```

## Cómo abrirlo

1. Instala Webots R2025a desde https://cyberbotics.com (Windows, Linux o macOS).
2. Instala Python 3 si no lo tienes. Webots lo usa para los controladores; en Windows, revisa en *Tools → Preferences → Python command* que apunte a tu `python`.
3. Abre `worlds/pista_ucab.wbt` con *File → Open World*.
4. Dale a ▶ (play). Haz clic en la vista 3D y usa el teclado.

| Tecla | Acción |
|---|---|
| W / S | avanzar / retroceder |
| A / D | moverse de lado (izquierda / derecha) |
| Q / E | girar a la izquierda / derecha |
| P | subir / bajar la pala |
| + / − | más / menos velocidad |

Los dos robots escuchan el mismo teclado. Para mover solo uno, cambia el campo `controller` del otro a `<none>` en el árbol de escena. En la consola de Webots, cada robot imprime cada medio segundo: si ve línea negra, la distancia del ultrasonido, si tiene un cubo en la pala, de qué color es y si es de sus colores.

## Qué tan fiel es al robot real

### Datos del fabricante o de las fotos
- **Chasis Hiwonder mecanum:** 186 × 161 mm, ruedas de 65 mm, motores TT 1:120 **sin encoder** (el modelo no tiene sensores de posición en las ruedas, igual que el real).
- **Velocidad:** ~100 rpm por motor (TT 1:120 a ~6 V) y límite del 70 % por la batería 2S → ~0,24 m/s como máximo.
- **Ruedas mecanum:** simuladas con fricción direccional a 45° (la misma técnica del KUKA youBot de Webots). Patinan algo al girar, como las reales.
- **Pista:** la línea negra se sacó de la foto rectificando la perspectiva. El ancho de 19 mm coincide con cinta aislante.
- **Reparto de colores del plan:** Phineas `rojo,naranja,amarillo` y Ferb `verde,azul,morado`, pasados en `controllerArgs`.

### Estimados (medir y ajustar)
| Medida | Valor en el modelo | Dónde se cambia |
|---|---|---|
| Tablero | 1,93 × 1,22 m | `PISTA` en el `.wbt` (y la textura) |
| Cuadrado de entrega | ~24 × 24 cm | sale de la textura |
| Cubos | 2,5 cm, 9,4 g | `CuboMadera.proto` |
| Masa del robot | ~0,89 kg en total | `masaCuerpo` en el robot |
| Torque de los TT | 0,15 N·m | `torqueMaxMotor` |
| Pala | U de 32 mm, a 2,5 mm del piso | `RecolectorUCAB.proto`, sección "Pala" |
| Cámara | 24° hacia abajo, a 11 cm de altura | `RecolectorUCAB.proto`, `Camera "camara"` |

Las medidas del tablero salen de comparar el ancho de la cinta en las dos direcciones de la foto. Si el tablero real mide otra cosa, avísame y ajusto la textura.

## Dispositivos del robot

Los nombres son los que usará el código, en Webots y después en ROS 2.

| Nombre | Pieza real | Qué devuelve |
|---|---|---|
| `motor_fl`, `motor_fr`, `motor_rl`, `motor_rr` | motores TT por el HAT Cokoino | se controlan por velocidad (rad/s) |
| `servo_pala` | SG90 | posición: 0 = abajo, −0,7 rad = arriba |
| `camara` | OV5647, 640×480, ~54° | imagen |
| `linea_fl`, `linea_fr`, `linea_rl`, `linea_rr` | TCRT5000 | ~240 en blanco, 0–30 en la línea negra |
| `ultrasonido` | HC-SR04 | distancia en metros (0,02 a 4) |
| `ir_pala` | FC-51 | 1 si hay cubo en la pala, 0 si no |
| `color_pala` | TCS34725 | imagen de 4×4 del cubo (promediar para sacar el RGB) |
| `acelerometro`, `giroscopio` | MPU6050 | m/s² y rad/s; no hay brújula, el rumbo se integra y deriva |

Arriba de cada robot hay un marcador ArUco (DICT_4X4_50: Phineas id 1, Ferb id 2), y al lado del cuadrado hay otro (id 10). El marcador del cuadrado depende de que las reglas lo permitan.

## Lo que el simulador no reproduce bien
- El patinaje exacto de las mecanum sobre el tablero real.
- Los colores con la luz del salón: los rangos HSV se calibran con los cubos reales.
- La caída de voltaje de la batería y el calentamiento de los motores.
- El robot de garra (Perry) no está modelado; el cuadrado queda libre.

## Conexión con ROS 2
`worlds/pista_ucab_ros.wbt` es el mismo mundo con Phineas en `<extern>` y Ferb quieto. En lugar de `webots_ros2` se usa un puente propio y pequeño, `webots_bridge` (`src/recolector_sim`): publica `image_raw` y `range`, y mueve las ruedas con `cmd_vel`, igual que los nodos del robot real. Así, lo que funciona en la simulación pasa a la Pi 5 cambiando solo los nodos que hablan con el hardware.

- Todo en Docker: `docker compose up sim` (Webots sin ventana) y abrir http://localhost:8080.
- Webots en Windows y nodos en Docker: abrir `pista_ucab_ros.wbt`, darle ▶ y `docker compose up sim-externo`.

Ver `docker/README.md`. Si cambias `pista_ucab.wbt`, corre `python3 tools/generar_mundo_ros.py`.
