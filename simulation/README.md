# simulation/

| Carpeta | Qué hay |
|---|---|
| `webots/` | mundo `pista_ucab.wbt`, PROTO `RecolectorUCAB` y `CuboMadera`, controlador por teclado `recolector_teleop` |

El mundo trae dos recolectores: uno es el nuestro y el otro sirve para ensayar con el recolector del otro equipo como obstáculo. Perry no está modelado.

Siguiente paso: conectar Webots con ROS 2 (`webots_ros2`) para que nuestros nodos muevan el robot simulado. Ver `agents/simulacion-integracion.md`.
