# simulation/

| Carpeta | Qué hay |
|---|---|
| `webots/` | mundo `pista_ucab.wbt`, PROTO `RecolectorUCAB` y `CuboMadera`, controlador por teclado `recolector_teleop` |

El mundo trae dos recolectores: uno es el nuestro y el otro sirve para ensayar con el recolector del otro equipo como obstáculo. Perry no está modelado.

Con ROS 2: el mundo `pista_ucab_ros.wbt` es igual, pero Phineas tiene controlador `<extern>` y lo maneja el nodo `webots_bridge` (`src/recolector_sim`). Se lanza con `ros2 launch bringup simulacion.launch.py` o con `docker compose up sim` (ver `docker/README.md`). Si cambias `pista_ucab.wbt`, regenera la variante con `python3 tools/generar_mundo_ros.py`.
