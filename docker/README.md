# Entorno con Docker

Una sola imagen con todo lo necesario para desarrollar sin instalar ROS en tu máquina:
Ubuntu 24.04, ROS 2 Jazzy, OpenCV, ruff y Webots R2025a (sin ventana).

| Archivo | Qué hace |
|---|---|
| `docker/Dockerfile` | etapa `base` (ROS + OpenCV, sirve también en arm64) y etapa `sim` (+ Webots) |
| `docker/entrypoint.sh` | carga ROS, compila el workspace con `colcon build` y corre el comando |
| `compose.yaml` (raíz) | servicios `sim`, `sim-externo` y `dev` |

El repo se monta en `/ws`: lo que edites en tu carpeta se usa al instante (cada arranque
recompila lo que cambió). `build/`, `install/` y `log/` quedan en volúmenes de Docker.

## Requisitos

- **Windows:** Docker Desktop con el backend de WSL2.
- **Linux:** Docker Engine con el plugin `compose`.
- ~4 GB libres para la imagen. La primera construcción tarda unos minutos (descarga Webots).

```bash
docker compose build          # una vez, o cuando cambie docker/Dockerfile
```

## 1. Todo dentro de Docker (lo más fácil)

```bash
docker compose up sim
```

Arranca Webots sin ventana con `pista_ucab_ros.wbt`, el puente con ROS y los nodos.
Abre **http://localhost:8080**: ves la cámara del robot con las cajas de los cubos, eliges
los colores objetivo, la distancia de parada y lo pausas o lo inicias.

El robot gira buscando, se acerca al cubo más cercano de los colores elegidos y se detiene
a `stop_distance_m` (0,17 m de la cámara por defecto, unos 5 cm delante de la pala).

> Webots sin GPU dibuja por software: la cámara va a ~6–10 cuadros por segundo y a veces se
> traba un instante (el estado pasa a `SIN_CAMARA` y el robot se queda quieto, a propósito).

## 2. Webots en tu máquina, nodos en Docker (para ver el robot moverse)

1. Instala Webots R2025a en Windows o Linux (https://cyberbotics.com).
2. Abre `simulation/webots/worlds/pista_ucab_ros.wbt` y dale a ▶. La consola dirá
   `Waiting for local or remote connection on port 1234 targeting robot named 'Phineas'`.
3. En otra terminal:

   ```bash
   docker compose up sim-externo
   ```

   El puente se conecta a `tcp://host.docker.internal:1234/Phineas`. Si Webots está en otra
   máquina: `WEBOTS_HOST=192.168.x.y docker compose up sim-externo`.
4. Abre http://localhost:8080.

Si el puente dice `Cannot connect to Webots instance`, revisa que Webots esté corriendo con
la simulación en marcha y que el firewall de Windows deje pasar el puerto 1234.

## 3. Consola para compilar, probar y lint

```bash
docker compose run --rm dev
# dentro:
colcon test && colcon test-result --verbose
ruff check src tools && ruff format --check src tools
ros2 topic list
```

Para abrir otra consola en un contenedor que ya corre: `docker compose exec sim bash`
(y adentro `source /ws/install/setup.bash`).

## Comandos útiles

```bash
docker compose exec sim bash -c "source /ws/install/setup.bash && ros2 topic echo /cubes --once"
docker compose exec sim bash -c "source /ws/install/setup.bash && \
  ros2 param set /behavior_fsm target_colors '[azul, verde]'"
docker compose down            # apaga todo
docker compose down -v         # además borra los volúmenes de build/install/log
```

## Detalles

- La imagen base se puede cambiar con `--build-arg BASE_IMAGE=...` (por ejemplo, un espejo
  de Docker Hub si da `429 Too Many Requests`: `mirror.gcr.io/library/ros:jazzy-ros-base`).
- La etapa `sim` solo existe para amd64 (Webots no publica paquete arm64). En la Pi se usa
  la instalación nativa: ver [`docs/despliegue-ssh.md`](../docs/despliegue-ssh.md).
