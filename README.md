# recolector-ucab

Robot recolector de Robótica UCAB. Es un robot mecanum que recoge cubos de madera dentro del área marcada por la línea negra de la pista y los empuja, uno a la vez con su pala, hasta el cuadrado negro de entrega. Lleva la cuenta de los cubos que entrega.

En la pista también trabajan otros robots (otro recolector y la garra que clasifica los cubos desde el cuadrado). **Son proyectos aparte, de otros equipos.** Este repositorio es solo nuestro robot; lo que haya que acordar con ellos se anota en [`docs/interfaz-externa.md`](docs/interfaz-externa.md).

> El código está vacío a propósito: cada archivo dice qué debe hacer y lleva un `TODO`. Lo escribe el equipo.
> Ya hay un primer pedazo: visión por color y acercarse a un cubo (sin pala), probado en la simulación.

## Tecnología

| Capa | Elección |
|---|---|
| Computador | Raspberry Pi 5, 8 GB |
| Sistema operativo | Ubuntu 24.04 LTS (Server en la Pi; Desktop o WSL2 en las laptops) |
| Middleware | ROS 2 Jazzy |
| Lenguaje | **Python 3.12** con `rclpy` |
| Visión | OpenCV (HSV) |
| GPIO | `lgpio` / `gpiozero` (RPi.GPIO no funciona en la Pi 5) |
| Parámetros | YAML |
| Simulación | Webots R2025a, luego `webots_ros2` |
| Calidad | `ruff`, `pytest`, GitHub Actions |

Por qué: [`docs/decisiones/`](docs/decisiones/).

## Estructura

```
recolector-ucab/
├── AGENTS.md                 reglas para asistentes de IA (Claude, Copilot, Codex…)
├── CLAUDE.md                 apunta a AGENTS.md
├── agents/                   guía por área para asistentes de IA y para revisar PR
├── docs/                     arquitectura, interfaz externa, equipo, hardware, plan y decisiones
├── src/                      workspace ROS 2 (colcon)
│   ├── recolector_interfaces/  mensajes propios
│   ├── mobile_base/            motores, línea, ultrasonido, pala, color y máquina de estados
│   ├── vision/                 cámara, detección de cubos y visor web
│   ├── recolector_sim/         puente Webots ↔ ROS 2 (solo simulación)
│   └── bringup/                launch files (robot real y simulación)
├── simulation/webots/        mundo de la pista y modelo del robot
├── docker/                   imagen de desarrollo (ROS 2 + Webots); se usa con compose.yaml
├── tools/                    calibración HSV, generar el mundo para ROS, control directo del HAT
├── scripts/                  instalación de la Pi y de las laptops, prueba del HAT
└── .github/                  CI, plantillas de PR e issues, CODEOWNERS
```

## Empezar

```bash
git clone <url-del-repo> ~/recolector-ucab
cd ~/recolector-ucab
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Simulación: abre `simulation/webots/worlds/pista_ucab.wbt` en Webots R2025a ([instrucciones](simulation/webots/README.md)).

### Con Docker (sin instalar ROS)

```bash
docker compose build
docker compose up sim        # Webots + nodos; abre http://localhost:8080 para ver la cámara
```

El robot simulado busca, se acerca al cubo más cercano de los colores elegidos y se detiene
antes de la pala. Detalles y otros modos en [`docker/README.md`](docker/README.md). Para
llevarlo a la Raspberry Pi: [`docs/despliegue-ssh.md`](docs/despliegue-ssh.md).

## Cómo trabajamos

- `main` siempre compila; nada se sube directo a `main`.
- Ramas `feature/…`, `fix/…`, `docs/…`; commits `area: qué`.
- Cada PR lo revisa otra persona del equipo.
- Código en inglés; comentarios y documentación en español.

Detalles en [`CONTRIBUTING.md`](CONTRIBUTING.md). Roles e hitos en [`docs/equipo.md`](docs/equipo.md).

## Documentos

- [Arquitectura](docs/arquitectura.md)
- [Interfaz con los otros robots](docs/interfaz-externa.md)
- [Equipo, roles e hitos](docs/equipo.md)
- [Hardware y conexiones](docs/hardware.md)
- [Pasar el código al robot por SSH](docs/despliegue-ssh.md)
- [Plan original del recolector](docs/plan-recolector.md) (incluye ideas sobre los otros robots que ya no son parte de este repo)
