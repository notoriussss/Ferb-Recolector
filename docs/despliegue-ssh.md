# Pasar el código al robot por SSH

Plan para llevar lo que hoy corre en la simulación (visión + acercarse a un cubo) a la
Raspberry Pi 5 del robot, trabajando desde la laptop por SSH. Va por fases: cada fase se
valida antes de pasar a la siguiente (regla del equipo: nada se prueba en el robot sin haber
pasado por la simulación).

En los ejemplos la Pi se llama `phineas` y el usuario es `ucab`; cámbialos por los suyos.

## Fase 0. La Pi lista para entrar por SSH

1. Con **Raspberry Pi Imager**, graba **Ubuntu Server 24.04 LTS (64 bits)** en la microSD.
   En las opciones (⚙):
   - nombre del equipo: `phineas`
   - usuario y contraseña: `ucab` / (la que quieran)
   - Wi‑Fi: el hotspot del equipo (ver `docs/interfaz-externa.md`)
   - activar SSH
2. Enciende la Pi y búscala desde la laptop (misma red):

   ```bash
   ping phineas.local           # si no responde, mira la IP en la lista del hotspot
   ssh ucab@phineas.local
   ```

3. Entra con llave en lugar de contraseña (en la laptop, una sola vez):

   ```bash
   # Linux / WSL / macOS
   ssh-keygen -t ed25519
   ssh-copy-id ucab@phineas.local
   ```

   ```powershell
   # Windows PowerShell (trae OpenSSH)
   ssh-keygen -t ed25519
   type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh ucab@phineas.local "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
   ```

4. Atajo en `~/.ssh/config` de la laptop:

   ```
   Host phineas
       HostName phineas.local      # o la IP fija
       User ucab
   ```

   Desde aquí basta con `ssh phineas`.

**Listo cuando:** `ssh phineas` entra sin pedir contraseña.

## Fase 1. Instalar ROS 2 Jazzy en la Pi (nativo)

En la Pi se instala nativo (no Docker): menos memoria y acceso directo a cámara y GPIO.
Son los pasos oficiales de ROS 2 Jazzy para Ubuntu 24.04 (arm64), versión base sin escritorio:

```bash
ssh phineas
sudo apt update && sudo apt install -y software-properties-common curl git tmux
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions \
    python3-rosdep python3-opencv python3-numpy v4l-utils
sudo rosdep init && rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

> Si los pasos oficiales cambian, manda la guía de ROS 2 Jazzy
> (docs.ros.org → Installation → Ubuntu deb packages). Cuando el rol 1 llene
> `scripts/setup_pi.sh`, estos comandos pasan a ese script.

**Listo cuando:** `ros2 doctor --report | head` corre sin errores.

## Fase 2. Llevar el código a la Pi

Hay dos caminos; usen **A** para lo que ya está en GitHub y **B** para probar cambios que
todavía no tienen commit.

**A. Clonar en la Pi** (lo normal):

```bash
ssh phineas
git clone https://github.com/notoriussss/Phineas-Recolector.git ~/recolector
cd ~/recolector && git checkout develop      # o la rama que quieran probar
# para actualizar después:  git pull
```

**B. Copiar desde la laptop con rsync** (cambios locales sin commit):

```bash
# desde la raíz del repo en la laptop (Linux o WSL)
rsync -avz --delete --exclude build --exclude install --exclude log --exclude .git \
    ./ phineas:~/recolector/
```

Compilar en la Pi (el paquete `recolector_sim` compila igual, pero solo se usa en la laptop):

```bash
ssh phineas
cd ~/recolector
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
echo "source ~/recolector/install/setup.bash" >> ~/.bashrc
```

**Listo cuando:** `ros2 pkg executables vision` muestra `camera_node`, `cube_detector` y
`web_viewer`.

## Fase 3. Cámara y visor (el robot no se mueve)

1. Conecta la cámara y revisa que el sistema la vea:

   ```bash
   v4l2-ctl --list-devices        # una webcam USB aparece como /dev/video0
   ```

   - **Webcam USB:** funciona directo con `device:=0`.
   - **Cámara de la Pi (OV5647, CSI):** en la Pi 5 pasa por libcamera, y OpenCV no la abre
     como `/dev/videoN`. `camera_node` acepta un pipeline de GStreamer en `device`
     (por ejemplo con `libcamerasrc`), pero **no está probado**: es una tarea del rol 2.
     Para empezar, usen una webcam USB.

2. Lanza solo la visión dentro de `tmux` (así sigue corriendo si se cae el SSH):

   ```bash
   ssh phineas
   tmux new -s robot
   ros2 launch bringup vision.launch.py device:=0
   # salir de tmux sin apagarlo: Ctrl+B y luego D.  Volver: tmux attach -t robot
   ```

3. Mira la cámara desde la laptop. Dos opciones:
   - misma red: abre `http://phineas.local:8080`
   - por túnel SSH (sirve aunque el puerto esté cerrado):

     ```bash
     ssh -N -L 8080:localhost:8080 phineas
     ```

     y abre `http://localhost:8080`.

4. Pon cubos delante del robot y revisa que las cajas salgan con el color correcto.

**Listo cuando:** el visor marca bien los 6 colores con la luz del laboratorio.

## Fase 4. Calibrar con el robot real

Los rangos HSV y la geometría de la cámara del YAML vienen de la **simulación**.

1. **Colores.** Cambia un rango en caliente y mira el visor al instante:

   ```bash
   ros2 param set /cube_detector hsv.rojo "[0, 110, 100, 6, 255, 255, 173, 110, 100, 179, 255, 255]"
   ```

   Cuando quede bien, copia los números a `src/vision/config/hsv_ranges.yaml` (en la laptop,
   con commit). El riesgo conocido es morado/vinotinto contra rojo.
2. **Cámara.** Mide la altura de la cámara sobre el piso, su inclinación y su campo de
   visión, y ponlos en `camera.*` del mismo YAML. Comprueba con una regla: pon un cubo a
   20 cm y mira que el visor diga ~20 cm.
3. **Distancia de parada.** Ajusta `stop_distance_m` en `src/mobile_base/config/recolector.yaml`
   para que el cubo quede justo delante de la pala.

## Fase 5. Que se mueva hacia el cubo

Hace falta lo que todavía no existe en el robot real:

- `motor_driver` (rol 1): `cmd_vel` → PWM de los 4 motores, con freno si no llegan comandos.
- `ultrasonic` (rol 1): sin `range`, `obstacle_guard` **no deja avanzar** (solo girar).

Cuando estén, se agregan a `bringup/launch/recolector.launch.py` y se prueba así:

1. Ruedas en el aire (robot sobre una caja).
2. `ros2 launch bringup recolector.launch.py` → arranca **en pausa**.
3. En el visor web, elige los colores y pulsa **Iniciar**. Revisa que las ruedas giren hacia
   el cubo; **Pausar** detiene todo.
4. En el piso, con la mano lista en el interruptor de la batería.

## Notas de red

- Todos los robots usan el mismo hotspot: pongan un `ROS_DOMAIN_ID` propio en la Pi
  (`echo "export ROS_DOMAIN_ID=<n>" >> ~/.bashrc`) para no mezclar tópicos con los otros
  equipos. El número se acuerda en `docs/interfaz-externa.md`.
- Para ver la cámara no hace falta ROS en la laptop: basta el navegador (el visor es HTTP).
- Si el multicast del hotspot falla y solo necesitan el visor, en la Pi pueden usar
  `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (todos los nodos corren en la Pi).

## Más adelante

- Servicio `systemd` para que `recolector.launch.py` arranque solo al encender la Pi.
- Script `scripts/deploy.sh` con el `rsync` + `colcon build` remoto de la fase 2.
