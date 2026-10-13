#!/usr/bin/env bash
# Carga ROS 2, compila el workspace montado en /ws (incremental) y ejecuta el comando.
set -e
source /opt/ros/jazzy/setup.bash

if [ -d /ws/src ] && [ "${SKIP_BUILD:-0}" != "1" ]; then
  echo "[entrypoint] colcon build..."
  (cd /ws && colcon build --symlink-install --event-handlers console_direct- >/tmp/colcon_build.log 2>&1) \
    || { cat /tmp/colcon_build.log; echo "[entrypoint] falló colcon build"; exit 1; }
fi
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

exec "$@"
