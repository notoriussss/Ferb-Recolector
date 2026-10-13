# Imagen de desarrollo del recolector: ROS 2 Jazzy + OpenCV (+ Webots R2025a en la etapa "sim").
#
#   base -> lo mínimo para compilar y correr los nodos (sirve también en la Pi 5, arm64)
#   sim  -> base + Webots R2025a sin pantalla (solo amd64)
#
# Se usa desde compose.yaml (raíz del repo). Ver docker/README.md.

ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-opencv \
      python3-numpy \
      python3-pytest \
      python3-pip \
      curl \
 && rm -rf /var/lib/apt/lists/*

# ruff para el lint (mismo que la CI)
RUN pip3 install --no-cache-dir --break-system-packages ruff

# Webots y ROS se quejan si no hay usuario
ENV USER=ros

WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]


FROM base AS sim

ARG WEBOTS_VERSION=R2025a
RUN apt-get update \
 && curl -fsSL -o /tmp/webots.deb \
      "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/webots_2025a_amd64.deb" \
 && apt-get install -y --no-install-recommends /tmp/webots.deb xvfb \
 && rm -f /tmp/webots.deb \
 && rm -rf /var/lib/apt/lists/*

ENV WEBOTS_HOME=/usr/local/webots
# Webots sin GPU: render por software
ENV LIBGL_ALWAYS_SOFTWARE=1
