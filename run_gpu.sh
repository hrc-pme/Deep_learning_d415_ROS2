#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="DL_lab_cuda"
HUB_USER="hrcnthu"                 
REPO_NAME="dl_lab_cuda"
TAG="${1:-humble-cuda12.8}"        # Default humble-cuda12.4, there are also: humble-cuda12.8
IMAGE="${HUB_USER}/${REPO_NAME}:${TAG}"

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ">>> Running container: ${CONTAINER_NAME} from image: ${IMAGE}"
xhost +local:root 1>/dev/null 2>&1 || true

mkdir -p /tmp/runtime-hrc

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "Image ${IMAGE} not found locally. Pulling from Docker Hub..."
  docker pull "${IMAGE}"
fi

docker run -it --rm \
  --gpus all \
  --net=host \
  --ipc=host \
  --privileged \
  --device=/dev/bus/usb:/dev/bus/usb \
  -e LOCAL_UID="$(id -u)" \
  -e LOCAL_GID="$(id -g)" \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp} \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  -e XDG_RUNTIME_DIR=/tmp/runtime-hrc \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${ROOT_DIR}":/home/hrc/Workspace:rw \
  -v /etc/localtime:/etc/localtime:ro \
  -v /tmp/runtime-hrc:/tmp/runtime-hrc:rw \
  --name "${CONTAINER_NAME}" \
  "${IMAGE}" \
  /bin/bash
