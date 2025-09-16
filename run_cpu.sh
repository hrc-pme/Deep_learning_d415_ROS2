#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="DL_lab_cpu"
IMAGE_NAME="dl_lab_cpu"
TAG="${1:-u$(id -u)-g$(id -g)}"   
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ">>> Running CPU container: ${CONTAINER_NAME} from image: ${IMAGE_NAME}:${TAG}"
xhost +local:root 1>/dev/null 2>&1 || true

mkdir -p /tmp/runtime-hrc

docker run -it --rm \
  --net=host \
  --ipc=host \
  --privileged \
  --device=/dev/bus/usb:/dev/bus/usb \
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
  "${IMAGE_NAME}:${TAG}" \
  /bin/bash
