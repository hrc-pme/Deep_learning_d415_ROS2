#!/usr/bin/env bash
set -euo pipefail
set -x

IMAGE_NAME="${IMAGE_NAME:-dl_app_ros2-cuda:humble-cuda12.4}"
CONTAINER_NAME="${CONTAINER_NAME:-deep_learning_ros2_gpu}"

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST_WS="${ROOT_DIR}/ros2_ws"

mkdir -p "${HOST_WS}/src"

# 放行 X 給本機
xhost +local:root 1>/dev/null 2>&1 || true

docker run -it --rm \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  --net=host \
  --ipc=host \
  --privileged \
  --device=/dev/bus/usb:/dev/bus/usb \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v "${HOST_WS}":/home/hrc/ros2_ws:rw \
  -v "${ROOT_DIR}":/home/hrc/workspace:rw \
  -w /home/hrc \
  -v /dev:/dev \
  --group-add video \
  --name "${CONTAINER_NAME}" \
  "${IMAGE_NAME}" \
  bash
