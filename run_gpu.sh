#!/usr/bin/env bash
set -euo pipefail

# -------------------------------
# Config
# -------------------------------
CONTAINER_NAME="DL_lab_cuda"
HUB_USER="hrcnthu"
REPO_NAME="dl_lab_cuda"
AS_ROOT=0
TAG_DEFAULT="humble-cuda12.4"       # humble-cuda12.4 | humble-cuda12.8 
TAG="${TAG_DEFAULT}"

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    -r) AS_ROOT=1; shift ;;
    -h|--help)
      echo "Usage: $0 [-r] [TAG]"
      echo "  -r      join/exec as root (default: hrc)"
      echo "  TAG     image tag (default: ${TAG_DEFAULT})"
      exit 0 ;;
    *) TAG="$1"; shift; break ;;
  esac
done

IMAGE="${HUB_USER}/${REPO_NAME}:${TAG}"
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

# -------------------------------
# Helpers
# -------------------------------
is_running() {
  docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

join_as_user() {
  local user_flag home_var shell_in="bash"
  if [[ "${AS_ROOT}" -eq 1 ]]; then
    user_flag=(--user 0:0); home_var="/root"
  else
    user_flag=(--user hrc);  home_var="/home/hrc"
  fi
  # fallback if bash is unavailable
  if ! docker exec "${CONTAINER_NAME}" which bash >/dev/null 2>&1; then
    shell_in="sh"
  fi
  docker exec -it \
    -e HOME="${home_var}" -w "${home_var}" \
    "${user_flag[@]}" \
    "${CONTAINER_NAME}" "${shell_in}" -l
}

start_container() {
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
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e XDG_RUNTIME_DIR=/tmp/runtime-hrc \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${ROOT_DIR}":/home/hrc/Workspace:rw \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp/runtime-hrc:/tmp/runtime-hrc:rw \
    --name "${CONTAINER_NAME}" \
    "${IMAGE}" \
    /bin/bash
}

# -------------------------------
# Main logic: run-or-join
# -------------------------------
if is_running; then
  join_as_user
else
  start_container
fi
