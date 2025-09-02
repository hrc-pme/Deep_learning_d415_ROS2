#!/usr/bin/env bash
set -euo pipefail

DEFAULT_IMAGE="dl_app_ros2-cuda"
DEFAULT_TAG="humble-cuda12.4"

INPUT="${1:-}"

IMAGE_NAME="$DEFAULT_IMAGE"
IMAGE_TAG="$DEFAULT_TAG"

if [[ -n "$INPUT" ]]; then
  if [[ "$INPUT" == *:* ]]; then
    IMAGE_NAME="${INPUT%%:*}"
    IMAGE_TAG="${INPUT##*:}"
  else
    IMAGE_NAME="$INPUT"
  fi
fi

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

echo "==> Building image: ${IMAGE_NAME}:${IMAGE_TAG}"
docker build \
  --build-arg NB_UID=$(id -u) \
  --build-arg NB_GID=$(id -g) \
  -t "${IMAGE_NAME}:${IMAGE_TAG}" .
echo "==> Done."
