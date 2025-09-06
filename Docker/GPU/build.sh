#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="dl_lab_cuda"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
TAG="${1:-u${HOST_UID}-g${HOST_GID}}"
DOCKERFILE="Dockerfile"
CONTEXT="."

echo ">>> Building image: ${IMAGE_NAME}:${TAG} (UID=${HOST_UID}, GID=${HOST_GID})"

docker build \
  -f "${DOCKERFILE}" \
  --build-arg NB_UID="${HOST_UID}" \
  --build-arg NB_GID="${HOST_GID}" \
  -t "${IMAGE_NAME}:${TAG}" \
  "${CONTEXT}"

echo "✅ Build complete: ${IMAGE_NAME}:${TAG}"
