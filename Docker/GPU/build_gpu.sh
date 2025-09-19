#!/usr/bin/env bash

# ./build_gpu.sh humble-cuda12.8

set -euo pipefail

IMAGE_NAME="dl_lab_cuda"
DOCKERFILE="Dockerfile.gpu"
CONTEXT="."

# 1) Local build (keeps UID/GID tag for your own machine)
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
LOCAL_TAG="u${HOST_UID}-g${HOST_GID}"

echo ">>> Local build: ${IMAGE_NAME}:${LOCAL_TAG}"
docker build -f "${DOCKERFILE}" -t "${IMAGE_NAME}:${LOCAL_TAG}" "${CONTEXT}"
echo "✅ Local build complete: ${IMAGE_NAME}:${LOCAL_TAG}"

# 2) Prepare local publish-style tags (no push)
PUBLISH_USER="${DOCKERHUB_USER:-your_dockerhub_name}"
PUBLISH_REPO="${PUBLISH_USER}/dl_lab_cuda"
PUBLISH_TAG="${1:-humble-cuda12.8}"

echo ">>> Creating local tags (no push):"
echo "    - ${PUBLISH_REPO}:${PUBLISH_TAG}"
echo "    - ${PUBLISH_REPO}:latest"
docker tag "${IMAGE_NAME}:${LOCAL_TAG}" "${PUBLISH_REPO}:${PUBLISH_TAG}"
docker tag "${IMAGE_NAME}:${LOCAL_TAG}" "${PUBLISH_REPO}:latest"

echo " Done. Images available locally:"
echo "    ${IMAGE_NAME}:${LOCAL_TAG}"
echo "    ${PUBLISH_REPO}:${PUBLISH_TAG}"
echo "    ${PUBLISH_REPO}:latest"
echo ""
echo " If you later want to push manually, run:"
echo "    docker push ${PUBLISH_REPO}:${PUBLISH_TAG}"
echo "    docker push ${PUBLISH_REPO}:latest"
