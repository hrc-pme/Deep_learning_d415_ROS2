#!/usr/bin/env bash
set -euo pipefail

# ====== 預設參數 ======
DEFAULT_IMAGE="dl_app_ros2-cuda"
DEFAULT_TAG="humble-cuda12.4"

# ====== 讀取 CLI 輸入 ======
# 格式： ./build.sh image:tag
INPUT="${1:-}"

if [[ -n "$INPUT" ]]; then
  # 如果有輸入 image:tag
  if [[ "$INPUT" == *:* ]]; then
    IMAGE_NAME="${INPUT%%:*}"
    IMAGE_TAG="${INPUT##*:}"
  else
    # 只輸入 image，tag 用預設
    IMAGE_NAME="$INPUT"
    IMAGE_TAG="$DEFAULT_TAG"
  fi
else
  # 都沒輸入 → 用預設
  IMAGE_NAME="$DEFAULT_IMAGE"
  IMAGE_TAG="$DEFAULT_TAG"
fi

# ====== BuildKit ======
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

echo "==> Building image"
echo "    IMAGE_NAME:TAG = ${IMAGE_NAME}:${IMAGE_TAG}"

docker build \
  -t "${IMAGE_NAME}:${IMAGE_TAG}" \
  .

echo "==> Done. Built ${IMAGE_NAME}:${IMAGE_TAG}"
