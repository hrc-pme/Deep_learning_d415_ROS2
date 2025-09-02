#!/bin/bash
set -e

# 專案路徑 (自動抓取 run.sh 所在的父目錄)
PROJECT_DIR=$(cd "$(dirname "$0")/.." && pwd)

# 容器名稱
CONTAINER_NAME=deep_learning_ros2_gpu

# 映像名稱 (記得 build.sh 裡面有設定)
IMAGE_NAME=dl_app_ros2-cuda:humble-cuda12.4

# 開啟 X11 (Linux 桌面)
xhost +local:root

docker run -it --rm \
    --gpus all \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \                       # 給 RealSense USB
    -v ${PROJECT_DIR}:/workspace \       # 掛載你的專案
    -w /workspace \                      # 預設工作目錄
    --name ${CONTAINER_NAME} \
    ${IMAGE_NAME} \
    bash
