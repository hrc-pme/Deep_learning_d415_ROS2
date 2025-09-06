# ===================================================================
# Dockerfile for ROS 2 Humble + CUDA 12 + cuDNN8 + RealSense
# ===================================================================
FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

# ===================================================================       
# Locale & non root user
# ===================================================================
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

ARG NB_USER=hrc
ARG NB_UID=1000
ARG NB_GID=1000

RUN groupadd -g ${NB_GID} ${NB_USER} \
    && useradd -m -s /bin/bash -u ${NB_UID} -g ${NB_GID} ${NB_USER} \
    && echo "${NB_USER}:111111" | chpasswd \
    && adduser ${NB_USER} sudo

ENV HOME=/home/${NB_USER}
WORKDIR ${HOME}

# ===================================================================
# Timezone
# ===================================================================
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime


# ==================================================================
# Install basic packages
# ===================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ── Build tools ─────────────────────────────────────────
    build-essential cmake ninja-build pkg-config gdb \
    # ── System basics ──────────────────────────────────────
    curl wget git vim nano less sudo unzip \
    net-tools iputils-ping socat \
    gnupg2 dirmngr lsb-release software-properties-common \
    tzdata locales \
    # ── Python ─────────────────────────────────────────────
    python3-pip python3-venv python3-dev python3-setuptools \
    # ── Media/Graphics ─────────────
    libjpeg-dev libpng-dev libtiff5-dev zlib1g-dev libbz2-dev liblzma-dev \
    mesa-utils libegl1 freeglut3-dev libglu1-mesa-dev libglfw3-dev \
    libgtk-3-dev \
    # ── USB/Udev ───────────────────────────────────────────
    libusb-1.0-0-dev libudev-dev \
    # ── OpenCV ─────────────────────────
    libopencv-dev \
    # ── GStreamer ───────────────────────────────
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    # ── Qt/X11 ────────────────────────────
    qtbase5-dev libqt5x11extras5-dev libcanberra-gtk-module libcanberra-gtk3-module \
    x11-xserver-utils xdotool wmctrl xterm \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

# ===================================================================
# ROS 2 Humble KEY & Source
# ===================================================================
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros2-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# ===================================================================
# Install ROS 2 
# ===================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop                \
        ros-${ROS_DISTRO}-rosbridge-server       \
        ros-${ROS_DISTRO}-teleop-twist-keyboard  \
        ros-${ROS_DISTRO}-teleop-twist-joy       \
        ros-${ROS_DISTRO}-tf2-tools              \
        ros-${ROS_DISTRO}-tf2-geometry-msgs      \
        ros-${ROS_DISTRO}-tf2-sensor-msgs        \
        ros-${ROS_DISTRO}-stereo-image-proc      \
        ros-${ROS_DISTRO}-image-tools            \
        ros-${ROS_DISTRO}-image-transport-plugins\
        ros-${ROS_DISTRO}-vision-opencv         \
        ros-${ROS_DISTRO}-diagnostic-updater \
        python3-colcon-common-extensions         \
        python3-argcomplete                      \
        python3-rosdep                           \
    && rosdep init && rosdep update              \
    && rm -rf /var/lib/apt/lists/*

# ===================================================================
# Python pkg（with PyTorch CUDA 12.8 ）
# ===================================================================
RUN pip install --upgrade pip && \
    pip install numpy scipy matplotlib pandas \
    opencv-python opencv-python-headless \
    scikit-learn scikit-image Pillow \
    pyserial pexpect future pyquaternion pyyaml \
    tensorflow tensorboard && \
    pip install --no-cache-dir --ignore-installed \
      torch torchvision torchaudio \
      --index-url https://download.pytorch.org/whl/cu124 

# ===================================================================
# RealSense 2.56.0
# ===================================================================
ENV LIBREALSENSE_VERSION=2.56.5
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential \
    libssl-dev libusb-1.0-0-dev pkg-config udev \
    libglfw3-dev libgtk-3-dev libgl1-mesa-dev libglu1-mesa-dev && \
    git clone -b v${LIBREALSENSE_VERSION} https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j"$(nproc)" && make install && \
    ldconfig && \
    cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    cd ../.. && rm -rf librealsense && \
    rm -rf /var/lib/apt/lists/*

# ===================================================================
# NVIDIA Runtime
# ===================================================================
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# ===================================================================
# ADD pkg
# ===================================================================


# ===================================================================
# Workspace & shell 
# ===================================================================
RUN printf '%s\n' \
  'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
  'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}' \
  'source /opt/ros/${ROS_DISTRO}/setup.bash' \
  > /etc/profile.d/ros.sh && \
  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc && \
  { \
    # 方便編譯的 alias（含正確的 --log-base 放在子命令前）
    echo 'alias cb="( cd \"$HOME/Workspace/ros2_ws\" && colcon --log-base $HOME/.colcon/log build --merge-install --symlink-install && source install/setup.bash )"'; \
    # 固定 colcon log 位置
    echo 'mkdir -p $HOME/.colcon/log'; \
    echo 'export COLCON_LOG_PATH=$HOME/.colcon/log'; \
    # 自動載入 ROS base 與（若存在）overlay
    echo '[ -f /opt/ros/$ROS_DISTRO/setup.bash ] && source /opt/ros/$ROS_DISTRO/setup.bash'; \
    echo '[ -f $HOME/Workspace/ros2_ws/install/setup.bash ] && source $HOME/Workspace/ros2_ws/install/setup.bash'; \
  } >> /home/${NB_USER}/.bashrc && \
  # 準備 ~/.ros/log 並給正確權限
  mkdir -p /home/${NB_USER}/.ros/log && \
  chown -R ${NB_USER}:${NB_USER} /home/${NB_USER}

# 讓 ROS/launch 的 log 固定寫到 ~/.ros/log
ENV ROS_LOG_DIR=/home/${NB_USER}/.ros/log \
    LAUNCH_LOG_DIR=/home/${NB_USER}/.ros/log

# ===================================================================
# switch user
# ===================================================================

USER ${NB_USER}
WORKDIR /home/${NB_USER}
CMD ["/bin/bash"]
