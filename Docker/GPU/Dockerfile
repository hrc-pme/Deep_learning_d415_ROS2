###############################################################################
# Base image : CUDA 12 + cuDNN8 + Ubuntu 22.04（Jammy）
###############################################################################
FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

###############################################################################
# Locale & 使用者
###############################################################################
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

# 建立非 root 使用者
ARG NB_USER=hrc
ARG NB_UID=1000
ARG NB_GID=1000

RUN groupadd -g ${NB_GID} ${NB_USER} \
 && useradd -m -s /bin/bash -u ${NB_UID} -g ${NB_GID} ${NB_USER} \
 && echo "root:root" | chpasswd \
 && echo "${NB_USER}:111111" | chpasswd \
 && adduser ${NB_USER} sudo

ENV HOME=/home/${NB_USER}
WORKDIR ${HOME}

###############################################################################
# 基本工具
###############################################################################
RUN apt-get update && apt-get -yq dist-upgrade && \
    apt-get install -yq --no-install-recommends  \
        build-essential  cmake  git  curl  wget  vim  sudo \
        gnupg2  lsb-release  software-properties-common   \
        python3-pip  python3-venv  python3-dev  python3-setuptools \
        python3-numpy  python3-yaml  python3-opencv  python3-tk \
        locales  tzdata  net-tools  xterm  qtbase5-dev \
        libcanberra-gtk-module  libcanberra-gtk3-module \
        libusb-1.0-0-dev  libglfw3-dev  libglu1-mesa-dev \
        libpng-dev  libjpeg-dev  libtiff5-dev             \
        libxinerama-dev  libxft-dev  freeglut3-dev         \
        libfltk1.3-dev  libpcl1  libgeographic-dev         \
        bash-completion  xdotool  wmctrl                  \
    && locale-gen en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

###############################################################################
# 時區（預設設為 Asia/Taipei，可自行調整）
###############################################################################
RUN ln -fs /usr/share/zoneinfo/Asia/Taipei /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

###############################################################################
# ROS 2 Humble 套件庫
###############################################################################
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros2-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

###############################################################################
# 安裝 ROS 2 + Gazebo Classic 11（Jammy 內建）
###############################################################################
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop                \
        ros-${ROS_DISTRO}-rosbridge-suite        \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs        \
        ros-${ROS_DISTRO}-gazebo-ros2-control    \
        ros-${ROS_DISTRO}-robot-state-publisher  \
        ros-${ROS_DISTRO}-joint-state-publisher  \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-xacro                  \
        ros-${ROS_DISTRO}-rviz2                  \
        ros-${ROS_DISTRO}-teleop-twist-keyboard  \
        ros-${ROS_DISTRO}-teleop-twist-joy       \
        ros-${ROS_DISTRO}-tf2-tools              \
        ros-${ROS_DISTRO}-tf2-geometry-msgs      \
        ros-${ROS_DISTRO}-tf2-sensor-msgs        \
        ros-${ROS_DISTRO}-stereo-image-proc      \
        ros-${ROS_DISTRO}-image-tools            \
        ros-${ROS_DISTRO}-image-transport-plugins\
        python3-colcon-common-extensions         \
        python3-argcomplete                      \
        python3-rosdep                           \
    && rosdep init && rosdep update              \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
# Python 套件（含 PyTorch CUDA 12.1 版）
###############################################################################
RUN python3 -m pip install --upgrade pip && \
    pip install --no-cache-dir --ignore-installed \
        sympy mpmath && \
    pip install --no-cache-dir --ignore-installed \
        torch torchvision torchaudio \
        --extra-index-url https://download.pytorch.org/whl/cu128 && \
    pip install --no-cache-dir ultralytics && \
    pip cache purge || true

##################### 安裝 cv_bridge #####################
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-cv-bridge && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


###############################################################################
# RealSense 2.50.0（Jammy 支援 OK）
###############################################################################
ENV LIBREALSENSE_VERSION=2.50.0
RUN apt-get update && apt-get install -y --no-install-recommends \
      libssl-dev libglfw3-dev pkg-config udev && \
    git clone -b v${LIBREALSENSE_VERSION} https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j"$(nproc)" && make install && \
    ldconfig && \
    cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    cd ../.. && rm -rf librealsense


###############################################################################
# 環境設定
###############################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# 建立 hrc 的工作區
RUN mkdir -p /home/${NB_USER}/ros2_ws/src && chown -R ${NB_USER}:${NB_USER} /home/${NB_USER}/ros2_ws

# 給 root 與 hrc 都加上常用設定
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"        | tee -a /root/.bashrc >/dev/null \
 && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"     | tee -a /root/.bashrc >/dev/null \
 && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}"        | tee -a /root/.bashrc >/dev/null \
 && echo "alias cb='cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash'" \
        | tee -a /root/.bashrc >/dev/null

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"        | tee -a /home/${NB_USER}/.bashrc >/dev/null \
 && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"     | tee -a /home/${NB_USER}/.bashrc >/dev/null \
 && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}"        | tee -a /home/${NB_USER}/.bashrc >/dev/null \
 && echo "alias cb='cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash'" \
        | tee -a /home/${NB_USER}/.bashrc >/dev/null \
 && chown ${NB_USER}:${NB_USER} /home/${NB_USER}/.bashrc

# 之後預設工作目錄就用 hrc 的 ros2_ws
USER ${NB_USER}
WORKDIR /home/${NB_USER}
CMD ["/bin/bash"]
