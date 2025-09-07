# Deep Learning D415 Repo

This repository contains code for working with the Intel RealSense D415 camera in ROS 2 for deep learning applications.

## Development Environment

- ROS 2 Humble
- Ubuntu 22.04
- Python 3.10
- RealSense SDK v2.56.5

## Setup Instructions

### 1. Clone the Repository

```bash
cd ~
git clone --recursive git@github.com:LesterLiou/Deep_learning_d415_ROS2.git
```

> **Note**: The `--recursive` flag is required to clone submodules properly

### 2. Update Repository and Submodules

```bash
cd ~/Deep_Learning_D415_ROS2
git pull
git submodule sync --recursive
git submodule update --init --recursive
```

### 3. Start Docker Container

Choose the appropriate Docker setup based on your hardware:
- Use GPU version if you have a compatible graphics card
- Use CPU version for systems without GPU

```bash
# For GPU systems:
cd Docker/GPU
./build.sh
cd ~/Deep_Learning_D415_ROS2
./run.sh

# For CPU systems:
cd Docker/CPU
./build.sh
cd ~/Deep_Learning_D415_ROS2
./run.sh
```

### 4. Build ROS 2 Workspace

Inside the Docker container:

```bash
colcon build --symlink-install
# Or simply use `cb` instead of typing the full command.
```
> **Note**: The alias `cb` had been defined as
`alias cb='colcon build --symlink-install`

### 5. RealSense D415 Camera Setup

#### 5.1 Find Camera Serial Number
```bash
rs-enumerate-devices -s
```
Expected Output
```
Device Name                   Serial Number       Firmware Version
Intel RealSense D415          241222061237        5.16.0.1
```

#### 5.2 Configure Camera Serial Number
1. Open the launch file:
   ```
   ~/Deep_Learning_D415_ROS2/ros2_ws/src/realsense-ros/realsense2_camera/launch/rs_launch.py
   ```
2. Update the serial number:
   ```python
   {'name': 'serial_no', 'default': "'YOUR_CAMERA_SERIAL_NUMBER'", 'description': 'choose device by serial number'},
   ```
3. Rebuild the workspace:
   ```bash
   cb
   ```

#### 5.3 Launch Camera

In Terminal 1:
```bash
ros2 launch realsense2_camera rs_launch.py
```

#### 5.4 Visualize Camera Feed
Turn on the Rviz2 to visualize the image.

In Terminal 2:
```bash
rviz2
```

Then in RViz2:
1. Click "Add"
2. Navigate to: By topic → /camera → /camera → /color → /image_raw → /Image

#### 5.5 Verify Camera Topics

Check if camera topics are being published:
```bash
ros2 topic list
```

Expected topics:
```
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/image_raw/compressed
/camera/camera/color/image_raw/compressedDepth
/camera/camera/color/image_raw/theora
...
```

### 6. Recording ROS Bags

#### 6.1 Configure Recording Settings

Edit the configuration file:
```bash
~/Workspace/ros2_ws/src/bag_recorder/config/bag_recorder.yaml
```

Default configuration:
```yaml
topics:
  - /tf_static
  - /camera/camera/color/image_raw/compressed

output_dir: ~/Workspace/Outputs/rosbags/test
```

#### 6.2 Start Recording

Choose one of these methods:

**Option 1: Using GUI**
```bash
ros2 launch bag_recorder record_with_ui.launch.py
```

**Option 2: Using Terminal**
```bash
ros2 launch bag_recorder record.launch.py
(Press Ctrl + C to stop recording)
```