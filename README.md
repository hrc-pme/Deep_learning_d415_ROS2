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
git clone --recursive git@github.com:hrc-pme/Deep_learning_d415_ROS2.git
```

> **Note**: The `--recursive` flag is required to clone submodules properly

### 2. Update Repository and Submodules

```bash
cd ~/Deep_Learning_d415_ROS2
git pull
git submodule sync --recursive
git submodule update --init --recursive
```

### 3. Copy the usb rule to machine
```bash
cd ~/Deep_learning_d415_ROS2
sudo cp .99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
```


## Start Docker Container

Choose the appropriate Docker setup based on your hardware:
- Use GPU version if you have a compatible graphics card
- Use CPU version for systems without GPU

### 1. GPU Version
 Before starting with GPU container check out your CUDA version first.

The default CUDA version will be 12.4.

```bash
# For GPU systems:
# Check your CUDA version first
nvidia-smi
```
> If you want to change your CUDA version Check on this repo. **[CUDA Version](https://github.com/hrc-pme/Deep_learning_d415_ROS2/blob/main/Tutorial/DockerImage.md#wrong-pytorch-gpu-version)**

If you have the correct version, you can open the container.

```bash
cd ~/Deep_Learning_d415_ROS2
chmod +x run_gpu.sh           # give the permission (you will only do one time)
./run_gpu.sh                  # default CUDA version 12.4
```

### 2. CPU Version
If you are using VMware or any virtual machine, you only can use CPU version.
```bash
# For CPU systems:
cd ~/Deep_Learning_d415_ROS2
chmod +x run_cpu.sh           # give the permission (you will only do one time)
./run_cpu.sh
```
### In both container the user ID will be 
```
user: hrc 
passward: 111111
```


## Build ROS 2 Workspace

Inside the Docker container:

```bash
cd  ~/Workspace/ros2_ws
colcon build --symlink-install
# Or simply use `cb` instead of typing the full command.
```
> **Note**: The alias `cb` had been defined as
`alias cb='colcon build --symlink-install`

## RealSense D415 Camera Setup

### 1. Find Camera Serial Number
```bash
rs-enumerate-devices -s
```
Expected Output
```
Device Name                   Serial Number       Firmware Version
Intel RealSense D415          241222061237        5.16.0.1
```

### 2. Configure Camera Serial Number
1. Open the launch file:
   ```bash
   ~/home/hrc/Workspace/ros2_ws/src/realsense-ros/realsense2_camera/launch/rs_launch.py

   # You can change multi setting here
   ```
2. Update the serial number:
   ```python
   {'name': 'serial_no', 'default': "'YOUR_CAMERA_SERIAL_NUMBER'", 'description': 'choose device by serial number'},
   ```
3. Rebuild the workspace:
   ```bash
   cb
   ```

### 3. Launch Camera

In Terminal 1:
```bash
ros2 launch realsense2_camera rs_launch.py
```

### 4. Visualize Camera Feed
Turn on the Rviz2 to visualize the image.

In Terminal 2:
```bash
cd ~/Deep_Learning_d415_ROS2
./run_[your_version].sh       #./run_gpu.sh OR ./run_cpu.sh 

rviz2
```
OR you can open rosbridge visualize in foxglove
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```


Then in RViz2:
1. Click "Add"
2. Navigate to: By topic → /camera → /camera → /color → /image_raw → /Image

### 5. Verify Camera Topics

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

## Recording ROS Bags

### 1. Configure Recording Settings

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

### 2. Start Recording
We will start to record a rosbg with camera topics.

Choose one of these methods:

Using GUI will be easier to record mulit bags.  

**Option 1: Using GUI**
```bash
ros2 launch bag_recorder record_with_ui.launch.py
```

**Option 2: Using Terminal**
```bash
ros2 launch bag_recorder record.launch.py
(Press Ctrl + C to stop recording)
```

## Convert ROSBags to MP4 Video
Edit the configuration file:
```bash
~/Workspace/Script/config/bags2mp4.yaml
```
Default configuration:
```yaml
jobs:
  - input_paths:                      # Paths to bag folders
      - "/home/hrc/Workspace/Outputs/rosbags/test"
    output_root: "/home/hrc/Workspace/Outputs/mp4/test"       # Output MP4 folder
    topic: "/camera/camera/color/image_raw/compressed"   
    # Image topic   If omit → auto-pick if only one; error if multiple
```  
> **NOTES**: If Image topic omit → auto-pick if only one; error if multiple

### Convert ROSBags to MP4 Video

```bash
cd ~/Workspace/Script
python3 bag2mp4.py
```
# Keep Going for Detection

## [Detection](https://github.com/hrc-pme/Deep_learning_d415_ROS2/blob/main/Tutorial/maskrcnn.md)
