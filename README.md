# Deep Learning D415 Repo


## 📁  Project Structure

```bash

```

## Development Environment
- ROS 2 Humble
- Ubuntu 22.04
- Python 3.10
- RealSense v2.56.5

## Setup

```
$: means command use in terminal  
#: means command use in docker terminal  
```

## 1.Clone the repo
```
$ cd ~
$ git clone --recursive git@github.com:LesterLiou/Deep_learning_d415_ROS2.git
```
* --recursive is the must have procedure for clone the submodule

## 2.Update repo and submodules

```
$ cd ~/Deep_Learning_D415_ROS2
~/Deep_Learning_D415_ROS2$ git pull
~/Deep_Learning_D415_ROS2$ git submodule sync --recursive
~/Deep_Learning_D415_ROS2$ git submodule update --init --recursive
```

## 3. Turn on the Docker
* Check the device you are using (if you have gpu please use gpu_run.sh, if you are none-gpu device, use cpu_run.sh)
```
~/Deep_Learning_D415_ROS2$ cd Docker/GPU
~/Deep_Learning_D415_ROS2$ ./build.sh
~/Deep_Learning_D415_ROS2$ cd ~/Deep_Learning_D415_ROS2
~/Deep_Learning_D415_ROS2$ ./run.sh
```

## 4. Build ROS2 Workspace
```
hrc@hrc-MS-7D98:~$ cb       # cb means colcon build
```

## 5. Find the Camera(D415)
```
rs-enumerate-devices -s # you can see the serial number of your D415 camera
```
### Update the repo
open VScode and find the ~/Deep_Learning_D415_ROS2/ros2_ws/src/realsense-ros/realsense2_camera/launch/rs_launch.py and fill in the serial_no blank.
```
{'name': 'serial_no', 'default': "'FILL_IN_HERE'", 'description': 'choose device by serial number'},
```
Remember to colcon build every time changing!
```
hrc@hrc-MS-7D98:~$ cb 
```

## 6. Terminal 1: Turn on D415 
```
ros2 launch realsense2_camera rs_launch.py
```

## 7. Terminal 2: Turn on Rviz for checking the image
```
rviz2 
```
Add
By topic
/camera
    /camera
        /color
            /image_raw
                Image