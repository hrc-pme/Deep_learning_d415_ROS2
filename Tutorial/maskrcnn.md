# Mask R-CNN Detection with Detectron2
This tutorial will guide you through setting up and running Mask R-CNN for object detection using Facebook's Detectron2 framework. Before starting, ensure that you have the latest version of the repository and Docker image and that you can turn on the camera and record a ROS bag.

# Running Inference with a Pretrained Model

## Case 1: Real-time Detection

### **For real-time detection, make sure the camera is active (check [D415_tutorial](https://github.com/LesterLiou/Deep_learning_d415_ROS2/tree/main)) !!!**


### Terminal: Start Real-time Mask R-CNN Inference
```
$ cd ~/Deep_learning_d415_ROS2
ros2 launch detection realtime_detection.launch.py
```
> **NOTES**: You’ll see a prompt displaying the recommended timer duration to optimize the inference quality:
[INFO] [Time_stamp][realtime_detection_node]: Inference Time: 0000s (avg 0000s). Hint: set timer_period≈0.1 

You can change your setting through `realtime_detection.yaml`
```yaml
file will be located at 
/home/hrc/Workspace/ros2_ws/src/detection/config/realtime_detection.yaml

# Confidence threshold
score_thresh: 0.8

# Inference period (s)
timer_period: 0.01

# Model
task: "bbox"   # bbox | instance | keypoint | panoptic

# Computation device
device: "auto"     # cpu | cuda | auto
``` 

### Terminal: Check Detection Results
```
rviz2
```
1. Click "Add"
2. Navigate to: By topic → /color → /image_detection → /Image

OR using `foxglove`
1. Choose the topic: `/camera/color/image_detection/compressed`

#

## Case2: Using Colab for Online Resources
If you prefer to use online resources, check this Colab notebook after you turn your bag to mp4: [maskrcnn_tutorial.ipynb](https://colab.research.google.com/drive/1bfrT6zPpv6CYZ2ITMb3698nsrlaHv2p7?usp=drive_link)  

### 1. Please convert to MP4 video
(same step we just done)

[Convert ROSBags to MP4 Video](Workspace/README.md#convert-rosbags-to-mp4-video)


### 2. Upload the MP4 to Colab
Check this Colab notebook: [maskrcnn_tutorial.ipynb](https://colab.research.google.com/drive/1bfrT6zPpv6CYZ2ITMb3698nsrlaHv2p7?usp=drive_link)
Once you’ve successfully converted the bag file to an MP4 video, upload it to Colab for online inference.  
Note: Do not use "Run All"; execute each cell individually to monitor the process.  
After inference, the result video will appear on the left side of Colab. Download it to review the detection results.  