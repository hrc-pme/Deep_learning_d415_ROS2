# Mask R-CNN Detection with Detectron2
This tutorial will guide you through setting up and running Mask R-CNN for object detection using Facebook's Detectron2 framework. Before starting, ensure that you have the latest version of the repository and Docker image and that you can turn on the camera and record a ROS bag.

# Running Inference with a Pretrained Model

## Case 1: Real-time Detection

### **For real-time detection, make sure the camera is active (check [D415_tutorial](https://github.com/LesterLiou/Deep_learning_d415_ROS2/tree/main###5.3-Launch-Camera)) !!!**


### Terminal: Start Real-time Mask R-CNN Inference
```
ros2 launch detection realtime_detection.launch.py
```
> **NOTES**: You’ll see a prompt displaying the recommended timer duration to optimize the inference quality:
[INFO] [Time_stamp][realtime_detection_node]: Inference Time: 0000s (avg 0000s). Hint: set timer_period≈0.1 

You can change your setting through `realtime_detection.yaml`

file will be located at 
`/home/hrc/Workspace/ros2_ws/src/detection/config/realtime_detection.yaml`
```yaml
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

[Convert ROSBags to MP4 Video](https://github.com/LesterLiou/Deep_learning_d415_ROS2/tree/main/README.md#convert-rosbags-to-mp4-video)


### 2. Upload the MP4 to Colab
Check this Colab notebook: [maskrcnn_tutorial.ipynb](https://colab.research.google.com/drive/1bfrT6zPpv6CYZ2ITMb3698nsrlaHv2p7?usp=drive_link)
Once you’ve successfully converted the bag file to an MP4 video, upload it to Colab for online inference.  
Note: Do not use "Run All"; execute each cell individually to monitor the process.  
After inference, the result video will appear on the left side of Colab. Download it to review the detection results.  

#

## Model Select
In this section, we will provide instructions on how to change the model configuration.
You can use this section to compare different models or experiment with various training hyperparameters to observe their effects on detection performance.

### 1. Model ZOO
You can find all available models in the [MODEL_ZOO](https://github.com/hrc-pme/detectron2/blob/8d85329aed8506ea3672e3e208971345973ea761/MODEL_ZOO.md)
Please take some time to read through the documentation to understand each model’s details — including its training configuration, inference time, box AP, mask AP, and other performance metrics.

### 2. Changing the Model in Realtime Detection
In the file `realtime_detection_node.py` (`~/Workspace/ros2_ws/src/detection/detection/realtime_detection_node.py`)
```python
# ---- Detectron2 config  ----
        self.declare_parameter('task', 'instance')  # bbox | instance | keypoint | panoptic
        task = self.get_parameter('task').get_parameter_value().string_value

        cfg_map = {
            'bbox':     "COCO-Detection/faster_rcnn_R_50_FPN_3x.yaml",
            'instance': "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml", 
            # TODO: change the model to LVIS to be compared with the previous work
            # e.g., 'instance': "LVIS-InstanceSegmentation/mask_rcnn_R_50_FPN_1x.yaml",
            'keypoint': "COCO-Keypoints/keypoint_rcnn_R_50_FPN_3x.yaml",
            'panoptic': "COCO-PanopticSegmentation/panoptic_fpn_R_50_3x.yaml",
        }
        cfg_name = cfg_map.get(task, cfg_map['instance'])

        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file(cfg_name))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = score_thr
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(cfg_name)
```
You can modify the model by updating the Detectron2 configuration API call. All available model configuration files can be found in: 
```
~/Workspace/detectron2/configs/
```
To change the model, simply copy the desired YAML file path and replace it in the corresponding section of the code.
Make sure to match the model type with the correct task mode:

 •	bbox → Detection

 •	instance → Instance Segmentation

•	keypoint → Keypoint Detection

•	panoptic → Panoptic Segmentation

> **REMIND:** LVIS will only be selected for instance → Instance Segmentation

### 3. Inference Test
Now you can select your desired model and start performing the analysis! Try different configurations — including bbox, instance, keypoint, and panoptic modes — and compare the performance of models trained on COCO and LVIS datasets.

Before running the test, make sure to select the correct task in the configuration file: 
`/home/hrc/Workspace/ros2_ws/src/detection/config/realtime_detection.yaml`
```yaml
# Model
    task: "WHATYOUWANT"   # bbox | instance | keypoint | panoptic
```
> **NOTE:** Set task: "instance" when you want to compare COCO vs. LVIS Instance Segmentation performance.

