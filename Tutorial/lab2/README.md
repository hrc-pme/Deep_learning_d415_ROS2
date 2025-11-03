# Lab-2: Object Detection and Segmentation

**Lester, Ching-I**

In this lab, we will explore real-time object detection using bounding boxes and segmentation, utilizing data recorded via rosbag. We will also cover person keypoint detection.

The tutorial includes two parts:

1. **Object detection**
   1. **Converting ROS Bag to MP4**
   2. **Case 1: Realtime detection**
   3. **Case 2: MP4-based detection**
   4. **Change the model: compare different algorithms and different datasets**
2. **Person Keypoint Detection**
3. **Panoptic Segmentation**

If you are using a virtual machine platform or you don't have GPU, you can follow these two steps: first, record a bag, and then play it online via Colab.

Please follow the instructions:
[https://github.com/hrc-pme/Deep_learning_d415_ROS2/blob/main/Tutorial/maskrcnn.md](https://github.com/hrc-pme/Deep_learning_d415_ROS2/blob/main/Tutorial/maskrcnn.md)

# 1. Object detection

## 1.1 Converting ROS Bag to MP4 ([ROSBag2MP4](https://github.com/hrc-pme/Deep_learning_d415_ROS2/tree/main?tab=readme-ov-file#convert-rosbags-to-mp4-video))

If you plan to perform MP4-based detection, please make sure you have followed the steps below to convert your ROS bag into an MP4 video. If you are instead interested in real-time detection, you may skip this section and go directly to **1.2**.

## 1.2 Case 1: Real-time Detection ([real-time_detection](https://github.com/hrc-pme/Deep_learning_d415_ROS2/blob/main/Tutorial/maskrcnn.md#case-1-real-time-detection))

In this section, we provide **four predefined object detection modes** for students to experiment with. You can modify the parameter under **#Model task:** to switch between the following options:

- **bbox** – Bounding Box Detection
- **instance** – Instance Segmentation
- **keypoint** – Keypoint Detection
- **panoptic** – Panoptic Segmentation

All four modes are trained using the **COCO dataset**. However, in this lab's **checkpoint**, you are also required to test the **Instance Segmentation model trained on the LVIS dataset** and compare its performance with the COCO-based model.

In the **1.4** section, we will show you **where and how to change the model configuration** to switch between the COCO and LVIS checkpoints.

## 1.3 Case 2: MP4-based Detection ([Colab_Detection](https://github.com/hrc-pme/Deep_learning_d415_ROS2/blob/main/Tutorial/maskrcnn.md#case2-using-colab-for-online-resources))

In this section, you will learn how to perform object detection using an MP4 video that was previously converted from a ROS bag. The process will be conducted on Google Colab. (If you don't have GPU it is recommend to use Colab)

### 1.3.1 Upload

After converting your ROS bag file into an MP4 video, upload the MP4 file to Colab to run the object detection task. Make sure you upload the correct video file in the designated cell.

### 1.3.2 Initialization

Before starting detection, **verify that the file name is correct**. Check the filename both in the Colab notebook and in your local script (bag2mp4.py) to ensure they match exactly.

### 1.3.3 Detection (Just Follow the Colab instruction)

Run the detection section in Colab and follow the step-by-step instructions provided in the notebook. This will perform object detection on the uploaded MP4 video.

### 1.3.4 Segmentation

Next, execute the segmentation section of the notebook. If you are using Colab, you will need to implement how to visualize the segmentation masks yourself. (It's not difficult if you're unsure, you can refer to the example code used in the Realtime Detection section for guidance.)

## 1.4 Change the model: Compare different algorithms and different datasets

In this section, we will provide instructions on how to change the model configuration. You can use this section to compare different models or experiment with various training hyperparameters to observe their effects on detection performance.

### 1.4.1 Model Zoo

You can find all available models in the [MODEL_ZOO.md](https://github.com/hrc-pme/detectron2/blob/8d85329aed8506ea3672e3e208971345973ea761/MODEL_ZOO.md). Please take some time to read through the documentation to understand each model's details — including its training configuration, inference time, box AP, mask AP, and other performance metrics.

### 1.4.2 Changing the Model in Realtime Detection

In the file `realtime_detection_node.py` (`~/Workspace/ros2_ws/src/detection/detection/realtime_detection_node.py`), you can modify the model by updating the Detectron2 configuration API call. All available model configuration files can be found in: (`~/Workspace/detectron2/configs/`)

To change the model, simply copy the desired YAML file path and replace it in the corresponding section of the code.

Make sure to match the model type with the correct task mode:

- **bbox** → Detection
- **instance** → Instance Segmentation
- **keypoint** → Keypoint Detection
- **panoptic** → Panoptic Segmentation

(**REMIND:** LVIS will only be selected for instance → Instance Segmentation)

Now you can select your desired model and start performing the analysis! Try different configurations — including bbox, instance, keypoint, and panoptic modes — and compare the performance of models trained on COCO and LVIS datasets.

Before running the test, make sure to select the correct task in the configuration file:
(`/home/hrc/Workspace/ros2_ws/src/detection/config/realtime_detection.yaml`)

### 1.4.3 Changing the Model in MP4-based detection

It follows the same idea as the Realtime detection. You only need to change the model path to the one you want from the Model Zoo. Just make sure that the model you select matches the correct mode — either Detection or Segmentation — depending on your task setting.

# 2. Person Keypoint Detection

Use the COCO Person Keypoint Detection Baselines with Keypoint R-CNN to detect and visualize human body keypoints such as the eyes, nose, shoulders, elbows, wrists, hips, knees, and ankles.
This task focuses on identifying specific joints or landmarks of each person in the image and drawing skeleton-like connections between them.

# 3. Panoptic Segmentation

Use the COCO Panoptic Segmentation Baselines to perform comprehensive image understanding that combines both semantic segmentation (classifying every pixel) and instance segmentation (separating individual objects).
The model provides a complete scene interpretation, labeling both background areas (like sky, road) and foreground objects (like people, cars).

# Check points

Your lab report (**Lab2_studentID_Name.pdf**) should include the following checkpoints.

All **videos must be uploaded to YouTube**, and the **links** should be included in your report.

**Do not submit video files directly.**

### 1. Object Detection Comparison (55 points)

Prepare two videos demonstrating:

1. Bounding Box Detection using a RetinaNet or Faster R-CNN model.
2. Instance Segmentation using a Mask R-CNN model.

In your report, analyze the differences between the bounding box and segmentation results, including visual differences, accuracy, and inference speed, and discuss how the algorithms and COCO-based training datasets influence performance.

### 2. Dataset Comparison (45 points)

Prepare two videos demonstrating:

1. Instance Segmentation using Mask R-CNN (COCO dataset).
2. Instance Segmentation using Mask R-CNN (LVIS baseline).

In your report, analyze the differences between COCO and LVIS models, focusing on class diversity, segmentation quality, and detection performance.

### 3. Bonus (Ubuntu User): Real-time Detection (5 points)

If you are using an Ubuntu system, attempt to perform real-time inference using a keypoint model and record a ROS bag. Use the command below to verify that the recorded topics are correct:

```bash
ros2 bag info your_recorded_file.bag
```

Include a **screenshot of the ROS bag info and a frame of the detection result** in your report.


### 4. Bonus (WSL User): MP4-based Detection_Segmentation (5 points)

Try to complete the TODO part in the segmentation section of the Colab notebook — for example, displaying segmentation masks on the detected objects.

Include a **screenshot of the Segmentation Inference and a frame of the detection result** in your report.
