#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import torch 

# Detectron2
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer, ColorMode
from detectron2.data import MetadataCatalog


class RealTimeInferenceNode(Node):
    def __init__(self):
        super().__init__('realtime_detection_node')

        # ---- ROS parameters ----
        self.declare_parameter('in_image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('out_image_topic', '/camera/color/image_detection/compressed')
        self.declare_parameter('score_thresh', 0.8)
        self.declare_parameter('timer_period', 0.2)
        self.declare_parameter('device', 'auto')

        in_topic  = self.get_parameter('in_image_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_image_topic').get_parameter_value().string_value
        score_thr = float(self.get_parameter('score_thresh').get_parameter_value().double_value)
        period    = float(self.get_parameter('timer_period').get_parameter_value().double_value)
        device_req = self.get_parameter('device').get_parameter_value().string_value

        # ---- Detectron2 config  ----
        self.declare_parameter('task', 'instance')  # bbox | instance | keypoint | panoptic
        task = self.get_parameter('task').get_parameter_value().string_value

        cfg_map = {
            'bbox':     "COCO-Detection/faster_rcnn_R_50_FPN_3x.yaml",
            'instance': "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml", 
            #TODO: change the model to LVIS to be compared with the previous work
            # e.g. 'instance': "LVIS-InstanceSegmentation/mask_rcnn_R_50_FPN_1x.yaml",
            'keypoint': "COCO-Keypoints/keypoint_rcnn_R_50_FPN_3x.yaml",
            'panoptic': "COCO-PanopticSegmentation/panoptic_fpn_R_50_3x.yaml",
        }
        cfg_name = cfg_map.get(task, cfg_map['instance'])

        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file(cfg_name))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = score_thr
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(cfg_name)

        # ---- Select compute device (cpu/cuda/auto) ----
        if device_req not in ("cpu", "cuda", "auto"):
            self.get_logger().warn(f'Unknown device: "{device_req}", fallback to "auto"')
            device_req = "auto"
        if device_req == "cpu":
            device_sel = "cpu"
        elif device_req == "cuda":
            if torch.cuda.is_available():
                device_sel = "cuda"
            else:
                self.get_logger().warn('device:=cuda but CUDA is not available; falling back to CPU.')
                device_sel = "cpu"
        else:  # auto
            device_sel = "cuda" if torch.cuda.is_available() else "cpu"
        self.cfg.MODEL.DEVICE = device_sel

        # print of current device
        if device_sel == "cuda":
            try:
                name = torch.cuda.get_device_name(0)
                self.get_logger().info(f'Using CUDA: {name}')
            except Exception:
                self.get_logger().info('Using CUDA')
        else:
            self.get_logger().info('Using CPU')

        self.task = task
        self.predictor = DefaultPredictor(self.cfg)

        # ---- ROS comms ----
        self.bridge = CvBridge()
        self.current_image = None
        self.last_header = None
        self.inference_times = []

        self.sub = self.create_subscription(
            CompressedImage, in_topic, self.image_callback, qos_profile_sensor_data
        )
        
        self.pub_compressed = self.create_publisher(CompressedImage, out_topic, 1)
        self.pub_image = self.create_publisher(Image, out_topic.replace('/compressed',''), 1)

        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"Subscribed: {in_topic} | Publishing: {out_topic} | "
            f"score_thresh={score_thr} | period={period}s | device={device_sel}"
        )
        self.last_header = None
    

    def image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                self.current_image = img
                self.last_header = msg.header  
        except Exception as e:
            self.get_logger().warn(f"image_callback decode error: {e}")

    def timer_callback(self):
        if self.current_image is None or self.last_header is None:
            return

        start_t = time.time()
        outputs = self.predictor(self.current_image)
        annotated = self.draw_detections(outputs)

        dt = time.time() - start_t
        self.inference_times.append(dt)
        avg = sum(self.inference_times) / len(self.inference_times)
        self.get_logger().info(
            f"Inference Time: {dt:.4f}s (avg {avg:.4f}s). "
            f"Hint: set timer_period≈{round(dt, 1)}"
        )

        try:
            # 1) CompressedImage（Foxglove/RViz2 Camera display）
            cmsg = CompressedImage()
            cmsg.header.stamp = self.last_header.stamp  
            cmsg.header.frame_id = self.last_header.frame_id
            cmsg.format = 'jpeg'
            cbytes = cv2.imencode('.jpg', annotated)[1]
            cmsg.data = np.asarray(cbytes).tobytes()
            self.pub_compressed.publish(cmsg)

            # 2) Image raw（RViz2 Image display）
            imsg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            imsg.header.stamp = self.last_header.stamp
            imsg.header.frame_id = self.last_header.frame_id
            self.pub_image.publish(imsg)
        except Exception as e:
            self.get_logger().warn(f"publish error: {e}")

    def draw_detections(self, outputs):
        meta_name = self.cfg.DATASETS.TRAIN[0] if len(self.cfg.DATASETS.TRAIN) > 0 else "coco_2017_train"
        v = Visualizer(
            self.current_image[:, :, ::-1],
            metadata=MetadataCatalog.get(meta_name),
            scale=1.0,
            instance_mode=ColorMode.IMAGE if self.task == 'bbox' else ColorMode.SEGMENTATION
        )

        if self.task in ['bbox', 'instance', 'keypoint']:
            instances = outputs["instances"].to("cpu")
            # bbox mode: draw boxes + labels only, no masks
            if self.task == 'bbox' and instances.has("pred_masks"):
                instances.remove("pred_masks")
            v = v.draw_instance_predictions(instances)
            return v.get_image()[:, :, ::-1]

        elif self.task == 'panoptic':
            panoptic_seg, segments_info = outputs["panoptic_seg"]
            v = v.draw_panoptic_seg_predictions(panoptic_seg.to("cpu"), segments_info)
            return v.get_image()[:, :, ::-1]

        # fallback
        return self.current_image


def main():
    rclpy.init()
    node = RealTimeInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
