#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bag2mp4.py — Convert ROS 2 rosbag2 (.db3) image topics to MP4 (single or batch via YAML)

Usage:
  # Single bag
  python3 bag2mp4.py /path/to/bag_dir /path/to/out.mp4 --topic /camera/... --fps 30

  # Batch mode (YAML)
  python3 bag2mp4.py --config /home/hrc/Workspace/Script/config/bags2mp4.yaml
  # If --config is not specified, the default path is:
  #   /home/hrc/Workspace/Script/config/bags2mp4.yaml
"""

import argparse
import os
import sys
from typing import List, Optional, Tuple
import glob
import yaml

# ROS 2 / message imports
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import Image as RosImage
    from sensor_msgs.msg import CompressedImage as RosCompressedImage
except Exception as e:
    print("[ERROR] Missing ROS 2 Python packages. Please:")
    print("  source /opt/ros/$ROS_DISTRO/setup.bash")
    print(f"Details: {e}")
    sys.exit(1)

# OpenCV & numpy
try:
    import cv2
    import numpy as np
except Exception as e:
    print("[ERROR] Need OpenCV (cv2) and numpy.")
    print("Try: pip install opencv-python numpy")
    print(f"Details: {e}")
    sys.exit(1)

# --- Robust cv_bridge detection: fallback to pure OpenCV if broken ---
import importlib.util as _iu
_BRIDGE = None
try:
    if _iu.find_spec("cv_bridge") is not None:
        try:
            from cv_bridge import CvBridge  # type: ignore
            _BRIDGE = CvBridge()
        except Exception as e:
            print(f"[WARN] cv_bridge unavailable, falling back to pure-OpenCV: {e}")
            _BRIDGE = None
except Exception as _e:
    print(f"[WARN] cv_bridge detection failed, fallback to pure-OpenCV: {_e}")
    _BRIDGE = None
# -----------------------------------------------------------------

def ns_to_s(ns: int) -> float:
    return ns * 1e-9

def median_fps_from_timestamps(tstamps_ns: List[int]) -> float:
    if len(tstamps_ns) < 2:
        return 30.0
    diffs = np.diff(np.array(tstamps_ns, dtype=np.int64))
    diffs = diffs[diffs > 0]
    if len(diffs) == 0:
        return 30.0
    med = float(np.median(diffs))
    if med <= 0:
        return 30.0
    fps = 1.0 / (med * 1e-9)
    return float(np.clip(fps, 1.0, 120.0))

def is_image_topic(typename: str) -> bool:
    return typename in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")

def list_image_topics(bag_uri: str) -> List[Tuple[str, str]]:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_uri, storage_id='sqlite3'), ConverterOptions('', ''))
    topics = []
    for tinfo in reader.get_all_topics_and_types():
        if is_image_topic(tinfo.type):
            topics.append((tinfo.name, tinfo.type))
    return topics

def collect_timestamps(bag_uri: str, target_topic: str) -> List[int]:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_uri, storage_id='sqlite3'), ConverterOptions('', ''))
    timestamps = []
    try:
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic == target_topic:
                timestamps.append(int(t))
    except Exception as e:
        print(f"[WARN] While scanning timestamps: {e}")
    return timestamps

def open_reader(bag_uri: str) -> SequentialReader:
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_uri, storage_id='sqlite3'), ConverterOptions('', ''))
    return r

# --- Decode raw Image without cv_bridge for common encodings ---
def _decode_raw_image_without_bridge(msg: RosImage) -> np.ndarray:
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or "").lower()
    arr = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ("bgr8", "rgb8"):
        img = arr.reshape(h, w, 3)
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    if enc in ("mono8", "8uc1"):
        img = arr.reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    try:
        img = arr.reshape(h, w, -1)
        if img.shape[2] == 3:
            return img
    except Exception:
        pass
    raise RuntimeError(f"Unsupported Image encoding (no cv_bridge): {msg.encoding}")
# -----------------------------------------------------------

def decode_frame(typename: str, data: bytes) -> np.ndarray:
    """Return BGR image decoded from ROS message bytes."""
    if typename == "sensor_msgs/msg/CompressedImage":
        msg = deserialize_message(data, RosCompressedImage)
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("Failed to cv2.imdecode compressed image")
        return frame

    elif typename == "sensor_msgs/msg/Image":
        msg = deserialize_message(data, RosImage)
        if _BRIDGE is not None:
            try:
                if msg.encoding in ("rgb8", "rgba8", "bgr8", "bgra8"):
                    if msg.encoding == "rgb8":
                        cv_img = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    elif msg.encoding == "rgba8":
                        cv_img = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgra8")
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
                    elif msg.encoding == "bgra8":
                        cv_img = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgra8")
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
                    else:
                        cv_img = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                else:
                    cv_img = _BRIDGE.imgmsg_to_cv2(msg)
                    if len(cv_img.shape) == 2:
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
                return cv_img
            except Exception as e:
                print(f"[WARN] cv_bridge decode failed, fallback to pure-OpenCV: {e}")
        return _decode_raw_image_without_bridge(msg)

    else:
        raise ValueError(f"Unsupported image type: {typename}")

def ensure_dir_for_file(path: str):
    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)

# ---------------------- Batch utilities ----------------------
def is_rosbag2_folder(path: str) -> bool:
    if not os.path.isdir(path):
        return False
    meta = os.path.join(path, 'metadata.yaml')
    db3s = glob.glob(os.path.join(path, '*.db3'))
    return os.path.exists(meta) or len(db3s) > 0

def scan_bag_folders(root: str, recursive: bool = True) -> List[str]:
    if is_rosbag2_folder(root):
        return [root]
    pattern = '**/' if recursive else ''
    candidates = sorted(set(
        os.path.dirname(p) for p in glob.glob(os.path.join(root, pattern, '*.db3'), recursive=recursive)
    ))
    meta_dirs = sorted(set(
        os.path.dirname(p) for p in glob.glob(os.path.join(root, pattern, 'metadata.yaml'), recursive=recursive)
    ))
    return sorted(set(candidates) | set(meta_dirs))

def output_path_for_bag(bag_dir: str, output_root: str) -> str:
    base = os.path.basename(os.path.normpath(bag_dir))
    return os.path.join(output_root, f"{base}.mp4")
# ------------------------------------------------------

def convert_single_bag(
    bag_uri: str,
    out_mp4: str,
    topic: Optional[str] = None,
    fps: Optional[float] = None,
    codec: str = "mp4v",
    resize_wh: Optional[Tuple[int,int]] = None,
    overlay_timestamp: bool = False,
    start_time_sec: Optional[float] = None,
    end_time_sec: Optional[float] = None,
    max_frames: Optional[int] = None
):
    # Select topic
    topics = list_image_topics(bag_uri)
    if not topics:
        raise RuntimeError("No image topics found in the bag.")
    if topic:
        matches = [(n,t) for (n,t) in topics if n == topic]
        if not matches:
            raise RuntimeError(f"Specified topic not found. Available: {topics}")
        target_topic, target_type = matches[0]
    else:
        if len(topics) > 1:
            print(f"[INFO] Multiple image topics found; selecting the first by default: {topics[0][0]}")
        target_topic, target_type = topics[0]

    # FPS
    ts_list = collect_timestamps(bag_uri, target_topic)
    if not ts_list:
        raise RuntimeError("Selected topic has no frames.")
    bag_start_ns, bag_end_ns = ts_list[0], ts_list[-1]
    fps_val = median_fps_from_timestamps(ts_list) if fps is None else float(fps)

    # Time range
    sel_start_ns = bag_start_ns if start_time_sec is None else bag_start_ns + int(start_time_sec * 1e9)
    sel_end_ns   = bag_end_ns   if end_time_sec   is None else min(bag_end_ns, bag_start_ns + int(end_time_sec * 1e9))
    if sel_end_ns <= sel_start_ns:
        raise RuntimeError("Invalid time range: end_time_sec <= start_time_sec")

    # Prepare writer
    ensure_dir_for_file(out_mp4)
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer: Optional[cv2.VideoWriter] = None
    written = 0

    # Decode & write
    reader = open_reader(bag_uri)
    try:
        first_w, first_h = None, None
        while reader.has_next():
            topic_name, data, t_ns = reader.read_next()
            if topic_name != target_topic:
                continue
            if t_ns < sel_start_ns or t_ns > sel_end_ns:
                continue

            frame = decode_frame(target_type, data)
            if resize_wh is not None:
                frame = cv2.resize(frame, resize_wh, interpolation=cv2.INTER_AREA)

            if writer is None:
                h, w = frame.shape[:2]
                first_w, first_h = w, h
                writer = cv2.VideoWriter(out_mp4, fourcc, fps_val, (w, h))
                if not writer.isOpened():
                    raise RuntimeError("Failed to open VideoWriter. Try --codec mp4v.")
                print(f"[INFO] Opened writer: {out_mp4}  size={w}x{h}  fps={fps_val:.3f}  codec={codec}")

            if (frame.shape[1], frame.shape[0]) != (first_w, first_h):
                frame = cv2.resize(frame, (first_w, first_h), interpolation=cv2.INTER_AREA)

            if overlay_timestamp:
                rel_s = ns_to_s(t_ns - bag_start_ns)
                text = f"{rel_s:10.3f}s"
                cv2.putText(frame, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(frame, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,0), 1, cv2.LINE_AA)

            writer.write(frame)
            written += 1
            if max_frames is not None and written >= max_frames:
                print(f"[INFO] Reached max_frames={max_frames}; stop.")
                break
    finally:
        if writer is not None:
            writer.release()

    print(f"[OK] {bag_uri} -> {out_mp4}  ({written} frames @ {fps_val:.3f} fps)")

def run_from_yaml(cfg_path: str):
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    jobs = cfg.get('jobs', [])
    if not jobs:
        raise RuntimeError("YAML has no 'jobs'.")

    for job in jobs:
        input_paths: List[str] = job.get('input_paths', [])
        output_root: Optional[str] = job.get('output_root')
        if not input_paths or not output_root:
            print("[WARN] job missing input_paths or output_root; skip.")
            continue

        recursive = bool(job.get('recursive', True))
        topic = job.get('topic', None)
        fps = job.get('fps', None)
        overlay_timestamp = bool(job.get('overlay_timestamp', False))
        resize = tuple(job['resize']) if job.get('resize') else None
        codec = job.get('codec', 'mp4v')
        start_time_sec = job.get('start_time_sec', None)
        end_time_sec = job.get('end_time_sec', None)
        max_frames = job.get('max_frames', None)

        bag_dirs: List[str] = []
        for p in input_paths:
            if is_rosbag2_folder(p):
                bag_dirs.append(p)
            elif os.path.isdir(p):
                bag_dirs.extend(scan_bag_folders(p, recursive=recursive))
            else:
                print(f"[WARN] Path not found: {p}")

        if not bag_dirs:
            print("[WARN] No bag found for this job; skip.")
            continue

        os.makedirs(output_root, exist_ok=True)
        print(f"[INFO] Batch converting {len(bag_dirs)} bags to {output_root}")

        for b in bag_dirs:
            try:
                out_mp4 = output_path_for_bag(b, output_root)
                convert_single_bag(
                    bag_uri=b,
                    out_mp4=out_mp4,
                    topic=topic,
                    fps=fps,
                    codec=codec,
                    resize_wh=resize,
                    overlay_timestamp=overlay_timestamp,
                    start_time_sec=start_time_sec,
                    end_time_sec=end_time_sec,
                    max_frames=max_frames
                )
            except Exception as e:
                print(f"[ERROR] {b}: {e}")

def parse_resize(s: Optional[str]) -> Optional[Tuple[int,int]]:
    if not s:
        return None
    try:
        w_str, h_str = s.lower().split("x")
        return (int(w_str), int(h_str))
    except Exception:
        raise argparse.ArgumentTypeError("--resize must be like 1280x720")

def main():
    default_cfg = "/home/hrc/Workspace/Script/config/bags2mp4.yaml"

    ap = argparse.ArgumentParser(description="Convert ROS 2 rosbag2 (.db3) image topics to MP4 (single/batch)")
    ap.add_argument("bag", nargs="?", help="rosbag2 folder or .db3 (single conversion mode)")
    ap.add_argument("output", nargs="?", help="Output MP4 file (single conversion mode)")
    ap.add_argument("--topic", help="Image topic (raw or /compressed)")
    ap.add_argument("--fps", type=float, default=None, help="Output FPS (default: auto infer)")
    ap.add_argument("--codec", default="mp4v", help="FourCC: mp4v/avc1/H264 ... (depends on system)")
    ap.add_argument("--resize", type=str, default=None, help="Resize WxH (e.g. 1280x720)")
    ap.add_argument("--max-frames", type=int, default=None, help="Limit max frames")
    ap.add_argument("--overlay-timestamp", action="store_true", help="Overlay relative timestamp (s)")
    ap.add_argument("--start-time-sec", type=float, default=None, help="Start time (sec, relative to bag start)")
    ap.add_argument("--end-time-sec", type=float, default=None, help="End time (sec, relative to bag start)")
    ap.add_argument("--config", type=str, default=None, help=f"YAML config file (default: {default_cfg})")
    args = ap.parse_args()

    cfg_path = args.config if args.config else (default_cfg if os.path.exists(default_cfg) else None)
    if cfg_path and os.path.exists(cfg_path):
        run_from_yaml(cfg_path)
        return

    if not args.bag or not args.output:
        ap.print_help()
        sys.exit(1)

    bag_path = args.bag
    if os.path.isdir(bag_path):
        bag_uri = bag_path
    elif os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        bag_uri = os.path.dirname(os.path.abspath(bag_path))
    else:
        print(f"[ERROR] '{args.bag}' is neither a directory nor a .db3 file")
        sys.exit(1)

    resize_wh = parse_resize(args.resize)
    convert_single_bag(
        bag_uri=bag_uri,
        out_mp4=args.output,
        topic=args.topic,
        fps=args.fps,
        codec=args.codec,
        resize_wh=resize_wh,
        overlay_timestamp=args.overlay_timestamp,
        start_time_sec=args.start_time_sec,
        end_time_sec=args.end_time_sec,
        max_frames=args.max_frames
    )

if __name__ == "__main__":
    main()
