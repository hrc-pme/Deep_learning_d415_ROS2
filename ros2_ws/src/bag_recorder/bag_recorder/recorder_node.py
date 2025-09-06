#!/usr/bin/env python3
import os
import re
import shlex
import signal
import subprocess
import time
import json
import yaml
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType
from std_srvs.srv import Trigger
from std_msgs.msg import String


class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')

        # 可由 launch 傳入（建議）；也可以在這裡寫死路徑
        self.declare_parameter('config_yaml', '')

        # --- 明確型別宣告，避免被推斷成 byte[] ---
        self.declare_parameter(
            'topics',
            [],  # !! 必須給真正的空 list
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter('output_dir', '~/rosbags')
        self.declare_parameter('storage', 'sqlite3')      # 或 mcap（需安裝外掛）
        self.declare_parameter('compression', 'none')      # none 或 zstd
        self.declare_parameter('compression_mode', 'file') # file 或 message
        self.declare_parameter('max_bag_size', 0)          # bytes；0=不切
        self.declare_parameter('max_bag_duration', 0)      # seconds；0=不切
        self.declare_parameter('qos_profile_overrides_path', '')

        # 只在 -a 模式會用到
        self.declare_parameter('include_regex', '')
        self.declare_parameter('exclude_regex', '')
        self.declare_parameter('allow_all', False)         # 允許 topics 空時走 -a？
        self.declare_parameter('index_width', 4)           # 連號寬度

        self.add_on_set_parameters_callback(self._on_param_set)

        # 狀態發布
        self.status_pub = self.create_publisher(String, 'bag_recorder/status', 10)
        self.status_timer = self.create_timer(0.5, self._publish_status)

        # 服務：Start / Stop
        self._start_srv = self.create_service(Trigger, 'start_recording', self.start_recording_cb)
        self._stop_srv  = self.create_service(Trigger, 'stop_recording',  self.stop_recording_cb)

        self._proc = None
        self._current_bag_path = None
        self._start_epoch = None

        # 載入 YAML（若提供）
        cfg_path = self.get_parameter('config_yaml').value or ''
        if cfg_path:
            self._load_yaml(cfg_path)

        self.get_logger().info('bag_recorder ready. Use /start_recording and /stop_recording')

    # -------- YAML loader（手動帶型別，與 Humble 相容） --------
    def _load_yaml(self, path: str):
        try:
            path = os.path.expanduser(path)
            if not os.path.isfile(path):
                self.get_logger().warn(f'Config not found, using defaults (may record ALL): {path}')
                return
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f) or {}

            params: list[Parameter] = []

            # topics: string[]
            if 'topics' in cfg:
                if not isinstance(cfg['topics'], (list, tuple)):
                    raise ValueError('topics must be a list of strings')
                topics_list = [str(x) for x in cfg['topics']]
                params.append(Parameter(name='topics', type_=Parameter.Type.STRING_ARRAY, value=topics_list))

            # strings
            for key in ['output_dir', 'storage', 'compression', 'compression_mode',
                        'qos_profile_overrides_path', 'include_regex', 'exclude_regex']:
                if key in cfg and cfg[key] is not None:
                    val = str(cfg[key])
                    if key == 'output_dir':
                        val = os.path.expanduser(val)
                    params.append(Parameter(name=key, type_=Parameter.Type.STRING, value=val))

            # integers
            for key in ['max_bag_size', 'max_bag_duration', 'index_width']:
                if key in cfg and cfg[key] is not None:
                    params.append(Parameter(name=key, type_=Parameter.Type.INTEGER, value=int(cfg[key])))

            # booleans
            if 'allow_all' in cfg and cfg['allow_all'] is not None:
                params.append(Parameter(name='allow_all', type_=Parameter.Type.BOOL, value=bool(cfg['allow_all'])))

            if params:
                self.set_parameters(params)
            self.get_logger().info(f'Loaded config from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YAML {path}: {e}')

    def _on_param_set(self, params):
        return SetParametersResult(successful=True)

    # -------- Helpers --------
    @staticmethod
    def _next_index_dir(base_dir: str, width: int = 4) -> str:
        os.makedirs(base_dir, exist_ok=True)
        nums = []
        for name in os.listdir(base_dir):
            if os.path.isdir(os.path.join(base_dir, name)) and re.fullmatch(r'\d{'+str(width)+r'}', name):
                nums.append(int(name))
        nxt = (max(nums) + 1) if nums else 1
        return f'{nxt:0{width}d}'

    # -------- Start / Stop --------
    def start_recording_cb(self, req, res):
        if self._proc and self._proc.poll() is None:
            res.success = False
            res.message = f'Already recording: {self._current_bag_path}'
            return res

        # 讀參數
        topics = list(self.get_parameter('topics').value or [])
        output_dir = os.path.expanduser(self.get_parameter('output_dir').value or '~/rosbags')
        storage = (self.get_parameter('storage').value or 'sqlite3').lower()
        compression = (self.get_parameter('compression').value or 'none').lower()
        compression_mode = (self.get_parameter('compression_mode').value or 'file').lower()
        max_bag_size = int(self.get_parameter('max_bag_size').value or 0)
        max_bag_duration = int(self.get_parameter('max_bag_duration').value or 0)
        qos_overrides = self.get_parameter('qos_profile_overrides_path').value or ''
        include_regex = self.get_parameter('include_regex').value or ''
        exclude_regex = self.get_parameter('exclude_regex').value or ''
        allow_all = bool(self.get_parameter('allow_all').value or False)
        index_width = int(self.get_parameter('index_width').value or 4)

        # 連號資料夾 + 檔名同號
        folder = self._next_index_dir(output_dir, width=index_width)
        bag_path = os.path.join(output_dir, folder, folder)
        os.makedirs(os.path.dirname(bag_path), exist_ok=True)

        # 組命令
        cmd = ['ros2', 'bag', 'record']

        if topics:
            # 僅錄指定 topics
            self.get_logger().info(f"Topics to record: {topics}")
            cmd += topics
        else:
            if not allow_all and not include_regex and not exclude_regex:
                res.success = False
                res.message = 'No topics provided. Refusing to record ALL (-a). Set allow_all:=true or include/exclude regex, or specify topics.'
                return res
            # 允許 -a，並可附帶 include/exclude regex
            sel_info = ['ALL (-a)']
            cmd += ['-a']
            if include_regex:
                cmd += ['-e', include_regex]
                sel_info.append(f'include_regex="{include_regex}"')
            if exclude_regex:
                cmd += ['-x', exclude_regex]
                sel_info.append(f'exclude_regex="{exclude_regex}"')
            self.get_logger().warn('Recording by selection: ' + ', '.join(sel_info))

        cmd += ['-o', bag_path, '--storage', storage]

        # 壓縮
        if compression and compression != 'none':
            cmd += ['--compression-mode', compression_mode, '--compression-format', compression]

        # 分檔
        if max_bag_size > 0:
            cmd += ['--max-bag-size', str(max_bag_size)]
        if max_bag_duration > 0:
            cmd += ['--max-bag-duration', str(max_bag_duration)]

        # QoS 覆寫
        if qos_overrides:
            cmd += ['--qos-profile-overrides-path', qos_overrides]

        self.get_logger().info('Starting: ' + ' '.join(shlex.quote(c) for c in cmd))

        try:
            # 建立子程序群組，方便優雅停止
            self._proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self._current_bag_path = os.path.dirname(bag_path)  # 指向資料夾
            self._start_epoch = time.time()
            res.success = True
            res.message = f'Started recording: {self._current_bag_path}'
        except Exception as e:
            res.success = False
            res.message = f'Failed to start: {e}'
        return res

    def stop_recording_cb(self, req, res):
        if not self._proc or self._proc.poll() is not None:
            res.success = False
            res.message = 'Not recording.'
            return res
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            self._proc.wait(timeout=10)
            bag_path = self._current_bag_path
            self._proc = None
            self._current_bag_path = None
            self._start_epoch = None
            res.success = True
            res.message = f'Stopped. Saved at: {bag_path}'
        except Exception as e:
            res.success = False
            res.message = f'Failed to stop: {e}'
        return res

    def _publish_status(self):
        now = time.time()
        is_recording = self._proc is not None and self._proc.poll() is None
        elapsed = int(now - self._start_epoch) if (self._start_epoch and is_recording) else 0
        msg = {
            'is_recording': is_recording,
            'bag_path': self._current_bag_path or '',
            'start_time_iso': datetime.fromtimestamp(self._start_epoch, tz=timezone.utc).isoformat() if self._start_epoch else '',
            'elapsed_seconds': elapsed
        }
        self.status_pub.publish(String(data=json.dumps(msg)))


def main():
    rclpy.init()
    node = BagRecorder()
    try:
        rclpy.spin(node)
    finally:
        if node._proc and node._proc.poll() is None:
            node.get_logger().warn('Shutting down: stopping active recording...')
            os.killpg(os.getpgid(node._proc.pid), signal.SIGINT)
            try:
                node._proc.wait(timeout=10)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
