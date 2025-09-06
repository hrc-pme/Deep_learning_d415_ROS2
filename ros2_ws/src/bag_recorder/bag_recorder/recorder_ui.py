#!/usr/bin/env python3
import json
import threading
import tkinter as tk
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


def _parse_iso8601(ts: str):
    """Parse ISO8601 with timezone; return aware datetime or None."""
    if not ts:
        return None
    # allow 'Z'
    if ts.endswith('Z'):
        ts = ts[:-1] + '+00:00'
    try:
        return datetime.fromisoformat(ts)
    except Exception:
        return None


class BagRecorderUI(Node):
    def __init__(self):
        super().__init__('bag_recorder_ui')
        self.status = {
            'is_recording': False,
            'bag_path': '',
            'start_time_iso': '',
            'elapsed_seconds': 0
        }
        self.sub = self.create_subscription(String, 'bag_recorder/status', self._on_status, 10)
        self.cli_start = self.create_client(Trigger, 'start_recording')
        self.cli_stop  = self.create_client(Trigger, 'stop_recording')

        # Tkinter UI
        self.root = tk.Tk()
        self.root.title('ROS2 Bag Recorder')
        self.root.protocol("WM_DELETE_WINDOW", self._on_close_clicked)

        self.label_state = tk.Label(self.root, text='State: idle', font=('Arial', 16))
        self.label_state.pack(padx=10, pady=5)

        self.label_path = tk.Label(self.root, text='Path: -', font=('Arial', 12))
        self.label_path.pack(padx=10, pady=5)

        # 顯示毫秒：HH:MM:SS.mmm
        self.label_elapsed = tk.Label(self.root, text='Elapsed: 00:00:00.000', font=('Arial', 22))
        self.label_elapsed.pack(padx=10, pady=12)

        frame_btn = tk.Frame(self.root)
        frame_btn.pack(pady=10)
        tk.Button(frame_btn, text='Start', width=12, command=self._call_start).pack(side=tk.LEFT, padx=6)
        tk.Button(frame_btn, text='Stop / Close', width=12, command=self._call_stop_or_close).pack(side=tk.LEFT, padx=6)

        # 每 100ms 更新一次，顯示毫秒更順
        self._tick()

    # -------- ROS callbacks --------
    def _on_status(self, msg: String):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    # -------- UI helpers --------
    @staticmethod
    def _fmt_mmss_mmm(total_ms: int) -> str:
        if total_ms < 0:
            total_ms = 0
        ms = total_ms % 1000
        total_sec = total_ms // 1000
        m = total_sec // 60
        s = total_sec % 60
        return f'{m:02d}:{s:02d}.{ms:03d}'

    def _current_elapsed_ms(self) -> int:
        """優先用 start_time_iso 計算，避免只得整秒；若缺就退回 elapsed_seconds。"""
        is_rec = bool(self.status.get('is_recording', False))
        start_iso = self.status.get('start_time_iso', '')
        dt_start = _parse_iso8601(start_iso)
        if is_rec and dt_start is not None:
            now = datetime.now(timezone.utc)
            # 兩者都應是 aware datetime
            delta = now - dt_start
            return int(delta.total_seconds() * 1000.0)
        # 退回（可能只有整秒）
        sec = int(self.status.get('elapsed_seconds', 0) or 0)
        return sec * 1000

    def _tick(self):
        is_rec = bool(self.status.get('is_recording', False))
        bag_path = self.status.get('bag_path', '') or '-'
        elapsed_ms = self._current_elapsed_ms()

        self.label_state.config(text=f'State: {"RECORDING" if is_rec else "STOP"}',
                                fg='green' if is_rec else 'gray')
        self.label_path.config(text=f'Path: {bag_path}')
        self.label_elapsed.config(text=f'Record: {self._fmt_mmss_mmm(elapsed_ms)}')

        # 100ms 更新一次，顯示毫秒
        self.root.after(100, self._tick)

    # -------- Buttons & window close behavior --------
    def _call_start(self):
        if not self.cli_start.service_is_ready():
            self.get_logger().info('Waiting for start_recording service...')
            self.cli_start.wait_for_service(2.0)
        req = Trigger.Request()
        fut = self.cli_start.call_async(req)
        fut.add_done_callback(
            lambda f: self.get_logger().info(f'Start: {getattr(f.result(), "message", "failed")}'))

    def _call_stop_or_close(self):
        """第一次 Stop：停止；若已是 IDLE，再按一次→直接關視窗。"""
        is_rec = bool(self.status.get('is_recording', False))
        if is_rec:
            self._send_stop()
        else:
            # 已經是 IDLE，就關閉 UI
            self._destroy_ui()

    def _send_stop(self, then_close=False):
        if not self.cli_stop.service_is_ready():
            self.get_logger().info('Waiting for stop_recording service...')
            self.cli_stop.wait_for_service(2.0)
        req = Trigger.Request()
        fut = self.cli_stop.call_async(req)

        def _after(f):
            self.get_logger().info(f'Stop: {getattr(f.result(), "message", "failed")}')
            if then_close:
                # 稍等 200ms 讓狀態回拋，再關
                self.root.after(200, self._destroy_ui)

        fut.add_done_callback(_after)

    def _on_close_clicked(self):
        """點 ✕：若在錄製→先 stop 再關；否則直接關。"""
        is_rec = bool(self.status.get('is_recording', False))
        if is_rec:
            self._send_stop(then_close=True)
        else:
            self._destroy_ui()

    def _destroy_ui(self):
        try:
            self.root.destroy()
        except Exception:
            pass

def _spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = BagRecorderUI()
    t = threading.Thread(target=_spin, args=(node,), daemon=True)
    t.start()
    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
