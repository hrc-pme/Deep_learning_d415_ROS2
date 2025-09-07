#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
import threading
import tkinter as tk
from tkinter import messagebox
import yaml


def expand_path(p: str) -> str:
    return os.path.abspath(os.path.expanduser(p))


def build_command_and_outdir(cfg: dict) -> tuple[list, str]:
    """Build ros2 bag record command and return output directory"""
    cmd = ["ros2", "bag", "record"]

    # Root output dir
    out_dir_root = expand_path(cfg.get("output_dir", "./rosbags"))
    Path(out_dir_root).mkdir(parents=True, exist_ok=True)

    # Find next index folder (0001, 0002, …)
    existing = []
    for p in Path(out_dir_root).iterdir():
        if p.is_dir() and p.name.isdigit():
            existing.append(int(p.name))
    next_idx = max(existing) + 1 if existing else 1
    folder_name = f"{next_idx:04d}"   # 0001 格式
    out_full = os.path.join(out_dir_root, folder_name)

    # ros2 bag record -o <out_full>
    cmd += ["-o", out_full]

    # Storage
    storage = cfg.get("storage", "sqlite3")
    if storage:
        cmd += ["--storage", storage]

    # Compression
    compression = cfg.get("compression", "none")
    if compression and str(compression).lower() != "none":
        cmd += ["--compression-format", compression]
        mode = cfg.get("compression_mode", "file")
        cmd += ["--compression-mode", mode]

    # Limits
    max_bag_size = int(cfg.get("max_bag_size", 0) or 0)
    if max_bag_size > 0:
        cmd += ["--max-bag-size", str(max_bag_size)]
    max_bag_duration = int(cfg.get("max_bag_duration", 0) or 0)
    if max_bag_duration > 0:
        cmd += ["--max-bag-duration", str(max_bag_duration)]

    # QoS overrides
    qos_path = cfg.get("qos_profile_overrides_path", "")
    if isinstance(qos_path, str) and qos_path.strip():
        cmd += ["--qos-profile-overrides-path", expand_path(qos_path)]

    # Topic selection
    allow_all = bool(cfg.get("allow_all", False))
    if allow_all:
        cmd.append("-a")
        exclude = cfg.get("exclude_topics", []) or []
        exclude = [e for e in exclude if isinstance(e, str) and e.strip()]
        if exclude:
            exclude_regex = "|".join(f"(?:{e})" for e in exclude)
            cmd += ["--exclude", exclude_regex]
    else:
        topics = cfg.get("topics", []) or []
        if not topics:
            raise ValueError("allow_all=false but no topics specified in YAML config.")
        cmd += topics

    return cmd, out_full


class RecorderUI:
    def __init__(self, root: tk.Tk, cfg_path: str):
        self.root = root
        self.cfg_path = cfg_path
        self.proc: subprocess.Popen | None = None
        self.start_time: float | None = None
        self.out_dir: str = ""
        self.running = False

        self.root.title("bag_recorder")
        self.root.geometry("720x200")

        # Widgets
        row = 0
        tk.Label(root, text="Config file:", anchor="w").grid(row=row, column=0, sticky="w", padx=8, pady=6)
        self.lbl_cfg = tk.Label(root, text=self.cfg_path, anchor="w", fg="blue")
        self.lbl_cfg.grid(row=row, column=1, columnspan=3, sticky="w", padx=8, pady=6)

        row += 1
        tk.Label(root, text="Output folder:", anchor="w").grid(row=row, column=0, sticky="w", padx=8, pady=6)
        self.lbl_out = tk.Label(root, text="(not started)", anchor="w")
        self.lbl_out.grid(row=row, column=1, columnspan=3, sticky="w", padx=8, pady=6)

        row += 1
        tk.Label(root, text="Status:", anchor="w").grid(row=row, column=0, sticky="w", padx=8, pady=6)
        self.lbl_status = tk.Label(root, text="Idle", anchor="w", fg="darkorange")
        self.lbl_status.grid(row=row, column=1, sticky="w", padx=8, pady=6)

        tk.Label(root, text="Record time:", anchor="w").grid(row=row, column=2, sticky="e", padx=8, pady=6)
        self.lbl_elapsed = tk.Label(root, text="00:00.000", anchor="w")
        self.lbl_elapsed.grid(row=row, column=3, sticky="w", padx=8, pady=6)

        row += 1
        self.btn_start = tk.Button(root, text="Start Recording", command=self.start_recording, width=18)
        self.btn_start.grid(row=row, column=1, padx=8, pady=12, sticky="e")

        self.btn_stop = tk.Button(root, text="Stop", command=self.stop_recording, state="disabled", width=12)
        self.btn_stop.grid(row=row, column=2, padx=8, pady=12, sticky="w")

        # Timer
        self.update_timer()

        # Close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def start_recording(self):
        if self.running:
            return
        # Load YAML
        try:
            with open(self.cfg_path, "r") as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load config file:\n{e}")
            return
        try:
            cmd, out_full = build_command_and_outdir(cfg)
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return

        self.out_dir = out_full
        self.lbl_out.config(text=out_full)
        self.lbl_status.config(text="Starting...", fg="darkorange")
        self.root.update_idletasks()

        # Start subprocess
        try:
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to launch ros2 bag record:\n{e}")
            self.proc = None
            return

        self.start_time = time.time()
        self.running = True
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.lbl_status.config(text="Recording", fg="green")

        # Watch process in background
        threading.Thread(target=self._watch_proc, daemon=True).start()

    def _watch_proc(self):
        if not self.proc:
            return
        rc = self.proc.wait()
        # Back to UI thread
        def done():
            self.running = False
            self.btn_start.config(state="normal")
            self.btn_stop.config(state="disabled")
            if rc == 0:
                self.lbl_status.config(text="Stopped (normal)", fg="black")
            else:
                self.lbl_status.config(text=f"Exited with error (rc={rc})", fg="red")
        self.root.after(0, done)

    def stop_recording(self):
        if not self.running or not self.proc:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except Exception as e:
            messagebox.showwarning("Warning", f"Failed to send stop signal:\n{e}")

    def update_timer(self):
        if self.running and self.start_time:
            elapsed = time.time() - self.start_time
            minutes = int(elapsed // 60)
            seconds = int(elapsed % 60)
            millis = int((elapsed * 1000) % 1000)
            self.lbl_elapsed.config(text=f"{minutes:02d}:{seconds:02d}.{millis:03d}")
        self.root.after(100, self.update_timer)  # update every 100 ms

    def on_close(self):
        if self.running and self.proc:
            if not messagebox.askyesno("Confirm", "Recording is still running. Do you want to stop and exit?"):
                return
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                time.sleep(1.0)
            except Exception:
                pass
        self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Simple UI to record ROS 2 bag with a YAML config.")
    parser.add_argument("-c", "--config", required=True, help="Path to config.yaml")
    args = parser.parse_args(argv)

    cfg_path = expand_path(args.config)
    if not os.path.isfile(cfg_path):
        print(f"[ERROR] Config file not found: {cfg_path}", file=sys.stderr)
        return 1

    root = tk.Tk()
    app = RecorderUI(root, cfg_path)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
