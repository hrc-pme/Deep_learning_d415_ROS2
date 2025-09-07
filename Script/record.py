#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
from pathlib import Path

import yaml


def expand_path(p: str) -> str:
    return os.path.abspath(os.path.expanduser(p))


def next_index_folder(root_dir: str) -> str:
    """
    Find the next incremental folder name under root_dir as 4-digit index (0001, 0002, ...).
    Returns the full absolute path to the new folder.
    """
    root = Path(root_dir)
    root.mkdir(parents=True, exist_ok=True)

    existing = []
    for p in root.iterdir():
        if p.is_dir() and p.name.isdigit():
            try:
                existing.append(int(p.name))
            except ValueError:
                pass
    next_idx = max(existing) + 1 if existing else 1
    return str(root / f"{next_idx:04d}")


def build_command(cfg: dict, out_dir_root: str) -> tuple[list, str]:
    cmd = ["ros2", "bag", "record"]

    # Output folder uses sequential numbering: 0001, 0002, ...
    out_full = next_index_folder(out_dir_root)
    cmd += ["-o", out_full]

    # Storage
    storage = cfg.get("storage", "sqlite3")
    if storage:
        cmd += ["--storage", storage]

    # Compression
    compression = cfg.get("compression", "none")
    if compression and compression.lower() != "none":
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

    # Topics selection
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
            raise ValueError("allow_all=false but 'topics' is empty! Please specify at least one topic in YAML.")
        cmd += topics

    return cmd, out_full


def main():
    parser = argparse.ArgumentParser(description="Record ROS 2 bag with YAML config.")
    parser.add_argument("-c", "--config", required=True, help="Path to config.yaml")
    args = parser.parse_args()

    cfg_path = expand_path(args.config)
    if not os.path.isfile(cfg_path):
        print(f"[ERROR] Config file not found: {cfg_path}", file=sys.stderr)
        sys.exit(1)

    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    out_dir_root = expand_path(cfg.get("output_dir", "./rosbags"))
    Path(out_dir_root).mkdir(parents=True, exist_ok=True)

    cmd, out_full = build_command(cfg, out_dir_root)

    print("[INFO] Executing:", " ".join(cmd))
    print("[INFO] Output folder:", out_full)
    print("[INFO] Press Ctrl+C to stop recording.")

    # Start subprocess
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

    def handle_sigint(signum, frame):
        print("\n[INFO] Interrupt signal received, stopping rosbag recording…")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception as e:
            print(f"[WARN] Failed to send SIGINT: {e}")
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            print("[WARN] Did not exit in time, forcing termination…")
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    rc = proc.wait()
    if rc == 0:
        print("[INFO] Recording finished.")
    else:
        print(f"[ERROR] ros2 bag record exited with code: {rc}")
    sys.exit(rc)


if __name__ == "__main__":
    main()
