#!/usr/bin/env bash
# open_video_hrc.sh
# Purpose: Make /dev/video* owned by user hrc.
# Usage:   sudo ./open_video_hrc.sh

set -euo pipefail

TARGET_USER="hrc"

for dev in /dev/video*; do
  if [[ -e "$dev" ]]; then
    sudo chown "${TARGET_USER}:${TARGET_USER}" "$dev"
    sudo chmod 660 "$dev"
    echo "[OK] Granted read/write access to $TARGET_USER for $dev (others denied)."
  fi
done

echo "✅ All /dev/video* are now owned by $TARGET_USER (read/write)."
