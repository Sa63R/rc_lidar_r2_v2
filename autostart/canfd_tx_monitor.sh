#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

exec /usr/bin/env python3 -u - <<'PY'
import csv
import glob
import os
import struct
import time
from pathlib import Path

root = Path(
    os.environ.get(
        "RC_CANFD_TX_MONITOR_LOG_ROOT",
        "/home/rc2/jwt_new_lidar/rc_lidar_r2_v2/logs/autostart",
    )
)
run_log_dir = os.environ.get("LOG_DIR", "")
current_run_csv = Path(run_log_dir) / "canfd_pose_payload.csv" if run_log_dir else None
can_id = int(os.environ.get("RC_CANFD_TX_MONITOR_CAN_ID", "18"), 0)
poll_sec = float(os.environ.get("RC_CANFD_TX_MONITOR_POLL_SEC", "0.05"))


def latest_csv():
    candidates = []
    if current_run_csv is not None:
        candidates.append(str(current_run_csv))
    candidates.extend(glob.glob(str(root / "run_*" / "canfd_pose_payload.csv")))
    files = []
    for path in candidates:
        try:
            if os.path.getsize(path) > 0:
                files.append(path)
        except OSError:
            pass
    if not files:
        return None
    return max(files, key=os.path.getmtime)


path = latest_csv()
while not path:
    print("waiting for canfd_pose_payload.csv ...", flush=True)
    time.sleep(1.0)
    path = latest_csv()

print(f"监听文件: {path}", flush=True)
print(
    f"CAN FD ID: 0x{can_id:03X}, payload: <x,y,z,yaw float32 little-endian>",
    flush=True,
)
print("按 Ctrl+C 退出\n", flush=True)

f = open(path, "r", newline="")
try:
    f.seek(0, os.SEEK_END)
    while True:
        line = f.readline()
        if not line:
            new_path = latest_csv()
            if new_path and new_path != path:
                print(f"\n检测到新运行日志，切换到: {new_path}", flush=True)
                f.close()
                path = new_path
                f = open(path, "r", newline="")
                f.seek(0, os.SEEK_END)
            time.sleep(poll_sec)
            continue

        try:
            row = next(csv.reader([line]))
            ts, mode = row[0], row[1]
            x, y, z, yaw = map(float, row[6:10])
            payload = struct.pack("<ffff", x, y, z, yaw)
            print(
                f"{ts}  id=0x{can_id:03X}  mode={mode}  "
                f"x={x:.4f} y={y:.4f} z={z:.4f} yaw={yaw:.4f}  "
                f"payload={payload.hex().upper()}",
                flush=True,
            )
        except Exception as exc:
            print(f"解析失败: {exc} line={line.strip()}", flush=True)
finally:
    f.close()
PY
