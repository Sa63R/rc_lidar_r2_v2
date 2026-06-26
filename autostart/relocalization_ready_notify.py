#!/usr/bin/env python3
import argparse
import json
import math
import os
import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def atomic_write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + f".tmp.{os.getpid()}")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    tmp.replace(path)
    try:
        os.sync()
    except OSError:
        pass


def popup_terminal(marker: dict, args: argparse.Namespace) -> bool:
    if not args.popup:
        return False
    if not os.environ.get("DISPLAY"):
        return False

    xfce = shutil.which("xfce4-terminal")
    gnome = shutil.which("gnome-terminal")
    xterm = shutil.which("xterm")
    if not (xfce or gnome or xterm):
        return False

    seconds = max(1, int(args.window_seconds))
    message_script = Path(args.marker_path).with_name("relocalization_ready_message.sh")
    message_script.write_text(
        "#!/usr/bin/env bash\n"
        "clear\n"
        "echo '========================================'\n"
        "echo 'RC LiDAR 重定位完成'\n"
        "echo '现在可以移动小车'\n"
        "echo '========================================'\n"
        f"echo '启动到就绪耗时: {marker.get('elapsed_sec', 0.0):.1f} s'\n"
        f"echo 'fitness: {marker.get('fitness', float('nan')):.6f}'\n"
        f"echo '连续接受次数: {marker.get('consecutive_accepts', 0)}'\n"
        f"echo 'pose: x={marker.get('pose_x', float('nan')):.4f}, "
        f"y={marker.get('pose_y', float('nan')):.4f}, "
        f"yaw={marker.get('pose_yaw_deg', float('nan')):.2f} deg'\n"
        "echo\n"
        f"echo '本窗口 {seconds} 秒后自动关闭。'\n"
        f"sleep {seconds}\n"
    )
    message_script.chmod(0o755)

    title = args.window_title
    try:
        if xfce:
            subprocess.Popen(
                [
                    xfce,
                    "--disable-server",
                    f"--title={title}",
                    "--command",
                    str(message_script),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
            )
        elif gnome:
            subprocess.Popen(
                [gnome, "--title", title, "--", str(message_script)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
            )
        else:
            subprocess.Popen(
                [xterm, "-T", title, "-e", str(message_script)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
            )
        return True
    except OSError:
        return False


class RelocalizationReadyWatcher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("relocalization_ready_notify")
        self.args = args
        self.start_time = time.monotonic()
        self.pose: tuple[float, PoseStamped] | None = None
        self.quality: tuple[float, list[float]] | None = None
        self.good_since: float | None = None
        self.done = False
        self.exit_code = 0
        self.last_reason = "waiting_for_topics"

        self.create_subscription(PoseStamped, args.pose_topic, self.on_pose, 20)
        self.create_subscription(Float32MultiArray, args.quality_topic, self.on_quality, 20)
        self.create_timer(args.poll_period, self.check_ready)

        self.get_logger().info(
            "Waiting for relocalization ready: "
            f"pose={args.pose_topic} quality={args.quality_topic} "
            f"stable_sec={args.stable_sec} timeout={args.timeout_sec}"
        )

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = (time.monotonic(), msg)

    def on_quality(self, msg: Float32MultiArray) -> None:
        self.quality = (time.monotonic(), list(msg.data))

    def quality_good(self, now: float) -> tuple[bool, dict]:
        if self.pose is None:
            return False, {"reason": "waiting_for_pose"}
        pose_stamp, pose = self.pose
        pose_age = now - pose_stamp
        if pose_age > self.args.max_topic_age:
            return False, {"reason": "pose_stale", "pose_age_sec": pose_age}

        if self.quality is None:
            return False, {"reason": "waiting_for_quality"}
        quality_stamp, values = self.quality
        quality_age = now - quality_stamp
        if quality_age > self.args.max_topic_age:
            return False, {"reason": "quality_stale", "quality_age_sec": quality_age}

        accepted = values[0] > 0.5 if len(values) > 0 else False
        converged = values[1] > 0.5 if len(values) > 1 else False
        fitness = values[2] if len(values) > 2 else math.nan
        consecutive_accepts = values[9] if len(values) > 9 else 0.0
        reason_code = values[10] if len(values) > 10 else math.nan
        scan_age = values[13] if len(values) > 13 else math.nan
        odom_age = values[14] if len(values) > 14 else math.nan

        good = (
            accepted
            and converged
            and math.isfinite(fitness)
            and fitness <= self.args.max_fitness
            and consecutive_accepts >= self.args.min_consecutive_accepts
        )
        details = {
            "reason": "quality_good" if good else "quality_not_ready",
            "accepted": accepted,
            "converged": converged,
            "fitness": fitness,
            "consecutive_accepts": consecutive_accepts,
            "reason_code": reason_code,
            "pose_age_sec": pose_age,
            "quality_age_sec": quality_age,
            "scan_age_sec": scan_age,
            "odom_age_sec": odom_age,
            "pose_x": float(pose.pose.position.x),
            "pose_y": float(pose.pose.position.y),
            "pose_z": float(pose.pose.position.z),
            "pose_yaw_deg": math.degrees(yaw_from_pose(pose)),
        }
        return good, details

    def check_ready(self) -> None:
        now = time.monotonic()
        elapsed = now - self.start_time
        good, details = self.quality_good(now)
        self.last_reason = str(details.get("reason", self.last_reason))

        if good:
            if self.good_since is None:
                self.good_since = now
            stable_for = now - self.good_since
            if stable_for >= self.args.stable_sec:
                marker = {
                    "event": "relocalization_ready",
                    "ready_at": datetime.now().isoformat(timespec="seconds"),
                    "elapsed_sec": elapsed,
                    "stable_for_sec": stable_for,
                    "pose_topic": self.args.pose_topic,
                    "quality_topic": self.args.quality_topic,
                    **details,
                }
                atomic_write_json(Path(self.args.marker_path), marker)
                popup_shown = popup_terminal(marker, self.args)
                marker["popup_shown"] = popup_shown
                atomic_write_json(Path(self.args.marker_path), marker)
                self.get_logger().info(
                    "Relocalization ready: "
                    f"elapsed={elapsed:.1f}s fitness={details.get('fitness', math.nan):.6f} "
                    f"popup_shown={popup_shown}"
                )
                self.done = True
                return
        else:
            self.good_since = None

        if elapsed >= self.args.timeout_sec:
            marker = {
                "event": "relocalization_ready_timeout",
                "timeout_at": datetime.now().isoformat(timespec="seconds"),
                "elapsed_sec": elapsed,
                "pose_topic": self.args.pose_topic,
                "quality_topic": self.args.quality_topic,
                "last_reason": self.last_reason,
                **details,
            }
            atomic_write_json(Path(self.args.timeout_marker_path), marker)
            self.get_logger().warning(
                f"Timed out waiting for relocalization ready: reason={self.last_reason}"
            )
            if self.args.popup_on_timeout:
                timeout_args = argparse.Namespace(**vars(self.args))
                timeout_args.marker_path = self.args.timeout_marker_path
                timeout_args.window_title = "RC LiDAR Not Ready"
                timeout_marker = {
                    "elapsed_sec": elapsed,
                    "fitness": details.get("fitness", math.nan),
                    "consecutive_accepts": details.get("consecutive_accepts", 0),
                    "pose_x": details.get("pose_x", math.nan),
                    "pose_y": details.get("pose_y", math.nan),
                    "pose_yaw_deg": details.get("pose_yaw_deg", math.nan),
                }
                popup_terminal(timeout_marker, timeout_args)
            self.done = True
            self.exit_code = 1


def parse_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def main() -> int:
    log_dir = Path(os.environ.get("LOG_DIR", os.getcwd()))
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/robot/pose_continuous")
    parser.add_argument("--quality-topic", default="/map_lidar/quality")
    parser.add_argument("--marker-path", default=str(log_dir / "relocalization_ready.json"))
    parser.add_argument(
        "--timeout-marker-path",
        default=str(log_dir / "relocalization_ready_timeout.json"),
    )
    parser.add_argument("--stable-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--max-topic-age", type=float, default=2.0)
    parser.add_argument("--max-fitness", type=float, default=0.05)
    parser.add_argument("--min-consecutive-accepts", type=float, default=5.0)
    parser.add_argument("--poll-period", type=float, default=0.2)
    parser.add_argument("--window-seconds", type=float, default=20.0)
    parser.add_argument("--window-title", default="RC LiDAR Ready")
    parser.add_argument("--popup", type=parse_bool, default=True)
    parser.add_argument("--popup-on-timeout", type=parse_bool, default=False)
    args = parser.parse_args()

    rclpy.init()
    node = RelocalizationReadyWatcher(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        return node.exit_code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
