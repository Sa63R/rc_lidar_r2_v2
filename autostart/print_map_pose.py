#!/usr/bin/env python3
import argparse
import math
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from rclpy.node import Node
from rclpy.time import Time


def quat_to_rpy(q: Quaternion) -> tuple[float, float, float]:
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def stamp_text(stamp) -> str:
    return f"{stamp.sec}.{stamp.nanosec:09d}"


class MapPosePrinter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("print_map_pose")
        self.args = args
        self.min_interval = 1.0 / args.hz if args.hz > 0.0 else 0.0
        self.last_print = 0.0
        self.done = False
        self.tf_buffer = None
        self.tf_listener = None

        if args.tf:
            from tf2_ros import Buffer, TransformException, TransformListener

            self.TransformException = TransformException
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(0.02, self._tick_tf)
            self.get_logger().info(
                f"printing TF {args.fixed_frame} -> {args.target_frame} at <= {args.hz} Hz"
            )
        else:
            self.create_subscription(PoseStamped, args.topic, self._on_pose, 20)
            self.get_logger().info(
                f"printing PoseStamped topic {args.topic} at <= {args.hz} Hz"
            )

    def _should_print(self) -> bool:
        now = time.monotonic()
        if self.min_interval > 0.0 and (now - self.last_print) < self.min_interval:
            return False
        self.last_print = now
        return True

    def _age_sec(self, stamp) -> Optional[float]:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        msg_time = Time.from_msg(stamp)
        return (self.get_clock().now() - msg_time).nanoseconds / 1e9

    def _print_pose(self, frame: str, child: str, stamp, x: float, y: float, z: float, q: Quaternion) -> None:
        roll, pitch, yaw = quat_to_rpy(q)
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        age = self._age_sec(stamp)
        age_str = "nan" if age is None else f"{age:.3f}"
        digits = self.args.digits

        sys.stdout.write(
            f"[map_pose] frame={frame} child={child} stamp={stamp_text(stamp)} age={age_str}s\n"
        )
        sys.stdout.write(
            f"  xyz: x={x:.{digits}f} y={y:.{digits}f} z={z:.{digits}f}\n"
        )
        sys.stdout.write(
            f"  rpy_deg: roll={roll_deg:.3f} pitch={pitch_deg:.3f} yaw={yaw_deg:.3f}\n"
        )
        sys.stdout.write(
            f"  rpy_rad: roll={roll:.{digits}f} pitch={pitch:.{digits}f} yaw={yaw:.{digits}f}\n"
        )
        if not self.args.no_env:
            sys.stdout.write(
                "  init_env: "
                "RC_RELOCALIZATION_INITIAL_GUESS=1 "
                f"RC_RELOCALIZATION_INITIAL_X={x:.{digits}f} "
                f"RC_RELOCALIZATION_INITIAL_Y={y:.{digits}f} "
                f"RC_RELOCALIZATION_INITIAL_Z={z:.{digits}f} "
                f"RC_RELOCALIZATION_INITIAL_ROLL={roll:.{digits}f} "
                f"RC_RELOCALIZATION_INITIAL_PITCH={pitch:.{digits}f} "
                f"RC_RELOCALIZATION_INITIAL_YAW={yaw:.{digits}f}\n"
            )
        sys.stdout.write("\n")
        sys.stdout.flush()

        if self.args.once:
            self.done = True

    def _on_pose(self, msg: PoseStamped) -> None:
        if not self._should_print():
            return
        p = msg.pose.position
        self._print_pose(
            msg.header.frame_id or "(empty)",
            self.args.topic,
            msg.header.stamp,
            p.x,
            p.y,
            p.z,
            msg.pose.orientation,
        )

    def _tick_tf(self) -> None:
        if not self._should_print() or self.tf_buffer is None:
            return
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.args.fixed_frame,
                self.args.target_frame,
                Time(),
            )
        except self.TransformException as exc:
            self.get_logger().warn(
                f"waiting for TF {self.args.fixed_frame} -> {self.args.target_frame}: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        self._print_pose(
            transform.header.frame_id or self.args.fixed_frame,
            transform.child_frame_id or self.args.target_frame,
            transform.header.stamp,
            t.x,
            t.y,
            t.z,
            q,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Print the current map-frame pose for recording relocalization initial values. "
            "Default source is /map_lidar/pose, which is map -> body."
        )
    )
    parser.add_argument("--topic", default="/map_lidar/pose", help="PoseStamped topic in map frame")
    parser.add_argument("--hz", type=float, default=5.0, help="maximum print rate; <=0 prints every callback/timer tick")
    parser.add_argument("--once", action="store_true", help="print one pose and exit")
    parser.add_argument("--digits", type=int, default=6, help="decimal digits for xyz and radians")
    parser.add_argument("--no-env", action="store_true", help="do not print RC_RELOCALIZATION_INITIAL_* values")
    parser.add_argument("--tf", action="store_true", help="read pose from TF instead of a PoseStamped topic")
    parser.add_argument("--fixed-frame", default="map", help="TF fixed frame when --tf is used")
    parser.add_argument("--target-frame", default="camera_init", help="TF target frame when --tf is used")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = MapPosePrinter(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
