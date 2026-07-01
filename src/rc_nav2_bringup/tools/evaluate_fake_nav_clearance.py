#!/usr/bin/env python3
"""Send a loop of Nav2 goals and measure fake robot wall clearance.

This is intended for the fake Nav2 bringup where map -> odom is static identity.
It records /odom, projects the rectangular robot footprint onto the occupancy map,
and reports the minimum distance from the footprint to occupied map cells.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path
from typing import Iterable

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


DEFAULT_GOALS = [
    (-8.70, 0.35, math.pi),
    (-8.70, -4.45, -math.pi / 2.0),
    (-0.80, -4.45, 0.0),
    (-0.80, 0.35, math.pi / 2.0),
    (-0.60, 0.40, math.pi),
]


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def parse_scalar(value: str):
    value = value.strip()
    if not value:
        return ""
    if value.startswith("[") and value.endswith("]"):
        items = [part.strip() for part in value[1:-1].split(",")]
        return [parse_scalar(item) for item in items if item]
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        if any(ch in value for ch in ".eE"):
            return float(value)
        return int(value)
    except ValueError:
        return value.strip("\"'")


def load_map_yaml(path: Path) -> dict:
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = parse_scalar(value)
    if "image" not in values or "resolution" not in values or "origin" not in values:
        raise ValueError(f"map yaml missing required keys: {path}")
    image = Path(str(values["image"]))
    if not image.is_absolute():
        image = path.parent / image
    values["image_path"] = image
    values["yaml_path"] = path
    values.setdefault("negate", 0)
    values.setdefault("occupied_thresh", 0.65)
    values.setdefault("free_thresh", 0.25)
    return values


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    data = path.read_bytes()
    index = 0

    def token() -> str:
        nonlocal index
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            while index < len(data) and data[index] not in b"\r\n":
                index += 1
            return token()
        start = index
        while index < len(data) and data[index] not in b" \t\r\n":
            index += 1
        return data[start:index].decode("ascii")

    magic = token()
    if magic not in {"P5", "P2"}:
        raise ValueError(f"unsupported PGM format {magic}: {path}")
    width = int(token())
    height = int(token())
    max_value = int(token())
    if max_value <= 0 or max_value > 255:
        raise ValueError(f"unsupported PGM max value {max_value}: {path}")
    while index < len(data) and data[index] in b" \t\r\n":
        index += 1
    if magic == "P5":
        pixels = list(data[index : index + width * height])
    else:
        text = data[index:].decode("ascii")
        pixels = [int(item) for item in text.split()]
    if len(pixels) != width * height:
        raise ValueError(f"PGM pixel count mismatch: {path}")
    return width, height, pixels


class OccupancyMap:
    def __init__(self, yaml_path: Path):
        meta = load_map_yaml(yaml_path)
        width, height, pixels = read_pgm(meta["image_path"])
        self.yaml_path = yaml_path
        self.image_path = meta["image_path"]
        self.width = width
        self.height = height
        self.pixels = pixels
        self.resolution = float(meta["resolution"])
        origin = meta["origin"]
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.negate = int(meta["negate"])
        self.occupied_thresh = float(meta["occupied_thresh"])
        self.occupied = self.build_occupied_mask()
        self.distance = self.build_distance_field()
        self.cell_margin = self.resolution * math.sqrt(2.0) * 0.5

    def build_occupied_mask(self) -> list[bool]:
        occupied = []
        for pixel in self.pixels:
            if self.negate:
                occ = pixel / 255.0
            else:
                occ = (255.0 - pixel) / 255.0
            occupied.append(occ >= self.occupied_thresh)
        return occupied

    def build_distance_field(self) -> list[float]:
        total = self.width * self.height
        dist = [math.inf] * total
        heap: list[tuple[float, int, int]] = []
        for row in range(self.height):
            for col in range(self.width):
                idx = row * self.width + col
                if self.occupied[idx]:
                    dist[idx] = 0.0
                    heapq.heappush(heap, (0.0, row, col))

        neighbors = [
            (-1, -1, math.sqrt(2.0)),
            (-1, 0, 1.0),
            (-1, 1, math.sqrt(2.0)),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (1, -1, math.sqrt(2.0)),
            (1, 0, 1.0),
            (1, 1, math.sqrt(2.0)),
        ]
        while heap:
            current, row, col = heapq.heappop(heap)
            idx = row * self.width + col
            if current > dist[idx]:
                continue
            for dr, dc, step in neighbors:
                nr = row + dr
                nc = col + dc
                if nr < 0 or nc < 0 or nr >= self.height or nc >= self.width:
                    continue
                nidx = nr * self.width + nc
                candidate = current + step * self.resolution
                if candidate < dist[nidx]:
                    dist[nidx] = candidate
                    heapq.heappush(heap, (candidate, nr, nc))
        return dist

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        col = math.floor((x - self.origin_x) / self.resolution)
        map_y = math.floor((y - self.origin_y) / self.resolution)
        row = self.height - 1 - map_y
        if row < 0 or col < 0 or row >= self.height or col >= self.width:
            return None
        return int(row), int(col)

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (self.height - row - 0.5) * self.resolution
        return x, y

    def clearance_at(self, x: float, y: float) -> float:
        grid = self.world_to_grid(x, y)
        if grid is None:
            return -1.0
        row, col = grid
        return self.distance[row * self.width + col] - self.cell_margin

    def is_occupied_world(self, x: float, y: float) -> bool:
        grid = self.world_to_grid(x, y)
        if grid is None:
            return True
        row, col = grid
        return self.occupied[row * self.width + col]


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    class Quaternion:
        x = 0.0
        y = 0.0
        z = math.sin(yaw * 0.5)
        w = math.cos(yaw * 0.5)

    return Quaternion()


def footprint_polygon(
    x: float,
    y: float,
    yaw: float,
    length: float,
    width: float,
) -> list[tuple[float, float]]:
    half_l = length * 0.5
    half_w = width * 0.5
    local = [
        (-half_l, -half_w),
        (-half_l, half_w),
        (half_l, half_w),
        (half_l, -half_w),
    ]
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [(x + c * lx - s * ly, y + s * lx + c * ly) for lx, ly in local]


def point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi) + xi
        ):
            inside = not inside
        j = i
    return inside


def perimeter_samples(
    polygon: list[tuple[float, float]],
    spacing: float,
) -> Iterable[tuple[float, float]]:
    for idx, start in enumerate(polygon):
        end = polygon[(idx + 1) % len(polygon)]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(length / spacing)))
        for step in range(steps + 1):
            ratio = step / steps
            yield start[0] + ratio * dx, start[1] + ratio * dy


def footprint_metrics(
    occ_map: OccupancyMap,
    x: float,
    y: float,
    yaw: float,
    length: float,
    width: float,
) -> tuple[float, bool]:
    polygon = footprint_polygon(x, y, yaw, length, width)
    spacing = occ_map.resolution * 0.5
    clearances = [occ_map.clearance_at(px, py) for px, py in perimeter_samples(polygon, spacing)]
    min_clearance = min(clearances) if clearances else -1.0

    min_x = min(px for px, _ in polygon)
    max_x = max(px for px, _ in polygon)
    min_y = min(py for _, py in polygon)
    max_y = max(py for _, py in polygon)
    min_grid = occ_map.world_to_grid(min_x, min_y)
    max_grid = occ_map.world_to_grid(max_x, max_y)
    collision = False
    if min_grid is None or max_grid is None:
        collision = True
    else:
        row_a, col_a = min_grid
        row_b, col_b = max_grid
        for row in range(min(row_a, row_b), max(row_a, row_b) + 1):
            for col in range(min(col_a, col_b), max(col_a, col_b) + 1):
                if row < 0 or col < 0 or row >= occ_map.height or col >= occ_map.width:
                    collision = True
                    continue
                if not occ_map.occupied[row * occ_map.width + col]:
                    continue
                cx, cy = occ_map.grid_to_world(row, col)
                if point_in_polygon(cx, cy, polygon):
                    collision = True
                    break
            if collision:
                break
    return min_clearance, collision


class ClearanceEvaluator(Node):
    def __init__(self, args):
        super().__init__("fake_nav_clearance_evaluator")
        self.args = args
        self.map = OccupancyMap(Path(args.map))
        self.samples: list[dict] = []
        self.goal_results: list[dict] = []
        self.action_client = ActionClient(self, NavigateToPose, args.action_name)
        self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 50)

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)
        clearance, collision = footprint_metrics(
            self.map, x, y, yaw, self.args.body_length, self.args.body_width
        )
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        self.samples.append(
            {
                "stamp": stamp,
                "x": x,
                "y": y,
                "yaw": yaw,
                "clearance_m": clearance,
                "collision": collision,
            }
        )

    def wait_for_initial_odom(self) -> bool:
        deadline = self.get_clock().now().nanoseconds * 1.0e-9 + self.args.start_timeout
        while rclpy.ok() and not self.samples:
            if self.get_clock().now().nanoseconds * 1.0e-9 > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def send_goal(self, x: float, y: float, yaw: float, index: int) -> dict:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        quat = quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.x = quat.x
        goal.pose.pose.orientation.y = quat.y
        goal.pose.pose.orientation.z = quat.z
        goal.pose.pose.orientation.w = quat.w

        self.get_logger().info(
            f"sending goal {index}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}"
        )
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.goal_send_timeout)
        if not future.done():
            return {"index": index, "x": x, "y": y, "yaw": yaw, "status": "SEND_TIMEOUT"}
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"index": index, "x": x, "y": y, "yaw": yaw, "status": "REJECTED"}

        result_future = goal_handle.get_result_async()
        start = self.get_clock().now().nanoseconds * 1.0e-9
        while rclpy.ok() and not result_future.done():
            elapsed = self.get_clock().now().nanoseconds * 1.0e-9 - start
            if elapsed > self.args.goal_timeout:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                return {"index": index, "x": x, "y": y, "yaw": yaw, "status": "TIMEOUT"}
            rclpy.spin_once(self, timeout_sec=0.1)

        result = result_future.result()
        status = STATUS_NAMES.get(result.status, str(result.status))
        return {"index": index, "x": x, "y": y, "yaw": yaw, "status": status}

    def run(self, goals: list[tuple[float, float, float]]) -> dict:
        self.get_logger().info(f"loading map {self.args.map}")
        if not self.action_client.wait_for_server(timeout_sec=self.args.start_timeout):
            raise RuntimeError(f"action server not available: {self.args.action_name}")
        if not self.wait_for_initial_odom():
            raise RuntimeError(f"no odom samples on {self.args.odom_topic}")

        for index, (x, y, yaw) in enumerate(goals, start=1):
            result = self.send_goal(x, y, yaw, index)
            self.goal_results.append(result)
            self.get_logger().info(f"goal {index} result: {result['status']}")
            if result["status"] != "SUCCEEDED" and self.args.stop_on_failure:
                break
            for _ in range(int(max(0.0, self.args.pause_sec) / 0.1)):
                rclpy.spin_once(self, timeout_sec=0.1)
        return self.summary()

    def summary(self) -> dict:
        if self.samples:
            first_stamp = self.samples[0]["stamp"]
            evaluated_samples = [
                sample
                for sample in self.samples
                if sample["stamp"] - first_stamp >= self.args.ignore_first_sec
            ]
        else:
            evaluated_samples = []
        clearances = [sample["clearance_m"] for sample in evaluated_samples]
        collisions = [sample for sample in evaluated_samples if sample["collision"]]
        raw_clearances = [sample["clearance_m"] for sample in self.samples]
        raw_collisions = [sample for sample in self.samples if sample["collision"]]
        min_sample = (
            min(evaluated_samples, key=lambda item: item["clearance_m"])
            if evaluated_samples
            else None
        )
        sorted_clearances = sorted(clearances)

        def percentile(q: float) -> float | None:
            if not sorted_clearances:
                return None
            index = q * (len(sorted_clearances) - 1)
            lo = math.floor(index)
            hi = math.ceil(index)
            if lo == hi:
                return sorted_clearances[lo]
            ratio = index - lo
            return sorted_clearances[lo] * (1.0 - ratio) + sorted_clearances[hi] * ratio

        return {
            "map": str(self.map.yaml_path),
            "odom_topic": self.args.odom_topic,
            "action_name": self.args.action_name,
            "body_length": self.args.body_length,
            "body_width": self.args.body_width,
            "ignored_initial_sec": self.args.ignore_first_sec,
            "sample_count": len(self.samples),
            "evaluated_sample_count": len(evaluated_samples),
            "collision_sample_count": len(collisions),
            "min_clearance_m": min(clearances) if clearances else None,
            "p05_clearance_m": percentile(0.05),
            "p50_clearance_m": percentile(0.50),
            "raw_collision_sample_count": len(raw_collisions),
            "raw_min_clearance_m": min(raw_clearances) if raw_clearances else None,
            "min_clearance_sample": min_sample,
            "goal_results": self.goal_results,
        }


def parse_goals(raw: str | None) -> list[tuple[float, float, float]]:
    if not raw:
        return DEFAULT_GOALS
    goals = []
    for chunk in raw.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [float(part.strip()) for part in chunk.split(",")]
        if len(parts) != 3:
            raise ValueError(f"goal must be x,y,yaw: {chunk}")
        goals.append((parts[0], parts[1], parts[2]))
    if not goals:
        raise ValueError("no goals provided")
    return goals


def write_reports(summary: dict, report_path: Path | None, json_path: Path | None) -> None:
    if json_path:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    if report_path:
        report_path.parent.mkdir(parents=True, exist_ok=True)
        lines = [
            "# Fake Nav Clearance Evaluation",
            "",
            f"- Map: `{summary['map']}`",
            f"- Odom topic: `{summary['odom_topic']}`",
            f"- Action: `{summary['action_name']}`",
            f"- Samples: {summary['sample_count']}",
            f"- Ignored initial seconds: {summary['ignored_initial_sec']:.2f}",
            f"- Evaluated samples: {summary['evaluated_sample_count']}",
            f"- Raw collision samples: {summary['raw_collision_sample_count']}",
            f"- Raw min clearance: {format_metric(summary['raw_min_clearance_m'])}",
            f"- Collision samples: {summary['collision_sample_count']}",
            f"- Min clearance: {format_metric(summary['min_clearance_m'])}",
            f"- P05 clearance: {format_metric(summary['p05_clearance_m'])}",
            f"- P50 clearance: {format_metric(summary['p50_clearance_m'])}",
            "",
            "## Goals",
            "",
        ]
        for result in summary["goal_results"]:
            lines.append(
                f"- {result['index']}: ({result['x']:.2f}, {result['y']:.2f}, "
                f"{result['yaw']:.2f}) -> {result['status']}"
            )
        sample = summary["min_clearance_sample"]
        if sample:
            lines.extend(
                [
                    "",
                    "## Worst Sample",
                    "",
                    f"- x: {sample['x']:.3f}",
                    f"- y: {sample['y']:.3f}",
                    f"- yaw: {sample['yaw']:.3f}",
                    f"- clearance: {sample['clearance_m']:.3f} m",
                    f"- collision: {sample['collision']}",
                ]
            )
        report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def format_metric(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.3f} m"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map",
        default="src/rc_nav2_bringup/maps/red_manual_simple.yaml",
        help="Nav2 occupancy map YAML.",
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--body-length", type=float, default=0.70)
    parser.add_argument("--body-width", type=float, default=0.80)
    parser.add_argument(
        "--goals",
        default=None,
        help="Semicolon-separated x,y,yaw goals. Defaults to a loop around red_manual_simple.",
    )
    parser.add_argument("--start-timeout", type=float, default=20.0)
    parser.add_argument("--goal-send-timeout", type=float, default=5.0)
    parser.add_argument("--goal-timeout", type=float, default=45.0)
    parser.add_argument("--pause-sec", type=float, default=0.5)
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument(
        "--ignore-first-sec",
        type=float,
        default=0.0,
        help="Ignore initial odom samples when computing clearance metrics.",
    )
    parser.add_argument(
        "--min-clearance",
        type=float,
        default=0.0,
        help="Minimum accepted clearance in meters for evaluated samples.",
    )
    parser.add_argument(
        "--min-p05-clearance",
        type=float,
        default=0.0,
        help="Minimum accepted 5th percentile clearance in meters for evaluated samples.",
    )
    parser.add_argument(
        "--require-all-goals",
        action="store_true",
        help="Return failure if any navigation goal does not succeed.",
    )
    parser.add_argument("--report", default="")
    parser.add_argument("--json", default="")
    args = parser.parse_args()

    goals = parse_goals(args.goals)
    rclpy.init()
    node = ClearanceEvaluator(args)
    try:
        summary = node.run(goals)
        print(json.dumps(summary, indent=2, sort_keys=True))
        write_reports(
            summary,
            Path(args.report) if args.report else None,
            Path(args.json) if args.json else None,
        )
        if summary["collision_sample_count"] > 0:
            return 2
        if args.require_all_goals and any(
            result["status"] != "SUCCEEDED" for result in summary["goal_results"]
        ):
            return 3
        min_clearance = summary["min_clearance_m"]
        if min_clearance is not None and min_clearance < args.min_clearance:
            return 1
        p05_clearance = summary["p05_clearance_m"]
        if p05_clearance is not None and p05_clearance < args.min_p05_clearance:
            return 4
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
