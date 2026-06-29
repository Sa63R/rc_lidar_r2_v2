#!/usr/bin/env python3
import csv
import errno
import math
import os
import socket
import struct
import subprocess
import termios
import time
from glob import glob
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .pose_math import rpy_from_quat


CANFD_BRS = 0x01
CAN_RAW_FD_FRAMES = 5
CANFD_FRAME_FORMAT = "=IBBBB64s"
CANFD_FRAME_SIZE = struct.calcsize(CANFD_FRAME_FORMAT)
CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
DEFAULT_SEND_RATE_HZ = 100.0
DEFAULT_AUTO_RETRANSMIT = True
DEFAULT_POSE_TOPIC = "/robot/pose_filtered"
DEFAULT_POSE_TIMEOUT_SEC = 0.5
DEFAULT_CAN_FRAME_MODE = "normal"
DEFAULT_RESTART_CAN_ID = 0x30
DEFAULT_RESTART_PAYLOAD = bytes.fromhex("A55A01FE00000000")
CAN_FRAME_MODES = (
    "normal",
    "yaw_180",
    "flip_xy_yaw_180",
    "flip_x",
    "flip_y",
    "swap_xy",
    "swap_xy_flip_x",
    "swap_xy_flip_y",
    "yaw_neg",
)
CANFD_DLC_BY_LENGTH = {
    0: "0",
    1: "1",
    2: "2",
    3: "3",
    4: "4",
    5: "5",
    6: "6",
    7: "7",
    8: "8",
    12: "9",
    16: "A",
    20: "B",
    24: "C",
    32: "D",
    48: "E",
    64: "F",
}
CANFD_LENGTH_BY_DLC = {code: length for length, code in CANFD_DLC_BY_LENGTH.items()}


def pack_pose_payload(x: float, y: float, z: float, yaw: float) -> bytes:
    return struct.pack("<ffff", float(x), float(y), float(z), float(yaw))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_int_value(value) -> int:
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def transform_can_pose(
    x: float,
    y: float,
    z: float,
    yaw: float,
    can_frame_mode: str,
    yaw_offset_rad: float = 0.0,
):
    mode = str(can_frame_mode).lower()
    if mode == "normal":
        tx = float(x)
        ty = float(y)
        tyaw = float(yaw)
    elif mode == "yaw_180":
        tx = float(x)
        ty = float(y)
        tyaw = float(yaw) + math.pi
    elif mode == "flip_xy_yaw_180":
        tx = -float(x)
        ty = -float(y)
        tyaw = float(yaw) + math.pi
    elif mode == "flip_x":
        tx = -float(x)
        ty = float(y)
        tyaw = float(yaw)
    elif mode == "flip_y":
        tx = float(x)
        ty = -float(y)
        tyaw = float(yaw)
    elif mode == "swap_xy":
        tx = float(y)
        ty = float(x)
        tyaw = float(yaw)
    elif mode == "swap_xy_flip_x":
        tx = -float(y)
        ty = float(x)
        tyaw = float(yaw)
    elif mode == "swap_xy_flip_y":
        tx = float(y)
        ty = -float(x)
        tyaw = float(yaw)
    elif mode == "yaw_neg":
        tx = float(x)
        ty = float(y)
        tyaw = -float(yaw)
    else:
        raise ValueError(
            f"can_frame_mode must be one of {', '.join(CAN_FRAME_MODES)}"
        )
    return tx, ty, float(z), normalize_angle(tyaw + float(yaw_offset_rad))


class SocketCanFdTransport:
    def __init__(self, interface: str) -> None:
        self.interface = interface
        can_raw = getattr(socket, "CAN_RAW", 1)
        sol_can_raw = getattr(socket, "SOL_CAN_RAW", 101)
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, can_raw)
        self.socket.setsockopt(sol_can_raw, CAN_RAW_FD_FRAMES, struct.pack("I", 1))
        self.socket.bind((interface,))
        self.socket.setblocking(False)

    def send(self, can_id: int, payload: bytes, enable_brs: bool) -> None:
        flags = CANFD_BRS if enable_brs else 0
        frame = struct.pack(
            CANFD_FRAME_FORMAT,
            can_id,
            len(payload),
            flags,
            0,
            0,
            payload.ljust(64, b"\x00"),
        )
        self.socket.send(frame)

    def close(self) -> None:
        self.socket.close()

    def read_frames(self):
        frames = []
        while True:
            try:
                data = self.socket.recv(CANFD_FRAME_SIZE)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise

            if len(data) >= CANFD_FRAME_SIZE:
                can_id, length, _flags, _res0, _res1, payload = struct.unpack(
                    CANFD_FRAME_FORMAT, data[:CANFD_FRAME_SIZE]
                )
                frames.append((can_id & 0x1FFFFFFF, payload[:length]))
            elif len(data) >= CAN_FRAME_SIZE:
                can_id, length, payload = struct.unpack(
                    CAN_FRAME_FORMAT, data[:CAN_FRAME_SIZE]
                )
                frames.append((can_id & 0x1FFFFFFF, payload[:length]))
        return frames


class SlcanFdTransport:
    def __init__(
        self,
        device: str,
        nominal_speed_command: str,
        data_speed_command: str,
        auto_retransmit: bool,
    ) -> None:
        self.device = device
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.rx_buffer = b""
        self.configure_serial_port()
        self.write_command("C")
        self.write_command(nominal_speed_command)
        self.write_command(data_speed_command)
        self.write_command("A1" if auto_retransmit else "A0")
        self.write_command("O")

    def configure_serial_port(self) -> None:
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        speed = getattr(termios, "B3000000", termios.B115200)
        attrs[4] = speed
        attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write_command(self, command: str) -> None:
        os.write(self.fd, f"{command}\r".encode("ascii"))
        time.sleep(0.02)

    def send(self, can_id: int, payload: bytes, enable_brs: bool) -> None:
        if len(payload) not in CANFD_DLC_BY_LENGTH:
            raise ValueError(f"invalid CAN FD payload length: {len(payload)}")
        command = "b" if enable_brs else "d"
        dlc = CANFD_DLC_BY_LENGTH[len(payload)]
        line = f"{command}{can_id:03X}{dlc}{payload.hex().upper()}\r"
        os.write(self.fd, line.encode("ascii"))

    def read_frames(self):
        lines = []
        while True:
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise

            if not chunk:
                break
            self.rx_buffer += chunk
            while True:
                cr = self.rx_buffer.find(b"\r")
                lf = self.rx_buffer.find(b"\n")
                indexes = [idx for idx in (cr, lf) if idx >= 0]
                if not indexes:
                    break
                end = min(indexes)
                raw_line = self.rx_buffer[:end]
                self.rx_buffer = self.rx_buffer[end + 1 :]
                line = raw_line.decode("ascii", errors="ignore").strip()
                if line:
                    lines.append(line)

            if len(self.rx_buffer) > 4096:
                self.rx_buffer = self.rx_buffer[-4096:]

        frames = []
        for line in lines:
            frame = parse_slcan_frame(line)
            if frame is not None:
                frames.append(frame)
        return frames

    def close(self) -> None:
        try:
            self.write_command("C")
        finally:
            os.close(self.fd)


def parse_slcan_frame(line: str):
    text = str(line).strip()
    if not text:
        return None

    command = text[0]
    if command in ("t", "r", "d", "b"):
        id_len = 3
        fd_frame = command in ("d", "b")
    elif command in ("T", "R", "D", "B"):
        id_len = 8
        fd_frame = command in ("D", "B")
    else:
        return None

    min_len = 1 + id_len + 1
    if len(text) < min_len:
        return None

    try:
        can_id = int(text[1 : 1 + id_len], 16)
    except ValueError:
        return None

    dlc_code = text[1 + id_len].upper()
    if fd_frame:
        length = CANFD_LENGTH_BY_DLC.get(dlc_code)
    else:
        try:
            length = int(dlc_code, 16)
        except ValueError:
            return None
        if length > 8:
            return None
    if length is None:
        return None

    data_start = min_len
    data_end = data_start + length * 2
    if len(text) < data_end:
        return None

    try:
        payload = bytes.fromhex(text[data_start:data_end])
    except ValueError:
        return None
    return can_id, payload


class CanFdPoseSender(Node):
    def __init__(self) -> None:
        super().__init__("canfd_pose_sender")

        self.declare_parameter("pose_topic", DEFAULT_POSE_TOPIC)
        self.declare_parameter("transport", "auto")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("serial_device", "auto")
        self.declare_parameter("can_id", 0x12)
        self.declare_parameter("send_rate_hz", DEFAULT_SEND_RATE_HZ)
        self.declare_parameter("pose_timeout_sec", DEFAULT_POSE_TIMEOUT_SEC)
        self.declare_parameter("enable_brs", True)
        self.declare_parameter("auto_retransmit", DEFAULT_AUTO_RETRANSMIT)
        self.declare_parameter("yaw_offset_deg", 0.0)
        self.declare_parameter("can_frame_mode", DEFAULT_CAN_FRAME_MODE)
        self.declare_parameter("debug_csv_path", "")
        self.declare_parameter("debug_log_period_sec", 1.0)
        self.declare_parameter("slcan_nominal_speed_command", "S8")
        self.declare_parameter("slcan_data_speed_command", "Y2")
        self.declare_parameter("reconnect_on_error", True)
        self.declare_parameter("reconnect_period_sec", 0.5)
        self.declare_parameter("reconnect_log_period_sec", 2.0)
        self.declare_parameter("serial_presence_check_period_sec", 0.5)
        self.declare_parameter("allow_serial_fallback", True)
        self.declare_parameter(
            "serial_fallback_globs",
            ["/dev/serial/by-id/usb-Openlight_Labs_CANable2*"],
        )
        self.declare_parameter("restart_enabled", True)
        self.declare_parameter("restart_can_id", DEFAULT_RESTART_CAN_ID)
        self.declare_parameter("restart_payload_hex", DEFAULT_RESTART_PAYLOAD.hex())
        self.declare_parameter(
            "restart_script",
            os.path.join(
                os.environ.get("RC_LIDAR_WORKSPACE", os.getcwd()),
                "autostart",
                "restart_lidar_stack.sh",
            ),
        )
        self.declare_parameter("restart_debounce_sec", 8.0)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.configured_transport = self.get_parameter("transport").value
        self.transport_name = self.configured_transport
        self.can_interface = self.get_parameter("can_interface").value
        self.configured_serial_device = self.get_parameter("serial_device").value
        self.serial_device = self.configured_serial_device
        self.can_id = parse_int_value(self.get_parameter("can_id").value)
        self.send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.enable_brs = bool(self.get_parameter("enable_brs").value)
        self.auto_retransmit = bool(self.get_parameter("auto_retransmit").value)
        self.slcan_nominal_speed_command = self.get_parameter(
            "slcan_nominal_speed_command"
        ).value
        self.slcan_data_speed_command = self.get_parameter(
            "slcan_data_speed_command"
        ).value
        self.yaw_offset_rad = math.radians(
            float(self.get_parameter("yaw_offset_deg").value)
        )
        self.can_frame_mode = str(self.get_parameter("can_frame_mode").value).lower()
        if self.can_frame_mode not in CAN_FRAME_MODES:
            raise ValueError(
                f"can_frame_mode must be one of {', '.join(CAN_FRAME_MODES)}"
            )
        self.debug_csv_path = str(self.get_parameter("debug_csv_path").value or "")
        self.debug_log_period_sec = float(
            self.get_parameter("debug_log_period_sec").value
        )
        self.reconnect_on_error = bool(self.get_parameter("reconnect_on_error").value)
        self.reconnect_period_sec = float(
            self.get_parameter("reconnect_period_sec").value
        )
        self.reconnect_log_period_sec = float(
            self.get_parameter("reconnect_log_period_sec").value
        )
        self.serial_presence_check_period_sec = float(
            self.get_parameter("serial_presence_check_period_sec").value
        )
        self.allow_serial_fallback = bool(
            self.get_parameter("allow_serial_fallback").value
        )
        self.serial_fallback_globs = self.get_string_list_parameter(
            "serial_fallback_globs"
        )
        self.restart_enabled = bool(self.get_parameter("restart_enabled").value)
        self.restart_can_id = parse_int_value(self.get_parameter("restart_can_id").value)
        self.restart_payload = bytes.fromhex(
            str(self.get_parameter("restart_payload_hex").value).replace(" ", "")
        )
        self.restart_script = str(self.get_parameter("restart_script").value)
        self.restart_debounce_sec = float(
            self.get_parameter("restart_debounce_sec").value
        )
        self.last_restart_monotonic = 0.0
        self.latest_pose: Optional[PoseStamped] = None
        self.latest_pose_monotonic: Optional[float] = None
        self.transport = None
        self.next_reconnect_monotonic = 0.0
        self.last_reconnect_log_monotonic = 0.0
        self.last_serial_presence_check_monotonic = 0.0
        self.last_debug_log_monotonic = 0.0
        self.debug_csv_file = None
        self.debug_csv_writer = None
        self.sent_payload_count = 0

        self.open_debug_csv()
        self.ensure_transport_open(initial=True)
        self.subscription = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, 10
        )
        self.timer = self.create_timer(1.0 / self.send_rate_hz, self.timer_callback)

        self.get_logger().info(
            f"sending {self.pose_topic} as CAN FD id=0x{self.can_id:03X} "
            f"via {self.transport_name} rate={self.send_rate_hz:.1f}Hz "
            f"pose_timeout_sec={self.pose_timeout_sec:.3f} "
            f"brs={'on' if self.enable_brs else 'off'} "
            f"auto_retransmit={'on' if self.auto_retransmit else 'off'} "
            f"can_frame_mode={self.can_frame_mode} "
            f"yaw_offset_deg={math.degrees(self.yaw_offset_rad):.3f} "
            f"debug_csv={self.debug_csv_path or 'off'} "
            f"restart={'on' if self.restart_enabled else 'off'} "
            f"restart_can_id=0x{self.restart_can_id:03X} "
            f"restart_debounce_sec={self.restart_debounce_sec:.3f} "
            "payload=<x,y,z,yaw float32 little-endian>"
        )

    def open_debug_csv(self) -> None:
        if not self.debug_csv_path:
            return

        parent = os.path.dirname(self.debug_csv_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        needs_header = (
            not os.path.exists(self.debug_csv_path)
            or os.path.getsize(self.debug_csv_path) == 0
        )
        self.debug_csv_file = open(
            self.debug_csv_path, "a", newline="", encoding="utf-8"
        )
        self.debug_csv_writer = csv.writer(self.debug_csv_file)
        if needs_header:
            self.debug_csv_writer.writerow(
                [
                    "time_sec",
                    "can_frame_mode",
                    "input_x",
                    "input_y",
                    "input_z",
                    "input_yaw",
                    "output_x",
                    "output_y",
                    "output_z",
                    "output_yaw",
                ]
            )
            self.debug_csv_file.flush()

    def ensure_transport_open(self, initial: bool = False) -> bool:
        if self.transport is not None:
            return True

        now = time.monotonic()
        if now < self.next_reconnect_monotonic:
            return False

        try:
            self.transport = self.open_transport()
        except (OSError, RuntimeError, ValueError) as exc:
            if not self.reconnect_on_error:
                raise
            self.next_reconnect_monotonic = now + self.reconnect_period_sec
            if initial or now - self.last_reconnect_log_monotonic >= self.reconnect_log_period_sec:
                self.last_reconnect_log_monotonic = now
                self.get_logger().warn(
                    "CAN FD transport unavailable; retrying in "
                    f"{self.reconnect_period_sec:.1f}s: {exc}"
                )
            return False

        self.get_logger().info(f"CAN FD transport ready: {self.transport_name}")
        return True

    def close_transport(self) -> None:
        transport = self.transport
        self.transport = None
        if transport is None:
            return

        try:
            transport.close()
        except OSError as exc:
            self.get_logger().warn(f"failed to close stale CAN FD transport: {exc}")

    def handle_transport_error(self, exc: OSError) -> None:
        self.get_logger().error(f"failed to send CAN FD frame: {exc}")
        if not self.reconnect_on_error:
            return

        self.close_transport()
        self.next_reconnect_monotonic = time.monotonic() + self.reconnect_period_sec
        self.get_logger().warn(
            "CAN FD transport will reconnect via configured device "
            f"in {self.reconnect_period_sec:.1f}s"
        )

    def check_transport_presence(self) -> None:
        if getattr(self, "transport", None) is None:
            return
        if self.serial_presence_check_period_sec <= 0.0:
            return
        if not str(self.transport_name).startswith("slcan_fd:"):
            return

        now = time.monotonic()
        if (
            now - self.last_serial_presence_check_monotonic
            < self.serial_presence_check_period_sec
        ):
            return
        self.last_serial_presence_check_monotonic = now

        device = getattr(self.transport, "device", self.serial_device)
        if device and not os.path.exists(device):
            self.get_logger().warn(
                f"CAN FD serial device disappeared: {device}; "
                "closing stale transport and waiting for replug"
            )
            self.close_transport()
            self.next_reconnect_monotonic = now + self.reconnect_period_sec

    def open_transport(self):
        transport = str(self.configured_transport).lower()
        if transport == "auto":
            if os.path.exists(f"/sys/class/net/{self.can_interface}"):
                transport = "socketcan"
            elif self.find_serial_device():
                transport = "slcan_fd"
            else:
                raise RuntimeError(
                    f"neither SocketCAN {self.can_interface} nor serial "
                    f"{self.serial_device} exists"
                )

        if transport == "socketcan":
            self.transport_name = f"socketcan:{self.can_interface}"
            return SocketCanFdTransport(self.can_interface)
        if transport in ("slcan_fd", "serial", "canable2"):
            self.serial_device = self.find_serial_device(required=True)
            self.transport_name = f"slcan_fd:{self.serial_device}"
            return SlcanFdTransport(
                self.serial_device,
                self.slcan_nominal_speed_command,
                self.slcan_data_speed_command,
                self.auto_retransmit,
            )

        raise ValueError(
            "transport must be one of: auto, socketcan, slcan_fd, serial, canable2"
        )

    def find_serial_device(self, required: bool = False) -> str:
        configured_device = self.configured_serial_device
        if configured_device != "auto":
            if os.path.exists(configured_device):
                return configured_device
            fallback = self.find_fallback_serial_device()
            if fallback:
                self.get_logger().warn(
                    f"configured serial device missing: {configured_device}; "
                    f"using fallback CANable device: {fallback}"
                )
                return fallback
            if required:
                raise RuntimeError(
                    f"serial device does not exist: {configured_device}"
                )
            return ""

        fallback = self.find_fallback_serial_device()
        if fallback:
            return fallback
        if required:
            raise RuntimeError("no matching serial CANable device found")
        return ""

    def find_fallback_serial_device(self) -> str:
        if not self.allow_serial_fallback:
            return ""

        candidates = []
        for pattern in self.serial_fallback_globs:
            candidates.extend(glob(pattern))
        candidates = sorted(set(candidates))
        return candidates[0] if candidates else ""

    def get_string_list_parameter(self, name: str):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return [value] if value else []
        return [str(item) for item in value]

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.latest_pose_monotonic = time.monotonic()

    def timer_callback(self) -> None:
        self.check_transport_presence()
        self.poll_restart_command()

        if self.latest_pose is None:
            self.get_logger().warn(
                f"waiting for {self.pose_topic}", throttle_duration_sec=2.0
            )
            return

        if self.pose_timeout_sec > 0.0:
            now = time.monotonic()
            pose_age = (
                now - self.latest_pose_monotonic
                if self.latest_pose_monotonic is not None
                else math.inf
            )
            if pose_age > self.pose_timeout_sec:
                self.get_logger().warn(
                    f"stale pose from {self.pose_topic}: "
                    f"age={pose_age:.3f}s > timeout={self.pose_timeout_sec:.3f}s; "
                    "skipping CAN FD send",
                    throttle_duration_sec=2.0,
                )
                return

        pose = self.latest_pose.pose
        _, _, yaw = rpy_from_quat(pose.orientation)
        input_x = pose.position.x
        input_y = pose.position.y
        input_z = pose.position.z
        input_yaw = yaw
        x, y, z, yaw = transform_can_pose(
            input_x,
            input_y,
            input_z,
            input_yaw,
            self.can_frame_mode,
            self.yaw_offset_rad,
        )

        payload = pack_pose_payload(
            x,
            y,
            z,
            yaw,
        )
        if self.send_payload(payload):
            self.log_can_payload_debug(
                input_x, input_y, input_z, input_yaw, x, y, z, yaw
            )
            self.write_debug_csv(input_x, input_y, input_z, input_yaw, x, y, z, yaw)

    def log_can_payload_debug(
        self,
        input_x: float,
        input_y: float,
        input_z: float,
        input_yaw: float,
        output_x: float,
        output_y: float,
        output_z: float,
        output_yaw: float,
    ) -> None:
        if self.debug_log_period_sec < 0.0:
            return

        now = time.monotonic()
        if now - self.last_debug_log_monotonic < self.debug_log_period_sec:
            return
        self.last_debug_log_monotonic = now

        self.get_logger().info(
            "CAN payload "
            f"mode={self.can_frame_mode} "
            f"input=(x={input_x:.4f}, y={input_y:.4f}, z={input_z:.4f}, "
            f"yaw={input_yaw:.4f}) "
            f"output=(x={output_x:.4f}, y={output_y:.4f}, z={output_z:.4f}, "
            f"yaw={output_yaw:.4f})"
        )

    def write_debug_csv(
        self,
        input_x: float,
        input_y: float,
        input_z: float,
        input_yaw: float,
        output_x: float,
        output_y: float,
        output_z: float,
        output_yaw: float,
    ) -> None:
        if self.debug_csv_writer is None:
            return

        self.debug_csv_writer.writerow(
            [
                f"{time.time():.6f}",
                self.can_frame_mode,
                f"{input_x:.9f}",
                f"{input_y:.9f}",
                f"{input_z:.9f}",
                f"{input_yaw:.9f}",
                f"{output_x:.9f}",
                f"{output_y:.9f}",
                f"{output_z:.9f}",
                f"{output_yaw:.9f}",
            ]
        )
        self.debug_csv_file.flush()

    def send_payload(self, payload: bytes) -> bool:
        if not self.ensure_transport_open():
            return False

        try:
            self.transport.send(self.can_id, payload, self.enable_brs)
        except OSError as exc:
            self.handle_transport_error(exc)
            return False
        self.sent_payload_count += 1
        if self.sent_payload_count == 1:
            self.get_logger().info(
                f"first CAN FD payload sent via {self.transport_name}"
            )
        return True

    def poll_restart_command(self) -> None:
        if not getattr(self, "restart_enabled", False):
            return
        if not self.ensure_transport_open():
            return

        read_frames = getattr(self.transport, "read_frames", None)
        if read_frames is None:
            return

        try:
            frames = read_frames()
        except OSError as exc:
            self.handle_transport_error(exc)
            return

        for can_id, payload in frames:
            if can_id == self.restart_can_id and payload == self.restart_payload:
                self.request_restart()
            elif can_id == self.restart_can_id:
                self.get_logger().warn(
                    f"restart CAN id 0x{self.restart_can_id:03X} payload mismatch: "
                    f"{payload.hex().upper()}",
                    throttle_duration_sec=2.0,
                )

    def request_restart(self) -> None:
        now = time.monotonic()
        if (
            self.restart_debounce_sec > 0.0
            and self.last_restart_monotonic > 0.0
            and now - self.last_restart_monotonic < self.restart_debounce_sec
        ):
            self.get_logger().warn(
                "CAN FD restart command ignored because of debounce "
                f"({self.restart_debounce_sec:.1f}s)",
                throttle_duration_sec=1.0,
            )
            return

        self.last_restart_monotonic = now
        try:
            process = subprocess.Popen(
                ["/bin/bash", self.restart_script],
                close_fds=True,
                start_new_session=True,
            )
        except OSError as exc:
            self.get_logger().error(
                f"failed to launch restart script {self.restart_script}: {exc}"
            )
            return

        self.get_logger().warn(
            "Received CAN FD restart command; launched "
            f"{self.restart_script} pid={process.pid}"
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return normalize_angle(angle)

    def destroy_node(self) -> bool:
        self.close_transport()
        if self.debug_csv_file is not None:
            self.debug_csv_file.close()
            self.debug_csv_file = None
            self.debug_csv_writer = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CanFdPoseSender()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
