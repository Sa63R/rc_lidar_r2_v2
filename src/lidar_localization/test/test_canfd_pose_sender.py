#!/usr/bin/env python3
import errno
import importlib.util
import struct
import sys
import types
import unittest
from pathlib import Path


def install_ros_test_fakes():
    if importlib.util.find_spec("rclpy") is not None:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.spin = lambda node: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None

    executors = types.ModuleType("rclpy.executors")

    class ExternalShutdownException(Exception):
        pass

    executors.ExternalShutdownException = ExternalShutdownException

    node_module = types.ModuleType("rclpy.node")

    class Node:
        pass

    node_module.Node = Node

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()

    class PoseStamped:
        def __init__(self):
            self.pose = Pose()

    geometry_msgs_msg.Point = Point
    geometry_msgs_msg.Pose = Pose
    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs_msg.Quaternion = Quaternion

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.executors"] = executors
    sys.modules["rclpy.node"] = node_module
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg


install_ros_test_fakes()

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import lidar_localization.canfd_pose_sender as canfd_pose_sender
from lidar_localization.canfd_pose_sender import CanFdPoseSender


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.infos = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)

    def warn(self, message, **kwargs):
        del kwargs
        self.warnings.append(message)


class FakeTransport:
    def __init__(self, fail=False):
        self.closed = False
        self.fail = fail
        self.sent = []

    def send(self, can_id, payload, enable_brs):
        if self.fail:
            raise OSError(errno.EIO, "Input/output error")
        self.sent.append((can_id, payload, enable_brs))

    def close(self):
        self.closed = True


class CanFdPoseSenderTest(unittest.TestCase):
    def test_canable_auto_retransmit_is_enabled_by_default(self):
        self.assertTrue(canfd_pose_sender.DEFAULT_AUTO_RETRANSMIT)

    def test_send_rate_is_100hz_by_default(self):
        self.assertEqual(canfd_pose_sender.DEFAULT_SEND_RATE_HZ, 100.0)

    def test_filtered_pose_topic_is_default_can_input(self):
        self.assertEqual(canfd_pose_sender.DEFAULT_POSE_TOPIC, "/robot/pose_filtered")

    def test_pose_timeout_default_is_half_second(self):
        self.assertEqual(canfd_pose_sender.DEFAULT_POSE_TIMEOUT_SEC, 0.5)

    def test_parse_int_value_accepts_hex_restart_id(self):
        self.assertEqual(canfd_pose_sender.parse_int_value("0x30"), 48)
        self.assertEqual(canfd_pose_sender.parse_int_value("48"), 48)

    def test_pose_payload_is_little_endian_float_xyz_yaw(self):
        payload = canfd_pose_sender.pack_pose_payload(1.0, 2.0, -3.0, 0.25)
        self.assertEqual(payload, struct.pack("<ffff", 1.0, 2.0, -3.0, 0.25))

    def test_can_frame_mode_normal_keeps_pose_values(self):
        transformed = canfd_pose_sender.transform_can_pose(
            1.0, -2.0, 0.5, 0.25, "normal"
        )
        self.assertEqual(transformed, (1.0, -2.0, 0.5, 0.25))

    def test_can_frame_mode_yaw_180_only_offsets_heading(self):
        x, y, z, yaw = canfd_pose_sender.transform_can_pose(
            1.0, -2.0, 0.5, 0.0, "yaw_180"
        )
        self.assertEqual((x, y, z), (1.0, -2.0, 0.5))
        self.assertAlmostEqual(abs(yaw), 3.141592653589793)

    def test_can_frame_mode_flip_xy_yaw_180_flips_position_and_heading(self):
        x, y, z, yaw = canfd_pose_sender.transform_can_pose(
            1.0, -2.0, 0.5, 0.25, "flip_xy_yaw_180"
        )
        self.assertEqual((x, y, z), (-1.0, 2.0, 0.5))
        self.assertAlmostEqual(yaw, 0.25 - 3.141592653589793)

    def test_can_frame_mode_extra_axis_test_modes_only_change_payload(self):
        cases = {
            "flip_x": (-1.0, -2.0, 0.5, 0.25),
            "flip_y": (1.0, 2.0, 0.5, 0.25),
            "swap_xy": (-2.0, 1.0, 0.5, 0.25),
            "swap_xy_flip_x": (2.0, 1.0, 0.5, 0.25),
            "swap_xy_flip_y": (-2.0, -1.0, 0.5, 0.25),
            "yaw_neg": (1.0, -2.0, 0.5, -0.25),
        }

        for mode, expected in cases.items():
            with self.subTest(mode=mode):
                self.assertEqual(
                    canfd_pose_sender.transform_can_pose(
                        1.0, -2.0, 0.5, 0.25, mode
                    ),
                    expected,
                )

    def test_can_frame_mode_keeps_yaw_offset_available_for_all_modes(self):
        x, y, z, yaw = canfd_pose_sender.transform_can_pose(
            1.0, -2.0, 0.5, 0.0, "flip_x", 1.5707963267948966
        )

        self.assertEqual((x, y, z), (-1.0, -2.0, 0.5))
        self.assertAlmostEqual(yaw, 1.5707963267948966)

    def test_slcan_send_writes_reference_canfd_frame_without_reading_rx(self):
        transport = canfd_pose_sender.SlcanFdTransport.__new__(
            canfd_pose_sender.SlcanFdTransport
        )
        transport.fd = 123
        writes = []

        original_read = canfd_pose_sender.os.read
        original_write = canfd_pose_sender.os.write
        try:
            canfd_pose_sender.os.read = lambda fd, size: (_ for _ in ()).throw(
                AssertionError("send must not read SLCAN RX")
            )
            canfd_pose_sender.os.write = lambda fd, data: writes.append(data) or len(data)

            transport.send(0x12, struct.pack("<ffff", 1.0, 2.0, -3.0, 0.25), True)
        finally:
            canfd_pose_sender.os.read = original_read
            canfd_pose_sender.os.write = original_write

        self.assertEqual(writes, [b"b012A0000803F00000040000040C00000803E\r"])

    def test_slcan_parser_accepts_fd_restart_frame(self):
        frame = canfd_pose_sender.parse_slcan_frame("b0308A55A01FE00000000")

        self.assertEqual(frame, (0x30, bytes.fromhex("A55A01FE00000000")))

    def test_slcan_parser_accepts_classic_restart_frame_too(self):
        frame = canfd_pose_sender.parse_slcan_frame("t0308A55A01FE00000000")

        self.assertEqual(frame, (0x30, bytes.fromhex("A55A01FE00000000")))

    def make_sender(self, transports):
        sender = CanFdPoseSender.__new__(CanFdPoseSender)
        sender.transport = transports.pop(0)
        sender.can_id = 0x12
        sender.enable_brs = True
        sender.reconnect_on_error = True
        sender.reconnect_period_sec = 0.0
        sender.reconnect_log_period_sec = 0.0
        sender.next_reconnect_monotonic = 0.0
        sender.last_reconnect_log_monotonic = 0.0
        sender.transport_name = "slcan_fd:/dev/serial/by-id/canable"
        sender.logger = FakeLogger()
        sender.get_logger = lambda: sender.logger
        sender.open_transport = lambda: transports.pop(0)
        return sender

    def make_restart_sender(self):
        sender = CanFdPoseSender.__new__(CanFdPoseSender)
        sender.restart_enabled = True
        sender.restart_can_id = 0x30
        sender.restart_payload = bytes.fromhex("A55A01FE00000000")
        sender.restart_script = "/tmp/restart_lidar_stack.sh"
        sender.restart_debounce_sec = 8.0
        sender.last_restart_monotonic = 0.0
        sender.logger = FakeLogger()
        sender.get_logger = lambda: sender.logger
        return sender

    def test_restart_request_launches_script_once(self):
        sender = self.make_restart_sender()
        calls = []

        class FakeProcess:
            pid = 1234

        original_popen = canfd_pose_sender.subprocess.Popen
        try:
            canfd_pose_sender.subprocess.Popen = (
                lambda args, close_fds, start_new_session:
                calls.append((args, close_fds, start_new_session)) or FakeProcess()
            )

            sender.request_restart()
            sender.request_restart()
        finally:
            canfd_pose_sender.subprocess.Popen = original_popen

        self.assertEqual(
            calls,
            [(["/bin/bash", "/tmp/restart_lidar_stack.sh"], True, True)],
        )
        self.assertTrue(any("debounce" in w for w in sender.logger.warnings))

    def test_poll_restart_command_uses_transport_rx_frames(self):
        sender = self.make_restart_sender()
        sender.transport = type(
            "RestartTransport",
            (),
            {"read_frames": lambda self: [(0x30, bytes.fromhex("A55A01FE00000000"))]},
        )()
        sender.ensure_transport_open = lambda: True
        called = []
        sender.request_restart = lambda: called.append(True)

        sender.poll_restart_command()

        self.assertEqual(called, [True])

    def make_timer_sender(self, pose_age_sec, pose_timeout_sec):
        sender = CanFdPoseSender.__new__(CanFdPoseSender)
        sender.pose_topic = "/robot/pose_continuous"
        sender.pose_timeout_sec = pose_timeout_sec
        sender.can_frame_mode = "normal"
        sender.yaw_offset_rad = 0.0
        sender.debug_log_period_sec = -1.0
        sender.debug_csv_writer = None
        sender.logger = FakeLogger()
        sender.get_logger = lambda: sender.logger

        msg = canfd_pose_sender.PoseStamped()
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 3.0
        sender.latest_pose = msg
        sender.latest_pose_monotonic = (
            canfd_pose_sender.time.monotonic() - pose_age_sec
        )

        sender.sent_payloads = []
        sender.send_payload = (
            lambda payload: sender.sent_payloads.append(payload) or True
        )
        return sender

    def test_timer_skips_send_when_pose_is_stale(self):
        sender = self.make_timer_sender(pose_age_sec=1.0, pose_timeout_sec=0.5)

        sender.timer_callback()

        self.assertEqual(sender.sent_payloads, [])
        self.assertTrue(
            any("stale pose from /robot/pose_continuous" in w for w in sender.logger.warnings)
        )

    def test_timer_keeps_previous_behavior_when_pose_timeout_disabled(self):
        sender = self.make_timer_sender(pose_age_sec=10.0, pose_timeout_sec=0.0)

        sender.timer_callback()

        self.assertEqual(
            sender.sent_payloads,
            [struct.pack("<ffff", 1.0, 2.0, 3.0, 0.0)],
        )

    def test_send_error_closes_stale_transport_and_reopens_next_device(self):
        stale = FakeTransport(fail=True)
        reopened = FakeTransport()
        sender = self.make_sender([stale, reopened])
        sender.sent_payload_count = 0

        sender.send_payload(b"1234567890abcdef")
        self.assertTrue(stale.closed)
        self.assertIsNone(sender.transport)

        sender.ensure_transport_open()
        self.assertIs(sender.transport, reopened)

        sender.send_payload(b"1234567890abcdef")
        self.assertEqual(reopened.sent, [(0x12, b"1234567890abcdef", True)])


if __name__ == "__main__":
    unittest.main()
