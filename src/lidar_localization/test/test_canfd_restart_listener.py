#!/usr/bin/env python3
import argparse
import importlib.util
from pathlib import Path
import socket
import tempfile
import threading
import time
import unittest


AUTOSTART = Path(__file__).resolve().parents[3] / "autostart"
SPEC = importlib.util.spec_from_file_location(
    "canfd_restart_listener", AUTOSTART / "canfd_restart_listener.py"
)
assert SPEC is not None and SPEC.loader is not None
listener_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(listener_module)


class FakeGatewayClient:
    def __init__(self, messages):
        self.messages = messages
        self.register_count = 0
        self.ping_count = 0

    def register(self):
        self.register_count += 1

    def ping(self):
        self.ping_count += 1

    def receive(self):
        messages, self.messages = self.messages, []
        return messages

    def close(self):
        pass


class UnavailableGatewayClient(FakeGatewayClient):
    def register(self):
        raise FileNotFoundError("gateway unavailable")


def make_args(tmpdir: str) -> argparse.Namespace:
    return argparse.Namespace(
        gateway_socket=str(Path(tmpdir) / "gateway.sock"),
        can_id=0x777,
        restart_script="/tmp/restart_lidar_stack.sh",
        debounce_sec=8.0,
        poll_sec=0.01,
        register_period_sec=1.0,
        heartbeat_period_sec=1.0,
        reconnect_log_period_sec=2.0,
        lock_path=str(Path(tmpdir) / "listener.lock"),
        heartbeat_path=str(Path(tmpdir) / "listener.heartbeat"),
        event_path=str(Path(tmpdir) / "listener.event"),
        probe=False,
        probe_timeout_sec=0.25,
    )


class RestartListenerTest(unittest.TestCase):
    def test_parse_gateway_rx_and_ignore_malformed_messages(self):
        self.assertEqual(
            listener_module.parse_gateway_message(b"RX 0x777 A55A01FE00000000\n"),
            ("rx", 0x777, bytes.fromhex("A55A01FE00000000")),
        )
        self.assertEqual(
            listener_module.parse_gateway_message(b"RX broken payload"),
            ("ignore", None, None),
        )
        self.assertEqual(
            listener_module.parse_gateway_message(b"PONG\n"),
            ("pong", None, None),
        )
        self.assertEqual(
            listener_module.parse_gateway_message(b"RX 0x777\n"),
            ("rx", 0x777, b""),
        )

    def test_only_target_can_id_requests_restart_and_debounces(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            listener = listener_module.RestartListener(make_args(tmpdir))
            fake_client = FakeGatewayClient(
                [
                    ("rx", 0x123, b"ignored"),
                    ("rx", 0x777, b"first"),
                    ("rx", 0x777, b"second"),
                ]
            )
            listener.client = fake_client
            calls = []

            class FakeProcess:
                pid = 42

                @staticmethod
                def poll():
                    return None

            original_popen = listener_module.subprocess.Popen
            try:
                listener_module.subprocess.Popen = (
                    lambda args, close_fds, start_new_session, env:
                    calls.append((args, close_fds, start_new_session, env)) or FakeProcess()
                )
                listener.run_once(time.monotonic())
            finally:
                listener_module.subprocess.Popen = original_popen

            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0][0], ["/bin/bash", "/tmp/restart_lidar_stack.sh"])
            self.assertIn("RC_CAN_RESTART_REQUEST_ID", calls[0][3])
            self.assertIn("RC_CAN_RESTART_RESULT_FILE", calls[0][3])
            self.assertEqual(fake_client.register_count, 1)
            self.assertEqual(fake_client.ping_count, 1)
            self.assertTrue(Path(listener.args.heartbeat_path).is_file())
            event_text = Path(listener.args.event_path).read_text(encoding="utf-8")
            self.assertIn("can_id=0x777", event_text)
            self.assertIn("action=rejected", event_text)
            self.assertIn("reason=debounce", event_text)

    def test_pending_restart_result_becomes_success(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            listener = listener_module.RestartListener(make_args(tmpdir))

            class FakeProcess:
                pid = 43

                @staticmethod
                def poll():
                    return None

            original_popen = listener_module.subprocess.Popen
            try:
                listener_module.subprocess.Popen = lambda *_args, **_kwargs: FakeProcess()
                listener.request_restart(b"restart")
            finally:
                listener_module.subprocess.Popen = original_popen

            self.assertEqual(len(listener.pending_restarts), 1)
            pending = next(iter(listener.pending_restarts.values()))
            pending.result_path.write_text(
                "\n".join(
                    [
                        "status=success",
                        "reason=stack_started",
                        "detail=core components are running",
                        "run_id=test_run",
                        f"run_dir={tmpdir}/test_run",
                        "",
                    ]
                ),
                encoding="utf-8",
            )
            listener.poll_pending_restarts()

            self.assertFalse(listener.pending_restarts)
            event_text = Path(listener.args.event_path).read_text(encoding="utf-8")
            self.assertIn("action=succeeded", event_text)
            self.assertIn("reason=stack_started", event_text)
            self.assertIn("run_id=test_run", event_text)
            self.assertFalse(pending.result_path.exists())

    def test_process_exit_without_result_becomes_failure(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            listener = listener_module.RestartListener(make_args(tmpdir))

            class FakeProcess:
                pid = 44

                @staticmethod
                def poll():
                    return 7

            original_popen = listener_module.subprocess.Popen
            try:
                listener_module.subprocess.Popen = lambda *_args, **_kwargs: FakeProcess()
                listener.request_restart(b"restart")
            finally:
                listener_module.subprocess.Popen = original_popen

            listener.poll_pending_restarts()
            event_text = Path(listener.args.event_path).read_text(encoding="utf-8")
            self.assertIn("action=failed", event_text)
            self.assertIn("reason=missing_result", event_text)

    def test_separate_requests_have_separate_ids_and_result_files(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            listener = listener_module.RestartListener(make_args(tmpdir))

            class FakeProcess:
                pid = 45

                @staticmethod
                def poll():
                    return None

            original_popen = listener_module.subprocess.Popen
            try:
                listener_module.subprocess.Popen = lambda *_args, **_kwargs: FakeProcess()
                listener.request_restart(b"first")
                listener.last_restart_monotonic = 0.0
                listener.request_restart(b"second")
            finally:
                listener_module.subprocess.Popen = original_popen

            request_ids = list(listener.pending_restarts)
            result_paths = [
                pending.result_path for pending in listener.pending_restarts.values()
            ]
            self.assertEqual(len(request_ids), 2)
            self.assertEqual(len(set(request_ids)), 2)
            self.assertEqual(len(set(result_paths)), 2)

    def test_single_instance_lock_rejects_second_listener(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            first = listener_module.RestartListener(make_args(tmpdir))
            second = listener_module.RestartListener(make_args(tmpdir))
            first.acquire_lock()
            try:
                with self.assertRaises(RuntimeError):
                    second.acquire_lock()
            finally:
                if second.lock_file is not None:
                    second.lock_file.close()
                if first.lock_file is not None:
                    first.lock_file.close()

    def test_listener_reconnects_after_gateway_unavailable(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            listener = listener_module.RestartListener(make_args(tmpdir))
            unavailable = UnavailableGatewayClient([])
            recovered = FakeGatewayClient([])
            clients = iter([unavailable, recovered])
            original_client = listener_module.GatewayClient
            try:
                listener_module.GatewayClient = lambda *_args: next(clients)
                listener.run_once(time.monotonic())
                self.assertIsNone(listener.client)
                listener.last_register_monotonic = 0.0
                listener.run_once(time.monotonic())
            finally:
                listener_module.GatewayClient = original_client

            self.assertIs(listener.client, recovered)
            self.assertEqual(recovered.register_count, 1)
            self.assertEqual(recovered.ping_count, 1)

    def test_gateway_probe_requires_pong(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            socket_path = str(Path(tmpdir) / "gateway.sock")
            server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            server.bind(socket_path)
            server.settimeout(1.0)

            def reply_once():
                data, address = server.recvfrom(4096)
                if data.strip() == b"PING":
                    server.sendto(b"PONG\n", address)

            thread = threading.Thread(target=reply_once)
            thread.start()
            try:
                self.assertTrue(listener_module.probe_gateway(socket_path, 0.5))
            finally:
                thread.join(timeout=1.0)
                server.close()

            self.assertFalse(
                listener_module.probe_gateway(str(Path(tmpdir) / "missing.sock"), 0.05)
            )


if __name__ == "__main__":
    unittest.main()
