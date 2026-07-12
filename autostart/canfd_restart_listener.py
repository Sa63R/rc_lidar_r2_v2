#!/usr/bin/env python3
"""Persistent 0x777 restart listener for the shared FDCAN gateway."""

from __future__ import annotations

import argparse
import errno
import fcntl
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time


DEFAULT_GATEWAY_SOCKET = "/tmp/rc_fdcan_gateway.sock"
DEFAULT_LOCK_PATH = "/tmp/rc_can_wakeup_listener.lock"
DEFAULT_HEARTBEAT_PATH = "/tmp/rc_can_wakeup_listener.heartbeat"
DEFAULT_EVENT_PATH = "/tmp/rc_can_wakeup_last_event"
DEFAULT_CAN_ID = 0x777


def log(message: str) -> None:
    print(f"[canfd_restart_listener] {time.strftime('%F %T')} {message}", flush=True)


def parse_can_id(value: str | int) -> int:
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def parse_gateway_message(data: bytes) -> tuple[str, int | None, bytes | None]:
    """Parse one gateway datagram, returning kind, CAN ID and payload."""
    text = data.decode("ascii", errors="ignore").strip()
    parts = text.split()
    if not parts:
        return "ignore", None, None
    if parts[0] == "PONG":
        return "pong", None, None
    if len(parts) not in (2, 3) or parts[0] != "RX":
        return "ignore", None, None
    try:
        can_id = int(parts[1], 0) & 0x1FFFFFFF
        payload = bytes.fromhex(parts[2]) if len(parts) == 3 else b""
    except ValueError:
        return "ignore", None, None
    return "rx", can_id, payload


class GatewayClient:
    def __init__(self, socket_path: str, client_name: str) -> None:
        self.socket_path = socket_path
        self.client_name = client_name
        self.client_path = (
            f"/tmp/rc_fdcan_client_{client_name}_{os.getpid()}_"
            f"{int(time.time() * 1000)}.sock"
        )
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            os.unlink(self.client_path)
        except FileNotFoundError:
            pass
        self.socket.bind(self.client_path)
        self.socket.setblocking(False)

    def send(self, line: str) -> None:
        self.socket.sendto((line + "\n").encode("ascii"), self.socket_path)

    def register(self) -> None:
        self.send(f"HELLO {self.client_name}")

    def ping(self) -> None:
        self.send("PING")

    def receive(self) -> list[tuple[str, int | None, bytes | None]]:
        messages = []
        while True:
            try:
                data = self.socket.recv(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise
            for line in data.splitlines():
                messages.append(parse_gateway_message(line))
        return messages

    def close(self) -> None:
        try:
            self.send("BYE")
            readable, _, _ = select.select([self.socket], [], [], 0.05)
            if readable:
                self.receive()
        except OSError:
            pass
        self.socket.close()
        try:
            os.unlink(self.client_path)
        except FileNotFoundError:
            pass


def probe_gateway(socket_path: str, timeout_sec: float = 0.25) -> bool:
    client = GatewayClient(socket_path, "can_wakeup_probe")
    try:
        client.ping()
        deadline = time.monotonic() + max(0.01, timeout_sec)
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            readable, _, _ = select.select([client.socket], [], [], remaining)
            if not readable:
                break
            if any(kind == "pong" for kind, _, _ in client.receive()):
                return True
        return False
    except OSError:
        return False
    finally:
        client.close()


class RestartListener:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.running = True
        self.lock_file = None
        self.client: GatewayClient | None = None
        self.last_restart_monotonic = 0.0
        self.last_register_monotonic = 0.0
        self.last_heartbeat_monotonic = 0.0
        self.last_unavailable_log_monotonic = 0.0

    def acquire_lock(self) -> None:
        self.lock_file = open(self.args.lock_path, "a+", encoding="utf-8")
        try:
            fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("another CAN restart listener is already running") from exc
        self.lock_file.seek(0)
        self.lock_file.truncate()
        self.lock_file.write(f"{os.getpid()}\n")
        self.lock_file.flush()

    def install_signal_handlers(self) -> None:
        def stop(_signum, _frame):
            self.running = False

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)

    def write_heartbeat(self) -> None:
        path = Path(self.args.heartbeat_path)
        tmp = path.with_name(f"{path.name}.tmp.{os.getpid()}")
        tmp.write_text(f"pid={os.getpid()}\nepoch={time.time():.6f}\n", encoding="utf-8")
        os.replace(tmp, path)

    def record_event(self, payload: bytes, action: str, detail: str = "") -> None:
        path = Path(self.args.event_path)
        tmp = path.with_name(f"{path.name}.tmp.{os.getpid()}")
        safe_detail = str(detail).replace("\n", " ").replace("\r", " ")
        tmp.write_text(
            "\n".join(
                [
                    f"event_id={time.time_ns()}-{os.getpid()}",
                    f"epoch={time.time():.6f}",
                    f"can_id=0x{self.args.can_id:X}",
                    f"payload={payload.hex().upper()}",
                    f"action={action}",
                    f"detail={safe_detail}",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        os.replace(tmp, path)

    def ensure_client(self) -> GatewayClient:
        if self.client is None:
            self.client = GatewayClient(self.args.gateway_socket, "canfd_restart_listener")
        return self.client

    def close_client(self) -> None:
        if self.client is None:
            return
        self.client.close()
        self.client = None

    def request_restart(self, payload: bytes) -> None:
        now = time.monotonic()
        if (
            self.last_restart_monotonic > 0.0
            and now - self.last_restart_monotonic < self.args.debounce_sec
        ):
            self.record_event(payload, "debounced", f"debounce_sec={self.args.debounce_sec:.1f}")
            log(f"restart frame ignored by debounce ({self.args.debounce_sec:.1f}s)")
            return
        self.last_restart_monotonic = now
        try:
            process = subprocess.Popen(
                ["/bin/bash", self.args.restart_script],
                close_fds=True,
                start_new_session=True,
            )
        except OSError as exc:
            self.record_event(payload, "launch_failed", str(exc))
            log(f"ERROR: failed to launch restart script: {exc}")
            return
        self.record_event(payload, "launched", f"pid={process.pid}")
        log(
            f"received CAN restart id=0x{self.args.can_id:X} "
            f"payload={payload.hex().upper()} launched pid={process.pid}"
        )

    def run_once(self, now: float) -> None:
        if now - self.last_heartbeat_monotonic >= self.args.heartbeat_period_sec:
            self.last_heartbeat_monotonic = now
            self.write_heartbeat()
        client = self.ensure_client()
        if now - self.last_register_monotonic >= self.args.register_period_sec:
            self.last_register_monotonic = now
            try:
                client.register()
                client.ping()
            except OSError as exc:
                if now - self.last_unavailable_log_monotonic >= self.args.reconnect_log_period_sec:
                    self.last_unavailable_log_monotonic = now
                    log(f"gateway unavailable, retrying: {exc}")
                self.close_client()
                return

        try:
            messages = client.receive()
        except OSError as exc:
            if now - self.last_unavailable_log_monotonic >= self.args.reconnect_log_period_sec:
                self.last_unavailable_log_monotonic = now
                log(f"gateway receive failed, reconnecting: {exc}")
            self.close_client()
            return

        for kind, can_id, payload in messages:
            if kind == "rx" and can_id == self.args.can_id and payload is not None:
                self.request_restart(payload)

    def run(self) -> None:
        self.acquire_lock()
        self.install_signal_handlers()
        log(
            f"started gateway={self.args.gateway_socket} "
            f"restart_id=0x{self.args.can_id:X} payload=any"
        )
        try:
            while self.running:
                self.run_once(time.monotonic())
                time.sleep(self.args.poll_sec)
        finally:
            self.close_client()
            try:
                Path(self.args.heartbeat_path).unlink()
            except FileNotFoundError:
                pass
            if self.lock_file is not None:
                self.lock_file.close()
            log("stopped")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Persistent CAN FD restart listener")
    parser.add_argument("--gateway-socket", default=DEFAULT_GATEWAY_SOCKET)
    parser.add_argument("--can-id", type=parse_can_id, default=DEFAULT_CAN_ID)
    parser.add_argument("--restart-script", required=False, default="restart_lidar_stack.sh")
    parser.add_argument("--debounce-sec", type=float, default=8.0)
    parser.add_argument("--poll-sec", type=float, default=0.05)
    parser.add_argument("--register-period-sec", type=float, default=1.0)
    parser.add_argument("--heartbeat-period-sec", type=float, default=1.0)
    parser.add_argument("--reconnect-log-period-sec", type=float, default=2.0)
    parser.add_argument("--lock-path", default=DEFAULT_LOCK_PATH)
    parser.add_argument("--heartbeat-path", default=DEFAULT_HEARTBEAT_PATH)
    parser.add_argument("--event-path", default=DEFAULT_EVENT_PATH)
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--probe-timeout-sec", type=float, default=0.25)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.probe:
        return 0 if probe_gateway(args.gateway_socket, args.probe_timeout_sec) else 1
    listener = RestartListener(args)
    try:
        listener.run()
    except RuntimeError as exc:
        log(f"ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
