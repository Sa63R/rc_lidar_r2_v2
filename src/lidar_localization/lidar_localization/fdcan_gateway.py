#!/usr/bin/env python3
import argparse
import errno
import fcntl
import os
import select
import signal
import socket
import sys
import termios
import time
from glob import glob


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
DEFAULT_SOCKET_PATH = "/tmp/rc_fdcan_gateway.sock"
DEFAULT_LOCK_PATH = "/tmp/rc_fdcan_gateway.lock"
DEFAULT_SERIAL_DEVICE = (
    "/dev/serial/by-id/"
    "usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_"
    "206D35C24831-if00"
)


def log(message: str) -> None:
    print(f"[fdcan_gateway] {time.strftime('%F %T')} {message}", flush=True)


def parse_int_value(value) -> int:
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def parse_payload_hex(text: str) -> bytes:
    clean = str(text).replace(" ", "").strip()
    if len(clean) % 2 != 0:
        raise ValueError("payload hex must have an even number of characters")
    return bytes.fromhex(clean)


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


class SlcanFdDevice:
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

    def send_classic(self, can_id: int, payload: bytes) -> None:
        if len(payload) > 8:
            raise ValueError(f"classic CAN payload too long: {len(payload)}")
        if can_id <= 0x7FF:
            line = f"t{can_id:03X}{len(payload):X}{payload.hex().upper()}\r"
        else:
            line = f"T{can_id:08X}{len(payload):X}{payload.hex().upper()}\r"
        os.write(self.fd, line.encode("ascii"))

    def send_fd(self, can_id: int, payload: bytes, enable_brs: bool) -> None:
        if len(payload) not in CANFD_DLC_BY_LENGTH:
            raise ValueError(f"invalid CAN FD payload length: {len(payload)}")
        if can_id <= 0x7FF:
            command = "b" if enable_brs else "d"
            id_text = f"{can_id:03X}"
        else:
            command = "B" if enable_brs else "D"
            id_text = f"{can_id:08X}"
        dlc = CANFD_DLC_BY_LENGTH[len(payload)]
        line = f"{command}{id_text}{dlc}{payload.hex().upper()}\r"
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
                indexes = [
                    idx for idx in (self.rx_buffer.find(b"\r"), self.rx_buffer.find(b"\n"))
                    if idx >= 0
                ]
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
        except OSError:
            pass
        os.close(self.fd)


class FdcanGateway:
    def __init__(self, args) -> None:
        self.args = args
        self.socket_path = args.socket_path
        self.lock_path = args.lock_path
        self.sock = None
        self.lock_fd = None
        self.device = None
        self.clients = set()
        self.running = True
        self.next_open_monotonic = 0.0
        self.last_open_log_monotonic = 0.0

    def install_signal_handlers(self) -> None:
        def stop(_signum, _frame):
            self.running = False

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)

    def acquire_lock(self) -> None:
        self.lock_fd = os.open(self.lock_path, os.O_CREAT | os.O_RDWR, 0o666)
        try:
            fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(
                f"another fdcan_gateway instance already holds {self.lock_path}"
            ) from exc

    def open_socket(self) -> None:
        parent = os.path.dirname(self.socket_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass

        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(self.socket_path)
        os.chmod(self.socket_path, 0o666)
        self.sock.setblocking(False)
        log(f"gateway socket ready: {self.socket_path}")

    def resolve_serial_device(self) -> str:
        if self.args.serial_device != "auto":
            return self.args.serial_device

        candidates = []
        for pattern in self.args.serial_fallback_globs:
            candidates.extend(glob(pattern))
        candidates = sorted(set(candidates))
        return candidates[0] if candidates else ""

    def ensure_device_open(self) -> bool:
        if self.device is not None:
            return True

        now = time.monotonic()
        if now < self.next_open_monotonic:
            return False

        serial_device = self.resolve_serial_device()
        if not serial_device:
            self.next_open_monotonic = now + self.args.reconnect_period_sec
            if now - self.last_open_log_monotonic >= self.args.reconnect_log_period_sec:
                self.last_open_log_monotonic = now
                log("waiting for Openlight CANable2 serial device")
            return False

        try:
            self.device = SlcanFdDevice(
                serial_device,
                self.args.nominal_speed_command,
                self.args.data_speed_command,
                self.args.auto_retransmit,
            )
        except OSError as exc:
            self.device = None
            self.next_open_monotonic = now + self.args.reconnect_period_sec
            if now - self.last_open_log_monotonic >= self.args.reconnect_log_period_sec:
                self.last_open_log_monotonic = now
                log(f"transport unavailable, retrying: {serial_device}: {exc}")
            return False

        log(
            "Openlight CANable2 ready: "
            f"{serial_device} nominal={self.args.nominal_speed_command} "
            f"data={self.args.data_speed_command}"
        )
        self.broadcast("STATUS READY")
        return True

    def close_device(self, reason: str) -> None:
        if self.device is None:
            return
        log(f"closing Openlight CANable2 transport: {reason}")
        try:
            self.device.close()
        except OSError as exc:
            log(f"close failed: {exc}")
        self.device = None
        self.next_open_monotonic = time.monotonic() + self.args.reconnect_period_sec
        self.broadcast("STATUS DOWN")

    def send_to_client(self, address, line: str) -> None:
        if not address:
            return
        try:
            self.sock.sendto((line + "\n").encode("ascii"), address)
        except OSError as exc:
            log(f"drop stale client {address}: {exc}")
            self.clients.discard(address)

    def broadcast(self, line: str) -> None:
        for address in list(self.clients):
            self.send_to_client(address, line)

    def handle_socket_ready(self) -> None:
        while True:
            try:
                data, address = self.sock.recvfrom(4096)
            except BlockingIOError:
                return

            if address:
                self.clients.add(address)

            try:
                reply = self.handle_command(data.decode("ascii", errors="ignore"), address)
            except Exception as exc:
                reply = f"ERR {exc}"

            if reply:
                self.send_to_client(address, reply)

    def handle_command(self, raw: str, address) -> str:
        parts = raw.strip().split()
        if not parts:
            return ""

        command = parts[0].upper()
        if command == "PING":
            return "PONG"
        if command == "HELLO":
            return "OK HELLO"
        if command == "BYE":
            if address:
                self.clients.discard(address)
            return ""

        if command == "TXCAN":
            if len(parts) != 3:
                raise ValueError("TXCAN requires: TXCAN <can_id> <payload_hex>")
            can_id = parse_int_value(parts[1])
            payload = parse_payload_hex(parts[2])
            self.send_classic(can_id, payload)
            return ""

        if command == "TXFD":
            if len(parts) != 4:
                raise ValueError("TXFD requires: TXFD <can_id> <brs 0|1> <payload_hex>")
            can_id = parse_int_value(parts[1])
            enable_brs = parts[2] in ("1", "true", "TRUE", "yes", "YES", "on", "ON")
            payload = parse_payload_hex(parts[3])
            self.send_fd(can_id, payload, enable_brs)
            return ""

        raise ValueError(f"unknown command: {parts[0]}")

    def send_classic(self, can_id: int, payload: bytes) -> None:
        if not self.ensure_device_open():
            raise RuntimeError("FDCAN transport is not ready")
        try:
            self.device.send_classic(can_id, payload)
        except OSError as exc:
            self.close_device(f"classic send failed: {exc}")
            raise

    def send_fd(self, can_id: int, payload: bytes, enable_brs: bool) -> None:
        if not self.ensure_device_open():
            raise RuntimeError("FDCAN transport is not ready")
        try:
            self.device.send_fd(can_id, payload, enable_brs)
        except OSError as exc:
            self.close_device(f"CAN FD send failed: {exc}")
            raise

    def handle_serial_ready(self) -> None:
        if self.device is None:
            return
        try:
            frames = self.device.read_frames()
        except OSError as exc:
            self.close_device(f"read failed: {exc}")
            return

        for can_id, payload in frames:
            self.broadcast(f"RX 0x{can_id:X} {payload.hex().upper()}")

    def run(self) -> None:
        self.acquire_lock()
        self.open_socket()
        self.install_signal_handlers()
        log("fdcan_gateway started")

        while self.running:
            self.ensure_device_open()
            read_list = [self.sock]
            if self.device is not None:
                read_list.append(self.device.fd)

            try:
                readable, _, _ = select.select(read_list, [], [], 0.05)
            except OSError as exc:
                if exc.errno == errno.EINTR:
                    continue
                raise

            if self.sock in readable:
                self.handle_socket_ready()
            if self.device is not None and self.device.fd in readable:
                self.handle_serial_ready()

        self.close()

    def close(self) -> None:
        self.close_device("shutdown")
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass
        if self.lock_fd is not None:
            os.close(self.lock_fd)
            self.lock_fd = None
        log("fdcan_gateway stopped")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Shared Openlight CANable2 FDCAN gateway")
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--lock-path", default=DEFAULT_LOCK_PATH)
    parser.add_argument("--serial-device", default=DEFAULT_SERIAL_DEVICE)
    parser.add_argument(
        "--serial-fallback-glob",
        dest="serial_fallback_globs",
        action="append",
        default=["/dev/serial/by-id/usb-Openlight_Labs_CANable2*"],
    )
    parser.add_argument("--nominal-speed-command", default="S8")
    parser.add_argument("--data-speed-command", default="Y2")
    parser.add_argument("--auto-retransmit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reconnect-period-sec", type=float, default=0.5)
    parser.add_argument("--reconnect-log-period-sec", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    gateway = FdcanGateway(args)
    try:
        gateway.run()
    except RuntimeError as exc:
        log(f"ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        gateway.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
