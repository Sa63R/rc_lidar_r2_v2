#!/usr/bin/env python3
from __future__ import annotations

import datetime as dt
import json
import math
import os
import queue
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import uuid
from pathlib import Path
from typing import Any, Callable

NONE_MAP_LABEL = "无"
APP_TITLE = "RC LiDAR 控制"
CAN_WAKEUP_CONTROL = Path(__file__).resolve().parent / "can_wakeup_control.sh"
MAP_POSE_PRINTER = Path(__file__).resolve().parent / "print_map_pose.sh"
CAN_FRAME_MODES = {
    "normal",
    "yaw_180",
    "flip_xy_yaw_180",
    "flip_x",
    "flip_y",
    "swap_xy",
    "swap_xy_flip_x",
    "swap_xy_flip_y",
    "yaw_neg",
}


def repo_paths() -> tuple[Path, Path]:
    autostart = Path(__file__).resolve().parent
    workspace = Path(os.environ.get("RC_LIDAR_WORKSPACE", autostart.parent)).resolve()
    return workspace, autostart


WORKSPACE, AUTOSTART = repo_paths()
MAPS_DIR = Path(os.environ.get("RC_LIDAR_MAP_DIR", WORKSPACE / "maps")).resolve()
POSES_DIR = Path(os.environ.get("RC_LIDAR_POSE_LOG_DIR", WORKSPACE / "logs" / "poses")).resolve()
LOG_ROOT = Path(os.environ.get("RC_LIDAR_LOG_ROOT", WORKSPACE / "logs" / "autostart")).resolve()
USER_DEFAULTS_PATH = Path(os.environ.get("RC_LIDAR_USER_DEFAULTS", AUTOSTART / "user_defaults.env")).expanduser().resolve()
INITIAL_POSE_ENV_DEFAULTS = {
    "RC_RELOCALIZATION_INITIAL_GUESS": "0",
    "RC_RELOCALIZATION_ASSUME_MAP_ORIGIN": "1",
    "RC_RELOCALIZATION_INITIAL_X": "0.0",
    "RC_RELOCALIZATION_INITIAL_Y": "0.0",
    "RC_RELOCALIZATION_INITIAL_Z": "0.0",
    "RC_RELOCALIZATION_INITIAL_ROLL": "0.0",
    "RC_RELOCALIZATION_INITIAL_PITCH": "0.0",
    "RC_RELOCALIZATION_INITIAL_YAW": "0.0",
}
INITIAL_POSE_ENV_KEYS = tuple(INITIAL_POSE_ENV_DEFAULTS.keys())


def default_config_path() -> Path:
    if os.environ.get("RC_LIDAR_UI_CONFIG"):
        return Path(os.environ["RC_LIDAR_UI_CONFIG"]).expanduser().resolve()
    if os.environ.get("XDG_CONFIG_HOME"):
        return Path(os.environ["XDG_CONFIG_HOME"]).expanduser() / "rc_lidar_r2_v2" / "ui_config.env"
    return Path.home() / ".config" / "rc_lidar_r2_v2" / "ui_config.env"


CONFIG_PATH = default_config_path()


def default_initial_pose_presets_path() -> Path:
    if os.environ.get("RC_LIDAR_INITIAL_POSE_PRESETS"):
        return Path(os.environ["RC_LIDAR_INITIAL_POSE_PRESETS"]).expanduser().resolve()
    if os.environ.get("XDG_CONFIG_HOME"):
        return Path(os.environ["XDG_CONFIG_HOME"]).expanduser() / "rc_lidar_r2_v2" / "initial_pose_presets.json"
    return Path.home() / ".config" / "rc_lidar_r2_v2" / "initial_pose_presets.json"


INITIAL_POSE_PRESETS_PATH = default_initial_pose_presets_path()


def now_iso() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def shell_quote(value: str) -> str:
    return "'" + value.replace("'", "'\\''") + "'"


def format_env_float(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError("数值必须是有限数字")
    text = f"{value:.10f}".rstrip("0").rstrip(".")
    if text in {"", "-0"}:
        text = "0"
    if "." not in text:
        text += ".0"
    return text


def format_can_restart_event(status: dict[str, str]) -> str:
    can_id = status.get("last_event_can_id", "0x777")
    payload = status.get("last_event_payload", "") or "<empty>"
    request_id = status.get("last_event_request_id", "")
    action = status.get("last_event_action", "")
    reason = status.get("last_event_reason", "")
    detail = status.get("last_event_detail", "")
    run_id = status.get("last_event_run_id", "")
    run_dir = status.get("last_event_run_dir", "")
    prefix = f"收到 CAN {can_id}: payload={payload}"
    request_text = f" request={request_id}" if request_id else ""

    if action == "pending":
        return f"{prefix}，CAN 重启处理中{request_text}"

    if action == "succeeded":
        run_text = f" run={run_id}" if run_id else ""
        dir_text = f" dir={run_dir}" if run_dir else ""
        return (
            f"{prefix}，CAN 重启成功：整条启动链路已拉起"
            f"（不检查坐标）{run_text}{dir_text}"
        )

    if action == "rejected":
        reason_text = {
            "debounce": "防抖时间内重复请求",
            "cooldown": "重启冷却时间未结束",
            "lock_busy": "已有重启正在进行",
            "dry_run": "当前为 dry-run",
        }.get(reason, reason or "请求未执行")
        detail_text = f"，{detail}" if detail else ""
        return f"{prefix}，CAN 重启被拒绝：{reason_text}{detail_text}{request_text}"

    if action == "failed":
        reason_text = {
            "launch_failed": "重启脚本无法启动",
            "start_failed": "启动脚本执行失败",
            "core_process_missing": "核心进程未全部存活",
            "missing_completion": "启动链路没有返回完成标志",
            "missing_result": "重启脚本没有返回结果",
            "invalid_result": "重启脚本返回了无效结果",
        }.get(reason, reason or "未知错误")
        detail_text = f"，{detail}" if detail else ""
        run_text = f" run={run_id}" if run_id else ""
        return f"{prefix}，CAN 重启失败：{reason_text}{detail_text}{run_text}{request_text}"

    detail_text = f"，{detail}" if detail else ""
    return f"{prefix}，{action or '已接收'}{detail_text}{request_text}"


def parse_float_value(raw: str, field: str) -> float:
    text = raw.strip()
    if not text:
        text = "0"
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError(f"{field} 必须是数字") from exc
    if not math.isfinite(value):
        raise ValueError(f"{field} 必须是有限数字")
    return value


def bool_env(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def parse_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, raw_value = line.split("=", 1)
        key = key.strip()
        if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", key):
            continue
        try:
            parts = shlex.split(raw_value, posix=True)
            values[key] = parts[0] if parts else ""
        except ValueError:
            values[key] = raw_value.strip().strip('"').strip("'")
    return values


def write_env_file(path: Path, values: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    boot_map = values.get("RC_UI_BOOT_MAP", "")
    lines = [
        "# Generated by RC LiDAR control UI. Edit with care.",
        f"RC_UI_BOOT_MAP={shell_quote(boot_map)}",
        f"RC_UI_LAST_SAVED_MAP={shell_quote(values.get('RC_UI_LAST_SAVED_MAP', ''))}",
        f"RC_UI_UPDATED_AT={shell_quote(now_iso())}",
    ]
    if boot_map:
        lines.extend(
            [
                f"RC_LIDAR_MAP_PATH={shell_quote(values.get('RC_LIDAR_MAP_PATH', boot_map))}",
                f"RC_LOCALIZATION_MAP_PATH={shell_quote(values.get('RC_LOCALIZATION_MAP_PATH', boot_map))}",
            ]
        )
    lines.append("")
    tmp = path.with_suffix(path.suffix + f".tmp.{os.getpid()}")
    tmp.write_text("\n".join(lines), encoding="utf-8")
    os.replace(tmp, path)


def read_config() -> dict[str, str]:
    return parse_env_file(CONFIG_PATH)


def update_config(*, boot_map: Path | None = None, last_saved_map: Path | None = None) -> dict[str, str]:
    values = read_config()
    if boot_map is not None:
        boot_value = str(boot_map)
        values["RC_UI_BOOT_MAP"] = boot_value
        values["RC_LIDAR_MAP_PATH"] = boot_value
        values["RC_LOCALIZATION_MAP_PATH"] = boot_value
    if last_saved_map is not None:
        values["RC_UI_LAST_SAVED_MAP"] = str(last_saved_map)
    write_env_file(CONFIG_PATH, values)
    return values


def user_initial_pose_env(x: float, y: float, z: float, yaw_deg: float) -> dict[str, str]:
    zero_pose = all(abs(value) < 1e-12 for value in (x, y, z, yaw_deg))
    yaw_rad = math.radians(yaw_deg)
    values = dict(INITIAL_POSE_ENV_DEFAULTS)
    if zero_pose:
        return values
    values.update(
        {
            "RC_RELOCALIZATION_INITIAL_GUESS": "1",
            "RC_RELOCALIZATION_ASSUME_MAP_ORIGIN": "0",
            "RC_RELOCALIZATION_INITIAL_X": format_env_float(x),
            "RC_RELOCALIZATION_INITIAL_Y": format_env_float(y),
            "RC_RELOCALIZATION_INITIAL_Z": format_env_float(z),
            "RC_RELOCALIZATION_INITIAL_ROLL": "0.0",
            "RC_RELOCALIZATION_INITIAL_PITCH": "0.0",
            "RC_RELOCALIZATION_INITIAL_YAW": format_env_float(yaw_rad),
        }
    )
    return values


def write_user_initial_pose(x: float, y: float, z: float, yaw_deg: float, path: Path = USER_DEFAULTS_PATH) -> dict[str, str]:
    values = user_initial_pose_env(x, y, z, yaw_deg)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Generated by RC LiDAR control UI. Do not commit local field defaults.",
        f"RC_LIDAR_USER_DEFAULTS_UPDATED_AT={shell_quote(now_iso())}",
    ]
    for key in INITIAL_POSE_ENV_KEYS:
        lines.append(f"{key}={shell_quote(values[key])}")
    lines.append("")
    tmp = path.with_suffix(path.suffix + f".tmp.{os.getpid()}")
    tmp.write_text("\n".join(lines), encoding="utf-8")
    os.replace(tmp, path)
    return values


def read_user_initial_pose_env(path: Path = USER_DEFAULTS_PATH) -> dict[str, str]:
    values = dict(INITIAL_POSE_ENV_DEFAULTS)
    values.update({key: value for key, value in parse_env_file(path).items() if key in INITIAL_POSE_ENV_KEYS})
    return values


def user_initial_pose_values(path: Path = USER_DEFAULTS_PATH) -> tuple[float, float, float, float, bool, bool]:
    values = read_user_initial_pose_env(path)
    x = parse_float_value(values.get("RC_RELOCALIZATION_INITIAL_X", "0.0"), "initial_x_m")
    y = parse_float_value(values.get("RC_RELOCALIZATION_INITIAL_Y", "0.0"), "initial_y_m")
    z = parse_float_value(values.get("RC_RELOCALIZATION_INITIAL_Z", "0.0"), "initial_z_m")
    yaw_rad = parse_float_value(values.get("RC_RELOCALIZATION_INITIAL_YAW", "0.0"), "initial_yaw_rad")
    enabled = bool_env(values.get("RC_RELOCALIZATION_INITIAL_GUESS"))
    assume_map_origin = bool_env(values.get("RC_RELOCALIZATION_ASSUME_MAP_ORIGIN"))
    return x, y, z, math.degrees(yaw_rad), enabled, assume_map_origin


def relocalization_defaults_env() -> dict[str, str]:
    return read_user_initial_pose_env()


def default_initial_pose_preset_name(x: float, y: float, z: float, yaw_deg: float) -> str:
    parts = [
        f"x{format_env_float(x)}",
        f"y{format_env_float(y)}",
    ]
    if abs(z) >= 1e-12:
        parts.append(f"z{format_env_float(z)}")
    parts.append(f"yaw{format_env_float(yaw_deg)}")
    return "_".join(parts)


def normalize_initial_pose_preset(raw: dict[str, Any]) -> dict[str, Any]:
    name = str(raw.get("name", "")).strip()
    if not name:
        raise ValueError("预设名称不能为空")
    preset_id = str(raw.get("id", "")).strip() or uuid.uuid4().hex
    created_at = str(raw.get("created_at", "")).strip() or now_iso()
    updated_at = str(raw.get("updated_at", "")).strip() or created_at
    return {
        "id": preset_id,
        "name": name,
        "x_m": parse_float_value(str(raw.get("x_m", "0.0")), "preset x_m"),
        "y_m": parse_float_value(str(raw.get("y_m", "0.0")), "preset y_m"),
        "z_m": parse_float_value(str(raw.get("z_m", "0.0")), "preset z_m"),
        "yaw_deg": parse_float_value(str(raw.get("yaw_deg", "0.0")), "preset yaw_deg"),
        "created_at": created_at,
        "updated_at": updated_at,
    }


def read_initial_pose_presets(path: Path = INITIAL_POSE_PRESETS_PATH) -> list[dict[str, Any]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"预设文件解析失败: {exc}") from exc
    raw_presets = data.get("presets", data) if isinstance(data, dict) else data
    if raw_presets is None:
        return []
    if not isinstance(raw_presets, list):
        raise ValueError("预设文件解析失败: presets 不是列表")
    return [normalize_initial_pose_preset(item) for item in raw_presets if isinstance(item, dict)]


def write_initial_pose_presets(presets: list[dict[str, Any]], path: Path = INITIAL_POSE_PRESETS_PATH) -> None:
    normalized = [normalize_initial_pose_preset(item) for item in presets]
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "version": 1,
        "updated_at": now_iso(),
        "presets": normalized,
    }
    tmp = path.with_suffix(path.suffix + f".tmp.{os.getpid()}")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def upsert_initial_pose_preset(
    presets: list[dict[str, Any]],
    name: str,
    x: float,
    y: float,
    z: float,
    yaw_deg: float,
) -> tuple[list[dict[str, Any]], dict[str, Any], bool]:
    normalized_name = name.strip()
    if not normalized_name:
        raise ValueError("预设名称不能为空")
    updated = [normalize_initial_pose_preset(item) for item in presets]
    timestamp = now_iso()
    for index, item in enumerate(updated):
        if item["name"] == normalized_name:
            preset = {
                "id": item["id"],
                "name": normalized_name,
                "x_m": x,
                "y_m": y,
                "z_m": z,
                "yaw_deg": yaw_deg,
                "created_at": item.get("created_at", timestamp),
                "updated_at": timestamp,
            }
            updated[index] = normalize_initial_pose_preset(preset)
            return updated, updated[index], True

    preset = normalize_initial_pose_preset(
        {
            "id": uuid.uuid4().hex,
            "name": normalized_name,
            "x_m": x,
            "y_m": y,
            "z_m": z,
            "yaw_deg": yaw_deg,
            "created_at": timestamp,
            "updated_at": timestamp,
        }
    )
    updated.append(preset)
    return updated, preset, False


def delete_initial_pose_preset(presets: list[dict[str, Any]], preset_id: str) -> list[dict[str, Any]]:
    return [normalize_initial_pose_preset(item) for item in presets if str(item.get("id", "")) != preset_id]


def list_pcd_maps() -> list[Path]:
    if not MAPS_DIR.exists():
        return []
    return sorted((p for p in MAPS_DIR.glob("*.pcd") if p.is_file()), key=lambda p: p.name.lower())


def normalize_map_name(name: str, maps_dir: Path = MAPS_DIR) -> Path:
    raw = name.strip()
    if not raw:
        raise ValueError("地图名不能为空")
    if "/" in raw or "\\" in raw or raw in {".", ".."} or raw.endswith("."):
        raise ValueError("地图名只能是文件名，不能包含路径")

    path = Path(raw)
    suffix = path.suffix
    if suffix:
        if suffix.lower() != ".pcd":
            raise ValueError("地图后缀必须是 .pcd，或者不写后缀")
        raw = raw[: -len(suffix)] + ".pcd"
    else:
        raw = raw + ".pcd"

    if not re.match(r"^[A-Za-z0-9_.-]+$", raw):
        raise ValueError("地图名只支持英文字母、数字、点、下划线和短横线")
    return (maps_dir / raw).resolve()


def map_is_valid(path: Path) -> bool:
    try:
        return path.is_file() and path.suffix.lower() == ".pcd" and path.stat().st_size > 0
    except OSError:
        return False


def read_start_summary() -> dict[str, str]:
    summary: dict[str, str] = {}
    summary_path = LOG_ROOT / "latest" / "start_summary.txt"
    if not summary_path.exists():
        return summary
    for line in summary_path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = re.match(r"^([A-Za-z0-9_]+):\s*(.*)$", line.strip())
        if match:
            summary[match.group(1)] = match.group(2).strip()
    return summary


def sanitize_slug(value: str, fallback: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("._-")
    return cleaned or fallback


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_can_pose(
    x: float,
    y: float,
    z: float,
    yaw: float,
    can_frame_mode: str = "normal",
    yaw_offset_rad: float = 0.0,
) -> tuple[float, float, float, float]:
    mode = can_frame_mode.lower()
    if mode not in CAN_FRAME_MODES:
        mode = "normal"
    if mode == "normal":
        tx, ty, tyaw = x, y, yaw
    elif mode == "yaw_180":
        tx, ty, tyaw = x, y, yaw + math.pi
    elif mode == "flip_xy_yaw_180":
        tx, ty, tyaw = -x, -y, yaw + math.pi
    elif mode == "flip_x":
        tx, ty, tyaw = -x, y, yaw
    elif mode == "flip_y":
        tx, ty, tyaw = x, -y, yaw
    elif mode == "swap_xy":
        tx, ty, tyaw = y, x, yaw
    elif mode == "swap_xy_flip_x":
        tx, ty, tyaw = -y, x, yaw
    elif mode == "swap_xy_flip_y":
        tx, ty, tyaw = y, -x, yaw
    else:
        tx, ty, tyaw = x, y, -yaw
    return float(tx), float(ty), float(z), normalize_angle(float(tyaw) + float(yaw_offset_rad))


def parse_pose_yaml(text: str) -> tuple[float, float, float, float]:
    section = ""
    position: dict[str, float] = {}
    orientation: dict[str, float] = {}
    direct_yaw: float | None = None
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped == "---":
            continue
        if stripped == "position:":
            section = "position"
            continue
        if stripped == "orientation:":
            section = "orientation"
            continue
        match = re.match(rf"^([xyzw]):\s*({number})\s*$", stripped)
        if match and section == "position" and match.group(1) in {"x", "y", "z"}:
            position[match.group(1)] = float(match.group(2))
            continue
        if match and section == "orientation" and match.group(1) in {"x", "y", "z", "w"}:
            orientation[match.group(1)] = float(match.group(2))
            continue
        yaw_match = re.match(rf"^yaw(?:_rad)?:\s*({number})\s*$", stripped)
        if yaw_match:
            direct_yaw = float(yaw_match.group(1))

    if not {"x", "y", "z"}.issubset(position):
        raise ValueError("没有从 topic 输出里解析到 position x/y/z")
    if {"x", "y", "z", "w"}.issubset(orientation):
        yaw = quat_to_yaw(orientation["x"], orientation["y"], orientation["z"], orientation["w"])
    elif direct_yaw is not None:
        yaw = direct_yaw
    else:
        raise ValueError("没有从 topic 输出里解析到 orientation 或 yaw")
    return position["x"], position["y"], position["z"], yaw


def parse_map_pose_output(text: str) -> tuple[float, float, float, float]:
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    required_keys = {
        "RC_RELOCALIZATION_INITIAL_X": "X m",
        "RC_RELOCALIZATION_INITIAL_Y": "Y m",
        "RC_RELOCALIZATION_INITIAL_Z": "Z m",
        "RC_RELOCALIZATION_INITIAL_YAW": "Yaw rad",
    }
    values: dict[str, float] = {}

    for line in text.splitlines():
        if "init_env:" not in line:
            continue
        for key, field in required_keys.items():
            match = re.search(rf"(?:^|\s){re.escape(key)}=({number})(?=\s|$)", line)
            if match:
                values[key] = parse_float_value(match.group(1), field)

    missing = [key for key in required_keys if key not in values]
    if missing:
        raise ValueError("没有从位置检测输出里解析到完整的 X/Y/Z/Yaw")

    return (
        values["RC_RELOCALIZATION_INITIAL_X"],
        values["RC_RELOCALIZATION_INITIAL_Y"],
        values["RC_RELOCALIZATION_INITIAL_Z"],
        math.degrees(values["RC_RELOCALIZATION_INITIAL_YAW"]),
    )


def run_command(args: list[str], env: dict[str, str] | None = None, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=str(WORKSPACE),
        env=env,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def parse_can_wakeup_status(output: str) -> dict[str, str]:
    status: dict[str, str] = {}
    for raw_line in output.splitlines():
        key, separator, value = raw_line.strip().partition("=")
        if separator and key:
            status[key] = value
    return status


def base_start_env() -> dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "RC_LIDAR_UI_AUTOSTART": "0",
            "RC_AUTOSTART_SKIP_PRESTOP": "1",
            "RC_AUTOSTART_HEADLESS": "1",
            "RC_AUTOSTART_PLOTJUGGLER": "1",
            "RC_AUTOSTART_CANFD": "1",
            "RC_AUTOSTART_CANFD_TX_MONITOR": "1",
            "RC_CAN_DEBUG_CSV": "1",
            "RC_CANFD_SERIAL_PRESENCE_CHECK_PERIOD_SEC": "0.5",
            "RC_MAP_LOCALIZER_LOCALIZE_PERIOD_SEC": "0.2",
            "RC_FASTLIO_FILTER_SIZE_SURF": "0.15",
            "RC_FASTLIO_FILTER_SIZE_MAP": "0.25",
            "RC_MAP_LOCALIZER_MAP_LEAF_SIZE": "0.10",
            "RC_MAP_LOCALIZER_SCAN_LEAF_SIZE": "0.10",
            "RC_FASTLIO_POINT_FILTER_NUM": "3",
            "RC_FASTLIO_RUNTIME_POS_LOG_ENABLE": "0",
            "RC_FASTLIO_DET_RANGE": "100",
            "FASTLIO_RVIZ": "false",
        }
    )
    return env


class LidarControlUi:
    def __init__(self) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.root = tk.Tk()
        self.root.title(APP_TITLE)
        self.root.geometry("860x700")
        self.root.minsize(780, 620)

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.map_paths: dict[str, Path] = {}
        self.initial_pose_presets: list[dict[str, Any]] = []
        self.pose_file: Path | None = None
        self.pose_file_run_id = ""
        self.pose_count = 0
        self.can_status_check_running = False
        self.closing = False
        self.last_can_event_id = ""

        self.map_var = tk.StringVar(value=NONE_MAP_LABEL)
        self.status_var = tk.StringVar(value="就绪")
        self.boot_map_var = tk.StringVar(value="")
        self.initial_x_var = tk.StringVar(value="0.0")
        self.initial_y_var = tk.StringVar(value="0.0")
        self.initial_z_var = tk.StringVar(value="0.0")
        self.initial_yaw_var = tk.StringVar(value="0.0")
        self.initial_pose_status_var = tk.StringVar(value="")
        self.can_listener_status_var = tk.StringVar(value="CAN 监听未运行")

        self.build_ui()
        self.load_initial_pose_fields()
        self.refresh_initial_pose_presets()
        self.refresh_maps(force_none=True)
        self.refresh_status()
        self.root.after(100, self.drain_events)
        self.root.after(0, self.refresh_can_listener_status_async)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def build_ui(self) -> None:
        ttk = self.ttk
        tk = self.tk

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(4, weight=1)

        top = ttk.Frame(self.root, padding=(14, 12, 14, 6))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="重定位地图").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.map_combo = ttk.Combobox(top, textvariable=self.map_var, state="readonly", width=36)
        self.map_combo.grid(row=0, column=1, sticky="ew")
        ttk.Button(top, text="刷新", command=self.refresh_maps).grid(row=0, column=2, padx=(8, 0))

        self.boot_label = ttk.Label(top, textvariable=self.boot_map_var)
        self.boot_label.grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))

        initial_pose = ttk.LabelFrame(self.root, text="重定位初始位姿", padding=(14, 8, 14, 8))
        initial_pose.grid(row=1, column=0, sticky="ew", padx=14, pady=(2, 6))
        initial_pose.columnconfigure(1, weight=1)
        initial_pose.columnconfigure(3, weight=1)
        initial_pose.columnconfigure(5, weight=1)
        initial_pose.columnconfigure(7, weight=1)

        ttk.Label(initial_pose, text="X m").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Entry(initial_pose, textvariable=self.initial_x_var, width=10).grid(row=0, column=1, sticky="ew", padx=(0, 10))
        ttk.Label(initial_pose, text="Y m").grid(row=0, column=2, sticky="w", padx=(0, 6))
        ttk.Entry(initial_pose, textvariable=self.initial_y_var, width=10).grid(row=0, column=3, sticky="ew", padx=(0, 10))
        ttk.Label(initial_pose, text="Z m").grid(row=0, column=4, sticky="w", padx=(0, 6))
        ttk.Entry(initial_pose, textvariable=self.initial_z_var, width=10).grid(row=0, column=5, sticky="ew", padx=(0, 10))
        ttk.Label(initial_pose, text="Yaw deg").grid(row=0, column=6, sticky="w", padx=(0, 6))
        ttk.Entry(initial_pose, textvariable=self.initial_yaw_var, width=10).grid(row=0, column=7, sticky="ew", padx=(0, 10))
        ttk.Button(initial_pose, text="设置初始位姿", command=self.on_set_initial_pose).grid(row=0, column=8, sticky="ew", padx=(0, 8))
        ttk.Button(initial_pose, text="重置为零", command=self.on_reset_initial_pose_fields).grid(row=0, column=9, sticky="ew")
        ttk.Label(initial_pose, textvariable=self.initial_pose_status_var).grid(row=1, column=0, columnspan=10, sticky="w", pady=(8, 0))

        preset_frame = ttk.Frame(initial_pose)
        preset_frame.grid(row=2, column=0, columnspan=10, sticky="ew", pady=(8, 0))
        preset_frame.columnconfigure(0, weight=1)
        self.preset_tree = ttk.Treeview(
            preset_frame,
            columns=("name", "x", "y", "z", "yaw"),
            show="headings",
            height=4,
            selectmode="browse",
        )
        self.preset_tree.heading("name", text="名称")
        self.preset_tree.heading("x", text="X")
        self.preset_tree.heading("y", text="Y")
        self.preset_tree.heading("z", text="Z")
        self.preset_tree.heading("yaw", text="Yaw")
        self.preset_tree.column("name", width=220, minwidth=120, anchor="w", stretch=True)
        self.preset_tree.column("x", width=90, minwidth=70, anchor="e", stretch=False)
        self.preset_tree.column("y", width=90, minwidth=70, anchor="e", stretch=False)
        self.preset_tree.column("z", width=90, minwidth=70, anchor="e", stretch=False)
        self.preset_tree.column("yaw", width=90, minwidth=70, anchor="e", stretch=False)
        self.preset_tree.grid(row=0, column=0, sticky="ew")
        preset_scroll = ttk.Scrollbar(preset_frame, orient="vertical", command=self.preset_tree.yview)
        preset_scroll.grid(row=0, column=1, sticky="ns")
        self.preset_tree.configure(yscrollcommand=preset_scroll.set)
        self.preset_tree.bind("<Double-1>", self.on_preset_double_click)

        preset_buttons = ttk.Frame(initial_pose)
        preset_buttons.grid(row=3, column=0, columnspan=10, sticky="ew", pady=(6, 0))
        for index in range(3):
            preset_buttons.columnconfigure(index, weight=1, uniform="preset_buttons")
        ttk.Button(preset_buttons, text="保存为预设", command=self.on_save_initial_pose_preset).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(preset_buttons, text="填入选中", command=self.on_apply_initial_pose_preset).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(preset_buttons, text="删除选中", command=self.on_delete_initial_pose_preset).grid(row=0, column=2, sticky="ew", padx=(6, 0))

        buttons = ttk.Frame(self.root, padding=(14, 6, 14, 6))
        buttons.grid(row=2, column=0, sticky="ew")
        for index in range(3):
            buttons.columnconfigure(index, weight=1, uniform="buttons")

        ttk.Button(buttons, text="启动建图", command=self.on_start_mapping).grid(row=0, column=0, sticky="ew", padx=(0, 6), pady=4)
        ttk.Button(buttons, text="保存地图", command=self.on_save_map).grid(row=0, column=1, sticky="ew", padx=6, pady=4)
        ttk.Button(buttons, text="启动重定位", command=self.on_start_relocalization).grid(row=0, column=2, sticky="ew", padx=(6, 0), pady=4)
        ttk.Button(buttons, text="保存坐标", command=self.on_save_pose).grid(row=1, column=0, sticky="ew", padx=(0, 6), pady=4)
        ttk.Button(buttons, text="停止雷达", command=self.on_stop_lidar).grid(row=1, column=1, sticky="ew", padx=6, pady=4)
        ttk.Button(buttons, text="设置开机/CAN地图", command=self.on_set_boot_map).grid(row=1, column=2, sticky="ew", padx=(6, 0), pady=4)
        ttk.Button(buttons, text="启动 CAN 监听", command=self.on_start_can_listener).grid(row=2, column=0, sticky="ew", padx=(0, 6), pady=4)
        ttk.Button(buttons, text="彻底停止全部", command=self.on_stop_everything).grid(row=2, column=1, sticky="ew", padx=6, pady=4)

        can_status = ttk.Frame(buttons)
        can_status.grid(row=2, column=2, sticky="ew", padx=(12, 0), pady=4)
        self.can_listener_indicator = tk.Canvas(
            can_status,
            width=18,
            height=18,
            highlightthickness=0,
            borderwidth=0,
            background=self.root.cget("background"),
        )
        self.can_listener_indicator.grid(row=0, column=0, padx=(0, 7))
        self.can_listener_indicator_item = self.can_listener_indicator.create_oval(
            2, 2, 16, 16, fill="#d32f2f", outline="#8b0000"
        )
        ttk.Label(can_status, textvariable=self.can_listener_status_var).grid(row=0, column=1, sticky="w")

        ttk.Button(buttons, text="启动位置检测", command=self.on_start_pose_monitor).grid(
            row=3, column=0, sticky="ew", padx=(0, 6), pady=4
        )
        ttk.Button(buttons, text="测定坐标存为预设", command=self.on_save_measured_pose_preset).grid(
            row=3, column=1, sticky="ew", padx=6, pady=4
        )

        status = ttk.Frame(self.root, padding=(14, 2, 14, 4))
        status.grid(row=3, column=0, sticky="ew")
        status.columnconfigure(0, weight=1)
        ttk.Label(status, textvariable=self.status_var).grid(row=0, column=0, sticky="ew")

        log_frame = ttk.Frame(self.root, padding=(14, 6, 14, 14))
        log_frame.grid(row=4, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=12, wrap="word", state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def log(self, message: str) -> None:
        self.events.put(("log", f"[{dt.datetime.now().strftime('%H:%M:%S')}] {message}"))

    def set_status(self, message: str) -> None:
        self.events.put(("status", message))

    def show_error(self, title: str, message: str) -> None:
        self.events.put(("error", (title, message)))

    def show_info(self, title: str, message: str) -> None:
        self.events.put(("info", (title, message)))

    def drain_events(self) -> None:
        from tkinter import messagebox

        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self.log_text.configure(state="normal")
                self.log_text.insert("end", str(payload) + "\n")
                self.log_text.see("end")
                self.log_text.configure(state="disabled")
            elif kind == "status":
                self.status_var.set(str(payload))
            elif kind == "refresh_maps":
                self.refresh_maps(force_none=False)
            elif kind == "error":
                title, message = payload  # type: ignore[misc]
                messagebox.showerror(str(title), str(message), parent=self.root)
            elif kind == "info":
                title, message = payload  # type: ignore[misc]
                messagebox.showinfo(str(title), str(message), parent=self.root)
            elif kind == "save_measured_pose_preset":
                x, y, z, yaw_deg = payload  # type: ignore[misc]
                self.root.after_idle(
                    self.save_initial_pose_preset_values,
                    float(x),
                    float(y),
                    float(z),
                    float(yaw_deg),
                )
            elif kind == "can_status":
                self.can_status_check_running = False
                self.apply_can_listener_status(payload)  # type: ignore[arg-type]
                if not self.closing:
                    self.root.after(1000, self.refresh_can_listener_status_async)
            elif kind == "can_status_refresh":
                self.refresh_can_listener_status_async()
        if not self.closing:
            self.root.after(100, self.drain_events)

    def apply_can_listener_status(self, status: dict[str, str]) -> None:
        self.apply_can_listener_event(status)
        if status.get("healthy") == "1":
            self.can_listener_indicator.itemconfigure(
                self.can_listener_indicator_item, fill="#2e7d32", outline="#145a20"
            )
            self.can_listener_status_var.set("CAN 监听正常")
            return

        self.can_listener_indicator.itemconfigure(
            self.can_listener_indicator_item, fill="#d32f2f", outline="#8b0000"
        )
        gateway_process = status.get("gateway_process", "stopped")
        gateway_socket = status.get("gateway_socket", "down")
        listener_process = status.get("listener_process", "stopped")
        listener_heartbeat = status.get("listener_heartbeat", "missing")
        if gateway_process == "stopped" and listener_process == "stopped":
            text = "CAN 监听未运行"
        elif gateway_process != "running":
            text = "异常：Gateway 未运行"
        elif gateway_socket != "ready":
            text = "异常：Gateway Socket 未就绪"
        elif listener_process != "running":
            text = "异常：监听器未运行"
        elif listener_heartbeat != "fresh":
            text = "异常：监听器心跳超时"
        else:
            text = "CAN 监听异常"
        self.can_listener_status_var.set(text)

    def apply_can_listener_event(self, status: dict[str, str]) -> None:
        event_id = status.get("last_event_event_id", "")
        if not event_id or event_id == self.last_can_event_id:
            return
        self.last_can_event_id = event_id
        self.log(format_can_restart_event(status))

    def refresh_can_listener_status_async(self) -> None:
        if self.closing or self.can_status_check_running:
            return
        self.can_status_check_running = True

        def check() -> None:
            try:
                result = run_command([str(CAN_WAKEUP_CONTROL), "status"], timeout=1.5)
                status = parse_can_wakeup_status(result.stdout)
                if not status:
                    status = {"healthy": "0"}
            except Exception:  # noqa: BLE001
                status = {"healthy": "0"}
            self.events.put(("can_status", status))

        threading.Thread(target=check, daemon=True).start()

    def request_can_listener_status_refresh(self) -> None:
        self.events.put(("can_status_refresh", None))

    def refresh_status(self) -> None:
        config = read_config()
        boot_map = config.get("RC_UI_BOOT_MAP", "")
        last_saved = config.get("RC_UI_LAST_SAVED_MAP", "")
        boot_text = Path(boot_map).name if boot_map else "未设置"
        saved_text = Path(last_saved).name if last_saved else "无"
        self.boot_map_var.set(f"开机/CAN地图: {boot_text}    最近保存: {saved_text}")
        self.refresh_initial_pose_status()

    def load_initial_pose_fields(self) -> None:
        try:
            x, y, z, yaw_deg, _, _ = user_initial_pose_values()
        except ValueError as exc:
            self.initial_pose_status_var.set(f"初始位姿配置错误: {exc}")
            return
        self.initial_x_var.set(format_env_float(x))
        self.initial_y_var.set(format_env_float(y))
        self.initial_z_var.set(format_env_float(z))
        self.initial_yaw_var.set(format_env_float(yaw_deg))
        self.refresh_initial_pose_status()

    def refresh_initial_pose_status(self) -> None:
        try:
            x, y, z, yaw_deg, enabled, assume_map_origin = user_initial_pose_values()
        except ValueError as exc:
            self.initial_pose_status_var.set(f"初始位姿配置错误: {exc}")
            return
        if enabled:
            self.initial_pose_status_var.set(
                "初始位姿: 已启用 "
                f"x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, yaw={yaw_deg:.3f} deg"
            )
        elif assume_map_origin:
            self.initial_pose_status_var.set("初始位姿: 未启用，按 map 原点启动")
        else:
            self.initial_pose_status_var.set("初始位姿: 未启用")

    def parse_initial_pose_entries(self) -> tuple[float, float, float, float]:
        return (
            parse_float_value(self.initial_x_var.get(), "X m"),
            parse_float_value(self.initial_y_var.get(), "Y m"),
            parse_float_value(self.initial_z_var.get(), "Z m"),
            parse_float_value(self.initial_yaw_var.get(), "Yaw deg"),
        )

    def refresh_initial_pose_presets(self, selected_id: str = "") -> None:
        try:
            presets = read_initial_pose_presets()
        except ValueError as exc:
            self.initial_pose_presets = []
            for item in self.preset_tree.get_children():
                self.preset_tree.delete(item)
            self.status_var.set(str(exc))
            self.log(str(exc))
            return

        self.initial_pose_presets = presets
        for item in self.preset_tree.get_children():
            self.preset_tree.delete(item)
        for preset in presets:
            preset_id = str(preset["id"])
            self.preset_tree.insert(
                "",
                "end",
                iid=preset_id,
                values=(
                    preset["name"],
                    format_env_float(float(preset["x_m"])),
                    format_env_float(float(preset["y_m"])),
                    format_env_float(float(preset["z_m"])),
                    format_env_float(float(preset["yaw_deg"])),
                ),
            )
        if selected_id and self.preset_tree.exists(selected_id):
            self.preset_tree.selection_set(selected_id)
            self.preset_tree.focus(selected_id)
            self.preset_tree.see(selected_id)

    def selected_initial_pose_preset(self) -> dict[str, Any]:
        selected = self.preset_tree.selection()
        if not selected:
            raise RuntimeError("请先选择一组预设")
        preset_id = selected[0]
        for preset in self.initial_pose_presets:
            if str(preset.get("id", "")) == preset_id:
                return preset
        raise RuntimeError("选中的预设不存在，请刷新后重试")

    def on_save_initial_pose_preset(self) -> None:
        from tkinter import messagebox

        try:
            x, y, z, yaw_deg = self.parse_initial_pose_entries()
        except ValueError as exc:
            messagebox.showerror("预设错误", str(exc), parent=self.root)
            return
        self.save_initial_pose_preset_values(x, y, z, yaw_deg)

    def save_initial_pose_preset_values(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        from tkinter import messagebox, simpledialog

        default_name = default_initial_pose_preset_name(x, y, z, yaw_deg)
        name = simpledialog.askstring("保存为预设", "输入预设名称", initialvalue=default_name, parent=self.root)
        if name is None:
            return
        name = name.strip()
        if not name:
            messagebox.showerror("预设错误", "预设名称不能为空", parent=self.root)
            return

        try:
            presets = read_initial_pose_presets()
        except ValueError as exc:
            messagebox.showerror("预设文件错误", str(exc), parent=self.root)
            return
        exists = any(str(preset.get("name", "")) == name for preset in presets)
        if exists and not messagebox.askyesno("覆盖预设", f"{name} 已存在，是否覆盖？", parent=self.root):
            return

        try:
            updated, preset, overwritten = upsert_initial_pose_preset(presets, name, x, y, z, yaw_deg)
            write_initial_pose_presets(updated)
        except (OSError, ValueError) as exc:
            messagebox.showerror("保存预设失败", str(exc), parent=self.root)
            return

        self.refresh_initial_pose_presets(str(preset["id"]))
        action = "覆盖" if overwritten else "保存"
        self.log(f"初始位姿预设已{action}: {name}")
        self.status_var.set(f"预设已{action}: {name}")

    def on_start_pose_monitor(self) -> None:
        from tkinter import messagebox

        if not MAP_POSE_PRINTER.is_file() or not os.access(MAP_POSE_PRINTER, os.X_OK):
            messagebox.showerror("启动位置检测", f"位置检测脚本不存在或不可执行:\n{MAP_POSE_PRINTER}", parent=self.root)
            return

        xfce = shutil.which("xfce4-terminal")
        gnome = shutil.which("gnome-terminal")
        xterm = shutil.which("xterm")
        title = "RC LiDAR 重定位位置检测"
        env = os.environ.copy()
        if xfce:
            command = [xfce, "--disable-server", f"--title={title}", "--command", str(MAP_POSE_PRINTER)]
        elif gnome:
            env.pop("GNOME_TERMINAL_SCREEN", None)
            command = [gnome, "--title", title, "--", str(MAP_POSE_PRINTER)]
        elif xterm:
            command = [xterm, "-T", title, "-e", str(MAP_POSE_PRINTER)]
        else:
            messagebox.showerror(
                "启动位置检测",
                "未找到可用终端，请安装 xfce4-terminal、gnome-terminal 或 xterm",
                parent=self.root,
            )
            return

        try:
            subprocess.Popen(
                command,
                cwd=str(WORKSPACE),
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
                start_new_session=True,
            )
        except OSError as exc:
            messagebox.showerror("启动位置检测", str(exc), parent=self.root)
            return

        self.log(f"已打开重定位位置检测终端: {MAP_POSE_PRINTER}")
        self.status_var.set("位置检测终端已打开")

    def on_save_measured_pose_preset(self) -> None:
        def task() -> None:
            self.log("读取当前重定位测定坐标: /map_lidar/pose")
            result = run_command([str(MAP_POSE_PRINTER), "--once"], timeout=5)
            output = (result.stdout + "\n" + result.stderr).strip()
            if result.returncode != 0:
                tail = output.splitlines()[-10:]
                raise RuntimeError("读取重定位测定坐标失败:\n" + "\n".join(tail))
            x, y, z, yaw_deg = parse_map_pose_output(output)
            self.log(f"已测定坐标: x={x:.6f}, y={y:.6f}, z={z:.6f}, yaw={yaw_deg:.3f}deg")
            self.events.put(("save_measured_pose_preset", (x, y, z, yaw_deg)))

        self.worker("测定坐标存为预设", task)

    def on_apply_initial_pose_preset(self) -> None:
        from tkinter import messagebox

        try:
            preset = self.selected_initial_pose_preset()
        except RuntimeError as exc:
            messagebox.showerror("填入预设", str(exc), parent=self.root)
            return

        self.initial_x_var.set(format_env_float(float(preset["x_m"])))
        self.initial_y_var.set(format_env_float(float(preset["y_m"])))
        self.initial_z_var.set(format_env_float(float(preset["z_m"])))
        self.initial_yaw_var.set(format_env_float(float(preset["yaw_deg"])))
        self.initial_pose_status_var.set(f"已填入预设: {preset['name']}，点击“设置初始位姿”后生效")
        self.log(f"已填入初始位姿预设: {preset['name']}")

    def on_delete_initial_pose_preset(self) -> None:
        from tkinter import messagebox

        try:
            preset = self.selected_initial_pose_preset()
        except RuntimeError as exc:
            messagebox.showerror("删除预设", str(exc), parent=self.root)
            return
        name = str(preset["name"])
        if not messagebox.askyesno("删除预设", f"删除 {name}？", parent=self.root):
            return

        try:
            presets = read_initial_pose_presets()
            write_initial_pose_presets(delete_initial_pose_preset(presets, str(preset["id"])))
        except (OSError, ValueError) as exc:
            messagebox.showerror("删除预设失败", str(exc), parent=self.root)
            return
        self.refresh_initial_pose_presets()
        self.log(f"初始位姿预设已删除: {name}")
        self.status_var.set(f"预设已删除: {name}")

    def on_preset_double_click(self, _event: object) -> None:
        if self.preset_tree.selection():
            self.on_apply_initial_pose_preset()

    def refresh_maps(self, force_none: bool = False) -> None:
        maps = list_pcd_maps()
        self.map_paths = {p.name: p for p in maps}
        values = [NONE_MAP_LABEL] + [p.name for p in maps]
        self.map_combo.configure(values=values)
        if force_none or self.map_var.get() not in values:
            self.map_var.set(NONE_MAP_LABEL)
        self.refresh_status()

    def worker(self, name: str, func: Callable[[], None]) -> None:
        def run() -> None:
            self.set_status(f"{name}...")
            try:
                func()
            except subprocess.TimeoutExpired as exc:
                self.show_error(name, f"命令超时: {' '.join(exc.cmd) if isinstance(exc.cmd, list) else exc.cmd}")
                self.log(f"{name} 超时")
            except Exception as exc:  # noqa: BLE001
                self.show_error(name, str(exc))
                self.log(f"{name} 失败: {exc}")
            finally:
                self.set_status("就绪")

        threading.Thread(target=run, daemon=True).start()

    def run_stop_all(self, keep_can_wakeup: bool = True) -> None:
        stop_script = AUTOSTART / "stop_all.sh"
        args = [str(stop_script), "--quiet"]
        if keep_can_wakeup:
            args.append("--keep-can-wakeup")
        result = run_command(args, timeout=20)
        if result.returncode != 0:
            tail = (result.stderr or result.stdout).strip().splitlines()[-6:]
            raise RuntimeError("stop_all.sh 失败:\n" + "\n".join(tail))

    def run_start_all(self, env: dict[str, str]) -> None:
        start_script = AUTOSTART / "start_all_terminals.sh"
        result = run_command([str(start_script)], env=env, timeout=180)
        output = (result.stdout + "\n" + result.stderr).strip()
        if result.returncode != 0:
            tail = output.splitlines()[-12:]
            raise RuntimeError("start_all_terminals.sh 失败:\n" + "\n".join(tail))
        for line in output.splitlines()[-8:]:
            if line.strip():
                self.log(line.strip())

    def on_start_mapping(self) -> None:
        from tkinter import messagebox, simpledialog

        name = simpledialog.askstring("启动建图", "输入地图名，例如 map 或 map.pcd", parent=self.root)
        if name is None:
            return
        try:
            map_path = normalize_map_name(name)
        except ValueError as exc:
            messagebox.showerror("地图名错误", str(exc), parent=self.root)
            return
        if map_path.exists() and not messagebox.askyesno("覆盖地图", f"{map_path.name} 已存在，是否覆盖？", parent=self.root):
            return

        def task() -> None:
            MAPS_DIR.mkdir(parents=True, exist_ok=True)
            self.log(f"清理旧雷达进程，准备建图: {map_path}")
            self.run_stop_all()
            env = base_start_env()
            env.update(
                {
                    "RUN_REASON": "ui_mapping",
                    "RC_LIDAR_MODE": "mapping",
                    "RC_FASTLIO_MAP_SAVE_PATH": str(map_path),
                }
            )
            self.run_start_all(env)
            self.log(f"建图已启动，保存目标: {map_path}")

        self.worker("启动建图", task)

    def on_save_map(self) -> None:
        def task() -> None:
            save_script = AUTOSTART / "save_fast_lio_map.sh"
            self.log("调用 FAST-LIO 保存地图...")
            result = run_command([str(save_script)], timeout=90)
            output = (result.stdout + "\n" + result.stderr).strip()
            if result.returncode != 0:
                tail = output.splitlines()[-12:]
                raise RuntimeError("保存地图失败:\n" + "\n".join(tail))
            saved_path = self.parse_saved_map_path(output)
            if saved_path is not None:
                update_config(last_saved_map=saved_path)
                self.log(f"地图已保存: {saved_path}")
                self.events.put(("refresh_maps", None))
            else:
                self.log("保存地图完成，但没有解析到保存路径")
            for line in output.splitlines()[-8:]:
                if line.strip():
                    self.log(line.strip())

        self.worker("保存地图", task)

    def parse_saved_map_path(self, output: str) -> Path | None:
        patterns = [
            r"Saved:\s*(.+)$",
            r"Map saved to\s+(.+?)(?:'|\")?$",
            r"Requested target was:\s*(.+)$",
            r"target path from FAST-LIO map_file_path:\s*(.+)$",
        ]
        for line in reversed(output.splitlines()):
            for pattern in patterns:
                match = re.search(pattern, line)
                if match:
                    raw = match.group(1).strip().strip("'\"")
                    if raw:
                        return Path(raw).expanduser().resolve()
        return None

    def choose_relocalization_map(self) -> Path:
        selected = self.map_var.get()
        if selected != NONE_MAP_LABEL and selected in self.map_paths:
            path = self.map_paths[selected]
            if map_is_valid(path):
                return path
            raise RuntimeError(f"选择的地图不存在或为空: {path}")

        config = read_config()
        candidates = [
            config.get("RC_UI_LAST_SAVED_MAP", ""),
            config.get("RC_UI_BOOT_MAP", ""),
            os.environ.get("RC_LIDAR_MAP_PATH", ""),
            str(MAPS_DIR / "lan1.pcd"),
        ]
        for candidate in candidates:
            if not candidate:
                continue
            path = Path(candidate).expanduser().resolve()
            if map_is_valid(path):
                return path
        raise RuntimeError("没有可用地图：请选择下拉栏里的 .pcd，或先保存一次建图")

    def on_set_initial_pose(self) -> None:
        from tkinter import messagebox

        try:
            x, y, z, yaw_deg = self.parse_initial_pose_entries()
            values = write_user_initial_pose(x, y, z, yaw_deg)
        except ValueError as exc:
            messagebox.showerror("初始位姿错误", str(exc), parent=self.root)
            return
        except OSError as exc:
            messagebox.showerror("保存初始位姿失败", str(exc), parent=self.root)
            return

        self.load_initial_pose_fields()
        enabled = bool_env(values.get("RC_RELOCALIZATION_INITIAL_GUESS"))
        yaw_rad = values.get("RC_RELOCALIZATION_INITIAL_YAW", "0.0")
        if enabled:
            self.log(f"初始位姿已设置: x={x:.4f}, y={y:.4f}, z={z:.4f}, yaw={yaw_deg:.3f}deg ({yaw_rad}rad)")
            messagebox.showinfo("初始位姿", "已启用重定位初始位姿", parent=self.root)
        else:
            self.log("初始位姿已清零: 后续重定位按 map 原点启动")
            messagebox.showinfo("初始位姿", "已恢复为按 map 原点启动", parent=self.root)

    def on_reset_initial_pose_fields(self) -> None:
        self.initial_x_var.set("0.0")
        self.initial_y_var.set("0.0")
        self.initial_z_var.set("0.0")
        self.initial_yaw_var.set("0.0")
        self.initial_pose_status_var.set("输入框已清零，点击“设置初始位姿”后生效")

    def on_start_relocalization(self) -> None:
        from tkinter import messagebox

        try:
            map_path = self.choose_relocalization_map()
            user_initial_pose_values()
        except RuntimeError as exc:
            messagebox.showerror("启动重定位", str(exc), parent=self.root)
            return
        except ValueError as exc:
            messagebox.showerror("初始位姿配置错误", str(exc), parent=self.root)
            return

        def task() -> None:
            self.pose_file = None
            self.pose_file_run_id = ""
            self.pose_count = 0
            self.log(f"清理旧雷达进程，准备重定位: {map_path}")
            self.run_stop_all()
            env = base_start_env()
            env.update(relocalization_defaults_env())
            env.update(
                {
                    "RUN_REASON": "ui_relocalization",
                    "RC_LIDAR_MODE": "relocalization",
                    "RC_LIDAR_MAP_PATH": str(map_path),
                    "RC_LOCALIZATION_MAP_PATH": str(map_path),
                    "RC_MATCH_ORIGIN_ENABLE": "0",
                }
            )
            self.run_start_all(env)
            self.log(f"重定位已启动: {map_path}")

        self.worker("启动重定位", task)

    def on_set_boot_map(self) -> None:
        from tkinter import messagebox

        selected = self.map_var.get()
        if selected == NONE_MAP_LABEL or selected not in self.map_paths:
            messagebox.showerror("设置地图", "请先在下拉栏选择一张 .pcd 地图", parent=self.root)
            return
        map_path = self.map_paths[selected]
        if not map_is_valid(map_path):
            messagebox.showerror("设置地图", f"地图不存在或为空: {map_path}", parent=self.root)
            return
        update_config(boot_map=map_path)
        self.refresh_status()
        self.log(f"开机/CAN 重启地图已设置: {map_path}")
        messagebox.showinfo("设置地图", f"已设置为:\n{map_path}", parent=self.root)

    def on_save_pose(self) -> None:
        def task() -> None:
            summary = read_start_summary()
            mode = summary.get("runtime_mode", "")
            if mode and mode != "relocalization":
                raise RuntimeError(f"当前最新运行模式是 {mode}，保存坐标请先启动重定位")
            topic = summary.get("can_pose_topic") or "/robot/pose_continuous"
            self.log(f"读取当前 CAN pose topic: {topic}")
            result = run_command(["ros2", "topic", "echo", "--once", topic], timeout=5)
            output = (result.stdout + "\n" + result.stderr).strip()
            if result.returncode != 0:
                tail = output.splitlines()[-10:]
                raise RuntimeError("读取 pose topic 失败:\n" + "\n".join(tail))
            input_x, input_y, input_z, input_yaw = parse_pose_yaml(output)
            can_mode = os.environ.get("RC_CAN_FRAME_MODE", os.environ.get("CAN_FRAME_MODE", "normal"))
            yaw_offset = float(os.environ.get("RC_CAN_YAW_OFFSET_RAD", os.environ.get("CAN_YAW_OFFSET_RAD", "0.0")))
            x, y, z, yaw = transform_can_pose(input_x, input_y, input_z, input_yaw, can_mode, yaw_offset)
            pose_file = self.ensure_pose_file(summary, topic)
            self.pose_count += 1
            yaw_deg = math.degrees(yaw)
            line = (
                f"| {self.pose_count} | {now_iso()} | `{topic}` | "
                f"{x:.6f} | {y:.6f} | {z:.6f} | {yaw:.6f} | {yaw_deg:.3f} | |\n"
            )
            with pose_file.open("a", encoding="utf-8") as handle:
                handle.write(line)
                handle.flush()
                os.fsync(handle.fileno())
            self.log(f"已保存坐标 #{self.pose_count}: x={x:.4f}, y={y:.4f}, yaw={yaw:.4f}")

        self.worker("保存坐标", task)

    def ensure_pose_file(self, summary: dict[str, str], topic: str) -> Path:
        run_id = summary.get("run_id") or "manual_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        map_path = summary.get("map_path") or summary.get("localization_map") or read_config().get("RC_UI_BOOT_MAP", "")
        map_name = Path(map_path).stem if map_path else "no_map"
        if self.pose_file is not None and self.pose_file_run_id == run_id:
            return self.pose_file

        POSES_DIR.mkdir(parents=True, exist_ok=True)
        file_name = f"{sanitize_slug(run_id, 'run')}_{sanitize_slug(map_name, 'map')}.md"
        pose_file = POSES_DIR / file_name
        needs_header = not pose_file.exists()
        if needs_header:
            with pose_file.open("w", encoding="utf-8") as handle:
                handle.write("# RC LiDAR Saved Poses\n\n")
                handle.write(f"- run_id: `{run_id}`\n")
                handle.write(f"- map: `{map_path or 'unknown'}`\n")
                handle.write(f"- topic: `{topic}`\n")
                handle.write(f"- created_at: `{now_iso()}`\n\n")
                handle.write("| idx | time | topic | x | y | z | yaw_rad | yaw_deg | note |\n")
                handle.write("| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |\n")
        self.pose_file = pose_file
        self.pose_file_run_id = run_id
        self.pose_count = self.count_existing_pose_rows(pose_file)
        self.log(f"坐标记录文件: {pose_file}")
        return pose_file

    def count_existing_pose_rows(self, path: Path) -> int:
        if not path.exists():
            return 0
        count = 0
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if re.match(r"^\|\s*\d+\s*\|", line):
                count += 1
        return count

    def append_pose_footer(self) -> None:
        if self.pose_file is None:
            return
        with self.pose_file.open("a", encoding="utf-8") as handle:
            handle.write(f"\n- stopped_at: `{now_iso()}`\n")
            handle.flush()
            os.fsync(handle.fileno())
        self.log(f"坐标记录已收尾: {self.pose_file}")
        self.pose_file = None
        self.pose_file_run_id = ""

    def on_stop_lidar(self) -> None:
        def task() -> None:
            self.log("停止雷达进程，保留 UI 和 CAN 唤醒监听")
            self.run_stop_all(keep_can_wakeup=True)
            self.append_pose_footer()
            self.log("雷达已停止")
            self.request_can_listener_status_refresh()

        self.worker("停止雷达", task)

    def on_start_can_listener(self) -> None:
        def task() -> None:
            self.log("启动后台 CAN Gateway 和 0x777 监听器")
            result = run_command([str(CAN_WAKEUP_CONTROL), "start"], timeout=15)
            output = (result.stdout + "\n" + result.stderr).strip()
            if result.returncode != 0:
                raise RuntimeError("CAN 监听启动失败:\n" + "\n".join(output.splitlines()[-10:]))
            self.log("CAN Gateway 和 0x777 监听器已运行")
            self.request_can_listener_status_refresh()

        self.worker("启动 CAN 监听", task)

    def on_stop_everything(self) -> None:
        def task() -> None:
            self.log("彻底停止雷达、CAN 监听和 Gateway，保留 UI")
            self.run_stop_all(keep_can_wakeup=False)
            self.append_pose_footer()
            self.log("全部雷达相关进程已停止")
            self.request_can_listener_status_refresh()

        self.worker("彻底停止全部", task)

    def on_close(self) -> None:
        self.closing = True
        self.append_pose_footer()
        self.root.destroy()

    def run(self) -> None:
        self.log(f"UI 已启动，地图配置: {CONFIG_PATH}")
        self.log(f"初始位姿默认值: {USER_DEFAULTS_PATH}")
        self.log(f"初始位姿预设: {INITIAL_POSE_PRESETS_PATH}")
        self.root.mainloop()


def self_test() -> None:
    can_status = parse_can_wakeup_status(
        "healthy=1\ngateway_process=running\nlistener_process=running\n"
    )
    assert can_status["healthy"] == "1"
    assert can_status["gateway_process"] == "running"
    test_dir = Path("/tmp/rc_lidar_ui_test_maps")
    assert normalize_map_name("map", test_dir) == test_dir / "map.pcd"
    assert normalize_map_name("map.pcd", test_dir) == test_dir / "map.pcd"
    try:
        normalize_map_name("map.txt", test_dir)
    except ValueError:
        pass
    else:
        raise AssertionError("map.txt should be rejected")
    x, y, z, yaw = parse_pose_yaml(
        """
pose:
  position:
    x: 1.0
    y: 2.0
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
---
"""
    )
    assert (x, y, z) == (1.0, 2.0, 0.5)
    assert abs(yaw) < 1e-9
    measured_x, measured_y, measured_z, measured_yaw_deg = parse_map_pose_output(
        """
[map_pose] frame=map child=/map_lidar/pose stamp=1.000000000 age=0.100s
  xyz: x=2.253850 y=-1.526798 z=0.074816
  rpy_deg: roll=-0.428 pitch=1.072 yaw=-7.183
  rpy_rad: roll=-0.007474 pitch=0.018718 yaw=-0.125366
  init_env: RC_RELOCALIZATION_INITIAL_GUESS=1 RC_RELOCALIZATION_INITIAL_X=2.253850 RC_RELOCALIZATION_INITIAL_Y=-1.526798 RC_RELOCALIZATION_INITIAL_Z=7.4816e-02 RC_RELOCALIZATION_INITIAL_ROLL=-0.007474 RC_RELOCALIZATION_INITIAL_PITCH=0.018718 RC_RELOCALIZATION_INITIAL_YAW=-0.125366
"""
    )
    assert measured_x == 2.25385
    assert measured_y == -1.526798
    assert measured_z == 0.074816
    assert abs(measured_yaw_deg - math.degrees(-0.125366)) < 1e-9
    for invalid_map_pose_output in ("", "init_env: RC_RELOCALIZATION_INITIAL_X=1.0"):
        try:
            parse_map_pose_output(invalid_map_pose_output)
        except ValueError:
            pass
        else:
            raise AssertionError("incomplete map pose output should fail")
    tx, ty, _, tyaw = transform_can_pose(1.0, 2.0, 0.0, 0.25, "flip_xy_yaw_180")
    assert tx == -1.0 and ty == -2.0
    assert abs(normalize_angle(tyaw - (0.25 + math.pi))) < 1e-9
    with tempfile.TemporaryDirectory() as tmp_dir:
        defaults_path = Path(tmp_dir) / "user_defaults.env"
        values = write_user_initial_pose(0.0, -11.0, 0.0, 0.0, defaults_path)
        assert values["RC_RELOCALIZATION_INITIAL_GUESS"] == "1"
        assert values["RC_RELOCALIZATION_ASSUME_MAP_ORIGIN"] == "0"
        assert values["RC_RELOCALIZATION_INITIAL_Y"] == "-11.0"
        x, y, z, yaw_deg, enabled, assume = user_initial_pose_values(defaults_path)
        assert (x, y, z) == (0.0, -11.0, 0.0)
        assert abs(yaw_deg) < 1e-9
        assert enabled and not assume
        values = write_user_initial_pose(0.0, 0.0, 0.0, 90.0, defaults_path)
        assert values["RC_RELOCALIZATION_INITIAL_YAW"] == "1.5707963268"
        values = write_user_initial_pose(0.0, 0.0, 0.0, 0.0, defaults_path)
        assert values["RC_RELOCALIZATION_INITIAL_GUESS"] == "0"
        assert values["RC_RELOCALIZATION_ASSUME_MAP_ORIGIN"] == "1"
    with tempfile.TemporaryDirectory() as tmp_dir:
        presets_path = Path(tmp_dir) / "initial_pose_presets.json"
        defaults_path = Path(tmp_dir) / "user_defaults.env"
        write_user_initial_pose(1.0, 2.0, 3.0, 4.0, defaults_path)
        assert read_initial_pose_presets(presets_path) == []
        presets, preset, overwritten = upsert_initial_pose_preset([], "start_a", 0.0, -11.0, 0.0, 0.0)
        assert not overwritten
        write_initial_pose_presets(presets, presets_path)
        loaded = read_initial_pose_presets(presets_path)
        assert len(loaded) == 1
        assert loaded[0]["id"] == preset["id"]
        assert loaded[0]["name"] == "start_a"
        assert loaded[0]["y_m"] == -11.0
        presets, preset, overwritten = upsert_initial_pose_preset(loaded, "start_a", 1.0, 2.0, 0.0, 90.0)
        assert overwritten
        write_initial_pose_presets(presets, presets_path)
        loaded = read_initial_pose_presets(presets_path)
        assert len(loaded) == 1
        assert loaded[0]["id"] == preset["id"]
        assert loaded[0]["yaw_deg"] == 90.0
        write_initial_pose_presets(delete_initial_pose_preset(loaded, str(preset["id"])), presets_path)
        assert read_initial_pose_presets(presets_path) == []
        unchanged = read_user_initial_pose_env(defaults_path)
        assert unchanged["RC_RELOCALIZATION_INITIAL_X"] == "1.0"
        presets_path.write_text("{bad json", encoding="utf-8")
        try:
            read_initial_pose_presets(presets_path)
        except ValueError:
            pass
        else:
            raise AssertionError("corrupt preset json should fail")


def main() -> int:
    if "--self-test" in sys.argv:
        self_test()
        print("lidar_control_ui self-test ok")
        return 0

    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("[lidar_ui] no DISPLAY/WAYLAND_DISPLAY; UI skipped")
        return 0

    try:
        LidarControlUi().run()
    except Exception as exc:  # noqa: BLE001
        print(f"[lidar_ui] ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
