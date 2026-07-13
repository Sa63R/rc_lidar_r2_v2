#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest


AUTOSTART = Path(__file__).resolve().parents[3] / "autostart"
SPEC = importlib.util.spec_from_file_location(
    "lidar_control_ui", AUTOSTART / "lidar_control_ui.py"
)
assert SPEC is not None and SPEC.loader is not None
ui_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ui_module)


class CanRestartEventFormattingTest(unittest.TestCase):
    def base_status(self, action: str, reason: str = "") -> dict[str, str]:
        return {
            "last_event_can_id": "0x777",
            "last_event_payload": "A55A01FE00000000",
            "last_event_request_id": "request-1",
            "last_event_action": action,
            "last_event_reason": reason,
        }

    def test_pending_message(self):
        message = ui_module.format_can_restart_event(self.base_status("pending", "received"))
        self.assertIn("CAN 重启处理中", message)
        self.assertNotIn("重启成功", message)

    def test_success_requires_no_coordinate_fields(self):
        status = self.base_status("succeeded", "stack_started")
        status.update(
            {
                "last_event_run_id": "run_test",
                "last_event_run_dir": "/tmp/run_test",
            }
        )
        message = ui_module.format_can_restart_event(status)
        self.assertIn("CAN 重启成功：整条启动链路已拉起", message)
        self.assertIn("不检查坐标", message)
        self.assertIn("run=run_test", message)

    def test_rejection_reasons_are_explicit(self):
        expected = {
            "debounce": "防抖时间内重复请求",
            "cooldown": "重启冷却时间未结束",
            "lock_busy": "已有重启正在进行",
        }
        for reason, reason_text in expected.items():
            with self.subTest(reason=reason):
                message = ui_module.format_can_restart_event(
                    self.base_status("rejected", reason)
                )
                self.assertIn("CAN 重启被拒绝", message)
                self.assertIn(reason_text, message)

    def test_core_process_failure_message(self):
        status = self.base_status("failed", "core_process_missing")
        status["last_event_detail"] = "missing_or_exited=FASTLIO"
        message = ui_module.format_can_restart_event(status)
        self.assertIn("CAN 重启失败：核心进程未全部存活", message)
        self.assertIn("FASTLIO", message)


if __name__ == "__main__":
    unittest.main()
