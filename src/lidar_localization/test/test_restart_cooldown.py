#!/usr/bin/env python3
from pathlib import Path
import subprocess
import unittest


COOLDOWN_SCRIPT = (
    Path(__file__).resolve().parents[3] / "autostart" / "restart_cooldown.sh"
)


def cooldown_active(now: int, last: int, interval: int) -> bool:
    command = (
        f'source "{COOLDOWN_SCRIPT}"; '
        f'restart_cooldown_active "{now}" "{last}" "{interval}"'
    )
    return subprocess.run(["/bin/bash", "-c", command], check=False).returncode == 0


class RestartCooldownTest(unittest.TestCase):
    def test_same_boot_recent_restart_is_in_cooldown(self):
        self.assertTrue(cooldown_active(100, 95, 15))

    def test_interval_boundary_is_not_in_cooldown(self):
        self.assertFalse(cooldown_active(110, 95, 15))

    def test_previous_boot_uptime_is_not_in_cooldown(self):
        self.assertFalse(cooldown_active(2361, 6838, 15))

    def test_invalid_saved_value_is_not_in_cooldown(self):
        command = (
            f'source "{COOLDOWN_SCRIPT}"; '
            'restart_cooldown_active "100" "invalid" "15"'
        )
        result = subprocess.run(["/bin/bash", "-c", command], check=False)
        self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
