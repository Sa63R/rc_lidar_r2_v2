#!/usr/bin/env python3
from pathlib import Path
import os
import subprocess
import tempfile
import unittest


AUTOSTART = Path(__file__).resolve().parents[3] / "autostart"
CONFIG_SCRIPT = AUTOSTART / "lidar_ui_config.sh"
POSE_KEYS = (
    "RC_RELOCALIZATION_INITIAL_GUESS",
    "RC_RELOCALIZATION_ASSUME_MAP_ORIGIN",
    "RC_RELOCALIZATION_INITIAL_X",
    "RC_RELOCALIZATION_INITIAL_Y",
    "RC_RELOCALIZATION_INITIAL_Z",
    "RC_RELOCALIZATION_INITIAL_ROLL",
    "RC_RELOCALIZATION_INITIAL_PITCH",
    "RC_RELOCALIZATION_INITIAL_YAW",
)


class LidarUserDefaultsTest(unittest.TestCase):
    def run_loader(
        self,
        defaults: dict[str, str],
        inherited: dict[str, str],
        mode: str = "preserve_existing",
    ) -> dict[str, str]:
        with tempfile.TemporaryDirectory() as tmp_dir:
            defaults_path = Path(tmp_dir) / "user_defaults.env"
            defaults_path.write_text(
                "\n".join(f"{key}='{value}'" for key, value in defaults.items()) + "\n",
                encoding="utf-8",
            )
            command = f"""
source {CONFIG_SCRIPT!s}
load_lidar_user_defaults {mode}
for key in {' '.join(POSE_KEYS)}; do
  if [ "${{!key+x}}" = x ]; then
    printf '%s=%s\\n' "$key" "${{!key}}"
  else
    printf '%s=<unset>\\n' "$key"
  fi
done
"""
            env = os.environ.copy()
            for key in POSE_KEYS:
                env.pop(key, None)
            env.update(inherited)
            env["RC_LIDAR_USER_DEFAULTS"] = str(defaults_path)
            result = subprocess.run(
                ["bash", "-c", command],
                env=env,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            return dict(line.split("=", 1) for line in result.stdout.splitlines())

    def test_default_mode_preserves_explicit_environment(self):
        values = self.run_loader(
            defaults={
                "RC_RELOCALIZATION_INITIAL_GUESS": "1",
                "RC_RELOCALIZATION_INITIAL_X": "11.130972",
            },
            inherited={
                "RC_RELOCALIZATION_INITIAL_GUESS": "0",
                "RC_RELOCALIZATION_INITIAL_X": "7.195",
            },
        )

        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_GUESS"], "0")
        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_X"], "7.195")

    def test_override_mode_replaces_stale_listener_environment(self):
        values = self.run_loader(
            defaults={
                "RC_RELOCALIZATION_INITIAL_GUESS": "1",
                "RC_RELOCALIZATION_ASSUME_MAP_ORIGIN": "0",
                "RC_RELOCALIZATION_INITIAL_X": "11.130972",
                "RC_RELOCALIZATION_INITIAL_Y": "-3.688902",
                "RC_RELOCALIZATION_INITIAL_Z": "0.179276",
                "RC_RELOCALIZATION_INITIAL_ROLL": "0.0",
                "RC_RELOCALIZATION_INITIAL_PITCH": "0.0",
                "RC_RELOCALIZATION_INITIAL_YAW": "-0.0595855407",
            },
            inherited={key: "stale" for key in POSE_KEYS},
            mode="override_existing",
        )

        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_GUESS"], "1")
        self.assertEqual(values["RC_RELOCALIZATION_ASSUME_MAP_ORIGIN"], "0")
        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_X"], "11.130972")
        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_Y"], "-3.688902")
        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_Z"], "0.179276")
        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_YAW"], "-0.0595855407")

    def test_override_mode_clears_keys_omitted_from_persisted_file(self):
        values = self.run_loader(
            defaults={"RC_RELOCALIZATION_INITIAL_GUESS": "1"},
            inherited={"RC_RELOCALIZATION_INITIAL_X": "7.195"},
            mode="override_existing",
        )

        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_GUESS"], "1")
        self.assertEqual(values["RC_RELOCALIZATION_INITIAL_X"], "<unset>")


if __name__ == "__main__":
    unittest.main()
