#!/usr/bin/env python3
"""Read-only host inventory; does not install packages or connect to hardware."""
import argparse
from datetime import datetime, timezone
import glob
import json
from pathlib import Path
import platform
import shutil
import subprocess


def ros_python_check(setup):
    script = 'set +u; source "$1"; /usr/bin/python3 -c "$2"'
    code = 'import rclpy, numpy, yaml, tkinter; print("imports_ok")'
    completed = subprocess.run(['bash', '-c', script, 'probe', setup, code],
                               capture_output=True, text=True, timeout=20)
    return {'ok': completed.returncode == 0,
            'detail': (completed.stdout+completed.stderr)[-1500:]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    if args.out and args.out.exists():
        parser.error('refusing overwrite')
    release = Path(__file__).resolve().parents[1]
    os_release = {}
    if Path('/etc/os-release').exists():
        for line in Path('/etc/os-release').read_text().splitlines():
            key, _, value = line.partition('=')
            os_release[key] = value.strip('"')
    setups = sorted(glob.glob('/opt/ros/*/setup.bash'))
    report = {'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'os': os_release.get('PRETTY_NAME', platform.system()), 'architecture': platform.machine(),
        'system_python': platform.python_version(), 'ros_setups': setups,
        'colcon': shutil.which('colcon'), 'sdk2_shared_library_present': bool(
            glob.glob('/usr/local/lib/liblivox_lidar_sdk_shared.so*')),
        'sdk2_header_present': Path('/usr/local/include/livox_lidar_api.h').exists(),
        'app_built_here': (release/'install/setup.bash').is_file(),
        'target_configuration_file': str(release/'config/deploy.env'),
        'tested_baseline': 'Ubuntu 24.04 / ROS 2 Jazzy; other environments need adaptation',
        'hardware_contacted': False, 'remote_deployment_verified': False}
    for setup in setups:
        report.setdefault('ros_python_checks', {})[setup] = ros_python_check(setup)
    rendered = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(rendered)
    print(rendered, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
