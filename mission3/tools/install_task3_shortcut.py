#!/usr/bin/env python3
"""Install a relocatable-on-install application-menu entry, without sudo."""
import argparse
from pathlib import Path


def install(root, applications):
    root, applications = Path(root).resolve(), Path(applications)
    if not (root/'bin/task3').is_file():
        raise ValueError('目标目录没有bin/task3')
    applications.mkdir(parents=True, exist_ok=True)
    path = str(root/'bin/task3').replace('%', '%%')
    for char in ('\\', '"', '`', '$'):
        path = path.replace(char, '\\'+char)
    target = applications/'task3-corner.desktop'
    target.write_text('[Desktop Entry]\nType=Application\nName=任务三角点坐标\n'
                      'Comment=启动雷达定位，在窗口建立原点并向视觉提供坐标\n'
                      f'Exec=/bin/bash "{path}" run\n'
                      'Terminal=true\nIcon=applications-science\nCategories=Education;Science;\n')
    target.chmod(0o755)
    return target


def main():
    import os
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    data = Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share')))
    path = install(args.root, data/'applications')
    print(f'已添加应用菜单入口：{path}\n在主机应用菜单搜索“任务三角点坐标”即可启动。')


if __name__ == '__main__':
    main()
