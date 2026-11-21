#!/usr/bin/env python3
"""Validate a source archive, update an already-built host, add its menu entry."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path, PurePosixPath
import tarfile


def sha(content):
    return hashlib.sha256(content).hexdigest()


def read_release(archive):
    with tarfile.open(archive, 'r:gz') as tar:
        files = {}
        roots = set()
        for member in tar:
            name = PurePosixPath(member.name)
            if name.is_absolute() or '..' in name.parts or member.issym() or member.islnk():
                raise ValueError('压缩包包含不安全路径')
            roots.add(name.parts[0])
            if member.isfile():
                files['/'.join(name.parts[1:])] = (tar.extractfile(member).read(), member.mode)
            elif not member.isdir():
                raise ValueError('压缩包包含不支持的文件')
    if len(roots) != 1 or 'MANIFEST.json' not in files:
        raise ValueError('压缩包不是任务三源码包')
    manifest = json.loads(files['MANIFEST.json'][0])
    for row in manifest['files']:
        content = files[row['path']][0]
        if len(content) != row['size'] or sha(content) != row['sha256']:
            raise ValueError('源码校验失败：'+row['path'])
    if set(files) != {'MANIFEST.json', *(row['path'] for row in manifest['files'])}:
        raise ValueError('源码包存在清单外文件')
    return files, manifest


def update(archive, target, shortcut=True):
    target = Path(target).expanduser().resolve()
    if not (target/'install/setup.bash').is_file():
        raise ValueError('这是一台主机的更新入口；目标目录尚未编译。请先按安装维护说明部署：'+str(target))
    files, manifest = read_release(archive)
    info = json.loads(files['build_information.json'][0])
    needs_rebuild = info.get('update_requires_rebuild') is True
    compatible = info.get('compatible_host_files')
    if not compatible and not needs_rebuild:
        raise ValueError('此版本未声明可免编译更新，不能使用本更新入口')
    for name, expected in (compatible or {}).items():
        if not (target/name).is_file() or sha((target/name).read_bytes()) != expected:
            raise ValueError('主机算法源码与免编译更新基线不同：'+name+'；请联系维护者')
    previous = json.loads((target/'MANIFEST.json').read_text())
    known = {row['path']: row['sha256'] for row in previous['files']}
    preserved, conflicts = [], []
    for name, (content, mode) in files.items():
        dest = target/name
        if any(parent.is_symlink() for parent in dest.parents if parent != target.parent):
            raise ValueError('目标路径包含链接：'+name)
        if dest.is_symlink():
            raise ValueError('拒绝覆盖链接：'+name)
        if dest.exists() and not dest.is_file():
            raise ValueError('文件路径被目录占用：'+name)
        if dest.exists() and name.startswith('config/'):
            preserved.append(name)
        elif dest.exists() and name != 'MANIFEST.json':
            current = sha(dest.read_bytes())
            if current != sha(content) and current != known.get(name):
                conflicts.append(name)
    if conflicts:
        raise ValueError('以下源码有本地修改，未执行更新：'+', '.join(conflicts))
    for name, (content, mode) in files.items():
        if name in preserved:
            continue
        dest = target/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        temporary = dest.with_name(dest.name+'.task3-update-tmp')
        with temporary.open('xb') as stream:
            stream.write(content)
        temporary.chmod(mode & 0o777)
        temporary.replace(dest)
    if shortcut:
        import os
        spec = importlib.util.spec_from_file_location('task3_shortcut', target/'tools/install_task3_shortcut.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        data = Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share')))
        module.install(target, data/'applications')
    return {'release': manifest['release'], 'target': str(target), 'preserved_configs': preserved,
            'needs_rebuild': needs_rebuild, 'needs_prepare_field': needs_rebuild,
            'hardware_started': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--target', type=Path, default=Path.home()/'rc_mission3/mission3')
    args = parser.parse_args()
    try:
        result = update(args.archive, args.target)
    except (OSError, ValueError, KeyError, tarfile.TarError) as exc:
        parser.exit(2, f'更新未完成：{exc}\n请保留错误信息；不要重新建立原点掩盖故障。\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result['needs_rebuild']:
        print('源码更新完成。先在目标目录依次运行prepare-field、build-map、build-app，再使用run或菜单启动。')
    else:
        print('更新完成。在主机应用菜单搜索“任务三角点坐标”；也可在目标目录执行：bash bin/task3 run')


if __name__ == '__main__':
    main()
