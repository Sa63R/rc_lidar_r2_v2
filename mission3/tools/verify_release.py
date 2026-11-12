#!/usr/bin/env python3
"""Verify source release hashes before configuring the destination host."""
import hashlib
import json
from pathlib import Path, PurePosixPath


def verify(root):
    root = Path(root).resolve()
    manifest = json.loads((root/'MANIFEST.json').read_text())
    for row in manifest['files']:
        name = PurePosixPath(row['path'])
        if name.is_absolute() or '..' in name.parts:
            raise ValueError('unsafe manifest path')
        path = root.joinpath(*name.parts)
        if not path.is_file() or path.is_symlink():
            raise ValueError('missing or linked release file: '+str(name))
        content = path.read_bytes()
        if len(content) != row['size'] or hashlib.sha256(content).hexdigest() != row['sha256']:
            raise ValueError('release file changed: '+str(name))
    return {'release': manifest['release'], 'verified_files': len(manifest['files']),
            'scope': '源码文件完整性；不代表目标主机部署或物理精度验收'}


if __name__ == '__main__':
    print(json.dumps(verify(Path(__file__).resolve().parents[1]), ensure_ascii=False, indent=2))
