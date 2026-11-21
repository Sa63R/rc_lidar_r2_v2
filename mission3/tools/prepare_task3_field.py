#!/usr/bin/env python3
"""Materialize a byte-identical saved map and relocate the bound field config.

No sensor access, map alignment, coordinate zeroing, or change of reference ID.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import lzma
import os
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/task1_fusion'))
from task1_fusion.task3_config import sha256, load_config
from task1_fusion.task3_fixed_map import validate, reference_id


def prepare(root):
    root = Path(root).resolve()
    seed = yaml.safe_load((root/'field_reference/task3_corner.yaml').read_text())
    p = seed['task3_corner']['ros__parameters']
    mechanical = root/'config/car_task3_local.yaml'
    if mechanical.is_file():
        local = load_config(mechanical)['task3_corner']['ros__parameters']
        keys = ('source_to_body_xyz', 'source_to_body_rpy', 'body_to_lidar_xyz',
                'body_to_lidar_rpy', 'lidar_to_diagram_rpy', 'lidar_to_corner_diagram_xyz')
        if any(local.get(key) != p.get(key) for key in keys):
            raise ValueError('local mechanical parameters differ from the bound field reference; no overwrite performed')
    expected = p['expected_map_sha256']
    if len(expected) != 64 or any(c not in '0123456789abcdef' for c in expected):
        raise ValueError('invalid field-map SHA256')
    map_path = root/'maps'/('field_map_'+expected[:12]+'.pcd')
    if map_path.is_symlink():
        raise ValueError('refusing a linked map destination')
    if map_path.exists():
        if sha256(map_path) != expected:
            raise ValueError('existing saved map has different content; keep it and contact maintainer')
    else:
        map_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = map_path.with_name(map_path.name+'.preparing-'+str(os.getpid()))
        digest, size = hashlib.sha256(), 0
        try:
            with lzma.open(root/'maps/field_map.pcd.xz', 'rb') as src, temporary.open('xb') as dst:
                while block := src.read(1024*1024):
                    size += len(block)
                    if size > 256*1024*1024:
                        raise ValueError('map exceeds the expected deployment size limit')
                    dst.write(block)
                    digest.update(block)
            if digest.hexdigest() != expected:
                raise ValueError('saved map SHA256 mismatch after lossless decompression')
            temporary.replace(map_path)
        finally:
            temporary.unlink(missing_ok=True)
    config = deepcopy(seed)
    config['task3_corner']['ros__parameters']['map_file'] = str(map_path)
    validate(config)
    destination = root/'config/car_task3_field.yaml'
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink():
        raise ValueError('refusing a linked field configuration')
    if destination.exists():
        current = load_config(destination)
        # Relocation may alter only the map path. Protect locally changed
        # geometry, origin, filter or availability settings from silent overwrite.
        before = deepcopy(current)
        before['task3_corner']['ros__parameters']['map_file'] = str(map_path)
        if before != config:
            raise ValueError('existing field configuration has local changes; no overwrite performed')
    temporary_config = destination.with_name(destination.name+'.preparing-'+str(os.getpid()))
    try:
        with temporary_config.open('x') as stream:
            yaml.safe_dump(config, stream, sort_keys=False, allow_unicode=True)
        temporary_config.replace(destination)
    finally:
        temporary_config.unlink(missing_ok=True)
    return {'map_file': str(map_path), 'map_sha256': expected, 'config': str(destination),
            'reference_id': config['task3_vision']['ros__parameters']['reference_id'],
            'startup_zeroing': False, 'hardware_started': False,
            'physical_accuracy_verified': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--map-path-only', action='store_true')
    args = parser.parse_args()
    if args.map_path_only:
        p, _ = validate(load_config(args.root/'config/car_task3_field.yaml'))
        print(p['map_file'])
    else:
        print(json.dumps(prepare(args.root), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
