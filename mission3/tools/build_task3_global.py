#!/usr/bin/env python3
"""Compile the local PCL global-search helper; never starts sensors."""
import argparse
import json
from pathlib import Path
from task3_auto_relocalize import build_solver, sha256

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT/'.cache/task3_global_search/search')
    args = parser.parse_args()
    source = ROOT/'tools/global_relocalization/search.cpp'
    expected = sha256(source)
    checksum = args.out.with_name(args.out.name+'.source_sha256')
    if not args.out.is_file() or not checksum.is_file() or checksum.read_text().strip() != expected:
        build_solver(args.out)
        checksum.write_text(expected+'\n')
    print(json.dumps({'global_search_binary': str(args.out.resolve()), 'source_sha256': expected,
                      'hardware_started': False}, indent=2))


if __name__ == '__main__':
    main()
