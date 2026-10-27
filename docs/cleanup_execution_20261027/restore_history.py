"""Restore deduplicated historical backups into a NEW directory only.

Needs history_unique.zip and the pre-cleanup GitHub workspace ZIP. It never
writes to the original backup locations. Paths are validated before writing.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import os
import zipfile


def safe_join(base, rel):
    parts = PurePosixPath(rel).parts
    if any(p in {'.', '..'} or ':' in p or '\\' in p for p in parts) or rel.startswith('/'):
        raise ValueError('Unsafe manifest path: ' + rel)
    target = base.joinpath(*parts)
    if target != base and base not in target.resolve().parents:
        raise ValueError('Path leaves restore destination')
    return target


def restore(history, baseline, destination):
    destination = destination.resolve()
    if destination.exists() and any(destination.iterdir()):
        raise ValueError('Destination must be new or empty.')
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(history) as hz, zipfile.ZipFile(baseline) as bz:
        manifest = json.loads(hz.read('history_manifest.json'))
        roots = {r['id']: r for r in manifest['roots']}
        for r in manifest['roots']:
            if r['is_directory']:
                base = safe_join(destination, r['restore_label'])
                base.mkdir(parents=True, exist_ok=True)
                for rel in r['directories']:
                    safe_join(base, rel).mkdir(parents=True, exist_ok=True)
        for n, row in enumerate(manifest['files'], 1):
            root = roots[row['root_id']]
            base = safe_join(destination, root['restore_label'])
            target = safe_join(base, row['relative_path']) if root['is_directory'] else base
            target.parent.mkdir(parents=True, exist_ok=True)
            archive, member = (bz, row['baseline_member']) if row['content_source'] == 'cloud_baseline' else (hz, row['history_member'])
            h = hashlib.sha256()
            count = 0
            with archive.open(member) as src, target.open('xb') as dst:
                for chunk in iter(lambda: src.read(2 ** 20), b''):
                    count += len(chunk)
                    h.update(chunk)
                    dst.write(chunk)
            if count != row['bytes'] or h.hexdigest() != row['sha256']:
                raise ValueError('Restored file hash mismatch: ' + str(target))
            os.utime(target, ns=(row['mtime_ns'], row['mtime_ns']))
            if n % 500 == 0:
                print(f'Restored {n}/{len(manifest["files"])} files', flush=True)
    print(f'Restored {len(manifest["files"])} files into {destination}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--history', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    restore(args.history, args.baseline, args.destination)
