"""Create and publish a complete workspace checkpoint to this repo's Releases.

Git credentials stay in memory and come from the system credential helper.
The ZIP includes ignored project files and empty directories, excludes .git,
and preserves raw file bytes. All generated assets/state go to --output,
which must be outside the project. No source file is edited by this script.
"""
from __future__ import annotations
import argparse
import base64
import csv
import datetime as dt
import hashlib
import json
import mimetypes
import os
from pathlib import Path
import re
import subprocess
import time
import urllib.parse
import zipfile
import requests

ROOT = Path(__file__).resolve().parents[2]
REPO = 'persiaf/MobileManipulationPlanning'
TZ = dt.timezone(dt.timedelta(hours=8))


def git(*args, binary=False):
    env = os.environ.copy()
    env['GIT_OPTIONAL_LOCKS'] = '0'
    p = subprocess.run(['git', '-c', 'core.quotepath=false', *args], cwd=ROOT,
                       capture_output=True, env=env)
    if p.returncode:
        raise RuntimeError('git failed: ' + ' '.join(args) + '\n' + p.stderr.decode('utf-8', errors='replace'))
    return p.stdout if binary else p.stdout.decode('utf-8', errors='replace')


def credential():
    env = os.environ.copy()
    env.update(GIT_TERMINAL_PROMPT='0', GCM_INTERACTIVE='Never')
    p = subprocess.run(['git', 'credential', 'fill'], input='protocol=https\nhost=github.com\n\n',
                       text=True, capture_output=True, env=env, cwd=ROOT)
    if p.returncode:
        raise RuntimeError('System Git credential helper could not supply GitHub credentials.')
    values = dict(line.split('=', 1) for line in p.stdout.splitlines() if '=' in line)
    if not values.get('password'):
        raise RuntimeError('No GitHub credential returned by the system helper.')
    return values['password']


def session():
    s = requests.Session()
    s.headers.update({'Authorization': 'Bearer ' + credential(),
                      'Accept': 'application/vnd.github+json',
                      'X-GitHub-Api-Version': '2022-11-28',
                      'User-Agent': 'MobileManipulationPlanning-checkpoint'})
    return s


def api(s, method, path, **kwargs):
    url = 'https://api.github.com' + path
    r = s.request(method, url, timeout=(30, 180), **kwargs)
    if not r.ok:
        try:
            message = r.json().get('message', 'API error')
        except ValueError:
            message = r.reason
        raise RuntimeError(f'GitHub API {method} {path}: {r.status_code} {message}')
    return r.json() if r.content else None


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(2 ** 20), b''):
            h.update(chunk)
    return h.hexdigest()


def workspace_files():
    files, empty = [], []
    for base, dirs, names in os.walk(ROOT, followlinks=False):
        dirs[:] = sorted(d for d in dirs if d != '.git')
        if not dirs and not names:
            empty.append(Path(base).relative_to(ROOT).as_posix())
        for name in sorted(names):
            p = Path(base) / name
            if p.is_symlink():
                raise RuntimeError('Symlink requires explicit snapshot support: ' + str(p))
            files.append(p)
    return files, empty


def inspect():
    s = session()
    repo = api(s, 'GET', '/repos/' + REPO)
    print(json.dumps({'repo': repo['full_name'], 'url': repo['html_url'],
                      'visibility': repo['visibility'], 'permissions': repo.get('permissions'),
                      'default_branch': repo['default_branch']}, ensure_ascii=True))


def prepare(output, tag):
    if output == ROOT or ROOT in output.parents:
        raise RuntimeError('Snapshot output must be outside the project tree.')
    output.mkdir(parents=True, exist_ok=True)
    commit = git('rev-parse', 'HEAD').strip()
    if git('status', '--porcelain=v1', '-uall').strip():
        raise RuntimeError('Commit all non-ignored workspace changes before preparing the snapshot.')
    if git('rev-parse', tag).strip() != commit:
        raise RuntimeError('Checkpoint tag does not resolve to HEAD; use a lightweight tag or peel it.')
    tracked = set(git('ls-files', '-z').split('\0'))
    ignored = set(git('ls-files', '--others', '-i', '--exclude-standard', '-z').split('\0'))
    files, empty = workspace_files()
    manifest = {'repository': REPO, 'commit': commit, 'branch': git('branch', '--show-current').strip(),
                'tag': tag, 'created_utc8': dt.datetime.now(TZ).isoformat(timespec='seconds'),
                'scope': 'Entire current project workspace including ignored files; .git excluded. External backup contents and Codex credentials are outside scope.',
                'git_status': git('status', '--porcelain=v1', '-uall'), 'empty_directories': empty,
                'files': []}
    archive = output / f'MobileManipulationPlanning-{tag}-workspace.zip'
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=1, allowZip64=True) as z:
        for n, p in enumerate(files, 1):
            rel = p.relative_to(ROOT).as_posix()
            before = p.stat()
            digest = hashlib.sha256()
            # Hash the same bytes that enter the ZIP, then check the file did not change.
            info = zipfile.ZipInfo.from_file(p, arcname='MobileManipulationPlanning/' + rel)
            info.compress_type = zipfile.ZIP_DEFLATED
            info._compresslevel = 1
            with p.open('rb') as src, z.open(info, 'w', force_zip64=True) as dst:
                for chunk in iter(lambda: src.read(2 ** 20), b''):
                    digest.update(chunk)
                    dst.write(chunk)
            after = p.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise RuntimeError('Workspace changed during snapshot: ' + rel)
            manifest['files'].append({'path': rel, 'bytes': before.st_size,
                                      'mtime_ns': before.st_mtime_ns, 'mode': before.st_mode,
                                      'sha256': digest.hexdigest(), 'tracked': rel in tracked,
                                      'ignored': rel in ignored})
            if n % 100 == 0:
                print(f'Archived {n}/{len(files)} files', flush=True)
        for rel in empty:
            z.writestr('MobileManipulationPlanning/' + rel + '/', b'')
    manifest['file_count'] = len(files)
    manifest['logical_bytes'] = sum(r['bytes'] for r in manifest['files'])
    manifest['ignored_file_count'] = sum(r['ignored'] for r in manifest['files'])
    manifest_path = output / 'workspace_manifest.json'
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    bundle = output / f'MobileManipulationPlanning-{tag}.bundle'
    git('bundle', 'create', str(bundle), 'refs/heads/' + manifest['branch'], 'refs/tags/' + tag, 'HEAD')
    git('bundle', 'verify', str(bundle))
    print('Verifying ZIP contents and CRC', flush=True)
    with zipfile.ZipFile(archive) as z:
        bad = z.testzip()
        if bad:
            raise RuntimeError('ZIP CRC failed: ' + bad)
        if len([i for i in z.infolist() if not i.is_dir()]) != len(files):
            raise RuntimeError('ZIP file count mismatch')
    restore = output / 'RESTORE.md'
    restore.write_text(f'''# Workspace checkpoint {tag}

Source commit: {commit}
Branch: {manifest['branch']}
Files: {len(files)}; logical bytes: {manifest['logical_bytes']}; ignored files: {manifest['ignored_file_count']}.

The workspace ZIP contains every current project file as raw bytes, including
ignored datasets, experiment states, logs, media, old models, and empty folders.
It excludes .git and all external project/Codex backups; their audit lists remain included.
The manifest records SHA-256, sizes, original timestamps, tracked/ignored state.
No MATLAB experiment or regression was run for this preservation checkpoint.

Restore into a NEW directory. Verify all downloaded assets against SHA256SUMS.txt.
Unzip the workspace archive to restore its MobileManipulationPlanning folder.
The Git bundle independently preserves the current branch history and checkpoint
tag. To restore both Git and exact workspace bytes, clone the bundle first and
then overlay the workspace ZIP into that fresh clone, keeping its .git folder.
The recorded source commit includes the existing deletions; stale files are not restored.
Git's own source-code ZIP omits ignored files; use this workspace ZIP for a complete restore.
''', encoding='utf-8')
    assets = [archive, bundle, manifest_path, restore]
    checksums = output / 'SHA256SUMS.txt'
    checksums.write_text(''.join(sha(p) + '  ' + p.name + '\n' for p in assets), encoding='utf-8')
    assets.append(checksums)
    state = {'repository': REPO, 'tag': tag, 'commit': commit, 'branch': manifest['branch'],
             'file_count': len(files), 'ignored_file_count': manifest['ignored_file_count'],
             'logical_bytes': manifest['logical_bytes'],
             'assets': [{'name': p.name, 'path': str(p), 'bytes': p.stat().st_size, 'sha256': sha(p)} for p in assets]}
    (output / 'upload_state.json').write_text(json.dumps(state, indent=2), encoding='utf-8')
    print(json.dumps(state, ensure_ascii=True), flush=True)


class ProgressFile:
    def __init__(self, path):
        self.file = path.open('rb')
        self.path = path
        self.next_log = 32 * 2 ** 20
    def __len__(self):
        return self.path.stat().st_size
    def read(self, size=-1):
        chunk = self.file.read(size)
        if self.file.tell() >= self.next_log:
            print(f'Uploading {self.path.name}: {self.file.tell() // 2**20} MiB sent', flush=True)
            self.next_log += 32 * 2 ** 20
        return chunk
    def tell(self):
        return self.file.tell()
    def fileno(self):
        return self.file.fileno()
    def close(self):
        self.file.close()


def upload(output):
    state = json.loads((output / 'upload_state.json').read_text(encoding='utf-8'))
    s = session()
    releases = api(s, 'GET', '/repos/' + REPO + '/releases?per_page=100')
    release = next((r for r in releases if r['tag_name'] == state['tag']), None)
    body = f'''Preservation checkpoint for the current MATLAB planning workspace.

Commit: `{state['commit']}`; branch: `{state['branch']}`.

The custom workspace ZIP contains **all {state['file_count']} current project files**, including {state['ignored_file_count']} ignored experiment/data/media files ({state['logical_bytes']:,} logical bytes total). The Git bundle preserves the current branch history and tag. SHA256SUMS.txt and workspace_manifest.json provide integrity and per-file restoration information. See RESTORE.md.

Code, configs, tests, figures, experiment results, and the 2026-10-27 cleanup audit are preserved. External backup contents and Codex credentials are outside the project snapshot; their audit inventories are included. This is a preservation checkpoint; MATLAB regressions were not rerun and existing behavior is not newly certified.

Use the custom **workspace ZIP**, rather than GitHub's automatically generated source ZIP, to restore ignored files.
'''
    if release is None:
        release = api(s, 'POST', '/repos/' + REPO + '/releases', json={
            'tag_name': state['tag'], 'target_commitish': state['commit'],
            'name': 'Full workspace checkpoint ' + state['tag'], 'body': body,
            'draft': True, 'prerelease': True, 'make_latest': 'false'})
    upload_url = release['upload_url'].split('{')[0]
    if urllib.parse.urlsplit(upload_url).hostname != 'uploads.github.com':
        raise RuntimeError('Unexpected GitHub upload host')
    asset_path = f'/repos/{REPO}/releases/{release["id"]}/assets?per_page=100'
    for item in state['assets']:
        existing = next((a for a in api(s, 'GET', asset_path) if a['name'] == item['name']), None)
        if existing:
            if existing['size'] != item['bytes'] or existing.get('digest') != 'sha256:' + item['sha256']:
                raise RuntimeError('Existing release asset does not match checkpoint: ' + item['name'])
            print('Verified existing asset: ' + item['name'], flush=True)
            continue
        reader = ProgressFile(Path(item['path']))
        try:
            response = s.post(upload_url, params={'name': item['name']}, data=reader,
                              headers={'Content-Type': mimetypes.guess_type(item['name'])[0] or 'application/octet-stream',
                                       'Content-Length': str(item['bytes'])}, timeout=(30, 600))
        finally:
            reader.close()
        if not response.ok:
            raise RuntimeError('Asset upload failed: ' + item['name'] + ' HTTP ' + str(response.status_code))
        uploaded = response.json()
        if uploaded['size'] != item['bytes'] or uploaded.get('digest') != 'sha256:' + item['sha256']:
            raise RuntimeError('Uploaded asset integrity mismatch: ' + item['name'])
        print('Uploaded and SHA256 verified: ' + item['name'], flush=True)
    release = api(s, 'PATCH', f'/repos/{REPO}/releases/{release["id"]}',
                  json={'draft': False, 'prerelease': True, 'make_latest': 'false', 'body': body})
    verified = api(s, 'GET', asset_path)
    result = {'release_url': release['html_url'], 'tag': state['tag'], 'commit': state['commit'],
              'draft': release['draft'], 'assets': [{'name': a['name'], 'bytes': a['size'],
                  'digest': a.get('digest'), 'url': a['browser_download_url']} for a in verified]}
    (output / 'remote_verification.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=True), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['inspect', 'prepare', 'upload'])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--tag')
    args = parser.parse_args()
    if args.action == 'inspect':
        inspect()
    elif args.action == 'prepare':
        prepare(args.output.resolve(), args.tag)
    else:
        upload(args.output.resolve())
