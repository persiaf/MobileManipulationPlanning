"""Prepare a checked, deduplicated archive and an exact cleanup plan.

This program deletes nothing. Actual removal is performed by the adjacent
PowerShell script, using independently validated literal target paths.
Credentials are only used in memory to reverify the existing private release.
"""
from __future__ import annotations
import collections
import csv
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import runpy
import shutil
import stat
import subprocess
import zipfile

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent.parent
PARENT = ROOT.parent
AUDIT = ROOT / 'docs/cleanup_audit_20261027'
ARCHIVE_DIR = PARENT / 'codex_backups/retained_history_20261027'
TEMP_SNAPSHOT = Path('C:/Users/Administrator/AppData/Local/Temp/MobileManipulationPlanning_github_checkpoint_20261027_095303')
CODEX = Path('C:/Users/Administrator/.codex')
BASELINE_TAG = 'checkpoint-20261027-095303'
EPOCH_TICKS = 621355968000000000
TZ = dt.timezone(dt.timedelta(hours=8))


def read_csv(p):
    with p.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def dump(p, value):
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda: f.read(2 ** 20), b''):
            h.update(chunk)
    return h.hexdigest()


def meta(p):
    s = p.stat()
    if getattr(s, 'st_file_attributes', 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT:
        raise RuntimeError('Refusing reparse point: ' + str(p))
    return {'absolute_path': str(p), 'bytes': s.st_size, 'mtime_ns': s.st_mtime_ns,
            'mtime_ticks': str(s.st_mtime_ns // 100 + EPOCH_TICKS), 'mode': s.st_mode}


def safe_external(p):
    p = p.resolve(strict=True)
    if p == ROOT or ROOT in p.parents:
        raise RuntimeError('External root unexpectedly overlaps active workspace: ' + str(p))
    valid = (p.parent in {PARENT / 'codex_backups/MobileManipulationPlanning', PARENT / 'codex_backups/paper_tac'}
             or (p.parent == PARENT and (p.name.startswith('MobileManipulationPlanning') or p.name in {'results', 'stage_output_custom_scene', 'stage_output_custom_scene_v2'}))
             or (p.parent.resolve() == Path(os.environ['TEMP']).resolve() and p.name == 'MobileManipulationPlanning_cleanup_20260831_181820'))
    if not valid:
        raise RuntimeError('External root outside explicitly audited locations: ' + str(p))
    meta(p)
    return p


def tree(p):
    if p.is_file():
        return [p], []
    files, directories = [], []
    def onerror(error):
        raise error
    for base, dirs, names in os.walk(p, followlinks=False, onerror=onerror):
        for name in sorted(dirs):
            sub = Path(base) / name
            meta(sub)
            directories.append(sub.relative_to(p).as_posix())
        files.extend(Path(base) / name for name in sorted(names))
    return sorted(files), sorted(directories)


def main():
    drive_free_before = {'D': shutil.disk_usage(ROOT.anchor).free, 'C': shutil.disk_usage(CODEX.anchor).free}
    if ARCHIVE_DIR.exists():
        raise RuntimeError('Archive output already exists; inspect it instead of overwriting: ' + str(ARCHIVE_DIR))
    helper = runpy.run_path(str(ROOT / 'docs/cloud_checkpoint_20261027/cloud_snapshot.py'))
    session = helper['session']()
    release = helper['api'](session, 'GET', '/repos/persiaf/MobileManipulationPlanning/releases/tags/' + BASELINE_TAG)
    expected = json.loads((TEMP_SNAPSHOT / 'upload_state.json').read_text(encoding='utf-8'))
    assets = []
    for item in expected['assets']:
        asset = next((a for a in release['assets'] if a['name'] == item['name']), None)
        if not asset or asset['size'] != item['bytes'] or asset.get('digest') != 'sha256:' + item['sha256']:
            raise RuntimeError('Pre-cleanup cloud checkpoint asset failed verification: ' + item['name'])
        assets.append({'name': item['name'], 'bytes': item['bytes'], 'sha256': item['sha256'], 'url': asset['browser_download_url']})
    if release['draft']:
        raise RuntimeError('Cloud checkpoint is a draft')
    baseline_path = TEMP_SNAPSHOT / 'workspace_manifest.json'
    if sha(baseline_path) != next(a['sha256'] for a in assets if a['name'] == 'workspace_manifest.json'):
        raise RuntimeError('Local baseline manifest differs from cloud verified manifest')
    baseline = json.loads(baseline_path.read_text(encoding='utf-8'))
    shutil.copyfile(baseline_path, OUT / 'baseline_workspace_manifest.json')
    cloud = {'verified_utc8': dt.datetime.now(TZ).isoformat(timespec='seconds'),
             'release_url': release['html_url'], 'commit': baseline['commit'], 'assets': assets}
    dump(OUT / 'cloud_verification.json', cloud)
    by_hash = {}
    for row in baseline['files']:
        by_hash.setdefault(row['sha256'], row['path'])
    by_path = {r['path']: r for r in baseline['files']}
    roots = []
    skipped_roots = []
    history_files = []
    unique = {}
    audit_roots = read_csv(AUDIT / 'external_backup_roots.csv')
    print('Hashing external backup contents; no files are being deleted', flush=True)
    for i, original in enumerate(audit_roots):
        if Path(original['absolute_path']).name == 'MobileManipulationPlanning_removed_python_cache_20260720':
            skipped_roots.append({'absolute_path': original['absolute_path'],
                                  'reason': 'Owner-specific ACL denies Administrator listing and ownership change; retained in place. Audit recorded six pytest cache files, 1259 bytes.'})
            print('Retaining inaccessible legacy pytest cache; continuing with all other backups', flush=True)
            continue
        p = safe_external(Path(original['absolute_path']))
        root_id = f'r{i:03d}'
        files, directories = tree(p)
        roots.append({'id': root_id, 'absolute_path': str(p), 'kind': original['kind'],
                      'is_directory': p.is_dir(), 'restore_label': root_id + '__' + p.name,
                      'directories': directories, 'files': len(files)})
        for f in files:
            row = meta(f)
            row.update(root_id=root_id, relative_path=f.relative_to(p).as_posix() if p.is_dir() else '', sha256=sha(f))
            if row['sha256'] in by_hash:
                row.update(content_source='cloud_baseline', baseline_member='MobileManipulationPlanning/' + by_hash[row['sha256']], history_member='')
            else:
                row.update(content_source='history_archive', history_member='objects/' + row['sha256'], baseline_member='')
                unique.setdefault(row['sha256'], (f, row))
            history_files.append(row)
        print(f'Indexed backup {i+1}/{len(audit_roots)}: {len(files)} files', flush=True)
    candidates = []
    chosen = {'LEGACY_LNN_EXPERIMENT', 'LEGACY_PAPER_CACHE', 'SMOKE_VISUAL_OUTPUT',
              'REDUNDANT_PARTIAL_CONFIRMED', 'HISTORICAL_LOG', 'PROJECT_ARCHIVE'}
    for row in read_csv(AUDIT / 'project_files.csv'):
        rel = row['relative_path']
        old_learning_partial = rel.startswith('paper_tac/results/raw/') and 'lnn' in Path(rel).name.lower()
        if row['category'] not in chosen and not old_learning_partial:
            continue
        p = (ROOT / rel).resolve(strict=True)
        if ROOT not in p.parents or '.git' in p.relative_to(ROOT).parts or p.suffix == '.m':
            raise RuntimeError('Invalid active-workspace deletion target: ' + str(p))
        expected_row = by_path.get(rel)
        digest = sha(p)
        if not expected_row or expected_row['sha256'] != digest:
            raise RuntimeError('Project cleanup candidate has no exact verified cloud copy: ' + rel)
        m = meta(p)
        m.update(relative_path=rel, sha256=digest, category='LEGACY_LEARNING_PARTIAL' if old_learning_partial else row['category'],
                 content_source='cloud_baseline', baseline_member='MobileManipulationPlanning/' + rel)
        candidates.append(m)
    keep = [r for r in baseline['files'] if r['path'].startswith('results/tables/p3p4_state_')
            or (r['path'].startswith('results/tables/position_only_resource_bounded_seed') and r['path'].endswith('_state.mat'))
            or r['path'] == 'results/tables/position_only_resource_bounded_four_mode_partial.csv'
            or r['path'] in {'results/tables/position_only_isolated_four_mode_partial.csv',
                             'results/tables/position_only_resource_bounded_full_audit_interrupted_9rows.csv'}]
    for row in keep:
        if sha(ROOT / row['path']) != row['sha256']:
            raise RuntimeError('Protected experiment dependency changed: ' + row['path'])
    old_configs = []
    for name in ['config.toml.bak-20260701-154201', 'config.toml.bak-before-code-skit-20260724',
                 'config.toml.bak-before-medium-20260903-0948', 'config.toml.bak-before-stream-fix-20260720-2155',
                 'tmp_large_scale_backup_path.txt']:
        p = CODEX / name
        m = meta(p)
        m.update(sha256=sha(p), category='OBSOLETE_CODEX_CONFIG_BACKUP' if name.startswith('config') else 'STALE_BACKUP_POINTER')
        old_configs.append(m)
    for name in ['config.toml', '.env', 'config.toml.bak-too-many-pending-20260915-1345',
                 '.env.bak-20261027-093020-proxy-fix', 'state_5.sqlite.bak-before-gpt55-low-20260903-0945']:
        if not (CODEX / name).is_file():
            raise RuntimeError('Required current config/retained rollback point missing: ' + name)
    # Only the already uploaded temporary snapshot directory is removable.
    temporary_files, temporary_dirs = tree(TEMP_SNAPSHOT)
    temporary = []
    for p in temporary_files:
        m = meta(p)
        m.update(sha256=sha(p))
        temporary.append(m)
    manifest = {'created_utc8': dt.datetime.now(TZ).isoformat(timespec='seconds'),
                'baseline': cloud, 'roots': roots, 'files': history_files,
                'unique_payloads': len(unique), 'scope': 'Complete audited external backup/output files; duplicate bytes reference the verified pre-cleanup GitHub workspace ZIP.'}
    ARCHIVE_DIR.mkdir(parents=True)
    archive_path = ARCHIVE_DIR / 'history_unique.zip'
    print(f'Writing {len(unique)} unique historical payloads; common data stays in verified cloud checkpoint', flush=True)
    with zipfile.ZipFile(archive_path, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=1, allowZip64=True) as z:
        for i, (digest, (p, row)) in enumerate(unique.items(), 1):
            if meta(p)['mtime_ns'] != row['mtime_ns'] or p.stat().st_size != row['bytes']:
                raise RuntimeError('History source changed during archive: ' + str(p))
            info = zipfile.ZipInfo('objects/' + digest)
            info.compress_type = zipfile.ZIP_STORED if p.suffix.lower() in {'.zip', '.gz', '.7z'} else zipfile.ZIP_DEFLATED
            info._compresslevel = 1
            h = hashlib.sha256()
            with p.open('rb') as src, z.open(info, 'w', force_zip64=True) as dst:
                for chunk in iter(lambda: src.read(2 ** 20), b''):
                    h.update(chunk)
                    dst.write(chunk)
            if h.hexdigest() != digest:
                raise RuntimeError('History object hash mismatch: ' + str(p))
            if i % 250 == 0:
                print(f'Archived unique payloads {i}/{len(unique)}', flush=True)
        z.writestr('history_manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf-8'))
        z.write(OUT / 'restore_history.py', 'restore_history.py')
        z.writestr('README.txt', 'This archive preserves unique old backup bytes. Common bytes are restored from the checkpoint-20261027-095303 workspace ZIP. Use restore_history.py into a NEW directory; no originals are overwritten. See history_manifest.json for all original paths, hashes, and root labels.\n')
    print('Checking retained archive CRC and complete restoration coverage', flush=True)
    with zipfile.ZipFile(archive_path) as z:
        if z.testzip() is not None:
            raise RuntimeError('Historical archive CRC failure')
        names = set(z.namelist())
        for row in history_files:
            if row['content_source'] == 'history_archive' and row['history_member'] not in names:
                raise RuntimeError('Missing historical payload')
        # Exercise both restore sources without reproducing tens of GiB on disk.
        with zipfile.ZipFile(TEMP_SNAPSHOT / next(a['name'] for a in assets if a['name'].endswith('workspace.zip'))) as bz:
            sampled = []
            for source in ('history_archive', 'cloud_baseline'):
                sample = [r for r in history_files if r['content_source'] == source][:10]
                for row in sample:
                    source_zip, member = (z, row['history_member']) if source == 'history_archive' else (bz, row['baseline_member'])
                    h = hashlib.sha256()
                    with source_zip.open(member) as src:
                        for chunk in iter(lambda: src.read(2 ** 20), b''):
                            h.update(chunk)
                    if h.hexdigest() != row['sha256']:
                        raise RuntimeError('Restore sample failed')
                    sampled.append(row['absolute_path'])
    archive_sha = sha(archive_path)
    (ARCHIVE_DIR / 'SHA256SUMS.txt').write_text(archive_sha + '  history_unique.zip\n', encoding='utf-8')
    manifest_path = ARCHIVE_DIR / 'history_manifest.json'
    dump(manifest_path, manifest)
    shutil.copyfile(OUT / 'restore_history.py', ARCHIVE_DIR / 'restore_history.py')
    project_dirs = sorted({str(p) for row in candidates for p in Path(row['absolute_path']).parents
                           if ROOT in p.parents and len(p.relative_to(ROOT).parts) >= 2 and '.git' not in p.parts},
                          key=lambda p: len(Path(p).parts), reverse=True)
    plan = {'workspace_root': str(ROOT), 'parent_root': str(PARENT), 'codex_root': str(CODEX),
            'retained_archive': str(archive_path), 'retained_archive_bytes': archive_path.stat().st_size,
            'retained_archive_sha256': archive_sha, 'history_manifest': str(manifest_path),
            'history_manifest_sha256': sha(manifest_path), 'external_roots': roots,
            'project_files': candidates, 'project_empty_directory_candidates': project_dirs,
            'codex_files': old_configs, 'temporary_root': str(TEMP_SNAPSHOT), 'temporary_files': temporary,
            'temporary_directories': temporary_dirs, 'protected_project_files': keep,
            'baseline': cloud, 'restoration_samples_verified': len(sampled), 'skipped_roots': skipped_roots,
            'drive_free_before': drive_free_before,
            'summary': {'external_roots': len(roots), 'external_files': len(history_files),
                        'external_logical_bytes': sum(r['bytes'] for r in history_files),
                        'external_files_from_cloud_baseline': sum(r['content_source'] == 'cloud_baseline' for r in history_files),
                        'unique_history_payloads': len(unique),
                        'project_files': len(candidates), 'project_logical_bytes': sum(r['bytes'] for r in candidates),
                        'old_codex_files': len(old_configs),
                        'temporary_bytes': sum(r['bytes'] for r in temporary),
                        'net_logical_bytes_reclaimed': sum(r['bytes'] for r in history_files + candidates + old_configs + temporary) - archive_path.stat().st_size}}
    dump(OUT / 'cleanup_plan.json', plan)
    with (OUT / 'planned_deletions.csv').open('w', encoding='utf-8-sig', newline='') as f:
        fields = ['scope', 'absolute_path', 'bytes', 'sha256', 'recovery']
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for scope, group in [('EXTERNAL_BACKUP', history_files), ('PROJECT_DERIVED_DATA', candidates), ('OLD_CODEX_CONFIG', old_configs), ('UPLOADED_TEMP_SNAPSHOT', temporary)]:
            for row in group:
                recovery = 'history_unique.zip + verified cloud baseline' if scope == 'EXTERNAL_BACKUP' else 'verified cloud workspace ZIP' if scope in {'PROJECT_DERIVED_DATA', 'UPLOADED_TEMP_SNAPSHOT'} else 'obsolete config backup; recent rollback retained'
                w.writerow({'scope': scope, 'absolute_path': row['absolute_path'], 'bytes': row['bytes'], 'sha256': row['sha256'], 'recovery': recovery})
    dump(OUT / 'preparation_verification.json', {'archive_crc_ok': True, 'history_file_coverage': len(history_files),
         'unique_payload_hashes_checked': len(unique), 'restore_source_samples': len(sampled),
         'cloud_assets_verified': len(assets), 'protected_dependencies_verified': len(keep), 'no_deletion_performed': True})
    print(json.dumps({'archive': str(archive_path), 'archive_bytes': archive_path.stat().st_size,
                      'plan': str(OUT / 'cleanup_plan.json'), 'summary': plan['summary']}, ensure_ascii=True), flush=True)


if __name__ == '__main__':
    main()
