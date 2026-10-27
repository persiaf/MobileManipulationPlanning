"""Verify repeated large data with SHA256; read-only outside this report folder."""
import collections
import concurrent.futures
import csv
import hashlib
import json
from pathlib import Path

OUT = Path(__file__).resolve().parent


def read(name):
    with (OUT / name).open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def digest(row):
    p = Path(row['absolute_path'])
    before = p.stat()
    h = hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda: f.read(2 ** 20), b''):
            h.update(chunk)
    after = p.stat()
    return {'absolute_path': str(p), 'bytes': before.st_size, 'sha256': h.hexdigest(),
            'stable_during_hash': before.st_size == after.st_size and before.st_mtime_ns == after.st_mtime_ns,
            'device': before.st_dev, 'file_id': before.st_ino,
            'source': row.get('scope_kind', 'CURRENT_PROJECT')}


current = read('project_files.csv')
external = read('external_backup_files.csv')
index = collections.defaultdict(list)
for row in external:
    if '.git' not in Path(row['absolute_path']).parts:
        index[(Path(row['absolute_path']).name, int(row['bytes']))].append(row)
results = []
groups = []
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    for row in sorted(current, key=lambda r: -int(r['bytes'])):
        if int(row['bytes']) <= 8 * 2 ** 20:
            continue
        matches = index[(Path(row['absolute_path']).name, int(row['bytes']))]
        if not matches:
            continue
        equal = list(pool.map(digest, [row] + matches))
        group = row['relative_path']
        ref = equal[0]['sha256']
        for item in equal:
            item['current_relative_path'] = group
            item['same_as_current'] = item['sha256'] == ref and item['stable_during_hash']
        confirmed = [i for i in equal[1:] if i['same_as_current']]
        groups.append({'current_relative_path': group, 'bytes_per_file': int(row['bytes']),
                       'external_candidates': len(matches), 'confirmed_equal_external_files': len(confirmed),
                       'equal_external_logical_bytes': sum(i['bytes'] for i in confirmed),
                       'distinct_file_ids_including_current': len({(i['device'], i['file_id']) for i in equal if i['same_as_current']})})
        results.extend(equal)
        print(f'{group}: {len(confirmed)}/{len(matches)} external copies match', flush=True)

for name, rows in [('large_copy_verification.csv', results), ('large_copy_groups.csv', groups)]:
    with (OUT / name).open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else [])
        w.writeheader()
        w.writerows(rows)
summary = {'verified_current_large_files': len(groups),
           'verified_external_copies_equal_to_current': sum(g['confirmed_equal_external_files'] for g in groups),
           'equal_external_logical_bytes': sum(g['equal_external_logical_bytes'] for g in groups),
           'unstable_files': sum(not r['stable_during_hash'] for r in results),
           'note': '只是相同内容的外部副本逻辑体积；不等于授权删除量或可释放物理空间。保留恢复链和关键基线。'}
(OUT / 'large_copy_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(summary, ensure_ascii=True))
