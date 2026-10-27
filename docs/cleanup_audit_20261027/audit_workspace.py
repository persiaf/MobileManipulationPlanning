"""Read-only inventory of project outputs and related backups; writes only here.

No deletion, move, restore, archive extraction, experiment execution, or config edit.
External configuration files are inventoried by metadata; their values are never
copied into the report. All times are explicitly UTC+08:00.
"""
from __future__ import annotations

import collections
import csv
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import zipfile

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent.parent
PARENT = ROOT.parent
CODEX = Path('C:/Users/Administrator/.codex')
TZ = dt.timezone(dt.timedelta(hours=8))
ERRORS = []
SCOPE = []
HASHES = {}


def stamp(ts):
    return dt.datetime.fromtimestamp(ts, TZ).isoformat(timespec='seconds')


def walk(root, exclude=()):
    root = Path(root)
    paths = []
    def onerror(error):
        ERRORS.append({'path': str(error.filename), 'error': str(error)})
    if root.is_file():
        return [root]
    for base, dirs, files in os.walk(root, onerror=onerror, followlinks=False):
        dirs[:] = [d for d in dirs if d not in exclude and not (Path(base) / d).is_symlink()
                   and Path(base) / d != OUT]
        for name in files:
            p = Path(base) / name
            if not p.is_symlink():
                paths.append(p)
    return sorted(paths, key=lambda p: str(p).lower())


def metadata(p):
    try:
        s = p.stat()
        return {'absolute_path': str(p), 'bytes': s.st_size,
                'mtime_utc8': stamp(s.st_mtime)}
    except OSError as error:
        ERRORS.append({'path': str(p), 'error': str(error)})
        return None


def write_csv(name, rows, fields=None):
    if fields is None:
        fields = list(dict.fromkeys(k for row in rows for k in row))
    with (OUT / name).open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def git(*args):
    result = subprocess.run(['git', '-c', 'core.quotepath=false', *args], cwd=ROOT,
                            capture_output=True)
    if result.returncode:
        ERRORS.append({'path': 'git ' + ' '.join(args),
                       'error': result.stderr.decode('utf-8', errors='replace')})
    return result.stdout.decode('utf-8', errors='replace')


def sha(p):
    key = str(p)
    if key not in HASHES:
        h = hashlib.sha256()
        try:
            with p.open('rb') as f:
                for chunk in iter(lambda: f.read(2 ** 20), b''):
                    h.update(chunk)
            HASHES[key] = h.hexdigest()
        except OSError as error:
            ERRORS.append({'path': key, 'error': str(error)})
            HASHES[key] = ''
    return HASHES[key]


def text(p):
    try:
        return p.read_text(encoding='utf-8-sig', errors='replace')
    except OSError as error:
        ERRORS.append({'path': str(p), 'error': str(error)})
        return ''


project_paths = walk(ROOT, exclude={'.git'})
tracked = set(git('ls-files', '-z').split('\0'))
ignored = set(git('ls-files', '--others', '-i', '--exclude-standard', '-z').split('\0'))
status_before = git('status', '--porcelain=v1', '-uall')
(OUT / 'git_status_at_scan.txt').write_text(status_before, encoding='utf-8')
texts = {p.relative_to(ROOT).as_posix(): text(p) for p in project_paths
         if p.suffix.lower() in {'.m', '.md', '.tex', '.py', '.ps1', '.sh', '.json', '.bib'}
         and p.stat().st_size < 2 ** 20}
source_texts = {k: v for k, v in texts.items() if Path(k).suffix.lower() in {'.m', '.py', '.ps1', '.sh'}}


def references(p):
    result = []
    for rel, body in texts.items():
        if rel == p.relative_to(ROOT).as_posix():
            continue
        for n, line in enumerate(body.splitlines(), 1):
            if p.name.lower() in line.lower():
                result.append(f'{rel}:{n}')
    return result


def classify(rel):
    name = Path(rel).name.lower()
    if re.search(r'(^|/)(__pycache__|\.pytest_cache)(/|$)', rel) or name.endswith(('.pyc', '.pyo', '.asv', '.m~', '.tmp')):
        return 'TEMP_REGENERABLE', '高', '可清理候选', '缓存/编辑器生成文件'
    if rel.startswith('paper_tac/build/') and Path(name).suffix in {'.aux', '.bbl', '.blg', '.log', '.out', '.toc', '.fls', '.fdb_latexmk', '.synctex', '.gz'}:
        return 'TEMP_REGENERABLE', '高', '可清理候选', 'LaTeX 构建中间文件；最终 PDF 单列保留'
    if any(rel.startswith(x) for x in ('results/managed_gui_smoke/', 'results/managed_gui_smoke2/',
           'results/figures/unified_smoke/', 'results/figures/visualization_validation/',
           'results/figures/visualization_report/')):
        return 'SMOKE_VISUAL_OUTPUT', '高', '可归档/清理候选', 'GUI/渲染验证的派生图片；保留测试源码'
    if rel.startswith('paper_tac/results/lnn/'):
        return 'LEGACY_LNN_EXPERIMENT', '中', '旧研究方向单独归档', 'paper_tac/README.md 明确当前稿件排除 LNN 主贡献；不据此认定无复现价值'
    if rel.startswith('paper_tac/results/cache/'):
        return 'LEGACY_PAPER_CACHE', '中', '核实可再生后清理', '旧论文缓存；当前仓库未找到完整再生工具，不能保证可再生'
    if re.match(r'results/tables/position_only_resource_bounded_seed\d+_state\.mat$', rel):
        return 'POSITION_STATE_DEPENDENCY', '高', '保留或迁入冷存储并修改读取路径', 'exp_position_only_isolated_four_mode.m 以动态文件名读取 state；不是纯垃圾'
    if re.match(r'results/tables/p3p4_state_.*\.mat$', rel):
        return 'P3P4_FROZEN_DEPENDENCY', '高', '保留', 'P3/P4 复现状态；其中 L2 由 P5 默认读取'
    if name.endswith('.csv') and re.search(r'partial|interrupted', name):
        return 'PARTIAL_RESULT_REVIEW', '中', '核对正式结果覆盖后归档', '中断/部分结果；须对比行内容，不能只看名字'
    if name.endswith('.log'):
        return 'HISTORICAL_LOG', '中', '历史诊断归档', '运行/回归日志；与报告一起保留证据后可从活动目录移出'
    if rel.startswith('results/hardware_matched_lab/'):
        return 'HARDWARE_DEBUG_HISTORY', '中', '旧轮次归档，保留最新验证', '9 月几何/可达性修复的轮次记录'
    if rel.endswith('.zip'):
        return 'PROJECT_ARCHIVE', '高', '旧快照归档，先验证内容', '压缩备份不是活动代码；不能以同名推断内容相同'
    if rel.startswith(('results/', 'paper_tac/results/')):
        return 'RESEARCH_EVIDENCE_RETAIN', '高', '保留；按实验族整理', '实验原始数据/正式结果/图表，不因早于当前代码而标为无用'
    if '/' not in rel and re.search(r'(report|audit|checkpoint|validation|s2_)', name):
        return 'ROOT_AUDIT_HISTORY', '中', '历史报告归档候选', '根目录修复/验收记录；不影响算法但有追溯价值'
    return 'SOURCE_DOCUMENT_OR_ASSET', '高', '保留', '源代码、配置、论文、文档或素材'


project = []
for p in project_paths:
    m = metadata(p)
    if not m:
        continue
    rel = p.relative_to(ROOT).as_posix()
    cat, conf, action, reason = classify(rel)
    refs = references(p) if cat != 'SOURCE_DOCUMENT_OR_ASSET' else []
    m.update(relative_path=rel, category=cat, confidence=conf, proposed_action=action,
             reason=reason, tracked=rel in tracked, ignored=rel in ignored,
             reference_count=len(refs), reference_locations=' | '.join(refs))
    project.append(m)
by_rel = {m['relative_path']: m for m in project}
SCOPE.append({'path': str(ROOT), 'coverage': '递归全部文件，排除 .git 和本审计目录', 'files': len(project)})

# Exact row-subset checks, limited to main project tables, never whole LNN datasets.
partial_checks = []
for m in project:
    rel = m['relative_path']
    if m['category'] != 'PARTIAL_RESULT_REVIEW':
        continue
    p = ROOT / rel
    target = re.sub(r'_partial(?:_first\d+)?(?=\.csv$)', '', rel)
    if not (ROOT / target).is_file() and target != rel:
        raw_target = target[:-4] + '_raw.csv'
        if (ROOT / raw_target).is_file():
            target = raw_target
    if 'interrupted' in rel:
        target = 'results/tables/position_only_resource_bounded_four_mode_raw.csv'
    row = {'partial_path': str(p), 'complete_path': str(ROOT / target), 'check': 'NO_COMPLETE_FILE'}
    if target != rel and (ROOT / target).is_file():
        try:
            with p.open(encoding='utf-8-sig', newline='') as f:
                a = list(csv.reader(f))
            with (ROOT / target).open(encoding='utf-8-sig', newline='') as f:
                b = list(csv.reader(f))
            same_header = bool(a and b and a[0] == b[0])
            unmatched = collections.Counter(map(tuple, a[1:])) - collections.Counter(map(tuple, b[1:]))
            row.update(partial_rows=max(len(a)-1, 0), complete_rows=max(len(b)-1, 0),
                       same_header=same_header, unmatched_rows=sum(unmatched.values()),
                       check='EXACT_ROW_SUBSET' if same_header and not unmatched else 'DIFFERENT_CONTENT_OR_SCHEMA')
            if row['check'] == 'EXACT_ROW_SUBSET':
                m.update(category='REDUNDANT_PARTIAL_CONFIRMED', confidence='高',
                         proposed_action='可归档/清理候选',
                         reason=f'逐行核实：{row["partial_rows"]} 行均包含于 {target}，表头一致')
                if p.name == 'position_only_resource_bounded_four_mode_partial.csv':
                    m.update(category='RESUME_CHECKPOINT_DUPLICATE',
                             proposed_action='保留：resume 读取入口；需修改读取逻辑后才能移走',
                             reason='与正式 raw 行内容相同，但 exp_position_only_isolated_four_mode 的 load_checkpoint 读取此文件')
        except (OSError, csv.Error) as error:
            row['check'] = 'READ_ERROR'
            ERRORS.append({'path': str(p), 'error': str(error)})
    partial_checks.append(row)

external_roots = []
for p in sorted(PARENT.iterdir()):
    if p == ROOT:
        continue
    if p.name == 'codex_backups':
        for family in sorted(p.iterdir()):
            if family.is_dir() and family.name in {'MobileManipulationPlanning', 'paper_tac'}:
                external_roots.extend((x, 'CODEX_PROJECT_BACKUP') for x in sorted(family.iterdir()))
    elif p.name.startswith('MobileManipulationPlanning'):
        kind = 'SIBLING_ARCHIVE' if p.suffix.lower() == '.zip' else 'SIBLING_PROJECT_BACKUP'
        if p.is_dir() or p.suffix.lower() == '.zip':
            external_roots.append((p, kind))
    elif p.name in {'results', 'stage_output_custom_scene', 'stage_output_custom_scene_v2'}:
        external_roots.append((p, 'EXTERNAL_EXPERIMENT_OUTPUT'))

temp = Path(os.environ.get('TEMP', 'C:/Users/Administrator/AppData/Local/Temp'))
try:
    external_roots.extend((p, 'TEMP_PROJECT_CLEANUP_BACKUP') for p in sorted(temp.iterdir())
                          if p.name.startswith('MobileManipulationPlanning'))
except OSError as error:
    ERRORS.append({'path': str(temp), 'error': str(error)})

external_files, external_summary = [], []
for root, kind in external_roots:
    paths = walk(root)
    group = []
    for p in paths:
        m = metadata(p)
        if not m:
            continue
        rel = p.relative_to(root).as_posix() if root.is_dir() else p.name
        # Most snapshots use project-relative names; some use files/ prefixes.
        mapped = re.sub(r'^(?:files|before|backup|source|project_snapshot|restore_files)/', '', rel)
        if root.parent.name == 'paper_tac':
            mapped = 'paper_tac/' + mapped
        current = by_rel.get(mapped)
        comparison = 'NO_SAME_PATH_IN_CURRENT'
        if '.git' in Path(rel).parts:
            comparison = 'GIT_METADATA_NOT_COMPARED'
        elif current:
            if m['bytes'] != current['bytes']:
                comparison = 'DIFFERENT_SIZE'
            elif m['bytes'] <= 8 * 2 ** 20:
                first, second = sha(p), sha(ROOT / mapped)
                comparison = 'IDENTICAL_SHA256' if first and first == second else 'DIFFERENT_SHA256'
            else:
                comparison = 'SAME_SIZE_CONTENT_NOT_HASHED'
        m.update(scope_root=str(root), scope_kind=kind, relative_path=rel,
                 current_relative_path=mapped if current else '', comparison=comparison)
        group.append(m)
        external_files.append(m)
    counts = collections.Counter(m['comparison'] for m in group)
    mapped_sources = sum(m['current_relative_path'].endswith('.m') for m in group)
    patch_files = sum(m['relative_path'].endswith(('.patch', '.diff')) and m['bytes'] > 0 for m in group)
    summary = {'absolute_path': str(root), 'kind': kind, 'files': len(group),
               'bytes': sum(m['bytes'] for m in group),
               'newest_file_utc8': max((m['mtime_utc8'] for m in group), default=''),
               'same_current_sha256': counts['IDENTICAL_SHA256'],
               'different_current': counts['DIFFERENT_SIZE'] + counts['DIFFERENT_SHA256'],
               'same_size_unverified': counts['SAME_SIZE_CONTENT_NOT_HASHED'],
               'not_mapped_to_current': counts['NO_SAME_PATH_IN_CURRENT'],
               'git_metadata_files': counts['GIT_METADATA_NOT_COMPARED'],
               'mapped_matlab_source_files': mapped_sources,
               'nonempty_patch_files': patch_files,
               'recovery_material': '源文件副本/快照；完整性未承诺' if mapped_sources else
                   '差异补丁及记录；需对应基线' if patch_files else '报告/数据/压缩包/其他材料，需逐项核实'}
    external_summary.append(summary)
    SCOPE.append({'path': str(root), 'coverage': '递归文件元数据；不执行恢复脚本/补丁', 'files': len(group)})

# Config/runtime backups: root metadata plus backup names in owned config folders.
# Excludes sandbox secrets, auth, sessions, plugin payloads, and all configuration values.
codex_rows = []
try:
    root_children = list(CODEX.iterdir())
except OSError as error:
    ERRORS.append({'path': str(CODEX), 'error': str(error)})
    root_children = []
codex_backup_files = [p for p in root_children if p.is_file() and re.search(r'(?i)\.bak|backup|before|restore', p.name)]
for folder in ('rules', 'rollout-migrations'):
    codex_backup_files.extend(p for p in walk(CODEX / folder)
                              if re.search(r'(?i)\.bak|backup|before|restore', p.name))
for p in sorted(set(codex_backup_files)):
    m = metadata(p)
    if not m:
        continue
    if p.name == 'tmp_large_scale_backup_path.txt':
        purpose = '备份位置指针；不是项目或配置内容备份'
        active_name = ''
    else:
        active_name = p.name.split('.bak')[0]
        purpose = ('代理修复前环境文件' if p.name.startswith('.env.bak') else
                   'Codex 配置历史快照' if p.name.startswith('config.toml.bak') else
                   '旧会话状态数据库快照' if p.name.startswith('state_') else
                   '应用状态自动备份' if p.name == '.codex-global-state.json.bak' else '待核实历史备份')
    active = p.parent / active_name if active_name else None
    same = 'NOT_COMPARED_LIVE_STATE'
    if active and active.is_file() and active_name in {'config.toml', '.env'}:
        same = 'IDENTICAL_SHA256' if sha(p) == sha(active) else 'DIFFERENT_SHA256'
    m.update(purpose=purpose, current_path=str(active) if active else '', comparison=same,
             proposed_action='仅列清单；保留可用回滚点，不自动恢复/删除')
    codex_rows.append(m)
SCOPE.append({'path': str(CODEX), 'coverage': '根层文件名元数据 + rules/rollout-migrations 备份名；未扫描插件/会话内容；配置值不导出', 'files': len(codex_rows)})

archive_summary, archive_members = [], []
archives = [ROOT / m['relative_path'] for m in project if m['relative_path'].endswith('.zip')]
archives += [Path(m['absolute_path']) for m in external_files if m['absolute_path'].lower().endswith('.zip')]
for p in sorted(set(archives)):
    try:
        with zipfile.ZipFile(p) as z:
            members = [i for i in z.infolist() if not i.is_dir()]
            archive_summary.append({'absolute_path': str(p), 'bytes': p.stat().st_size,
                                    'members': len(members), 'uncompressed_bytes': sum(i.file_size for i in members),
                                    'earliest_member_time': str(min((i.date_time for i in members), default='')),
                                    'latest_member_time': str(max((i.date_time for i in members), default='')),
                                    'verification': 'CENTRAL_DIRECTORY_ONLY; NO_EXTRACTION; NO_FULL_CRC_TEST'})
            archive_members.extend({'archive_path': str(p), 'member_path': i.filename,
                                    'bytes': i.file_size, 'compressed_bytes': i.compress_size,
                                    'crc32': f'{i.CRC:08x}'} for i in members)
    except (OSError, zipfile.BadZipFile) as error:
        ERRORS.append({'path': str(p), 'error': str(error)})

# Exact duplicates only within the active project, hash only same-sized files.
sizes = collections.defaultdict(list)
for m in project:
    if m['bytes']:
        sizes[m['bytes']].append(m)
duplicates = []
group_id = 0
for size, rows in sizes.items():
    if len(rows) < 2:
        continue
    digests = collections.defaultdict(list)
    for m in rows:
        digest = sha(Path(m['absolute_path']))
        if digest:
            digests[digest].append(m)
    for digest, equal in digests.items():
        if len(equal) > 1:
            group_id += 1
            for m in equal:
                duplicates.append({'duplicate_group': group_id, 'sha256': digest, 'bytes': size,
                                   'absolute_path': m['absolute_path'], 'category': m['category'],
                                   'note': '内容相同不代表可删除；论文素材副本和算法对照输出可能有意保留'})

directory_rows = []
for prefix in sorted({m['relative_path'].split('/')[0] for m in project}):
    group = [m for m in project if m['relative_path'].split('/')[0] == prefix]
    directory_rows.append({'relative_path': prefix, 'files': len(group), 'bytes': sum(m['bytes'] for m in group)})
category_rows = []
for category in sorted({m['category'] for m in project}):
    group = [m for m in project if m['category'] == category]
    category_rows.append({'category': category, 'files': len(group), 'bytes': sum(m['bytes'] for m in group),
                          'action': group[0]['proposed_action']})
source_rows = []
for rel, body in source_texts.items():
    p = Path(rel)
    callers = [k for k, v in source_texts.items() if k != rel and re.search(r'\b' + re.escape(p.stem) + r'\b', v)]
    source_rows.append({'relative_path': rel, 'absolute_path': str(ROOT / rel), 'bytes': by_rel[rel]['bytes'],
                        'name_reference_files': len(callers), 'reference_files': ' | '.join(callers),
                        'method': '词法名字引用，含注释/自定义入口；不是完整 MATLAB 动态调用图'})

write_csv('project_files.csv', project)
write_csv('candidate_files.csv', [m for m in project if m['category'] not in {'SOURCE_DOCUMENT_OR_ASSET', 'RESEARCH_EVIDENCE_RETAIN'}])
write_csv('category_summary.csv', category_rows)
write_csv('directory_summary.csv', directory_rows)
write_csv('source_index.csv', source_rows)
write_csv('partial_result_checks.csv', partial_checks)
write_csv('external_backup_roots.csv', external_summary)
write_csv('external_backup_files.csv', external_files)
write_csv('codex_config_backups.csv', codex_rows)
write_csv('archive_summary.csv', archive_summary)
write_csv('archive_members.csv', archive_members)
write_csv('exact_duplicates.csv', duplicates)
write_csv('scan_scope.csv', SCOPE)
summary = {'generated_utc8': dt.datetime.now(TZ).isoformat(timespec='seconds'),
           'project_root': str(ROOT), 'head': git('rev-parse', 'HEAD').strip(),
           'branch': git('branch', '--show-current').strip(),
           'project_files': len(project), 'project_bytes_excluding_git_and_audit': sum(m['bytes'] for m in project),
           'source_files': len(source_rows), 'matlab_files': sum(m['relative_path'].endswith('.m') for m in project),
           'external_roots': len(external_summary), 'external_files': len(external_files),
           'external_bytes': sum(m['bytes'] for m in external_files),
           'codex_backups': sum(m['current_path'] != '' for m in codex_rows),
           'codex_backup_pointer_files': sum(m['current_path'] == '' for m in codex_rows),
           'codex_backup_bytes': sum(m['bytes'] for m in codex_rows if m['current_path']),
           'duplicate_groups': group_id, 'archives': len(archive_summary),
           'categories': category_rows, 'errors': ERRORS,
           'limits': ['只读审计；仅在本审计目录写出报告', '未运行 MATLAB 或正式实验',
                      '外部备份同路径且不大于 8 MiB 的文件才比较 SHA256；其他文件明确标注未核实',
                      '配置备份仅元数据和与当前文件的相等性；不导出任何配置值',
                      '压缩包仅中央目录读取，未做完整解压/CRC 完整性测试',
                      '未扫描整盘、无关兄弟项目、Codex 会话及插件内部；不宣称全盘穷尽']}
(OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(summary, ensure_ascii=True, indent=2))
