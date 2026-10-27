"""Verify cleanup results and document recovery/retained dependencies."""
import collections
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent.parent


def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda: f.read(2 ** 20), b''):
            h.update(chunk)
    return h.hexdigest()


def link(p, label=None):
    p = Path(p)
    return f'[{label or p.name}](<{p.as_posix()}>)'


def gib(n):
    return f'{n / 2 ** 30:,.2f} GiB'


def mib(n):
    return f'{n / 2 ** 20:,.2f} MiB'


def table(headers, rows):
    return '\n'.join(['| ' + ' | '.join(headers) + ' |', '| ' + ' | '.join('---' for _ in headers) + ' |'] +
                     ['| ' + ' | '.join(str(x).replace('|', '\\|') for x in row) + ' |' for row in rows])


def main():
    plan = json.loads((OUT / 'cleanup_plan.json').read_text(encoding='utf-8'))
    baseline = json.loads((OUT / 'baseline_workspace_manifest.json').read_text(encoding='utf-8'))
    targets = [r['absolute_path'] for r in plan['external_roots'] + plan['project_files'] + plan['codex_files']] + [plan['temporary_root']]
    remaining = [p for p in targets if Path(p).exists()]
    deleted_project = {r['relative_path'] for r in plan['project_files']}
    mismatches = []
    checked = 0
    source_count = 0
    for row in baseline['files']:
        if row['path'] in deleted_project:
            continue
        p = ROOT / row['path']
        if not p.is_file() or sha(p) != row['sha256']:
            mismatches.append(row['path'])
        checked += 1
        if row['path'].endswith('.m'):
            source_count += 1
    protected = [r['path'] for r in plan['protected_project_files']]
    protected_ok = all((ROOT / r['path']).is_file() and sha(ROOT / r['path']) == r['sha256'] for r in plan['protected_project_files'])
    archive = Path(plan['retained_archive'])
    history_ok = archive.is_file() and sha(archive) == plan['retained_archive_sha256']
    current_configs = ['config.toml', '.env', 'config.toml.bak-too-many-pending-20260915-1345',
                       '.env.bak-20261027-093020-proxy-fix', 'state_5.sqlite.bak-before-gpt55-low-20260903-0945']
    configs_ok = all((Path(plan['codex_root']) / name).is_file() for name in current_configs)
    with (OUT / 'deletion_log.csv').open(encoding='utf-8-sig', newline='') as f:
        log = list(csv.DictReader(f))
    free_after = {'D': shutil.disk_usage(ROOT.anchor).free, 'C': shutil.disk_usage(Path(plan['codex_root']).anchor).free}
    source_changes = subprocess.check_output(['git', 'diff', '--name-only', '--', '*.m', '*.tex', '*.bib'], cwd=ROOT).decode('utf-8').splitlines()
    result = {'verified_utc8': dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(timespec='seconds'),
              'remaining_planned_targets': remaining, 'retained_baseline_files_checked': checked,
              'retained_file_hash_mismatches': mismatches, 'unchanged_matlab_source_files': source_count,
              'protected_experiment_files': protected, 'protected_files_ok': protected_ok,
              'historical_archive_ok': history_ok, 'current_configs_and_recent_rollbacks_ok': configs_ok,
              'source_changes_from_precleanup_commit': source_changes,
              'deletion_log_entries': len(log), 'summary': plan['summary'],
              'skipped_permission_roots': plan['skipped_roots'], 'drive_free_before': plan['drive_free_before'],
              'drive_free_after': free_after,
              'observed_free_space_increase': {d: free_after[d] - plan['drive_free_before'][d] for d in free_after}}
    result['passed'] = not remaining and not mismatches and protected_ok and history_ok and configs_ok and not source_changes
    (OUT / 'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    if not result['passed']:
        print(json.dumps(result, ensure_ascii=True, indent=2))
        raise RuntimeError('Cleanup verification requires attention; no success report written.')
    groups = collections.defaultdict(list)
    for row in plan['project_files']:
        groups[row['category']].append(row)
    deleted_file_count = plan['summary']['external_files'] + plan['summary']['project_files'] + len(plan['codex_files']) + len(plan['temporary_files'])
    lines = [
        '# 清理执行报告（2026-10-27）',
        '用户已明确授权删除无用备份和实验残留。本次已执行清理，并保留云端完整保存点和去重历史归档。',
        f'清理前保存点：[{baseline["commit"][:7]}]({plan["baseline"]["release_url"]})。所有 5 个云端附件在删除前核对了大小和 GitHub 返回的 SHA-256。',
        '## 清理结果',
        table(['类别', '数量', '原逻辑体积', '处置'], [
            ['旧备份和散落实验输出', f'{plan["summary"]["external_roots"]} 个位置 / {plan["summary"]["external_files"]:,} 个文件', gib(plan['summary']['external_logical_bytes']), '原散落目录/压缩包已删除；全部内容可由去重历史归档和云端基线重建'],
            ['项目内实验残留', f'{plan["summary"]["project_files"]} 个文件', mib(plan['summary']['project_logical_bytes']), '已删除；每个文件均有完全相同的云端保存点副本'],
            ['过期 Codex 配置备份', '4 个', '14,915 B', '已删除；保留最近配置回滚点、代理修复前环境文件和状态库回滚点'],
            ['失效备份路径指针', '1 个', '154 B', '原指向已归档的旧备份目录，已删除'],
            ['已上传的本地临时快照', f'{len(plan["temporary_files"])} 个文件', mib(plan['summary']['temporary_bytes']), '已删除；云端附件保留'],
            ['集中保留的独有历史归档', f'{plan["summary"]["unique_history_payloads"]} 个去重内容对象', gib(plan['retained_archive_bytes']), '保留 history_unique.zip；减少重复占用']]),
        f'共移除 {deleted_file_count:,} 个原文件（目录删除按完整文件清单计数）。扣除历史归档 ZIP 后，净减少逻辑数据约 **{gib(plan["summary"]["net_logical_bytes_reclaimed"])}**；另有少量清单和报告文件。磁盘可用空间变化见 verification.json，受文件系统分配及其他进程写入影响。',
        table(['项目内已删除类别', '文件数', '体积'], [(k, len(v), mib(sum(r['bytes'] for r in v))) for k, v in sorted(groups.items())]),
        '项目内删除包括旧 LNN 数据/模型/导出及对应缓存与 9 份旧学习实验 partial，37 份 GUI/渲染试跑图片，5 份被正式结果覆盖的 partial CSV，50 份历史日志和两个重复的本地项目/论文 ZIP。正式 raw/summary 表、论文成品、源代码和冻结复现输入保留。',
        '## 保留项与验证',
        f'- 383 个原 MATLAB 源文件均存在且 SHA-256 与清理前一致；所有 {checked} 个未列入删除清单的原项目文件均逐个核对，无内容变化。',
        '- 8 个 P3/P4 冻结状态、3 个 Position-only seed state、resource_bounded 续跑 CSV，以及两份没有重复覆盖依据的中断/部分结果均保留并核对哈希。',
        '- 当前 Codex config.toml/.env 未作为删除目标；保留最近 config.toml 备份、今天代理修复前 .env 备份、旧状态库备份及应用自动状态备份。',
        '- 没有运行会重写结果的 MATLAB 实验；本次验证覆盖删除范围、源文件哈希、复现依赖、ZIP 完整性和恢复材料。',
        '## 独有历史记录如何恢复',
        f'归档目录：`{archive.parent}`。',
        '- ' + link(archive, 'history_unique.zip') + f'（{gib(plan["retained_archive_bytes"])}）',
        '- ' + link(archive.parent / 'history_manifest.json', '完整原路径与内容映射') + '；' + link(archive.parent / 'SHA256SUMS.txt', '归档 SHA-256') + '；' + link(archive.parent / 'restore_history.py', '恢复工具'),
        f'归档保存 {plan["summary"]["external_files"]:,} 个旧文件的完整路径映射；其中 {plan["summary"]["external_files_from_cloud_baseline"]:,} 个文件的内容已经存在于云端完整基线中，其余内容以去重对象保存。旧 .git 历史、补丁、报告和较早压缩包的独有内容也在此范围内。',
        '恢复时从清理前 Release 下载自定义 workspace ZIP，并准备好 history_unique.zip。使用新的空目录：',
        '```powershell\npython restore_history.py --history history_unique.zip --baseline MobileManipulationPlanning-checkpoint-20261027-095303-workspace.zip --destination D:\\recovered_history_new\n```',
        '恢复工具逐文件校验 SHA-256，不写回原备份位置。历史归档依赖云端基线，二者共同构成恢复材料；该历史归档当前保存在本机，并未上传其独有内容。',
        '## 尚未删除的权限例外',
        f'`{plan["skipped_roots"][0]["absolute_path"]}` 中的旧 `.pytest_cache` 保留原位。原审计记录为 6 个文件、1,259 字节。Administrator 无法列出该缓存的内容，限定在此缓存目录的所有权修正也被 Windows 拒绝，因此本轮未对它实施删除。该限制来自 Windows 文件权限，并非自动审批拒绝。其他清理已完成。',
        '## 明细',
        '- ' + link(OUT / 'planned_deletions.csv', '逐文件删除清单（含原路径、大小、SHA-256 和恢复来源）'),
        '- ' + link(OUT / 'deletion_log.csv', '实际删除日志'),
        '- ' + link(OUT / 'cleanup_plan.json', '精确执行计划') + '；' + link(OUT / 'verification.json', '清理后验证结果'),
        '- ' + link(OUT / 'cloud_verification.json', '删除前云端保存点验证') + '；' + link(OUT / 'baseline_workspace_manifest.json', '云端基线文件清单'),
        '- ' + link(OUT / 'preparation_verification.json', '历史归档和恢复来源检查'),
        '此前 docs/cleanup_audit_20261027/ 是删除前的盘点快照，其中原备份路径属于历史记录；本报告反映执行后的状态。',
    ]
    (OUT / 'README.md').write_text('\n\n'.join(lines) + '\n', encoding='utf-8')
    print(json.dumps({'passed': True, 'deleted_original_files': deleted_file_count,
                      'net_logical_gib': round(plan['summary']['net_logical_bytes_reclaimed'] / 2**30, 2),
                      'retained_history_gib': round(plan['retained_archive_bytes'] / 2**30, 2),
                      'unchanged_original_matlab_files': source_count, 'unchanged_retained_original_files': checked,
                      'remaining_permission_exceptions': len(plan['skipped_roots']),
                      'observed_free_space_increase': result['observed_free_space_increase'],
                      'report': str(OUT / 'README.md')}, ensure_ascii=True))


if __name__ == '__main__':
    main()
