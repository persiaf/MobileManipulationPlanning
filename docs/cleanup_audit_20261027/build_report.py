"""Build the human-readable inventory from the read-only audit CSVs."""
import collections
import csv
import json
import os
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent.parent
PARENT = ROOT.parent


def read(name):
    with (OUT / name).open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def link(path, label=None):
    p = Path(path)
    return f'[{label or p.name}](<{p.as_posix()}>)'


def table(headers, rows):
    def cell(x):
        return str(x).replace('|', '\\|').replace('\n', '<br>')
    return '\n'.join(['| ' + ' | '.join(map(cell, headers)) + ' |',
                      '| ' + ' | '.join('---' for _ in headers) + ' |'] +
                     ['| ' + ' | '.join(map(cell, row)) + ' |' for row in rows])


def mib(n):
    return f'{int(n) / 2 ** 20:,.2f} MiB'


def gib(n):
    return f'{int(n) / 2 ** 30:,.2f} GiB'


summary = json.loads((OUT / 'summary.json').read_text(encoding='utf-8'))
large = json.loads((OUT / 'large_copy_summary.json').read_text(encoding='utf-8'))
project = read('project_files.csv')
external = read('external_backup_roots.csv')
external_files = read('external_backup_files.csv')
configs = read('codex_config_backups.csv')
partials = read('partial_result_checks.csv')
large_groups = read('large_copy_groups.csv')
sources = read('source_index.csv')
cats = {r['category']: r for r in summary['categories']}
lines = [
    '# 代码框架、实验残留与修复备份盘点（2026-10-27）',
    f'扫描时间：{summary["generated_utc8"]}；时区 UTC+08:00。',
    '**本轮只整理清单和核对证据，没有删除、移动、恢复任何原文件，也没有修改项目配置。** 所有新增文件仅位于本报告目录。',
    f'项目根目录：`{ROOT}`。Git 分支 `{summary["branch"]}`；HEAD `{summary["head"]}`。工作区存在大量未提交修改和未跟踪文件，详见 ' + link(OUT / 'git_status_at_scan.txt') + '。',
    '## 1. 盘点结论',
    table(['范围', '数量', '逻辑体积', '说明'], [
        ['当前项目', summary['project_files'], gib(summary['project_bytes_excluding_git_and_audit']), '排除 .git 与本审计目录；383 个 MATLAB 文件'],
        ['外部备份/散落输出位置', summary['external_roots'], gib(summary['external_bytes']), f'{summary["external_files"]:,} 个文件；含完整快照、增量补丁、压缩包和试跑输出'],
        ['已核实的大文件相同副本', large['verified_external_copies_equal_to_current'], gib(large['equal_external_logical_bytes']), '计入上行外部体积；SHA-256 一致，不是额外相加的空间'],
        ['Codex 自身备份', summary['codex_backups'], mib(summary['codex_backup_bytes']), '5 份 config.toml、1 份 .env、1 份旧状态库、1 份应用状态自动备份；另有 1 个路径指针']]),
    '占空间的主要原因是多轮快照反复复制同一批原始数据。313 份大文件副本中，P3/P4 和 Position-only 状态在外部各有 10 份相同副本，主要旧学习数据各有 11 份。**22.78 GiB 是相同外部文件的逻辑体积，不代表这些备份全部可以删除，也不是实测可释放磁盘空间。**',
    '可以优先处理的活动目录残留是：旧 LNN 研究数据、GUI/渲染试跑图片、5 份已被正式 CSV 覆盖的 partial 文件，以及历史日志。源代码、当前论文素材、P3/P4/P5 依赖和独有的中断结果单独保留。',
    '## 2. 当前代码框架和实际路径',
    table(['职责', '真实位置', '结论'], [
        ['默认入口', link(ROOT / 'main_mobile_manipulation_planning.m'), 'stage1 → stage2 → stage3；可选 benchmark/video'],
        ['统一实验入口', link(ROOT / 'experiment_manager/run_selected_experiment.m'), '注册表/配置构建器驱动；5 个已注册 profile'],
        ['GUI', link(ROOT / 'experiment_manager/launch_mobile_manipulation_experiment_manager.m'), '单独加入 manager 与 pipeline 路径后初始化'],
        ['路径初始化', link(ROOT / 'pipeline/init_paths.m'), '实际位于 pipeline，根目录没有 init_paths.m'],
        ['配置', link(ROOT / 'config'), 'scene/robot/planner + resolve_effective_config；可视化规范配置在 visualization'],
        ['场景', link(ROOT / 'environment/build_scene.m') + '；' + link(ROOT / 'utils/build_scene_from_cfg.m'), '默认构建和外部配置构建分工不同'],
        ['候选库', link(ROOT / 'planning_strategy/common/build_candidate_library.m'), '配合 docking、manipulator、collision'],
        ['三种策略', link(ROOT / 'planning_strategy/run_planning_strategy.m'), 'bottom_up / top_down / bidirectional'],
        ['全局路径', link(ROOT / 'global_planner/run_path_planner.m') + '；' + link(ROOT / 'pipeline/stage3_global_planning.m'), '算法分派、dense/sparse 路由和真实路径拼接'],
        ['动态修复', link(ROOT / 'replanning/local_replan.m') + '；' + link(ROOT / 'replanning/execute_replanning_action.m'), '与 P5 实验、状态一致性/增量修复测试相连'],
        ['实机边界', link(ROOT / 'integration') + '；' + link(ROOT / 'calibration'), '未在默认流程启用不代表无用'],
        ['实验与测试', link(ROOT / 'experiments') + '；' + link(ROOT / 'tests'), '当前逐文件索引见 source_index.csv'],
        ['论文', link(ROOT / 'paper_tac/main.tex'), 'sections/、figures/、references.bib 和最终 PDF 保留']]),
    '### 2.1 路径问题',
    '1. `README.md:18` 和 `docs/QUICK_START.md:6` 在根目录直接调用 `init_paths`，对未预置 MATLAB 路径的新会话不完整。`init_paths.m` 在 `pipeline/`；其 folders 列表还没有 `experiment_manager`。这也使“先运行旧 main，再直接调用统一 runner”不能保证 runner 可见。GUI launcher 和 builder 内部虽有部分补齐逻辑，但第一次解析函数仍需要路径入口。',
    '以下是与现有文件布局一致的初始化方式；本轮未执行实验：',
    "```matlab\nproject_root = '" + ROOT.as_posix() + "';\ncd(project_root);\naddpath(fullfile(project_root, 'pipeline'));\naddpath(fullfile(project_root, 'experiment_manager'));\ninit_paths(project_root);\n```",
    '2. `docs/CODE_ARCHITECTURE.md:8` 写的 `environment/build_scene_from_cfg.m` 不存在，实际文件为 `utils/build_scene_from_cfg.m`；`:9` 写的 `docking/build_candidate_library.m` 不存在，实际文件为 `planning_strategy/common/build_candidate_library.m`。部分旧清单仍写 309 个源文件或 76 个测试，不能代表当前 383 个 MATLAB 文件。',
    '3. `run_selected_experiment.m:19-20` 的默认输出是相对路径 `results/managed_experiments/...`，不绑定 project_root。同级目录确实另有 `results/managed_experiments/...`，含一个 record.mat 和 5 张图，共 2.44 MiB；这与从父目录运行时的路径行为一致，但仅凭输出不能证明当时的启动命令。后续运行宜显式给绝对 output_dir。',
    '4. `../stage_output_custom_scene/` 和 `../stage_output_custom_scene_v2/` 各只剩一份 1,490 字节日志，其余目录为空。两份日志均记录了完成结果，不是完整实验归档。',
    '5. `paper_tac/build/` 里的两个 PDF 都是文稿成品，不能按 build 目录一概当作缓存。论文 figures 中的素材副本也不能仅因 SHA-256 相同就移除。',
    '### 2.2 各代码目录规模',
    table(['目录', 'MATLAB 文件数'], [(d, sum(Path(p['relative_path']).parts[0] == d and p['relative_path'].endswith('.m') for p in project))
          for d in sorted({Path(p['relative_path']).parts[0] for p in project if '/' in p['relative_path'] and p['relative_path'].endswith('.m')})]),
    '完整源文件路径、名字引用位置见 ' + link(OUT / 'source_index.csv') + '。该索引采用词法引用，包含注释和测试，不能据此认定“零引用函数”可以删除。',
    '## 3. 实验残留分类',
    table(['类别', '文件数', '体积', '建议'], [
        [k, cats[k]['files'], mib(cats[k]['bytes']), cats[k]['action']]
        for k in ['LEGACY_LNN_EXPERIMENT', 'LEGACY_PAPER_CACHE', 'SMOKE_VISUAL_OUTPUT', 'REDUNDANT_PARTIAL_CONFIRMED',
                  'HISTORICAL_LOG', 'HARDWARE_DEBUG_HISTORY', 'ROOT_AUDIT_HISTORY', 'PROJECT_ARCHIVE',
                  'PARTIAL_RESULT_REVIEW', 'RESUME_CHECKPOINT_DUPLICATE', 'P3P4_FROZEN_DEPENDENCY', 'POSITION_STATE_DEPENDENCY']]),
    '### 3.1 已脱离当前主要研究方向的旧数据',
    f'- `{ROOT / "paper_tac/results/lnn"}`：84 个文件，699.79 MiB。包含历史数据集、模型、CSV 导出、quick_check 和临时 MAT。`paper_tac/README.md:87` 明确当前稿件不把 LNN 作为主要贡献；`docs/cleanup_audit_20260831.md` 和 `docs/maintenance_followup_20260831.md` 也已把它列为历史产物。建议整个实验族归档，避免逐个拆散训练/验证/模型配套关系。',
    f'- `{ROOT / "paper_tac/results/cache/lnn_cases/lnn_stress/seed_31/case_cache.mat"}`：47.66 MiB。归入旧研究缓存；当前源码未找到完整再生工具，故不把“cache”字样当作可无条件重建的证明。',
    '这两项合计约 747.45 MiB。只把它们移出活动树并不能消除各历史快照中的副本，外部副本还需要单独整理。',
    '### 3.2 已核实被正式结果覆盖的 partial 文件',
    table(['partial 文件', '正式文件', '行数', '处置'], [
        [Path(r['partial_path']).name, Path(r['complete_path']).name,
         f'{r.get("partial_rows", "")} / {r.get("complete_rows", "")}',
         '保留续跑入口' if Path(r['partial_path']).name == 'position_only_resource_bounded_four_mode_partial.csv' else '可归档/清理候选']
        for r in partials if r['check'] == 'EXACT_ROW_SUBSET']),
    '核对采用相同表头 + CSV 行的多重集合包含关系，不只是文件名匹配。其中 4 对还是 SHA-256 完全相同：full_validation、heldout_coverage、resource_bounded_four_mode 和 heldout_failures；最后一对为空结果表。',
    '`position_only_resource_bounded_four_mode_partial.csv` 虽然 24 行与 raw 相同，仍是 `exp_position_only_isolated_four_mode.m` 中 `load_checkpoint` 的续跑输入。因此 6 份匹配的 partial 中，只把其余 5 份列为清理候选。',
    '### 3.3 需要保留的中断证据',
    '- `position_only_isolated_four_mode_partial.csv`：找不到对应完整结果，保留；不能以另一个 resource_bounded 协议的结果替代。',
    '- `position_only_resource_bounded_full_audit_interrupted_9rows.csv`：同表头，但 9 行均不在后来的 24 行 raw 中，属于独有的历史批次；建议与其崩溃/重试日志归档，不按重复结果删除。',
    '- `position_only_resource_bounded_console_retry1.log` 和 `retry2.log` 均记录 MATLAB `0xc0000374, Heap corruption`。它们是失败诊断记录，不能当作有效完成的实验。',
    '### 3.4 应保留的复现输入',
    f'- `results/tables/p3p4_state_*.mat`：8 个，{mib(cats["P3P4_FROZEN_DEPENDENCY"]["bytes"])}。`exp_dynamic_local_replanning_p5.m:44-49` 默认读取 L2 seed 2026/2027/2028 状态；其余 L1/L3 仍属 P3/P4 冻结复现状态。',
    f'- `results/tables/position_only_resource_bounded_seed{{1,3,5}}_state.mat`：3 个，{mib(cats["POSITION_STATE_DEPENDENCY"]["bytes"])}。`exp_position_only_isolated_four_mode.m:125-150` 的 resume 模式会读取。',
    '- 正式 raw/summary/task/trace 表、cafeteria 路径与 footprint 验证、当前论文 figures/ 和 results/paper/ 支撑报告保留。失败实验也可能支撑负面结论或消融解释，不仅按“失败”二字删除。',
    '### 3.5 试跑图片和历史报告',
    'GUI/渲染试跑图片分布在 `results/managed_gui_smoke/`、`managed_gui_smoke2/`、`results/figures/unified_smoke/`、`visualization_validation/`、`visualization_report/`，共 37 个文件、11.88 MiB。它们是派生的验证输出；保留测试源码后可优先从活动目录移出。',
    '根目录 21 份 S2/footprint/config/checkpoint 等审计文件、50 份历史日志和 hardware_matched_lab 的 32 份轮次记录建议按日期/修复轮次归档。原路径、体积、修改时间和引用位置都在 ' + link(OUT / 'candidate_files.csv') + '。',
    '## 4. 外部备份和散落输出：完整位置清单',
    '以下每行是一个扫描根位置，下面的所有文件都已登记在 ' + link(OUT / 'external_backup_files.csv') + '。表中的体积包含对应目录内的全部文件；若同时保留目录和 ZIP，两者分别计数。补丁恢复依赖对应 HEAD 和原有未提交状态，不能用“有 changes.patch”推断能够独立恢复完整工作区。',
    table(['类别', '位置数', '文件数', '体积'], [
        [kind, len(rows), sum(int(r['files']) for r in rows), gib(sum(int(r['bytes']) for r in rows))]
        for kind in sorted({r['kind'] for r in external})
        for rows in [[r for r in external if r['kind'] == kind]]]),
]
for kind in sorted({r['kind'] for r in external}):
    rows = [r for r in external if r['kind'] == kind]
    lines += ['### ' + kind,
              table(['完整路径', '文件数', '体积', '材料类型'],
                    [(link(r['absolute_path'], r['absolute_path']), r['files'], mib(r['bytes']), r['recovery_material']) for r in rows])]

lines += [
    '### 4.1 重复大文件的证据',
    '逐文件读取并计算 SHA-256，比对期间文件大小和修改时间均稳定；共核实当前 30 个大文件与 313 份外部副本相同。完整绝对路径、哈希及文件 ID 见 ' + link(OUT / 'large_copy_verification.csv') + '。以下数量只计算外部副本，不包含当前项目自身。',
    table(['当前文件', '外部相同副本数', '外部相同副本逻辑体积'],
          [(r['current_relative_path'], r['confirmed_equal_external_files'], gib(r['equal_external_logical_bytes'])) for r in large_groups]),
    '最值得合并的是 2026-09-23 的三个约 2.29 GiB 完整副本，以及 2026-10-19/20 的多次约 2.3–2.5 GiB 快照。应先保留关键修复前后的独有源文件、补丁、HEAD 和报告，再把大数据改为共享的一份证据归档。不能直接删掉整个快照来追求表里的重复体积。',
    '部分 10 月最新目录主要由报告、诊断脚本、before/after 清单和数据组成，未发现与当前路径匹配的源文件副本。它们是修复记录，不一定是能够直接覆盖还原的完整备份。',
    '当前项目内部另核实 13 组相同内容文件，详见 ' + link(OUT / 'exact_duplicates.csv') + '。例如 `lnn_boundary_medium_model.mat` 与 `_v2.mat` 相同；但论文素材拷贝、用于两个实验口径的相同汇总表需要保留各自入口。',
    '### 4.2 压缩包',
    f'共读取 {summary["archives"]} 个 ZIP 的中央目录，完整清单在 ' + link(OUT / 'archive_summary.csv') + '，每个包的文件成员在 ' + link(OUT / 'archive_members.csv') + '。没有解压、执行或做完整 CRC 测试。',
    '项目根目录 `MobileManipulationPlanning.zip` 是 47.26 MiB 的旧快照，内部 911 个文件、未压缩体积约 48.37 MiB，远小于当前 2.31 GiB 活动树，不能代替当前完整备份。该包还在外部出现 6 份 SHA-256 相同副本。',
    '同级 `(2).zip`、`(3).zip`、`(4).zip`、`pre_refactor_20260720.zip` 和早期 `MobileManipulationPlanning.zip` 共约 1.90 GiB；它们并非同一个文件的同名副本，需按来源和内容选择保留点。',
    '## 5. Codex 自身配置修复前备份',
    '位置：`C:\\Users\\Administrator\\.codex`。只登记元数据以及配置备份与当前文件是否相同；没有导出配置值、代理地址、认证信息或数据库内容。',
    table(['完整文件路径', '体积', '用途', '与当前文件比较'],
          [(link(r['absolute_path'], r['absolute_path']), r['bytes'] + ' B', r['purpose'], r['comparison']) for r in configs]),
    '其中 `.env.bak-20261027-093020-proxy-fix` 最直接对应本次日期的代理修复前备份；文件名记载备份操作时间，mtime 仍保留 2026-07-13 的源文件时间，二者不能混为一谈。其他配置文件也存在同样的时间差，因此清单同时保留完整文件名和 mtime。',
    '5 份 config.toml 备份以及 .env 备份都与当前文件不同；本轮没有恢复它们。应用自动备份和旧状态库可能涉及历史会话/状态，不以“大于配置文件”或“有 .bak 后缀”为删除依据。`tmp_large_scale_backup_path.txt` 只是指向 20260902 大规模实验 checkpoint 的路径指针，不算第 9 份配置备份。',
    '## 6. 建议整理顺序',
    '1. 先把本报告作为索引使用；当前已有未提交修改，先建立一个包含当前源码和独有数据的可验证保留点。',
    '2. 优先整理 5 份已覆盖 partial 和 37 张试跑图；这些体积不大，但分类最清晰。保留被 resume 读取的 checkpoint。',
    '3. 把旧 LNN 实验族和对应缓存整体迁到历史研究归档，连同原实验说明与数据配套关系；将失败/中断批次和日志按日期收纳。',
    '4. 合并外部完整快照中已核实相同的原始大文件，只保留一份基准数据及其哈希；各轮代码差异、恢复说明、报告和关键版本保留。快照中的补丁与源文件存在独有内容，不整目录盲删。',
    '5. 单独管理 Codex 配置回滚点，至少保留当前已知可用版本与所需修复前版本；不混入 MATLAB 实验归档。',
    '以上是后续整理建议，本轮没有实施文件迁移或清理。',
    '## 7. 清单入口与核验边界',
    table(['文件', '用途'], [
        [link(OUT / 'project_files.csv'), '当前项目 940 个文件：绝对/相对路径、体积、时间、Git 跟踪/忽略、分类、引用'],
        [link(OUT / 'candidate_files.csv'), '残留候选与大数据保留依赖，逐项给出处置理由'],
        [link(OUT / 'source_index.csv'), '383 个 MATLAB + 其他运行脚本的路径和名字引用'],
        [link(OUT / 'directory_summary.csv'), '一级目录文件数和体积'],
        [link(OUT / 'category_summary.csv'), '互斥分类汇总，避免重复加总'],
        [link(OUT / 'partial_result_checks.csv'), '8 份 partial/中断 CSV 对比结果'],
        [link(OUT / 'external_backup_roots.csv'), '75 个外部根位置体积、部分内容比较及恢复材料类型'],
        [link(OUT / 'external_backup_files.csv'), '14,338 个外部文件明细；常规同路径比较限 8 MiB 以下'],
        [link(OUT / 'large_copy_groups.csv'), '30 个大文件族和 313 份相同外部副本'],
        [link(OUT / 'large_copy_verification.csv'), '补充大文件 SHA-256 结果；优先于常规清单中的未哈希标记'],
        [link(OUT / 'codex_config_backups.csv'), '8 份 Codex 备份 + 1 个路径指针'],
        [link(OUT / 'archive_summary.csv') + '；' + link(OUT / 'archive_members.csv'), '40 个 ZIP 和所有成员目录'],
        [link(OUT / 'exact_duplicates.csv'), '当前树内 13 组哈希相同文件'],
        [link(OUT / 'empty_directories.csv'), '当前树和三个外部输出位置的 20 个空目录；可能是输出结构占位'],
        [link(OUT / 'scan_scope.csv'), '逐个列出已扫描位置与覆盖边界'],
        [link(OUT / 'verification.json'), '940 个原文件元数据未变、原 Git 状态未变、报告链接均存在、大文件比对全部一致'],
        [link(OUT / 'summary.json') + '；' + link(OUT / 'large_copy_summary.json'), '机器可读总览和错误记录'],
        [link(OUT / 'audit_workspace.py') + '；' + link(OUT / 'verify_large_copies.py') + '；' + link(OUT / 'build_report.py'), '可复跑的只读审计脚本，输出仅写本目录']]),
    '扫描覆盖当前项目、项目同级所有 MobileManipulationPlanning 命名的备份/压缩包、codex_backups 下该项目及 paper_tac 的快照、三个相关散落输出目录、Temp 中该项目清理备份，以及 Codex 根层配置备份和指定配置子目录。未进行全盘扫描，未递归检查无关兄弟项目或 Codex 插件/会话内容。先前访问用户主目录的顶层列表被系统拒绝；直接访问已知的 .codex 和 Temp 路径成功，所列扫描范围没有读取错误。',
    '本轮没有运行 MATLAB 测试或实验，结论基于现存源码、元数据、精确 CSV 行核对和文件哈希。只读脚本已完成，范围内扫描错误为 0，大文件比对期间变化文件为 0。旧日志中的 PASS/FAIL 只说明当时记录，不代表当前代码运行结果。',
]
(OUT / 'README.md').write_text('\n\n'.join(lines) + '\n', encoding='utf-8')

# Focused empty-directory inventory; do not recurse unrelated sibling projects.
empty = []
for root in [ROOT] + [Path(r['absolute_path']) for r in external if r['kind'] == 'EXTERNAL_EXPERIMENT_OUTPUT']:
    for base, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d != '.git' and Path(base) / d != OUT]
        if not dirs and not files:
            empty.append({'absolute_path': base, 'scope_root': str(root), 'note': '空目录；可能是输出结构占位，不自动删除'})
with (OUT / 'empty_directories.csv').open('w', encoding='utf-8-sig', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['absolute_path', 'scope_root', 'note'])
    writer.writeheader()
    writer.writerows(empty)
print(json.dumps({'report': str(OUT / 'README.md'), 'report_lines': len('\n\n'.join(lines).splitlines()),
                  'empty_directories': len(empty), 'project_count': len(project),
                  'external_count': len(external_files)}, ensure_ascii=True))
