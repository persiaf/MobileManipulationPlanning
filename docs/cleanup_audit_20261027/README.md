# 代码框架、实验残留与修复备份盘点（2026-10-27）

扫描时间：2026-10-27T09:40:32+08:00；时区 UTC+08:00。

**本轮只整理清单和核对证据，没有删除、移动、恢复任何原文件，也没有修改项目配置。** 所有新增文件仅位于本报告目录。

项目根目录：`D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning`。Git 分支 `research/cafeteria-scene-planning`；HEAD `4fb8333a725b68778b7e03125bfed3ae566a7ac8`。工作区存在大量未提交修改和未跟踪文件，详见 [git_status_at_scan.txt](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/git_status_at_scan.txt>)。

## 1. 盘点结论

| 范围 | 数量 | 逻辑体积 | 说明 |
| --- | --- | --- | --- |
| 当前项目 | 940 | 2.31 GiB | 排除 .git 与本审计目录；383 个 MATLAB 文件 |
| 外部备份/散落输出位置 | 75 | 26.70 GiB | 14,338 个文件；含完整快照、增量补丁、压缩包和试跑输出 |
| 已核实的大文件相同副本 | 313 | 22.78 GiB | 计入上行外部体积；SHA-256 一致，不是额外相加的空间 |
| Codex 自身备份 | 8 | 2.09 MiB | 5 份 config.toml、1 份 .env、1 份旧状态库、1 份应用状态自动备份；另有 1 个路径指针 |

占空间的主要原因是多轮快照反复复制同一批原始数据。313 份大文件副本中，P3/P4 和 Position-only 状态在外部各有 10 份相同副本，主要旧学习数据各有 11 份。**22.78 GiB 是相同外部文件的逻辑体积，不代表这些备份全部可以删除，也不是实测可释放磁盘空间。**

可以优先处理的活动目录残留是：旧 LNN 研究数据、GUI/渲染试跑图片、5 份已被正式 CSV 覆盖的 partial 文件，以及历史日志。源代码、当前论文素材、P3/P4/P5 依赖和独有的中断结果单独保留。

## 2. 当前代码框架和实际路径

| 职责 | 真实位置 | 结论 |
| --- | --- | --- |
| 默认入口 | [main_mobile_manipulation_planning.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/main_mobile_manipulation_planning.m>) | stage1 → stage2 → stage3；可选 benchmark/video |
| 统一实验入口 | [run_selected_experiment.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/experiment_manager/run_selected_experiment.m>) | 注册表/配置构建器驱动；5 个已注册 profile |
| GUI | [launch_mobile_manipulation_experiment_manager.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/experiment_manager/launch_mobile_manipulation_experiment_manager.m>) | 单独加入 manager 与 pipeline 路径后初始化 |
| 路径初始化 | [init_paths.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/pipeline/init_paths.m>) | 实际位于 pipeline，根目录没有 init_paths.m |
| 配置 | [config](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/config>) | scene/robot/planner + resolve_effective_config；可视化规范配置在 visualization |
| 场景 | [build_scene.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/environment/build_scene.m>)；[build_scene_from_cfg.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/utils/build_scene_from_cfg.m>) | 默认构建和外部配置构建分工不同 |
| 候选库 | [build_candidate_library.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/planning_strategy/common/build_candidate_library.m>) | 配合 docking、manipulator、collision |
| 三种策略 | [run_planning_strategy.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/planning_strategy/run_planning_strategy.m>) | bottom_up / top_down / bidirectional |
| 全局路径 | [run_path_planner.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/global_planner/run_path_planner.m>)；[stage3_global_planning.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/pipeline/stage3_global_planning.m>) | 算法分派、dense/sparse 路由和真实路径拼接 |
| 动态修复 | [local_replan.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/replanning/local_replan.m>)；[execute_replanning_action.m](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/replanning/execute_replanning_action.m>) | 与 P5 实验、状态一致性/增量修复测试相连 |
| 实机边界 | [integration](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/integration>)；[calibration](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/calibration>) | 未在默认流程启用不代表无用 |
| 实验与测试 | [experiments](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/experiments>)；[tests](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/tests>) | 当前逐文件索引见 source_index.csv |
| 论文 | [main.tex](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/paper_tac/main.tex>) | sections/、figures/、references.bib 和最终 PDF 保留 |

### 2.1 路径问题

1. `README.md:18` 和 `docs/QUICK_START.md:6` 在根目录直接调用 `init_paths`，对未预置 MATLAB 路径的新会话不完整。`init_paths.m` 在 `pipeline/`；其 folders 列表还没有 `experiment_manager`。这也使“先运行旧 main，再直接调用统一 runner”不能保证 runner 可见。GUI launcher 和 builder 内部虽有部分补齐逻辑，但第一次解析函数仍需要路径入口。

以下是与现有文件布局一致的初始化方式；本轮未执行实验：

```matlab
project_root = 'D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning';
cd(project_root);
addpath(fullfile(project_root, 'pipeline'));
addpath(fullfile(project_root, 'experiment_manager'));
init_paths(project_root);
```

2. `docs/CODE_ARCHITECTURE.md:8` 写的 `environment/build_scene_from_cfg.m` 不存在，实际文件为 `utils/build_scene_from_cfg.m`；`:9` 写的 `docking/build_candidate_library.m` 不存在，实际文件为 `planning_strategy/common/build_candidate_library.m`。部分旧清单仍写 309 个源文件或 76 个测试，不能代表当前 383 个 MATLAB 文件。

3. `run_selected_experiment.m:19-20` 的默认输出是相对路径 `results/managed_experiments/...`，不绑定 project_root。同级目录确实另有 `results/managed_experiments/...`，含一个 record.mat 和 5 张图，共 2.44 MiB；这与从父目录运行时的路径行为一致，但仅凭输出不能证明当时的启动命令。后续运行宜显式给绝对 output_dir。

4. `../stage_output_custom_scene/` 和 `../stage_output_custom_scene_v2/` 各只剩一份 1,490 字节日志，其余目录为空。两份日志均记录了完成结果，不是完整实验归档。

5. `paper_tac/build/` 里的两个 PDF 都是文稿成品，不能按 build 目录一概当作缓存。论文 figures 中的素材副本也不能仅因 SHA-256 相同就移除。

### 2.2 各代码目录规模

| 目录 | MATLAB 文件数 |
| --- | --- |
| calibration | 4 |
| collision | 3 |
| config | 8 |
| demos | 5 |
| docking | 6 |
| environment | 8 |
| execution | 9 |
| experiment_manager | 5 |
| experiments | 58 |
| global_planner | 37 |
| integration | 5 |
| manipulator | 6 |
| pipeline | 6 |
| planning_strategy | 36 |
| replanning | 8 |
| tests | 106 |
| utils | 33 |
| visualization | 37 |

完整源文件路径、名字引用位置见 [source_index.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/source_index.csv>)。该索引采用词法引用，包含注释和测试，不能据此认定“零引用函数”可以删除。

## 3. 实验残留分类

| 类别 | 文件数 | 体积 | 建议 |
| --- | --- | --- | --- |
| LEGACY_LNN_EXPERIMENT | 84 | 699.79 MiB | 旧研究方向单独归档 |
| LEGACY_PAPER_CACHE | 1 | 47.66 MiB | 核实可再生后清理 |
| SMOKE_VISUAL_OUTPUT | 37 | 11.88 MiB | 可归档/清理候选 |
| REDUNDANT_PARTIAL_CONFIRMED | 5 | 0.02 MiB | 可归档/清理候选 |
| HISTORICAL_LOG | 50 | 3.76 MiB | 历史诊断归档 |
| HARDWARE_DEBUG_HISTORY | 32 | 0.22 MiB | 旧轮次归档，保留最新验证 |
| ROOT_AUDIT_HISTORY | 21 | 0.04 MiB | 历史报告归档候选 |
| PROJECT_ARCHIVE | 2 | 54.29 MiB | 旧快照归档，先验证内容 |
| PARTIAL_RESULT_REVIEW | 2 | 0.00 MiB | 核对正式结果覆盖后归档 |
| RESUME_CHECKPOINT_DUPLICATE | 1 | 0.01 MiB | 保留：resume 读取入口；需修改读取逻辑后才能移走 |
| P3P4_FROZEN_DEPENDENCY | 8 | 886.91 MiB | 保留 |
| POSITION_STATE_DEPENDENCY | 3 | 611.47 MiB | 保留或迁入冷存储并修改读取路径 |

### 3.1 已脱离当前主要研究方向的旧数据

- `D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning\paper_tac\results\lnn`：84 个文件，699.79 MiB。包含历史数据集、模型、CSV 导出、quick_check 和临时 MAT。`paper_tac/README.md:87` 明确当前稿件不把 LNN 作为主要贡献；`docs/cleanup_audit_20260831.md` 和 `docs/maintenance_followup_20260831.md` 也已把它列为历史产物。建议整个实验族归档，避免逐个拆散训练/验证/模型配套关系。

- `D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning\paper_tac\results\cache\lnn_cases\lnn_stress\seed_31\case_cache.mat`：47.66 MiB。归入旧研究缓存；当前源码未找到完整再生工具，故不把“cache”字样当作可无条件重建的证明。

这两项合计约 747.45 MiB。只把它们移出活动树并不能消除各历史快照中的副本，外部副本还需要单独整理。

### 3.2 已核实被正式结果覆盖的 partial 文件

| partial 文件 | 正式文件 | 行数 | 处置 |
| --- | --- | --- | --- |
| candidate_efficiency_p2_raw_partial.csv | candidate_efficiency_p2_raw.csv | 12 / 42 | 可归档/清理候选 |
| candidate_efficiency_p2_raw_partial_first30.csv | candidate_efficiency_p2_raw.csv | 30 / 42 | 可归档/清理候选 |
| position_only_full_validation_partial.csv | position_only_full_validation_raw.csv | 30 / 30 | 可归档/清理候选 |
| position_only_heldout_coverage_partial.csv | position_only_heldout_coverage_raw.csv | 15 / 15 | 可归档/清理候选 |
| position_only_heldout_failures_partial.csv | position_only_heldout_failures.csv | 0 / 0 | 可归档/清理候选 |
| position_only_resource_bounded_four_mode_partial.csv | position_only_resource_bounded_four_mode_raw.csv | 24 / 24 | 保留续跑入口 |

核对采用相同表头 + CSV 行的多重集合包含关系，不只是文件名匹配。其中 4 对还是 SHA-256 完全相同：full_validation、heldout_coverage、resource_bounded_four_mode 和 heldout_failures；最后一对为空结果表。

`position_only_resource_bounded_four_mode_partial.csv` 虽然 24 行与 raw 相同，仍是 `exp_position_only_isolated_four_mode.m` 中 `load_checkpoint` 的续跑输入。因此 6 份匹配的 partial 中，只把其余 5 份列为清理候选。

### 3.3 需要保留的中断证据

- `position_only_isolated_four_mode_partial.csv`：找不到对应完整结果，保留；不能以另一个 resource_bounded 协议的结果替代。

- `position_only_resource_bounded_full_audit_interrupted_9rows.csv`：同表头，但 9 行均不在后来的 24 行 raw 中，属于独有的历史批次；建议与其崩溃/重试日志归档，不按重复结果删除。

- `position_only_resource_bounded_console_retry1.log` 和 `retry2.log` 均记录 MATLAB `0xc0000374, Heap corruption`。它们是失败诊断记录，不能当作有效完成的实验。

### 3.4 应保留的复现输入

- `results/tables/p3p4_state_*.mat`：8 个，886.91 MiB。`exp_dynamic_local_replanning_p5.m:44-49` 默认读取 L2 seed 2026/2027/2028 状态；其余 L1/L3 仍属 P3/P4 冻结复现状态。

- `results/tables/position_only_resource_bounded_seed{1,3,5}_state.mat`：3 个，611.47 MiB。`exp_position_only_isolated_four_mode.m:125-150` 的 resume 模式会读取。

- 正式 raw/summary/task/trace 表、cafeteria 路径与 footprint 验证、当前论文 figures/ 和 results/paper/ 支撑报告保留。失败实验也可能支撑负面结论或消融解释，不仅按“失败”二字删除。

### 3.5 试跑图片和历史报告

GUI/渲染试跑图片分布在 `results/managed_gui_smoke/`、`managed_gui_smoke2/`、`results/figures/unified_smoke/`、`visualization_validation/`、`visualization_report/`，共 37 个文件、11.88 MiB。它们是派生的验证输出；保留测试源码后可优先从活动目录移出。

根目录 21 份 S2/footprint/config/checkpoint 等审计文件、50 份历史日志和 hardware_matched_lab 的 32 份轮次记录建议按日期/修复轮次归档。原路径、体积、修改时间和引用位置都在 [candidate_files.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/candidate_files.csv>)。

## 4. 外部备份和散落输出：完整位置清单

以下每行是一个扫描根位置，下面的所有文件都已登记在 [external_backup_files.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/external_backup_files.csv>)。表中的体积包含对应目录内的全部文件；若同时保留目录和 ZIP，两者分别计数。补丁恢复依赖对应 HEAD 和原有未提交状态，不能用“有 changes.patch”推断能够独立恢复完整工作区。

| 类别 | 位置数 | 文件数 | 体积 |
| --- | --- | --- | --- |
| CODEX_PROJECT_BACKUP | 49 | 10084 | 17.90 GiB |
| EXTERNAL_EXPERIMENT_OUTPUT | 3 | 8 | 0.00 GiB |
| SIBLING_ARCHIVE | 5 | 5 | 1.90 GiB |
| SIBLING_PROJECT_BACKUP | 17 | 4228 | 6.89 GiB |
| TEMP_PROJECT_CLEANUP_BACKUP | 1 | 13 | 0.00 GiB |

### CODEX_PROJECT_BACKUP

| 完整路径 | 文件数 | 体积 | 材料类型 |
| --- | --- | --- | --- |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260723_163642](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260723_163642>) | 202 | 8.18 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260724_094823](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260724_094823>) | 107 | 3.50 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260724_152116](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260724_152116>) | 253 | 10.04 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260831_194248_full_large_scale_baseline](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260831_194248_full_large_scale_baseline>) | 9 | 0.08 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260901_153609_coverage_preserving_candidate_augmentation](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260901_153609_coverage_preserving_candidate_augmentation>) | 15 | 0.20 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260901_182949_position_only_final_validation](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260901_182949_position_only_final_validation>) | 11 | 0.13 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260902_092419_resource_bounded_isolated_timing](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260902_092419_resource_bounded_isolated_timing>) | 10 | 0.13 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260902_141201_github_large_scale_checkpoint](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260902_141201_github_large_scale_checkpoint>) | 12 | 0.13 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260902_151624_p0_p1_scene_generator](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260902_151624_p0_p1_scene_generator>) | 10 | 0.20 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260902_155657_p15_c2_geometry_diagnosis](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260902_155657_p15_c2_geometry_diagnosis>) | 9 | 0.15 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260902_175728_p17_local_feasibility_oracle](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260902_175728_p17_local_feasibility_oracle>) | 8 | 0.13 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260902_181623_p18_radial_operational_feasibility](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260902_181623_p18_radial_operational_feasibility>) | 9 | 0.15 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260902_190049_p2_fast_candidate_efficiency](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260902_190049_p2_fast_candidate_efficiency>) | 9 | 0.14 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260902_194806_p3_p4_large_scale_sparse_routing](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260902_194806_p3_p4_large_scale_sparse_routing>) | 7 | 0.18 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260903_082411_p3_p4_final_validation](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260903_082411_p3_p4_final_validation>) | 6 | 0.08 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260903_092437_p5_dynamic_local_replanning](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260903_092437_p5_dynamic_local_replanning>) | 7 | 0.12 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260903_103316_p5_final_actual_full_replan](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260903_103316_p5_final_actual_full_replan>) | 6 | 0.07 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260903_142827_complex_scene_visualization](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260903_142827_complex_scene_visualization>) | 8 | 3.63 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260914_140050_architecture_config_visualization_cleanup](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260914_140050_architecture_config_visualization_cleanup>) | 963 | 68.04 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260914_151150_cafeteria_1x3_2x3_scene](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260914_151150_cafeteria_1x3_2x3_scene>) | 8 | 0.39 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260914_213508_cafeteria_navigation_docking_pipeline](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260914_213508_cafeteria_navigation_docking_pipeline>) | 18 | 8.23 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260922_154935_thesis_research_architecture_audit](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260922_154935_thesis_research_architecture_audit>) | 500 | 11.24 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20260922_163205_restore_candidate_generation_and_cafeteria](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20260922_163205_restore_candidate_generation_and_cafeteria>) | 172 | 1,529.99 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_113020_rc2_final_consolidation](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_113020_rc2_final_consolidation>) | 934 | 2,384.54 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_120000_round_c1_audit](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_120000_round_c1_audit>) | 5 | 0.02 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_122418_round_c2_2_dock_repair](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_122418_round_c2_2_dock_repair>) | 934 | 2,382.96 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_123000_round_c2_1_path_repair](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_123000_round_c2_1_path_repair>) | 30 | 9.44 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_132622_round_c2_3_task_reallocation](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_132622_round_c2_3_task_reallocation>) | 935 | 2,367.83 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_145537_round_c2_3_1_consistency_fix](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_145537_round_c2_3_1_consistency_fix>) | 20 | 0.41 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_151739_round_c2_4_dstar_incremental](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_151739_round_c2_4_dstar_incremental>) | 48 | 0.66 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_162848_round_c2_4_1_dstar_correctness](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_162848_round_c2_4_1_dstar_correctness>) | 33 | 0.51 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_172746_round_c2_4_2_e2e_reuse](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_172746_round_c2_4_2_e2e_reuse>) | 44 | 0.90 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_192033_round_c2_4_3_dstar_e2e_fix](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_192033_round_c2_4_3_dstar_e2e_fix>) | 38 | 1.31 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261019_200844_round_c2_5_unified_actions](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261019_200844_round_c2_5_unified_actions>) | 39 | 0.50 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261020_092619_c2_5_1_stage_review_v1](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261020_092619_c2_5_1_stage_review_v1>) | 603 | 441.83 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261020_140743_round_s1_rc123_feasibility](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261020_140743_round_s1_rc123_feasibility>) | 32 | 1.05 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261020_140743_round_s1_rc123_feasibility.zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261020_140743_round_s1_rc123_feasibility.zip>) | 1 | 0.24 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261020_172659_round_s2_rc2_four_tables](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261020_172659_round_s2_rc2_four_tables>) | 977 | 2,594.69 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261020_183401_round_s2_visual_style_unification](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261020_183401_round_s2_visual_style_unification>) | 970 | 2,368.34 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261020_191406_round_s2_monochrome_reference_style](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261020_191406_round_s2_monochrome_reference_style>) | 975 | 2,368.44 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261021_090523_round_s2_1_coverage_diagnosis](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261021_090523_round_s2_1_coverage_diagnosis>) | 29 | 0.26 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261021_090523_round_s2_1_coverage_diagnosis.zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261021_090523_round_s2_1_coverage_diagnosis.zip>) | 1 | 0.05 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261021_094259_round_s2_2_grid_coordinate_fix](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261021_094259_round_s2_2_grid_coordinate_fix>) | 53 | 44.07 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261021_094259_round_s2_2_grid_coordinate_fix.zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261021_094259_round_s2_2_grid_coordinate_fix.zip>) | 1 | 9.00 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261021_110913_round_s2_3_geometric_feasibility](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261021_110913_round_s2_3_geometric_feasibility>) | 37 | 41.58 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\20261021_110913_round_s2_3_geometric_feasibility.zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/20261021_110913_round_s2_3_geometric_feasibility.zip>) | 1 | 4.44 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\MobileManipulationPlanning\LATEST_ARCHITECTURE_CLEANUP_BACKUP.txt](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/MobileManipulationPlanning/LATEST_ARCHITECTURE_CLEANUP_BACKUP.txt>) | 1 | 0.00 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\paper_tac\20260807_101059_reproducibility_revision](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/paper_tac/20260807_101059_reproducibility_revision>) | 488 | 831.87 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\paper_tac\20260807_104917_final_initial_draft_revision](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/paper_tac/20260807_104917_final_initial_draft_revision>) | 496 | 832.02 MiB | 差异补丁及记录；需对应基线 |

### EXTERNAL_EXPERIMENT_OUTPUT

| 完整路径 | 文件数 | 体积 | 材料类型 |
| --- | --- | --- | --- |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\results](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/results>) | 6 | 2.44 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\stage_output_custom_scene](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/stage_output_custom_scene>) | 1 | 0.00 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\stage_output_custom_scene_v2](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/stage_output_custom_scene_v2>) | 1 | 0.00 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |

### SIBLING_ARCHIVE

| 完整路径 | 文件数 | 体积 | 材料类型 |
| --- | --- | --- | --- |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning (2).zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning (2).zip>) | 1 | 955.48 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning (3).zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning (3).zip>) | 1 | 242.27 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning (4).zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning (4).zip>) | 1 | 732.00 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning.zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning.zip>) | 1 | 5.87 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_pre_refactor_20260720.zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_pre_refactor_20260720.zip>) | 1 | 11.25 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |

### SIBLING_PROJECT_BACKUP

| 完整路径 | 文件数 | 体积 | 材料类型 |
| --- | --- | --- | --- |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_backup_20260915_074915](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_backup_20260915_074915>) | 8 | 8.00 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_backup_20260923_153015](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_backup_20260923_153015>) | 1313 | 2,347.65 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_backup_app_20260923_160708](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_backup_app_20260923_160708>) | 1317 | 2,347.67 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_backup_before_codex_verify_20260723](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_backup_before_codex_verify_20260723>) | 194 | 8.12 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_backup_viz_20260923_155502](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_backup_viz_20260923_155502>) | 1316 | 2,347.67 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_diag_backup_20260915_085725](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_diag_backup_20260915_085725>) | 6 | 0.03 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_geom_backup_20260915_084029](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_geom_backup_20260915_084029>) | 9 | 0.04 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_removed_python_cache_20260720](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_removed_python_cache_20260720>) | 6 | 0.00 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_round10_20260915_135947](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_round10_20260915_135947>) | 9 | 0.03 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_round11_20260915_143320](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_round11_20260915_143320>) | 3 | 0.00 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_round11_20260915_150659](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_round11_20260915_150659>) | 5 | 0.00 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_round4_20260915_090931](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_round4_20260915_090931>) | 6 | 0.03 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_round5_20260915_112314](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_round5_20260915_112314>) | 6 | 0.03 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_round6_20260915_115242](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_round6_20260915_115242>) | 6 | 0.03 MiB | 差异补丁及记录；需对应基线 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_round8_20260915_125648](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_round8_20260915_125648>) | 10 | 0.02 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_round9_20260915_132927](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_round9_20260915_132927>) | 8 | 0.01 MiB | 源文件副本/快照；完整性未承诺 |
| [D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_task2_backup_20260915_084530](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning_task2_backup_20260915_084530>) | 6 | 0.02 MiB | 差异补丁及记录；需对应基线 |

### TEMP_PROJECT_CLEANUP_BACKUP

| 完整路径 | 文件数 | 体积 | 材料类型 |
| --- | --- | --- | --- |
| [C:\Users\ADMINI~1\AppData\Local\Temp\MobileManipulationPlanning_cleanup_20260831_181820](<C:/Users/ADMINI~1/AppData/Local/Temp/MobileManipulationPlanning_cleanup_20260831_181820>) | 13 | 0.19 MiB | 报告/数据/压缩包/其他材料，需逐项核实 |

### 4.1 重复大文件的证据

逐文件读取并计算 SHA-256，比对期间文件大小和修改时间均稳定；共核实当前 30 个大文件与 313 份外部副本相同。完整绝对路径、哈希及文件 ID 见 [large_copy_verification.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/large_copy_verification.csv>)。以下数量只计算外部副本，不包含当前项目自身。

| 当前文件 | 外部相同副本数 | 外部相同副本逻辑体积 |
| --- | --- | --- |
| results/tables/position_only_resource_bounded_seed3_state.mat | 10 | 2.05 GiB |
| results/tables/position_only_resource_bounded_seed5_state.mat | 10 | 2.02 GiB |
| results/tables/position_only_resource_bounded_seed1_state.mat | 10 | 1.90 GiB |
| results/tables/p3p4_state_l3_seed2026_bottom_up.mat | 10 | 1.75 GiB |
| results/tables/p3p4_state_l3_seed2027_bottom_up.mat | 10 | 1.59 GiB |
| paper_tac/results/lnn/python_dataset/tmp_train_dataset.mat | 11 | 1.53 GiB |
| paper_tac/results/lnn/lnn_limited_best_stress_dataset.mat | 11 | 1.15 GiB |
| results/tables/p3p4_state_l2_seed2026_bottom_up.mat | 10 | 1.03 GiB |
| results/tables/p3p4_state_l2_seed2028_bottom_up.mat | 10 | 1.03 GiB |
| results/tables/p3p4_state_l2_seed2027_bottom_up.mat | 10 | 0.95 GiB |
| results/tables/p3p4_state_l1_seed2026_bottom_up.mat | 10 | 0.82 GiB |
| results/tables/p3p4_state_l1_seed2028_bottom_up.mat | 10 | 0.80 GiB |
| results/tables/p3p4_state_l1_seed2027_bottom_up.mat | 10 | 0.70 GiB |
| paper_tac/results/lnn/python_dataset/lnn_train.csv | 11 | 0.72 GiB |
| paper_tac/results/lnn/python_dataset/tmp_val_dataset.mat | 11 | 0.72 GiB |
| paper_tac/results/lnn/python_dataset/tmp_test_dataset.mat | 11 | 0.70 GiB |
| paper_tac/results/cache/lnn_cases/lnn_stress/seed_31/case_cache.mat | 11 | 0.51 GiB |
| MobileManipulationPlanning.zip | 6 | 0.28 GiB |
| paper_tac/results/lnn/lnn_stress_dataset.mat | 11 | 0.40 GiB |
| paper_tac/results/lnn/lnn_best_selection_dataset.mat | 11 | 0.34 GiB |
| paper_tac/results/lnn/python_dataset/lnn_val.csv | 11 | 0.34 GiB |
| paper_tac/results/lnn/python_dataset/lnn_test.csv | 11 | 0.33 GiB |
| paper_tac/results/lnn/python_dataset_boundary_quick_check/tmp_train_dataset.mat | 11 | 0.23 GiB |
| results/tables/center_row_candidate_geometry_trace.csv | 10 | 0.18 GiB |
| paper_tac/results/lnn/lnn_docking_dataset.mat | 11 | 0.17 GiB |
| paper_tac/results/lnn/python_dataset_quick_check/tmp_train_dataset.mat | 11 | 0.14 GiB |
| paper_tac/results/lnn/python_dataset_boundary_quick_check/lnn_train.csv | 11 | 0.12 GiB |
| paper_tac/results/lnn/python_dataset_boundary_quick_check/tmp_val_dataset.mat | 11 | 0.10 GiB |
| paper_tac/results/lnn/python_dataset_boundary_quick_check/tmp_test_dataset.mat | 11 | 0.09 GiB |
| paper_tac/results/lnn/python_dataset_quick_check/lnn_train.csv | 11 | 0.09 GiB |

最值得合并的是 2026-09-23 的三个约 2.29 GiB 完整副本，以及 2026-10-19/20 的多次约 2.3–2.5 GiB 快照。应先保留关键修复前后的独有源文件、补丁、HEAD 和报告，再把大数据改为共享的一份证据归档。不能直接删掉整个快照来追求表里的重复体积。

部分 10 月最新目录主要由报告、诊断脚本、before/after 清单和数据组成，未发现与当前路径匹配的源文件副本。它们是修复记录，不一定是能够直接覆盖还原的完整备份。

当前项目内部另核实 13 组相同内容文件，详见 [exact_duplicates.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/exact_duplicates.csv>)。例如 `lnn_boundary_medium_model.mat` 与 `_v2.mat` 相同；但论文素材拷贝、用于两个实验口径的相同汇总表需要保留各自入口。

### 4.2 压缩包

共读取 40 个 ZIP 的中央目录，完整清单在 [archive_summary.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/archive_summary.csv>)，每个包的文件成员在 [archive_members.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/archive_members.csv>)。没有解压、执行或做完整 CRC 测试。

项目根目录 `MobileManipulationPlanning.zip` 是 47.26 MiB 的旧快照，内部 911 个文件、未压缩体积约 48.37 MiB，远小于当前 2.31 GiB 活动树，不能代替当前完整备份。该包还在外部出现 6 份 SHA-256 相同副本。

同级 `(2).zip`、`(3).zip`、`(4).zip`、`pre_refactor_20260720.zip` 和早期 `MobileManipulationPlanning.zip` 共约 1.90 GiB；它们并非同一个文件的同名副本，需按来源和内容选择保留点。

## 5. Codex 自身配置修复前备份

位置：`C:\Users\Administrator\.codex`。只登记元数据以及配置备份与当前文件是否相同；没有导出配置值、代理地址、认证信息或数据库内容。

| 完整文件路径 | 体积 | 用途 | 与当前文件比较 |
| --- | --- | --- | --- |
| [C:\Users\Administrator\.codex\.codex-global-state.json.bak](<C:/Users/Administrator/.codex/.codex-global-state.json.bak>) | 565194 B | 应用状态自动备份 | NOT_COMPARED_LIVE_STATE |
| [C:\Users\Administrator\.codex\.env.bak-20261027-093020-proxy-fix](<C:/Users/Administrator/.codex/.env.bak-20261027-093020-proxy-fix>) | 204 B | 代理修复前环境文件 | DIFFERENT_SHA256 |
| [C:\Users\Administrator\.codex\config.toml.bak-20260701-154201](<C:/Users/Administrator/.codex/config.toml.bak-20260701-154201>) | 3634 B | Codex 配置历史快照 | DIFFERENT_SHA256 |
| [C:\Users\Administrator\.codex\config.toml.bak-before-code-skit-20260724](<C:/Users/Administrator/.codex/config.toml.bak-before-code-skit-20260724>) | 2845 B | Codex 配置历史快照 | DIFFERENT_SHA256 |
| [C:\Users\Administrator\.codex\config.toml.bak-before-medium-20260903-0948](<C:/Users/Administrator/.codex/config.toml.bak-before-medium-20260903-0948>) | 4727 B | Codex 配置历史快照 | DIFFERENT_SHA256 |
| [C:\Users\Administrator\.codex\config.toml.bak-before-stream-fix-20260720-2155](<C:/Users/Administrator/.codex/config.toml.bak-before-stream-fix-20260720-2155>) | 3709 B | Codex 配置历史快照 | DIFFERENT_SHA256 |
| [C:\Users\Administrator\.codex\config.toml.bak-too-many-pending-20260915-1345](<C:/Users/Administrator/.codex/config.toml.bak-too-many-pending-20260915-1345>) | 4468 B | Codex 配置历史快照 | DIFFERENT_SHA256 |
| [C:\Users\Administrator\.codex\state_5.sqlite.bak-before-gpt55-low-20260903-0945](<C:/Users/Administrator/.codex/state_5.sqlite.bak-before-gpt55-low-20260903-0945>) | 1609728 B | 旧会话状态数据库快照 | NOT_COMPARED_LIVE_STATE |
| [C:\Users\Administrator\.codex\tmp_large_scale_backup_path.txt](<C:/Users/Administrator/.codex/tmp_large_scale_backup_path.txt>) | 154 B | 备份位置指针；不是项目或配置内容备份 | NOT_COMPARED_LIVE_STATE |

其中 `.env.bak-20261027-093020-proxy-fix` 最直接对应本次日期的代理修复前备份；文件名记载备份操作时间，mtime 仍保留 2026-07-13 的源文件时间，二者不能混为一谈。其他配置文件也存在同样的时间差，因此清单同时保留完整文件名和 mtime。

5 份 config.toml 备份以及 .env 备份都与当前文件不同；本轮没有恢复它们。应用自动备份和旧状态库可能涉及历史会话/状态，不以“大于配置文件”或“有 .bak 后缀”为删除依据。`tmp_large_scale_backup_path.txt` 只是指向 20260902 大规模实验 checkpoint 的路径指针，不算第 9 份配置备份。

## 6. 建议整理顺序

1. 先把本报告作为索引使用；当前已有未提交修改，先建立一个包含当前源码和独有数据的可验证保留点。

2. 优先整理 5 份已覆盖 partial 和 37 张试跑图；这些体积不大，但分类最清晰。保留被 resume 读取的 checkpoint。

3. 把旧 LNN 实验族和对应缓存整体迁到历史研究归档，连同原实验说明与数据配套关系；将失败/中断批次和日志按日期收纳。

4. 合并外部完整快照中已核实相同的原始大文件，只保留一份基准数据及其哈希；各轮代码差异、恢复说明、报告和关键版本保留。快照中的补丁与源文件存在独有内容，不整目录盲删。

5. 单独管理 Codex 配置回滚点，至少保留当前已知可用版本与所需修复前版本；不混入 MATLAB 实验归档。

以上是后续整理建议，本轮没有实施文件迁移或清理。

## 7. 清单入口与核验边界

| 文件 | 用途 |
| --- | --- |
| [project_files.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/project_files.csv>) | 当前项目 940 个文件：绝对/相对路径、体积、时间、Git 跟踪/忽略、分类、引用 |
| [candidate_files.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/candidate_files.csv>) | 残留候选与大数据保留依赖，逐项给出处置理由 |
| [source_index.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/source_index.csv>) | 383 个 MATLAB + 其他运行脚本的路径和名字引用 |
| [directory_summary.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/directory_summary.csv>) | 一级目录文件数和体积 |
| [category_summary.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/category_summary.csv>) | 互斥分类汇总，避免重复加总 |
| [partial_result_checks.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/partial_result_checks.csv>) | 8 份 partial/中断 CSV 对比结果 |
| [external_backup_roots.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/external_backup_roots.csv>) | 75 个外部根位置体积、部分内容比较及恢复材料类型 |
| [external_backup_files.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/external_backup_files.csv>) | 14,338 个外部文件明细；常规同路径比较限 8 MiB 以下 |
| [large_copy_groups.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/large_copy_groups.csv>) | 30 个大文件族和 313 份相同外部副本 |
| [large_copy_verification.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/large_copy_verification.csv>) | 补充大文件 SHA-256 结果；优先于常规清单中的未哈希标记 |
| [codex_config_backups.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/codex_config_backups.csv>) | 8 份 Codex 备份 + 1 个路径指针 |
| [archive_summary.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/archive_summary.csv>)；[archive_members.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/archive_members.csv>) | 40 个 ZIP 和所有成员目录 |
| [exact_duplicates.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/exact_duplicates.csv>) | 当前树内 13 组哈希相同文件 |
| [empty_directories.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/empty_directories.csv>) | 当前树和三个外部输出位置的 20 个空目录；可能是输出结构占位 |
| [scan_scope.csv](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/scan_scope.csv>) | 逐个列出已扫描位置与覆盖边界 |
| [verification.json](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/verification.json>) | 940 个原文件元数据未变、原 Git 状态未变、报告链接均存在、大文件比对全部一致 |
| [summary.json](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/summary.json>)；[large_copy_summary.json](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/large_copy_summary.json>) | 机器可读总览和错误记录 |
| [audit_workspace.py](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/audit_workspace.py>)；[verify_large_copies.py](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/verify_large_copies.py>)；[build_report.py](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_audit_20261027/build_report.py>) | 可复跑的只读审计脚本，输出仅写本目录 |

扫描覆盖当前项目、项目同级所有 MobileManipulationPlanning 命名的备份/压缩包、codex_backups 下该项目及 paper_tac 的快照、三个相关散落输出目录、Temp 中该项目清理备份，以及 Codex 根层配置备份和指定配置子目录。未进行全盘扫描，未递归检查无关兄弟项目或 Codex 插件/会话内容。先前访问用户主目录的顶层列表被系统拒绝；直接访问已知的 .codex 和 Temp 路径成功，所列扫描范围没有读取错误。

本轮没有运行 MATLAB 测试或实验，结论基于现存源码、元数据、精确 CSV 行核对和文件哈希。只读脚本已完成，范围内扫描错误为 0，大文件比对期间变化文件为 0。旧日志中的 PASS/FAIL 只说明当时记录，不代表当前代码运行结果。
