# 代码清理审查报告

日期：2026-08-31
范围：当前 `MobileManipulationPlanning` 工作区

## A. 修改前基线

- MATLAB 文件数量：195
- 分支：`paper/tac-format-v0.2.0`
- 提交：`4620652 feat: add hierarchical strategy diagnostics and maintenance docs`
- 跟踪文件状态：无已修改跟踪文件；存在大量未跟踪的 `paper_tac/results` 生成物
- 基线测试：`run('tests/run_all_tests.m')` 失败 1 项，唯一失败为 `test_no_removed_learning_references`；原因是未跟踪的 LNN 元数据文件被扫描到
- 基线安全文件：保存在仓库外 `C:\Users\Administrator\AppData\Local\Temp\MobileManipulationPlanning_cleanup_20260831_181820`

## B. 实际删除文件

没有删除完整文件。根据删除标准，没有发现同时满足“非入口、非动态目标、非实机接口、非 schema、非论文用途且有明确替代”的高置信度整文件。

## C. 合并/优化文件

| 文件 | 优化内容 | 是否改变算法 |
|---|---|---|
| `utils/generate_hard_table_tasks.m` | 删除无当前入口引用的 `lnn_stress` 分支及专用采样 helper；删除重复局部赋值。保留四种非学习 hard difficulty。 | 否 |
| `utils/create_empty_result_record.m` | 删除已移除学习模块遗留的 `lnn_enable`、`lnn_alpha`、`lnn_label_strategy` 字段。 | 否 |
| `visualize_single_algorithm.m` | 修复标题表达式语法错误；使用统一 `compute_path_length`；移除函数内部 `clear all`/`clc`。 | 否 |

## D. 保留但疑似冗余的文件

| 文件/区域 | 保留原因 |
|---|---|
| `global_planner/common/path_length.m` 与 `utils/compute_path_length.m` | 数学结果相近，但前者被 `compute_path_cost_matrix.m` 使用，后者承担世界路径统计；需进一步统一调用并做数值回归，不能仅凭名称删除。 |
| `environment/build_scene.m` 与 `utils/build_scene_from_cfg.m` | 前者是默认配置入口，后者支持随机/批量实验传入配置，职责不同。 |
| `environment/load_real_scene_json.m` 与 `integration/import_real_scene_json.m` | 前者是环境层适配器，后者是跨平台接口实现，保留分层边界。 |
| `visualization/plot_expand.m` | 当前主流程和论文导出未调用，但可能是独立的旧算法调试入口；按中置信度候选保留。 |
| `global_planner` 中扩展和随机规划器 wrapper | `run_path_planner.m` 通过算法名分派，`stage4_benchmark_all.m` 支持 extended/stochastic 配置；不能按默认未启用删除。 |
| `pipeline/stage5_generate_video.m` 与 `visualization/animate_task_execution.m` | 可选视频阶段，代码完整且有调用关系；不因默认关闭删除。 |
| `integration/`、`calibration/`、`experiments/real/` | 后续实机接口和数据边界，当前仿真未调用也必须保留。 |

## E. 测试结果

| 项目 | 结果 | 说明 |
|---|---|---|
| `run('tests/run_all_tests.m')` | FAIL | 仅 `test_no_removed_learning_references` 失败；未跟踪 `paper_tac/results/lnn/models/*.json` 污染扫描。 |
| 功能测试（排除上述污染检测） | PASS | 所有其余 MATLAB 测试通过。 |
| `run('main_mobile_manipulation_planning.m')` | PASS | 默认场景、全局路径和 A*/JPS/GBFS/Theta* benchmark 完成。 |
| `run('experiments/exp_compare_hierarchical_strategies.m')` | PASS | Bottom-Up、Top-Down、Bidirectional 均完成；PDF 导出有页面尺寸 warning。 |
| `visualize_single_algorithm` smoke | PASS | 修复语法后成功生成可视化。 |

## F. 数值回归

策略实验重跑后的核心结果保持：36 个任务、100% verified completion、27 个停靠点、任务路径 57.3436 m、返程路径 5.9512 m、总路径 63.2948 m。重新运行改变了规划耗时和图形二进制内容，但未改变任务覆盖、停靠数量或路径长度；这些运行时产物未纳入本轮清理提交。

## G. 修改规模与提交边界

- 删除 MATLAB 文件：0
- 修改 MATLAB 文件：3
- 新增 MATLAB 文件：0
- 代码净变化：1 行新增，39 行删除（不含本报告）
- 未提交/未删除：`paper_tac/results` 下的 LNN 模型、缓存、临时数据、构建产物和压缩包；这些文件仍属于工作区用户数据，按要求不自动清理

本轮不执行 `git clean`、强制 reset、checkout 覆盖或 GitHub push。清理代码可在本地提交；远程发布需另行审查未跟踪论文产物后再决定。
