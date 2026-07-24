# Codex 任务指令：MobileManipulationPlanning 清理、实机化重构与接口预留

请在 **MobileManipulationPlanning 项目根目录**执行本任务。你的角色是 MATLAB 机器人规划项目的高级软件架构师，目标是在保留现有 RCH-MT-MMP 主流程和论文仿真基线的前提下：

1. 完整删除 LNN 相关代码、配置、实验脚本和结果；
2. 删除确定未使用、失效、重复或纯缓存文件；
3. 梳理并维护现有 MATLAB 分层规划框架；
4. 将代码结构调整为面向真实移动双机械臂平台长期维护；
5. 为 Windows/MATLAB 离线规划与 Ubuntu 20.04 + ROS Noetic 实机执行预留稳定接口；
6. 不编造实机功能、实验数据或运行结果。

---

## 一、执行原则

### 1. 先分析，后删除

修改前必须：

- 输出当前项目文件结构树；
- 使用以下标签标记文件：
  - `[保留]`
  - `[修改]`
  - `[新增]`
  - `[删除候选]`
  - `[确认删除]`
- 全局检索每个删除候选的：
  - 函数调用；
  - 动态字符串调用；
  - 配置引用；
  - README 引用；
  - 实验脚本引用；
  - 结果生成与读取关系。
- 不得只根据普通文本搜索判断全局规划器 wrapper 未使用，因为 `global_planner/run_path_planner.m` 通过字符串和 `switch` 动态调度算法。

### 2. 建立可恢复点

若当前目录属于 Git 仓库：

```bash
git status
git switch -c refactor/real-platform-cleanup
git add -A
git commit -m "backup before LNN removal and real-platform refactor"
```

若不能提交，至少在项目目录外创建完整备份。最终清理后的仓库内不要保留 `_backup`、`old`、`trash` 等重复副本，恢复依赖 Git 历史或项目外备份。

### 3. 不改变核心研究逻辑

本次禁止无依据重写以下核心逻辑：

- 场景构建；
- 任务区域划分；
- 候选停靠位姿生成；
- 启发式候选代价；
- 多停靠任务覆盖；
- 全局路径代价矩阵；
- 最近邻访问排序；
- 2-opt 可选优化；
- 路径拼接和碰撞复核；
- 非 LNN 消融实验和随机批量实验。

允许修复明确的代码错误、配置失效和“规划任务与模拟执行任务不一致”的问题，但必须记录修改前后行为差异。

---

# 二、第一阶段：完整删除 LNN

## 1. 删除目录

确认无非 LNN 依赖后，删除：

```text
lnn/
python_lnn/
experiments/results/lnn/
```

## 2. 删除 LNN 实验脚本

删除所有仅服务于 LNN、Python LNN、学习排序或模型训练的脚本，包括但不限于：

```text
experiments/debug_python_lnn_matlab_parity.m
experiments/debug_python_lnn_single_prediction.m
experiments/exp_compare_lnn_docking.m
experiments/exp_eval_python_lnn_model.m
experiments/exp_export_lnn_dataset_for_python.m
experiments/exp_lnn_alpha_ablation.m
experiments/exp_lnn_best_model_selection.m
experiments/exp_lnn_hard_scenario.m
experiments/exp_lnn_label_strategy_compare.m
experiments/exp_lnn_limited_best_stress_selection.m
experiments/exp_lnn_random_generalization.m
experiments/exp_lnn_stress_scenario.m
experiments/exp_python_lnn_alpha_debug.m
experiments/exp_python_lnn_alpha_debug_fast.m
experiments/exp_python_lnn_boundary_aware_eval.m
experiments/exp_python_lnn_model_grid_eval.m
experiments/exp_python_lnn_model_selection_summary.m
experiments/exp_python_lnn_no_leakage_eval.m
experiments/exp_python_lnn_risk_adaptive_eval.m
experiments/exp_train_lnn_docking.m
experiments/resolve_lnn_eval_mode.m
experiments/summarize_lnn_results.m
experiments/summarize_python_lnn_alpha_debug.m
```

同时删除相关 `.asv` 自动备份文件。

使用文件名匹配再次核查：

```text
*lnn*
*python_lnn*
```

但不得误删普通单词中偶然包含相同字符的文件。

## 3. 删除 LNN 配置

从 `config/planner_config.m` 中完整删除：

```matlab
cfg.lnn.*
```

包括：

- enable；
- mode；
- backend；
- ranker_backend；
- label_strategy；
- model_type；
- alpha 与 alpha_mode；
- fusion_mode；
- cost_calibration；
- risk/uncertainty/gating 参数；
- 模型、数据集和缓存路径；
- reservoir、ridge、GPU 和 fallback 参数；
- 所有 LNN 注释。

删除后 `planner_config.m` 不得保留任何失效的 `cfg.lnn` 字段。

## 4. 清理候选生成中的 LNN 分支

重点修改：

```text
docking/generate_docking_candidates.m
docking/select_multi_poses.m
```

从 `generate_docking_candidates.m` 删除：

- `lnn_enabled`；
- `heuristic_cfg.lnn.enable=false`；
- LNN 推理和混合分支；
- `apply_lnn_cost_calibration`；
- `annotate_lnn_rank_changes`；
- `safe_lnn_alpha`；
- LNN cost、rank、correlation、alpha 等辅助函数和字段。

候选代价统一保留现有**纯启发式代价**。清理后 `candidate.cost_breakdown` 中不得再生成：

```text
lnn_cost
lnn_cost_calibrated
alpha_effective
heuristic_lnn_corr
rank_changed_by_lnn_flag
```

从 `select_multi_poses.m` 删除：

- LNN 数据采集开关；
- `lnn_samples`；
- `append_lnn_selection_samples`；
- `collect_lnn_docking_sample`；
- `save_lnn_dataset`；
- `is_lnn_collect_enabled`；
- 所有 LNN warning 和统计字段。

必须保证启发式多停靠选择逻辑保持可运行。

## 5. 清理文档中的 LNN 内容

重写 `README.md`，删除：

- LNN maintenance note；
- Python LNN 训练说明；
- 模型路径；
- LNN 实验命令；
- LNN 结果解释；
- LNN 目录树；
- 学习排序和 GPU 训练内容。

README 应重新聚焦：

- RCH-MT-MMP 分层规划；
- 真实移动双机械臂平台；
- MATLAB 离线规划；
- ROS Noetic 实机任务执行接口；
- 仿真与实机数据的区别；
- 项目运行、配置、测试和接口说明。

检查并修改：

```text
paper_experiment_section.md
output.txt
其他 Markdown、注释和示例
```

不得残留 `LNN`、`python_lnn`、`cfg.lnn` 等内容。

## 6. 删除 LNN 结果

在 `results/` 内删除路径或文件名包含以下内容的全部结果：

```text
lnn
python_lnn
ltr_ranker
lightgbm
xgboost
reservoir
risk_adaptive
boundary_aware_eval
alpha_debug
model_grid
```

重点清理：

```text
results/raw/*lnn*
results/tables/*lnn*
results/figures/*lnn*
results/cache/lnn_cases/
results/lnn/
```

最后执行全局检查：

```bash
rg -n -i "cfg\.lnn|\blnn\b|python_lnn|ltr_ranker|reservoir" .
```

除迁移说明或历史变更记录外，正式代码、配置、README 和结果目录中应为零匹配。若 `CHANGELOG.md` 记录“已移除 LNN”，允许保留一次历史描述。

---

# 三、第二阶段：删除未使用和无效文件

## 1. 已发现的明确删除候选

逐一确认后删除：

```text
collision/compute_clearance_cost.m
docking/optimize_docking_pose.m
manipulator/check_front_workspace_constraint.m
config/scene_config.asv
experiments/exp_lnn_stress_scenario.asv
__pycache__/
*.pyc
output.txt
```

已知情况：

- `compute_clearance_cost.m` 当前无调用，并引用不一致的 `cost_weights.clearance`；
- `optimize_docking_pose.m` 未进入当前主流程；
- `check_front_workspace_constraint.m` 未被当前可达性流程调用；
- `.asv`、`__pycache__`、`.pyc` 为编辑器或解释器缓存；
- `output.txt` 为旧目录/调试输出，不应作为源码。

## 2. 条件删除候选

以下文件必须先检查引用和用途：

```text
experiment_record.mat
task_execution_demo.mp4
paper_experiment_section.md
python_project/MobileManipulationPlanningPython/
```

处理原则：

- `experiment_record.mat`：若不是测试夹具且没有代码读取，删除；
- `task_execution_demo.mp4`：若 README 或论文不使用，删除；若仍有展示价值，移动至 `docs/media/`；
- `paper_experiment_section.md`：若仍用于论文维护，移动至 `docs/paper/`；否则删除；
- `python_project/MobileManipulationPlanningPython/`：该目录是独立 Python 原型，当前不参与 MATLAB 主流程且与 MATLAB 语义不完全一致。若没有外部依赖，从主项目删除，不要保留在最终仓库中。

## 3. 配置字段清理

扫描所有配置字段是否真正被代码读取。已发现的候选包括：

```matlab
cfg.reachability.score_resolution
cfg.cluster.merge_distance
cfg.candidate_orientation_count
cfg.local_optimization.*
```

处理规则：

- 若字段确实未被任何主流程、实验或测试读取，删除字段和说明；
- 若字段属于近期实机接口规划，应实现实际使用并补测试，不得只保留“未来可能用到”的空参数；
- 不允许保留“看起来有用但实际上无效”的配置。

## 4. 动态调用保护

以下内容不能仅因静态引用少就删除：

- `global_planner/run_*_wrapper.m`；
- `global_planner/common/`；
- `third_party/`；
- 被 `run_path_planner.m` 的 `switch` 分支动态调用的算法；
- 被 benchmark 配置字符串引用的算法。

对全局规划器输出一张表：

| 算法 | `run_path_planner` 支持 | benchmark 使用 | 主流程可选 | 依赖完整 | 处理 |
|---|---:|---:|---:|---:|---|
| A* | | | | | 保留/删除 |
| JPS | | | | | |
| GBFS | | | | | |
| Theta* | | | | | |
| ... | | | | | |

稳定算法保留；明显是空壳、完全无法运行且论文不使用的算法，才可删除或移出主路径。

---

# 四、第三阶段：维护现有框架，面向真实移动双机械臂

不要大规模搬动现有稳定模块。保留以下主干：

```text
main_mobile_manipulation_planning.m
config/
environment/
collision/
manipulator/
docking/
global_planner/
pipeline/
execution/
experiments/
visualization/
utils/
```

新增真实平台维护模块：

```text
integration/
calibration/
tests/
docs/
```

推荐最终结构：

```text
MobileManipulationPlanning/
├─ main_mobile_manipulation_planning.m             [保留/修改]
├─ README.md                                       [重写]
├─ CHANGELOG.md                                    [新增]
├─ .gitignore                                      [新增/修改]
│
├─ config/
│  ├─ robot_config.m                               [修改]
│  ├─ scene_config.m                               [维护]
│  ├─ planner_config.m                             [删除LNN并精简]
│  └─ real_platform_config.m                       [新增]
│
├─ environment/
│  ├─ build_scene.m                                [保留仿真入口]
│  ├─ build_grid_map.m
│  ├─ assign_tasks_to_tables.m
│  ├─ build_task_regions.m
│  ├─ validate_scene_config.m
│  ├─ load_ros_map.m                               [新增接口]
│  └─ load_real_scene_json.m                       [新增接口]
│
├─ collision/
│  ├─ check_footprint_collision.m
│  ├─ compute_footprint_clearance.m                 [新增或由现有逻辑抽取]
│  └─ validate_service_config_collision.m           [新增接口]
│
├─ manipulator/
│  ├─ evaluate_single_arm_reachability.m
│  ├─ evaluate_dual_arm_reachability.m
│  ├─ evaluate_bimanual_reachability.m              [新增接口/占位实现]
│  ├─ evaluate_ik_feasibility.m                     [新增接口/占位实现]
│  └─ evaluate_robust_reachability.m                [新增接口]
│
├─ docking/
│  ├─ compute_reachability_field.m
│  ├─ generate_docking_candidates.m
│  ├─ evaluate_docking_candidate.m
│  ├─ select_multi_poses.m
│  ├─ build_service_config.m                        [新增]
│  └─ expand_lift_candidates.m                      [新增接口]
│
├─ global_planner/
│  ├─ run_path_planner.m
│  ├─ compute_path_cost_matrix.m
│  ├─ nearest_neighbor_order.m
│  ├─ two_opt_order.m
│  ├─ stitch_global_path.m
│  ├─ post_validate_path.m
│  ├─ common/
│  └─ run_*_wrapper.m
│
├─ pipeline/
│  ├─ init_paths.m
│  ├─ stage1_build_scene.m
│  ├─ stage2_docking_planning.m
│  ├─ stage3_global_planning.m
│  ├─ stage4_benchmark_all.m
│  └─ stage5_generate_video.m
│
├─ execution/
│  ├─ build_experiment_stats.m
│  ├─ estimate_execution_time.m
│  ├─ update_task_state.m
│  ├─ get_planned_tasks_for_dock.m                  [新增]
│  ├─ simulate_planned_task_execution.m             [新增/替代旧逻辑]
│  └─ separate_execution_status.m                   [新增]
│
├─ integration/
│  ├─ schemas/
│  │  ├─ mission_plan_schema.json                   [新增]
│  │  ├─ real_scene_schema.json                     [新增]
│  │  └─ execution_log_schema.json                  [新增]
│  ├─ export_mission_plan_json.m                    [新增]
│  ├─ import_real_scene_json.m                      [新增]
│  ├─ import_ros_execution_log.m                    [新增]
│  ├─ validate_frame_convention.m                   [新增]
│  └─ validate_mission_plan.m                       [新增]
│
├─ calibration/
│  ├─ load_docking_error_model.m                    [新增接口]
│  ├─ load_lift_calibration.m                       [新增接口]
│  ├─ load_arm_workspace_map.m                      [新增接口]
│  └─ README.md                                     [新增]
│
├─ experiments/
│  ├─ exp_ablation_study.m
│  ├─ exp_hard_ablation.m
│  ├─ exp_hard_scenario.m
│  ├─ exp_random_batch.m
│  ├─ profile_single_case.m
│  └─ real/
│     ├─ exp_real_docking_accuracy.m                [新增模板]
│     ├─ exp_real_multi_docking.m                   [新增模板]
│     ├─ exp_real_multi_height.m                    [新增模板]
│     └─ exp_real_full_pipeline.m                   [新增模板]
│
├─ tests/
│  ├─ run_all_tests.m                               [新增]
│  ├─ test_no_lnn_references.m                      [新增]
│  ├─ test_main_smoke.m                             [新增]
│  ├─ test_candidate_generation.m                   [新增]
│  ├─ test_planned_execution_consistency.m          [新增]
│  ├─ test_mission_plan_export.m                    [新增]
│  └─ test_coordinate_roundtrip.m                   [新增]
│
├─ visualization/
├─ utils/
├─ docs/
│  ├─ architecture.md                               [新增]
│  ├─ real_platform_integration.md                  [新增]
│  ├─ data_contracts.md                             [新增]
│  └─ paper/                                        [可选]
│
└─ results/
   ├─ figures/
   ├─ tables/
   ├─ raw/
   ├─ real/
   └─ logs/
```

---

# 五、真实平台数据结构与兼容原则

## 1. 保留现有 `[x,y,theta]`

现有大量路径规划、可视化和排序代码默认：

```matlab
pose = [x, y, theta];
```

不要直接将全部位姿数组改为四列，否则会破坏现有代码。

新增服务配置结构体：

```matlab
service_config.id
service_config.pose2d              % [x, y, theta]
service_config.lift_height_m
service_config.table_id
service_config.task_ids
service_config.arm_assignments
service_config.arm_mode            % left/right/either/bimanual
service_config.predicted_reachability
service_config.robust_reachability
service_config.position_tolerance_m
service_config.yaw_tolerance_rad
service_config.policy_id
service_config.expected_execution_time_s
```

全局规划层只读取 `pose2d`；升降、机械臂和实机执行层读取完整 `service_config`。

## 2. 扩展任务结构，但保持兼容

保留现有字段：

```matlab
task.id
task.table_id
task.x
task.y
task.z
task.status
```

新增可选字段：

```matlab
task.object_pose_map
task.task_type
task.left_grasp_pose
task.right_grasp_pose
task.destination_pose
task.object_size
task.required_height_range
task.policy_id
```

允许旧仿真任务缺失这些字段，由默认值兼容；不得要求现有仿真一次性全部重写。

## 3. 升降接口

新增离散高度接口，第一版不做连续联合优化：

```matlab
cfg.real_platform.lift.enabled = false;
cfg.real_platform.lift.candidate_heights_m = [0.00, 0.15, 0.30];
cfg.real_platform.lift.min_height_m = 0.00;
cfg.real_platform.lift.max_height_m = 0.60;
```

当 `enabled=false` 时，主仿真结果应与清理前启发式版本一致；当 `enabled=true` 时，每个底盘候选可扩展为多个 `service_config`。

---

# 六、必须修复：规划任务与模拟执行一致性

当前 `stage3_global_planning.m` 使用：

```matlab
covered = select_covered_tasks(actual_pose, scene, robot_cfg, planner_cfg);
```

它会在执行阶段重新遍历场景中的全部未完成任务，可能把未被阶段二分配的任务算作完成。

请改成四级状态：

```text
predicted_tasks  — 阶段二预测可覆盖任务
commanded_tasks  — 本停靠点计划执行任务
executed_tasks   — 执行器报告已执行任务
verified_tasks   — 视觉/传感器确认成功任务
```

仿真模式下：

- `commanded_tasks` 必须来自 `task_sequence_summary{pose_idx-1}.covered_tasks` 或 `pose_info_list` 中的明确任务分配；
- 不得在执行阶段全场重新寻找任务；
- 默认仿真可将 `executed_tasks=commanded_tasks`；
- 只有通过仿真验证规则的任务进入 `verified_tasks`；
- 完成率分别输出：
  - predicted coverage rate；
  - commanded task rate；
  - verified completion rate。

保留旧 `select_covered_tasks.m` 时，应重命名为更准确的：

```text
predict_geometrically_reachable_tasks.m
```

或删除，避免继续被误用。

这是一次**有意的语义修复**，可能导致旧完成率变化。必须在最终报告中说明，不得为了保持旧 100% 指标而继续使用错误逻辑。

---

# 七、真实机器人接口设计

MATLAB 规划器不直接依赖 ROS Toolbox，也不直接控制机器人。采用 JSON/CSV 作为 Windows 与 Linux 的稳定边界。

## 1. `mission_plan.json`

输出至少包含：

```json
{
  "schema_version": "1.0",
  "frame_id": "map",
  "scene_id": "real_scene_001",
  "planner_version": "",
  "start_pose": [0.5, 0.5, 0.0],
  "route_order": [1, 2],
  "service_configs": [
    {
      "id": 1,
      "pose2d": [2.15, 1.05, 1.57],
      "lift_height_m": 0.18,
      "table_id": 1,
      "task_ids": [1, 2],
      "arm_mode": "bimanual",
      "arm_assignments": ["left", "right"],
      "policy_id": "act_box_pick",
      "position_tolerance_m": 0.03,
      "yaw_tolerance_rad": 0.05
    }
  ]
}
```

## 2. `real_scene.json`

用于导入：

- ROS map 元数据；
- 桌面/工作台位姿与尺寸；
- 真实目标物体位姿；
- 任务类型；
- 抓取位姿；
- 投放区；
- 坐标系；
- 时间戳和场景版本。

## 3. `execution_log.json/csv`

用于回传：

- 计划停靠位姿；
- 实际停靠位姿；
- 位置误差；
- 航向误差；
- 升降目标和实际高度；
- 任务开始/结束时间；
- ACT policy ID；
- 执行是否成功；
- 视觉验证结果；
- 失败原因；
- 是否重试和重规划。

## 4. 坐标约定

必须在 `docs/data_contracts.md` 中明确：

- 所有世界坐标使用米和弧度；
- 默认 `frame_id=map`；
- MATLAB 栅格行列与 ROS PGM 的 y 轴方向；
- `world2grid` 和 `grid2world` 的对应关系；
- `theta` 正方向；
- JSON 使用 UTF-8；
- Windows 与 Linux 使用相对路径，不写死 `D:\...` 或 `/home/...`。

新增坐标往返测试：

```text
world -> grid -> world
```

误差不得超过半个栅格分辨率。

---

# 八、结果目录清理与维护

## 1. 删除无用结果

清理：

- LNN/Python LNN 全部结果；
- `.fig` 中与正式论文无关且可重新生成的重复文件；
- `_partial`、`tmp`、cache 和中断恢复文件；
- 调试日志；
- 旧模型；
- 重复图片；
- 未被 README、论文或实验脚本引用的结果。

## 2. 保留最小可复现结果

保留非 LNN 的核心结果：

```text
results/figures/
  主场景
  基线对比
  消融实验
  困难场景
  规模与运行时间
  噪声/参数敏感性
  全局规划器 benchmark（若论文仍使用）

results/tables/
  主场景指标
  baseline summary
  ablation summary
  hard scenario summary
  random batch summary
  planner benchmark summary

results/raw/
  仅保留生成上述表图所需的最小非 LNN 数据
```

新增：

```text
results/real/
results/logs/
```

不要将大型原始 ROS bag、视频或完整批量缓存放入主仓库。README 中说明外部数据目录和导入方式。

## 3. `.gitignore`

至少加入：

```gitignore
*.asv
*.m~
__pycache__/
*.pyc
results/cache/
results/logs/
results/raw/*_partial.mat
results/raw/tmp*
*.tmp
.DS_Store
```

不要忽略论文真正需要的最小表格和图片。

---

# 九、测试与验收

## 1. 静态验收

执行并记录：

```bash
rg -n -i "cfg\.lnn|\blnn\b|python_lnn|ltr_ranker|reservoir" .
```

正式代码、配置、README、实验脚本和结果目录中不得残留 LNN。

检查：

- 无 `.asv`；
- 无 `__pycache__`；
- 无 `.pyc`；
- 无失效绝对路径；
- 无调用不存在函数；
- 无读取已删除结果；
- 无无效配置字段；
- 主目录树与 README 一致。

## 2. MATLAB 代码检查

对全部修改和新增 `.m` 文件执行：

```matlab
checkcode('文件路径')
```

尽可能运行：

```matlab
run('tests/run_all_tests.m')
```

至少包含：

- 项目路径初始化；
- 场景构建；
- 候选生成；
- 多停靠选择；
- 全局路径规划；
- 规划任务与执行任务一致；
- mission plan JSON 导出与读回；
- 坐标往返；
- 无 LNN 引用。

## 3. 回归测试

在相同随机种子和配置下，记录清理前后：

- 任务数；
- 候选数；
- 选中停靠位姿数；
- 阶段二预测覆盖率；
- 访问顺序；
- 路径长度；
- 规划时间；
- fallback 次数。

LNN 原本默认关闭，因此纯 LNN 删除不应改变阶段一、阶段二启发式选择和路径输出。若因执行语义修复导致最终完成率变化，要单独说明。

## 4. 实机接口只做可运行框架，不伪造结果

可以实现：

- JSON schema；
- 数据结构；
- 导出/导入；
- 参数验证；
- 占位函数；
- 模板实验脚本。

不得声称：

- 已连接 ROS；
- 已控制真实底盘；
- 已完成机械臂 IK；
- 已完成 ACT 执行；
- 已获得真实成功率。

未实现的接口应明确：

```matlab
error('NotImplemented: real robot IK adapter is not configured.');
```

或返回带状态的结构体，不能静默伪造成功。

---

# 十、最终输出格式

修改完成后，按以下顺序输出。

## 1. 修改前项目结构树

标记：

```text
[保留] [修改] [新增] [确认删除]
```

## 2. 删除清单

分为：

- LNN 代码；
- LNN 结果；
- 无用代码；
- 缓存与自动备份；
- 独立原型；
- 过期文档。

每项说明删除依据。

## 3. 修改后项目结构树

突出：

- 核心仿真规划；
- 实机 integration；
- calibration；
- tests；
- docs；
- 最小 results。

## 4. 逐文件修改说明

至少说明：

- `planner_config.m`；
- `generate_docking_candidates.m`；
- `select_multi_poses.m`；
- `stage3_global_planning.m`；
- `README.md`；
- 新增接口文件。

## 5. 框架梳理

说明完整数据流：

```text
仿真/真实场景输入
→ 任务区域构建
→ 停靠候选生成
→ 几何/双臂可达性评价
→ 多停靠选择
→ 全局访问排序
→ mission_plan.json
→ ROS执行端
→ execution_log
→ MATLAB结果分析
```

## 6. 测试结果

列出：

- 静态搜索结果；
- `checkcode` 结果；
- smoke test；
- 回归数据；
- 尚未运行的测试。

## 7. Windows → Linux 同步清单

明确哪些文件需要同步至 Ubuntu/ROS：

```text
integration/schemas/
mission_plan.json
real_scene.json
config/real_platform_config.m 的导出快照
docs/data_contracts.md
```

强调：

- 路径差异；
- UTF-8；
- LF 换行符；
- Linux 执行权限；
- `map` 坐标系一致；
- Windows 生成的规划结果与 Linux 实机使用同一场景版本。

## 8. 最终状态声明

若没有在真实平台运行，必须写：

> 已完成项目静态代码检查、LNN 清理、结构维护和实机接口框架修改；尚未在 Linux ROS Noetic 与真实移动双机械臂平台中进行运行验证。

---

# 十一、特别注意

1. 不要为了“看起来整洁”大规模重命名全部函数，避免破坏现有论文实验。
2. 不要把 ROS 代码直接塞进 MATLAB 核心目录。
3. 不要将 MATLAB 规划路径的全部栅格点直接作为真实机器人控制指令；实机接口以有序停靠目标和服务配置为主。
4. 不要把几何可达性描述成完整 IK 可行性。
5. 当前 `evaluate_dual_arm_reachability` 的 `max(left,right)` 仅表示任一机械臂可达，不等于双臂协同可达。保留兼容逻辑，同时新增独立 `bimanual` 接口。
6. 不要保留空配置、伪实现或永远不触发的分支。
7. 删除操作必须建立在调用关系和动态调度检查上。
8. 不得编造实验数据或实机结果。
