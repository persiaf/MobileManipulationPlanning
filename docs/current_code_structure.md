# 当前代码结构说明

更新时间：2026-08-31
适用版本：当前工作区 `MobileManipulationPlanning`

本文档按当前目录实际内容整理，作为代码阅读、维护和实验复现的入口。项目主体为 MATLAB 代码；`integration/schemas` 定义 MATLAB 与 Linux/ROS 执行端之间的 JSON 数据边界。本文档不把 `results/` 中的实验产物当作源代码。

## 1. 项目定位与总体数据流

项目实现移动双臂操作任务的分层规划流程：

```text
配置/仿真场景或 real_scene.json
  -> 环境与任务区域构建
  -> 停靠候选生成、碰撞检查、几何可达性评估
  -> Bottom-Up / Top-Down / Bidirectional 策略选择停靠位姿
  -> 全局访问顺序与栅格路径规划
  -> 执行状态仿真与时间统计
  -> mission_plan.json 导出 / execution_log.json 导入
  -> 实验统计、图表和论文结果
```

主流程由 `main_mobile_manipulation_planning.m` 启动，并依次调用 `pipeline/stage1_*` 到 `stage4_*`。`stage5_generate_video.m` 是可选的视频生成阶段。

## 2. 入口与调用层次

| 文件 | 作用 |
|---|---|
| `main_mobile_manipulation_planning.m` | 项目主入口；初始化路径，执行场景、停靠、全局规划、执行统计和算法基准。 |
| `visualize_single_algorithm.m` | 针对单个全局规划算法生成可视化结果。 |
| `pipeline/init_paths.m` | 将项目各层目录加入 MATLAB path，保证从任意当前工作目录启动都可运行。 |
| `pipeline/stage1_build_scene.m` | 读取配置并构建场景、栅格地图、任务与任务区域。 |
| `pipeline/stage2_docking_planning.m` | 构建/复用候选库，运行层次化策略并输出选中的服务位姿及任务分配。 |
| `pipeline/stage3_global_planning.m` | 计算停靠点间路径代价，确定访问顺序，拼接全局路径并执行仿真。 |
| `pipeline/stage4_benchmark_all.m` | 在同一场景和位姿集合上批量比较全局规划算法。 |
| `pipeline/stage5_generate_video.m` | 可选的任务执行动画/视频生成阶段。 |

## 3. 目录职责与文件清单

下面的清单覆盖当前仓库中的 MATLAB 源文件（共 195 个）及接口 schema。文件作用按函数签名、调用关系和所在层归纳。

### `config/`：配置

- `scene_config.m`：地图、桌面、任务、障碍物及停靠采样参数。
- `robot_config.m`：底盘尺寸、双臂工作空间和几何可达性参数。
- `planner_config.m`：策略、代价、全局规划器、执行计时、实验和可视化参数。
- `real_platform_config.m`：实机坐标系、容差、升降高度、接口路径和 schema 版本。

### `environment/`：场景、地图与任务区域

- `build_scene.m`：根据默认配置创建完整仿真场景。
- `build_grid_map.m`：由场景配置生成占据栅格和地图元数据。
- `build_task_regions.m`：按桌面/区域组织任务。
- `assign_tasks_to_tables.m`：将任务映射到所属桌面。
- `validate_scene_config.m`：检查场景配置与地图的一致性。
- `load_ros_map.m`：读取 ROS 地图 YAML/图像并转换为 MATLAB 地图格式。
- `load_real_scene_json.m`：加载实测场景 JSON（环境层适配器）。

### `collision/`：底盘碰撞与服务位姿校验

- `check_footprint_collision.m`：判断指定底盘位姿是否与栅格障碍物碰撞。
- `compute_footprint_clearance.m`：计算底盘 footprint 到障碍物的最小间隙。
- `validate_service_config_collision.m`：校验完整服务配置的位姿安全性。

### `manipulator/`：几何可达性与 IK 边界

- `evaluate_single_arm_reachability.m`：计算单臂对单任务点的几何可达分数。
- `evaluate_dual_arm_reachability.m`：分别评估左右臂并给出分配与合成分数。
- `evaluate_bimanual_reachability.m`：评估双臂同时处理目标对的几何可达性。
- `evaluate_robust_reachability.m`：结合停靠误差模型计算鲁棒可达性。
- `evaluate_ik_feasibility.m`：真实 IK 适配器接口；未提供适配器时明确报错，不伪装成已验证 IK。

### `docking/`：停靠候选与服务配置

- `compute_reachability_field.m`：在桌面周围计算可达性场。
- `generate_docking_candidates.m`：生成候选 `[x,y,theta]` 停靠位姿并进行基础过滤。
- `evaluate_docking_candidate.m`：按碰撞、距离和任务覆盖等项计算候选代价。
- `select_multi_poses.m`：从候选集中选择覆盖任务所需的多个停靠位姿。
- `build_service_config.m`：将停靠位姿、任务摘要和实机字段组合为服务配置。
- `expand_lift_candidates.m`：按离散升降高度扩展服务配置，不进行连续联合优化。

### `planning_strategy/`：层次化停靠策略

- `run_planning_strategy.m`：按 `cfg.strategy.name` 分派策略。
- `bottom_up/plan_bottom_up.m`：Bottom-Up 基线，先局部选停靠点，再规划全局访问。
- `top_down/plan_top_down.m`：Top-Down，先按候选锚点和路径代价排序区域，再做路由感知选点。
- `top_down/plan_region_order.m`：计算区域代价矩阵并确定区域访问顺序。
- `top_down/estimate_docking_budget.m`：估计各区域停靠点预算。
- `top_down/select_route_aware_multi_docks.m`：在区域顺序约束下选择停靠点。
- `top_down/repair_uncovered_tasks.m`：对未覆盖任务进行覆盖修复。
- `bidirectional/plan_bidirectional.m`：从 Bottom-Up 解出发执行保守双向改进。
- `bidirectional/refine_docks_with_route_context.m`：尝试覆盖保持的停靠点替换/删除。
- `bidirectional/refine_route_with_selected_docks.m`：固定停靠集合后重新插入/优化访问路线。
- `common/build_candidate_library.m`：一次性构建各区域候选库，供多个策略共享。
- `common/validate_candidate_library.m`：检查候选库结构和区域覆盖字段。
- `common/build_region_anchors.m`：为 Top-Down 提取区域锚点短名单。
- `common/build_region_cost_matrix.m`：计算区域间锚点路径代价并使用缓存。
- `common/initialize_path_cost_cache.m`：初始化路径代价缓存。
- `common/get_cached_path_cost.m`：读取或计算两位姿间缓存路径代价。
- `common/build_unified_plan.m`：将不同策略结果规范化为统一 plan 结构。
- `common/append_region_plan.m`：追加区域的停靠、任务和位姿信息。
- `common/compute_strategy_objective.m`：计算策略目标函数。
- `common/validate_plan_consistency.m`：校验计划中的区域、位姿、任务关联。
- `common/normalize_visit_order.m`：把访问顺序规范化为本地停靠索引。
- `common/build_route_edge_set.m`：生成有向/无向路线边集合。
- `common/build_plan_diagnostics.m`：整理策略诊断指标。
- `common/compare_strategy_plans.m`：计算策略间路线、位姿和任务分配差异。

### `global_planner/`：全局路径规划

- `run_path_planner.m`：动态分派规划算法并执行必要的回退。
- `compute_path_cost_matrix.m`：计算所有停靠位姿对之间的路径代价矩阵。
- `nearest_neighbor_order.m`：最近邻访问顺序启发式。
- `two_opt_order.m`：对访问顺序执行 2-opt 改善。
- `stitch_global_path.m`：按访问顺序拼接各段栅格路径并输出分段统计。
- `reconstruct_path_grid.m`：由 parent 指针重建栅格路径。
- `post_validate_path.m`：对规划输出做可行性后校验。
- `run_astar_wrapper.m`、`run_jps_wrapper.m`、`run_gbfs_wrapper.m`、`run_theta_star_wrapper.m`：A*、JPS、GBFS、Theta* 规划器适配器。
- `run_dijkstra_wrapper.m`、`run_dstar_wrapper.m`、`run_dstar_lite_wrapper.m`、`run_lpa_star_wrapper.m`：Dijkstra、D*、D* Lite、LPA* 适配器。
- `run_lazy_theta_star_wrapper.m`、`run_convex_astar_wrapper.m`：Lazy Theta* 与 Convex-A* 适配器。
- `run_rrt_wrapper.m`、`run_rrt_star_wrapper.m`、`run_rrt_connect_wrapper.m`、`run_informed_rrt_wrapper.m`、`run_hybrid_astar_wrapper.m`：采样式/混合式规划器适配器。
- `run_aco_wrapper.m`、`run_ga_wrapper.m`、`run_pso_wrapper.m`、`run_voronoi_wrapper.m`：ACO、GA、PSO、Voronoi 等扩展算法适配器。
- `common/build_convex_segment.m`、`common/convex_neighbor_relation.m`、`common/find_convex_corners.m`：凸邻接和 Convex-A* 辅助计算。
- `common/collision_free.m`、`common/nearest_free.m`、`common/prepare_start_goal.m`：起终点修正和线段碰撞检查。
- `common/convert_path_grid2world.m`、`common/path_length.m`：坐标转换和路径长度计算。

### `execution/`：执行语义与时间模型

- `predict_geometrically_reachable_tasks.m`：预测停靠位姿可几何处理的任务。
- `get_planned_tasks_for_dock.m`：取得某停靠点的预测任务和显式指令任务。
- `simulate_planned_task_execution.m`：模拟执行并生成执行/验证任务集合。
- `separate_execution_status.m`：区分 predicted、commanded、executed、verified 四类状态。
- `update_task_state.m`：更新场景任务完成状态。
- `estimate_execution_time.m`：根据路径、转向、停靠和操作项估计执行时间。
- `estimate_path_turn_angle.m`：计算路径转角总量与转弯次数。
- `build_experiment_stats.m`：汇总路径、覆盖率、时间和策略指标。
- `print_experiment_stats.m`：以可读格式打印实验统计。

### `integration/`：跨平台数据接口

- `export_mission_plan_json.m`：导出有序服务配置为 `mission_plan.json`。
- `import_real_scene_json.m`：导入实测场景 JSON。
- `import_ros_execution_log.m`：导入 ROS 执行日志 JSON/CSV。
- `validate_frame_convention.m`：检查坐标系和 yaw 约定。
- `validate_mission_plan.m`：校验任务计划字段和结构。
- `schemas/real_scene_schema.json`：实测场景输入 schema。
- `schemas/mission_plan_schema.json`：MATLAB 输出任务计划 schema。
- `schemas/execution_log_schema.json`：Linux/ROS 执行反馈 schema。

### `calibration/`：标定数据加载

- `load_calibration_file.m`：读取标定文件并检查必需字段。
- `load_arm_workspace_map.m`：加载机械臂工作空间地图。
- `load_docking_error_model.m`：加载停靠误差模型。
- `load_lift_calibration.m`：加载升降机构标定数据。

### `utils/`：通用数学、坐标和实验辅助

- `build_scene_from_cfg.m`、`create_scaled_scene_config.m`：从配置构建场景及生成规模化实验配置。
- `build_result_record.m`、`create_empty_result_record.m`、`align_record_fields.m`：结果记录创建、对齐和兼容。
- `compute_path_length.m`、`densify_path.m`、`interpolate_path_by_distance.m`：路径长度、加密和按距离插值。
- `world2grid.m`、`grid2world.m`、`validate_world_path_collision.m`：世界/栅格坐标转换和路径碰撞验证。
- `rot2.m`、`wrapToPiLocal.m`、`rectMask.m`：旋转、角度归一化和矩形掩膜数学工具。
- `normalize_task_fields.m`：统一任务结构字段。
- `sample_task_near_reach_boundary.m`、`sample_task_near_table_corner.m`、`sample_task_on_opposite_edges.m`：困难任务采样。
- `generate_hard_table_tasks.m`：生成困难场景桌面任务。
- `configure_headless_experiment.m`：配置无图形界面的实验环境。
- `export_pub_figure.m`：导出论文图形。

### `experiments/`：实验驱动脚本

- `exp_ablation_study.m`：基础消融实验。
- `exp_hard_ablation.m`：困难场景消融实验。
- `exp_hard_scenario.m`：困难场景实验。
- `exp_random_batch.m`：随机批量规模实验。
- `exp_compare_hierarchical_strategies.m`：Bottom-Up、Top-Down、Bidirectional 单场景比较。
- `exp_strategy_hard_scenarios.m`：多策略困难场景比较。
- `exp_strategy_random_batch.m`：多策略随机批量比较。
- `real/exp_real_docking_accuracy.m`：实机停靠精度日志分析。
- `real/exp_real_multi_docking.m`：实机多停靠实验分析。
- `real/exp_real_multi_height.m`：多升降高度实验分析。
- `real/exp_real_full_pipeline.m`：实测场景、任务计划和执行日志的全流程分析。

### `visualization/`：绘图与动画

- `draw_scene.m`、`draw_scene_background.m`、`draw_robot.m`：场景背景、障碍物和机器人绘制。
- `draw_reachability_field.m`、`draw_local_manipulation_coverage.m`、`draw_docking_result.m`：可达性、局部覆盖和停靠结果绘制。
- `draw_global_path.m`、`draw_comprehensive_overview.m`：全局路径和综合总览。
- `animate_task_execution.m`：任务执行动画。
- `apply_publication_style.m`、`format_method_label.m`：论文风格和方法标签格式化。
- `plot_grid.m`、`plot_path.m`、`plot_square.m`、`plot_expand.m`：基础地图/路径绘图工具。
- `plot_framework_pipeline.m`：框架流程图。
- `plot_main_scenario_publication.m`、`plot_normal_vs_hard_scene.m`、`plot_normal_vs_hard_task_distribution.m`：主场景与困难场景图。
- `plot_ablation_bar.m`、`plot_hard_ablation_comparison.m`、`plot_hard_scenario_comparison.m`、`plot_scalability_curves.m`：消融、困难场景和规模性曲线。
- `save_publication_figures.m`：批量保存论文图形。

### `tests/`：自动化测试

- `run_all_tests.m`：测试总入口。
- `create_test_fixture.m`：创建小型、确定性的测试场景。
- `test_main_smoke.m`：主流程冒烟测试。
- `test_candidate_generation.m`、`test_shared_candidate_library.m`：候选生成与共享候选库测试。
- `test_bottom_up_regression.m`、`test_top_down_region_order.m`、`test_top_down_coverage_repair.m`：策略回归、区域排序和覆盖修复测试。
- `test_strategy_comparison_smoke.m`、`test_strategy_plan_consistency.m`、`test_strategy_plan_difference_metrics.m`：策略比较、计划一致性和差异指标测试。
- `test_bidirectional_identity_diagnostics.m`、`test_bidirectional_non_decreasing_coverage.m`：双向策略身份与覆盖率单调性测试。
- `test_path_cost_cache.m`、`test_route_edge_set_consistency.m`、`test_route_edge_difference_identity.m`、`test_route_edge_difference_directionality.m`：路径缓存和路线边诊断测试。
- `test_normalize_visit_order.m`、`test_preferred_visit_order.m`、`test_region_anchor_set_diversity.m`：访问顺序与区域锚点测试。
- `test_coordinate_roundtrip.m`、`test_ros_map_import.m`：坐标往返和 ROS 地图导入测试。
- `test_execution_time_breakdown.m`、`test_navigation_time_path_basis.m`、`test_path_length_decomposition.m`：时间模型和路径长度分解测试。
- `test_planned_execution_consistency.m`：计划任务与执行状态一致性测试。
- `test_mission_plan_export.m`：任务计划 JSON 导出/校验测试。
- `test_no_removed_learning_references.m`：检查已移除学习模块的旧引用。

## 4. 结果与文档目录

- `results/raw/`：MAT 文件形式的原始实验记录，不参与规划运行。
- `results/tables/`：CSV 汇总表，可被绘图和论文脚本读取。
- `results/figures/`：PNG/PDF/FIG 论文图形。
- `results/logs/`：本地执行日志（通常由 Git 忽略）。
- `docs/architecture.md`：简版架构说明。
- `docs/data_contracts.md`：JSON/CSV 数据契约。
- `docs/real_platform_integration.md`：实机集成边界。
- `docs/planner_algorithms.md`：全局规划算法说明。
- `docs/hierarchical_strategy_design.md`：三种层次化策略设计。
- `docs/strategy_experiment_protocol.md`：策略实验协议和指标解释。
- `docs/paper/`：论文实验段落等写作材料。
- `calibration/README.md`：标定数据目录说明。

## 5. 常用运行方式

在 MATLAB 当前目录为项目根目录时：

```matlab
run('main_mobile_manipulation_planning.m')
run('tests/run_all_tests.m')
run('experiments/exp_compare_hierarchical_strategies.m')
```

建议先运行测试，再运行实验脚本。真实平台流程只通过 `integration` 的 JSON/CSV 接口交换数据；MATLAB 代码本身不直接控制 ROS、底盘或机械臂。

## 6. 维护注意事项

1. 新增或重命名 MATLAB 文件时，同时更新本说明和 `README.md` 的仓库布局。
2. 修改配置字段、策略名称、规划算法或 schema 时，同步更新对应文档和测试。
3. 几何可达性是筛选模型，不等价于碰撞感知 IK；真实 IK 需要实现 `manipulator/evaluate_ik_feasibility.m` 的适配器。
4. 路径长度字段区分任务路径、返程路径和总路径，实验统计不要混用。
5. 大型实测数据放在仓库外，通过相对路径和 JSON 元数据引用，避免写入机器相关绝对路径。

## 7. GitHub 项目维护要求

### 7.1 远程项目与版本线

GitHub 项目地址为 `https://github.com/persiaf/MobileManipulationPlanning`。当前代码维护应区分两条版本线：

- 规划代码线：MATLAB 源码及实验结果；接口 `schema_version` 为 `1.0`，`planner_version` 为 `rch-mt-mmp-matlab`。
- 论文代码线：`paper_tac/`；`paper_tac/VERSION` 当前为 `0.2.0-format-revision`，对应分支 `paper/tac-format-v0.2.0`。

本地对照信息（2026-08-31）：默认分支快照 `origin/main` 指向 `1a03b74`（`v0.2.0-hierarchical-strategies`），当前论文格式分支提交为 `1448082`（`paper-tac-v0.2.0-format`）。工作区存在未提交修改，因此这些修改不能在 GitHub README、Release 或项目简介中提前宣称为已发布版本。远程认证失败时，只能记录本地核对结果，不能把本地状态当作已推送状态。

### 7.2 发布前核对清单

1. 在 MATLAB 中运行 `run('tests/run_all_tests.m')`，保存测试结果或失败原因。
2. 重新生成 README 中引用的实验表/图，确认数字与 `results/tables/*.csv`、`results/raw/*.mat` 一致，并写明生成日期。
3. 同步检查 `README.md`、`CHANGELOG.md`、本文件、`paper_tac/VERSION`、`config/real_platform_config.m` 和 `integration/schemas/*.json`。
4. 使用 `git status` 清理或排除 `results/logs/`、缓存、临时构建文件、编辑器文件、大型 ROS bag 和批量传感器数据。
5. 将源码、文档和必要的可复现实验产物作为一个逻辑提交，推送到明确的目标分支；不要把论文格式修订自动合并为规划代码版本。
6. 推送后在 GitHub 页面复核默认分支 README、分支名、标签、Release、仓库 Description 和 Topics；网页上的说明只能描述已提交且可复现的内容。
7. 若 GitHub Description、Topics、默认分支或 Release 内容发生变化，在 `CHANGELOG.md` 或对应 Release notes 中留下日期和变更原因。

### 7.3 GitHub 说明文件的内容边界

- README 必须链接本文件，并反映当前真实目录、入口命令、策略名称和接口 schema。
- 所有实验数字必须能追溯到仓库内 CSV/MAT 文件；不能只修改 README 中的数字。
- 必须保留“几何可达性不是完整 IK”和“ROS/Ubuntu/真实平台尚未验证”等限制，除非新增代码、测试和实机证据同时完成。
- 根项目规划版本与 `paper_tac` 论文格式版本独立管理；论文排版提交不应改变 MATLAB 规划器的 `planner_version`。
- GitHub 项目简介适合写稳定定位，不应写入临时实验结果、未提交改动、个人绝对路径或本地缓存路径。
