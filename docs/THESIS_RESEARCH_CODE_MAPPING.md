> 历史记录说明（2026-10-27）：餐厅场景、实机匹配场景及另一套机器人几何已移除。本文的旧入口、维护状态与文件数量仅对应当时版本；当前唯一模型及删除范围见 [主场景参数统一说明](main_robot_unification.md)。

# THESIS_RESEARCH_CODE_MAPPING

移动底盘–双臂机器人任务规划项目 · 硕士论文三项研究内容（RC1/RC2/RC3）代码一致性审计映射表

- 审计时间：2026-09-22
- 审计前 HEAD：`4fb8333a725b68778b7e03125bfed3ae566a7ac8`（分支 `research/cafeteria-scene-planning`）
- 审计性质：只读审计 + 文档归档；未修改任何算法数值、未改动任何历史 CSV
- 证据粒度：`文件:行号`
- 备份目录：`codex_backups/MobileManipulationPlanning/20260922_154935_thesis_research_architecture_audit/`

---

## 0. 论文主线与代码分层

```text
RC1 面向操作任务的移动底盘–双臂可达性建模
      task → docking candidate → footprint collision → arm reachability → task-dock feasible relation
        ↓（产出 candidate_library / pose_info_list / task_sequence_summary）
RC2 基于可达性的多停靠任务规划与移动路径优化
      candidate library → Bottom-Up / Top-Down / Bidirectional → selected docks
      → task assignment → route order → global path
        ↓（产出 plan / stats.route_order / global_path / stats.sparse_graph）
RC3 面向环境变化的可达性约束任务–路径协同在线重规划
      dynamic event → change impact → local repair（P5 / RULE_BASED_REPLANNING_BASELINE）
```

代码分层现状（顶层目录 → 研究内容归属）：

| 目录 | 归属 | 说明 |
| --- | --- | --- |
| `config/` | COMMON | 唯一配置源；`resolve_effective_config` 为分层解析入口 |
| `environment/` | COMMON（baseline） | 基线场景构建链（`build_scene` 系列） |
| `utils/` | COMMON | 坐标转换、任务采样、结果记录、`build_scene_from_cfg` |
| `collision/` | COMMON（RC1 消费） | 底盘 footprint 几何硬约束 |
| `manipulator/` | RC1 | 单臂/双臂可达性；含 3 个未接入主流程的 API |
| `docking/` | RC1（+ RC2 接口） | 候选停靠位姿生成与代价；`select_multi_poses` 被 Bottom-Up 调用 |
| `planning_strategy/` | RC2 | 三策略统一入口与公共工具 |
| `global_planner/` | RC2（COMMON 子层 `common/`） | 多算法 wrapper + 稀疏图 + 路径拼接 |
| `pipeline/` | COMMON（主流程编排） | stage1–stage5 |
| `replanning/` | RC3 | `apply_dynamic_event` / `analyze_change_impact` / `local_replan` |
| `execution/` | EXECUTION | 执行仿真与统计（不被 RC1/RC2 反向依赖） |
| `visualization/` | COMMON | 场景/停靠/路径/动画/论文图 |
| `experiments/` | EXPERIMENT | 全部 P0–P18 实验脚本与 cafeteria 实验入口 |
| `integration/` | EXECUTION（实机边界） | mission plan JSON 导出/校验、ROS 日志导入、帧约定校验 |
| `calibration/` | COMPATIBILITY | 标定文件加载（当前为占位实现） |
| `tests/` | COMMON（回归） | 86 个 functiontest，含 cafeteria / P5 / 稀疏图回归 |
| `demos/` | EXPERIMENT（示例） | 4 个 demo 入口 |

---

## 1. RC1 映射表（可达性建模）

| Research Content | Algorithm Function | Directory/File | Main Function | Input | Output | Config | Called By | Current Status | Missing Function | Recommended Action |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RC1 | 单臂几何可达性（硬几何 + 打分） | `manipulator/evaluate_single_arm_reachability.m` | `evaluate_single_arm_reachability` | `pose, target_xyz, arm_side, robot_cfg, threshold` | `score∈[0,1]` | `robot_cfg`（R_min/R_max/z_min/z_max） | `evaluate_dual_arm_reachability:6-7`；`evaluate_bimanual_reachability:16,18`；`experiments/exp_local_reachability_oracle:336`、`exp_radial_operability_audit:286,308` | ACTIVE（RC1 核心） | — | 保持数值不动；仅补头部注释说明阈值来源 |
| RC1 | 双臂可达性（取 max + 臂分配） | `manipulator/evaluate_dual_arm_reachability.m` | `evaluate_dual_arm_reachability` | 同上 | `left_score,right_score,combined_score,arm_assignment` | `planner_cfg.reachability.threshold`（默认 0.30） | `docking/generate_docking_candidates:86`；`planning_strategy/common/augment_candidate_library:197`；`execution/simulate_planned_task_execution:14`；`execution/predict_geometrically_reachable_tasks:15` | ACTIVE（RC1 核心） | — | 保持 |
| RC1 | 候选—任务快速预筛（不近似正式评估） | `manipulator/candidate_reachability_quick_filter.m` | `candidate_reachability_quick_filter` | `pose,target_xyz,robot_cfg` | `reject,reason` | `robot_cfg` | `docking/generate_docking_candidates:75`；`experiments/exp_candidate_efficiency_p2:133` | ACTIVE | — | 保持（已有 `test_candidate_quick_filter_no_false_negative` 守护） |
| RC1 | 双臂协同可达性（双手同任务对） | `manipulator/evaluate_bimanual_reachability.m` | `evaluate_bimanual_reachability` | `pose,target_pair_xyz,robot_cfg,threshold` | `score,details` | `robot_cfg` | 仅自身；无外部调用 | **UNUSED** | 论文若声称“双臂协同操作”，缺少主流程接口 | 保留文件；在论文中明确当前仅为几何单臂+分配，bimanual 未进主流程 |
| RC1 | 鲁棒可达性（停靠误差摄动） | `manipulator/evaluate_robust_reachability.m` | `evaluate_robust_reachability` | `pose,target_xyz,robot_cfg,threshold,error_model` | `minimum_score,mean_score,success_fraction,...` | 内置 `position_offsets_m/yaw_offsets_rad` | 仅自身；无外部调用 | **UNUSED（ROBUST_NOT_IN_CORE_PIPELINE）** | 未接入 stage2/stage3 | 保留为独立 API；若论文要写“鲁棒”，需显式接入或在论文中降级表述 |
| RC1 | IK 可行性（适配器模式） | `manipulator/evaluate_ik_feasibility.m` | `evaluate_ik_feasibility` | `request,adapter` | `result` | — | 仅自身；无外部调用 | **UNUSED（占位 API）** | 无真实 IK 求解器 | 保留；论文不要声称已做 IK 验证 |
| RC1 | 底盘 footprint 碰撞（几何硬约束） | `collision/check_footprint_collision.m` | `check_footprint_collision` | `pose,grid_map,map_info,robot_cfg` | `is_collision` | `robot_cfg.chassis.*` | `docking/generate_docking_candidates:48-87` 段内过滤 | ACTIVE | — | 保持 |
| RC1 | footprint 最小间隙 | `collision/compute_footprint_clearance.m` | `compute_footprint_clearance` | `pose,grid_map,map_info,robot_cfg` | `clearance(m)` | `planner_cfg.candidate.safety_distance_m`（0.25） | `docking/evaluate_docking_candidate:21-40`；`experiments/build_cafeteria_navigation_library:64` | ACTIVE | — | 保持 |
| RC1 | service config 碰撞校验 | `collision/validate_service_config_collision.m` | `validate_service_config_collision` | `service_config,grid_map,map_info,robot_cfg` | `is_valid,message` | — | `integration/validate_mission_plan.m`（实机边界） | ACTIVE（EXECUTION 侧） | — | 保持 |
| RC1 | 候选停靠位姿生成（task→candidate→reachability 主链） | `docking/generate_docking_candidates.m` | `generate_docking_candidates` | `region,scene,robot_cfg,planner_cfg` | `raw candidates（含 covered_tasks/reach_scores/arm_assignment/coverage_ratio）` | `scene_cfg.docking_*`（0.25/0.40/0.05/0.15） | `planning_strategy/common/build_candidate_library`；`experiments/build_cafeteria_navigation_library:12` | ACTIVE（RC1→RC2 接口） | — | 保持 |
| RC1 | 候选代价融合（reachability+coverage+orientation+safety+approach+maneuver） | `docking/evaluate_docking_candidate.m` | `evaluate_docking_candidate` | `candidate,region,scene,robot_cfg,planner_cfg,task_positions,task_centroid` | `cost`（含 `total/reachability/coverage_ratio/covered_count/min_obstacle_dist_m`） | `planner_cfg.cost_weights.*`；`planner_cfg.reachability.threshold` | `docking/generate_docking_candidates:89-124` | ACTIVE（RC1 打分出口） | — | 保持；论文中区分 hard 约束与 score 项 |
| RC1/RC2 | 区域内多停靠选择（贪心覆盖） | `docking/select_multi_poses.m` | `select_multi_poses` | `candidates,region,scene,robot_cfg,planner_cfg` | `selected_poses,pose_info` | `planner_cfg.docking.ablation_mode` | `planning_strategy/bottom_up/plan_bottom_up:39`；`tests/test_planned_execution_consistency:8` | ACTIVE（Bottom-Up 专用路径） | Top-Down/Bidirectional 不经过此函数 | 论文中说明 Bottom-Up 用 `select_multi_poses`，Top-Down 用 `select_route_aware_multi_docks` |
| RC1 | 可达性场（桌面坐标系） | `docking/compute_reachability_field.m` | `compute_reachability_field` | `table_center,table_theta,scene_cfg,robot_cfg,...` | `reach_field` | `robot_cfg`/`scene_cfg` | `planning_strategy/common/augment_candidate_library:119`；`experiments/exp_c2_candidate_failure_diagnosis:84`、`exp_orientation_fast_validation:124` | ACTIVE（增强候选路径） | — | 仅在 `candidate_augmentation.enable=true` 时进入主流程 |
| RC1 | 升举高度候选扩展 | `docking/expand_lift_candidates.m` | `expand_lift_candidates` | `service_config,real_cfg` | `expanded` | `real_platform.lift.*`（默认 `enabled=false`） | `planning_strategy/common/append_region_plan:50` | ACTIVE（默认关闭） | — | 保持默认关闭，实机联调时再启用 |
| RC1 | service config 组装 | `docking/build_service_config.m` | `build_service_config` | `id,pose_info,summary_entry,real_cfg` | `service_config` | `real_platform_config` | `planning_strategy/common/append_region_plan:48`；`tests/test_mission_plan_export:11` | ACTIVE（EXECUTION 边界） | — | 保持 |

**RC1 结论：`RC1_PARTIAL`** — 主链 `task → candidate → footprint collision → dual-arm reachability → feasible/score relation` 已在默认主流程闭合；缺口是 robust / IK / bimanual 三个 API 未接入。

---

## 2. RC2 映射表（任务规划与移动路径优化）

| Research Content | Algorithm Function | Directory/File | Main Function | Input | Output | Config | Called By | Current Status | Missing Function | Recommended Action |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RC2 | 策略分发 | `planning_strategy/run_planning_strategy.m` | `run_planning_strategy` | `candidate_library,regions,scene,robot_cfg,planner_cfg` | `plan` | `planner_cfg.strategy.name`（默认 `bottom_up`） | `pipeline/stage2_docking_planning:12` | ACTIVE（RC2 唯一入口） | — | 保持为唯一策略入口 |
| RC2 | 统一 plan 结构 | `planning_strategy/common/build_unified_plan.m` | `build_unified_plan` | 策略输出 | plan 字段：`selected_poses/pose_to_region_map/task_sequence_summary/pose_info_list/region_order/preferred_visit_order/covered_tasks_predicted/uncovered_tasks_predicted/metrics/timing/service_configs` | `planner_cfg.strategy.*` | 三策略各自调用 | ACTIVE（“Task→Dock→Route”稳定接口） | — | 后续论文/实验均以该结构为准 |
| RC2 | 候选库构建 | `planning_strategy/common/build_candidate_library.m` | `build_candidate_library` | `regions,scene,robot_cfg,planner_cfg` | `candidate_library`（`regions/total_candidates/total_generation_time_s`） | RC1 配置 | `pipeline/stage2_docking_planning:7`；`main_mobile_manipulation_planning:18` | ACTIVE | — | 保持 |
| RC2 | 候选库增强（可选） | `planning_strategy/common/augment_candidate_library.m` | `augment_candidate_library` | 同上 | 增强候选 | `planner_cfg.candidate_augmentation.enable`（默认 false） | `build_candidate_library` 内部 | ACTIVE（默认 opt-in 关闭） | — | 保持“默认关闭保证 baseline 可复现” |
| RC2 | Bottom-Up 策略 | `planning_strategy/bottom_up/plan_bottom_up.m` | `plan_bottom_up` | 同上 | plan | `planner_cfg.strategy.bottom_up.weights.*` | `run_planning_strategy` | ACTIVE（默认策略） | — | 保持 |
| RC2 | Top-Down 策略 | `planning_strategy/top_down/plan_top_down.m` | `plan_top_down` | 同上 | plan | `planner_cfg.strategy.top_down.*` | `run_planning_strategy` | ACTIVE | — | 保持 |
| RC2 | Top-Down 区域顺序 | `planning_strategy/top_down/plan_region_order.m` | `plan_region_order` | `region_anchors,planner_cfg` | `region_order` | `strategy.top_down.region_order_method` | `plan_top_down:4-31` | ACTIVE | — | 保持 |
| RC2 | route-aware 多停靠选择 | `planning_strategy/top_down/select_route_aware_multi_docks.m` | `select_route_aware_multi_docks` | anchors/region_order | `selected_poses/task_sequence_summary/pose_info_list` | `strategy.top_down.max_docks_per_region` 等 | `plan_top_down` | ACTIVE | — | 与 `select_multi_poses` 的职责差异需在论文中写明 |
| RC2 | 停靠预算估计 | `planning_strategy/top_down/estimate_docking_budget.m` | `estimate_docking_budget` | 区域/候选 | `budget` | `strategy.top_down.*` | `select_route_aware_multi_docks` | ACTIVE | — | 保持 |
| RC2 | 覆盖修复 | `planning_strategy/top_down/repair_uncovered_tasks.m` | `repair_uncovered_tasks` | 未覆盖任务 | 修复结果 | `strategy.top_down.enable_coverage_repair` | `select_route_aware_multi_docks` | ACTIVE | — | 保持 |
| RC2 | Bidirectional 策略 | `planning_strategy/bidirectional/plan_bidirectional.m` | `plan_bidirectional` | 同上 | plan | `strategy.bidirectional.*` | `run_planning_strategy` | ACTIVE | — | 保持 |
| RC2 | 路径上下文精修 | `planning_strategy/bidirectional/refine_docks_with_route_context.m` / `refine_route_with_selected_docks.m` | 同名 | plan 片段 | 精修后 dock/route | `strategy.common.route_shortlist_size` | `plan_bidirectional` | ACTIVE | — | 保持 |
| RC2 | 区域锚点 | `planning_strategy/common/build_region_anchors.m` | `build_region_anchors` | 区域候选 | anchors | `strategy.top_down.max_anchors_per_region` | `plan_top_down` | ACTIVE | — | 保持 |
| RC2 | 区域代价矩阵 | `planning_strategy/common/build_region_cost_matrix.m` | `build_region_cost_matrix` | anchors/共享缓存 | cost matrix | `strategy.common.use_path_cost_cache` | `plan_region_order` | ACTIVE | — | 保持 |
| RC2 | 访问顺序规范化 | `planning_strategy/common/normalize_visit_order.m` | `normalize_visit_order` | order | 规范化 order | — | 三策略/测试 | ACTIVE | — | 保持（有 `test_normalize_visit_order`） |
| RC2 | 路径代价共享缓存 | `planning_strategy/common/SharedPathCostContext.m` 及 `get_cached_path_cost/query_path_with_shared_cache/initialize_path_cost_cache/set_shared_path_cost_source/get_shared_path_cost_diagnostics/path_cost_audit.m` | 同名 | 位姿对 | 代价/命中统计 | `strategy.common.use_path_cost_cache` | 三策略 + stage3 | ACTIVE | — | 保持 |
| RC2 | 策略对比/目标函数/一致性 | `compare_strategy_plans.m` / `compute_strategy_objective.m` / `validate_plan_consistency.m` / `build_plan_diagnostics.m` / `build_route_edge_set.m` / `validate_candidate_library.m` / `append_region_plan.m` | 同名 | plan | 指标/校验 | — | 策略与实验 | ACTIVE | — | 保持 |
| RC2 | 稀疏停靠图（P3/P4） | `global_planner/build_sparse_docking_graph.m` | `build_sparse_docking_graph` | `all_poses,grid_map,map_info,planner_cfg,pose_region_ids` | `sparse_graph`（`adjacency/cost_matrix/shortest_cost_matrix/region_ids`） | `planner_cfg.routing.mode='sparse_knn'`；`k=6,k_local=4,k_cross=1,max_local_k=12` | `pipeline/stage3_global_planning:35` | ACTIVE | — | 保持；formal sparse k=4 属实验覆盖 |
| RC2 | 稀疏图连通性校验 | `global_planner/validate_sparse_graph_connectivity.m` | `validate_sparse_graph_connectivity` | sparse_graph | `ok` | — | `build_sparse_docking_graph` | ACTIVE | — | 保持 |
| RC2 | 密集/稀疏路径拼接 | `global_planner/stitch_global_path.m` / `stitch_sparse_graph_path.m` | 同名 | `all_poses,route_order,...` | `global_path,segment_stats` | `execution.return_to_start` | `pipeline/stage3_global_planning:83-87` | ACTIVE（所有策略共用的唯一拼接点） | — | 保持 |
| RC2 | 距离矩阵（dense） | `global_planner/compute_path_cost_matrix.m` | `compute_path_cost_matrix` | `poses,grid_map,map_info,planner_cfg` | `dist_matrix` | `cfg.global_planner` | `stage3:43` | ACTIVE | — | 与 `experiments/exp_path_cost_reuse_audit` 配合审计 |
| RC2 | 访问顺序（最近邻/2-opt） | `global_planner/nearest_neighbor_order.m` / `two_opt_order.m` | 同名 | `dist_matrix` | `visit_order` | `planner_cfg.enable_2opt`（默认 false） | `stage3:63` | ACTIVE | — | 保持 |
| RC2 | 多算法 wrapper | `global_planner/run_*_wrapper.m`（26 个） | 同名 | `grid_map,start_pose,goal_pose,map_info,planner_cfg` | `path` | `cfg.global_planner` / `benchmark.algorithms` | `global_planner/run_path_planner.m:80-87` 等 case 分发；`pipeline/stage4_benchmark_all` | ACTIVE（benchmark 用）；部分未进主流程 | — | 不要把 wrapper 误判为重复代码 |
| RC2 | 公共工具（去重后） | `global_planner/common/`：`collision_free.m`、`nearest_free.m`、`path_length.m`、`prepare_start_goal.m`、`convert_path_grid2world.m`、`build_convex_segment.m`、`convex_neighbor_relation.m`、`find_convex_corners.m` | 同名 | 位姿/栅格 | 判定/长度 | — | 各 wrapper | ACTIVE（公共层已存在） | — | 后续继续把 wrapper 内联逻辑迁入该层 |
| RC2 | 路径后校验 | `global_planner/post_validate_path.m` / `reconstruct_path_grid.m` | 同名 | path/grid | 校验结果 | — | wrappers | ACTIVE | — | 保持 |
| RC2 | 主流程编排 | `pipeline/stage2_docking_planning.m` / `pipeline/stage3_global_planning.m` | 同名 | 见签名 | `optimal_poses,...,plan` / `global_path,visit_order,stats` | `planner_cfg.*` | `main_mobile_manipulation_planning.m:19-37` | ACTIVE | — | 保持接口不变 |

**RC2 结论：`RC2_PARTIAL`（接口稳定，鲁棒项未接入）** — “Task→Dock→Route”已由 `build_unified_plan` 固定为统一结构；`ROBUST_NOT_IN_CORE_PIPELINE`。

---

## 3. RC3 映射表（在线重规划基线）

| Research Content | Algorithm Function | Directory/File | Main Function | Input | Output | Config | Called By | Current Status | Missing Function | Recommended Action |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RC3 | 动态事件注入 | `replanning/apply_dynamic_event.m` | `apply_dynamic_event` | `planning_state,event_type,event_id` | `changed_state,event` | 事件类型常量：`ADD_LOCAL_OBSTACLE` / `INVALIDATE_DOCK` / `ADD_TASKS`（`:20-47`）；`experiments/dynamic_event_config.m` | `experiments/exp_dynamic_local_replanning_p5:20-39`；`tests/test_dynamic_event_determinism` | ACTIVE（基线事件集） | 论文事件名 `PATH_FAILURE` / `DOCK_FAILURE` / `TASK_COVERAGE_FAILURE` 未实现 | 论文中做“事件→论文事件”对照，不要改 P5 语义 |
| RC3 | 失效影响传播 | `replanning/analyze_change_impact.m` | `analyze_change_impact` | `planning_state,change_event` | `impact`（`invalid_docks/invalid_sparse_edges/path_refresh_edges/affected_regions/affected_tasks/candidate_refresh_regions/*_count/region_locality_ratio`） | 同上 | `local_replan:5`；`tests/test_change_impact_locality` | ACTIVE | 无 `TASK_COVERAGE_FAILURE`（覆盖失效）传播分支 | 保持；缺口写入论文“未来工作” |
| RC3 | 局部修复 | `replanning/local_replan.m` | `local_replan` | `planning_state,changed_state,change_event,impact` | `result`（`status/fallback/replanning_time_s/final_path_length/verified_completion/edge_recompute_ratio/dock_recompute_ratio/reused_edge_ratio/...`） | — | P5 实验；`tests/test_local_replan_*` | ACTIVE（RULE_BASED_REPLANNING_BASELINE） | 无 D* Lite 增量重规划、无学习型决策 | 保持 P5 语义；RC3 增强另开模块 |
| RC3 | 重规划可视化 | `visualization/visualize_dynamic_replan.m`、`demos/demo_dynamic_replanning.m` | 同名 | 前后状态 | 图 | `visualization_config` | demo/P5 | ACTIVE | — | 保持 |
| RC3 | P5 实验 | `experiments/exp_dynamic_local_replanning_p5.m` | 同名 | frozen P3/P4 L2 状态 | raw/summary CSV；full-replan 为 `REFERENCE_ESTIMATE`（`:151-190`） | `experiments/dynamic_event_config.m` | 人工/批量运行 | ACTIVE（EXPERIMENT） | — | 运行会写 results CSV，审计中不要覆盖 |
| RC3 | 增量规划算法 | `global_planner/run_dstar_lite_wrapper.m` | `run_dstar_lite_wrapper` | `grid_map,start_pose,goal_pose,map_info,planner_cfg` | `path` | 由 `cfg.global_planner='dstar_lite'` 触发 | `global_planner/run_path_planner.m:86-87`（case `'dstar_lite'`） | **IMPLEMENTED**（g/rhs/open 主循环，`:5-89`），但**未进入 RC3 主流程** | 无增量复用接口（每次调用重建状态）；无 tests 覆盖 | 保留；若要支撑 RC3 需新增“状态复用/edget 更新”接口 |

**RC3 结论：`RC3_BASELINE_ONLY`** — 现有 `replanning/` 是确定性、局部、规则式修复链，可作为 `RULE_BASED_REPLANNING_BASELINE`。

---

## 4. EXECUTION / EXPERIMENT / COMPATIBILITY 归属

| 归属 | 文件 | 说明 |
| --- | --- | --- |
| EXECUTION | `execution/get_planned_tasks_for_dock.m`、`simulate_planned_task_execution.m`、`update_task_state.m`、`predict_geometrically_reachable_tasks.m`、`separate_execution_status.m`、`estimate_execution_time.m`、`estimate_path_turn_angle.m`、`build_experiment_stats.m`、`print_experiment_stats.m` | 由 `stage3_global_planning:140-164,225` 调用；`predict_geometrically_reachable_tasks` 会再次调用 RC1 的 `evaluate_dual_arm_reachability` |
| EXECUTION | `integration/export_mission_plan_json.m`、`validate_mission_plan.m`、`import_ros_execution_log.m`、`validate_frame_convention.m`、`import_real_scene_json.m` | 离线规划→实机接口边界，基于 `planner_cfg.real_platform` |
| EXPERIMENT | `experiments/`（54 个脚本：P0–P18、cafeteria、real） | 消费 RC1/RC2/RC3 接口，不反向被主流程调用 |
| COMPATIBILITY | `config/legacy_visualization_config.m` | 兼容层：把 canonical `visualization_config` 摊平给旧 stage 可视化函数（`planner_config.m:204` 注入） |
| COMPATIBILITY | `calibration/load_*.m`（4 个，5–29 行） | 占位/加载器；未接入主流程 |
| COMPATIBILITY | `environment/build_scene.m`（无参基线构建器） | 仅供 `pipeline/stage1_build_scene.m:14` 与 `visualization/plot_main_scenario_publication.m:27` 使用 |
| COMMON | `utils/build_scene_from_cfg.m` | 实验/demo/cafeteria/tests 使用的 cfg 驱动构建器（与上者逻辑重复） |

---

## 5. 缺口与建议动作（本轮不实施，除文档外不改代码）

| 编号 | 缺口 | 严重度 | 涉及文件 | 建议动作 |
| --- | --- | --- | --- | --- |
| G1 | `evaluate_robust_reachability` / `evaluate_ik_feasibility` / `evaluate_bimanual_reachability` 未接入主流程 | 高（论文表述风险） | `manipulator/*` | 论文中明确为“已实现 API、未进入主流程”；或后续显式接入 stage2 并做 ablation |
| G2 | 论文事件名与 P5 事件名不一致 | 中 | `replanning/apply_dynamic_event.m:20-47` | 增加“事件映射表”注释或文档，不改语义 |
| G3 | 场景构建双实现 | 中 | `environment/build_scene.m` vs `utils/build_scene_from_cfg.m` | 长期收敛为单一实现 + thin wrapper；本轮不动 |
| G4 | D* Lite 无增量复用接口、无测试 | 中 | `global_planner/run_dstar_lite_wrapper.m` | 若 RC3 要写增量规划，新增状态复用接口 + `tests/test_dstar_lite_*` |
| G5 | RL 在线决策接口缺失 | 中 | 新增 `online_decision/`（本轮不建） | 见第 6 节 |
| G6 | wrapper 中仍有内联公共逻辑 | 低 | `global_planner/run_*_wrapper.m` | 逐步迁入 `global_planner/common/`，不改算法行为 |

---

## 6. 未来架构建议（仅映射，不搬迁现有稳定目录）

```text
Environment Layer      environment/ + utils/(scene,task,coordinate) + config/
Reachability Layer     manipulator/ + docking/ + collision/            (RC1)
Task Planning Layer    planning_strategy/ + global_planner/ + pipeline/(stage2,stage3)  (RC2)
Online Replanning Layer replanning/ (+ 未来 online_decision/)          (RC3)
Execution Layer        execution/ + integration/
```

建议新增（仅必要项）：

| 模块 | 目的 | 最少内容 |
| --- | --- | --- |
| `replanning/event_mapping.md` 或注释块 | 论文事件 ↔ P5 事件 | 纯文档 |
| `online_decision/` | RC3 决策层（DDQN 预备） | `build_decision_state.m`、`enumerate_actions.m`、`action_safety_mask.m`、`compute_decision_reward.m`、`apply_decision_action.m` |
| `replanning/incremental_planner_iface.m` | 增量规划统一接口（D* Lite 复用） | 初始化/更新/查询三接口 |

---

## 7. 审计发现汇总

- `DSTAR_LITE_STATUS`：**IMPLEMENTED**（`global_planner/run_dstar_lite_wrapper.m:5-89`），由 `run_path_planner.m:86-87` 分发，出现在 `stage4_benchmark_all.m:39-40` 的 `extended_algorithms`（默认 `benchmark.include_extended=false`）。无 tests 覆盖，未进入 RC3 主流程。
- `ROBUST_PIPELINE_STATUS`：**ROBUST_NOT_IN_CORE_PIPELINE**。
- `DDQN_READINESS`：**DDQN_INTERFACE_PARTIAL**（详见最终报告）。
- `CAFETERIA_ROLE`：见最终报告。
- 配置审计：(a)(b)(c)(d)(e) 全部成立，见最终报告。

---

## 8. 回归测试与工作区回归发现（2026-09-22）

测试环境：MATLAB R2022a，短生命周期 `-batch` 进程；测试脚本与日志保存在审计备份目录。

### 8.1 选定回归子集（14 项）

`test_main_smoke`、`test_candidate_generation`、`test_candidate_quick_filter_no_false_negative`、`test_bottom_up_regression`、`test_strategy_plan_consistency`、`test_preferred_visit_order`、`test_cafeteria_scene_planning_identity`、`test_cafeteria_navigation_only_mode`、`test_sparse_graph_connectivity`、`test_sparse_cache_semantic_identity`、`test_local_replan_route_feasibility`、`test_local_replan_fallback`、`test_visualization_smoke`、`test_world_grid_roundtrip`

结果：**pass=12 / fail=2 / missing=0**（日志 `regression_subset.log`）。

### 8.2 两个失败项与 HEAD 对照

| 测试 | 工作区（未提交改动） | HEAD `4fb8333` 快照 | 结论 |
| --- | --- | --- | --- |
| `test_main_smoke` | FAIL（`stats.completed=1`，期望 2；`STAGE2_STATUS=BLOCKED / PARTIAL_TASK_COVERAGE`） | PASS | 工作区回归 |
| `test_cafeteria_navigation_only_mode` | FAIL（proxy 模式 0 停靠、`CAFETERIA_CANDIDATE_COVERAGE_BLOCKED`） | 该测试文件在 HEAD 不存在（新增未跟踪） | 新功能未完成 |
| `test_cafeteria_scene_planning_identity` | PASS | 同上（HEAD 不存在） | — |
| `test_bottom_up_regression` | PASS | PASS | — |

### 8.3 默认主场景基线主链对照（headless）

同一脚本在 HEAD 快照与工作区分别运行 `stage1(scene+regions) → build_candidate_library → run_planning_strategy`：

| 指标 | HEAD `4fb8333` | 工作区（未提交改动） |
| --- | --- | --- |
| 候选总数 | 7893 | 1630 |
| 选中停靠位姿 | 27 | 14 |
| 预测覆盖率 | **36/36（100%）** | **18/36（50%）** |

根因（证据）：工作区把 `docking/generate_docking_candidates.m` 的候选生成从 `compute_reachability_field` 改为硬编码“南侧 standoff 采样”（`standoffs = [0.42 0.45 0.48 0.10 0.12 0.15 0.18 0.22 0.26]`、`lateral_offsets = -0.15:0.05:0.15`、`heading_offsets = ±20°`），并用 `reach_field = struct('candidates',...,'side_ids',ones(...))` 兜底，导致：
1. 非南侧任务不可覆盖 → 基线覆盖率跌至 50%；
2. RC1 叙事从“可达性场驱动”退化为“固定侧向采样”，与 RC1 建模描述不一致；
3. cafeteria navigation-only proxy 模式下 proxy 支撑候选为 0 → 停靠与路径链路被阻断。

### 8.4 重复与边界检查结论

| 项 | 结论 |
| --- | --- |
| `world2grid` / `grid2world` | 单一实现（`utils/`），消费者统一调用；`global_planner/common/convert_path_grid2world.m` 为薄包装 → **无重复** |
| `collision_free` / `nearest_free` | 单一实现（`global_planner/common/`），多个 wrapper 复用 → **无重复**（wrapper 不可删） |
| `path_length` vs `compute_path_length` | **真重复**：`global_planner/common/path_length.m` 与 `utils/compute_path_length.m` 语义相同，分别被稀疏/密集拼接路径使用 |
| 场景构建 | **真重复**：`environment/build_scene.m`（含本地 `generate_manual_tasks:149`）与 `utils/build_scene_from_cfg.m`（本地 `generate_manual_tasks:212`） |
| 硬件匹配场景 | **重复且其一为死代码**：`config/scene_config_lab_hardware_matched.m` 声明函数名 `scene_config`（与 `config/scene_config.m` 同名）、无任何调用者；生效的是 `experiments/hardware_matched_scene_profile_config.m` |
| 动态事件 | 单一实现（`replanning/apply_dynamic_event.m`）+ 单一配置（`experiments/dynamic_event_config.m`）→ 无重复 |
| 路线表示 | `visit_order` 与 `route_order`（= visit_order + 返航）并存，由 `stage3_global_planning:58-76` 明确区分 → 命名风险，非重复 |
| 机器人绘制 | 单一实现 `visualization/draw_robot.m` → 无重复 |
| `clf` 使用 | `draw_scene:6`（底图重绘，合理）、`draw_local_manipulation_coverage:11`、`draw_comprehensive_overview:15`、`animate_task_execution:22`（在帧循环之前，一次） → 无“每帧 clf”闪烁问题 |

### 8.5 配置审计结论

| 断言 | 结论 | 证据 |
| --- | --- | --- |
| scene_config = baseline，profile = 实验覆盖层 | 成立 | `config/resolve_effective_config.m:16-27`、`:39-49` |
| visualization_config = canonical | 成立 | `config/visualization_config.m:2,27-29`、`config/legacy_visualization_config.m:4-5` |
| legacy_visualization_config = 兼容层 | 成立（但存在两条构造路径，见下） | `config/planner_config.m:204`、`config/resolve_effective_config.m:28-30` |
| formal sparse k=4 = 实验覆盖 | 成立 | 默认 `config/planner_config.m:177` `k=6`；P5 覆盖 `experiments/dynamic_event_config.m:8`、`exp_dynamic_local_replanning_p5.m:59,201` |
| navigation_docking_only ≠ full_manipulation | 成立 | 默认仅在 cafeteria 入口 `main_cafeteria_mobile_manipulation_planning.m:10`；门控 `experiments/run_cafeteria_planning.m:72-89,189-197` |
| real_platform_config 只描述实机接口 | 成立 | `config/real_platform_config.m:1-22`（无 chassis/arm 几何） |

配置层问题（不计入算法改动）：
1. 两条 legacy 可视化构造路径（`planner_config.m:204` 与 `resolve_effective_config.m:28-30`）；
2. `config/` 反向依赖 `experiments/` 下的 profile 实现（`resolve_effective_config.m:41-46`）；
3. `config/scene_config_lab_hardware_matched.m` 文件名与函数名不一致且无调用者（死配置）；
4. checkout 默认 `routing.mode='sparse_knn', k=6` 未被任何正式实验验证（`planner_config.m:174-179` 注释已自述“do not treat this checkout default as a validated profile”）。

### 8.6 RC1 几何/可达性层级的真实可用性（按用户要求区分）

| 层级 | 实现位置 | 默认主流程可用性 | 证据 |
| --- | --- | --- | --- |
| hard geometry（硬约束） | `manipulator/evaluate_single_arm_reachability.m`（R_min/R_max/z_min/z_max） | **可用**（主链） | 被 `evaluate_dual_arm_reachability` 调用 |
| score threshold（评分阈值） | `config/planner_config.m:10` `reachability.threshold = 0.30` | **可用**（主链） | 与 hard geometry 联合判定 |
| IK | `manipulator/evaluate_ik_feasibility.m` | **不可用（UNUSED）** | 全仓库无非自身调用者 |
| robust / 扰动 | `manipulator/evaluate_robust_reachability.m` | **不可用（UNUSED）** | 同上 → 见 `ROBUST_NOT_IN_CORE_PIPELINE` |
| bimanual | `manipulator/evaluate_bimanual_reachability.m` | **不可用（UNUSED）** | 同上 |
| `legacy_validated` | `utils/robot_geometry_profile.m:16-17` | 等价于 `robot_config()` 原值 | 方法体直接 `return`，不改任何字段 |
| `physical_robot` | `utils/robot_geometry_profile.m:18-21` | 覆盖 `base_spacing=0.40`、`arm_base_mount_x=0.06`、`arm_base_mount_z=0.40` | 使用方：cafeteria 校验测试、`resolve_effective_config:17`、geometry smoke 实验；**主流程不可达** |

主流程配置边界（影响 RC1/RC2 场景一致性）：
- `main_mobile_manipulation_planning.m:14` 以**零参数**调用 `pipeline/stage1_build_scene.m`，该函数内部硬取 `robot_config()` / `scene_config()` / `planner_config()`，不经过 `resolve_effective_config`。
- 因此 `stage1_build_scene.m:6-8` 的 `lab_hardware_matched → robot_geometry_profile('physical_robot')` 分支在主流程中**不可达**（`config/scene_config.m` 仅在 `:366` 以注释形式提到该 `scene_name`）。
- 结论：主流程恒为 baseline 场景 + `robot_config()` 几何；“实验 profile 场景”只能经 `experiments/*` + `build_scene_from_cfg` 路径进入。这与配置审计断言 (a) 一致，但属于结构性硬编码边界，而非配置覆盖。