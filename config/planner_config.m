function cfg = planner_config()

    % Path planning consumes robot geometry through this config contract.
    % Stage 1 overwrites it with the resolved effective profile.
    cfg.robot_cfg = robot_config();

    cfg.cost_weights.w_reach        = 0.25;
    cfg.cost_weights.w_coverage     = 0.20;
    cfg.cost_weights.w_orientation  = 0.12;
    cfg.cost_weights.w_safety       = 0.08;
    cfg.cost_weights.w_approach     = 0.20;
    cfg.cost_weights.w_maneuver     = 0.15;

    cfg.reachability.threshold = 0.30;   % 0.45过严：任务点z=0.50m接近机械臂z_max，需用较保守阈值保证可覆盖
    cfg.reachability.mode = 'nominal';
    cfg.reachability.robust_success_fraction_min = 1.00;
    cfg.docking_error_model.source = 'configured_bound';
    cfg.docking_error_model.measured = false;
    cfg.docking_error_model.confidence = 'unverified_bound';
    cfg.docking_error_model.position_offsets_m = [0, 0; 0.02, 0; -0.02, 0; 0, 0.02; 0, -0.02];
    cfg.docking_error_model.yaw_offsets_rad = [0, 0.03, -0.03];

    cfg.candidate.safety_distance_m = 0.25;
    cfg.candidate.side_switch_penalty = 0.15;

    % Task-driven augmentation is opt-in so the maintained baseline remains
    % exactly reproducible. Sampling values are relative to scene geometry.
    cfg.candidate_augmentation.enable = false;
    cfg.candidate_augmentation.variant = 'joint';
    cfg.candidate_augmentation.normal_offset_step_factors = [0, 0.25, 0.5, 0.75];
    cfg.candidate_augmentation.tangential_offset_step_factors = [-0.5, 0, 0.5];
    cfg.candidate_augmentation.baseline_orientation_step_factors = [-2, -1, 0, 1, 2];
    cfg.candidate_augmentation.refined_orientation_step_factors = ...
        [-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2];
    cfg.candidate_augmentation.orientation_step_rad = pi / 12;
    cfg.candidate_augmentation.max_supplementary_candidates_per_task = 500;
    cfg.candidate_augmentation.max_supplementary_candidates_per_region = 40;
    cfg.candidate_augmentation.max_supplementary_candidates_total = 400;
    cfg.candidate_augmentation.position_tolerance_m = 0.01;
    cfg.candidate_augmentation.heading_tolerance_rad = deg2rad(2);

    % Conservative candidate-task pair prefilter. It is disabled by default
    % so the maintained baseline remains exactly reproducible. P2 experiments
    % enable it explicitly after a false-negative audit.
    cfg.candidate_pair_prefilter.enable = false;

    % ablation_mode: 消融实验模式，仅影响 stage2 停靠位姿选择逻辑。
    %   'full'                    — 完整流程（默认），所有代价项均参与
    %   'no_reachability'         — 去掉可达性评分项 (mean_reach_new = 0)
    %   'single_docking_per_table'— 每张桌子仅选择 1 个停靠位姿
    %   'nearest_docking'         — 仅选择距离桌子中心最近的候选位姿
    %   'no_coverage_cost'        — 仅去掉代价函数中的 coverage cost (total_cost 剔除 w_c*coverage)
    %   'no_coverage_priority'    — 仅去掉贪心 score 中的覆盖优先级 (new_coverage_term = 0)
    %   'no_coverage_all'         — 同时去掉 coverage cost 和 coverage priority
    cfg.docking.ablation_mode = 'full';

    cfg.strategy.name = 'bottom_up';
    % Opt-in exact finite-pool task/navigation experiment. Hard budgets fail
    % explicitly; they never truncate search and retain an optimality label.
    cfg.strategy.joint.max_task_count = 16;
    cfg.strategy.joint.max_enumerated_combinations = 1000000;
    cfg.strategy.joint.max_covers = 100000;
    cfg.strategy.joint.max_selected_docks = 8;
    cfg.strategy.joint.max_cover_products = 100000;
    cfg.strategy.joint.max_route_evaluations = 1000000;
    cfg.strategy.joint.max_navigation_queries = 10000;
    cfg.strategy.common.route_shortlist_size = 25;
    cfg.strategy.common.preserve_coverage_union = true;
    cfg.strategy.common.use_path_cost_cache = true;
    cfg.strategy.diagnostics.position_tolerance_m = 0.05;
    cfg.strategy.diagnostics.yaw_tolerance_rad = deg2rad(5);

    cfg.strategy.bottom_up.weights.coverage_gain = 0.35;
    cfg.strategy.bottom_up.weights.reachability = 0.25;
    cfg.strategy.bottom_up.weights.local_cost = 0.20;
    cfg.strategy.bottom_up.weights.intra_region_move = 0.10;
    cfg.strategy.bottom_up.weights.side_switch = 0.10;
    cfg.strategy.bottom_up.transition_normalization_m = 3.0;

    cfg.strategy.top_down.anchor_mode = 'best_feasible_candidate';
    cfg.strategy.top_down.region_order_method = 'nearest_neighbor';
    cfg.strategy.top_down.enable_region_2opt = true;
    cfg.strategy.top_down.max_anchors_per_region = 8;
    cfg.strategy.top_down.max_anchor_pair_queries = 4;
    cfg.strategy.top_down.preserve_side_diversity = true;
    cfg.strategy.top_down.preserve_unique_task_cover = true;
    cfg.strategy.top_down.max_docks_per_region = 3;
    cfg.strategy.top_down.enable_coverage_repair = true;
    cfg.strategy.top_down.max_repair_docks_per_region = 2;
    cfg.strategy.top_down.weights.coverage_gain = 0.35;
    cfg.strategy.top_down.weights.reachability = 0.20;
    cfg.strategy.top_down.weights.local_cost = 0.15;
    cfg.strategy.top_down.weights.route_context = 0.25;
    cfg.strategy.top_down.weights.side_switch = 0.05;

    cfg.strategy.bidirectional.max_iterations = 5;
    cfg.strategy.bidirectional.improvement_tolerance = 1e-3;
    cfg.strategy.bidirectional.require_non_decreasing_coverage = true;
    cfg.strategy.bidirectional.enable_redundant_removal = true;
    cfg.strategy.bidirectional.enable_single_replacement = true;
    cfg.strategy.bidirectional.enable_two_to_one_replacement = true;
    cfg.strategy.bidirectional.enable_route_reinsertion = true;
    cfg.strategy.bidirectional.use_shared_path_cost_cache = false;
    cfg.strategy.bidirectional.objective_weights.uncovered = 1000;
    cfg.strategy.bidirectional.objective_weights.docks = 1;
    cfg.strategy.bidirectional.objective_weights.path = 1;
    cfg.strategy.bidirectional.objective_weights.local_cost = 1;
    cfg.strategy.bidirectional.objective_weights.execution_time = 0.1;

    %% ===================== Runtime / batch experiment controls =====================
    % Main program defaults keep visualization enabled. Batch experiment
    % scripts override these fields to run headlessly.
    cfg.runtime.headless = false;
    cfg.runtime.disable_visualization = false;
    cfg.runtime.close_figures_after_save = true;
    cfg.runtime.figure_visible = 'on';          % 'on' / 'off'
    cfg.runtime.disable_video = false;
    cfg.runtime.fast_experiment_mode = false;
    cfg.runtime.suppress_plot_warning = true;
    cfg.runtime.quiet = false;
    cfg.runtime.quiet_stage2 = false;
    % C2-4 incremental D* Lite is opt-in; frozen RC2 behavior remains unchanged.
    cfg.incremental_replanning_enabled = false;

    cfg.astar.allow_diagonal = true;
    cfg.astar.heuristic_weight = 1.0;

    cfg.convex_astar.enable = true;
    cfg.convex_astar.max_corner_ratio = 0.025;
    cfg.convex_astar.max_corner_count = inf;
    cfg.convex_astar.fallback_planner = 'jps';
    cfg.convex_astar.enable_fallback = true;
    cfg.convex_astar.heuristic = 'octile';
    cfg.convex_astar.debug = false;

    cfg.benchmark.enable = true;
    cfg.benchmark.include_convex_astar = false;

    % 默认 benchmark 只运行稳定核心算法，避免主流程调试耗时过长。
    cfg.benchmark.algorithms = {'astar', 'jps', 'gbfs', 'theta_star'};
    cfg.benchmark.include_extended = false;
    cfg.benchmark.include_stochastic = false;
    cfg.benchmark.suppress_warnings = true;

    % order_mode: 'fixed_order' — 所有算法共用主流程 route_order
    %             'independent_order' — 各算法独立计算距离矩阵和访问顺序
    cfg.benchmark.order_mode = 'fixed_order';

    % include_return_to_start: true  — 路径拼接包含返回起点段
    %                          false — 路径拼接不包含返回起点
    cfg.benchmark.include_return_to_start = true;

    cfg.timing.enable = true;
    cfg.timing.verbose = true;

    cfg.execution.return_to_start = true;
    cfg.execution.nominal_linear_speed_mps = 0.30;
    cfg.execution.nominal_angular_speed_radps = 0.50;
    cfg.execution.docking_align_time_s = 3.0;
    cfg.execution.arm_task_time_s = 6.0;
    cfg.execution.arm_retract_time_s = 1.5;
    cfg.execution.dual_arm_parallel = true;
    cfg.execution_time.navigation_speed_mps = cfg.execution.nominal_linear_speed_mps;
    cfg.execution_time.docking_time_per_pose_s = cfg.execution.docking_align_time_s;
    cfg.execution_time.manipulation_time_per_task_s = cfg.execution.arm_task_time_s;
    cfg.execution_time.arm_transition_time_s = cfg.execution.arm_retract_time_s;
    cfg.execution_time.side_switch_time_s = 0.0;
    cfg.execution_time.turning_speed_radps = cfg.execution.nominal_angular_speed_radps;
    cfg.execution_time.lift_speed_mps = 0.0;
    cfg.execution_time.replanning_penalty_s = 0.0;

    cfg.global_planner = 'convex_astar';
    cfg.footprint_aware_navigation = true;
    % Recover passages blocked only by the all-heading disk approximation.
    % Every recovered fixed-heading route retains raw-map footprint checks.
    cfg.navigation.fixed_heading_fallback = true;
% graph_planner 图搜索类:
% cfg.global_planner = 'astar';           % A* alternative
% cfg.global_planner = 'convex_astar';    % Convex-corner A* (current default)
% cfg.global_planner = 'jps';             % 跳跃点搜索
% cfg.global_planner = 'gbfs';            % Greedy Best-First Search
% cfg.global_planner = 'dijkstra';        % Dijkstra
% cfg.global_planner = 'dstar';           % D* 动态规划
% cfg.global_planner = 'lpa_star';        % LPA* 终身规划A*
% cfg.global_planner = 'voronoi';         % Voronoi 通道规划
% cfg.global_planner = 'dstar_lite';      % D* Lite
% cfg.global_planner = 'theta_star';      % Theta* any-angle
% cfg.global_planner = 'lazy_theta_star'; % Lazy Theta*
% cfg.global_planner = 'hybrid_astar';    % Hybrid A* 运动学约束
% sample_planner 采样类:
% cfg.global_planner = 'rrt';             % RRT 快速探索随机树
% cfg.global_planner = 'rrt_star';        % RRT* 最优采样
% cfg.global_planner = 'rrt_connect';     % RRT-Connect 双向搜索
% cfg.global_planner = 'informed_rrt';    % Informed RRT* 椭圆采样
% evolutionary_planner 进化类:
% cfg.global_planner = 'aco';             % 蚁群算法
% cfg.global_planner = 'pso';             % 粒子群算法
% cfg.global_planner = 'ga';              % 遗传算法
    cfg.inflation_radius_m = 0.25;

    % P3/P4 routing infrastructure. Formal experiments set routing mode and k
    % explicitly; do not treat this checkout default as a validated profile.
    cfg.routing.mode = 'sparse_knn';              % 'dense' / 'sparse_knn'
    cfg.routing.sparse_knn.k = 6;
    cfg.routing.sparse_knn.k_local = 4;
    cfg.routing.sparse_knn.k_cross = 1;
    cfg.routing.sparse_knn.max_local_k = 12;
    cfg.routing.sparse_knn.enable_region_rule = true;

    cfg.enable_2opt = false;
    cfg.edge_cost.lambda_theta = 0.15;

    %% ===================== 批量实验配置（默认关闭） ======================
    % 以下配置仅由 experiments/exp_random_batch.m 使用，
    % 不影响主程序 main_mobile_manipulation_planning.m。
    %
    % num_trials   : 每种规模的随机重复次数，默认 20
    % scales       : 实验场景规模列表
    % base_seed    : 基础随机种子，第 i 个 trial 使用 base_seed + trial_idx
    % run_benchmark: 批量实验是否运行多算法 benchmark，默认关闭以提升效率

    cfg.experiment.enabled       = false;
    cfg.experiment.num_trials    = 20;
    cfg.experiment.scales        = {'small', 'medium', 'large'};
    cfg.experiment.base_seed     = 2026;
    cfg.experiment.run_benchmark = false;
    cfg.experiment.quick_mode    = false;

    % Compatibility view consumed by the original stage visualizers. All
    % maintained visualization defaults are defined in visualization_config.
    cfg.visualization = legacy_visualization_config();

    % Stable offline-planning/robot-execution boundary. The returned
    % structure is consumed by service-configuration and JSON interfaces.
    cfg.real_platform = real_platform_config();
    cfg.experiment_protocol = experiment_protocol_config(cfg);
end
