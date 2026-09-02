function [scene, global_path, all_poses, visit_order, executed_tasks, total_path_len, stats] = stage3_global_planning(scene, optimal_poses, task_sequence_summary, task_table_map, robot_cfg, planner_cfg, pose_info_list, timing_input, plan)
    if nargin < 9
        plan = [];
    end
    if nargin < 8
        timing_input = struct();
    end
    if nargin < 7
        pose_info_list = {};
    end

    stats = struct();
    if isstruct(plan) && isfield(plan, 'shared_path_cost_context') && ...
            isa(plan.shared_path_cost_context, 'SharedPathCostContext')
        planner_cfg.runtime.shared_path_cost_context = plan.shared_path_cost_context;
    end

    fprintf('\n=== 阶段三: 全局路径规划与执行 ===\n');
    fprintf('  当前算法: %s\n', planner_cfg.global_planner);

    fprintf('\n--- 8. 连接起点和所有最优停靠位姿 ---\n');
    all_poses = [scene.start_pose; optimal_poses];
    n_poses = size(all_poses, 1);
    fprintf('  需连接位姿总数: %d (起点 + %d停靠位姿)\n', n_poses, n_poses - 1);

    routing_mode = get_routing_mode(planner_cfg);
    fprintf('  计算距离矩阵（%s, routing=%s）...\n', planner_cfg.global_planner, routing_mode);
    t_dist = tic;
    strategy_source = upper(get_plan_field(plan, 'strategy_name', 'bottom_up'));
    path_cost_audit('set_source', [strategy_source '_STAGE3']);
    set_shared_path_cost_source(planner_cfg, [strategy_source '_STAGE3']);
    sparse_graph = [];
    if strcmpi(routing_mode, 'sparse_knn')
        pose_region_ids = get_pose_region_ids(plan, scene, n_poses);
        sparse_graph = build_sparse_docking_graph(all_poses, scene.grid_map, ...
            scene.map_info, planner_cfg, pose_region_ids);
        if ~sparse_graph.connectivity.ok
            error('stage3_global_planning:SparseGraphDisconnected', ...
                'Sparse graph failed connectivity validation after repair.');
        end
        dist_matrix = sparse_graph.shortest_cost_matrix;
    else
        dist_matrix = compute_path_cost_matrix(all_poses, scene.grid_map, scene.map_info, planner_cfg);
    end
    distance_matrix_time_s = toc(t_dist);
    fprintf('  距离矩阵计算完成，用时 %.3f s\n', distance_matrix_time_s);

    enable_2opt = false;
    if isfield(planner_cfg, 'enable_2opt')
        enable_2opt = planner_cfg.enable_2opt;
    end

    fprintf('\n--- 9. 最近邻策略生成访问顺序 ---\n');
    if enable_2opt
        fprintf('  启用 2-opt 优化\n');
    end
    t_route = tic;
    [preferred_ok, preferred_order] = get_preferred_visit_order(plan, n_poses);
    if preferred_ok
        visit_order = preferred_order;
        route_order_source = 'strategy_preferred';
    else
        visit_order = nearest_neighbor_order(dist_matrix, 1, enable_2opt);
        route_order_source = 'nearest_neighbor';
    end
    route_order_time_s = toc(t_route);
    fprintf('  访问顺序: [%s]\n', num2str(visit_order));
    fprintf('  route order source: %s\n', route_order_source);

    route_order = visit_order;
    if isfield(planner_cfg, 'execution') && isfield(planner_cfg.execution, 'return_to_start') && ...
            planner_cfg.execution.return_to_start
        route_order = [route_order, 1];
        fprintf('  已启用返航: 路径拼接顺序追加起点 1\n');
    end
    fprintf('  路径拼接顺序: [%s]\n', num2str(route_order));

    fprintf('\n--- 10-11. 生成全局路径 ---\n');
    t_global_path = tic;
    path_cost_audit('set_source', 'FINAL_ROUTE');
    set_shared_path_cost_source(planner_cfg, 'FINAL_ROUTE');
    if strcmpi(routing_mode, 'sparse_knn')
        [global_path, segment_stats] = stitch_sparse_graph_path(all_poses, route_order, sparse_graph);
    else
        [global_path, segment_stats] = stitch_global_path(all_poses, route_order, ...
                                          scene.grid_map, scene.map_info, planner_cfg);
    end
    global_path_time_s = toc(t_global_path);
    total_path_len = compute_path_length(global_path);
    fprintf('  全局路径长度: %.2f m\n', total_path_len);

    t_execution_sim = tic;
    exec_stats = estimate_execution_time(global_path, route_order, all_poses, ...
                                         task_sequence_summary, segment_stats, planner_cfg);

    if planner_cfg.visualization.show_global_path && ~runtime_flag(planner_cfg, 'disable_visualization', false)
        ordered_route_poses = all_poses(route_order, :);
        visit_strs = cell(length(route_order), 1);
        visit_strs{1} = '起点';
        for vi = 2:length(route_order)
            if route_order(vi) == 1
                visit_strs{vi} = '返回起点';
            else
                s = task_sequence_summary{route_order(vi)-1};
                visit_strs{vi} = sprintf('T%d-D%d', s.table_id, s.dock_index);
            end
        end

        hFig3 = figure('Name', '阶段三: RCH-MT-MMP 全局路径规划', 'Position', [80, 80, 900, 700]);
        draw_scene(scene, hFig3);
        draw_docking_result(optimal_poses, scene, hFig3, visit_order, pose_info_list, planner_cfg.visualization);
        draw_global_path(global_path, hFig3, ordered_route_poses, visit_strs);
        title(sprintf('阶段三: RCH-MT-MMP 全局路径 — %.2f m | %d次停靠 | 返回起点: %s', ...
              total_path_len, size(optimal_poses,1), bool2cn(route_order(end) == 1)), 'FontSize', 12);
        drawnow;
    end

    fprintf('\n--- 12-15. 模拟执行过程 ---\n');
    predicted_tasks = [];
    for i = 1:numel(task_sequence_summary)
        if isstruct(task_sequence_summary{i}) && isfield(task_sequence_summary{i}, 'covered_tasks')
            predicted_tasks = union(predicted_tasks, task_sequence_summary{i}.covered_tasks, 'stable');
        end
    end
    commanded_tasks = [];
    executed_tasks = [];
    verified_tasks = [];

    for vi = 1:length(visit_order)
        pose_idx = visit_order(vi);
        if pose_idx == 1
            continue;
        end

        s = task_sequence_summary{pose_idx-1};
        fprintf('\n  停靠 %d/%d (桌子%d-停%d):\n', vi-1, length(visit_order)-1, ...
                s.table_id, s.dock_index);
        actual_pose = all_poses(pose_idx, :);
        dock_index = pose_idx - 1;
        [~, dock_commands] = get_planned_tasks_for_dock( ...
            task_sequence_summary, pose_info_list, dock_index);
        dock_commands = setdiff(dock_commands, commanded_tasks, 'stable');
        [dock_executed, dock_verified] = simulate_planned_task_execution( ...
            scene, dock_commands, actual_pose, robot_cfg, planner_cfg);

        commanded_tasks = union(commanded_tasks, dock_commands, 'stable');
        executed_tasks = union(executed_tasks, dock_executed, 'stable');
        verified_tasks = union(verified_tasks, dock_verified, 'stable');
        scene = update_task_state(scene, dock_verified);

        fprintf('    指令任务: [%s]\n', num2str(dock_commands));
        fprintf('    执行器报告: [%s]\n', num2str(dock_executed));
        fprintf('    仿真验证: [%s]\n', num2str(dock_verified));

        remaining = setdiff(1:scene.num_tasks, verified_tasks);
        fprintf('    已验证: %d/%d, 剩余: %d\n', ...
                length(verified_tasks), scene.num_tasks, length(remaining));

        if isempty(remaining)
            fprintf('\n  所有任务完成!\n');
            break;
        end
    end
    execution_sim_time_s = toc(t_execution_sim);

    fprintf('\n===== RCH-MT-MMP 规划完成 =====\n');
    fprintf('总任务数: %d\n', scene.num_tasks);
    execution_status = separate_execution_status(predicted_tasks, commanded_tasks, ...
        executed_tasks, verified_tasks);
    predicted_rate = 100 * numel(execution_status.predicted_tasks) / max(scene.num_tasks, 1);
    commanded_rate = 100 * numel(execution_status.commanded_tasks) / max(scene.num_tasks, 1);
    verified_rate = 100 * numel(execution_status.verified_tasks) / max(scene.num_tasks, 1);
    fprintf('预测覆盖率: %.1f%%\n', predicted_rate);
    fprintf('指令任务率: %.1f%%\n', commanded_rate);
    fprintf('验证完成率: %.1f%%\n', verified_rate);
    fprintf('停靠位姿数: %d\n', size(optimal_poses, 1));
    fprintf('任务路径长度: %.2f m\n', exec_stats.task_path_len_m);
    fprintf('返回起点路径长度: %.2f m\n', exec_stats.return_path_len_m);
    fprintf('全局路径长度: %.2f m\n', total_path_len);

    total_cost_all = 0;
    for i = 1:length(pose_info_list)
        if ~isempty(pose_info_list{i}) && isfield(pose_info_list{i}, 'cost_breakdown') ...
                && ~isempty(pose_info_list{i}.cost_breakdown) ...
                && isfield(pose_info_list{i}.cost_breakdown, 'total')
            total_cost_all = total_cost_all + pose_info_list{i}.cost_breakdown.total;
        end
    end
    fprintf('总代价: %.4f\n', total_cost_all);

    timing.distance_matrix_time_s = distance_matrix_time_s;
    timing.routing_mode = routing_mode;
    timing.route_order_time_s = route_order_time_s;
    timing.global_path_time_s = global_path_time_s;
    timing.execution_sim_time_s = execution_sim_time_s;
    timing.scene_time_s = get_timing_field(timing_input, 'scene_time_s', 0);
    timing.docking_time_s = get_timing_field(timing_input, 'docking_time_s', 0);
    timing.candidate_generation_time_s = get_timing_field(timing_input, 'candidate_generation_time_s', 0);
    timing.strategy_total_time_s = get_timing_field(timing_input, 'strategy_total_time_s', 0);
    timing.region_order_time_s = get_timing_field(timing_input, 'region_order_time_s', 0);
    timing.local_selection_time_s = get_timing_field(timing_input, 'local_selection_time_s', 0);
    timing.route_refinement_time_s = get_timing_field(timing_input, 'route_refinement_time_s', 0);
    timing.strategy_name = get_plan_field(plan, 'strategy_name', 'bottom_up');
    timing.region_order = get_plan_field(plan, 'region_order', []);
    timing.route_order_source = route_order_source;
    timing.coverage_repair_count = 0;
    timing.strategy_objective_value = get_plan_field(plan, 'objective_value', 0);
    timing.path_cache_hits = 0;
    timing.path_cache_misses = 0;
    if isstruct(plan) && isfield(plan, 'metrics')
        timing.coverage_repair_count = get_timing_field(plan.metrics, 'coverage_repair_count', 0);
        timing.path_cache_hits = get_timing_field(plan.metrics, 'cache_hits', 0);
        timing.path_cache_misses = get_timing_field(plan.metrics, 'cache_misses', 0);
    end
    timing.stage3_time_s = distance_matrix_time_s + route_order_time_s + global_path_time_s;
    shared_cache = get_shared_path_cost_diagnostics(planner_cfg);
    timing.shared_path_cache = shared_cache;
    timing.shared_cache_query_count = shared_cache.cache_query_count;
    timing.shared_cache_hit_count = shared_cache.cache_hit_count;
    timing.shared_cache_miss_count = shared_cache.cache_miss_count;
    timing.expensive_planner_calls = shared_cache.expensive_planner_calls;
    timing.total_planning_time_s = timing.scene_time_s + timing.docking_time_s + ...
                                   distance_matrix_time_s + route_order_time_s + global_path_time_s;

    experiment_stats = build_experiment_stats(scene, verified_tasks, optimal_poses, global_path, ...
        exec_stats, timing, segment_stats, task_sequence_summary, execution_status);

    stats.total_tasks = scene.num_tasks;
    stats.completed   = length(verified_tasks);
    stats.path_len    = total_path_len;
    stats.n_poses     = size(optimal_poses, 1);
    stats.rate        = verified_rate;
    stats.predicted_coverage_rate = predicted_rate;
    stats.commanded_task_rate = commanded_rate;
    stats.verified_completion_rate = verified_rate;
    stats.execution_status = execution_status;
    stats.num_tables  = size(scene.scene_cfg.table_positions, 1);
    stats.table_pos   = scene.scene_cfg.table_positions;
    stats.visit_order = visit_order;
    stats.route_order = route_order;
    stats.route_order_source = route_order_source;
    stats.routing_mode = routing_mode;
    stats.sparse_graph = sparse_graph;
    stats.strategy_name = get_plan_field(plan, 'strategy_name', 'bottom_up');
    stats.plan = plan;
    if isstruct(plan) && isfield(plan, 'metrics')
        stats.strategy_metrics = plan.metrics;
        stats.coverage_repair_count = get_timing_field(plan.metrics, 'coverage_repair_count', 0);
        stats.strategy_objective_value = get_plan_field(plan, 'objective_value', 0);
    else
        stats.coverage_repair_count = 0;
        stats.strategy_objective_value = 0;
    end
    stats.return_to_start = route_order(end) == 1;
    stats.return_path_len_m = exec_stats.return_path_len_m;
    stats.task_path_len_m = exec_stats.task_path_len_m;
    stats.segment_stats = segment_stats;
    stats.exec_stats = exec_stats;
    stats.timing = timing;
    stats.experiment_stats = experiment_stats;

    if ~runtime_flag(planner_cfg, 'disable_visualization', false)
        fprintf('\n--- 生成综合总图 ---\n');

        hFigFinal = draw_comprehensive_overview(scene, optimal_poses, global_path, ...
                                                 all_poses, visit_order, verified_tasks, ...
                                                 task_table_map, stats, pose_info_list, planner_cfg.visualization); %#ok<NASGU>
    end

    fprintf('\n输出:\n');
    fprintf('  全局路径: [%d个路径点]\n', size(global_path, 1));
    fprintf('  最优停靠位姿: [%d个]\n', size(optimal_poses, 1));
    fprintf('  机械臂任务序列: [%d个任务]\n', length(executed_tasks));
    if ~runtime_flag(planner_cfg, 'disable_visualization', false)
        fprintf('  可视化图片: 4张\n');
    else
        fprintf('  可视化已禁用\n');
    end
end

function mode = get_routing_mode(planner_cfg)
    mode = 'dense';
    if isfield(planner_cfg, 'routing') && isfield(planner_cfg.routing, 'mode') && ...
            ~isempty(planner_cfg.routing.mode)
        mode = lower(char(planner_cfg.routing.mode));
    end
    if ~ismember(mode, {'dense', 'sparse_knn'})
        error('stage3_global_planning:UnknownRoutingMode', ...
            'Unknown routing mode "%s".', mode);
    end
end

function ids = get_pose_region_ids(plan, scene, n_poses)
    ids = ones(n_poses, 1);
    ids(1) = 0;
    table_region_id = [];
    if isfield(scene, 'scene_cfg') && isfield(scene.scene_cfg, 'table_region_id')
        table_region_id = reshape(scene.scene_cfg.table_region_id, [], 1);
    end
    if isstruct(plan) && isfield(plan, 'pose_info_list') && numel(plan.pose_info_list) == n_poses - 1
        for i = 1:numel(plan.pose_info_list)
            info = plan.pose_info_list{i};
            if isstruct(info) && isfield(info, 'table_id') && ...
                    info.table_id >= 1 && info.table_id <= numel(table_region_id)
                ids(i + 1) = table_region_id(info.table_id);
            elseif isstruct(plan) && isfield(plan, 'pose_to_region_map') && ...
                    numel(plan.pose_to_region_map) == n_poses - 1
                ids(i + 1) = plan.pose_to_region_map(i);
            end
        end
    elseif isstruct(plan) && isfield(plan, 'pose_to_region_map') && ...
            numel(plan.pose_to_region_map) == n_poses - 1
        ids(2:end) = reshape(plan.pose_to_region_map, [], 1);
    end
end

function s = bool2cn(tf)
    if tf
        s = '是';
    else
        s = '否';
    end
end

function v = get_timing_field(timing_input, field_name, default_value)
    if isstruct(timing_input) && isfield(timing_input, field_name)
        v = timing_input.(field_name);
    else
        v = default_value;
    end
end

function tf = runtime_flag(cfg, field_name, default_value)
    tf = default_value;
    if isfield(cfg, 'runtime') && isfield(cfg.runtime, field_name)
        tf = logical(cfg.runtime.(field_name));
    end
end

function [ok, order] = get_preferred_visit_order(plan, n_poses)
    ok = false;
    order = [];
    if ~isstruct(plan) || ~isfield(plan, 'preferred_visit_order') || isempty(plan.preferred_visit_order)
        return;
    end
    order = reshape(plan.preferred_visit_order, 1, []);
    expected = 1:n_poses;
    if numel(order) ~= n_poses || numel(unique(order)) ~= n_poses || ~isequal(sort(order), expected)
        error('stage3_global_planning:InvalidPreferredVisitOrder', ...
            'preferred_visit_order must contain every node in [start; selected_poses] exactly once.');
    end
    ok = true;
end

function v = get_plan_field(plan, field_name, default_value)
    if isstruct(plan) && isfield(plan, field_name)
        v = plan.(field_name);
    else
        v = default_value;
    end
end
