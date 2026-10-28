function row = run_p3p4_validation_route(state_file, routing_mode, k, cache_on, mode_label, output_csv)
%RUN_P3P4_VALIDATION_ROUTE Run one routing mode and append one CSV row.

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    if nargin < 5 || isempty(mode_label)
        mode_label = sprintf('%s_cache_%d', routing_mode, cache_on);
    end
    if nargin < 6 || isempty(output_csv)
        out_dir = fullfile(root, 'results', 'repair_20261027');
        if ~isfolder(out_dir), mkdir(out_dir); end
        output_csv = fullfile(out_dir, 'p3p4_routing_only_raw.csv');
    end

    data = load(state_file);
    expected_identity = demo_state_cache_identity(data.scene,data.robot_cfg,data.planner_cfg,data.scale,data.seed);
    if ~isfield(data,'validation_identity') || ~isequaln(data.validation_identity,expected_identity)
        error('run_p3p4_validation_route:StaleState', ...
            'Frozen state predates current geometry/config/source validation. Regenerate with prepare_p3p4_validation_state.');
    end
    row = empty_row();
    row.timestamp = datestr(now, 30);
    row.scale = data.scale;
    row.seed = data.seed;
    row.strategy = data.strategy;
    row.mode = mode_label;
    row.routing_mode = routing_mode;
    row.k = k;
    row.cache_on = logical(cache_on);
    row.status = 'ERROR';
    try
        cfg = data.planner_cfg;
        cfg.routing.mode = routing_mode;
        cfg.routing.sparse_knn.k = k;
        cfg.strategy.bidirectional.use_shared_path_cost_cache = logical(cache_on);
        if cache_on
            cfg.runtime.shared_path_cost_context = SharedPathCostContext(data.scene, cfg);
        elseif isfield(cfg, 'runtime') && isfield(cfg.runtime, 'shared_path_cost_context')
            cfg.runtime = rmfield(cfg.runtime, 'shared_path_cost_context');
        end
        cfg.candidate_pair_prefilter.enable = false;

        timing = struct('scene_time_s', 0, ...
            'candidate_generation_time_s',0,'strategy_total_time_s',0, ...
            'timing_scope','routing_only_replay_upstream_excluded');
        t_stage3 = tic;
        [~, ~, ~, visit_order, executed_tasks, total_path, stats] = stage3_global_planning( ...
            data.scene, data.plan.selected_poses, data.plan.task_sequence_summary, ...
            data.task_table_map, data.robot_cfg, cfg, data.plan.pose_info_list, timing, data.plan);
        row.T_stage3 = toc(t_stage3);
        row.T_path_cost = stats.timing.distance_matrix_time_s;
        row.T_total = row.T_stage3;
        row.timing_scope = 'routing_only_replay_upstream_excluded';
        row.evidence_source = 'geometric_simulation';
        row.task_count = data.scene.num_tasks;
        row.table_count = size(data.scene.scene_cfg.table_positions, 1);
        row.region_count = data.scene.region_metadata.region_count;
        row.feasible_tasks = data.scene.num_tasks - numel(data.model_infeasible_tasks);
        row.selected_docks = data.plan.num_docking_poses;
        row.compression_ratio = row.task_count / max(row.selected_docks, 1);
        row.feasible_compression_ratio = row.feasible_tasks / max(row.selected_docks, 1);
        row.dense_pair_count = (data.plan.num_docking_poses + 1) * data.plan.num_docking_poses / 2;
        row.coverage = 100 * numel(data.plan.covered_tasks_predicted) / max(data.scene.num_tasks, 1);
        acc = task_operability_accounting(1:data.scene.num_tasks, ...
            data.plan.covered_tasks_predicted, data.model_infeasible_tasks);
        row.feasible_task_coverage = acc.feasible_task_coverage_percent;
        row.verified_completion = stats.verified_completion_rate;
        row.completed_tasks = numel(executed_tasks);
        row.total_path_length = total_path;
        row.visit_order = mat2str(visit_order);
        row.objective = getfield_default(data.plan, 'objective_value', NaN);
        row.cache_query_count = stats.timing.shared_cache_query_count;
        row.cache_hit_count = stats.timing.shared_cache_hit_count;
        row.cache_miss_count = stats.timing.shared_cache_miss_count;
        row.expensive_planner_calls = stats.timing.expensive_planner_calls;
        if strcmpi(routing_mode, 'sparse_knn')
            s = stats.sparse_graph.stats;
            row.sparse_geometric_edges = s.sparse_geometric_edges;
            row.sparse_edge_count = s.validated_sparse_edges;
            row.actual_astar_calls = s.actual_astar_calls;
            row.failed_edges = s.failed_edges;
            row.adaptive_bridge_edges_added = s.adaptive_bridge_edges_added;
            row.connectivity_ok = stats.sparse_graph.connectivity.ok;
        else
            row.actual_astar_calls = row.dense_pair_count;
            row.connectivity_ok = true;
        end
        row.status = 'SUCCESS';
    catch ME
        row.status = 'ERROR';
        row.failure_reason = sprintf('%s: %s', ME.identifier, ME.message);
    end
    append_row(output_csv, row);
    fprintf('P3P4_ROUTE_RESULT mode=%s status=%s scale=%s seed=%d tasks=%d docks=%d astar=%d T_stage3=%.3f path=%.4f\n', ...
        row.mode, row.status, row.scale, row.seed, row.task_count, row.selected_docks, ...
        row.actual_astar_calls, row.T_stage3, row.total_path_length);
end

function append_row(output_csv, row)
    out_dir = fileparts(output_csv);
    if ~isfolder(out_dir), mkdir(out_dir); end
    t = struct2table(row, 'AsArray', true);
    if isfile(output_csv)
        writetable(t, output_csv, 'WriteMode', 'append', 'WriteVariableNames', false);
    else
        writetable(t, output_csv);
    end
end

function row = empty_row()
    row = struct('timestamp', '', 'scale', '', 'seed', 0, 'strategy', '', ...
        'mode', '', 'routing_mode', '', 'k', 0, 'cache_on', false, ...
        'status', '', 'failure_reason', '', 'task_count', 0, ...
        'table_count', 0, 'region_count', 0, 'feasible_tasks', 0, ...
        'selected_docks', 0, 'compression_ratio', NaN, ...
        'feasible_compression_ratio', NaN, 'dense_pair_count', NaN, ...
        'sparse_geometric_edges', NaN, 'sparse_edge_count', NaN, ...
        'actual_astar_calls', 0, 'failed_edges', 0, ...
        'adaptive_bridge_edges_added', 0, 'connectivity_ok', false, ...
        'coverage', NaN, 'feasible_task_coverage', NaN, ...
        'verified_completion', NaN, 'completed_tasks', 0, ...
        'total_path_length', NaN, 'route_degradation', NaN, ...
        'T_path_cost', NaN, 'T_stage3', NaN, 'T_total', NaN, ...
        'timing_scope','routing_only_replay_upstream_excluded','evidence_source','geometric_simulation', ...
        'cache_query_count', 0, 'cache_hit_count', 0, 'cache_miss_count', 0, ...
        'expensive_planner_calls', 0, 'objective', NaN, 'visit_order', '');
end

function v = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end
