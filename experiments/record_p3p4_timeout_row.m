function row = record_p3p4_timeout_row(state_file, routing_mode, k, cache_on, mode_label, output_csv, timeout_s)
%RECORD_P3P4_TIMEOUT_ROW Append a resource-timeout row for P3/P4 validation.
%
% This helper is intentionally narrow: it records metadata from a prepared
% scene/docking state after an external process-level timeout. It does not run
% or alter the planner.

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    if nargin < 5 || isempty(mode_label)
        mode_label = sprintf('%s_cache_%d', routing_mode, cache_on);
    end
    if nargin < 6 || isempty(output_csv)
        out_dir = fullfile(root, 'results', 'tables');
        if ~isfolder(out_dir), mkdir(out_dir); end
        output_csv = fullfile(out_dir, 'p3p4_final_validation_raw.csv');
    end
    if nargin < 7 || isempty(timeout_s)
        timeout_s = NaN;
    end

    data = load(state_file);
    row = empty_timeout_row();
    row.timestamp = datestr(now, 30);
    row.scale = data.scale;
    row.seed = data.seed;
    row.strategy = data.strategy;
    row.mode = mode_label;
    row.routing_mode = routing_mode;
    row.k = k;
    row.cache_on = logical(cache_on);
    row.status = 'TIMEOUT';
    if isnan(timeout_s)
        row.failure_reason = 'external_timeout';
    else
        row.failure_reason = sprintf('external_timeout_%gs', timeout_s);
    end
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
    if strcmpi(routing_mode, 'dense')
        row.actual_astar_calls = row.dense_pair_count;
    end

    append_timeout_row(output_csv, row);
    fprintf('P3P4_TIMEOUT_ROW file=%s mode=%s scale=%s seed=%d timeout_s=%g\n', ...
        output_csv, row.mode, row.scale, row.seed, timeout_s);
end

function append_timeout_row(output_csv, row)
    out_dir = fileparts(output_csv);
    if ~isfolder(out_dir), mkdir(out_dir); end
    t = struct2table(row, 'AsArray', true);
    if isfile(output_csv)
        writetable(t, output_csv, 'WriteMode', 'append', 'WriteVariableNames', false);
    else
        writetable(t, output_csv);
    end
end

function row = empty_timeout_row()
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
        'cache_query_count', 0, 'cache_hit_count', 0, 'cache_miss_count', 0, ...
        'expensive_planner_calls', 0, 'objective', NaN, 'visit_order', '');
end
