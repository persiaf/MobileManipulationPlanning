function [raw, summary] = exp_large_scale_sparse_routing(options)
%EXP_LARGE_SCALE_SPARSE_ROUTING P3/P4 dense-vs-sparse large-scene benchmark.
%
%   Aggregate-only output; no per-query traces are saved.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    rows = repmat(empty_row(), 0, 1);
    t_all = tic;
    for si = 1:numel(options.scales)
        scale = options.scales{si};
        for seed = options.seeds
            modes = mode_matrix(scale, seed, options);
            for mi = 1:size(modes, 1)
                if toc(t_all) > options.max_runtime_s
                    row = empty_row();
                    row.scale = scale; row.seed = seed; row.routing_mode = modes{mi, 1};
                    row.status = 'TIMEOUT_BEFORE_RUN';
                    rows(end + 1, 1) = row; %#ok<AGROW>
                    continue;
                end
                row = run_case(scale, seed, modes{mi, 1}, modes{mi, 2}, options);
                rows(end + 1, 1) = row; %#ok<AGROW>
                fprintf('P3P4 scale=%s seed=%d mode=%s k=%d status=%s astar=%d docks=%d T=%.3f\n', ...
                    row.scale, row.seed, row.routing_mode, row.k, row.status, ...
                    row.actual_astar_calls, row.selected_docks, row.T_total);
                if options.write_files
                    write_outputs(struct2table(rows, 'AsArray', true), summarize(struct2table(rows, 'AsArray', true)), root);
                end
            end
        end
    end
    raw = struct2table(rows, 'AsArray', true);
    summary = summarize(raw);
    if options.write_files
        write_outputs(raw, summary, root);
    end
end

function row = run_case(scale, seed, routing_mode, k, options)
    row = empty_row();
    row.scale = scale; row.seed = seed; row.routing_mode = routing_mode; row.k = k;
    try
        t_scene = tic;
        scene = build_scene_from_cfg(large_scale_profile_config(scale, seed));
        row.T_scene = toc(t_scene);
        robot_cfg = robot_config();
        cfg = experiment_config(routing_mode, k, 'bottom_up', false);
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        row = fill_scene(row, scene);

        t_candidate = tic;
        library = build_candidate_library(regions, scene, robot_cfg, cfg);
        row.T_candidate = toc(t_candidate);
        row.final_candidate_count = library.total_candidates;
        covered_by_library = library_covered_tasks(library);
        model_infeasible = setdiff(1:scene.num_tasks, covered_by_library, 'stable');

        t_strategy = tic;
        plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg);
        row.T_strategy = toc(t_strategy);
        row.selected_docks = plan.num_docking_poses;
        row.task_to_dock_compression = scene.num_tasks / max(plan.num_docking_poses, 1);
        row.coverage = 100 * numel(plan.covered_tasks_predicted) / max(scene.num_tasks, 1);
        acc = task_operability_accounting(1:scene.num_tasks, plan.covered_tasks_predicted, model_infeasible);
        row.feasible_task_coverage = acc.feasible_task_coverage_percent;

        timing = struct('scene_time_s', row.T_scene, ...
            'candidate_generation_time_s', library.total_generation_time_s, ...
            'strategy_total_time_s', row.T_strategy);
        path_cost_audit('reset_count_only', sprintf('p3p4_%s_%d_%s_k%d', scale, seed, routing_mode, k));
        t_stage3 = tic;
        [~, ~, ~, ~, executed_tasks, total_path, stats] = stage3_global_planning( ...
            scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
            robot_cfg, cfg, plan.pose_info_list, timing, plan);
        row.T_stage3 = toc(t_stage3);
        row.T_path_cost = stats.timing.distance_matrix_time_s;
        row.T_total = row.T_scene + row.T_candidate + row.T_strategy + row.T_stage3;
        row.total_path_length = total_path;
        row.verified_completion = stats.verified_completion_rate;
        row.completed_tasks = numel(executed_tasks);
        row.dense_pair_count = (size(plan.selected_poses, 1) + 1) * size(plan.selected_poses, 1) / 2;
        if strcmpi(routing_mode, 'sparse_knn')
            s = stats.sparse_graph.stats;
            row.sparse_edge_count = s.validated_sparse_edges;
            row.sparse_geometric_edges = s.sparse_geometric_edges;
            row.actual_astar_calls = s.actual_astar_calls;
            row.failed_edges = s.failed_edges;
            row.adaptive_bridge_edges_added = s.adaptive_bridge_edges_added;
            row.connectivity_ok = stats.sparse_graph.connectivity.ok;
        else
            row.actual_astar_calls = row.dense_pair_count;
            row.sparse_edge_count = NaN;
            row.sparse_geometric_edges = NaN;
            row.connectivity_ok = true;
        end
        row.status = 'PASS';
        path_cost_audit('disable');
    catch ME
        path_cost_audit('disable');
        row.status = 'FAIL';
        row.failure_reason = sprintf('%s: %s', ME.identifier, ME.message);
    end
end

function cfg = experiment_config(routing_mode, k, strategy, cache_on)
    cfg = planner_config();
    cfg.strategy.name = strategy;
    cfg.routing.mode = routing_mode;
    cfg.routing.sparse_knn.k = k;
    cfg.candidate_pair_prefilter.enable = false;
    cfg.runtime.disable_visualization = true;
    cfg.runtime.disable_video = true;
    cfg.runtime.quiet = true;
    cfg.runtime.quiet_stage2 = true;
    cfg.benchmark.enable = false;
    cfg.visualization.show_scene = false;
    cfg.visualization.show_docking = false;
    cfg.visualization.show_global_path = false;
    cfg.strategy.bidirectional.use_shared_path_cost_cache = cache_on;
end

function row = fill_scene(row, scene)
    row.map_width = scene.map_info.width_m;
    row.map_height = scene.map_info.height_m;
    row.table_count = size(scene.scene_cfg.table_positions, 1);
    row.task_count = scene.num_tasks;
    row.requested_task_count = getfield_default(scene.scene_cfg, 'requested_task_count', NaN);
    row.region_count = scene.region_metadata.region_count;
end

function modes = mode_matrix(scale, seed, options) %#ok<INUSD>
    if options.quick_mode
        modes = {'dense', options.k_selected; 'sparse_knn', options.k_selected};
        return;
    end
    switch lower(scale)
        case 'l1'
            modes = {'dense', 0; 'sparse_knn', 4; 'sparse_knn', 6; 'sparse_knn', 8};
        case 'l2'
            modes = {'dense', 0; 'sparse_knn', options.k_selected};
        case 'l3'
            modes = {'sparse_knn', options.k_selected};
            if seed == options.seeds(1)
                modes = [{'dense', 0}; modes];
            end
        otherwise
            modes = {'dense', 0; 'sparse_knn', options.k_selected};
    end
end

function summary = summarize(raw)
    if isempty(raw)
        summary = table();
        return;
    end
    groups = unique(raw(:, {'scale','routing_mode','k'}), 'rows');
    summary = groups;
    metrics = {'task_count','table_count','region_count','selected_docks','task_to_dock_compression', ...
        'dense_pair_count','sparse_edge_count','actual_astar_calls','T_path_cost','T_stage3', ...
        'T_total','coverage','feasible_task_coverage','verified_completion','total_path_length'};
    for mi = 1:numel(metrics)
        summary.([metrics{mi} '_mean']) = NaN(height(groups), 1);
    end
    summary.run_count = zeros(height(groups), 1);
    summary.pass_count = zeros(height(groups), 1);
    for gi = 1:height(groups)
        mask = strcmp(raw.scale, groups.scale{gi}) & ...
            strcmp(raw.routing_mode, groups.routing_mode{gi}) & raw.k == groups.k(gi);
        ok = mask & strcmp(raw.status, 'PASS');
        summary.run_count(gi) = sum(mask);
        summary.pass_count(gi) = sum(ok);
        for mi = 1:numel(metrics)
            vals = raw.(metrics{mi})(ok);
            vals = vals(isfinite(vals));
            if ~isempty(vals), summary.([metrics{mi} '_mean'])(gi) = mean(vals); end
        end
    end
end

function write_outputs(raw, summary, root)
    out_dir = fullfile(root, 'results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
    writetable(raw, fullfile(out_dir, 'large_scale_sparse_routing_raw.csv'));
    writetable(summary, fullfile(out_dir, 'large_scale_sparse_routing_summary.csv'));
end

function options = defaults(options)
    if ~isfield(options, 'scales'), options.scales = {'L1','L2','L3'}; end
    if ~isfield(options, 'seeds'), options.seeds = [2026, 2027, 2028]; end
    if ~isfield(options, 'k_selected'), options.k_selected = 6; end
    if ~isfield(options, 'quick_mode'), options.quick_mode = false; end
    if ~isfield(options, 'max_runtime_s'), options.max_runtime_s = 1800; end
    if ~isfield(options, 'write_files'), options.write_files = true; end
end

function row = empty_row()
    row = struct('scale', '', 'seed', 0, 'routing_mode', '', 'k', 0, ...
        'status', '', 'failure_reason', '', 'map_width', NaN, 'map_height', NaN, ...
        'table_count', 0, 'task_count', 0, 'requested_task_count', NaN, ...
        'region_count', 0, 'final_candidate_count', 0, 'selected_docks', 0, ...
        'task_to_dock_compression', NaN, 'coverage', NaN, ...
        'feasible_task_coverage', NaN, 'verified_completion', NaN, ...
        'completed_tasks', 0, 'dense_pair_count', NaN, 'sparse_geometric_edges', NaN, ...
        'sparse_edge_count', NaN, 'actual_astar_calls', 0, 'failed_edges', 0, ...
        'adaptive_bridge_edges_added', 0, 'connectivity_ok', false, ...
        'T_scene', NaN, 'T_candidate', NaN, 'T_strategy', NaN, ...
        'T_path_cost', NaN, 'T_stage3', NaN, 'T_total', NaN, ...
        'total_path_length', NaN);
end

function tasks = library_covered_tasks(library)
    tasks = [];
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            tasks = union(tasks, library.regions(ri).candidates(ci).covered_tasks, 'stable');
        end
    end
end

function v = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end
