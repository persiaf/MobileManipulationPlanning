function [raw, summary] = exp_sparse_cache_ablation(options)
%EXP_SPARSE_CACHE_ABLATION Bidirectional B0/B1/B2/B3 sparse-cache comparison.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    modes = {
        'B0_dense_cache_off', 'dense', false
        'B1_dense_cache_on', 'dense', true
        'B2_sparse_cache_off', 'sparse_knn', false
        'B3_sparse_cache_on', 'sparse_knn', true
    };

    rows = repmat(empty_row(), 0, 1);
    t_all = tic;
    for seed = options.seeds
        for mi = 1:size(modes, 1)
            if toc(t_all) > options.max_runtime_s
                row = empty_row(); row.seed = seed; row.mode = modes{mi, 1};
                row.status = 'TIMEOUT_BEFORE_RUN';
            else
                row = run_case(options.scale, seed, modes{mi, 1}, modes{mi, 2}, ...
                    modes{mi, 3}, options.k_selected);
            end
            rows(end + 1, 1) = row; %#ok<AGROW>
            fprintf('SPARSE_CACHE seed=%d mode=%s status=%s total=%.3f calls=%d\n', ...
                row.seed, row.mode, row.status, row.T_total, row.actual_astar_calls);
            if options.write_files
                write_outputs(struct2table(rows, 'AsArray', true), summarize(struct2table(rows, 'AsArray', true)), root);
            end
        end
    end
    raw = struct2table(rows, 'AsArray', true);
    summary = summarize(raw);
    if options.write_files
        write_outputs(raw, summary, root);
    end
end

function row = run_case(scale, seed, mode, routing_mode, cache_on, k)
    row = empty_row();
    row.scale = scale; row.seed = seed; row.mode = mode;
    row.routing_mode = routing_mode; row.cache_on = cache_on; row.k = k;
    try
        scene = build_scene_from_cfg(large_scale_profile_config(scale, seed));
        robot_cfg = robot_config();
        cfg = planner_config();
        cfg.strategy.name = 'bidirectional';
        cfg.strategy.bidirectional.use_shared_path_cost_cache = cache_on;
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

        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        library = build_candidate_library(regions, scene, robot_cfg, cfg);
        t_strategy = tic;
        plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg);
        row.T_strategy = toc(t_strategy);
        t_stage3 = tic;
        [~, ~, ~, visit_order, ~, total_path, stats] = stage3_global_planning( ...
            scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
            robot_cfg, cfg, plan.pose_info_list, struct(), plan);
        row.T_stage3 = toc(t_stage3);
        row.T_total = row.T_strategy + row.T_stage3;
        row.task_count = scene.num_tasks;
        row.selected_docks = plan.num_docking_poses;
        row.coverage = 100 * numel(plan.covered_tasks_predicted) / max(scene.num_tasks, 1);
        row.verified_completion = stats.verified_completion_rate;
        row.total_path_length = total_path;
        row.visit_order = mat2str(visit_order);
        row.dense_pair_count = (plan.num_docking_poses + 1) * plan.num_docking_poses / 2;
        if strcmpi(routing_mode, 'sparse_knn')
            row.sparse_edge_count = stats.sparse_graph.stats.validated_sparse_edges;
            row.actual_astar_calls = stats.sparse_graph.stats.actual_astar_calls;
            row.adaptive_bridge_edges_added = stats.sparse_graph.stats.adaptive_bridge_edges_added;
        else
            row.actual_astar_calls = row.dense_pair_count;
        end
        cache = stats.timing.shared_path_cache;
        row.cache_query_count = cache.cache_query_count;
        row.cache_hit_count = cache.cache_hit_count;
        row.cache_miss_count = cache.cache_miss_count;
        row.status = 'PASS';
    catch ME
        row.status = 'FAIL';
        row.failure_reason = sprintf('%s: %s', ME.identifier, ME.message);
    end
end

function summary = summarize(raw)
    modes = unique(raw.mode, 'stable');
    summary = table(modes, 'VariableNames', {'mode'});
    metrics = {'T_total','T_strategy','T_stage3','actual_astar_calls','cache_hit_count', ...
        'coverage','verified_completion','selected_docks','total_path_length'};
    for mi = 1:numel(metrics)
        summary.([metrics{mi} '_mean']) = NaN(height(summary), 1);
    end
    for i = 1:height(summary)
        ok = strcmp(raw.mode, summary.mode{i}) & strcmp(raw.status, 'PASS');
        for mi = 1:numel(metrics)
            vals = raw.(metrics{mi})(ok);
            vals = vals(isfinite(vals));
            if ~isempty(vals), summary.([metrics{mi} '_mean'])(i) = mean(vals); end
        end
    end
    b0 = summary.T_total_mean(strcmp(summary.mode, 'B0_dense_cache_off'));
    b1 = summary.T_total_mean(strcmp(summary.mode, 'B1_dense_cache_on'));
    b2 = summary.T_total_mean(strcmp(summary.mode, 'B2_sparse_cache_off'));
    b3 = summary.T_total_mean(strcmp(summary.mode, 'B3_sparse_cache_on'));
    summary.speedup_vs_B0 = summary.T_total_mean;
    if ~isempty(b0)
        summary.speedup_vs_B0 = b0 ./ summary.T_total_mean;
    end
    if ~isempty(b0) && ~isempty(b1), summary.cache_only_speedup = repmat(b0 / b1, height(summary), 1); end
    if ~isempty(b0) && ~isempty(b2), summary.sparse_only_speedup = repmat(b0 / b2, height(summary), 1); end
    if ~isempty(b0) && ~isempty(b3), summary.combined_speedup = repmat(b0 / b3, height(summary), 1); end
end

function write_outputs(raw, summary, root)
    out_dir = fullfile(root, 'results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
    writetable(raw, fullfile(out_dir, 'sparse_cache_ablation_raw.csv'));
    writetable(summary, fullfile(out_dir, 'sparse_cache_ablation_summary.csv'));
end

function options = defaults(options)
    if ~isfield(options, 'scale'), options.scale = 'L2'; end
    if ~isfield(options, 'seeds'), options.seeds = [2026, 2027, 2028]; end
    if ~isfield(options, 'k_selected'), options.k_selected = 6; end
    if ~isfield(options, 'max_runtime_s'), options.max_runtime_s = 1800; end
    if ~isfield(options, 'write_files'), options.write_files = true; end
end

function row = empty_row()
    row = struct('scale', '', 'seed', 0, 'mode', '', 'routing_mode', '', ...
        'cache_on', false, 'k', 0, 'status', '', 'failure_reason', '', ...
        'task_count', 0, 'selected_docks', 0, 'coverage', NaN, ...
        'verified_completion', NaN, 'dense_pair_count', NaN, ...
        'sparse_edge_count', NaN, 'actual_astar_calls', 0, ...
        'adaptive_bridge_edges_added', 0, 'cache_query_count', 0, ...
        'cache_hit_count', 0, 'cache_miss_count', 0, 'T_strategy', NaN, ...
        'T_stage3', NaN, 'T_total', NaN, 'total_path_length', NaN, ...
        'visit_order', '');
end
