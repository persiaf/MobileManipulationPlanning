function [raw_table, summary_table] = exp_large_scale_complex_tasks(options)
%EXP_LARGE_SCALE_COMPLEX_TASKS Profile planning cost over increasing scales.
% The driver reuses the existing scene, candidate-library, strategy, and
% stage-3 implementations. It records timings and diagnostics without
% changing planner behavior. Use options.quick_mode=true for one smoke run.

    if nargin < 1 || isempty(options)
        options = struct();
    end
    options = default_options(options);
    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    if options.quick_mode
        scales = options.scales(1);
        seeds = options.seeds(1);
    else
        scales = options.scales;
        seeds = options.seeds;
    end

    rows = repmat(empty_row(), 0, 1);
    for si = 1:numel(scales)
        for seed = seeds
            row = empty_row();
            row.timestamp = datestr(now, 30);
            row.scale_id = scales{si};
            row.seed = seed;
            try
                scene_cfg = large_scale_profile_config(scales{si}, seed);
                robot_cfg = robot_config();
                planner_cfg = planner_config();
                planner_cfg.runtime.disable_visualization = true;
                planner_cfg.runtime.disable_video = true;
                planner_cfg.runtime.quiet = true;
                planner_cfg.runtime.quiet_stage2 = true;
                planner_cfg.benchmark.enable = false;
                planner_cfg.visualization.show_scene = false;
                planner_cfg.visualization.show_docking = false;
                planner_cfg.visualization.show_global_path = false;
                planner_cfg.strategy.common.route_shortlist_size = min(15, planner_cfg.strategy.common.route_shortlist_size);

                t_scene = tic;
                scene = build_scene_from_cfg(scene_cfg);
                row.T_scene = toc(t_scene);
                task_table_map = assign_tasks_to_tables(scene);
                regions = build_task_regions(scene, task_table_map);
                row.map_width = scene.map_info.width_m;
                row.map_height = scene.map_info.height_m;
                row.region_count = numel(regions);
                row.table_count = size(scene.scene_cfg.table_positions, 1);
                row.requested_task_count = scene_cfg.requested_task_count;
                row.actual_task_count = scene.num_tasks;

                t_candidate = tic;
                candidate_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
                row.T_candidate = toc(t_candidate);
                row.candidate_count = candidate_library.total_candidates;
                counts = arrayfun(@(r) r.num_candidates, candidate_library.regions);
                row.candidate_count_mean_per_region = mean_or_zero(counts);
                row.candidate_count_max_per_region = max_or_zero(counts);

                for strategy = {'bottom_up', 'top_down', 'bidirectional'}
                    strategy_name = strategy{1};
                    cfg_i = planner_cfg;
                    cfg_i.strategy.name = strategy_name;
                    fprintf('LARGE_SCALE_START scale=%s seed=%d strategy=%s tasks=%d candidates=%d\n', ...
                        row.scale_id, row.seed, strategy_name, row.actual_task_count, row.candidate_count);
                    try
                        result = profile_strategy(scene, regions, task_table_map, robot_cfg, cfg_i, candidate_library);
                    catch strategy_error
                        result = empty_row();
                        result.strategy = strategy_name;
                        result.success = false;
                        result.failure_reason = sprintf('%s: %s', strategy_error.identifier, strategy_error.message);
                    end
                    result.timestamp = row.timestamp;
                    result.scale_id = row.scale_id;
                    result.seed = row.seed;
                    result.map_width = row.map_width;
                    result.map_height = row.map_height;
                    result.region_count = row.region_count;
                    result.table_count = row.table_count;
                    result.requested_task_count = row.requested_task_count;
                    result.actual_task_count = row.actual_task_count;
                    result.candidate_count = row.candidate_count;
                    result.candidate_count_mean_per_region = row.candidate_count_mean_per_region;
                    result.candidate_count_max_per_region = row.candidate_count_max_per_region;
                    result.T_scene = row.T_scene;
                    result.T_candidate = row.T_candidate;
                    result.T_total = result.T_total + row.T_scene + row.T_candidate;
                    rows(end + 1, 1) = result; %#ok<AGROW>
                    write_raw_checkpoint(rows, project_root);
                    fprintf('LARGE_SCALE_END scale=%s seed=%d strategy=%s success=%d tasks=%d candidates=%d T_total=%.6f\n', ...
                        result.scale_id, result.seed, result.strategy, result.success, ...
                        result.actual_task_count, result.candidate_count, result.T_total);
                end
            catch ME
                for strategy = {'bottom_up', 'top_down', 'bidirectional'}
                    failed = row;
                    failed.strategy = strategy{1};
                    failed.success = false;
                    failed.failure_reason = sprintf('%s: %s', ME.identifier, ME.message);
                    rows(end + 1, 1) = failed; %#ok<AGROW>
                end
                write_raw_checkpoint(rows, project_root);
            end
        end
    end

    ensure_result_dirs();
    raw_table = struct2table(rows, 'AsArray', true);
    raw_file = fullfile(project_root, 'results', 'tables', 'large_scale_complex_tasks_raw.csv');
    writetable(raw_table, raw_file);
    summary_table = summarize_rows(raw_table);
    writetable(summary_table, fullfile(project_root, 'results', 'tables', 'large_scale_complex_tasks_summary.csv'));
    make_diagnostic_figures(raw_table, project_root);
end

function write_raw_checkpoint(rows, project_root)
    ensure_result_dirs();
    writetable(struct2table(rows, 'AsArray', true), fullfile(project_root, 'results', 'tables', ...
        'large_scale_complex_tasks_raw.csv'));
end

function result = profile_strategy(scene, regions, task_table_map, robot_cfg, planner_cfg, candidate_library)
    result = empty_row();
    result.strategy = planner_cfg.strategy.name;
    result.success = false;
    t_strategy = tic;
    plan = run_planning_strategy(candidate_library, regions, scene, robot_cfg, planner_cfg);
    result.T_strategy = toc(t_strategy);
    result.selected_dock_count = plan.num_docking_poses;
    result.covered_tasks = numel(plan.covered_tasks_predicted);
    result.coverage_rate = 100 * result.covered_tasks / max(scene.num_tasks, 1);
    result.objective = get_num(plan, 'objective_value', NaN);

    timing = struct('scene_time_s', 0, 'candidate_generation_time_s', candidate_library.total_generation_time_s, ...
        'strategy_total_time_s', result.T_strategy);
    t_path = tic;
    [~, ~, ~, ~, executed_tasks, total_path, stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, planner_cfg, plan.pose_info_list, timing, plan);
    result.T_path_total = toc(t_path);
    result.T_path_cost = get_num(stats.timing, 'distance_matrix_time_s', NaN);
    result.T_route = get_num(stats.timing, 'route_order_time_s', NaN);
    result.T_execution_sim = get_num(stats.timing, 'execution_sim_time_s', NaN);
    result.T_total = result.T_strategy + result.T_path_total;
    result.task_path_length = get_num(stats, 'task_path_len_m', NaN);
    result.return_path_length = get_num(stats, 'return_path_len_m', NaN);
    result.total_path_length = total_path;
    result.verified_completion = get_num(stats, 'verified_completion_rate', ...
        100 * numel(executed_tasks) / max(scene.num_tasks, 1));
    result.cache_hit = get_num(stats.timing, 'path_cache_hits', 0);
    result.cache_miss = get_num(stats.timing, 'path_cache_misses', 0);
    result.path_cost_cache_hit = result.cache_hit;
    result.path_cost_cache_miss = result.cache_miss;
    result.path_cost_query_count = result.cache_hit + result.cache_miss;
    if result.path_cost_query_count == 0
        n = plan.num_docking_poses + 1;
        result.path_cost_query_count = n * max(n - 1, 0);
    end
    result.unique_path_cost_computations = result.cache_miss;
    result.cache_hit_ratio = result.cache_hit / max(result.cache_hit + result.cache_miss, 1);
    result.coverage_repair_count = get_num(stats, 'coverage_repair_count', 0);
    result.success = true;
end

function options = default_options(options)
    if ~isfield(options, 'quick_mode'), options.quick_mode = false; end
    if ~isfield(options, 'scales'), options.scales = {'s1', 's2', 's3', 's4'}; end
    if ~isfield(options, 'seeds'), options.seeds = 1:5; end
end

function row = empty_row()
    row = struct('timestamp', '', 'scale_id', '', 'seed', 0, 'strategy', '', ...
        'map_width', NaN, 'map_height', NaN, 'region_count', 0, 'table_count', 0, ...
        'requested_task_count', 0, 'actual_task_count', 0, 'candidate_count', 0, ...
        'candidate_count_mean_per_region', 0, 'candidate_count_max_per_region', 0, ...
        'selected_dock_count', 0, 'covered_tasks', 0, 'coverage_rate', 0, ...
        'verified_completion', 0, 'task_path_length', NaN, 'return_path_length', NaN, ...
        'total_path_length', NaN, 'T_scene', 0, 'T_candidate', 0, 'T_strategy', NaN, ...
        'T_path_cost', NaN, 'T_route', NaN, 'T_execution_sim', NaN, 'T_path_total', NaN, ...
        'T_total', NaN, 'path_cost_query_count', 0, 'cache_hit', 0, 'cache_miss', 0, ...
        'path_cost_cache_hit', 0, 'path_cost_cache_miss', 0, ...
        'unique_path_cost_computations', 0, 'cache_hit_ratio', 0, 'coverage_repair_count', 0, ...
        'objective', NaN, 'success', false, 'failure_reason', '');
end

function table_out = summarize_rows(raw)
    valid = ~strcmp(raw.strategy, 'all');
    groups = unique(raw(valid, {'scale_id', 'strategy'}), 'rows');
    if isempty(groups), table_out = groups; return; end
    table_out = groups;
    table_out.run_count = zeros(height(groups), 1);
    table_out.success_count = zeros(height(groups), 1);
    table_out.failure_count = zeros(height(groups), 1);
    table_out.success_rate = zeros(height(groups), 1);
    metrics = {'T_total', 'T_candidate', 'T_path_cost', 'T_strategy', 'total_path_length', ...
        'selected_dock_count', 'coverage_rate', 'candidate_count', 'cache_hit_ratio'};
    for mi = 1:numel(metrics)
        name = metrics{mi};
        mean_values = zeros(height(groups), 1); std_values = zeros(height(groups), 1); median_values = zeros(height(groups), 1);
        for gi = 1:height(groups)
            mask = valid & strcmp(raw.scale_id, groups.scale_id{gi}) & strcmp(raw.strategy, groups.strategy{gi}) & raw.success;
            values = raw.(name)(mask);
            values = values(isfinite(values));
            if ~isempty(values)
                mean_values(gi) = mean(values); std_values(gi) = std(values); median_values(gi) = median(values);
            end
            all_mask = valid & strcmp(raw.scale_id, groups.scale_id{gi}) & strcmp(raw.strategy, groups.strategy{gi});
            table_out.run_count(gi) = sum(all_mask);
            table_out.success_count(gi) = sum(raw.success(all_mask));
            table_out.failure_count(gi) = table_out.run_count(gi) - table_out.success_count(gi);
            table_out.success_rate(gi) = 100 * table_out.success_count(gi) / max(table_out.run_count(gi), 1);
        end
        table_out.([name '_mean']) = mean_values;
        table_out.([name '_std']) = std_values;
        table_out.([name '_median']) = median_values;
    end
end

function make_diagnostic_figures(raw, root)
    ok = raw.success & ~strcmp(raw.strategy, 'all');
    raw = raw(ok, :); if isempty(raw), return; end
    scales = unique(raw.scale_id, 'stable'); x = zeros(size(scales));
    for i = 1:numel(scales), x(i) = mean(raw.actual_task_count(strcmp(raw.scale_id, scales{i}))); end
    names = {'bottom_up', 'top_down', 'bidirectional'};
    h = figure('Visible', 'off'); hold on;
    for name = names, y = arrayfun(@(i) mean(raw.T_total(strcmp(raw.scale_id, scales{i}) & strcmp(raw.strategy, name{1}))), 1:numel(scales)); plot(x, y, '-o', 'DisplayName', name{1}); end
    xlabel('Task count'); ylabel('Total planning time (s)'); legend('Location', 'best'); grid on; saveas(h, fullfile(root, 'results', 'figures', 'fig_large_scale_total_time.png')); close(h);
    h = figure('Visible', 'off'); data = zeros(numel(scales), 4); for i = 1:numel(scales), m = strcmp(raw.scale_id, scales{i}) & strcmp(raw.strategy, 'bottom_up'); data(i,:) = [mean(raw.T_candidate(m)), mean(raw.T_path_cost(m)), mean(raw.T_strategy(m)), mean(raw.T_route(m))]; end; bar(x, data, 'stacked'); xlabel('Task count'); ylabel('Stage time (s)'); legend({'Candidate','Path cost','Strategy','Route'}, 'Location', 'best'); saveas(h, fullfile(root, 'results', 'figures', 'fig_large_scale_stage_time.png')); close(h);
    h = figure('Visible', 'off'); q = arrayfun(@(i) mean(raw.path_cost_query_count(strcmp(raw.scale_id, scales{i}))), 1:numel(scales)); hit = arrayfun(@(i) mean(raw.cache_hit_ratio(strcmp(raw.scale_id, scales{i}))), 1:numel(scales)); yyaxis left; plot(x, q, '-o'); ylabel('Path-cost queries'); yyaxis right; plot(x, hit, '-s'); ylabel('Cache hit ratio'); xlabel('Task count'); grid on; saveas(h, fullfile(root, 'results', 'figures', 'fig_large_scale_path_cost_diagnostics.png')); close(h);
end

function ensure_result_dirs()
    dirs = {fullfile('results', 'tables'), fullfile('results', 'figures')};
    for i = 1:numel(dirs), if ~isfolder(dirs{i}), mkdir(dirs{i}); end, end
end
function v = get_num(s, name, default_value)
    v = default_value; if isstruct(s) && isfield(s, name) && isnumeric(s.(name)) && isscalar(s.(name)), v = s.(name); end
end
function v = mean_or_zero(x), if isempty(x), v = 0; else, v = mean(x); end, end
function v = max_or_zero(x), if isempty(x), v = 0; else, v = max(x); end, end
