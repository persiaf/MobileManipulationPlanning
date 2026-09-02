function [raw_table, summary_table] = exp_scene_complexity_p1(options)
%EXP_SCENE_COMPLEXITY_P1 Evaluate planning under scene-structure complexity.
%
%   P1 changes scene generation only. Planner, reachability, collision,
%   dock-selection, and candidate-augmentation parameters are not tuned here.

    if nargin < 1 || isempty(options)
        options = struct();
    end
    options = default_options(options);
    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    cases = options.cases;
    seeds = options.seeds;
    if options.quick_mode
        cases = cases(1);
        seeds = seeds(1);
    end

    rows = repmat(empty_row(), 0, 1);
    for ci = 1:numel(cases)
        for seed = seeds
            for ai = 1:numel(options.augmentation_modes)
                base = empty_row();
                base.timestamp = datestr(now, 30);
                base.case_id = cases{ci};
                base.seed = seed;
                base.augmentation_mode = options.augmentation_modes{ai};
                try
                    scene_cfg = scene_complexity_profile_config(cases{ci}, seed);
                    robot_cfg = robot_config();
                    planner_cfg = planner_config();
                    planner_cfg = configure_planner(planner_cfg, options.augmentation_modes{ai});

                    t_scene = tic;
                    scene = build_scene_from_cfg(scene_cfg);
                    base.T_scene = toc(t_scene);
                    task_table_map = assign_tasks_to_tables(scene);
                    regions = build_task_regions(scene, task_table_map);
                    base = fill_scene_fields(base, scene, scene_cfg, regions);

                    t_candidate = tic;
                    candidate_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
                    base.T_candidate = toc(t_candidate);
                    base = fill_candidate_fields(base, candidate_library, scene);

                    for si = 1:numel(options.strategies)
                        strategy_name = options.strategies{si};
                        cfg_i = planner_cfg;
                        cfg_i.strategy.name = strategy_name;
                        fprintf('SCENE_P1_START case=%s seed=%d aug=%s strategy=%s tasks=%d candidates=%d\n', ...
                            base.case_id, base.seed, base.augmentation_mode, strategy_name, ...
                            base.actual_task_count, base.final_library_candidate_count);
                        row = base;
                        row.strategy = strategy_name;
                        try
                            row = profile_strategy(row, scene, regions, task_table_map, robot_cfg, cfg_i, candidate_library, options);
                        catch ME
                            row.success = false;
                            row.failure_reason = sprintf('%s: %s', ME.identifier, ME.message);
                        end
                        rows(end + 1, 1) = row; %#ok<AGROW>
                        write_raw(rows, project_root);
                        fprintf('SCENE_P1_END case=%s seed=%d aug=%s strategy=%s success=%d coverage=%.2f verified=%.2f T_total=%.6f\n', ...
                            row.case_id, row.seed, row.augmentation_mode, row.strategy, row.success, ...
                            row.candidate_library_coverage, row.verified_completion, row.T_total);
                    end
                catch ME
                    for si = 1:numel(options.strategies)
                        row = base;
                        row.strategy = options.strategies{si};
                        row.success = false;
                        row.failure_reason = sprintf('%s: %s', ME.identifier, ME.message);
                        rows(end + 1, 1) = row; %#ok<AGROW>
                    end
                    write_raw(rows, project_root);
                end
            end
        end
    end

    raw_table = struct2table(rows, 'AsArray', true);
    summary_table = summarize_rows(raw_table);
    ensure_result_dir();
    writetable(raw_table, fullfile(project_root, 'results', 'tables', 'scene_complexity_p1_raw.csv'));
    writetable(summary_table, fullfile(project_root, 'results', 'tables', 'scene_complexity_p1_summary.csv'));
end

function planner_cfg = configure_planner(planner_cfg, augmentation_mode)
    planner_cfg.runtime.disable_visualization = true;
    planner_cfg.runtime.disable_video = true;
    planner_cfg.runtime.quiet = true;
    planner_cfg.runtime.quiet_stage2 = true;
    planner_cfg.benchmark.enable = false;
    planner_cfg.visualization.show_scene = false;
    planner_cfg.visualization.show_docking = false;
    planner_cfg.visualization.show_global_path = false;
    planner_cfg.strategy.common.route_shortlist_size = min(12, planner_cfg.strategy.common.route_shortlist_size);
    planner_cfg.candidate_augmentation.enable = false;
    if strcmpi(augmentation_mode, 'position_only')
        planner_cfg.candidate_augmentation.enable = true;
        planner_cfg.candidate_augmentation.variant = 'position_only';
    elseif ~strcmpi(augmentation_mode, 'off')
        error('exp_scene_complexity_p1:UnknownAugmentationMode', ...
              'Unknown augmentation mode "%s".', augmentation_mode);
    end
end

function row = fill_scene_fields(row, scene, scene_cfg, regions)
    row.map_width = scene.map_info.width_m;
    row.map_height = scene.map_info.height_m;
    row.table_count = size(scene.scene_cfg.table_positions, 1);
    row.task_count = scene.num_tasks;
    row.region_count = numel(regions);
    row.requested_task_count = getfield_default(scene_cfg, 'requested_task_count', NaN);
    row.actual_task_count = scene.num_tasks;
    row.layout_pattern = getfield_default(scene_cfg.layout, 'pattern', '');
    row.orientation_mode = getfield_default(scene_cfg.layout, 'orientation_mode', '');
    row.task_position_mode = getfield_default(scene_cfg.task, 'position_mode', 'discrete');
    row.task_difficulty = getfield_default(scene_cfg.task, 'difficulty', 'normal');
    row.layout_valid = scene.scene_cfg.layout_validation.is_valid;
    row.layout_failure_reason = scene.scene_cfg.layout_validation.failure_reason;
end

function row = fill_candidate_fields(row, library, scene)
    diag = [library.regions.generation_diagnostics];
    row.raw_candidate_count = sum([diag.raw_candidate_count]);
    row.collision_valid_candidate_count = sum([diag.collision_valid_candidate_count]);
    row.reachability_valid_candidate_count = sum([diag.reachability_valid_candidate_count]);
    row.final_library_candidate_count = library.total_candidates;
    row.supplementary_candidate_count = getfield_default(library.augmentation_diagnostics, ...
        'selected_supplementary_count', 0);
    covered = library_covered_tasks(library);
    row.candidate_library_coverage = 100 * numel(intersect(1:scene.num_tasks, covered)) / max(scene.num_tasks, 1);
end

function row = profile_strategy(row, scene, regions, task_table_map, robot_cfg, planner_cfg, candidate_library, options)
    t_strategy = tic;
    plan = run_planning_strategy(candidate_library, regions, scene, robot_cfg, planner_cfg);
    row.T_strategy = toc(t_strategy);
    row.selected_docking_pose_count = plan.num_docking_poses;
    row.covered_tasks = numel(plan.covered_tasks_predicted);
    row.coverage_rate = 100 * row.covered_tasks / max(scene.num_tasks, 1);
    row.objective = get_num(plan, 'objective_value', NaN);
    if options.skip_stage3
        row.T_total = row.T_scene + row.T_candidate + row.T_strategy;
        row.success = true;
        return;
    end
    timing = struct('scene_time_s', row.T_scene, ...
        'candidate_generation_time_s', candidate_library.total_generation_time_s, ...
        'strategy_total_time_s', row.T_strategy);
    t_path = tic;
    [~, ~, ~, ~, executed_tasks, total_path, stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, planner_cfg, plan.pose_info_list, timing, plan);
    row.T_path_total = toc(t_path);
    row.T_path_cost = get_num(stats.timing, 'distance_matrix_time_s', NaN);
    row.T_route = get_num(stats.timing, 'route_order_time_s', NaN);
    row.T_execution_sim = get_num(stats.timing, 'execution_sim_time_s', NaN);
    row.T_total = row.T_scene + row.T_candidate + row.T_strategy + row.T_path_total;
    row.task_path_length = get_num(stats, 'task_path_len_m', NaN);
    row.return_path_length = get_num(stats, 'return_path_len_m', NaN);
    row.total_path_length = total_path;
    row.verified_completion = get_num(stats, 'verified_completion_rate', ...
        100 * numel(executed_tasks) / max(scene.num_tasks, 1));
    row.cache_hit = get_num(stats.timing, 'path_cache_hits', 0);
    row.cache_miss = get_num(stats.timing, 'path_cache_misses', 0);
    row.path_cost_query_count = row.cache_hit + row.cache_miss;
    row.expensive_planner_calls = row.cache_miss;
    row.success = true;
end

function summary = summarize_rows(raw)
    groups = unique(raw(:, {'case_id', 'augmentation_mode', 'strategy'}), 'rows');
    summary = groups;
    summary.run_count = zeros(height(groups), 1);
    summary.success_count = zeros(height(groups), 1);
    metrics = {'candidate_library_coverage','verified_completion','final_library_candidate_count', ...
        'supplementary_candidate_count','selected_docking_pose_count','T_candidate','T_strategy', ...
        'T_path_cost','T_total','total_path_length','expensive_planner_calls'};
    for mi = 1:numel(metrics)
        summary.([metrics{mi} '_mean']) = NaN(height(groups), 1);
    end
    for gi = 1:height(groups)
        mask = strcmp(raw.case_id, groups.case_id{gi}) & ...
               strcmp(raw.augmentation_mode, groups.augmentation_mode{gi}) & ...
               strcmp(raw.strategy, groups.strategy{gi});
        ok = mask & raw.success;
        summary.run_count(gi) = sum(mask);
        summary.success_count(gi) = sum(ok);
        for mi = 1:numel(metrics)
            vals = raw.(metrics{mi})(ok);
            vals = vals(isfinite(vals));
            if ~isempty(vals)
                summary.([metrics{mi} '_mean'])(gi) = mean(vals);
            end
        end
    end
end

function row = empty_row()
    row = struct('timestamp', '', 'case_id', '', 'seed', 0, 'augmentation_mode', '', ...
        'strategy', '', 'layout_pattern', '', 'orientation_mode', '', ...
        'task_position_mode', '', 'task_difficulty', '', 'map_width', NaN, ...
        'map_height', NaN, 'table_count', 0, 'task_count', 0, 'region_count', 0, ...
        'requested_task_count', NaN, 'actual_task_count', 0, ...
        'raw_candidate_count', 0, 'collision_valid_candidate_count', 0, ...
        'reachability_valid_candidate_count', 0, 'final_library_candidate_count', 0, ...
        'supplementary_candidate_count', 0, 'selected_docking_pose_count', 0, ...
        'candidate_library_coverage', NaN, 'covered_tasks', 0, 'coverage_rate', NaN, ...
        'verified_completion', NaN, 'task_path_length', NaN, 'return_path_length', NaN, ...
        'total_path_length', NaN, 'T_scene', NaN, 'T_candidate', NaN, ...
        'T_strategy', NaN, 'T_path_cost', NaN, 'T_route', NaN, ...
        'T_execution_sim', NaN, 'T_path_total', NaN, 'T_total', NaN, ...
        'path_cost_query_count', 0, 'expensive_planner_calls', 0, ...
        'cache_hit', 0, 'cache_miss', 0, 'objective', NaN, ...
        'layout_valid', false, 'layout_failure_reason', '', ...
        'success', false, 'failure_reason', '');
end

function options = default_options(options)
    if ~isfield(options, 'quick_mode'), options.quick_mode = false; end
    if ~isfield(options, 'cases'), options.cases = {'C0', 'C1', 'C2', 'C3'}; end
    if ~isfield(options, 'seeds'), options.seeds = 2026; end
    if ~isfield(options, 'augmentation_modes'), options.augmentation_modes = {'off', 'position_only'}; end
    if ~isfield(options, 'strategies'), options.strategies = {'bottom_up', 'top_down', 'bidirectional'}; end
    if ~isfield(options, 'skip_stage3'), options.skip_stage3 = false; end
end

function write_raw(rows, project_root)
    ensure_result_dir();
    writetable(struct2table(rows, 'AsArray', true), ...
        fullfile(project_root, 'results', 'tables', 'scene_complexity_p1_raw.csv'));
end

function ensure_result_dir()
    out_dir = fullfile('results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
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

function v = get_num(s, name, default_value)
    v = default_value;
    if isstruct(s) && isfield(s, name) && isnumeric(s.(name)) && isscalar(s.(name))
        v = s.(name);
    end
end
