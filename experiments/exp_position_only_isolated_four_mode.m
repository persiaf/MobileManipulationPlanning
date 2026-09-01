function outputs = exp_position_only_isolated_four_mode(options)
%EXP_POSITION_ONLY_ISOLATED_FOUR_MODE Single-process paired cache benchmark.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'pipeline')); init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    if options.run_warmup, run_warmup(); end
    rows = repmat(empty_row(), 0, 1);

    for seed = options.seeds
        scene = build_scene_from_cfg(large_scale_profile_config('s4', seed));
        robot_cfg = robot_config(); cfg = experiment_config();
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        for repeat = 1:options.repeats
            t_baseline = tic;
            baseline = build_candidate_library(regions, scene, robot_cfg, cfg);
            baseline_time = toc(t_baseline);
            cfg_position = cfg; cfg_position.candidate_augmentation.enable = true;
            cfg_position.candidate_augmentation.variant = 'position_only';
            [position, diagnostics] = augment_candidate_library( ...
                baseline, regions, scene, robot_cfg, cfg_position);
            position_time = baseline_time + diagnostics.supplementation_time_s;
            order_seed = options.timing_order_seed + seed * 100 + repeat;
            stream = RandStream('mt19937ar', 'Seed', order_seed);
            order = randperm(stream, 4);

            repeat_rows = repmat(empty_row(), 4, 1);
            for execution_index = 1:4
                mode_id = order(execution_index);
                [mode, augmentation, cache_enabled] = mode_config(mode_id);
                if augmentation
                    library = position; candidate_time = position_time; cfg_run = cfg_position;
                else
                    library = baseline; candidate_time = baseline_time; cfg_run = cfg;
                end
                cfg_run.strategy.bidirectional.use_shared_path_cost_cache = cache_enabled;
                scene_id = sprintf('isolated_s4_seed%d_repeat%d_%s', seed, repeat, mode);
                path_cost_audit('reset', scene_id);
                path_cost_audit('set_context', 'bidirectional', 'OTHER', scene_id);
                row = run_mode(scene, regions, task_table_map, robot_cfg, cfg_run, ...
                    library, candidate_time, mode, seed, repeat, execution_index, ...
                    order_seed, mat2str(order));
                entries = path_cost_audit('snapshot'); path_cost_audit('disable');
                row.expensive_planner_calls = sum([entries.expensive_computation]);
                repeat_rows(mode_id) = row; rows(end + 1, 1) = row; %#ok<AGROW>
                fprintf(['ISOLATED seed=%d repeat=%d order=%d mode=%s coverage=%.3f ' ...
                    'calls=%d total=%.3f\n'], seed, repeat, execution_index, mode, ...
                    row.coverage_percent, row.expensive_planner_calls, row.T_total_s);
                if options.write_files, write_checkpoint(rows, root); end
            end
            assert_cache_semantics(repeat_rows(1), repeat_rows(2));
            assert_cache_semantics(repeat_rows(3), repeat_rows(4));
        end
    end

    outputs.raw = struct2table(rows, 'AsArray', true);
    outputs.summary = summarize(outputs.raw);
    outputs.paired_speedup = paired_speedups(outputs.raw);
    if options.write_files, write_outputs(outputs, root); end
end

function row = run_mode(scene, regions, task_table_map, robot_cfg, cfg, ...
        library, candidate_time, mode, seed, repeat, execution_index, order_seed, order)
    t_strategy = tic;
    plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg);
    strategy_time = toc(t_strategy);
    t_stage3 = tic;
    [~,~,~,visit_order,~,total_path,stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, cfg, plan.pose_info_list, struct(), plan);
    stage3_time = toc(t_stage3); cache = stats.timing.shared_path_cache;
    row = empty_row(); row.seed = seed; row.repeat = repeat; row.mode = mode;
    row.execution_index = execution_index; row.timing_order_seed = order_seed;
    row.execution_order = order; row.tasks = scene.num_tasks;
    row.coverage_percent = 100 * numel(plan.covered_tasks_predicted) / scene.num_tasks;
    row.verified_completion_percent = stats.verified_completion_rate;
    row.added_candidates = library.augmentation_diagnostics.selected_supplementary_count;
    row.selected_docks = plan.num_docking_poses;
    row.task_path_m = stats.task_path_len_m; row.return_path_m = stats.return_path_len_m;
    row.total_path_m = total_path; row.objective = plan.objective_value;
    row.T_candidate_s = candidate_time; row.T_strategy_s = strategy_time;
    row.T_stage3_s = stage3_time; row.T_path_cost_s = stats.timing.distance_matrix_time_s;
    row.T_total_s = candidate_time + strategy_time + stage3_time;
    row.cache_queries = cache.cache_query_count; row.cache_hits = cache.cache_hit_count;
    row.cache_misses = cache.cache_miss_count; row.cache_memory_bytes = cache.memory_estimate_bytes;
    row.visit_order = mat2str(visit_order);
end

function run_warmup()
    fprintf('ISOLATED warm-up start (excluded from results)\n');
    scene = build_scene_from_cfg(large_scale_profile_config('smoke', 1));
    robot_cfg = robot_config(); cfg = experiment_config();
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    library = build_candidate_library(regions, scene, robot_cfg, cfg);
    cfg.strategy.bidirectional.use_shared_path_cost_cache = true;
    plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg);
    stage3_global_planning(scene, plan.selected_poses, plan.task_sequence_summary, ...
        task_table_map, robot_cfg, cfg, plan.pose_info_list, struct(), plan);
    fprintf('ISOLATED warm-up complete\n');
end

function assert_cache_semantics(off, on)
    assert(abs(off.coverage_percent - on.coverage_percent) < 1e-9, 'Cache changed coverage.');
    assert(abs(off.verified_completion_percent - on.verified_completion_percent) < 1e-9, ...
        'Cache changed verified completion.');
    assert(off.selected_docks == on.selected_docks, 'Cache changed selected docks.');
    assert(strcmp(off.visit_order, on.visit_order), 'Cache changed visit order.');
    assert(abs(off.total_path_m - on.total_path_m) < 1e-9, 'Cache changed path length.');
    assert(abs(off.objective - on.objective) < 1e-9, 'Cache changed objective.');
end

function [name, augmentation, cache_enabled] = mode_config(id)
    names = {'baseline_off','baseline_on','position_off','position_on'};
    name = names{id}; augmentation = id >= 3; cache_enabled = mod(id, 2) == 0;
end

function out = summarize(raw)
    modes = {'baseline_off','baseline_on','position_off','position_on'};
    metrics = {'coverage_percent','expensive_planner_calls','T_candidate_s', ...
        'T_strategy_s','T_stage3_s','T_path_cost_s','T_total_s','cache_memory_bytes'};
    rows = repmat(struct(), numel(modes), 1);
    for i = 1:numel(modes)
        q = raw(strcmp(raw.mode, modes{i}), :); rows(i).mode = modes{i}; rows(i).runs = height(q);
        for mi = 1:numel(metrics)
            name = metrics{mi}; values = q.(name);
            rows(i).([name '_mean']) = mean(values);
            rows(i).([name '_median']) = median(values);
            rows(i).([name '_std']) = std(values);
        end
    end
    out = struct2table(rows, 'AsArray', true);
end

function out = paired_speedups(raw)
    comparisons = {'baseline_cache','position_cache','combined'};
    off_modes = {'baseline_off','position_off','baseline_off'};
    on_modes = {'baseline_on','position_on','position_on'};
    rows = repmat(struct('comparison','','pairs',0,'mean_speedup',0, ...
        'median_speedup',0,'std_speedup',0), numel(comparisons), 1);
    for i = 1:numel(comparisons)
        off = sortrows(raw(strcmp(raw.mode, off_modes{i}), :), {'seed','repeat'});
        on = sortrows(raw(strcmp(raw.mode, on_modes{i}), :), {'seed','repeat'});
        assert(isequal(off.seed, on.seed) && isequal(off.repeat, on.repeat), ...
            'Paired timing rows do not align.');
        speedup = off.T_total_s ./ on.T_total_s;
        rows(i).comparison = comparisons{i}; rows(i).pairs = numel(speedup);
        rows(i).mean_speedup = mean(speedup); rows(i).median_speedup = median(speedup);
        rows(i).std_speedup = std(speedup);
    end
    out = struct2table(rows, 'AsArray', true);
end

function write_checkpoint(rows, root)
    d = output_dir(root);
    writetable(struct2table(rows, 'AsArray', true), ...
        fullfile(d, 'position_only_isolated_four_mode_partial.csv'));
end

function write_outputs(outputs, root)
    d = output_dir(root);
    writetable(outputs.raw, fullfile(d, 'position_only_isolated_four_mode_raw.csv'));
    writetable(outputs.summary, fullfile(d, 'position_only_isolated_four_mode_summary.csv'));
    writetable(outputs.paired_speedup, fullfile(d, 'position_only_isolated_paired_speedup.csv'));
    save(fullfile(d, 'position_only_isolated_four_mode_20260901.mat'), 'outputs');
end

function d = output_dir(root)
    d = fullfile(root, 'results', 'tables'); if ~isfolder(d), mkdir(d); end
end

function cfg = experiment_config()
    cfg = planner_config(); cfg.strategy.name = 'bidirectional';
    cfg.candidate_augmentation.enable = false;
    cfg.runtime.disable_visualization = true; cfg.runtime.disable_video = true;
    cfg.runtime.quiet = true; cfg.runtime.quiet_stage2 = true;
    cfg.benchmark.enable = false; cfg.visualization.show_scene = false;
    cfg.visualization.show_docking = false; cfg.visualization.show_global_path = false;
    cfg.strategy.common.route_shortlist_size = min(15, cfg.strategy.common.route_shortlist_size);
end

function options = defaults(options)
    if ~isfield(options, 'seeds'), options.seeds = 1:5; end
    if ~isfield(options, 'repeats'), options.repeats = 3; end
    if ~isfield(options, 'timing_order_seed'), options.timing_order_seed = 91001; end
    if ~isfield(options, 'run_warmup'), options.run_warmup = true; end
    if ~isfield(options, 'write_files'), options.write_files = true; end
end

function r = empty_row()
    r = struct('seed',0,'repeat',0,'mode','','execution_index',0, ...
        'timing_order_seed',0,'execution_order','','tasks',0,'coverage_percent',0, ...
        'verified_completion_percent',0,'added_candidates',0,'selected_docks',0, ...
        'task_path_m',0,'return_path_m',0,'total_path_m',0,'objective',0, ...
        'T_candidate_s',0,'T_strategy_s',0,'T_stage3_s',0,'T_path_cost_s',0, ...
        'T_total_s',0,'expensive_planner_calls',0,'cache_queries',0,'cache_hits',0, ...
        'cache_misses',0,'cache_memory_bytes',0,'visit_order','');
end
