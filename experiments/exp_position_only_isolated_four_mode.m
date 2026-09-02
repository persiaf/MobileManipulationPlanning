function outputs = exp_position_only_isolated_four_mode(options)
%EXP_POSITION_ONLY_ISOLATED_FOUR_MODE Resource-bounded single-process benchmark.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'pipeline')); init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    if options.run_warmup, run_warmup(); end
    rows = load_checkpoint(root, options);

    for seed = options.seeds
        if seed_complete(rows, seed, options.repeats)
            fprintf('RESOURCE-BOUNDED seed=%d already complete; validating checkpoint\n', seed);
            assert_seed_semantics(rows, seed, options.repeats);
            continue;
        end
        state = load_or_build_seed_state(root, seed, options);
        for repeat = 1:options.repeats
            order_seed = options.timing_order_seed + seed * 100 + repeat;
            stream = RandStream('mt19937ar', 'Seed', order_seed);
            order = randperm(stream, 4);
            for execution_index = 1:4
                mode_id = order(execution_index);
                if has_row(rows, seed, repeat, mode_id)
                    fprintf('RESOURCE-BOUNDED seed=%d repeat=%d mode=%s already complete\n', ...
                        seed, repeat, mode_config(mode_id));
                    continue;
                end
                [mode, augmentation, cache_enabled] = mode_config(mode_id);
                if augmentation
                    library = state.position;
                    candidate_time = state.position_time;
                    cfg_run = state.cfg_position;
                else
                    library = state.baseline;
                    candidate_time = state.baseline_time;
                    cfg_run = state.cfg;
                end
                cfg_run.strategy.bidirectional.use_shared_path_cost_cache = cache_enabled;
                scene_id = sprintf('resource_s4_seed%d_repeat%d_%s', seed, repeat, mode);
                path_cost_audit('reset_count_only', scene_id);
                path_cost_audit('set_context', 'bidirectional', 'OTHER', scene_id);
                row = run_mode(state, cfg_run, library, candidate_time, mode, seed, ...
                    repeat, execution_index, order_seed, mat2str(order));
                row.expensive_planner_calls = path_cost_audit('expensive_count');
                path_cost_audit('disable');
                rows(end + 1, 1) = row; %#ok<AGROW>
                assert_unique_rows(rows);
                fprintf(['RESOURCE-BOUNDED seed=%d repeat=%d order=%d mode=%s ' ...
                    'coverage=%.3f calls=%d total=%.3f\n'], seed, repeat, ...
                    execution_index, mode, row.coverage_percent, ...
                    row.expensive_planner_calls, row.T_total_s);
                if options.write_files, write_checkpoint(rows, root); end
            end
            assert_repeat_semantics(rows, seed, repeat);
        end
    end

    assert_batch_complete(rows, options);
    rows = sort_rows(rows);
    outputs.raw = struct2table(rows, 'AsArray', true);
    outputs.summary = summarize(outputs.raw);
    outputs.paired = paired_summary(outputs.raw);
    outputs.seed_summary = seed_summary(outputs.raw);
    outputs.augmentation_cost = augmentation_cost(outputs.raw);
    outputs.combined = combined_summary(outputs.raw);
    outputs.protocol = struct('name', 'resource-bounded isolated four-mode timing', ...
        'seeds', options.seeds, 'repeats', options.repeats, ...
        'candidate_generation', 'once per seed; time reused across repeats', ...
        'single_matlab_process', true);
    if options.write_files, write_outputs(outputs, root); end
end

function row = run_mode(state, cfg, library, candidate_time, mode, seed, ...
        repeat, execution_index, order_seed, order)
    t_strategy = tic;
    plan = run_planning_strategy(library, state.regions, state.scene, ...
        state.robot_cfg, cfg);
    strategy_time = toc(t_strategy);
    t_stage3 = tic;
    [~,~,~,visit_order,~,total_path,stats] = stage3_global_planning( ...
        state.scene, plan.selected_poses, plan.task_sequence_summary, ...
        state.task_table_map, state.robot_cfg, cfg, plan.pose_info_list, ...
        struct(), plan);
    stage3_time = toc(t_stage3);
    cache = stats.timing.shared_path_cache;
    row = empty_row();
    row.seed = seed; row.repeat = repeat; row.mode = mode;
    row.execution_index = execution_index; row.timing_order_seed = order_seed;
    row.execution_order = order; row.tasks = state.scene.num_tasks;
    row.coverage_percent = 100 * numel(plan.covered_tasks_predicted) / state.scene.num_tasks;
    row.verified_completion_percent = stats.verified_completion_rate;
    row.added_candidates = library.augmentation_diagnostics.selected_supplementary_count;
    row.selected_docks = plan.num_docking_poses;
    row.task_path_m = stats.task_path_len_m; row.return_path_m = stats.return_path_len_m;
    row.total_path_m = total_path; row.objective = plan.objective_value;
    row.T_candidate_s = candidate_time; row.T_strategy_s = strategy_time;
    row.T_stage3_s = stage3_time; row.T_path_cost_s = stats.timing.distance_matrix_time_s;
    row.T_total_s = candidate_time + strategy_time + stage3_time;
    row.cache_queries = cache.cache_query_count; row.cache_hits = cache.cache_hit_count;
    row.cache_misses = cache.cache_miss_count;
    row.cache_memory_bytes = cache.memory_estimate_bytes;
    row.visit_order = mat2str(visit_order);
end

function run_warmup()
    fprintf('RESOURCE-BOUNDED warm-up start (excluded from results)\n');
    scene = build_scene_from_cfg(large_scale_profile_config('smoke', 1));
    robot_cfg = robot_config(); cfg = experiment_config();
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    baseline = build_candidate_library(regions, scene, robot_cfg, cfg);
    cfg.candidate_augmentation.enable = true;
    cfg.candidate_augmentation.variant = 'position_only';
    augmented = augment_candidate_library(baseline, regions, scene, robot_cfg, cfg);
    cfg.strategy.bidirectional.use_shared_path_cost_cache = true;
    plan = run_planning_strategy(augmented, regions, scene, robot_cfg, cfg);
    stage3_global_planning(scene, plan.selected_poses, plan.task_sequence_summary, ...
        task_table_map, robot_cfg, cfg, plan.pose_info_list, struct(), plan);
    fprintf(['RESOURCE-BOUNDED warm-up complete (planner, convex_astar, ' ...
        'Position-only, shared cache)\n']);
end

function state = load_or_build_seed_state(root, seed, options)
    file = seed_state_file(root, seed);
    if options.resume && options.write_files && isfile(file)
        loaded = load(file, 'state');
        assert(isfield(loaded, 'state') && loaded.state.protocol_version == 1 && ...
            loaded.state.seed == seed, 'Invalid resource-bounded seed-state file.');
        state = loaded.state;
        fprintf('RESOURCE-BOUNDED seed=%d state restored\n', seed);
        return;
    end
    state = struct(); state.protocol_version = 1; state.seed = seed;
    state.scene = build_scene_from_cfg(large_scale_profile_config('s4', seed));
    state.robot_cfg = robot_config(); state.cfg = experiment_config();
    state.task_table_map = assign_tasks_to_tables(state.scene);
    state.regions = build_task_regions(state.scene, state.task_table_map);
    t_baseline = tic;
    state.baseline = build_candidate_library(state.regions, state.scene, ...
        state.robot_cfg, state.cfg);
    state.baseline_time = toc(t_baseline);
    state.cfg_position = state.cfg;
    state.cfg_position.candidate_augmentation.enable = true;
    state.cfg_position.candidate_augmentation.variant = 'position_only';
    [state.position, diagnostics] = augment_candidate_library(state.baseline, ...
        state.regions, state.scene, state.robot_cfg, state.cfg_position);
    state.position_time = state.baseline_time + diagnostics.supplementation_time_s;
    if options.write_files, save(file, 'state', '-v7.3'); end
end

function rows = load_checkpoint(root, options)
    rows = repmat(empty_row(), 0, 1);
    file = checkpoint_file(root);
    if ~options.resume || ~options.write_files || ~isfile(file), return; end
    table_rows = readtable(file, 'TextType', 'string');
    expected = fieldnames(empty_row())';
    assert(isequal(table_rows.Properties.VariableNames, expected), ...
        'Resource-bounded checkpoint schema does not match this driver.');
    rows = table2struct(table_rows);
    for i = 1:numel(rows)
        rows(i).mode = char(rows(i).mode);
        rows(i).execution_order = char(rows(i).execution_order);
        rows(i).visit_order = char(rows(i).visit_order);
    end
    assert_unique_rows(rows);
    fprintf('RESOURCE-BOUNDED restored %d completed mode runs\n', numel(rows));
end

function assert_repeat_semantics(rows, seed, repeat)
    pair = repeat_rows(rows, seed, repeat);
    assert_cache_semantics(pair(1), pair(2));
    assert_cache_semantics(pair(3), pair(4));
end

function assert_seed_semantics(rows, seed, repeats)
    for repeat = 1:repeats, assert_repeat_semantics(rows, seed, repeat); end
end

function pair = repeat_rows(rows, seed, repeat)
    pair = repmat(empty_row(), 4, 1);
    for mode_id = 1:4
        index = find(row_mask(rows, seed, repeat, mode_id));
        assert(numel(index) == 1, 'Expected exactly one row for each completed mode.');
        pair(mode_id) = rows(index);
    end
end

function identical = cache_semantics_identical(off, on)
    identical = abs(off.coverage_percent - on.coverage_percent) < 1e-9 && ...
        abs(off.verified_completion_percent - on.verified_completion_percent) < 1e-9 && ...
        off.selected_docks == on.selected_docks && strcmp(off.visit_order, on.visit_order) && ...
        abs(off.total_path_m - on.total_path_m) < 1e-9 && ...
        abs(off.objective - on.objective) < 1e-9;
end

function assert_cache_semantics(off, on)
    assert(cache_semantics_identical(off, on), ...
        'Cache changed coverage, completion, docks, visit order, path, or objective.');
end

function [name, augmentation, cache_enabled] = mode_config(id)
    names = {'baseline_off','baseline_on','position_off','position_on'};
    name = names{id}; augmentation = id >= 3; cache_enabled = mod(id, 2) == 0;
end

function out = summarize(raw)
    modes = {'baseline_off','baseline_on','position_off','position_on'};
    metrics = {'coverage_percent','verified_completion_percent','selected_docks', ...
        'expensive_planner_calls','T_candidate_s','T_strategy_s','T_stage3_s', ...
        'T_path_cost_s','T_total_s','cache_memory_bytes'};
    rows = repmat(struct(), numel(modes), 1);
    for i = 1:numel(modes)
        q = raw(strcmp(raw.mode, modes{i}), :);
        rows(i).mode = modes{i}; rows(i).runs = height(q);
        for mi = 1:numel(metrics)
            name = metrics{mi}; values = q.(name);
            rows(i).([name '_mean']) = mean(values);
            rows(i).([name '_median']) = median(values);
            rows(i).([name '_std']) = std(values);
            rows(i).([name '_min']) = min(values);
            rows(i).([name '_max']) = max(values);
        end
    end
    out = struct2table(rows, 'AsArray', true);
end

function out = paired_summary(raw)
    comparisons = {'M0_to_M1','M2_to_M3'};
    off_modes = {'baseline_off','position_off'};
    on_modes = {'baseline_on','position_on'};
    rows = repmat(comparison_stats('', 0, 0), numel(comparisons), 1);
    for i = 1:numel(comparisons)
        [off, on] = aligned_rows(raw, off_modes{i}, on_modes{i});
        speedup = off.T_total_s ./ on.T_total_s;
        reduction = 1 - on.expensive_planner_calls ./ off.expensive_planner_calls;
        rows(i) = comparison_stats(comparisons{i}, speedup, reduction);
        identity = true(height(off), 1);
        off_struct = table2struct(off); on_struct = table2struct(on);
        for j = 1:height(off)
            identity(j) = cache_semantics_identical(off_struct(j), on_struct(j));
        end
        rows(i).semantic_identity = all(identity);
    end
    out = struct2table(rows, 'AsArray', true);
end

function row = comparison_stats(name, speedup, reduction)
    row = struct('comparison', name, 'pairs', numel(speedup), ...
        'calls_reduction_mean', mean(reduction), ...
        'calls_reduction_median', median(reduction), ...
        'calls_reduction_std', std(reduction), ...
        'calls_reduction_min', min(reduction), ...
        'calls_reduction_max', max(reduction), ...
        'paired_speedup_mean', mean(speedup), ...
        'paired_speedup_median', median(speedup), ...
        'paired_speedup_std', std(speedup), ...
        'paired_speedup_min', min(speedup), ...
        'paired_speedup_max', max(speedup), 'semantic_identity', false);
end

function out = seed_summary(raw)
    seeds = unique(raw.seed)';
    rows = repmat(struct('seed',0,'M0_coverage_percent',0,'M3_coverage_percent',0, ...
        'M0_T_total_mean_s',0,'M3_T_total_mean_s',0), numel(seeds), 1);
    for i = 1:numel(seeds)
        m0 = raw(raw.seed == seeds(i) & strcmp(raw.mode, 'baseline_off'), :);
        m3 = raw(raw.seed == seeds(i) & strcmp(raw.mode, 'position_on'), :);
        rows(i).seed = seeds(i); rows(i).M0_coverage_percent = mean(m0.coverage_percent);
        rows(i).M3_coverage_percent = mean(m3.coverage_percent);
        rows(i).M0_T_total_mean_s = mean(m0.T_total_s);
        rows(i).M3_T_total_mean_s = mean(m3.T_total_s);
    end
    out = struct2table(rows, 'AsArray', true);
end

function out = augmentation_cost(raw)
    [m0, m2] = aligned_rows(raw, 'baseline_off', 'position_off');
    out = table(m0.seed, m0.repeat, m2.selected_docks - m0.selected_docks, ...
        m2.expensive_planner_calls - m0.expensive_planner_calls, ...
        m2.T_candidate_s - m0.T_candidate_s, m2.T_total_s - m0.T_total_s, ...
        'VariableNames', {'seed','repeat','additional_docks','additional_planner_calls', ...
        'additional_candidate_time_s','additional_total_time_s'});
end

function out = combined_summary(raw)
    [m0, m3] = aligned_rows(raw, 'baseline_off', 'position_on');
    if isempty(m0)
        out = table(0, NaN, NaN, NaN, NaN, NaN, NaN, false, false, ...
            'VariableNames', {'pairs','coverage_gain_mean_percent', ...
            'coverage_gain_median_percent','paired_speedup_mean', ...
            'paired_speedup_median','paired_speedup_min','paired_speedup_max', ...
            'coverage_higher_all_pairs','runtime_lower_all_pairs'});
        return;
    end
    coverage_gain = m3.coverage_percent - m0.coverage_percent;
    speedup = m0.T_total_s ./ m3.T_total_s;
    out = table(height(m0), mean(coverage_gain), median(coverage_gain), ...
        mean(speedup), median(speedup), min(speedup), max(speedup), ...
        all(coverage_gain > 0), all(m3.T_total_s < m0.T_total_s), ...
        'VariableNames', {'pairs','coverage_gain_mean_percent','coverage_gain_median_percent', ...
        'paired_speedup_mean','paired_speedup_median','paired_speedup_min', ...
        'paired_speedup_max','coverage_higher_all_pairs','runtime_lower_all_pairs'});
end

function [off, on] = aligned_rows(raw, off_mode, on_mode)
    off = sortrows(raw(strcmp(raw.mode, off_mode), :), {'seed','repeat'});
    on = sortrows(raw(strcmp(raw.mode, on_mode), :), {'seed','repeat'});
    assert(isequal(off.seed, on.seed) && isequal(off.repeat, on.repeat), ...
        'Paired timing rows do not align.');
end

function write_checkpoint(rows, root)
    rows = sort_rows(rows);
    writetable(struct2table(rows, 'AsArray', true), checkpoint_file(root));
end

function write_outputs(outputs, root)
    d = output_dir(root);
    writetable(outputs.raw, fullfile(d, 'position_only_resource_bounded_four_mode_raw.csv'));
    writetable(outputs.summary, fullfile(d, 'position_only_resource_bounded_four_mode_summary.csv'));
    writetable(outputs.paired, fullfile(d, 'position_only_resource_bounded_paired.csv'));
    writetable(outputs.seed_summary, fullfile(d, 'position_only_resource_bounded_seed_summary.csv'));
    writetable(outputs.augmentation_cost, fullfile(d, ...
        'position_only_resource_bounded_augmentation_cost.csv'));
    writetable(outputs.combined, fullfile(d, 'position_only_resource_bounded_combined.csv'));
    save(fullfile(d, 'position_only_resource_bounded_four_mode_20260902.mat'), ...
        'outputs', '-v7.3');
end

function complete = seed_complete(rows, seed, repeats)
    complete = true;
    for repeat = 1:repeats
        for mode_id = 1:4
            complete = complete && has_row(rows, seed, repeat, mode_id);
        end
    end
end

function found = has_row(rows, seed, repeat, mode_id)
    found = any(row_mask(rows, seed, repeat, mode_id));
end

function mask = row_mask(rows, seed, repeat, mode_id)
    if isempty(rows), mask = false(0, 1); return; end
    modes = {rows.mode};
    mask = [rows.seed]' == seed & [rows.repeat]' == repeat & ...
        strcmp(modes', mode_config(mode_id));
end

function assert_unique_rows(rows)
    if isempty(rows), return; end
    keys = arrayfun(@(r) sprintf('%d/%d/%s', r.seed, r.repeat, r.mode), ...
        rows, 'UniformOutput', false);
    assert(numel(unique(keys)) == numel(keys), 'Checkpoint contains duplicate mode runs.');
end

function assert_batch_complete(rows, options)
    assert_unique_rows(rows);
    assert(numel(rows) == numel(options.seeds) * options.repeats * 4, ...
        'Resource-bounded batch is incomplete.');
    for seed = options.seeds
        assert(seed_complete(rows, seed, options.repeats), 'Batch has missing mode runs.');
        assert_seed_semantics(rows, seed, options.repeats);
    end
end

function rows = sort_rows(rows)
    if isempty(rows), return; end
    table_rows = struct2table(rows, 'AsArray', true);
    table_rows = sortrows(table_rows, {'seed','repeat','execution_index'});
    rows = table2struct(table_rows);
end

function file = checkpoint_file(root)
    file = fullfile(output_dir(root), 'position_only_resource_bounded_four_mode_partial.csv');
end

function file = seed_state_file(root, seed)
    file = fullfile(output_dir(root), sprintf( ...
        'position_only_resource_bounded_seed%d_state.mat', seed));
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
    cfg.strategy.common.route_shortlist_size = min(15, ...
        cfg.strategy.common.route_shortlist_size);
end

function options = defaults(options)
    if ~isfield(options, 'seeds'), options.seeds = [1, 3, 5]; end
    if ~isfield(options, 'repeats'), options.repeats = 2; end
    if ~isfield(options, 'timing_order_seed'), options.timing_order_seed = 91001; end
    if ~isfield(options, 'run_warmup'), options.run_warmup = true; end
    if ~isfield(options, 'write_files'), options.write_files = true; end
    if ~isfield(options, 'resume'), options.resume = true; end
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
