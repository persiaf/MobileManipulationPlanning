function outputs = exp_position_only_full_validation(options)
%EXP_POSITION_ONLY_FULL_VALIDATION Compare Position-only and Joint planners.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'pipeline')); init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    rows = repmat(empty_row(), 0, 1);

    for seed = options.seeds
        scene = build_scene_from_cfg(large_scale_profile_config('s4', seed));
        robot_cfg = robot_config(); cfg = experiment_config();
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        t_baseline = tic;
        baseline = build_candidate_library(regions, scene, robot_cfg, cfg);
        baseline_time = toc(t_baseline);

        for vi = 1:numel(options.variants)
            variant = options.variants{vi};
            cfg_variant = cfg;
            cfg_variant.candidate_augmentation.enable = true;
            cfg_variant.candidate_augmentation.variant = variant;
            [library, diagnostics] = augment_candidate_library( ...
                baseline, regions, scene, robot_cfg, cfg_variant);
            candidate_time = baseline_time + diagnostics.supplementation_time_s;
            dock_requirement = greedy_cover_count(library, 1:scene.num_tasks);

            for si = 1:numel(options.strategies)
                strategy = options.strategies{si};
                cfg_run = cfg_variant; cfg_run.strategy.name = strategy;
                cfg_run.strategy.bidirectional.use_shared_path_cost_cache = ...
                    strcmp(strategy, 'bidirectional') && options.bidirectional_cache;
                scene_id = sprintf('s4_seed%d_%s_%s', seed, variant, strategy);
                path_cost_audit('reset', scene_id);
                path_cost_audit('set_context', strategy, 'OTHER', scene_id);
                row = run_once(scene, regions, task_table_map, robot_cfg, cfg_run, ...
                    library, candidate_time, diagnostics, dock_requirement, variant);
                entries = path_cost_audit('snapshot'); path_cost_audit('disable');
                row.expensive_planner_calls = sum([entries.expensive_computation]);
                rows(end + 1, 1) = row; %#ok<AGROW>
                fprintf(['POSITION_FULL seed=%d variant=%s strategy=%s coverage=%.3f ' ...
                    'verified=%.3f docks=%d total=%.3f\n'], seed, variant, strategy, ...
                    row.coverage_percent, row.verified_completion_percent, ...
                    row.selected_docks, row.T_total_s);
            end
            if options.write_files, write_checkpoint(rows, root); end
        end
    end

    outputs.raw = struct2table(rows, 'AsArray', true);
    outputs.summary = summarize(outputs.raw);
    outputs.paired = paired_comparison(outputs.raw);
    if options.write_files, write_outputs(outputs, root); end
end

function row = run_once(scene, regions, task_table_map, robot_cfg, cfg, ...
        library, candidate_time, diagnostics, dock_requirement, variant)
    t_strategy = tic;
    plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg);
    strategy_time = toc(t_strategy);
    t_stage3 = tic;
    [~,~,~,visit_order,~,total_path,stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, cfg, plan.pose_info_list, struct(), plan);
    stage3_time = toc(t_stage3);
    row = empty_row(); row.seed = scene.scene_cfg.random_seed;
    row.variant = variant; row.strategy = cfg.strategy.name;
    row.library_coverage_percent = 100 * diagnostics.library_coverage_after;
    row.added_candidates = diagnostics.selected_supplementary_count;
    row.selected_docks = plan.num_docking_poses;
    row.greedy_dock_requirement = dock_requirement;
    row.coverage_percent = 100 * numel(plan.covered_tasks_predicted) / scene.num_tasks;
    row.verified_completion_percent = stats.verified_completion_rate;
    row.task_path_m = stats.task_path_len_m;
    row.return_path_m = stats.return_path_len_m;
    row.total_path_m = total_path; row.objective = plan.objective_value;
    row.T_candidate_s = candidate_time; row.T_strategy_s = strategy_time;
    row.T_path_cost_s = stats.timing.distance_matrix_time_s;
    row.T_stage3_s = stage3_time;
    row.T_total_s = candidate_time + strategy_time + stage3_time;
    row.cache_memory_bytes = stats.timing.shared_path_cache.memory_estimate_bytes;
    row.visit_order = mat2str(visit_order);
end

function out = paired_comparison(raw)
    p = raw(strcmp(raw.variant, 'position_only'), :);
    j = raw(strcmp(raw.variant, 'joint'), :);
    [~, ip, ij] = intersect(strcat(string(p.seed), "_", string(p.strategy)), ...
        strcat(string(j.seed), "_", string(j.strategy)), 'stable');
    p = p(ip, :); j = j(ij, :);
    out = table(p.seed, p.strategy, p.coverage_percent - j.coverage_percent, ...
        p.verified_completion_percent - j.verified_completion_percent, ...
        p.added_candidates - j.added_candidates, ...
        j.T_candidate_s - p.T_candidate_s, p.selected_docks - j.selected_docks, ...
        p.total_path_m - j.total_path_m, p.objective - j.objective, ...
        p.T_total_s - j.T_total_s, ...
        'VariableNames', {'seed','strategy','coverage_delta_percent', ...
        'verified_delta_percent','added_candidate_delta', ...
        'candidate_time_saved_by_position_s','selected_docks_delta', ...
        'total_path_delta_m','objective_delta','T_total_delta_s'});
end

function out = summarize(raw)
    groups = unique(raw(:, {'variant','strategy'}), 'stable');
    rows = repmat(empty_summary(), height(groups), 1);
    for i = 1:height(groups)
        q = raw(strcmp(raw.variant, groups.variant{i}) & ...
            strcmp(raw.strategy, groups.strategy{i}), :);
        r = empty_summary(); r.variant = groups.variant{i};
        r.strategy = groups.strategy{i}; r.runs = height(q);
        r.coverage_mean_percent = mean(q.coverage_percent);
        r.coverage_min_percent = min(q.coverage_percent);
        r.verified_mean_percent = mean(q.verified_completion_percent);
        r.added_candidates_mean = mean(q.added_candidates);
        r.selected_docks_mean = mean(q.selected_docks);
        r.total_path_mean_m = mean(q.total_path_m);
        r.T_candidate_mean_s = mean(q.T_candidate_s);
        r.T_total_mean_s = mean(q.T_total_s); rows(i) = r;
    end
    out = struct2table(rows, 'AsArray', true);
end

function write_checkpoint(rows, root)
    d = output_dir(root);
    writetable(struct2table(rows, 'AsArray', true), ...
        fullfile(d, 'position_only_full_validation_partial.csv'));
end

function write_outputs(outputs, root)
    d = output_dir(root);
    writetable(outputs.raw, fullfile(d, 'position_only_full_validation_raw.csv'));
    writetable(outputs.summary, fullfile(d, 'position_only_full_validation_summary.csv'));
    writetable(outputs.paired, fullfile(d, 'position_only_joint_paired.csv'));
    save(fullfile(d, 'position_only_full_validation_20260901.mat'), 'outputs');
end

function d = output_dir(root)
    d = fullfile(root, 'results', 'tables'); if ~isfolder(d), mkdir(d); end
end

function cfg = experiment_config()
    cfg = planner_config(); cfg.candidate_augmentation.enable = false;
    cfg.runtime.disable_visualization = true; cfg.runtime.disable_video = true;
    cfg.runtime.quiet = true; cfg.runtime.quiet_stage2 = true;
    cfg.benchmark.enable = false; cfg.visualization.show_scene = false;
    cfg.visualization.show_docking = false; cfg.visualization.show_global_path = false;
    cfg.strategy.common.route_shortlist_size = min(15, cfg.strategy.common.route_shortlist_size);
end

function count = greedy_cover_count(library, target)
    candidates = {};
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            candidates{end+1} = library.regions(ri).candidates(ci).covered_tasks; %#ok<AGROW>
        end
    end
    remaining = target; available = true(1, numel(candidates)); count = 0;
    while ~isempty(remaining)
        gains = zeros(1, numel(candidates));
        for i = find(available), gains(i) = numel(intersect(candidates{i}, remaining)); end
        [gain, best] = max(gains); if gain == 0, break; end
        count = count + 1; remaining = setdiff(remaining, candidates{best});
        available(best) = false;
    end
    if ~isempty(remaining), count = Inf; end
end

function options = defaults(options)
    if ~isfield(options, 'seeds'), options.seeds = 1:5; end
    if ~isfield(options, 'variants'), options.variants = {'position_only','joint'}; end
    if ~isfield(options, 'strategies'), options.strategies = {'bottom_up','top_down','bidirectional'}; end
    if ~isfield(options, 'bidirectional_cache'), options.bidirectional_cache = true; end
    if ~isfield(options, 'write_files'), options.write_files = true; end
end

function r = empty_row()
    r = struct('seed',0,'variant','','strategy','','library_coverage_percent',0, ...
        'added_candidates',0,'selected_docks',0,'greedy_dock_requirement',0, ...
        'coverage_percent',0,'verified_completion_percent',0,'task_path_m',0, ...
        'return_path_m',0,'total_path_m',0,'objective',0,'T_candidate_s',0, ...
        'T_strategy_s',0,'T_path_cost_s',0,'T_stage3_s',0,'T_total_s',0, ...
        'expensive_planner_calls',0,'cache_memory_bytes',0,'visit_order','');
end

function r = empty_summary()
    r = struct('variant','','strategy','','runs',0,'coverage_mean_percent',0, ...
        'coverage_min_percent',0,'verified_mean_percent',0,'added_candidates_mean',0, ...
        'selected_docks_mean',0,'total_path_mean_m',0,'T_candidate_mean_s',0, ...
        'T_total_mean_s',0);
end
