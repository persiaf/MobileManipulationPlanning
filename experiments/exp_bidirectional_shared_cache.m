function [raw, summary] = exp_bidirectional_shared_cache(options)
%EXP_BIDIRECTIONAL_SHARED_CACHE Compare cache-off and cache-on wall clock.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath'))); init_paths(root);
    if isfield(options, 'raw_table')
        raw = options.raw_table; summary = summarize(raw);
        if options.write_files, write_outputs(raw, summary, root); end
        return;
    end
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    rows = repmat(empty_row(), 0, 1);
    for si = 1:numel(options.scales)
        scale = options.scales{si};
        for seed = options.seeds
            t_scene = tic; scene = build_scene_from_cfg(large_scale_profile_config(scale, seed)); T_scene = toc(t_scene);
            robot_cfg = robot_config(); cfg = experiment_config();
            task_table_map = assign_tasks_to_tables(scene); regions = build_task_regions(scene, task_table_map);
            t_candidate = tic; library = build_candidate_library(regions, scene, robot_cfg, cfg); T_candidate = toc(t_candidate);
            for enabled = [false, true]
                mode = cache_mode(enabled); cfg_i = cfg;
                cfg_i.strategy.bidirectional.use_shared_path_cost_cache = enabled;
                scene_id = sprintf('%s_seed%d_%s', scale, seed, mode);
                path_cost_audit('reset', scene_id);
                path_cost_audit('set_context', 'bidirectional', 'OTHER', scene_id);
                t_strategy = tic; plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg_i); T_strategy = toc(t_strategy);
                t_stage3 = tic;
                [~, ~, ~, visit_order, ~, total_path, stats] = stage3_global_planning( ...
                    scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
                    robot_cfg, cfg_i, plan.pose_info_list, struct(), plan);
                T_stage3 = toc(t_stage3);
                entries = path_cost_audit('snapshot'); path_cost_audit('disable');
                cache = stats.timing.shared_path_cache;
                row = empty_row(); row.scale = scale; row.seed = seed; row.mode = mode;
                row.task_count = scene.num_tasks; row.T_scene = T_scene; row.T_candidate = T_candidate;
                row.T_strategy = T_strategy; row.T_stage3 = T_stage3;
                row.T_path_cost_stage3 = stats.timing.distance_matrix_time_s;
                row.T_total = T_scene + T_candidate + T_strategy + T_stage3;
                row.expensive_planner_calls = sum([entries.expensive_computation]);
                row.cache_query_count = cache.cache_query_count; row.cache_hit_count = cache.cache_hit_count;
                row.cache_miss_count = cache.cache_miss_count; row.strategy_cache_hits = cache.strategy_cache_hits;
                row.refinement_cache_hits = cache.refinement_cache_hits; row.stage3_cache_hits = cache.stage3_cache_hits;
                row.final_route_cache_hits = cache.final_route_cache_hits; row.memory_estimate_bytes = cache.memory_estimate_bytes;
                row.coverage = 100 * numel(plan.covered_tasks_predicted) / max(scene.num_tasks, 1);
                row.selected_docks = plan.num_docking_poses; row.total_path_length = total_path;
                row.objective = plan.objective_value; row.visit_order = mat2str(visit_order);
                rows(end + 1, 1) = row; %#ok<AGROW>
                fprintf('SHARED_CACHE scale=%s seed=%d mode=%s calls=%d total=%.3f\n', ...
                    scale, seed, mode, row.expensive_planner_calls, row.T_total);
            end
        end
    end
    raw = struct2table(rows, 'AsArray', true); summary = summarize(raw);
    if options.write_files, write_outputs(raw, summary, root); end
end

function write_outputs(raw, summary, root)
    out_dir = fullfile(root, 'results', 'tables'); if ~isfolder(out_dir), mkdir(out_dir); end
    writetable(raw, fullfile(out_dir, 'bidirectional_shared_cache_raw.csv'));
    writetable(summary, fullfile(out_dir, 'bidirectional_shared_cache_summary.csv'));
end

function cfg = experiment_config()
    cfg = planner_config(); cfg.strategy.name = 'bidirectional';
    cfg.runtime.disable_visualization = true; cfg.runtime.disable_video = true;
    cfg.runtime.quiet = true; cfg.runtime.quiet_stage2 = true; cfg.benchmark.enable = false;
    cfg.visualization.show_scene = false; cfg.visualization.show_docking = false;
    cfg.visualization.show_global_path = false;
    cfg.strategy.common.route_shortlist_size = min(15, cfg.strategy.common.route_shortlist_size);
end

function mode = cache_mode(enabled)
    if enabled, mode = 'cache_on'; else, mode = 'cache_off'; end
end

function out = summarize(raw)
    scales = unique(raw.scale, 'stable'); out = table(scales, 'VariableNames', {'scale'});
    metrics = {'T_total','T_strategy','T_stage3','T_path_cost_stage3','expensive_planner_calls', ...
        'cache_hit_count','memory_estimate_bytes','coverage','selected_docks','total_path_length','objective'};
    for mi = 1:numel(metrics)
        name = metrics{mi}; off = zeros(numel(scales),1); on = off;
        for si = 1:numel(scales)
            off(si) = mean(raw.(name)(strcmp(raw.scale, scales{si}) & strcmp(raw.mode, 'cache_off')));
            on(si) = mean(raw.(name)(strcmp(raw.scale, scales{si}) & strcmp(raw.mode, 'cache_on')));
        end
        out.([name '_off_mean']) = off; out.([name '_on_mean']) = on;
    end
    out.total_speedup = out.T_total_off_mean ./ out.T_total_on_mean;
    out.strategy_speedup = out.T_strategy_off_mean ./ out.T_strategy_on_mean;
    out.stage3_speedup = out.T_stage3_off_mean ./ out.T_stage3_on_mean;
end

function o = defaults(o)
    if ~isfield(o, 'scales'), o.scales = {'s1','s4'}; end
    if ~isfield(o, 'seeds'), o.seeds = 1:5; end
    if ~isfield(o, 'write_files'), o.write_files = true; end
end

function r = empty_row()
    r = struct('scale','','seed',0,'mode','','task_count',0,'T_scene',0,'T_candidate',0, ...
        'T_strategy',0,'T_stage3',0,'T_path_cost_stage3',0,'T_total',0, ...
        'expensive_planner_calls',0,'cache_query_count',0,'cache_hit_count',0,'cache_miss_count',0, ...
        'strategy_cache_hits',0,'refinement_cache_hits',0,'stage3_cache_hits',0, ...
        'final_route_cache_hits',0,'memory_estimate_bytes',0,'coverage',0,'selected_docks',0, ...
        'total_path_length',0,'objective',0,'visit_order','');
end
