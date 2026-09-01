function outputs = exp_path_cost_reuse_audit(options)
%EXP_PATH_COST_REUSE_AUDIT Measure repeated path-cost queries without caching.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    if isfield(options, 'query_table')
        outputs = analyze_queries(options.query_table);
        if options.write_files, write_outputs(outputs, root); end
        return;
    end
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    all_entries = repmat(empty_entry(), 0, 1);

    for si = 1:numel(options.scales)
        scale_id = options.scales{si};
        for seed = options.seeds
            scene_id = sprintf('%s_seed%d', lower(scale_id), seed);
            scene = build_scene_from_cfg(large_scale_profile_config(scale_id, seed));
            robot_cfg = robot_config(); cfg = audit_planner_config();
            task_table_map = assign_tasks_to_tables(scene);
            regions = build_task_regions(scene, task_table_map);
            library = build_candidate_library(regions, scene, robot_cfg, cfg);
            path_cost_audit('reset', scene_id);
            for name = {'bottom_up','top_down','bidirectional'}
                strategy = name{1}; cfg_i = cfg; cfg_i.strategy.name = strategy;
                path_cost_audit('set_context', strategy, 'OTHER', scene_id);
                plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg_i);
                timing = struct('candidate_generation_time_s', library.total_generation_time_s);
                stage3_global_planning(scene, plan.selected_poses, plan.task_sequence_summary, ...
                    task_table_map, robot_cfg, cfg_i, plan.pose_info_list, timing, plan);
            end
            entries = path_cost_audit('snapshot');
            all_entries = [all_entries; entries(:)]; %#ok<AGROW>
            fprintf('PATH_AUDIT scale=%s seed=%d queries=%d\n', scale_id, seed, numel(entries));
        end
    end
    path_cost_audit('disable');
    outputs = analyze_queries(struct2table(all_entries, 'AsArray', true));
    if options.write_files, write_outputs(outputs, root); end
end

function outputs = analyze_queries(query_table)
    outputs.queries = query_table;
    outputs.strategy = summarize_group(outputs.queries, 'strategy');
    outputs.source = summarize_group(outputs.queries, 'query_source');
    outputs.stage3 = summarize_stage3(outputs.queries);
    outputs.duplicate_scope = summarize_duplicate_scope(outputs.queries);
    outputs.bidirectional_top20 = top_bidirectional_pairs(outputs.queries);
    outputs.overall = summarize_overall(outputs.queries);
end

function cfg = audit_planner_config()
    cfg = planner_config();
    cfg.runtime.disable_visualization = true; cfg.runtime.disable_video = true;
    cfg.runtime.quiet = true; cfg.runtime.quiet_stage2 = true;
    cfg.benchmark.enable = false; cfg.visualization.show_scene = false;
    cfg.visualization.show_docking = false; cfg.visualization.show_global_path = false;
    cfg.strategy.common.route_shortlist_size = min(15, cfg.strategy.common.route_shortlist_size);
end

function out = summarize_group(q, field)
    groups = unique(q.(field), 'stable'); n = numel(groups);
    queries = zeros(n,1); unique_pairs = zeros(n,1); repeated = zeros(n,1);
    expensive = zeros(n,1); potential = zeros(n,1);
    for i = 1:n
        part = q(strcmp(q.(field), groups{i}), :); queries(i) = height(part);
        unique_pairs(i) = numel(unique(part.directed_key)); repeated(i) = queries(i) - unique_pairs(i);
        expensive(i) = sum(part.expensive_computation); potential(i) = repeated(i) / max(queries(i), 1);
    end
    out = table(groups, queries, unique_pairs, repeated, potential, expensive, ...
        'VariableNames', {field,'total_path_cost_queries','unique_path_pairs', ...
        'repeated_queries','potential_reuse_ratio','expensive_computations'});
end

function out = summarize_stage3(q)
    strategies = unique(q.strategy, 'stable'); n = numel(strategies);
    total = zeros(n,1); unique_count = zeros(n,1); duplicates = zeros(n,1); prior = zeros(n,1);
    for i = 1:n
        own = q(strcmp(q.strategy, strategies{i}), :);
        stage_mask = endsWith(own.query_source, '_STAGE3'); stage = own(stage_mask, :);
        total(i) = height(stage); keys = unique(stage.directed_key); unique_count(i) = numel(keys);
        duplicates(i) = total(i) - unique_count(i);
        for k = 1:numel(keys)
            first_stage = min(stage.sequence(strcmp(stage.directed_key, keys{k})));
            if any(strcmp(own.directed_key, keys{k}) & own.sequence < first_stage), prior(i) = prior(i) + 1; end
        end
    end
    share = prior ./ max(unique_count, 1);
    out = table(strategies, total, unique_count, duplicates, prior, share, ...
        'VariableNames', {'strategy','stage3_query_count','stage3_unique_pair_count', ...
        'stage3_duplicate_pair_count','pairs_already_computed_before_stage3','share'});
end

function out = summarize_duplicate_scope(q)
    scenes = unique(q.scene_id, 'stable');
    repeated = zeros(numel(scenes),1); within_count = zeros(numel(scenes),1);
    cross_count = zeros(numel(scenes),1); opportunity = zeros(numel(scenes),1);
    for i = 1:numel(scenes)
        part = q(strcmp(q.scene_id, scenes{i}), :); keys = unique(part.directed_key);
        within = 0; cross = 0;
        for k = 1:numel(keys)
            kp = part(strcmp(part.directed_key, keys{k}), :); sources = unique(kp.query_source);
            for s = 1:numel(sources), within = within + max(sum(strcmp(kp.query_source, sources{s})) - 1, 0); end
            cross = cross + max(numel(sources) - 1, 0);
        end
        repeated(i) = height(part)-numel(keys); within_count(i) = within; cross_count(i) = cross;
        opportunity(i) = cross/max(height(part),1);
    end
    out = table(scenes, repeated, within_count, cross_count, opportunity, ...
        'VariableNames', {'scene_id','repeated_queries', ...
        'within_module_duplicate_count','cross_module_duplicate_count','cross_stage_reuse_opportunity'});
end

function out = top_bidirectional_pairs(q)
    part = q(strcmp(q.strategy, 'bidirectional') & strcmp(q.query_source, 'BIDIRECTIONAL_REFINEMENT'), :);
    keys = unique(part.directed_key); count = zeros(numel(keys),1);
    for i = 1:numel(keys), count(i) = sum(strcmp(part.directed_key, keys{i})); end
    [count, order] = sort(count, 'descend'); order = order(1:min(20,numel(order))); count = count(1:numel(order));
    keys = keys(order); out = table(keys, count, 'VariableNames', {'directed_key','query_count'});
end

function out = summarize_overall(q)
    total = height(q); unique_pairs = numel(unique(q.directed_key)); repeated = total - unique_pairs;
    expensive = sum(q.expensive_computation); max_reduction = max(expensive - unique_pairs, 0);
    out = table(total, unique_pairs, repeated, repeated/max(total,1), expensive, max_reduction, ...
        max_reduction/max(expensive,1), 'VariableNames', {'total_path_cost_queries', ...
        'unique_path_pairs','repeated_queries','potential_reuse_ratio','expensive_computations', ...
        'maximum_theoretical_expensive_call_reduction','maximum_theoretical_reduction_ratio'});
end

function write_outputs(o, root)
    dirpath = fullfile(root, 'results', 'tables'); if ~isfolder(dirpath), mkdir(dirpath); end
    writetable(o.queries, fullfile(dirpath, 'path_cost_reuse_queries.csv'));
    writetable(o.strategy, fullfile(dirpath, 'path_cost_reuse_by_strategy.csv'));
    writetable(o.source, fullfile(dirpath, 'path_cost_reuse_by_source.csv'));
    writetable(o.stage3, fullfile(dirpath, 'path_cost_reuse_stage3.csv'));
    writetable(o.duplicate_scope, fullfile(dirpath, 'path_cost_reuse_duplicate_scope.csv'));
    writetable(o.bidirectional_top20, fullfile(dirpath, 'path_cost_reuse_bidirectional_top20.csv'));
    writetable(o.overall, fullfile(dirpath, 'path_cost_reuse_overall.csv'));
end

function o = defaults(o)
    if ~isfield(o, 'scales'), o.scales = {'s1','s4'}; end
    if ~isfield(o, 'seeds'), o.seeds = 1:3; end
    if ~isfield(o, 'write_files'), o.write_files = true; end
end

function e = empty_entry()
    e = struct('sequence',{},'scene_id',{},'strategy',{},'query_source',{},'planner',{}, ...
        'start_x',{},'start_y',{},'start_theta',{},'goal_x',{},'goal_y',{},'goal_theta',{}, ...
        'directed_key',{},'expensive_computation',{});
end
