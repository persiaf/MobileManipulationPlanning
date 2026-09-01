function outputs = exp_candidate_augmentation_strategies(options)
%EXP_CANDIDATE_AUGMENTATION_STRATEGIES Evaluate Joint with full planners.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath'))); init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    strategy_rows = repmat(empty_strategy_row(), 0, 1);
    combined_rows = repmat(empty_combined_row(), 0, 1);

    for seed = options.seeds
        scene = build_scene_from_cfg(large_scale_profile_config(options.scale, seed));
        robot_cfg = robot_config(); cfg = experiment_config();
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        cfg.candidate_augmentation.enable = true;
        cfg.candidate_augmentation.variant = 'joint';
        t_library = tic; library = build_candidate_library(regions, scene, robot_cfg, cfg);
        library_time = toc(t_library);
        dock_requirement = greedy_cover_count(library, 1:scene.num_tasks);

        for strategy = {'bottom_up','top_down'}
            cfg_i = cfg; cfg_i.strategy.name = strategy{1};
            result = run_strategy(scene, regions, task_table_map, robot_cfg, ...
                cfg_i, library, library_time, dock_requirement);
            strategy_rows(end + 1, 1) = result; %#ok<AGROW>
            print_strategy(result);
        end

        for cache_enabled = [false, true]
            cfg_i = cfg; cfg_i.strategy.name = 'bidirectional';
            cfg_i.strategy.bidirectional.use_shared_path_cost_cache = cache_enabled;
            scene_id = sprintf('%s_seed%d_aug_cache%d', options.scale, seed, cache_enabled);
            path_cost_audit('reset', scene_id);
            path_cost_audit('set_context', 'bidirectional', 'OTHER', scene_id);
            result = run_strategy(scene, regions, task_table_map, robot_cfg, ...
                cfg_i, library, library_time, dock_requirement);
            entries = path_cost_audit('snapshot'); path_cost_audit('disable');
            result.expensive_planner_calls = sum([entries.expensive_computation]);
            combined = empty_combined_row();
            combined.scale = options.scale; combined.seed = seed;
            combined.augmentation = true; combined.cache_enabled = cache_enabled;
            combined.coverage_percent = result.coverage_percent;
            combined.verified_completion_percent = result.verified_completion_percent;
            combined.selected_docks = result.selected_docks;
            combined.total_path_length_m = result.total_path_length_m;
            combined.objective = result.objective;
            combined.T_total_s = result.T_total_s;
            combined.T_candidate_s = result.T_candidate_s;
            combined.T_strategy_s = result.T_strategy_s;
            combined.T_stage3_s = result.T_stage3_s;
            combined.expensive_planner_calls = result.expensive_planner_calls;
            combined.added_candidates = result.added_candidates;
            combined_rows(end + 1, 1) = combined; %#ok<AGROW>
            if cache_enabled
                strategy_rows(end + 1, 1) = result; %#ok<AGROW>
                print_strategy(result);
            end
            fprintf('AUGMENT_COMBINED seed=%d cache=%d coverage=%.3f total=%.3f calls=%d\n', ...
                seed, cache_enabled, combined.coverage_percent, combined.T_total_s, ...
                combined.expensive_planner_calls);
        end
    end

    outputs.strategy = struct2table(strategy_rows, 'AsArray', true);
    outputs.combined = struct2table(combined_rows, 'AsArray', true);
    outputs.strategy_summary = summarize_strategy(outputs.strategy);
    outputs.combined_summary = summarize_combined(outputs.combined);
    if options.write_files, write_outputs(outputs, root); end
end

function result = run_strategy(scene, regions, task_table_map, robot_cfg, cfg, ...
        library, library_time, dock_requirement)
    t_strategy = tic;
    plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg);
    strategy_time = toc(t_strategy);
    t_stage3 = tic;
    [~,~,~,~,executed,total_path,stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, cfg, plan.pose_info_list, struct(), plan);
    stage3_time = toc(t_stage3);
    result = empty_strategy_row(); result.scale = 's4';
    result.seed = scene.scene_cfg.random_seed; result.strategy = cfg.strategy.name;
    result.cache_enabled = cfg.strategy.bidirectional.use_shared_path_cost_cache && ...
        strcmp(cfg.strategy.name, 'bidirectional');
    result.tasks = scene.num_tasks;
    result.library_coverage_percent = 100 * library.augmentation_diagnostics.library_coverage_after;
    result.added_candidates = library.augmentation_diagnostics.selected_supplementary_count;
    result.candidate_count = library.total_candidates;
    result.greedy_full_library_docks = dock_requirement;
    result.selected_docks = plan.num_docking_poses;
    result.coverage_percent = 100 * numel(plan.covered_tasks_predicted) / scene.num_tasks;
    result.verified_completion_percent = get_num(stats,'verified_completion_rate', ...
        100*numel(executed)/scene.num_tasks);
    result.total_path_length_m = total_path; result.objective = plan.objective_value;
    result.T_candidate_s = library_time; result.T_strategy_s = strategy_time;
    result.T_stage3_s = stage3_time;
    result.T_total_s = library_time + strategy_time + stage3_time;
end

function count = greedy_cover_count(library, target)
    candidates = {};
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            candidates{end+1} = library.regions(ri).candidates(ci).covered_tasks; %#ok<AGROW>
        end
    end
    remaining = target; available = true(1,numel(candidates)); count = 0;
    while ~isempty(remaining)
        gains = zeros(1,numel(candidates));
        for i = find(available), gains(i)=numel(intersect(candidates{i},remaining)); end
        [gain,best] = max(gains); if gain==0, break; end
        count=count+1; remaining=setdiff(remaining,candidates{best}); available(best)=false;
    end
    if ~isempty(remaining), count=Inf; end
end

function cfg = experiment_config()
    cfg = planner_config(); cfg.runtime.disable_visualization = true;
    cfg.runtime.disable_video = true; cfg.runtime.quiet = true;
    cfg.runtime.quiet_stage2 = true; cfg.benchmark.enable = false;
    cfg.visualization.show_scene = false; cfg.visualization.show_docking = false;
    cfg.visualization.show_global_path = false;
    cfg.strategy.common.route_shortlist_size = min(15,cfg.strategy.common.route_shortlist_size);
end

function print_strategy(r)
    fprintf(['AUGMENT_STRATEGY seed=%d strategy=%s coverage=%.3f verified=%.3f ' ...
        'docks=%d requirement=%d path=%.4f total=%.3f\n'], r.seed,r.strategy, ...
        r.coverage_percent,r.verified_completion_percent,r.selected_docks, ...
        r.greedy_full_library_docks,r.total_path_length_m,r.T_total_s);
end

function out = summarize_strategy(raw)
    names = unique(raw.strategy,'stable'); rows=repmat(empty_strategy_summary(),numel(names),1);
    for i=1:numel(names)
        q=raw(strcmp(raw.strategy,names{i}),:); r=empty_strategy_summary(); r.strategy=names{i}; r.runs=height(q);
        r.coverage_mean_percent=mean(q.coverage_percent); r.coverage_min_percent=min(q.coverage_percent);
        r.verified_mean_percent=mean(q.verified_completion_percent);
        r.selected_docks_mean=mean(q.selected_docks); r.dock_requirement_mean=mean(q.greedy_full_library_docks);
        r.total_path_mean_m=mean(q.total_path_length_m); r.T_total_mean_s=mean(q.T_total_s); r.objective_mean=mean(q.objective);
        rows(i)=r;
    end
    out=struct2table(rows,'AsArray',true);
end

function out = summarize_combined(raw)
    modes=[false,true]; rows=repmat(empty_combined_summary(),2,1);
    for i=1:2
        q=raw(raw.cache_enabled==modes(i),:); r=empty_combined_summary(); r.cache_enabled=modes(i); r.runs=height(q);
        r.coverage_mean_percent=mean(q.coverage_percent); r.T_total_mean_s=mean(q.T_total_s);
        r.expensive_planner_calls_mean=mean(q.expensive_planner_calls);
        r.total_path_mean_m=mean(q.total_path_length_m); r.selected_docks_mean=mean(q.selected_docks); rows(i)=r;
    end
    out=struct2table(rows,'AsArray',true);
end

function write_outputs(o,root)
    d=fullfile(root,'results','tables'); if ~isfolder(d),mkdir(d);end
    writetable(o.strategy,fullfile(d,'candidate_augmentation_strategy_raw.csv'));
    writetable(o.combined,fullfile(d,'candidate_augmentation_combined_raw.csv'));
    writetable(o.strategy_summary,fullfile(d,'candidate_augmentation_strategy_summary.csv'));
    writetable(o.combined_summary,fullfile(d,'candidate_augmentation_combined_summary.csv'));
    outputs=o; save(fullfile(d,'candidate_augmentation_strategies_20260901.mat'),'outputs');
end

function o=defaults(o)
    if ~isfield(o,'scale'),o.scale='s4';end
    if ~isfield(o,'seeds'),o.seeds=1:5;end
    if ~isfield(o,'write_files'),o.write_files=true;end
end

function r=empty_strategy_row()
    r=struct('scale','','seed',0,'strategy','','cache_enabled',false,'tasks',0, ...
        'library_coverage_percent',0,'added_candidates',0,'candidate_count',0, ...
        'greedy_full_library_docks',0,'selected_docks',0,'coverage_percent',0, ...
        'verified_completion_percent',0,'total_path_length_m',0,'objective',0, ...
        'T_candidate_s',0,'T_strategy_s',0,'T_stage3_s',0,'T_total_s',0, ...
        'expensive_planner_calls',0);
end
function r=empty_combined_row()
    r=struct('scale','','seed',0,'augmentation',true,'cache_enabled',false, ...
        'coverage_percent',0,'verified_completion_percent',0,'selected_docks',0, ...
        'total_path_length_m',0,'objective',0,'T_total_s',0,'T_candidate_s',0, ...
        'T_strategy_s',0,'T_stage3_s',0,'expensive_planner_calls',0,'added_candidates',0);
end
function r=empty_strategy_summary()
    r=struct('strategy','','runs',0,'coverage_mean_percent',0,'coverage_min_percent',0, ...
        'verified_mean_percent',0,'selected_docks_mean',0,'dock_requirement_mean',0, ...
        'total_path_mean_m',0,'T_total_mean_s',0,'objective_mean',0);
end
function r=empty_combined_summary()
    r=struct('cache_enabled',false,'runs',0,'coverage_mean_percent',0, ...
        'T_total_mean_s',0,'expensive_planner_calls_mean',0, ...
        'total_path_mean_m',0,'selected_docks_mean',0);
end
function v=get_num(s,n,d)
    if isstruct(s)&&isfield(s,n)&&isnumeric(s.(n))&&isscalar(s.(n)),v=s.(n);else,v=d;end
end
