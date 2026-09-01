function outputs = exp_large_scale_coverage_diagnosis(options)
%EXP_LARGE_SCALE_COVERAGE_DIAGNOSIS Diagnose library and strategy coverage.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    if isfield(options, 'task_table')
        outputs.task = options.task_table;
        outputs.run = options.run_table;
        outputs.region = options.region_table;
        outputs.scale = summarize_scales(outputs.run);
        outputs.s4_seed = outputs.run(strcmp(outputs.run.scale, 's4'), :);
        outputs.s4_failure = summarize_failures(outputs.task(strcmp(outputs.task.scale, 's4'), :));
        outputs.s4_region = summarize_regions(outputs.region(strcmp(outputs.region.scale, 's4'), :));
        if options.write_files, write_outputs(outputs, root); end
        return;
    end
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    task_rows = repmat(empty_task_row(), 0, 1);
    run_rows = repmat(empty_run_row(), 0, 1);
    region_rows = repmat(empty_region_row(), 0, 1);

    for si = 1:numel(options.scales)
        scale_id = options.scales{si};
        for seed = options.seeds
            scene = build_scene_from_cfg(large_scale_profile_config(scale_id, seed));
            robot_cfg = robot_config();
            planner_cfg = diagnostic_planner_config();
            task_table_map = assign_tasks_to_tables(scene);
            regions = build_task_regions(scene, task_table_map);
            library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
            [library_tasks, task_region, task_table, nearest_dist, nearest_reach] = ...
                inspect_library(library, regions, scene);
            greedy_docks = greedy_cover_count(library, library_tasks);
            plans = struct();
            for name = {'bottom_up', 'top_down', 'bidirectional'}
                cfg_i = planner_cfg; cfg_i.strategy.name = name{1};
                plans.(name{1}) = run_planning_strategy(library, regions, scene, robot_cfg, cfg_i);
            end

            rr = empty_run_row(); rr.scale = scale_id; rr.seed = seed;
            rr.total_tasks = scene.num_tasks; rr.candidate_count = library.total_candidates;
            rr.library_coverable = numel(library_tasks);
            rr.library_coverage_rate = 100 * rr.library_coverable / max(scene.num_tasks, 1);
            rr.greedy_full_library_docks_approx = greedy_docks;
            rr.estimated_initial_dock_budget = sum_region_budgets(library, regions, planner_cfg);
            for name = {'bottom_up', 'top_down', 'bidirectional'}
                plan = plans.(name{1});
                rr.([name{1} '_covered']) = numel(plan.covered_tasks_predicted);
                rr.([name{1} '_coverage_rate']) = 100 * numel(plan.covered_tasks_predicted) / max(scene.num_tasks, 1);
                rr.([name{1} '_selected_docks']) = plan.num_docking_poses;
            end
            run_rows(end + 1, 1) = rr; %#ok<AGROW>

            for tid = 1:scene.num_tasks
                tr = empty_task_row(); tr.scale = scale_id; tr.seed = seed; tr.task_id = tid;
                tr.region_id = task_region(tid); tr.table_id = task_table(tid);
                tr.library_coverable = ismember(tid, library_tasks);
                tr.bottom_up_covered = ismember(tid, plans.bottom_up.covered_tasks_predicted);
                tr.top_down_covered = ismember(tid, plans.top_down.covered_tasks_predicted);
                tr.bidirectional_covered = ismember(tid, plans.bidirectional.covered_tasks_predicted);
                tr.nearest_feasible_dock_distance_m = nearest_dist(tid);
                tr.nearest_candidate_reachability_score = nearest_reach(tid);
                if ~tr.library_coverable
                    tr.failure_category = 'NOT_IN_LIBRARY';
                elseif ~(tr.bottom_up_covered && tr.top_down_covered && tr.bidirectional_covered)
                    tr.failure_category = 'LIBRARY_COVERABLE_NOT_SELECTED';
                else
                    tr.failure_category = '';
                end
                task_rows(end + 1, 1) = tr; %#ok<AGROW>
            end
            region_rows = append_region_rows(region_rows, scale_id, seed, library, regions, plans);
            fprintf('COVERAGE_DIAG scale=%s seed=%d library=%.1f%% strategies=[%.1f %.1f %.1f]\n', ...
                scale_id, seed, rr.library_coverage_rate, rr.bottom_up_coverage_rate, ...
                rr.top_down_coverage_rate, rr.bidirectional_coverage_rate);
        end
    end

    outputs.task = struct2table(task_rows, 'AsArray', true);
    outputs.run = struct2table(run_rows, 'AsArray', true);
    outputs.region = struct2table(region_rows, 'AsArray', true);
    outputs.scale = summarize_scales(outputs.run);
    outputs.s4_seed = outputs.run(strcmp(outputs.run.scale, 's4'), :);
    outputs.s4_failure = summarize_failures(outputs.task(strcmp(outputs.task.scale, 's4'), :));
    outputs.s4_region = summarize_regions(outputs.region(strcmp(outputs.region.scale, 's4'), :));
    if options.write_files, write_outputs(outputs, root); end
end

function cfg = diagnostic_planner_config()
    cfg = planner_config();
    cfg.runtime.disable_visualization = true; cfg.runtime.disable_video = true;
    cfg.runtime.quiet = true; cfg.runtime.quiet_stage2 = true;
    cfg.benchmark.enable = false; cfg.visualization.show_scene = false;
    cfg.visualization.show_docking = false; cfg.visualization.show_global_path = false;
    cfg.strategy.common.route_shortlist_size = min(15, cfg.strategy.common.route_shortlist_size);
end

function [covered, task_region, task_table, nearest_dist, nearest_reach] = inspect_library(library, regions, scene)
    covered = []; task_region = zeros(scene.num_tasks, 1); task_table = zeros(scene.num_tasks, 1);
    nearest_dist = inf(scene.num_tasks, 1); nearest_reach = nan(scene.num_tasks, 1);
    for ri = 1:numel(regions)
        tids = regions(ri).task_ids(:)'; task_region(tids) = ri; task_table(tids) = regions(ri).table_id;
        candidates = library.regions(ri).candidates;
        if isempty(candidates), continue; end
        covered = union(covered, unique([candidates.covered_tasks]), 'stable');
        for local = 1:numel(tids)
            tid = tids(local); task_xy = [scene.tasks(tid).x, scene.tasks(tid).y];
            poses = reshape([candidates.pose], 3, [])';
            nearest_dist(tid) = min(vecnorm(poses(:, 1:2) - task_xy, 2, 2));
            values = arrayfun(@(c) c.reach_scores(local, 3), candidates);
            nearest_reach(tid) = max(values);
        end
    end
end

function count = greedy_cover_count(library, target)
    remaining = target(:)'; count = 0;
    candidate_sets = {};
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            candidate_sets{end + 1} = library.regions(ri).candidates(ci).covered_tasks; %#ok<AGROW>
        end
    end
    active = true(1, numel(candidate_sets));
    while ~isempty(remaining)
        gains = cellfun(@(tasks) numel(intersect(tasks, remaining)), candidate_sets);
        gains(~active) = 0;
        [gain, idx] = max(gains);
        if isempty(gain) || gain == 0, count = NaN; return; end
        remaining = setdiff(remaining, candidate_sets{idx}, 'stable');
        active(idx) = false; count = count + 1;
    end
end

function total = sum_region_budgets(library, regions, cfg)
    total = 0;
    for ri = 1:numel(regions)
        total = total + estimate_docking_budget(library.regions(ri).candidates, regions(ri), cfg);
    end
end

function rows = append_region_rows(rows, scale, seed, library, regions, plans)
    for ri = 1:numel(regions)
        r = empty_region_row(); r.scale = scale; r.seed = seed; r.region_id = ri;
        r.table_id = regions(ri).table_id; r.task_count = numel(regions(ri).task_ids);
        r.candidate_count = library.regions(ri).num_candidates;
        if r.candidate_count > 0, lib = unique([library.regions(ri).candidates.covered_tasks]); else, lib = []; end
        r.library_coverable = numel(intersect(lib, regions(ri).task_ids));
        r.bottom_up_covered = numel(intersect(plans.bottom_up.covered_tasks_predicted, regions(ri).task_ids));
        r.top_down_covered = numel(intersect(plans.top_down.covered_tasks_predicted, regions(ri).task_ids));
        r.bidirectional_covered = numel(intersect(plans.bidirectional.covered_tasks_predicted, regions(ri).task_ids));
        rows(end + 1, 1) = r; %#ok<AGROW>
    end
end

function out = summarize_scales(run)
    scales = unique(run.scale, 'stable'); out = table(scales, 'VariableNames', {'scale'});
    fields = {'total_tasks','library_coverage_rate','bottom_up_coverage_rate','top_down_coverage_rate','bidirectional_coverage_rate'};
    for fi = 1:numel(fields), out.(fields{fi}) = arrayfun(@(i) mean(run.(fields{fi})(strcmp(run.scale, scales{i}))), (1:numel(scales))'); end
end

function out = summarize_failures(task)
    failed = task(~strcmp(task.failure_category, ''), :);
    reasons = unique(failed.failure_category, 'stable'); counts = zeros(numel(reasons), 1);
    for i = 1:numel(reasons), counts(i) = sum(strcmp(failed.failure_category, reasons{i})); end
    shares = 100 * counts / max(height(failed), 1);
    out = table(reasons, counts, shares, 'VariableNames', {'failure_category','task_count','share_percent'});
end

function out = summarize_regions(region)
    ids = unique(region.region_id); out = table(ids, 'VariableNames', {'region_id'});
    fields = {'task_count','candidate_count','library_coverable','bottom_up_covered','top_down_covered','bidirectional_covered'};
    for fi = 1:numel(fields), out.(fields{fi}) = arrayfun(@(id) sum(region.(fields{fi})(region.region_id == id)), ids); end
    out.library_coverage_rate = 100 * out.library_coverable ./ max(out.task_count, 1);
    out.final_coverage_rate = 100 * out.bottom_up_covered ./ max(out.task_count, 1);
end

function write_outputs(o, root)
    dirpath = fullfile(root, 'results', 'tables'); if ~isfolder(dirpath), mkdir(dirpath); end
    writetable(o.task, fullfile(dirpath, 'large_scale_coverage_diagnostics.csv'));
    writetable(o.run, fullfile(dirpath, 'large_scale_coverage_run_summary.csv'));
    writetable(o.scale, fullfile(dirpath, 'large_scale_coverage_scale_summary.csv'));
    writetable(o.s4_seed, fullfile(dirpath, 'large_scale_coverage_s4_seed_summary.csv'));
    writetable(o.s4_failure, fullfile(dirpath, 'large_scale_coverage_s4_failure_summary.csv'));
    writetable(o.s4_region, fullfile(dirpath, 'large_scale_coverage_s4_region_summary.csv'));
end

function o = defaults(o)
    if ~isfield(o, 'scales'), o.scales = {'s1','s2','s3','s4'}; end
    if ~isfield(o, 'seeds'), o.seeds = 1:5; end
    if ~isfield(o, 'write_files'), o.write_files = true; end
end

function r = empty_task_row()
    r = struct('scale','','seed',0,'task_id',0,'region_id',0,'table_id',0, ...
        'library_coverable',false,'bottom_up_covered',false,'top_down_covered',false, ...
        'bidirectional_covered',false,'failure_category','', ...
        'nearest_feasible_dock_distance_m',NaN,'nearest_candidate_reachability_score',NaN);
end
function r = empty_run_row()
    r = struct('scale','','seed',0,'total_tasks',0,'candidate_count',0,'library_coverable',0, ...
        'library_coverage_rate',0,'greedy_full_library_docks_approx',0,'estimated_initial_dock_budget',0, ...
        'bottom_up_covered',0,'bottom_up_coverage_rate',0,'bottom_up_selected_docks',0, ...
        'top_down_covered',0,'top_down_coverage_rate',0,'top_down_selected_docks',0, ...
        'bidirectional_covered',0,'bidirectional_coverage_rate',0,'bidirectional_selected_docks',0);
end
function r = empty_region_row()
    r = struct('scale','','seed',0,'region_id',0,'table_id',0,'task_count',0,'candidate_count',0, ...
        'library_coverable',0,'bottom_up_covered',0,'top_down_covered',0,'bidirectional_covered',0);
end
