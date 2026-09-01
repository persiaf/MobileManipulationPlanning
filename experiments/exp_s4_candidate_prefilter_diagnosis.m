function outputs = exp_s4_candidate_prefilter_diagnosis(options)
%EXP_S4_CANDIDATE_PREFILTER_DIAGNOSIS Trace candidate support before library use.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    task_rows = repmat(empty_task_row(), 0, 1);
    region_rows = repmat(empty_region_row(), 0, 1);
    run_rows = repmat(empty_run_row(), 0, 1);

    for seed = options.seeds
        scene = build_scene_from_cfg(large_scale_profile_config(options.scale, seed));
        robot_cfg = robot_config(); planner_cfg = diagnostic_planner_config();
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
        run = empty_run_row(); run.scale = options.scale; run.seed = seed;
        run.total_tasks = scene.num_tasks;

        for ri = 1:numel(regions)
            d = library.regions(ri).generation_diagnostics;
            region_task_rows = repmat(empty_task_row(), numel(d.task_ids), 1);
            for ti = 1:numel(d.task_ids)
                tid = d.task_ids(ti); row = empty_task_row();
                row.scale = options.scale; row.seed = seed; row.task_id = tid;
                row.region_id = ri; row.table_id = regions(ri).table_id;
                row.x = scene.tasks(tid).x; row.y = scene.tasks(tid).y;
                [row.table_local_x, row.table_local_y] = table_local_position( ...
                    scene, row.table_id, row.x, row.y);
                row.raw_support_count = d.raw_support_count(ti);
                row.collision_valid_support_count = d.collision_valid_support_count(ti);
                row.reachable_support_count = d.reachable_support_count(ti);
                row.library_support_count = d.library_support_count(ti);
                row.distance_to_table_edge_m = distance_to_table_edge(scene, row.table_id, row.x, row.y);
                row.distance_to_nearest_valid_dock_m = nearest_valid_dock(library.regions(ri).candidates, row.x, row.y);
                row.failure_category = classify_task(row);
                region_task_rows(ti) = row;
            end
            task_rows = [task_rows; region_task_rows]; %#ok<AGROW>
            rr = empty_region_row(); rr.scale = options.scale; rr.seed = seed;
            rr.region_id = ri; rr.table_id = regions(ri).table_id;
            rr.task_count = numel(d.task_ids); rr.failed_tasks = sum(~strcmp({region_task_rows.failure_category}, 'LIBRARY_COVERABLE'));
            rr.raw_candidate_count = d.raw_candidate_count;
            rr.collision_rejected_candidate_count = d.collision_rejected_candidate_count;
            rr.collision_valid_candidate_count = d.collision_valid_candidate_count;
            rr.reachability_rejected_candidate_count = d.reachability_rejected_candidate_count;
            rr.reachability_valid_candidate_count = d.reachability_valid_candidate_count;
            rr.library_candidate_count = d.library_candidate_count;
            rr.main_failure_reason = main_failure_reason(region_task_rows);
            region_rows(end + 1, 1) = rr; %#ok<AGROW>
        end
        seed_rows = task_rows(strcmp({task_rows.scale}, options.scale) & [task_rows.seed] == seed);
        run.library_coverable_tasks = sum(strcmp({seed_rows.failure_category}, 'LIBRARY_COVERABLE'));
        run.library_coverage_rate = 100 * run.library_coverable_tasks / max(run.total_tasks, 1);
        run_rows(end + 1, 1) = run; %#ok<AGROW>
        fprintf('PREFILTER_DIAG scale=%s seed=%d coverage=%.1f%%\n', ...
            options.scale, seed, run.library_coverage_rate);
    end

    outputs.task = struct2table(task_rows, 'AsArray', true);
    outputs.region = struct2table(region_rows, 'AsArray', true);
    outputs.run = struct2table(run_rows, 'AsArray', true);
    outputs.failure = failure_summary(outputs.task);
    outputs.region_summary = region_summary(outputs.region);
    if options.write_files, write_outputs(outputs, root); end
end

function cfg = diagnostic_planner_config()
    cfg = planner_config();
    cfg.runtime.disable_visualization = true; cfg.runtime.disable_video = true;
    cfg.visualization.show_scene = false; cfg.visualization.show_docking = false;
    cfg.visualization.show_global_path = false; cfg.benchmark.enable = false;
end

function category = classify_task(row)
    if row.raw_support_count == 0
        category = 'NO_RAW_SUPPORT';
    elseif row.collision_valid_support_count == 0
        category = 'ALL_RAW_COLLISION_REJECTED';
    elseif row.reachable_support_count == 0
        category = 'COLLISION_VALID_BUT_UNREACHABLE';
    elseif row.library_support_count == 0
        category = 'REMOVED_DURING_LIBRARY_BUILD';
    else
        category = 'LIBRARY_COVERABLE';
    end
end

function reason = main_failure_reason(rows)
    categories = {rows.failure_category}; categories = categories(~strcmp(categories, 'LIBRARY_COVERABLE'));
    if isempty(categories), reason = 'NONE'; return; end
    names = unique(categories, 'stable'); counts = cellfun(@(x) sum(strcmp(categories, x)), names);
    [~, idx] = max(counts); reason = names{idx};
end

function d = distance_to_table_edge(scene, table_id, x, y)
    [local_x, local_y] = table_local_position(scene, table_id, x, y);
    half_length = scene.scene_cfg.table.length / 2;
    half_width = scene.scene_cfg.table.width / 2;
    d = min(abs(half_length - abs(local_x)), abs(half_width - abs(local_y)));
end

function [local_x, local_y] = table_local_position(scene, table_id, x, y)
    center = scene.scene_cfg.table_positions(table_id, :);
    theta = scene.scene_cfg.table_orientations(table_id);
    local = rot2(-theta) * ([x, y] - center)';
    local_x = local(1); local_y = local(2);
end

function d = nearest_valid_dock(candidates, x, y)
    if isempty(candidates), d = NaN; return; end
    poses = reshape([candidates.pose], 3, [])';
    d = min(vecnorm(poses(:, 1:2) - [x, y], 2, 2));
end

function out = failure_summary(task)
    failed = task(~strcmp(task.failure_category, 'LIBRARY_COVERABLE'), :);
    names = {'NO_RAW_SUPPORT';'ALL_RAW_COLLISION_REJECTED'; ...
        'COLLISION_VALID_BUT_UNREACHABLE';'REMOVED_DURING_LIBRARY_BUILD'};
    count = cellfun(@(x) sum(strcmp(failed.failure_category, x)), names);
    share = 100 * count / max(height(failed), 1);
    out = table(names, count, share, 'VariableNames', {'failure_category','count','share_percent'});
end

function out = region_summary(region)
    ids = unique(region.region_id); n = numel(ids);
    tasks = zeros(n,1); failed = zeros(n,1); reason = cell(n,1);
    for i = 1:n
        rows = region(region.region_id == ids(i), :);
        tasks(i) = sum(rows.task_count); failed(i) = sum(rows.failed_tasks);
        names = rows.main_failure_reason(~strcmp(rows.main_failure_reason, 'NONE'));
        if isempty(names), reason{i} = 'NONE'; else, reason{i} = names{1}; end
    end
    out = table(ids, tasks, failed, reason, 'VariableNames', ...
        {'region_id','tasks','failed_tasks','main_failure_reason'});
end

function write_outputs(o, root)
    out_dir = fullfile(root, 'results', 'tables'); if ~isfolder(out_dir), mkdir(out_dir); end
    writetable(o.task, fullfile(out_dir, 's4_candidate_prefilter_task_trace.csv'));
    writetable(o.region, fullfile(out_dir, 's4_candidate_prefilter_region_trace.csv'));
    writetable(o.run, fullfile(out_dir, 's4_candidate_prefilter_run_summary.csv'));
    writetable(o.failure, fullfile(out_dir, 's4_candidate_prefilter_failure_summary.csv'));
    writetable(o.region_summary, fullfile(out_dir, 's4_candidate_prefilter_region_summary.csv'));
end

function o = defaults(o)
    if ~isfield(o, 'scale'), o.scale = 's4'; end
    if ~isfield(o, 'seeds'), o.seeds = 1:5; end
    if ~isfield(o, 'write_files'), o.write_files = true; end
end

function r = empty_task_row()
    r = struct('scale','','seed',0,'task_id',0,'region_id',0,'table_id',0,'x',NaN,'y',NaN, ...
        'table_local_x',NaN,'table_local_y',NaN, ...
        'raw_support_count',0,'collision_valid_support_count',0,'reachable_support_count',0, ...
        'library_support_count',0,'distance_to_table_edge_m',NaN, ...
        'distance_to_nearest_valid_dock_m',NaN,'failure_category','');
end
function r = empty_region_row()
    r = struct('scale','','seed',0,'region_id',0,'table_id',0,'task_count',0,'failed_tasks',0, ...
        'raw_candidate_count',0,'collision_rejected_candidate_count',0, ...
        'collision_valid_candidate_count',0,'reachability_rejected_candidate_count',0, ...
        'reachability_valid_candidate_count',0,'library_candidate_count',0,'main_failure_reason','');
end
function r = empty_run_row()
    r = struct('scale','','seed',0,'total_tasks',0,'library_coverable_tasks',0,'library_coverage_rate',0);
end
