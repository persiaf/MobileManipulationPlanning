function [trace_table, summary_table] = exp_c2_candidate_failure_diagnosis(options)
%EXP_C2_CANDIDATE_FAILURE_DIAGNOSIS Trace C2 uncovered task geometry.

    if nargin < 1 || isempty(options), options = struct(); end
    options = default_options(options);
    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    rows = repmat(empty_trace_row(), 0, 1);
    for seed = options.seeds
        cfg = scene_complexity_profile_config('C2', seed);
        scene = build_scene_from_cfg(cfg);
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        robot_cfg = robot_geometry_profile('legacy_validated');

        baseline = build_library(scene, regions, robot_cfg, 'off');
        augmented = build_library(scene, regions, robot_cfg, 'position_only');
        baseline_support = task_support_counts(baseline, scene.num_tasks);
        augmented_support = task_support_counts(augmented, scene.num_tasks);

        uncovered = find(baseline_support == 0 | augmented_support == 0);
        for i = 1:numel(uncovered)
            tid = uncovered(i);
            rows(end + 1, 1) = trace_task(scene, regions, robot_cfg, tid, ...
                baseline_support(tid), augmented_support(tid)); %#ok<AGROW>
            rows(end).seed = seed;
        end
    end

    trace_table = struct2table(rows, 'AsArray', true);
    summary_table = summarize_trace(trace_table);
    ensure_result_dir();
    writetable(trace_table, fullfile(project_root, 'results', 'tables', ...
        'c2_failure_geometry_trace.csv'));
    writetable(summary_table, fullfile(project_root, 'results', 'tables', ...
        'c2_failure_geometry_summary.csv'));
end

function library = build_library(scene, regions, robot_cfg, mode)
    planner_cfg = planner_config();
    planner_cfg.runtime.disable_visualization = true;
    planner_cfg.runtime.disable_video = true;
    planner_cfg.benchmark.enable = false;
    planner_cfg.visualization.show_scene = false;
    planner_cfg.visualization.show_docking = false;
    planner_cfg.visualization.show_global_path = false;
    planner_cfg.candidate_augmentation.enable = false;
    if strcmpi(mode, 'position_only')
        planner_cfg.candidate_augmentation.enable = true;
        planner_cfg.candidate_augmentation.variant = 'position_only';
    end
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
end

function row = trace_task(scene, regions, robot_cfg, task_id, base_support, aug_support)
    row = empty_trace_row();
    row.task_id = task_id;
    task = scene.tasks(task_id);
    row.table_id = task.table_id;
    cfg = scene.scene_cfg;
    center = cfg.table_positions(task.table_id, :);
    theta = cfg.table_orientations(task.table_id);
    local = (rot2(-theta) * ([task.x, task.y] - center)')';
    row.task_local_x = local(1);
    row.task_local_y = local(2);
    row.task_world_x = task.x;
    row.task_world_y = task.y;
    row.table_x = center(1);
    row.table_y = center(2);
    row.table_theta = theta;
    [row.nearest_table_edge, row.nearest_edge_distance] = nearest_edge(local, cfg.table.length, cfg.table.width);
    row.baseline_support_count = base_support;
    row.position_augmented_support_count = aug_support;

    region = regions([regions.table_id] == task.table_id);
    task_positions = zeros(numel(region.task_ids), 2);
    for i = 1:numel(region.task_ids)
        tid = region.task_ids(i);
        task_positions(i, :) = [scene.tasks(tid).x, scene.tasks(tid).y];
    end
    reach_field = compute_reachability_field(center, theta, cfg, robot_cfg, ...
        scene.map_info, mean(task_positions, 1), task_positions);
    raw = reach_field.candidates;
    side_ids = reach_field.side_ids(:);
    best = nearest_collision_valid_candidate(raw, side_ids, scene, robot_cfg, task);
    if isempty(best.pose)
        row.position_only_failure_category = 'A_NO_COLLISION_VALID_POSE';
        row.geometry_class = 'COLLISION_REACHABILITY_COUPLING';
        return;
    end

    pose = best.pose;
    pose_local = (rot2(-theta) * (pose(1:2) - center)')';
    row.nearest_candidate_side = best.side_id;
    row.normal_offset = normal_offset_from_side(pose_local, best.side_id, cfg.table.length, cfg.table.width);
    row.tangential_offset = tangential_offset_from_side(pose_local, best.side_id);
    row.candidate_heading = pose(3);
    row.table_relative_heading = wrapToPiLocal(pose(3) - theta);
    row.task_relative_heading = wrapToPiLocal(atan2(task.y - pose(2), task.x - pose(1)) - pose(3));
    row.candidate_to_task_distance = hypot(task.x - pose(1), task.y - pose(2));
    [ls, rs, cs] = evaluate_dual_arm_reachability(pose, [task.x, task.y, task.z], ...
        robot_cfg, planner_config().reachability.threshold);
    row.left_arm_reach_score = ls;
    row.right_arm_reach_score = rs;
    row.combined_reach_score = cs;

    if aug_support > base_support
        row.position_only_failure_category = 'RECOVERED_BY_POSITION_ONLY';
    elseif aug_support == 0 && cs <= 0
        row.position_only_failure_category = 'C_COLLISION_VALID_BUT_UNREACHABLE';
    elseif aug_support == 0
        row.position_only_failure_category = 'D_NO_SUPPORT_FOR_CONTINUOUS_POSITION';
    else
        row.position_only_failure_category = 'PARTIAL_SUPPORT_NOT_SELECTED';
    end
    row.geometry_class = classify_geometry(row);
end

function best = nearest_collision_valid_candidate(raw, side_ids, scene, robot_cfg, task)
    best.pose = [];
    best.side_id = 0;
    best.distance = inf;
    for i = 1:size(raw, 1)
        pose = raw(i, :);
        if check_footprint_collision(pose, scene.grid_map, scene.map_info, robot_cfg)
            continue;
        end
        d = hypot(task.x - pose(1), task.y - pose(2));
        if d < best.distance
            best.pose = pose;
            best.side_id = side_ids(i);
            best.distance = d;
        end
    end
end

function counts = task_support_counts(library, n_tasks)
    counts = zeros(n_tasks, 1);
    for ri = 1:numel(library.regions)
        candidates = library.regions(ri).candidates;
        for ci = 1:numel(candidates)
            ids = candidates(ci).covered_tasks;
            counts(ids) = counts(ids) + 1;
        end
    end
end

function [edge, dist] = nearest_edge(local, L, W)
    distances = [W/2 - local(2), W/2 + local(2), L/2 + local(1), L/2 - local(1)];
    names = {'plus_y','minus_y','minus_x','plus_x'};
    [dist, idx] = min(distances);
    edge = names{idx};
end

function value = normal_offset_from_side(local, side_id, L, W)
    switch side_id
        case 1, value = local(2) - W / 2;
        case 2, value = -W / 2 - local(2);
        case 3, value = -L / 2 - local(1);
        otherwise, value = local(1) - L / 2;
    end
end

function value = tangential_offset_from_side(local, side_id)
    if side_id <= 2
        value = local(1);
    else
        value = local(2);
    end
end

function cls = classify_geometry(row)
    if abs(row.task_local_y) < 0.05
        cls = 'KNOWN_POSITION_GAP';
    elseif row.combined_reach_score <= 0 && row.candidate_to_task_distance > 0.25
        cls = 'CONTINUOUS_REACH_BOUNDARY';
    elseif row.combined_reach_score <= 0
        cls = 'ORIENTATION_COUPLED_GAP';
    else
        cls = 'UNKNOWN';
    end
end

function summary = summarize_trace(trace)
    if isempty(trace)
        summary = table();
        return;
    end
    groups = unique(trace(:, {'geometry_class','position_only_failure_category'}), 'rows');
    summary = groups;
    summary.task_count = zeros(height(groups), 1);
    summary.mean_candidate_to_task_distance = NaN(height(groups), 1);
    summary.mean_combined_reach_score = NaN(height(groups), 1);
    for i = 1:height(groups)
        mask = strcmp(trace.geometry_class, groups.geometry_class{i}) & ...
            strcmp(trace.position_only_failure_category, groups.position_only_failure_category{i});
        summary.task_count(i) = sum(mask);
        summary.mean_candidate_to_task_distance(i) = mean(trace.candidate_to_task_distance(mask), 'omitnan');
        summary.mean_combined_reach_score(i) = mean(trace.combined_reach_score(mask), 'omitnan');
    end
end

function row = empty_trace_row()
    row = struct('scene_case','C2','seed',0,'task_id',0,'table_id',0, ...
        'task_local_x',NaN,'task_local_y',NaN,'task_world_x',NaN,'task_world_y',NaN, ...
        'table_x',NaN,'table_y',NaN,'table_theta',NaN,'nearest_table_edge','', ...
        'nearest_edge_distance',NaN,'baseline_support_count',0, ...
        'position_augmented_support_count',0,'nearest_candidate_side',0, ...
        'normal_offset',NaN,'tangential_offset',NaN,'candidate_heading',NaN, ...
        'table_relative_heading',NaN,'task_relative_heading',NaN, ...
        'candidate_to_task_distance',NaN,'left_arm_reach_score',NaN, ...
        'right_arm_reach_score',NaN,'combined_reach_score',NaN, ...
        'position_only_failure_category','','geometry_class','UNKNOWN');
end

function options = default_options(options)
    if ~isfield(options, 'seeds'), options.seeds = [2026, 2027, 2028]; end
end

function ensure_result_dir()
    out_dir = fullfile('results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
end
