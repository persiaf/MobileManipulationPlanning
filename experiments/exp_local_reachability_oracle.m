function outputs = exp_local_reachability_oracle(options)
%EXP_LOCAL_REACHABILITY_ORACLE Dense local feasibility oracle for P1.7.
%
% The oracle is diagnostic only. It does not add poses to the production
% candidate library and does not change reachability, collision, or geometry
% parameters.

    if nargin < 1 || isempty(options), options = struct(); end
    options = default_options(options);

    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    rows = repmat(empty_row(), 0, 1);
    aggregate = initialize_aggregate(options.case_id, options.seed);

    scene_cfg = make_scene_config(options.case_id, options.seed);
    scene = build_scene_from_cfg(scene_cfg);
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    robot_cfg = robot_geometry_profile('legacy_validated');
    planner_cfg = planner_config();
    planner_cfg.candidate_augmentation.enable = false;
    planner_cfg.runtime.disable_visualization = true;
    planner_cfg.runtime.disable_video = true;
    planner_cfg.runtime.quiet = true;
    planner_cfg.runtime.quiet_stage2 = true;
    planner_cfg.benchmark.enable = false;

    baseline_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    covered = library_covered_tasks(baseline_library);
    uncovered = setdiff(1:scene.num_tasks, covered, 'stable');

    aggregate.task_count = scene.num_tasks;
    aggregate.uncovered_tasks = numel(uncovered);
    aggregate.baseline_candidate_count = baseline_library.total_candidates;
    aggregate.baseline_coverage_percent = 100 * numel(covered) / max(scene.num_tasks, 1);

    if isfinite(options.max_uncovered_tasks)
        uncovered = uncovered(1:min(numel(uncovered), options.max_uncovered_tasks));
    end

    for i = 1:numel(uncovered)
        task_id = uncovered(i);
        region_idx = find_task_region(regions, task_id);
        row = evaluate_task_oracle(scene, regions(region_idx), baseline_library.regions(region_idx), ...
            task_id, robot_cfg, planner_cfg, options);
        rows(end + 1, 1) = row; %#ok<AGROW>
    end

    task_table = struct2table(rows, 'AsArray', true);
    aggregate = summarize_oracle(aggregate, task_table);
    summary_table = struct2table(aggregate, 'AsArray', true);

    outputs.summary = summary_table;
    outputs.task = task_table;

    ensure_result_dir();
    writetable(summary_table, fullfile(project_root, 'results', 'tables', ...
        'local_reachability_oracle_summary.csv'));
    writetable(task_table, fullfile(project_root, 'results', 'tables', ...
        'local_reachability_oracle_task.csv'));
end

function row = evaluate_task_oracle(scene, region, library_region, task_id, ...
        robot_cfg, planner_cfg, options)
    task = scene.tasks(task_id);
    table_id = task.table_id;
    table_center = scene.scene_cfg.table_positions(table_id, :);
    table_theta = scene.scene_cfg.table_orientations(table_id);
    target = [task.x, task.y, task.z];
    target_xy = target(1:2);
    target_local = rot2(-table_theta) * (target_xy - table_center)';
    table = scene.scene_cfg.table;

    best = initialize_pose_record();
    first_feasible = initialize_pose_record();
    reason_counts = initialize_reason_counts();
    collision_valid_count = 0;
    sampled_pose_count = 0;

    normal_values = local_normal_values(scene.scene_cfg, scene.map_info);
    heading_offsets = deg2rad(options.heading_offsets_deg);

    for side_id = 1:4
        tangents = local_tangent_values(table, scene.map_info, side_id, options);
        for ni = 1:numel(normal_values)
            d = normal_values(ni);
            for ti = 1:numel(tangents)
                tangent = tangents(ti);
                local_xy = side_local_position(side_id, tangent, d, table);
                world_xy = table_center(:) + rot2(table_theta) * local_xy;
                theta_face = atan2(task.y - world_xy(2), task.x - world_xy(1));
                for hi = 1:numel(heading_offsets)
                    pose = [world_xy', wrapToPiLocal(theta_face + heading_offsets(hi))];
                    sampled_pose_count = sampled_pose_count + 1;
                    if check_footprint_collision(pose, scene.grid_map, scene.map_info, robot_cfg)
                        continue;
                    end
                    collision_valid_count = collision_valid_count + 1;
                    [ls, rs, cs] = evaluate_dual_arm_reachability( ...
                        pose, target, robot_cfg, planner_cfg.reachability.threshold);
                    [left_diag, right_diag] = decompose_reachability( ...
                        pose, target, robot_cfg, planner_cfg.reachability.threshold);
                    reason_counts = add_reason(reason_counts, left_diag.reason);
                    reason_counts = add_reason(reason_counts, right_diag.reason);

                    if cs > best.combined_reach_score
                        best = pose_record(pose, table_center, table_theta, target_xy, ...
                            side_id, tangent, d, ls, rs, cs, scene, robot_cfg, ...
                            min(left_diag.task_arm_distance, right_diag.task_arm_distance));
                    end
                    if cs > 0 && ~first_feasible.is_valid
                        first_feasible = pose_record(pose, table_center, table_theta, target_xy, ...
                            side_id, tangent, d, ls, rs, cs, scene, robot_cfg, ...
                            min(left_diag.task_arm_distance, right_diag.task_arm_distance));
                    end
                end
            end
        end
    end

    nearest_production = nearest_production_candidate(library_region, scene, robot_cfg, ...
        planner_cfg, task, table_center, table_theta);

    if first_feasible.is_valid
        selected = first_feasible;
        oracle_class = 'ORACLE_FEASIBLE';
    else
        selected = best;
        oracle_class = 'ORACLE_INFEASIBLE';
    end

    row = empty_row();
    row.case_id = options.case_id;
    row.seed = options.seed;
    row.task_id = task_id;
    row.table_id = table_id;
    row.table_orientation_deg = rad2deg(table_theta);
    row.task_local_x = target_local(1);
    row.task_local_y = target_local(2);
    row.oracle_class = oracle_class;
    row.collision_valid_oracle_poses = collision_valid_count;
    row.sampled_oracle_poses = sampled_pose_count;
    row.max_left_reach_score = max(best.left_reach_score, 0);
    row.max_right_reach_score = max(best.right_reach_score, 0);
    row.max_combined_reach_score = max(best.combined_reach_score, 0);
    row.minimum_task_arm_distance_m = best.minimum_task_arm_distance_m;
    row.best_x = selected.pose(1);
    row.best_y = selected.pose(2);
    row.best_theta = selected.pose(3);
    row.best_table_local_x = selected.table_local_x;
    row.best_table_local_y = selected.table_local_y;
    row.best_normal_offset = selected.normal_offset;
    row.best_tangential_offset = selected.tangential_offset;
    row.best_candidate_side = selected.side_id;
    row.best_heading_relative_to_table_deg = selected.heading_relative_to_table_deg;
    row.best_heading_relative_to_task_deg = selected.heading_relative_to_task_deg;
    row.best_left_reach_score = selected.left_reach_score;
    row.best_right_reach_score = selected.right_reach_score;
    row.best_combined_reach_score = selected.combined_reach_score;
    row.best_footprint_clearance_m = selected.footprint_clearance_m;
    row.nearest_production_distance_m = nearest_production.distance_m;
    row.nearest_production_side = nearest_production.side_id;
    row.delta_normal = selected.normal_offset - nearest_production.normal_offset;
    row.delta_tangential = selected.tangential_offset - nearest_production.tangential_offset;
    row.delta_theta_deg = rad2deg(wrapToPiLocal(selected.pose(3) - nearest_production.pose(3)));
    row.side_difference = selected.side_id ~= nearest_production.side_id;
    row.missing_geometry_class = classify_missing_geometry(row);
    row.infeasible_reason = classify_infeasible_reason(collision_valid_count, reason_counts);
    row.fail_backward_count = reason_counts.FAIL_BACKWARD;
    row.fail_lateral_count = reason_counts.FAIL_LATERAL;
    row.fail_radial_count = reason_counts.FAIL_RADIAL;
    row.fail_vertical_count = reason_counts.FAIL_VERTICAL;
    row.fail_score_threshold_count = reason_counts.FAIL_SCORE_THRESHOLD;
    row.pass_reachability_count = reason_counts.PASS;
end

function normal_values = local_normal_values(scene_cfg, map_info)
    step = max(map_info.resolution / 2, 0.025);
    margin = max(scene_cfg.docking_distance_step_m, 2 * map_info.resolution);
    d0 = max(0.05, scene_cfg.docking_min_distance_m - margin);
    d1 = scene_cfg.docking_max_distance_m + margin;
    normal_values = d0:step:d1;
end

function tangents = local_tangent_values(table, map_info, side_id, options)
    step = max(map_info.resolution / 2, options.tangent_step_m);
    margin = max(options.diagnostic_tangent_margin_m, 2 * map_info.resolution);
    if side_id <= 2
        half_span = table.length / 2 + margin;
    else
        half_span = table.width / 2 + margin;
    end
    tangents = -half_span:step:half_span;
end

function local = side_local_position(side_id, tangent, d, table)
    switch side_id
        case 1
            local = [tangent; table.width / 2 + d];
        case 2
            local = [tangent; -table.width / 2 - d];
        case 3
            local = [-table.length / 2 - d; tangent];
        otherwise
            local = [table.length / 2 + d; tangent];
    end
end

function nearest = nearest_production_candidate(library_region, scene, robot_cfg, ...
        planner_cfg, task, table_center, table_theta)
    nearest = initialize_pose_record();
    nearest.distance_m = inf;
    candidates = library_region.candidates;
    for ci = 1:numel(candidates)
        pose = candidates(ci).pose;
        d = hypot(task.x - pose(1), task.y - pose(2));
        if d < nearest.distance_m
            [ls, rs, cs] = evaluate_dual_arm_reachability( ...
                pose, [task.x, task.y, task.z], robot_cfg, planner_cfg.reachability.threshold);
            local = rot2(-table_theta) * (pose(1:2) - table_center)';
            tangent = local_tangent_from_side(local, candidates(ci).side_id);
            normal = normal_offset_from_side(local, candidates(ci).side_id, scene.scene_cfg.table);
            nearest = pose_record(pose, table_center, table_theta, [task.x, task.y], ...
                candidates(ci).side_id, tangent, normal, ls, rs, cs, scene, robot_cfg, d);
            nearest.distance_m = d;
        end
    end
end

function tangent = local_tangent_from_side(local, side_id)
    if side_id <= 2
        tangent = local(1);
    else
        tangent = local(2);
    end
end

function normal = normal_offset_from_side(local, side_id, table)
    switch side_id
        case 1
            normal = local(2) - table.width / 2;
        case 2
            normal = -local(2) - table.width / 2;
        case 3
            normal = -local(1) - table.length / 2;
        otherwise
            normal = local(1) - table.length / 2;
    end
end

function record = pose_record(pose, table_center, table_theta, target_xy, side_id, ...
        tangent, normal, ls, rs, cs, scene, robot_cfg, task_arm_distance)
    local = rot2(-table_theta) * (pose(1:2) - table_center)';
    record = initialize_pose_record();
    record.is_valid = all(isfinite(pose));
    record.pose = pose;
    record.table_local_x = local(1);
    record.table_local_y = local(2);
    record.normal_offset = normal;
    record.tangential_offset = tangent;
    record.side_id = side_id;
    record.heading_relative_to_table_deg = rad2deg(wrapToPiLocal(pose(3) - table_theta));
    theta_task = atan2(target_xy(2) - pose(2), target_xy(1) - pose(1));
    record.heading_relative_to_task_deg = rad2deg(wrapToPiLocal(pose(3) - theta_task));
    record.left_reach_score = ls;
    record.right_reach_score = rs;
    record.combined_reach_score = cs;
    record.footprint_clearance_m = footprint_clearance(pose, scene, robot_cfg);
    record.minimum_task_arm_distance_m = task_arm_distance;
end

function record = initialize_pose_record()
    record = struct('is_valid', false, 'pose', [NaN, NaN, NaN], ...
        'table_local_x', NaN, 'table_local_y', NaN, 'normal_offset', NaN, ...
        'tangential_offset', NaN, 'side_id', 0, ...
        'heading_relative_to_table_deg', NaN, ...
        'heading_relative_to_task_deg', NaN, ...
        'left_reach_score', -inf, 'right_reach_score', -inf, ...
        'combined_reach_score', -inf, 'footprint_clearance_m', NaN, ...
        'minimum_task_arm_distance_m', NaN, 'distance_m', NaN);
end

function clearance = footprint_clearance(pose, scene, robot_cfg)
    [occupied_i, occupied_j] = find(scene.grid_map ~= 0);
    if isempty(occupied_i)
        clearance = inf;
        return;
    end
    x = (occupied_j - 0.5) * scene.map_info.resolution;
    y = scene.map_info.height_m - (occupied_i - 0.5) * scene.map_info.resolution;
    center_dist = hypot(x - pose(1), y - pose(2));
    footprint_radius = hypot(robot_cfg.chassis.length, robot_cfg.chassis.width) / 2;
    clearance = max(0, min(center_dist) - footprint_radius);
end

function [left_diag, right_diag] = decompose_reachability(pose, target_xyz, robot_cfg, threshold)
    left_diag = single_arm_reachability_diag(pose, target_xyz, 'left', robot_cfg, threshold);
    right_diag = single_arm_reachability_diag(pose, target_xyz, 'right', robot_cfg, threshold);
end

function diag = single_arm_reachability_diag(pose, target_xyz, arm_side, robot_cfg, threshold)
    if strcmp(arm_side, 'left')
        base_offset = [robot_cfg.arm_base_mount_x, robot_cfg.dual_arm.base_spacing / 2];
    else
        base_offset = [robot_cfg.arm_base_mount_x, -robot_cfg.dual_arm.base_spacing / 2];
    end

    arm_base = [pose(1); pose(2)] + rot2(pose(3)) * base_offset';
    dz = target_xyz(3) - robot_cfg.arm_base_mount_z;
    delta = [target_xyz(1) - arm_base(1); target_xyz(2) - arm_base(2)];
    d_horizontal = hypot(delta(1), delta(2));
    local_dir = rot2(-pose(3)) * delta;
    diag.task_arm_distance = d_horizontal;

    if d_horizontal < 0.01 || d_horizontal > robot_cfg.single_arm.reach_radius
        diag.reason = 'FAIL_RADIAL';
        return;
    end
    if dz < robot_cfg.single_arm.reach_z_min || dz > robot_cfg.single_arm.reach_z_max
        diag.reason = 'FAIL_VERTICAL';
        return;
    end
    if local_dir(1) < -robot_cfg.single_arm.reach_backward
        diag.reason = 'FAIL_BACKWARD';
        return;
    end
    if abs(local_dir(2)) > robot_cfg.single_arm.reach_lateral * 1.1
        diag.reason = 'FAIL_LATERAL';
        return;
    end

    score = evaluate_single_arm_reachability(pose, target_xyz, arm_side, robot_cfg, threshold);
    if score > 0
        diag.reason = 'PASS';
    else
        diag.reason = 'FAIL_SCORE_THRESHOLD';
    end
end

function reason_counts = initialize_reason_counts()
    reason_counts = struct('FAIL_BACKWARD', 0, 'FAIL_LATERAL', 0, ...
        'FAIL_RADIAL', 0, 'FAIL_VERTICAL', 0, ...
        'FAIL_SCORE_THRESHOLD', 0, 'FAIL_OTHER', 0, 'PASS', 0);
end

function reason_counts = add_reason(reason_counts, reason)
    if isfield(reason_counts, reason)
        reason_counts.(reason) = reason_counts.(reason) + 1;
    else
        reason_counts.FAIL_OTHER = reason_counts.FAIL_OTHER + 1;
    end
end

function cls = classify_missing_geometry(row)
    if ~strcmp(row.oracle_class, 'ORACLE_FEASIBLE')
        cls = 'N_A';
    elseif row.side_difference
        cls = 'MISSING_SIDE';
    elseif abs(row.delta_normal) > 0.05 && abs(row.delta_theta_deg) > 10
        cls = 'POSITION_HEADING_INTERACTION';
    elseif abs(row.delta_normal) > 0.05
        cls = 'MISSING_NORMAL_OFFSET';
    elseif abs(row.delta_tangential) > 0.05
        cls = 'MISSING_TANGENTIAL_ALIGNMENT';
    elseif abs(row.delta_theta_deg) > 10
        cls = 'MISSING_HEADING';
    else
        cls = 'OTHER';
    end
end

function reason = classify_infeasible_reason(collision_valid_count, counts)
    if collision_valid_count == 0
        reason = 'NO_COLLISION_VALID_POSE';
        return;
    end
    names = {'FAIL_BACKWARD','FAIL_LATERAL','FAIL_RADIAL','FAIL_VERTICAL', ...
        'FAIL_SCORE_THRESHOLD','FAIL_OTHER'};
    values = zeros(1, numel(names));
    for i = 1:numel(names), values(i) = counts.(names{i}); end
    [~, idx] = max(values);
    reason = names{idx};
    if values(idx) == 0
        reason = 'COLLISION_VALID_BUT_MODEL_UNREACHABLE';
    end
end

function aggregate = summarize_oracle(aggregate, task_table)
    if isempty(task_table)
        aggregate.oracle_feasible = 0;
        aggregate.oracle_infeasible = 0;
        aggregate.feasible_ratio = 0;
        aggregate.primary_bottleneck = 'NONE';
        aggregate.main_rejection_condition = 'NONE';
        return;
    end
    feasible = strcmp(task_table.oracle_class, 'ORACLE_FEASIBLE');
    aggregate.oracle_feasible = sum(feasible);
    aggregate.oracle_infeasible = sum(~feasible);
    aggregate.feasible_ratio = aggregate.oracle_feasible / max(height(task_table), 1);
    if aggregate.feasible_ratio >= 0.80
        aggregate.primary_bottleneck = 'CANDIDATE_GENERATION_IS_PRIMARY_BOTTLENECK';
    elseif aggregate.feasible_ratio <= 0.20
        aggregate.primary_bottleneck = 'REACHABILITY_MODEL_IS_PRIMARY_BOTTLENECK';
    else
        aggregate.primary_bottleneck = 'MIXED_BOTTLENECK';
    end
    aggregate.main_rejection_condition = majority_class(task_table.infeasible_reason(~feasible));
    aggregate.collision_valid_oracle_poses = sum(task_table.collision_valid_oracle_poses);
    aggregate.max_combined_reach_score = max(task_table.max_combined_reach_score);
    aggregate.minimum_task_arm_distance_m = min(task_table.minimum_task_arm_distance_m);
end

function label = majority_class(labels)
    if isempty(labels)
        label = 'NONE';
        return;
    end
    [u, ~, idx] = unique(labels);
    counts = accumarray(idx(:), 1);
    [~, best] = max(counts);
    label = u{best};
end

function idx = find_task_region(regions, task_id)
    idx = find(arrayfun(@(r) ismember(task_id, r.task_ids), regions), 1);
end

function tasks = library_covered_tasks(library)
    tasks = [];
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            tasks = union(tasks, library.regions(ri).candidates(ci).covered_tasks, 'stable');
        end
    end
end

function scene_cfg = make_scene_config(case_id, seed)
    if startsWith(upper(case_id), 'F')
        scene_cfg = scene_complexity_factor_config(case_id, seed);
    else
        scene_cfg = scene_complexity_profile_config(case_id, seed);
    end
end

function aggregate = initialize_aggregate(case_id, seed)
    aggregate = struct('case_id', case_id, 'seed', seed, 'task_count', 0, ...
        'baseline_candidate_count', 0, 'baseline_coverage_percent', NaN, ...
        'uncovered_tasks', 0, 'oracle_feasible', 0, 'oracle_infeasible', 0, ...
        'feasible_ratio', NaN, 'primary_bottleneck', '', ...
        'main_rejection_condition', '', 'collision_valid_oracle_poses', 0, ...
        'max_combined_reach_score', NaN, 'minimum_task_arm_distance_m', NaN);
end

function row = empty_row()
    row = struct('case_id', '', 'seed', 0, 'task_id', 0, 'table_id', 0, ...
        'table_orientation_deg', NaN, 'task_local_x', NaN, 'task_local_y', NaN, ...
        'oracle_class', '', 'collision_valid_oracle_poses', 0, ...
        'sampled_oracle_poses', 0, 'max_left_reach_score', 0, ...
        'max_right_reach_score', 0, 'max_combined_reach_score', 0, ...
        'minimum_task_arm_distance_m', NaN, 'best_x', NaN, 'best_y', NaN, ...
        'best_theta', NaN, 'best_table_local_x', NaN, ...
        'best_table_local_y', NaN, 'best_normal_offset', NaN, ...
        'best_tangential_offset', NaN, 'best_candidate_side', 0, ...
        'best_heading_relative_to_table_deg', NaN, ...
        'best_heading_relative_to_task_deg', NaN, ...
        'best_left_reach_score', NaN, 'best_right_reach_score', NaN, ...
        'best_combined_reach_score', NaN, 'best_footprint_clearance_m', NaN, ...
        'nearest_production_distance_m', NaN, 'nearest_production_side', 0, ...
        'delta_normal', NaN, 'delta_tangential', NaN, 'delta_theta_deg', NaN, ...
        'side_difference', false, 'missing_geometry_class', '', ...
        'infeasible_reason', '', 'fail_backward_count', 0, ...
        'fail_lateral_count', 0, 'fail_radial_count', 0, ...
        'fail_vertical_count', 0, 'fail_score_threshold_count', 0, ...
        'pass_reachability_count', 0);
end

function options = default_options(options)
    if ~isfield(options, 'case_id'), options.case_id = 'F010'; end
    if ~isfield(options, 'seed'), options.seed = 2026; end
    if ~isfield(options, 'heading_offsets_deg')
        options.heading_offsets_deg = [0, 15, -15, 30, -30, 45, -45, 60, -60];
    end
    if ~isfield(options, 'tangent_step_m'), options.tangent_step_m = 0.025; end
    if ~isfield(options, 'diagnostic_tangent_margin_m')
        options.diagnostic_tangent_margin_m = 0.10;
    end
    if ~isfield(options, 'max_uncovered_tasks')
        options.max_uncovered_tasks = inf;
    end
end

function ensure_result_dir()
    out_dir = fullfile('results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
end
