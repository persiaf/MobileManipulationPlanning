function outputs = exp_radial_operability_audit(options)
%EXP_RADIAL_OPERABILITY_AUDIT P1.8 radial/operational feasibility audit.
%
% Diagnostic only: no reachability threshold, collision margin, robot
% geometry, physical profile, planner, or candidate-library semantics change.

    if nargin < 1 || isempty(options), options = struct(); end
    options = default_options(options);

    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    scene_cfg = scene_complexity_factor_config(options.case_id, options.seed);
    scene = build_scene_from_cfg(scene_cfg);
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    robot_cfg = robot_geometry_profile('legacy_validated');
    planner_cfg = planner_config();
    planner_cfg.candidate_augmentation.enable = false;
    planner_cfg.runtime.disable_visualization = true;
    planner_cfg.runtime.disable_video = true;
    planner_cfg.benchmark.enable = false;

    baseline_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    covered = library_covered_tasks(baseline_library);
    uncovered = setdiff(1:scene.num_tasks, covered, 'stable');
    oracle_rows = load_oracle_rows(project_root, options, uncovered);
    target_tasks = oracle_rows.task_id(strcmp(oracle_rows.oracle_class, 'ORACLE_INFEASIBLE') & ...
        strcmp(oracle_rows.infeasible_reason, 'FAIL_RADIAL'));

    if isfinite(options.max_tasks)
        target_tasks = target_tasks(1:min(numel(target_tasks), options.max_tasks));
    end

    rows = repmat(empty_row(), 0, 1);
    for i = 1:numel(target_tasks)
        task_id = target_tasks(i);
        region_idx = find_task_region(regions, task_id);
        rows(end + 1, 1) = audit_task(scene, regions(region_idx), task_id, ...
            robot_cfg, planner_cfg, options); %#ok<AGROW>
    end

    task_table = struct2table(rows, 'AsArray', true);
    summary = summarize_task_rows(task_table, options.case_id, options.seed, ...
        scene.num_tasks, baseline_library.total_candidates, ...
        100 * numel(covered) / max(scene.num_tasks, 1), numel(uncovered));

    outputs.summary = struct2table(summary, 'AsArray', true);
    outputs.task = task_table;

    if options.write_outputs
        ensure_result_dir();
        writetable(outputs.summary, fullfile(project_root, 'results', 'tables', ...
            'radial_operability_audit_summary.csv'));
        writetable(outputs.task, fullfile(project_root, 'results', 'tables', ...
            'radial_operability_audit_task.csv'));
    end
end

function row = audit_task(scene, region, task_id, robot_cfg, planner_cfg, options)
    task = scene.tasks(task_id);
    full_result = radial_counterfactual_search(scene, task, robot_cfg, planner_cfg, options);
    isolated_scene = make_isolated_table_scene(scene, task.table_id, task);
    isolated_result = standard_oracle_search(isolated_scene, isolated_scene.tasks(1), ...
        robot_cfg, planner_cfg, options);

    row = empty_row();
    row.case_id = options.case_id;
    row.seed = options.seed;
    row.task_id = task_id;
    row.table_id = task.table_id;
    row.table_orientation_deg = rad2deg(scene.scene_cfg.table_orientations(task.table_id));
    local = rot2(-scene.scene_cfg.table_orientations(task.table_id)) * ...
        ([task.x, task.y] - scene.scene_cfg.table_positions(task.table_id, :))';
    row.task_local_x = local(1);
    row.task_local_y = local(2);
    row.task_edge_distance_m = task_edge_distance(local, scene.scene_cfg.table);
    row.radial_only_blocked = full_result.radial_only_pass_count > 0;
    row.multi_constraint_blocked = ~row.radial_only_blocked;
    row.collision_valid_full_scene = full_result.collision_valid_count;
    row.non_radial_pass_pose_count = full_result.radial_only_pass_count;
    row.R_required = full_result.min_required_radial;
    row.R_limit = robot_cfg.single_arm.reach_radius;
    row.radial_excess = row.R_required - row.R_limit;
    row.excess_class = classify_excess(row.radial_excess, scene.map_info, row.R_limit);
    row.full_scene_max_combined_reach_score = full_result.max_combined_score;
    row.full_scene_main_nonradial_failure = full_result.main_nonradial_failure;
    row.isolated_table_feasible = isolated_result.feasible;
    row.isolated_table_collision_valid = isolated_result.collision_valid_count;
    row.isolated_table_max_combined_reach_score = isolated_result.max_combined_score;
    row.isolated_table_main_failure = isolated_result.main_failure;
    row.final_classification = classify_final(row);
    row.model_feasible = row.isolated_table_feasible;
    row.candidate_generation_residual = row.radial_only_blocked && ~row.isolated_table_feasible;
end

function result = radial_counterfactual_search(scene, task, robot_cfg, planner_cfg, options)
    samples = local_pose_samples(scene, task, options);
    result = initialize_search_result();
    reason_counts = initialize_reason_counts();
    min_required = inf;

    for i = 1:size(samples, 1)
        pose = samples(i, 1:3);
        if check_footprint_collision(pose, scene.grid_map, scene.map_info, robot_cfg)
            continue;
        end
        result.collision_valid_count = result.collision_valid_count + 1;
        [ls, rs, cs] = evaluate_dual_arm_reachability( ...
            pose, [task.x, task.y, task.z], robot_cfg, planner_cfg.reachability.threshold);
        result.max_combined_score = max(result.max_combined_score, cs);

        left = arm_diag_ignore_radial(pose, [task.x, task.y, task.z], 'left', ...
            robot_cfg, planner_cfg.reachability.threshold);
        right = arm_diag_ignore_radial(pose, [task.x, task.y, task.z], 'right', ...
            robot_cfg, planner_cfg.reachability.threshold);
        reason_counts = add_reason(reason_counts, left.reason);
        reason_counts = add_reason(reason_counts, right.reason);
        if left.nonradial_pass
            min_required = min(min_required, left.radial_distance);
            result.radial_only_pass_count = result.radial_only_pass_count + 1;
        end
        if right.nonradial_pass
            min_required = min(min_required, right.radial_distance);
            result.radial_only_pass_count = result.radial_only_pass_count + 1;
        end
    end

    if isfinite(min_required)
        result.min_required_radial = min_required;
    end
    result.main_nonradial_failure = majority_reason(reason_counts);
end

function result = standard_oracle_search(scene, task, robot_cfg, planner_cfg, options)
    samples = local_pose_samples(scene, task, options);
    result = initialize_search_result();
    reason_counts = initialize_reason_counts();

    for i = 1:size(samples, 1)
        pose = samples(i, 1:3);
        if check_footprint_collision(pose, scene.grid_map, scene.map_info, robot_cfg)
            continue;
        end
        result.collision_valid_count = result.collision_valid_count + 1;
        [ls, rs, cs] = evaluate_dual_arm_reachability( ...
            pose, [task.x, task.y, task.z], robot_cfg, planner_cfg.reachability.threshold);
        result.max_combined_score = max(result.max_combined_score, cs);
        if cs > 0
            result.feasible = true;
            return;
        end
        left = arm_diag_standard(pose, [task.x, task.y, task.z], 'left', ...
            robot_cfg, planner_cfg.reachability.threshold);
        right = arm_diag_standard(pose, [task.x, task.y, task.z], 'right', ...
            robot_cfg, planner_cfg.reachability.threshold);
        reason_counts = add_reason(reason_counts, left.reason);
        reason_counts = add_reason(reason_counts, right.reason);
    end
    result.main_failure = majority_reason(reason_counts);
end

function samples = local_pose_samples(scene, task, options)
    table_id = task.table_id;
    center = scene.scene_cfg.table_positions(table_id, :);
    table_theta = scene.scene_cfg.table_orientations(table_id);
    table = scene.scene_cfg.table;
    normal_values = local_normal_values(scene.scene_cfg, scene.map_info);
    heading_offsets = deg2rad(options.heading_offsets_deg);
    samples = zeros(0, 4);
    for side_id = 1:4
        tangents = local_tangent_values(table, scene.map_info, side_id, options);
        for ni = 1:numel(normal_values)
            d = normal_values(ni);
            for ti = 1:numel(tangents)
                tangent = tangents(ti);
                local_xy = side_local_position(side_id, tangent, d, table);
                world_xy = center(:) + rot2(table_theta) * local_xy;
                theta_face = atan2(task.y - world_xy(2), task.x - world_xy(1));
                for hi = 1:numel(heading_offsets)
                    samples(end + 1, :) = [world_xy', ...
                        wrapToPiLocal(theta_face + heading_offsets(hi)), side_id]; %#ok<AGROW>
                end
            end
        end
    end
end

function scene_iso = make_isolated_table_scene(scene, table_id, task)
    cfg = scene.scene_cfg;
    original_table_id = table_id;
    cfg.table_mode = 'custom';
    cfg.table_positions = scene.scene_cfg.table_positions(original_table_id, :);
    cfg.table_orientations = scene.scene_cfg.table_orientations(original_table_id);
    cfg.num_tables = 1;
    cfg.manual_tasks = [1, task.x, task.y, task.z];
    cfg.task_mode = 'manual';
    cfg.layout = rmfield_if_present(cfg.layout, {'num_tables','rows','cols','pattern'});
    scene_iso = build_scene_from_cfg(cfg);
end

function s = rmfield_if_present(s, names)
    for i = 1:numel(names)
        if isfield(s, names{i}), s = rmfield(s, names{i}); end
    end
end

function values = local_normal_values(scene_cfg, map_info)
    step = max(map_info.resolution / 2, 0.025);
    margin = max(scene_cfg.docking_distance_step_m, 2 * map_info.resolution);
    d0 = max(0.05, scene_cfg.docking_min_distance_m - margin);
    d1 = scene_cfg.docking_max_distance_m + margin;
    values = d0:step:d1;
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

function diag = arm_diag_ignore_radial(pose, target_xyz, arm_side, robot_cfg, threshold)
    diag = base_arm_diag(pose, target_xyz, arm_side, robot_cfg);
    diag.nonradial_pass = false;
    if diag.dz < robot_cfg.single_arm.reach_z_min || diag.dz > robot_cfg.single_arm.reach_z_max
        diag.reason = 'FAIL_VERTICAL';
        return;
    end
    if diag.local_dir(1) < -robot_cfg.single_arm.reach_backward
        diag.reason = 'FAIL_BACKWARD';
        return;
    end
    if abs(diag.local_dir(2)) > robot_cfg.single_arm.reach_lateral * 1.1
        diag.reason = 'FAIL_LATERAL';
        return;
    end
    pseudo_cfg = robot_cfg;
    pseudo_cfg.single_arm.reach_radius = max(robot_cfg.single_arm.reach_radius, diag.radial_distance + eps);
    score = evaluate_score_without_radial_gate(pose, target_xyz, arm_side, pseudo_cfg, threshold);
    if score > 0
        diag.nonradial_pass = true;
        diag.reason = 'RADIAL_ONLY_BLOCKED';
    else
        diag.reason = 'FAIL_SCORE_THRESHOLD';
    end
end

function diag = arm_diag_standard(pose, target_xyz, arm_side, robot_cfg, threshold)
    diag = base_arm_diag(pose, target_xyz, arm_side, robot_cfg);
    if diag.radial_distance < 0.01 || diag.radial_distance > robot_cfg.single_arm.reach_radius
        diag.reason = 'FAIL_RADIAL';
        return;
    end
    if diag.dz < robot_cfg.single_arm.reach_z_min || diag.dz > robot_cfg.single_arm.reach_z_max
        diag.reason = 'FAIL_VERTICAL';
        return;
    end
    if diag.local_dir(1) < -robot_cfg.single_arm.reach_backward
        diag.reason = 'FAIL_BACKWARD';
        return;
    end
    if abs(diag.local_dir(2)) > robot_cfg.single_arm.reach_lateral * 1.1
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

function diag = base_arm_diag(pose, target_xyz, arm_side, robot_cfg)
    if strcmp(arm_side, 'left')
        base_offset = [robot_cfg.arm_base_mount_x, robot_cfg.dual_arm.base_spacing / 2];
    else
        base_offset = [robot_cfg.arm_base_mount_x, -robot_cfg.dual_arm.base_spacing / 2];
    end
    arm_base = [pose(1); pose(2)] + rot2(pose(3)) * base_offset';
    delta = [target_xyz(1) - arm_base(1); target_xyz(2) - arm_base(2)];
    diag.radial_distance = hypot(delta(1), delta(2));
    diag.dz = target_xyz(3) - robot_cfg.arm_base_mount_z;
    diag.local_dir = rot2(-pose(3)) * delta;
end

function score = evaluate_score_without_radial_gate(pose, target_xyz, arm_side, robot_cfg, threshold)
    score = evaluate_single_arm_reachability(pose, target_xyz, arm_side, robot_cfg, threshold);
end

function d = task_edge_distance(local, table)
    d = min([table.length / 2 - abs(local(1)), table.width / 2 - abs(local(2))]);
end

function cls = classify_excess(excess, map_info, R_limit)
    small = max(map_info.resolution, 0.05 * R_limit);
    moderate = max(3 * map_info.resolution, 0.15 * R_limit);
    if ~isfinite(excess)
        cls = 'NO_NONRADIAL_PASS';
    elseif excess <= small
        cls = 'SMALL_EXCESS';
    elseif excess <= moderate
        cls = 'MODERATE_EXCESS';
    else
        cls = 'LARGE_EXCESS';
    end
end

function cls = classify_final(row)
    if row.isolated_table_feasible
        cls = 'ENVIRONMENT_CLEARANCE_LIMIT';
    elseif ~row.isolated_table_feasible && row.radial_only_blocked
        cls = 'TASK_GEOMETRY_REACH_LIMIT';
    elseif ~row.isolated_table_feasible
        cls = 'TASK_GEOMETRY_REACH_LIMIT';
    else
        cls = 'MIXED_OR_UNRESOLVED';
    end
end

function result = initialize_search_result()
    result = struct('collision_valid_count', 0, 'radial_only_pass_count', 0, ...
        'min_required_radial', NaN, 'max_combined_score', 0, ...
        'main_nonradial_failure', 'NONE', 'feasible', false, ...
        'main_failure', 'NONE');
end

function counts = initialize_reason_counts()
    counts = struct('FAIL_RADIAL', 0, 'FAIL_VERTICAL', 0, 'FAIL_BACKWARD', 0, ...
        'FAIL_LATERAL', 0, 'FAIL_SCORE_THRESHOLD', 0, ...
        'RADIAL_ONLY_BLOCKED', 0, 'PASS', 0, 'FAIL_OTHER', 0);
end

function counts = add_reason(counts, reason)
    if isfield(counts, reason)
        counts.(reason) = counts.(reason) + 1;
    else
        counts.FAIL_OTHER = counts.FAIL_OTHER + 1;
    end
end

function label = majority_reason(counts)
    names = fieldnames(counts);
    values = zeros(numel(names), 1);
    for i = 1:numel(names), values(i) = counts.(names{i}); end
    [~, idx] = max(values);
    label = names{idx};
end

function oracle_rows = load_oracle_rows(project_root, options, uncovered)
    task_path = fullfile(project_root, 'results', 'tables', 'local_reachability_oracle_task.csv');
    if isfile(task_path)
        oracle_rows = readtable(task_path);
        if all(ismember({'task_id','oracle_class','infeasible_reason'}, oracle_rows.Properties.VariableNames))
            return;
        end
    end
    warning('exp_radial_operability_audit:MissingOracleTaskCsv', ...
        'Using uncovered tasks from rebuilt baseline because oracle task CSV is missing.');
    oracle_rows = table(uncovered(:), repmat({'ORACLE_INFEASIBLE'}, numel(uncovered), 1), ...
        repmat({'FAIL_RADIAL'}, numel(uncovered), 1), ...
        'VariableNames', {'task_id', 'oracle_class', 'infeasible_reason'});
end

function summary = summarize_task_rows(task_table, case_id, seed, task_count, ...
        baseline_candidate_count, baseline_coverage, baseline_uncovered)
    summary = struct('case_id', case_id, 'seed', seed, 'task_count', task_count, ...
        'baseline_candidate_count', baseline_candidate_count, ...
        'baseline_coverage_percent', baseline_coverage, ...
        'baseline_uncovered_tasks', baseline_uncovered, ...
        'audited_fail_radial_tasks', height(task_table), ...
        'radial_only_blocked_count', 0, 'multi_constraint_blocked_count', 0, ...
        'R_excess_mean', NaN, 'R_excess_median', NaN, 'R_excess_min', NaN, ...
        'R_excess_max', NaN, 'isolated_table_feasible_count', 0, ...
        'isolated_table_infeasible_count', 0, ...
        'environment_clearance_limited_count', 0, ...
        'task_geometry_reach_limited_count', 0, ...
        'model_radial_overconservative_count', 0, ...
        'mixed_or_unresolved_count', 0, 'model_feasible_task_count', 0, ...
        'feasible_task_coverage_percent', NaN, ...
        'candidate_generation_residual_count', 0, ...
        'main_rejection_condition', 'NONE', 'selected_action', 'none', ...
        'p2_gate', 'ONE_REACHABILITY_FIX_REQUIRED');
    if isempty(task_table), return; end

    summary.radial_only_blocked_count = sum(task_table.radial_only_blocked);
    summary.multi_constraint_blocked_count = sum(task_table.multi_constraint_blocked);
    excess = task_table.radial_excess(isfinite(task_table.radial_excess));
    if ~isempty(excess)
        summary.R_excess_mean = mean(excess);
        summary.R_excess_median = median(excess);
        summary.R_excess_min = min(excess);
        summary.R_excess_max = max(excess);
    end
    summary.isolated_table_feasible_count = sum(task_table.isolated_table_feasible);
    summary.isolated_table_infeasible_count = sum(~task_table.isolated_table_feasible);
    summary.environment_clearance_limited_count = ...
        sum(strcmp(task_table.final_classification, 'ENVIRONMENT_CLEARANCE_LIMIT'));
    summary.task_geometry_reach_limited_count = ...
        sum(strcmp(task_table.final_classification, 'TASK_GEOMETRY_REACH_LIMIT'));
    summary.model_radial_overconservative_count = ...
        sum(strcmp(task_table.final_classification, 'MODEL_RADIAL_OVERCONSERVATIVE'));
    summary.mixed_or_unresolved_count = ...
        sum(strcmp(task_table.final_classification, 'MIXED_OR_UNRESOLVED'));
    summary.model_feasible_task_count = task_count - summary.task_geometry_reach_limited_count;
    covered_model_feasible = summary.model_feasible_task_count - ...
        (baseline_uncovered - summary.task_geometry_reach_limited_count);
    summary.feasible_task_coverage_percent = ...
        100 * covered_model_feasible / max(summary.model_feasible_task_count, 1);
    summary.candidate_generation_residual_count = ...
        baseline_uncovered - summary.task_geometry_reach_limited_count - ...
        summary.environment_clearance_limited_count;
    summary.main_rejection_condition = 'FAIL_RADIAL';
    if summary.model_feasible_task_count > 0 && ...
            summary.candidate_generation_residual_count <= max(4, 0.10 * summary.model_feasible_task_count)
        summary.selected_action = 'operability accounting';
        summary.p2_gate = 'READY_FOR_P2';
    end
end

function row = empty_row()
    row = struct('case_id', '', 'seed', 0, 'task_id', 0, 'table_id', 0, ...
        'table_orientation_deg', NaN, 'task_local_x', NaN, 'task_local_y', NaN, ...
        'task_edge_distance_m', NaN, 'radial_only_blocked', false, ...
        'multi_constraint_blocked', false, 'collision_valid_full_scene', 0, ...
        'non_radial_pass_pose_count', 0, 'R_required', NaN, ...
        'R_limit', NaN, 'radial_excess', NaN, 'excess_class', '', ...
        'full_scene_max_combined_reach_score', 0, ...
        'full_scene_main_nonradial_failure', '', ...
        'isolated_table_feasible', false, 'isolated_table_collision_valid', 0, ...
        'isolated_table_max_combined_reach_score', 0, ...
        'isolated_table_main_failure', '', 'final_classification', '', ...
        'model_feasible', false, 'candidate_generation_residual', false);
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
    if ~isfield(options, 'max_tasks'), options.max_tasks = inf; end
    if ~isfield(options, 'write_outputs'), options.write_outputs = true; end
end

function ensure_result_dir()
    out_dir = fullfile('results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
end
