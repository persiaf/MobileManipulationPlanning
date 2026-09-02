function outputs = exp_orientation_fast_validation(options)
%EXP_ORIENTATION_FAST_VALIDATION Minimal P1.6-Fast orientation diagnosis.
%
% This experiment intentionally stays at the candidate layer. It does not
% run the full Top-Down/Bidirectional matrix, does not alter robot geometry,
% reachability thresholds, or collision margins, and does not expand the map.

    if nargin < 1 || isempty(options), options = struct(); end
    options = default_options(options);
    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    summary_rows = repmat(empty_summary_row(), 0, 1);
    trace_rows = repmat(empty_trace_row(), 0, 1);

    for seed = options.orientation_sweep_seeds
        for ai = 1:numel(options.orientation_sweep_deg)
            angle_deg = options.orientation_sweep_deg(ai);
            scene_cfg = fixed_orientation_scene(seed, angle_deg);
            [row, trace] = evaluate_case('orientation_sweep', ...
                sprintf('fixed_%+ddeg', angle_deg), seed, 'baseline', ...
                scene_cfg, false, options);
            row.angle_deg = angle_deg;
            summary_rows(end + 1, 1) = row; %#ok<AGROW>
            trace_rows = append_trace(trace_rows, trace); %#ok<AGROW>
        end
    end

    if options.run_validation_matrix
        for ci = 1:numel(options.validation_cases)
            case_id = options.validation_cases{ci};
            for seed = options.validation_seeds
                scene_cfg = validation_scene(case_id, seed);
                for vi = 1:numel(options.validation_variants)
                    variant = options.validation_variants{vi};
                    include_trace = should_trace_validation_case(case_id, seed, variant, options);
                    [row, trace] = evaluate_case('validation', case_id, seed, ...
                        variant, scene_cfg, include_trace, options);
                    summary_rows(end + 1, 1) = row; %#ok<AGROW>
                    trace_rows = append_trace(trace_rows, trace); %#ok<AGROW>
                end
            end
        end
    else
        scene_cfg = validation_scene(options.trace_case_id, options.trace_seed);
        [row, trace] = evaluate_case('reachability_probe', options.trace_case_id, ...
            options.trace_seed, 'baseline', scene_cfg, true, options);
        summary_rows(end + 1, 1) = row; %#ok<AGROW>
        trace_rows = append_trace(trace_rows, trace); %#ok<AGROW>
    end

    outputs.summary = struct2table(summary_rows, 'AsArray', true);
    outputs.trace = struct2table(trace_rows, 'AsArray', true);
    ensure_result_dir();
    writetable(outputs.summary, fullfile(project_root, 'results', 'tables', ...
        'orientation_fast_validation_summary.csv'));
    writetable(outputs.trace, fullfile(project_root, 'results', 'tables', ...
        'orientation_fast_validation_raw.csv'));
end

function [row, trace_rows] = evaluate_case(record_type, case_id, seed, variant, scene_cfg, ...
        include_failure_trace, options)
    scene = build_scene_from_cfg(scene_cfg);
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    planner_cfg = configure_planner(variant);
    robot_cfg = robot_geometry_profile('legacy_validated');

    t_candidate = tic;
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    T_candidate = toc(t_candidate);
    covered = library_covered_tasks(library);
    uncovered = setdiff(1:scene.num_tasks, covered, 'stable');

    row = empty_summary_row();
    row.record_type = record_type;
    row.case_id = case_id;
    row.seed = seed;
    row.variant = variant;
    row.layout_pattern = scene.scene_cfg.layout.pattern;
    row.orientation_mode = scene.scene_cfg.layout.orientation_mode;
    row.task_position_mode = scene.scene_cfg.task.position_mode;
    row.table_count = size(scene.scene_cfg.table_positions, 1);
    row.task_count = scene.num_tasks;
    row.coverage_percent = 100 * numel(covered) / max(scene.num_tasks, 1);
    row.uncovered_tasks = numel(uncovered);
    row.baseline_candidate_count = getfield_default(library.augmentation_diagnostics, ...
        'baseline_candidate_count', library.total_candidates);
    row.position_only_added = 0;
    if strcmpi(variant, 'position_only')
        row.position_only_added = getfield_default(library.augmentation_diagnostics, ...
            'selected_supplementary_count', 0);
    end
    row.orientation_aware_added = 0;
    row.final_candidate_count = library.total_candidates;
    row.added_candidate_ratio = ...
        (row.final_candidate_count - row.baseline_candidate_count) / max(row.baseline_candidate_count, 1);
    row.T_candidate = T_candidate;

    trace_rows = repmat(empty_trace_row(), 0, 1);
    if ~include_failure_trace || isempty(uncovered) || ~strcmpi(variant, 'baseline')
        row.failure_majority_class = 'NONE';
        return;
    end

    for ui = 1:numel(uncovered)
        trace_rows(end + 1, 1) = diagnose_uncovered_task( ...
            record_type, case_id, seed, scene, regions, robot_cfg, planner_cfg, uncovered(ui), options); %#ok<AGROW>
    end
    row.failure_majority_class = majority_class({trace_rows.failure_type});
end

function row = diagnose_uncovered_task(record_type, case_id, seed, scene, regions, ...
        robot_cfg, planner_cfg, task_id, options)
    task = scene.tasks(task_id);
    table_id = task.table_id;
    table_center = scene.scene_cfg.table_positions(table_id, :);
    table_theta = scene.scene_cfg.table_orientations(table_id);
    region_idx = find(arrayfun(@(r) ismember(task_id, r.task_ids), regions), 1);
    region = regions(region_idx);
    task_positions = region_task_positions(region, scene);
    reach_field = compute_reachability_field(table_center, table_theta, scene.scene_cfg, ...
        robot_cfg, scene.map_info, mean(task_positions, 1), task_positions);

    best = nearest_collision_valid_candidate(reach_field, scene, robot_cfg, task);
    probe = local_repair_probe(task, scene, robot_cfg, planner_cfg, options);

    row = empty_trace_row();
    row.record_type = record_type;
    row.case_id = case_id;
    row.seed = seed;
    row.task_id = task_id;
    row.table_id = table_id;
    row.table_orientation_deg = rad2deg(table_theta);
    row.candidate_side = best.side_id;
    row.candidate_table_relative_heading_deg = ...
        rad2deg(wrapToPiLocal(best.pose(3) - table_theta));
    row.candidate_task_relative_heading_deg = ...
        rad2deg(wrapToPiLocal(best.pose(3) - atan2(task.y - best.pose(2), task.x - best.pose(1))));
    row.nearest_collision_valid_candidate_distance_m = best.distance_m;
    row.left_reach_score = best.left_score;
    row.right_reach_score = best.right_score;
    row.combined_reach_score = best.combined_score;
    row.local_probe_collision_valid_count = probe.collision_valid_count;
    row.local_probe_best_reach_score = probe.best_reach_score;
    row.local_probe_recovered = probe.recovered;
    row.failure_type = classify_failure(best, probe);
end

function probe = local_repair_probe(task, scene, robot_cfg, planner_cfg, options)
    center = scene.scene_cfg.table_positions(task.table_id, :);
    table_theta = scene.scene_cfg.table_orientations(task.table_id);
    table = scene.scene_cfg.table;
    target = [task.x, task.y];
    target_local = rot2(-table_theta) * (target - center)';
    distances = scene.scene_cfg.docking_min_distance_m : ...
        scene.scene_cfg.docking_distance_step_m : scene.scene_cfg.docking_max_distance_m;
    tangential_offsets = options.local_probe_tangential_offsets_m;
    heading_offsets = deg2rad(options.heading_shortlist_deg);

    probe = struct('collision_valid_count', 0, 'best_reach_score', 0, 'recovered', false);
    for side_id = 1:4
        if side_id <= 2
            tangent0 = target_local(1);
        else
            tangent0 = target_local(2);
        end
        for d = distances
            for tangent = tangent0 + tangential_offsets
                local = side_local_position(side_id, tangent, d, table);
                world = center(:) + rot2(table_theta) * local;
                theta_face = atan2(task.y - world(2), task.x - world(1));
                for hi = 1:numel(heading_offsets)
                    pose = [world', wrapToPiLocal(theta_face + heading_offsets(hi))];
                    if check_footprint_collision(pose, scene.grid_map, scene.map_info, robot_cfg)
                        continue;
                    end
                    probe.collision_valid_count = probe.collision_valid_count + 1;
                    [~, ~, cs] = evaluate_dual_arm_reachability( ...
                        pose, [task.x, task.y, task.z], robot_cfg, planner_cfg.reachability.threshold);
                    probe.best_reach_score = max(probe.best_reach_score, cs);
                    if cs > 0
                        probe.recovered = true;
                        return;
                    end
                end
            end
        end
    end
end

function best = nearest_collision_valid_candidate(reach_field, scene, robot_cfg, task)
    best = struct('pose', [NaN, NaN, NaN], 'side_id', 0, 'distance_m', NaN, ...
        'left_score', NaN, 'right_score', NaN, 'combined_score', NaN);
    poses = reach_field.candidates;
    sides = reach_field.side_ids;
    best_dist = inf;
    for ci = 1:size(poses, 1)
        pose = poses(ci, :);
        if check_footprint_collision(pose, scene.grid_map, scene.map_info, robot_cfg)
            continue;
        end
        d = hypot(task.x - pose(1), task.y - pose(2));
        if d < best_dist
            [ls, rs, cs] = evaluate_dual_arm_reachability( ...
                pose, [task.x, task.y, task.z], robot_cfg, 0.30);
            best.pose = pose;
            best.side_id = sides(ci);
            best.distance_m = d;
            best.left_score = ls;
            best.right_score = rs;
            best.combined_score = cs;
            best_dist = d;
        end
    end
end

function failure_type = classify_failure(best, probe)
    if probe.collision_valid_count == 0
        failure_type = 'POSITION_GAP';
    elseif probe.recovered
        failure_type = 'HEADING_GAP';
    elseif isfinite(best.combined_score) && best.combined_score <= 0 && probe.best_reach_score <= 0
        failure_type = 'REACHABILITY_MODEL_LIMIT';
    elseif best.side_id == 0
        failure_type = 'SIDE_SAMPLING_GAP';
    else
        failure_type = 'MIXED';
    end
end

function scene_cfg = fixed_orientation_scene(seed, angle_deg)
    scene_cfg = scene_complexity_profile_config('C0', seed);
    scene_cfg.layout.pattern = 'grid';
    scene_cfg.layout.orientation_mode = 'fixed';
    scene_cfg.layout.orientation_candidates_deg = angle_deg;
    scene_cfg.task.position_mode = 'discrete';
    scene_cfg.task.difficulty = 'normal';
end

function scene_cfg = validation_scene(case_id, seed)
    if startsWith(upper(case_id), 'F')
        scene_cfg = scene_complexity_factor_config(case_id, seed);
    else
        scene_cfg = scene_complexity_profile_config(case_id, seed);
    end
end

function tf = should_trace_validation_case(case_id, seed, variant, options)
    tf = strcmpi(variant, 'baseline') && ...
        strcmpi(case_id, options.trace_case_id) && ...
        seed == options.trace_seed;
end

function planner_cfg = configure_planner(variant)
    planner_cfg = planner_config();
    planner_cfg.runtime.disable_visualization = true;
    planner_cfg.runtime.disable_video = true;
    planner_cfg.runtime.quiet = true;
    planner_cfg.runtime.quiet_stage2 = true;
    planner_cfg.benchmark.enable = false;
    planner_cfg.visualization.show_scene = false;
    planner_cfg.visualization.show_docking = false;
    planner_cfg.visualization.show_global_path = false;
    planner_cfg.candidate_augmentation.enable = false;
    if strcmpi(variant, 'position_only')
        planner_cfg.candidate_augmentation.enable = true;
        planner_cfg.candidate_augmentation.variant = 'position_only';
    elseif ~strcmpi(variant, 'baseline')
        error('exp_orientation_fast_validation:UnknownVariant', ...
            'Unknown variant "%s".', variant);
    end
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

function out = append_trace(out, add)
    if ~isempty(add)
        out = [out; add(:)]; %#ok<AGROW>
    end
end

function tasks = library_covered_tasks(library)
    tasks = [];
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            tasks = union(tasks, library.regions(ri).candidates(ci).covered_tasks, 'stable');
        end
    end
end

function positions = region_task_positions(region, scene)
    positions = zeros(numel(region.task_ids), 2);
    for i = 1:numel(region.task_ids)
        tid = region.task_ids(i);
        positions(i, :) = [scene.tasks(tid).x, scene.tasks(tid).y];
    end
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

function row = empty_summary_row()
    row = struct('record_type', '', 'case_id', '', 'seed', 0, 'angle_deg', NaN, ...
        'variant', '', 'layout_pattern', '', 'orientation_mode', '', ...
        'task_position_mode', '', 'table_count', 0, 'task_count', 0, ...
        'coverage_percent', NaN, 'uncovered_tasks', 0, ...
        'baseline_candidate_count', 0, 'position_only_added', 0, ...
        'orientation_aware_added', 0, 'final_candidate_count', 0, ...
        'added_candidate_ratio', 0, 'failure_majority_class', '', ...
        'T_candidate', NaN);
end

function row = empty_trace_row()
    row = struct('record_type', '', 'case_id', '', 'seed', 0, 'task_id', 0, ...
        'table_id', 0, 'table_orientation_deg', NaN, 'candidate_side', 0, ...
        'candidate_table_relative_heading_deg', NaN, ...
        'candidate_task_relative_heading_deg', NaN, ...
        'nearest_collision_valid_candidate_distance_m', NaN, ...
        'left_reach_score', NaN, 'right_reach_score', NaN, ...
        'combined_reach_score', NaN, 'local_probe_collision_valid_count', 0, ...
        'local_probe_best_reach_score', NaN, 'local_probe_recovered', false, ...
        'failure_type', '');
end

function options = default_options(options)
    if ~isfield(options, 'orientation_sweep_deg')
        options.orientation_sweep_deg = [0, 45, -45, 90];
    end
    if ~isfield(options, 'orientation_sweep_seeds')
        options.orientation_sweep_seeds = [2026, 2027];
    end
    if ~isfield(options, 'validation_cases')
        options.validation_cases = {'C0', 'F010', 'F110', 'F111', 'C2'};
    end
    if ~isfield(options, 'validation_seeds')
        options.validation_seeds = [2026, 2027, 2028];
    end
    if ~isfield(options, 'validation_variants')
        options.validation_variants = {'baseline', 'position_only'};
    end
    if ~isfield(options, 'run_validation_matrix')
        options.run_validation_matrix = false;
    end
    if ~isfield(options, 'trace_case_id')
        options.trace_case_id = 'F010';
    end
    if ~isfield(options, 'trace_seed')
        options.trace_seed = 2026;
    end
    if ~isfield(options, 'heading_shortlist_deg')
        options.heading_shortlist_deg = [0, 15, -15, 30, -30];
    end
    if ~isfield(options, 'local_probe_tangential_offsets_m')
        options.local_probe_tangential_offsets_m = [-0.15, -0.075, 0, 0.075, 0.15];
    end
end

function ensure_result_dir()
    out_dir = fullfile('results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
end

function v = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end
