function outputs = exp_center_row_candidate_geometry(options)
%EXP_CENTER_ROW_CANDIDATE_GEOMETRY Compare center-row docking geometry.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    rows = repmat(empty_row(), 0, 1);
    run_rows = repmat(empty_run_row(), 0, 1);

    for seed = options.seeds
        scene = build_scene_from_cfg(large_scale_profile_config(options.scale, seed));
        robot_cfg = robot_config();
        planner_cfg = planner_config();
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
        seed_rows = repmat(empty_row(), 0, 1);

        for ri = 1:numel(regions)
            candidates = library.regions(ri).candidates;
            task_ids = regions(ri).task_ids;
            center_indices = find_center_row_tasks(scene, task_ids, options.center_row_tolerance_m);
            if isempty(center_indices), continue; end
            table_id = regions(ri).table_id;
            table_center = scene.scene_cfg.table_positions(table_id, :);
            table_theta = scene.scene_cfg.table_orientations(table_id);

            for ci = 1:numel(candidates)
                candidate = candidates(ci);
                pose = candidate.pose;
                candidate_local = rot2(-table_theta) * (pose(1:2) - table_center)';
                [normal_offset, tangential_offset] = side_offsets( ...
                    candidate_local, candidate.side_id, scene.scene_cfg.table);
                clearance = compute_footprint_clearance( ...
                    pose, scene.grid_map, scene.map_info, robot_cfg);

                for ii = 1:numel(center_indices)
                    ti = center_indices(ii);
                    tid = task_ids(ti);
                    target = [scene.tasks(tid).x, scene.tasks(tid).y];
                    task_local = rot2(-table_theta) * (target - table_center)';
                    delta_world = target - pose(1:2);
                    delta_base = rot2(-pose(3)) * delta_world';
                    row = empty_row();
                    row.scale = options.scale; row.seed = seed;
                    row.region_id = ri; row.table_id = table_id;
                    row.candidate_index = ci; row.task_id = tid;
                    row.side_id = candidate.side_id;
                    row.reference_region = ri >= 9 && ri <= 12;
                    row.candidate_x = pose(1); row.candidate_y = pose(2);
                    row.candidate_theta = pose(3);
                    row.candidate_table_local_x = candidate_local(1);
                    row.candidate_table_local_y = candidate_local(2);
                    row.task_table_local_x = task_local(1);
                    row.task_table_local_y = task_local(2);
                    row.normal_offset_m = normal_offset;
                    row.tangential_offset_m = tangential_offset;
                    row.task_projection_tangential_error_m = tangential_offset - ...
                        task_tangential_coordinate(task_local, candidate.side_id);
                    row.task_dx_world_m = delta_world(1);
                    row.task_dy_world_m = delta_world(2);
                    row.task_dx_base_m = delta_base(1);
                    row.task_dy_base_m = delta_base(2);
                    row.candidate_to_task_distance_m = norm(delta_world);
                    row.heading_relative_table_rad = wrapToPiLocal(pose(3) - table_theta);
                    row.heading_relative_task_rad = wrapToPiLocal( ...
                        pose(3) - atan2(delta_world(2), delta_world(1)));
                    row.footprint_clearance_m = clearance;
                    row.left_arm_reach_score = candidate.reach_scores(ti, 1);
                    row.right_arm_reach_score = candidate.reach_scores(ti, 2);
                    row.combined_reach_score = candidate.reach_scores(ti, 3);
                    row.covered_task_count = numel(candidate.covered_tasks);
                    row.supports_center_row_task = candidate.reach_scores(ti, 3) > 0;
                    seed_rows(end + 1, 1) = row; %#ok<AGROW>
                end
            end
        end

        rows = [rows; seed_rows]; %#ok<AGROW>
        run = empty_run_row(); run.scale = options.scale; run.seed = seed;
        run.candidate_task_pairs = numel(seed_rows);
        run.supporting_pairs = sum([seed_rows.supports_center_row_task]);
        run.reference_supporting_pairs = sum([seed_rows.reference_region] & ...
            [seed_rows.supports_center_row_task]);
        run.other_supporting_pairs = sum(~[seed_rows.reference_region] & ...
            [seed_rows.supports_center_row_task]);
        run_rows(end + 1, 1) = run; %#ok<AGROW>
        fprintf('CENTER_GEOMETRY scale=%s seed=%d pairs=%d support=%d\n', ...
            options.scale, seed, run.candidate_task_pairs, run.supporting_pairs);
    end

    outputs.candidate_task = struct2table(rows, 'AsArray', true);
    outputs.run = struct2table(run_rows, 'AsArray', true);
    outputs.group = build_group_summary(outputs.candidate_task);
    if options.write_files, write_outputs(outputs, root); end
end

function coordinate = task_tangential_coordinate(task_local, side_id)
    if side_id == 1 || side_id == 2
        coordinate = task_local(1);
    else
        coordinate = task_local(2);
    end
end

function indices = find_center_row_tasks(scene, task_ids, tolerance)
    indices = [];
    for ti = 1:numel(task_ids)
        tid = task_ids(ti); table_id = scene.tasks(tid).table_id;
        center = scene.scene_cfg.table_positions(table_id, :);
        theta = scene.scene_cfg.table_orientations(table_id);
        local = rot2(-theta) * ([scene.tasks(tid).x, scene.tasks(tid).y] - center)';
        if abs(local(2)) <= tolerance
            indices(end + 1) = ti; %#ok<AGROW>
        end
    end
end

function [normal_offset, tangential_offset] = side_offsets(local, side_id, table)
    switch side_id
        case 1
            normal_offset = local(2) - table.width / 2; tangential_offset = local(1);
        case 2
            normal_offset = -local(2) - table.width / 2; tangential_offset = local(1);
        case 3
            normal_offset = -local(1) - table.length / 2; tangential_offset = local(2);
        case 4
            normal_offset = local(1) - table.length / 2; tangential_offset = local(2);
        otherwise
            normal_offset = NaN; tangential_offset = NaN;
    end
end

function out = build_group_summary(rows)
    names = {'reference_support'; 'reference_no_support'; ...
        'other_support'; 'other_no_support'};
    masks = {rows.reference_region & rows.supports_center_row_task, ...
        rows.reference_region & ~rows.supports_center_row_task, ...
        ~rows.reference_region & rows.supports_center_row_task, ...
        ~rows.reference_region & ~rows.supports_center_row_task};
    out_rows = repmat(empty_group_row(), numel(names), 1);
    for i = 1:numel(names)
        subset = rows(masks{i}, :); g = empty_group_row(); g.group = names{i};
        g.count = height(subset);
        g.normal_offset_min_m = safe_stat(subset.normal_offset_m, @min);
        g.normal_offset_median_m = safe_stat(subset.normal_offset_m, @median);
        g.normal_offset_max_m = safe_stat(subset.normal_offset_m, @max);
        g.abs_tangential_offset_median_m = safe_stat(abs(subset.tangential_offset_m), @median);
        g.abs_heading_relative_task_median_rad = safe_stat(abs(subset.heading_relative_task_rad), @median);
        g.clearance_median_m = safe_stat(subset.footprint_clearance_m, @median);
        g.combined_reach_median = safe_stat(subset.combined_reach_score, @median);
        out_rows(i) = g;
    end
    out = struct2table(out_rows, 'AsArray', true);
end

function value = safe_stat(values, fn)
    values = values(isfinite(values));
    if isempty(values), value = NaN; else, value = fn(values); end
end

function write_outputs(outputs, root)
    out_dir = fullfile(root, 'results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
    writetable(outputs.candidate_task, fullfile(out_dir, ...
        'center_row_candidate_geometry_trace.csv'));
    writetable(outputs.run, fullfile(out_dir, 'center_row_candidate_geometry_run.csv'));
    writetable(outputs.group, fullfile(out_dir, 'center_row_candidate_geometry_summary.csv'));
    save(fullfile(out_dir, 'center_row_candidate_geometry_20260901.mat'), 'outputs');
end

function options = defaults(options)
    if ~isfield(options, 'scale'), options.scale = 's4'; end
    if ~isfield(options, 'seeds'), options.seeds = 1:5; end
    if ~isfield(options, 'center_row_tolerance_m'), options.center_row_tolerance_m = 1e-8; end
    if ~isfield(options, 'write_files'), options.write_files = true; end
end

function row = empty_row()
    row = struct('scale','','seed',0,'region_id',0,'table_id',0, ...
        'candidate_index',0,'task_id',0,'side_id',0,'reference_region',false, ...
        'candidate_x',NaN,'candidate_y',NaN,'candidate_theta',NaN, ...
        'candidate_table_local_x',NaN,'candidate_table_local_y',NaN, ...
        'task_table_local_x',NaN,'task_table_local_y',NaN, ...
        'normal_offset_m',NaN,'tangential_offset_m',NaN, ...
        'task_projection_tangential_error_m',NaN, ...
        'task_dx_world_m',NaN,'task_dy_world_m',NaN, ...
        'task_dx_base_m',NaN,'task_dy_base_m',NaN, ...
        'candidate_to_task_distance_m',NaN, ...
        'heading_relative_table_rad',NaN,'heading_relative_task_rad',NaN, ...
        'footprint_clearance_m',NaN,'left_arm_reach_score',0, ...
        'right_arm_reach_score',0,'combined_reach_score',0, ...
        'covered_task_count',0,'supports_center_row_task',false);
end

function row = empty_run_row()
    row = struct('scale','','seed',0,'candidate_task_pairs',0, ...
        'supporting_pairs',0,'reference_supporting_pairs',0,'other_supporting_pairs',0);
end

function row = empty_group_row()
    row = struct('group','','count',0,'normal_offset_min_m',NaN, ...
        'normal_offset_median_m',NaN,'normal_offset_max_m',NaN, ...
        'abs_tangential_offset_median_m',NaN, ...
        'abs_heading_relative_task_median_rad',NaN, ...
        'clearance_median_m',NaN,'combined_reach_median',NaN);
end
