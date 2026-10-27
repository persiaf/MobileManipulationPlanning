function result = run_cafeteria_planning(task_mode, seed, save_figures, figure_visible, planning_mode)
%RUN_CAFETERIA_PLANNING Execute the gated cafeteria experiment.

    if nargin < 1 || isempty(task_mode), task_mode = 'complex12'; end
    if isnumeric(task_mode)
        legacy_seed = task_mode;
        if nargin >= 2 && islogical(seed), save_figures = seed; end
        task_mode = 'complex12';
        seed = legacy_seed;
    elseif nargin < 2 || isempty(seed)
        seed = 2026;
    end
    if nargin < 3 || isempty(save_figures), save_figures = true; end
    if nargin < 4 || isempty(figure_visible), figure_visible = 'off'; end
    if nargin < 5 || isempty(planning_mode), planning_mode = 'full_manipulation'; end
    task_mode = lower(char(task_mode));
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    override = cafeteria_experiment_override();
    override.planner.runtime.headless = true;
    override.planner.runtime.disable_visualization = true;
    override.planner.runtime.disable_video = true;
    profile_override.scene = cafeteria_scene_profile_config(task_mode, seed);
    profile_override.scene.cafeteria.planning_mode = lower(char(planning_mode));
    effective = resolve_effective_config('profile', 'CAFETERIA_1X3_2X3', ...
        'seed', seed, 'robot_profile', 'physical_robot', ...
        'profile_override', profile_override, 'experiment_override', override);
    effective_summary = print_effective_config(effective);
    scene = build_scene_from_cfg(effective.scene);
    robot_cfg = effective.robot;
    planner_cfg = effective.planner;
    validation = validate_cafeteria_scene(scene, robot_cfg);
    if strcmpi(planning_mode, 'navigation_docking_only')
        operability = navigation_only_operability(scene, robot_cfg);
        fprintf('MANIPULATION_REACHABILITY_GATE = IGNORED\nSERVICE_SUPPORT = CAFETERIA_2D_PROXY\n');
    else
        operability = evaluate_cafeteria_operability(scene, robot_cfg);
    end

    result = initial_result(scene, robot_cfg, planner_cfg, effective_summary, validation, operability);
    result.planning_mode = lower(char(planning_mode));
    result.service_support_mode = ternary(strcmpi(planning_mode,'navigation_docking_only'), ...
        'cafeteria_2d_proxy', 'not_applicable');
    out_root = cafeteria_output_root(root);
    vis_cfg = effective.visualization;
    vis_cfg.figure_visible = char(figure_visible);
    vis_cfg.output_dir = fullfile(out_root, 'results', 'figures', 'cafeteria');
    if save_figures
        visualize_cafeteria_scene(scene, vis_cfg, 'title', 'CAFETERIA 1x3 / 2x3 Scene', ...
            'filename', sprintf('cafeteria_%s_scene', task_mode));
    end

    if ~validation.is_valid
        result.status = 'CAFETERIA_SCENE_INVALID';
        finish_outputs(result, vis_cfg, out_root);
        return;
    end
    if ~all([operability.model_feasible])
        result.status = 'CAFETERIA_REACHABILITY_BLOCKED';
        if save_figures
            visualize_cafeteria_scene(scene, vis_cfg, ...
                'title', 'CAFETERIA Operability: Reachability Blocked', ...
                'filename', sprintf('cafeteria_%s_operability', task_mode), ...
                'operability', operability);
        end
        finish_outputs(result, vis_cfg, out_root);
        return;
    end

    t_plan = tic;
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    if strcmpi(planning_mode, 'navigation_docking_only')
        library = build_cafeteria_navigation_library(regions, scene, robot_cfg, planner_cfg);
    else
        library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    end
    result.raw_candidates = raw_candidate_count(library);
    result.valid_candidates = library.total_candidates;
    result.collision_valid_candidates = collision_valid_candidate_count(library);
    result.candidate_diagnostic = candidate_radial_diagnostic(library, scene, robot_cfg);
    covered = library_covered_tasks(library);
    if numel(covered) < scene.num_tasks
        planner_cfg.candidate_augmentation.enable = true;
        planner_cfg.candidate_augmentation.variant = 'position_only';
        if strcmpi(planning_mode, 'navigation_docking_only')
            library = build_cafeteria_navigation_library(regions, scene, robot_cfg, planner_cfg);
        else
            library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
        end
        result.augmentation = 'position_only';
        result.raw_candidates = raw_candidate_count(library);
        result.valid_candidates = library.total_candidates;
        result.collision_valid_candidates = collision_valid_candidate_count(library);
        result.candidate_diagnostic = candidate_radial_diagnostic(library, scene, robot_cfg);
        covered = library_covered_tasks(library);
    end
    result.candidate_coverage = numel(covered) / scene.num_tasks;
    if result.candidate_coverage < 1
        result.status = 'CAFETERIA_CANDIDATE_COVERAGE_BLOCKED';
        result.planning_time_s = toc(t_plan);
        finish_outputs(result, vis_cfg, out_root);
        return;
    end

    [~, ~, ~, ~, plan] = stage2_docking_planning(regions, scene, robot_cfg, planner_cfg, library);
    [~, path, ~, visit_order, ~, path_length, stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, planner_cfg, plan.pose_info_list, struct(), plan);
    stats.global_path = path;
    result.plan = plan;
    result.stats = stats;
    result.global_path = path;
    result.selected_docks = size(plan.selected_poses, 1);
    result.node_visit_order = visit_order;
    result.visit_order = table_visit_order(visit_order, plan.task_sequence_summary);
    result.tasks_per_selected_dock = cellfun(@(s) s.covered_count, ...
        plan.task_sequence_summary(:))';
    result.multi_task_dock_count = sum(result.tasks_per_selected_dock >= 2);
    result.path_length_m = path_length;
    path_report = validate_cafeteria_path(scene, path, robot_cfg);
    result.waypoint_count = path_report.waypoint_count;
    result.path_collision_count = path_report.path_collision_count;
    result.footprint_collision_count = path_report.footprint_collision_count;
    result.minimum_clearance_m = path_report.minimum_clearance_m;
    result.planning_time_s = toc(t_plan);
    if result.path_collision_count > 0 || result.footprint_collision_count > 0
        result.status = 'CAFETERIA_PATH_COLLISION_FAILURE';
    else
        result.status = 'CAFETERIA_PLANNING_SUCCESS';
    end
    if save_figures
        visualize_cafeteria_scene(scene, vis_cfg, 'title', 'CAFETERIA Selected Docks', ...
            'filename', sprintf('cafeteria_%s_docking', task_mode), ...
            'operability', operability, 'plan', plan);
        visualize_cafeteria_scene(scene, vis_cfg, 'title', 'CAFETERIA Collision-free Route', ...
            'filename', sprintf('cafeteria_%s_planning', task_mode), ...
            'operability', operability, ...
            'plan', plan, 'path', path);
        visualize_cafeteria_scene(scene, vis_cfg, 'title', 'CAFETERIA Video Reference', ...
            'filename', sprintf('cafeteria_%s_video_reference', task_mode), ...
            'operability', operability, 'plan', plan, 'path', path);
    end
    export_cafeteria_docking_csv(result, out_root, task_mode);
    export_cafeteria_waypoint_csv(result, out_root, task_mode);
    finish_outputs(result, vis_cfg, out_root);
end

function export_cafeteria_docking_csv(result, root, task_mode)
    if ~isfield(result, 'plan') || ~isfield(result.plan, 'selected_poses'), return; end
    n = size(result.plan.selected_poses, 1);
    rows = repmat(struct('dock_id',0,'visit_index',0,'table_id',0,'table_side','', ...
        'dock_x_m',0,'dock_y_m',0,'dock_yaw_rad',0,'covered_task_ids','', 'task_count',0), n, 1);
    for i = 1:n
        s = result.plan.task_sequence_summary{i}; p = result.plan.selected_poses(i,:);
        info = result.plan.pose_info_list{i};
        rows(i) = struct('dock_id',i,'visit_index',find(result.visit_order==s.table_id,1), ...
            'table_id',s.table_id,'table_side',cafeteria_side_name(info.side_id), ...
            'dock_x_m',p(1),'dock_y_m',p(2),'dock_yaw_rad',p(3), ...
            'covered_task_ids',mat2str(s.covered_tasks),'task_count',s.covered_count);
    end
    out = fullfile(root,'results','cafeteria'); if ~isfolder(out), mkdir(out); end
    writetable(struct2table(rows), fullfile(out, sprintf('cafeteria_%s_selected_docks.csv',task_mode)));
end

function export_cafeteria_waypoint_csv(result, root, task_mode)
    path = result.global_path; if isempty(path), return; end
    n = size(path,1); rows = repmat(struct('waypoint_id',0,'x_m',0,'y_m',0,'yaw_rad',0, ...
        'segment_id',0,'from_dock_id',0,'to_dock_id',0,'is_dock_waypoint',false), n, 1);
    dock_poses = result.plan.selected_poses;
    for i=1:n
        yaw = 0; if size(path,2)>=3, yaw=path(i,3); elseif i<n, yaw=atan2(path(i+1,2)-path(i,2),path(i+1,1)-path(i,1)); end
        [is_dock, did] = nearest_dock(path(i,1:2),dock_poses);
        rows(i)=struct('waypoint_id',i,'x_m',path(i,1),'y_m',path(i,2),'yaw_rad',yaw, ...
            'segment_id',max(i-1,1),'from_dock_id',max(did-1,0),'to_dock_id',did,'is_dock_waypoint',is_dock);
    end
    out = fullfile(root,'results','cafeteria'); if ~isfolder(out), mkdir(out); end
    writetable(struct2table(rows), fullfile(out, sprintf('cafeteria_%s_waypoints.csv',task_mode)));
end

function [tf, did] = nearest_dock(xy, docks)
    if isempty(docks), tf=false; did=0; return; end
    [d,did]=min(vecnorm(docks(:,1:2)-xy,2,2)); tf=d<1e-9;
end

function name = cafeteria_side_name(side_id)
    names={'NORTH','SOUTH','WEST','EAST'}; name=names{max(1,min(4,side_id))};
end

function op = navigation_only_operability(scene, robot_cfg)
    op = repmat(struct('task_id',0,'table_id',0,'robot_profile',robot_cfg.geometry_profile, ...
        'arm_base_z',robot_cfg.arm_base_mount_z,'task_z',0,'relative_z',NaN, ...
        'reach_z_max',robot_cfg.single_arm.reach_z_max,'reachability_score',NaN, ...
        'reachability_result','IGNORED_NAVIGATION_ONLY','model_feasible',true), scene.num_tasks, 1);
    for i = 1:scene.num_tasks
        op(i).task_id = i; op(i).table_id = scene.tasks(i).table_id; op(i).task_z = scene.tasks(i).z;
    end
end

function result = initial_result(scene, robot, planner, summary, validation, operability)
    result = struct('status', 'CAFETERIA_SCENE_INVALID', 'scene', scene, ...
        'robot_cfg', robot, 'planner_cfg', planner, 'effective_config', summary, ...
        'validation', validation, 'operability', operability, ...
        'raw_candidates', 0, 'valid_candidates', 0, ...
        'collision_valid_candidates', 0, 'candidate_coverage', NaN, ...
        'planning_mode', 'full_manipulation', 'service_support_mode', 'not_applicable', ...
        'candidate_diagnostic', struct('reason', '', 'min_task_distance_m', NaN, ...
        'reach_radius_m', robot.single_arm.reach_radius), ...
        'selected_docks', 0, 'multi_task_dock_count', NaN, ...
        'tasks_per_selected_dock', [], 'node_visit_order', [], ...
        'visit_order', [], 'path_length_m', 0, ...
        'waypoint_count', 0, 'path_collision_count', NaN, ...
        'footprint_collision_count', NaN, 'minimum_clearance_m', NaN, ...
        'planning_time_s', 0, 'augmentation', 'disabled', ...
        'plan', struct(), 'stats', struct(), 'global_path', zeros(0, 3));
end

function diagnostic = candidate_radial_diagnostic(library, scene, robot_cfg)
    diagnostic = struct('reason', 'HORIZONTAL_RADIAL_REACH_LIMIT', ...
        'min_task_distance_m', inf, 'reach_radius_m', robot_cfg.single_arm.reach_radius);
    for ri = 1:numel(library.regions)
        ids = library.regions(ri).task_ids;
        for ci = 1:numel(library.regions(ri).candidates)
            p = library.regions(ri).candidates(ci).pose;
            for ti = 1:numel(ids)
                task = scene.tasks(ids(ti));
                diagnostic.min_task_distance_m = min(diagnostic.min_task_distance_m, ...
                    norm(p(1:2) - [task.x, task.y]));
            end
        end
    end
    if ~isfinite(diagnostic.min_task_distance_m)
        diagnostic.min_task_distance_m = NaN;
    end
end

function out = ternary(condition, yes_value, no_value)
    if condition, out = yes_value; else, out = no_value; end
end

function out_root = cafeteria_output_root(root)
%CAFETERIA_OUTPUT_ROOT Redirect cafeteria outputs when MMP_CAFETERIA_OUTPUT_ROOT is set.
%   Tests set this environment variable so that historical result files under
%   <root>/results are never overwritten. Default is the project root.
    out_root = root;
    override = getenv('MMP_CAFETERIA_OUTPUT_ROOT');
    if ~isempty(override)
        out_root = override;
    end
end

function order = table_visit_order(node_order, task_sequence_summary)
    order = zeros(1, max(numel(node_order) - 1, 0));
    count = 0;
    for i = 1:numel(node_order)
        if node_order(i) == 1, continue; end
        count = count + 1;
        order(count) = task_sequence_summary{node_order(i) - 1}.table_id;
    end
    order = order(1:count);
end

function n = raw_candidate_count(library)
    n = 0;
    for i = 1:numel(library.regions)
        n = n + library.regions(i).generation_diagnostics.raw_candidate_count;
    end
end

function n = collision_valid_candidate_count(library)
    n = 0;
    for i = 1:numel(library.regions)
        diagnostic = library.regions(i).generation_diagnostics;
        if isfield(diagnostic, 'collision_valid_candidate_count')
            n = n + diagnostic.collision_valid_candidate_count;
        elseif isfield(diagnostic, 'library_candidate_count')
            n = n + diagnostic.library_candidate_count;
        end
    end
end

function covered = library_covered_tasks(library)
    covered = [];
    for i = 1:numel(library.regions)
        for j = 1:numel(library.regions(i).candidates)
            covered = union(covered, library.regions(i).candidates(j).covered_tasks, 'stable');
        end
    end
end

function finish_outputs(result, vis_cfg, root)
    if isfinite(result.path_collision_count) && result.path_collision_count > 0
        error('run_cafeteria_planning:PathCollision', ...
            'Formal validator found %d colliding path segments.', result.path_collision_count);
    end
    out_dir = fullfile(root, 'results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
    cfg = result.scene.scene_cfg;
    tasks_per_table = accumarray(cfg.manual_tasks(:, 1), 1, [cfg.num_tables, 1])';
    row = struct('profile', cfg.profile_id, 'seed', cfg.random_seed, ...
        'task_mode', cfg.cafeteria.task_mode, ...
        'planning_mode', result.planning_mode, ...
        'map_width', cfg.map_width_m, 'map_height', cfg.map_height_m, ...
        'table_count', cfg.num_tables, 'total_tasks', result.scene.num_tasks, ...
        'active_tables', numel(cfg.active_table_ids), ...
        'region_count', result.scene.region_metadata.region_count, ...
        'tasks_per_table', mat2str(tasks_per_table), ...
        'main_aisle_width', cfg.cafeteria.main_aisle_width_m, ...
        'robot_profile', result.robot_cfg.geometry_profile, ...
        'table_height', cfg.cafeteria.table_height_m, 'task_z', cfg.task_offset_z, ...
        'model_feasible_tasks', sum([result.operability.model_feasible]), ...
        'service_support_mode', result.service_support_mode, ...
        'candidate_block_reason', result.candidate_diagnostic.reason, ...
        'candidate_min_task_distance', result.candidate_diagnostic.min_task_distance_m, ...
        'reach_radius', result.candidate_diagnostic.reach_radius_m, ...
        'raw_candidates', result.raw_candidates, ...
        'valid_candidates', result.valid_candidates, ...
        'collision_valid_candidates', result.collision_valid_candidates, ...
        'supported_candidates', result.valid_candidates, ...
        'selected_docks', result.selected_docks, ...
        'multi_task_docks', result.multi_task_dock_count, ...
        'tasks_per_selected_dock', mat2str(result.tasks_per_selected_dock), ...
        'multi_task_dock_count', result.multi_task_dock_count, ...
        'coverage', result.candidate_coverage, ...
        'task_coverage', result.candidate_coverage, ...
        'feasible_task_coverage', result.candidate_coverage, ...
        'visit_order', mat2str(result.visit_order), ...
        'actual_visit_order', mat2str(result.visit_order), ...
        'path_length', result.path_length_m, 'path_length_m', result.path_length_m, ...
        'waypoint_count', result.waypoint_count, ...
        'collision_count', result.path_collision_count, ...
        'path_collision_count', result.path_collision_count, ...
        'footprint_collision_count', result.footprint_collision_count, ...
        'minimum_clearance', result.minimum_clearance_m, ...
        'minimum_clearance_m', result.minimum_clearance_m, ...
        'planning_time', result.planning_time_s, 'planning_time_s', result.planning_time_s, ...
        'return_to_start', planner_return_to_start(result), ...
        'status', result.status);
    writetable(struct2table(row), fullfile(out_dir, ...
        sprintf('cafeteria_%s_planning_summary.csv', cfg.cafeteria.task_mode)));
    fprintf('CAFETERIA_STATUS=%s\n', result.status);
    if strcmp(vis_cfg.figure_visible, 'off'), close all force; end
end

function value = planner_return_to_start(result)
    value = false;
    if isfield(result, 'planner_cfg') && isfield(result.planner_cfg, 'execution') && ...
            isfield(result.planner_cfg.execution, 'return_to_start')
        value = result.planner_cfg.execution.return_to_start;
    elseif isfield(result, 'effective_config') && isfield(result.effective_config, 'planner') && ...
            isfield(result.effective_config.planner, 'execution')
        value = result.effective_config.planner.execution.return_to_start;
    end
end
