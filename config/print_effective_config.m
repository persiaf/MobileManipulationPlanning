function summary = print_effective_config(effective)
%PRINT_EFFECTIVE_CONFIG Print a stable summary of final runtime parameters.

    if nargin < 1 || isempty(effective)
        effective = resolve_effective_config();
    end
    scene = effective.scene;
    robot = effective.robot;
    planner = effective.planner;
    real = effective.real_platform;
    meta = getfield_default(effective, 'meta', struct());

    summary = struct();
    summary.profile = getfield_default(meta, 'profile', '');
    summary.seed = getfield_default(meta, 'seed', getfield_default(scene, 'random_seed', NaN));
    summary.map_size_m = [scene.map_width_m, scene.map_height_m];
    summary.resolution_m = scene.grid_resolution_m;
    summary.table_count = table_count(scene);
    summary.task_count = requested_task_count(scene);
    summary.region_count = region_count(scene);
    summary.static_obstacle_count = obstacle_count(scene);
    summary.robot_profile = getfield_default(robot, 'geometry_profile', 'legacy_validated');
    summary.strategy = planner.strategy.name;
    summary.global_planner = planner.global_planner;
    summary.routing_mode = planner.routing.mode;
    summary.sparse_k = planner.routing.sparse_knn.k;
    summary.inflation_radius_m = planner.inflation_radius_m;
    summary.augmentation_enable = planner.candidate_augmentation.enable;
    summary.augmentation_variant = planner.candidate_augmentation.variant;
    summary.candidate_prefilter = planner.candidate_pair_prefilter.enable;
    summary.path_cost_cache = planner.strategy.common.use_path_cost_cache;
    summary.shared_lazy_cache = planner.strategy.bidirectional.use_shared_path_cost_cache;
    summary.return_to_start = planner.execution.return_to_start;
    summary.visualization_source = getfield_default(meta, 'visualization_source', 'visualization_config');
    summary.real_platform_enabled = real.enabled;
    summary.lift_enabled = real.lift.enabled;
    if isfield(scene, 'cafeteria')
        summary.planning_mode = getfield_default(scene.cafeteria, 'planning_mode', 'full_manipulation');
        summary.service_support_mode = ternary(strcmpi(summary.planning_mode, ...
            'navigation_docking_only'), 'cafeteria_2d_proxy', 'not_applicable');
    else
        summary.planning_mode = 'full_manipulation';
        summary.service_support_mode = 'not_applicable';
    end

    fprintf('===== EFFECTIVE CONFIG =====\n');
    fprintf('profile: %s\nseed: %s\n', display_value(summary.profile), display_value(summary.seed));
    fprintf('map size: %.3f x %.3f m\nresolution: %.4f m/cell\n', ...
        summary.map_size_m(1), summary.map_size_m(2), summary.resolution_m);
    fprintf('tables/tasks/regions/static obstacles: %s / %s / %s / %s\n', ...
        display_value(summary.table_count), display_value(summary.task_count), ...
        display_value(summary.region_count), display_value(summary.static_obstacle_count));
    fprintf('robot profile: %s\nstrategy: %s\nglobal planner: %s\n', ...
        summary.robot_profile, summary.strategy, summary.global_planner);
    fprintf('routing: %s (k=%d)\ninflation radius: %.3f m\n', ...
        summary.routing_mode, summary.sparse_k, summary.inflation_radius_m);
    fprintf('candidate augmentation: %d (%s)\ncandidate prefilter: %d\n', ...
        summary.augmentation_enable, summary.augmentation_variant, summary.candidate_prefilter);
    fprintf('path-cost cache/shared lazy cache: %d / %d\nreturn to start: %d\n', ...
        summary.path_cost_cache, summary.shared_lazy_cache, summary.return_to_start);
    fprintf('visualization source: %s\nreal platform/lift: %d / %d\n', ...
        summary.visualization_source, summary.real_platform_enabled, summary.lift_enabled);
    fprintf('planning mode: %s\nservice support: %s\n', ...
        summary.planning_mode, summary.service_support_mode);
    fprintf('============================\n');
end

function out = ternary(condition, yes_value, no_value)
    if condition, out = yes_value; else, out = no_value; end
end

function n = table_count(scene)
    uses_layout = isfield(scene, 'table_mode') && strcmpi(scene.table_mode, 'layout');
    if uses_layout && isfield(scene, 'layout') && isfield(scene.layout, 'num_tables')
        n = scene.layout.num_tables;
    elseif isfield(scene, 'table_positions')
        n = size(scene.table_positions, 1);
    elseif isfield(scene, 'layout') && isfield(scene.layout, 'num_tables')
        n = scene.layout.num_tables;
    else
        n = NaN;
    end
end

function n = requested_task_count(scene)
    if isfield(scene, 'requested_task_count')
        n = scene.requested_task_count;
    elseif isfield(scene, 'manual_tasks') && ~isempty(scene.manual_tasks)
        n = size(scene.manual_tasks, 1);
    elseif isfield(scene, 'tasks_per_table_range')
        n = table_count(scene) * mean(scene.tasks_per_table_range);
    else
        n = NaN;
    end
end

function n = region_count(scene)
    if isfield(scene, 'layout') && isfield(scene.layout, 'cluster_count')
        n = scene.layout.cluster_count;
    elseif isfield(scene, 'table_region_id')
        n = numel(unique(scene.table_region_id));
    else
        n = NaN;
    end
end

function n = obstacle_count(scene)
    if isfield(scene, 'visual_static_obstacles')
        n = numel(scene.visual_static_obstacles);
    else
        n = 0;
    end
end

function value = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        value = s.(name);
    else
        value = default_value;
    end
end

function text = display_value(value)
    if ischar(value) || isstring(value)
        text = char(value);
    elseif isnumeric(value) || islogical(value)
        text = mat2str(value);
    else
        text = '<n/a>';
    end
end
