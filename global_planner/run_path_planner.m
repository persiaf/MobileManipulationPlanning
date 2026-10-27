function [path, fallback_info, search_state, search_diagnostics] = run_path_planner(grid_map, start_pose, goal_pose, map_info, planner_cfg, algo_override, run_opts)
%RUN_PATH_PLANNER 统一路径规划入口。
%   所有算法输出与 fallback 到 A* 的输出都会经过同一后处理链：
%   world path -> grid path -> post_validate_path -> world path -> densify -> collision check。

    if nargin < 7 || isempty(run_opts)
        run_opts = struct();
    end
    search_state = [];
    search_diagnostics = struct();
    if nargin < 6 || isempty(algo_override)
        algo_override = '';
    end

    % Keep the planner implementation unchanged while providing it a map on
    % which a center point is guaranteed to represent the full base footprint.
    % Candidate generation and task/dock validation continue using scene.grid_map.
    if footprint_navigation_enabled(planner_cfg)
        robot_cfg = get_robot_config(planner_cfg);
        [grid_map, inflation_info] = build_footprint_inflated_map( ...
            grid_map, robot_cfg.footprint, map_info);
        planner_cfg.footprint_inflation = inflation_info;
    end

    fallback_info = struct('fallback_used', false, ...
                           'fallback_count', 0, ...
                           'collision_segments', 0);
    suppress_warn = isfield(run_opts, 'suppress_warnings') && run_opts.suppress_warnings;

    if isempty(algo_override)
        if isfield(planner_cfg, 'global_planner')
            algorithm = planner_cfg.global_planner;
        else
            algorithm = 'astar';
        end
    else
        algorithm = algo_override;
    end

    [raw_path, search_state, search_diagnostics] = run_selected_algorithm(algorithm, grid_map, start_pose, goal_pose, map_info, planner_cfg, run_opts);
    if isstruct(search_diagnostics) && isfield(run_opts, 'changed_cells') && ...
            isnumeric(run_opts.changed_cells)
        search_diagnostics.changed_cell_count = size(run_opts.changed_cells, 1);
    end
    if strcmpi(algorithm, 'dstar_lite') && isstruct(search_diagnostics)
        search_diagnostics.actual_planner = 'dstar_lite';
        search_diagnostics.fallback_used = false;
        search_diagnostics.fallback_reason = '';
        search_diagnostics.search_state_valid = isstruct(search_state) && ...
            isfield(search_diagnostics, 'path_safe') && search_diagnostics.path_safe;
    end
    [path, is_valid, n_collision] = normalize_and_validate_path(raw_path, grid_map, map_info, suppress_warn);

    if ~is_valid && ~strcmpi(algorithm, 'astar')
        fallback_info.fallback_used = true;
        fallback_info.fallback_count = 1;
        fallback_info.collision_segments = n_collision;

        if ~suppress_warn
            warning('run_path_planner:collision', ...
                    '%s: 路径存在 %d 个碰撞段，回退到 A* 重规划。', ...
                    algorithm, n_collision);
        end

        raw_path = run_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        [path, is_valid_after_fallback, n_collision_after] = ...
            normalize_and_validate_path(raw_path, grid_map, map_info, suppress_warn);
        if strcmpi(algorithm, 'dstar_lite') && isstruct(search_diagnostics)
            search_diagnostics.fallback_used = true;
            search_diagnostics.fallback_reason = 'D* Lite path failed unified map validation';
            search_diagnostics.actual_planner = 'astar';
            search_diagnostics.search_state_valid = false;
            % Diagnostics describe the path returned by this entry point.  A
            % successful A* fallback therefore replaces the D* Lite search
            % result fields as well as the planner label.
            search_diagnostics.path_found = ~isempty(path) && size(path, 1) >= 2;
            search_diagnostics.path_safe = is_valid_after_fallback;
            search_diagnostics.fallback_collision_segments = n_collision_after;
            if isfield(search_diagnostics, 'path_length_m')
                search_diagnostics.path_length_m = compute_path_length(path);
                search_diagnostics.path_length = search_diagnostics.path_length_m;
            end
            % The returned route belongs to A*, so the D* Lite search state
            % cannot be attached to or reused as the producer of this route.
            search_state = [];
        end

        if ~is_valid_after_fallback
            fallback_info.collision_segments = fallback_info.collision_segments + n_collision_after;
            if ~suppress_warn
                warning('run_path_planner:collision', ...
                        'A* fallback 后仍存在 %d 个碰撞段。', n_collision_after);
            end
        end
    elseif ~is_valid
        fallback_info.collision_segments = n_collision;
        if ~suppress_warn
            warning('run_path_planner:collision', ...
                    'A* 路径存在 %d 个碰撞段。', n_collision);
        end
    end
    if strcmpi(algorithm, 'dstar_lite') && isstruct(search_diagnostics) && ~isfield(search_diagnostics, 'path_safe')
        search_diagnostics.path_safe = is_valid;
        search_diagnostics.fallback_collision_segments = 0;
    end
end

function robot_cfg = get_robot_config(planner_cfg)
    if isstruct(planner_cfg) && isfield(planner_cfg, 'robot_cfg') && ...
            isstruct(planner_cfg.robot_cfg)
        robot_cfg = planner_cfg.robot_cfg;
        return;
    end
    if isstruct(planner_cfg) && isfield(planner_cfg, 'robot') && ...
            isstruct(planner_cfg.robot)
        robot_cfg = planner_cfg.robot;
        return;
    end
    error('run_path_planner:MissingRobotConfig', ...
        ['Footprint-aware navigation requires the effective robot config. ' ...
         'Pass planner_cfg.robot_cfg (or planner_cfg.robot).']);
end

function enabled = footprint_navigation_enabled(planner_cfg)
    enabled = true;
    if isstruct(planner_cfg) && isfield(planner_cfg, 'footprint_aware_navigation')
        enabled = logical(planner_cfg.footprint_aware_navigation);
    end
end

%% ===================== 内部函数 =====================

function [path, search_state, search_diagnostics] = run_selected_algorithm(algorithm, grid_map, start_pose, goal_pose, map_info, planner_cfg, run_opts)
    search_state = [];
    search_diagnostics = struct();
    switch lower(algorithm)
        case {'convex_astar', 'convex_corner_astar'}
            path = run_convex_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg, run_opts);
        case 'astar'
            path = run_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'dijkstra'
            path = run_dijkstra_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'jps'
            path = run_jps_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'gbfs'
            path = run_gbfs_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'theta_star'
            path = run_theta_star_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'lazy_theta_star'
            path = run_lazy_theta_star_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'dstar'
            path = run_dstar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'lpa_star'
            path = run_lpa_star_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'voronoi'
            path = run_voronoi_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'dstar_lite'
            [path, search_state, search_diagnostics] = run_dstar_lite_wrapper( ...
                grid_map, start_pose, goal_pose, map_info, planner_cfg, run_opts);
        case 'hybrid_astar'
            path = run_hybrid_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'rrt'
            path = run_rrt_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'rrt_star'
            path = run_rrt_star_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'rrt_connect'
            path = run_rrt_connect_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'informed_rrt'
            path = run_informed_rrt_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'aco'
            path = run_aco_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'pso'
            path = run_pso_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        case 'ga'
            path = run_ga_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        otherwise
            warning('未知算法 "%s"，回退到 A*', algorithm);
            path = run_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
    end
end

function [path_world, is_valid, n_collision] = normalize_and_validate_path(path_in, grid_map, map_info, suppress_warn)
%NORMALIZE_AND_VALIDATE_PATH 统一路径后处理与碰撞校验。
    path_world = [];
    is_valid = false;
    n_collision = 0;

    if isempty(path_in) || size(path_in, 1) < 2
        return;
    end

    path_grid = zeros(size(path_in, 1), 2);
    for k = 1:size(path_in, 1)
        [pi, pj] = world2grid(path_in(k, 1), path_in(k, 2), ...
                              map_info.resolution, map_info.height_m);
        path_grid(k, :) = [pi, pj];
    end

    traversable = (grid_map == 0);
    [path_grid, repair_failed] = post_validate_path(path_grid, traversable);

    path_world = zeros(size(path_grid, 1), 3);
    for k = 1:size(path_grid, 1)
        [wx, wy] = grid2world(path_grid(k, 1), path_grid(k, 2), ...
                              map_info.resolution, map_info.height_m);
        path_world(k, :) = [wx, wy, 0];
    end

    if size(path_world, 1) >= 2
        path_world = densify_path(path_world, map_info.resolution);
        [col_ok, bad_idx] = validate_world_path_collision(grid_map, path_world, map_info, ~suppress_warn);
        is_valid = col_ok && ~repair_failed;
        if ~col_ok
            n_collision = size(bad_idx, 1);
        elseif repair_failed
            % A failed segment repair is a planning failure even when the
            % truncated prefix contains no collision cells.
            n_collision = 1;
        end
    end
end
