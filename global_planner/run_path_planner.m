function [path, fallback_info, search_state, search_diagnostics] = run_path_planner(grid_map, start_pose, goal_pose, map_info, planner_cfg, algo_override, run_opts)
%RUN_PATH_PLANNER Grid search followed by exact-pose, raw-map validation.
%   Grid snapping is an internal search operation. Successful returned paths
%   include collision-checked connectors to the exact requested SE(2) poses.

    if nargin < 7 || isempty(run_opts)
        run_opts = struct();
    end
    search_state = [];
    search_diagnostics = struct();
    if nargin < 6 || isempty(algo_override)
        algo_override = '';
    end
    start_pose = start_pose(:)';goal_pose = goal_pose(:)';
    if numel(start_pose)==2,start_pose(3)=0;end
    if numel(goal_pose)==2,goal_pose(3)=0;end
    if numel(start_pose)~=3 || numel(goal_pose)~=3 || ...
            any(~isfinite([start_pose goal_pose]))
        error('run_path_planner:Pose','Endpoints must be finite [x y theta].');
    end

    raw_map = grid_map;
    origin = map_origin(map_info);
    % Legacy algorithm wrappers search in map-local coordinates. Restore
    % the world origin before any endpoint or footprint validation.
    local_start = start_pose; local_goal = goal_pose;
    local_start(1:2) = start_pose(1:2)-reshape(origin,size(start_pose(1:2)));
    local_goal(1:2) = goal_pose(1:2)-reshape(origin,size(goal_pose(1:2)));
    % Keep the planner implementation unchanged while providing it a map on
    % which a center point is guaranteed to represent the full base footprint.
    % Candidate generation and task/dock validation continue using scene.grid_map.
    if footprint_navigation_enabled(planner_cfg)
        robot_cfg = get_robot_config(planner_cfg);
        [grid_map, inflation_info] = build_footprint_inflated_map( ...
            grid_map, robot_cfg.footprint, map_info);
        planner_cfg.footprint_inflation = inflation_info;
    end

    fallback_info = struct('fallback_used', false,'fallback_count',0, ...
        'collision_segments',0,'requested_planner','','actual_planner','', ...
        'actual_planner_known',true,'fallback_reason','','failure_reason','', ...
        'path_valid',false,'endpoint_validation',struct(), ...
        'postprocessing_repair_used',false);
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

    algorithm = lower(char(algorithm));
    fallback_info.requested_planner = algorithm;
    [fallback_info.actual_planner,fallback_info.actual_planner_known] = planner_identity(algorithm,planner_cfg);
    if strcmp(fallback_info.actual_planner,'astar') && ~strcmp(algorithm,'astar')
        fallback_info.fallback_used=true;fallback_info.fallback_count=1;
        fallback_info.fallback_reason='UNKNOWN_REQUESTED_PLANNER';
    end
    [raw_path, search_state, search_diagnostics] = run_selected_algorithm(algorithm, grid_map, local_start, local_goal, map_info, planner_cfg, run_opts);
    raw_path = restore_origin(raw_path,origin);
    if isstruct(search_diagnostics) && isfield(run_opts, 'changed_cells') && ...
            isnumeric(run_opts.changed_cells)
        search_diagnostics.changed_cell_count = size(run_opts.changed_cells, 1);
    end
    [path,is_valid,n_collision,endpoint_report,repaired] = validate_attempt( ...
        raw_path,grid_map,raw_map,start_pose,goal_pose,map_info,planner_cfg);
    fallback_info.postprocessing_repair_used=repaired;
    fallback_info.endpoint_validation=endpoint_report;

    % A colliding requested pose cannot be made reachable by snapping or by
    % running another grid search. Failed connectors may benefit from A*.
    if ~is_valid && endpoint_report.requested_poses_valid && ...
            ~strcmp(fallback_info.actual_planner, 'astar')
        fallback_info.fallback_used = true;
        fallback_info.fallback_count = fallback_info.fallback_count+1;
        fallback_info.collision_segments = n_collision;
        fallback_info.fallback_reason=endpoint_report.reason;

        raw_path = run_astar_wrapper(grid_map,local_start,local_goal,map_info,planner_cfg);
        raw_path = restore_origin(raw_path,origin);
        [path,is_valid,n_collision_after,endpoint_report,repaired_after] = validate_attempt( ...
            raw_path,grid_map,raw_map,start_pose,goal_pose,map_info,planner_cfg);
        fallback_info.actual_planner='astar';fallback_info.actual_planner_known=true;
        fallback_info.endpoint_validation=endpoint_report;
        fallback_info.postprocessing_repair_used=repaired_after;
        search_state = []; % The returned route cannot own the D* search state.
        if ~is_valid
            fallback_info.collision_segments = fallback_info.collision_segments+n_collision_after;
        end
    elseif ~is_valid
        fallback_info.collision_segments = n_collision;
    end

    if ~is_valid && endpoint_report.requested_poses_valid && ...
            footprint_navigation_enabled(planner_cfg) && fixed_heading_fallback_enabled(planner_cfg)
        previous_reason=endpoint_report.reason;
        [recovered,recovery_report]=run_fixed_heading_astar( ...
            raw_map,start_pose,goal_pose,map_info,planner_cfg);
        search_diagnostics.fixed_heading_recovery=recovery_report;
        if recovery_report.valid
            path=recovered;is_valid=true;endpoint_report=recovery_report;
            fallback_info.fallback_used=true;
            fallback_info.fallback_count=fallback_info.fallback_count+1;
            fallback_info.fallback_reason=previous_reason;
            fallback_info.actual_planner='fixed_heading_astar';
            fallback_info.actual_planner_known=true;
            fallback_info.endpoint_validation=endpoint_report;
            fallback_info.postprocessing_repair_used=false;
            search_state=[];
        end
    end
    fallback_info.path_valid=is_valid;
    if ~is_valid
        path=zeros(0,3);search_state=[];
        fallback_info.failure_reason=endpoint_report.reason;
        if ~suppress_warn
            warning('run_path_planner:collision', ...
                '%s: path rejected (%s; %d collision segments).', ...
                algorithm,endpoint_report.reason,fallback_info.collision_segments);
        end
    end
    if fallback_info.postprocessing_repair_used
        fallback_info.actual_planner=[fallback_info.actual_planner '+grid_postprocess'];
    end
    search_diagnostics.requested_planner=algorithm;
    search_diagnostics.actual_planner=fallback_info.actual_planner;
    search_diagnostics.actual_planner_known=fallback_info.actual_planner_known;
    search_diagnostics.fallback_used=fallback_info.fallback_used;
    search_diagnostics.fallback_reason=fallback_info.fallback_reason;
    search_diagnostics.path_found=is_valid && size(path,1)>=2;
    search_diagnostics.path_safe=is_valid;
    search_diagnostics.endpoint_validation=endpoint_report;
    search_diagnostics.path_length_m=NaN;
    if is_valid,search_diagnostics.path_length_m=compute_path_length(path);end
    search_diagnostics.path_length=search_diagnostics.path_length_m;
    search_diagnostics.search_state_valid=is_valid && isstruct(search_state) && ~isempty(search_state);
    search_diagnostics.fallback_collision_segments=fallback_info.collision_segments;
    if search_diagnostics.search_state_valid
        search_state.validated_path_safe=true;
        search_state.validated_navigation_endpoints=[start_pose(:)';goal_pose(:)'];
    end
end

function enabled=fixed_heading_fallback_enabled(cfg)
    enabled=true;
    if isfield(cfg,'navigation') && isfield(cfg.navigation,'fixed_heading_fallback')
        enabled=logical(cfg.navigation.fixed_heading_fallback);
    end
end

function [path,valid,n_collision,report,repaired]=validate_attempt(raw,search_map,raw_map,start_pose,goal_pose,info,cfg)
    [core,core_valid,n_collision,repaired]=normalize_and_validate_path(raw,search_map,info);
    [path,report]=complete_navigation_endpoints(core,start_pose,goal_pose,raw_map,info,cfg);
    valid=core_valid && report.valid;
    if ~core_valid && report.requested_poses_valid
        report.valid=false;report.reason='INVALID_GRID_CORE';
    end
    if ~valid
        path=zeros(0,3);
        if isfield(report.trajectory_validation,'bad_sample_index') && report.trajectory_validation.bad_sample_index>0
            n_collision=max(n_collision,1);
        end
    end
end

function [name,known]=planner_identity(name,cfg)
    known=true;
    names={'convex_astar','convex_corner_astar','astar','dijkstra','jps','gbfs','theta_star', ...
        'lazy_theta_star','dstar','lpa_star','voronoi','dstar_lite','hybrid_astar', ...
        'rrt','rrt_star','rrt_connect','informed_rrt','aco','pso','ga'};
    if ~any(strcmp(name,names)),name='astar';return;end
    if any(strcmp(name,{'convex_astar','convex_corner_astar'})) && ...
            isfield(cfg,'convex_astar') && isfield(cfg.convex_astar,'enable_fallback') && ...
            cfg.convex_astar.enable_fallback
        name='convex_astar_or_internal_fallback';known=false;
    end
end

function path=restore_origin(path,origin)
    if ~isempty(path),path(:,1:2)=path(:,1:2)+origin;end
end

function origin=map_origin(info)
    origin=[0 0];if isfield(info,'origin_xy'),origin=reshape(info.origin_xy,1,2);end
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
            if ~isfield(run_opts,'suppress_warnings') || ~run_opts.suppress_warnings
                warning('Unknown algorithm "%s"; falling back to A*.',algorithm);
            end
            path = run_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
    end
end

function [path_world, is_valid, n_collision, repaired] = normalize_and_validate_path(path_in, grid_map, map_info)
%NORMALIZE_AND_VALIDATE_PATH Validate the search core without hiding failure.
    path_world = [];
    is_valid = false;
    n_collision = 0;
    repaired=false;

    if isempty(path_in) || size(path_in,2)<2 || any(~isfinite(path_in(:)))
        return;
    end

    origin=map_origin(map_info);
    path_grid = zeros(size(path_in, 1), 2);
    for k = 1:size(path_in, 1)
        [pi, pj] = world2grid(path_in(k, 1), path_in(k, 2), ...
                              map_info.resolution, map_info.height_m,origin);
        path_grid(k, :) = [pi, pj];
    end

    traversable = (grid_map == 0);
    for k=1:size(path_grid,1)
        if ~collision_free(path_grid(max(1,k-1),:),path_grid(k,:),traversable)
            repaired=true;break;
        end
    end
    % The caller reports a failure only after all safe fallbacks are tried.
    [path_grid, repair_failed] = post_validate_path(path_grid, traversable, true);

    path_world = zeros(size(path_grid, 1), 3);
    for k = 1:size(path_grid, 1)
        [wx, wy] = grid2world(path_grid(k, 1), path_grid(k, 2), ...
                              map_info.resolution, map_info.height_m,origin);
        path_world(k, :) = [wx, wy, 0];
    end

    if isempty(path_grid),return;end
    col_ok=true;
    for k=1:size(path_grid,1)
        a=path_grid(max(1,k-1),:);b=path_grid(k,:);
        if ~collision_free(a,b,traversable),col_ok=false;n_collision=n_collision+1;end
    end
    is_valid=col_ok && ~repair_failed;
    if repair_failed,n_collision=max(n_collision,1);end
end
