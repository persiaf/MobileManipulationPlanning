function signature = build_path_cache_signature(grid_map, map_info, planner_cfg)
%BUILD_PATH_CACHE_SIGNATURE Hash all path-semantics inputs into a cache key.
    payload = struct();
    payload.geometry_revision = 'world_ccw_sat_exact_endpoints_20261027_v2';
    payload.grid = uint8(grid_map ~= 0);
    payload.resolution = map_info.resolution;
    payload.map_info = map_info;
    payload.planner = getfield_default(planner_cfg, 'global_planner', 'astar');
    payload.footprint_aware = getfield_default(planner_cfg, 'footprint_aware_navigation', true);
    payload.inflation_radius_m = getfield_default(planner_cfg, 'inflation_radius_m', NaN);
    robot = getfield_default(planner_cfg, 'robot_cfg', getfield_default(planner_cfg, 'robot', struct()));
    payload.footprint = getfield_default(robot, 'footprint', struct());
    payload.semantic_options = getfield_default(planner_cfg, 'semantic_options', struct());
    option_fields = {'astar','convex_astar','dijkstra','jps','gbfs','theta_star', ...
        'lazy_theta_star','dstar','lpa_star','voronoi','hybrid_astar','rrt', ...
        'rrt_star','rrt_connect','informed_rrt','aco','pso','ga'};
    payload.algorithm_options = struct();
    for i = 1:numel(option_fields)
        if isfield(planner_cfg, option_fields{i})
            payload.algorithm_options.(option_fields{i}) = planner_cfg.(option_fields{i});
        end
    end
    signature = build_struct_signature(payload);
end

function value = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name), value = s.(name); else, value = default_value; end
end
