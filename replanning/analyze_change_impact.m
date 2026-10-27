function impact = analyze_change_impact(planning_state, change_event)
%ANALYZE_CHANGE_IMPACT Identify local P5 invalidation scope.

    planning_state = normalize_planning_state(planning_state);

    impact = struct();
    impact.event_type = change_event.type;
    impact.total_regions = planning_state.scene.region_metadata.region_count;
    impact.original_docks = size(planning_state.plan.selected_poses, 1);
    impact.original_sparse_edges = nnz(triu(planning_state.stats.sparse_graph.adjacency, 1));
    impact.affected_regions = [];
    impact.affected_tables = [];
    impact.affected_tasks = [];
    impact.affected_docks = [];
    impact.invalidated_coverage_pairs = zeros(0, 2);
    impact.invalid_docks = [];
    impact.invalid_sparse_edges = zeros(0, 2);
    impact.candidate_refresh_regions = [];
    impact.path_refresh_edges = zeros(0, 2);

    switch upper(change_event.type)
        case 'ADD_LOCAL_OBSTACLE'
            footprint_radius = footprint_circumscribed_radius(planning_state);
            impact.footprint_radius_m = footprint_radius;
            impact.invalid_sparse_edges = find_edges_intersecting_obstacle( ...
                planning_state, change_event.center, change_event.radius_m + footprint_radius);
            impact.path_refresh_edges = impact.invalid_sparse_edges;
            impact.affected_regions = regions_for_edges(planning_state, impact.invalid_sparse_edges);
        case 'INVALIDATE_DOCK'
            impact.invalid_docks = unique(reshape(change_event.target_dock_index, 1, []), 'stable');
            impact.affected_regions = reshape(change_event.target_region, 1, []);
            impact.affected_tasks = [];
            impact.invalid_sparse_edges = zeros(0, 2);
            for di = impact.invalid_docks
                impact.affected_tasks = union(impact.affected_tasks, ...
                    covered_tasks_for_dock(planning_state, di), 'stable');
                if di <= numel(planning_state.plan.pose_to_region_map)
                    impact.affected_regions(end + 1) = planning_state.plan.pose_to_region_map(di); %#ok<AGROW>
                end
                impact.invalid_sparse_edges = [impact.invalid_sparse_edges; ...
                    edges_incident_to_dock(planning_state, di)]; %#ok<AGROW>
            end
            impact.affected_regions = unique(impact.affected_regions, 'stable');
            impact.candidate_refresh_regions = impact.affected_regions;
            impact.invalid_sparse_edges = unique(impact.invalid_sparse_edges, 'rows');
            impact.path_refresh_edges = impact.invalid_sparse_edges;
        case 'ADD_TASKS'
            impact.affected_regions = change_event.target_region;
            impact.affected_tasks = change_event.added_task_ids;
            impact.candidate_refresh_regions = impact.affected_regions;
        case 'TASK_COVERAGE_FAILURE'
            impact.invalidated_coverage_pairs = reshape(change_event.invalidated_coverage_pairs, [], 2);
            impact.affected_tasks = change_event.target_task_id;
            impact.affected_docks = change_event.target_dock_index;
            impact.affected_regions = change_event.target_region;
            impact.candidate_refresh_regions = impact.affected_regions;
            impact.invalid_sparse_edges = zeros(0, 2);
            if impact.affected_docks >= 1 && impact.affected_docks <= size(planning_state.plan.selected_poses, 1)
                impact.invalid_sparse_edges = edges_incident_to_dock(planning_state, impact.affected_docks);
            end
            impact.path_refresh_edges = impact.invalid_sparse_edges;
        otherwise
            error('analyze_change_impact:UnknownEvent', 'Unknown event type "%s".', change_event.type);
    end

    impact.affected_regions = unique(impact.affected_regions(impact.affected_regions > 0), 'stable');
    impact.affected_region_count = numel(impact.affected_regions);
    impact.affected_task_count = numel(unique(impact.affected_tasks, 'stable'));
    impact.invalid_dock_count = numel(unique(impact.invalid_docks, 'stable'));
    impact.invalid_edge_count = size(unique(sort(impact.invalid_sparse_edges, 2), 'rows'), 1);
    impact.region_locality_ratio = impact.affected_region_count / max(impact.total_regions, 1);
end

function edges = find_edges_intersecting_obstacle(state, center, radius)
    graph = state.stats.sparse_graph.adjacency;
    [ii, jj] = find(triu(graph, 1));
    edges = zeros(0, 2);
    for k = 1:numel(ii)
        path = [];
        if isfield(state.stats.sparse_graph, 'direct_paths')
            path = state.stats.sparse_graph.direct_paths{ii(k), jj(k)};
        end
        if isempty(path)
            path = state.all_poses([ii(k), jj(k)], 1:2);
        end
        affected = false;
        for p = 1:size(path, 1)-1
            if point_segment_distance(center, path(p, 1:2), path(p+1, 1:2)) <= radius
                affected = true;
                break;
            end
        end
        if affected
            edges(end + 1, :) = [ii(k), jj(k)]; %#ok<AGROW>
        end
    end
end

function radius = footprint_circumscribed_radius(state)
    radius = 0;
    if ~isfield(state, 'robot_cfg') || ~isstruct(state.robot_cfg) || ...
            ~isfield(state.robot_cfg, 'footprint')
        return;
    end
    fp = state.robot_cfg.footprint;
    if isfield(fp, 'length_m') && isfield(fp, 'width_m')
        radius = 0.5 * hypot(fp.length_m, fp.width_m);
    end
end

function d = point_segment_distance(p, a, b)
    ab = b - a;
    denom = dot(ab, ab);
    if denom < eps
        d = norm(p - a);
        return;
    end
    t = max(0, min(1, dot(p - a, ab) / denom));
    proj = a + t * ab;
    d = norm(p - proj);
end

function regions = regions_for_edges(state, edges)
    regions = [];
    ids = state.stats.sparse_graph.region_ids(:);
    for e = 1:size(edges, 1)
        nodes = edges(e, :);
        r = ids(nodes(nodes > 1));
        regions = [regions; r(:)]; %#ok<AGROW>
    end
end

function tasks = covered_tasks_for_dock(state, dock_idx)
    tasks = [];
    if dock_idx >= 1 && dock_idx <= numel(state.plan.pose_info_list)
        info = state.plan.pose_info_list{dock_idx};
        if isstruct(info) && isfield(info, 'covered_tasks')
            tasks = info.covered_tasks(:)';
        end
    end
end

function edges = edges_incident_to_dock(state, dock_idx)
    node = dock_idx + 1;
    graph = state.stats.sparse_graph.adjacency;
    nbrs = find(graph(node, :));
    edges = sort([repmat(node, numel(nbrs), 1), nbrs(:)], 2);
    edges = unique(edges, 'rows');
end
