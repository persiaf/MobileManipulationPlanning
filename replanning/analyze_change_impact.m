function impact = analyze_change_impact(planning_state, change_event)
%ANALYZE_CHANGE_IMPACT Identify local P5 invalidation scope.

    impact = struct();
    impact.event_type = change_event.type;
    impact.total_regions = planning_state.scene.region_metadata.region_count;
    impact.original_docks = size(planning_state.plan.selected_poses, 1);
    impact.original_sparse_edges = nnz(triu(planning_state.stats.sparse_graph.adjacency, 1));
    impact.affected_regions = [];
    impact.affected_tables = [];
    impact.affected_tasks = [];
    impact.invalid_docks = [];
    impact.invalid_sparse_edges = zeros(0, 2);
    impact.candidate_refresh_regions = [];
    impact.path_refresh_edges = zeros(0, 2);

    switch upper(change_event.type)
        case 'ADD_LOCAL_OBSTACLE'
            impact.invalid_sparse_edges = find_edges_intersecting_obstacle( ...
                planning_state, change_event.center, change_event.radius_m);
            impact.path_refresh_edges = impact.invalid_sparse_edges;
            impact.affected_regions = regions_for_edges(planning_state, impact.invalid_sparse_edges);
        case 'INVALIDATE_DOCK'
            impact.invalid_docks = change_event.target_dock_index;
            impact.affected_regions = change_event.target_region;
            impact.affected_tasks = covered_tasks_for_dock(planning_state, change_event.target_dock_index);
            impact.candidate_refresh_regions = impact.affected_regions;
            impact.invalid_sparse_edges = edges_incident_to_dock(planning_state, change_event.target_dock_index);
            impact.path_refresh_edges = impact.invalid_sparse_edges;
        case 'ADD_TASKS'
            impact.affected_regions = change_event.target_region;
            impact.affected_tasks = change_event.added_task_ids;
            impact.candidate_refresh_regions = impact.affected_regions;
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
        a = state.all_poses(ii(k), 1:2);
        b = state.all_poses(jj(k), 1:2);
        if point_segment_distance(center, a, b) <= radius
            edges(end + 1, :) = [ii(k), jj(k)]; %#ok<AGROW>
        end
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
