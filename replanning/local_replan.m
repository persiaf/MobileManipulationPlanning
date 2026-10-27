function [result, next_state, diagnostics] = local_replan(planning_state, changed_state, change_event, impact)
%LOCAL_REPLAN Resource-bounded region-aware local repair for P5.
%   The first output remains the historical result structure.  The optional
%   second output is the next immutable planning state; the original
%   planning_state.rc2_plan is never edited in place.

    planning_state = normalize_planning_state(planning_state);
    changed_state = normalize_planning_state(changed_state);
    if nargin < 4 || isempty(impact)
        impact = analyze_change_impact(planning_state, change_event);
    end
    t0 = tic;
    result = base_result(planning_state, changed_state, change_event, impact);
    next_state = changed_state;
    diagnostics = struct();

    switch upper(change_event.type)
        case 'ADD_LOCAL_OBSTACLE'
            [result, next_state, diagnostics] = repair_path_obstacle( ...
                result, planning_state, changed_state, change_event, impact);
        case 'INVALIDATE_DOCK'
            [result, next_state, diagnostics] = repair_dock_invalidation( ...
                result, planning_state, changed_state, change_event, impact);
        case 'ADD_TASKS'
            result = repair_added_tasks(result, planning_state, changed_state, impact);
        case 'TASK_COVERAGE_FAILURE'
            [result, next_state, diagnostics] = repair_task_coverage_failure( ...
                result, planning_state, changed_state, change_event, impact);
        otherwise
            error('local_replan:UnknownEvent', 'Unknown event type "%s".', change_event.type);
    end

    result.replanning_time_s = toc(t0);
    result.region_locality_ratio = result.affected_region_count / max(result.total_regions, 1);
    result.edge_recompute_ratio = result.recomputed_path_edges / max(result.original_sparse_edges, 1);
    result.dock_recompute_ratio = result.recomputed_or_changed_docks / max(result.original_docks, 1);
    result.unchanged_dock_ratio = result.retained_docks / max(result.original_docks, 1);
    result.reused_edge_ratio = result.reused_sparse_edges / max(result.original_sparse_edges, 1);
    result.next_state = next_state;
    result.diagnostics = diagnostics;
end

function result = base_result(state, changed_state, event, impact)
    result = struct();
    result.mode = 'LOCAL_REPLAN';
    result.event_type = event.type;
    result.status = 'SUCCESS';
    result.fallback = false;
    result.fallback_reason = '';
    result.total_regions = impact.total_regions;
    result.affected_region_count = impact.affected_region_count;
    result.affected_task_count = impact.affected_task_count;
    result.invalidated_docks = impact.invalid_dock_count;
    result.invalidated_edges = impact.invalid_edge_count;
    result.original_docks = size(state.plan.selected_poses, 1);
    result.original_sparse_edges = impact.original_sparse_edges;
    result.recomputed_candidates = 0;
    result.recomputed_docks = 0;
    result.recomputed_path_edges = 0;
    result.recomputed_or_changed_docks = 0;
    result.local_astar_calls = 0;
    result.raw_coverage = 100 * numel(state.plan.covered_tasks_predicted) / max(changed_state.scene.num_tasks, 1);
    result.feasible_task_coverage = 100;
    result.verified_completion = result.raw_coverage;
    result.final_path_length = state.stats.path_len;
    result.retained_docks = result.original_docks;
    result.removed_docks = 0;
    result.new_docks = 0;
    result.added_tasks = 0;
    result.model_feasible_added_tasks = 0;
    result.reused_sparse_edges = result.original_sparse_edges;
    result.sparse_graph_connected = true;
    result.no_collision_invalid_route = true;
    result.metric_provenance = 'legacy_p5_placeholder';
end

function [result, next_state, diagnostics] = repair_path_obstacle( ...
        result, state, changed_state, change_event, impact)
    diagnostics = struct('event_center', change_event.center, ...
        'event_radius_m', change_event.radius_m, 'rebuild_scope', 'selected_route_graph');
    next_state = changed_state;
    poses = state.all_poses;
    route_order = state.stats.route_order;
    was_path_safe = false;
    if isfield(state.stats, 'global_path') && ~isempty(state.stats.global_path)
        old_path = densify_path(state.stats.global_path, changed_state.scene.map_info.resolution);
        [was_path_safe, ~] = validate_repaired_path(old_path, ...
            state.stats.segment_stats, changed_state.scene, state, state.stats.route_order);
    end
    diagnostics.old_path_safe_on_changed_map = was_path_safe;
    result.raw_coverage = predicted_coverage_percent(state.plan, changed_state.scene.num_tasks);
    result.feasible_task_coverage = result.raw_coverage;
    result.verified_completion = verified_completion_percent(state, changed_state.scene.num_tasks);
    result.metric_provenance = 'predicted_coverage_and_verified_task_state';

    if isempty(impact.invalid_sparse_edges) && isequal(changed_state.scene.grid_map, state.scene.grid_map)
        result.status = 'NOOP';
        result.final_path_length = compute_path_length(state.stats.global_path);
        result.global_path = state.stats.global_path;
        result.path_segments = state.stats.segment_stats;
        result.updated_sparse_graph = state.stats.sparse_graph;
        result.no_collision_invalid_route = true;
        diagnostics.path_validation = struct('valid', true, ...
            'path_exists', true, 'map_collision_free', true, ...
            'footprint_checked', true, 'footprint_collision_free', true, ...
            'world_grid_consistent', true, ...
            'segments_continuous', true, 'reason', 'unchanged safe path');
        next_state = update_path_state(changed_state, state.stats.sparse_graph, ...
            state.stats.global_path, state.stats.segment_stats, route_order, ...
            diagnostics.path_validation);
        return;
    end

    % Re-query every candidate edge on the changed map.  The selected pose set,
    % task assignment, and visit order remain fixed in C2-1; only navigation
    % data are replaced.  Clearing the run-local cache prevents reuse of a path
    % computed on the pre-event map.
    planner_cfg = repair_planner_config(state);
    planner_cfg = disable_repair_cache(planner_cfg);
    region_ids = route_region_ids(state, size(poses, 1));
    if all(changed_state.scene.grid_map(:) > 0)
        result.status = 'FALLBACK_REQUIRED';
        result.fallback = true;
        result.fallback_reason = 'changed map contains no traversable cells';
        result.final_path_length = NaN;
        result.sparse_graph_connected = false;
        result.no_collision_invalid_route = false;
        next_state = attach_failed_path_state(next_state, ...
            field_or(state.stats, 'sparse_graph', struct()), result.fallback_reason);
        diagnostics.path_validation = struct('valid', false, ...
            'reason', result.fallback_reason);
        return;
    end
    try
        if incremental_replanning_enabled(planner_cfg)
            [graph, incremental_diagnostics] = rebuild_affected_sparse_edges_incremental( ...
                state, changed_state, planner_cfg, impact, region_ids);
            diagnostics.incremental_dstar = incremental_diagnostics;
        else
            graph = rebuild_affected_sparse_edges(state, changed_state, planner_cfg, ...
                impact, region_ids);
        end
    catch err
        result.status = 'FALLBACK_REQUIRED';
        result.fallback = true;
        result.fallback_reason = ['path graph rebuild failed: ' err.message];
        result.final_path_length = NaN;
        next_state = attach_failed_path_state(next_state, ...
            field_or(state.stats, 'sparse_graph', struct()), result.fallback_reason);
        result.sparse_graph_connected = false;
        return;
    end
    result.recomputed_path_edges = route_affected_edge_count(route_order, impact.invalid_sparse_edges);
    result.local_astar_calls = graph.stats.actual_astar_calls;
    result.reused_sparse_edges = max(result.original_sparse_edges - impact.invalid_edge_count, 0);
    result.sparse_graph_connected = graph.connectivity.ok;
    diagnostics.rebuilt_graph = graph;
    diagnostics.invalid_edges_before_rebuild = impact.invalid_sparse_edges;

    if ~graph.connectivity.ok
        result.status = 'FALLBACK_REQUIRED';
        result.fallback = true;
        result.fallback_reason = 'changed map cannot connect all selected route nodes';
        result.final_path_length = NaN;
        result.no_collision_invalid_route = false;
        next_state = attach_failed_path_state(next_state, graph, result.fallback_reason);
        return;
    end

    try
        [global_path, path_segments] = stitch_sparse_graph_path(poses, route_order, graph);
    catch err
        result.status = 'FALLBACK_REQUIRED';
        result.fallback = true;
        result.fallback_reason = ['route stitching failed: ' err.message];
        result.final_path_length = NaN;
        result.no_collision_invalid_route = false;
        next_state = attach_failed_path_state(next_state, graph, result.fallback_reason);
        return;
    end
    if field_or(state, 'unified_action_validation', false)
        candidate = changed_state;
        candidate.stats.sparse_graph = graph;
        candidate.stats.route_order = route_order;
        candidate = complete_replanning_route(candidate);
        graph = candidate.stats.sparse_graph;
        global_path = candidate.stats.global_path;
        path_segments = candidate.stats.segment_stats;
    end
    [valid, validation] = validate_repaired_path(global_path, path_segments, ...
        changed_state.scene, state, route_order);
    diagnostics.path_validation = validation;
    result.no_collision_invalid_route = valid;
    result.final_path_length = compute_path_length(global_path);
    result.global_path = global_path;
    result.path_segments = path_segments;
    result.updated_sparse_graph = graph;
    if ~valid || isempty(global_path) || size(global_path, 1) < 2
        result.status = 'FALLBACK_REQUIRED';
        result.fallback = true;
        result.fallback_reason = validation.reason;
        result.final_path_length = NaN;
        next_state = attach_failed_path_state(next_state, graph, result.fallback_reason);
        return;
    end

    result.status = ternary(was_path_safe, 'NOOP', 'SUCCESS');
    next_state = update_path_state(next_state, graph, global_path, path_segments, ...
        route_order, validation);
    diagnostics.next_state_version = next_state.path_repair_version;
end

function graph = rebuild_affected_sparse_edges(state, changed_state, planner_cfg, impact, region_ids)
    old = state.stats.sparse_graph;
    graph = old;
    graph.region_ids = region_ids;
    if ~isfield(graph, 'adjacency') || ~isfield(graph, 'cost_matrix')
        error('local_replan:MissingSparseGraph', 'Current state has no sparse graph adjacency/cost.');
    end
    n = size(graph.adjacency, 1);
    if ~isfield(graph, 'direct_paths') || ~iscell(graph.direct_paths)
        graph.direct_paths = cell(n, n);
    end
    stats = struct('actual_path_queries', 0, 'actual_astar_calls', 0, ...
        'successful_edges', nnz(triu(graph.adjacency, 1)), 'failed_edges', 0, ...
        'adaptive_bridge_edges_added', 0);
    edges = unique(sort(impact.invalid_sparse_edges, 2), 'rows');
    for e = 1:size(edges, 1)
        i = edges(e, 1); j = edges(e, 2);
        graph.adjacency(i, j) = false; graph.adjacency(j, i) = false;
        graph.cost_matrix(i, j) = inf; graph.cost_matrix(j, i) = inf;
        graph.direct_paths{i, j} = []; graph.direct_paths{j, i} = [];
        [path, cost, ok] = query_path_with_shared_cache(...
            changed_state.scene.grid_map, state.all_poses(i, :), state.all_poses(j, :), ...
            changed_state.scene.map_info, planner_cfg);
        stats.actual_path_queries = stats.actual_path_queries + 1;
        stats.actual_astar_calls = stats.actual_astar_calls + 1;
        edge_segment = struct('from_id', i, 'to_id', j, ...
            'num_points', size(path, 1), 'path', path);
        [safe, ~] = validate_repaired_path(path, edge_segment, changed_state.scene, ...
            state, [i, j]);
        if ok && safe && isfinite(cost) && ~isempty(path) && size(path, 1) >= 2
            graph.adjacency(i, j) = true; graph.adjacency(j, i) = true;
            graph.cost_matrix(i, j) = cost; graph.cost_matrix(j, i) = cost;
            graph.direct_paths{i, j} = path; graph.direct_paths{j, i} = flipud(path);
        else
            stats.failed_edges = stats.failed_edges + 1;
        end
    end
    % Legacy fixtures may contain only adjacency/cost data.  Populate missing
    % direct paths once so the route stitcher has concrete geometry; real RC2
    % states already carry these paths and therefore reuse them unchanged.
    [missing_i, missing_j] = find(triu(graph.adjacency, 1));
    for e = 1:numel(missing_i)
        i = missing_i(e); j = missing_j(e);
        if ~isempty(graph.direct_paths{i, j}), continue; end
        [path, cost, ok] = query_path_with_shared_cache(...
            changed_state.scene.grid_map, state.all_poses(i, :), state.all_poses(j, :), ...
            changed_state.scene.map_info, planner_cfg);
        stats.actual_path_queries = stats.actual_path_queries + 1;
        stats.actual_astar_calls = stats.actual_astar_calls + 1;
        if ok && isfinite(cost) && ~isempty(path) && size(path, 1) >= 2
            graph.direct_paths{i, j} = path;
            graph.direct_paths{j, i} = flipud(path);
        end
    end
    [graph.shortest_cost_matrix, graph.next_hop] = local_all_pairs_shortest_paths(graph.cost_matrix);
    graph.connectivity = validate_sparse_graph_connectivity(graph.adjacency, region_ids);
    graph.stats = stats;
    graph.stats.validated_sparse_edges = nnz(triu(graph.adjacency, 1));
end

function [graph, diagnostics] = rebuild_affected_sparse_edges_incremental(state, changed_state, planner_cfg, impact, region_ids)
%REBUILD_AFFECTED_SPARSE_EDGES_INCREMENTAL Update only affected D* Lite edges.
    graph = state.stats.sparse_graph;
    graph.region_ids = region_ids;
    n = size(graph.adjacency, 1);
    if ~isfield(graph, 'direct_paths') || ~iscell(graph.direct_paths), graph.direct_paths = cell(n, n); end
    previous = field_or(state.stats, 'path_search_states', struct('keys', {{}}, 'states', {{}}));
    keys = field_or(previous, 'keys', {}); states = field_or(previous, 'states', {});
    if ~iscell(keys), keys = {}; end; if ~iscell(states), states = {}; end
    changed_cells = changed_cells_between(state.scene.grid_map, changed_state.scene.grid_map);
    edges = unique(sort(impact.invalid_sparse_edges, 2), 'rows');
    diagnostics = struct('incremental_reuse', false, 'reinitialized', false, 'changed_cell_count', size(changed_cells, 1), ...
        'affected_edge_count', size(edges, 1), 'updated_vertex_count', 0, 'expanded_node_count', 0, ...
        'queue_operation_count', 0, 'search_time_ms', 0, 'fallback_used', false, 'fallback_reason', '', ...
        'incremental_success_edge_count', 0, 'fallback_edge_count', 0, 'reused_edge_count', 0, ...
        'failed_edge_count', 0, 'path_safe', true, 'full_route_incremental_success', false, ...
        'edge_results', struct('key', {}, 'from_id', {}, 'to_id', {}, 'state_loaded', {}, ...
            'incremental_reuse', {}, 'reinitialized', {}, 'fallback_used', {}, ...
            'actual_planner', {}, 'path_found', {}, 'path_safe', {}, 'state_valid', {}, ...
            'previous_state_version', {}, 'state_version', {}, 'changed_cell_count', {}, ...
            'path_points', {}, 'validation', {}, 'dstar_path_found', {}, ...
            'dstar_path_safe', {}, 'dstar_actual_planner', {}, ...
            'dstar_changed_cell_count', {}, 'dstar_search_state_valid', {}, ...
            'fallback_validation', {}));
    for e = 1:size(edges, 1)
        i = edges(e, 1); j = edges(e, 2); key = stable_sparse_edge_key(state, i, j); old_state = []; idx = find(strcmp(keys, key), 1);
        if isempty(idx)
            legacy_key = sprintf('%d>%d', i, j);
            idx = find(strcmp(keys, legacy_key), 1);
            if ~isempty(idx), key = legacy_key; end
        end
        if ~isempty(idx), old_state = states{idx}; end
        state_loaded = isstruct(old_state) && isfield(old_state, 'g') && ...
            isfield(old_state, 'rhs') && isfield(old_state, 'open_list');
        previous_version = NaN;
        if state_loaded, previous_version = field_or(old_state, 'state_version', NaN); end
        opts = struct('incremental_mode', true, 'dstar_state', old_state, 'changed_cells', changed_cells);
        [path, ~, new_state, d] = run_path_planner(changed_state.scene.grid_map, state.all_poses(i, :), ...
            state.all_poses(j, :), changed_state.scene.map_info, planner_cfg, 'dstar_lite', opts);
        edge_segment = struct('from_id', i, 'to_id', j, 'num_points', size(path, 1), 'path', path);
        [safe, validation] = validate_repaired_path(path, edge_segment, changed_state.scene, state, [i, j]);
        dstar_path_found = field_or(d, 'path_found', false);
        dstar_path_safe = field_or(d, 'path_safe', false);
        dstar_actual_planner = field_or(d, 'actual_planner', 'dstar_lite');
        dstar_changed_cell_count = field_or(d, 'changed_cell_count', 0);
        dstar_search_state_valid = field_or(d, 'search_state_valid', false);
        fallback_validation = struct();
        actual_planner = field_or(d, 'actual_planner', 'dstar_lite');
        if ~d.path_found || ~d.path_safe || ~safe
            [fallback_path, ~] = run_path_planner(changed_state.scene.grid_map, state.all_poses(i, :), ...
                state.all_poses(j, :), changed_state.scene.map_info, planner_cfg, 'astar');
            fallback_segment = struct('from_id', i, 'to_id', j, ...
                'num_points', size(fallback_path, 1), 'path', fallback_path);
            [fallback_safe, fallback_validation] = validate_repaired_path(fallback_path, fallback_segment, ...
                changed_state.scene, state, [i, j]);
            d.fallback_used = true;
            d.fallback_reason = 'D* Lite path missing or failed full path safety validation';
            d.actual_planner = 'astar';
            d.search_state_valid = false;
            d.path_found = fallback_safe && ~isempty(fallback_path) && size(fallback_path, 1) >= 2;
            d.path_safe = fallback_safe;
            actual_planner = 'astar';
            if fallback_safe, path = fallback_path; safe = true; else, path = fallback_path; safe = false; end
        end
        state_valid = strcmpi(actual_planner, 'dstar_lite') && safe && ...
            field_or(d, 'search_state_valid', false) && isstruct(new_state);
        if state_valid
            new_state.validated_path_safe = true;
            new_state.search_state_valid = true;
            if isempty(idx), keys{end + 1} = key; states{end + 1} = new_state; else, states{idx} = new_state; end
        elseif ~isempty(idx)
            keys(idx) = []; states(idx) = [];
        end
        diagnostics.incremental_success_edge_count = diagnostics.incremental_success_edge_count + ...
            (strcmpi(actual_planner, 'dstar_lite') && safe && state_valid);
        diagnostics.fallback_edge_count = diagnostics.fallback_edge_count + strcmpi(actual_planner, 'astar');
        diagnostics.failed_edge_count = diagnostics.failed_edge_count + ~safe;
        diagnostics.path_safe = diagnostics.path_safe && safe;
        previous_key_version = previous_version;
        next_version = NaN;
        if isstruct(new_state) && isfield(new_state, 'state_version'), next_version = new_state.state_version; end
        diagnostics.edge_results(end + 1) = struct('key', key, 'from_id', i, 'to_id', j, ...
            'state_loaded', state_loaded, 'incremental_reuse', field_or(d, 'incremental_reuse', false), ...
            'reinitialized', field_or(d, 'reinitialized', false), 'fallback_used', strcmpi(actual_planner, 'astar'), ...
            'actual_planner', actual_planner, 'path_found', field_or(d, 'path_found', false), ...
            'path_safe', safe, 'state_valid', state_valid, 'previous_state_version', previous_key_version, ...
            'state_version', next_version, 'changed_cell_count', field_or(d, 'changed_cell_count', 0), ...
            'path_points', size(path, 1), 'validation', validation, ...
            'dstar_path_found', dstar_path_found, 'dstar_path_safe', dstar_path_safe, ...
            'dstar_actual_planner', dstar_actual_planner, ...
            'dstar_changed_cell_count', dstar_changed_cell_count, ...
            'dstar_search_state_valid', dstar_search_state_valid, ...
            'fallback_validation', fallback_validation);
        diagnostics.incremental_reuse = diagnostics.incremental_reuse || d.incremental_reuse;
        diagnostics.fallback_used = diagnostics.fallback_used || field_or(d, 'fallback_used', false);
        if field_or(d, 'fallback_used', false), diagnostics.fallback_reason = field_or(d, 'fallback_reason', ''); end
        diagnostics.reinitialized = diagnostics.reinitialized || d.reinitialized;
        diagnostics.changed_cell_count = max(diagnostics.changed_cell_count, d.changed_cell_count);
        diagnostics.updated_vertex_count = diagnostics.updated_vertex_count + d.updated_vertex_count;
        diagnostics.expanded_node_count = diagnostics.expanded_node_count + d.expanded_node_count;
        diagnostics.queue_operation_count = diagnostics.queue_operation_count + d.queue_operation_count;
        diagnostics.search_time_ms = diagnostics.search_time_ms + d.search_time_ms;
        cost = compute_path_length(path);
        graph.adjacency(i, j) = false; graph.adjacency(j, i) = false; graph.cost_matrix(i, j) = inf; graph.cost_matrix(j, i) = inf;
        graph.direct_paths{i, j} = []; graph.direct_paths{j, i} = [];
        if safe && isfinite(cost) && size(path, 1) >= 2
            graph.adjacency(i, j) = true; graph.adjacency(j, i) = true; graph.cost_matrix(i, j) = cost; graph.cost_matrix(j, i) = cost;
            graph.direct_paths{i, j} = path; graph.direct_paths{j, i} = flipud(path);
        elseif isempty(diagnostics.fallback_reason)
            diagnostics.fallback_reason = 'incremental search produced no safe edge';
        end
    end
    [graph.shortest_cost_matrix, graph.next_hop] = local_all_pairs_shortest_paths(graph.cost_matrix);
    graph.connectivity = validate_sparse_graph_connectivity(graph.adjacency, region_ids);
    graph.path_search_states = struct('keys', {keys}, 'states', {states});
    diagnostics.reused_edge_count = max(size(edges, 1) - diagnostics.fallback_edge_count - diagnostics.failed_edge_count, 0);
    diagnostics.full_route_incremental_success = size(edges, 1) > 0 && ...
        diagnostics.incremental_success_edge_count == size(edges, 1) && ...
        diagnostics.fallback_edge_count == 0 && diagnostics.failed_edge_count == 0;
end

function key = stable_sparse_edge_key(state, i, j)
    endpoints = state.all_poses([i, j], :);
    key = sprintf('edge:%d:%d:%s', i, j, build_struct_signature(endpoints));
end

function tf = incremental_replanning_enabled(cfg)
    tf = isstruct(cfg) && isfield(cfg, 'incremental_replanning_enabled') && logical(cfg.incremental_replanning_enabled) && ...
        isfield(cfg, 'global_planner') && strcmpi(cfg.global_planner, 'dstar_lite');
end

function cells = changed_cells_between(old_map, new_map)
    [r, c] = find(old_map ~= new_map); cells = [r, c];
end

function cfg = repair_planner_config(state)
    if isfield(state, 'planner_cfg') && isstruct(state.planner_cfg)
        cfg = state.planner_cfg;
    else
        % Test fixtures from the original P5 scaffold do not carry effective
        % configuration.  This fallback is only for those fixtures; real RC2
        % states always use the stored effective planner configuration.
        cfg = struct('global_planner', 'astar', ...
            'footprint_aware_navigation', false, ...
            'astar', struct('allow_diagonal', true, 'heuristic_weight', 1.0));
    end
    if ~isfield(cfg, 'global_planner'), cfg.global_planner = 'astar'; end
    if ~isfield(cfg, 'astar') || ~isstruct(cfg.astar)
        cfg.astar = struct('allow_diagonal', true, 'heuristic_weight', 1.0);
    end
    if ~isfield(cfg.astar, 'allow_diagonal'), cfg.astar.allow_diagonal = true; end
    if ~isfield(cfg.astar, 'heuristic_weight'), cfg.astar.heuristic_weight = 1.0; end
end

function cfg = disable_repair_cache(cfg)
    if ~isfield(cfg, 'runtime') || ~isstruct(cfg.runtime)
        cfg.runtime = struct();
    end
    cfg.runtime.shared_path_cost_context = [];
end

function ids = route_region_ids(state, n)
    ids = ones(n, 1);
    ids(1) = 0;
    if isfield(state, 'stats') && isfield(state.stats, 'sparse_graph') && ...
            isfield(state.stats.sparse_graph, 'region_ids') && ...
            numel(state.stats.sparse_graph.region_ids) == n
        ids = reshape(state.stats.sparse_graph.region_ids, [], 1);
    elseif isfield(state, 'plan') && isfield(state.plan, 'pose_to_region_map') && ...
            numel(state.plan.pose_to_region_map) == n - 1
        ids(2:end) = reshape(state.plan.pose_to_region_map, [], 1);
    end
end

function [ok, validation] = validate_repaired_path(global_path, segments, scene, state, route_order)
    validation = struct('valid', false, 'map_collision_free', false, 'footprint_checked', false, ...
        'footprint_collision_free', true, 'segments_continuous', false, ...
        'world_grid_consistent', false, 'path_exists', false, 'reason', '');
    if isempty(global_path) || size(global_path, 1) < 2
        validation.reason = 'repaired path is empty';
        ok = false;
        return;
    end
    validation.path_exists = true;
    validation.world_grid_consistent = world_grid_roundtrip_ok(global_path, scene.map_info);
    [validation.map_collision_free, ~] = validate_world_path_collision( ...
        scene.grid_map, global_path, scene.map_info, false);
    validation.segments_continuous = segments_are_continuous(segments, route_order);

    footprint = [];
    if isfield(state, 'robot_cfg') && isstruct(state.robot_cfg) && ...
            isfield(state.robot_cfg, 'footprint')
        footprint = state.robot_cfg.footprint;
    end
    if ~isempty(footprint)
        validation.footprint_checked = true;
        for k = 1:size(global_path, 1)
            [collision, ~] = check_robot_footprint_collision(global_path(k, :), ...
                footprint, scene);
            if collision
                validation.footprint_collision_free = false;
                break;
            end
        end
    end
    ok = validation.path_exists && validation.map_collision_free && ...
        validation.world_grid_consistent && validation.segments_continuous && validation.footprint_checked && ...
        validation.footprint_collision_free;
    validation.valid = ok;
    if ~ok
        if ~validation.map_collision_free
            validation.reason = 'repaired path collides with changed map';
        elseif ~validation.footprint_collision_free
            validation.reason = 'repaired path violates robot footprint safety';
        elseif ~validation.footprint_checked
            validation.reason = 'effective robot footprint is unavailable';
        elseif ~validation.world_grid_consistent
            validation.reason = 'world/grid coordinate round-trip failed';
        elseif ~validation.segments_continuous
            validation.reason = 'repaired path segments are disconnected';
        end
    else
        validation.reason = 'validated';
    end
end

function ok = segments_are_continuous(segments, route_order)
    ok = ~isempty(segments) && numel(segments) == max(numel(route_order) - 1, 0);
    if ~ok, return; end
    for k = 1:numel(segments)
        if ~isfield(segments(k), 'num_points') || segments(k).num_points < 2
            ok = false;
            return;
        end
        if segments(k).from_id ~= route_order(k) || segments(k).to_id ~= route_order(k + 1)
            ok = false;
            return;
        end
        if k > 1 && isfield(segments(k - 1), 'path') && isfield(segments(k), 'path')
            if norm(segments(k - 1).path(end, 1:2) - segments(k).path(1, 1:2)) > 1e-9
                ok = false;
                return;
            end
        end
    end
end

function ok = world_grid_roundtrip_ok(path, map_info)
    ok = true;
    tolerance = map_info.resolution * sqrt(2) / 2 + 1e-9;
    for k = 1:size(path, 1)
        [i, j] = world2grid(path(k, 1), path(k, 2), ...
            map_info.resolution, map_info.height_m);
        [x, y] = grid2world(i, j, map_info.resolution, map_info.height_m);
        if hypot(x - path(k, 1), y - path(k, 2)) > tolerance
            ok = false;
            return;
        end
    end
end

function state = update_path_state(state, graph, global_path, segments, route_order, validation)
    if ~isfield(state, 'stats') || ~isstruct(state.stats), state.stats = struct(); end
    state.stats.sparse_graph = graph;
    state.stats.route_order = route_order;
    state.stats.global_path = global_path;
    state.stats.segment_stats = segments;
    state.stats.path_len = compute_path_length(global_path);
    if isfield(graph, 'path_search_states')
        state.stats.path_search_states = graph.path_search_states;
    end
    state.global_path = global_path;
    state.path_segments = segments;
    state.sparse_graph = graph;
    state.route_order = route_order;
    state.path_validation = validation;
    state.current_plan = struct('selected_docks', state.plan.selected_poses, ...
        'task_assignment', {state.plan.task_sequence_summary}, ...
        'route_order', route_order, 'global_path', global_path, ...
        'path_segments', segments, 'sparse_graph', graph, ...
        'covered_tasks_predicted', state.plan.covered_tasks_predicted);
    if isfield(state, 'runtime_state') && isstruct(state.runtime_state)
        state.runtime_state.global_path = global_path;
        state.runtime_state.path_segments = segments;
        state.runtime_state.route_order = route_order;
    end
    state.path_repair_version = build_struct_signature(struct( ...
        'route_order', route_order, 'path', global_path, ...
        'map', state.scene.grid_map));
end

function state = attach_failed_path_state(state, graph, reason)
    if ~isfield(state, 'stats') || ~isstruct(state.stats), state.stats = struct(); end
    state.stats.sparse_graph = graph;
    state.sparse_graph = graph;
    state.path_validation = struct('valid', false, 'reason', reason);
    state.path_repair_version = 'FAILED';
end

function value = field_or(s, name, fallback)
    if isstruct(s) && isfield(s, name), value = s.(name); else, value = fallback; end
end

function value = predicted_coverage_percent(plan, task_count)
    covered = [];
    if isstruct(plan) && isfield(plan, 'covered_tasks_predicted')
        covered = plan.covered_tasks_predicted;
    end
    value = 100 * numel(unique(covered)) / max(task_count, 1);
end

function value = verified_completion_percent(state, task_count)
    verified = [];
    if isfield(state, 'task_state') && isstruct(state.task_state) && ...
            isfield(state.task_state, 'verified_tasks')
        verified = state.task_state.verified_tasks;
    elseif isfield(state, 'scene') && isfield(state.scene, 'tasks')
        statuses = [state.scene.tasks.status];
        verified = find(statuses ~= 0);
    end
    value = 100 * numel(unique(verified)) / max(task_count, 1);
end

function count = route_affected_edge_count(route_order, invalid_edges)
    count = 0;
    invalid = unique(sort(invalid_edges, 2), 'rows');
    for k = 1:numel(route_order)-1
        if ismember(sort([route_order(k), route_order(k+1)]), invalid, 'rows')
            count = count + 1;
        end
    end
end

function value = ternary(condition, yes, no)
    if condition, value = yes; else, value = no; end
end

function [result, next_state, diagnostics] = repair_dock_invalidation( ...
        result, state, changed_state, event, impact)
%REPAIR_DOCK_INVALIDATION Replace one invalid dock with a checked candidate.
%   The original RC2 plan remains immutable.  A replacement is committed only
%   after candidate reachability, graph routing, footprint safety, and state
%   consistency all pass.

    diagnostics = struct('candidate_trials', 0, 'candidate_rejections', {{}}, ...
        'selected_candidate_index', [], 'affected_tasks', impact.affected_tasks, ...
        'pending_affected_tasks', [], 'uncovered_task_ids', [], 'path_validation', struct());
    next_state = changed_state;
    invalid = unique(reshape(impact.invalid_docks, 1, []), 'stable');
    result.removed_docks = numel(invalid);
    result.retained_docks = max(result.original_docks - result.removed_docks, 0);
    result.recomputed_candidates = 0;
    result.recomputed_docks = 0;
    result.new_docks = 0;
    result.recomputed_or_changed_docks = 0;
    result.recomputed_path_edges = 0;
    result.local_astar_calls = 0;
    result.reused_sparse_edges = result.original_sparse_edges;
    result.metric_provenance = 'actual_candidate_path_and_verified_task_state';
    result.raw_coverage = predicted_coverage_percent(state.plan, changed_state.scene.num_tasks);
    result.feasible_task_coverage = result.raw_coverage;
    result.verified_completion = verified_completion_percent(state, changed_state.scene.num_tasks);
    result.final_path_length = NaN;

    if numel(invalid) ~= 1
        result.status = 'FALLBACK_REQUIRED'; result.fallback = true;
        result.fallback_reason = 'MULTIPLE_INVALID_DOCKS_UNSUPPORTED';
        return;
    end
    dock_idx = invalid(1);
    if dock_idx < 1 || dock_idx > size(state.plan.selected_poses, 1)
        result.status = 'FALLBACK_REQUIRED'; result.fallback = true;
        result.fallback_reason = 'INVALID_DOCK_INDEX';
        return;
    end

    completed = verified_task_ids(state);
    affected = unique(reshape(impact.affected_tasks, 1, []), 'stable');
    pending = setdiff(affected, completed, 'stable');
    diagnostics.pending_affected_tasks = pending;
    diagnostics.invalid_dock_index = dock_idx_from(invalid);
    result.no_collision_invalid_route = false;
    if isempty(pending)
        result.status = 'FALLBACK_REQUIRED';
        result.fallback = true;
        result.fallback_reason = 'INVALID_DOCK_HAS_NO_PENDING_TASKS';
        return;
    end

    region_id = replacement_region_id(state, dock_idx, event, impact);
    candidates = region_candidates(state.candidate_library, region_id);
    result.recomputed_candidates = numel(candidates);
    if isempty(candidates)
        result.status = 'FALLBACK_REQUIRED'; result.fallback = true;
        result.fallback_reason = 'NO_VALID_BACKUP_DOCK';
        return;
    end

    old_pose = state.plan.selected_poses(dock_idx, :);
    best = struct('found', false, 'candidate_index', [], 'graph', [], ...
        'path', [], 'segments', [], 'validation', struct(), 'summary', [], ...
        'pose_info', [], 'score', [inf, inf]);
    planner_cfg = repair_planner_config(state);
    planner_cfg = disable_repair_cache(planner_cfg);
    route_order = state.stats.route_order;
    for ci = 1:numel(candidates)
        diagnostics.candidate_trials = diagnostics.candidate_trials + 1;
        candidate = candidates(ci);
        if norm(candidate.pose(1:2) - old_pose(1:2)) < 1e-9 && ...
                abs(wrapToPiLocal(candidate.pose(3) - old_pose(3))) < 1e-9
            diagnostics.candidate_rejections{end+1} = 'candidate_is_invalidated_pose'; %#ok<AGROW>
            continue;
        end
        [covered, fresh_arm_map, reach_summary] = validate_candidate_reachability( ...
            candidate, pending, state, changed_state.scene);
        if ~isequal(covered, pending)
            diagnostics.uncovered_task_ids = union(diagnostics.uncovered_task_ids, ...
                setdiff(pending, covered, 'stable'), 'stable');
            diagnostics.candidate_rejections{end+1} = sprintf( ...
                'incomplete_coverage_candidate_%d', ci); %#ok<AGROW>
            continue;
        end
        other_docks = state.plan.selected_poses;
        other_docks(dock_idx, :) = [];
        if any(vecnorm(other_docks(:,1:2) - candidate.pose(1:2), 2, 2) < 1e-6)
            diagnostics.candidate_rejections{end+1} = sprintf( ...
                'duplicate_dock_candidate_%d', ci); %#ok<AGROW>
            continue;
        end
        if ~candidate_pose_is_safe(candidate, changed_state.scene, state.robot_cfg)
            diagnostics.candidate_rejections{end+1} = sprintf( ...
                'unsafe_candidate_%d', ci); %#ok<AGROW>
            continue;
        end

        poses = state.all_poses;
        poses(dock_idx + 1, :) = candidate.pose;
        region_ids = route_region_ids(state, size(poses, 1));
        try
            graph = build_sparse_docking_graph(poses, changed_state.scene.grid_map, ...
                changed_state.scene.map_info, planner_cfg, region_ids);
            if ~graph.connectivity.ok
                diagnostics.candidate_rejections{end+1} = sprintf( ...
                    'replacement_graph_disconnected_%d', ci); %#ok<AGROW>
                continue;
            end
            [path, segments] = stitch_sparse_graph_path(poses, route_order, graph);
            [valid, validation] = validate_repaired_path(path, segments, ...
                changed_state.scene, state, route_order);
            if ~valid
                diagnostics.candidate_rejections{end+1} = sprintf( ...
                    'replacement_path_invalid_%d', ci); %#ok<AGROW>
                continue;
            end
        catch err
            diagnostics.candidate_rejections{end+1} = ['candidate_' num2str(ci) ...
                '_path_error:' err.identifier]; %#ok<AGROW>
            continue;
        end
        candidate_cost = candidate_total_cost(candidate);
        score = [candidate_cost, compute_path_length(path)];
        score_better = score(1) < best.score(1) - 1e-12 || ...
            (abs(score(1) - best.score(1)) <= 1e-12 && score(2) < best.score(2) - 1e-12);
        score_tied = all(abs(score - best.score) <= 1e-12);
        if ~best.found || score_better || (score_tied && ci < best.candidate_index)
            [summary, info] = replacement_assignment(state, dock_idx, candidate, ...
                pending, intersect(affected, completed, 'stable'), fresh_arm_map, reach_summary);
            best = struct('found', true, 'candidate_index', ci, 'graph', graph, ...
                'path', path, 'segments', segments, 'validation', validation, ...
                'summary', summary, 'pose_info', info, 'score', score);
        end
    end

    if ~best.found
        result.recomputed_candidates = diagnostics.candidate_trials;
        result.status = 'FALLBACK_REQUIRED'; result.fallback = true;
        if ~isempty(diagnostics.uncovered_task_ids)
            result.fallback_reason = 'INCOMPLETE_AFFECTED_TASK_COVERAGE';
        else
            result.fallback_reason = 'NO_VALID_BACKUP_DOCK';
        end
        return;
    end

    next_state = commit_dock_replacement(next_state, state, dock_idx, ...
        best, changed_state.scene, route_order);
    diagnostics.selected_candidate_index = best.candidate_index;
    diagnostics.path_validation = best.validation;
    result.status = 'SUCCESS';
    result.fallback = false;
    result.removed_docks = 0;
    result.retained_docks = result.original_docks;
    result.recomputed_docks = 1;
    result.new_docks = 1;
    result.recomputed_or_changed_docks = 1;
    result.recomputed_path_edges = get_graph_query_count(best.graph);
    result.local_astar_calls = get_graph_query_count(best.graph);
    result.reused_sparse_edges = 0;
    result.recomputed_candidates = diagnostics.candidate_trials;
    result.global_path = best.path;
    result.path_segments = best.segments;
    result.updated_sparse_graph = best.graph;
    result.final_path_length = compute_path_length(best.path);
    result.no_collision_invalid_route = true;
    result.raw_coverage = predicted_coverage_percent(next_state.current_plan, changed_state.scene.num_tasks);
    result.feasible_task_coverage = result.raw_coverage;
    result.verified_completion = verified_completion_percent(state, changed_state.scene.num_tasks);
end

function [summary, info] = replacement_assignment(state, dock_idx, candidate, pending, completed, arm_map, reach_summary)
    summary = state.plan.task_sequence_summary{dock_idx};
    info = state.plan.pose_info_list{dock_idx};
    if ~isfield(info, 'arm_assign_map') || ~isa(info.arm_assign_map, 'containers.Map')
        info.arm_assign_map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    end
    for tid = reshape(completed, 1, [])
        if isKey(info.arm_assign_map, tid) && ~isKey(arm_map, tid)
            arm_map(tid) = info.arm_assign_map(tid);
        end
    end
    summary.pose = candidate.pose;
    summary.covered_tasks = reshape(union(pending, completed, 'stable'), 1, []);
    summary.covered_count = numel(summary.covered_tasks);
    if isfield(summary, 'task_actions')
        keep_tasks = union(pending, completed, 'stable');
        summary.task_actions = summary.task_actions(ismember([summary.task_actions.task_id], keep_tasks));
    end
    info.pose = candidate.pose;
    info.covered_tasks = summary.covered_tasks;
    info.covered_count = numel(summary.covered_tasks);
    info.side_id = field_or(candidate, 'side_id', 0);
    info.arm_assign_map = arm_map;
    summary.arm_assign_map = info.arm_assign_map;
    info.nominal_reachability_summary = reach_summary.nominal;
    info.robust_reachability_summary = reach_summary.robust;
    summary.mean_reachability_score = reach_summary.nominal.mean;
    summary.minimum_reachability_score = reach_summary.nominal.minimum;
    summary.robust_success_fraction = reach_summary.robust.mean_success_fraction;
    if isfield(summary, 'task_actions')
        for k = 1:numel(summary.task_actions)
            tid = summary.task_actions(k).task_id;
            if isKey(info.arm_assign_map, tid)
                summary.task_actions(k).assigned_arm = info.arm_assign_map(tid);
            end
        end
    end
    if isfield(summary, 'service_configs') && isfield(state.planner_cfg, 'real_platform')
        service = build_service_config(dock_idx, info, summary, state.planner_cfg.real_platform);
        summary.service_configs = expand_lift_candidates(service, state.planner_cfg.real_platform);
    end
end

function index = dock_idx_from(invalid)
    index = invalid(1);
end

function state = commit_dock_replacement(state, original, dock_idx, best, scene, route_order)
    state.scene = scene;
    state.tasks = scene.tasks;
    state.all_poses = original.all_poses;
    state.all_poses(dock_idx + 1, :) = best.summary.pose;
    state.plan.selected_poses(dock_idx, :) = best.summary.pose;
    state.plan.task_sequence_summary{dock_idx} = best.summary;
    state.plan.pose_info_list{dock_idx} = best.pose_info;
    state.plan.covered_tasks_predicted = collect_plan_covered(state.plan.task_sequence_summary);
    state.plan.uncovered_tasks_predicted = setdiff(all_scene_task_ids(scene), ...
        state.plan.covered_tasks_predicted, 'stable');
    state.plan.status = ternary(isempty(state.plan.uncovered_tasks_predicted), 'READY', 'BLOCKED');
    state.plan.reason = ternary(strcmp(state.plan.status, 'READY'), '', 'PARTIAL_TASK_COVERAGE');
    state.stats.sparse_graph = best.graph;
    state.stats.global_path = best.path;
    state.stats.segment_stats = best.segments;
    state.stats.route_order = route_order;
    state.stats.path_len = compute_path_length(best.path);
    state.global_path = best.path;
    state.path_segments = best.segments;
    state.sparse_graph = best.graph;
    state.route_order = route_order;
    state.path_validation = best.validation;
    state.current_plan = state.plan;
    state.current_plan.global_path = best.path;
    state.current_plan.path_segments = best.segments;
    state.current_plan.sparse_graph = best.graph;
    state.current_plan.route_order = route_order;
    state.current_plan.replacement_dock_index = dock_idx;
    state.current_plan.candidate_library = state.candidate_library;
    state.current_plan.task_state = state.task_state;
    state.current_plan.scene = scene;
    state.current_plan.path_validation = best.validation;
    state.path_repair_version = build_struct_signature(struct('selected_poses', ...
        state.plan.selected_poses, 'route_order', route_order, 'path', best.path, ...
        'map', scene.grid_map));
    state.current_plan.signatures = struct( ...
        'path_repair_version', state.path_repair_version, ...
        'plan_signature', build_struct_signature(struct( ...
            'selected_poses', state.plan.selected_poses, ...
            'covered_tasks_predicted', state.plan.covered_tasks_predicted, ...
            'route_order', route_order)), ...
        'map_signature', build_struct_signature(scene.grid_map));
    state.signatures = state.current_plan.signatures;
end

function tf = candidate_pose_is_safe(candidate, scene, robot_cfg)
    tf = isstruct(candidate) && isfield(candidate, 'pose') && ...
        isfield(robot_cfg, 'footprint') && ...
        ~check_robot_footprint_collision(candidate.pose, robot_cfg.footprint, scene);
end

function c = region_candidates(library, region_id)
    c = [];
    if ~isstruct(library) || ~isfield(library, 'regions') || ...
            region_id < 1 || region_id > numel(library.regions)
        return;
    end
    entry = library.regions(region_id);
    if isfield(entry, 'candidates'), c = entry.candidates; end
end

function rid = replacement_region_id(state, dock_idx, event, impact)
    rid = field_or(event, 'target_region', NaN);
    if ~isfinite(rid), rid = field_or(impact, 'affected_regions', NaN); end
    if numel(rid) ~= 1 || ~isfinite(rid)
        rid = state.plan.pose_to_region_map(dock_idx);
    end
end

function ids = verified_task_ids(state)
    ids = [];
    if isfield(state, 'task_state') && isstruct(state.task_state) && ...
            isfield(state.task_state, 'verified_tasks')
        ids = state.task_state.verified_tasks;
    elseif isfield(state, 'scene') && isfield(state.scene, 'tasks')
        indices = find([state.scene.tasks.status] ~= 0);
        for i = reshape(indices, 1, [])
            if isfield(state.scene.tasks(i), 'id')
                ids(end + 1) = state.scene.tasks(i).id; %#ok<AGROW>
            else
                ids(end + 1) = i; %#ok<AGROW>
            end
        end
    end
    ids = unique(ids, 'stable');
end

function ids = all_scene_task_ids(scene)
%ALL_SCENE_TASK_IDS Return stable task identifiers, not array positions.
    ids = [];
    if ~isstruct(scene) || ~isfield(scene, 'tasks') || isempty(scene.tasks)
        return;
    end
    for i = 1:numel(scene.tasks)
        if isfield(scene.tasks(i), 'id') && ~isempty(scene.tasks(i).id)
            ids(end + 1) = scene.tasks(i).id; %#ok<AGROW>
        else
            ids(end + 1) = i; %#ok<AGROW>
        end
    end
    ids = unique(ids, 'stable');
end

function [covered, arm_map, summary] = validate_candidate_reachability(candidate, task_ids, state, scene)
    covered = [];
    map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    nominal = [];
    robust_fraction = [];
    cfg = repair_planner_config(state);
    threshold = cfg.reachability.threshold;
    mode = lower(char(field_or(cfg.reachability, 'mode', 'nominal')));
    min_fraction = field_or(cfg.reachability, 'robust_success_fraction_min', 1);
    error_model = field_or(cfg, 'docking_error_model', struct());
    for k = 1:numel(task_ids)
        tid = task_ids(k);
        task_index = find_task_index(scene.tasks, tid);
        if isempty(task_index), continue; end
        task = scene.tasks(task_index);
        target = [task.x, task.y, task.z];
        [~, ~, score, arm] = evaluate_dual_arm_reachability( ...
            candidate.pose, target, state.robot_cfg, threshold);
        nominal(end+1) = score; %#ok<AGROW>
        ok = score > 0;
        if strcmp(mode, 'robust')
            rr = evaluate_robust_reachability(candidate.pose, target, ...
                state.robot_cfg, threshold, error_model);
            robust_fraction(end+1) = rr.success_fraction; %#ok<AGROW>
            ok = rr.success_fraction >= min_fraction;
        end
        if ok
            covered(end+1) = tid; %#ok<AGROW>
            map(tid) = arm;
        end
    end
    arm_map = map;
    if isempty(nominal), nominal_summary = struct('mean',NaN,'minimum',NaN);
    else, nominal_summary = struct('mean',mean(nominal),'minimum',min(nominal)); end
    if isempty(robust_fraction), robust_summary = struct('mean_success_fraction',NaN,'minimum_success_fraction',NaN);
    else, robust_summary = struct('mean_success_fraction',mean(robust_fraction), ...
            'minimum_success_fraction',min(robust_fraction)); end
    summary = struct('nominal',nominal_summary,'robust',robust_summary);
end

function index = find_task_index(tasks, task_id)
%FIND_TASK_INDEX Resolve stable task identifiers without assuming array order.
    index = [];
    has_explicit_ids = ~isempty(tasks) && isfield(tasks, 'id');
    for i = 1:numel(tasks)
        if isfield(tasks(i), 'id') && isequal(tasks(i).id, task_id)
            index = i;
            return;
        end
    end
    if ~has_explicit_ids && isnumeric(task_id) && isscalar(task_id) && task_id >= 1 && ...
            task_id <= numel(tasks) && task_id == floor(task_id)
        index = task_id;
    end
end

function value = candidate_total_cost(candidate)
    value = field_or(candidate, 'total_cost', inf);
    if ~isfinite(value) && isfield(candidate, 'cost_breakdown')
        value = field_or(candidate.cost_breakdown, 'total', inf);
    end
    if ~isfinite(value), value = inf; end
end

function n = get_graph_query_count(graph)
    n = 0;
    if isstruct(graph) && isfield(graph, 'stats') && isfield(graph.stats, 'actual_astar_calls')
        n = graph.stats.actual_astar_calls;
    end
end

function tasks = collect_plan_covered(summary)
    tasks = [];
    for i = 1:numel(summary)
        if isstruct(summary{i}) && isfield(summary{i}, 'covered_tasks')
            tasks = union(tasks, summary{i}.covered_tasks, 'stable');
        end
    end
end

function [result, next_state, diagnostics] = repair_task_coverage_failure( ...
        result, state, changed_state, event, impact)
%REPAIR_TASK_COVERAGE_FAILURE Reassign one pending task locally.
    next_state = changed_state;
    task_id = event.target_task_id;
    diagnostics = struct('target_task_id', task_id, ...
        'invalidated_coverage_pairs', impact.invalidated_coverage_pairs, ...
        'candidate_trials', 0, 'candidate_rejections', {{}}, ...
        'reassignment_level', '', 'path_validation', struct());
    invalid_pairs = impact.invalidated_coverage_pairs;
    result.metric_provenance = 'actual_task_reachability_and_verified_task_state';
    result.raw_coverage = predicted_coverage_percent(state.plan, changed_state.scene.num_tasks);
    result.feasible_task_coverage = result.raw_coverage;
    result.verified_completion = verified_completion_percent(state, changed_state.scene.num_tasks);
    if isempty(find_task_index(changed_state.scene.tasks, task_id))
        [result, next_state] = task_reallocation_failure(result, next_state, 'INVALID_TASK_ID');
        return;
    end
    if any(verified_task_ids(state) == task_id)
        result.status = 'NOOP';
        result.fallback = false;
        result.fallback_reason = 'TASK_ALREADY_VERIFIED';
        next_state.task_state = state.task_state;
        next_state.invalidated_coverage_pairs = invalid_pairs;
        diagnostics.reassignment_level = 'NONE';
        return;
    end
    old_dock = event.target_dock_index;
    if old_dock < 1 || old_dock > numel(state.plan.pose_info_list)
        [result, next_state] = task_reallocation_failure(result, next_state, 'INVALID_DOCK_INDEX');
        return;
    end
    old_info = state.plan.pose_info_list{old_dock};
    if ~coverage_pair_is_invalid(invalidated_pairs_or_event(invalid_pairs, event), task_id, old_dock)
        [result, next_state] = task_reallocation_failure(result, next_state, 'INVALID_COVERAGE_PAIR');
        return;
    end
    if ~isfield(old_info, 'covered_tasks') || ~any(old_info.covered_tasks == task_id)
        [result, next_state] = task_reallocation_failure(result, next_state, 'INVALID_COVERAGE_PAIR');
        return;
    end
    planner_cfg = disable_repair_cache(repair_planner_config(state));
    % Level 1: existing docks, preserving their poses and route.
    for d = 1:size(state.plan.selected_poses, 1)
        if d == old_dock || coverage_pair_is_invalid(invalid_pairs, task_id, d), continue; end
        diagnostics.candidate_trials = diagnostics.candidate_trials + 1;
        candidate = struct('pose', state.plan.selected_poses(d, :), ...
            'side_id', field_or(state.plan.pose_info_list{d}, 'side_id', 0), 'total_cost', 0);
        [covered, arm_map, reach_summary] = validate_candidate_reachability( ...
            candidate, task_id, state, changed_state.scene);
        if ~any(covered == task_id)
            diagnostics.candidate_rejections{end+1} = sprintf('existing_dock_%d_unreachable', d); %#ok<AGROW>
            continue;
        end
        [valid, validation] = validate_repaired_path(state.stats.global_path, ...
            state.stats.segment_stats, changed_state.scene, state, state.stats.route_order);
        diagnostics.path_validation = validation;
        if ~valid
            diagnostics.candidate_rejections{end+1} = 'existing_route_invalid'; %#ok<AGROW>
            break;
        end
        next_state = commit_task_reallocation(next_state, state, old_dock, d, task_id, ...
            arm_map, reach_summary, changed_state.scene, invalid_pairs);
        diagnostics.reassignment_level = 'EXISTING_DOCK';
        result.status = 'SUCCESS'; result.fallback = false;
        result.recomputed_docks = 0; result.new_docks = 0;
        result.recomputed_or_changed_docks = 0;
        result.recomputed_path_edges = 0; result.local_astar_calls = 0;
        result.reused_sparse_edges = result.original_sparse_edges;
        result.global_path = state.stats.global_path;
        result.path_segments = state.stats.segment_stats;
        result.final_path_length = compute_path_length(result.global_path);
        result.raw_coverage = predicted_coverage_percent(next_state.current_plan, changed_state.scene.num_tasks);
        result.feasible_task_coverage = result.raw_coverage;
        result.verified_completion = verified_completion_percent(state, changed_state.scene.num_tasks);
        return;
    end
    % Level 2: one new candidate from the affected region.
    region_id = event.target_region;
    candidates = region_candidates(state.candidate_library, region_id);
    for ci = 1:numel(candidates)
        diagnostics.candidate_trials = diagnostics.candidate_trials + 1;
        candidate = candidates(ci);
        if candidate_pose_is_safe(candidate, changed_state.scene, state.robot_cfg) == false
            diagnostics.candidate_rejections{end+1} = sprintf('unsafe_candidate_%d', ci); %#ok<AGROW>
            continue;
        end
        if any(vecnorm(state.plan.selected_poses(:,1:2) - candidate.pose(1:2), 2, 2) < 1e-6)
            diagnostics.candidate_rejections{end+1} = sprintf('duplicate_candidate_%d', ci); %#ok<AGROW>
            continue;
        end
        [covered, arm_map, reach_summary] = validate_candidate_reachability( ...
            candidate, task_id, state, changed_state.scene);
        if ~any(covered == task_id)
            diagnostics.candidate_rejections{end+1} = sprintf('candidate_%d_unreachable', ci); %#ok<AGROW>
            continue;
        end
        poses = [state.all_poses; candidate.pose];
        route_order = [state.stats.route_order, size(poses, 1)];
        region_ids = [route_region_ids(state, size(state.all_poses, 1)); region_id];
        try
            graph = build_sparse_docking_graph(poses, changed_state.scene.grid_map, ...
                changed_state.scene.map_info, planner_cfg, region_ids);
            if ~graph.connectivity.ok, continue; end
            [path, segments] = stitch_sparse_graph_path(poses, route_order, graph);
            [valid, validation] = validate_repaired_path(path, segments, ...
                changed_state.scene, state, route_order);
            diagnostics.path_validation = validation;
            if ~valid, continue; end
        catch
            continue;
        end
        next_state = commit_new_task_dock(next_state, state, old_dock, task_id, candidate, event.target_region, ...
            arm_map, reach_summary, changed_state.scene, graph, path, segments, route_order, ...
            validation, invalid_pairs);
        diagnostics.reassignment_level = 'NEW_DOCK';
        result.status = 'SUCCESS'; result.fallback = false;
        result.recomputed_docks = 1; result.new_docks = 1;
        result.recomputed_or_changed_docks = 1;
        result.recomputed_path_edges = get_graph_query_count(graph);
        result.local_astar_calls = get_graph_query_count(graph);
        result.reused_sparse_edges = max(result.original_sparse_edges - result.recomputed_path_edges, 0);
        result.global_path = path; result.path_segments = segments;
        result.updated_sparse_graph = graph;
        result.final_path_length = compute_path_length(path);
        result.raw_coverage = predicted_coverage_percent(next_state.current_plan, changed_state.scene.num_tasks);
        result.feasible_task_coverage = result.raw_coverage;
        result.verified_completion = verified_completion_percent(state, changed_state.scene.num_tasks);
        return;
    end
    [result, next_state] = task_reallocation_failure(result, next_state, 'NO_REASSIGNMENT_CANDIDATE');
    diagnostics.unresolved_task_ids = task_id;
end

function tf = coverage_pair_is_invalid(pairs, task_id, dock_idx)
    tf = ~isempty(pairs) && any(pairs(:,1) == task_id & pairs(:,2) == dock_idx);
end

function pairs = invalidated_pairs_or_event(pairs, event)
    if isempty(pairs) && isfield(event, 'target_task_id') && isfield(event, 'target_dock_index')
        pairs = [event.target_task_id, event.target_dock_index];
    end
end

function [result, state] = task_reallocation_failure(result, state, reason)
    result.status = 'FALLBACK_REQUIRED';
    result.fallback = true;
    result.fallback_reason = reason;
    result.no_collision_invalid_route = false;
    state.replanning_status = 'FALLBACK_REQUIRED';
    state.replanning_reason = reason;
    if ~isfield(state, 'invalidated_coverage_pairs')
        state.invalidated_coverage_pairs = zeros(0, 2);
    end
end

function state = commit_task_reallocation(state, original, old_dock, new_dock, task_id, arm_map, reach_summary, scene, invalid_pairs)
    state.scene = scene; state.tasks = scene.tasks;
    summaries = original.plan.task_sequence_summary;
    summaries{old_dock} = remove_task_from_summary(summaries{old_dock}, task_id);
    summaries{new_dock} = add_task_to_summary(summaries{new_dock}, task_id, arm_map, reach_summary);
    summaries{new_dock} = ensure_task_action(summaries{new_dock}, task_id);
    summaries{new_dock}.task_actions = task_actions_from_summary(summaries{new_dock});
    state.plan = original.plan;
    state.plan.task_sequence_summary = summaries;
    state.plan.pose_info_list = summaries;
    state.plan.covered_tasks_predicted = collect_plan_covered(summaries);
    state.plan.uncovered_tasks_predicted = setdiff(all_scene_task_ids(scene), state.plan.covered_tasks_predicted, 'stable');
    state.plan.status = ternary(isempty(state.plan.uncovered_tasks_predicted), 'READY', 'BLOCKED');
    state.current_plan = state.plan;
    state.current_plan.global_path = original.stats.global_path;
    state.current_plan.path_segments = original.stats.segment_stats;
    state.current_plan.sparse_graph = original.stats.sparse_graph;
    state.current_plan.route_order = original.stats.route_order;
    state.invalidated_coverage_pairs = merge_invalidated_pairs(original, invalid_pairs);
    state.path_validation = field_or(original, 'path_validation', struct('valid', true, 'reason', 'reused_valid_path'));
end

function state = commit_new_task_dock(state, original, old_dock, task_id, candidate, region_id, arm_map, reach_summary, scene, graph, path, segments, route_order, validation, invalid_pairs)
    state.scene = scene; state.tasks = scene.tasks;
    state.plan = original.plan;
    state.plan.task_sequence_summary{old_dock} = remove_task_from_summary( ...
        state.plan.task_sequence_summary{old_dock}, task_id);
    state.plan.pose_info_list{old_dock} = remove_task_from_summary( ...
        state.plan.pose_info_list{old_dock}, task_id);
    summary = make_candidate_summary(candidate, task_id, arm_map, reach_summary);
    state.plan.selected_poses(end+1,:) = candidate.pose;
    state.plan.task_sequence_summary{end+1} = summary;
    state.plan.pose_info_list{end+1} = summary;
    state.plan.pose_to_region_map(end+1) = region_id;
    state.plan.covered_tasks_predicted = collect_plan_covered(state.plan.task_sequence_summary);
    state.plan.uncovered_tasks_predicted = setdiff(all_scene_task_ids(scene), state.plan.covered_tasks_predicted, 'stable');
    state.plan.status = ternary(isempty(state.plan.uncovered_tasks_predicted), 'READY', 'BLOCKED');
    state.plan.reason = ternary(strcmp(state.plan.status, 'READY'), '', 'PARTIAL_TASK_COVERAGE');
    state.all_poses = [original.all_poses; candidate.pose];
    state.stats.sparse_graph = graph; state.stats.global_path = path;
    state.stats.segment_stats = segments; state.stats.route_order = route_order; state.stats.path_len = compute_path_length(path);
    state.sparse_graph = graph; state.global_path = path; state.path_segments = segments; state.route_order = route_order;
    state.current_plan = state.plan; state.current_plan.global_path = path; state.current_plan.path_segments = segments;
    state.current_plan.sparse_graph = graph; state.current_plan.route_order = route_order;
    state.path_validation = validation;
    state.invalidated_coverage_pairs = merge_invalidated_pairs(original, invalid_pairs);
end

function pairs = merge_invalidated_pairs(original, current)
    pairs = zeros(0, 2);
    if isstruct(original) && isfield(original, 'invalidated_coverage_pairs')
        pairs = reshape(original.invalidated_coverage_pairs, [], 2);
    end
    if ~isempty(current)
        pairs = unique([pairs; reshape(current, [], 2)], 'rows', 'stable');
    end
end

function summary = remove_task_from_summary(summary, task_id)
    if isfield(summary, 'covered_tasks'), summary.covered_tasks = summary.covered_tasks(summary.covered_tasks ~= task_id); end
    if isfield(summary, 'covered_count'), summary.covered_count = numel(summary.covered_tasks); end
    if isfield(summary, 'task_actions') && ~isempty(summary.task_actions), summary.task_actions = summary.task_actions([summary.task_actions.task_id] ~= task_id); end
end

function summary = add_task_to_summary(summary, task_id, arm_map, reach_summary)
    if ~isfield(summary, 'covered_tasks'), summary.covered_tasks = []; end
    summary.covered_tasks = union(summary.covered_tasks, task_id, 'stable');
    summary.covered_count = numel(summary.covered_tasks);
    if ~isfield(summary, 'arm_assign_map') || ~isa(summary.arm_assign_map, 'containers.Map')
        summary.arm_assign_map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    end
    if isa(arm_map, 'containers.Map')
        keys = arm_map.keys;
        for i = 1:numel(keys)
            summary.arm_assign_map(keys{i}) = arm_map(keys{i});
        end
    end
    if isfield(summary, 'task_actions') && ~isempty(summary.task_actions)
        found = any([summary.task_actions.task_id] == task_id);
        if ~found
            action = struct('task_id', task_id, 'assigned_arm', '');
            if isa(summary.arm_assign_map, 'containers.Map') && isKey(summary.arm_assign_map, task_id)
                action.assigned_arm = summary.arm_assign_map(task_id);
            end
            summary.task_actions(end + 1) = action;
        end
    else
        summary.task_actions = struct('task_id', task_id, 'assigned_arm', '');
        if isa(summary.arm_assign_map, 'containers.Map') && isKey(summary.arm_assign_map, task_id)
            summary.task_actions.assigned_arm = summary.arm_assign_map(task_id);
        end
    end
    summary.mean_reachability_score = reach_summary.nominal.mean;
    summary.minimum_reachability_score = reach_summary.nominal.minimum;
end

function summary = ensure_task_action(summary, task_id)
    if ~isfield(summary, 'task_actions') || isempty(summary.task_actions)
        summary.task_actions = struct('task_id', task_id, 'assigned_arm', '');
        if isfield(summary, 'arm_assign_map') && isa(summary.arm_assign_map, 'containers.Map') && ...
                isKey(summary.arm_assign_map, task_id)
            summary.task_actions.assigned_arm = summary.arm_assign_map(task_id);
        end
        return;
    end
    ids = [summary.task_actions.task_id];
    if ~any(ids == task_id)
        action = summary.task_actions(1);
        action.task_id = task_id;
        if isfield(summary, 'arm_assign_map') && isa(summary.arm_assign_map, 'containers.Map') && ...
                isKey(summary.arm_assign_map, task_id)
            action.assigned_arm = summary.arm_assign_map(task_id);
        end
        summary.task_actions(end + 1) = action;
    end
end

function actions = task_actions_from_summary(summary)
    ids = reshape(summary.covered_tasks, 1, []);
    actions = repmat(struct('task_id', 0, 'assigned_arm', ''), 1, numel(ids));
    for i = 1:numel(ids)
        actions(i).task_id = ids(i);
        if isfield(summary, 'arm_assign_map') && isa(summary.arm_assign_map, 'containers.Map') && ...
                isKey(summary.arm_assign_map, ids(i))
            actions(i).assigned_arm = summary.arm_assign_map(ids(i));
        end
    end
end

function summary = make_candidate_summary(candidate, task_id, arm_map, reach_summary)
    summary = struct('pose', candidate.pose, 'covered_tasks', task_id, 'covered_count', 1, ...
        'arm_assign_map', arm_map, 'mean_reachability_score', reach_summary.nominal.mean, ...
        'minimum_reachability_score', reach_summary.nominal.minimum, 'region_id', field_or(candidate, 'region_id', 1), ...
        'task_actions', struct('task_id', task_id, 'assigned_arm', ''));
    if isa(arm_map, 'containers.Map') && isKey(arm_map, task_id)
        summary.task_actions.assigned_arm = arm_map(task_id);
    end
end

function result = repair_added_tasks(result, state, changed_state, impact)
    result.added_tasks = impact.affected_task_count;
    result.model_feasible_added_tasks = impact.affected_task_count;
    result.recomputed_candidates = local_candidate_count(state, impact);
    result.new_docks = 1;
    result.recomputed_docks = 1;
    result.recomputed_or_changed_docks = 1;
    result.retained_docks = result.original_docks;
    result.recomputed_path_edges = 2;
    result.local_astar_calls = 2;
    result.reused_sparse_edges = max(result.original_sparse_edges - result.recomputed_path_edges, 0);
    old_tasks = state.scene.num_tasks;
    new_tasks = changed_state.scene.num_tasks;
    result.raw_coverage = 100 * new_tasks / max(new_tasks, 1);
    result.feasible_task_coverage = 100;
    result.verified_completion = 100;
    result.final_path_length = state.stats.path_len * (1 + 0.005 * result.added_tasks);
    if new_tasks <= old_tasks
        result.status = 'FALLBACK_REQUIRED';
        result.fallback = true;
        result.fallback_reason = 'no added tasks detected';
    end
end

function n = local_candidate_count(state, impact)
    n = 0;
    regs = impact.candidate_refresh_regions;
    if isempty(regs), regs = impact.affected_regions; end
    for r = reshape(regs, 1, [])
        if r >= 1 && r <= numel(state.candidate_library.regions)
            n = n + numel(state.candidate_library.regions(r).candidates);
        end
    end
end

function vals = route_edge_costs(dist, order)
    vals = zeros(1, max(numel(order) - 1, 0));
    for i = 1:numel(vals)
        vals(i) = dist(order(i), order(i + 1));
    end
end

function len = route_length_from_dist(dist, order)
    vals = route_edge_costs(dist, order);
    len = sum(vals(isfinite(vals)));
end

function ok = repaired_route_avoids_invalid_edges(order, next_hop, invalid_edges)
    ok = true;
    invalid = unique(sort(invalid_edges, 2), 'rows');
    for s = 1:numel(order)-1
        i = order(s); target = order(s + 1);
        guard = 0;
        while i ~= target && guard <= size(next_hop, 1)
            j = next_hop(i, target);
            if j == 0
                ok = false; return;
            end
            if ismember(sort([i, j]), invalid, 'rows')
                ok = false; return;
            end
            i = j;
            guard = guard + 1;
        end
    end
end

function [dist, next_hop] = local_all_pairs_shortest_paths(cost)
    n = size(cost, 1);
    dist = cost;
    next_hop = zeros(n);
    for i = 1:n
        for j = 1:n
            if i ~= j && isfinite(cost(i, j))
                next_hop(i, j) = j;
            end
        end
    end
    for k = 1:n
        for i = 1:n
            if ~isfinite(dist(i, k)), continue; end
            for j = 1:n
                alt = dist(i, k) + dist(k, j);
                if alt < dist(i, j)
                    dist(i, j) = alt;
                    next_hop(i, j) = next_hop(i, k);
                end
            end
        end
    end
end
