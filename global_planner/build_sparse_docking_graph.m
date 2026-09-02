function graph_data = build_sparse_docking_graph(poses, grid_map, map_info, planner_cfg, region_ids)
%BUILD_SPARSE_DOCKING_GRAPH Build a region-aware sparse path-cost graph.
%
%   Nodes are [start; selected docking poses]. Candidate edges are generated
%   geometrically by kNN, then each edge is validated with the configured
%   global planner. Connectivity repair adds only minimal bridge candidates;
%   it never silently falls back to a dense all-pairs matrix.

    if nargin < 5 || isempty(region_ids)
        region_ids = ones(size(poses, 1), 1);
        region_ids(1) = 0;
    end
    region_ids = region_ids(:);
    n = size(poses, 1);
    opts = sparse_options(planner_cfg);
    potential_dense_pairs = n * (n - 1) / 2;

    geometric = build_geometric_edges(poses, region_ids, opts);
    [adjacency, cost_matrix, direct_paths, stats] = validate_edges( ...
        geometric, poses, grid_map, map_info, planner_cfg);

    geo_report = validate_sparse_graph_connectivity(geometric, region_ids);
    report = validate_sparse_graph_connectivity(adjacency, region_ids);
    while ~report.ok
        bridge = nearest_component_bridge(poses, adjacency, report.component_ids);
        if isempty(bridge)
            break;
        end
        [ok, cost, path, cache_hit] = query_edge(bridge(1), bridge(2), poses, grid_map, map_info, planner_cfg);
        stats.actual_path_queries = stats.actual_path_queries + 1;
        stats.actual_astar_calls = stats.actual_astar_calls + double(~cache_hit);
        stats.adaptive_bridge_edges_added = stats.adaptive_bridge_edges_added + 1;
        if ok
            i = bridge(1); j = bridge(2);
            adjacency(i, j) = true; adjacency(j, i) = true;
            cost_matrix(i, j) = cost; cost_matrix(j, i) = cost;
            direct_paths{i, j} = path; direct_paths{j, i} = flipud(path);
            stats.successful_edges = stats.successful_edges + 1;
        else
            geometric(bridge(1), bridge(2)) = true;
            geometric(bridge(2), bridge(1)) = true;
            stats.failed_edges = stats.failed_edges + 1;
        end
        next_report = validate_sparse_graph_connectivity(adjacency, region_ids);
        if isequal(next_report.component_ids, report.component_ids)
            break;
        end
        report = next_report;
    end

    [shortest_cost_matrix, next_hop] = all_pairs_shortest_paths(cost_matrix);
    graph_data = struct();
    graph_data.mode = 'sparse_knn';
    graph_data.options = opts;
    graph_data.region_ids = region_ids;
    graph_data.geometric_adjacency = geometric;
    graph_data.adjacency = adjacency;
    graph_data.cost_matrix = cost_matrix;
    graph_data.shortest_cost_matrix = shortest_cost_matrix;
    graph_data.next_hop = next_hop;
    graph_data.direct_paths = direct_paths;
    graph_data.connectivity = report;
    graph_data.geometric_connectivity = geo_report;
    graph_data.stats = stats;
    graph_data.stats.potential_dense_pairs = potential_dense_pairs;
    graph_data.stats.sparse_geometric_edges = nnz(triu(geometric, 1));
    graph_data.stats.validated_sparse_edges = nnz(triu(adjacency, 1));
end

function opts = sparse_options(planner_cfg)
    opts = struct('k', 6, 'k_local', 4, 'k_cross', 1, ...
        'max_local_k', 12, 'enable_region_rule', true);
    if isfield(planner_cfg, 'routing') && isfield(planner_cfg.routing, 'sparse_knn')
        user = planner_cfg.routing.sparse_knn;
        names = fieldnames(opts);
        for i = 1:numel(names)
            if isfield(user, names{i})
                opts.(names{i}) = user.(names{i});
            end
        end
    end
end

function adjacency = build_geometric_edges(poses, region_ids, opts)
    n = size(poses, 1);
    adjacency = false(n);
    xy = poses(:, 1:2);
    for i = 1:n
        d = sqrt(sum((xy - xy(i, :)).^2, 2));
        d(i) = inf;
        if i == 1 || ~opts.enable_region_rule
            nbrs = nearest_indices(d, opts.k);
        else
            same = region_ids == region_ids(i);
            same(i) = false;
            local_nbrs = nearest_indices(mask_dist(d, same), opts.k_local);
            cross_nbrs = nearest_indices(mask_dist(d, ~same), opts.k_cross);
            nbrs = unique([local_nbrs(:); cross_nbrs(:)]', 'stable');
            if numel(nbrs) < opts.k
                nbrs = unique([nbrs(:); nearest_indices(d, opts.k)], 'stable');
            end
        end
        adjacency(i, nbrs) = true;
        adjacency(nbrs, i) = true;
    end
end

function d2 = mask_dist(d, mask)
    d2 = d;
    d2(~mask) = inf;
end

function idx = nearest_indices(d, k)
    [vals, order] = sort(d, 'ascend');
    order = order(isfinite(vals));
    idx = order(1:min(k, numel(order)));
end

function [adjacency, cost_matrix, direct_paths, stats] = validate_edges(edge_candidates, poses, grid_map, map_info, planner_cfg)
    n = size(poses, 1);
    adjacency = false(n);
    cost_matrix = inf(n);
    cost_matrix(1:n+1:end) = 0;
    direct_paths = cell(n, n);
    stats = struct('potential_dense_pairs', 0, 'sparse_geometric_edges', 0, ...
        'actual_path_queries', 0, 'actual_astar_calls', 0, ...
        'successful_edges', 0, 'failed_edges', 0, ...
        'adaptive_bridge_edges_added', 0);
    [ii, jj] = find(triu(edge_candidates, 1));
    for e = 1:numel(ii)
        [ok, cost, path, cache_hit] = query_edge(ii(e), jj(e), poses, grid_map, map_info, planner_cfg);
        stats.actual_path_queries = stats.actual_path_queries + 1;
        stats.actual_astar_calls = stats.actual_astar_calls + double(~cache_hit);
        if ok
            i = ii(e); j = jj(e);
            adjacency(i, j) = true; adjacency(j, i) = true;
            cost_matrix(i, j) = cost; cost_matrix(j, i) = cost;
            direct_paths{i, j} = path; direct_paths{j, i} = flipud(path);
            stats.successful_edges = stats.successful_edges + 1;
        else
            stats.failed_edges = stats.failed_edges + 1;
        end
    end
end

function [ok, cost, path, cache_hit] = query_edge(i, j, poses, grid_map, map_info, planner_cfg)
    [path, cost, ok, cache_hit] = query_path_with_shared_cache( ...
        grid_map, poses(i, :), poses(j, :), map_info, planner_cfg);
    if ~ok || cost > 1e6 || isnan(cost)
        ok = false;
        cost = inf;
        path = [];
    end
end

function bridge = nearest_component_bridge(poses, adjacency, component_ids)
    bridge = [];
    comps = unique(component_ids);
    if numel(comps) <= 1
        return;
    end
    start_comp = component_ids(1);
    in_start = find(component_ids == start_comp);
    outside = find(component_ids ~= start_comp);
    best = inf;
    for a = in_start
        for b = outside
            if adjacency(a, b), continue; end
            d = norm(poses(a, 1:2) - poses(b, 1:2));
            if d < best
                best = d;
                bridge = [a, b]; %#ok<AGROW>
            end
        end
    end
end

function [dist, next_hop] = all_pairs_shortest_paths(cost)
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
