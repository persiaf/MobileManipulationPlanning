function report = validate_sparse_graph_connectivity(adjacency, region_ids)
%VALIDATE_SPARSE_GRAPH_CONNECTIVITY Check node and region reachability.
%
%   Node 1 is the start pose. Required docks are nodes 2..N. The function
%   only inspects graph topology; edge costs and path feasibility are owned
%   by build_sparse_docking_graph.

    if nargin < 2 || isempty(region_ids)
        region_ids = ones(size(adjacency, 1), 1);
        region_ids(1) = 0;
    end
    adjacency = logical(adjacency | adjacency');
    n = size(adjacency, 1);
    if n == 0
        report = empty_report(false);
        return;
    end

    visited = bfs(adjacency, 1);
    dock_nodes = 2:n;
    report.start_connected = true;
    report.all_required_docks_reachable = all(visited(dock_nodes));
    report.connected_node_count = sum(visited);
    report.node_count = n;
    report.component_ids = connected_components(adjacency);
    report.component_count = max(report.component_ids);

    positive_regions = unique(region_ids(region_ids > 0), 'stable');
    region_adj = false(numel(positive_regions));
    for i = 1:n
        ri = find(positive_regions == region_ids(i), 1);
        if isempty(ri), continue; end
        nbrs = find(adjacency(i, :));
        for j = nbrs
            rj = find(positive_regions == region_ids(j), 1);
            if ~isempty(rj) && ri ~= rj
                region_adj(ri, rj) = true;
                region_adj(rj, ri) = true;
            end
        end
    end
    if isempty(positive_regions)
        report.region_graph_connected = true;
    elseif numel(positive_regions) == 1
        report.region_graph_connected = true;
    else
        report.region_graph_connected = all(bfs(region_adj, 1));
    end
    report.region_count = numel(positive_regions);
    report.ok = report.start_connected && report.all_required_docks_reachable && ...
        report.region_graph_connected;
end

function report = empty_report(ok)
    report = struct('ok', ok, 'start_connected', ok, ...
        'all_required_docks_reachable', ok, 'region_graph_connected', ok, ...
        'connected_node_count', 0, 'node_count', 0, ...
        'component_ids', [], 'component_count', 0, 'region_count', 0);
end

function visited = bfs(adjacency, start_idx)
    n = size(adjacency, 1);
    visited = false(1, n);
    if n == 0 || start_idx < 1 || start_idx > n
        return;
    end
    queue = start_idx;
    visited(start_idx) = true;
    head = 1;
    while head <= numel(queue)
        u = queue(head);
        head = head + 1;
        nbrs = find(adjacency(u, :));
        for v = nbrs
            if ~visited(v)
                visited(v) = true;
                queue(end + 1) = v; %#ok<AGROW>
            end
        end
    end
end

function ids = connected_components(adjacency)
    n = size(adjacency, 1);
    ids = zeros(1, n);
    cid = 0;
    for i = 1:n
        if ids(i) ~= 0, continue; end
        cid = cid + 1;
        visited = bfs(adjacency, i);
        ids(visited) = cid;
    end
end
