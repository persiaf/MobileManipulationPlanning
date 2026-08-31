function edge_info = build_route_edge_set(normalized_order, options)
%BUILD_ROUTE_EDGE_SET Build directed and undirected route-edge diagnostics.
% By default only dock-to-dock edges are included. Start/return edges can be
% included explicitly; start is represented as node 0.

    if nargin < 2 || isempty(options)
        options = struct();
    end
    include_start = get_option(options, 'include_start_edge', false);
    include_return = get_option(options, 'include_return_edge', false);

    dock_order = reshape(normalized_order.dock_order, 1, []);
    directed = zeros(0, 2);
    start_edges = zeros(0, 2);
    return_edges = zeros(0, 2);

    if include_start && ~isempty(dock_order)
        start_edges = [0, dock_order(1)];
        directed = [directed; start_edges]; %#ok<AGROW>
    end
    if numel(dock_order) >= 2
        directed = [directed; [dock_order(1:end-1)', dock_order(2:end)']]; %#ok<AGROW>
    end
    if include_return && ~isempty(dock_order)
        return_edges = [dock_order(end), 0];
        directed = [directed; return_edges]; %#ok<AGROW>
    end

    undirected = sort(directed, 2);
    if ~isempty(undirected)
        undirected = unique(undirected, 'rows');
    end

    edge_info.directed_edges = directed;
    edge_info.undirected_edges = undirected;
    edge_info.num_edges = size(directed, 1);
    edge_info.includes_start_edge = include_start && ~isempty(start_edges);
    edge_info.includes_return_edge = include_return && ~isempty(return_edges);
    edge_info.start_edges = start_edges;
    edge_info.return_edges = return_edges;
end

function v = get_option(options, field_name, default_value)
    if isstruct(options) && isfield(options, field_name)
        v = options.(field_name);
    else
        v = default_value;
    end
end
