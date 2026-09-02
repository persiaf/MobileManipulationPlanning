function [global_path, segment_stats] = stitch_sparse_graph_path(poses, route_order, graph_data)
%STITCH_SPARSE_GRAPH_PATH Stitch route segments through sparse graph paths.

    global_path = [];
    segment_stats = struct('from_id', {}, 'to_id', {}, 'plan_time_s', {}, ...
                           'path_length_m', {}, 'num_points', {}, ...
                           'is_return_segment', {});
    for k = 1:length(route_order) - 1
        from_idx = route_order(k);
        to_idx = route_order(k + 1);
        node_path = expand_node_path(graph_data.next_hop, from_idx, to_idx);
        segment = [];
        for p = 1:numel(node_path) - 1
            a = node_path(p); b = node_path(p + 1);
            edge_path = graph_data.direct_paths{a, b};
            if isempty(edge_path)
                error('stitch_sparse_graph_path:MissingEdgePath', ...
                    'Sparse route uses missing edge %d -> %d.', a, b);
            end
            if isempty(segment)
                segment = edge_path;
            else
                segment = [segment; edge_path(2:end, :)]; %#ok<AGROW>
            end
        end
        segment_stats(k).from_id = from_idx;
        segment_stats(k).to_id = to_idx;
        segment_stats(k).plan_time_s = 0;
        segment_stats(k).path_length_m = path_length(segment);
        segment_stats(k).num_points = size(segment, 1);
        segment_stats(k).is_return_segment = (k == length(route_order) - 1) && (to_idx == 1);
        if isempty(global_path)
            global_path = segment;
        elseif ~isempty(segment)
            global_path = [global_path; segment(2:end, :)]; %#ok<AGROW>
        end
    end
end

function nodes = expand_node_path(next_hop, source, target)
    if source == target
        nodes = source;
        return;
    end
    if next_hop(source, target) == 0
        error('stitch_sparse_graph_path:DisconnectedRoute', ...
            'No sparse graph path from node %d to node %d.', source, target);
    end
    nodes = source;
    current = source;
    guard = 0;
    while current ~= target
        current = next_hop(current, target);
        nodes(end + 1) = current; %#ok<AGROW>
        guard = guard + 1;
        if guard > size(next_hop, 1)
            error('stitch_sparse_graph_path:Cycle', ...
                'Sparse next-hop expansion cycled from node %d to %d.', source, target);
        end
    end
end
