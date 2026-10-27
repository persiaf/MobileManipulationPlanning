function h = draw_sparse_graph_connectivity(ax, poses, adjacency)
%DRAW_SPARSE_GRAPH_CONNECTIVITY Draw topology, not a robot trajectory.

    [ii, jj] = find(triu(adjacency, 1));
    for k = 1:numel(ii)
        plot(ax, poses([ii(k), jj(k)], 1), poses([ii(k), jj(k)], 2), '-', ...
            'Color', [0.45 0.45 0.45 0.35], 'LineWidth', 0.4, ...
            'HandleVisibility', 'off');
    end
    h = plot(ax, NaN, NaN, '-', 'Color', [0.45 0.45 0.45], ...
        'LineWidth', 0.8, 'DisplayName', 'Sparse graph connectivity');
end
