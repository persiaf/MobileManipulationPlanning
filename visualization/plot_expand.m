function plot_expand(expand, map_size, G, planner_name)
    [rows, cols] = size(map_size);
    cmap = winter(max(expand(:)));
    for i = 1:rows
        for j = 1:cols
            if expand(i, j) > 0
                alpha = 0.3 + 0.4 * (expand(i, j) / max(expand(:)));
                rectangle('Position', [j-0.5, i-0.5, 1, 1], ...
                          'FaceColor', [0.2, 0.8, 0.2], ...
                          'EdgeColor', 'none', ...
                          'FaceAlpha', alpha);
            end
        end
    end
end