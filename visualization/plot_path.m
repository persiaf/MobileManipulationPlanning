function plot_path(path_grid, G)
    if size(path_grid, 1) < 2
        return;
    end
    x = path_grid(:, 2);
    y = path_grid(:, 1);
    plot(x, y, 'r-', 'LineWidth', 2);
    plot(x, y, 'ro', 'MarkerSize', 5);
end