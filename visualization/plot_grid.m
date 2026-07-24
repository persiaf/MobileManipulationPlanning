function plot_grid(grid_map, res, height_m)
    [rows, cols] = size(grid_map);
    hold on;

    mask_free = logical(grid_map == 0);
    mask_wall = logical(grid_map == 1);
    mask_table = logical(grid_map == 2);

    img_r = 0.95 * ones(rows, cols);
    img_g = 0.95 * ones(rows, cols);
    img_b = 0.95 * ones(rows, cols);

    img_r(mask_wall) = 0.10;
    img_g(mask_wall) = 0.10;
    img_b(mask_wall) = 0.10;

    img_r(mask_table) = 0.20;
    img_g(mask_table) = 0.15;
    img_b(mask_table) = 0.15;

    img = cat(3, img_r, img_g, img_b);

    if nargin < 3
        imagesc(img);
    else
        x_range = [res/2, cols*res - res/2];
        y_range = [height_m - res/2, res/2];
        imagesc(x_range, y_range, img);
    end

    axis equal;
    axis tight;

    if nargin >= 3
        x_edges = 0:res:cols*res;
        y_edges = 0:res:rows*res;
        [Xg, Yg] = meshgrid(x_edges(1:4:end), y_edges(1:4:end));
        for xi = 1:size(Xg, 2)-1
            plot([Xg(1,xi), Xg(1,xi)], [Yg(1,1), Yg(end,1)], ...
                 'Color', [0.85 0.85 0.85], 'LineWidth', 0.3);
        end
        for yi = 1:size(Yg, 1)-1
            plot([Xg(1,1), Xg(1,end)], [Yg(yi,1), Yg(yi,1)], ...
                 'Color', [0.85 0.85 0.85], 'LineWidth', 0.3);
        end
    end
end