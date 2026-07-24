function path = convert_path_grid2world(grid_path, map_info)
    res = map_info.resolution;
    height_m = map_info.height_m;

    path = zeros(size(grid_path, 1), 3);
    for k = 1:size(grid_path, 1)
        [wx, wy] = grid2world(grid_path(k, 1), grid_path(k, 2), res, height_m);
        path(k, 1) = wx;
        path(k, 2) = wy;
        path(k, 3) = 0;
    end
end