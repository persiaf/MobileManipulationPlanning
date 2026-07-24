function path = reconstruct_path_grid(gi, gj, si, sj, parent, res, height_m)
    grid_path = [gi, gj];
    ci = gi;
    cj = gj;
    while ~(ci == si && cj == sj)
        pi = parent(ci, cj, 1);
        pj = parent(ci, cj, 2);
        if pi == 0 && pj == 0
            break;
        end
        grid_path = [pi, pj; grid_path];
        ci = pi;
        cj = pj;
    end

    path = zeros(size(grid_path, 1), 3);
    for k = 1:size(grid_path, 1)
        [wx, wy] = grid2world(grid_path(k, 1), grid_path(k, 2), res, height_m);
        path(k, 1) = wx;
        path(k, 2) = wy;
        path(k, 3) = 0;
    end
end