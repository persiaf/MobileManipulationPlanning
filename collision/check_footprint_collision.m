function is_collision = check_footprint_collision(pose, grid_map, map_info, robot_cfg)
    rows = map_info.rows;
    cols = map_info.cols;
    res  = map_info.resolution;
    height_m = map_info.height_m;

    [bi, bj] = world2grid(pose(1), pose(2), res, height_m);

    half_w = (robot_cfg.chassis.width  / 2) / res;
    half_l = (robot_cfg.chassis.length / 2) / res;

    cos_t = cos(pose(3));
    sin_t = sin(pose(3));

    check_range = ceil(max(half_l, half_w)) + 2;
    di_range = max(1, bi - check_range) : min(rows, bi + check_range);
    dj_range = max(1, bj - check_range) : min(cols, bj + check_range);

    is_collision = false;
    for i = di_range
        for j = dj_range
            if grid_map(i, j) == 0
                continue;
            end
            dx = j - bj;
            dy = i - bi;
            dr =  dx * cos_t + dy * sin_t;
            dc = -dx * sin_t + dy * cos_t;
            if abs(dr) <= half_w + 0.5 && abs(dc) <= half_l + 0.5
                is_collision = true;
                return;
            end
        end
    end
end