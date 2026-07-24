function [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info)
    rows = map_info.rows;
    cols = map_info.cols;
    res = map_info.resolution;
    height_m = map_info.height_m;

    [si, sj] = world2grid(start_pose(1), start_pose(2), res, height_m);
    [gi, gj] = world2grid(goal_pose(1),  goal_pose(2),  res, height_m);

    si = max(1, min(rows, si));
    sj = max(1, min(cols, sj));
    gi = max(1, min(rows, gi));
    gj = max(1, min(cols, gj));

    traversable = (grid_map == 0);

    if ~traversable(si, sj)
        [si, sj] = nearest_free(grid_map, si, sj);
    end
    if ~traversable(gi, gj)
        [gi, gj] = nearest_free(grid_map, gi, gj);
    end
end