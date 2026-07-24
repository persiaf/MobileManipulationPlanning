function clearance_m = compute_footprint_clearance(pose, grid_map, map_info, robot_cfg)
%COMPUTE_FOOTPRINT_CLEARANCE Distance from the base centre to occupied cells.
%   Footprint collision is checked separately by check_footprint_collision.

    [ci, cj] = world2grid(pose(1), pose(2), map_info.resolution, ...
        map_info.height_m, get_origin(map_info));
    search_radius_m = 0.5;
    search_cells = ceil(search_radius_m / map_info.resolution);

    i0 = max(1, ci - search_cells);
    i1 = min(map_info.rows, ci + search_cells);
    j0 = max(1, cj - search_cells);
    j1 = min(map_info.cols, cj + search_cells);
    obstacle_cells = find(grid_map(i0:i1, j0:j1) > 0);
    if isempty(obstacle_cells)
        clearance_m = search_radius_m;
        return;
    end

    [ri, rj] = ind2sub([i1 - i0 + 1, j1 - j0 + 1], obstacle_cells);
    centre_dist_m = hypot((ri + i0 - 1) - ci, (rj + j0 - 1) - cj) * map_info.resolution;
    clearance_m = max(min(centre_dist_m), 0.005);
end

function origin = get_origin(map_info)
    origin = [0, 0];
    if isfield(map_info, 'origin_xy')
        origin = map_info.origin_xy;
    end
end
