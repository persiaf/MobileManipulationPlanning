function clearance_m = compute_footprint_clearance(pose, grid_map, map_info, robot_cfg)
%COMPUTE_FOOTPRINT_CLEARANCE True base-edge separation from cells/boundaries.
%   Distance is zero for touching/intersecting geometry. It is not a distance
%   from the robot centre and is not capped at an arbitrary search radius.
    [~, clearance_m] = check_footprint_collision(pose, grid_map, map_info, robot_cfg);
end
