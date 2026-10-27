function report = validate_cafeteria_path(scene, path, robot_cfg)
%VALIDATE_CAFETERIA_PATH Run formal centerline and existing footprint checks.

    report.waypoint_count = size(path, 1);
    report.path_collision_count = 0;
    report.footprint_collision_count = 0;
    report.minimum_clearance_m = NaN;
    if isempty(path), return; end
    [~, bad_segments] = validate_world_path_collision(scene.grid_map, path, scene.map_info, false);
    report.path_collision_count = size(bad_segments, 1);
    clearances = zeros(size(path, 1), 1);
    for i = 1:size(path, 1)
        pose = path_pose(path, i);
        report.footprint_collision_count = report.footprint_collision_count + ...
            check_footprint_collision(pose, scene.grid_map, scene.map_info, robot_cfg);
        clearances(i) = compute_footprint_clearance(pose, scene.grid_map, scene.map_info, robot_cfg);
    end
    report.minimum_clearance_m = min(clearances);
end

function pose = path_pose(path, i)
    pose = [path(i, 1:2), 0];
    if size(path, 2) >= 3
        pose(3) = path(i, 3);
    elseif size(path, 1) > 1
        j = min(i + 1, size(path, 1));
        k = max(i - 1, 1);
        delta = path(j, 1:2) - path(k, 1:2);
        if any(abs(delta) > eps), pose(3) = atan2(delta(2), delta(1)); end
    end
end
