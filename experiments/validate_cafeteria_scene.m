function report = validate_cafeteria_scene(scene, robot_cfg)
%VALIDATE_CAFETERIA_SCENE Validate fixed geometry without changing semantics.

    cfg = scene.scene_cfg;
    [layout_ok, layout] = validate_table_layout(cfg);
    report.table_overlap_count = sum(contains(layout.failures, '_overlap'));
    report.table_boundary_violation_count = sum(contains(layout.failures, '_map_boundary_collision'));
    report.start_collision = check_footprint_collision(scene.start_pose, ...
        scene.grid_map, scene.map_info, robot_cfg);
    report.a4_inside_table = true(scene.num_tasks, 1);
    marker = cfg.task_marker.size;
    tol = 1e-12;
    for i = 1:scene.num_tasks
        task = scene.tasks(i);
        theta = cfg.table_orientations(task.table_id);
        marker_world = oriented_rectangle_corners([task.x, task.y], theta, marker(1), marker(2));
        local = (rot2(-theta) * ...
            (marker_world - cfg.table_positions(task.table_id, :))')';
        report.a4_inside_table(i) = all(abs(local(:, 1)) <= cfg.table.length / 2 + tol) && ...
            all(abs(local(:, 2)) <= cfg.table.width / 2 + tol);
    end
    report.main_aisle_connected = aisle_centerline_free(scene);
    report.is_valid = layout_ok && report.table_overlap_count == 0 && ...
        report.table_boundary_violation_count == 0 && ~report.start_collision && ...
        all(report.a4_inside_table) && report.main_aisle_connected;
end

function connected = aisle_centerline_free(scene)
    cfg = scene.scene_cfg;
    x = mean(cfg.cafeteria.main_aisle_x);
    y = (cfg.border_thickness_m + cfg.safety_margin_m + scene.map_info.resolution): ...
        scene.map_info.resolution: ...
        (cfg.map_height_m - cfg.border_thickness_m - cfg.safety_margin_m - scene.map_info.resolution);
    connected = true;
    for k = 1:numel(y)
        [i, j] = world2grid(x, y(k), scene.map_info.resolution, scene.map_info.height_m);
        if scene.grid_map(i, j) ~= 0
            connected = false;
            return;
        end
    end
end
