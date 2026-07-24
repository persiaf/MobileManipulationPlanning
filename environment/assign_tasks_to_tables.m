function task_table_map = assign_tasks_to_tables(scene)
    scene_cfg = scene.scene_cfg;
    num_tables = size(scene_cfg.table_positions, 1);
    task_table_map = cell(num_tables, 1);

    for ti = 1:scene.num_tasks
        tx = scene.tasks(ti).x;
        ty = scene.tasks(ti).y;
        min_dist = inf;
        assigned_table = 0;
        for t = 1:num_tables
            table_cx = scene_cfg.table_positions(t, 1);
            table_cy = scene_cfg.table_positions(t, 2);
            d = sqrt((tx - table_cx)^2 + (ty - table_cy)^2);
            max_table_radius = sqrt((scene_cfg.table.length/2)^2 + (scene_cfg.table.width/2)^2);
            if d < max_table_radius + 0.2 && d < min_dist
                min_dist = d;
                assigned_table = t;
            end
        end
        if assigned_table > 0
            task_table_map{assigned_table} = [task_table_map{assigned_table}; ti];
        end
    end
end