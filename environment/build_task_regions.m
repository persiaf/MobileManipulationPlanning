function regions = build_task_regions(scene, task_table_map)
    robot_cfg = robot_config();
    scene_cfg = scene.scene_cfg;
    num_tables = size(scene_cfg.table_positions, 1);

    regions = struct('table_id', {}, 'task_ids', {}, ...
                     'center', {}, 'radius', {}, ...
                     'left_side_free', {}, 'right_side_free', {}, ...
                     'top_side_free', {}, 'bottom_side_free', {});

    for t = 1:num_tables
        if isempty(task_table_map{t})
            continue;
        end

        regions(end+1).table_id = t;

        regions(end).task_ids = task_table_map{t};

        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        regions(end).center = [cx, cy];

        task_pos = zeros(length(task_table_map{t}), 2);
        for i = 1:length(task_table_map{t})
            tid = task_table_map{t}(i);
            task_pos(i, :) = [scene.tasks(tid).x, scene.tasks(tid).y];
        end
        regions(end).radius = max(sqrt((task_pos(:,1)-cx).^2 + (task_pos(:,2)-cy).^2));

        regions(end).left_side_free  = true;
        regions(end).right_side_free = true;
        regions(end).top_side_free   = true;
        regions(end).bottom_side_free = true;
    end
end