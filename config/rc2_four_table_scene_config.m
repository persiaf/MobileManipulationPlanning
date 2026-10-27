function cfg = rc2_four_table_scene_config()
%RC2_FOUR_TABLE_SCENE_CONFIG Parameterized RC2 four-table/eight-task scene.
% Table centers and task offsets are the only geometry inputs users normally edit.
    cfg = scene_config();
    cfg.scene_name = 'rc2_four_table_eight_tasks';
    cfg.map_width_m = 8.0;
    cfg.map_height_m = 8.0;
    cfg.grid_resolution_m = 0.05;
    cfg.border_thickness_m = 0.10;
    cfg.table.length = 1.20;
    cfg.table.width = 0.40;
    cfg.table.height = 0.50;
    cfg.table_mode = 'custom';
    cfg.table_positions = [2.2 2.2; 5.6 2.2; 2.2 5.6; 5.6 5.6];
    cfg.num_tables = size(cfg.table_positions, 1);
    cfg.table_orientations = zeros(cfg.num_tables, 1);
    cfg.table_min_spacing_m = 1.0;
    cfg.table_wall_margin_m = 0.4;
    cfg.table_start_margin_m = 0.8;
    cfg.start_pose = [0.55 0.55 0];
    cfg.tasks_per_table = 2;
    cfg.task_offsets_local = [-0.20 0.00; 0.20 0.00];
    cfg.task_mode = 'manual';
    cfg.task_offset_z = cfg.table.height;
    cfg.random_seed = 2026;
    manual = zeros(cfg.num_tables * cfg.tasks_per_table, 4);
    k = 0;
    for table_id = 1:cfg.num_tables
        for j = 1:cfg.tasks_per_table
            k = k + 1;
            xy = table_local_to_world(cfg.table_positions(table_id,:), ...
                cfg.table_orientations(table_id), cfg.task_offsets_local(j,:));
            manual(k,:) = [table_id xy cfg.task_offset_z];
        end
    end
    cfg.manual_tasks = manual;
    cfg.expected_table_count = cfg.num_tables;
    cfg.expected_task_count = size(manual,1);
end
