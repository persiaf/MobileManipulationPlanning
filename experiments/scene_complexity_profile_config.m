function cfg = scene_complexity_profile_config(case_id, seed)
%SCENE_COMPLEXITY_PROFILE_CONFIG Build C0-C3 scene-complexity configs.

    cfg = create_scaled_scene_config('large', seed);
    cfg.map_width_m = 10.0;
    cfg.map_height_m = 10.0;
    cfg.layout.num_tables = 16;
    cfg.layout.rows = 4;
    cfg.layout.cols = 4;
    cfg.layout.spacing_x_m = 2.3;
    cfg.layout.spacing_y_m = 2.4;
    cfg.layout.center = [5.0, 5.0];
    cfg.layout.jitter_range_m = 0.20;
    cfg.layout.orientation_mode = 'fixed';
    cfg.layout.orientation_candidates_deg = [0, 30, -30, 45, -45, 90];
    cfg.layout.max_retries = 50;
    cfg.table_mode = 'layout';
    cfg.table_min_spacing_m = 1.0;
    cfg.table_min_aisle_clearance_m = derive_min_aisle_clearance();
    cfg.tasks_per_table_range = [3, 6];
    cfg.requested_task_count = cfg.layout.num_tables * mean(cfg.tasks_per_table_range);
    cfg.task.position_mode = 'discrete';
    cfg.task.difficulty = 'normal';
    cfg.task.edge_margin_m = 0.04;

    switch lower(case_id)
        case 'c0'
            cfg.scene_complexity_case = 'C0';
            cfg.layout.pattern = 'grid';
            cfg.layout.orientation_mode = 'fixed';
            cfg.task.position_mode = 'discrete';
            cfg.task.difficulty = 'normal';
        case 'c1'
            cfg.scene_complexity_case = 'C1';
            cfg.layout.pattern = 'jittered_grid';
            cfg.layout.orientation_mode = 'fixed';
            cfg.task.position_mode = 'discrete';
            cfg.task.difficulty = 'normal';
        case 'c2'
            cfg.scene_complexity_case = 'C2';
            cfg.layout.pattern = 'jittered_grid';
            cfg.layout.orientation_mode = 'discrete_random';
            cfg.task.position_mode = 'continuous';
            cfg.task.difficulty = 'normal';
        case 'c3'
            cfg.scene_complexity_case = 'C3';
            cfg.layout.pattern = 'cluster';
            cfg.layout.orientation_mode = 'discrete_random';
            cfg.layout.cluster_count = 4;
            cfg.layout.cluster_spacing_m = 4.2;
            cfg.layout.cluster_local_spacing_x_m = 2.4;
            cfg.layout.cluster_local_spacing_y_m = 2.0;
            cfg.task.position_mode = 'continuous';
            cfg.task.difficulty = 'hard_mixed';
        otherwise
            error('scene_complexity_profile_config:UnknownCase', ...
                  'Unknown scene complexity case "%s".', case_id);
    end
end

function aisle = derive_min_aisle_clearance()
    robot = robot_config();
    footprint = hypot(robot.chassis.length, robot.chassis.width);
    aisle = footprint + 0.10;
end
