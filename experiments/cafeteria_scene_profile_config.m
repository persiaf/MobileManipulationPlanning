function cfg = cafeteria_scene_profile_config(task_mode, seed)
%CAFETERIA_SCENE_PROFILE_CONFIG Fixed cafeteria 1x3/2x3 experiment profile.

    if nargin < 1 || isempty(task_mode), task_mode = 'complex12'; end
    if isnumeric(task_mode)
        seed = task_mode;
        task_mode = 'complex12';
    elseif nargin < 2 || isempty(seed)
        seed = 2026;
    end
    task_mode = lower(char(task_mode));
    cfg = scene_config();
    cfg.profile_id = 'CAFETERIA_1X3_2X3';
    cfg.map_width_m = 9.2;
    cfg.map_height_m = 8.2;
    cfg.grid_resolution_m = 0.05;
    cfg.border_thickness_m = 0.10;
    cfg.start_pose = [0.8, 0.8, 0];
    cfg.random_seed = seed;

    cfg.table.length = 1.20;
    cfg.table.width = 0.60;
    cfg.table.height = 0.75;
    cfg.cafeteria.table_height_m = 0.75;
    cfg.cafeteria.height_source = 'assumed_unmeasured';
    cfg.table_mode = 'custom';
    cfg.table_positions = [
        1.80, 1.40; 1.80, 4.00; 1.80, 6.60;
        5.10, 1.40; 6.45, 1.40;
        5.10, 4.00; 6.45, 4.00;
        5.10, 6.60; 6.45, 6.60];
    cfg.num_tables = size(cfg.table_positions, 1);
    cfg.table_orientations = zeros(cfg.num_tables, 1);
    cfg.table_region_id = [1; 1; 1; 2; 2; 3; 3; 4; 4];
    cfg.table_min_spacing_m = 1.0;
    cfg.table_min_aisle_clearance_m = 0.149;
    cfg.table_wall_margin_m = cfg.border_thickness_m;
    cfg.table_start_margin_m = 0.8;

    cfg.cafeteria.main_aisle_x = [2.40, 4.50];
    cfg.cafeteria.main_aisle_width_m = diff(cfg.cafeteria.main_aisle_x);
    cfg.cafeteria.right_table_gap_m = 0.15;
    cfg.cafeteria.right_table_gap_is_corridor = false;
    cfg.cafeteria.task_mode = task_mode;
    cfg.cafeteria.planning_mode = 'full_manipulation';
    cfg.cafeteria.service_tangential_tolerance_m = 0.30;
    cfg.cafeteria.chairs.enable = false;
    cfg.cafeteria.chairs.phase_2_note = 'movable chair obstacles for P5 local replanning';

    cfg.task_mode = 'manual';
    cfg.task_offset_z = cfg.cafeteria.table_height_m;
    cfg.task_marker.type = 'a4_icon';
    cfg.task_marker.size = [0.297, 0.210];
    switch task_mode
        case 'basic3'
            cfg.task_local_specs = [2, 0.43, 0.00; 4, -0.43, 0.00; 9, 0.43, 0.00];
        case 'complex12'
            cfg.task_local_specs = [
                1,  0.43,  0.00;
                2,  0.35,  0.12;
                2,  0.35, -0.12;
                3,  0.43,  0.00;
                4, -0.35,  0.12;
                4, -0.35, -0.12;
                6,  0.00,  0.17;
                7,  0.00, -0.17;
                8, -0.35,  0.12;
                8, -0.35, -0.12;
                9,  0.35,  0.12;
                9,  0.35, -0.12];
        otherwise
            error('cafeteria_scene_profile_config:UnknownTaskMode', ...
                'Expected basic3 or complex12; received "%s".', task_mode);
    end
    cfg.active_table_ids = unique(cfg.task_local_specs(:, 1), 'stable')';
    cfg.manual_tasks = zeros(size(cfg.task_local_specs, 1), 4);
    for i = 1:size(cfg.task_local_specs, 1)
        table_id = cfg.task_local_specs(i, 1);
        world_xy = table_local_to_world(cfg.table_positions(table_id, :), ...
            cfg.table_orientations(table_id), cfg.task_local_specs(i, 2:3));
        cfg.manual_tasks(i, :) = [table_id, world_xy, cfg.task_offset_z];
    end
    cfg.requested_task_count = size(cfg.manual_tasks, 1);

    cfg.safety_margin_m = 0.08;
    cfg.docking_min_distance_m = 0.25;
    cfg.docking_max_distance_m = 0.40;
    cfg.docking_distance_step_m = 0.05;
    cfg.docking_lateral_step_m = 0.10;
end
