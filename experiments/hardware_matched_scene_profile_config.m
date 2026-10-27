function cfg = hardware_matched_scene_profile_config()
%HARDWARE_MATCHED_SCENE_PROFILE_CONFIG Initial measured-scene reconstruction.
% Edit only table_left_bottom and hardware_tasks after ruler measurement.
    cfg = scene_config();
    cfg.profile_id = 'LAB_HARDWARE_MATCHED';
    cfg.map_width_m = 6.2; cfg.map_height_m = 6.6;
    cfg.grid_resolution_m = 0.05; cfg.border_thickness_m = 0.10;
    cfg.table.length = 1.20; cfg.table.width = 0.60;
    cfg.table.height = 0.700;
    cfg.table_mode = 'custom';
    cfg.table_positions = [1.2 2.3; 1.2 3.9; 1.2 5.5; 5.0 2.3; 5.0 3.9; 5.0 5.5];
    cfg.table_left_bottom = [cfg.table_positions(:,1)-cfg.table.length/2, ...
        cfg.table_positions(:,2)-cfg.table.width/2, repmat(cfg.table.length,size(cfg.table_positions,1),1), repmat(cfg.table.width,size(cfg.table_positions,1),1)];
    cfg.num_tables = size(cfg.table_positions,1);
    cfg.table_orientations = zeros(cfg.num_tables,1);
    cfg.table_min_spacing_m = 0.1; cfg.table_wall_margin_m = 0.05;
    cfg.table_start_margin_m = 0.5; cfg.start_pose = [0.35 0.35 0];
    cfg.task_mode = 'manual'; cfg.task_marker.type = 'a4_reference';
    cfg.task_marker.size = [0.297 0.210];
    % [task_id, table_id, local_x, local_y, z]; local coordinates are table-center based.
    cfg.hardware_tasks = [1 4 -0.20 -0.45 NaN; 2 4 +0.20 -0.45 NaN; 3 1 0.00 -0.45 NaN; ...
                          4 2 -0.20 -0.40 NaN; 5 5 0.00 -0.40 NaN; 6 6 0.00 -0.45 NaN; ...
                          7 3 0.00 -0.45 NaN; 8 2 +0.20 -0.40 NaN];
    cfg.manual_tasks = zeros(size(cfg.hardware_tasks,1),4);
    for k=1:size(cfg.hardware_tasks,1)
        tid=cfg.hardware_tasks(k,2); xy=table_local_to_world(cfg.table_positions(tid,:),0,cfg.hardware_tasks(k,3:4));
        z=cfg.hardware_tasks(k,5); if ~isfinite(z), z=cfg.table.height; end
        cfg.manual_tasks(k,:)=[tid xy z];
        fprintf('Task %d (T%d) world = [%.2f, %.2f, %.2f]\n', k, tid, xy(1), xy(2), z);
    end
    cfg.task_offset_z = cfg.table.height;
    cfg.safety_margin_m = 0.05; cfg.docking_min_distance_m=0.25;
    cfg.docking_max_distance_m=0.40; cfg.docking_distance_step_m=0.05;
    cfg.docking_lateral_step_m=0.10;
end
