function cfg = large_scale_profile_config(scale_id, seed)
%LARGE_SCALE_PROFILE_CONFIG Build the deterministic configuration for a scale.

    switch lower(scale_id)
        case 'smoke'
            base = create_scaled_scene_config('small', seed); target = 4; per_table = [1, 1];
        case 's1'
            base = create_scaled_scene_config('medium', seed); target = 36; per_table = [4, 4];
        case 's2'
            base = create_scaled_scene_config('large', seed); target = 60; per_table = [4, 4];
        case 's3'
            base = create_scaled_scene_config('large', seed); target = 90; per_table = [5, 6];
        case 's4'
            base = create_scaled_scene_config('large', seed); target = 120; per_table = [7, 8];
        case 'l1'
            base = create_p3p4_cluster_profile(seed, 12.0, 20, 5, [4, 5], 90, 'L1');
            target = 90; per_table = [4, 5];
        case 'l2'
            base = create_p3p4_cluster_profile(seed, 15.0, 28, 7, [4, 5], 126, 'L2');
            target = 126; per_table = [4, 5];
        case 'l3'
            base = create_p3p4_cluster_profile(seed, 20.0, 40, 8, [4, 5], 180, 'L3');
            target = 180; per_table = [4, 5];
        case 'l1_jittered'
            base = create_p3p4_jittered_profile(seed, 12.0, 20, [4, 5], 90, 'L1_JITTERED');
            target = 90; per_table = [4, 5];
        case 'v1'
            base = create_p3p4_cluster_profile(seed, 15.0, 28, 7, [4, 5], 126, 'V1');
            base.task.difficulty = 'hard_two_side';
            target = 126; per_table = [4, 5];
        case 'v2'
            base = create_p3p4_cluster_profile(seed, 20.0, 40, 8, [4, 5], 180, 'V2');
            base.layout.cluster_jitter_range_m = 0.12;
            base.visual_static_obstacles = default_visual_static_obstacles('V2');
            target = 180; per_table = [4, 5];
        case 'v3'
            base = create_p3p4_cluster_profile(seed, 15.0, 28, 7, [4, 5], 126, 'V3');
            base.task.difficulty = 'hard_mixed';
            target = 126; per_table = [4, 5];
        otherwise
            error('large_scale_profile_config:UnknownScale', 'Unknown scale %s.', scale_id);
    end
    cfg = base;
    cfg.tasks_per_table_range = per_table;
    cfg.requested_task_count = target;
    if target >= 120
        [x, y] = meshgrid(linspace(-0.24, 0.24, 4), linspace(-0.16, 0.16, 3));
        cfg.task_candidate_local = [x(:), y(:)];
    end
end

function obstacles = default_visual_static_obstacles(label)
    switch upper(label)
        case 'V2'
            obstacles = struct( ...
                'id', {'O1','O2','O3','O4'}, ...
                'type', {'block','block','block','block'}, ...
                'center', {[6.65, 6.70], [13.35, 6.70], [6.65, 14.10], [10.25, 13.35]}, ...
                'size', {[0.55, 2.20], [0.55, 2.20], [0.55, 2.20], [2.20, 0.50]}, ...
                'theta', {0, 0, 0, 0});
        otherwise
            obstacles = struct('id', {}, 'type', {}, 'center', {}, 'size', {}, 'theta', {});
    end
end

function cfg = create_p3p4_cluster_profile(seed, map_size, n_tables, n_regions, per_table, target, label)
    cfg = create_scaled_scene_config('large', seed);
    cfg.map_width_m = map_size;
    cfg.map_height_m = map_size;
    cfg.table_mode = 'layout';
    cfg.layout.pattern = 'cluster';
    cfg.layout.num_tables = n_tables;
    cfg.layout.cluster_count = n_regions;
    if n_regions == 5
        cfg.layout.cluster_cols = 2;
        cfg.layout.cluster_rows = 3;
    elseif n_regions == 7 || n_regions == 8
        cfg.layout.cluster_cols = 3;
        cfg.layout.cluster_rows = 3;
    end
    cfg.layout.cluster_spacing_m = max(5.0, map_size / max(ceil(sqrt(n_regions)), 1));
    cfg.layout.cluster_local_spacing_x_m = 2.20;
    cfg.layout.cluster_local_spacing_y_m = 1.80;
    cfg.layout.cluster_jitter_range_m = 0.02;
    cfg.layout.center = [map_size / 2, map_size / 2];
    cfg.layout.orientation_mode = 'discrete_random';
    cfg.layout.orientation_candidates_deg = [0, 30, -30, 45, -45, 90];
    cfg.layout.max_retries = 80;
    cfg.table_min_aisle_clearance_m = derive_min_aisle_clearance();
    cfg.table_wall_margin_m = 0.45;
    cfg.start_pose = [0.6, 0.6, 0];
    cfg.tasks_per_table_range = per_table;
    cfg.requested_task_count = target;
    cfg.scene_complexity_case = label;
    cfg.task.position_mode = 'continuous';
    cfg.task.difficulty = 'hard_mixed';
end

function cfg = create_p3p4_jittered_profile(seed, map_size, n_tables, per_table, target, label)
    cfg = create_p3p4_cluster_profile(seed, map_size, n_tables, 4, per_table, target, label);
    cfg.layout.pattern = 'jittered_grid';
    cfg.layout.rows = 4;
    cfg.layout.cols = 5;
    cfg.layout.spacing_x_m = 2.25;
    cfg.layout.spacing_y_m = 2.45;
    cfg.layout.jitter_range_m = 0.18;
    cfg.layout = rmfield_if_exists(cfg.layout, {'cluster_count','cluster_spacing_m', ...
        'cluster_local_spacing_x_m','cluster_local_spacing_y_m','cluster_jitter_range_m'});
end

function aisle = derive_min_aisle_clearance()
    robot = robot_config();
    footprint = hypot(robot.chassis.length, robot.chassis.width);
    aisle = footprint + 0.10;
end

function s = rmfield_if_exists(s, names)
    for i = 1:numel(names)
        if isfield(s, names{i})
            s = rmfield(s, names{i});
        end
    end
end
