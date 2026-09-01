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
