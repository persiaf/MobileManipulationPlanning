function cfg = scene_complexity_factor_config(factor_id, seed)
%SCENE_COMPLEXITY_FACTOR_CONFIG Build the P1.5 2x2x2 factor matrix.

    factor_id = upper(char(factor_id));
    if numel(factor_id) ~= 4 || factor_id(1) ~= 'F'
        error('scene_complexity_factor_config:BadId', ...
              'Factor id must be F000..F111.');
    end
    bits = factor_id(2:4) - '0';
    if any(bits < 0 | bits > 1)
        error('scene_complexity_factor_config:BadId', ...
              'Factor id must be F000..F111.');
    end

    cfg = scene_complexity_profile_config('C0', seed);
    layout_bit = bits(1);
    orientation_bit = bits(2);
    task_bit = bits(3);

    if layout_bit == 0
        cfg.layout.pattern = 'grid';
    else
        cfg.layout.pattern = 'jittered_grid';
    end

    if orientation_bit == 0
        cfg.layout.orientation_mode = 'fixed';
    else
        cfg.layout.orientation_mode = 'discrete_random';
    end

    if task_bit == 0
        cfg.task.position_mode = 'discrete';
    else
        cfg.task.position_mode = 'continuous';
    end

    cfg.task.difficulty = 'normal';
    cfg.scene_complexity_case = factor_id;
end
