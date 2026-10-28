function scene = build_scene()
    scene_cfg = scene_config();
    scene_cfg = resolve_table_positions(scene_cfg);

    [grid_map, map_info] = build_grid_map(scene_cfg);

    scene.tasks = struct('id', {}, 'x', {}, 'y', {}, 'z', {}, ...
                         'status', {}, 'table_id', {});
    num_tasks = 0;

    if strcmp(scene_cfg.random_seed, 'shuffle')
        rng('shuffle');
    elseif ~isempty(scene_cfg.random_seed)
        rng(scene_cfg.random_seed);
    end

    num_tables = size(scene_cfg.table_positions, 1);

    difficulty = safe_field(safe_field(scene_cfg, 'task', struct()), ...
                            'difficulty', 'normal');
    if ~strcmpi(difficulty, 'normal')
        hard_tasks = generate_hard_table_tasks(scene_cfg);
        scene.tasks = hard_tasks;
        num_tasks = length(hard_tasks);
    else
        switch lower(scene_cfg.task_mode)
            case 'manual'
                [scene, num_tasks] = generate_manual_tasks(scene, scene_cfg, num_tables);
            case 'mixed'
                [scene, num_tasks] = generate_mixed_tasks(scene, scene_cfg, num_tables);
            otherwise
                [scene, num_tasks] = generate_random_tasks(scene, scene_cfg, num_tables);
        end
    end

    validate_tabletop_tasks(scene_cfg, scene.tasks);
    scene.grid_map = grid_map;
    scene.map_info = map_info;
    scene.scene_cfg = scene_cfg;
    scene.start_pose = scene_cfg.start_pose;
    scene.num_tasks = num_tasks;
    scene.tasks = normalize_task_fields(scene.tasks);
end


function cfg = resolve_table_positions(cfg)
    switch lower(cfg.table_mode)
        case 'custom'
            cfg.num_tables = size(cfg.table_positions, 1);
            if length(cfg.table_orientations) < cfg.num_tables
                cfg.table_orientations(end+1:cfg.num_tables) = 0;
            end
        case 'random'
            cfg = generate_random_tables(cfg);
        otherwise
            cfg.num_tables = 6;
            cfg.table_positions = [
                2.0, 1.5; 4.0, 1.5; 6.0, 1.5;
                2.0, 4.0; 4.0, 4.0; 6.0, 4.0
            ];
            cfg.table_orientations = [0, 0, 0, 0, 0, 0];
    end
end


function cfg = generate_random_tables(cfg)
    n = cfg.num_tables;
    L = cfg.table.length;
    W = cfg.table.width;
    min_space = cfg.table_min_spacing_m;
    wall_margin = cfg.table_wall_margin_m;
    start_margin = cfg.table_start_margin_m;
    max_attempts = 500;

    half_diag = sqrt(L^2 + W^2) / 2;
    margin = max(wall_margin, half_diag + 0.1);
    start_pt = cfg.start_pose(1:2);

    positions = zeros(n, 2);
    orient = zeros(1, n);
    count = 0;
    attempt = 0;

    while count < n && attempt < max_attempts
        attempt = attempt + 1;
        cx = margin + (cfg.map_width_m - 2 * margin) * rand();
        cy = margin + (cfg.map_height_m - 2 * margin) * rand();

        if norm([cx, cy] - start_pt) < start_margin
            continue;
        end

        overlap = false;
        for i = 1:count
            if norm([cx, cy] - positions(i, :)) < min_space
                overlap = true;
                break;
            end
        end
        if overlap
            continue;
        end

        count = count + 1;
        positions(count, :) = [cx, cy];
        orient(count) = randi([0, 1]) * pi;
    end

    if count < n
        warning('build_scene:partial', ...
                '随机生成桌子: 仅成功生成 %d/%d 张桌子 (尝试 %d 次)\n', count, n, max_attempts);
    end

    cfg.num_tables = count;
    cfg.table_positions = positions(1:count, :);
    cfg.table_orientations = orient(1:count);
end


function [scene, num_tasks] = generate_random_tasks(scene, scene_cfg, num_tables)
    num_tasks = 0;
    num_candidates = size(scene_cfg.task_candidate_local, 1);
    range = scene_cfg.tasks_per_table_range;

    for t = 1:num_tables
        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        theta = scene_cfg.table_orientations(min(t, end));
        R = rot2(theta);

        num_select = randi([range(1), range(2)]);
        indices = randperm(num_candidates, min(num_select, num_candidates));

        for si = 1:length(indices)
            tp = indices(si);
            local = scene_cfg.task_candidate_local(tp, :)';
            world = R * local;
            num_tasks = num_tasks + 1;
            scene.tasks(num_tasks).x = cx + world(1);
            scene.tasks(num_tasks).y = cy + world(2);
            scene.tasks(num_tasks).z = scene_cfg.task_offset_z;
            scene.tasks(num_tasks).id = num_tasks;
            scene.tasks(num_tasks).status = 0;
            scene.tasks(num_tasks).table_id = t;
        end
    end
end


function [scene, num_tasks] = generate_manual_tasks(scene, scene_cfg, num_tables)
    num_tasks = 0;
    manual = scene_cfg.manual_tasks;
    if isempty(manual)
        warning('build_scene:no_manual_tasks', ...
                'task_mode=manual 但 manual_tasks 为空，未生成任务点');
        return;
    end

    table_ids = unique(manual(:, 1));
    for tidx = 1:length(table_ids)
        tid = table_ids(tidx);
        if tid < 1 || tid > num_tables
            error('build_scene:bad_table_id', ...
                  'manual_tasks 中 table_id=%d 超出范围 [1, %d]', tid, num_tables);
        end
    end

    for mi = 1:size(manual, 1)
        tid = manual(mi, 1);
        tx  = manual(mi, 2);
        ty  = manual(mi, 3);
        if size(manual, 2) >= 4
            tz = manual(mi, 4);
        else
            tz = scene_cfg.task_offset_z;
        end
        num_tasks = num_tasks + 1;
        scene.tasks(num_tasks).x = tx;
        scene.tasks(num_tasks).y = ty;
        scene.tasks(num_tasks).z = tz;
        scene.tasks(num_tasks).id = num_tasks;
        scene.tasks(num_tasks).status = 0;
        scene.tasks(num_tasks).table_id = tid;
    end
end


function [scene, num_tasks] = generate_mixed_tasks(scene, scene_cfg, num_tables)
    num_tasks = 0;
    manual = scene_cfg.manual_tasks;

    manual_table_ids = [];
    if ~isempty(manual)
        manual_table_ids = unique(manual(:, 1));
    end

    for t = 1:num_tables
        if ismember(t, manual_table_ids)
            mat = manual(manual(:, 1) == t, :);
            for mi = 1:size(mat, 1)
                tx = mat(mi, 2);
                ty = mat(mi, 3);
                if size(mat, 2) >= 4
                    tz = mat(mi, 4);
                else
                    tz = scene_cfg.task_offset_z;
                end
                num_tasks = num_tasks + 1;
                scene.tasks(num_tasks).x = tx;
                scene.tasks(num_tasks).y = ty;
                scene.tasks(num_tasks).z = tz;
                scene.tasks(num_tasks).id = num_tasks;
                scene.tasks(num_tasks).status = 0;
                scene.tasks(num_tasks).table_id = t;
            end
        else
            cx = scene_cfg.table_positions(t, 1);
            cy = scene_cfg.table_positions(t, 2);
            theta = scene_cfg.table_orientations(min(t, end));
            R = rot2(theta);
            nc = size(scene_cfg.task_candidate_local, 1);
            rg = scene_cfg.tasks_per_table_range;
            ns = randi([rg(1), rg(2)]);
            idx = randperm(nc, min(ns, nc));
            for si = 1:length(idx)
                local = scene_cfg.task_candidate_local(idx(si), :)';
                world = R * local;
                num_tasks = num_tasks + 1;
                scene.tasks(num_tasks).x = cx + world(1);
                scene.tasks(num_tasks).y = cy + world(2);
                scene.tasks(num_tasks).z = scene_cfg.task_offset_z;
                scene.tasks(num_tasks).id = num_tasks;
                scene.tasks(num_tasks).status = 0;
                scene.tasks(num_tasks).table_id = t;
            end
        end
    end
end


function v = safe_field(s, field_name, default_val)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_val;
    end
end
