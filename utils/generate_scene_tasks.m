function tasks_struct = generate_scene_tasks(scene_cfg)
%GENERATE_SCENE_TASKS Generate scene.tasks from task configuration.

    difficulty = getfield_default(getfield_default(scene_cfg, 'task', struct()), ...
                                  'difficulty', 'normal');
    if ~strcmpi(difficulty, 'normal')
        tasks_struct = generate_hard_table_tasks(scene_cfg);
        return;
    end

    switch lower(scene_cfg.task_mode)
        case 'manual'
            tasks_struct = manual_tasks(scene_cfg);
        case 'mixed'
            tasks_struct = mixed_tasks(scene_cfg);
        otherwise
            position_mode = getfield_default(getfield_default(scene_cfg, 'task', struct()), ...
                                             'position_mode', 'discrete');
            switch lower(position_mode)
                case 'discrete'
                    tasks_struct = discrete_random_tasks(scene_cfg);
                case 'continuous'
                    tasks_struct = continuous_random_tasks(scene_cfg);
                otherwise
                    error('generate_scene_tasks:UnknownPositionMode', ...
                          'Unknown task.position_mode "%s".', position_mode);
            end
    end
end

function tasks = discrete_random_tasks(cfg)
    tasks = empty_tasks();
    n_tables = size(cfg.table_positions, 1);
    n_candidates = size(cfg.task_candidate_local, 1);
    range = cfg.tasks_per_table_range;
    task_id = 0;
    for t = 1:n_tables
        n_select = randi([range(1), range(2)]);
        indices = randperm(n_candidates, min(n_select, n_candidates));
        for i = 1:numel(indices)
            local = cfg.task_candidate_local(indices(i), :);
            task_id = task_id + 1;
            tasks(task_id) = local_task_to_struct(cfg, t, local, task_id); %#ok<AGROW>
        end
    end
end

function tasks = continuous_random_tasks(cfg)
    tasks = empty_tasks();
    n_tables = size(cfg.table_positions, 1);
    range = cfg.tasks_per_table_range;
    bounds = continuous_bounds(cfg);
    task_id = 0;
    for t = 1:n_tables
        n_select = randi([range(1), range(2)]);
        xs = bounds(1, 1) + (bounds(1, 2) - bounds(1, 1)) * rand(n_select, 1);
        ys = bounds(2, 1) + (bounds(2, 2) - bounds(2, 1)) * rand(n_select, 1);
        for i = 1:n_select
            task_id = task_id + 1;
            tasks(task_id) = local_task_to_struct(cfg, t, [xs(i), ys(i)], task_id); %#ok<AGROW>
        end
    end
end

function tasks = manual_tasks(cfg)
    tasks = empty_tasks();
    manual = cfg.manual_tasks;
    if isempty(manual)
        warning('generate_scene_tasks:no_manual_tasks', ...
                'task_mode=manual but manual_tasks is empty.');
        return;
    end
    n_tables = size(cfg.table_positions, 1);
    table_ids = unique(manual(:, 1));
    for i = 1:numel(table_ids)
        tid = table_ids(i);
        if tid < 1 || tid > n_tables
            error('generate_scene_tasks:bad_table_id', ...
                  'manual_tasks table_id=%d outside [1, %d].', tid, n_tables);
        end
    end
    for i = 1:size(manual, 1)
        tid = manual(i, 1);
        z = cfg.task_offset_z;
        if size(manual, 2) >= 4
            z = manual(i, 4);
        end
        tasks(i).x = manual(i, 2);
        tasks(i).y = manual(i, 3);
        tasks(i).z = z;
        tasks(i).id = i;
        tasks(i).status = 0;
        tasks(i).table_id = tid;
    end
end

function tasks = mixed_tasks(cfg)
    tasks = empty_tasks();
    manual = cfg.manual_tasks;
    manual_table_ids = [];
    if ~isempty(manual)
        manual_table_ids = unique(manual(:, 1));
    end
    n_tables = size(cfg.table_positions, 1);
    range = cfg.tasks_per_table_range;
    n_candidates = size(cfg.task_candidate_local, 1);
    task_id = 0;
    for t = 1:n_tables
        if ismember(t, manual_table_ids)
            rows = manual(manual(:, 1) == t, :);
            for i = 1:size(rows, 1)
                task_id = task_id + 1;
                z = cfg.task_offset_z;
                if size(rows, 2) >= 4
                    z = rows(i, 4);
                end
                tasks(task_id).x = rows(i, 2);
                tasks(task_id).y = rows(i, 3);
                tasks(task_id).z = z;
                tasks(task_id).id = task_id;
                tasks(task_id).status = 0;
                tasks(task_id).table_id = t;
            end
        else
            n_select = randi([range(1), range(2)]);
            indices = randperm(n_candidates, min(n_select, n_candidates));
            for i = 1:numel(indices)
                task_id = task_id + 1;
                tasks(task_id) = local_task_to_struct(cfg, t, cfg.task_candidate_local(indices(i), :), task_id); %#ok<AGROW>
            end
        end
    end
end

function bounds = continuous_bounds(cfg)
    margin = getfield_default(getfield_default(cfg, 'task', struct()), ...
                              'edge_margin_m', 0.04);
    half_L = max(cfg.table.length / 2 - margin, 0);
    half_W = max(cfg.table.width / 2 - margin, 0);
    default_bounds = [-half_L, half_L; -half_W, half_W];
    bounds = getfield_default(getfield_default(cfg, 'task', struct()), ...
                              'continuous_local_bounds', default_bounds);
end

function task = local_task_to_struct(cfg, table_id, local_xy, task_id)
    center = cfg.table_positions(table_id, :);
    theta = cfg.table_orientations(min(table_id, end));
    world = rot2(theta) * local_xy(:);
    task.x = center(1) + world(1);
    task.y = center(2) + world(2);
    task.z = cfg.task_offset_z;
    task.id = task_id;
    task.status = 0;
    task.table_id = table_id;
end

function tasks = empty_tasks()
    tasks = struct('id', {}, 'x', {}, 'y', {}, 'z', {}, ...
                   'status', {}, 'table_id', {});
end

function v = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end
