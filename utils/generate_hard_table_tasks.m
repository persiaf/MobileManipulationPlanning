function tasks_struct = generate_hard_table_tasks(scene_cfg)
%GENERATE_HARD_TABLE_TASKS 为所有桌子生成 hard scenario 任务点。
%
%   tasks_struct = generate_hard_table_tasks(scene_cfg)
%
%   输入:
%       scene_cfg - 场景配置结构体，必须包含:
%                   .table_positions, .table_orientations, .table.length,
%                   .table.width, .task.difficulty, .task_offset_z,
%                   .tasks_per_table_range, .task.hard_mixed_ratio
%
%   输出:
%       tasks_struct - 与 scene.tasks 格式一致的结构体数组
%
%   支持的 difficulty:
%       'hard_corner'         — 调用 sample_task_near_table_corner
%       'hard_two_side'       — 调用 sample_task_on_opposite_edges
%       'hard_reach_boundary' — 调用 sample_task_near_reach_boundary
%       'hard_mixed'          — 混合以上三种

    difficulty = scene_cfg.task.difficulty;
    L = scene_cfg.table.length;
    W = scene_cfg.table.width;
    num_tables = size(scene_cfg.table_positions, 1);
    range = scene_cfg.tasks_per_table_range;
    mixed_ratio = safe_field(safe_field(scene_cfg, 'task', struct()), ...
                             'hard_mixed_ratio', [0.4, 0.4, 0.2]);

    tasks_struct = struct('id', {}, 'x', {}, 'y', {}, 'z', {}, ...
                          'status', {}, 'table_id', {});

    task_id = 0;
    for t = 1:num_tables
        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        theta = scene_cfg.table_orientations(min(t, end));
        n_tasks = randi([range(1), range(2)]);

        switch lower(difficulty)
            case 'hard_corner'
                local_pts = sample_task_near_table_corner(L, W, n_tasks);
            case 'hard_two_side'
                local_pts = sample_task_on_opposite_edges(L, W, n_tasks);
            case 'hard_reach_boundary'
                local_pts = sample_task_near_reach_boundary(L, W, n_tasks);
            case 'hard_mixed'
                local_pts = sample_hard_mixed(L, W, n_tasks, mixed_ratio);
            otherwise
                error('generate_hard_table_tasks: 未知 difficulty "%s"', difficulty);
        end

        R = rot2(theta);
        for i = 1:size(local_pts, 1)
            world = R * local_pts(i, :)';
            task_id = task_id + 1;
            tasks_struct(task_id).x = cx + world(1);
            tasks_struct(task_id).y = cy + world(2);
            tasks_struct(task_id).z = scene_cfg.task_offset_z;
            tasks_struct(task_id).id = task_id;
            tasks_struct(task_id).status = 0;
            tasks_struct(task_id).table_id = t;
        end
    end
end

%% ==================== 内部辅助 ==========================

function local_pts = sample_hard_mixed(L, W, n_tasks, ratio)
%SAMPLE_HARD_MIXED 按比例混合 corner / two_side / reach_boundary 三类任务。
    r_corner = ratio(1);
    r_twoside = ratio(2);
    r_boundary = ratio(3);
    r_sum = r_corner + r_twoside + r_boundary;
    r_corner = r_corner / r_sum;
    r_twoside = r_twoside / r_sum;

    n_corner = max(1, round(n_tasks * r_corner));
    n_twoside = max(1, round(n_tasks * r_twoside));
    n_boundary = n_tasks - n_corner - n_twoside;
    if n_boundary < 1
        n_boundary = 1;
        n_corner = max(1, n_corner - 1);
    end

    pts_c = sample_task_near_table_corner(L, W, n_corner);
    pts_t = sample_task_on_opposite_edges(L, W, n_twoside);
    pts_b = sample_task_near_reach_boundary(L, W, n_boundary);

    all_pts = [pts_c; pts_t; pts_b];
    if size(all_pts, 1) > n_tasks
        sel = randperm(size(all_pts, 1), n_tasks);
        all_pts = all_pts(sel, :);
    end
    local_pts = all_pts;
end

function v = safe_field(s, field_name, default_val)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_val;
    end
end
