function scene = build_scene_from_cfg(scene_cfg)
%BUILD_SCENE_FROM_CFG 使用预先构造的场景配置构建场景。
%
%   scene = build_scene_from_cfg(scene_cfg)
%
%   与 build_scene.m 功能相同，但接受外部传入的 scene_cfg，
%   不调用 scene_config()，用于随机批量实验等需要动态切换场景的场景。
%
%   输入:
%       scene_cfg - 完整场景配置结构体（通常来自 create_scaled_scene_config）
%
%   输出:
%       scene - 与 build_scene() 相同格式的 scene 结构体

    % —— 解析桌子位置 ——
    scene_cfg = resolve_table_positions(scene_cfg);
    if exist('validate_table_layout', 'file') == 2
        [layout_ok, layout_report] = validate_table_layout(scene_cfg);
        scene_cfg.layout_validation = layout_report;
        if ~layout_ok
            error('build_scene_from_cfg:InvalidTableLayout', ...
                  'Invalid table layout: %s', layout_report.failure_reason);
        end
    end

    % —— 构建栅格地图 ——
    [grid_map, map_info] = build_grid_map(scene_cfg);

    % —— 设置随机种子 ——
    if strcmp(scene_cfg.random_seed, 'shuffle')
        rng('shuffle');
    elseif ~isempty(scene_cfg.random_seed)
        rng(scene_cfg.random_seed);
    end

    % —— 生成任务 ——
    scene.tasks = generate_scene_tasks(scene_cfg);
    num_tasks = length(scene.tasks);
    scene_cfg.actual_task_count = num_tasks;

    scene.grid_map  = grid_map;
    scene.map_info  = map_info;
    scene.scene_cfg = scene_cfg;
    scene.start_pose = scene_cfg.start_pose;
    scene.num_tasks  = num_tasks;
    scene.tasks = normalize_task_fields(scene.tasks);
    if isfield(scene_cfg, 'static_obstacles') && ~isempty(scene_cfg.static_obstacles)
        obs = repmat(struct('id','','type','block','center',[0 0],'size',[0 0],'theta',0), size(scene_cfg.static_obstacles,1), 1);
        for oi=1:numel(obs)
            q=scene_cfg.static_obstacles(oi,:); obs(oi).id=sprintf('keep_out_%d',oi); obs(oi).center=q(1:2)+q(3:4)/2; obs(oi).size=q(3:4);
        end
        scene = apply_static_obstacles_to_scene(scene, obs);
    end
    scene.region_metadata = build_scene_region_metadata(scene, []);
end

function metadata = build_scene_region_metadata(scene, selected_pose_region_ids)
    scene_cfg = scene.scene_cfg;
    table_count = size(scene_cfg.table_positions, 1);
    if isfield(scene_cfg, 'table_region_id') && numel(scene_cfg.table_region_id) == table_count
        table_region_id = reshape(scene_cfg.table_region_id, [], 1);
    else
        table_region_id = (1:table_count)';
    end

    task_region_id = zeros(scene.num_tasks, 1);
    for ti = 1:scene.num_tasks
        if isfield(scene.tasks(ti), 'table_id') && scene.tasks(ti).table_id >= 1 && ...
                scene.tasks(ti).table_id <= table_count
            task_region_id(ti) = table_region_id(scene.tasks(ti).table_id);
        end
    end
    region_ids = unique(table_region_id(table_region_id > 0), 'stable');
    metadata = struct();
    metadata.table_region_id = table_region_id;
    metadata.task_region_id = task_region_id;
    metadata.region_ids = region_ids;
    metadata.region_count = numel(region_ids);
    metadata.tables_per_region = count_by_region(table_region_id, region_ids);
    metadata.tasks_per_region = count_by_region(task_region_id, region_ids);
    metadata.candidate_count_per_region = zeros(numel(region_ids), 1);
    if nargin >= 2 && ~isempty(selected_pose_region_ids)
        metadata.dock_region_id = reshape(selected_pose_region_ids, [], 1);
        metadata.selected_docks_per_region = count_by_region(metadata.dock_region_id, region_ids);
    else
        metadata.dock_region_id = [];
        metadata.selected_docks_per_region = zeros(numel(region_ids), 1);
    end
end

function counts = count_by_region(ids, region_ids)
    counts = zeros(numel(region_ids), 1);
    for i = 1:numel(region_ids)
        counts(i) = sum(ids == region_ids(i));
    end
end

%% ==================== 局部函数（从 build_scene.m 复制） ====================

function cfg = resolve_table_positions(cfg)
    switch lower(cfg.table_mode)
        case 'layout'
            cfg = resolve_layout_with_retries(cfg);
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

function cfg = resolve_layout_with_retries(cfg)
    if strcmp(cfg.random_seed, 'shuffle')
        rng('shuffle');
    elseif ~isempty(cfg.random_seed)
        rng(cfg.random_seed);
    end

    max_retries = safe_field(safe_field(cfg, 'layout', struct()), 'max_retries', 1);
    last_report = struct('failure_reason', 'layout was not generated');
    for attempt = 1:max_retries
        candidate = generate_table_layout(cfg);
        [ok, report] = validate_table_layout(candidate);
        candidate.layout_generation_attempts = attempt;
        candidate.layout_validation = report;
        if ok
            cfg = candidate;
            return;
        end
        last_report = report;
    end
    error('build_scene_from_cfg:LayoutGenerationFailed', ...
          'Failed to generate a valid layout after %d attempts: %s', ...
          max_retries, last_report.failure_reason);
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
        cx = margin + (cfg.map_width_m - 2*margin) * rand();
        cy = margin + (cfg.map_height_m - 2*margin) * rand();
        if norm([cx, cy] - start_pt) < start_margin, continue; end
        overlap = false;
        for i = 1:count
            if norm([cx, cy] - positions(i, :)) < min_space
                overlap = true; break;
            end
        end
        if overlap, continue; end
        count = count + 1;
        positions(count, :) = [cx, cy];
        orient(count) = randi([0, 1]) * pi;
    end
    if count < n
        warning('build_scene_from_cfg:partial', ...
                '随机生成桌子: 仅成功生成 %d/%d 张桌子 (尝试 %d 次)', count, n, max_attempts);
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
        warning('build_scene_from_cfg:no_manual', ...
                'task_mode=manual 但 manual_tasks 为空，未生成任务点');
        return;
    end
    table_ids = unique(manual(:, 1));
    for tidx = 1:length(table_ids)
        tid = table_ids(tidx);
        if tid < 1 || tid > num_tables
            error('build_scene_from_cfg:bad_table_id', ...
                  'manual_tasks 中 table_id=%d 超出范围 [1, %d]', tid, num_tables);
        end
    end
    for mi = 1:size(manual, 1)
        tid = manual(mi, 1);
        tx  = manual(mi, 2);
        ty  = manual(mi, 3);
        tz = scene_cfg.task_offset_z;
        if size(manual, 2) >= 4, tz = manual(mi, 4); end
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
    num_candidates = size(scene_cfg.task_candidate_local, 1);
    range = scene_cfg.tasks_per_table_range;
    for t = 1:num_tables
        if ismember(t, manual_table_ids)
            mat = manual(manual(:, 1) == t, :);
            for mi = 1:size(mat, 1)
                tx = mat(mi, 2); ty = mat(mi, 3);
                tz = scene_cfg.task_offset_z;
                if size(mat, 2) >= 4, tz = mat(mi, 4); end
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
end

function v = safe_field(s, field_name, default_val)
%SAFE_FIELD 安全读取结构体字段，缺失时返回 default_val
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_val;
    end
end
