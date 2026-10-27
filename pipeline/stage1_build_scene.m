function [robot_cfg, scene_cfg, planner_cfg, scene, task_table_map, regions, effective] = stage1_build_scene(effective)
    fprintf('===== 移动操作任务规划系统（RCH-MT-MMP）=====\n');

    if nargin < 1 || isempty(effective)
        effective = resolve_effective_config();
    end
    robot_cfg = effective.robot;
    scene_cfg = effective.scene;
    planner_cfg = effective.planner;
    planner_cfg.robot_cfg = robot_cfg;

    fprintf('\n--- 1. 场景构建 ---\n');
    fprintf('  模式: table=%s, task=%s\n', scene_cfg.table_mode, scene_cfg.task_mode);

    scene = build_scene_from_cfg(scene_cfg);
    scene_cfg = scene.scene_cfg;

    fprintf('  地图: %.1f x %.1f m, 分辨率 %.2f m\n', ...
            scene.map_info.width_m, scene.map_info.height_m, scene.map_info.resolution);
    fprintf('  桌子数量: %d\n', size(scene_cfg.table_positions, 1));
    for t = 1:size(scene_cfg.table_positions, 1)
        fprintf('    桌子 %d: (%.1f, %.1f, %.1f°)\n', ...
                t, scene_cfg.table_positions(t,1), scene_cfg.table_positions(t,2), ...
                rad2deg(scene_cfg.table_orientations(min(t,end))));
    end
    fprintf('  任务点数量: %d\n', scene.num_tasks);
    fprintf('  机器人起点: (%.2f, %.2f, %.1f°)\n', ...
            scene.start_pose(1), scene.start_pose(2), rad2deg(scene.start_pose(3)));

    fprintf('\n--- 2. 场景合法性检查 ---\n');
    if exist('validate_scene_config', 'file') ~= 2
        error(['未找到 validate_scene_config.m。请确认文件位于 environment/validate_scene_config.m，' ...
               '并且 main_mobile_manipulation_planning.m 已调用 init_paths(project_root)。']);
    end
    scene_ok = validate_scene_config(scene_cfg, scene.grid_map, scene.map_info);
    if ~scene_ok
        warning('stage1_build_scene:InvalidScene', '场景配置存在警告或错误，请先检查桌子、任务点、起点和地图边界。');
    end

    if planner_cfg.visualization.show_scene && ~runtime_flag(planner_cfg, 'disable_visualization', false)
        hFig1 = figure('Name', '阶段一: 场景初始化', 'Position', [80, 80, 900, 700]);
        draw_scene(scene, hFig1);
        title('阶段一：场景初始化 — 地图 & 桌子 & 任务点');
        drawnow;
    end

    fprintf('\n--- 3. 任务归属判断 ---\n');
    task_table_map = assign_tasks_to_tables(scene);
    for t = 1:length(task_table_map)
        if ~isempty(task_table_map{t})
            fprintf('  桌子 %d: 任务点 [%s]\n', t, num2str(task_table_map{t}'));
        else
            fprintf('  桌子 %d: 无任务点 (跳过规划)\n', t);
        end
    end

    fprintf('\n--- 4. 构建任务区域 ---\n');
    regions = build_task_regions(scene, task_table_map);
    fprintf('  任务区域数量: %d (有任务桌子数: %d)\n', ...
            length(regions), sum(~cellfun(@isempty, task_table_map)));

    empty_tables = find(cellfun(@isempty, task_table_map));
    for et = empty_tables(:)'
        fprintf('  警告: 桌子 %d 无分配任务点，已跳过区域规划\n', et);
    end
end

function tf = runtime_flag(cfg, field_name, default_value)
    tf = default_value;
    if isfield(cfg, 'runtime') && isfield(cfg.runtime, field_name)
        tf = logical(cfg.runtime.(field_name));
    end
end
