function cfg = create_scaled_scene_config(scale, trial_seed)
%CREATE_SCALED_SCENE_CONFIG 按实验规模生成完整场景配置。
%
%   cfg = create_scaled_scene_config(scale, trial_seed)
%
%   输入:
%       scale      - 'small', 'medium', 或 'large'
%       trial_seed - 本次实验的随机种子（用于任务点随机生成）
%
%   输出:
%       cfg - 完整的 scene_config 结构体，可直接传给 build_scene_from_cfg
%
%   规模定义:
%       small:  6x6 m 地图, 2x2=4 张桌子,  ~8-12 任务点
%       medium: 9x9 m 地图, 3x3=9 张桌子, ~18-27 任务点
%       large: 12x12 m 地图, 4x4=16 张桌子, ~32-48 任务点
%
%   注意: 本函数不修改 scene_config.m，仅返回独立 cfg，不影响主流程。

    %% ===================== 规模参数表 =====================
    switch lower(scale)
        case 'small'
            map_size    = 6.0;
            grid_cols   = 2;
            grid_rows   = 2;
            spacing     = 2.5;
        case 'medium'
            map_size    = 9.0;
            grid_cols   = 3;
            grid_rows   = 3;
            spacing     = 2.5;
        case 'large'
            map_size    = 12.0;
            grid_cols   = 4;
            grid_rows   = 4;
            spacing     = 2.5;
        otherwise
            error('create_scaled_scene_config: 未知规模 "%s"，可选: small / medium / large', scale);
    end

    %% ===================== 基础场景参数 =====================
    cfg.map_width_m  = map_size;
    cfg.map_height_m = map_size;
    cfg.grid_resolution_m = 0.05;
    cfg.border_thickness_m = 0.1;

    cfg.table.length = 1.200;
    cfg.table.width  = 0.400;
    cfg.table.height = 0.500;

    %% ===================== 桌子布局 (网格) =====================
    cfg.table_mode = 'custom';
    n_tables = grid_cols * grid_rows;
    margin = (map_size - (grid_cols - 1) * spacing) / 2;
    cfg.table_positions = zeros(n_tables, 2);
    idx = 1;
    for row = 1:grid_rows
        for col = 1:grid_cols
            cfg.table_positions(idx, 1) = margin + (col - 1) * spacing;
            cfg.table_positions(idx, 2) = margin + (row - 1) * spacing;
            idx = idx + 1;
        end
    end
    cfg.num_tables = n_tables;
    cfg.table_orientations = zeros(cfg.num_tables, 1);

    %% ===================== 安全检查参数 =====================
    cfg.table_min_spacing_m = spacing * 0.7;
    cfg.table_wall_margin_m = 0.4;
    cfg.table_start_margin_m = 0.8;

    %% ===================== 起点 =====================
    cfg.start_pose = [0.5, 0.5, 0];

    %% ===================== 任务点 =====================
    cfg.task_mode = 'random';
    cfg.tasks_per_table_range = [2, 3];
    cfg.random_seed = trial_seed;

    cfg.task_candidate_local = [
        -0.18,  0.12;
         0.00,  0.12;
         0.18,  0.12;
        -0.18, -0.12;
         0.00, -0.12;
         0.18, -0.12
    ];

    cfg.manual_tasks = [];

    % 任务分布难度（默认 normal，不改变随机批量实验结果）
    cfg.task.difficulty = 'normal';
    cfg.task.hard_mixed_ratio = [0.4, 0.4, 0.2];

    %% ===================== 任务和停靠相关参数 =====================
    cfg.task_offset_z = cfg.table.height;
    cfg.safety_margin_m = 0.05;
    cfg.docking_min_distance_m = 0.25;
    cfg.docking_max_distance_m = cfg.docking_min_distance_m + 0.15;
    cfg.docking_distance_step_m = 0.05;
    cfg.docking_lateral_step_m  = 0.15;
end
