function cfg = scene_config()
    %% ============================================================
    %  scene_config.m
    %
    %  功能：
    %  1. 配置地图大小、栅格分辨率和边界厚度；
    %  2. 配置桌子数量、桌子位置、桌子朝向和桌子尺寸；
    %  3. 配置任务点生成方式，包括随机任务和手动任务；
    %  4. 配置停靠位姿候选生成参数。
    %
    %  注意：
    %  后续修改场景时，优先修改本文件。
    %  不建议直接修改 build_scene.m、build_grid_map.m 或主程序。
    %% ============================================================


    %% ===================== 1. 地图参数 ==========================
    % 地图宽度，单位 m，对应 X 方向范围：0 ~ map_width_m
    cfg.map_width_m  = 10.0; %%%%%%%%%%%%%%   8.0

    % 地图高度，单位 m，对应 Y 方向范围：0 ~ map_height_m
    cfg.map_height_m = 10.0;  %%%%%%%%%%%%%%   6.0

    % 栅格地图分辨率，单位 m/cell
    % 0.05 表示每个栅格代表 5 cm
    cfg.grid_resolution_m = 0.05;

    % 地图边界障碍物厚度，单位 m
    % 例如 0.1 m 表示地图四周有 10 cm 厚的黑色边界障碍
    cfg.border_thickness_m = 0.1;


    %% ===================== 2. 桌子尺寸参数 ======================
    % 桌子长度，单位 m
    % 当前为 1.2 m，对应 1200 mm
    cfg.table.length = 1.200;

    % 桌子宽度，单位 m
    % 当前为 0.4 m，对应 400 mm
    cfg.table.width  = 0.400;

    % 桌面高度，单位 m
    % 当前为 0.5 m，对应 500 mm
    % 任务点 z 坐标默认等于该高度
    cfg.table.height = 0.500;


    %% ===================== 3. 桌子布局模式 ======================
    % table_mode 用于控制桌子布局方式
    %
    % 'custom'  : 使用 cfg.table_positions 手动指定桌子中心位置；
    % 'default' : 使用程序默认桌子布局，如果 build_scene 中实现了 default 模式；
    % 'random'  : 随机生成桌子位置，如果 build_scene 中实现了 random 模式。
    %
    % 当前推荐使用 'custom'，最稳定，也最适合论文实验。
    cfg.table_mode = 'custom';

    % 桌子数量
    % 推荐写法：由 table_positions 的行数自动决定
    % 这样你修改 table_positions 后，不需要手动再改 num_tables
    cfg.table_positions = [
        2.0, 1.5;
        4.0, 1.5;
        6.0, 1.5;
        8.0, 1.5;
        2.0, 4.0;
        4.0, 4.0;
        6.0, 4.0;
        8.0, 4.0;
        2.0, 6.5;
        4.0, 6.5;
        6.0, 6.5;
        8.0, 6.5;
        2.0, 9.0;
        4.0, 9.0;
        6.0, 9.0;
        8.0, 9.0
    ];

    cfg.num_tables = size(cfg.table_positions, 1);

    % 桌子朝向，单位 rad
    % 每个元素对应一张桌子的朝向
    %
    % deg2rad(0)  表示桌子不旋转；
    % deg2rad(90) 表示桌子旋转 90 度；
    % deg2rad(30) 表示桌子旋转 30 度。
    %
    % 注意：
    % table_orientations 的数量必须等于 cfg.num_tables。
    cfg.table_orientations = zeros(cfg.num_tables, 1);

    % 如果想手动设置每张桌子的朝向，可以使用下面这种写法：
    % cfg.table_orientations = [
    %     deg2rad(0);
    %     deg2rad(0);
    %     deg2rad(90);
    %     deg2rad(0);
    %     deg2rad(30);
    %     deg2rad(-30)
    % ];


    %% ===================== 4. 场景安全检查参数 ==================
    % 桌子之间的最小中心距离，单位 m
    % 若两张桌子中心距离小于该值，validate_scene_config 可能报警
    cfg.table_min_spacing_m = 1.0;

    % 桌子距离墙壁的最小安全距离，单位 m
    % 防止桌子太靠近边界，导致底盘无法绕行或候选位姿生成失败
    cfg.table_wall_margin_m = 0.4;

    % 起点与桌子中心之间的最小距离，单位 m
    % 防止机器人起点与桌子或障碍物重叠
    cfg.table_start_margin_m = 0.8;


    %% ===================== 5. 机器人起点 ========================
    % 机器人起始位姿 [x, y, theta]
    % x, y 单位 m，theta 单位 rad
    % 当前起点在地图左下方，朝向 0 度
    cfg.start_pose = [0.5, 0.5, 0];


    %% ===================== 6. 任务点生成模式 =====================
    % task_mode 用于控制任务点生成方式
    %
    % 'random' : 每张桌子从 task_candidate_local 中随机选取若干任务点；
    % 'manual' : 使用 cfg.manual_tasks 手动指定任务点；
    % 'mixed'  : 混合模式，如果 build_scene 中实现了 mixed 模式。
    %
    % 当前推荐：
    % - 日常算法测试使用 'random'
    % - 论文固定实验使用 'manual'
    cfg.task_mode = 'random';

    % task.difficulty 用于控制任务点空间分布难度
    %   'normal'              — 默认，使用 task_candidate_local 随机选取
    %   'hard_corner'         — 任务点靠近桌面四角
    %   'hard_two_side'       — 任务点分布在桌子相对两侧
    %   'hard_reach_boundary' — 任务点靠近机械臂可达边界（桌边全周分布）
    %   'hard_mixed'          — 混合以上三种 hard 模式
    cfg.task.difficulty = 'normal';
    % hard_mixed 模式下三种类型的混合比例 [corner, two_side, reach_boundary]
    cfg.task.hard_mixed_ratio = [0.4, 0.4, 0.2];

    % 每张桌子随机生成任务点数量范围
    % [2, 3] 表示每张桌子随机生成 2 到 3 个任务点
    cfg.tasks_per_table_range = [2, 3];

    % 随机种子
    %
    % 'shuffle' 表示每次运行都随机；
    % 固定数字，例如 2026，表示每次运行生成完全相同的任务点。
    %
    % 论文实验建议使用固定数字，方便复现实验。

%     cfg.random_seed = 'shuffle';
    cfg.random_seed = 2026;


    %% ===================== 7. 随机任务候选点 =====================
    % task_candidate_local 是桌子局部坐标系下的任务候选点。
    %
    % 每一行表示一个候选任务点 [local_x, local_y]，单位 m。
    % local_x 沿桌子长度方向；
    % local_y 沿桌子宽度方向。
    %
    % 对于 1.2 m x 0.4 m 桌子：
    % local_x 推荐范围：-0.50 ~ 0.50
    % local_y 推荐范围：-0.18 ~ 0.18
    %
    % 当前设置为 6 个候选点：
    % 上边 3 个，下边 3 个。
    % 它们不会在桌子正中心，而是靠近桌边，更方便机械臂从桌边操作。

    cfg.task_candidate_local = [
        -0.18,  0.12;
         0.00,  0.12;
         0.18,  0.12;
        -0.18, -0.12;
         0.00, -0.12;
         0.18, -0.12
    ];

    % 如果任务点太难覆盖，可以将 local_y 的绝对值增大到 0.15 或 0.16，
    % 使任务点更靠近桌边，例如：
    %
    % cfg.task_candidate_local = [
    %     -0.18,  0.16;
    %      0.00,  0.16;
    %      0.18,  0.16;
    %     -0.18, -0.16;
    %      0.00, -0.16;
    %      0.18, -0.16
    % ];


    %% ===================== 8. 手动任务点 =========================
    % manual_tasks 只在 cfg.task_mode = 'manual' 时使用。
    %
    % 推荐格式：
    % [table_id, x, y, z]
    %
    % table_id : 任务属于哪张桌子；
    % x, y     : 世界坐标，单位 m；
    % z        : 任务高度，单位 m，通常等于桌子高度 0.50。
    %
    % 示例：
%     cfg.manual_tasks = [
%         1, 1.82, 1.62, 0.50;
%         1, 2.18, 1.38, 0.50;
%         2, 4.00, 1.62, 0.50
%     ];
    cfg.manual_tasks = [];


    %% ===================== 9. 任务高度 ===========================
    % 任务点 z 方向偏移
    % 当前设置为桌面高度，即任务点位于桌面上
    cfg.task_offset_z = cfg.table.height;


    %% ===================== 10. 停靠位姿候选参数 ==================
    % 安全距离，单位 m
    % 用于生成候选停靠点或碰撞膨胀等
    cfg.safety_margin_m = 0.05;

    % 机器人底盘中心距离桌边的最小停靠距离，单位 m
    % 当前要求桌子不可穿越，因此底盘不能压到桌面投影上
    cfg.docking_min_distance_m = 0.25;

    % 机器人底盘中心距离桌边的最大停靠距离，单位 m
    % 当前为 0.25 + 0.15 = 0.40 m
    cfg.docking_max_distance_m = cfg.docking_min_distance_m + 0.15;

    % 停靠距离采样步长，单位 m
    % 例如 0.05 表示从 0.25、0.30、0.35、0.40 这些距离采样
    cfg.docking_distance_step_m = 0.05;

    % 沿桌边方向的候选点采样间隔，单位 m
    % 越小候选越密，计算更慢；越大候选更少，可能漏掉可达位姿
    cfg.docking_lateral_step_m  = 0.15;

end

% function cfg = scene_config()
%     %% ============================================================
%     %  scene_config.m
%     %
%     %  Hardware-matched laboratory scene for ICRA experiment
%     %
%     %  Scene basis:
%     %  - Experimental area: 9 x 7 floor tiles
%     %  - Tile size: 0.60 m x 0.60 m
%     %  - Map size: 5.40 m x 4.20 m
%     %  - Six physical tables
%     %  - Eight manually specified tabletop task points
%     %
%     %  Coordinate convention:
%     %  - origin (0,0): lower-left corner of the experimental area
%     %  - +x: left -> right in the top-view MATLAB map
%     %  - +y: front -> back in the top-view MATLAB map
%     %
%     %  IMPORTANT:
%     %  The table/task coordinates below are a PHOTO-DERIVED INITIALIZATION
%     %  based on the 0.60 m floor-tile grid and the annotated experiment image.
%     %  Before final paper submission, replace them with tape-measured values
%     %  if available.
%     %% ============================================================
% 
%     %% ===================== 1. Map parameters =====================
%     cfg.map_width_m  = 6.20;
%     cfg.map_height_m = 6.60;
%     cfg.grid_resolution_m = 0.05;
%     cfg.border_thickness_m = 0.05;
%     cfg.tile_size_m = 0.60;
% 
%     %% ===================== 2. Table geometry =====================
%     cfg.table.length = 1.200;
%     cfg.table.width  = 0.600;
%     cfg.table.height = 0.700;
% 
%     %% ===================== 3. Table layout =======================
%     cfg.table_mode = 'custom';
% 
%     % Existing project convention: TABLE CENTER coordinates [x, y].
%     cfg.table_positions = [
%         1.20, 2.30;   % T1 left-front
%         1.20, 3.90;   % T2 left-middle
%         1.20, 5.50;   % T3 left-rear
%         5.00, 2.30;   % T4 right-front
%         5.00, 3.90;   % T5 right-middle
%         5.00, 5.50    % T6 right-rear
%     ];
% 
%     cfg.num_tables = size(cfg.table_positions, 1);
%     cfg.table_orientations = zeros(cfg.num_tables, 1);
% 
%     %% ===================== 4. Scene validation ===================
%     cfg.table_min_spacing_m = 0.80;
%     cfg.table_wall_margin_m = 0.05;
%     cfg.table_start_margin_m = 0.80;
% 
%     %% ===================== 5. Robot start pose ===================
%     % Start just outside T1's lower-left corner; keep clearance from the
%     % table footprint and the 5 cm safety inflation.
%     cfg.start_pose = [0.35, 0.35, 0.0];
% 
%     %% ===================== 6. Task generation mode ===============
%     cfg.task_mode = 'manual';
%     cfg.task.difficulty = 'normal';
%     cfg.task.hard_mixed_ratio = [0.4, 0.4, 0.2];
%     cfg.tasks_per_table_range = [1, 1];
%     cfg.random_seed = 2026;
% 
%     %% ===================== 7. Candidate local points =============
%     % Retained for compatibility; manual tasks are used in this experiment.
%     cfg.task_candidate_local = [
%         -0.30,  0.20;
%          0.00,  0.20;
%          0.30,  0.20;
%         -0.30, -0.20;
%          0.00, -0.20;
%          0.30, -0.20
%     ];
% 
%     %% ===================== 8. Manual physical tasks ==============
%     % Format: [table_id, x_world, y_world, z_world]
%     %
%     % Task numbering follows row order:
%     % T4: Task 1, Task 2
%     % T1: Task 3
%     % T2: Task 4, Task 8
%     % T5: Task 5
%     % T6: Task 6
%     % T3: Task 7
% 
%     z_task = cfg.table.height;
% 
%     cfg.manual_tasks_local = [
%         1, 4, -0.20, -0.45; 2, 4,  0.20, -0.45;
%         3, 1,  0.00, -0.45; 4, 2, -0.20, -0.40;
%         5, 5,  0.00, -0.40; 6, 6,  0.00, -0.45;
%         7, 3,  0.00, -0.45; 8, 2,  0.20, -0.40];
%     cfg.manual_tasks = zeros(size(cfg.manual_tasks_local,1),4);
%     cfg.task_ids = cfg.manual_tasks_local(:,1);
%     for k=1:size(cfg.manual_tasks_local,1)
%         task_id=cfg.manual_tasks_local(k,1); tid=cfg.manual_tasks_local(k,2);
%         q=cfg.manual_tasks_local(k,3:4); th=cfg.table_orientations(tid);
%         R=[cos(th),-sin(th);sin(th),cos(th)]; xy=cfg.table_positions(tid,:)' + R*q';
%         cfg.manual_tasks(k,:)=[tid,xy',z_task]; %#ok<AGROW>
%     end
% 
%     %% ===================== 9. Task height ========================
%     cfg.task_offset_z = cfg.table.height;
% 
%     %% ===================== 10. Docking candidate parameters ======
%     cfg.safety_margin_m = 0.05;
%     cfg.docking_min_distance_m = 0.25;
%     cfg.docking_max_distance_m = 0.40;
%     cfg.docking_distance_step_m = 0.05;
%     cfg.docking_lateral_step_m = 0.10;
% 
%     %% ===================== 11. Hardware-matched metadata =========
%     cfg.scene_name = 'lab_hardware_matched';
%     cfg.scene_source = 'annotated_physical_lab_photo';
%     cfg.scene_coordinate_status = 'photo_initialized_not_final_measured';
%     cfg.expected_task_count = 8;
%     cfg.expected_table_count = 6;
% 
%     % Geometry audit metadata (consumed by validate_scene_config and reports).
%     cfg.tile_size_m = 0.60;
%     cfg.main_aisle_width_m = (cfg.table_positions(4,1)-cfg.table.length/2) - ...
%         (cfg.table_positions(1,1)+cfg.table.length/2);
%     cfg.all_tables_parallel = all(abs(cfg.table_orientations-cfg.table_orientations(1)) < 1e-12);
%     cfg.row_alignment_valid = max(abs(cfg.table_positions(1:3,2)-cfg.table_positions(4:6,2))) < 1e-12;
%     cfg.column_alignment_valid = max(abs(cfg.table_positions([1 2 3],1)-cfg.table_positions(1,1))) < 1e-12 && ...
%         max(abs(cfg.table_positions([4 5 6],1)-cfg.table_positions(4,1))) < 1e-12;
%     cfg.static_obstacles = [0.00, 0.85, 0.55, 4.55; 3.65, 0.85, 0.55, 4.55];
%     cfg.keep_out_names = {'LEFT_KEEP_OUT','RIGHT_KEEP_OUT'};
% 
% end
