%% exp_hard_scenario.m
% Hard Scenario 对比实验：在 small 场景下运行 5 种任务分布难度，
% 每种子难度使用 10 个随机种子，用于 SCI 论文 hard scenario 消融实验。
%
% 测试的子难度:
%   normal              — 基准 normal random 任务分布
%   hard_corner         — 任务靠近桌面四角
%   hard_two_side       — 任务分布在桌子相对两侧
%   hard_reach_boundary — 任务靠近可达边界（桌边全周）
%   hard_mixed          — 混合以上三种
%
% 输出:
%   results/raw/hard_scenario_records.mat   - 所有 trial 的完整 record
%   results/tables/hard_scenario_summary.csv - 统计汇总表

clear;
close all;
clc;

%% ===================== 初始化 =====================
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(project_root, 'pipeline')));
addpath(genpath(fullfile(project_root, 'utils')));
addpath(genpath(fullfile(project_root, 'config')));
addpath(genpath(fullfile(project_root, 'environment')));
addpath(genpath(fullfile(project_root, 'docking')));
addpath(genpath(fullfile(project_root, 'path_planning')));
addpath(genpath(fullfile(project_root, 'visualization')));
init_paths(project_root);

%% ===================== 实验配置 =====================
planner_cfg = planner_config();

scale = 'small';
difficulty_modes = {'normal', 'hard_corner', 'hard_two_side', ...
                    'hard_reach_boundary', 'hard_mixed'};
num_trials = 10;
base_seed = 2026;

% 关闭可视化
planner_cfg.visualization.show_scene        = false;
planner_cfg.visualization.show_candidates   = false;
planner_cfg.visualization.show_docking      = false;
planner_cfg.visualization.show_global_path  = false;
planner_cfg.visualization.show_reachability = false;
planner_cfg.visualization.show_task_links   = false;
planner_cfg.visualization.show_arm_assign   = false;
planner_cfg.visualization.show_cost_heatmap = false;

% 关闭 benchmark 提升效率
if isfield(planner_cfg, 'experiment')
    planner_cfg.experiment.run_benchmark = false;
end

% 输出目录
out_table_dir = fullfile(project_root, 'results', 'tables');
out_raw_dir   = fullfile(project_root, 'results', 'raw');
if ~exist(out_table_dir, 'dir'), mkdir(out_table_dir); end
if ~exist(out_raw_dir,   'dir'), mkdir(out_raw_dir);   end

%% ===================== 逐模式逐 trial 运行 =====================
robot_cfg = robot_config();

n_modes = length(difficulty_modes);
total_trials = n_modes * num_trials;

template = create_empty_result_record();
all_records = repmat(template, 0, 1);

trial_counter = 0;
fprintf('========================================\n');
fprintf('  Hard Scenario 实验: %d 种难度 x %d trials = %d 次\n', ...
        n_modes, num_trials, total_trials);
fprintf('========================================\n');

for mi = 1:n_modes
    diff_mode = difficulty_modes{mi};
    fprintf('\n===== 难度: %s =====\n', diff_mode);

    for ti = 1:num_trials
        trial_counter = trial_counter + 1;
        trial_seed = base_seed + mi * 100 + ti;
        trial_id = trial_counter;

        fprintf('  [%3d/%3d] %s trial=%d seed=%d ... ', ...
                trial_counter, total_trials, diff_mode, ti, trial_seed);

        try
            % 构建场景 cfg 并覆写 difficulty
            scene_cfg = create_scaled_scene_config(scale, trial_seed);
            scene_cfg.task.difficulty = diff_mode;

            scene = build_scene_from_cfg(scene_cfg);

            % 场景合法性检查
            scene_ok = validate_scene_config(scene_cfg, scene.grid_map, scene.map_info);
            if ~scene_ok
                warning('exp_hard_scenario: scene 验证未通过，继续运行');
            end

            % 任务归属 & 区域构建
            task_table_map = assign_tasks_to_tables(scene);
            regions = build_task_regions(scene, task_table_map);

            % 停靠规划
            t_dock = tic;
            [optimal_poses, ~, task_sequence_summary, pose_info_list] = ...
                stage2_docking_planning(regions, scene, robot_cfg, planner_cfg); %#ok<NASGU>
            dock_time = toc(t_dock);

            % 全局路径规划
            t_path = tic;
            [scene, global_path, all_poses, ~, executed_tasks, total_path_len, stats] = ...
                stage3_global_planning(scene, optimal_poses, task_sequence_summary, ...
                                       task_table_map, robot_cfg, planner_cfg, pose_info_list); %#ok<ASGLU>
            path_time = toc(t_path);

            % 汇总 timing
            stats.timing.scene_time_s = 0;
            stats.timing.docking_time_s = dock_time;
            stats.timing.stage3_route_order_time_s = path_time;
            stats.timing.total_planning_time_s = dock_time + path_time;

            % 汇总 record
            record = build_result_record(scene, optimal_poses, executed_tasks, ...
                                         global_path, total_path_len, stats, ...
                                         planner_cfg, pose_info_list, []);
            record.scale = scale;
            record.trial_id = trial_id;
            record.trial_idx = ti;
            record.trial_seed = trial_seed;
            record.status = 'OK';
            record.error_message = '';
            record.difficulty = diff_mode;
            record = align_record_fields(record, template);

            all_records(end+1, 1) = record; %#ok<SAGROW>

            fprintf('OK  compl=%.1f%%  docks=%d  path=%.1fm  diff=%s\n', ...
                    record.completion_rate, record.num_docking_poses, ...
                    record.total_path_length, diff_mode);

        catch ME
            fprintf('FAILED: %s\n', ME.message);

            fail_record = template;
            fail_record.scale = scale;
            fail_record.trial_id = trial_id;
            fail_record.trial_idx = ti;
            fail_record.trial_seed = trial_seed;
            fail_record.seed = trial_seed;
            fail_record.status = 'FAILED';
            fail_record.success = false;
            fail_record.error_message = ME.message;
            fail_record.difficulty = diff_mode;
            fail_record = align_record_fields(fail_record, template);

            all_records(end+1, 1) = fail_record; %#ok<SAGROW>
        end

        close all;
    end
end

%% ===================== 统计汇总 =====================
fprintf('\n========== 统计汇总 ==========\n');

metrics = {'completion_rate', 'num_docking_poses', 'total_path_length', ...
           'planning_time_total'};

summary_cells = cell(n_modes + 1, 1 + 4 * length(metrics));
summary_cells{1, 1} = 'difficulty';
col = 2;
for mi = 1:length(metrics)
    mname = metrics{mi};
    summary_cells{1, col} = [mname, '_mean']; col = col + 1;
    summary_cells{1, col} = [mname, '_std'];  col = col + 1;
    summary_cells{1, col} = [mname, '_min'];  col = col + 1;
    summary_cells{1, col} = [mname, '_max'];  col = col + 1;
end

for mi = 1:n_modes
    diff_mode = difficulty_modes{mi};

    idx = [];
    for ri = 1:length(all_records)
        if isfield(all_records(ri), 'difficulty') && ...
           strcmp(all_records(ri).difficulty, diff_mode) && ...
           strcmp(all_records(ri).status, 'OK')
            idx(end+1) = ri; %#ok<AGROW>
        end
    end

    if isempty(idx)
        fprintf('  难度 %s: 无成功 trial\n', diff_mode);
        continue;
    end

    row = mi + 1;
    summary_cells{row, 1} = diff_mode;
    col = 2;
    for mj = 1:length(metrics)
        vals = arrayfun(@(k) nz(all_records(k), metrics{mj}), idx);
        summary_cells{row, col} = mean(vals); col = col + 1;
        summary_cells{row, col} = std(vals);  col = col + 1;
        summary_cells{row, col} = min(vals);  col = col + 1;
        summary_cells{row, col} = max(vals);  col = col + 1;
    end

    fprintf('\n  %s (%d/%d 成功):\n', diff_mode, length(idx), num_trials);
    for mj = 1:length(metrics)
        vals = arrayfun(@(k) nz(all_records(k), metrics{mj}), idx);
        fprintf('    %-22s  mean=%-10.4f  std=%-10.4f  [%.4f, %.4f]\n', ...
                metrics{mj}, mean(vals), std(vals), min(vals), max(vals));
    end
end

%% ===================== 保存 =====================
mat_path = fullfile(out_raw_dir, 'hard_scenario_records.mat');
save(mat_path, 'all_records', 'difficulty_modes', 'num_trials', ...
     'base_seed', 'scale');
fprintf('\n完整记录已保存: %s\n', mat_path);

csv_path = fullfile(out_table_dir, 'hard_scenario_summary.csv');
fid = fopen(csv_path, 'w', 'n', 'UTF-8');
for r = 1:size(summary_cells, 1)
    for c = 1:size(summary_cells, 2)
        if c > 1, fprintf(fid, ','); end
        val = summary_cells{r, c};
        if isnumeric(val)
            fprintf(fid, '%.6f', val);
        elseif ischar(val) || isstring(val)
            fprintf(fid, '%s', char(val));
        end
    end
    fprintf(fid, '\n');
end
fclose(fid);
fprintf('汇总表格已保存: %s\n', csv_path);

fprintf('\n实验完成。\n');

%% ==================== 辅助函数 =========================
function v = nz(s, field)
    if isfield(s, field)
        v = s.(field);
    else
        v = 0;
    end
end