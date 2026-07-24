%% exp_hard_ablation.m
% Hard Ablation 实验：在 hard_mixed 任务分布下对比不同停靠规划消融模式。
%
% 输出:
%   results/raw/hard_ablation_records.mat
%   results/raw/hard_ablation_records_partial.mat
%   results/tables/hard_ablation_summary.csv

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
addpath(genpath(fullfile(project_root, 'global_planner')));
addpath(genpath(fullfile(project_root, 'visualization')));
init_paths(project_root);

%% ===================== 实验配置 =====================
planner_cfg = planner_config();
robot_cfg = robot_config();

scale = 'small';
difficulty = 'hard_mixed';
% 调试时可临时改为 3；正式论文实验建议保持 20。
num_trials = 20;
base_seed = 2026;

ablation_modes = {'full', 'no_reachability', 'single_docking_per_table', ...
                  'nearest_docking', 'no_coverage_cost', ...
                  'no_coverage_priority', 'no_coverage_all'};

% 批量实验默认关闭 benchmark 和可视化，仅记录主方法结果。
planner_cfg.experiment.run_benchmark = false;
planner_cfg.visualization.show_scene        = false;
planner_cfg.visualization.show_candidates   = false;
planner_cfg.visualization.show_docking      = false;
planner_cfg.visualization.show_global_path  = false;
planner_cfg.visualization.show_reachability = false;
planner_cfg.visualization.show_task_links   = false;
planner_cfg.visualization.show_arm_assign   = false;
planner_cfg.visualization.show_cost_heatmap = false;

out_table_dir = fullfile(project_root, 'results', 'tables');
out_raw_dir   = fullfile(project_root, 'results', 'raw');
if ~exist(out_table_dir, 'dir'), mkdir(out_table_dir); end
if ~exist(out_raw_dir,   'dir'), mkdir(out_raw_dir);   end
partial_path = fullfile(out_raw_dir, 'hard_ablation_records_partial.mat');

%% ===================== 逐模式逐 trial 运行 =====================
template = create_empty_result_record();
all_records = repmat(template, 0, 1);
trial_counter = 0;
total_trials = length(ablation_modes) * num_trials;

fprintf('========================================\n');
fprintf('  Hard Ablation: %d modes x %d trials = %d runs\n', ...
        length(ablation_modes), num_trials, total_trials);
fprintf('  difficulty = %s, scale = %s\n', difficulty, scale);
fprintf('========================================\n');

for mi = 1:length(ablation_modes)
    mode = ablation_modes{mi};
    fprintf('\n===== Ablation Mode: %s =====\n', mode);

    for ti = 1:num_trials
        trial_counter = trial_counter + 1;
        trial_seed = base_seed + mi * 1000 + ti;

        fprintf('  [%3d/%3d] mode=%s trial=%d seed=%d ... ', ...
                trial_counter, total_trials, mode, ti, trial_seed);

        try
            scene_cfg = create_scaled_scene_config(scale, trial_seed);
            scene_cfg.task.difficulty = difficulty;
            scene = build_scene_from_cfg(scene_cfg);
            scene_cfg = scene.scene_cfg;

            scene_ok = validate_scene_config(scene_cfg, scene.grid_map, scene.map_info);
            if ~scene_ok
                warning('exp_hard_ablation: scene 验证未通过，继续运行');
            end

            task_table_map = assign_tasks_to_tables(scene);
            regions = build_task_regions(scene, task_table_map);

            trial_planner_cfg = planner_cfg;
            trial_planner_cfg.docking.ablation_mode = mode;

            t_dock = tic;
            [optimal_poses, ~, task_sequence_summary, pose_info_list] = ...
                stage2_docking_planning(regions, scene, robot_cfg, trial_planner_cfg);
            dock_time = toc(t_dock);

            t_path = tic;
            [scene, global_path, all_poses, ~, executed_tasks, total_path_len, stats] = ...
                stage3_global_planning(scene, optimal_poses, task_sequence_summary, ...
                                       task_table_map, robot_cfg, trial_planner_cfg, pose_info_list); %#ok<ASGLU>
            path_time = toc(t_path);

            stats.timing.scene_time_s = 0;
            stats.timing.docking_time_s = dock_time;
            stats.timing.global_path_time_s = path_time;
            stats.timing.total_planning_time_s = dock_time + path_time;

            record = build_result_record(scene, optimal_poses, executed_tasks, ...
                                         global_path, total_path_len, stats, ...
                                         trial_planner_cfg, pose_info_list, []);
            record.scale = scale;
            record.trial_id = trial_counter;
            record.trial_idx = ti;
            record.trial_seed = trial_seed;
            record.status = 'OK';
            record.error_message = '';
            record.difficulty = difficulty;
            record.method = mode;
            record.ablation_mode = mode;
            record = align_record_fields(record, template);

            all_records(end+1, 1) = record; %#ok<SAGROW>

            fprintf('OK  compl=%.1f%%  uncov=%d  docks=%d  path=%.1fm\n', ...
                    record.completion_rate, record.uncovered_tasks, ...
                    record.num_docking_poses, record.total_path_length);

        catch ME
            fprintf('FAILED: %s\n', ME.message);

            fail_record = template;
            fail_record.scale = scale;
            fail_record.trial_id = trial_counter;
            fail_record.trial_idx = ti;
            fail_record.trial_seed = trial_seed;
            fail_record.seed = trial_seed;
            fail_record.status = 'FAILED';
            fail_record.success = false;
            fail_record.error_message = ME.message;
            fail_record.difficulty = difficulty;
            fail_record.method = mode;
            fail_record.ablation_mode = mode;
            fail_record = align_record_fields(fail_record, template);

            all_records(end+1, 1) = fail_record; %#ok<SAGROW>
        end

        save(partial_path, 'all_records', 'ablation_modes', 'num_trials', ...
             'base_seed', 'scale', 'difficulty', 'trial_counter');
        close all;
    end
end

%% ===================== 统计汇总 =====================
metrics = {'completion_rate', 'uncovered_tasks', 'num_docking_poses', ...
           'total_path_length', 'planning_time_total', 'docking_cost_total'};

summary_cells = cell(length(ablation_modes) + 1, 1 + 4 * length(metrics));
summary_cells{1, 1} = 'method';
col = 2;
for mi = 1:length(metrics)
    mname = metrics{mi};
    summary_cells{1, col} = [mname, '_mean']; col = col + 1;
    summary_cells{1, col} = [mname, '_std'];  col = col + 1;
    summary_cells{1, col} = [mname, '_min'];  col = col + 1;
    summary_cells{1, col} = [mname, '_max'];  col = col + 1;
end

fprintf('\n========== Hard Ablation Summary ==========\n');
for mi = 1:length(ablation_modes)
    mode = ablation_modes{mi};
    idx = [];
    for ri = 1:length(all_records)
        if strcmp(all_records(ri).ablation_mode, mode) && strcmp(all_records(ri).status, 'OK')
            idx(end+1) = ri; %#ok<AGROW>
        end
    end

    row = mi + 1;
    summary_cells{row, 1} = mode;
    if isempty(idx)
        fprintf('  %s: no successful trial\n', mode);
        continue;
    end

    col = 2;
    fprintf('\n  %s (%d/%d successful):\n', mode, length(idx), num_trials);
    for mj = 1:length(metrics)
        vals = arrayfun(@(k) nz(all_records(k), metrics{mj}), idx);
        summary_cells{row, col} = mean(vals); col = col + 1;
        summary_cells{row, col} = std(vals);  col = col + 1;
        summary_cells{row, col} = min(vals);  col = col + 1;
        summary_cells{row, col} = max(vals);  col = col + 1;
        fprintf('    %-24s mean=%-10.4f std=%-10.4f [%.4f, %.4f]\n', ...
                metrics{mj}, mean(vals), std(vals), min(vals), max(vals));
    end
end

%% ===================== 保存 =====================
mat_path = fullfile(out_raw_dir, 'hard_ablation_records.mat');
save(mat_path, 'all_records', 'ablation_modes', 'num_trials', ...
     'base_seed', 'scale', 'difficulty');
fprintf('\n完整记录已保存: %s\n', mat_path);

csv_path = fullfile(out_table_dir, 'hard_ablation_summary.csv');
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
fprintf('中间结果已保存: %s\n', partial_path);

fprintf('\n实验完成。\n');

%% ==================== 辅助函数 =========================
function v = nz(s, field)
    if isfield(s, field)
        v = s.(field);
    else
        v = 0;
    end
end
