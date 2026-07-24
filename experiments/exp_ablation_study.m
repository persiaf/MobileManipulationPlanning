%% exp_ablation_study.m
% 消融实验：对比 full / no_reachability / single_docking_per_table /
%           nearest_docking / no_coverage_cost / no_coverage_priority /
%           no_coverage_all 七种停靠规划模式。
%
% 输出:
%   results/tables/ablation_summary.csv   - 汇总表格
%   results/raw/ablation_records.mat      - 完整 record 结构体数组

clear;
close all;
clc;

%% ===================== 初始化路径 =====================
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'pipeline'));
addpath(fullfile(project_root, 'utils'));
init_paths(project_root);

%% ===================== 实验配置 =====================
% 七种消融模式
ablation_modes = {'full', 'no_reachability', 'single_docking_per_table', ...
                  'nearest_docking', 'no_coverage_cost', ...
                  'no_coverage_priority', 'no_coverage_all'};

% 是否显示图形（批量实验建议 false）
show_figures = false;

% 输出目录
out_table_dir = fullfile(project_root, 'results', 'tables');
out_raw_dir   = fullfile(project_root, 'results', 'raw');
if ~exist(out_table_dir, 'dir'), mkdir(out_table_dir); end
if ~exist(out_raw_dir,   'dir'), mkdir(out_raw_dir);   end

%% ===================== 逐模式运行 =====================
n_modes = length(ablation_modes);
all_records = [];

fprintf('========================================\n');
fprintf('  消融实验: %d 种模式\n', n_modes);
fprintf('========================================\n');

for mi = 1:n_modes
    mode = ablation_modes{mi};
    fprintf('\n##################################################\n');
    fprintf('#  模式 %d/%d: %s\n', mi, n_modes, mode);
    fprintf('##################################################\n');

    % ---- 阶段 1: 构建场景 ----
    [robot_cfg, scene_cfg, planner_cfg, scene, task_table_map, regions] = stage1_build_scene();

    % ---- 注入消融模式 & 关闭批量可视化 ----
    planner_cfg.docking.ablation_mode = mode;
    if ~show_figures
        planner_cfg.visualization.show_scene        = false;
        planner_cfg.visualization.show_docking      = false;
        planner_cfg.visualization.show_global_path  = false;
    end

    % ---- 阶段 2: 停靠规划 ----
    [optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list] = ...
        stage2_docking_planning(regions, scene, robot_cfg, planner_cfg); %#ok<ASGLU>

    % ---- 阶段 3: 全局路径规划 ----
    [scene, global_path, all_poses, visit_order, executed_tasks, total_path_len, stats] = ...
        stage3_global_planning(scene, optimal_poses, task_sequence_summary, ...
                               task_table_map, robot_cfg, planner_cfg, pose_info_list); %#ok<ASGLU>

    % ---- 阶段 4: benchmark (可选，若耗时过长可注释) ----
    try
        benchmark_results = stage4_benchmark_all(scene, all_poses, optimal_poses, planner_cfg, stats);
    catch ME
        warning('exp_ablation_study: benchmark 失败 (%s)，继续收集主流程结果。', ME.message);
        benchmark_results = [];
    end

    % ---- 汇总 record ----
    record = build_result_record(scene, optimal_poses, executed_tasks, ...
                                 global_path, total_path_len, stats, ...
                                 planner_cfg, pose_info_list, benchmark_results);
    % 用 mode 覆盖 ablation_mode 字段（确保记录准确）
    record.method = mode;
    record.ablation_mode = mode;

    all_records = [all_records, record]; %#ok<AGROW>

    % ---- 关闭所有图形 ----
    if ~show_figures
        close all;
    end
end

%% ===================== 保存结果 =====================

% --- 1. 保存完整 MAT ---
mat_path = fullfile(out_raw_dir, 'ablation_records.mat');
save(mat_path, 'all_records', 'ablation_modes');
fprintf('\n完整记录已保存: %s\n', mat_path);

% --- 2. 生成汇总 CSV ---
csv_path = fullfile(out_table_dir, 'ablation_summary.csv');

% 构建表头和数据行
header = {'method', 'completion_rate', 'num_docking_poses', 'uncovered_tasks', ...
          'task_path_length', 'total_path_length', 'planning_time_total', 'docking_cost_total'};

fid = fopen(csv_path, 'w', 'n', 'UTF-8');
fprintf(fid, '%s,%s,%s,%s,%s,%s,%s,%s\n', header{:});

for ri = 1:length(all_records)
    r = all_records(ri);
    fprintf(fid, '%s,%.2f,%d,%d,%.2f,%.2f,%.3f,%.4f\n', ...
            safe_str(r, 'method'), ...
            safe_num(r, 'completion_rate'), ...
            safe_num(r, 'num_docking_poses'), ...
            safe_num(r, 'uncovered_tasks'), ...
            safe_num(r, 'task_path_length'), ...
            safe_num(r, 'total_path_length'), ...
            safe_num(r, 'planning_time_total'), ...
            safe_num(r, 'docking_cost_total'));
end

fclose(fid);
fprintf('汇总表格已保存: %s\n', csv_path);

%% ===================== 终端汇总表 =====================
fprintf('\n========== 消融实验汇总 ==========\n');
fprintf('%-24s %8s %8s %8s %10s %10s %10s %10s\n', ...
        'method', 'compl%', '#docks', 'uncov', 'task(m)', 'total(m)', 'plan(s)', 'cost');
fprintf('%s\n', repmat('-', 1, 100));
for ri = 1:length(all_records)
    r = all_records(ri);
    fprintf('%-24s %8.1f %8d %8d %10.2f %10.2f %10.3f %10.4f\n', ...
            safe_str(r, 'method'), ...
            safe_num(r, 'completion_rate'), ...
            safe_num(r, 'num_docking_poses'), ...
            safe_num(r, 'uncovered_tasks'), ...
            safe_num(r, 'task_path_length'), ...
            safe_num(r, 'total_path_length'), ...
            safe_num(r, 'planning_time_total'), ...
            safe_num(r, 'docking_cost_total'));
end

fprintf('\n实验完成。\n');

%% ======================== 辅助函数 =========================
function v = safe_num(s, field)
    if isfield(s, field)
        v = s.(field);
    else
        v = 0;
    end
end

function v = safe_str(s, field)
    if isfield(s, field)
        val = s.(field);
        if ischar(val)
            v = val;
        else
            v = num2str(val);
        end
    else
        v = '';
    end
end