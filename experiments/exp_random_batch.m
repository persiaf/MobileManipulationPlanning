%% exp_random_batch.m
% 随机批量实验：在 small / medium / large 三种场景规模下各运行多个随机种子，
% 用于 SCI 论文统计实验。每个 trial 的任务点随机生成，种子可复现。
%
% 输出:
%   results/tables/random_batch_summary.csv       - 每个规模下各指标的 mean/std/min/max
%   results/raw/random_batch_records.mat          - 所有 trial 的完整 record 结构体
%   results/raw/random_batch_records_partial.mat  - 每完成一个 trial 自动保存的中间结果

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

num_trials  = planner_cfg.experiment.num_trials;
scales      = planner_cfg.experiment.scales;
base_seed   = planner_cfg.experiment.base_seed;

run_benchmark = false;
if isfield(planner_cfg, 'experiment') && isfield(planner_cfg.experiment, 'run_benchmark')
    run_benchmark = planner_cfg.experiment.run_benchmark;
end

% 批量实验默认关闭多算法 benchmark，避免每个 trial 重复运行大量对比算法。
if isfield(planner_cfg, 'benchmark')
    planner_cfg.benchmark.enabled = run_benchmark;
else
    planner_cfg.benchmark = struct('enabled', run_benchmark);
end

% 关闭可视化（批量实验不需要）
planner_cfg.visualization.show_scene        = false;
planner_cfg.visualization.show_candidates   = false;
planner_cfg.visualization.show_docking      = false;
planner_cfg.visualization.show_global_path  = false;
planner_cfg.visualization.show_reachability = false;
planner_cfg.visualization.show_task_links   = false;
planner_cfg.visualization.show_arm_assign   = false;
planner_cfg.visualization.show_cost_heatmap = false;

% 输出目录
out_table_dir = fullfile(project_root, 'results', 'tables');
out_raw_dir   = fullfile(project_root, 'results', 'raw');
if ~exist(out_table_dir, 'dir'), mkdir(out_table_dir); end
if ~exist(out_raw_dir,   'dir'), mkdir(out_raw_dir);   end

partial_path = fullfile(out_raw_dir, 'random_batch_records_partial.mat');

%% ===================== 逐规模逐 trial 运行 =====================
robot_cfg = robot_config();

n_scales = length(scales);
total_trials = n_scales * num_trials;

template = create_empty_result_record();
all_records = repmat(template, 0, 1);

trial_counter = 0;
fprintf('========================================\n');
fprintf('  随机批量实验: %d 种规模 x %d trials = %d 次\n', ...
        n_scales, num_trials, total_trials);
fprintf('  run_benchmark = %d\n', run_benchmark);
fprintf('========================================\n');

for si = 1:n_scales
    scale = scales{si};
    fprintf('\n===== 规模: %s =====\n', scale);

    for ti = 1:num_trials
        trial_counter = trial_counter + 1;
        trial_seed = base_seed + si * 1000 + ti;
        trial_id = trial_counter;

        fprintf('  [%3d/%3d] scale=%s trial=%d seed=%d ... ', ...
                trial_counter, total_trials, scale, ti, trial_seed);

        try
            % --- 1) 构建场景 ---
            scene_cfg = create_scaled_scene_config(scale, trial_seed);
            scene = build_scene_from_cfg(scene_cfg);
            scene_cfg = scene.scene_cfg;

            % --- 2) 场景合法性检查 ---
            scene_ok = validate_scene_config(scene_cfg, scene.grid_map, scene.map_info);
            if ~scene_ok
                warning('exp_random_batch: scene 验证未通过，继续运行');
            end

            % --- 3) 任务归属 & 区域构建 ---
            task_table_map = assign_tasks_to_tables(scene);
            regions = build_task_regions(scene, task_table_map);

            % --- 4) 停靠规划 ---
            t_dock = tic;
            [optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list] = ...
                stage2_docking_planning(regions, scene, robot_cfg, planner_cfg); %#ok<NASGU,ASGLU>
            dock_time = toc(t_dock);

            % --- 5) 全局路径规划 ---
            t_path = tic;
            [scene, global_path, all_poses, visit_order, executed_tasks, total_path_len, stats] = ...
                stage3_global_planning(scene, optimal_poses, task_sequence_summary, ...
                                       task_table_map, robot_cfg, planner_cfg, pose_info_list); %#ok<ASGLU>
            path_time = toc(t_path);

            % --- 6) 汇总 timing ---
            stats.timing.scene_time_s = 0;
            stats.timing.docking_time_s = dock_time;
            stats.timing.stage3_route_order_time_s = path_time;
            stats.timing.total_planning_time_s = dock_time + path_time;

            % --- 7) benchmark（默认关闭） ---
            benchmark_results = [];
            if run_benchmark
                try
                    benchmark_results = stage4_benchmark_all(scene, all_poses, ...
                        optimal_poses, planner_cfg, stats);
                catch ME_bench
                    warning('exp_random_batch: benchmark 失败: %s', ME_bench.message);
                end
            end

            % --- 8) 汇总 record ---
            record = build_result_record(scene, optimal_poses, executed_tasks, ...
                                         global_path, total_path_len, stats, ...
                                         planner_cfg, pose_info_list, benchmark_results);
            record.scale = scale;
            record.trial_id = trial_id;
            record.trial_idx = ti;
            record.trial_seed = trial_seed;
            record.status = 'OK';
            record.error_message = '';
            record = align_record_fields(record, template);

            all_records(end+1, 1) = record; %#ok<SAGROW>

            fprintf('OK  compl=%.1f%%  docks=%d  path=%.1fm\n', ...
                    record.completion_rate, record.num_docking_poses, ...
                    record.total_path_length);

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
            fail_record = align_record_fields(fail_record, template);

            all_records(end+1, 1) = fail_record; %#ok<SAGROW>
        end

        save(partial_path, 'all_records', 'scales', 'num_trials', 'base_seed', ...
             'run_benchmark', 'trial_counter');
        close all;
    end
end

%% ===================== 统计汇总 =====================
fprintf('\n========== 统计汇总 ==========\n');

metrics = {'completion_rate', 'num_docking_poses', 'total_path_length', ...
           'planning_time_total'};

summary_cells = cell(n_scales + 1, 1 + 4 * length(metrics));
summary_cells{1, 1} = 'scale';
col = 2;
for mi = 1:length(metrics)
    mname = metrics{mi};
    summary_cells{1, col} = [mname, '_mean']; col = col + 1;
    summary_cells{1, col} = [mname, '_std'];  col = col + 1;
    summary_cells{1, col} = [mname, '_min'];  col = col + 1;
    summary_cells{1, col} = [mname, '_max'];  col = col + 1;
end

for si = 1:n_scales
    scale = scales{si};

    idx = [];
    for ri = 1:length(all_records)
        if strcmp(all_records(ri).scale, scale) && strcmp(all_records(ri).status, 'OK')
            idx(end+1) = ri; %#ok<AGROW>
        end
    end

    if isempty(idx)
        fprintf('  规模 %s: 无成功 trial\n', scale);
        continue;
    end

    row = si + 1;
    summary_cells{row, 1} = scale;
    col = 2;
    for mi = 1:length(metrics)
        vals = arrayfun(@(k) nz(all_records(k), metrics{mi}), idx);
        summary_cells{row, col} = mean(vals); col = col + 1;
        summary_cells{row, col} = std(vals);  col = col + 1;
        summary_cells{row, col} = min(vals);  col = col + 1;
        summary_cells{row, col} = max(vals);  col = col + 1;
    end

    fprintf('\n  %s (%d/%d 成功):\n', scale, length(idx), num_trials);
    for mi = 1:length(metrics)
        vals = arrayfun(@(k) nz(all_records(k), metrics{mi}), idx);
        fprintf('    %-22s  mean=%-10.4f  std=%-10.4f  [%.4f, %.4f]\n', ...
                metrics{mi}, mean(vals), std(vals), min(vals), max(vals));
    end
end

%% ===================== 保存 =====================
mat_path = fullfile(out_raw_dir, 'random_batch_records.mat');
save(mat_path, 'all_records', 'scales', 'num_trials', 'base_seed', 'run_benchmark');
fprintf('\n完整记录已保存: %s\n', mat_path);

csv_path = fullfile(out_table_dir, 'random_batch_summary.csv');
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
