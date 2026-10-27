clear;
close all;
clc;
% optimal_poses
% Robust project-root based path initialization. This avoids failures when
% MATLAB is launched from a directory other than the project root.
project_root = fileparts(mfilename('fullpath'));
addpath(fullfile(project_root, 'pipeline'));
init_paths(project_root);

timing = struct();

t1 = tic;
effective = resolve_effective_config();
[robot_cfg, scene_cfg, planner_cfg, scene, task_table_map, regions] = stage1_build_scene(effective);
timing.scene_time_s = toc(t1);

t2 = tic;
candidate_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
[optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list, plan] = ...
    stage2_docking_planning(regions, scene, robot_cfg, planner_cfg, candidate_library); %#ok<NASGU>
if isstruct(plan) && isfield(plan, 'status') && strcmpi(plan.status, 'BLOCKED')
    fprintf('STAGE2_STATUS = BLOCKED\nSTAGE2_REASON = %s\n', plan.reason);
    return;
end
timing.docking_time_s = toc(t2);
timing.candidate_generation_time_s = candidate_library.total_generation_time_s;
if isfield(plan, 'timing')
    timing.strategy_total_time_s = plan.timing.strategy_total_time_s;
    timing.region_order_time_s = plan.timing.region_order_time_s;
    timing.local_selection_time_s = plan.timing.local_selection_time_s;
    timing.route_refinement_time_s = plan.timing.route_refinement_time_s;
end

t3 = tic;
[scene, global_path, all_poses, visit_order, executed_tasks, total_path_len, stats] = ...
    stage3_global_planning(scene, optimal_poses, task_sequence_summary, task_table_map, ...
    robot_cfg, planner_cfg, pose_info_list, timing, plan); %#ok<NASGU>
timing.stage3_time_s = toc(t3);

if isfield(stats, 'timing')
    timing.distance_matrix_time_s = get_timing_field(stats.timing, 'distance_matrix_time_s', 0);
    timing.route_order_time_s = get_timing_field(stats.timing, 'route_order_time_s', 0);
    timing.global_path_time_s = get_timing_field(stats.timing, 'global_path_time_s', 0); 
else
    timing.distance_matrix_time_s = 0;
    timing.route_order_time_s = 0;
    timing.global_path_time_s = timing.stage3_time_s;
end
timing.total_planning_time_s = timing.scene_time_s + timing.docking_time_s + ...
                               timing.distance_matrix_time_s + ...
                               timing.route_order_time_s + timing.global_path_time_s;

if isfield(stats, 'exec_stats') && isfield(stats, 'segment_stats')
    stats.timing = timing;
    execution_status = [];
    completed_for_stats = executed_tasks;
    if isfield(stats, 'execution_status')
        execution_status = stats.execution_status;
        completed_for_stats = execution_status.verified_tasks;
    end
    stats.experiment_stats = build_experiment_stats(scene, completed_for_stats, optimal_poses, global_path, ...
        stats.exec_stats, timing, stats.segment_stats, task_sequence_summary, execution_status);
    if planner_cfg.timing.enable && planner_cfg.timing.verbose
        print_experiment_stats(stats.experiment_stats);
    end
end

benchmark_results = stage4_benchmark_all(scene, all_poses, optimal_poses, planner_cfg, stats);

% stage5_generate_video(scene, global_path, all_poses, visit_order, optimal_poses, task_sequence_summary, executed_tasks, robot_cfg, planner_cfg, stats);

% record = build_result_record(scene, optimal_poses, executed_tasks, ...
%                              global_path, total_path_len, stats, ...
%                              planner_cfg, pose_info_list, benchmark_results);
% save('experiment_record.mat', 'record');

function v = get_timing_field(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_value;
    end
end
