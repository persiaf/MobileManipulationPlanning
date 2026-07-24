function plan = plan_bottom_up(candidate_library, regions, scene, robot_cfg, planner_cfg)
%PLAN_BOTTOM_UP Freeze the original region-first docking selection behavior.

    validate_candidate_library(candidate_library, regions);
    t_strategy = tic;
    t_local = tic;
    [selected_poses, pose_to_region_map, task_sequence_summary, pose_info_list] = ...
        build_bottom_up_outputs(candidate_library, regions, scene, robot_cfg, planner_cfg);
    timing.local_selection_time_s = toc(t_local);
    timing.candidate_generation_time_s = candidate_library.total_generation_time_s;
    timing.strategy_total_time_s = toc(t_strategy);
    diagnostics.coverage_repair_count = 0;
    diagnostics.uncovered_task_ids = [];
    plan = build_unified_plan('bottom_up', selected_poses, pose_to_region_map, ...
        task_sequence_summary, pose_info_list, 1:numel(regions), [], ...
        candidate_library, scene, planner_cfg, timing, diagnostics);
    validate_plan_consistency(plan, scene);
end

function [optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list] = ...
    build_bottom_up_outputs(candidate_library, regions, scene, robot_cfg, planner_cfg)
    optimal_poses = [];
    pose_to_region_map = [];
    task_sequence_summary = {};
    pose_info_list = {};
    for ri = 1:numel(regions)
        candidates = candidate_library.regions(ri).candidates;
        if isempty(candidates)
            continue;
        end
        [region_poses, pose_info] = select_multi_poses(candidates, regions(ri), scene, robot_cfg, planner_cfg);
        [optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list] = ...
            append_region_plan(optimal_poses, pose_to_region_map, task_sequence_summary, ...
            pose_info_list, ri, regions(ri), region_poses, pose_info, planner_cfg);
    end
    if isempty(optimal_poses)
        error('plan_bottom_up:NoFeasibleDockingPose', 'No feasible docking poses were selected.');
    end
end
