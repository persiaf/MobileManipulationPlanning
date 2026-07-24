function [optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list] = ...
    append_region_plan(optimal_poses, pose_to_region_map, task_sequence_summary, ...
    pose_info_list, region_index, region, region_poses, pose_info, planner_cfg)
%APPEND_REGION_PLAN Convert per-region pose_info into legacy summary cells.

    if isfield(planner_cfg, 'real_platform')
        real_cfg = planner_cfg.real_platform;
    else
        real_cfg = real_platform_config();
    end
    n_poses_here = size(region_poses, 1);
    for pi = 1:n_poses_here
        optimal_poses = [optimal_poses; region_poses(pi, :)]; %#ok<AGROW>
        pose_to_region_map = [pose_to_region_map; region_index]; %#ok<AGROW>
        summary_entry = struct();
        summary_entry.region = region_index;
        summary_entry.table_id = region.table_id;
        summary_entry.pose = region_poses(pi, :);
        summary_entry.task_ids = region.task_ids;
        summary_entry.covered_count = 0;
        summary_entry.covered_tasks = [];
        summary_entry.arm_assign_map = [];
        if pi <= numel(pose_info)
            summary_entry.covered_count = pose_info(pi).covered_count;
            summary_entry.covered_tasks = pose_info(pi).covered_tasks;
            summary_entry.arm_assign_map = pose_info(pi).arm_assign_map;
            info = pose_info(pi);
        else
            info = make_empty_pose_info(region_poses(pi, :), region.table_id);
        end
        summary_entry.total_tasks = numel(region.task_ids);
        summary_entry.dock_index = pi;
        summary_entry.total_docks = n_poses_here;
        service_base = build_service_config(numel(task_sequence_summary) + 1, ...
            info, summary_entry, real_cfg);
        summary_entry.service_configs = expand_lift_candidates(service_base, real_cfg);
        task_sequence_summary{end + 1} = summary_entry; %#ok<AGROW>
        pose_info_list{end + 1} = info; %#ok<AGROW>
    end
end

function info = make_empty_pose_info(pose, table_id)
    info.pose = pose;
    info.covered_tasks = [];
    info.covered_count = 0;
    info.arm_assign_map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    info.table_id = table_id;
    info.side_id = 0;
    info.cost_breakdown = struct();
    info.remaining_after = 0;
end
