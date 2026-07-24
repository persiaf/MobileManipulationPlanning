function task_ids = predict_geometrically_reachable_tasks(docking_pose, scene, robot_cfg, planner_cfg, allowed_task_ids)
%PREDICT_GEOMETRICALLY_REACHABLE_TASKS Geometric prediction for an explicit set.
%   This helper must not be used to discover execution commands globally.

    if nargin < 5 || isempty(allowed_task_ids)
        allowed_task_ids = 1:scene.num_tasks;
    end
    tau_r = planner_cfg.reachability.threshold;
    task_ids = [];
    for tid = allowed_task_ids(:)'
        if tid < 1 || tid > scene.num_tasks
            continue;
        end
        target = [scene.tasks(tid).x, scene.tasks(tid).y, scene.tasks(tid).z];
        [~, ~, combined] = evaluate_dual_arm_reachability(docking_pose, target, robot_cfg, tau_r);
        if combined > 0
            task_ids(end + 1) = tid; %#ok<AGROW>
        end
    end
end
