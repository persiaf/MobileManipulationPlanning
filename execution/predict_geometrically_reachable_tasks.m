function task_ids = predict_geometrically_reachable_tasks(docking_pose, scene, robot_cfg, planner_cfg, allowed_task_ids)
%PREDICT_GEOMETRICALLY_REACHABLE_TASKS Configured reach prediction for a set.
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
        [~, ~, combined] = evaluate_task_reachability( ...
            docking_pose, scene.tasks(tid), robot_cfg, tau_r);
        if combined > 0
            task_ids(end + 1) = tid; %#ok<AGROW>
        end
    end
end
