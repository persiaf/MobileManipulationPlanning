function [executed_tasks, verified_tasks, verification] = simulate_planned_task_execution(scene, commanded_tasks, docking_pose, robot_cfg, planner_cfg)
%SIMULATE_PLANNED_TASK_EXECUTION Execute and verify only commanded tasks.

    commanded_tasks = unique(commanded_tasks(:)', 'stable');
    commanded_tasks = commanded_tasks(commanded_tasks >= 1 & commanded_tasks <= scene.num_tasks);
    executed_tasks = commanded_tasks;
    verified_tasks = [];
    verification = struct('task_id', {}, 'geometric_score', {}, 'verified', {});
    tau_r = planner_cfg.reachability.threshold;

    for k = 1:numel(executed_tasks)
        tid = executed_tasks(k);
        target = [scene.tasks(tid).x, scene.tasks(tid).y, scene.tasks(tid).z];
        [~, ~, score] = evaluate_dual_arm_reachability(docking_pose, target, robot_cfg, tau_r);
        verification(k).task_id = tid;
        verification(k).geometric_score = score;
        verification(k).verified = score > 0;
        if verification(k).verified
            verified_tasks(end + 1) = tid; %#ok<AGROW>
        end
    end
end
