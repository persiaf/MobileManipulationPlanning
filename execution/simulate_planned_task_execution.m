function [executed_tasks, verified_tasks, verification, evidence] = simulate_planned_task_execution(scene, commanded_tasks, docking_pose, robot_cfg, planner_cfg)
%SIMULATE_PLANNED_TASK_EXECUTION Simulate commands using geometric acceptance.
%   Legacy verified/executed outputs describe simulation, never measured
%   completion or IK. The fourth output makes that evidence boundary explicit.

    commanded_tasks = unique(commanded_tasks(:)', 'stable');
    commanded_tasks = commanded_tasks(commanded_tasks >= 1 & commanded_tasks <= scene.num_tasks);
    executed_tasks = commanded_tasks;
    verified_tasks = [];
    verification = struct('task_id', {}, 'geometric_score', {}, 'verified', {}, ...
        'geometric_accepted', {}, 'evidence_source', {}, 'ik_checked', {}, ...
        'ik_feasible', {}, 'measured_completed', {});
    tau_r = planner_cfg.reachability.threshold;

    for k = 1:numel(executed_tasks)
        tid = executed_tasks(k);
        target = [scene.tasks(tid).x, scene.tasks(tid).y, scene.tasks(tid).z];
        [~, ~, score] = evaluate_dual_arm_reachability(docking_pose, target, robot_cfg, tau_r);
        verification(k).task_id = tid;
        verification(k).geometric_score = score;
        verification(k).verified = score > 0;
        verification(k).geometric_accepted = score > 0;
        verification(k).evidence_source = 'geometric_simulation';
        verification(k).ik_checked = false;
        verification(k).ik_feasible = NaN;
        verification(k).measured_completed = NaN;
        if verification(k).verified
            verified_tasks(end + 1) = tid; %#ok<AGROW>
        end
    end
    status = separate_execution_status(commanded_tasks,commanded_tasks, ...
        executed_tasks,verified_tasks,struct('source','geometric_simulation'));
    evidence = status.evidence;
end
