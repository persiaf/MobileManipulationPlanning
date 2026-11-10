function [executed_tasks, verified_tasks, verification, evidence] = simulate_planned_task_execution(scene, commanded_tasks, docking_pose, robot_cfg, planner_cfg)
%SIMULATE_PLANNED_TASK_EXECUTION Validate simulated commands with reach model.
%   Legacy verified/executed outputs describe simulation, never measured
%   completion or IK. The fourth output makes that evidence boundary explicit.

    commanded_tasks = unique(commanded_tasks(:)', 'stable');
    commanded_tasks = commanded_tasks(commanded_tasks >= 1 & commanded_tasks <= scene.num_tasks);
    executed_tasks = commanded_tasks;
    verified_tasks = [];
    verification = struct('task_id', {}, 'geometric_score', {}, 'verified', {}, ...
        'geometric_accepted', {}, 'evidence_source', {}, 'ik_checked', {}, ...
        'ik_feasible', {}, 'measured_completed', {}, 'reachability_score', {}, ...
        'reachability_mode', {}, 'collision_checked', {});
    tau_r = planner_cfg.reachability.threshold;
    task_pose_mode = isfield(robot_cfg, 'manipulator') && ...
        isfield(robot_cfg.manipulator, 'mode') && ...
        strcmp(robot_cfg.manipulator.mode, 'task_pose_ik');
    if task_pose_mode, source = 'ik_validation'; else, source = 'geometric_simulation'; end

    for k = 1:numel(executed_tasks)
        tid = executed_tasks(k);
        [~, ~, score, ~, details] = evaluate_task_reachability( ...
            docking_pose, scene.tasks(tid), robot_cfg, tau_r);
        verification(k).task_id = tid;
        verification(k).geometric_score = score;
        if task_pose_mode, verification(k).geometric_score = NaN; end
        verification(k).verified = score > 0;
        verification(k).geometric_accepted = ~task_pose_mode && score > 0;
        verification(k).evidence_source = source;
        verification(k).ik_checked = details.ik_checked;
        verification(k).ik_feasible = details.ik_feasible;
        verification(k).measured_completed = NaN;
        verification(k).reachability_score = score;
        verification(k).reachability_mode = details.mode;
        verification(k).collision_checked = false;
        if verification(k).verified
            verified_tasks(end + 1) = tid; %#ok<AGROW>
        end
    end
    status = separate_execution_status(commanded_tasks,commanded_tasks, ...
        executed_tasks,verified_tasks,struct('source',source));
    evidence = status.evidence;
end
