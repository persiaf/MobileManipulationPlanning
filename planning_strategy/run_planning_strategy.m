function plan = run_planning_strategy(candidate_library, regions, scene, robot_cfg, planner_cfg)
%RUN_PLANNING_STRATEGY Dispatch a shared candidate library to one strategy.

    strategy_name = 'bottom_up';
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'name')
        strategy_name = planner_cfg.strategy.name;
    end

    switch lower(strategy_name)
        case 'bottom_up'
            plan = plan_bottom_up(candidate_library, regions, scene, robot_cfg, planner_cfg);
        case 'top_down'
            plan = plan_top_down(candidate_library, regions, scene, robot_cfg, planner_cfg);
        case 'bidirectional'
            plan = plan_bidirectional(candidate_library, regions, scene, robot_cfg, planner_cfg);
        case 'task_navigation_joint'
            plan = plan_task_navigation_joint(candidate_library, regions, scene, robot_cfg, planner_cfg);
        case 'task_navigation_joint_local'
            plan = plan_task_navigation_joint_local(candidate_library, regions, scene, robot_cfg, planner_cfg);
        otherwise
            error('run_planning_strategy:UnknownStrategy', ...
                'Unknown planning strategy: %s', strategy_name);
    end
end
