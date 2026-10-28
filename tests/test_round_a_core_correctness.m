function tests = test_round_a_core_correctness
    tests = functiontests(localfunctions);
end

function testFootprintInflationUsesHalfDiagonal(testCase)
    obstacle_map = zeros(21, 21);
    obstacle_map(11, 11) = 1;
    map_info = struct('resolution', 0.05);
    [~, info] = build_footprint_inflated_map(obstacle_map, [0.423, 0.366], map_info);
    verifyEqual(testCase, info.radius_m, 0.5 * hypot(0.423, 0.366), 'AbsTol', 1e-12);
    verifyGreaterThan(testCase, info.radius_m, 0.5 * max(0.423, 0.366));
end

function testUnifiedPlanPropagatesPartialCoverage(testCase)
    task_summary = {struct('covered_tasks', 1)};
    candidate_library = struct('total_candidates', 1, 'total_generation_time_s', 0);
    scene = struct('num_tasks', 2);
    plan = build_unified_plan('bottom_up', zeros(0, 3), [], task_summary, {}, ...
        [], [], candidate_library, scene, planner_config(), struct(), struct());
    verifyEqual(testCase, plan.status, 'BLOCKED');
    verifyEqual(testCase, plan.reason, 'PARTIAL_TASK_COVERAGE');
    verifyEqual(testCase, plan.uncovered_tasks_predicted, 2);
end

function testStage3RejectsBlockedPlan(testCase)
    blocked = struct('status', 'BLOCKED', 'reason', 'PARTIAL_TASK_COVERAGE');
    verifyError(testCase, @() stage3_global_planning([], [], [], [], [], [], [], [], blocked), ...
        'stage3_global_planning:BlockedPlan');
end

function testEffectiveConfigCarriesRobotIntoPlanner(testCase)
    effective = resolve_effective_config();
    verifyEqual(testCase, effective.robot, robot_config());
    verifyEqual(testCase, effective.planner.robot_cfg.chassis.length, ...
        effective.robot.chassis.length);
end
