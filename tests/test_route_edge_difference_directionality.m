function tests = test_route_edge_difference_directionality
    tests = functiontests(localfunctions);
end

function testReversedThreeDockRouteDiffersOnlyByDirection(testCase)
    cfg = planner_config();
    ref = make_plan('bottom_up', [1, 2, 3]);
    plan = make_plan('reversed', [3, 2, 1]);
    cmp = compare_strategy_plans(ref, plan, struct(), struct(), cfg);
    verifyGreaterThan(testCase, cmp.directed_route_edge_difference_count, 0);
    verifyEqual(testCase, cmp.undirected_route_edge_difference_count, 0);
    verifyEqual(testCase, cmp.route_edge_difference_count, 0);
end

function plan = make_plan(name, dock_order)
    n = numel(dock_order);
    plan = struct();
    plan.strategy_name = name;
    plan.num_docking_poses = n;
    plan.selected_poses = [(1:n)', zeros(n, 1), zeros(n, 1)];
    plan.final_visit_order = dock_order + 1;
    plan.task_sequence_summary = arrayfun(@(k) struct('covered_tasks', k), ...
        1:n, 'UniformOutput', false);
end
