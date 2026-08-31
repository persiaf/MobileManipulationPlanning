function tests = test_route_edge_difference_identity
    tests = functiontests(localfunctions);
end

function testIdenticalPlansHaveZeroRouteDifferences(testCase)
    cfg = planner_config();
    plan = struct();
    plan.strategy_name = 'bottom_up';
    plan.num_docking_poses = 3;
    plan.selected_poses = [0, 0, 0; 1, 0, 0; 2, 0, 0];
    plan.final_visit_order = [1, 3, 2, 4];
    plan.region_order = [1, 2, 3];
    plan.task_sequence_summary = {
        struct('covered_tasks', 1), ...
        struct('covered_tasks', 2), ...
        struct('covered_tasks', 3)};
    cmp = compare_strategy_plans(plan, plan, ...
        struct('total_path_length_m', 5, 'estimated_execution_time_s', 10), ...
        struct('total_path_length_m', 5, 'estimated_execution_time_s', 10), cfg);

    verifyEqual(testCase, cmp.visit_order_hamming, 0);
    verifyEqual(testCase, cmp.directed_route_edge_difference_count, 0);
    verifyEqual(testCase, cmp.undirected_route_edge_difference_count, 0);
    verifyEqual(testCase, cmp.route_edge_difference_count, 0);
end
