function tests = test_bidirectional_identity_diagnostics
    tests = functiontests(localfunctions);
end

function testAcceptedZeroIdenticalPlanHasZeroRouteDiff(testCase)
    cfg = planner_config();
    bottom = make_identity_plan('bottom_up');
    bi = make_identity_plan('bidirectional');
    bi.diagnostics.accepted_iterations = 0;
    cmp = compare_strategy_plans(bottom, bi, ...
        struct('total_path_length_m', 4, 'estimated_execution_time_s', 20), ...
        struct('total_path_length_m', 4, 'estimated_execution_time_s', 20), cfg);
    verifyEqual(testCase, cmp.pose_difference_count, 0);
    verifyEqual(testCase, cmp.task_assignment_difference_count, 0);
    verifyEqual(testCase, cmp.visit_order_hamming, 0);
    verifyEqual(testCase, cmp.route_edge_difference_count, 0);
    verifyEqual(testCase, cmp.path_length_difference_m, 0);
end

function plan = make_identity_plan(name)
    plan = struct();
    plan.strategy_name = name;
    plan.num_docking_poses = 2;
    plan.selected_poses = [1, 0, 0; 2, 0, 0];
    plan.region_order = [1, 2];
    plan.final_visit_order = [1, 3, 2];
    plan.task_sequence_summary = {
        struct('covered_tasks', 1), ...
        struct('covered_tasks', 2)};
    plan.diagnostics = struct('accepted_iterations', 0);
end
