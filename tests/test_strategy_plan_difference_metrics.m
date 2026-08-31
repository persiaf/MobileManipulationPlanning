function tests = test_strategy_plan_difference_metrics
    tests = functiontests(localfunctions);
end

function testPlanDifferenceMetricsDetectChanges(testCase)
    cfg = planner_config();
    cfg.strategy.diagnostics.position_tolerance_m = 0.05;
    cfg.strategy.diagnostics.yaw_tolerance_rad = deg2rad(5);

    ref = struct();
    ref.strategy_name = 'bottom_up';
    ref.region_order = [1, 2, 3];
    ref.preferred_visit_order = [1, 2, 3, 4];
    ref.selected_poses = [0, 0, 0; 1, 0, 0; 2, 0, 0];
    ref.num_docking_poses = 3;
    ref.task_sequence_summary = {
        struct('covered_tasks', [1, 2]), ...
        struct('covered_tasks', 3), ...
        struct('covered_tasks', 4)};

    plan = ref;
    plan.strategy_name = 'top_down';
    plan.region_order = [1, 3, 2];
    plan.preferred_visit_order = [1, 3, 2, 4];
    plan.selected_poses = [0, 0, 0; 1.2, 0, 0; 2, 0, 0];
    plan.task_sequence_summary{2}.covered_tasks = [3, 4];

    ref_record = struct('total_path_length', 10, 'estimated_execution_time_s', 100);
    record = struct('total_path_length', 12.5, 'estimated_execution_time_s', 130);
    cmp = compare_strategy_plans(ref, plan, ref_record, record, cfg);

    verifyEqual(testCase, cmp.region_order_hamming, 2);
    verifyEqual(testCase, cmp.visit_order_hamming, 2);
    verifyEqual(testCase, cmp.pose_match_count, 2);
    verifyEqual(testCase, cmp.pose_difference_count, 1);
    verifyGreaterThan(testCase, cmp.task_assignment_difference_count, 0);
    verifyEqual(testCase, cmp.path_length_difference_m, 2.5, 'AbsTol', 1e-12);
    verifyEqual(testCase, cmp.execution_time_difference_s, 30, 'AbsTol', 1e-12);
end
