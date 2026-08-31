function tests = test_execution_time_breakdown
    tests = functiontests(localfunctions);
end

function testExecutionTimeComponentsSumToTotal(testCase)
    planner_cfg = planner_config();
    planner_cfg.execution.return_to_start = false;
    planner_cfg.execution_time.navigation_speed_mps = 1.0;
    planner_cfg.execution_time.docking_time_per_pose_s = 2.0;
    planner_cfg.execution_time.manipulation_time_per_task_s = 3.0;
    planner_cfg.execution_time.arm_transition_time_s = 4.0;
    planner_cfg.execution_time.side_switch_time_s = 5.0;
    planner_cfg.execution_time.turning_speed_radps = 1.0;

    global_path = [0, 0; 1, 0; 1, 1];
    route_order = [1, 2, 3];
    all_poses = [0, 0, 0; 1, 0, 0; 1, 1, pi / 2]; %#ok<NASGU>
    task_sequence_summary = {
        struct('covered_tasks', [1, 2], 'side_id', 1), ...
        struct('covered_tasks', 3, 'side_id', 2)};
    segment_stats = struct('path_length_m', {}, 'is_return_segment', {});

    stats = estimate_execution_time(global_path, route_order, all_poses, ...
        task_sequence_summary, segment_stats, planner_cfg);
    b = stats.time_breakdown;
    component_sum = b.navigation_time_s + b.docking_time_s + ...
        b.manipulation_time_s + b.arm_transition_time_s + ...
        b.side_switch_time_s + b.turning_time_s + ...
        b.lift_time_s + b.replanning_time_s;

    verifyEqual(testCase, b.total_estimated_time_s, component_sum, 'AbsTol', 1e-10);
    verifyEqual(testCase, stats.num_docks, 2);
    verifyEqual(testCase, stats.num_side_switches, 1);
end
