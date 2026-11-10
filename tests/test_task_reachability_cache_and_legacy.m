function tests = test_task_reachability_cache_and_legacy
    tests = functiontests(localfunctions);
end

function testTaskConstraintsInvalidateCandidateSignature(testCase)
    [scene, robot, planner, regions] = create_test_fixture();
    original = build_candidate_library_signature(regions, scene, robot, planner);
    scene.tasks(1).task_type = 'top_down_grasp';
    typed = build_candidate_library_signature(regions, scene, robot, planner);
    verifyNotEqual(testCase, original, typed);
    scene.tasks(1).manipulation = struct('orientation_tolerance_rad', 0.1);
    constrained = build_candidate_library_signature(regions, scene, robot, planner);
    verifyNotEqual(testCase, typed, constrained);
    scene.tasks(1).manipulation.orientation_tolerance_rad = 0.2;
    changed = build_candidate_library_signature(regions, scene, robot, planner);
    verifyNotEqual(testCase, constrained, changed);
    scene.tasks(1).continuous_points = [scene.tasks(1).x, scene.tasks(1).y, scene.tasks(1).z];
    path = build_candidate_library_signature(regions, scene, robot, planner);
    verifyNotEqual(testCase, changed, path);
end

function testExecutionStateDoesNotInvalidateCandidateSignature(testCase)
    [scene, robot, planner, regions] = create_test_fixture();
    original = build_candidate_library_signature(regions, scene, robot, planner);
    scene.tasks(1).status = 2;
    scene.tasks(1).measured_completed = true;
    changed = build_candidate_library_signature(regions, scene, robot, planner);
    verifyEqual(testCase, original, changed);
end

function testLegacyTaskWrapperMatchesPositionEvaluator(testCase)
    robot = robot_config();
    task = struct('x', robot.arm_base_mount_x + 0.15, ...
        'y', robot.dual_arm.base_spacing / 2, ...
        'z', robot.arm_base_mount_z + 0.096, 'task_type', 'position_only');
    [ls, rs, cs, arm] = evaluate_dual_arm_reachability( ...
        [0 0 0], [task.x task.y task.z], robot, 0.3);
    [tls, trs, tcs, tarm, details] = evaluate_task_reachability([0 0 0], task, robot, 0.3);
    verifyEqual(testCase, [tls trs tcs], [ls rs cs]);
    verifyEqual(testCase, tarm, arm);
    verifyFalse(testCase, details.ik_checked);
end

function testLegacyModeDoesNotSilentlyIgnoreTaskConstraints(testCase)
    robot = robot_config();
    task = struct('x', 0.3, 'y', 0.1, 'z', 0.4, 'task_type', 'top_down_grasp');
    verifyError(testCase, @() evaluate_task_reachability([0 0 0], task, robot, 0.3), ...
        'evaluate_task_reachability:TaskModeRequired');
    task.task_type = 'TOP_DOWN_GRASP';
    verifyError(testCase, @() evaluate_task_reachability([0 0 0], task, robot, 0.3), ...
        'evaluate_task_reachability:TaskModeRequired');
    task.task_type = 'position_only';
    task.manipulation = 3;
    verifyError(testCase, @() evaluate_task_reachability([0 0 0], task, robot, 0.3), ...
        'evaluate_task_reachability:TaskModeRequired');
end

function testLegacyRobustTaskArgumentRetainsScores(testCase)
    robot = robot_config();
    task = struct('x', 1.25, 'y', 1.05, 'z', 0.5);
    model = struct('position_offsets_m', [0 0; 0.01 0], 'yaw_offsets_rad', [0 0.01]);
    a = evaluate_robust_reachability([1 1 0.2], [task.x task.y task.z], robot, 0.3, model);
    b = evaluate_robust_reachability([1 1 0.2], [task.x task.y task.z], robot, 0.3, model, task);
    verifyEqual(testCase, a, b);
end

function testLegacyEnvelopeCannotRejectTaskIKModel(testCase)
    robot = robot_config();
    [reject, ~] = candidate_reachability_quick_filter([0 0 0], [10 10 10], robot);
    verifyTrue(testCase, reject);
    robot.manipulator.mode = 'task_pose_ik';
    [reject, reason] = candidate_reachability_quick_filter([0 0 0], [10 10 10], robot);
    verifyFalse(testCase, reject);
    verifyEqual(testCase, reason, 'PASS_TASK_POSE_IK_DEFERRED');
end
