function tests = test_planned_execution_consistency
    tests = functiontests(localfunctions);
end

function testExecutionUsesOnlyCommands(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    candidates = generate_docking_candidates(regions(1), scene, robot_cfg, planner_cfg);
    [~, info] = select_multi_poses(candidates, regions(1), scene, robot_cfg, planner_cfg);
    commanded = info(1).covered_tasks;
    [executed, verified] = simulate_planned_task_execution(scene, commanded, ...
        info(1).pose, robot_cfg, planner_cfg);
    verifyEqual(testCase, executed, commanded);
    verifyTrue(testCase, all(ismember(verified, commanded)));
    verifyFalse(testCase, any(ismember(setdiff(1:scene.num_tasks, commanded), executed)));
end
