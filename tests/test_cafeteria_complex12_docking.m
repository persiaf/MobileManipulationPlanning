function tests = test_cafeteria_complex12_docking
    tests = functiontests(localfunctions);
end

function testBottomUpSelectsProxyDocksForAllTasks(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'navigation_docking_only');

    verifyEqual(testCase, result.planning_mode, 'navigation_docking_only');
    verifyEqual(testCase, result.service_support_mode, 'cafeteria_2d_proxy');
    verifyEqual(testCase, result.candidate_coverage, 1, 'AbsTol', 0);
    verifyGreaterThan(testCase, result.selected_docks, 0);
    verifyEqual(testCase, sum(result.tasks_per_selected_dock), result.scene.num_tasks);
    verifyEqual(testCase, result.selected_docks, size(result.plan.selected_poses, 1));
end

function testSharedDocksAreUsedAcrossTasks(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'navigation_docking_only');
    verifyGreaterThan(testCase, result.multi_task_dock_count, 0);
    verifyEqual(testCase, result.multi_task_dock_count, ...
        sum(result.tasks_per_selected_dock >= 2));
end

function testSelectedDocksAreCollisionFree(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'navigation_docking_only');
    n = size(result.plan.selected_poses, 1);
    verifyGreaterThan(testCase, n, 0);
    for i = 1:n
        pose = result.plan.selected_poses(i, :);
        verifyFalse(testCase, check_footprint_collision(pose, result.scene.grid_map, ...
            result.scene.map_info, result.robot_cfg));
    end
end

function testVisitOrderVisitsEverySelectedDockAndReturnsToStart(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'navigation_docking_only');
    node_order = result.node_visit_order;
    % Node indexing follows [start; selected_poses]: node 1 is the start pose and
    % the remaining nodes are the selected docks. The return leg is appended by
    % stage3 as stats.route_order(end) == 1.
    verifyEqual(testCase, numel(node_order), result.selected_docks + 1);
    verifyEqual(testCase, node_order(1), 1);
    verifyEqual(testCase, sort(node_order), 1:numel(node_order));
    verifyEqual(testCase, numel(result.visit_order), result.selected_docks);
    verifyTrue(testCase, result.stats.return_to_start);
    verifyEqual(testCase, result.stats.route_order(end), 1);
end