function tests = test_cafeteria_scene_planning_identity
    tests = functiontests(localfunctions);
end

function testSceneAndPlanningResolveSameProfile(testCase)
    profile_override.scene = cafeteria_scene_profile_config('complex12', 2026);
    effective = resolve_effective_config('profile', 'CAFETERIA_1X3_2X3', ...
        'seed', 2026, 'robot_profile', 'physical_robot', ...
        'profile_override', profile_override, ...
        'experiment_override', cafeteria_experiment_override());
    scene = build_cafeteria_scene('complex12', 2026);
    verifyEqual(testCase, effective.scene.table_positions, scene.scene_cfg.table_positions, 'AbsTol', 0);
    verifyEqual(testCase, effective.scene.manual_tasks, scene.scene_cfg.manual_tasks, 'AbsTol', 0);
    verifyEqual(testCase, effective.scene.map_width_m, scene.map_info.width_m, 'AbsTol', 0);
    verifyEqual(testCase, effective.scene.map_height_m, scene.map_info.height_m, 'AbsTol', 0);
    verifyEqual(testCase, effective.planner.routing.mode, 'dense');
    verifyEqual(testCase, effective.planner.global_planner, 'convex_astar');
    verifyEqual(testCase, scene.num_tasks, 12);
    verifyEqual(testCase, scene.region_metadata.region_count, 4);
end
