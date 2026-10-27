function tests = test_effective_config_resolution
    tests = functiontests(localfunctions);
end

function testOverridePrecedence(testCase)
    profile.planner.routing.sparse_knn.k = 5;
    experiment.planner.routing.sparse_knn.k = 4;
    user.planner.routing.sparse_knn.k = 3;
    effective = resolve_effective_config('profile', 'L2', 'seed', 17, ...
        'profile_override', profile, 'experiment_override', experiment, ...
        'user_override', user);
    verifyEqual(testCase, effective.scene.scene_complexity_case, 'L2');
    verifyEqual(testCase, effective.scene.random_seed, 17);
    verifyEqual(testCase, effective.planner.routing.sparse_knn.k, 3);
    verifyEqual(testCase, effective.meta.override_order, ...
        {'base','named_profile','profile_override','experiment_override','user_override'});
end

function testRobotAndInterfaceRemainSeparate(testCase)
    effective = resolve_effective_config('robot_profile', 'physical_robot');
    verifyEqual(testCase, effective.robot.geometry_profile, 'physical_robot');
    verifyFalse(testCase, effective.real_platform.enabled);
    verifyFalse(testCase, effective.real_platform.lift.enabled);
end

function testSummaryReflectsResolvedValues(testCase)
    effective = resolve_effective_config('profile', 'V2', 'seed', 2026);
    text = evalc('summary = print_effective_config(effective);');
    verifyEqual(testCase, summary.map_size_m, [20, 20]);
    verifyEqual(testCase, summary.table_count, 40);
    verifyEqual(testCase, summary.static_obstacle_count, 4);
    verifyNotEmpty(testCase, strfind(text, 'visualization source: visualization_config')); %#ok<STRCLFH>
end
