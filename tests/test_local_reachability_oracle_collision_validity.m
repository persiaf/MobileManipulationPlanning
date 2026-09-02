function tests = test_local_reachability_oracle_collision_validity
%TEST_LOCAL_REACHABILITY_ORACLE_COLLISION_VALIDITY Reported poses are collision-free.
    tests = functiontests(localfunctions);
end

function testReportedBestPoseIsCollisionFree(testCase)
    options = struct('case_id', 'F010', 'seed', 2026, ...
        'max_uncovered_tasks', 3);
    outputs = exp_local_reachability_oracle(options);

    scene_cfg = scene_complexity_factor_config('F010', 2026);
    scene = build_scene_from_cfg(scene_cfg);
    robot_cfg = robot_geometry_profile('legacy_validated');

    for i = 1:height(outputs.task)
        pose = [outputs.task.best_x(i), outputs.task.best_y(i), outputs.task.best_theta(i)];
        verifyTrue(testCase, all(isfinite(pose)));
        verifyFalse(testCase, check_footprint_collision( ...
            pose, scene.grid_map, scene.map_info, robot_cfg));
    end
end
