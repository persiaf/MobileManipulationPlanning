function tests = test_cafeteria_start_collision_free
    tests = functiontests(localfunctions);
end

function testPhysicalRobotStartIsFree(testCase)
    scene = build_cafeteria_scene('complex12', 2026);
    report = validate_cafeteria_scene(scene, robot_geometry_profile('physical_robot'));
    verifyFalse(testCase, report.start_collision);
    verifyEqual(testCase, scene.start_pose, [0.8, 0.8, 0], 'AbsTol', 1e-12);
end
