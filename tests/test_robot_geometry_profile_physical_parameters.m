function tests = test_robot_geometry_profile_physical_parameters
%TEST_ROBOT_GEOMETRY_PROFILE_PHYSICAL_PARAMETERS Physical profile mapping.
    tests = functiontests(localfunctions);
end

function testPhysicalProfileValues(testCase)
    cfg = robot_geometry_profile('physical_robot');
    verifyEqual(testCase, cfg.dual_arm.base_spacing, 0.40, 'AbsTol', 1e-12);
    verifyEqual(testCase, cfg.arm_base_mount_x, 0.06, 'AbsTol', 1e-12);
    verifyEqual(testCase, cfg.arm_base_mount_z, 0.40, 'AbsTol', 1e-12);
    verifyEqual(testCase, cfg.geometry_profile, 'physical_robot');
end
