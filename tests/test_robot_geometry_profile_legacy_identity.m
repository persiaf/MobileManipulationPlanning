function tests = test_robot_geometry_profile_legacy_identity
%TEST_ROBOT_GEOMETRY_PROFILE_LEGACY_IDENTITY Legacy profile equals robot_config.
    tests = functiontests(localfunctions);
end

function testLegacyIdentity(testCase)
    base = robot_config();
    legacy = robot_geometry_profile('legacy_validated');
    verifyEqual(testCase, legacy.chassis, base.chassis);
    verifyEqual(testCase, legacy.dual_arm, base.dual_arm);
    verifyEqual(testCase, legacy.single_arm, base.single_arm);
    verifyEqual(testCase, legacy.arm_base_mount_x, base.arm_base_mount_x);
    verifyEqual(testCase, legacy.arm_base_mount_z, base.arm_base_mount_z);
end
