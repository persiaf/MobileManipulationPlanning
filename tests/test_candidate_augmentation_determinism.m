function tests = test_candidate_augmentation_determinism
    tests = functiontests(localfunctions);
end

function testRepeatedRunIsIdentical(testCase)
    [~, damaged, regions, scene, robot_cfg, cfg] = candidate_augmentation_fixture();
    [~, first] = augment_candidate_library(damaged, regions, scene, robot_cfg, cfg);
    [~, second] = augment_candidate_library(damaged, regions, scene, robot_cfg, cfg);
    verifyEqual(testCase, second.selected_region_ids, first.selected_region_ids);
    verifyEqual(testCase, second.selected_poses, first.selected_poses, 'AbsTol', 1e-12);
    verifyEqual(testCase, second.library_coverage_after, first.library_coverage_after);
end
