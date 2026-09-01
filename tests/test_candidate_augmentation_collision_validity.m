function tests = test_candidate_augmentation_collision_validity
    tests = functiontests(localfunctions);
end

function testSelectedCandidatesAreCollisionFree(testCase)
    [~, damaged, regions, scene, robot_cfg, cfg] = candidate_augmentation_fixture();
    [~, diagnostics] = augment_candidate_library( ...
        damaged, regions, scene, robot_cfg, cfg);
    verifyGreaterThan(testCase, diagnostics.selected_supplementary_count, 0);
    for i = 1:size(diagnostics.selected_poses, 1)
        verifyFalse(testCase, check_footprint_collision(diagnostics.selected_poses(i,:), ...
            scene.grid_map, scene.map_info, robot_cfg));
    end
end
