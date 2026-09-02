function tests = test_scene_layout_inside_map
%TEST_SCENE_LAYOUT_INSIDE_MAP P1 layouts keep rotated tables inside bounds.
    tests = functiontests(localfunctions);
end

function testAllSceneComplexityLayoutsAreInsideMap(testCase)
    for case_id = {'C0', 'C1', 'C2', 'C3'}
        cfg = scene_complexity_profile_config(case_id{1}, 2026);
        scene = build_scene_from_cfg(cfg);
        verifyTrue(testCase, scene.scene_cfg.layout_validation.is_valid);
        verifyEmpty(testCase, regexp(scene.scene_cfg.layout_validation.failure_reason, 'map_boundary', 'once'));
    end
end
