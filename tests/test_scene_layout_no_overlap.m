function tests = test_scene_layout_no_overlap
%TEST_SCENE_LAYOUT_NO_OVERLAP P1 layouts must avoid table overlap.
    tests = functiontests(localfunctions);
end

function testAllSceneComplexityLayoutsHaveNoOverlap(testCase)
    for case_id = {'C0', 'C1', 'C2', 'C3'}
        cfg = scene_complexity_profile_config(case_id{1}, 2026);
        scene = build_scene_from_cfg(cfg);
        [ok, report] = validate_table_layout(scene.scene_cfg);
        verifyTrue(testCase, ok, report.failure_reason);
        verifyEmpty(testCase, regexp(report.failure_reason, 'overlap', 'once'));
    end
end
