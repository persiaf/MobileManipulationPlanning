function tests = test_task_facing_heading_coverage_preservation
%TEST_TASK_FACING_HEADING_COVERAGE_PRESERVATION Placeholder audit for shortlist.
    tests = functiontests(localfunctions);
end

function testShortlistNotEnabled(testCase)
    cfg = planner_config();
    verifyFalse(testCase, isfield(cfg, 'task_facing_heading_shortlist'));
end
