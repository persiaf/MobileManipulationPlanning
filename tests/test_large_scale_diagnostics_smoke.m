function tests = test_large_scale_diagnostics_smoke
%TEST_LARGE_SCALE_DIAGNOSTICS_SMOKE Small coverage and audit checks.
    tests = functiontests(localfunctions);
end

function testCoverageDiagnostic(testCase)
    out = exp_large_scale_coverage_diagnosis(struct('scales', {{'smoke'}}, ...
        'seeds', 1, 'write_files', false));
    verifyEqual(testCase, height(out.run), 1);
    verifyEqual(testCase, out.run.library_coverage_rate, 100);
    verifyTrue(testCase, all(out.task.library_coverable));
end

function testPathAuditLogger(testCase)
    path_cost_audit('reset', 'unit_scene');
    path_cost_audit('set_context', 'bottom_up', 'BOTTOM_UP_STAGE3', 'unit_scene');
    path_cost_audit('record', [0 0 0], [1 0 0], 'astar', true);
    path_cost_audit('record', [0 0 0], [1 0 0], 'astar', false);
    entries = path_cost_audit('snapshot'); path_cost_audit('disable');
    verifyEqual(testCase, numel(entries), 2);
    verifyEqual(testCase, entries(1).directed_key, entries(2).directed_key);
    verifyTrue(testCase, entries(1).expensive_computation);
    verifyFalse(testCase, entries(2).expensive_computation);
end
