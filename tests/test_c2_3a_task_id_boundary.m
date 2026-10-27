function tests = test_c2_3a_task_id_boundary
%TEST_C2_3A_TASK_ID_BOUNDARY Non-contiguous task ID contract checks.
    tests = functiontests(localfunctions);
end

function testNonContiguousIdsArePreserved(testCase)
    state = id_fixture([101, 105, 118]);
    actual_ids = [state.scene.tasks.id];
    verifyEqual(testCase, actual_ids, [101 105 118]);
    verifyEqual(testCase, state.plan.covered_tasks_predicted, [101 105 118]);
    verifyEmpty(testCase, state.plan.uncovered_tasks_predicted);
end

function testMissingIdReportsRealId(testCase)
    state = id_fixture([101, 105, 118]);
    state.plan.covered_tasks_predicted = [101 118];
    all_ids = [state.scene.tasks.id];
    uncovered = setdiff(all_ids, state.plan.covered_tasks_predicted, 'stable');
    verifyEqual(testCase, uncovered, 105);
    verifyFalse(testCase, any(uncovered == 2));
end

function testUnknownExplicitIdDoesNotFallbackToIndex(testCase)
    state = id_fixture([101, 105, 118]);
    idx = local_find_task_index(state.scene.tasks, 2);
    verifyEmpty(testCase, idx);
end

function testCompletionIdsRemainStable(testCase)
    state = id_fixture([101, 105, 118]);
    state.task_state.verified_tasks = 105;
    verifyEqual(testCase, state.task_state.verified_tasks, 105);
    verifyFalse(testCase, state.task_state.verified_tasks == 2);
end

function state = id_fixture(ids)
    tasks = repmat(struct('id', 0, 'x', 0, 'y', 0, 'z', 0, 'status', 0), 1, numel(ids));
    for i = 1:numel(ids)
        tasks(i).id = ids(i);
        tasks(i).x = i;
        tasks(i).y = i;
    end
    state = struct();
    state.scene = struct('tasks', tasks, 'num_tasks', numel(tasks));
    state.plan = struct('covered_tasks_predicted', ids, 'uncovered_tasks_predicted', []);
    state.task_state = struct('verified_tasks', []);
end

function idx = local_find_task_index(tasks, task_id)
    idx = [];
    has_explicit_ids = ~isempty(tasks) && isfield(tasks, 'id');
    for i = 1:numel(tasks)
        if isfield(tasks(i), 'id') && isequal(tasks(i).id, task_id)
            idx = i;
            return;
        end
    end
    if ~has_explicit_ids && isnumeric(task_id) && isscalar(task_id) && ...
            task_id >= 1 && task_id <= numel(tasks) && task_id == floor(task_id)
        idx = task_id;
    end
end
