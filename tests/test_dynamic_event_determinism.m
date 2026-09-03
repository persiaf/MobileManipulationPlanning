function tests = test_dynamic_event_determinism
    tests = functiontests(localfunctions);
end

function testAddTasksDeterministic(testCase)
    state = p5_fixture_state();
    [changed_a, event_a] = apply_dynamic_event(state, 'ADD_TASKS', 2);
    [changed_b, event_b] = apply_dynamic_event(state, 'ADD_TASKS', 2);
    verifyEqual(testCase, event_a, event_b);
    verifyEqual(testCase, changed_a.scene.num_tasks, changed_b.scene.num_tasks);
    verifyEqual(testCase, [[changed_a.scene.tasks.x]', [changed_a.scene.tasks.y]'], ...
        [[changed_b.scene.tasks.x]', [changed_b.scene.tasks.y]'], 'AbsTol', 1e-12);
end
