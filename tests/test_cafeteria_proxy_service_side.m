function tests = test_cafeteria_proxy_service_side
    tests = functiontests(localfunctions);
end

function testExpectedSides(testCase)
    scene = build_cafeteria_scene('complex12', 2026);
    expected = [4 4 4 4 3 3 1 2 3 3 4 4];
    actual = zeros(1, scene.num_tasks);
    for i = 1:scene.num_tasks
        task = scene.tasks(i); cfg = scene.scene_cfg;
        local = rot2(-cfg.table_orientations(task.table_id)) * ...
            ([task.x task.y] - cfg.table_positions(task.table_id, :))';
        if abs(local(1))/(cfg.table.length/2) > abs(local(2))/(cfg.table.width/2)
            actual(i) = 3 + (local(1)>0);
        else
            actual(i) = 1 + (local(2)<0);
        end
    end
    verifyEqual(testCase, actual, expected);
end
