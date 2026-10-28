function tests = test_table_task_consistency_repair
    tests = functiontests(localfunctions);
end

function testRotatedTaskCheckedInTableFrame(testCase)
    cfg = rc2_four_table_scene_config();
    cfg.table_positions = [2 2]; cfg.table_orientations = pi/4; cfg.num_tables = 1;
    p = [2 2]+(rot2(pi/4)*[0.45;0])';
    cfg.manual_tasks = [1 p 0.5];
    scene = build_scene_from_cfg(cfg);
    report = validate_tabletop_tasks(cfg,scene.tasks);
    verifyTrue(testCase, report.valid);
    cfg.manual_tasks(1,2:3) = [2 2]+(rot2(pi/4)*[0;0.4])';
    verifyError(testCase, @() build_scene_from_cfg(cfg), 'validate_tabletop_tasks:OutsideTable');
end

function testExplicitOwnershipIsPreserved(testCase)
    cfg = rc2_four_table_scene_config(); scene = build_scene_from_cfg(cfg);
    map = assign_tasks_to_tables(scene);
    for k=1:numel(map)
        verifyEqual(testCase, [scene.tasks(map{k}).table_id], repmat(k,1,numel(map{k})));
    end
end

function testAddedTasksRotateWithTable(testCase)
    state = p5_fixture_state();
    state.scene.scene_cfg.table_orientations(:) = pi/4;
    [changed,event] = apply_dynamic_event(state,'ADD_TASKS',1);
    tasks = changed.scene.tasks(ismember([changed.scene.tasks.id],event.added_task_ids));
    center = state.scene.scene_cfg.table_positions(tasks(1).table_id,:);
    local = (rot2(-pi/4)*([[tasks.x]' [tasks.y]']-center)')';
    verifyEqual(testCase, local, [0.18 0.12;-0.18 0.12;0.18 -0.12;-0.18 -0.12], 'AbsTol',1e-12);
end
