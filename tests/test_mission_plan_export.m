function tests = test_mission_plan_export
    tests = functiontests(localfunctions);
end

function testJsonRoundTrip(testCase)
    [scene, ~, planner_cfg] = create_test_fixture();
    summary = struct('covered_tasks', 1, 'pose', [1.5, 1.2, 0], 'table_id', 1);
    arm_map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    arm_map(1) = 'left';
    info = struct('covered_count', 1, 'arm_assign_map', arm_map);
    service = build_service_config(1, info, summary, planner_cfg.real_platform);
    output_path = [tempname, '.json'];
    cleanup = onCleanup(@() delete_if_present(output_path)); %#ok<NASGU>
    plan = export_mission_plan_json(output_path, scene, 1, service, planner_cfg.real_platform);
    raw_json = fileread(output_path);
    verifyNotEmpty(testCase, regexp(raw_json, '"route_order"\s*:\s*\[', 'once'));
    verifyNotEmpty(testCase, regexp(raw_json, '"service_configs"\s*:\s*\[', 'once'));
    verifyNotEmpty(testCase, regexp(raw_json, '"task_ids"\s*:\s*\[', 'once'));
    decoded = jsondecode(raw_json);
    [ok, errors] = validate_mission_plan(decoded);
    verifyTrue(testCase, ok, strjoin(errors, newline));
    verifyEqual(testCase, decoded.scene_id, plan.scene_id);
end

function delete_if_present(path_value)
    if isfile(path_value), delete(path_value); end
end
