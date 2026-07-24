function mission_plan = export_mission_plan_json(output_path, scene, route_order, service_configs, real_cfg)
%EXPORT_MISSION_PLAN_JSON Export ordered service targets for a Linux executor.

    if nargin < 5 || isempty(real_cfg)
        real_cfg = real_platform_config();
    end
    mission_plan = struct();
    mission_plan.schema_version = real_cfg.schema_version;
    mission_plan.frame_id = real_cfg.frame_id;
    mission_plan.scene_id = real_cfg.scene_id;
    mission_plan.planner_version = real_cfg.planner_version;
    mission_plan.start_pose = reshape(scene.start_pose, 1, 3);
    mission_plan.route_order = reshape(route_order, 1, []);
    mission_plan.service_configs = service_configs;

    [ok, errors] = validate_mission_plan(mission_plan);
    if ~ok
        error('export_mission_plan_json:InvalidPlan', '%s', strjoin(errors, newline));
    end
    if nargin < 1 || isempty(output_path)
        return;
    end
    output_dir = fileparts(output_path);
    if ~isempty(output_dir) && ~isfolder(output_dir)
        mkdir(output_dir);
    end
    json_payload = mission_plan;
    json_payload.route_order = num2cell(mission_plan.route_order);
    service_cells = cell(1, numel(service_configs));
    for k = 1:numel(service_configs)
        service_payload = service_configs(k);
        service_payload.task_ids = num2cell(service_payload.task_ids);
        service_cells{k} = service_payload;
    end
    json_payload.service_configs = service_cells;
    try
        json_text = jsonencode(json_payload, 'PrettyPrint', true);
    catch
        json_text = jsonencode(json_payload);
    end
    fid = fopen(output_path, 'w', 'n', 'UTF-8');
    if fid < 0
        error('export_mission_plan_json:OpenFailed', 'Cannot write %s', output_path);
    end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fwrite(fid, json_text, 'char');
end
