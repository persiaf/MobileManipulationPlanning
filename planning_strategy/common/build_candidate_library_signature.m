function signature = build_candidate_library_signature(regions, scene, robot_cfg, planner_cfg)
%BUILD_CANDIDATE_LIBRARY_SIGNATURE Identify candidate generation semantics.
    payload = struct();
    payload.grid_map = uint8(scene.grid_map ~= 0);
    payload.map_info = scene.map_info;
    payload.scene_cfg = scene.scene_cfg;
    payload.tasks = zeros(scene.num_tasks, 3);
    for i = 1:scene.num_tasks
        payload.tasks(i,:) = [scene.tasks(i).x, scene.tasks(i).y, scene.tasks(i).z];
    end
    payload.region_task_ids = arrayfun(@(r) r.task_ids, regions, 'UniformOutput', false);
    payload.robot_cfg = robot_cfg;
    payload.reachability = planner_cfg.reachability;
    if isfield(planner_cfg, 'docking_error_model')
        payload.docking_error_model = planner_cfg.docking_error_model;
    else
        payload.docking_error_model = struct();
    end
    if isfield(planner_cfg, 'candidate_augmentation')
        payload.candidate_augmentation = planner_cfg.candidate_augmentation;
    else
        payload.candidate_augmentation = struct();
    end
    signature = build_struct_signature(payload);
end
