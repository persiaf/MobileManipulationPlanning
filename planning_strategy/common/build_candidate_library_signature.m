function signature = build_candidate_library_signature(regions, scene, robot_cfg, planner_cfg)
%BUILD_CANDIDATE_LIBRARY_SIGNATURE Identify candidate generation semantics.
    payload = struct();
    payload.geometry_revision = 'task_pose_ik_20261110_v1';
    payload.grid_map = uint8(scene.grid_map ~= 0);
    payload.map_info = scene.map_info;
    payload.scene_cfg = scene.scene_cfg;
    payload.tasks = zeros(scene.num_tasks, 3);
    for i = 1:scene.num_tasks
        payload.tasks(i,:) = [scene.tasks(i).x, scene.tasks(i).y, scene.tasks(i).z];
    end
    % Execution status must not invalidate a candidate library, but all
    % task semantics consumed by the manipulation evaluator must do so.
    payload.task_constraints = cell(1, scene.num_tasks);
    constraint_fields = {'task_type', 'manipulation', 'continuous_points', ...
        'object_pose_map', 'left_grasp_pose', 'right_grasp_pose', ...
        'destination_pose', 'object_size', 'required_height_range'};
    for i = 1:scene.num_tasks
        constraints = struct();
        for k = 1:numel(constraint_fields)
            name = constraint_fields{k};
            if isfield(scene.tasks, name) && ~isempty(scene.tasks(i).(name))
                constraints.(name) = scene.tasks(i).(name);
            end
        end
        payload.task_constraints{i} = constraints;
    end
    payload.region_task_ids = arrayfun(@(r) r.task_ids, regions, 'UniformOutput', false);
    payload.robot_cfg = robot_cfg;
    payload.reachability = planner_cfg.reachability;
    payload.cost_weights = planner_cfg.cost_weights;
    payload.candidate_cost_options = planner_cfg.candidate;
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
