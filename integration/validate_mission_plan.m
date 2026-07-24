function [is_valid, errors] = validate_mission_plan(plan)
%VALIDATE_MISSION_PLAN Validate the MATLAB mission-plan structure.

    errors = {};
    required = {'schema_version', 'frame_id', 'scene_id', 'planner_version', ...
        'start_pose', 'route_order', 'service_configs'};
    if ~isstruct(plan)
        errors = {'Mission plan must be a struct.'};
        is_valid = false;
        return;
    end
    for k = 1:numel(required)
        if ~isfield(plan, required{k})
            errors{end + 1} = ['Missing field: ', required{k}]; %#ok<AGROW>
        end
    end
    [frame_ok, frame_errors] = validate_frame_convention(plan);
    if ~frame_ok, errors = [errors, frame_errors]; end %#ok<AGROW>

    if isfield(plan, 'service_configs')
        required_service = {'id', 'pose2d', 'lift_height_m', 'table_id', ...
            'task_ids', 'arm_mode', 'arm_assignments', 'policy_id', ...
            'position_tolerance_m', 'yaw_tolerance_rad'};
        for i = 1:numel(plan.service_configs)
            for k = 1:numel(required_service)
                if ~isfield(plan.service_configs(i), required_service{k})
                    errors{end + 1} = sprintf('service_configs(%d) missing %s.', ...
                        i, required_service{k}); %#ok<AGROW>
                end
            end
        end
    end
    is_valid = isempty(errors);
end
