function data = exp_real_full_pipeline(real_scene_path, mission_plan_path, execution_log_path)
%EXP_REAL_FULL_PIPELINE Load artifacts from one real-platform run.
    if nargin < 3 || any(cellfun(@isempty, {real_scene_path, mission_plan_path, execution_log_path}))
        error('exp_real_full_pipeline:MissingInput', ...
            'Provide real_scene.json, mission_plan.json, and an execution log.');
    end
    data.scene = import_real_scene_json(real_scene_path);
    data.mission_plan = jsondecode(fileread(mission_plan_path));
    [ok, errors] = validate_mission_plan(data.mission_plan);
    if ~ok
        error('exp_real_full_pipeline:InvalidMission', '%s', strjoin(errors, newline));
    end
    data.execution = import_ros_execution_log(execution_log_path);
end
