function scene_data = import_real_scene_json(json_path)
%IMPORT_REAL_SCENE_JSON Import versioned real-scene metadata from UTF-8 JSON.

    if ~isfile(json_path)
        error('import_real_scene_json:MissingFile', 'Scene JSON does not exist: %s', json_path);
    end
    scene_data = jsondecode(fileread(json_path));
    required = {'schema_version', 'frame_id', 'scene_id', 'map', 'tables', 'tasks'};
    for k = 1:numel(required)
        if ~isfield(scene_data, required{k})
            error('import_real_scene_json:MissingField', 'Missing field: %s', required{k});
        end
    end
    if ~strcmp(char(string(scene_data.frame_id)), 'map')
        error('import_real_scene_json:FrameMismatch', 'Only frame_id=map is supported.');
    end
    if isfield(scene_data, 'tasks') && ~isempty(scene_data.tasks)
        scene_data.tasks = normalize_task_fields(scene_data.tasks);
    end
end
