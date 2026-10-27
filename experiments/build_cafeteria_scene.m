function scene = build_cafeteria_scene(task_mode, seed)
%BUILD_CAFETERIA_SCENE Build the deterministic named cafeteria scene.

    if nargin < 1 || isempty(task_mode), task_mode = 'complex12'; end
    if isnumeric(task_mode)
        seed = task_mode;
        task_mode = 'complex12';
    elseif nargin < 2 || isempty(seed)
        seed = 2026;
    end
    scene = build_scene_from_cfg(cafeteria_scene_profile_config(task_mode, seed));
end
