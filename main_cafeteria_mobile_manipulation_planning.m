function result = main_cafeteria_mobile_manipulation_planning(task_mode, seed, planning_mode)
%MAIN_CAFETERIA_MOBILE_MANIPULATION_PLANNING Interactive cafeteria entry point.
%
%   result = main_cafeteria_mobile_manipulation_planning()
%   result = main_cafeteria_mobile_manipulation_planning('basic3', 2026)
%   result = main_cafeteria_mobile_manipulation_planning('complex12', 2026, 'navigation_docking_only')

    if nargin < 1 || isempty(task_mode), task_mode = 'complex12'; end
    if nargin < 2 || isempty(seed), seed = 2026; end
    if nargin < 3 || isempty(planning_mode), planning_mode = 'navigation_docking_only'; end

    project_root = fileparts(mfilename('fullpath'));
    addpath(fullfile(project_root, 'pipeline'));
    init_paths(project_root);

    fprintf('===== CAFETERIA MOBILE MANIPULATION PLANNING =====\n');
    fprintf('Task mode: %s | seed: %d\n', lower(char(task_mode)), seed);
    fprintf('Figures remain visible in the MATLAB desktop.\n');
    result = run_cafeteria_planning(task_mode, seed, true, 'on', planning_mode);

    fprintf('\n===== CAFETERIA RESULT =====\n');
    fprintf('Status: %s\n', result.status);
    fprintf('Tasks: %d | active tables: %d | regions: %d\n', ...
        result.scene.num_tasks, numel(result.scene.scene_cfg.active_table_ids), ...
        result.scene.region_metadata.region_count);
    fprintf('Selected docks: %d | multi-task docks: %s\n', ...
        result.selected_docks, display_number(result.multi_task_dock_count));
    fprintf('Coverage: %s | path waypoints: %d | collision count: %s\n', ...
        display_number(result.candidate_coverage), result.waypoint_count, ...
        display_number(result.path_collision_count));
    fprintf('=============================\n');
end

function text = display_number(value)
    if isempty(value) || ~isfinite(value)
        text = 'N/A';
    else
        text = sprintf('%.6g', value);
    end
end
