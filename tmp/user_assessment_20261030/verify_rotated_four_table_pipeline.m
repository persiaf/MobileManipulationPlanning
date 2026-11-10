% One current-config, headless end-to-end rotated-table scenario.
project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(project_root, 'pipeline')); init_paths(project_root);
addpath(fullfile(project_root, 'experiment_manager'));
out_dir = fullfile(fileparts(mfilename('fullpath')), 'rotated_four_tables');
if ~isfolder(out_dir), mkdir(out_dir); end
diary(fullfile(out_dir, 'pipeline.log'));
log_cleanup = onCleanup(@() diary('off'));
s = struct('table_positions', [2 2;5 2;2 5;5 5], ...
    'num_tables', 4, 'table_orientations', deg2rad([0;30;90;-45]), ...
    'random_seed', 2026);
cfg = experiment_config_builder('baseline', 'scene_override', s);
fprintf('MATLAB_VERSION=%s\n', version);
fprintf('ANGLES_DEG=%s\n', mat2str(rad2deg(cfg.scene_cfg.table_orientations')));
fprintf('PLANNER=%s STRATEGY=%s REACHABILITY_THRESHOLD=%.6f\n', ...
    cfg.planner_cfg.global_planner, cfg.planner_cfg.strategy.name, cfg.planner_cfg.reachability.threshold);
t0 = tic;
try
    result = run_selected_experiment(cfg, 'headless', true, ...
        'render_outputs', false, 'save_record', true, 'output_dir', out_dir);
    elapsed = toc(t0);
    fprintf('PIPELINE_STATUS=%s ELAPSED_S=%.6f\n', result.status, elapsed);
    fprintf('TABLE_COUNT=%d TASK_COUNT=%d CANDIDATE_COUNT=%d SELECTED_DOCKS=%d\n', ...
        size(result.scene.scene_cfg.table_positions,1), result.scene.num_tasks, ...
        result.candidate_library.total_candidates, size(result.optimal_poses,1));
    if isfield(result,'reason'), fprintf('REASON=%s\n',result.reason); end
    if isfield(result,'stats')
        fprintf('VERIFIED_COMPLETION_RATE=%.6f PATH_LENGTH_M=%.9f\n', ...
            result.stats.verified_completion_rate, result.total_path_len);
        fprintf('PATH_VALIDATION_JSON=%s\n',jsonencode(result.stats.path_validation));
    end
    % The manager exits before saving when Stage 2 is blocked; preserve it too.
    save(fullfile(out_dir,'assessment_result.mat'),'result','elapsed');
catch err
    fprintf('PIPELINE_STATUS=ERROR ELAPSED_S=%.6f\n',toc(t0));
    fprintf('ERROR_IDENTIFIER=%s\nERROR_MESSAGE=%s\n',err.identifier,err.message);
    fprintf('%s\n', getReport(err,'extended','hyperlinks','off'));
    diary('off'); rethrow(err);
end
diary('off');
