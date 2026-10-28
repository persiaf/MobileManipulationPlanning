function result = run_selected_experiment(experiment, varargin)
%RUN_SELECTED_EXPERIMENT Execute an experiment through one unified config.
%   RESULT = RUN_SELECTED_EXPERIMENT(NAME) runs the maintained planning
%   pipeline with the registry profile NAME. Existing main scripts remain
%   valid and are intentionally not called or modified.

    project_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(project_root, 'experiment_manager'));
    addpath(fullfile(project_root, 'pipeline'));
    init_paths(project_root);
    options = parse_options(varargin{:});
    if isstruct(experiment), cfg = experiment; else, cfg = experiment_config_builder(experiment, options.builder_args{:}); end
    if isempty(options.output_dir)
        options.output_dir = fullfile('results', 'managed_experiments', cfg.experiment_id);
    end
    if ~isfield(cfg, 'scene_cfg') || ~isfield(cfg, 'robot_cfg') || ~isfield(cfg, 'planner_cfg')
        error('run_selected_experiment:InvalidConfig', 'Config must contain scene_cfg, robot_cfg, and planner_cfg.');
    end
    cfg.robot_cfg = validate_main_robot_config(cfg.robot_cfg);
    if isfield(cfg, 'robot'), cfg.robot = validate_main_robot_config(cfg.robot); end
    if isfield(cfg.planner_cfg, 'robot_cfg')
        cfg.planner_cfg.robot_cfg = validate_main_robot_config(cfg.planner_cfg.robot_cfg);
    end
    cfg.planner_cfg.robot_cfg = cfg.robot_cfg;
    if options.headless
        cfg.planner_cfg.runtime.headless = true;
        cfg.planner_cfg.runtime.disable_visualization = true;
        cfg.planner_cfg.runtime.disable_video = true;
        cfg.planner_cfg.runtime.quiet = true;
        cfg.planner_cfg.runtime.quiet_stage2 = true;
    end

    t = tic;
    scene = build_scene_from_cfg(cfg.scene_cfg);
    scene_cfg = scene.scene_cfg;
    scene_ok = true;
    if isfield(cfg, 'validation') && cfg.validation.validate_scene
        scene_ok = validate_scene_config(scene_cfg, scene.grid_map, scene.map_info);
        if ~scene_ok && cfg.validation.error_on_invalid_scene
            error('run_selected_experiment:InvalidScene', 'Scene validation failed.');
        end
    end
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    timing.scene_time_s = toc(t);

    t = tic;
    candidate_library = build_candidate_library(regions, scene, cfg.robot_cfg, cfg.planner_cfg);
    [optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list, plan] = ...
        stage2_docking_planning(regions, scene, cfg.robot_cfg, cfg.planner_cfg, candidate_library);
    timing.docking_time_s = toc(t);
    result = struct();
    result.status = 'OK'; result.config = cfg; result.scene = scene;
    result.scene_cfg = scene_cfg; result.robot_cfg = cfg.robot_cfg;
    result.planner_cfg = cfg.planner_cfg; result.task_table_map = task_table_map;
    result.regions = regions; result.candidate_library = candidate_library;
    result.plan = plan; result.optimal_poses = optimal_poses;
    result.pose_to_region_map = pose_to_region_map;
    result.task_sequence_summary = task_sequence_summary;
    result.pose_info_list = pose_info_list;
    result.validation = struct('scene_ok', scene_ok); result.timing = timing;
    if options.render_outputs
        result.visualization = render_result_outputs(result, cfg, options.output_dir);
    end
    if isfield(plan, 'status') && strcmpi(plan.status, 'BLOCKED')
        result.status = 'BLOCKED';
        if isfield(plan, 'reason'), result.reason = plan.reason;
        else, result.reason = 'STAGE2_BLOCKED'; end
        return;
    end
    t = tic;
    [scene, global_path, all_poses, visit_order, executed_tasks, total_path_len, stats] = ...
        stage3_global_planning(scene, optimal_poses, task_sequence_summary, task_table_map, ...
        cfg.robot_cfg, cfg.planner_cfg, pose_info_list, timing, plan);
    result.scene = scene; result.global_path = global_path; result.all_poses = all_poses;
    result.visit_order = visit_order; result.executed_tasks = executed_tasks;
    result.total_path_len = total_path_len; result.stats = stats;
    result.timing.stage3_time_s = toc(t);
    if options.render_outputs
        result.visualization = render_result_outputs(result, cfg, options.output_dir);
    end
    if options.save_record
        if ~isfolder(options.output_dir), mkdir(options.output_dir); end
        record = result; %#ok<NASGU>
        save(fullfile(options.output_dir, [cfg.experiment_id '_record.mat']), 'record');
    end
end

function outputs = render_result_outputs(result, cfg, output_dir)
%RENDER_RESULT_OUTPUTS Adapt the runner result to the renderer's public API.
% The renderer expects scene/robot/plan/stats as separate arguments; passing
% the aggregate runner result silently produces invalid scene data.
    vis_cfg = visualization_config();
    vis_cfg.output_dir = output_dir;
    stats = struct();
    if isfield(result, 'global_path')
        stats.global_path = result.global_path;
    end
    outputs = render_experiment_outputs(result.scene, cfg.robot_cfg, result.plan, ...
        stats, vis_cfg, 'candidate_library', result.candidate_library);
end

function o = parse_options(varargin)
    o = struct('headless', false, 'save_record', false, 'render_outputs', true, ...
        'output_dir', '', 'builder_args', {{}});
    if mod(numel(varargin), 2) ~= 0, error('run_selected_experiment:Options', 'Options must be name/value pairs.'); end
    for k = 1:2:numel(varargin)
        key = lower(char(varargin{k})); val = varargin{k+1};
        switch key
            case 'headless', o.headless = logical(val);
            case 'save_record', o.save_record = logical(val);
            case 'render_outputs', o.render_outputs = logical(val);
            case 'output_dir', o.output_dir = char(val);
            case 'builder_args', o.builder_args = val;
            otherwise, error('run_selected_experiment:UnknownOption', 'Unknown option "%s".', key);
        end
    end
end
