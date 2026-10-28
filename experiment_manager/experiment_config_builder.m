function cfg = experiment_config_builder(profile, varargin)
%EXPERIMENT_CONFIG_BUILDER Build one complete experiment configuration.
%   CFG = EXPERIMENT_CONFIG_BUILDER(NAME) uses the registry entry NAME.
%   CFG = EXPERIMENT_CONFIG_BUILDER(NAME, 'seed', 7, 'override', S) applies
%   deterministic seed and recursive overrides without changing base files.

    % Keep this builder usable as a standalone entry point as well as after
    % init_paths() has already been called by a legacy main script.
    manager_root = fileparts(mfilename('fullpath'));
    project_root = fileparts(manager_root);
    pipeline_dir = fullfile(project_root, 'pipeline');
    if exist('init_paths', 'file') ~= 2 && isfile(fullfile(pipeline_dir, 'init_paths.m'))
        addpath(pipeline_dir);
    end
    if exist('init_paths', 'file') == 2 && exist('scene_config', 'file') ~= 2
        init_paths(project_root);
    end
    if nargin < 1 || isempty(profile), profile = 'baseline'; end
    options = parse_options(varargin{:});
    if isstruct(profile)
        entry = experiment_registry('baseline');
        base = profile;
        if isfield(base, 'experiment_id'), entry.id = base.experiment_id; end
    else
        entry = experiment_registry(profile);
        base = struct();
    end

    seed = options.seed;
    if isempty(seed), seed = 2026; end
    base.scene_profile = entry.scene_profile;
    base.robot_profile = entry.robot_profile;
    base.planner_profile = entry.planner_profile;
    base.strategy_profile = entry.strategy_profile;
    base.validation_profile = entry.validation_profile;
    base.experiment_id = entry.id;
    base.seed = seed;

    base.scene_cfg = make_scene(entry.scene_profile, seed);
    base.robot_cfg = make_robot(entry.robot_profile);
    base.planner_cfg = planner_config();
    if ~isempty(entry.planner_profile) && ~strcmpi(entry.planner_profile, 'planner_config')
        base.planner_cfg = apply_profile(base.planner_cfg, entry.planner_profile);
    end
    base.planner_cfg = apply_strategy(base.planner_cfg, entry.strategy_profile);
    base.validation = make_validation(entry.validation_profile);

    % Expose semantic profile sections as well as legacy *_cfg fields.
    base.scene = base.scene_cfg;
    base.robot = base.robot_cfg;
    base.planner = base.planner_cfg;
    base.strategy = base.planner_cfg.strategy;
    if isfield(options, 'scene_override'), base.scene_cfg = merge_struct_recursive(base.scene_cfg, options.scene_override); end
    if isfield(options, 'robot_override'), base.robot_cfg = merge_struct_recursive(base.robot_cfg, options.robot_override); end
    if isfield(options, 'planner_override'), base.planner_cfg = merge_struct_recursive(base.planner_cfg, options.planner_override); end
    if isfield(options, 'strategy_override')
        base.planner_cfg.strategy = merge_struct_recursive(base.planner_cfg.strategy, options.strategy_override);
        base.strategy = base.planner_cfg.strategy;
    end
    if isfield(options, 'validation_override'), base.validation = merge_struct_recursive(base.validation, options.validation_override); end
    if isfield(options, 'override'), base = merge_struct_recursive(base, options.override); end
    base.robot_cfg = validate_main_robot_config(base.robot_cfg);
    base.robot = validate_main_robot_config(base.robot);
    base.planner_cfg.robot_cfg = validate_main_robot_config(base.planner_cfg.robot_cfg);
    base.planner.robot_cfg = validate_main_robot_config(base.planner.robot_cfg);
    robot_geometry_profile(base.robot_profile);
    base.robot_profile = 'robot_config';
    base.planner_cfg.robot_cfg = base.robot_cfg;
    base.planner_cfg.experiment_protocol=experiment_protocol_config();
    base.planner_cfg.experiment_protocol.enforce=true;
    validate_experiment_protocol(base.planner_cfg);
    base.scene = base.scene_cfg; base.robot = base.robot_cfg; base.planner = base.planner_cfg;
    base.strategy = base.planner_cfg.strategy;
    cfg = base;
end

function cfg = make_scene(profile, seed)
    switch lower(char(profile))
        case {'scene_config','default','baseline'}, cfg = scene_config();
        otherwise, error('experiment_config_builder:UnknownSceneProfile', 'Unknown scene profile "%s".', char(profile));
    end
    if isfield(cfg, 'random_seed') && ~strcmpi(char(cfg.random_seed), 'shuffle'), cfg.random_seed = seed; end
end

function cfg = make_robot(profile)
    switch lower(char(profile))
        case {'robot_config','default'}, cfg = robot_config();
        otherwise, error('experiment_config_builder:UnknownRobotProfile', 'Unknown robot profile "%s".', char(profile));
    end
end

function cfg = apply_profile(cfg, profile)
    if isstruct(profile), cfg = merge_struct_recursive(cfg, profile); end
end

function cfg = apply_strategy(cfg, profile)
    if isstruct(profile), cfg.strategy = merge_struct_recursive(cfg.strategy, profile);
    elseif ~isempty(profile), cfg.strategy.name = char(profile); end
end

function v = make_validation(profile)
    v = struct('profile', char(profile), 'validate_scene', true, 'validate_coverage', true, 'error_on_invalid_scene', false);
end

function o = parse_options(varargin)
    o = struct('seed', []);
    if mod(numel(varargin), 2) ~= 0, error('experiment_config_builder:Options', 'Options must be name/value pairs.'); end
    for k = 1:2:numel(varargin)
        key = lower(char(varargin{k})); val = varargin{k+1};
        switch key
            case 'seed', o.seed = val;
            case {'override','scene_override','robot_override','planner_override','strategy_override','validation_override'}, o.(key) = val;
            otherwise, error('experiment_config_builder:UnknownOption', 'Unknown option "%s".', key);
        end
    end
end
