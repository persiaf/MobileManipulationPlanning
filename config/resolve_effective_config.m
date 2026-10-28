function effective = resolve_effective_config(varargin)
%RESOLVE_EFFECTIVE_CONFIG Resolve base, profile, experiment, and user layers.
%
% Name-value inputs:
%   profile             C0-C3, L1-L3, V1-V3, or empty for scene_config.
%   seed                Profile seed (default 2026).
%   robot_profile       robot_config (legacy_validated is an identical alias).
%   profile_override    Component struct applied after named profiles.
%   experiment_override Component struct applied after profile overrides.
%   user_override       Highest-priority component struct.
%
% Component structs may contain scene, robot, planner, visualization, and
% real_platform fields. Existing config entry points remain unchanged.

    opts = parse_options(varargin{:});
    effective.scene = scene_config();
    effective.robot = robot_geometry_profile(opts.robot_profile);
    effective.planner = planner_config();
    effective.visualization = visualization_config();
    effective.real_platform = real_platform_config();

    if ~isempty(opts.profile)
        effective.scene = named_scene_profile(opts.profile, opts.seed);
    end
    effective = apply_layer(effective, opts.profile_override);
    effective = apply_layer(effective, opts.experiment_override);
    effective = apply_layer(effective, opts.user_override);
    effective.robot = validate_main_robot_config(effective.robot);
    effective.planner.robot_cfg = validate_main_robot_config(effective.planner.robot_cfg);
    effective.planner.robot_cfg = effective.robot;
    effective.planner.experiment_protocol=experiment_protocol_config();
    effective.planner.experiment_protocol.enforce=true;
    validate_experiment_protocol(effective.planner);
    effective.planner.visualization = legacy_visualization_config( ...
        getfield_default(effective.planner, 'visualization', struct()));
    effective.planner.real_platform = effective.real_platform;
    effective.meta.profile = opts.profile;
    effective.meta.seed = opts.seed;
    effective.meta.robot_profile = 'robot_config';
    effective.meta.override_order = ...
        {'base', 'named_profile', 'profile_override', 'experiment_override', 'user_override'};
    effective.meta.visualization_source = 'visualization_config';
end

function scene = named_scene_profile(profile, seed)
    name = upper(char(profile));
    if any(strcmp(name, {'C0','C1','C2','C3'}))
        scene = scene_complexity_profile_config(name, seed);
    elseif any(strcmp(name, {'L1','L2','L3','V1','V2','V3'}))
        scene = large_scale_profile_config(name, seed);
    else
        error('resolve_effective_config:UnknownProfile', 'Unknown profile "%s".', name);
    end
end

function effective = apply_layer(effective, layer)
    if isempty(layer), return; end
    components = {'scene','robot','planner','visualization','real_platform'};
    for i = 1:numel(components)
        name = components{i};
        if isfield(layer, name)
            effective.(name) = merge_struct_recursive(effective.(name), layer.(name));
        end
    end
end

function opts = parse_options(varargin)
    opts = struct('profile', '', 'seed', 2026, 'robot_profile', 'robot_config', ...
        'profile_override', struct(), 'experiment_override', struct(), ...
        'user_override', struct());
    if mod(numel(varargin), 2) ~= 0
        error('resolve_effective_config:NameValuePairs', 'Arguments must be name-value pairs.');
    end
    for i = 1:2:numel(varargin)
        name = lower(char(varargin{i}));
        if ~isfield(opts, name)
            error('resolve_effective_config:UnknownOption', 'Unknown option "%s".', name);
        end
        opts.(name) = varargin{i + 1};
    end
end

function value = getfield_default(s, name, default_value)
    if isfield(s, name), value = s.(name); else, value = default_value; end
end
