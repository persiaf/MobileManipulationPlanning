function entry = experiment_registry(name)
%EXPERIMENT_REGISTRY Registry of reproducible experiment configurations.
%   R = EXPERIMENT_REGISTRY() returns all maintained experiment entries.
%   E = EXPERIMENT_REGISTRY(NAME) returns one entry by id or alias.
%
% The registry contains metadata only. It does not execute experiments or
% change planner implementations/results.

    entries = [ ...
        make_entry('baseline', {'default','main'}, 'Default project scene and planner.', 'scene_config', 'robot_config', 'planner_config', 'bottom_up', 'default'); ...
        make_entry('cafeteria_basic3', {'cafeteria','cafeteria_basic'}, 'Cafeteria 3-task profile.', 'cafeteria_basic3', 'robot_config', 'planner_config', 'bottom_up', 'default'); ...
        make_entry('cafeteria_complex12', {'cafeteria_full'}, 'Cafeteria 12-task profile.', 'cafeteria_complex12', 'robot_config', 'planner_config', 'bottom_up', 'default'); ...
        make_entry('hardware_matched_lab', {'lab','physical_lab'}, 'Hardware-matched laboratory scene.', 'hardware_matched_lab', 'physical_robot', 'planner_config', 'bottom_up', 'default'); ...
        make_entry('strategy_comparison', {'strategies'}, 'Default scene with the selected strategy profile.', 'scene_config', 'robot_config', 'planner_config', 'bottom_up', 'default') ...
    ];

    if nargin == 0 || isempty(name)
        entry = entries;
        return;
    end
    query = lower(char(name));
    hit = false(size(entries));
    for k = 1:numel(entries)
        hit(k) = strcmpi(entries(k).id, query) || any(strcmpi(entries(k).aliases, query));
    end
    if ~any(hit)
        error('experiment_registry:UnknownExperiment', ...
            'Unknown experiment "%s". Available ids: %s', query, strjoin({entries.id}, ', '));
    end
    entry = entries(find(hit, 1, 'first'));
end

function e = make_entry(id, aliases, description, scene_profile, robot_profile, planner_profile, strategy_profile, validation_profile)
    e = struct('id', id, 'aliases', {aliases}, 'description', description, ...
        'scene_profile', scene_profile, 'robot_profile', robot_profile, ...
        'planner_profile', planner_profile, 'strategy_profile', strategy_profile, ...
        'validation_profile', validation_profile);
end
