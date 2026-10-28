function cfg = robot_geometry_profile(profile_name)
%ROBOT_GEOMETRY_PROFILE Compatibility names for the one main-scene robot.
%   Both accepted names return robot_config() without parameter overrides.

    if nargin < 1 || isempty(profile_name)
        profile_name = 'robot_config';
    end

    switch lower(char(profile_name))
        case {'robot_config', 'legacy_validated'}
            cfg = robot_config();
        otherwise
            error('robot_geometry_profile:UnknownProfile', ...
                  'Unsupported robot profile "%s". Use robot_config.', char(profile_name));
    end
end
