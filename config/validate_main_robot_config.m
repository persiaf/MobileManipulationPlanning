function cfg = validate_main_robot_config(cfg)
%VALIDATE_MAIN_ROBOT_CONFIG Reject alternate models at planning entry points.
%   robot_config() is the sole parameter source. Extra metadata is allowed;
%   every supplied parameter must retain its main-scene value. Older snapshots
%   may omit the derived footprint; fill it from the one canonical model.
%   Low-level geometry
%   helpers remain usable with synthetic fixtures for collision/cache tests.
    canonical = robot_config();
    if isstruct(cfg) && isscalar(cfg) && ~isfield(cfg, 'footprint')
        cfg.footprint = canonical.footprint;
    end
    compare_fields(cfg, canonical, 'robot');
    if isfield(cfg, 'geometry_profile') && ...
            ~any(strcmpi(char(cfg.geometry_profile), {'robot_config','legacy_validated'}))
        conflict('robot.geometry_profile');
    end
end

function compare_fields(actual, expected, prefix)
    if ~isstruct(actual) || ~isscalar(actual)
        conflict(prefix);
    end
    names = fieldnames(expected);
    for k = 1:numel(names)
        key = names{k}; path = [prefix '.' key];
        if ~isfield(actual, key), conflict(path); end
        a = actual.(key); b = expected.(key);
        if isstruct(b)
            compare_fields(a, b, path);
        elseif isnumeric(b)
            if ~isnumeric(a) || ~isequal(size(a), size(b)) || ...
                    any(~isfinite(a(:))) || any(abs(double(a(:))-double(b(:))) > 1e-12)
                conflict(path);
            end
        elseif ~isequaln(a, b)
            conflict(path);
        end
    end
end

function conflict(path)
    error('validate_main_robot_config:ConflictingParameter', ...
        '%s must match config/robot_config.m used by main_mobile_manipulation_planning.m.', path);
end
