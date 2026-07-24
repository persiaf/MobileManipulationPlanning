function [is_valid, errors] = validate_frame_convention(data)
%VALIDATE_FRAME_CONVENTION Validate map-frame metric/radian conventions.

    errors = {};
    if ~isstruct(data)
        errors{end + 1} = 'Input must be a struct.';
    else
        if ~isfield(data, 'frame_id') || ~strcmp(char(string(data.frame_id)), 'map')
            errors{end + 1} = 'frame_id must be map.';
        end
        if isfield(data, 'start_pose') && (~isnumeric(data.start_pose) || numel(data.start_pose) ~= 3)
            errors{end + 1} = 'start_pose must be [x, y, theta] in metres/radians.';
        end
        if isfield(data, 'service_configs')
            configs = data.service_configs;
            for k = 1:numel(configs)
                if ~isfield(configs(k), 'pose2d') || ~isnumeric(configs(k).pose2d) || numel(configs(k).pose2d) ~= 3
                    errors{end + 1} = sprintf('service_configs(%d).pose2d is invalid.', k); %#ok<AGROW>
                end
            end
        end
    end
    is_valid = isempty(errors);
end
