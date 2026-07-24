function [is_valid, message] = validate_service_config_collision(service_config, grid_map, map_info, robot_cfg)
%VALIDATE_SERVICE_CONFIG_COLLISION Validate one service pose against the map.

    is_valid = false;
    message = '';
    if ~isstruct(service_config) || ~isfield(service_config, 'pose2d') || ...
            numel(service_config.pose2d) ~= 3
        message = 'service_config.pose2d must be [x, y, theta].';
        return;
    end
    if check_footprint_collision(service_config.pose2d, grid_map, map_info, robot_cfg)
        message = 'Service pose collides with an occupied or inflated map cell.';
        return;
    end
    is_valid = true;
end
