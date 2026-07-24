function [grid_map, map_info, metadata] = load_ros_map(yaml_path)
%LOAD_ROS_MAP Load a ROS map_server YAML/PGM pair without ROS Toolbox.

    if ~isfile(yaml_path)
        error('load_ros_map:MissingYaml', 'Map YAML does not exist: %s', yaml_path);
    end
    yaml_text = fileread(yaml_path);
    metadata.image = yaml_string(yaml_text, 'image');
    metadata.resolution = yaml_number(yaml_text, 'resolution');
    metadata.origin = yaml_vector(yaml_text, 'origin');
    if abs(metadata.origin(3)) > 1e-9
        error('load_ros_map:RotatedOriginUnsupported', ...
            'A rotated ROS map origin is not supported by the current grid planner.');
    end
    metadata.negate = yaml_number(yaml_text, 'negate', 0);
    metadata.occupied_thresh = yaml_number(yaml_text, 'occupied_thresh', 0.65);
    metadata.free_thresh = yaml_number(yaml_text, 'free_thresh', 0.196);

    image_path = metadata.image;
    if ~isfile(image_path)
        image_path = fullfile(fileparts(yaml_path), image_path);
    end
    if ~isfile(image_path)
        error('load_ros_map:MissingImage', 'Map image does not exist: %s', image_path);
    end
    pixels = double(imread(image_path));
    if ndims(pixels) == 3
        pixels = mean(pixels, 3);
    end
    scale = max(pixels(:));
    if scale <= 0, scale = 255; end
    pixels = pixels / scale;
    if metadata.negate
        occupancy = pixels;
    else
        occupancy = 1 - pixels;
    end

    grid_map = ones(size(occupancy));
    grid_map(occupancy < metadata.free_thresh) = 0;
    grid_map(occupancy > metadata.occupied_thresh) = 1;

    map_info.rows = size(grid_map, 1);
    map_info.cols = size(grid_map, 2);
    map_info.resolution = metadata.resolution;
    map_info.width_m = map_info.cols * metadata.resolution;
    map_info.height_m = map_info.rows * metadata.resolution;
    map_info.origin_xy = metadata.origin(1:2);
    map_info.origin_yaw_rad = metadata.origin(3);
end

function value = yaml_string(text, key)
    token = regexp(text, ['(?m)^\s*', key, '\s*:\s*([^#\r\n]+)'], 'tokens', 'once');
    if isempty(token), error('load_ros_map:MissingKey', 'Missing YAML key: %s', key); end
    value = strtrim(token{1});
    if numel(value) >= 2 && ismember(value(1), ['"', '''']) && value(end) == value(1)
        value = value(2:end-1);
    end
end

function value = yaml_number(text, key, default_value)
    if nargin < 3, default_value = []; end
    token = regexp(text, ['(?m)^\s*', key, '\s*:\s*([-+0-9.eE]+)'], 'tokens', 'once');
    if isempty(token)
        if isempty(default_value), error('load_ros_map:MissingKey', 'Missing YAML key: %s', key); end
        value = default_value;
    else
        value = str2double(token{1});
    end
end

function value = yaml_vector(text, key)
    token = regexp(text, ['(?m)^\s*', key, '\s*:\s*\[([^\]]+)\]'], 'tokens', 'once');
    if isempty(token), error('load_ros_map:MissingKey', 'Missing YAML vector: %s', key); end
    values = textscan(token{1}, '%f', 'Delimiter', ',');
    value = values{1}';
    if numel(value) ~= 3
        error('load_ros_map:InvalidOrigin', 'origin must contain [x, y, yaw].');
    end
end
