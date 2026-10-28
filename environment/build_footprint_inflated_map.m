function [inflated_map, inflation_info] = build_footprint_inflated_map(obstacle_map, robot_footprint, map_info)
%BUILD_FOOTPRINT_INFLATED_MAP Build a conservative footprint-safe search map.
%   Search dilation includes the physical circumscribed radius plus the
%   occupied cell's half diagonal. Free grid centres can therefore rotate
%   safely against finite occupied squares. The physical radius metadata and
%   robot dimensions remain unchanged. Raw-map trajectory checks are still
%   required for continuous motion and exact endpoint connectors.

    if nargin < 3 || isempty(map_info)
        error('build_footprint_inflated_map:Arguments', ...
            'obstacle_map, robot_footprint, and map_info are required.');
    end
    if isstruct(obstacle_map) && isfield(obstacle_map, 'grid_map')
        obstacle_map = obstacle_map.grid_map;
    end
    if ~ismatrix(obstacle_map) || ~isnumeric(obstacle_map)
        error('build_footprint_inflated_map:Map', 'obstacle_map must be a numeric 2-D grid.');
    end
    [length_m, width_m, model] = footprint_dimensions(robot_footprint);
    if ~strcmp(model, 'rectangle')
        error('build_footprint_inflated_map:Model', ...
            'Only the configured rectangular footprint is supported.');
    end
    % A heading-independent 2-D layer must contain the rectangle for every
    % base heading.  Its conservative radius is the half diagonal.
    radius_m = 0.5 * hypot(length_m, width_m);
    radius_cells = ceil(radius_m / map_info.resolution);
    cell_margin_m = sqrt(2) * map_info.resolution / 2;
    search_radius_m = radius_m + cell_margin_m;
    search_radius_cells = ceil(search_radius_m / map_info.resolution);
    occupied = obstacle_map > 0;
    inflated_occupied = occupied;
    [rows, cols] = size(occupied);
    for di = -search_radius_cells:search_radius_cells
        for dj = -search_radius_cells:search_radius_cells
            % Compare in metres. Comparing against radius_cells directly
            % over-inflates diagonal offsets because hypot(di,dj) is in
            % cells while radius_cells is an axis-aligned cell count.
            if hypot(di, dj) * map_info.resolution > search_radius_m + 1e-12, continue; end
            src_i = max(1, 1-di):min(rows, rows-di);
            src_j = max(1, 1-dj):min(cols, cols-dj);
            dst_i = src_i + di;
            dst_j = src_j + dj;
            inflated_occupied(dst_i, dst_j) = inflated_occupied(dst_i, dst_j) | ...
                occupied(src_i, src_j);
        end
    end
    % No obstacle pixel is needed to represent the outside of the map. A
    % free grid centre must also have room to rotate within its physical box.
    map_width_m=field_or(map_info,'width_m',cols*map_info.resolution);
    map_height_m=field_or(map_info,'height_m',rows*map_info.resolution);
    x=((1:cols)-0.5)*map_info.resolution;
    y=map_height_m-((1:rows)'-0.5)*map_info.resolution;
    boundary_blocked=(min(x,map_width_m-x)<=radius_m+1e-12) | ...
        (min(y,map_height_m-y)<=radius_m+1e-12);
    inflated_occupied=inflated_occupied | boundary_blocked;
    inflated_map = zeros(size(obstacle_map), 'like', obstacle_map);
    inflated_map(inflated_occupied) = 1;
    % Preserve obstacle classes where the original map already had them.
    inflated_map(occupied) = obstacle_map(occupied);
    inflation_info = struct('model', model, 'length_m', length_m, ...
        'width_m', width_m, 'radius_m', radius_m, ...
        'radius_cells', radius_cells, 'resolution_m', map_info.resolution, ...
        'cell_discretization_margin_m',cell_margin_m,'search_radius_m',search_radius_m, ...
        'search_radius_cells',search_radius_cells,'boundary_clearance_radius_m',radius_m);
end

function [length_m, width_m, model] = footprint_dimensions(fp)
    if isstruct(fp) && isfield(fp, 'chassis'), fp = fp.chassis; end
    if isstruct(fp) && isfield(fp, 'footprint'), fp = fp.footprint; end
    if isnumeric(fp) && numel(fp) >= 2
        length_m = fp(1); width_m = fp(2); model = 'rectangle'; return;
    end
    if ~isstruct(fp), error('build_footprint_inflated_map:Footprint', 'Invalid footprint.'); end
    length_m = field_or(fp, 'length_m', field_or(fp, 'length', NaN));
    width_m = field_or(fp, 'width_m', field_or(fp, 'width', NaN));
    model = lower(char(field_or(fp, 'model', field_or(fp, 'type', 'rectangle'))));
    if ~all(isfinite([length_m width_m])) || any([length_m width_m] <= 0)
        error('build_footprint_inflated_map:Footprint', 'Positive length and width are required.');
    end
end

function value = field_or(s, name, fallback)
    if isfield(s, name), value = s.(name); else, value = fallback; end
end
