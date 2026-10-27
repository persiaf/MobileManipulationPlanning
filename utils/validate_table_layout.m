function [is_valid, report] = validate_table_layout(cfg)
%VALIDATE_TABLE_LAYOUT Structured validity checks for table layouts.

    report = struct('is_valid', true, 'failure_reason', '', ...
                    'failures', {{}}, 'min_pair_clearance_m', inf);
    failures = {};

    positions = cfg.table_positions;
    orientations = cfg.table_orientations(:);
    n = size(positions, 1);
    if numel(orientations) < n
        orientations(end + 1:n) = 0;
    end

    if any(~isfinite(positions(:)))
        failures{end + 1} = 'invalid_position'; %#ok<AGROW>
    end
    if any(~isfinite(orientations))
        failures{end + 1} = 'invalid_orientation'; %#ok<AGROW>
    end
    if size(unique(round(positions, 9), 'rows'), 1) < n
        failures{end + 1} = 'duplicate_placement'; %#ok<AGROW>
    end

    min_aisle = getfield_default(cfg, 'table_min_aisle_clearance_m', ...
        getfield_default(cfg, 'table_min_spacing_m', 0) - hypot(cfg.table.length, cfg.table.width));
    min_aisle = max(min_aisle, 0);

    polygons = cell(n, 1);
    for i = 1:n
        polygons{i} = oriented_rectangle_corners(positions(i, :), orientations(i), ...
            cfg.table.length, cfg.table.width);
        if is_outside_map(polygons{i}, cfg)
            failures{end + 1} = sprintf('table_%d_map_boundary_collision', i); %#ok<AGROW>
        end
    end

    for i = 1:n
        for j = i + 1:n
            if polygons_overlap(polygons{i}, polygons{j})
                failures{end + 1} = sprintf('table_%d_table_%d_overlap', i, j); %#ok<AGROW>
            end
            clearance = polygon_clearance(polygons{i}, polygons{j});
            report.min_pair_clearance_m = min(report.min_pair_clearance_m, clearance);
            if clearance < min_aisle
                failures{end + 1} = sprintf('table_%d_table_%d_aisle_clearance_%.3f_lt_%.3f', ...
                                            i, j, clearance, min_aisle); %#ok<AGROW>
            end
        end
    end

    report.failures = unique(failures, 'stable');
    report.is_valid = isempty(report.failures);
    if report.is_valid
        report.failure_reason = '';
    else
        report.failure_reason = strjoin(report.failures, '; ');
    end
    is_valid = report.is_valid;
end

function tf = is_outside_map(poly, cfg)
    margin = getfield_default(cfg, 'table_wall_margin_m', 0);
    tf = any(poly(:, 1) < margin) || any(poly(:, 1) > cfg.map_width_m - margin) || ...
         any(poly(:, 2) < margin) || any(poly(:, 2) > cfg.map_height_m - margin);
end

function tf = polygons_overlap(a, b)
    axes = edge_normals(a);
    axes = [axes; edge_normals(b)];
    tf = true;
    for k = 1:size(axes, 1)
        axis = axes(k, :);
        pa = a * axis';
        pb = b * axis';
        if max(pa) < min(pb) || max(pb) < min(pa)
            tf = false;
            return;
        end
    end
end

function axes = edge_normals(poly)
    next = poly([2:end, 1], :);
    edges = next - poly;
    axes = [-edges(:, 2), edges(:, 1)];
    lens = sqrt(sum(axes.^2, 2));
    axes = axes ./ max(lens, eps);
end

function d = polygon_clearance(a, b)
    if polygons_overlap(a, b)
        d = 0;
        return;
    end
    d = inf;
    for i = 1:size(a, 1)
        d = min(d, point_polygon_distance(a(i, :), b));
    end
    for i = 1:size(b, 1)
        d = min(d, point_polygon_distance(b(i, :), a));
    end
end

function d = point_polygon_distance(p, poly)
    d = inf;
    next = poly([2:end, 1], :);
    for i = 1:size(poly, 1)
        d = min(d, point_segment_distance(p, poly(i, :), next(i, :)));
    end
end

function d = point_segment_distance(p, a, b)
    ab = b - a;
    if all(abs(ab) < eps)
        d = norm(p - a);
        return;
    end
    t = dot(p - a, ab) / dot(ab, ab);
    t = max(0, min(1, t));
    q = a + t * ab;
    d = norm(p - q);
end

function v = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end
