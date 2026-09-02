function cfg = generate_table_layout(cfg)
%GENERATE_TABLE_LAYOUT Populate table poses from cfg.layout.
%
%   Supported patterns:
%       grid          - deterministic regular grid
%       jittered_grid - grid plus bounded xy perturbation
%       cluster       - separated workspace clusters

    if ~isfield(cfg, 'layout') || ~isfield(cfg.layout, 'pattern')
        error('generate_table_layout:MissingPattern', ...
              'cfg.layout.pattern is required.');
    end

    pattern = lower(cfg.layout.pattern);
    region_ids = [];
    switch pattern
        case 'grid'
            positions = grid_positions(cfg);
        case 'jittered_grid'
            positions = jittered_grid_positions(cfg);
        case 'cluster'
            [positions, region_ids] = cluster_positions(cfg);
        otherwise
            error('generate_table_layout:UnknownPattern', ...
                  'Unknown layout pattern "%s".', cfg.layout.pattern);
    end

    cfg.table_mode = 'custom';
    cfg.table_positions = positions;
    cfg.num_tables = size(positions, 1);
    cfg.table_orientations = generate_table_orientations(cfg);
    if isempty(region_ids)
        cfg.table_region_id = (1:cfg.num_tables)';
    else
        cfg.table_region_id = region_ids(:);
    end
end

function positions = grid_positions(cfg)
    rows = get_layout_value(cfg, 'rows', []);
    cols = get_layout_value(cfg, 'cols', []);
    if isempty(rows) || isempty(cols)
        n = get_layout_value(cfg, 'num_tables', getfield_default(cfg, 'num_tables', 16));
        cols = ceil(sqrt(n));
        rows = ceil(n / cols);
    end
    n_tables = get_layout_value(cfg, 'num_tables', rows * cols);
    spacing_x = get_layout_value(cfg, 'spacing_x_m', get_layout_value(cfg, 'spacing_m', 2.0));
    spacing_y = get_layout_value(cfg, 'spacing_y_m', get_layout_value(cfg, 'spacing_m', 2.5));
    center = get_layout_value(cfg, 'center', [cfg.map_width_m / 2, cfg.map_height_m / 2]);
    x0 = center(1) - (cols - 1) * spacing_x / 2;
    y0 = center(2) - (rows - 1) * spacing_y / 2;

    positions = zeros(rows * cols, 2);
    idx = 1;
    for r = 1:rows
        for c = 1:cols
            positions(idx, :) = [x0 + (c - 1) * spacing_x, y0 + (r - 1) * spacing_y];
            idx = idx + 1;
        end
    end
    positions = positions(1:n_tables, :);
end

function positions = jittered_grid_positions(cfg)
    positions = grid_positions(cfg);
    jitter = get_layout_value(cfg, 'jitter_range_m', 0.20);
    offsets = (rand(size(positions)) * 2 - 1) * jitter;
    positions = positions + offsets;
end

function [positions, region_ids] = cluster_positions(cfg)
    n_tables = get_layout_value(cfg, 'num_tables', getfield_default(cfg, 'num_tables', 16));
    n_clusters = get_layout_value(cfg, 'cluster_count', 4);
    per_cluster = ceil(n_tables / n_clusters);
    cluster_spacing = get_layout_value(cfg, 'cluster_spacing_m', 3.6);
    local_spacing_x = get_layout_value(cfg, 'cluster_local_spacing_x_m', 1.5);
    local_spacing_y = get_layout_value(cfg, 'cluster_local_spacing_y_m', 1.3);
    jitter = get_layout_value(cfg, 'cluster_jitter_range_m', 0.12);

    cluster_cols = get_layout_value(cfg, 'cluster_cols', ceil(sqrt(n_clusters)));
    cluster_rows = get_layout_value(cfg, 'cluster_rows', ceil(n_clusters / cluster_cols));
    local_cols = ceil(sqrt(per_cluster));
    local_rows = ceil(per_cluster / local_cols);
    center = get_layout_value(cfg, 'center', [cfg.map_width_m / 2, cfg.map_height_m / 2]);
    wall_margin = getfield_default(cfg, 'table_wall_margin_m', 0.4);
    footprint_margin = hypot(cfg.table.length, cfg.table.width) / 2 + wall_margin;
    local_half_x = (local_cols - 1) * local_spacing_x / 2 + footprint_margin;
    local_half_y = (local_rows - 1) * local_spacing_y / 2 + footprint_margin;
    x_min = local_half_x;
    x_max = cfg.map_width_m - local_half_x;
    y_min = local_half_y;
    y_max = cfg.map_height_m - local_half_y;
    if cluster_cols == 1
        cluster_x = center(1);
    else
        desired_x0 = center(1) - (cluster_cols - 1) * cluster_spacing / 2;
        desired_x = desired_x0 + (0:cluster_cols-1) * cluster_spacing;
        cluster_x = min(max(desired_x, x_min), x_max);
        if any(diff(cluster_x) < cluster_spacing * 0.8)
            cluster_x = linspace(x_min, x_max, cluster_cols);
        end
    end
    if cluster_rows == 1
        cluster_y = center(2);
    else
        desired_y0 = center(2) - (cluster_rows - 1) * cluster_spacing / 2;
        desired_y = desired_y0 + (0:cluster_rows-1) * cluster_spacing;
        cluster_y = min(max(desired_y, y_min), y_max);
        if any(diff(cluster_y) < cluster_spacing * 0.8)
            cluster_y = linspace(y_min, y_max, cluster_rows);
        end
    end

    positions = zeros(n_clusters * per_cluster, 2);
    region_ids = zeros(n_clusters * per_cluster, 1);
    idx = 1;
    for k = 1:n_clusters
        cr = floor((k - 1) / cluster_cols) + 1;
        cc = mod(k - 1, cluster_cols) + 1;
        cluster_center = [cluster_x(cc), cluster_y(cr)];
        lx0 = cluster_center(1) - (local_cols - 1) * local_spacing_x / 2;
        ly0 = cluster_center(2) - (local_rows - 1) * local_spacing_y / 2;
        for r = 1:local_rows
            for c = 1:local_cols
                if idx > size(positions, 1), break; end
                pos = [lx0 + (c - 1) * local_spacing_x, ly0 + (r - 1) * local_spacing_y];
                pos = pos + (rand(1, 2) * 2 - 1) * jitter;
                positions(idx, :) = pos;
                region_ids(idx) = k;
                idx = idx + 1;
            end
        end
    end
    positions = positions(1:n_tables, :);
    region_ids = region_ids(1:n_tables);
end

function orientations = generate_table_orientations(cfg)
    n = size(cfg.table_positions, 1);
    mode = get_layout_value(cfg, 'orientation_mode', 'fixed');
    switch lower(mode)
        case 'fixed'
            value = get_layout_value(cfg, 'fixed_orientation_rad', 0);
            orientations = repmat(value, n, 1);
        case 'discrete_random'
            degrees = get_layout_value(cfg, 'orientation_candidates_deg', [0, 30, -30, 45, -45, 90]);
            candidates = deg2rad(degrees(:));
            idx = randi(numel(candidates), n, 1);
            orientations = candidates(idx);
        otherwise
            error('generate_table_layout:UnknownOrientationMode', ...
                  'Unknown orientation mode "%s".', mode);
    end
end

function v = get_layout_value(cfg, name, default_value)
    if isfield(cfg, 'layout') && isfield(cfg.layout, name)
        v = cfg.layout.(name);
    else
        v = default_value;
    end
end

function v = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end
