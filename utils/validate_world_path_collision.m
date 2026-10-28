function [ok, bad_idx] = validate_world_path_collision(grid_map, path, map_info, verbose)
    if nargin < 4
        verbose = true;
    end
    ok = true;
    bad_idx = [];

    if isempty(path) || size(path, 1) < 2
        return;
    end

    traversable = (grid_map == 0);
    origin=[0 0];
    if isfield(map_info,'origin_xy'),origin=map_info.origin_xy;end

    for k = 2:size(path, 1)
        p1 = path(k-1, 1:2);
        p2 = path(k, 1:2);

        [gi1, gj1] = world2grid(p1(1), p1(2), map_info.resolution, map_info.height_m,origin);
        [gi2, gj2] = world2grid(p2(1), p2(2), map_info.resolution, map_info.height_m,origin);

        if ~collision_free([gi1, gj1], [gi2, gj2], traversable)
            ok = false;
            bad_idx = [bad_idx; k-1, k]; %#ok<AGROW>
        end
    end

    if ~ok && verbose
        fprintf('  validate_world_path_collision: %d 个碰撞段\n', size(bad_idx, 1));
    end
end
