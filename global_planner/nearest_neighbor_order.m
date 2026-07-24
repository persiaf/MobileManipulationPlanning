function visit_order = nearest_neighbor_order(distance_matrix, start_idx, enable_2opt)
    n = size(distance_matrix, 1);
    if nargin < 3
        enable_2opt = false;
    end
    if nargin < 2
        start_idx = 1;
    end

    unvisited = true(1, n);
    unvisited(start_idx) = false;
    visit_order = start_idx;

    current = start_idx;
    while any(unvisited)
        unvisited_indices = find(unvisited);
        [~, min_loc] = min(distance_matrix(current, unvisited_indices));
        next = unvisited_indices(min_loc);
        visit_order = [visit_order, next];
        unvisited(next) = false;
        current = next;
    end

    if enable_2opt && n > 3
        visit_order = two_opt_order(distance_matrix, visit_order);
    end
end