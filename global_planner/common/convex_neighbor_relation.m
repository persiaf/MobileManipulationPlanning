function [is_neighbor, segment_cells] = convex_neighbor_relation(p1, p2, traversable)
%CONVEX_NEIGHBOR_RELATION Test convex A* neighbor relation.
%   Two nodes are neighbors if they can be connected by at most one diagonal
%   run and at most one horizontal/vertical run without crossing obstacles.

    [ok_a, seg_a] = build_convex_segment(p1, p2, traversable, 'diag_first');
    if ok_a
        is_neighbor = true;
        segment_cells = seg_a;
        return;
    end

    [ok_b, seg_b] = build_convex_segment(p1, p2, traversable, 'straight_first');
    if ok_b
        is_neighbor = true;
        segment_cells = seg_b;
        return;
    end

    is_neighbor = false;
    segment_cells = [];
end
