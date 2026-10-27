function corners = oriented_rectangle_corners(center, theta, length_m, width_m)
%ORIENTED_RECTANGLE_CORNERS Return four world-frame corners as an N-by-2 matrix.

    local = [-length_m/2, -width_m/2; length_m/2, -width_m/2; ...
        length_m/2, width_m/2; -length_m/2, width_m/2];
    corners = (rot2(theta) * local')' + center(1:2);
end
