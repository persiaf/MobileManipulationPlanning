function world_xy = table_local_to_world(table_center, table_theta, local_xy)
%TABLE_LOCAL_TO_WORLD Transform table-local points using the canonical rotation.

    world_xy = (rot2(table_theta) * local_xy')' + table_center(1:2);
end
