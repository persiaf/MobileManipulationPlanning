function report = validate_tabletop_tasks(scene_cfg, tasks)
%VALIDATE_TABLETOP_TASKS Check declared table ownership in the table frame.
%   Tabletop task positions are checked against geometry, not the inflated
%   navigation occupancy map. A valid task is normally inside occupied cells.
    n = size(scene_cfg.table_positions,1);
    report = struct('task_count',numel(tasks),'valid',true);
    for k = 1:numel(tasks)
        task = tasks(k);
        tid = task.table_id;
        if ~isscalar(tid) || ~isfinite(tid) || tid ~= fix(tid) || tid < 1 || tid > n
            error('validate_tabletop_tasks:InvalidTableId','Task %d has invalid table ownership.',k);
        end
        p = [task.x task.y task.z];
        if any(~isfinite(p))
            error('validate_tabletop_tasks:InvalidPosition','Task %d has non-finite coordinates.',k);
        end
        local = rot2(-scene_cfg.table_orientations(tid)) * ...
            (p(1:2)-scene_cfg.table_positions(tid,:))';
        if abs(local(1)) > scene_cfg.table.length/2 + 1e-9 || ...
                abs(local(2)) > scene_cfg.table.width/2 + 1e-9
            error('validate_tabletop_tasks:OutsideTable', ...
                'Task %d is outside declared table %d: local [%.6f %.6f] m.',k,tid,local(1),local(2));
        end
    end
end
