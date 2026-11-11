function task=comparison_manipulation_task(anchor,mode,matched_path)
%COMPARISON_MANIPULATION_TASK Frozen task protocol, independent of outcomes.
    if nargin<3,matched_path=false;end
    task=anchor;task.task_type=mode;task.manipulation=struct();
    p=[task.x,task.y,task.z];
    path=[p+[-.06,0,0];p+[.06,0,0]];
    if strcmp(mode,'table_wipe'),task.manipulation.waypoints_world=path;end
    if matched_path
        % Same path, TCP, error tolerances and continuity rule. Only the
        % allowed orientation changes: none -> downward axis -> full pose.
        cfg=manipulation_task_profile(mode);
        task.task_type='position_only';
        task.manipulation=struct('orientation_mode',cfg.orientation_mode, ...
            'orientation_tolerance_rad',deg2rad(5), ...
            'approach_distance_m',0,'waypoints_world',path);
    end
end
