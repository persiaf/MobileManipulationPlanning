function workspace_map = load_arm_workspace_map(file_path)
%LOAD_ARM_WORKSPACE_MAP Load a measured arm workspace artifact.
    workspace_map = load_calibration_file(file_path, ...
        {'frame_id', 'sample_points_xyz', 'reachable'});
end
