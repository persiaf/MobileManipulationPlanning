function model = load_docking_error_model(file_path)
%LOAD_DOCKING_ERROR_MODEL Load measured docking position/yaw errors.
    model = load_calibration_file(file_path, ...
        {'position_offsets_m', 'yaw_offsets_rad'});
end
