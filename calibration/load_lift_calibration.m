function calibration = load_lift_calibration(file_path)
%LOAD_LIFT_CALIBRATION Load measured commanded/actual lift heights.
    calibration = load_calibration_file(file_path, ...
        {'commanded_heights_m', 'measured_heights_m'});
end
