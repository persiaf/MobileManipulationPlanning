function data = exp_real_multi_height(execution_log_path, lift_calibration_path)
%EXP_REAL_MULTI_HEIGHT Load measured execution and lift calibration data.
    if nargin < 2 || isempty(execution_log_path) || isempty(lift_calibration_path)
        error('exp_real_multi_height:MissingInput', ...
            'Provide measured execution-log and lift-calibration paths.');
    end
    data.execution = import_ros_execution_log(execution_log_path);
    data.lift = load_lift_calibration(lift_calibration_path);
end
