function data = exp_real_docking_accuracy(execution_log_path)
%EXP_REAL_DOCKING_ACCURACY Load measured docking records for later analysis.
    if nargin < 1
        execution_log_path = '';
    end
    require_input(execution_log_path);
    data = import_ros_execution_log(execution_log_path);
end

function require_input(path_value)
    if nargin < 1 || isempty(path_value)
        error('exp_real_docking_accuracy:MissingInput', ...
            'Provide a measured execution_log.json or execution_log.csv.');
    end
end
