function data = exp_real_multi_docking(execution_log_path)
%EXP_REAL_MULTI_DOCKING Load a measured multi-docking execution log.
    if nargin < 1 || isempty(execution_log_path)
        error('exp_real_multi_docking:MissingInput', ...
            'Provide a measured multi-docking execution log.');
    end
    data = import_ros_execution_log(execution_log_path);
end
