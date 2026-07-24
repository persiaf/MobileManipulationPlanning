function log_data = import_ros_execution_log(log_path)
%IMPORT_ROS_EXECUTION_LOG Import JSON or CSV execution feedback.

    if ~isfile(log_path)
        error('import_ros_execution_log:MissingFile', 'Execution log does not exist: %s', log_path);
    end
    [~, ~, ext] = fileparts(log_path);
    switch lower(ext)
        case '.json'
            log_data = jsondecode(fileread(log_path));
        case '.csv'
            log_data = table2struct(readtable(log_path));
        otherwise
            error('import_ros_execution_log:UnsupportedFormat', ...
                'Expected a .json or .csv execution log.');
    end
end
