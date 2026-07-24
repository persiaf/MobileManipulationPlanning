function data = load_calibration_file(file_path, required_fields)
%LOAD_CALIBRATION_FILE Load a measured MAT/JSON calibration artifact.

    if ~isfile(file_path)
        error('load_calibration_file:MissingFile', ...
            'Calibration file does not exist: %s', file_path);
    end
    [~, ~, ext] = fileparts(file_path);
    switch lower(ext)
        case '.mat'
            loaded = load(file_path);
            names = fieldnames(loaded);
            if numel(names) == 1 && isstruct(loaded.(names{1}))
                data = loaded.(names{1});
            else
                data = loaded;
            end
        case '.json'
            data = jsondecode(fileread(file_path));
        otherwise
            error('load_calibration_file:UnsupportedFormat', ...
                'Calibration files must be .mat or .json.');
    end
    for k = 1:numel(required_fields)
        if ~isfield(data, required_fields{k})
            error('load_calibration_file:MissingField', ...
                'Calibration file is missing %s.', required_fields{k});
        end
    end
end
