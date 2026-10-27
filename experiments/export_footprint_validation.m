function report = export_footprint_validation(result, output_dir)
%EXPORT_FOOTPRINT_VALIDATION Export one row per waypoint using the same path.
    if nargin < 2 || isempty(output_dir)
        output_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
            'results','cafeteria','footprint_validation');
    end
    if ~isfield(result,'global_path') || isempty(result.global_path)
        error('export_footprint_validation:Path','result.global_path is empty.');
    end
    if ~isfolder(output_dir), mkdir(output_dir); end
    fp = struct('type','rectangle','length_m',result.robot_cfg.chassis.length, ...
        'width_m',result.robot_cfg.chassis.width);
    path = result.global_path; n=size(path,1);
    rows = repmat(struct('waypoint_id',0,'x',0,'y',0,'theta',0, ...
        'distance_to_obstacle',NaN,'collision_flag',false), n, 1);
    for k=1:n
        pose=path(k,1:min(3,size(path,2))); if numel(pose)<3, pose(3)=0; end
        [flag,distance] = check_robot_footprint_collision(pose,fp,result.scene);
        rows(k)=struct('waypoint_id',k,'x',pose(1),'y',pose(2),'theta',pose(3), ...
            'distance_to_obstacle',distance,'collision_flag',flag);
    end
    report=struct2table(rows);
    writetable(report,fullfile(output_dir,'footprint_collision_report.csv'));
end
