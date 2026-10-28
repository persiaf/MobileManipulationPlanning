function raw = exp_repair_sampling_weight_sensitivity(options)
%EXP_REPAIR_SAMPLING_WEIGHT_SENSITIVITY Paired one-factor main-robot study.
    if nargin<1,options=struct();end
    root=fileparts(fileparts(mfilename('fullpath'))); init_paths(root);
    out=field_or(options,'output_dir',fullfile(root,'results','repair_20261027','sensitivity'));
    if ~isfolder(out),mkdir(out);end
    seeds=field_or(options,'seeds',[2026 2027 2028]);
    variants={'baseline','lateral_coarse','lateral_fine','normal_coarse','normal_fine', ...
        'yaw_coarse','yaw_fine','grid_coarse','grid_fine', ...
        'cost_reach_half','cost_reach_double','cost_coverage_half','cost_coverage_double', ...
        'selection_coverage_half','selection_coverage_double'};
    variants=field_or(options,'variants',variants); rows=struct([]);
    for seed=seeds
        reference_geometry='';
        for vi=1:numel(variants)
            name=variants{vi}; scene_cfg=repair_study_scene_config('SENS4',seed);
            cfg=planner_config();
            switch name
                case 'baseline'
                case 'lateral_coarse',scene_cfg.docking_lateral_step_m=0.20;
                case 'lateral_fine',scene_cfg.docking_lateral_step_m=0.075;
                case 'normal_coarse',scene_cfg.docking_distance_step_m=0.075;
                case 'normal_fine',scene_cfg.docking_distance_step_m=0.025;
                case 'yaw_coarse',scene_cfg.docking_orientation_step_rad=pi/6;
                case 'yaw_fine',scene_cfg.docking_orientation_step_rad=pi/24;
                case 'grid_coarse',scene_cfg.grid_resolution_m=0.10;
                case 'grid_fine',scene_cfg.grid_resolution_m=0.025;
                case 'cost_reach_half',cfg.cost_weights.w_reach=cfg.cost_weights.w_reach/2;
                case 'cost_reach_double',cfg.cost_weights.w_reach=cfg.cost_weights.w_reach*2;
                case 'cost_coverage_half',cfg.cost_weights.w_coverage=cfg.cost_weights.w_coverage/2;
                case 'cost_coverage_double',cfg.cost_weights.w_coverage=cfg.cost_weights.w_coverage*2;
                case 'selection_coverage_half',cfg.strategy.bottom_up.weights.coverage_gain=cfg.strategy.bottom_up.weights.coverage_gain/2;
                case 'selection_coverage_double',cfg.strategy.bottom_up.weights.coverage_gain=cfg.strategy.bottom_up.weights.coverage_gain*2;
                otherwise,error('exp_repair_sampling_weight_sensitivity:Variant','Unknown variant %s.',name);
            end
            [row,state,audit]=run_protocol_planning_case(scene_cfg,cfg,struct('variant',name, ...
                'run_audit',field_or(options,'run_audit',true),'run_routing',field_or(options,'run_routing',true)));
            if isempty(reference_geometry),reference_geometry=row.task_geometry_signature;
            else,assert(strcmp(reference_geometry,row.task_geometry_signature),'Paired variants changed table/task geometry.');end
            if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
            raw=struct2table(rows,'AsArray',true); writetable(raw,fullfile(out,'sensitivity_raw.csv'));
            write_repair_case_artifacts(state,row,audit,fullfile(out,'cases'),sprintf('seed%d_%s',seed,name));
            fprintf('SENSITIVITY seed=%d variant=%s status=%s coverage=%.3f T=%.3f\n', ...
                seed,name,row.status,row.geometric_coverage_percent,row.planning_time_s);
        end
    end
end

function v=field_or(s,n,d)
    if isstruct(s)&&isfield(s,n),v=s.(n);else,v=d;end
end
