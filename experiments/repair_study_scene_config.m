function cfg = repair_study_scene_config(scene_id,seed)
%REPAIR_STUDY_SCENE_CONFIG Explicit full scenes and controlled test subsets.
    id=upper(char(scene_id));
    switch id
        case 'MAIN'
            cfg=scene_config(); cfg.random_seed=seed;
        case 'MAIN4'
            cfg=scene_config(); cfg.random_seed=seed;
            cfg.map_width_m=8; cfg.map_height_m=8;
            cfg.table_positions=[2.2 2.2;5.6 2.2;2.2 5.6;5.6 5.6];
            cfg.table_orientations=zeros(4,1); cfg.num_tables=4;
            cfg.table_region_id=(1:4)';
            cfg.repair_study_scope='four_table_subset_of_main_task_distribution';
        case 'SENS4'
            cfg=scene_complexity_profile_config('C2',seed);
            cfg.map_width_m=8; cfg.map_height_m=8;
            cfg.layout.num_tables=4; cfg.layout.rows=2; cfg.layout.cols=2;
            cfg.requested_task_count=cfg.layout.num_tables*mean(cfg.tasks_per_table_range);
            cfg.layout.center=[4 4]; cfg.layout.spacing_x_m=3; cfg.layout.spacing_y_m=3;
            cfg.repair_study_scope='four_table_controlled_C2_sensitivity_subset';
        case {'C0','C1','C2','C3'}
            cfg=scene_complexity_profile_config(id,seed);
        case {'L1','L2','L3','V1','V2','V3'}
            cfg=large_scale_profile_config(id,seed);
        otherwise
            error('repair_study_scene_config:UnknownScene','Unknown scene %s.',id);
    end
    cfg.repair_study_id=id;
end
