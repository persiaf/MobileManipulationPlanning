function setup=task_navigation_robustness_setup(case_definition,reference_setup)
%TASK_NAVIGATION_ROBUSTNESS_SETUP Rebind tasks to a shared layout-only pool.
% reference_setup is the D0 reference for this same layout; supplying it
% avoids regenerating its dense field for D1/D2. No task scoring is done.
    if nargin<1||~isstruct(case_definition)||~isscalar(case_definition)|| ...
            ~all(isfield(case_definition,{'scene_cfg','reference_scene_cfg','case_id'}))
        error('task_navigation_robustness_setup:Case','Provide a fixed case definition.');
    end
    reference_cfg=case_definition.reference_scene_cfg;
    if nargin<2||isempty(reference_setup)
        reference_setup=main_task_navigation_setup(reference_cfg);
    else
        if ~isstruct(reference_setup),error('task_navigation_robustness_setup:Reference','Invalid shared pool.');end
        expected=[];
        if isfield(reference_setup,'reference_scene_cfg'),expected=reference_setup.reference_scene_cfg;
        elseif isfield(reference_setup,'source_scene_cfg'),expected=reference_setup.source_scene_cfg;end
        if ~isequaln(expected,reference_cfg)
            error('task_navigation_robustness_setup:Reference','The shared pool must be the D0 reference for this layout.');
        end
    end
    if ~isequaln(layout_geometry(reference_setup.scene.scene_cfg),layout_geometry(reference_cfg))
        error('task_navigation_robustness_setup:Reference','Reference pool geometry differs from the case layout.');
    end
    setup=reference_setup;
    if isequaln(case_definition.scene_cfg,reference_setup.scene.scene_cfg)
        scene=reference_setup.scene;
    else
        scene=build_scene_from_cfg(case_definition.scene_cfg);
    end
    if ~isequaln(layout_geometry(scene.scene_cfg),layout_geometry(reference_cfg))
        error('task_navigation_robustness_setup:Geometry','Task distributions cannot change the shared layout geometry.');
    end
    task_map=assign_tasks_to_tables(scene);regions=build_task_regions(scene,task_map);
    counts=accumarray(reshape([scene.tasks.table_id],[],1),1,[case_definition.table_count,1]);
    if scene.num_tasks~=case_definition.task_count||~isequal(counts,case_definition.per_table_task_counts)
        error('task_navigation_robustness_setup:TaskCount','All original per-table task counts must be preserved.');
    end
    check_table_geometry(scene);
    check_wipe_on_table(scene);
    setup.scene=scene;setup.source_scene_cfg=case_definition.scene_cfg;
    setup.regions=regions;setup.task_map=task_map;
    setup.case_definition=case_definition;setup.reference_scene_cfg=reference_cfg;
    setup.reference_task_anchors=case_definition.original_task_local_xy;
    setup.reference_pool_signature=build_struct_signature(struct('cfg',reference_cfg,'pool',setup.pool));
    setup.geometry_pool_scope='same layout D0-reference dense subpool reused without regeneration for every distribution';
    setup.wipe_paths_inside_table=true;
end

function geometry=layout_geometry(cfg)
    fields={'map_width_m','map_height_m','grid_resolution_m','border_thickness_m', ...
        'table','table_positions','table_orientations','num_tables','start_pose', ...
        'table_min_spacing_m','table_wall_margin_m','table_start_margin_m', ...
        'safety_margin_m','docking_min_distance_m','docking_max_distance_m', ...
        'docking_distance_step_m','docking_lateral_step_m'};
    geometry=struct();
    for k=1:numel(fields),geometry.(fields{k})=cfg.(fields{k});end
    if isfield(cfg,'docking_orientation_step_rad'),geometry.docking_orientation_step_rad=cfg.docking_orientation_step_rad;end
end

function check_table_geometry(scene)
    cfg=scene.scene_cfg;n=cfg.num_tables;polygons=cell(1,n);
    corners=[-.5,-.5;.5,-.5;.5,.5;-.5,.5].*[cfg.table.length,cfg.table.width];
    for t=1:n
        polygons{t}=(rot2(cfg.table_orientations(t))*corners')'+cfg.table_positions(t,:);
        p=polygons{t};m=cfg.table_wall_margin_m;
        if any(p(:,1)<m-1e-12|p(:,1)>cfg.map_width_m-m+1e-12| ...
                p(:,2)<m-1e-12|p(:,2)>cfg.map_height_m-m+1e-12)
            error('task_navigation_robustness_setup:TableBounds','A predefined table violates the wall margin.');
        end
    end
    for a=1:n
        for b=a+1:n
            if polygons_overlap(polygons{a},polygons{b})
                error('task_navigation_robustness_setup:TableOverlap','Predefined table polygons overlap.');
            end
        end
    end
end

function overlap=polygons_overlap(A,B)
    axes=[diff([A;A(1,:)],1,1);diff([B;B(1,:)],1,1)];
    axes=[-axes(:,2),axes(:,1)];overlap=true;
    for i=1:size(axes,1)
        pa=A*axes(i,:)';pb=B*axes(i,:)';
        if max(pa)<=min(pb)+1e-12||max(pb)<=min(pa)+1e-12,overlap=false;return;end
    end
end

function check_wipe_on_table(scene)
    cfg=scene.scene_cfg;
    for ti=1:scene.num_tasks
        task=comparison_manipulation_task(scene.tasks(ti),'table_wipe');
        constraints=resolve_manipulation_task(task);t=task.table_id;
        local=(rot2(-cfg.table_orientations(t))* ...
            (constraints.samples_world(:,1:2)-cfg.table_positions(t,:))')';
        if any(abs(local(:,1))>cfg.table.length/2+1e-10)|| ...
                any(abs(local(:,2))>cfg.table.width/2+1e-10)
            error('task_navigation_robustness_setup:WipeOutsideTable','A fixed world-X wipe sample leaves its own table.');
        end
    end
end
