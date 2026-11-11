function cases=task_navigation_robustness_cases()
%TASK_NAVIGATION_ROBUSTNESS_CASES Predeclared 3-layout x 3-task factorial.
% No IK/navigation outcomes are used to construct or discard a case.
% All cases retain the original table-wise task counts (36 total), model,
% table sizes, map, start, and candidate-generation/selection budgets.
    original_cfg=scene_config();original=build_scene_from_cfg(original_cfg);
    nt=original.num_tasks;table_ids=reshape([original.tasks.table_id],[],1);
    counts=accumarray(table_ids,1,[original_cfg.num_tables,1]);
    original_local=zeros(nt,2);
    for ti=1:nt
        t=table_ids(ti);p=[original.tasks(ti).x,original.tasks(ti).y];
        original_local(ti,:)=(rot2(-original_cfg.table_orientations(t))* ...
            (p-original_cfg.table_positions(t,:))')';
    end
    layouts=cell(1,3);layouts{1}=original_cfg;
    layouts{2}=original_cfg;
    ys=unique(original_cfg.table_positions(:,2),'sorted');
    for ri=1:numel(ys)
        rows=original_cfg.table_positions(:,2)==ys(ri);
        layouts{2}.table_positions(rows,1)=layouts{2}.table_positions(rows,1)+.30*(-1)^ri;
    end
    layouts{3}=original_cfg;
    pattern=[0;pi/6;-pi/6;pi/2];
    layouts{3}.table_orientations=repmat(pattern,ceil(original_cfg.num_tables/4),1);
    layouts{3}.table_orientations=layouts{3}.table_orientations(1:original_cfg.num_tables);
    layout_labels={'Original grid','Alternating-row shift','Mixed table orientations'};
    distribution_labels={'Original task distribution','Same-side clustered tasks','Opposite-side long-span tasks'};
    cases=struct([]);
    for li=1:3
        cfg_layout=layouts{li};reference_cfg=cfg_layout;
        if li>1
            reference_cfg.task_mode='manual';
            reference_cfg.manual_tasks=world_tasks(original_local,table_ids,cfg_layout);
        end
        for di=1:3
            local=original_local;
            if di>1
                for t=1:original_cfg.num_tables
                    rows=find(table_ids==t);n=numel(rows);sign_y=(-1)^t;
                    if di==2
                        points=[-.18,.12*sign_y;.18,.12*sign_y;0,.12*sign_y];
                    else
                        points=[-.45,-.12*sign_y;.45,.12*sign_y;0,-.12*sign_y];
                    end
                    if n>size(points,1)
                        error('task_navigation_robustness_cases:TaskCount','The fixed design expects at most three original tasks per table.');
                    end
                    local(rows,:)=points(1:n,:);
                end
            end
            cfg=cfg_layout;
            if li>1||di>1
                cfg.task_mode='manual';cfg.manual_tasks=world_tasks(local,table_ids,cfg);
            end
            entry=struct('case_id',sprintf('L%d_D%d',li-1,di-1), ...
                'layout_id',li-1,'distribution_id',di-1, ...
                'layout_label',layout_labels{li},'distribution_label',distribution_labels{di}, ...
                'scene_cfg',cfg,'reference_scene_cfg',reference_cfg, ...
                'task_local_xy',local,'original_task_local_xy',original_local, ...
                'task_table_ids',table_ids,'per_table_task_counts',counts, ...
                'task_count',nt,'table_count',original_cfg.num_tables, ...
                'factorial_revision','main_3layouts_3distributions_v1', ...
                'pool_rule','one D0-reference geometry pool per layout, shared by every distribution and task mode', ...
                'task_distribution_rule','original local anchors; alternating-side clusters; opposite-side x=+-0.45,y=+-0.12 with fixed third anchor');
            if isempty(cases),cases=entry;else,cases(end+1)=entry;end %#ok<AGROW>
        end
    end
end

function manual=world_tasks(local,table_ids,cfg)
    manual=zeros(numel(table_ids),4);
    for ti=1:numel(table_ids)
        t=table_ids(ti);p=cfg.table_positions(t,:)+(rot2(cfg.table_orientations(t))*local(ti,:)')';
        manual(ti,:)=[t,p,cfg.task_offset_z];
    end
end
