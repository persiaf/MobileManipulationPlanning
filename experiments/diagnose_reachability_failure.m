function report = diagnose_reachability_failure(scene, library, robot_cfg, planner_cfg)
%DIAGNOSE_REACHABILITY_FAILURE Read-only per-task reachability audit.
    n=scene.num_tasks; report=repmat(struct('task_id',0,'table_id',0,'total_safe_candidates',0,'pass_z_count',0,'pass_radial_count',0,'pass_forward_count',0,'pass_lateral_count',0,'pass_all_geometric_count',0,'pass_rho_count',0,'min_radial_distance',Inf,'max_raw_score',-Inf,'best_relative_z',NaN,'best_u',NaN,'best_v',NaN,'best_score',0,'primary_failure_reason',''),n,1); rho=planner_cfg.reachability.threshold;
    for t=1:n
        report(t).task_id=t; report(t).table_id=scene.tasks(t).table_id;
        for ri=1:numel(library.regions)
            if library.regions(ri).table_id~=report(t).table_id, continue; end
            cs=library.regions(ri).candidates;
            for ci=1:numel(cs)
                p=cs(ci).pose; report(t).total_safe_candidates=report(t).total_safe_candidates+1; task=[scene.tasks(t).x scene.tasks(t).y scene.tasks(t).z];
                for side={'left','right'}
                    if strcmp(side{1},'left'), off=[robot_cfg.arm_base_mount_x robot_cfg.dual_arm.base_spacing/2]; else, off=[robot_cfg.arm_base_mount_x -robot_cfg.dual_arm.base_spacing/2]; end
                    ab=[p(1);p(2)]+rot2(p(3))*off'; dv=rot2(-p(3))*([task(1);task(2)]-ab); d=norm(dv); dz=task(3)-robot_cfg.arm_base_mount_z; zok=dz>=robot_cfg.single_arm.reach_z_min && dz<=robot_cfg.single_arm.reach_z_max; rok=d>=.01 && d<=robot_cfg.single_arm.reach_radius; fwd=dv(1)>=-robot_cfg.single_arm.reach_backward; lat=abs(dv(2))<=robot_cfg.single_arm.reach_lateral*1.1; raw=reachability_raw_score(d,dz,dv,robot_cfg);
                    report(t).min_radial_distance=min(report(t).min_radial_distance,d); report(t).max_raw_score=max(report(t).max_raw_score,raw); report(t).pass_z_count=report(t).pass_z_count+zok; report(t).pass_radial_count=report(t).pass_radial_count+rok; report(t).pass_forward_count=report(t).pass_forward_count+fwd; report(t).pass_lateral_count=report(t).pass_lateral_count+lat; report(t).pass_all_geometric_count=report(t).pass_all_geometric_count+(zok&&rok&&fwd&&lat); report(t).pass_rho_count=report(t).pass_rho_count+(raw>=rho);
                end
            end
        end
        report(t).best_score=report(t).max_raw_score; report(t).best_relative_z=scene.tasks(t).z-robot_cfg.arm_base_mount_z; if report(t).pass_z_count==0, report(t).primary_failure_reason='VERTICAL_REACH_LIMIT'; elseif report(t).pass_radial_count==0, report(t).primary_failure_reason='HORIZONTAL_RADIAL_REACH_LIMIT'; elseif report(t).pass_rho_count==0, report(t).primary_failure_reason='SCORE_THRESHOLD'; else, report(t).primary_failure_reason='OTHER_GEOMETRIC_LIMIT'; end
    end
end
function s=reachability_raw_score(d,dz,dv,r), R=r.single_arm.reach_radius; z1=r.single_arm.reach_z_min; z2=r.single_arm.reach_z_max; s=max(0,min(1,.30*max(0,1-d/R)+.25*max(0,1-abs(d-(.01+R)/2)/((R-.01)/2))+.20*max(0,1-abs(dz-(z1+z2)/2)/((z2-z1)/2))+.15*max(0,min(1,dv(1)/r.single_arm.reach_forward))+.10*(1-abs(dv(2))/r.single_arm.reach_lateral))); end
