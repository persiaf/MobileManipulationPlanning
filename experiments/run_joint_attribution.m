function run_joint_attribution(layout_id)
%RUN_JOINT_ATTRIBUTION Navigation-only replay of immutable signed IK libraries.
    if nargin<1,layout_id=0;end
    maxNumCompThreads(1);source=fullfile('results','task_navigation_robustness_20261111');
    output=fullfile('results','joint_attribution_20261113');modes={'position_only','top_down_grasp','table_wipe'};
    methods={'fixed_optimum','legacy_strong_init','balanced_raw_init','balanced_strong_init'};
    for distribution=0:2
        case_id=sprintf('L%d_D%d',layout_id,distribution);src=fullfile(source,'cases',case_id);dest=fullfile(output,'cases',case_id);
        loaded=load(fullfile(src,'main_setup.mat'),'setup');setup=loaded.setup;
        poses=[setup.scene.start_pose;reshape([setup.pool.pose],3,[])'];n=size(poses,1);nav=setup.planner;nav.robot_cfg=setup.robot;
        memo=containers.Map('KeyType','char','ValueType','double');edges=readtable(fullfile(src,'navigation_edges.csv'));
        for e=1:height(edges),memo(sprintf('%d>%d',edges.from_node(e),edges.to_node(e)))=edges.navigation_cost(e);end
        common_keys=keys(memo);common_values=values(memo);
        new_edges=zeros(0,5);summary=cell(0,18);paths=cell(0,6);dock_rows=cell(0,10);assignments=cell(0,6);events={};
        old_docks=readtable(fullfile(src,'docking_poses.csv'),'TextType','string');
        for mi=1:3
            mode=modes{mi};stored=load(fullfile(src,['candidate_library_' mode '.mat']),'library','scene');
            library=stored.library;scene=stored.scene;
            validate_candidate_library(library,setup.regions,scene,setup.robot,setup.planner);
            fixed=jsondecode(fileread(fullfile(dest,['fixed_' mode '.json'])));
            original=readtable(fullfile(src,['seed_navigation_costs_' mode '_staged.csv']));
            D=inf(n);known=logical(eye(n));D(1:n+1:end)=0;
            for e=1:height(original)
                a=original.from_node(e);b=original.to_node(e);D(a,b)=original.navigation_cost(e);known(a,b)=true;
            end
            rows=old_docks(old_docks.mode==mode & old_docks.method=="minimum_seed",:);rows=sortrows(rows,'visit_rank');
            raw_route=[1;rows.candidate_id+1]';strong_route=reshape(fixed.route_nodes,1,[]);
            for method_id=1:4
                method=methods{method_id};fprintf('ATTRIBUTION %s %s %s\n',case_id,mode,method);
                memo=containers.Map(common_keys,common_values);
                nav.runtime.shared_path_cost_context=SharedPathCostContext(scene,nav);
                opts=struct('control','joint_local','initial_candidate_indices',strong_route(2:end)-1, ...
                    'initial_matrix_visit_order',strong_route,'cost_matrix',D,'known_edge_mask',known, ...
                    'edge_cost_callback',@oracle,'return_cost_cache',true,'max_passes',3, ...
                    'max_navigation_queries',1500,'max_neighbor_evaluations',10000,'max_wall_time_s',120);
                t=tic;
                if method_id==1
                    opts.control='minimum_seed';plan=plan_task_navigation_joint_local(library,setup.regions,scene,setup.robot,setup.planner,opts);
                elseif method_id==2
                    plan=plan_task_navigation_joint_local(library,setup.regions,scene,setup.robot,setup.planner,opts);
                else
                    if method_id==3,opts.initial_matrix_visit_order=raw_route;opts.initial_candidate_indices=raw_route(2:end)-1;end
                    plan=plan_task_navigation_joint_budgeted(library,setup.regions,scene,setup.robot,setup.planner,opts);
                end
                elapsed=toc(t);j=plan.joint_local;route=j.matrix_visit_order;closed=[route,1];P=zeros(0,3);L=0;
                for ei=1:numel(route)
                    a=closed(ei);b=closed(ei+1);
                    [segment,distance,ok]=query_path_with_shared_cache(scene.grid_map,poses(a,:),poses(b,:),scene.map_info,nav);
                    assert(ok,'JointAttribution:MissingFinalPath');L=L+distance;
                    assert(norm(segment(1,1:2)-poses(a,1:2))<1e-7 && norm(segment(end,1:2)-poses(b,1:2))<1e-7);
                    assert(abs(wrapToPiLocal(segment(1,3)-poses(a,3)))<1e-7 && abs(wrapToPiLocal(segment(end,3)-poses(b,3)))<1e-7);
                    expected=distance+nav.edge_cost.lambda_theta*abs(wrapToPiLocal(poses(b,3)-poses(a,3)));
                    assert(abs(expected-oracle(a,b))<1e-7,'JointAttribution:ChangedOracle');
                    if isempty(P),P=segment;else,P=[P;segment(2:end,:)];end %#ok<AGROW>
                end
                [valid,~,report]=validate_navigation_trajectory(P,scene.grid_map,scene.map_info,nav);assert(valid,report.reason);
                J=L+nav.edge_cost.lambda_theta*sum(abs(wrapToPiLocal(diff(poses(closed,3)))));
                assert(abs(J-j.route_cost)<1e-7,'JointAttribution:ChangedCost');
                assert(numel(plan.covered_tasks_predicted)==36 && numel(route)-1==j.minimum_dock_count);
                for k=1:size(P,1),paths(end+1,:)={mode,method,k,P(k,1),P(k,2),P(k,3)};end %#ok<AGROW>
                for di=1:numel(j.candidate_indices)
                    ci=j.candidate_indices(di);info=plan.task_sequence_summary{di};rank=find(route==ci+1)-1;
                    dock_rows(end+1,:)={mode,method,di,ci,poses(ci+1,1),poses(ci+1,2),poses(ci+1,3), ...
                        strjoin(string(info.covered_tasks),'|'),rank,setup.pool(ci).table_id}; %#ok<AGROW>
                    for tid=info.covered_tasks
                        assignments(end+1,:)={mode,method,tid,di,ci,info.arm_assign_map(tid)}; %#ok<AGROW>
                    end
                end
                order_converged=false;if isfield(j,'fixed_set_order_converged'),order_converged=j.fixed_set_order_converged;end
                init_time=0;if method_id~=3,init_time=fixed.elapsed_s;end
                summary(end+1,:)={case_id,mode,method,J,L,numel(route)-1,j.minimum_dock_count,36,valid, ...
                    j.oracle_queries,j.neighbor_evaluations,elapsed,init_time,fixed.dense_directed_edges, ...
                    numel(j.accepted_moves),order_converged,j.termination_reason,'frozen_task_support_replay_no_new_IK'}; %#ok<AGROW>
                if isfield(j,'cost_matrix'),j=rmfield(j,{'cost_matrix','known_edge_mask'});end
                events{end+1}=struct('case_id',case_id,'mode',mode,'method',method,'search',j); %#ok<AGROW>
                write_outputs();
                fprintf('ATTRIBUTION RESULT J=%.6f requests=%d valid=%d stop=%s\n',J,j.oracle_queries,valid,j.termination_reason);
            end
            % Old joint selected-set diagnostic is never fed into the new runs.
            rows=old_docks(old_docks.mode==mode & old_docks.method=="joint_local",:);rows=sortrows(rows,'visit_rank');
            nodes=[1;rows.candidate_id+1]';matrix=zeros(numel(nodes));
            for a=1:numel(nodes),for b=1:numel(nodes),matrix(a,b)=oracle(nodes(a),nodes(b));end,end
            [a,b]=ndgrid(1:numel(nodes),1:numel(nodes));
            T=table(reshape(nodes(a),[],1),reshape(nodes(b),[],1),matrix(:), ...
                'VariableNames',{'from_node','to_node','navigation_cost'});
            writetable(T,fullfile(dest,['old_joint_graph_' mode '.csv']));
        end
        write_outputs();write_json(fullfile(dest,'run_state.json'),struct('status','COMPLETE','rows',size(summary,1)));
    end
    function value=oracle(a,b)
        if a==b,value=0;return;end
        key=sprintf('%d>%d',a,b);if isKey(memo,key),value=memo(key);return;end
        clock=tic;[~,distance,ok]=query_path_with_shared_cache(setup.scene.grid_map,poses(a,:),poses(b,:),setup.scene.map_info,nav);
        value=Inf;if ok&&isfinite(distance)&&distance<=1e6,value=distance+nav.edge_cost.lambda_theta*abs(wrapToPiLocal(poses(b,3)-poses(a,3)));end
        memo(key)=value;new_edges(end+1,:)=[a,b,distance,value,toc(clock)];
    end
    function write_outputs()
        writetable(cell2table(summary,'VariableNames',{'case_id','mode','method','navigation_cost','route_length_m', ...
            'docks','minimum_docks','covered_tasks','route_valid','new_directed_requests','neighbor_evaluations', ...
            'search_time_s','shared_exact_init_time_s','shared_init_directed_edges','accepted_moves', ...
            'fixed_set_order_converged','termination_reason','coverage_source'}),fullfile(dest,'summary.csv'));
        writetable(cell2table(paths,'VariableNames',{'mode','method','point_id','x_m','y_m','yaw_rad'}),fullfile(dest,'navigation_paths.csv'));
        writetable(cell2table(dock_rows,'VariableNames',{'mode','method','dock_id','candidate_id','x_m','y_m','yaw_rad','covered_task_ids','visit_rank','table_id'}),fullfile(dest,'docking_poses.csv'));
        writetable(cell2table(assignments,'VariableNames',{'mode','method','task_id','dock_id','candidate_id','arm'}),fullfile(dest,'task_assignments.csv'));
        writetable(array2table(new_edges,'VariableNames',{'from_node','to_node','path_length_m','navigation_cost','query_time_s'}),fullfile(dest,'new_navigation_edges.csv'));
        write_json(fullfile(dest,'search_events.json'),events);
    end
end

function write_json(path,value)
    fid=fopen(path,'w','n','UTF-8');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
