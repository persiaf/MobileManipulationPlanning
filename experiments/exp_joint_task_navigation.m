function result=exp_joint_task_navigation(output_dir)
%EXP_JOINT_TASK_NAVIGATION Fixed-evidence staged/reorder/joint comparison.
% Cover all tasks -> minimum dock count -> minimum directed navigation cost.
% Reuses the previous frozen IK matrix; recomputes every final base route.
    if nargin<1||isempty(output_dir),output_dir=fullfile('results','joint_task_navigation_20261111');end
    if ~isfolder(output_dir),mkdir(output_dir);end
    data=prepare_joint_task_navigation();graph=build_joint_navigation_graph(data,output_dir);
    modes=data.setup.mode_order;methods={'staged','reorder_only','joint'};
    robot=data.setup.robot;scene0=data.setup.scene;
    metadata=data.source_metadata;metadata.experiment_id='joint_task_navigation_20261111';
    metadata.methods=methods;metadata.source_geometry_signature=data.source_metadata.geometry_signature;
    metadata.source_evidence=data.source_dir;metadata.graph_signature=graph.signature;
    metadata.scene.border_thickness_m=scene0.scene_cfg.border_thickness_m;
    metadata.lambda_theta_m_per_rad=graph.lambda_theta_m_per_rad;
    metadata.navigation_cost_definition='sum actual directed path lengths + lambda_theta * wrapped endpoint yaw changes';
    metadata.optimization_order={'full_task_support','minimum_dock_count','closed_tour_navigation_cost'};
    metadata.graph_time_s=graph.build_time_s;metadata.graph_edge_count=graph.edge_count;
    metadata.graph_preprocessing_shared=true;
    metadata.main_scene_style_reference='visualization/plot_main_scenario_publication.m';
    metadata.scope='finite frozen IK-support pool and directed computed navigation oracle; no continuous/physical optimum';
    write_json(fullfile(output_dir,'metadata.json'),metadata);
    copyfile(fullfile(data.source_dir,'tasks.csv'),fullfile(output_dir,'tasks.csv'));
    copyfile(fullfile(data.source_dir,'pair_support.csv'),fullfile(output_dir,'frozen_pair_support.csv'));
    pool_rows=cell(numel(data.setup.pool),7);
    for k=1:numel(data.setup.pool)
        p=data.setup.pool(k);pool_rows(k,:)={k+1,p.candidate_id,p.table_id,p.side_id,p.pose(1),p.pose(2),p.pose(3)};
    end
    writetable(cell2table(pool_rows,'VariableNames',{'node_index','candidate_id','table_id','side_id','x_m','y_m','yaw_rad'}),fullfile(output_dir,'candidate_pool.csv'));
    cover_rows={};
    for mi=1:3
        for ri=1:numel(data.regions)
            C=data.covers{mi,ri};
            for ci=1:size(C,1)
                ids=arrayfun(@(x)data.setup.pool(x-1).candidate_id,C(ci,:));
                cover_rows(end+1,:)={modes{mi},ri,ci,strjoin(string(ids),'|'),strjoin(string(C(ci,:)),'|')}; %#ok<AGROW>
            end
        end
    end
    writetable(cell2table(cover_rows,'VariableNames',{'mode','region_id','cover_id','candidate_ids','matrix_node_indices'}),fullfile(output_dir,'minimum_covers.csv'));
    summary_rows={};dock_rows={};path_rows={};assignment_rows={};case_results=cell(3,3);
    for mi=1:3
        scene=data.scenes{mi};library=data.libraries{mi};
        minimum=sum(cellfun(@(x)x.minimum_dock_count,data.certificates(mi,:)));
        for method_index=1:3
            method=methods{method_index};planner=data.planner;
            fprintf('\nJOINT COMPARISON: %s / %s\n',modes{mi},method);
            if strcmp(method,'staged')
                planner.strategy.name='bottom_up';
            else
                planner.strategy.name='task_navigation_joint';
                planner.strategy.joint.cost_matrix=graph.cost_matrix;
                planner.strategy.joint.control=method;
                if strcmp(method,'reorder_only')
                    planner.strategy.joint.fixed_candidate_indices=data.baseline{mi}.selected_pool_ids;
                end
            end
            clock=tic;
            [poses,~,summaries,info,plan]=stage2_docking_planning(data.regions,scene,robot,planner,library);
            selection_time=toc(clock);
            assert(strcmp(plan.status,'READY'),'JointComparison:BlockedPlan','All comparison cases must cover every frozen task.');
            assert(size(poses,1)==minimum,'JointComparison:DockCountChanged');
            indices=zeros(1,size(poses,1));
            for di=1:size(poses,1)
                [distance,index]=min(vecnorm(data.all_poses(2:end,:)-poses(di,:),2,2));
                assert(distance<1e-10);indices(di)=index;
            end
            if strcmp(method,'staged')
                assert(isequal(indices(:),data.baseline{mi}.selected_pool_ids(:)), ...
                    'JointComparison:BaselineChanged','Staged selection differs from the frozen baseline.');
            elseif strcmp(method,'reorder_only')
                assert(isequal(sort(indices),sort(data.baseline{mi}.selected_pool_ids(:)')));
            end
            navigation_clock=tic;
            [~,path,~,~,~,length_m,stats]=stage3_global_planning(scene,poses,summaries, ...
                data.task_map,robot,planner,info,struct('scene_time_s',0,'docking_time_s',selection_time),plan);
            navigation_time=toc(navigation_clock);
            assert(stats.path_validation.valid);
            assert(numel(stats.execution_status.verified_tasks)==scene.num_tasks);
            local_to_global=[1,indices+1];closed_nodes=local_to_global(stats.route_order);
            length_graph=sum(graph.length_matrix(sub2ind(size(graph.length_matrix),closed_nodes(1:end-1),closed_nodes(2:end))));
            predicted=sum(graph.cost_matrix(sub2ind(size(graph.cost_matrix),closed_nodes(1:end-1),closed_nodes(2:end))));
            yaw=sum(abs(wrapToPiLocal(diff(data.all_poses(closed_nodes,3)))));
            actual_cost=length_m+graph.lambda_theta_m_per_rad*yaw;
            assert(abs(length_graph-length_m)<1e-7,'JointComparison:PathOracleMismatch');
            assert(abs(predicted-actual_cost)<1e-7,'JointComparison:CostOracleMismatch');
            if ~strcmp(method,'staged'),assert(abs(plan.joint.route_cost-actual_cost)<1e-7);end
            for di=1:size(poses,1)
                rank=find(stats.visit_order==di+1,1)-1;
                p=data.setup.pool(indices(di));
                dock_rows(end+1,:)={modes{mi},method,di,p.candidate_id,poses(di,1),poses(di,2),poses(di,3), ...
                    strjoin(string(summaries{di}.covered_tasks),'|'),rank}; %#ok<AGROW>
                for tid=reshape(summaries{di}.covered_tasks,1,[])
                    arm=summaries{di}.arm_assign_map(tid);
                    assignment_rows(end+1,:)={modes{mi},method,tid,di,p.candidate_id,arm}; %#ok<AGROW>
                end
            end
            for k=1:size(path,1)
                path_rows(end+1,:)={modes{mi},method,k,path(k,1),path(k,2),path(k,3)}; %#ok<AGROW>
            end
            evaluations=0;order_optimal=false;
            if ~strcmp(method,'staged')
                evaluations=plan.joint.route_solution.routes_evaluated;order_optimal=true;
            end
            summary_rows(end+1,:)={modes{mi},method,numel(plan.covered_tasks_predicted),scene.num_tasks, ...
                size(poses,1),minimum,length_m,actual_cost,selection_time,stats.path_validation.valid, ...
                strcmp(method,'joint'),stats.exec_stats.navigation_time_s,graph.build_time_s,selection_time, ...
                minimum,strcmp(method,'joint'),order_optimal,navigation_time,evaluations,yaw}; %#ok<AGROW>
            case_results{mi,method_index}=struct('mode',modes{mi},'method',method, ...
                'candidate_indices',indices,'closed_matrix_nodes',closed_nodes, ...
                'navigation_cost',actual_cost,'route_length_m',length_m,'plan',plan,'stats',stats);
            write_tables();
        end
        costs=cellfun(@(x)x.navigation_cost,case_results(mi,:));
        assert(costs(2)<=costs(1)+1e-7&&costs(3)<=costs(2)+1e-7, ...
            'JointComparison:OptimizationRegression','Exact controls must not worsen the shared navigation objective.');
    end
    result=struct('metadata',metadata,'case_results',{case_results}, ...
        'region_certificates',{data.certificates},'graph',graph);
    save(fullfile(output_dir,'joint_comparison_result.mat'),'result','-v7');
    fprintf('\nJOINT COMPARISON COMPLETE\n');disp(readtable(fullfile(output_dir,'summary.csv')));

    function write_tables()
        writetable(cell2table(summary_rows,'VariableNames',{'mode','method','covered_tasks','target_count','docks', ...
            'minimum_docks','route_length_m','navigation_cost','selection_time_s','route_valid','exact_on_candidate_pool', ...
            'nav_estimated_time_s','graph_time_s','search_time_s','exact_minimum_docks','route_cost_optimal', ...
            'route_order_optimal','navigation_planning_time_s','routes_evaluated','endpoint_yaw_change_rad'}),fullfile(output_dir,'summary.csv'));
        writetable(cell2table(dock_rows,'VariableNames',{'mode','method','dock_id','candidate_id','x_m','y_m','yaw_rad', ...
            'covered_task_ids','visit_rank'}),fullfile(output_dir,'docking_poses.csv'));
        writetable(cell2table(path_rows,'VariableNames',{'mode','method','point_id','x_m','y_m','yaw_rad'}),fullfile(output_dir,'navigation_paths.csv'));
        writetable(cell2table(assignment_rows,'VariableNames',{'mode','method','task_id','dock_id','candidate_id','arm'}),fullfile(output_dir,'task_assignments.csv'));
    end
end

function write_json(path,data)
    fid=fopen(path,'w','n','UTF-8');if fid<0,error('JointComparison:WriteFailed','Cannot write output.');end
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',jsonencode(data,'PrettyPrint',true));
end
