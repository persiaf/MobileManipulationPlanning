function graph=build_joint_navigation_graph(data,output_dir)
%BUILD_JOINT_NAVIGATION_GRAPH Directed real-path costs for every usable edge.
% Necessary edges preserve every minimum-cover tour and the staged controls.
% Within one region, candidates not sharing a minimum cover cannot co-occur.
    if ~isfolder(output_dir),mkdir(output_dir);end
    cache_file=fullfile(output_dir,'navigation_graph.mat');
    if isfile(cache_file)
        saved=load(cache_file,'graph');
        if strcmp(saved.graph.signature,data.graph_signature)
            graph=saved.graph;fprintf('Reusing frozen directed navigation graph.\n');return;
        end
        error('build_joint_navigation_graph:StaleGraph','Existing navigation graph has different semantics. Use a fresh output folder.');
    end
    poses=data.all_poses;n=size(poses,1);scene=data.setup.scene;planner=data.planner;
    L=inf(n);D=inf(n);L(1:n+1:end)=0;D(1:n+1:end)=0;
    [from,to]=find(data.needed);count=numel(from);rows=cell(count,9);clock=tic;
    for k=1:count
        a=from(k);b=to(k);query_clock=tic;
        [~,length_m,ok]=query_path_with_shared_cache(scene.grid_map,poses(a,:),poses(b,:),scene.map_info,planner);
        elapsed=toc(query_clock);yaw=abs(wrapToPiLocal(poses(b,3)-poses(a,3)));
        if ok&&isfinite(length_m)&&length_m<=1e6
            L(a,b)=length_m;D(a,b)=length_m+planner.edge_cost.lambda_theta*yaw;
        end
        ca=0;cb=0;
        if a>1,ca=data.setup.pool(a-1).candidate_id;end
        if b>1,cb=data.setup.pool(b-1).candidate_id;end
        rows(k,:)={a,b,ca,cb,L(a,b),yaw,D(a,b),ok,elapsed};
        if mod(k,100)==0,fprintf('DIRECTED GRAPH %d/%d edges, %.1f s\n',k,count,toc(clock));end
    end
    graph=struct('signature',data.graph_signature,'poses',poses,'length_matrix',L, ...
        'cost_matrix',D,'needed',data.needed,'edge_count',count,'build_time_s',toc(clock), ...
        'lambda_theta_m_per_rad',planner.edge_cost.lambda_theta, ...
        'directed',true,'scope','actual planner edge costs on the frozen candidate graph');
    writetable(cell2table(rows,'VariableNames',{'from_node','to_node','from_candidate_id', ...
        'to_candidate_id','path_length_m','endpoint_yaw_change_rad','navigation_cost','path_found','query_time_s'}), ...
        fullfile(output_dir,'navigation_edges.csv'));
    save(cache_file,'graph','-v7');
    fprintf('DIRECTED GRAPH COMPLETE: %d edges, %.3f s\n',count,graph.build_time_s);
end
