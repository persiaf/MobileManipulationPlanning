function s=complete_replanning_route(s)
%COMPLETE_REPLANNING_ROUTE Connect snapped navigation ends to exact dock poses.
% No search/selection changes: only explicit short endpoint motions. The
% caller MUST validate the resulting candidate before allowing execution.
    s=normalize_planning_state(s); g=s.stats.sparse_graph;
    [ii,jj]=find(triu(g.adjacency,1));
    for k=1:numel(ii)
        i=ii(k);j=jj(k);p=g.direct_paths{i,j};
        if isempty(p), continue; end
        a=s.all_poses(i,:);b=s.all_poses(j,:);
        pre=motion(a,p(1,:),s.scene.map_info.resolution,false);
        post=motion(p(end,:),b,s.scene.map_info.resolution,true);
        q=[pre;p(2:end,:);post(2:end,:)];
        g.direct_paths{i,j}=q;g.direct_paths{j,i}=flipud(q);
        g.cost_matrix(i,j)=compute_path_length(q);g.cost_matrix(j,i)=g.cost_matrix(i,j);
    end
    dist=g.cost_matrix; n=size(dist,1); hop=zeros(n);
    for i=1:n
        for j=1:n
            if i~=j&&isfinite(dist(i,j)),hop(i,j)=j;end
        end
    end
    for k=1:n
        for i=1:n
            for j=1:n
                if dist(i,k)+dist(k,j)<dist(i,j)
                    dist(i,j)=dist(i,k)+dist(k,j);hop(i,j)=hop(i,k);
                end
            end
        end
    end
    g.shortest_cost_matrix=dist;g.next_hop=hop;
    [path,segments]=stitch_sparse_graph_path(s.all_poses,s.stats.route_order,g);
    s.stats.sparse_graph=g;s.stats.global_path=path;s.stats.segment_stats=segments;
    s.stats.path_len=compute_path_length(path);
    s.global_path=path;s.path_segments=segments;s.sparse_graph=g;s.route_order=s.stats.route_order;
    s.current_plan=s.plan;s.current_plan.global_path=path;
    s.current_plan.path_segments=segments;s.current_plan.sparse_graph=g;
    s.current_plan.route_order=s.stats.route_order;
end
function p=motion(a,b,res,incoming)
    if isequal(a,b),p=a;return;end
    if incoming
        dock=b;
        if abs(sin(dock(3)))>abs(cos(dock(3))),via=[b(1),a(2),b(3)];
        else,via=[a(1),b(2),b(3)];end
        points=[a; a(1:2),b(3);via;b];
    else
        dock=a;
        if abs(sin(dock(3)))>abs(cos(dock(3))),via=[a(1),b(2),a(3)];
        else,via=[b(1),a(2),a(3)];end
        points=[a;via;b(1:2),a(3);b];
    end
    p=points(1,:);
    for k=2:size(points,1)
        q=interpolate(points(k-1,:),points(k,:),res);p=[p;q(2:end,:)]; %#ok<AGROW>
    end
end
function p=interpolate(a,b,res)
    steps=max([1,ceil(norm(b(1:2)-a(1:2))/(res/3)),ceil(abs(b(3)-a(3))/(pi/36))]);
    t=(0:steps)'/steps;p=a+t.*(b-a);
end
