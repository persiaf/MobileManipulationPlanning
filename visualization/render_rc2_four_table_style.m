function manifest = render_rc2_four_table_style(raw_file, output_dir)
%RENDER_RC2_FOUR_TABLE_STYLE Render S2's real four-table result in one style.
% The renderer never calls the Stage Review V1 renderer and never invents a path.
    if nargin < 2, error('raw_file and output_dir are required.'); end
    raw_file = char(java.io.File(raw_file).getCanonicalPath());
    output_dir = char(java.io.File(output_dir).getCanonicalPath());
    if ~isfile(raw_file), error('Missing S2 result file: %s', raw_file); end
    mkdir(output_dir);
    raw = load(raw_file);
    required = {'scene','library','plan','global_path'};
    for i=1:numel(required)
        if ~isfield(raw,required{i}), error('S2 result is missing field %s.',required{i}); end
    end
    scene=raw.scene; library=raw.library; plan=raw.plan; path=raw.global_path;
    names={'RC2_4TABLE_01_SCENE','RC2_4TABLE_02_CANDIDATES', ...
        'RC2_4TABLE_03_DOCK_ASSIGNMENT','RC2_4TABLE_04_GLOBAL_PATH'};
    manifest=struct('raw_file',raw_file,'output_dir',output_dir,'table_count',size(scene.scene_cfg.table_positions,1), ...
        'task_count',numel(scene.tasks),'candidate_count',library.total_candidates, ...
        'selected_dock_count',size(plan.selected_poses,1),'path_present',~isempty(path));
    for k=1:4
        f=figure('Visible','off','Color','w','Position',[80 80 980 760]);
        ax=axes(f); hold(ax,'on'); draw_map(ax,scene); draw_tasks(ax,scene);
        if k>=2, draw_candidates(ax,library); end
        if k>=3, draw_docks_and_assignments(ax,scene,plan); end
        if k==4
            if ~isempty(path)
                plot(ax,path(:,1),path(:,2),'-','Color',[0.05 0.75 0.40],'LineWidth',2.5);
            end
        end
        plot(ax,scene.start_pose(1),scene.start_pose(2),'p','MarkerSize',13, ...
            'MarkerFaceColor',[1 .82 .05],'MarkerEdgeColor','k');
        [ttl,sub]=reference_title(plan,path,k);
        title(ax,ttl,'Interpreter','none','FontWeight','normal');
        subtitle(ax,sub,'Interpreter','none');
        xlabel(ax,'World x (m)'); ylabel(ax,'World y (m)');
        xlim(ax,[0 scene.map_info.width_m]); ylim(ax,[0 scene.map_info.height_m]);
        axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
        exportgraphics(f,fullfile(output_dir,[names{k} '.png']),'Resolution',180);
        exportgraphics(f,fullfile(output_dir,[names{k} '.pdf']),'ContentType','vector');
        close(f);
    end
end

function draw_map(ax,scene)
    map=scene.grid_map; rgb=zeros(size(map,1),size(map,2),3);
    free_color=[0.97 0.97 0.97]; obstacle_color=[0.18 0.20 0.23]; table_color=[0.18 0.20 0.23];
    for c=1:3
        layer=ones(size(map))*free_color(c);
        layer(map==1)=obstacle_color(c); layer(map>=2)=table_color(c); rgb(:,:,c)=layer;
    end
    image(ax,[0 scene.map_info.width_m],[0 scene.map_info.height_m],flipud(rgb));
    set(ax,'YDir','normal');
end

function draw_tasks(ax,scene)
    x=[scene.tasks.x]; y=[scene.tasks.y];
    scatter(ax,x,y,70,[0.93 0.35 0.05],'filled','MarkerEdgeColor','none');
    for i=1:numel(scene.tasks)
        text(ax,x(i)+.06,y(i)+.05,sprintf('T%d',scene.tasks(i).id),'FontWeight','bold','Color','k');
    end
end

function draw_candidates(ax,library)
    for ri=1:numel(library.regions)
        cs=library.regions(ri).candidates;
        for ci=1:numel(cs)
            if isfield(cs(ci),'pose') && ~isempty(cs(ci).pose)
                plot(ax,cs(ci).pose(1),cs(ci).pose(2),'.','Color',[0.25 0.45 0.85],'MarkerSize',6);
            end
        end
    end
end

function draw_docks_and_assignments(ax,scene,plan)
    if ~isfield(plan,'selected_poses') || isempty(plan.selected_poses), return; end
    p=plan.selected_poses;
    scatter(ax,p(:,1),p(:,2),75,[0.05 0.35 0.85],'s','filled','MarkerEdgeColor','none');
    quiver(ax,p(:,1),p(:,2),.18*cos(p(:,3)),.18*sin(p(:,3)),0,'Color',[0.05 0.35 0.85],'LineWidth',1.4);
    for d=1:size(p,1), text(ax,p(d,1)+.06,p(d,2)+.04,sprintf('D%d',d),'Color',[0.05 .25 .75],'FontWeight','normal'); end
    if isfield(plan,'task_sequence_summary')
        for d=1:min(numel(plan.task_sequence_summary),size(p,1))
            s=plan.task_sequence_summary{d};
            if ~isfield(s,'covered_tasks'), continue; end
            for tid=s.covered_tasks
                idx=find([scene.tasks.id]==tid,1); if isempty(idx), continue; end
                plot(ax,[p(d,1) scene.tasks(idx).x],[p(d,2) scene.tasks(idx).y],':', ...
                    'Color',[0.75 .10 .65],'LineWidth',1.2);
            end
        end
    end
end

function [ttl,sub] = reference_title(plan,path,figure_index)
    strategy='bottom_up';
    if isfield(plan,'strategy_name'), strategy=char(plan.strategy_name); end
    if isempty(path)
        state='blocked'; distance='';
    else
        state='safe'; distance=sprintf(' | %.3f m',compute_path_length(path));
    end
    ttl=sprintf('RC2 | %s | %s%s',strategy,state,distance);
    if isfield(plan,'preferred_visit_order') && ~isempty(plan.preferred_visit_order)
        order=plan.preferred_visit_order-1;
    elseif isfield(plan,'route_order') && ~isempty(plan.route_order)
        order=plan.route_order-1;
    else
        order=[];
    end
    if isempty(order)
        if figure_index==4 && isempty(path)
            sub='Visit order (start=0): unavailable | NO GLOBAL PATH';
        else
            sub='Visit order (start=0): unavailable';
        end
    else
        sub=['Visit order (start=0): ' mat2str(order)];
    end
end
