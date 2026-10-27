function outputs = render_experiment_report(result, varargin)
%RENDER_EXPERIMENT_REPORT Render the frozen Stage1/Stage2/Stage3 report set.
%   The result struct is intentionally data-only: scene, robot_cfg, plan,
%   stats, candidate_library, and optional replanning fields are consumed.

    if nargin < 1 || ~isstruct(result), error('render_experiment_report:Input', ...
            'result must be a struct containing the experiment state.'); end
    cfg = visualization_config();
    if nargin >= 2 && ~isempty(varargin{1}), cfg = merge_struct_recursive(cfg, varargin{1}); end
    if isfield(result,'visualization_config')
        cfg = merge_struct_recursive(cfg, result.visualization_config);
    end
    root = getfield_default(result, 'output_root', '');
    if ~isempty(root), cfg.output_dir = fullfile(root, 'results', 'figures', 'visualization_report'); end
    if ~isfolder(cfg.output_dir), mkdir(cfg.output_dir); end
    outputs = struct();
    stage = upper(char(getfield_default(result, 'stage', 'STAGE1')));
    scene = getfield_default(result, 'scene', []);
    plan = getfield_default(result, 'plan', struct());
    stats = getfield_default(result, 'stats', struct());

    if startsWith(stage, 'STAGE1')
        outputs.scene = render_scene(scene, cfg, 'scene');
        if isfield(result,'candidate_library')
            outputs.docking = render_docking(scene, plan, cfg, 'docking');
        end
        outputs.planning = render_planning(scene, plan, stats, cfg, 'planning');
    elseif startsWith(stage, 'STAGE2')
        outputs.scene_region = render_scene(scene, cfg, 'scene_region');
        outputs.sparse_graph = render_graph(scene, plan, stats, cfg, 'sparse_graph');
        outputs.planning = render_planning(scene, plan, stats, cfg, 'planning');
    elseif startsWith(stage, 'STAGE3')
        outputs = render_stage3(result, scene, plan, stats, cfg, outputs);
    else
        error('render_experiment_report:Stage', 'Unsupported stage: %s', stage);
    end
end

function outputs = render_stage3(result, scene, plan, stats, cfg, outputs)
    rp = getfield_default(result, 'replanning', struct());
    if ~isfield(rp,'planning_state_before'), rp.planning_state_before = state_from_result(result); end
    if isfield(rp,'change_event')
        state = rp.planning_state_before;
        [changed,event] = apply_dynamic_event(state, rp.change_event.type, rp.change_event.event_id);
        impact = analyze_change_impact(state,event); %#ok<NASGU>
        outputs.before_event = render_scene_state(state, cfg, 'before_event');
        outputs.event = render_scene_event(changed, event, cfg, 'event');
        local = getfield_default(rp,'local_result',struct());
        outputs.replanning = render_scene_event(changed, event, cfg, 'replanning', local);
        outputs.comparison = render_comparison(state, changed, event, cfg, 'comparison');
    else
        outputs.before_event = render_scene(scene, cfg, 'before_event');
        outputs.event = render_scene(scene, cfg, 'event');
        outputs.replanning = render_planning(scene, plan, stats, cfg, 'replanning');
        outputs.comparison = render_planning(scene, plan, stats, cfg, 'comparison');
    end
end

function state = state_from_result(result)
    state = struct('scene',result.scene,'plan',getfield_default(result,'plan',struct()), ...
        'stats',getfield_default(result,'stats',struct()), ...
        'all_poses',getfield_default(result,'all_poses',[]));
end

function path = render_scene(scene,cfg,name)
    fig=visualize_complex_scene(scene,cfg,'title','','filename','');
    finalize(fig,cfg,name); path=fullfile(cfg.output_dir,[name '.png']);
end

function path = render_scene_state(state,cfg,name)
    path=render_scene(state.scene,cfg,name);
end

function path = render_docking(scene,plan,cfg,name)
    fig=visualize_planning_overlay(scene,plan,struct(),cfg,'title','','filename','');
    finalize(fig,cfg,name); path=fullfile(cfg.output_dir,[name '.png']);
end

function path = render_planning(scene,plan,stats,cfg,name)
    fig=visualize_planning_overlay(scene,plan,stats,cfg,'title','','filename','');
    finalize(fig,cfg,name); path=fullfile(cfg.output_dir,[name '.png']);
end

function path = render_graph(scene,plan,stats,cfg,name)
    fig=visualize_planning_overlay(scene,plan,stats,cfg,'title','','filename','');
    finalize(fig,cfg,name); path=fullfile(cfg.output_dir,[name '.png']);
end

function path = render_scene_event(state,event,cfg,name,local_result)
    if nargin < 5, local_result=struct(); end
    fig=visualize_planning_overlay(state.scene,state.plan,state.stats,cfg,'title','','filename','');
    ax=findobj(fig,'Type','axes');
    if ~isempty(ax)
        ax=ax(1); hold(ax,'on');
        if isfield(event,'center') && all(isfinite(event.center))
            plot(ax,event.center(1),event.center(2),'x','Color',cfg.style.obstacle,'LineWidth',2,'MarkerSize',10);
        end
        text(ax,0.02,0.98,sprintf('%s | local A*: %d',event.type,getfield_default(local_result,'local_astar_calls',0)), ...
            'Units','normalized','VerticalAlignment','top','FontName',cfg.style.font_name,'FontSize',cfg.style.font_size,'BackgroundColor','w');
    end
    finalize(fig,cfg,name); path=fullfile(cfg.output_dir,[name '.png']);
end

function path = render_comparison(before,after,event,cfg,name)
    fig=figure('Color','w','Visible',cfg.figure_visible,'Position',[80 80 1400 650]);
    tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
    for k=1:2
        ax=nexttile(tl); hold(ax,'on'); axis(ax,'equal');
        if k==1, s=before; label='Before'; else, s=after; label='Event'; end
        visualize_scene_axes(ax,s.scene,cfg);
        if isfield(event,'center') && all(isfinite(event.center)), plot(ax,event.center(1),event.center(2),'x','Color',cfg.style.obstacle,'LineWidth',2,'MarkerSize',10); end
        text(ax,0.02,0.98,label,'Units','normalized','VerticalAlignment','top','FontName',cfg.style.font_name,'FontSize',cfg.style.font_size,'BackgroundColor','w');
    end
    finalize(fig,cfg,name); path=fullfile(cfg.output_dir,[name '.png']);
end

function visualize_scene_axes(ax,scene,cfg)
    cla(ax); xlim(ax,[0 scene.map_info.width_m]); ylim(ax,[0 scene.map_info.height_m]);
    if cfg.show_grid, grid(ax,'on'); else, grid(ax,'off'); end
    draw_map_local(ax,scene,cfg); draw_tables_local(ax,scene,cfg); draw_tasks_local(ax,scene,cfg);
    plot(ax,scene.start_pose(1),scene.start_pose(2),'p','MarkerSize',12,'MarkerFaceColor',cfg.style.robot,'MarkerEdgeColor','k');
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
end

function draw_map_local(ax,s,cfg)
    [i,j]=find(s.grid_map>0); if isempty(i),return;end
    xy=zeros(numel(i),2); for k=1:numel(i), [xy(k,1),xy(k,2)]=grid2world(i(k),j(k),s.map_info.resolution,s.map_info.height_m); end
    plot(ax,xy(:,1),xy(:,2),'.','Color',cfg.style.obstacle,'MarkerSize',2,'HandleVisibility','off');
end
function draw_tables_local(ax,s,cfg)
    c=s.scene_cfg; for k=1:size(c.table_positions,1), q=oriented_rectangle_corners(c.table_positions(k,:),c.table_orientations(k),c.table.length,c.table.width); patch(ax,q(:,1),q(:,2),cfg.style.table,'FaceAlpha',.18,'EdgeColor',cfg.style.table,'LineWidth',cfg.style.line_width,'HandleVisibility','off'); end
end
function draw_tasks_local(ax,s,cfg)
    if s.num_tasks>0, plot(ax,[s.tasks.x],[s.tasks.y],'o','MarkerSize',4,'MarkerFaceColor',cfg.style.task,'MarkerEdgeColor','none','HandleVisibility','off'); end
end

function finalize(fig,cfg,name)
    ax=findobj(fig,'Type','axes'); for k=1:numel(ax), title(ax(k),''); set(ax(k),'FontName',cfg.style.font_name,'FontSize',cfg.style.font_size,'LineWidth',cfg.style.axis_line_width,'Color','w'); end
    save_visualization_figure(fig,cfg,name); if cfg.close_figures_after_save, close(fig); end
end
function v=getfield_default(s,n,d), if isstruct(s)&&isfield(s,n),v=s.(n);else,v=d;end,end
