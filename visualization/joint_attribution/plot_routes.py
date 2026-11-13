from common import *

def main():
    style();case='L0_D2';src=SOURCE/'cases'/case;dest=OUT/'cases'/case;meta=read(src/'metadata.json');scene=meta['scene'];tasks=rows(src/'tasks.csv');FIG.mkdir(parents=True,exist_ok=True)
    old_summary=rows(src/'summary.csv');new_summary=rows(dest/'summary.csv')
    old_docks=rows(src/'docking_poses.csv');new_docks=rows(dest/'docking_poses.csv')
    old_path=rows(src/'navigation_paths.csv');new_path=rows(dest/'navigation_paths.csv')
    methods=[('joint_local','Previous joint'),('fixed_optimum','Original docks, optimal order'),('balanced_strong_init','New joint')]
    fig,axes=plt.subplots(3,3,figsize=(13,12.6),layout='constrained')
    for mi,mode in enumerate(MODES):
        for ci,(method,label) in enumerate(methods):
            ax=axes[mi,ci];draw_map(ax,scene,scene['border_thickness_m'])
            ds=old_docks if ci==0 else new_docks;ps=old_path if ci==0 else new_path;ss=old_summary if ci==0 else new_summary
            route=np.array([[float(r[k]) for k in ('x_m','y_m')] for r in ps if r['mode']==mode and r['method']==method]);ax.plot(route[:,0],route[:,1],color=PATH_COLOR,lw=1.1,zorder=3)
            ax.scatter([float(t['x_m']) for t in tasks],[float(t['y_m']) for t in tasks],s=14,c=TASK_FACE,edgecolors=TASK_EDGE,zorder=5,linewidth=.3)
            for d in ds:
                if d['mode']==mode and d['method']==method:draw_base(ax,[float(d[k]) for k in ('x_m','y_m','yaw_rad')],meta['robot_footprint']['length_m'],meta['robot_footprint']['width_m'])
            ax.scatter(*scene['start_pose'][:2],marker='*',s=95,c=START_COLOR,edgecolors='black',linewidth=.5,zorder=10)
            r=next(r for r in ss if r['mode']==mode and r['method']==method)
            panel(ax,f'({chr(97+3*mi+ci)}) {MODE_LABELS[mode]}')
            ax.text(.98,.025,f'J={float(r["navigation_cost"]):.2f} | docks={r["docks"]}',transform=ax.transAxes,ha='right',fontsize=9,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
            if mi==0:ax.text(.5,1.12,label,transform=ax.transAxes,ha='center',fontsize=10)
    fig.legend(handles=scene_legend(),loc='outside lower center',ncol=3,frameon=False,fontsize=9)
    save(fig,FIG,'main_scene_attribution_routes')
    caption(FIG,'main_scene_attribution_routes','预先指定的 L0_D2：原始主场景布局与两侧长跨度任务分布。每行依次为仅位置、朝下抓取、桌面擦拭；列为旧联合方案、原停靠集合最优重排、新联合方案。采用真实导出路径与停靠姿态，保持主场景灰色桌面、绿色任务、蓝色路线、橙色停靠、红色起点风格。J为端点朝向加权导航代价，不代表执行时间。')

if __name__=='__main__':main()
