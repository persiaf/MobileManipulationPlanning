from common import *

def main():
    style();data=rows(OUT/'paired_attribution.csv');FIG.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(3,1,figsize=(10.5,9),layout='constrained')
    for mi,mode in enumerate(MODES):
        group=[r for r in data if r['mode']==mode];x=np.arange(len(group));order=np.array([float(r['order_gain']) for r in group]);dock=np.array([float(r['docking_gain']) for r in group])
        ax=axes[mi];ax.bar(x,order,.65,color='#0072B2',label='Visit-order gain on original docks');ax.bar(x,dock,.65,bottom=order,color='#E69F00',label='Additional gain requiring new docks')
        ax.set_xticks(x,[r['case_id'] for r in group]);ax.set_ylabel('Cost reduction (m-equivalent)');ax.grid(axis='y',alpha=.2);panel(ax,f'({chr(97+mi)}) {MODE_LABELS[mode]}')
    handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',ncol=2,frameon=False)
    save(fig,FIG,'order_and_docking_decomposition')
    caption(FIG,'order_and_docking_decomposition','蓝色为原分阶段路线到原停靠集合最优顺序的收益，橙色为固定集合最优顺序到新联合结果的剩余收益。两段之和等于相对分阶段总收益，避免把基线未充分重排的收益计作停靠优化贡献。各组均保留相同覆盖证据和最少停靠数。')

if __name__=='__main__':main()
