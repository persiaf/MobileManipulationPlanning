from common import *

def main():
    style();data=rows(OUT/'paired_attribution.csv');FIG.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(3,1,figsize=(11,9),layout='constrained')
    series=[('legacy_strong_J','I: strong init + old allocation','#E69F00','s'),
            ('balanced_raw_J','N: raw init + new allocation','#0072B2','^'),
            ('new_joint_J','J: strong init + new allocation','#009E73','o')]
    for mi,mode in enumerate(MODES):
        group=[r for r in data if r['mode']==mode];ax=axes[mi];x=np.arange(len(group))
        for ci,(key,label,color,marker) in enumerate(series):
            y=[100*(float(r['fixed_optimum_J'])-float(r[key]))/float(r['fixed_optimum_J']) for r in group]
            ax.plot(x,y,marker=marker,color=color,label=label,linewidth=1,markersize=5)
        ax.axhline(0,color='#666666',linestyle='--',linewidth=.8);ax.set_xticks(x,[r['case_id'] for r in group]);ax.set_ylabel('Gain vs fixed optimum (%)');ax.grid(axis='y',alpha=.2);panel(ax,f'({chr(97+mi)}) {MODE_LABELS[mode]}')
    handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',ncol=3,frameon=False,fontsize=8.5)
    save(fig,FIG,'initialization_budget_ablations')
    caption(FIG,'initialization_budget_ablations','预定的三种实现使用相同冻结任务支持、候选池与额外搜索上限。纵轴为相对原停靠集合最优顺序的收益，零线代表固定集合最优基线。I与J比较整体预算分配和末轮重排组合，N与J比较初始路线。所有组均保留，不根据结果在N/J之间择优替换主方法。连线仅辅助逐场景配对阅读，不代表连续自变量或随机重复试验。')

if __name__=='__main__':main()
