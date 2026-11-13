from common import *
from matplotlib.colors import TwoSlopeNorm

def main():
    style();data=rows(OUT/'paired_attribution.csv');FIG.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(3,2,figsize=(10.5,10),layout='constrained')
    cols=[('gain_vs_fixed_pct','certified_vs_original_set','Original set, optimal order'),
          ('gain_vs_old_joint_fixed_pct','certified_vs_old_joint_set','Old joint set, optimal order')]
    lim=max(1,max(abs(float(r[c[0]])) for r in data for c in cols))
    for mi,mode in enumerate(MODES):
        for ci,(gain,cert,label) in enumerate(cols):
            ax=axes[mi,ci];A=np.zeros((3,3));C={}
            for r in data:
                if r['mode']!=mode:continue
                a,b=int(r['case_id'][1]),int(r['case_id'][4]);A[a,b]=float(r[gain]);C[a,b]=r[cert]=='True'
            im=ax.pcolormesh(np.arange(4)-.5,np.arange(4)-.5,A,cmap='RdYlGn',norm=TwoSlopeNorm(0,-lim,lim),edgecolors='white',linewidth=2)
            for a in range(3):
                for b in range(3):
                    value_label=f'{A[a,b]:.4f}%' if 0<abs(A[a,b])<.1 else f'{A[a,b]:.1f}%'
                    ax.text(b,a,value_label+ ('\ncertified' if C[a,b] else '\nnot certified'),ha='center',va='center',fontsize=9)
            ax.set(xlim=(-.5,2.5),ylim=(2.5,-.5),aspect='equal');ax.set_xticks(range(3),['Original','Clustered','Spread']);ax.set_yticks(range(3),['Original','Staggered','Rotated'])
            panel(ax,f'({chr(97+2*mi+ci)}) {MODE_LABELS[mode]}')
            if mi==0:ax.text(.5,1.12,label,transform=ax.transAxes,ha='center',fontsize=11)
    cb=fig.colorbar(im,ax=axes,shrink=.65,label='New joint navigation-cost reduction (%)');cb.solids.set_rasterized(False)
    save(fig,FIG,'certified_docking_gains')
    caption(FIG,'certified_docking_gains','全部27组配对，左侧比较原分阶段停靠集合的最优访问顺序，右侧比较旧联合停靠集合的最优访问顺序。正值为新联合方法代价降低。certified 表示新路线严格小于对照集合的保守有效下界，因此其收益无法由该集合的任何重排解释；其余格保留。J为实际路径长度加0.15 m/rad乘服务端点朝向差。所有任务支持证据和最少停靠数沿用冻结输入。')

if __name__=='__main__':main()
