import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
E = ROOT / 'results/repair_20261027/evidence_20261027_221823'

def rows(name):
    return list(csv.DictReader((E / name).open(encoding='utf-8-sig')))

main = rows('main/main_raw.csv')
complex_rows = rows('complex/complex_raw.csv')
sens = rows('sensitivity/sensitivity_raw.csv')
summary = json.loads((E / 'analysis_summary.json').read_text(encoding='utf-8-sig'))
assert summary['valid_complete_snapshot'] and not summary['data_quality_issues']
assert len(main) == 3 and len(complex_rows) == 12 and len(sens) == 45
assert all(x['threshold'] == '0.3' and x['audit_status'] == 'PASS' for x in main+complex_rows+sens)
assert all(float(x['independent_pair_mismatch_count']) == 0 for x in main+complex_rows+sens)
manifest = json.loads((E / 'source_manifest.json').read_text(encoding='utf-8-sig'))
relevant = ['config/robot_config.m','config/scene_config.m',
 'manipulator/evaluate_single_arm_reachability.m','manipulator/evaluate_dual_arm_reachability.m',
 'docking/generate_docking_candidates.m','docking/evaluate_docking_candidate.m',
 'collision/check_footprint_collision.m','collision/compute_footprint_clearance.m']
source_check = []
for item in manifest['files']:
    if item['path'] in relevant:
        actual = hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()
        assert actual == item['sha256'], item['path']
        source_check.append({'path':str(ROOT/item['path']),'sha256':actual,'snapshot_match':True})

blocks=[]
def h(level,text): blocks.append({'type':'heading','level':level,'text':text})
def p(text): blocks.append({'type':'p','text':text})
def eq(latex,label): blocks.append({'type':'eq','latex':latex,'label':label})
def table(caption,headers,data): blocks.append({'type':'table','caption':caption,'headers':headers,'rows':data})

h(2,'3.7 本章实验')
p('本节通过几何一致性检验、不同任务场景的候选覆盖评价以及采样参数敏感性实验，考察所建模型能否稳定形成任务与停靠位姿之间的可达关系。实验评价对象是固定高度条件下的几何可服务性及其计算开销，完整访问路线和任务执行效果将在后续章节分别讨论。')
h(3,'3.7.1 实验设置与评价指标')
p('实验在 MATLAB R2022a 环境下进行，机器人几何参数保持与第二章一致，单臂评分阈值固定为0.30，采用名义几何评价模式并关闭候选库增强。桌面尺寸为1.20 m×0.40 m，高度为0.50 m。基础采样设置为沿边步长0.15 m、法向距离步长0.05 m和航向角步长15°，地图栅格分辨率为0.05 m。每组使用2026、2027和2028三个随机种子，计时前丢弃一次预热运行。')
p('实验包括原始16桌场景MAIN和四组16桌场景C0—C3。C0采用规则排列和离散任务，C1在桌面位置上加入扰动；C2采用位置扰动、离散随机桌面朝向和连续任务；C3采用分簇布局与混合任务分布。由于场景之间同时包含布局和任务分布差异，跨场景结果用于检验方法适用范围，不将其解释为某一个因素的独立作用。')
p(r'设场景包含的全部任务为$\mathcal T$，候选位姿集合为$\mathcal P$，用候选覆盖率衡量至少存在一个合格停靠位姿的任务比例：')
eq(r'\eta_{\mathrm{lib}}=\frac{\left|\bigcup_{p\in\mathcal P}\mathcal C(p)\right|}{|\mathcal T|}\times100\%', '3-50')
p('分母始终取全部任务，不因任务暂未被覆盖而将其剔除。候选生成耗时包含候选采样、底盘碰撞检查及可达性评价，不包含后续停靠选择、路径规划、独立复核和绘图。各场景的覆盖率汇总采用三个种子的算术平均：')
eq(r'\overline{\eta}_{\mathrm{lib}}=\frac{1}{3}\sum_{s=1}^{3}\eta_{\mathrm{lib}}^{(s)}', '3-51')
h(3,'3.7.2 几何一致性与独立复核')
p(r'为降低坐标变换及边界处理错误对评价结果的影响，采用独立的几何计算实现进行交叉检验。该实现先将目标转换到底盘坐标系，再分别减去左右臂安装偏移，独立计算工作空间边界与评分。随机检验包含750组位姿—目标样本，分别在0、0.25、0.30和0.75四个阈值下比较左右臂评分、联合评分和臂分配；对应测试通过，评分差异满足$2\times10^{-12}$的绝对误差容限。边界检验还覆盖内外径、高度上下界和后向可达范围，以及旋转、平移变换下的局部坐标一致性。')
p('底盘几何检验针对长宽方向、旋转矩形与占用栅格交叠、角点相交、地图越界及边缘净空开展验证，相关测试均通过。进一步对MAIN、C0—C3及敏感性实验的60个静态案例逐一复核，候选—任务二值覆盖判定的不一致数均为0。该结果表明候选库中的覆盖记录与独立几何判定一致；它不等同于完整关节空间可行性验证。')
h(3,'3.7.3 不同场景下的候选覆盖结果')
coverage_table=[]
for name in ['MAIN','C0','C1','C2','C3']:
    selected=[x for x in main+complex_rows if x['scene_id']==name]
    coverage_table.append([name, ' / '.join(x['task_count'] for x in selected),
        f"{sum(float(x['candidate_coverage_percent']) for x in selected)/3:.2f}",
        str(sum(int(x['sampling_gap_count']) for x in selected))])
table('表3-5 不同场景下的候选覆盖结果',
      ['场景','三个种子的任务数','平均候选覆盖率/%','独立搜索新增覆盖任务数'], coverage_table)
p('表3-5中三个种子的排列顺序为2026、2027、2028。MAIN、C0、C1和C3的候选覆盖率均达到100%，说明在这些任务分布下，有限候选集能够为全部任务提供符合几何评分阈值且底盘无碰撞的停靠位姿。C2的平均覆盖率为75.73%，表明连续任务分布下仍存在明显覆盖缺口。C3并未表现出更低的覆盖率，说明场景编号不能直接作为单调增加的难度等级。')
p('对C2的未覆盖任务，在桌面局部坐标系中以0.025 m平移步长、5°航向步长及面向任务方向的±90°范围进行有限加密搜索，并对合格位姿复核底盘碰撞。三个种子共220个任务中，原候选库覆盖167个任务；加密搜索为另外19个任务找到了安全几何见证，另有34个任务在本次搜索范围内未找到支持。逐种子结果见表3-6。')
audit_table=[]
for x in complex_rows:
    if x['scene_id']=='C2':
        n=int(x['task_count']); gap=int(x['sampling_gap_count']); unresolved=int(x['unresolved_task_count'])
        covered=n-gap-unresolved
        audit_table.append([x['seed'],str(covered),str(gap),str(unresolved)])
table('表3-6 C2场景候选覆盖缺口的独立搜索分析',
      ['随机种子','原候选库覆盖任务数','新增几何见证数','有限搜索未找到任务数'],audit_table)
p('加入上述见证后，C2三个种子的平均几何支持率由75.73%提高至84.47%。新增见证直接说明部分失败来自有限候选采样遗漏，而不是评分器认为任务在任意停靠位置都不可达。其余34个任务只能判定为本次有限搜索未找到支持，不能据此证明物理不可操作。与此同时，MAIN虽然全部任务均有候选支持，后续路线仍出现连接失败，说明本章建立的局部可服务关系尚不能保证全局导航连通。')
h(3,'3.7.4 采样参数与评价权重敏感性')
p('为考察采样密度的影响，从C2构建8 m×8 m的四桌受控场景SENS4。对每个种子保持连续桌面和任务几何不变，分别改变沿边步长、法向步长、航向步长或栅格分辨率，其余设置保持基准值。表3-7列出各设置的平均候选覆盖率和候选生成耗时。')
labels={'baseline':'基准','lateral_coarse':'沿边0.20 m','lateral_fine':'沿边0.075 m',
 'normal_coarse':'法向0.075 m','normal_fine':'法向0.025 m',
 'yaw_coarse':'航向30°','yaw_fine':'航向7.5°','grid_coarse':'栅格0.10 m','grid_fine':'栅格0.025 m'}
sensitivity_table=[]
for key,label in labels.items():
    subset=[x for x in sens if x['variant']==key]
    sensitivity_table.append([label,
      f"{sum(float(x['candidate_coverage_percent']) for x in subset)/3:.2f}",
      f"{sum(float(x['candidate_time_s']) for x in subset)/3:.2f}"])
table('表3-7 SENS4场景的采样参数敏感性结果',
      ['变更设置','平均候选覆盖率/%','平均候选生成耗时/s'],sensitivity_table)
p('沿边步长由0.15 m减小至0.075 m后，平均覆盖率从69.08%提高至70.53%，候选生成耗时由7.59 s增加至28.99 s；法向步长减小至0.025 m时，覆盖率为70.93%，耗时为22.54 s。航向步长减半后，覆盖率仍为69.08%，但耗时增加至17.67 s。这说明在当前样本中，统一加密候选并未获得与计算开销相称的覆盖收益，围绕未覆盖任务开展有针对性的补采样更值得进一步研究。')
p('栅格分辨率增大至0.10 m时，覆盖率降至14.35%；减小至0.025 m时，覆盖率为71.61%。栅格变化同时影响障碍边界和安全带的取整近似，因此该结果反映地图离散表示对短工作半径操作任务的综合影响，不能仅归因于路径搜索精度。表中的各设置均在任务覆盖阶段终止，所列耗时不能解释为完整任务规划时间。')
p('进一步将候选可达性权重、候选覆盖权重和停靠选择覆盖增益权重分别调整为基准的0.5倍与2倍。六种变体的候选覆盖率均与对应种子基准相同，说明在本组试验中，调整排序权重不能补充候选库中缺失的几何覆盖关系。权重使用前进行归一化，单项原始权重变化会同时改变其它项的最终占比；本结果不构成权重无影响或当前权重最优的证明。')
h(3,'3.7.5 实验结论与适用范围')
p('实验支持以下认识：在给定几何模型和固定阈值下，所建候选库能够形成一致的任务—停靠覆盖关系；离散采样质量及地图边界近似对覆盖效果具有明显影响；单纯加密全部候选或调整综合评分权重，均不足以稳定消除连续任务分布中的覆盖缺口。这些结果为后续多停靠选择提供了可追溯的输入，也明确了候选补采样与导航连通约束需要共同考虑。')
p('本节结果属于三个随机种子的先导性几何实验，未报告统计显著性或实机完成率。当前任务高度0.50 m距所设几何高度上界0.5173 m仅0.0173 m，垂向评分裕量较小；评价结论与该固定安装高度和工作空间参数有关。逆运动学、机械臂连杆碰撞、双臂同步动作和真实执行验证不在本节证据范围内，尚需在完整运动学模型和实际平台上进一步检验。')

(OUT/'chapter3_experiments.json').write_text(json.dumps(blocks,ensure_ascii=False,indent=2),encoding='utf-8')
evidence={
 'input_root':str(E),
 'read_only_analysis':True,'new_matlab_experiments_run':False,
 'data_quality':{'valid_complete_snapshot':True,'issues':[]},
 'source_current_matches_snapshot':source_check,
 'tables':{'3-5':[str(E/'main/main_raw.csv'),str(E/'complex/complex_raw.csv')],
 '3-6':[str(E/f'complex/cases/C2_seed{s}_task_audit.csv') for s in [2026,2027,2028]],
 '3-7':[str(E/'sensitivity/sensitivity_raw.csv')]},
 'methods':['Mean percentages are arithmetic means across 3 seeds, not pooled proportions.',
 'Table 3-7 times use candidate_time_s, not planning_time_s.',
 'C2 covered counts are task_count-sampling_gap_count-unresolved_task_count, crosschecked below against task audit CSVs.',
 'independent_pair_mismatch_count tests binary coverage decisions; it does not report floating-point score error.',
 '750 samples x 4 thresholds numerical score comparison and 2e-12 tolerance refer specifically to testRandomCrossCheckAgainstProduction in final_validation.json, not candidate-pair audit.',
 'All SENS4 cases are BLOCKED_COVERAGE; no full-mission acceleration ratios reported.',
 'No historical cafeteria or alternate physical robot geometries used.'],
 'test_evidence':[str(ROOT/'tests/test_independent_geometric_reachability.m'),str(ROOT/'tests/test_geometry_consistency_repair.m'),str(ROOT/'results/repair_20261027/final_validation.json')],
 'audit_source':[str(ROOT/'experiments/audit_independent_reachability.m'),str(ROOT/'manipulator/independent_geometric_reachability.m')],
 'scope':'nominal fixed-height geometric model; no IK or measured task execution evidence',
}
all_counts=Counter()
for seed in [2026,2027,2028]:
    rr=rows(f'complex/cases/C2_seed{seed}_task_audit.csv')
    counts=Counter(x['classification'] for x in rr)
    evidence[f'C2_seed{seed}_audit_counts']=dict(counts)
    all_counts.update(counts)
assert all_counts==Counter({'covered_in_library':167,'candidate_sampling_gap':19,'not_found_in_audited_samples':34})
evidence['C2_combined_counts']=dict(all_counts)
(OUT/'experiment_evidence.txt').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
chars=sum(len(b.get('text','')) for b in blocks)
print(json.dumps({'file':str(OUT/'chapter3_experiments.json'),'blocks':len(blocks),'prose_chars':chars,'tables':3,'equations':2},ensure_ascii=False))
