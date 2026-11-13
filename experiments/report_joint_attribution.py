"""Write the Chinese evidence report from completed paired exports."""
from analyze_joint_attribution import OUT, MODES, rows, read
import numpy as np

LABELS=dict(zip(MODES,('仅位置','朝下抓取','桌面擦拭')))


def tally(values):
    return f'{sum(v>1e-7 for v in values)}/{sum(abs(v)<=1e-7 for v in values)}/{sum(v< -1e-7 for v in values)}'


def main():
    data=rows(OUT/'paired_attribution.csv');agg=rows(OUT/'aggregate_statistics.csv');raw=rows(OUT/'summary.csv');audit=read(OUT/'audit.json')
    n0=sum(r['certified_vs_original_set']=='True' for r in data);n1=sum(r['certified_vs_old_joint_set']=='True' for r in data)
    jrows=[r for r in raw if r['method']=='balanced_strong_init']
    converged=sum(r['fixed_set_order_converged']=='1' for r in jrows)
    exact=sum(read(p)['certified_optimal'] for p in (OUT/'cases').glob('*/fixed_*.json'))
    old_exact=sum(read(p)['certified_optimal'] for p in (OUT/'cases').glob('*/old_joint_fixed_*.json'))
    lines=['# 联合规划初始化、预算分配与停靠收益归因','',
      f'2026-11-13。原始输入 b4e859f；9 场景 × 3 任务 × 4 方法，共 {len(raw)} 条新结果。'
      f'独立于重排强弱的下界判据在 {n0}/27 组证明新联合路线优于原停靠集合的任何访问顺序；'
      f'更强的旧联合集合判据在 {n1}/27 组通过。'
      f'但新分配对同强初始化的旧分配为 {tally([float(r["allocation_gain"]) for r in data])}（胜/平/负），'
      '不支持将该预算规则宣称为普遍更好的搜索策略。','',
      '## 输入与方法','',
      f'冻结输入与模型文件 {audit["unchanged_input_files"]} 份哈希保持一致。机械臂、任务姿态/路径约束、9 个场景、候选池和原始 IK/任务支持证据均不改动。'
      f'所有 {len(raw)} 个方案均复用 36/36 任务支持，保持原有限候选区域最少停靠数（16 或 32），实际导出底盘路线通过几何检查；本次新增 IK 求解为 0。','',
      '| 名称 | 含义 |','|---|---|',
      '| F：fixed_optimum | 原 minimum_seed 停靠集合的 ATSP 最优访问顺序；冻结停靠消融 |',
      '| I：legacy_strong_init | 从 F 出发，沿用旧区域扫描与每轮一次 2-opt 分配 |',
      '| N：balanced_raw_init | 从原 minimum_seed 路线出发，使用新区域配额与末轮重排 |',
      '| J：balanced_strong_init | 从 F 出发，使用同一新配额与末轮重排，预定主方法 |','',
      'I/N/J 均最多 1500 次新增有向边请求、10000 次邻域评估、120 s 协作墙钟上限、3 次区域扫描。'
      '新分配每区域最多测试 12 个按几何代价排序的候选，实际导航计价后保留最优下降；为最终选中图补全和至少一轮排序邻域评估预留预算，随后用有向 2-opt 与 relocate 搜索。'
      '每方法从相同历史边表启动且路径缓存独立；命中外部缓存的新逻辑请求仍占本方法配额。'
      '共享初始完整图 n(n−1) 条有向边与 F 求解成本单列，1500 是额外搜索预算，不是端到端总预算。','',
      '## 归因证书','',
      f'原停靠集合 {exact}/27、旧联合停靠集合 {old_exact}/27 的固定集合 ATSP 界差闭合。'
      '二元入出度 MILP 通过 DFJ 割去除子环，只把 Hamilton 闭环计为可行上界；下界另存，归因前减去数值裕量。','',
      '设固定集合最优值为 F(S)，有效下界为 LB(S)。若新路线 J_new < LB(S)，则对任意访问顺序 π 都有 '
      '`J_new < LB(S) ≤ F(S) ≤ J(S,π)`。因此增加原集合重排预算也无法解释该收益。'
      '证明针对冻结候选、同任务支持及相同导航边代价；新联合解自身未被宣称全局最优。','',
      '| 任务 | 对原集合证实组数 | 相对原集合最优顺序中位降幅 | 对旧联合集合证实组数 | 相对旧联合集合最优顺序中位降幅 |',
      '|---|---:|---:|---:|---:|']
    for a in agg:
        lines.append(f'| {LABELS[a["mode"]]} | {a["certified_vs_original"]}/9 | {float(a["gain_vs_fixed_median_pct"]):.2f}% | {a["certified_vs_old_joint"]}/9 | {float(a["gain_vs_old_joint_fixed_median_pct"]):.2f}% |')
    lines+=['','相对旧联合集合未通过的组也保留。新路线优于旧输出的实际代价，并不自动证明其停靠集合优于旧集合；旧顺序本身可能仍有改进空间。','',
      '## 初始化与预算消融','',
      '| 任务 | J 对 I 胜/平/负（调度与末轮排序组合） | J 对 N 胜/平/负（强初始化） | J 对历史旧联合实际路线胜/平/负 |','|---|---:|---:|---:|']
    for mode in MODES:
        g=[r for r in data if r['mode']==mode]
        lines.append(f'| {LABELS[mode]} | {tally([float(r["allocation_gain"]) for r in g])} | {tally([float(r["initialization_gain"]) for r in g])} | {tally([float(r["old_joint_J"])-float(r["new_joint_J"]) for r in g])} |')
    lines+=['','| 方法 | 新边请求中位数 | 邻域评估中位数 | 墙钟上限终止 | 查询上限终止 | 邻域上限终止 |',
            '|---|---:|---:|---:|---:|---:|']
    for method,label in [('legacy_strong_init','I'),('balanced_raw_init','N'),('balanced_strong_init','J')]:
        group=[r for r in raw if r['method']==method]
        counts=[sum(r['termination_reason']==reason for r in group) for reason in
                ('wall_time_budget_exhausted','navigation_query_budget_exhausted','neighbor_evaluation_budget_exhausted')]
        lines.append(f'| {label} | {np.median([float(r["new_directed_requests"]) for r in group]):.0f} | '
                     f'{np.median([float(r["neighbor_evaluations"]) for r in group]):.0f} | {counts[0]}/27 | {counts[1]}/27 | {counts[2]}/27 |')
    lines+=['',f'J 的最终固定集合 2-opt/relocate 邻域在 {converged}/27 组完成收敛；其余组的停止原因见原表。'
      '这是有限邻域收敛，不是全局最优。保持 F 为可行已知路线使 J 不会退化到 F 以上，这属于算法保留机制，不能单独当作实验成功率证据。','',
      '强初始化可能把区域替换带到不同局部搜索路径；若 N 优于 J，不能将结果改标签为主方法。'
      'I/J 的差异属于整套分配、候选选择与最终重排组合，不能分别归因于单一算子；墙钟限制受共享机器负载影响，不据此宣称运行时间加速。','',
      '**协议解释修正：** 冻结协议曾称 I 与历史旧联合可隔离初始化影响，此说法过强。历史实现将部分已定价原集合边的回调也计入内部查询配额，而 I 预先持有完整原集合图。'
      '二者同时涉及初始化和已知图计账差异；干净的初始路线消融是 J 与 N（完整图、分配规则和配额均相同）。协议原文不回写，修正另存 `PROTOCOL_CLARIFICATION.md`。','',
      '## 全部逐组结果','',
      '| 场景 | 任务 | 最少停靠 | 原最优 F | I | N | 新联合 J | 旧联合集合最优 | 证实原/旧集合 |',
      '|---|---|---:|---:|---:|---:|---:|---:|---|']
    for r in data:
        nums=' | '.join(f'{float(r[k]):.3f}' for k in ('fixed_optimum_J','legacy_strong_J','balanced_raw_J','new_joint_J','old_joint_fixed_J'))
        flags='/'.join('是' if r[k]=='True' else '否' for k in ('certified_vs_original_set','certified_vs_old_joint_set'))
        lines.append(f'| {r["case_id"]} | {LABELS[r["mode"]]} | {r["minimum_docks"]} | {nums} | {flags} |')
    lines+=['','## 研究含义与使用范围','',
      '本轮将两个容易混淆的问题分开：先用固定集合最优解消除“顺序还没排好”的解释，再用严格下界证明位姿替换的剩余价值。'
      '在多桌抓取或擦拭中，这对应于：每次停靠仍承担有原始证据支持的完整任务，停靠总数不增加，而选择不同服务侧、位置和朝向减少绕障与换位代价。','',
      'J 仍是实际导航长度加端点朝向代价，不是实测能耗、任务耗时或擦拭质量。所有场景为沿用的开发场景，结论是这组冻结有限模型中的收益归因，不能外推为任意布局的最优性。','',
      '## 复现与文件','',
      '- 协议：`paper/JOINT_ATTRIBUTION_PROTOCOL_20261113.md`；条件证明：`paper/JOINT_ATTRIBUTION_ARGUMENT_20261113.md`。',
      '- 入口、预算和复现命令：`planning_strategy/joint/README_attribution.md`。',
      '- 原始四方法结果：`summary.csv`；归因与全部对照：`paired_attribution.csv`。',
      '- 固定图证书：`cases/*/fixed_*.json`、`cases/*/old_joint_fixed_*.json`。',
      '- 图：`figures/certified_docking_gains`、`order_and_docking_decomposition`、`main_scene_attribution_routes`、`initialization_budget_ablations`（PNG/PDF）。',
      '- 算术与冻结证据审计：`audit.json`；独立核对见 `independent_audit.json`。','']
    validation=read(OUT/'validation_summary.json');independent=read(OUT/'independent_audit.json')
    lines += [f'{validation["matlab"]["passed"]} 项 MATLAB 测试与 {validation["python"]["passed"]} 项 Python 测试通过。'
              f'独立审计核对 {independent["assignments_checked"]} 次任务分配、{independent["strict_moves_checked"]} 次严格下降移动、'
              f'{independent["path_samples_checked"]} 个路径样本与 {independent["fixed_set_certificates_checked"]} 份证书，未重新求解 IK、导航或优化器。',
              '', '独立审查支持有限图中的选位收益归因；整体方法优越性结论为 partial。新预算规则保留为可选实验策略，不将其包装为已验证的普遍改进。审查记录见 `paper/review-traces/experiment-result-to-claim/2026-11-13_run01/`。','']
    (OUT/'ANALYSIS.md').write_text('\n'.join(lines),encoding='utf-8')


if __name__=='__main__':main()
