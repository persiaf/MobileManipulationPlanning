"""Rebuild a compact Chinese result report and LaTeX table from raw exports."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

MODES = ("position_only", "top_down_grasp", "table_wipe")
LABELS = ("仅位置约束", "朝下抓取", "桌面擦拭")


def rows(folder, filename):
    with (folder / filename).open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def main(folder):
    summary = {r["mode"]: r for r in rows(folder, "summary.csv")}
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8-sig"))
    diag = json.loads((folder / "diagnostics.json").read_text(encoding="utf-8-sig"))
    audit = json.loads((folder / "evidence_audit.json").read_text(encoding="utf-8-sig"))
    trans = {(r["planned_mode"], r["evaluated_mode"]): int(r["supported_tasks"])
             for r in rows(folder, "plan_transfer.csv")}
    text = ["# 相同机械臂与桌面场景下的任务约束对照", "",
            "日期：2026-11-11。结果来自一次冻结场景的确定性实验，不是跨场景统计结论。", "",
            f"同一 VX300s 链、安装变换与 TCP；两张 1.2×0.4×0.5 m 桌，10 个固定目标；"
            f"同一 {meta['candidate_count']} 个无底盘碰撞候选，同一 A*、代价权重、访问规则与返回起点条件。", "",
            "| 条件 | 固定底盘下可行采样点 | 候选—任务支持 | 所选停靠 | IK 核查任务 | 往返路线 | 匀速导航估计 |",
            "|---|---:|---:|---:|---:|---:|---:|"]
    texrows = []
    for i, (mode, label) in enumerate(zip(MODES, LABELS)):
        r = summary[mode]
        support = int(r["supported_pairs"])
        pairs = int(r["pair_count"])
        count = diag["workspace_supported_counts"][i]
        n = diag["workspace_count"]
        text.append(f"| {label} | {count}/{n} ({count/n:.1%}) | {support}/{pairs} ({support/pairs:.1%}) | "
                    f"{r['selected_docks']} | {r['verified_tasks']}/{r['target_count']} | "
                    f"{float(r['route_length_m']):.3f} m | {float(r['nav_estimated_time_s']):.2f} s |")
        texrows.append(f"{('Position only','Downward grasp','Table wipe')[i]} & {count}/{n} & {support}/{pairs} & "
                       f"{r['selected_docks']} & {r['verified_tasks']}/{r['target_count']} & "
                       f"{float(r['route_length_m']):.3f} \\\\")
    text += ["", "支持比例表示本次预算下找到 IK 证据的候选—任务对；它与全任务覆盖率不同。"
             "可操作点只对应第一桌固定参考底盘 (2,1.45,π/2)、z=0.50 m 的有限网格。"
             "三条路线均须检查 route_valid；匀速导航估计按相同 0.30 m/s 计算，不含实际操纵、接触、停靠或实测控制时间。", "",
             "## 停靠与路线的变化", ""]
    baseline = float(summary[MODES[0]]["route_length_m"])
    for mode, label in zip(MODES[1:], LABELS[1:]):
        r = summary[mode]
        distance = float(r["route_length_m"])
        text.append(f"- {label}：相对于仅位置约束，往返路线变化 {distance-baseline:+.3f} m"
                    f"（{distance/baseline-1:+.1%}），停靠由 {summary[MODES[0]]['selected_docks']} 次变为 {r['selected_docks']} 次。")
    text += ["", "这是满足不同任务约束产生的移动需求差异，不是导航算法之间的优劣比较，也不是‘操作约束一定增加路线长度’的证明。", "",
             "对每张桌的五个任务另外用 2^5 状态动态规划求解固定候选池上的精确最少停靠集合覆盖。"
             "三模式的最少停靠数分别为 " + "、".join(str(audit['derived'][m]['minimum_docks_on_frozen_pool']) for m in MODES)
             + "；这只认证离散候选矩阵上的最少停靠数量，不认证连续空间或导航路线的全局最优。", "",
             "## 原停靠计划能否直接迁移", "",
             "| 生成停靠集合时的条件 | 按位置条件检查 | 按抓取条件检查 | 按擦拭条件检查 |",
             "|---|---:|---:|---:|"]
    for mode, label in zip(MODES, LABELS):
        text.append("| " + label + " | " + " | ".join(f"{trans[mode, target]}/10" for target in MODES) + " |")
    text += ["", "这张表允许在已有停靠集合内重新分配任务，所以表示停靠集合的任务支持上限。"
             "它不是保持原逐任务分配后的真实执行成功率。对角线是各任务模式的重新规划结果。", "",
             "## 只改变姿态自由度的补充对照", "",
             f"固定相同 12 cm 横向路径、TCP、采样间距、位置精度、5° 姿态容差和关节连续性规则，"
             f"只改变 none/axis/full 姿态模式。{diag['matched_path_count']} 个采样点的支持数依次为 "
             + "、".join(str(v) for v in diag["matched_path_supported_counts"]) + "。", "",
             f"数值非单调计数：axis 找到解但 none 未找到 {diag['matched_axis_success_position_failure']} 点；"
             f"full 找到解但 axis 未找到 {diag['matched_full_success_axis_failure']} 点。"
             "这些是有限搜索现象，不改变理论上的可行集合包含关系。主三任务包含不同路径，不直接套用此包含关系。", "",
             "在这 55 个点上，朝下轴向约束与完整姿态约束得到相同支持集合，未观察到额外固定面内朝向进一步减少支持。"
             "不能据此推断任意任务中两种姿态约束都等价。", "",
             "## 研究价值与目前可支持的结论", "",
             "1. **将机械臂约束传递到移动决策。** 将任务相关的候选—任务可行矩阵作为移动规划输入，"
             "能够解释为什么同一目标位置在不同任务下需要不同停靠集合。本次三组均使用相同 IK 模型，不把旧固定半径近似混入数值对照。", "",
             "2. **区分点可达、姿态可达与整段操作的采样证据。** 同一臂、相邻关节变化受限的离散序列、工具坐标、接触高度与方向均可独立检查，"
             "避免以目标中心可达替代整段任务支持。当前仍不认证连续无碰撞运动。", "",
             "3. **为覆盖—移动代价的联合研究建立接口。** 本版是停靠选择后再导航的两阶段启发式。"
             "下一步可在共同候选集合上研究导航代价、姿态/关节裕量和停靠误差对选点的共同影响，不能把本版称为联合全局最优算法。", "",
             "4. **给出可被审查的失败来源与计算成本。** 任务支持矩阵、错误残差、关节步长、迁移表和实路径均有原始记录。"
             "单次评分时间只能说明这一次计算的开销，不能支持通用效率结论。", "",
             "## 创新边界与一手文献", "",
             "姿态约束表达、可达性地图和任务相关基座规划已有充分研究。"
             "因此‘加入 IK/姿态约束’本身应作为建模基础，后续贡献应来自更好的约束反馈、停靠/路线决策或有证据的鲁棒机制：", "",
             "- [Task Space Regions（Berenson 等，2011）](https://personalrobotics.cs.washington.edu/publications/berenson2011task/)：末端位姿约束的表示与规划。",
             "- [Reuleaux](https://arxiv.org/abs/1710.01328)：基于可达性分析的机器人基座放置。",
             "- [Efficient and Robust Base Sequence（Xu 等）](https://arxiv.org/abs/2001.08042)：多次取放任务的基座序列与定位误差。",
             "- [End-effector Trajectory Continuity（Nguyen、Pham）](https://arxiv.org/abs/2309.12251)：末端连续轨迹约束下的移动操作规划。", "",
             "上述文献用于明确已有问题范围，不表示本次已完成与这些方法的复现对比。", "",
             "## 限制与验证", "",
             f"独立 CSV/几何审计：**{audit['status']}**。新增协议测试 7/7 通过。"
             "本次未重跑全仓测试，机械臂模型沿用上一版经验证的 FK/IK。", "",
             "安装变换未做实体标定；机械臂自碰撞、互碰、桌面避碰、采样点间运动、接触力、清洁效果和实机完成均未验证。"
             "三模式按固定顺序各运行一次，未做统计显著性或普遍性能主张。还需要多布局/高度/任务方向、强基线、碰撞感知验证以及有来源的停靠误差实验。", "",
             "独立论文架构/审查角色的结论：限定场景的机制主张得到支持；论文新颖性和通用优越性尚无证据。"
             "因缺少 manuscript-to-evidence 审计，判断保持 provisional。"
             "[完整独立审查记录](../../paper/review-traces/experiment-result-to-claim/2026-11-11_run01/raw_response.md)。", "",
             "## 文件与复现", "",
             "- 协议：EXPERIMENT_PROTOCOL.md；几何与模型参数：metadata.json；实际任务覆盖参数：effective_task_settings.json。"
             "metadata.ik_budget 中的容差是模型默认值，实际运行由 task 级 3 mm、10°/5° 覆盖。",
             "- 原始证据：summary.csv、pair_support.csv、candidate_support.csv、docking_poses.csv、navigation_paths.csv、plan_transfer.csv。",
             "- 图：figures/workspace_support.pdf、figures/dock_routes.pdf、figures/comparison_metrics.pdf，附 300 dpi PNG 与图注。",
             "- MATLAB：`addpath('pipeline'); init_paths; result=exp_task_constraint_comparison;`",
             "- 审计：`python experiments/audit_task_constraint_comparison.py`",
             "- 画图：`python visualization/task_constraint_comparison/run_all.py`",
             "- 本报告：`python experiments/analyze_task_constraint_comparison.py`", ""]
    (folder / "RESULTS_ANALYSIS.md").write_text("\n".join(text), encoding="utf-8")
    table = ("\\begin{table*}[t]\n\\centering\n"
             "\\caption{Task-conditioned numerical IK support and round-trip navigation cost in one frozen scene. "
             "Workspace samples use a fixed reference base. Verified tasks indicate sampled IK checks, not physical completion.}\n"
             "\\label{tab:task-constraint-comparison}\n\\begin{tabular}{lrrrrr}\n\\toprule\n"
             "Condition & Workspace samples & Supported pairs & Docks & IK checked & Route (m) \\\\\n\\midrule\n"
             + "\n".join(texrows) + "\n\\bottomrule\n\\end{tabular}\n\\end{table*}\n")
    (folder / "comparison_table.tex").write_text(table, encoding="utf-8")
    print((folder / "RESULTS_ANALYSIS.md").resolve())


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("folder", nargs="?", type=Path, default=Path("results/task_constraint_comparison_20261111"))
    main(p.parse_args().folder)
