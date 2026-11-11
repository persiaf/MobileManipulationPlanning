"""Rebuild failure-inclusive paired stability statistics from batch CSVs."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"visualization/task_navigation_robustness"))
from batch_common import (ALIASES, Batch, COMPARATORS, DEFAULT_DATA, METHODS, METHOD_ZH, MODES,
                          TIE_TOL, aggregate_pairs, full_mission, full_tasks, minimum_mission,
                          mission_counts, num)

MODE_ZH = dict(zip(MODES, ("仅位置约束", "朝下抓取", "桌面擦拭")))


def fmt(value, digits=2):
    value = num(value)
    return f"{value:.{digits}f}" if math.isfinite(value) else "N/A"


def pct(value):
    return f"{num(value):.2f}%" if math.isfinite(num(value)) else "N/A"


def write_csv(path, rows):
    if not rows:
        raise ValueError(f"Cannot write empty derived table {path}")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(folder):
    batch = Batch(folder)
    pairs = batch.pairs()
    aggregates = aggregate_pairs(pairs, len(batch.cases))
    missions = mission_counts(batch)
    for filename, rows in (("paired_gains.csv", pairs), ("aggregate_statistics.csv", aggregates),
                           ("mission_success_counts.csv", missions)):
        write_csv(folder/filename, rows)
    all_primary = len(batch.cases)*len(MODES)*len(METHODS)
    primary_valid = sum(minimum_mission(case.rows[mode, method]) for case in batch.cases for mode in MODES for method in METHODS)
    joint_valid = sum(minimum_mission(case.rows[mode, "joint_local"]) for case in batch.cases for mode in MODES)
    total_joint = len(batch.cases)*len(MODES)
    text = ["# 多布局与任务分布：任务约束和导航联合停靠对照", "",
            f"本轮预先声明 {len(batch.cases)} 个因子组合，包含 {len(batch.layout_ids)} 类桌子布局与 "
            f"{len(batch.distribution_ids)} 类任务分布。每个场景比较仅位置、朝下抓取与桌面擦拭三种任务约束。"
            "机械臂继续使用相同 VX300s 六关节模型、TCP、安装假设与底盘尺寸。", "",
            "同一布局的候选子池从 reference distribution（D0）按几何规则生成并冻结，随后供该布局全部任务分布使用；"
            "所有方法和任务模式也共用该场景的候选池。完整任务数与逐桌任务数保持不变。"
            "原dense候选的朝向增补使用D0参考任务锚点，不把它称为完全独立于任务锚点的候选生成。"
            "不同布局具有各自真实地图与候选位姿，不能把不同布局的坐标当作同一个候选集合。", "",
            "## 对照定义与不可混淆的修复", "",
            "主比较固定三种方法：", "",
            "1. **分阶段+共同最少覆盖修复**（原始方法名 `minimum_seed`，主比较别名 `staged_minimum`）："
            "原 bottom_up 选位后，必要时统一修复到该场景的区域最少覆盖；不进行额外导航局部优化。",
            "2. **仅重排**（`reorder_only`）：固定同一个修复后的最少停靠位姿集合，只进行有向2-opt。",
            "3. **联合局部优化**（`joint_local`）：在相同最少覆盖层内替换区域停靠组合，并结合有向2-opt。", "",
            "原始 `staged` 贪心结果仍完整保存在 batch_summary.csv 及每场景 summary.csv 中，作为辅助诊断。"
            "它可能超过最少停靠数，因此不把它暗中当作等停靠次数的主比较。共同最少覆盖修复本身可能改变路线，"
            "必须与后续导航改善分开。", "",
            "所有主方法共同采用真实有向代价矩阵和有限最近邻闭环恢复：起点1的闭环无效才按节点编号试其它起点，"
            "取首个完整有限闭环并旋转回真实起点，不从多次重启中挑最低代价。"
            "这些共同导航修复不计为联合方法的独有收益。", "",
            "最少停靠证书严格限于各场景冻结有限子池、数值IK支持矩阵和区域任务归属。"
            "局部搜索保留共同预算，不提供全局最优或所有布局保证。", "",
            "下文‘完整数值规划有效’专指当前任务IK后端复核与底盘路线检查，非实机任务完成率。", "",
            "## 全部任务与最少停靠前提", "",
            f"主比较共 {all_primary} 个结果，其中 {primary_valid} 个满足完整任务与最少停靠前提；"
            f"联合方法满足该前提的比例为 {joint_valid}/{total_joint}。"
            "支持覆盖、当前数值后端复核、完整底盘路线、返回起点和停靠数均需结合原始记录核查。", "",
            "| 任务 | 方法 | 完整任务支持及复核 | 完整数值规划有效 | 数值规划有效且最少停靠 | 总场景 |",
            "|---|---|---:|---:|---:|---:|"]
    for row in missions:
        text.append(f"| {MODE_ZH[row['mode']]} | {METHOD_ZH[row['method']]} | {row['full_task_support_and_recheck']} | "
                    f"{row['full_missions']} | {row['full_minimum_missions']} | {row['total_scenarios']} |")
    text += ["", "失败始终保留在总场景分母中。完整任务无效、最少停靠证书缺失、停靠数不等或导航代价非有限时，"
             "不参与等条件代价配对，不把缺失值当零或删除整组场景。", "",
             "## 配对导航代价是否稳定改善", "",
             "沿用 `J = L + 0.15 m/rad × Σ|wrap(相邻服务端点朝向差)|`。"
             "L为实际导出路径长度，J的端点朝向惩罚不是实际累计转角、机器人执行时间或能耗。"
             "主统计采用三种主方法共同有效的场景：三者均完成全部任务并达到相同有限子池最少停靠数，"
             "两种参照的改善使用同一场景子集与分母。paired_gains.csv另保留pairwise_eligible辅助标志，主统计不使用该较宽松口径。", "",
             f"胜/平/负按原始J的绝对差 {TIE_TOL:g} m-equivalent 判定；百分比为 `100×(J参照−J联合)/J参照`。"
             "中位数、四分位数及最差值均基于逐场景配对百分比，不以两个未配对均值的比值替代。"
             "这些是有限确定性因子组合的描述性统计，不提供伪造置信区间或统计显著性。", "",
             "| 任务 | 参照 | 可比较/全部 | 胜/平/负 | J改善中位数 | 四分位区间 | 最差J改善 | L改善中位数 |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    for row in aggregates:
        interval = (f"[{fmt(row['J_gain_q1_pct'])}, {fmt(row['J_gain_q3_pct'])}]%"
                    if row["eligible_pairs"] else "N/A")
        text.append(f"| {MODE_ZH[row['mode']]} | {METHOD_ZH[row['comparator']]} | "
                    f"{row['eligible_pairs']}/{row['total_scenarios']} | {row['wins']}/{row['ties']}/{row['losses']} | "
                    f"{pct(row['J_gain_median_pct'])} | {interval} | {pct(row['J_gain_worst_pct'])} | {pct(row['L_gain_median_pct'])} |")
    for comparator in COMPARATORS:
        selected = [p for p in pairs if p["comparator"] == comparator]
        valid = [p for p in selected if p["eligible"]]
        wins = sum(p["outcome"] == "win" for p in valid)
        ties = sum(p["outcome"] == "tie" for p in valid)
        losses = sum(p["outcome"] == "loss" for p in valid)
        text += ["", f"相对**{METHOD_ZH[comparator]}**：全部 {len(selected)} 个预设配对中，"
                 f"{len(valid)} 个可比较，胜/平/负为 {wins}/{ties}/{losses}。"]
        if len(valid) != len(selected):
            text.append("存在未满足完整任务或最少停靠条件的组，改善统计是可比较子集上的条件结果，不能据此宣称全部场景稳定有效。")
        elif losses:
            text.append("本批次包含联合方法变差的组，不能表述为每组必然改善；应结合最差案例与预算停机原因分析。")
        elif wins == len(valid):
            text.append("本批次全部预设配对均观察到严格加权代价降低；该结果仅支持这些布局、分布、模型和预算中的一致观察。")
        else:
            text.append("本批次预设配对均未观察到加权代价增加，但其中有持平组，不能把持平报告为严格改善。")
    text += ["", "## 逐场景结果与失败保留", "",
             "| 场景 | 布局 / 分布 | 任务 | 分阶段修复 L / J | 仅重排 L / J | 联合 L / J | 最少停靠 |",
             "|---|---|---|---:|---:|---:|---:|"]
    for case in batch.cases:
        for mode in MODES:
            costs = []
            for method in METHODS:
                row = case.rows[mode, method]
                costs.append(f"{fmt(row['route_length_m'],3)} / {fmt(row['navigation_cost'],3)}" if minimum_mission(row) else "N/A")
            minimum = case.rows[mode, "minimum_seed"]["minimum_docks"]
            text.append(f"| {case.case_id} | {case.layout_label} / {case.distribution_label} | {MODE_ZH[mode]} | "
                        " | ".join(costs)+f" | {minimum} |")
    failures = [(case, mode, method, case.rows[mode, method]) for case in batch.cases for mode in MODES for method in METHODS
                if not minimum_mission(case.rows[mode, method])]
    if failures:
        text += ["", "不满足主比较前提的原始结果：", ""]
        for case, mode, method, row in failures:
            text.append(f"- {case.case_id}/{mode}/{method}：status={row['status']}，"
                        f"covered/verified={row['covered_tasks']}/{row['verified_tasks']}，"
                        f"docks/min={row['docks']}/{row['minimum_docks']}，原因：{row.get('failure_reason','')}。")
    else:
        text += ["", "本批次主比较没有因完整任务或最少停靠条件被排除的结果。"]
    text += ["", "## 原始贪心分阶段结果的辅助记录", "",
             "| 任务 | 原始staged数值规划有效 | 原始staged有效且最少 | 原始staged停靠超过证书 | 全部场景 |",
             "|---|---:|---:|---:|---:|"]
    for mode in MODES:
        raw = [case.rows[mode, "staged"] for case in batch.cases]
        text.append(f"| {MODE_ZH[mode]} | {sum(full_mission(r) for r in raw)} | {sum(minimum_mission(r) for r in raw)} | "
                    f"{sum(math.isfinite(num(r['minimum_docks'])) and num(r['docks'])>num(r['minimum_docks']) for r in raw)} | {len(raw)} |")
    text += ["", "此表只揭示原始贪心与共同最少覆盖修复的差异，不混入主比较改善百分比。", "",
             "## 可视化与复现", "",
             "`figures/robustness_gain_heatmaps` 展示全部布局×任务分布的两种配对J改善及有效/全部分母；"
             "`robustness_paired_costs` 保留逐场景点和胜/平/负；`robustness_mission_rates` 展示完整数值规划有效率及最少停靠条件。"
             "`robustness_scenario_maps` 展示全部真实地图和任务点，沿用灰桌、绿任务、红起点；"
             "固定代表场景 `L0_D2` 的三模式×三方法实际路线沿用蓝路径、橙停靠、蓝色底盘足迹/朝向。"
             "额外最差图按全部可比较joint-vs-reorder配对的最小J改善百分比选择，规则透明且允许负改善。", "",
             "```text", "python experiments/analyze_task_navigation_robustness.py",
             "python visualization/task_navigation_robustness/run_all.py", "```", "",
             "本脚本只读取COMPLETE批次，并交叉检查顶层汇总与每场景全部四种原始方法记录。"
             "派生配对表为 `paired_gains.csv`，汇总为 `aggregate_statistics.csv` 和 `mission_success_counts.csv`；"
             "它们不替代原始失败记录。`analysis_manifest.json` 和图manifest保留输入、脚本及输出SHA256。", "",
             "## 结论边界与研究价值", "",
             "该比较检验任务可行性约束、最少停靠覆盖与真实导航代价之间的耦合：覆盖和停靠数固定后，"
             "允许改变可行停靠位姿与访问顺序是否仍带来收益。仅重排控制有助于检查额外选位自由度的价值；"
             "差异包含选位与排序的相互作用，不能解释为独立选位主效应或等计算量优势。", "",
             "同一布局的三类分布是确定性设计，有限预设组合不能证明任意布局、任意任务或普遍稳定性。"
             "抓取与擦拭的路径及姿态要求不同，不构成严格嵌套可行集。最少停靠只对子池和当前数值IK证据成立。"
             "当前实验仍缺少标定安装、机械臂自碰撞/互碰/桌面扫掠、任务间及连续机械臂运动、接触力与实机执行验证。", "",
             "各场景和方法共享或复用IK证据与路径缓存，固定执行顺序和冷/热状态可能影响计时。"
             "oracle_queries是外部回调数，可以命中共享缓存；不等于总A*调用。搜索阶段耗时不含全部预处理与Stage3重建，"
             "本分析不作端到端加速、实时性、统计显著性或算法新颖性主张。独立审查结论以该批次审计与review trace为准。", ""]
    (folder/"ANALYSIS.md").write_text("\n".join(text), encoding="utf-8")
    tex = [r"\begin{table*}[t]", r"\centering",
           r"\caption{Failure-inclusive paired navigation-cost comparison across predefined layouts and task distributions. "
           r"Eligible pairs require full missions and equal certified finite-subpool minimum dock counts. "
           r"Quartiles describe deterministic scenario groups, not confidence intervals.}",
           r"\label{tab:task-navigation-robustness}", r"\begin{tabular}{llrrrrr}", r"\toprule",
           r"Task & Reference & Eligible/all & W/T/L & Median gain (\%) & IQR (\%) & Worst (\%) \\", r"\midrule"]
    labels = dict(zip(MODES, ("Position", "Downward grasp", "Table wipe")))
    for row in aggregates:
        reference = "Staged + minimum repair" if row["comparator"] == "minimum_seed" else "Reorder only"
        tex.append(f"{labels[row['mode']]} & {reference} & {row['eligible_pairs']}/{row['total_scenarios']} & "
                   f"{row['wins']}/{row['ties']}/{row['losses']} & {fmt(row['J_gain_median_pct'])} & "
                   f"[{fmt(row['J_gain_q1_pct'])}, {fmt(row['J_gain_q3_pct'])}] & {fmt(row['J_gain_worst_pct'])} " + r"\\")
    tex += [r"\bottomrule", r"\end{tabular}", r"\end{table*}", ""]
    (folder/"comparison_table.tex").write_text("\n".join(tex), encoding="utf-8")
    outputs = [folder/name for name in ("paired_gains.csv", "aggregate_statistics.csv", "mission_success_counts.csv", "ANALYSIS.md", "comparison_table.tex")]
    scripts = [Path(__file__).resolve(), ROOT/"visualization/task_navigation_robustness/batch_common.py"]
    manifest = {"scenario_count": len(batch.cases), "raw_results": len(batch.cases)*12, "primary_results": all_primary,
                "valid_minimum_primary_results": primary_valid, "absolute_J_tie_tolerance": TIE_TOL,
                "primary_eligibility": "all three primary methods full valid and equal certified minimum; common subset for both comparators",
                "inputs_sha256": {str(p.resolve()): sha256(p) for p in sorted(batch.source_paths())},
                "scripts_sha256": {str(p.resolve()): sha256(p) for p in scripts},
                "outputs_sha256": {p.name: sha256(p) for p in outputs}}
    (folder/"analysis_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    print((folder/"ANALYSIS.md").resolve())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?", type=Path, default=DEFAULT_DATA)
    main(parser.parse_args().folder)
