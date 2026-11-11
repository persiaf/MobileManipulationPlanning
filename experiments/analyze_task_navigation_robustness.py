"""Rebuild failure-inclusive paired stability statistics from batch CSVs."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import median
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"visualization/task_navigation_robustness"))
from batch_common import (Batch, COMPARATORS, DEFAULT_DATA, METHODS, METHOD_ZH, MODES,
                          TIE_TOL, aggregate_pairs, boolean, full_mission, minimum_mission,
                          mission_counts, num, read_csv)

MODE_ZH = dict(zip(MODES, ("仅位置约束", "朝下抓取", "桌面擦拭")))


def fmt(value, digits=2):
    value = num(value)
    if math.isfinite(value) and abs(value) < .5*10**(-digits):
        value = 0.0
    return f"{value:.{digits}f}" if math.isfinite(value) else "N/A"


def pct(value):
    return f"{num(value):.2f}%" if math.isfinite(num(value)) else "N/A"


def precise_pct(value):
    value = num(value)
    if not math.isfinite(value):
        return "N/A"
    return f"{value:.2e}%" if 0 < abs(value) < .00005 else f"{value:.4f}%"


def write_csv(path, rows):
    if not rows:
        raise ValueError(f"Cannot write empty derived table {path}")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reorder_diagnostic_section(batch, pairs):
    """Optional post-hoc graph reference; never changes primary statistics."""
    path = batch.directory/"reorder_convergence_diagnostic.csv"
    if not path.exists():
        return [], None
    rows = read_csv(path, ("case_id", "mode", "status", "local_optimum_certified", "diagnostic_graph_J",
                           "main_reorder_J", "main_joint_J"), allow_empty=True)
    batch.sources.add(path)
    common = {(p["case_id"], p["mode"]) for p in pairs if p["eligible"]}
    text = ["", "## 后验图重排诊断", "",
            "从同一最少停靠种子出发，在已定价固定集合图上用best-improvement有向2-opt搜索至局部收敛，"
            "对照主实验最多3轮B的排序空间。"
            "该诊断不新增IK或导航查询，另列结果，不改变主比较及其分母。", "",
            "| 任务 | 已收敛图参照/全部场景 | B−图参照J中位数（范围） | C对图参照胜/平/负 |",
            "|---|---:|---:|---:|"]
    totals = {"eligible": 0, "wins": 0, "ties": 0, "losses": 0}
    for mode in MODES:
        selected = [row for row in rows if row["mode"] == mode and (row["case_id"], mode) in common
                    and row["status"] == "GRAPH_DIAGNOSTIC_COMPLETE" and boolean(row["local_optimum_certified"])
                    and all(math.isfinite(num(row[key])) for key in ("diagnostic_graph_J", "main_reorder_J", "main_joint_J"))]
        gaps = [num(row["main_reorder_J"])-num(row["diagnostic_graph_J"]) for row in selected]
        deltas = [num(row["diagnostic_graph_J"])-num(row["main_joint_J"]) for row in selected]
        wins = sum(value > TIE_TOL for value in deltas)
        losses = sum(value < -TIE_TOL for value in deltas)
        ties = len(deltas)-wins-losses
        for name, value in (("eligible", len(selected)), ("wins", wins), ("ties", ties), ("losses", losses)):
            totals[name] += value
        gap = f"{fmt(median(gaps),3)} [{fmt(min(gaps),3)}, {fmt(max(gaps),3)}]" if gaps else "N/A"
        text.append(f"| {MODE_ZH[mode]} | {len(selected)}/{len(batch.cases)} | {gap} | {wins}/{ties}/{losses} |")
    text += ["", "B−图参照为正表示主实验B仍有可释放的重排收益。这里的C胜负只针对有完整主比较且图重排已局部收敛的记录；"
             "达到诊断预算或未形成同集合图的组保留在原诊断表中。"
             "诊断与B沿不同搜索轨迹，可能收敛到不同局部解，因此图参照不一定比B代价更低。"]
    return text, totals


def regional_neighborhood_section(batch):
    if not {"L0_D0", "L0_D2"}.issubset(batch.by_id):
        return []
    data = {}
    for case_id in ("L0_D0", "L0_D2"):
        path = batch.by_id[case_id].directory/"minimum_cover_certificates.csv"
        if not path.exists():
            return []
        rows = read_csv(path, ("mode", "minimum_docks", "cover_count"))
        batch.sources.add(path)
        for mode in ("top_down_grasp", "table_wipe"):
            selected = [row for row in rows if row["mode"] == mode]
            if not selected or not all(math.isfinite(num(row["minimum_docks"])) for row in selected):
                continue
            sizes = [int(num(row["minimum_docks"])) for row in selected]
            options = sum(int(num(row["cover_count"]))*math.factorial(k)-1 for row, k in zip(selected, sizes))
            data[case_id, mode] = sum(sizes), options
    text = ["", "同一原布局从D0变为长跨度D2后，区域替换的潜在排列数扩大：", "",
            "| 任务 | 最少停靠D0→D2 | 潜在区域覆盖排列D0→D2 |", "|---|---:|---:|"]
    for mode in ("top_down_grasp", "table_wipe"):
        if not all((case_id, mode) in data for case_id in ("L0_D0", "L0_D2")):
            continue
        a, b = data["L0_D0", mode], data["L0_D2", mode]
        text.append(f"| {MODE_ZH[mode]} | {a[0]}→{b[0]} | {a[1]}→{b[1]} |")
    text += ["", "排列数按 `Σ(cover_count×k!−1)` 计算。它与负例中的预算停机相对应，提示大停靠数下邻域规模与预算分配是当前瓶颈。"]
    return text


def research_judgment(batch, aggregates, pairs, diagnostic):
    totals = {}
    for comparator in COMPARATORS:
        selected = [row for row in aggregates if row["comparator"] == comparator]
        totals[comparator] = {key: sum(row[key] for row in selected)
                              for key in ("eligible_pairs", "wins", "ties", "losses")}
    a, b = totals["minimum_seed"], totals["reorder_only"]
    n = len(batch.cases)*len(MODES)
    first = (f"本批{n}个场景—任务组合中，联合方法相对分阶段为{a['wins']}胜、{a['ties']}平、{a['losses']}负，"
             "在保持任务覆盖和最少停靠的条件下展现了广泛的J降低。"
             f"相对仅重排为{b['wins']}胜、{b['ties']}平、{b['losses']}负。")
    declines = [p for p in pairs if p["comparator"] == "reorder_only" and p["eligible"] and p["outcome"] == "loss"]
    if declines:
        cases = [batch.by_id[p["case_id"]] for p in declines]
        docks = sorted({int(num(case.rows[p["mode"], "joint_local"]["docks"])) for case, p in zip(cases, declines)})
        dock_note = "、".join(map(str, docks))
        if all(case.distribution == 2 for case in cases):
            first += f"退化均发生在长跨度分布的{dock_note}次停靠组，当前联合策略并非逐例优于仅重排。"
        else:
            first += f"退化组停靠数为{dock_note}次，当前联合策略并非逐例优于仅重排。"
    second = ""
    if diagnostic:
        second = (f"固定集合图重排达到局部收敛后，联合对该参照为{diagnostic['wins']}胜、"
                  f"{diagnostic['ties']}平、{diagnostic['losses']}负。"
                  "这进一步说明，仅用最多3轮B还不足以隔离纯选位贡献，主比较的收益包含优化充分程度的差异。")
    second += "下一步应优先改进联合初始化，以及区域替换与重排之间的预算分配，再用本批固定条件复验。"
    return ["", "## 研究判断", "", first, "", second, ""]


def main(folder, report_only=False):
    batch = Batch(folder)
    pairs = batch.pairs()
    aggregates = aggregate_pairs(pairs, len(batch.cases))
    missions = mission_counts(batch)
    if not report_only:
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
            "每场景保留16桌、36任务及原逐桌任务数量。每布局用D0参考锚点生成并冻结候选子池，"
            "供三类任务分布和全部方法共用；不同布局按真实地图生成各自位姿。", "",
            "## 对照与评价", "",
            "主比较固定三种方法：", "",
            "1. **分阶段+共同最少覆盖修复**（原始方法名 `minimum_seed`，主比较别名 `staged_minimum`）："
            "原 bottom_up 选位后，必要时统一修复到该场景的区域最少覆盖；不进行额外导航局部优化。",
            "2. **仅重排**（`reorder_only`）：固定同一个修复后的最少停靠位姿集合，只进行有向2-opt。",
            "3. **联合局部优化**（`joint_local`）：在相同最少覆盖层内替换区域停靠组合，并结合有向2-opt。", "",
            "原始 `staged` 贪心行作为辅助记录保留。主方法共用最少覆盖修复、有向导航代价及预设的有限闭环恢复，"
            "这些共同处理不计为联合优化收益。最少停靠数指当前有限子池和区域支持矩阵的证书。", "",
            "## 全部任务与最少停靠前提", "",
            f"主比较共 {all_primary} 个结果，其中 {primary_valid} 个满足完整任务与最少停靠前提；"
            f"联合方法满足该前提的比例为 {joint_valid}/{total_joint}。"
            "‘完整数值规划有效’指任务IK复核与底盘路线均通过。", "",
            "| 任务 | 方法 | 完整任务支持及复核 | 完整数值规划有效 | 数值规划有效且最少停靠 | 总场景 |",
            "|---|---|---:|---:|---:|---:|"]
    for row in missions:
        text.append(f"| {MODE_ZH[row['mode']]} | {METHOD_ZH[row['method']]} | {row['full_task_support_and_recheck']} | "
                    f"{row['full_missions']} | {row['full_minimum_missions']} | {row['total_scenarios']} |")
    text += ["", "## 配对导航代价是否稳定改善", "",
             "沿用 `J = L + 0.15 m/rad × Σ|wrap(相邻服务端点朝向差)|`。"
             "L为实际导出路径长度。"
             "主统计采用三种主方法共同有效的场景：三者均完成全部任务并达到相同有限子池最少停靠数，"
             "两种参照使用相同场景子集；其余结果保留在总分母，代价记N/A。", "",
             f"胜/平/负按原始J的绝对差 {TIE_TOL:g} m-equivalent 判定；百分比为 `100×(J参照−J联合)/J参照`。"
             "以下按逐场景配对百分比报告中位数、四分位区间和最差值。", "",
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
            text.append("该改善统计仅覆盖表中可比较组，未满足前提的场景见下表。")
        elif losses:
            text.append("存在负例，当前联合方法并非逐例优于该参照。")
        elif wins == len(valid):
            text.append("本批次全部预设配对均观察到严格加权代价降低。")
        else:
            text.append("本批次未观察到加权代价增加，其中包含持平组。")
    losses = [p for p in pairs if p["eligible"] and p["outcome"] == "loss"]
    if losses:
        text += ["", "### 负例与搜索预算", ""]
        accepted_rows = {}
        for pair in losses:
            case = batch.by_id[pair["case_id"]]
            joint = case.rows[pair["mode"], "joint_local"]
            move_note = ""
            move_path = case.directory/"accepted_moves.csv"
            if move_path.exists():
                if case.case_id not in accepted_rows:
                    accepted_rows[case.case_id] = read_csv(move_path, ("mode", "method", "operation"), allow_empty=True)
                    batch.sources.add(move_path)
                accepted = [row for row in accepted_rows[case.case_id]
                            if row["mode"] == pair["mode"] and row["method"] == "joint_local"]
                regional = sum(row["operation"] == "regional_minimum_cover_replacement" for row in accepted)
                reorder = sum(row["operation"] == "directed_two_opt" for row in accepted)
                move_note = f"接受改进中区域替换{regional}次、2-opt重排{reorder}次。"
            stop = joint.get("termination_reason", "")
            if "wall" in stop.lower():
                seconds = case.metadata.get("local_search_budget", {}).get("max_wall_time_s", 120)
                stop = f"达到{seconds:g}s搜索限额"
            text.append(f"- {case.case_id}／{MODE_ZH[pair['mode']]}：联合J={fmt(pair['joint_J'],3)}，"
                        f"{METHOD_ZH[pair['comparator']]}J={fmt(pair['baseline_J'],3)}，"
                        f"J增加{fmt(-pair['J_delta'],6)}（改善率{precise_pct(pair['J_gain_pct'])}）；"
                        f"两者完成{joint['verified_tasks']}/{joint['target_count']}，"
                        f"停靠{joint['docks']}次。联合搜索用时{fmt(joint.get('selection_time_s'),1)}s，"
                        f"{stop}，回调查询{joint.get('oracle_queries','')}次。{move_note}")
        text.append("这是同等覆盖和最少停靠下的导航负例，而非可行性失败。")
        text.extend(regional_neighborhood_section(batch))
    diagnostic_text, diagnostic_totals = reorder_diagnostic_section(batch, pairs)
    text.extend(diagnostic_text)
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
                        + " | ".join(costs)+f" | {minimum} |")
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
    text += ["", "该表单列共同最少覆盖修复的影响。", "",
             "## 可视化与复现", "",
             "`figures/robustness_gain_heatmaps` 展示全部布局×任务分布的两种配对J改善及有效/全部分母；"
             "`robustness_paired_costs` 保留逐场景点和胜/平/负；`robustness_mission_rates` 展示完整数值规划有效率及最少停靠条件。"
             "`robustness_scenario_maps` 展示全部真实地图和任务点，沿用灰桌、绿任务、红起点；"
             "固定代表场景 `L0_D2` 的三模式×三方法实际路线沿用蓝路径、橙停靠、蓝色底盘足迹/朝向。"
             "额外最差图按全部可比较joint-vs-reorder配对的最小J改善百分比选择，规则透明且允许负改善。", "",
             "```text", "python experiments/analyze_task_navigation_robustness.py",
             "python visualization/task_navigation_robustness/run_all.py", "```", "",
             "派生配对表为 `paired_gains.csv`，汇总为 `aggregate_statistics.csv` 和 `mission_success_counts.csv`；"
             "输入及图表来源见相应manifest。"]
    text.extend(research_judgment(batch, aggregates, pairs, diagnostic_totals))
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
    if not report_only:
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
    parser.add_argument("--report-only", action="store_true", help="Update narrative/manifest while preserving statistical CSVs and the table")
    args = parser.parse_args()
    main(args.folder, report_only=args.report_only)
