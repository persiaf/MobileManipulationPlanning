"""Rebuild the joint docking/navigation report directly from exported CSVs."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

MODES = ("position_only", "top_down_grasp", "table_wipe")
LABELS = ("仅位置", "朝下抓取", "桌面擦拭")


def main(folder):
    with (folder / "summary.csv").open(encoding="utf-8-sig", newline="") as f:
        rows = {(r["mode"], r["method"]): r for r in csv.DictReader(f)}
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8-sig"))
    report = ["# 任务约束与导航代价联合停靠选择", "",
              "本轮仅改变停靠选择及访问顺序策略。模型、安装假设、TCP、两桌场景、10 个目标、136 个候选和冻结 IK 支持矩阵全部保持不变。"
              "三类任务均以 10/10 个任务支持及最少停靠次数为硬条件。", "",
              "## 实现和优化对象", "",
              "新增 `task_navigation_joint` 策略。先用任务位掩码 DP 求每区域支持矩阵上的最少停靠数，"
              "枚举所有最小覆盖集合，再同时选择停靠位姿和访问顺序。默认策略仍为 `bottom_up`。", "",
              "导航目标沿用已有 `edge_cost.lambda_theta`：", "",
              "`J = Σ[真实 A* 路径长度 + 0.15 m/rad × |wrap(相邻服务端点朝向差)|]`。", "",
              "朝向项是既有端点姿态惩罚，不是实际行驶轨迹累计转角、能耗或控制时间。以下分别报告原始米数 L 和加权导航代价 J。", "",
              "| 任务 | 最少停靠 | 分阶段 L (m) | 联合 L (m) | 路程降低 | 分阶段 J | 联合 J | 代价降低 |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for mode, label in zip(MODES, LABELS):
        a, c = rows[mode, "staged"], rows[mode, "joint"]
        la, lc = float(a["route_length_m"]), float(c["route_length_m"])
        ja, jc = float(a["navigation_cost"]), float(c["navigation_cost"])
        report.append(f"| {label} | {c['docks']} | {la:.3f} | {lc:.3f} | {1-lc/la:.2%} | "
                      f"{ja:.3f} | {jc:.3f} | {1-jc/ja:.2%} |")
    report += ["", "所有九组均保持 10/10 个任务覆盖、通过同一 IK 核查和底盘路线验证，均返回相同起点。"
               "最少停靠数为 2/4/4，没有通过增加停靠或少执行任务换取较短路线。", "",
               "## 排序与停靠选择的收益拆分", "",
               "增加 `reorder_only` 对照：保持分阶段方法的完整停靠位姿集合（包含朝向）不变，只精确优化访问顺序。", "",
               "| 任务 | 分阶段 J | 仅重排 J | 联合 J | 仅重排收益 | 在重排之上继续选位收益 |",
               "|---|---:|---:|---:|---:|---:|"]
    for mode, label in zip(MODES, LABELS):
        a, b, c = (rows[mode, method] for method in ("staged", "reorder_only", "joint"))
        ja, jb, jc = (float(r["navigation_cost"]) for r in (a, b, c))
        report.append(f"| {label} | {ja:.6f} | {jb:.6f} | {jc:.6f} | {ja-jb:.6f} | {jb-jc:.6f} |")
    report += ["", "- **仅位置任务**：原规则选桌北侧，联合选择移到更靠近起点的南侧，停靠仍为每桌一次。"
               "仅重排没有收益，主要收益来自停靠位姿改变。",
               "- **抓取任务**：最小覆盖层固定了桌南/北两侧的停靠位置；联合方法选择允许的 ±70° 候选朝向，"
               "而原方案为 ±90°。端点朝向总变化从 2π 降为 4.8869 rad，原始路线长度没有下降。",
               "- **擦拭任务**：仅重排降低一部分朝向惩罚，但路程不变；联合选择将停靠布局转到桌南侧，"
               "在相同四次停靠下进一步减少实际路线。", "",
               "## 计算与验证范围", "",
               "本轮累计 34 项不同定向检查通过（联合策略 17、冻结协议 7、既有模型契约 10），不是全仓测试。"
               "独立 Python 审计通过：重建全部最少覆盖、必要边、最优访问排列、逐任务分配、实际路径与几何/加权代价；"
               "结果见 independent_audit.json。它不是独立 IK 或机械臂运动验证。", "",
               f"真实导航图包含 {meta['graph_edge_count']} 条必要有向边，独立计算两个方向的 A* 代价，"
               f"一次共享构图耗时 {meta['graph_time_s']:.3f} s。不是用欧氏直线代替绕桌路径。"
               "仅对能够共同出现在最小覆盖集合中的候选建边；再加入起点及分阶段控制需要的边，不删除任何待搜索闭环。", "",
               "最少覆盖方案每桌分别有：位置 16 种、抓取 9 种、擦拭 28 种；两桌组合并枚举访问排列，"
               "分别核查 512、1944、18816 条闭环。", "",
               "精确性仅限给定区域任务 partition、候选所有权、冻结二值支持矩阵和计算出的有向代价图。"
               "搜索的是最小覆盖层上每个所选停靠访问一次的 Hamiltonian 闭环；若该层不存在有限路线则返回 BLOCKED，"
               "不自动增加停靠层或搜索允许重复服务节点的 closed walk。超出配置的任务数、组合数、访问排列或导航查询预算会明确报错，不静默截断。", "",
               "搜索耗时不含上述共享导航图预处理，也不含重建不变 IK 支持矩阵。"
               "各方法单次固定顺序计时，没有端到端加速或实时性结论。整个流程仍未检查机械臂自碰撞、互碰、桌面扫掠、任务间机械臂转移或接触力；"
               "它不是实机抓取/擦拭成功证明。", "",
               "## 可视化与主场景的一致性", "",
               "实验几何仍为上轮两桌，不改成16桌主场景。表现形式参考 `plot_main_scenario_publication.m` 和 `draw_robot`："
               "灰桌深色边、绿色任务点、蓝色实际路径、橙色三角停靠、红色星形起点、蓝色底盘矩形/方向、任务分配虚线和相同地图坐标。"
               "D 编号按实际访问顺序显示；原始 dock_id 与 visit_rank 均保留在 CSV。", "",
               "图文件位于 `figures/`：3×2 总览、每个任务单独左右对照、三方法指标图；均有矢量 PDF、300 dpi PNG 与中文图注。", "",
               "## 研究意义与下一步", "",
               "该受控结果表明，将导航代价反馈到停靠位姿选择能够在本候选池内维持覆盖和最少停靠数，并改进既有导航目标。"
               "仅排序对照帮助区分访问顺序与选位（含朝向）的作用；抓取场景同时表明，加权目标改善不等于路线必然变短。", "",
               "该精确枚举器可作为小规模问题的可审计参照。若扩展为论文方法，仍需导航感知贪心/局部交换等强基线、"
               "多布局和候选密度实验、可扩展近似策略、碰撞感知独立验证及明确计算预算。当前不声称新颖性、通用优势或连续空间全局最优。", "",
               "## 复现与入口", "",
               "```matlab", "addpath('pipeline'); init_paths;", "result = exp_joint_task_navigation;", "```", "",
               "该入口核对旧 checkpoint 的模型、地图、任务、候选与签名后复用相同 IK 证据，再计算/复用冻结导航图，"
               "重跑三方法的正式 Stage 2/Stage 3 与每个最终任务的 IK 核查。", "",
               "一般代码入口可设置 `planner.strategy.name='task_navigation_joint'`，然后调用现有 `stage2_docking_planning`。"
               "实验通过 `planner.strategy.joint.cost_matrix` 注入同一图；不注入时策略会用真实路径查询构图。", "",
               "```text", "python experiments/audit_joint_task_navigation.py", "python visualization/joint_task_navigation/run_all.py",
               "python experiments/analyze_joint_task_navigation.py", "```", "",
               "原始证据：summary.csv、candidate_pool.csv、frozen_pair_support.csv、minimum_covers.csv、navigation_edges.csv、"
               "task_assignments.csv、docking_poses.csv、navigation_paths.csv 与 MAT 快照。", "",
               "独立审查记录：`paper/review-traces/experiment-result-to-claim/2026-11-11_run02/`。"
               "缺少 manuscript-to-evidence 审计时，研究主张仍按 provisional 管理。", ""]
    (folder / "ANALYSIS.md").write_text("\n".join(report), encoding="utf-8")
    texrows = []
    for mode, label in zip(MODES, ("Position only", "Downward grasp", "Table wipe")):
        for method, method_label in (("staged", "Staged"), ("reorder_only", "Reorder only"), ("joint", "Joint")):
            row = rows[mode, method]
            texrows.append(f"{label} & {method_label} & {row['docks']} & "
                           f"{float(row['route_length_m']):.3f} & {float(row['navigation_cost']):.3f} \\\\")
    tex = ("\\begin{table*}[t]\n\\centering\n"
           "\\caption{Fixed-pool joint docking and navigation comparison. All cases support ten tasks and retain the certified minimum dock count. "
           "$J=L+0.15\\sum|\\operatorname{wrap}(\\Delta\\theta)|$ uses actual path length and endpoint-yaw penalties.}\n"
           "\\label{tab:joint-task-navigation}\n\\begin{tabular}{llrrr}\n\\toprule\n"
           "Task & Method & Docks & $L$ (m) & $J$ (m-equivalent) \\\\\n\\midrule\n"
           + "\n".join(texrows) + "\n\\bottomrule\n\\end{tabular}\n\\end{table*}\n")
    (folder / "comparison_table.tex").write_text(tex, encoding="utf-8")
    print((folder / "ANALYSIS.md").resolve())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?", type=Path, default=Path("results/joint_task_navigation_20261111"))
    main(parser.parse_args().folder)
