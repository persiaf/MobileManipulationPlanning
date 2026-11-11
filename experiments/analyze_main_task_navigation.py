"""Rebuild the full-main-scene analysis from failure-inclusive raw exports."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

MODES = ("position_only", "top_down_grasp", "table_wipe")
MODE_LABELS = dict(zip(MODES, ("仅位置约束", "朝下抓取", "桌面擦拭")))
METHODS = ("staged", "minimum_seed", "reorder_only", "joint_local")
METHOD_LABELS = dict(zip(METHODS, ("分阶段（共同导航修复）", "最少停靠种子", "仅重排", "联合局部优化")))


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def number(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return math.nan


def fmt(value, digits=3):
    value = number(value)
    return f"{value:.{digits}f}" if math.isfinite(value) else "N/A"


def gain(before, after):
    before, after = number(before), number(after)
    if not (math.isfinite(before) and math.isfinite(after) and before > 0):
        return "N/A"
    return f"{1-after/before:.2%}"


def valid(row):
    return row["mission_valid"] == "1"


def main(folder: Path):
    state = read_json(folder / "run_state.json")
    if state.get("status") != "COMPLETE" or state.get("completed_cases") != 12:
        raise RuntimeError("The twelve-case main-scene run is not complete; do not publish a partial report.")
    meta = read_json(folder / "metadata.json")
    raw = read_csv(folder / "summary.csv")
    rows = {(r["mode"], r["method"]): r for r in raw}
    expected = {(mode, method) for mode in MODES for method in METHODS}
    if len(raw) != 12 or set(rows) != expected:
        raise RuntimeError("Missing or duplicate mode/method cases.")
    proofs = {mode: read_json(folder / f"ik_cache_{mode}.json") for mode in MODES}
    docks = read_csv(folder / "docking_poses.csv")
    edges = read_csv(folder / "navigation_edges.csv")
    certificates = read_csv(folder / "minimum_cover_certificates.csv")
    validation_path = folder / "validation_summary.json"
    validation = read_json(validation_path) if validation_path.exists() else None
    test_note = (f"本轮 {validation['distinct_targeted_checks']} 项不同定向测试通过（具体分组与日志摘要见 `validation_summary.json`），不是全仓测试。"
                 if validation else "本输出目录没有定向测试摘要，本次分析不声明额外测试通过数量。")
    nvalid = sum(valid(r) for r in raw)
    text = [
        "# 默认主场景：任务约束与导航联合停靠实验", "",
        f"本轮使用 `scene_config()` 的真实默认主场景：{meta['table_count']} 张桌、"
        f"{meta['task_count']} 个原始任务、10 × 10 m 地图、seed 2026。桌子布局、完整任务集和起点均未更换。"
        "机械臂继续使用前两轮的可选 VX300s 六关节模型及相同 TCP/安装假设。", "",
        f"**完成情况：12 组中 {nvalid} 组任务与底盘路线均通过当前程序复核。** "
        "失败组保留在原始文件中，可比较的 L/J 为 N/A，不用零或部分路线冒充完整任务代价。", "",
        "## 固定条件与比较范围", "",
        f"默认场景生成 {meta['dense_candidate_count']:,} 个原始位姿，"
        f"其中 {meta['footprint_valid_dense_count']:,} 个通过统一底盘 footprint 检查。"
        f"为控制全六关节任务 IK 的计算量，在求解任何任务之前按几何规则冻结 {meta['candidate_count']} 个位姿："
        "每桌每侧最多 8 个 XY，保留两个已有的法向附近朝向，每桌最多 64 个。"
        "该子池不依据 IK 成败、任务模式或导航改善筛选；所有模式、方法使用同一子池。"
        "`dense_to_subpool.csv` 保留全部原始位姿及选择标志。", "",
        "最少停靠数由各桌二值任务支持矩阵的精确 DP 给出，仅对这个有限子池和当前数值 IK 证据成立。"
        "这不等于全原始候选池或连续空间中的最少停靠证书。各桌任务归属互不重叠，故区域最少数可相加。", "",
        "实验统一使用 A*、dense 服务节点路由矩阵、相同矩形底盘检查和返回起点条件。"
        "这是本实验的共同导航设置；原 `main_mobile_manipulation_planning` 的默认 `bottom_up`、"
        "`convex_astar` 与历史机器人配置均保持原样。", "",
        "四个对照：", "",
        "1. **分阶段（共同导航修复）**：原 bottom-up 停靠选择；四方法统一使用真实有向代价，最近邻闭环失败时按节点序号取首个有限重启闭环。",
        "2. **最少停靠种子**：必要时把原结果修复为各桌精确最少覆盖，单独展示修复代价，不进行导航搜索。",
        "3. **仅重排**：固定上述种子的全部停靠位姿，只做有向 2-opt。",
        "4. **联合局部优化**：在各桌最少覆盖层替换停靠组合，并结合有向 2-opt；只接受真实加权导航代价严格下降的完整路线。", "",
        "三个优化控制均从同一共同导航修复后的分阶段结果开始，经同一确定性最少覆盖修复。"
        "联合方法按需查询真实有向 A* 边并共享标量代价，不以欧氏直线代替绕桌路线。"
        "本轮是有预算的局部启发式，区别于前两桌实验的精确枚举。", "",
        "## 任务覆盖、停靠次数与导航结果", "",
        "目标为 `J = L + 0.15 m/rad × Σ |wrap(相邻服务端点朝向差)|`。"
        "L 是实际底盘路径长度，J 的朝向项不是轨迹累计转角、时间或能耗。", "",
        "| 任务 | 方法 | 支持/复核任务 | 停靠/子池最少 | L (m) | J | 完整任务有效 |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for mode in MODES:
        for method in METHODS:
            r = rows[mode, method]
            text.append(f"| {MODE_LABELS[mode]} | {METHOD_LABELS[method]} | "
                        f"{r['covered_tasks']}/{r['verified_tasks']} | {r['docks']}/{r['minimum_docks']} | "
                        f"{fmt(r['route_length_m'])} | {fmt(r['navigation_cost'])} | "
                        f"{'是' if valid(r) else '否：'+r['status']} |")
    if all(valid(rows[mode, "staged"]) and
           number(rows[mode, "staged"]["docks"]) == number(rows[mode, "staged"]["minimum_docks"])
           for mode in MODES):
        text += ["", "本轮三模式的分阶段方案已经达到子池最少停靠数，最少种子控制没有通过减少停靠数获得导航收益。"
                 "因此后续差异反映同等停靠次数下的路线重排与候选位姿联合调整。"]
    text += ["", "下表把共同修复后的分阶段、固定停靠重排与联合选位的改善分开；正值表示下降。", "",
             "| 任务 | 分阶段→联合 L 下降 | 分阶段→联合 J 下降 | 最少种子→仅重排 J 下降 | 仅重排→联合 L 下降 | 仅重排→联合 J 下降 |",
             "|---|---:|---:|---:|---:|---:|"]
    for mode in MODES:
        a, seed, b, c = (rows[mode, method] for method in METHODS)
        text.append(f"| {MODE_LABELS[mode]} | {gain(a['route_length_m'], c['route_length_m'])} | "
                    f"{gain(a['navigation_cost'], c['navigation_cost'])} | "
                    f"{gain(seed['navigation_cost'], b['navigation_cost'])} | "
                    f"{gain(b['route_length_m'], c['route_length_m'])} | "
                    f"{gain(b['navigation_cost'], c['navigation_cost'])} |")
    text += ["", "不同任务的路径和姿态要求同时变化，三模式不构成严格嵌套可行集，"
             "不能把模式间所有差异单独归因于姿态自由度。", "",
             "## 每个模式的观察", ""]
    for mode in MODES:
        a, seed, b, c = (rows[mode, method] for method in METHODS)
        p = proofs[mode]
        if not all(valid(r) for r in (a, seed, b, c)):
            observation = "存在完整任务无效的对照，保留失败原因，不作四方法完整任务成本优势结论。"
        else:
            bset = [r for r in docks if r["mode"] == mode and r["method"] == "reorder_only"]
            cset = [r for r in docks if r["mode"] == mode and r["method"] == "joint_local"]
            ids = {r["candidate_id"] for r in bset}
            pose_changes = sum(r["candidate_id"] not in ids for r in cset)
            xy = {(round(float(r["x_m"]), 8), round(float(r["y_m"]), 8)) for r in bset}
            xy_changes = sum((round(float(r["x_m"]), 8), round(float(r["y_m"]), 8)) not in xy for r in cset)
            observation = (f"共同修复后的分阶段 {a['docks']} 次、最少种子及联合方法 {c['docks']} 次停靠。"
                           f"联合相对该分阶段的 L/J 下降为 {gain(a['route_length_m'],c['route_length_m'])}/"
                           f"{gain(a['navigation_cost'],c['navigation_cost'])}；相对仅重排的 J 变化为下降 "
                           f"{gain(b['navigation_cost'],c['navigation_cost'])}。"
                           f"联合集合中有 {pose_changes} 个完整位姿不在仅重排集合内，其中 {xy_changes} 个 XY 不在其中。"
                           "集合和顺序共同变化，超出仅重排的收益不能解释为孤立选位主效应。")
        text.append(f"- **{MODE_LABELS[mode]}**：{p['supported_pairs']}/{p['pair_count']} 个同桌候选–任务配对"
                    f"有当前 IK 支持，任务并集 {p['union_supported_count']}/{meta['task_count']}。{observation}")
    text += ["", "## 搜索预算、证据缓存与计算开销", "",
             "| 任务 | 方法 | 搜索/选择时间 (s) | 搜索回调查询（含共享缓存命中） | 接受移动 | 停机原因 | 局部最优已证 |",
             "|---|---|---:|---:|---:|---|---|"]
    for mode in MODES:
        for method in METHODS:
            r = rows[mode, method]
            text.append(f"| {MODE_LABELS[mode]} | {METHOD_LABELS[method]} | {fmt(r['selection_time_s'])} | "
                        f"{r['oracle_queries']} | {r['accepted_moves']} | {r['termination_reason'] or '原选择器'} | "
                        f"{'是' if r['local_optimum_certified']=='1' else '否'} |")
    budget = meta["local_search_budget"]
    text += ["", f"局部搜索每次最多 {budget['max_passes']} 轮、{budget['max_navigation_queries']} 次外部 oracle 回调查询、"
             f"{budget['max_neighbor_evaluations']} 个邻居、{budget['max_wall_time_s']} s。"
             "预算或轮数用尽时返回最后一个已完整定价的合法解；这不是全局最优，也不一定是局部最优。"
             "仅完整无改善的一轮才可标记对应邻域的局部最优。预算对比是本次执行记录，不是等算力优势证明。", "",
             f"最终标量导航缓存含 {len(edges)} 条被需要的有向边，其中 "
             f"{sum(r['path_found']=='0' for r in edges)} 条路径被拒绝。被拒绝的碰撞/无路边计为 Inf，"
             "不允许作为候选捷径。实际最终路线由 Stage 3 重建并验证。", "",
             "| 任务 | IK 库构建 (s) | 正见证复用 | 精确负复用 | FK 回退 | 定期直接检查/分歧 |",
             "|---|---:|---:|---:|---:|---:|"]
    for mode in MODES:
        p = proofs[mode]
        text.append(f"| {MODE_LABELS[mode]} | {fmt(p['elapsed_time_s'])} | {p['positive_hits']} | "
                    f"{p['exact_negative_hits']} | {p['witness_fallbacks']} | "
                    f"{p['direct_checks']}/{p['direct_disagreements']} |")
    text += ["", "缓存量化仅用于检索桶。正命中使用当前未量化目标，逐样本 FK 检查残差、关节限位、接触和步长；"
             "负命中要求完整原始局部请求严格相同且失败类别允许复用，不能由近似失败推断物理不可达。"
             "有限搜索的再次失败或分歧按数值证据处理。每个最终计划还走现有程序的任务复核。"
             "这不是独立 IK 求解器验证。", "",
             "上表计时不包含几何候选预处理、共同 IK 库和最终 Stage 3 重建。真实导航边缓存跨方法/模式共享，"
             "执行顺序固定，各方法遇到的冷/热缓存不同。oracle_queries 包含跨方法缓存命中的回调，"
             "导航 memo 边数也不包含 Stage 3 dense 矩阵和拼接的其他实际查询，均不能解释为总 A* 调用数。"
             "本轮没有端到端加速、实时性或统计显著性结论。", "",
             "## 研究价值及结论边界", "",
             "本实验把任务级姿态/路径可行性传递到停靠覆盖，再把真实导航代价反馈到停靠组合选择。"
             "覆盖和最少停靠数是硬条件，导航优化不能通过少做任务或额外停靠换取更低代价。"
             "原分阶段、最少种子与固定集合重排对照使覆盖修复、访问顺序和联合选位的作用可分别检查。", "",
             "两桌精确枚举器可作小规模参照；本次主场景按需有向边查询与最少覆盖局部替换提供一个可继续研究的规模扩展。"
             "单个确定性场景的改善只能支持该主场景、子池和预算内的可行性与观察到的导航收益，"
             "不能证明普遍优于强基线、新颖性或规模复杂度。仍需候选密度、布局种子、任务方向与预算敏感性实验。", "",
             "VX300s 的安装点未经实测标定；未检查机械臂自碰撞、双臂互碰、桌面扫掠、任务间机械臂转移、"
             "连续采样间运动及接触力。任务支持/复核不是实机抓取或擦拭成功率。", "",
             "## 可视化、复现与审查", "",
             "图使用真实 16 桌主场景几何，并沿用主场景的灰桌、绿任务、蓝色实际路径、橙色停靠三角、红色起点样式。"
             "图中停靠按实际访问顺序编号；总览隐藏密集任务编号，详细原始分配保存在 CSV。"
             "`figures/` 提供 300 dpi PNG、矢量 PDF 与图注。", "",
             "```matlab", "addpath('pipeline'); init_paths;", "result = exp_main_task_navigation;", "```", "",
             "```text", "python experiments/audit_main_task_navigation.py",
             "python visualization/main_joint_task_navigation/run_all.py",
             "python experiments/analyze_main_task_navigation.py", "```", "",
             "主入口：`experiments/exp_main_task_navigation.m`；协议：`EXPERIMENT_PROTOCOL.md`。"
             "原始数据包含全部配对、候选来源、最少覆盖证书、导航边、实际路径、任务分配、接受移动及 MAT 快照。"
             "签名相同才复用已完成 IK 库，不改动历史两桌结果。", "",
             test_note + "独立 CSV/几何审计状态以 `independent_audit.json` 为准；"
             "审查记录见 `paper/review-traces/experiment-result-to-claim/2026-11-11_run03/`。"
             "缺少 manuscript-to-evidence 审计，论文主张仍为 provisional。", ""]
    if meta.get("navigation_matrix_directed"):
        text += ["## 初轮失败与共同导航修复", "",
                 "初轮使用历史 dense 矩阵的对称赋值，朝下抓取在服务节点 16→4、桌面擦拭在 17→4 拼接失败。"
                 "对抓取失败段的独立诊断确认两个端点 footprint 均合法，4→16 有有效路径，而相反方向的当前 A* 查询返回无效路径。"
                 "把有限的正向代价复制到反向使原排序使用了无效边；无效查询的零长度也不能作为免费可通行边。", "",
                 "最终四方法统一开启 `routing.directed_costs=true`，分别查询两个方向并核查成功标志，"
                 "无效边记为 Inf。仅纠正矩阵仍不能保证贪心最近邻闭环有效，因此同时开启 `routing.finite_nn_restarts=true`："
                 "起点 1 的闭环失败才依次尝试起点 2…N，取首个完整有限闭环并旋转回真实起点 1，"
                 "不比较多次尝试取最低代价，也不改变停靠集合。导航算法、碰撞标准、候选池和 IK 证据均不变。"
                 "该开关缺省关闭，历史主入口保持原配置。初轮结果完整保留在 "
                 f"`{meta.get('initial_trial_directory', 'results/main_joint_task_navigation_20261111_initial_symmetric')}`。", "",
                 "共同导航修复解决的可行性问题不能计作联合停靠选择的收益；"
                 "正文百分比均比较修复后同一导航协议下的四方法。"
                 "`frozen_library_reuse.json` 记录原始 IK 库与复用文件的哈希，保证没有为重跑重新筛选支持。", ""]
    failures = [r for r in raw if not valid(r)]
    if failures:
        text += ["## 保留的失败", ""]
        for r in failures:
            text.append(f"- {r['mode']}/{r['method']}: {r['status']}；{r['failure_reason']}。"
                        f"raw L/J={r['raw_route_length_m']}/{r['raw_navigation_cost']} 仅供排查。")
    (folder / "ANALYSIS.md").write_text("\n".join(text), encoding="utf-8")
    tex = [r"\begin{table*}[t]", r"\centering",
           r"\caption{Default 16-table scene with all 36 tasks and a shared 906-pose geometric subpool. "
           r"The minimum-dock certificate is conditional on that subpool; joint search is budgeted local descent. "
           r"Invalid full missions are reported as N/A.}",
           r"\label{tab:main-task-navigation}", r"\begin{tabular}{llrrrr}", r"\toprule",
           r"Task & Method & Verified & Docks & $L$ (m) & $J$ \\", r"\midrule"]
    for mode, label in zip(MODES, ("Position", "Downward grasp", "Table wipe")):
        for method, name in zip(METHODS, ("Staged", "Minimum seed", "Reorder only", "Joint local")):
            r = rows[mode, method]
            tex.append(f"{label} & {name} & {r['verified_tasks']}/36 & {r['docks']} & "
                       f"{fmt(r['route_length_m'])} & {fmt(r['navigation_cost'])} " + r"\\")
    tex += [r"\bottomrule", r"\end{tabular}", r"\end{table*}", ""]
    (folder / "comparison_table.tex").write_text("\n".join(tex), encoding="utf-8")
    print((folder / "ANALYSIS.md").resolve())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?", type=Path,
                        default=Path("results/main_joint_task_navigation_20261111"))
    main(parser.parse_args().folder)
