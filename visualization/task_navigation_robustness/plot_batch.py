"""Failure-inclusive stability plots and actual main-style scenario maps."""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator
import numpy as np

from batch_common import (COMPARATORS, DISTRIBUTION_LABELS, LAYOUT_LABELS, METHODS, METHOD_COLORS, METHOD_LABELS, MODES, MODE_LABELS,
                          PATH_COLOR, REFERENCE, START_COLOR, TASK_EDGE, TASK_FACE, TIE_TOL,
                          aggregate_pairs, caption, draw_base, draw_map, integer, mission_counts,
                          num, save, style)


def panel_label(ax, label):
    ax.text(0, 1.035, label, transform=ax.transAxes, ha="left", va="bottom", fontsize=9.5)


def common_note(batch):
    return (f"本图覆盖预先声明的全部 {len(batch.cases)} 个场景，三种任务模式为仅位置、朝下抓取和桌面擦拭。"
            "主比较中的 Staged + minimum repair 对应原始导出的 minimum_seed，即分阶段选择后接受共同最少覆盖修复；"
            "原始 staged 行仅作辅助记录，不把非最少停靠的原始贪心结果悄悄当作等停靠对照。"
            "所有方法使用相同机械臂、同场景候选池、冻结任务支持矩阵、真实有向导航查询和共同有限闭环恢复。"
            "最少停靠证书只针对各场景共用的有限候选子池与区域任务归属。联合搜索有固定预算，未声称全局最优。")


def gain_heatmaps(batch, out, pairs):
    fig, axes = plt.subplots(3, 2, figsize=(11.5, 10.3), layout="constrained")
    finite = [abs(p["J_gain_pct"]) for p in pairs if p["eligible"]]
    limit = max(1.0, max(finite, default=1.0))
    norm = TwoSlopeNorm(vmin=-limit, vcenter=0, vmax=limit)
    cmap = plt.get_cmap("RdYlGn").copy()
    cmap.set_bad("#D6D6D6")
    for ri, mode in enumerate(MODES):
        for ci, comparator in enumerate(COMPARATORS):
            ax = axes[ri, ci]
            values = np.full((len(batch.layout_ids), len(batch.distribution_ids)), np.nan)
            counts = {}
            for y, layout in enumerate(batch.layout_ids):
                for x, distribution in enumerate(batch.distribution_ids):
                    cell = [p for p in pairs if p["mode"] == mode and p["comparator"] == comparator and
                            p["layout_id"] == layout and p["distribution_id"] == distribution]
                    valid = [p["J_gain_pct"] for p in cell if p["eligible"]]
                    counts[y, x] = (len(valid), len(cell))
                    if valid:
                        values[y, x] = np.median(valid)
            # QuadMesh keeps the PDF vector; imshow would embed a raster.
            im = ax.pcolormesh(np.arange(values.shape[1]+1)-.5, np.arange(values.shape[0]+1)-.5,
                               np.ma.masked_invalid(values), cmap=cmap, norm=norm,
                               shading="flat", edgecolors="white", linewidth=1.2, rasterized=False)
            ax.set(xlim=(-.5, values.shape[1]-.5), ylim=(values.shape[0]-.5, -.5), aspect="equal")
            ax.set_facecolor("#D6D6D6")
            for y in range(values.shape[0]):
                for x in range(values.shape[1]):
                    n, total = counts[y, x]
                    text = f"{values[y,x]:.1f}%\n{n}/{total}" if n else f"N/A\n{n}/{total}"
                    ax.text(x, y, text, ha="center", va="center", fontsize=9, color="#202020")
            ax.set_xticks(range(len(batch.distribution_ids)), [DISTRIBUTION_LABELS.get(v, batch.distribution_labels[v]) for v in batch.distribution_ids])
            ax.set_yticks(range(len(batch.layout_ids)), [LAYOUT_LABELS.get(v, batch.layout_labels[v]) for v in batch.layout_ids])
            ax.tick_params(length=0)
            ax.set_xticks(np.arange(-.5, len(batch.distribution_ids), 1), minor=True)
            ax.set_yticks(np.arange(-.5, len(batch.layout_ids), 1), minor=True)
            ax.grid(which="minor", color="white", linewidth=1.8)
            ax.tick_params(which="minor", length=0)
            if ri == 2:
                ax.set_xlabel("Task distribution")
            if ci == 0:
                ax.set_ylabel("Table layout")
            panel_label(ax, f"({chr(97+ri*2+ci)}) {MODE_LABELS[mode]} — vs {METHOD_LABELS[comparator]}")
    colorbar = fig.colorbar(im, ax=axes, shrink=.72, pad=.025, label="Joint navigation-cost reduction (%)")
    if colorbar.solids is not None:
        colorbar.solids.set_rasterized(False)
    outputs = save(fig, out, "robustness_gain_heatmaps")
    outputs.append(caption(out, "robustness_gain_heatmaps", "**图：联合方法在全部布局与任务分布中的配对导航代价变化。** "
                           "每行是一种任务约束，左列对比分阶段+共同最少覆盖修复，右列对比固定最少停靠集合的仅重排。"
                           "单元格为 100×(J对照−J联合)/J对照，正值表示降低，负值表示变差，色标以零居中。"
                           "每格第二行是可比较配对数/该格全部场景数；多种子时显示可比较百分比的中位数。"
                           "只有三种主方法均完成全部任务、通过路线检查且达到相同最少停靠数时，才同时进入两列配对，保证两种参照使用同一分母。"
                           "失败或不满足最少停靠前提的比较显示灰色 N/A，不按零改善处理，也不从总分母删除。"
                           "J为实际路径长度加0.15 m/rad乘服务端点朝向差绝对值，不是能耗或机器人执行时间。"+common_note(batch)))
    return outputs


def paired_costs(batch, out, pairs):
    fig, axes = plt.subplots(3, 2, figsize=(11.5, 12), layout="constrained")
    colors = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]
    markers = ["o", "s", "^", "D"]
    summaries = {(r["mode"], r["comparator"]): r for r in aggregate_pairs(pairs, len(batch.cases))}
    for ri, mode in enumerate(MODES):
        for ci, comparator in enumerate(COMPARATORS):
            ax = axes[ri, ci]
            rows = [p for p in pairs if p["mode"] == mode and p["comparator"] == comparator and p["eligible"]]
            costs = [p[key] for p in rows for key in ("baseline_J", "joint_J")]
            lo, hi = (min(costs), max(costs)) if costs else (0, 1)
            pad = max((hi-lo)*.10, .5)
            ax.plot([max(0, lo-pad), hi+pad], [max(0, lo-pad), hi+pad], color="#777777", linewidth=.8, linestyle="--", zorder=1)
            for p in rows:
                lc = batch.layout_ids.index(p["layout_id"])
                dc = batch.distribution_ids.index(p["distribution_id"])
                ax.scatter(p["baseline_J"], p["joint_J"], marker=markers[dc % len(markers)],
                           s=43, facecolor=colors[lc % len(colors)], edgecolor="#303030", linewidth=.45, zorder=3)
            ax.set(xlim=(max(0, lo-pad), hi+pad), ylim=(max(0, lo-pad), hi+pad), aspect="equal")
            ax.grid(color="#CCCCCC", linewidth=.5, alpha=.6)
            ax.set_xlabel("Reference navigation cost J")
            ax.set_ylabel("Joint navigation cost J")
            summary = summaries[mode, comparator]
            ax.text(.035, .965, f"win/tie/loss: {summary['wins']}/{summary['ties']}/{summary['losses']}\n"
                    f"eligible: {summary['eligible_pairs']}/{len(batch.cases)}", transform=ax.transAxes,
                    va="top", fontsize=8, bbox={"facecolor": "white", "alpha": .85, "edgecolor": "none", "pad": 2})
            panel_label(ax, f"({chr(97+ri*2+ci)}) {MODE_LABELS[mode]} — vs {METHOD_LABELS[comparator]}")
    handles = [Line2D([], [], marker="o", linestyle="none", markerfacecolor=colors[i % len(colors)],
                      markeredgecolor="#303030", label=batch.layout_labels[v]) for i, v in enumerate(batch.layout_ids)]
    handles += [Line2D([], [], marker=markers[i % len(markers)], linestyle="none", markerfacecolor="white",
                       markeredgecolor="#303030", label=batch.distribution_labels[v]) for i, v in enumerate(batch.distribution_ids)]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False)
    outputs = save(fig, out, "robustness_paired_costs")
    outputs.append(caption(out, "robustness_paired_costs", "**图：逐场景配对导航代价及胜/平/负。** "
                           "每个点所属场景要求三种主方法共同满足完整任务和相同最少停靠前提，两种参照使用同一场景子集；"
                           "同一场景的参照J为横坐标、联合J为纵坐标，"
                           "低于虚线表示联合代价较低。颜色对应布局，符号对应任务分布。"
                           f"胜/平/负以绝对J差{TIE_TOL:g} m-equivalent为容差，未使用百分比阈值。"
                           "图内始终注明有效配对数/全部场景数；未完成的配对不画假造成本点，其失败仍进入任务率分母与原始表。"
                           "这些点是预设因子组的描述性结果，不是独立同分布的统计样本，不绘制伪造置信区间。"+common_note(batch)))
    return outputs


def mission_rates(batch, out):
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.0), layout="constrained")
    counts = {(r["mode"], r["method"]): r for r in mission_counts(batch)}
    x, width = np.arange(3), .22
    fields = ("full_missions", "full_minimum_missions")
    labels = ("Full checked plans", "Full checked plans with minimum docks")
    for panel, (ax, field, label) in enumerate(zip(axes, fields, labels)):
        for mi, method in enumerate(METHODS):
            values = [counts[mode, method][field] for mode in MODES]
            bars = ax.bar(x+(mi-1)*width, values, width*.94, facecolor=METHOD_COLORS[method], edgecolor="#333333", linewidth=.45)
            for bar, value in zip(bars, values):
                ax.annotate(f"{value}/{len(batch.cases)}", (bar.get_x()+bar.get_width()/2, value),
                            xytext=(0, 3), textcoords="offset points", ha="center", va="bottom", fontsize=8)
        ax.set_xticks(x, [MODE_LABELS[v] for v in MODES])
        ax.set_ylim(0, len(batch.cases)*1.18)
        ax.set_ylabel("Scenario count / all scenarios")
        ax.yaxis.set_major_locator(MaxNLocator(integer=True, nbins=5))
        ax.set_axisbelow(True)
        ax.grid(axis="y", color="#CCCCCC", linewidth=.5)
        ax.spines[["top", "right"]].set_visible(False)
        panel_label(ax, f"({chr(97+panel)}) {label}")
    fig.legend(handles=[Patch(facecolor=METHOD_COLORS[m], edgecolor="#333333", label=METHOD_LABELS[m]) for m in METHODS],
               loc="outside lower center", ncol=3, frameon=False)
    outputs = save(fig, out, "robustness_mission_rates")
    outputs.append(caption(out, "robustness_mission_rates", "**图：完整数值规划有效率（任务IK复核+底盘路线）与最少停靠前提。** "
                           "左图计数要求任务支持覆盖及当前后端复核均完整、底盘路线有效、导出完整任务有效；右图另要求停靠数等于"
                           "该场景和模式的区域最少覆盖证书。每根柱标注成功场景数/预先声明全部场景数，失败保留在分母，"
                           "零有效率可以为零，但缺失导航成本从不当作零成本。任务复核是当前数值后端检查，不是实机完成率；"
                           "安装假设未经标定，机械臂自碰撞、互碰和桌面避碰仍未验证。"+common_note(batch)))
    return outputs


def draw_tasks_and_start(ax, case, *, compact=True):
    scene = case.scene()
    border = num(case.metadata.get("border_thickness_m", scene.get("border_thickness_m", 0)))
    draw_map(ax, scene, border)
    xy = np.asarray(list(case.tasks().values()))
    ax.scatter(xy[:, 0], xy[:, 1], s=18 if compact else 26, facecolor=TASK_FACE, edgecolor=TASK_EDGE, linewidth=.6, zorder=9)
    start = scene["start_pose"]
    ax.scatter(start[0], start[1], marker="*", s=95 if compact else 140, facecolor=START_COLOR,
               edgecolor="black", linewidth=.7, zorder=12)


def scenario_maps(batch, out, stem="robustness_scenario_maps", inputs_only=False):
    rows, columns = len(batch.layout_ids), len(batch.distribution_ids)
    fig, axes = plt.subplots(rows, columns, figsize=(4.25*columns, 4.5*rows+.55), squeeze=False, layout="constrained")
    for y, layout in enumerate(batch.layout_ids):
        for x, distribution in enumerate(batch.distribution_ids):
            ax = axes[y, x]
            cases = [c for c in batch.cases if c.layout == layout and c.distribution == distribution]
            if len(cases) != 1:
                raise ValueError("Scenario-map overview expects exactly one predeclared case per layout/distribution cell")
            case = cases[0]
            draw_tasks_and_start(ax, case)
            panel_label(ax, f"{case.case_id}: {case.layout_label} / {case.distribution_label}")
            if x:
                ax.set_ylabel("")
            if y < rows-1:
                ax.set_xlabel("")
    fig.legend(handles=[Patch(facecolor=REFERENCE.TABLE_FACE, edgecolor=REFERENCE.TABLE_EDGE, label="Table"),
                        Line2D([], [], marker="o", linestyle="none", markerfacecolor=TASK_FACE, markeredgecolor=TASK_EDGE, label="Task point"),
                        Line2D([], [], marker="*", markersize=11, linestyle="none", markerfacecolor=START_COLOR, markeredgecolor="black", label="Start")],
               loc="outside lower center", ncol=3, frameon=True, fancybox=False, edgecolor="#555555")
    outputs = save(fig, out, stem)
    outcome_note = ("本图仅展示已经冻结的真实输入；不读取任务成功、导航代价或优化结果，不能据此判断规划成功或改善。"
                    if inputs_only else common_note(batch))
    geometry_source = "case_definition.json中冻结scene_cfg" if inputs_only else "metadata.json"
    outputs.append(caption(out, stem, "**图：全部预设布局和任务分布的真实地图。** "
                           f"每个格子来自对应场景{geometry_source}的桌中心、实际桌子朝向、地图范围和tasks.csv原始任务点；"
                           "未为绘图移动、旋转或删减目标，旋转布局按真实table_orientations_rad绘制。"
                           "灰色为桌面、绿点为任务锚点、红星为共同起点，沿用主场景视觉编码。此图不显示机器人路径，"
                           "不以最优结果挑选地图，也不把任务锚点当作实体操作成功记录。"+outcome_note))
    return outputs


def route_grid(batch, out, case, stem, selection_note):
    records = case.routes()
    scene = case.scene()
    aspect = num(scene["map_height_m"])/num(scene["map_width_m"])
    fig, axes = plt.subplots(3, 3, figsize=(15, 3*(4.7*aspect+.65)+.55), squeeze=False, layout="constrained")
    outcome_lines = []
    for row, mode in enumerate(MODES):
        for col, method in enumerate(METHODS):
            ax = axes[row, col]
            record = records[mode, method]
            draw_tasks_and_start(ax, case)
            summary = record["summary"]
            if record["valid"]:
                xy = record["xy"]
                ax.plot(xy[:, 0], xy[:, 1], color=PATH_COLOR, linewidth=1.5, zorder=4)
            footprint = case.metadata["robot_footprint"]
            selected = record["docks"]
            shown = set(np.linspace(0, len(selected)-1, min(5, len(selected)), dtype=int)) if selected else set()
            for di, dock in enumerate(selected):
                pose = [num(dock[k]) for k in ("x_m", "y_m", "yaw_rad")]
                draw_base(ax, pose, num(footprint["length_m"]), num(footprint["width_m"]))
                if di in shown:
                    prefix = "D" if record["ranks"] else "S"
                    rank = integer(dock["visit_rank"] if record["ranks"] else dock["dock_id"])
                    ax.annotate(f"{prefix}{rank}", xy=pose[:2], xytext=(4, -6), textcoords="offset points", fontsize=6.2,
                                ha="left", va="top", color="#493014", zorder=11,
                                bbox={"facecolor": "white", "alpha": .8, "edgecolor": "none", "pad": .4})
            text = (f"{integer(summary['verified_tasks'])}/{integer(summary['target_count'])} checked | "
                    f"{integer(summary['docks'])} docks | {num(summary['route_length_m']):.2f} m" if record["valid"] else "NO VALID MINIMUM-DOCK MISSION")
            ax.text(.995, 1.025, text, transform=ax.transAxes, ha="right", va="bottom", fontsize=7,
                    color="#202020" if record["valid"] else "#A32121")
            ax.text(0, 1.085, f"({chr(97+row*3+col)}) {MODE_LABELS[mode]} — {METHOD_LABELS[method]}",
                    transform=ax.transAxes, ha="left", va="bottom", fontsize=8.3)
            if col:
                ax.set_ylabel("")
            if row < 2:
                ax.set_xlabel("")
            outcome_lines.append(f"{mode}/{method}: " + (f"L={num(summary['route_length_m']):.6f}m, J={num(summary['navigation_cost']):.6f}, "
                f"docks={summary['docks']}, minimum={summary['minimum_docks']}" if record["valid"] else
                f"N/A ({summary.get('status','')}; {summary.get('failure_reason','')})"))
    fig.legend(handles=main_scene_legend(), loc="outside lower center", ncol=3, frameon=True,
               fancybox=False, edgecolor="#555555")
    outputs = save(fig, out, stem)
    outputs.append(caption(out, stem, f"**图：{case.case_id}的三方法实际路线对照。** {selection_note} "
                           f"本图使用真实{case.layout_label}布局和{case.distribution_label}任务分布，各行依次为三种任务约束，"
                           "各列依次为分阶段+共同最少覆盖修复、仅重排和联合局部搜索。灰桌、绿任务点、蓝色实际路径、橙色停靠三角、"
                           "红色起点以及蓝色矩形底盘足迹/朝向沿用主场景；少量D编号按真实visit_rank显示，省略其余标签不删停靠。"
                           "蓝线直接来自navigation_paths.csv的有序样本，并按导出L复算，不连接停靠中心伪造路径。"
                           "失败或未达最少停靠前提时不绘制完整任务路径、不补齐未导出的位姿，标N/A；保留真实地图和任务点。"
                           +common_note(batch)+"\n\n"+"\n\n".join(outcome_lines)))
    return outputs


def main_scene_legend():
    return REFERENCE.scene_legend()[:-1]


def generate(batch, out, representative_case):
    style()
    pairs = batch.pairs()
    outputs = []
    for create in (gain_heatmaps, paired_costs):
        outputs.extend(create(batch, out, pairs))
    outputs.extend(mission_rates(batch, out))
    outputs.extend(scenario_maps(batch, out))
    if representative_case not in batch.by_id:
        raise ValueError(f"Predeclared representative scenario {representative_case!r} is missing")
    selection_note = ("代表场景在查看结果前固定为原布局的spread分布（L0_D2）；不是按最大改善挑选。"
                      if representative_case == "L0_D2" else
                      f"本图由--representative-case显式指定为{representative_case}，未由程序按最大改善挑选；该覆盖参数不是默认预设代表场景。")
    outputs.extend(route_grid(batch, out, batch.by_id[representative_case], "robustness_representative_routes", selection_note))
    candidates = [p for p in pairs if p["comparator"] == "reorder_only" and p["eligible"]]
    worst = min(candidates, key=lambda p: p["J_gain_pct"]) if candidates else None
    worst_case = worst["case_id"] if worst else None
    if worst and worst_case != representative_case:
        outputs.extend(route_grid(batch, out, batch.by_id[worst_case], "robustness_worst_routes",
                                  f"按预先声明规则，取全部可比较joint-vs-reorder配对中J下降百分比最小的场景；"
                                  f"本次由{worst['mode']}触发，下降{worst['J_gain_pct']:.6f}%。相同比例按manifest/任务模式顺序取首项。"))
    return outputs, {"representative_case": representative_case, "worst_eligible_pair": worst,
                     "worst_same_as_representative": worst_case == representative_case}
