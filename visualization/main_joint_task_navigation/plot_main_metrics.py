"""Summarize available main-scene methods without assuming exact optimality."""
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator
import numpy as np

from main_common import (METHOD_COLORS, METHOD_LABELS, MODE_LABELS, Experiment, arguments,
                         boolean, caption, integer, label_panel, number, output_directory,
                         save, style, valid_mission)


def generate(data_dir: Path, output_dir: Path | None = None, details=False):
    style()
    experiment = Experiment(data_dir)
    records = experiment.summaries()
    out = output_directory(data_dir, output_dir)
    time_field = "search_time_s" if all("search_time_s" in row for row in records.values()) else "selection_time_s"
    if not all(time_field in row for row in records.values()):
        raise ValueError("summary.csv must export search_time_s or selection_time_s for every result")
    specs = [("route_length_m", "Navigation length (m)", ".2f"),
             ("navigation_cost", "Navigation cost (m-equivalent)", ".2f"),
             ("docks", "Selected docks", ".0f"),
             (time_field, "Selection / search time (s)", ".2f")]
    fig, axes = plt.subplots(2, 2, figsize=(11.3, 7.0), layout="constrained")
    x = np.arange(len(experiment.modes))
    width = 0.72/len(experiment.methods)
    patterns = {"staged": "", "minimum_seed": "xx", "reorder_only": "//", "joint_local": "..", "joint": ".."}
    for panel, (ax, (field, ylabel, fmt)) in enumerate(zip(axes.flat, specs)):
        all_values = []
        for column, method in enumerate(experiment.methods):
            valid_modes = [index for index, mode in enumerate(experiment.modes)
                           if valid_mission(records[(mode, method)])]
            values = np.array([number(records[(experiment.modes[index], method)][field], field) for index in valid_modes])
            if (values < 0).any():
                raise ValueError(f"summary.csv: {field} cannot be negative")
            all_values.extend(values)
            positions = x[valid_modes]+(column-(len(experiment.methods)-1)/2)*width
            bars = ax.bar(positions, values, width=width*0.9, color=METHOD_COLORS[method],
                          edgecolor="#333333", linewidth=0.45, hatch=patterns[method])
            for bar, value in zip(bars, values):
                ax.annotate(format(value, fmt),
                            (bar.get_x()+bar.get_width()/2, value), xytext=(0, 3), textcoords="offset points",
                            ha="center", va="bottom", fontsize=7.5)
            for index in set(range(len(experiment.modes)))-set(valid_modes):
                ax.annotate("N/A", (x[index]+(column-(len(experiment.methods)-1)/2)*width, 0),
                            xytext=(0, 3), textcoords="offset points", ha="center", va="bottom",
                            rotation=90, fontsize=6.5, color="#A32121")
        ax.set_xticks(x, [MODE_LABELS[mode] for mode in experiment.modes])
        ax.tick_params(axis="x", length=0, pad=5)
        ax.set_ylabel(ylabel)
        ax.set_ylim(0, max(max(all_values)*1.25 if all_values else 1, 1e-6))
        ax.set_axisbelow(True)
        ax.grid(axis="y", color="#D4D4D4", alpha=0.65, linewidth=0.55)
        ax.spines[["top", "right"]].set_visible(False)
        ax.yaxis.set_major_locator(MaxNLocator(integer=field == "docks", nbins=5, min_n_ticks=2))
        label_panel(ax, f"({chr(97+panel)})")
    fig.legend(handles=[Patch(facecolor=METHOD_COLORS[method], edgecolor="#333333", hatch=patterns[method],
                              label=METHOD_LABELS[method]) for method in experiment.methods],
               loc="outside lower center", ncol=len(experiment.methods), frameon=False)
    outputs = save(fig, out, "main_joint_metrics")
    entries = []
    for mode in experiment.modes:
        lines = []
        for method in experiment.methods:
            row = records[(mode, method)]
            if not valid_mission(row):
                lines.append(f"{METHOD_LABELS[method]}：覆盖 {integer(row['covered_tasks'])}/{integer(row['target_count'])}，"
                             f"停靠 {integer(row['docks'])}，NO VALID MISSION；该方法对应柱形留空，不将 NaN 当作零")
                continue
            lines.append(f"{METHOD_LABELS[method]}：覆盖 {integer(row['covered_tasks'])}/{integer(row['target_count'])}，"
                         f"停靠 {integer(row['docks'])}，距离 {number(row['route_length_m']):.3f} m，"
                         f"加权代价 {number(row['navigation_cost']):.3f} m-equivalent，{time_field}={number(row[time_field]):.3f} s，"
                         f"route_valid={row['route_valid']}")
        entries.append(f"{MODE_LABELS[mode]}："+"；".join(lines))
    weight = experiment.metadata.get("lambda_theta_m_per_rad")
    cost_note = (f"加权代价为实际距离加 {number(weight):g} m/rad 乘以相邻访问端点的 wrapped 朝向差绝对值总和；"
                 "朝向项不是沿实际路径逐点转角的积分。" if weight is not None else
                 "加权导航代价的具体定义以 metadata.json 和实验协议为准。")
    text = ("**图：真实主场景的导航结果与搜索开销。** 仅展示 summary.csv 中实际完成的方法组合，未为缺失方法填充数据。"
            "Staged 为原 bottom_up 加共同导航修复；所有方法统一使用有向代价/有效性检查和有限闭环的 NN 起点重试恢复，"
            "这两项共同修复不计为 joint_local 收益。初轮失败保留在独立归档，不混入本图比较。"
            "(a) 原始路线距离；(b) 加权导航代价，两者使用独立单位的纵轴。"+cost_note+
            f"(c) 停靠数量；(d) {time_field}，该阶段耗时不能解释为端到端总规划时间或机器人执行时间，"
            "构图、路径查询和最终路径生成的计时范围以原始结果和协议为准。minimum_seed 是对 baseline 停靠覆盖种子的显式修复；"
            "若存在 reorder_only，它仅对修复后的最少停靠种子重排访问顺序，用于分离重排与联合选择的影响。"
            "joint_local 为有限预算局部搜索，不保证固定候选池全局最优；是否达到最少停靠数以 minimum_seed 的独立覆盖证书为准，"
            "局部搜索本身不提供额外的最少停靠证明。所有结果来自一次运行，"
            "不绘制未观测的误差条，不声称统计显著性或普遍改进。覆盖不足或导航失败时，该方法四个面板的柱形均留空并标 N/A，"
            "缺失的 NaN 代价不按零处理，原始失败记录仍保留。\n\n"+
            "\n\n".join(entries)+"。\n\n数据：summary.csv、metadata.json。")
    outputs.append(caption(out, "main_joint_metrics", text))
    return outputs, experiment.sources


if __name__ == "__main__":
    args = arguments(__doc__)
    paths, _ = generate(args.data_dir, args.output_dir, args.details)
    for path in paths:
        print(path.resolve())
