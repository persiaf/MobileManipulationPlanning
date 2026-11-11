"""Separate visit-order effects from joint dock-selection/navigation effects."""
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator
import numpy as np

from common import (METHODS, METHOD_COLORS, METHOD_LABELS, MODE_LABELS, MODE_TICKS,
                    Experiment, arguments, boolean, caption, integer, label_panel,
                    number, output_directory, save, style)


def generate(data_dir: Path, output_dir: Path | None = None):
    style()
    experiment = Experiment(data_dir)
    out = output_directory(data_dir, output_dir)
    records = experiment.summaries()
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 6.55), layout="constrained")
    specifications = [
        ("route_length_m", "Navigation length (m)", ".2f"),
        ("navigation_cost", "Navigation cost (m-equivalent)", ".2f"),
        ("docks", "Selected docks", ".0f"),
        ("search_time_s", "Selection / search time (s)", ".2f"),
    ]
    positions = np.arange(len(experiment.modes))
    bar_width = 0.235
    hatches = {"staged": "", "reorder_only": "//", "joint": ".."}
    for panel, (ax, (key, ylabel, fmt)) in enumerate(zip(axes.flat, specifications)):
        all_values = []
        for method_index, method in enumerate(METHODS):
            values = np.array([number(records[(mode, method)][key], key) for mode in experiment.modes])
            if (values < 0).any():
                raise ValueError(f"summary.csv: {key} cannot be negative")
            if key == "docks" and not np.equal(values, values.astype(int)).all():
                raise ValueError("summary.csv: docks must be integer counts")
            all_values.extend(values)
            x = positions + (method_index-1)*bar_width
            bars = ax.bar(x, values, width=bar_width*0.92, color=METHOD_COLORS[method],
                          edgecolor="#333333", linewidth=0.45, hatch=hatches[method])
            for row, (bar, value) in enumerate(zip(bars, values)):
                invalid_route = panel < 2 and not boolean(records[(experiment.modes[row], method)]["route_valid"])
                ax.annotate(format(value, fmt) + ("*" if invalid_route else ""),
                            (bar.get_x()+bar.get_width()/2, value), xytext=(0, 3),
                            textcoords="offset points", ha="center", va="bottom", fontsize=7.2)
        ax.set_xticks(positions, [MODE_TICKS[mode] for mode in experiment.modes])
        ax.tick_params(axis="x", length=0, pad=5)
        ax.set_ylabel(ylabel)
        ax.set_ylim(0, max(max(all_values)*1.25, 1e-6))
        ax.set_axisbelow(True)
        ax.grid(axis="y", color="#D4D4D4", alpha=0.65, linewidth=0.55)
        ax.spines[["top", "right"]].set_visible(False)
        label_panel(ax, f"({chr(97+panel)})")
        if key == "docks":
            ax.yaxis.set_major_locator(MaxNLocator(integer=True, min_n_ticks=2))
        else:
            ax.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
    handles = [Patch(facecolor=METHOD_COLORS[m], edgecolor="#333333", linewidth=0.45,
                     hatch=hatches[m], label=METHOD_LABELS[m]) for m in METHODS]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False)
    outputs = save(fig, out, "joint_navigation_metrics")
    details = []
    for mode in experiment.modes:
        results = []
        for method in METHODS:
            row = records[(mode, method)]
            results.append(f"{METHOD_LABELS[method]}：覆盖 {integer(row['covered_tasks'])}/{integer(row['target_count'])}，"
                           f"停靠 {integer(row['docks'])}，路线 {number(row['route_length_m']):.3f} m，"
                           f"加权导航代价 {number(row['navigation_cost']):.3f} m-equivalent，"
                           f"估计导航时间 {number(row['nav_estimated_time_s']):.3f} s，选点/优化时间 {number(row['selection_time_s']):.3f} s，"
                           f"graph_time_s={number(row['graph_time_s']):.3f}、search_time_s={number(row['search_time_s']):.3f}，"
                           f"exact_on_candidate_pool={row['exact_on_candidate_pool']}，exact_minimum_docks={row['exact_minimum_docks']}，"
                           f"route_cost_optimal={row['route_cost_optimal']}，route_valid={row['route_valid']}")
        details.append(f"{MODE_LABELS[mode]}：" + "；".join(results))
    graph_times = sorted({number(row["graph_time_s"], "graph_time_s") for row in records.values()})
    graph_note = "、".join(f"{value:.3f}" for value in graph_times)
    yaw_weight = number(experiment.metadata["lambda_theta_m_per_rad"], "lambda_theta_m_per_rad")
    shared_graph = boolean(experiment.metadata["graph_preprocessing_shared"], "graph_preprocessing_shared")
    if yaw_weight < 0:
        raise ValueError("lambda_theta_m_per_rad must be nonnegative")
    graph_scope = "图在方法间复用且只构建一次" if shared_graph else "具体构图次数和归属以实验协议为准"
    text = ("**图：访问顺序重排与联合选择的收益及搜索耗时。** 三种方法分别为 staged、reorder_only 和 joint。"
            "(a) 实际导航路线长度，单位 m；(b) 加权导航代价，单位 m-equivalent，定义为实际路线长度加 "
            f"{yaw_weight:g} m/rad 乘以访问序列相邻端点的 wrapped 朝向差绝对值总和，端点包含起止底盘位姿；"
            "该朝向项不是沿导航路径逐点转角的积分。加权代价与原始距离分别放在独立纵轴，不能混为同一物理量。"
            "(c) 所选停靠位姿数量；(d) summary.csv 的 search_time_s，即本次选点/访问序列搜索阶段的耗时，单位 s。"
            f"导航图预处理记录为 {graph_note} s，{graph_scope}，不包含在 (d) 中；最终导航路径生成时间也另行记录。"
            "因此 (d) 不能称为总规划开销，也不能用于宣称联合方法端到端更快。"
            "reorder_only 用于区分重排访问顺序与联合改变停靠选择的影响，具体算法与计时范围以实验协议为准。"
            "每个纵轴仅采用一种量纲，柱形从零开始；不同方法同时使用颜色和纹理区分。所有数据来自一次导出运行，"
            "没有重复运行方差，因此不绘制误差条、不声称统计显著性或普遍性能提升。路线校验失败时，相关柱形数值后加星号，"
            "不能视为有效导航结果。候选池最优性和最少停靠结论仅由导出的 exact_on_candidate_pool、"
            "exact_minimum_docks、route_cost_optimal 字段及独立审计支持，不能由柱形高低直接推出。\n\n" +
            "\n\n".join(details) + "。\n\n数据：summary.csv、metadata.json。")
    outputs.append(caption(out, "joint_navigation_metrics", text))
    return outputs, experiment.sources


if __name__ == "__main__":
    args = arguments(__doc__)
    paths, _ = generate(args.data_dir, args.output_dir)
    for path in paths:
        print(path.resolve())
