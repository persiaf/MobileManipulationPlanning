"""Plot task support, selected docks, navigation length and scoring cost."""
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator, PercentFormatter
import numpy as np

from common import (COLORS, LABELS, SHORT_LABELS, Experiment, apply_style, arguments,
                    boolean, integer, number, output_directory, panel_label,
                    save_figure, write_caption)


def generate(data_dir: Path, output_dir: Path | None = None):
    apply_style()
    experiment = Experiment(data_dir)
    out = output_directory(data_dir, output_dir)
    columns = ("mode", "target_count", "union_supported_count", "supported_pairs", "pair_count",
               "selected_docks", "verified_tasks", "route_valid", "route_length_m",
               "nav_estimated_time_s", "scoring_time_s")
    raw = experiment.rows("summary.csv", columns)
    records = {}
    for row in raw:
        mode = row["mode"]
        if mode in records:
            raise ValueError(f"summary.csv: duplicate mode {mode}")
        records[mode] = row
    if set(records) != set(experiment.modes):
        raise ValueError("summary.csv: modes differ from metadata.mode_order")
    rows = [records[mode] for mode in experiment.modes]
    pairs = np.array([integer(row["supported_pairs"], "supported_pairs") for row in rows])
    pair_totals = np.array([integer(row["pair_count"], "pair_count") for row in rows])
    if (pair_totals <= 0).any() or (pairs < 0).any() or (pairs > pair_totals).any():
        raise ValueError("summary.csv: invalid candidate-target pair support counts")
    proportions = pairs / pair_totals
    docks = np.array([integer(row["selected_docks"], "selected_docks") for row in rows])
    route_length = np.array([number(row["route_length_m"], "route_length_m") for row in rows])
    scoring_time = np.array([number(row["scoring_time_s"], "scoring_time_s") for row in rows])
    route_valid = [boolean(row["route_valid"], "route_valid") for row in rows]
    if (docks < 0).any() or (route_length < 0).any() or (scoring_time < 0).any():
        raise ValueError("summary.csv: dock counts, route length and scoring time must be nonnegative")
    fig, axes = plt.subplots(1, 4, figsize=(11.4, 3.0), layout="constrained")
    specifications = [
        (proportions, "Supported pairs (%)", ".1%"),
        (docks, "Selected docks", ".0f"),
        (route_length, "Navigation length (m)", ".2f"),
        (scoring_time, "Scoring time (s)", ".2f"),
    ]
    x = np.arange(len(experiment.modes))
    for index, (ax, (values, ylabel, fmt)) in enumerate(zip(axes, specifications)):
        bars = ax.bar(x, values, width=0.60, color=[COLORS[m] for m in experiment.modes],
                      edgecolor="#333333", linewidth=0.45)
        for bar_index, (bar, value) in enumerate(zip(bars, values)):
            if index == 2 and not route_valid[bar_index]:
                bar.set_hatch("///")
            label = format(value, fmt)
            if index == 2 and not route_valid[bar_index]:
                label += "*"
            ax.annotate(label, (bar.get_x() + bar.get_width() / 2, value), xytext=(0, 3),
                        textcoords="offset points", ha="center", va="bottom", fontsize=8)
        ax.set_xticks(x, [SHORT_LABELS[m] for m in experiment.modes])
        ax.tick_params(axis="x", length=0)
        ax.set_ylabel(ylabel)
        ax.set_ylim(0, max(float(np.max(values)) * 1.26, 1e-6))
        ax.set_axisbelow(True)
        ax.grid(axis="y", color="#DDE0E2", linewidth=0.45)
        panel_label(ax, chr(ord("a") + index))
        if index == 0:
            ax.yaxis.set_major_formatter(PercentFormatter(1))
            ax.set_ylim(0, min(1.15, max(float(np.max(values)) * 1.26, 0.10)))
        elif index == 1:
            ax.yaxis.set_major_locator(MaxNLocator(integer=True, min_n_ticks=2))
        else:
            ax.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
    paths = save_figure(fig, out, "comparison_metrics")
    numbers = []
    for mode, row, ratio, valid in zip(experiment.modes, rows, proportions, route_valid):
        numbers.append(f"{LABELS[mode]}：支持配对 {integer(row['supported_pairs'])}/{integer(row['pair_count'])}"
                       f"（{ratio:.2%}），任务并集支持 {integer(row['union_supported_count'])}/{integer(row['target_count'])}，"
                       f"所选停靠 {integer(row['selected_docks'])}，verified_tasks={integer(row['verified_tasks'])}，"
                       f"路线 {number(row['route_length_m']):.3f} m，评分耗时 {number(row['scoring_time_s']):.3f} s，"
                       f"route_valid={str(valid).lower()}")
    caption = """**图：同场景任务约束比较的量化结果。** (a) 候选—任务配对支持比例，分母为 summary.csv 中各模式实际评估的 pair_count；候选若只评估其所属桌任务，分母不等于候选数乘以全场任务数。该指标不同于任务并集覆盖率。(b) 所选停靠位姿数量。(c) 实际导出导航路线的长度，单位为 m。(d) 候选任务支持评分的实测耗时，单位为 s，不能解释为机器人执行时长。每个坐标轴只有一种量纲，所有柱形从零开始。柱形为该次导出运行的观测值，没有重复种子统计，因此不绘制误差条、不声称统计显著性或普遍改进。若 route_valid=false，对应路线柱使用斜线和星号标识，数值不应作为有效导航结果。\n\n""" + "；".join(numbers) + "。verified_tasks 为当前导出校验流程的结果，不能解释为实机完成数。任务特有路径和约束同时变化时，(a) 不能单独归因于姿态；姿态影响请结合 workspace_support 图的同路径消融下排判断。\n\n数据：summary.csv、metadata.json。"
    paths.append(write_caption(out, "comparison_metrics", caption))
    return paths, experiment.sources


if __name__ == "__main__":
    args = arguments(__doc__)
    generated, _ = generate(args.data_dir, args.output_dir)
    for path in generated:
        print(path)
