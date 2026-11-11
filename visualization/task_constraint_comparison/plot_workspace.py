"""Plot sampled workspace support and the matched-path orientation ablation."""
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from common import (COLORS, LABELS, Experiment, apply_style, arguments, draw_base,
                    draw_tables, output_directory, panel_label, save_figure, write_caption)


def _workspace(experiment, filename):
    columns = ("x_m", "y_m", "z_m", *experiment.modes)
    data = experiment.numeric(filename, columns)
    if not np.isin(data[:, 3:], [0, 1]).all():
        raise ValueError(f"{filename}: support observations must be binary")
    return data


def _projection(data, column):
    # Explicit union projection when a future export includes multiple heights.
    coordinates, inverse = np.unique(data[:, :2], axis=0, return_inverse=True)
    support = np.zeros(len(coordinates), dtype=bool)
    np.logical_or.at(support, inverse, data[:, column].astype(bool))
    return coordinates, support


def generate(data_dir: Path, output_dir: Path | None = None):
    apply_style()
    experiment = Experiment(data_dir)
    out = output_directory(data_dir, output_dir)
    top = _workspace(experiment, "workspace.csv")
    bottom = _workspace(experiment, "matched_path_workspace.csv")
    scene = experiment.scene()
    length, width, footprint_source = experiment.footprint()
    dock = experiment.metadata["reference_dock"]
    half_x = (abs(np.cos(dock[2])) * length + abs(np.sin(dock[2])) * width) / 2
    half_y = (abs(np.sin(dock[2])) * length + abs(np.cos(dock[2])) * width) / 2
    base_extent = np.array([[dock[0]-half_x, dock[1]-half_y], [dock[0]+half_x, dock[1]+half_y]])
    all_xy = np.concatenate([top[:, :2], bottom[:, :2], base_extent])
    lo, hi = all_xy.min(axis=0) - 0.13, all_xy.max(axis=0) + 0.13
    extent = hi - lo
    # Equal display scales across all six panels; no rescaling by method.
    fig, axes = plt.subplots(2, 3, figsize=(10.7, 6.5), sharex=True, sharey=True,
                             layout="constrained")
    matched_labels = {"position_only": "Position only", "top_down_grasp": "Downward axis", "table_wipe": "Full pose"}
    counts = []
    for row, data in enumerate((top, bottom)):
        row_counts = {}
        unique_xy = np.unique(data[:, :2], axis=0)
        area_per_point = np.prod(extent) / max(1, len(unique_xy))
        marker_size = float(np.clip(450 * area_per_point, 4, 22))
        for col, mode in enumerate(experiment.modes):
            ax = axes[row, col]
            coordinates, supported = _projection(data, col + 3)
            draw_tables(ax, scene)
            ax.scatter(coordinates[:, 0], coordinates[:, 1], s=marker_size, marker="s",
                       color="#DFE2E4", edgecolors="none", zorder=2)
            ax.scatter(coordinates[supported, 0], coordinates[supported, 1], s=marker_size,
                       marker="s", color=COLORS[mode], edgecolors="none", zorder=3)
            draw_base(ax, dock, length, width, alpha=0.09, zorder=5)
            ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]))
            ax.set_aspect("equal", adjustable="box")
            ax.set_xlabel("X (m)" if row else "")
            ax.set_ylabel("Y (m)" if col == 0 else "")
            ax.tick_params(labelbottom=bool(row))
            label = LABELS[mode] if row == 0 else matched_labels[mode]
            panel_label(ax, chr(ord("a") + row * 3 + col), label)
            count = int(data[:, col + 3].sum())
            row_counts[mode] = (count, len(data))
            ax.text(0.985, 0.025, f"{count}/{len(data)} anchors", transform=ax.transAxes,
                    ha="right", va="bottom", fontsize=8,
                    bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none", "pad": 2})
        counts.append(row_counts)
    legend = [Line2D([], [], marker="s", linestyle="none", color="#90979A", label="No IK sequence found"),
              Line2D([], [], marker="s", linestyle="none", color="#333333", label="Supported sample"),
              Line2D([], [], color="#333333", linewidth=1.1, label="Reference base pose")]
    fig.legend(handles=legend, loc="outside lower center", ncol=3, frameon=False)
    paths = save_figure(fig, out, "workspace_support")
    z_top, z_bottom = np.unique(top[:, 2]), np.unique(bottom[:, 2])
    height_note = []
    for label, heights in (("上排", z_top), ("下排", z_bottom)):
        if len(heights) == 1:
            height_note.append(f"{label}目标锚点采样高度为 {heights[0]:.3f} m（任务路径可包含其他高度）")
        else:
            height_note.append(f"{label}含 {len(heights)} 个采样高度，XY 投影中任一高度支持即着色；角标计数仍按三维采样行统计")
    own = "，".join(f"{LABELS[m]} {counts[0][m][0]}/{counts[0][m][1]}" for m in experiment.modes)
    matched = "，".join(f"{matched_labels[m]} {counts[1][m][0]}/{counts[1][m][1]}" for m in experiment.modes)
    caption = f"""**图：任务约束与目标锚点上的离散可操作支持。** 上排按各任务自身的动作定义比较（位置任务、朝下抓取任务、桌面擦拭任务）；下排使用相同的采样路径，比较仅位置、工具轴朝下与完整姿态约束，区分动作路径变化与姿态约束的影响。上排三列依次为 Position only、Top-down grasp 和 Table wipe；下排对应标为 Position only、Downward axis 和 Full pose，实际列顺序由 metadata.json 的 mode_order 确定。彩色方块表示以该坐标为锚点定义的任务路径找到满足约束的采样关节序列，灰色方块表示本次有限初值求解未找到可行序列；灰色不构成不可达的数学证明。黑色矩形和箭头为固定参考底盘及其朝向，桌面以浅褐色投影表示。各面板使用相同世界坐标范围。{'；'.join(height_note)}。上排支持锚点行数为 {own}；下排为 {matched}。角标分母是对应 CSV 的实际目标锚点行数，两排可能不同。底盘尺寸 {length:.3f} × {width:.3f} m，读取自 {footprint_source}。结果是所建运动学模型与离散路径的可行性证据，不代表连续轨迹避碰、接触力控制或实机任务完成。\n\n数据：workspace.csv、matched_path_workspace.csv、metadata.json。"""
    paths.append(write_caption(out, "workspace_support", caption))
    return paths, experiment.sources


if __name__ == "__main__":
    args = arguments(__doc__)
    generated, _ = generate(args.data_dir, args.output_dir)
    for path in generated:
        print(path)
