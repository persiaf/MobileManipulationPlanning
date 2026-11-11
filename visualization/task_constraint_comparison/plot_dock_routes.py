"""Compare shared-scene candidate support, selected docks and navigation paths."""
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import PercentFormatter
import numpy as np

from common import (COLORS, LABELS, LINESTYLES, Experiment, apply_style, arguments,
                    draw_base, draw_tables, integer, number, output_directory,
                    panel_label, save_figure, scene_axes, write_caption)


def generate(data_dir: Path, output_dir: Path | None = None):
    apply_style()
    experiment = Experiment(data_dir)
    out = output_directory(data_dir, output_dir)
    scene = experiment.scene()
    length, width, footprint_source = experiment.footprint()
    tasks = experiment.numeric("tasks.csv", ("task_id", "table_id", "x_m", "y_m", "z_m"))
    candidates = experiment.rows("candidate_support.csv", ("mode", "candidate_id", "table_id", "x_m", "y_m", "yaw_rad", "support_count", "task_count"))
    docks = experiment.rows("docking_poses.csv", ("mode", "dock_id", "candidate_id", "x_m", "y_m", "yaw_rad"))
    routes = experiment.rows("navigation_paths.csv", ("mode", "point_id", "x_m", "y_m", "yaw_rad"))
    fig, axes = plt.subplots(1, 3, figsize=(11.4, 3.75), sharex=True, sharey=True,
                             layout="constrained")
    norm = Normalize(0, 1)
    route_counts = []
    dock_counts = []
    for index, (ax, mode) in enumerate(zip(axes, experiment.modes)):
        draw_tables(ax, scene)
        subset = [row for row in candidates if row["mode"] == mode]
        if not subset:
            raise ValueError(f"candidate_support.csv: no candidates for {mode}")
        xy = np.array([[number(row["x_m"]), number(row["y_m"])] for row in subset])
        counts = np.array([integer(row["support_count"], "support_count") for row in subset])
        totals = np.array([integer(row["task_count"], "task_count") for row in subset])
        if (totals <= 0).any() or (counts < 0).any() or (counts > totals).any():
            raise ValueError("Candidate support must satisfy 0 <= support_count <= positive task_count")
        fractions = counts / totals
        # Multiple yaw candidates share a position. A direct scatter would
        # silently hide all but the final heading; display the explicit maximum.
        positions, inverse = np.unique(xy, axis=0, return_inverse=True)
        best_heading_fraction = np.zeros(len(positions))
        np.maximum.at(best_heading_fraction, inverse, fractions)
        zero = best_heading_fraction == 0
        ax.scatter(positions[zero, 0], positions[zero, 1], marker="x", s=10, linewidths=0.6,
                   color="#BDC3C6", zorder=2)
        colored = ax.scatter(positions[~zero, 0], positions[~zero, 1], c=best_heading_fraction[~zero], cmap="cividis",
                             norm=norm, s=18, edgecolors="#4D5053", linewidths=0.2, zorder=3)
        ax.scatter(tasks[:, 2], tasks[:, 3], marker="*", s=42,
                   facecolors="white", edgecolors="#252525", linewidths=0.75, zorder=5)
        selected = sorted((row for row in docks if row["mode"] == mode), key=lambda row: integer(row["dock_id"]))
        for row in selected:
            draw_base(ax, [number(row[key]) for key in ("x_m", "y_m", "yaw_rad")], length, width,
                      color=COLORS[mode], alpha=0.16)
        route = sorted((row for row in routes if row["mode"] == mode), key=lambda row: integer(row["point_id"]))
        if len(route) < 2:
            raise ValueError(f"navigation_paths.csv: {mode} needs at least two actual path samples")
        point_ids = [integer(row["point_id"]) for row in route]
        if len(point_ids) != len(set(point_ids)):
            raise ValueError(f"navigation_paths.csv: duplicate point_id in {mode}")
        path = np.array([[number(row["x_m"]), number(row["y_m"])] for row in route])
        ax.plot(path[:, 0], path[:, 1], color=COLORS[mode], linestyle=LINESTYLES[mode],
                linewidth=1.25, zorder=6)
        start = scene["start_pose"]
        ax.scatter(start[0], start[1], marker="D", s=29, facecolors="white",
                   edgecolors="#252525", linewidths=1, zorder=8)
        scene_axes(ax, scene)
        if index:
            ax.set_ylabel("")
        panel_label(ax, chr(ord("a") + index), LABELS[mode])
        ax.text(0.985, 0.025, f"{len(selected)} docks", transform=ax.transAxes,
                ha="right", va="bottom", fontsize=8,
                bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none", "pad": 2})
        route_counts.append(len(route))
        dock_counts.append(len(selected))
    colorbar = fig.colorbar(colored, ax=axes, location="right", fraction=0.025, pad=0.012, shrink=0.84)
    # Matplotlib rasterizes dense colorbar solids by default; retain vector PDF.
    colorbar.solids.set_rasterized(False)
    colorbar.set_label("Best sampled-heading support (%)", fontsize=9)
    colorbar.ax.yaxis.set_major_formatter(PercentFormatter(1))
    legend = [Line2D([], [], color="#333333", linewidth=1.25, label="Navigation path"),
              Patch(facecolor="#E8E8E8", edgecolor="#333333", label="Selected base pose"),
              Line2D([], [], marker="*", linestyle="none", markerfacecolor="white", color="#252525", markersize=8, label="Target"),
              Line2D([], [], marker="D", linestyle="none", markerfacecolor="white", color="#252525", label="Start"),
              Line2D([], [], marker="x", linestyle="none", color="#BDC3C6", label="Zero-support candidate")]
    fig.legend(handles=legend, loc="outside lower center", ncol=5, frameon=False, columnspacing=1.3)
    paths = save_figure(fig, out, "dock_routes")
    detail = "，".join(f"{LABELS[mode]} 选择 {count} 个停靠位姿、导出 {points} 个导航路径点"
                       for mode, count, points in zip(experiment.modes, dock_counts, route_counts))
    caption = f"""**图：任务约束对候选支持、停靠选择与导航路线的影响。** 三个面板使用相同地图、桌面、目标位置与起始底盘位姿。浅褐色矩形为桌面，星形为任务目标，菱形为起点。候选点采用跨面板统一的 0–100% 色阶，支持比例为支持任务数除以该候选的 task_count。一个位置具有多个采样朝向时，二维图仅显示这些朝向中的最大支持比例，灰色叉号表示该位置所有采样朝向均为零支持；该聚合仅用于显示，规划仍使用独立的完整底盘位姿。矩形和箭头给出所选底盘的实际尺寸及朝向，折线直接连接 navigation_paths.csv 中按 point_id 排序的导航路径采样点，未将停靠中心用直线替代实际路径。{detail}。底盘尺寸为 {length:.3f} × {width:.3f} m，来源为 {footprint_source}。图中候选支持属于任务相关数值运动学检查，路线有效性由配套 summary.csv 的 route_valid 和导航校验记录判断；本图不提供机械臂避碰或擦拭接触力的实验证据。\n\n数据：candidate_support.csv、docking_poses.csv、navigation_paths.csv、tasks.csv、metadata.json。"""
    paths.append(write_caption(out, "dock_routes", caption))
    return paths, experiment.sources


if __name__ == "__main__":
    args = arguments(__doc__)
    generated, _ = generate(args.data_dir, args.output_dir)
    for path in generated:
        print(path)
