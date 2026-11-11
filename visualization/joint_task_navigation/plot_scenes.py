"""Main-scenario-style staged/joint overview and individual task comparisons."""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from common import (ASSIGNMENT_COLOR, MODE_LABELS, METHOD_LABELS, PATH_COLOR, SCENE_METHODS,
                    START_COLOR, TASK_EDGE, TASK_FACE, Experiment, REFERENCES, arguments,
                    boolean, caption, draw_base, draw_map, integer, label_panel, number,
                    output_directory, save, scene_legend, style)


def _read_scene_results(experiment):
    task_rows = experiment.rows("tasks.csv", ("task_id", "table_id", "x_m", "y_m", "z_m"))
    tasks = {}
    for row in task_rows:
        task_id = integer(row["task_id"], "task_id")
        if task_id in tasks:
            raise ValueError("tasks.csv: duplicate task_id")
        tasks[task_id] = (number(row["x_m"]), number(row["y_m"]))
    dock_rows = experiment.rows("docking_poses.csv", ("mode", "method", "dock_id", "candidate_id",
                                                     "x_m", "y_m", "yaw_rad", "covered_task_ids", "visit_rank"))
    route_rows = experiment.rows("navigation_paths.csv", ("mode", "method", "point_id", "x_m", "y_m", "yaw_rad"))
    results = {}
    for mode in experiment.modes:
        for method in SCENE_METHODS:
            key = mode, method
            docks = sorted((row for row in dock_rows if (row["mode"], row["method"]) == key),
                           key=lambda row: integer(row["visit_rank"], "visit_rank"))
            route = sorted((row for row in route_rows if (row["mode"], row["method"]) == key),
                           key=lambda row: integer(row["point_id"], "point_id"))
            summary = experiment.summaries()[key]
            if len(docks) != integer(summary["docks"], "docks"):
                raise ValueError(f"{key}: dock CSV count differs from summary")
            ranks = [integer(row["visit_rank"]) for row in docks]
            if sorted(ranks) != list(range(1, len(docks)+1)):
                raise ValueError(f"{key}: visit_rank must be a permutation of 1..dock_count")
            if len(route) < 2:
                raise ValueError(f"{key}: no actual navigation path to draw")
            point_ids = [integer(row["point_id"]) for row in route]
            if len(set(point_ids)) != len(point_ids):
                raise ValueError(f"{key}: duplicate navigation point_id")
            xy = np.array([[number(row["x_m"]), number(row["y_m"])] for row in route])
            distance = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
            if abs(distance-number(summary["route_length_m"])) > 1e-5:
                raise ValueError(f"{key}: exported polyline length differs from summary")
            assigned = []
            for dock in docks:
                text = dock["covered_task_ids"].strip()
                dock["assigned_ids"] = [integer(value, "covered_task_ids") for value in text.split("|")] if text else []
                assigned.extend(dock["assigned_ids"])
            if len(set(assigned)) != len(assigned) or not set(assigned).issubset(tasks):
                raise ValueError(f"{key}: task allocation is duplicated or references unknown IDs")
            if len(assigned) != integer(summary["covered_tasks"], "covered_tasks"):
                raise ValueError(f"{key}: assigned task count differs from summary")
            results[key] = docks, xy, summary
    return tasks, results


def _draw_panel(ax, experiment, tasks, record, *, compact):
    docks, xy, summary = record
    scene = experiment.scene()
    border = number(experiment.metadata.get("border_thickness_m", scene.get("border_thickness_m", 0)), "border_thickness_m")
    if border < 0:
        raise ValueError("border_thickness_m must be nonnegative")
    draw_map(ax, scene, border)
    length, width = experiment.footprint()
    # Draw allocation behind the actual navigation path and point symbols.
    for dock in docks:
        x, y = number(dock["x_m"]), number(dock["y_m"])
        for task_id in dock["assigned_ids"]:
            tx, ty = tasks[task_id]
            ax.plot([x, tx], [y, ty], linestyle=(0, (3, 2)), linewidth=0.75,
                    color=ASSIGNMENT_COLOR, alpha=0.8, zorder=3)
    valid = boolean(summary["route_valid"], "route_valid")
    ax.plot(xy[:, 0], xy[:, 1], color=PATH_COLOR, linewidth=1.7 if compact else 2.0,
            linestyle="-" if valid else ":", zorder=4)
    for dock_index, dock in enumerate(docks):
        pose = [number(dock[name]) for name in ("x_m", "y_m", "yaw_rad")]
        draw_base(ax, pose, length, width)
        # D refers to the actual visit rank, not selected-candidate ordering.
        rank = integer(dock["visit_rank"])
        nearby = [i for i, item in enumerate(docks)
                  if np.linalg.norm(np.array([number(item["x_m"]), number(item["y_m"])])-pose[:2]) < 0.30]
        # Different yaw candidates can occupy one position. Separate their
        # labels rather than silently covering one visit with another.
        group_rank = nearby.index(dock_index)
        offsets = ((7, -12), (-7, 11), (7, 11), (-7, -12), (16, 0), (-16, 0))
        offset = (7, -11) if len(nearby) == 1 else offsets[group_rank % len(offsets)]
        dock_label = f"D{rank}"
        if summary["mode"] == "top_down_grasp":
            # Raw path length can be unchanged while orientation cost changes.
            # Label actual yaw explicitly instead of implying a shorter path.
            degrees = np.rad2deg(np.arctan2(np.sin(pose[2]), np.cos(pose[2])))
            dock_label += f"\n{degrees:+.0f}°"
            table_y = np.mean(np.asarray(scene["table_centers"])[:, 1])
            offset = (7, 10) if pose[1] > table_y else (7, -11)
        ax.annotate(dock_label, xy=pose[:2], xytext=offset, textcoords="offset points",
                    ha="left" if offset[0] > 0 else "right", va="top" if offset[1] < 0 else "bottom", fontsize=7 if compact else 8,
                    color="#493014", zorder=10,
                    bbox={"facecolor": "white", "alpha": 0.84, "edgecolor": "none", "pad": 0.7})
    positions = np.array(list(tasks.values()))
    ax.scatter(positions[:, 0], positions[:, 1], s=30 if compact else 39, marker="o",
               facecolors=TASK_FACE, edgecolors=TASK_EDGE, linewidths=0.7, zorder=9)
    for task_id, position in tasks.items():
        ax.annotate(str(task_id), xy=position, xytext=(0, 5), textcoords="offset points",
                    ha="center", va="bottom", fontsize=6.5 if compact else 7.5, color="#163B1C", zorder=10)
    start = scene["start_pose"]
    ax.scatter(start[0], start[1], marker="*", s=125 if compact else 160,
               facecolors=START_COLOR, edgecolors="black", linewidths=0.8, zorder=11)
    text = f"{integer(summary['covered_tasks'])}/{integer(summary['target_count'])} tasks   |   {len(docks)} docks   |   {number(summary['route_length_m']):.2f} m"
    if not valid:
        text += "   |   INVALID ROUTE"
    ax.text(0.985, 0.975, text, transform=ax.transAxes, ha="right", va="top",
            fontsize=7.2 if compact else 8.2, color="#202020",
            bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "none", "pad": 2}, zorder=12)


def _caption(experiment, selected_modes, records, *, overview):
    details = []
    for mode in selected_modes:
        values = []
        for method in SCENE_METHODS:
            summary = records[(mode, method)][2]
            values.append(f"{METHOD_LABELS[method]}：覆盖 {integer(summary['covered_tasks'])}/{integer(summary['target_count'])}，"
                          f"{integer(summary['docks'])} 个停靠位姿，导航路线 {number(summary['route_length_m']):.3f} m，"
                          f"route_valid={str(boolean(summary['route_valid'])).lower()}")
        details.append(f"{MODE_LABELS[mode]}（{'；'.join(values)}）")
    organization = "三行分别为三个任务模式，每行左列为 staged、右列为 joint。" if overview else f"{MODE_LABELS[selected_modes[0]]} 任务下，左图为 staged，右图为 joint。"
    scene = experiment.scene()
    length, width = experiment.footprint()
    pool_note = ""
    if "candidate_count" in experiment.metadata:
        pool_note = f"固定候选池含 {integer(experiment.metadata['candidate_count'])} 个通过底盘足迹过滤的完整位姿。"
    equality_notes = []
    for mode in selected_modes:
        staged = records[(mode, "staged")][2]
        joint = records[(mode, "joint")][2]
        if abs(number(staged["route_length_m"])-number(joint["route_length_m"])) < 1e-6:
            equality_notes.append(f"{MODE_LABELS[mode]} 的原始路线长度没有减少；若加权导航代价下降，应归因于所定义的端点朝向项，不能声称该模式路径变短。")
    return ("**图：任务可达性约束下的分阶段规划与联合规划对照。** " + organization +
            "视觉语言参考本项目 results/figures/fig_main_scenario_publication.png 及 "
            "visualization/plot_main_scenario_publication.m：灰色矩形为桌面，绿色圆点为任务位置，蓝线为实际导出的导航路径，"
            "橙色三角为所选停靠中心，红星为起点。半透明蓝色矩形按实际底盘尺寸与姿态绘制，深蓝箭头表示底盘朝向；"
            "紫灰虚线表示正式独占任务分配，不能解释为机械臂执行轨迹或无碰撞运动。数字为任务 ID；D1、D2 等按 visit_rank "
            "标记实际访问顺序，原始 selected-order dock_id 和 candidate_id 仍保存在 CSV 中。抓取模式额外标注实际底盘 yaw（单位为度），以呈现朝向变化。"
            f"各面板保持相同的 {number(scene['map_width_m']):g} × {number(scene['map_height_m']):g} m 地图和"
            f" {len(scene['table_centers'])} 张桌面，不将参考图的十六桌布局引入该实验；底盘尺寸 {length:.3f} × {width:.3f} m。"
            "未绘制密集候选点。" + pool_note + "；".join(details) + "。" + "".join(equality_notes) +
            "蓝色路线直接使用 navigation_paths.csv 的有序路径点，未以停靠位姿之间的直线代替。若路线校验失败，"
            "图内会标记 INVALID ROUTE 并使用点线。该比较属于所建模型、固定候选池和本次运行，不代表实机成功率或普遍最优保证。\n\n"
            "数据：metadata.json、tasks.csv、docking_poses.csv、navigation_paths.csv、summary.csv。")


def generate(data_dir: Path, output_dir: Path | None = None):
    style()
    experiment = Experiment(data_dir)
    out = output_directory(data_dir, output_dir)
    tasks, records = _read_scene_results(experiment)
    outputs = []
    fig, axes = plt.subplots(3, 2, figsize=(11.4, 10.7), sharex=True, sharey=True, layout="constrained")
    for row, mode in enumerate(experiment.modes):
        for col, method in enumerate(SCENE_METHODS):
            ax = axes[row, col]
            _draw_panel(ax, experiment, tasks, records[(mode, method)], compact=True)
            label_panel(ax, f"({chr(97+row*2+col)}) {MODE_LABELS[mode]} — {METHOD_LABELS[method]}")
            if row < 2:
                ax.set_xlabel("")
            if col:
                ax.set_ylabel("")
    fig.legend(handles=scene_legend(), loc="outside lower center", ncol=4, frameon=True,
               fancybox=False, edgecolor="#555555", columnspacing=1.5)
    stem = "main_style_overview"
    outputs.extend(save(fig, out, stem))
    outputs.append(caption(out, stem, _caption(experiment, experiment.modes, records, overview=True)))
    for mode in experiment.modes:
        fig, axes = plt.subplots(1, 2, figsize=(11.4, 4.35), sharex=True, sharey=True, layout="constrained")
        for col, method in enumerate(SCENE_METHODS):
            _draw_panel(axes[col], experiment, tasks, records[(mode, method)], compact=False)
            label_panel(axes[col], f"({chr(97+col)}) {METHOD_LABELS[method]}")
            if col:
                axes[col].set_ylabel("")
        fig.legend(handles=scene_legend(), loc="outside lower center", ncol=4, frameon=True,
                   fancybox=False, edgecolor="#555555", columnspacing=1.5)
        stem = f"main_style_{mode}"
        outputs.extend(save(fig, out, stem))
        outputs.append(caption(out, stem, _caption(experiment, [mode], records, overview=False)))
    return outputs, experiment.sources | {path for path in REFERENCES if path.is_file()}


if __name__ == "__main__":
    args = arguments(__doc__)
    paths, _ = generate(args.data_dir, args.output_dir)
    for path in paths:
        print(path.resolve())
