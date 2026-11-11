"""Render the exported main scene, with uncluttered full-map comparisons."""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from main_common import (ASSIGNMENT_COLOR, METHOD_LABELS, MODE_LABELS, PATH_COLOR, REFERENCES,
                         START_COLOR, TASK_EDGE, TASK_FACE, Experiment, arguments, boolean,
                         caption, draw_base, draw_map, integer, label_panel, number,
                         output_directory, save, scene_legend, style, valid_mission)


def read_results(experiment):
    summaries = experiment.summaries()
    tasks = {}
    for row in experiment.rows("tasks.csv", ("task_id", "table_id", "x_m", "y_m", "z_m")):
        task_id = integer(row["task_id"])
        if task_id in tasks:
            raise ValueError("tasks.csv: duplicate task ID")
        tasks[task_id] = number(row["x_m"]), number(row["y_m"])
    all_docks = experiment.rows("docking_poses.csv", ("mode", "method", "dock_id", "candidate_id", "x_m", "y_m", "yaw_rad"), allow_empty=True)
    all_paths = experiment.rows("navigation_paths.csv", ("mode", "method", "point_id", "x_m", "y_m", "yaw_rad"), allow_empty=True)
    records = {}
    for mode in experiment.modes:
        for method in experiment.scene_methods():
            key = mode, method
            summary = summaries[key]
            mission_valid = valid_mission(summary)
            docks = [dict(row) for row in all_docks if (row["mode"], row["method"]) == key]
            if mission_valid and len(docks) != integer(summary["docks"]):
                raise ValueError(f"{key}: selected dock count differs from summary")
            def rank_available(row):
                try:
                    value = float(row.get("visit_rank", ""))
                    return np.isfinite(value) and value == int(value) and value > 0
                except (TypeError, ValueError):
                    return False
            has_visit_ranks = bool(docks) and all(rank_available(row) for row in docks)
            for row in docks:
                row["display_rank"] = integer(row["visit_rank"] if has_visit_ranks else row["dock_id"])
            docks.sort(key=lambda row: row["display_rank"])
            if has_visit_ranks and [row["display_rank"] for row in docks] != list(range(1, len(docks)+1)):
                raise ValueError(f"{key}: visit_rank must be a permutation of 1..dock_count")
            assigned = []
            has_assignments = bool(docks) and all("covered_task_ids" in row for row in docks)
            for row in docks:
                text = row.get("covered_task_ids", "").strip()
                row["assigned_ids"] = [integer(value) for value in text.split("|")] if text else []
                assigned.extend(row["assigned_ids"])
            if has_assignments and (len(set(assigned)) != len(assigned) or not set(assigned).issubset(tasks)):
                raise ValueError(f"{key}: duplicated or unknown assigned tasks")
            if mission_valid and has_assignments and len(assigned) != integer(summary["covered_tasks"]):
                raise ValueError(f"{key}: assigned task count differs from summary")
            route = sorted((row for row in all_paths if (row["mode"], row["method"]) == key),
                           key=lambda row: integer(row["point_id"]))
            if mission_valid:
                if len(route) < 2 or len({integer(row["point_id"]) for row in route}) != len(route):
                    raise ValueError(f"{key}: missing or duplicate actual navigation samples")
                xy = np.array([[number(row["x_m"]), number(row["y_m"])] for row in route])
                length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
                if abs(length-number(summary["route_length_m"])) > 1e-5:
                    raise ValueError(f"{key}: polyline length differs from summary")
            else:
                # A partial or failed run has no valid complete mission line.
                # Do not connect its docks or fill a missing route with zeros.
                xy = np.zeros((0, 2))
            records[key] = {"docks": docks, "xy": xy, "summary": summary,
                            "visit_ranks": has_visit_ranks, "assignments": has_assignments,
                            "mission_valid": mission_valid,
                            "all_dock_poses_exported": len(docks) == integer(summary["docks"])}
    return tasks, records


def draw_panel(ax, experiment, tasks, result, *, compact, details):
    scene = experiment.scene()
    border = number(experiment.metadata.get("border_thickness_m", scene.get("border_thickness_m", 0)))
    if border < 0:
        raise ValueError("border_thickness_m must be nonnegative")
    draw_map(ax, scene, border)
    summary, xy, docks = result["summary"], result["xy"], result["docks"]
    if details:
        for dock in docks:
            x, y = number(dock["x_m"]), number(dock["y_m"])
            for task_id in dock["assigned_ids"]:
                tx, ty = tasks[task_id]
                ax.plot([x, tx], [y, ty], linestyle=(0, (3, 2)), linewidth=0.55,
                        color=ASSIGNMENT_COLOR, alpha=0.6, zorder=3)
    valid = result["mission_valid"]
    if valid:
        ax.plot(xy[:, 0], xy[:, 1], color=PATH_COLOR, linewidth=1.65 if compact else 1.9,
                linestyle="-", zorder=4)
    length, width = experiment.footprint()
    label_count = len(docks) if details else min(len(docks), 6 if compact else 10)
    shown = set(np.linspace(0, len(docks)-1, label_count, dtype=int)) if docks else set()
    for index, dock in enumerate(docks):
        pose = [number(dock[field]) for field in ("x_m", "y_m", "yaw_rad")]
        draw_base(ax, pose, length, width)
        if index in shown:
            prefix = "D" if result["visit_ranks"] else "S"
            ax.annotate(f"{prefix}{dock['display_rank']}", xy=pose[:2], xytext=(5, -8),
                        textcoords="offset points", ha="left", va="top",
                        fontsize=6.5 if compact else 7.2, color="#493014", zorder=10,
                        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none", "pad": 0.5})
    coordinates = np.array(list(tasks.values()))
    ax.scatter(coordinates[:, 0], coordinates[:, 1], marker="o", s=25 if compact else 32,
               facecolors=TASK_FACE, edgecolors=TASK_EDGE, linewidths=0.65, zorder=9)
    if details:
        for task_id, xy_task in tasks.items():
            ax.annotate(str(task_id), xy=xy_task, xytext=(0, 4), textcoords="offset points",
                        ha="center", va="bottom", fontsize=6, color="#163B1C", zorder=10)
    start = scene["start_pose"]
    ax.scatter(start[0], start[1], marker="*", s=125 if compact else 160,
               facecolors=START_COLOR, edgecolors="black", linewidths=0.8, zorder=11)
    text = f"{integer(summary['covered_tasks'])}/{integer(summary['target_count'])} supported"
    if result["all_dock_poses_exported"]:
        text += f"  |  {len(docks)} docks"
    text += f"  |  {number(summary['route_length_m']):.2f} m" if valid else "  |  NO VALID MISSION"
    # Keep statistics outside the physical map: upper-row docks can sit very
    # near the wall, and an in-map annotation would cover their footprints.
    ax.text(0.995, 1.03, text, transform=ax.transAxes, ha="right", va="bottom",
            fontsize=7 if compact else 8.2, color="#202020" if valid else "#A32121", zorder=12,
            bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "none", "pad": 2})


def make_caption(experiment, modes, records, *, overview, details):
    scene = experiment.scene()
    methods = experiment.scene_methods()
    phrases = []
    for mode in modes:
        entries = []
        for method in methods:
            result = records[(mode, method)]
            row = result["summary"]
            outcome = (f"距离 {number(row['route_length_m']):.3f} m，加权代价 {number(row['navigation_cost']):.3f} m-equivalent"
                       if result["mission_valid"] else "无有效完整任务路线（不绘制导航线，也不将缺失代价值作为零）")
            reason = row.get("failure_reason", "").strip()
            if reason:
                outcome += f"，导出失败原因：{reason}"
            dock_note = f"{integer(row['docks'])} 个停靠"
            if not result["all_dock_poses_exported"]:
                dock_note += (f"（摘要报告数量；位姿 CSV 仅导出 {len(result['docks'])} 个，"
                              "缺失位姿未绘制，未按参考布局猜测或补齐）")
            entries.append(f"{METHOD_LABELS[method]}：支持覆盖 {integer(row['covered_tasks'])}/{integer(row['target_count'])} 任务，"
                           f"{dock_note}，{outcome}，route_valid={row['route_valid']}")
        phrases.append(f"{MODE_LABELS[mode]}（{'；'.join(entries)}）")
    organization = "各行表示一种任务模式，" if overview else f"{MODE_LABELS[modes[0]]} 任务下，"
    display = ("详细版显示全部任务 ID、停靠编号和已导出的分配虚线；虚线不是机械臂运动轨迹。" if details else
               "为避免密集遮挡，主图保留全部任务点与全部停靠足迹，省略任务 ID 和分配连线，只标少量访问编号；省略文字不代表删除任务或停靠。")
    pool = f"候选池数量为 metadata.json 记录的 {integer(experiment.metadata['candidate_count'])}。" if "candidate_count" in experiment.metadata else ""
    return ("**图：真实主场景中的任务约束与导航联合规划尝试。** " + organization +
            f"左图为 {methods[0]}，右图为 {methods[1]}。图中完整显示 metadata.json 所记录的 "
            f"{number(scene['map_width_m']):g} × {number(scene['map_height_m']):g} m 地图、{len(scene['table_centers'])} 张桌面和"
            " tasks.csv 的原始任务位置，未为绘图改动布局或目标。" + pool +
            "视觉编码参考 results/figures/fig_main_scenario_publication.png：灰桌、绿色圆任务点、蓝色实际导航路径、"
            "橙色三角停靠中心、红色起点星号；半透明蓝色矩形与深蓝箭头分别表示实际底盘足迹及朝向。" + display +
            "D 编号取 visit_rank（实际访问顺序）；未导出访问顺序时才用 S 编号标记原始 selected-order dock_id。"
            "覆盖不足或导航失败的面板保留真实地图和已导出的停靠，并标记 NO VALID MISSION，不虚构完整路线。"
            "路线直接使用 CSV 中的有序导航采样点，不以停靠中心间的直线代替。" + "；".join(phrases) +
            "。图中 Staged 指原 bottom_up 加共同导航修复后的分阶段方法；初轮失败另行归档，不作为本图的对照结果。"
            "各方法统一使用有向导航代价与有效性检查（directed_costs）以及有限闭环的 NN 起点重试恢复（finite_nn_restarts），"
            "几何和 IK 支持库保持冻结。方向性修复与有限闭环恢复不作为 joint_local 的单独收益。"
            "joint_local 是该次有限预算的局部搜索方法，不能继承此前小场景的候选池全局精确最优结论。"
            "绿色点表示任务位置，不表示任务已在实机完成；覆盖与路径有效性应结合原始结果和独立审计判断。\n\n"
            "数据：metadata.json、tasks.csv、summary.csv、docking_poses.csv、navigation_paths.csv。")


def generate(data_dir: Path, output_dir: Path | None = None, details=False):
    style()
    experiment = Experiment(data_dir)
    out = output_directory(data_dir, output_dir)
    tasks, records = read_results(experiment)
    modes, methods = experiment.modes, experiment.scene_methods()
    scene = experiment.scene()
    # Derive panel height from the actual world-coordinate aspect ratio.
    aspect = number(scene["map_height_m"])/number(scene["map_width_m"])
    row_height = 5.6*aspect+0.50
    suffix = "_details" if details else ""
    outputs = []
    fig, axes = plt.subplots(len(modes), 2, figsize=(12, row_height*len(modes)+0.55),
                             sharex=True, sharey=True, squeeze=False, layout="constrained")
    for row, mode in enumerate(modes):
        for col, method in enumerate(methods):
            ax = axes[row, col]
            draw_panel(ax, experiment, tasks, records[(mode, method)], compact=True, details=details)
            label_panel(ax, f"({chr(97+row*2+col)}) {MODE_LABELS[mode]} — {METHOD_LABELS[method]}")
            if row < len(modes)-1:
                ax.set_xlabel("")
            if col:
                ax.set_ylabel("")
    fig.legend(handles=scene_legend(details), loc="outside lower center", ncol=3, frameon=True,
               fancybox=False, edgecolor="#555555", columnspacing=1.7)
    stem = "main_scene_overview"+suffix
    outputs.extend(save(fig, out, stem))
    outputs.append(caption(out, stem, make_caption(experiment, modes, records, overview=True, details=details)))
    for mode in modes:
        fig, axes = plt.subplots(1, 2, figsize=(12, row_height+0.55), sharex=True, sharey=True, layout="constrained")
        for col, method in enumerate(methods):
            draw_panel(axes[col], experiment, tasks, records[(mode, method)], compact=False, details=details)
            label_panel(axes[col], f"({chr(97+col)}) {METHOD_LABELS[method]}")
            if col:
                axes[col].set_ylabel("")
        fig.legend(handles=scene_legend(details), loc="outside lower center", ncol=3, frameon=True,
                   fancybox=False, edgecolor="#555555", columnspacing=1.7)
        stem = f"main_scene_{mode}"+suffix
        outputs.extend(save(fig, out, stem))
        outputs.append(caption(out, stem, make_caption(experiment, [mode], records, overview=False, details=details)))
    return outputs, experiment.sources | {path for path in REFERENCES if path.is_file()}


if __name__ == "__main__":
    args = arguments(__doc__)
    paths, _ = generate(args.data_dir, args.output_dir, args.details)
    for path in paths:
        print(path.resolve())
