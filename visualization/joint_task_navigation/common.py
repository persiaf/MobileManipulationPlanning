"""Read exported joint-planning results and reproduce the main-scene style.

The palette and visual encoding follow plot_main_scenario_publication.m;
footprints/orientation arrows follow draw_robot.m. No planning is performed.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon, Rectangle
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA = REPO_ROOT / "results" / "joint_task_navigation_20261111"
MODES = ("position_only", "top_down_grasp", "table_wipe")
METHODS = ("staged", "reorder_only", "joint")
SCENE_METHODS = ("staged", "joint")
MODE_LABELS = {"position_only": "Position only", "top_down_grasp": "Top-down grasp", "table_wipe": "Table wipe"}
METHOD_LABELS = {"staged": "Staged", "reorder_only": "Reorder only", "joint": "Joint"}
MODE_TICKS = {"position_only": "Position only", "top_down_grasp": "Top-down grasp", "table_wipe": "Table wipe"}
# Exact main-scene semantic palette; methods are identified by panel position.
TABLE_FACE, TABLE_EDGE = "#DBDBDB", "#474747"
TASK_FACE, TASK_EDGE = "#33B34D", "#0D591A"
PATH_COLOR = "#1A40D9"
DOCK_FACE, DOCK_EDGE = "#F28C26", "#73400D"
START_COLOR = "#F23326"
BASE_FACE, BASE_EDGE = "#2659A6", "#0D2666"
ASSIGNMENT_COLOR = "#A1749B"
METHOD_COLORS = {"staged": "#0072B2", "reorder_only": "#E69F00", "joint": "#009E73"}
REFERENCES = (
    REPO_ROOT / "results" / "figures" / "fig_main_scenario_publication.png",
    REPO_ROOT / "visualization" / "plot_main_scenario_publication.m",
    REPO_ROOT / "visualization" / "draw_robot.m",
    REPO_ROOT / "visualization" / "draw_docking_result.m",
)


def style() -> None:
    matplotlib.rcParams.update({
        "font.family": "serif", "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 10, "axes.labelsize": 10, "xtick.labelsize": 9, "ytick.labelsize": 9,
        "legend.fontsize": 9, "axes.linewidth": 0.8,
        "axes.spines.top": True, "axes.spines.right": True,
        "pdf.fonttype": 42, "ps.fonttype": 42,
        "figure.dpi": 150, "savefig.dpi": 300, "savefig.pad_inches": 0.06,
        "text.usetex": False, "mathtext.fontset": "stix",
    })


def arguments(description: str):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def output_directory(data_dir: Path, output_dir: Path | None) -> Path:
    directory = Path(output_dir) if output_dir else Path(data_dir) / "figures"
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def number(value, name="value") -> float:
    try:
        result = float(value)
    except (ValueError, TypeError) as error:
        raise ValueError(f"{name}: expected a number, got {value!r}") from error
    if not math.isfinite(result):
        raise ValueError(f"{name}: expected finite exported data")
    return result


def integer(value, name="value") -> int:
    result = number(value, name)
    if int(result) != result:
        raise ValueError(f"{name}: expected an integer")
    return int(result)


def boolean(value, name="value") -> bool:
    if str(value).strip().lower() in ("1", "true"):
        return True
    if str(value).strip().lower() in ("0", "false"):
        return False
    raise ValueError(f"{name}: expected an exported Boolean")


class Experiment:
    def __init__(self, directory: Path):
        self.directory = Path(directory).resolve()
        path = self.directory / "metadata.json"
        with path.open(encoding="utf-8-sig") as source:
            self.metadata = json.load(source)
        self.sources = {path}
        self.modes = tuple(self.metadata.get("mode_order", MODES))
        if len(self.modes) != 3 or set(self.modes) != set(MODES):
            raise ValueError("metadata.mode_order must contain the three task modes exactly once")
        self.methods = METHODS
        self._summary = None

    def rows(self, filename: str, required: tuple[str, ...]):
        path = self.directory / filename
        with path.open(newline="", encoding="utf-8-sig") as source:
            reader = csv.DictReader(source)
            missing = set(required) - set(reader.fieldnames or ())
            if missing:
                raise ValueError(f"{filename}: missing columns {sorted(missing)}")
            rows = list(reader)
        if not rows:
            raise ValueError(f"{filename}: empty export")
        self.sources.add(path)
        return rows

    def summaries(self):
        if self._summary is None:
            required = ("mode", "method", "covered_tasks", "target_count", "docks", "minimum_docks",
                        "route_length_m", "navigation_cost", "selection_time_s", "route_valid", "exact_on_candidate_pool",
                        "nav_estimated_time_s", "graph_time_s", "search_time_s", "exact_minimum_docks", "route_cost_optimal")
            rows = self.rows("summary.csv", required)
            records = {}
            for row in rows:
                key = (row["mode"], row["method"])
                if key in records:
                    raise ValueError(f"summary.csv: duplicate result {key}")
                records[key] = row
            expected = {(mode, method) for mode in self.modes for method in METHODS}
            if set(records) != expected:
                raise ValueError("summary.csv must contain exactly the nine mode/method results")
            self._summary = records
        return self._summary

    def scene(self):
        scene = self.metadata["scene"]
        for key in ("map_width_m", "map_height_m", "table_length_m", "table_width_m"):
            if number(scene[key], key) <= 0:
                raise ValueError(f"scene.{key} must be positive")
        return scene

    def footprint(self):
        footprint = self.metadata["robot_footprint"]
        length, width = number(footprint["length_m"]), number(footprint["width_m"])
        if min(length, width) <= 0:
            raise ValueError("Robot footprint dimensions must be positive")
        return length, width


def save(fig, directory: Path, stem: str):
    paths = [directory / f"{stem}.pdf", directory / f"{stem}.png"]
    fig.savefig(paths[0], bbox_inches="tight", metadata={"CreationDate": None, "ModDate": None})
    fig.savefig(paths[1], bbox_inches="tight", dpi=300)
    plt.close(fig)
    return paths


def caption(directory: Path, stem: str, text: str):
    path = directory / f"{stem}_caption.md"
    path.write_text(text.strip() + "\n", encoding="utf-8")
    return path


def label_panel(ax, label: str):
    ax.text(0, 1.03, label, transform=ax.transAxes, ha="left", va="bottom", fontsize=10)


def draw_map(ax, scene, border_thickness):
    width, height = float(scene["map_width_m"]), float(scene["map_height_m"])
    ax.set(xlim=(0, width), ylim=(0, height), xlabel="X (m)", ylabel="Y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_axisbelow(True)
    ax.grid(True, color="#6F6F6F", alpha=0.18, linewidth=0.65)
    ax.tick_params(direction="in", top=True, right=True, width=0.8, length=3)
    if border_thickness > 0:
        b = border_thickness
        for xy, w, h in [((0, 0), width, b), ((0, height-b), width, b),
                         ((0, 0), b, height), ((width-b, 0), b, height)]:
            ax.add_patch(Rectangle(xy, w, h, facecolor="#303030", edgecolor="none", zorder=1))
    angles = scene.get("table_orientations_rad", [0] * len(scene["table_centers"]))
    length, table_width = float(scene["table_length_m"]), float(scene["table_width_m"])
    for index, center in enumerate(scene["table_centers"]):
        yaw = float(angles[index])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        corners = np.array([[-length/2, -table_width/2], [length/2, -table_width/2],
                            [length/2, table_width/2], [-length/2, table_width/2]]) @ rotation.T + center
        ax.add_patch(Polygon(corners, closed=True, facecolor=TABLE_FACE, edgecolor=TABLE_EDGE,
                             linewidth=0.9, zorder=2))


def draw_base(ax, pose, length, width):
    x, y, yaw = pose
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    local = np.array([[-length/2, -width/2], [length/2, -width/2],
                      [length/2, width/2], [-length/2, width/2]])
    ax.add_patch(Polygon(local @ rotation.T + [x, y], closed=True,
                         facecolor=matplotlib.colors.to_rgba(BASE_FACE, 0.16),
                         edgecolor=BASE_EDGE, linewidth=0.9, zorder=5))
    direction = np.array([math.cos(yaw), math.sin(yaw)])
    ax.annotate("", xy=np.array([x, y]) + length*0.67*direction,
                xytext=np.array([x, y]) + length*0.12*direction,
                arrowprops={"arrowstyle": "-|>", "color": BASE_EDGE, "lw": 1.0, "mutation_scale": 7}, zorder=6)
    ax.scatter(x, y, marker="^", s=46, facecolors=DOCK_FACE,
               edgecolors=DOCK_EDGE, linewidths=0.7, zorder=8)


def scene_legend():
    return [Patch(facecolor=TABLE_FACE, edgecolor=TABLE_EDGE, label="Table"),
            Line2D([], [], marker="o", linestyle="none", markerfacecolor=TASK_FACE, color=TASK_EDGE, label="Task point"),
            Line2D([], [], color=PATH_COLOR, linewidth=1.8, label="Actual navigation path"),
            Line2D([], [], marker="^", linestyle="none", markerfacecolor=DOCK_FACE, color=DOCK_EDGE, label="Selected dock"),
            Patch(facecolor=matplotlib.colors.to_rgba(BASE_FACE, 0.16), edgecolor=BASE_EDGE, label="Base footprint / heading"),
            Line2D([], [], marker="*", linestyle="none", markerfacecolor=START_COLOR, color="black", markersize=11, label="Start"),
            Line2D([], [], color=ASSIGNMENT_COLOR, linestyle="--", linewidth=0.8, label="Assigned task")]
