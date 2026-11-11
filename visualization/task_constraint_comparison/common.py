"""Validated experiment readers and shared, print-ready plotting utilities.

Inputs are measured/exported CSV and JSON files only. No synthetic observations
or interpolation of missing outcomes are introduced by these plotting scripts.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA = REPO_ROOT / "results" / "task_constraint_comparison_20261111"
MODES = ("position_only", "top_down_grasp", "table_wipe")
LABELS = {
    "position_only": "Position only",
    "top_down_grasp": "Top-down grasp",
    "table_wipe": "Table wipe",
}
SHORT_LABELS = {
    "position_only": "Position\nonly",
    "top_down_grasp": "Top-down\ngrasp",
    "table_wipe": "Table\nwipe",
}
# Okabe-Ito palette, reinforced by distinct markers and line styles.
COLORS = {"position_only": "#0072B2", "top_down_grasp": "#D55E00", "table_wipe": "#009E73"}
MARKERS = {"position_only": "o", "top_down_grasp": "^", "table_wipe": "s"}
LINESTYLES = {"position_only": "-", "top_down_grasp": "--", "table_wipe": "-."}


def apply_style() -> None:
    matplotlib.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 10,
        "axes.labelsize": 10,
        "axes.titlesize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 8.5,
        "figure.dpi": 150,
        "savefig.dpi": 300,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": False,
        "text.usetex": False,
        "mathtext.fontset": "stix",
        "savefig.pad_inches": 0.06,
    })


def arguments(description: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args()


def output_directory(data_dir: Path, output_dir: Path | None) -> Path:
    result = Path(output_dir) if output_dir is not None else Path(data_dir) / "figures"
    result.mkdir(parents=True, exist_ok=True)
    return result


class Experiment:
    def __init__(self, data_dir: Path):
        self.directory = Path(data_dir).resolve()
        with (self.directory / "metadata.json").open(encoding="utf-8-sig") as source:
            self.metadata = json.load(source)
        self.modes = tuple(self.metadata.get("mode_order", MODES))
        if set(self.modes) != set(MODES) or len(self.modes) != len(MODES):
            raise ValueError("metadata.mode_order must contain the three supported modes exactly once")
        self.sources = {self.directory / "metadata.json"}

    def rows(self, filename: str, columns: tuple[str, ...]) -> list[dict[str, str]]:
        path = self.directory / filename
        with path.open(newline="", encoding="utf-8-sig") as source:
            reader = csv.DictReader(source)
            missing = set(columns) - set(reader.fieldnames or [])
            if missing:
                raise ValueError(f"{path.name}: missing columns {sorted(missing)}")
            rows = list(reader)
        if not rows:
            raise ValueError(f"{path.name}: no exported observations")
        self.sources.add(path)
        return rows

    def numeric(self, filename: str, columns: tuple[str, ...]) -> np.ndarray:
        rows = self.rows(filename, columns)
        values = np.array([[number(row[name], f"{filename}:{name}") for name in columns] for row in rows])
        return values

    def scene(self) -> dict:
        scene = self.metadata["scene"]
        for name in ("table_length_m", "table_width_m", "map_width_m", "map_height_m"):
            if number(scene[name], name) <= 0:
                raise ValueError(f"scene.{name} must be positive")
        return scene

    def footprint(self) -> tuple[float, float, str]:
        meta = self.metadata
        if "robot_footprint" in meta:
            length = float(meta["robot_footprint"]["length_m"])
            width = float(meta["robot_footprint"]["width_m"])
            origin = "metadata.json:robot_footprint"
        elif "footprint_length_m" in meta and "footprint_width_m" in meta:
            length, width = float(meta["footprint_length_m"]), float(meta["footprint_width_m"])
            origin = "metadata.json"
        else:
            # Source the actual canonical chassis dimensions, not an invented
            # illustration size. Metadata can override this for archived runs.
            path = REPO_ROOT / "config" / "robot_config.m"
            text = path.read_text(encoding="utf-8-sig")
            length = _config_number(text, "length")
            width = _config_number(text, "width")
            origin = "config/robot_config.m"
            self.sources.add(path)
        if not (math.isfinite(length) and math.isfinite(width) and length > 0 and width > 0):
            raise ValueError("Footprint length and width must be finite and positive")
        return length, width, origin


def _config_number(text: str, field: str) -> float:
    match = re.search(rf"cfg\.chassis\.{field}\s*=\s*([0-9.eE+-]+)\s*;", text)
    if match is None:
        raise ValueError(f"Cannot read canonical chassis {field}; provide footprint dimensions in metadata")
    return float(match.group(1))


def number(value, context="value") -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{context}: expected numeric data, got {value!r}") from error
    if not math.isfinite(result):
        raise ValueError(f"{context}: expected a finite observation, got {value!r}")
    return result


def integer(value, context="value") -> int:
    result = number(value, context)
    if result != int(result):
        raise ValueError(f"{context}: expected an integer")
    return int(result)


def boolean(value, context="value") -> bool:
    if isinstance(value, bool):
        return value
    if str(value).strip().lower() in ("1", "true"):
        return True
    if str(value).strip().lower() in ("0", "false"):
        return False
    raise ValueError(f"{context}: expected an exported Boolean")


def panel_label(ax, letter: str, label: str = "") -> None:
    ax.text(0, 1.035, f"({letter}) {label}".strip(), transform=ax.transAxes,
            ha="left", va="bottom", fontsize=10)


def draw_tables(ax, scene: dict, *, labels=False) -> None:
    length, width = float(scene["table_length_m"]), float(scene["table_width_m"])
    for index, center in enumerate(scene["table_centers"], start=1):
        cx, cy = map(float, center)
        ax.add_patch(Rectangle((cx - length / 2, cy - width / 2), length, width,
                               facecolor="#E9E3D9", edgecolor="#706B63", linewidth=0.8, zorder=1))
        if labels:
            ax.text(cx, cy, f"T{index}", ha="center", va="center", fontsize=8, color="#514C44", zorder=2)


def draw_base(ax, pose, length, width, *, color="#333333", label=None, alpha=0.2, zorder=7) -> None:
    x, y, yaw = map(float, pose)
    local = np.array([[length / 2, width / 2], [length / 2, -width / 2],
                      [-length / 2, -width / 2], [-length / 2, width / 2]])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    corners = local @ rotation.T + [x, y]
    ax.add_patch(Polygon(corners, closed=True, facecolor=matplotlib.colors.to_rgba(color, alpha),
                         edgecolor=color, linewidth=1.05, zorder=zorder, label=label))
    tip = np.array([x, y]) + length * 0.58 * np.array([math.cos(yaw), math.sin(yaw)])
    ax.annotate("", xy=tip, xytext=(x, y),
                arrowprops={"arrowstyle": "-|>", "color": color, "lw": 1.1, "mutation_scale": 8},
                zorder=zorder + 1)


def scene_axes(ax, scene: dict) -> None:
    ax.set_xlim(0, float(scene["map_width_m"]))
    ax.set_ylim(0, float(scene["map_height_m"]))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")


def save_figure(fig, directory: Path, stem: str) -> list[Path]:
    paths = [directory / f"{stem}.pdf", directory / f"{stem}.png"]
    fig.savefig(paths[0], bbox_inches="tight", metadata={"CreationDate": None, "ModDate": None})
    fig.savefig(paths[1], bbox_inches="tight", dpi=300)
    plt.close(fig)
    return paths


def write_caption(directory: Path, stem: str, caption: str) -> Path:
    path = directory / f"{stem}_caption.md"
    path.write_text(caption.rstrip() + "\n", encoding="utf-8")
    return path
