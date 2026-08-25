#!/usr/bin/env python3
"""Render task-delivery density overlays for all packaged benchmark maps."""

from __future__ import annotations

import math
from collections import Counter
from pathlib import Path

from PIL import Image, ImageDraw

from generate_benchmark_task_matrix import BENCHMARKS
from render_warehouse_comparison import DETAIL_FONT, LEGEND_FONT, PANEL_FONT, TITLE_FONT


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "benchmark_instances"
MAP_DIR = PACKAGE / "maps"
TASK_DIR = PACKAGE / "tasks"
OUTPUT_DIR = PACKAGE / "visualizations/delivery_heatmaps"

BASE_COLORS = {
    ".": (247, 247, 242),
    "@": (36, 39, 43),
    "e": (218, 238, 224),
    "r": (39, 125, 161),
}
LOW_HEAT = (142, 213, 174)
HIGH_HEAT = (0, 72, 34)


def read_map(path: Path) -> tuple[list[str], list[tuple[int, int]], int]:
    lines = path.read_text().splitlines()
    rows, cols = map(int, lines[0].split(","))
    endpoint_count = int(lines[1])
    agents = int(lines[2])
    grid = lines[4:4 + rows]
    if len(grid) != rows or any(len(row) != cols for row in grid):
        raise ValueError(f"{path}: invalid map dimensions")
    endpoints = [
        (y, x) for y, row in enumerate(grid)
        for x, value in enumerate(row) if value == "e"
    ]
    if len(endpoints) != endpoint_count:
        raise ValueError(f"{path}: endpoint-count mismatch")
    return grid, endpoints, agents


def read_deliveries(path: Path,
                    endpoint_count: int) -> tuple[Counter[int], int]:
    lines = path.read_text().splitlines()
    task_count = int(lines[0])
    if len(lines) != task_count + 1:
        raise ValueError(f"{path}: task-count mismatch")
    deliveries: Counter[int] = Counter()
    for task_id, line in enumerate(lines[1:]):
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{path}: task {task_id} is not standard MAPD")
        delivery = int(fields[2])
        if not 0 <= delivery < endpoint_count:
            raise ValueError(f"{path}: invalid delivery endpoint {delivery}")
        deliveries[delivery] += 1
    return deliveries, task_count


def mix(base: tuple[int, int, int], overlay: tuple[int, int, int],
        alpha: float) -> tuple[int, int, int]:
    return tuple(round(base[i] * (1 - alpha) + overlay[i] * alpha)
                 for i in range(3))


def heat_color(count: int, maximum: int) -> tuple[int, int, int]:
    normalized = math.log1p(count) / math.log1p(maximum)
    heat = tuple(round(LOW_HEAT[i] * (1 - normalized) +
                       HIGH_HEAT[i] * normalized) for i in range(3))
    alpha = 0.28 + 0.68 * normalized
    return mix(BASE_COLORS["e"], heat, alpha)


def raster(grid: list[str], endpoints: list[tuple[int, int]],
           deliveries: Counter[int]) -> tuple[Image.Image, int]:
    rows, cols = len(grid), len(grid[0])
    image = Image.new("RGB", (cols, rows))
    pixels = image.load()
    for y, row in enumerate(grid):
        for x, value in enumerate(row):
            pixels[x, y] = BASE_COLORS[value]

    maximum = max(deliveries.values(), default=0)
    if maximum:
        for endpoint_id, count in deliveries.items():
            y, x = endpoints[endpoint_id]
            pixels[x, y] = heat_color(count, maximum)
    return image, maximum


def render_panel(title: str, map_path: Path, task_path: Path,
                 width: int = 860, height: int = 610) -> Image.Image:
    grid, endpoints, agents = read_map(map_path)
    deliveries, task_count = read_deliveries(task_path, len(endpoints))
    source, maximum = raster(grid, endpoints, deliveries)

    panel = Image.new("RGB", (width, height), "#ffffff")
    draw = ImageDraw.Draw(panel)
    draw.text((width / 2, 25), title, font=PANEL_FONT,
              fill="#17191c", anchor="mm")
    detail = (
        f"{len(grid[0])} × {len(grid)}  •  {agents:,} agents  •  "
        f"{task_count:,} tasks  •  max {maximum} deliveries/cell")
    draw.text((width / 2, 57), detail, font=DETAIL_FONT,
              fill="#555b61", anchor="mm")

    max_width, max_height = width - 70, height - 140
    scale = min(max_width / source.width, max_height / source.height)
    rendered = source.resize(
        (max(1, round(source.width * scale)),
         max(1, round(source.height * scale))),
        Image.Resampling.NEAREST)
    x = (width - rendered.width) // 2
    y = 82 + (max_height - rendered.height) // 2
    panel.paste(rendered, (x, y))
    draw.rectangle((x - 1, y - 1, x + rendered.width, y + rendered.height),
                   outline="#777b80", width=2)

    legend_y = height - 24
    legend = [
        (BASE_COLORS["@"], "Shelf"),
        (BASE_COLORS["r"], "Agent home"),
        (BASE_COLORS["e"], "0 deliveries"),
        (heat_color(1, max(1, maximum)), "Low"),
        (heat_color(maximum, max(1, maximum)), "Most deliveries"),
    ]
    item_width = 170
    start = (width - item_width * len(legend)) // 2
    for index, (color, label) in enumerate(legend):
        lx = start + index * item_width
        draw.rectangle((lx, legend_y - 9, lx + 18, legend_y + 9),
                       fill=color, outline="#777b80")
        draw.text((lx + 25, legend_y), label, font=LEGEND_FONT,
                  fill="#24272b", anchor="lm")
    return panel


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    canonical_panels = []

    for benchmark in BENCHMARKS:
        for agents in benchmark.agent_counts:
            map_path = MAP_DIR / f"benchmark_{benchmark.name}_a{agents}.map"
            task_path = TASK_DIR / (
                f"benchmark_{benchmark.name}_a{agents}_"
                f"f{benchmark.default_frequency}.task")
            title = benchmark.name.replace("_", " ").upper()
            panel = render_panel(f"{title} — {agents:,} agents",
                                 map_path, task_path, 1600, 1050)
            output = OUTPUT_DIR / (
                f"benchmark_{benchmark.name}_a{agents}_delivery_heatmap.png")
            panel.save(output, optimize=True)
            print(output)
            if agents == benchmark.default_agents:
                canonical_panels.append(panel)

    panel_width = 1600
    panel_height = 1050
    outer_padding = 40
    panel_gap = 30
    header_height = 100
    comparison_width = outer_padding * 2 + panel_width * 2 + panel_gap
    comparison_height = (
        header_height + panel_height * 2 + panel_gap + outer_padding)
    comparison = Image.new(
        "RGB", (comparison_width, comparison_height), "#f3f4f5")
    draw = ImageDraw.Draw(comparison)
    draw.text((comparison_width / 2, 32), "Task delivery-location density",
              font=TITLE_FONT, fill="#17191c", anchor="mm")
    draw.text((comparison_width / 2, 65),
              "Darker endpoint cells received more deliveries",
              font=DETAIL_FONT, fill="#555b61", anchor="mm")
    positions = (
        (outer_padding, header_height),
        (outer_padding + panel_width + panel_gap, header_height),
        (outer_padding, header_height + panel_height + panel_gap),
        (outer_padding + panel_width + panel_gap,
         header_height + panel_height + panel_gap),
    )
    for panel, position in zip(canonical_panels, positions):
        comparison.paste(panel, position)
    comparison_path = (
        PACKAGE / "visualizations/benchmark_delivery_heatmaps_comparison.png")
    comparison.save(comparison_path, optimize=True)
    print(comparison_path)


if __name__ == "__main__":
    main()
