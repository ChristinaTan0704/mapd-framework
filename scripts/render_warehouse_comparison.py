#!/usr/bin/env python3
"""Render the SMALL, MEDIUM, generated LARGE, and sparse warehouse maps."""

from __future__ import annotations

from pathlib import Path
from typing import NamedTuple

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]


class MapSpec(NamedTuple):
    title: str
    path: Path


MAPS = [
    MapSpec(
        "SMALL — original benchmark",
        ROOT / "data/Instances/small/kiva-10-500-5.map"),
    MapSpec(
        "SMALL — structured side banks",
        ROOT / "data/Instances/structured/small/"
        "kiva-structured-small-10.map"),
    MapSpec(
        "MEDIUM — original reference asset",
        ROOT.parent / "reference_code/CENTRAL-TP-TPTS/Instances/large/"
        "kiva-100-1000-50.map"),
    MapSpec(
        "MEDIUM — structured side banks",
        ROOT / "data/Instances/structured/medium/"
        "kiva-structured-medium-100.map"),
    MapSpec(
        "LARGE — generated paper-style reconstruction",
        ROOT / "data/Instances/large/kiva-large-1000-paper-style.map"),
    MapSpec(
        "SPARSE — SMALL setup on MEDIUM grid",
        ROOT / "data/Instances/structured/sparse/"
        "kiva-sparse-medium-grid-small-load-10.map"),
]

COLORS = {
    ".": (247, 247, 242),
    "@": (36, 39, 43),
    "e": (242, 169, 59),
    "r": (39, 125, 161),
}


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    paths = []
    if bold:
        paths.append("/System/Library/Fonts/Supplemental/Arial Bold.ttf")
    paths.extend([
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ])
    for path in paths:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default()


TITLE_FONT = font(29, True)
PANEL_FONT = font(22, True)
DETAIL_FONT = font(17)
LEGEND_FONT = font(17)


def read_map(path: Path) -> tuple[list[str], int, int, int, int]:
    lines = path.read_text().splitlines()
    rows, cols = map(int, lines[0].split(","))
    task_endpoints = int(lines[1])
    agents = int(lines[2])
    grid = lines[4:4 + rows]
    if len(grid) != rows or any(len(row) != cols for row in grid):
        raise ValueError(f"invalid map dimensions in {path}")
    return grid, rows, cols, task_endpoints, agents


def raster(grid: list[str]) -> Image.Image:
    rows, cols = len(grid), len(grid[0])
    image = Image.new("RGB", (cols, rows))
    pixels = image.load()
    for y, row in enumerate(grid):
        for x, value in enumerate(row):
            pixels[x, y] = COLORS[value]
    return image


def render_panel(spec: MapSpec, width: int = 860,
                 height: int = 610) -> Image.Image:
    grid, rows, cols, endpoints, agents = read_map(spec.path)
    panel = Image.new("RGB", (width, height), "#ffffff")
    draw = ImageDraw.Draw(panel)
    draw.text((width / 2, 26), spec.title, font=PANEL_FONT,
              fill="#17191c", anchor="mm")
    detail = f"{cols} × {rows}  •  {endpoints:,} task endpoints  •  {agents:,} homes"
    draw.text((width / 2, 58), detail, font=DETAIL_FONT,
              fill="#555b61", anchor="mm")

    source = raster(grid)
    max_width, max_height = width - 70, height - 125
    scale = min(max_width / cols, max_height / rows)
    rendered = source.resize(
        (max(1, round(cols * scale)), max(1, round(rows * scale))),
        Image.Resampling.NEAREST)
    x = (width - rendered.width) // 2
    y = 88 + (max_height - rendered.height) // 2
    panel.paste(rendered, (x, y))
    draw.rectangle((x - 1, y - 1, x + rendered.width, y + rendered.height),
                   outline="#777b80", width=2)
    return panel


def render_individual(spec: MapSpec, output: Path) -> None:
    panel = render_panel(spec, 1600, 1050)
    draw = ImageDraw.Draw(panel)
    legend = [
        ("#f7f7f2", "Traversable aisle"),
        ("#24272b", "Obstacle / shelf"),
        ("#f2a93b", "Task endpoint"),
        ("#277da1", "Agent home"),
    ]
    y = 1030
    item_width = 340
    start = (1600 - item_width * len(legend)) // 2
    for index, (color, label) in enumerate(legend):
        x = start + index * item_width
        draw.rectangle((x, y - 12, x + 26, y + 14), fill=color,
                       outline="#777b80")
        draw.text((x + 38, y + 1), label, font=LEGEND_FONT,
                  fill="#24272b", anchor="lm")
    panel.save(output, optimize=True)


def main() -> None:
    output_dir = ROOT / "visualizations"
    output_dir.mkdir(parents=True, exist_ok=True)

    panels = [render_panel(spec) for spec in MAPS]
    canvas = Image.new("RGB", (1800, 1980), "#f3f4f5")
    draw = ImageDraw.Draw(canvas)
    draw.text((900, 38), "Warehouse layout comparison", font=TITLE_FONT,
              fill="#17191c", anchor="mm")
    positions = [
        (30, 75), (910, 75),
        (30, 695), (910, 695),
        (30, 1315), (910, 1315),
    ]
    for panel, position in zip(panels, positions):
        canvas.paste(panel, position)

    legend = [
        ("#f7f7f2", "Traversable aisle"),
        ("#24272b", "Obstacle / shelf"),
        ("#f2a93b", "Task endpoint"),
        ("#277da1", "Agent home"),
    ]
    y = 1945
    item_width = 360
    start = (1800 - item_width * len(legend)) // 2
    for index, (color, label) in enumerate(legend):
        x = start + index * item_width
        draw.rectangle((x, y - 12, x + 26, y + 14), fill=color,
                       outline="#777b80")
        draw.text((x + 38, y + 1), label, font=LEGEND_FONT,
                  fill="#24272b", anchor="lm")

    comparison = output_dir / "warehouse-layout-comparison.png"
    canvas.save(comparison, optimize=True)
    print(comparison)

    names = [
        "small",
        "small-structured",
        "medium",
        "medium-structured",
        "large-paper-style",
        "sparse-medium-grid",
    ]
    for spec, name in zip(MAPS, names):
        output = output_dir / f"warehouse-{name}.png"
        render_individual(spec, output)
        print(output)


if __name__ == "__main__":
    main()
