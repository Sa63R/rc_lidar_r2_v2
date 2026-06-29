#!/usr/bin/env python3
"""Draw manual obstacles into a Nav2 occupancy map.

The input config uses map-frame coordinates in meters. This keeps edits
repeatable and preserves the map resolution/origin from the original YAML.
"""

import argparse
import json
import math
from pathlib import Path
import shutil
import sys

from PIL import Image, ImageDraw


def parse_args():
    parser = argparse.ArgumentParser(
        description="Draw rectangles/polygons into a Nav2 PGM map."
    )
    parser.add_argument("--map", required=True, help="Input Nav2 map YAML.")
    parser.add_argument(
        "--config",
        required=True,
        help="JSON config describing manual obstacles in map coordinates.",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output YAML path. The output PGM is written next to it.",
    )
    return parser.parse_args()


def parse_map_yaml(path):
    values = {}
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()

    for key in ("image", "resolution", "origin"):
        if key not in values:
            raise ValueError(f"map yaml missing {key}")

    origin = values["origin"].strip()
    if not (origin.startswith("[") and origin.endswith("]")):
        raise ValueError("origin must be [x, y, yaw]")
    origin_values = [float(part.strip()) for part in origin[1:-1].split(",")]
    if len(origin_values) != 3:
        raise ValueError("origin must have exactly 3 values")

    image_path = Path(values["image"])
    if not image_path.is_absolute():
        image_path = Path(path).parent / image_path

    return values, image_path, float(values["resolution"]), origin_values


def map_to_pixel(point, resolution, origin, width, height):
    x, y = point
    col = (x - origin[0]) / resolution
    row_from_bottom = (y - origin[1]) / resolution
    row = (height - 1) - row_from_bottom
    return (col, row)


def rectangle_points(rect):
    cx, cy = rect["center"]
    sx, sy = rect["size"]
    yaw = float(rect.get("yaw", 0.0))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    corners = [
        (-sx * 0.5, -sy * 0.5),
        (sx * 0.5, -sy * 0.5),
        (sx * 0.5, sy * 0.5),
        (-sx * 0.5, sy * 0.5),
    ]
    return [
        (
            cx + dx * cos_yaw - dy * sin_yaw,
            cy + dx * sin_yaw + dy * cos_yaw,
        )
        for dx, dy in corners
    ]


def expand_polygon(points, padding):
    if padding <= 0.0:
        return points
    cx = sum(x for x, _ in points) / len(points)
    cy = sum(y for _, y in points) / len(points)
    expanded = []
    for x, y in points:
        dx = x - cx
        dy = y - cy
        length = math.hypot(dx, dy)
        if length == 0.0:
            expanded.append((x, y))
        else:
            expanded.append((x + padding * dx / length, y + padding * dy / length))
    return expanded


def write_yaml(template_values, output_yaml, output_image_name):
    values = dict(template_values)
    values["image"] = output_image_name
    ordered_keys = [
        "image",
        "mode",
        "resolution",
        "origin",
        "negate",
        "occupied_thresh",
        "free_thresh",
    ]
    lines = []
    for key in ordered_keys:
        if key in values:
            lines.append(f"{key}: {values[key]}")
    for key in sorted(values):
        if key not in ordered_keys:
            lines.append(f"{key}: {values[key]}")
    Path(output_yaml).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    args = parse_args()
    try:
        values, image_path, resolution, origin = parse_map_yaml(args.map)
        config = json.loads(Path(args.config).read_text(encoding="utf-8"))
        output_yaml = Path(args.output)
        output_pgm = output_yaml.with_suffix(".pgm")

        image = Image.open(image_path).convert("L")
        width, height = image.size
        draw = ImageDraw.Draw(image)

        obstacles = config.get("obstacles", [])
        if not isinstance(obstacles, list):
            raise ValueError("config field 'obstacles' must be a list")

        for obstacle in obstacles:
            padding = float(obstacle.get("padding", config.get("padding", 0.0)))
            value = int(obstacle.get("value", config.get("value", 0)))
            if obstacle.get("type") == "rectangle":
                points = rectangle_points(obstacle)
            elif obstacle.get("type") == "polygon":
                points = obstacle["points"]
            else:
                raise ValueError(f"unsupported obstacle type: {obstacle.get('type')}")
            points = expand_polygon([(float(x), float(y)) for x, y in points], padding)
            pixels = [
                map_to_pixel(point, resolution, origin, width, height)
                for point in points
            ]
            draw.polygon(pixels, fill=value)

        output_yaml.parent.mkdir(parents=True, exist_ok=True)
        image.save(output_pgm)
        write_yaml(values, output_yaml, output_pgm.name)

        preview = output_yaml.with_suffix(".png")
        shutil.copyfile(output_pgm, preview)
        print(f"wrote: {output_pgm}")
        print(f"wrote: {output_yaml}")
        print(f"preview: {preview}")
    except Exception as exc:
        print(f"draw_manual_obstacles: error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
